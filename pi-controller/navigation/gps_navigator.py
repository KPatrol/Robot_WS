"""
gps_navigator.py — Outdoor GPS waypoint navigation for K-Patrol.

Strategy
--------
A simple, robust pure-pursuit-style controller for low-speed outdoor patrol:

    1. Compute target bearing + distance from current GPS fix to next
       waypoint using the great-circle (Haversine + initial-bearing)
       formulae — accurate enough at human walking distances (< 1 km).
    2. Pick a heading reference: prefer IMU yaw (always available, no
       latency); fuse in GPS course-over-ground when ground speed is
       healthy (> ~1 km/h) so we self-correct against IMU drift.
    3. Compute heading error (signed shortest angle), then:
         * |error| > coarse_align_deg → in-place rotation only (TURNING)
         * else                       → forward drive + heading PD (DRIVING)
    4. Reactive safety: ToF front sensor short-circuits any forward
       command. front < emergency_mm → SAFETY_STOP (twist 0). The mode
       does NOT trip EMERGENCY by itself; that's reserved for the
       detection bridge / operator e-stop. SAFETY_STOP self-recovers
       when the obstacle clears.

State machine
-------------
    IDLE         → start() with a route → TURNING (or NO_FIX while waiting)
    NO_FIX       → wait for usable GPS fix → TURNING
    TURNING      → in-place rotate until heading error < align_ok_deg
    DRIVING      → forward + heading PD; transition to ARRIVED when
                   distance < waypoint.radius_m
    ARRIVED      → advance to next waypoint or DONE if last (loop=False)
                   / wrap to first (loop=True)
    SAFETY_STOP  → ToF emergency; resume previous state when clear
    DONE         → terminal; tick() returns done=True so caller can
                   drop back to MANUAL.

Coordinate conventions
----------------------
    Bearing  — degrees clockwise from true North (0=N, 90=E, 180=S, 270=W)
    IMU yaw  — same convention assumed (BNO08x calibrated to magnetic N).
               If the user mounts the IMU offset, configure
               `imu_yaw_offset_deg` to align with true North.
    Twist    — (vx, vy, wz, spd) for ESP32 MEC: command. Ground frame:
               vx > 0 → forward, vy > 0 → strafe-left, wz > 0 → CCW.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Any, Dict, List, NamedTuple, Optional


# ─── Geo helpers ─────────────────────────────────────────────────────
_EARTH_R_M = 6_371_000.0


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance between two WGS-84 points, in metres."""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dlam / 2) ** 2
    return 2 * _EARTH_R_M * math.asin(min(1.0, math.sqrt(a)))


def initial_bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing (degrees true) from point 1 to point 2."""
    p1 = math.radians(lat1)
    p2 = math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dlam)
    return (math.degrees(math.atan2(y, x)) + 360.0) % 360.0


def shortest_angle_deg(target_deg: float, current_deg: float) -> float:
    """Signed shortest angular difference target - current, in (-180, 180]."""
    err = (target_deg - current_deg + 540.0) % 360.0 - 180.0
    return err


# ─── Data classes ────────────────────────────────────────────────────
@dataclass
class Waypoint:
    lat: float
    lon: float
    radius_m: float = 3.0      # arrival tolerance
    speed_pct: int  = 60       # 0..100, scales base_vx
    label: Optional[str] = None

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "Waypoint":
        if "lat" not in d or "lon" not in d:
            raise ValueError("waypoint missing lat/lon")
        return cls(
            lat=float(d["lat"]),
            lon=float(d["lon"]),
            radius_m=float(d.get("radius_m", d.get("radiusM", 3.0))),
            speed_pct=int(d.get("speed_pct", d.get("speedPct", 60))),
            label=d.get("label"),
        )

    def to_dict(self) -> Dict[str, Any]:
        return {
            "lat": self.lat,
            "lon": self.lon,
            "radiusM": self.radius_m,
            "speedPct": self.speed_pct,
            "label": self.label,
        }


@dataclass
class GPSNavigatorConfig:
    # ── Heading control ─────────────────────────────────────────────
    coarse_align_deg:   float = 25.0   # |err| > this → rotate in place
    align_ok_deg:       float =  8.0   # exit TURNING when |err| < this
    heading_kp:         float =  1.4   # error_deg → wz scale
    heading_max_wz:     int   =  60    # cap rotational speed
    turn_speed:         int   =  55    # in-place TURNING wz magnitude

    # ── Forward speed ───────────────────────────────────────────────
    base_vx:            int   =  80    # MEC vx at 100% speed_pct
    base_spd:           int   =  90    # ESP32 absolute (0..255) at 100%
    slow_distance_m:    float =  4.0   # below this distance → ramp speed down
    slow_min_factor:    float =  0.40  # min multiplier near waypoint

    # ── Reactive safety ─────────────────────────────────────────────
    safety_emergency_mm: int  = 350    # full stop if front below this
    safety_slow_mm:      int  = 800    # halve speed when below this

    # ── GPS fusion ──────────────────────────────────────────────────
    gps_course_min_kmh:  float = 1.5   # use COG above this speed only
    gps_course_weight:   float = 0.30  # 0=imu only, 1=cog only
    require_fix:         bool  = True
    max_hdop:            float = 5.0
    no_fix_grace_s:      float = 8.0   # tolerate brief outages while DRIVING

    # ── IMU calibration ─────────────────────────────────────────────
    imu_yaw_offset_deg:  float = 0.0   # added to imu_yaw before use


class GPSState(str, Enum):
    IDLE        = "IDLE"
    NO_FIX      = "NO_FIX"
    TURNING     = "TURNING"
    DRIVING     = "DRIVING"
    ARRIVED     = "ARRIVED"
    SAFETY_STOP = "SAFETY_STOP"
    DONE        = "DONE"


class GPSNavResult(NamedTuple):
    vx: int
    vy: int
    wz: int
    spd: int
    state: str
    done: bool


@dataclass
class _RuntimeStats:
    last_distance_m:  float = float("nan")
    last_bearing_deg: float = float("nan")
    last_heading_deg: float = float("nan")
    last_error_deg:   float = float("nan")
    last_fix_ts:      float = 0.0
    waypoint_index:   int   = 0
    started_ts:       float = 0.0


# ─── Navigator ──────────────────────────────────────────────────────
class GPSNavigator:
    """GPS waypoint controller. Stateful, single-thread (called from main loop)."""

    def __init__(self, config: Optional[GPSNavigatorConfig] = None):
        self.cfg = config or GPSNavigatorConfig()
        self._route: List[Waypoint] = []
        self._loop: bool = False
        self._wp_idx: int = 0
        self._state: GPSState = GPSState.IDLE
        self._prev_state: GPSState = GPSState.IDLE
        self._stats = _RuntimeStats()
        self._running: bool = False

    # ── Route management ──────────────────────────────────────────
    def set_route(self, waypoints: List[Waypoint], loop: bool = False) -> None:
        self._route = list(waypoints)
        self._loop = bool(loop)
        self._wp_idx = 0
        self._stats.waypoint_index = 0

    def has_route(self) -> bool:
        return len(self._route) > 0

    def route_summary(self) -> Dict[str, Any]:
        return {
            "count": len(self._route),
            "loop":  self._loop,
            "waypoints": [w.to_dict() for w in self._route],
        }

    def start(self) -> None:
        if not self._route:
            return
        self._running = True
        self._wp_idx  = 0
        self._stats.waypoint_index = 0
        self._stats.started_ts = time.monotonic()
        self._state  = GPSState.NO_FIX  # bumped to TURNING on first usable fix

    def stop(self) -> None:
        self._running = False
        self._state   = GPSState.IDLE

    def is_running(self) -> bool:
        return self._running

    # ── Diagnostics ──────────────────────────────────────────────
    def diagnostics(self) -> Dict[str, Any]:
        wp = self._current_waypoint()
        return {
            "running":         self._running,
            "state":           self._state.value,
            "wp_index":        self._wp_idx,
            "wp_total":        len(self._route),
            "wp_label":        wp.label if wp else None,
            "distance_m":      _safe_round(self._stats.last_distance_m, 2),
            "bearing_deg":     _safe_round(self._stats.last_bearing_deg, 1),
            "heading_deg":     _safe_round(self._stats.last_heading_deg, 1),
            "error_deg":       _safe_round(self._stats.last_error_deg, 1),
            "loop":            self._loop,
        }

    # ── Tick (called every control loop iteration) ───────────────
    def tick(
        self,
        gps_data:      Optional[Any],   # GPSData snapshot or dict
        imu_yaw_deg:   float,
        tof_dict:      Dict[str, float],
        speed_pct:     int = 60,
    ) -> GPSNavResult:
        if not self._running or not self._route:
            self._state = GPSState.IDLE
            return GPSNavResult(0, 0, 0, 0, self._state.value, done=False)

        wp = self._current_waypoint()
        if wp is None:
            self._state = GPSState.DONE
            return GPSNavResult(0, 0, 0, 0, self._state.value, done=True)

        # ── Step 1: validate GPS fix ────────────────────────────
        fix_ok, lat, lon, cog, gnd_kmh = self._extract_fix(gps_data)
        now = time.monotonic()
        if fix_ok:
            self._stats.last_fix_ts = now
        else:
            grace = self.cfg.no_fix_grace_s
            if self._state == GPSState.DRIVING and now - self._stats.last_fix_ts < grace:
                # short outage while moving — coast on last command at idle
                self._state = GPSState.NO_FIX
                return GPSNavResult(0, 0, 0, 0, self._state.value, done=False)
            self._state = GPSState.NO_FIX
            return GPSNavResult(0, 0, 0, 0, self._state.value, done=False)

        # ── Step 2: compute distance + bearing to waypoint ──────
        distance_m = haversine_m(lat, lon, wp.lat, wp.lon)
        bearing    = initial_bearing_deg(lat, lon, wp.lat, wp.lon)
        self._stats.last_distance_m  = distance_m
        self._stats.last_bearing_deg = bearing

        # ── Step 3: arrival check ───────────────────────────────
        if distance_m <= max(0.5, wp.radius_m):
            return self._advance_waypoint()

        # ── Step 4: compute heading reference ───────────────────
        heading = self._fuse_heading(imu_yaw_deg, cog, gnd_kmh)
        self._stats.last_heading_deg = heading
        err = shortest_angle_deg(bearing, heading)
        self._stats.last_error_deg = err

        # ── Step 5: reactive ToF safety ─────────────────────────
        front_mm = int(tof_dict.get("front", 9999))
        if front_mm <= self.cfg.safety_emergency_mm:
            if self._state != GPSState.SAFETY_STOP:
                self._prev_state = self._state
            self._state = GPSState.SAFETY_STOP
            return GPSNavResult(0, 0, 0, 0, self._state.value, done=False)

        # ── Step 6: pick state ──────────────────────────────────
        if abs(err) > self.cfg.coarse_align_deg:
            self._state = GPSState.TURNING
            wz = int(math.copysign(self.cfg.turn_speed, err))
            spd = self._scaled_spd(speed_pct, wp.speed_pct, factor=0.6)
            return GPSNavResult(0, 0, wz, spd, self._state.value, done=False)

        # In alignment band — drive forward with heading PD.
        self._state = GPSState.DRIVING
        wz = int(_clip(self.cfg.heading_kp * err, -self.cfg.heading_max_wz, self.cfg.heading_max_wz))

        # Speed shaping: ramp down inside slow_distance_m, halve in slow_mm zone.
        ramp = 1.0
        if distance_m < self.cfg.slow_distance_m:
            t = max(0.0, distance_m / self.cfg.slow_distance_m)
            ramp = self.cfg.slow_min_factor + (1.0 - self.cfg.slow_min_factor) * t
        if front_mm <= self.cfg.safety_slow_mm:
            ramp *= 0.5

        sp = _clip(min(speed_pct, wp.speed_pct) / 100.0, 0.0, 1.0) * ramp
        vx  = int(self.cfg.base_vx * sp)
        spd = max(20, int(self.cfg.base_spd * sp))
        return GPSNavResult(vx, 0, wz, spd, self._state.value, done=False)

    # ── Helpers ─────────────────────────────────────────────────
    def _current_waypoint(self) -> Optional[Waypoint]:
        if not self._route:
            return None
        if self._wp_idx >= len(self._route):
            return None
        return self._route[self._wp_idx]

    def _advance_waypoint(self) -> GPSNavResult:
        self._wp_idx += 1
        self._stats.waypoint_index = self._wp_idx
        if self._wp_idx >= len(self._route):
            if self._loop:
                self._wp_idx = 0
                self._stats.waypoint_index = 0
                self._state = GPSState.TURNING
                return GPSNavResult(0, 0, 0, 0, GPSState.ARRIVED.value, done=False)
            self._running = False
            self._state = GPSState.DONE
            return GPSNavResult(0, 0, 0, 0, self._state.value, done=True)
        self._state = GPSState.TURNING
        return GPSNavResult(0, 0, 0, 0, GPSState.ARRIVED.value, done=False)

    def _extract_fix(self, gps_data: Any):
        """Return (has_fix, lat, lon, cog_deg, speed_kmh)."""
        if gps_data is None:
            return False, 0.0, 0.0, float("nan"), 0.0
        if isinstance(gps_data, dict):
            has_fix = bool(gps_data.get("has_fix", gps_data.get("hasFix", False)))
            lat     = float(gps_data.get("latitude", 0.0))
            lon     = float(gps_data.get("longitude", 0.0))
            cog     = float(gps_data.get("heading", float("nan")))
            kmh     = float(gps_data.get("speed_kmh", gps_data.get("speedKmh", 0.0)))
            hdop    = float(gps_data.get("hdop", 99.9))
        else:
            has_fix = bool(getattr(gps_data, "has_fix", False))
            lat     = float(getattr(gps_data, "latitude", 0.0))
            lon     = float(getattr(gps_data, "longitude", 0.0))
            cog     = float(getattr(gps_data, "heading", float("nan")))
            kmh     = float(getattr(gps_data, "speed_kmh", 0.0))
            hdop    = float(getattr(gps_data, "hdop", 99.9))
        if self.cfg.require_fix and (not has_fix or hdop > self.cfg.max_hdop):
            return False, lat, lon, cog, kmh
        return True, lat, lon, cog, kmh

    def _fuse_heading(self, imu_yaw_deg: float, cog_deg: float, gnd_kmh: float) -> float:
        imu = (imu_yaw_deg + self.cfg.imu_yaw_offset_deg) % 360.0
        if (
            math.isfinite(cog_deg)
            and gnd_kmh >= self.cfg.gps_course_min_kmh
            and 0.0 <= cog_deg <= 360.0
        ):
            w = self.cfg.gps_course_weight
            err = shortest_angle_deg(cog_deg, imu)
            return (imu + w * err + 360.0) % 360.0
        return imu

    def _scaled_spd(self, op_pct: int, wp_pct: int, factor: float = 1.0) -> int:
        sp = _clip(min(op_pct, wp_pct) / 100.0, 0.0, 1.0) * factor
        return max(20, int(self.cfg.base_spd * sp))


# ─── Module helpers ─────────────────────────────────────────────────
def _clip(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))


def _safe_round(x: float, n: int):
    if not math.isfinite(x):
        return None
    return round(x, n)


__all__ = [
    "GPSNavigator",
    "GPSNavigatorConfig",
    "GPSNavResult",
    "GPSState",
    "Waypoint",
    "haversine_m",
    "initial_bearing_deg",
    "shortest_angle_deg",
]
