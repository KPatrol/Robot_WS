"""
free_coverage.py — Map-aware free-space coverage for indoor patrol.

Strategy
--------
Three layered behaviors, each handing off when the previous can't make
progress:

    1. WALL_FOLLOW  — if a wall is within range on the right, hug it
                      with a PD controller (right-hand rule). Reliable
                      coverage of walled rooms; produces a clean spiral
                      inward over time.
    2. FORWARD      — open space, no usable wall reference. Drive ahead
                      with random turns at obstacles (legacy fallback).
    3. FRONTIER     — when the local 1 m radius is mostly visited,
                      query the occupancy grid for the nearest unvisited
                      cell adjacent to known space and drive toward it.
                      Escapes "stuck in a visited corner" loops.

Backwards compatibility
-----------------------
`tick(front, left, right)` keeps working. When `pose` is omitted we fall
back to pure random-walk + (optional) wall-follow without a map. That
means the v3 code paths in older tests still pass, and operators can
disable map-aware mode via config flags.

State machine
-------------
    FORWARD       → drive vx=base_vx until front ≤ turn_trigger_mm
                    or right wall comes into range
    WALL_FOLLOW   → PD on right-side ToF; abort to TURNING if front blocked
    TURNING       → in-place rotation until timer or heading hit
    BACK          → emergency reverse when front ≤ emergency_mm
    FRONTIER_SEEK → rotate to bearing then drive to target cell

Map persistence
---------------
If `save_path` is set, the grid is loaded on init and saved every
`save_period_s` seconds (default 30 s). On crash, the next session
resumes with the same coverage knowledge.
"""

from __future__ import annotations

import math
import random
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, NamedTuple, Optional, Tuple

from .nav_utils import clamp, wrap_pi
from .occupancy_grid import (
    CELL_OBSTACLE,
    CELL_VISITED,
    GridConfig,
    OccupancyGrid,
)


# Default 6-ToF mounting bearings (degrees, body frame). Used when callers
# pass `all_tofs` as the natural `{name: distance_mm}` shape — without this
# fallback the int values would be silently rejected as unrecognized item
# types and no rays would reach the OccupancyGrid.
_DEFAULT_TOF_BEARINGS_DEG = {
    "front":        0.0,
    "front_left":  45.0,
    "front_right": -45.0,
    "left":        90.0,
    "right":      -90.0,
    "rear":       180.0,
}


@dataclass
class CoverageConfig:
    # ── Forward / emergency behavior ─────────────────────────────────
    base_vx:          int   = 70    # forward speed (0–127)
    base_spd:         int   = 80    # absolute ESP32 speed (0–255)
    turn_trigger_mm:  int   = 500   # start turn when front ≤ this (low-speed baseline)
    emergency_mm:     int   = 220   # immediate reverse threshold (low-speed baseline)
    turn_speed:       int   = 60    # wz magnitude during rotation
    turn_min_deg:     float = 70.0
    turn_max_deg:     float = 160.0
    deg_per_sec:      float = 90.0  # approx rotation rate at turn_speed
    back_duration_s:  float = 0.40

    # ── Speed-aware safety scaling ──────────────────────────────────
    # ToF callbacks land at ~5–10 Hz per sensor; at high vx the robot can
    # cover 100+ mm between samples, so we widen the emergency / turn
    # zones proportionally to base_vx. Linearly interpolated between
    # safety_scale_speed_ref (no extra) and 127 (full extra).
    safety_scale_speed_ref:    int = 60
    safety_emergency_extra_mm: int = 220
    safety_turn_extra_mm:      int = 250

    # ── ToF staleness gate ───────────────────────────────────────────
    # When the freshest ToF timestamp is older than tof_stale_ms, treat
    # the readings as unreliable: throttle forward speed and tighten the
    # emergency threshold so a pending obstacle doesn't get hit while we
    # wait for the next sample. Disabled when tof_timestamp_ms is None.
    tof_stale_ms:        int   = 280
    tof_stale_throttle:  float = 0.45
    tof_stale_margin_mm: int   = 150

    # ── Wall-follow PD ───────────────────────────────────────────────
    wall_follow_enabled:  bool  = True
    wall_target_mm:       int   = 350     # desired distance to right wall
    wall_deadband_mm:     int   = 80      # ignore tiny errors
    wall_lost_mm:         int   = 900     # right reading ≥ this → no wall
    wall_kp_lateral:      float = 0.10    # mm error → vy units
    wall_kp_yaw:          float = 0.15    # mm error → wz units
    wall_max_vy:          int   = 30
    wall_max_wz:          int   = 35
    wall_hard_err_mm:     int   = 200     # |err| ≥ this → throttle base_vx
    wall_hard_throttle:   float = 0.7     # multiplier when correcting hard

    # ── Frontier seek heading PD ─────────────────────────────────────
    frontier_align_deg:   float = 30.0    # |err| > this → rotate in place first
    frontier_kp_yaw:      float = 1.0     # deg → wz units while driving

    # ── Frontier exploration (requires pose) ─────────────────────────
    frontier_enabled:                bool  = True
    frontier_check_period_s:         float = 4.0   # how often to re-evaluate
    frontier_radius_m:               float = 3.5   # max search radius
    frontier_local_visited_threshold: int  = 4     # visit count to call "saturated"
    frontier_local_radius_m:         float = 1.0   # area to call "saturated"
    frontier_saturation_pct:         float = 0.7   # 70 % of local cells saturated → seek
    frontier_max_seek_s:             float = 6.0   # abort timeout
    frontier_arrived_m:              float = 0.4   # within this of target → done

    # ── Occupancy map ─────────────────────────────────────────────────
    map_enabled:           bool  = True
    map_cell_size_m:       float = 0.25
    map_width_cells:       int   = 60
    map_height_cells:      int   = 60
    save_path:             Optional[str] = None
    save_period_s:         float = 30.0


class _State(Enum):
    FORWARD       = "FORWARD"
    WALL_FOLLOW   = "WALL_FOLLOW"
    TURNING       = "TURNING"
    BACK          = "BACK"
    FRONTIER_SEEK = "FRONTIER_SEEK"


class CoverageResult(NamedTuple):
    vx:    int
    vy:    int
    wz:    int
    spd:   int
    state: str


class FreeCoverage:
    """Map-aware free-coverage navigator.

    Call tick() at ~20 Hz. If `pose` is provided (a navigation.odometry.Pose
    or any object with `.x`, `.y`, `.theta` attributes), the occupancy grid
    is updated and frontier-seek becomes available. Without `pose` it
    degrades to wall-follow + random-walk (no map memory).
    """

    def __init__(self, config: Optional[CoverageConfig] = None):
        self.cfg = config or CoverageConfig()
        self._state = _State.FORWARD
        self._state_until: float = 0.0   # monotonic deadline for timed states
        self._turn_wz: int = 0

        # Occupancy grid — only built if map_enabled.
        self._grid: Optional[OccupancyGrid] = None
        if self.cfg.map_enabled:
            grid_cfg = GridConfig(
                cell_size_m=self.cfg.map_cell_size_m,
                width=self.cfg.map_width_cells,
                height=self.cfg.map_height_cells,
                origin_cell_x=self.cfg.map_width_cells  // 2,
                origin_cell_y=self.cfg.map_height_cells // 2,
            )
            self._grid = OccupancyGrid(grid_cfg)
            if self.cfg.save_path:
                self._grid.load_npz(self.cfg.save_path)

        # Frontier-seek bookkeeping
        self._frontier_target:      Optional[Tuple[float, float]] = None
        self._frontier_started_at:  float = 0.0
        self._last_frontier_check:  float = 0.0

        # Persistence
        self._last_save: float = 0.0

    # ── Public API ────────────────────────────────────────────────────

    def tick(
        self,
        front_mm: int,
        left_mm:  int,
        right_mm: int,
        pose: Any = None,                       # Optional[Pose] from odometry
        all_tofs: Optional[dict] = None,        # Optional 6-ToF dict for richer ray casts
        tof_timestamp_ms: Optional[float] = None,  # ms since epoch of latest ToF sample
    ) -> CoverageResult:
        cfg = self.cfg
        now = time.monotonic()

        # ── Effective thresholds: speed-aware + stale-ToF aware ───────
        # tof_timestamp_ms can also ride inside all_tofs["timestamp"] so the
        # caller can pass safety_controller.tof_data.to_float_dict() directly.
        if tof_timestamp_ms is None and all_tofs is not None:
            ts = all_tofs.get("timestamp")
            if isinstance(ts, (int, float)) and ts > 0:
                tof_timestamp_ms = float(ts)

        ref      = cfg.safety_scale_speed_ref
        span     = max(1.0, 127.0 - ref)
        sf       = max(0.0, min(1.0, (cfg.base_vx - ref) / span))
        eff_emergency_mm    = cfg.emergency_mm    + int(cfg.safety_emergency_extra_mm * sf)
        eff_turn_trigger_mm = cfg.turn_trigger_mm + int(cfg.safety_turn_extra_mm      * sf)
        eff_base_vx         = cfg.base_vx

        tof_stale = False
        if tof_timestamp_ms is not None and tof_timestamp_ms > 0:
            age_ms = (time.time() * 1000.0) - tof_timestamp_ms
            if age_ms > cfg.tof_stale_ms:
                tof_stale = True
                eff_emergency_mm += cfg.tof_stale_margin_mm
                eff_base_vx       = max(0, int(eff_base_vx * cfg.tof_stale_throttle))

        # Cache for diagnostics + _frontier_tick.
        self._eff_emergency_mm    = eff_emergency_mm
        self._eff_turn_trigger_mm = eff_turn_trigger_mm
        self._eff_base_vx         = eff_base_vx
        self._tof_stale_flag      = tof_stale

        # ── Map update ─────────────────────────────────────────────────
        if pose is not None and self._grid is not None:
            self._update_map(pose, front_mm, left_mm, right_mm, all_tofs, now)

        # ── Emergency reverse takes priority over everything ──────────
        if front_mm <= eff_emergency_mm and self._state != _State.BACK:
            self._state       = _State.BACK
            self._state_until = now + cfg.back_duration_s
            self._frontier_target = None

        if self._state == _State.BACK:
            if now < self._state_until:
                return CoverageResult(-eff_base_vx, 0, 0, cfg.base_spd, "BACK")
            self._begin_turn(left_mm, right_mm, now)

        if self._state == _State.TURNING:
            if now < self._state_until:
                return CoverageResult(0, 0, self._turn_wz, cfg.base_spd, "TURNING")
            self._state = _State.FORWARD

        if self._state == _State.FRONTIER_SEEK:
            return self._frontier_tick(front_mm, left_mm, right_mm, pose, now)

        # ── Frontier check: switch out of FORWARD when local coverage saturated
        if (pose is not None and self._grid is not None
                and cfg.frontier_enabled
                and now - self._last_frontier_check >= cfg.frontier_check_period_s):
            self._last_frontier_check = now
            if self._is_local_saturated(pose):
                target = self._grid.nearest_frontier(
                    pose.x, pose.y, pose.theta,
                    max_radius_m=cfg.frontier_radius_m,
                )
                if target is not None:
                    self._frontier_target     = (target[0], target[1])
                    self._frontier_started_at = now
                    self._state               = _State.FRONTIER_SEEK
                    return self._frontier_tick(front_mm, left_mm, right_mm, pose, now)

        # ── Forward: front blocked → turn ──────────────────────────────
        if front_mm <= eff_turn_trigger_mm:
            self._begin_turn(left_mm, right_mm, now)
            return CoverageResult(0, 0, self._turn_wz, cfg.base_spd, "TURNING")

        # ── Wall-follow takes priority over open-space straight-line ──
        if cfg.wall_follow_enabled and right_mm < cfg.wall_lost_mm:
            return self._wall_follow_tick(right_mm, eff_base_vx)

        return CoverageResult(eff_base_vx, 0, 0, cfg.base_spd, "FORWARD")

    def reset(self) -> None:
        self._state              = _State.FORWARD
        self._state_until        = 0.0
        self._turn_wz            = 0
        self._frontier_target    = None
        self._frontier_started_at = 0.0
        self._last_frontier_check = 0.0

    def diagnostics(self) -> dict:
        out: dict = {
            "state":            self._state.value,
            "frontier_target":  list(self._frontier_target) if self._frontier_target else None,
            "eff_emergency_mm":    getattr(self, "_eff_emergency_mm",    self.cfg.emergency_mm),
            "eff_turn_trigger_mm": getattr(self, "_eff_turn_trigger_mm", self.cfg.turn_trigger_mm),
            "eff_base_vx":         getattr(self, "_eff_base_vx",         self.cfg.base_vx),
            "tof_stale":           bool(getattr(self, "_tof_stale_flag", False)),
        }
        if self._grid is not None:
            out["map"] = self._grid.diagnostics()
        return out

    @property
    def grid(self) -> Optional[OccupancyGrid]:
        return self._grid

    # ── Wall-follow PD ────────────────────────────────────────────────

    def _wall_follow_tick(self, right_mm: int, base_vx: Optional[int] = None) -> CoverageResult:
        cfg = self.cfg
        vx_ref = cfg.base_vx if base_vx is None else base_vx
        # Error: + means wall is too far → drift right (negative vy / negative wz).
        err = right_mm - cfg.wall_target_mm
        if abs(err) < cfg.wall_deadband_mm:
            err = 0
        vy = -int(clamp(err * cfg.wall_kp_lateral, -cfg.wall_max_vy, cfg.wall_max_vy))
        wz = -int(clamp(err * cfg.wall_kp_yaw,     -cfg.wall_max_wz, cfg.wall_max_wz))
        # Slow down a bit when correcting hard so we don't overshoot.
        vx = vx_ref if abs(err) < cfg.wall_hard_err_mm else int(vx_ref * cfg.wall_hard_throttle)
        return CoverageResult(vx, vy, wz, cfg.base_spd, "WALL_FOLLOW")

    # ── Frontier seek ─────────────────────────────────────────────────

    def _frontier_tick(
        self,
        front_mm: int,
        left_mm:  int,
        right_mm: int,
        pose:     Any,
        now:      float,
    ) -> CoverageResult:
        cfg = self.cfg
        eff_emergency_mm    = getattr(self, "_eff_emergency_mm",    cfg.emergency_mm)
        eff_turn_trigger_mm = getattr(self, "_eff_turn_trigger_mm", cfg.turn_trigger_mm)
        eff_base_vx         = getattr(self, "_eff_base_vx",         cfg.base_vx)

        # Abort conditions: timeout, no target, no pose, blocked.
        if (self._frontier_target is None
                or pose is None
                or now - self._frontier_started_at > cfg.frontier_max_seek_s):
            self._frontier_target = None
            self._state = _State.FORWARD
            return CoverageResult(0, 0, 0, cfg.base_spd, "FORWARD")

        if front_mm <= eff_emergency_mm:
            self._frontier_target = None
            self._state       = _State.BACK
            self._state_until = now + cfg.back_duration_s
            return CoverageResult(-eff_base_vx, 0, 0, cfg.base_spd, "BACK")

        fx, fy = self._frontier_target
        dx, dy = fx - pose.x, fy - pose.y
        dist   = math.hypot(dx, dy)
        if dist < cfg.frontier_arrived_m:
            self._frontier_target = None
            self._state = _State.FORWARD
            return CoverageResult(eff_base_vx, 0, 0, cfg.base_spd, "FORWARD")

        bearing     = math.atan2(dy, dx)
        heading_err = wrap_pi(bearing - pose.theta)

        # Big heading error → rotate in place.
        if abs(heading_err) > math.radians(cfg.frontier_align_deg):
            wz = cfg.turn_speed if heading_err > 0 else -cfg.turn_speed
            return CoverageResult(0, 0, wz, cfg.base_spd, "FRONTIER_SEEK")

        # Heading aligned → drive forward with mild yaw correction. If
        # an obstacle pops up mid-seek, abandon target and turn away.
        if front_mm <= eff_turn_trigger_mm:
            self._frontier_target = None
            self._begin_turn(left_mm, right_mm, now)
            return CoverageResult(0, 0, self._turn_wz, cfg.base_spd, "TURNING")

        wz = int(clamp(math.degrees(heading_err) * cfg.frontier_kp_yaw,
                       -cfg.wall_max_wz, cfg.wall_max_wz))
        return CoverageResult(eff_base_vx, 0, wz, cfg.base_spd, "FRONTIER_SEEK")

    def _is_local_saturated(self, pose: Any) -> bool:
        """True when most cells within frontier_local_radius_m have been
        visited heavily — heuristic for "we've covered this spot, move on".
        """
        cfg = self.cfg
        if self._grid is None:
            return False
        ix, iy = self._grid.world_to_cell(pose.x, pose.y)
        if not self._grid.in_bounds(ix, iy):
            return False
        radius_cells = max(2, int(cfg.frontier_local_radius_m / cfg.map_cell_size_m))
        n_total      = 0
        n_saturated  = 0
        for di in range(-radius_cells, radius_cells + 1):
            for dj in range(-radius_cells, radius_cells + 1):
                if di * di + dj * dj > radius_cells * radius_cells:
                    continue
                i, j = ix + di, iy + dj
                if not self._grid.in_bounds(i, j):
                    continue
                if self._grid.flags[i, j] & CELL_OBSTACLE:
                    continue
                n_total += 1
                if int(self._grid.visit_count[i, j]) >= cfg.frontier_local_visited_threshold:
                    n_saturated += 1
        if n_total < 8:
            return False
        return (n_saturated / n_total) >= cfg.frontier_saturation_pct

    # ── Map maintenance ───────────────────────────────────────────────

    def _update_map(
        self,
        pose:       Any,
        front_mm:   int,
        left_mm:    int,
        right_mm:   int,
        all_tofs:   Optional[dict],
        now:        float,
    ) -> None:
        assert self._grid is not None
        self._grid.mark_visited(pose.x, pose.y)

        # Build a list of (bearing_rad_body, distance_mm) rays. By default
        # we use just the 3 cardinal directions (front/left/right). When
        # all_tofs is provided as a dict, we accept three shapes:
        #   {"name": distance_mm}                       — bearings looked up from
        #                                                  _DEFAULT_TOF_BEARINGS_DEG
        #   {"name": (bearing_deg, distance_mm)}
        #   {"name": {"bearing": deg, "distance": mm}}
        rays: list[Tuple[float, int]] = []
        if all_tofs:
            for name, item in all_tofs.items():
                try:
                    if isinstance(item, dict):
                        b = math.radians(float(item.get("bearing", 0.0)))
                        d = float(item.get("distance", 9999))
                    elif isinstance(item, (tuple, list)) and len(item) == 2:
                        b = math.radians(float(item[0]))
                        d = float(item[1])
                    elif isinstance(item, (int, float)):
                        bearing_deg = _DEFAULT_TOF_BEARINGS_DEG.get(name)
                        if bearing_deg is None:
                            continue
                        b = math.radians(bearing_deg)
                        d = float(item)
                    else:
                        continue
                except (TypeError, ValueError):
                    continue
                rays.append((b, int(d)))
        else:
            rays = [
                (0.0,           front_mm),
                (math.pi / 2,   left_mm),
                (-math.pi / 2,  right_mm),
            ]

        for bearing, dist_mm in rays:
            if dist_mm <= 0 or dist_mm > 9000:
                continue
            self._grid.cast_ray(
                pose.x, pose.y,
                pose.theta + bearing,
                dist_mm / 1000.0,
            )

        self._grid.tick()

        if self.cfg.save_path and now - self._last_save >= self.cfg.save_period_s:
            try:
                self._grid.save_npz(self.cfg.save_path)
                self._last_save = now
            except Exception:
                # Disk full / permission issue shouldn't crash navigation.
                pass

    # ── Turn helper ──────────────────────────────────────────────────

    def _begin_turn(self, left_mm: int, right_mm: int, now: float) -> None:
        cfg = self.cfg
        angle_deg = random.uniform(cfg.turn_min_deg, cfg.turn_max_deg)
        duration  = angle_deg / cfg.deg_per_sec
        # Turn away from the closer side wall.
        if left_mm < right_mm:
            self._turn_wz = -cfg.turn_speed   # CW (right)
        else:
            self._turn_wz =  cfg.turn_speed   # CCW (left)
        self._state       = _State.TURNING
        self._state_until = now + duration


__all__ = ["FreeCoverage", "CoverageConfig", "CoverageResult"]
