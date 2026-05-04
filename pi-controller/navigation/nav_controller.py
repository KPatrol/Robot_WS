"""
nav_controller.py — K-Patrol Mecanum navigation controller.

Modes (5 total)
---------------
    MANUAL              — operator drives via /motor, controller is pass-through
    AUTO_FREE_COVERAGE  — autonomous indoor random-walk + frontier exploration
    AUTO_LINE_FOLLOW    — camera floor-line PD follower
    AUTO_GPS_WAYPOINT   — outdoor GPS waypoint route (Haversine planner +
                          IMU yaw fusion + reactive ToF safety)
    EMERGENCY           — motors stopped, waiting for operator clear

The three AUTO_* modes are mutually exclusive autonomous behaviours; switching
between them goes through MANUAL. EMERGENCY is the only mode that can be
entered from the safety bridge (fire detection or operator e-stop) and
requires explicit clear_emergency() to recover.
"""

from __future__ import annotations

import time
from enum import Enum
from typing import Any, Callable, Dict, List, Optional, Tuple

from .free_coverage import FreeCoverage, CoverageConfig
from .gps_navigator import GPSNavigator, GPSNavigatorConfig, Waypoint
from .line_follower import LineFollower, LineFollowerConfig


class Mode(str, Enum):
    MANUAL             = "MANUAL"
    AUTO_FREE_COVERAGE = "AUTO_FREE_COVERAGE"
    AUTO_LINE_FOLLOW   = "AUTO_LINE_FOLLOW"
    AUTO_GPS_WAYPOINT  = "AUTO_GPS_WAYPOINT"
    EMERGENCY          = "EMERGENCY"


_AUTO_MODES = {Mode.AUTO_FREE_COVERAGE, Mode.AUTO_LINE_FOLLOW, Mode.AUTO_GPS_WAYPOINT}


class NavController:
    """Navigation FSM: MANUAL + 3 AUTO_* + EMERGENCY."""

    def __init__(
        self,
        lf_config: Optional[LineFollowerConfig] = None,
        fc_config: Optional[CoverageConfig] = None,
        gps_config: Optional[GPSNavigatorConfig] = None,
    ):
        self._mode: Mode = Mode.MANUAL
        self._speed: int = 60  # percent — used as manual-mode default
        self._line_follower = LineFollower(lf_config)
        self._free_coverage = FreeCoverage(fc_config)
        self._gps_navigator = GPSNavigator(gps_config)
        # Stored so it can be re-applied if the line-follower is reinstantiated
        # later via auto_line_follow_start(lf_config=...).
        self._lf_front_distance_provider: Optional[Callable[[], Optional[float]]] = None
        # Detection→navigation safety bridge state
        self._alert_cooldowns:  Dict[str, float]    = {}
        self._alert_pause_until: float              = 0.0
        self._mode_before_pause: Optional[Mode]     = None
        self._last_alert: Optional[Dict[str, Any]]  = None

    # ── Mode control ──────────────────────────────────────────────
    def get_mode(self) -> str:
        return self._mode.value

    def is_auto(self) -> bool:
        return self._mode in _AUTO_MODES

    def set_mode(self, mode: str) -> bool:
        try:
            new_mode = Mode(str(mode).upper())
        except ValueError:
            return False
        # Tear down any active autonomous behaviour when leaving its mode.
        if self._mode == Mode.AUTO_GPS_WAYPOINT and new_mode != Mode.AUTO_GPS_WAYPOINT:
            self._gps_navigator.stop()
        if self._mode == Mode.AUTO_FREE_COVERAGE and new_mode != Mode.AUTO_FREE_COVERAGE:
            self._free_coverage.reset()
        self._mode = new_mode
        return True

    def set_speed(self, speed_pct: int) -> None:
        self._speed = max(0, min(100, int(speed_pct)))

    # ── AUTO_LINE_FOLLOW control ─────────────────────────────────
    def set_line_follow_front_distance_provider(
        self, provider: Optional[Callable[[], Optional[float]]]
    ) -> None:
        """Wire the front-facing ToF reader into the line follower's
        emergency cut-off path. Reapplied automatically on subsequent
        auto_line_follow_start() calls so a re-configured follower keeps
        the safety hookup."""
        self._lf_front_distance_provider = provider
        self._line_follower.set_front_distance_provider(provider)

    def auto_line_follow_start(
        self, lf_config: Optional[LineFollowerConfig] = None
    ) -> Dict[str, Any]:
        if self._mode not in (Mode.MANUAL, Mode.AUTO_LINE_FOLLOW):
            return {"ok": False, "error": f"cannot start line-follow from mode {self._mode.value}"}
        if lf_config is not None:
            self._line_follower = LineFollower(lf_config)
            # Reapply the wired ToF provider — a fresh LineFollower has none.
            if self._lf_front_distance_provider is not None:
                self._line_follower.set_front_distance_provider(
                    self._lf_front_distance_provider
                )
        else:
            self._line_follower.reset_pid()
        self._mode = Mode.AUTO_LINE_FOLLOW
        return {"ok": True}

    def auto_line_follow_stop(self) -> Dict[str, Any]:
        if self._mode == Mode.AUTO_LINE_FOLLOW:
            self._mode = Mode.MANUAL
        return {"ok": True}

    def auto_line_follow_tick(
        self,
        frame,                          # numpy BGR frame from camera
        produce_overlay: bool = True,
    ):
        return self._line_follower.tick(frame, produce_overlay=produce_overlay)

    # ── AUTO_FREE_COVERAGE control ───────────────────────────────
    def auto_free_coverage_start(
        self, fc_config: Optional[CoverageConfig] = None
    ) -> Dict[str, Any]:
        if self._mode not in (Mode.MANUAL, Mode.AUTO_FREE_COVERAGE):
            return {"ok": False, "error": f"cannot start free-coverage from mode {self._mode.value}"}
        if fc_config is not None:
            self._free_coverage = FreeCoverage(fc_config)
        else:
            self._free_coverage.reset()
        self._mode = Mode.AUTO_FREE_COVERAGE
        return {"ok": True}

    def auto_free_coverage_stop(self) -> Dict[str, Any]:
        if self._mode == Mode.AUTO_FREE_COVERAGE:
            self._mode = Mode.MANUAL
        return {"ok": True}

    # ── AUTO_GPS_WAYPOINT control ────────────────────────────────
    def auto_gps_waypoint_set_route(
        self, waypoints: List[Dict[str, Any]], loop: bool = False
    ) -> Dict[str, Any]:
        """Load a waypoint route. Each waypoint dict needs lat/lon (and optional
        radius_m, speed_pct, label). Does NOT change mode — call _start to run.
        """
        try:
            wps = [Waypoint.from_dict(w) for w in waypoints]
        except Exception as e:
            return {"ok": False, "error": f"invalid waypoint: {e}"}
        if not wps:
            return {"ok": False, "error": "empty route"}
        self._gps_navigator.set_route(wps, loop=bool(loop))
        return {"ok": True, "count": len(wps), "loop": bool(loop)}

    def auto_gps_waypoint_start(
        self, waypoints: Optional[List[Dict[str, Any]]] = None, loop: bool = False
    ) -> Dict[str, Any]:
        if waypoints is not None:
            r = self.auto_gps_waypoint_set_route(waypoints, loop=loop)
            if not r.get("ok"):
                return r
        if self._mode not in (Mode.MANUAL, Mode.AUTO_GPS_WAYPOINT):
            return {"ok": False, "error": f"cannot start GPS waypoint from mode {self._mode.value}"}
        if not self._gps_navigator.has_route():
            return {"ok": False, "error": "no route loaded"}
        self._gps_navigator.start()
        self._mode = Mode.AUTO_GPS_WAYPOINT
        return {"ok": True, "route": self._gps_navigator.route_summary()}

    def auto_gps_waypoint_stop(self) -> Dict[str, Any]:
        self._gps_navigator.stop()
        if self._mode == Mode.AUTO_GPS_WAYPOINT:
            self._mode = Mode.MANUAL
        return {"ok": True}

    # ── Detection → navigation safety bridge ─────────────────────
    # Reaction policy:
    #   fire   → immediate EMERGENCY stop (operator must clear_emergency)
    #   person → 5 s pause of any active autonomous mode, then auto-resume
    # Cooldowns prevent flapping when the detector keeps firing on the same target.
    _ALERT_COOLDOWNS_S = {"fire": 5.0, "person": 8.0}
    _PERSON_PAUSE_S    = 5.0

    def on_alert(
        self,
        kind: str,
        confidence: float,
        bbox: Optional[Tuple[int, int, int, int]] = None,
    ) -> Dict[str, Any]:
        kind = str(kind).lower().strip()
        now  = time.monotonic()

        cd_s = self._ALERT_COOLDOWNS_S.get(kind, 5.0)
        last = self._alert_cooldowns.get(kind, 0.0)
        if now - last < cd_s:
            return {
                "ok":          True,
                "action":      "cooldown",
                "kind":        kind,
                "remaining_s": round(cd_s - (now - last), 2),
            }
        self._alert_cooldowns[kind] = now
        self._last_alert = {
            "kind":       kind,
            "confidence": float(confidence),
            "bbox":       bbox,
            "ts":         now,
        }

        if kind == "fire":
            self._mode_before_pause = None
            self._alert_pause_until = 0.0
            self._mode              = Mode.EMERGENCY
            self._gps_navigator.stop()
            return {
                "ok":         True,
                "action":     "emergency_stop",
                "kind":       kind,
                "mode":       self._mode.value,
                "confidence": float(confidence),
            }

        if kind == "person":
            if self._mode == Mode.EMERGENCY:
                return {"ok": True, "action": "ignored_in_emergency", "kind": kind}
            self._alert_pause_until = now + self._PERSON_PAUSE_S
            if self._mode_before_pause is None and self._mode != Mode.MANUAL:
                self._mode_before_pause = self._mode
            return {
                "ok":         True,
                "action":     "pause",
                "kind":       kind,
                "duration_s": self._PERSON_PAUSE_S,
                "mode":       self._mode.value,
                "confidence": float(confidence),
            }

        return {"ok": True, "action": "noop", "kind": kind}

    def get_last_alert(self) -> Optional[Dict[str, Any]]:
        return self._last_alert

    def trigger_emergency(self, source: str = "operator") -> Dict[str, Any]:
        """External hard-stop entry-point (physical e-stop, remote relay, MQTT)."""
        self._mode_before_pause = None
        self._alert_pause_until = 0.0
        self._mode              = Mode.EMERGENCY
        self._gps_navigator.stop()
        self._last_alert = {
            "kind":       "estop",
            "confidence": 1.0,
            "bbox":       None,
            "ts":         time.monotonic(),
            "source":     source,
        }
        return {"ok": True, "action": "emergency_stop", "source": source}

    def clear_emergency(self) -> bool:
        if self._mode == Mode.EMERGENCY:
            self._mode = Mode.MANUAL
        self._alert_pause_until = 0.0
        self._mode_before_pause = None
        return True

    def diagnostics(self) -> Dict[str, Any]:
        """Snapshot of bridge + FSM state for telemetry / debug endpoints.

        Cheap, allocation-light dict — safe to call from a 1 Hz status publisher.
        """
        now = time.monotonic()
        cd_remaining: Dict[str, float] = {}
        for kind, last in self._alert_cooldowns.items():
            cd_s = self._ALERT_COOLDOWNS_S.get(kind, 5.0)
            rem = cd_s - (now - last)
            if rem > 0:
                cd_remaining[kind] = round(rem, 2)
        pause_rem = max(0.0, self._alert_pause_until - now) if self._alert_pause_until else 0.0
        return {
            "mode":                 self._mode.value,
            "speed":                self._speed,
            "is_auto":              self.is_auto(),
            "alert_cooldowns_s":    cd_remaining,
            "alert_pause_remaining_s": round(pause_rem, 2),
            "mode_before_pause":    self._mode_before_pause.value if self._mode_before_pause else None,
            "last_alert":           self._last_alert,
        }

    # ── Control-loop tick ────────────────────────────────────────
    def tick(
        self,
        tof_dict:       Dict[str, float],
        imu_yaw_deg:    float,
        encoder_counts: Optional[Dict[str, int]] = None,
        pose:           Optional[Any] = None,
        gps_data:       Optional[Any] = None,
    ) -> Tuple[Optional[str], Optional[int], Optional[Tuple[int, int, int, int]], Dict[str, Any]]:
        """One control-loop tick.

        Returns (motor_cmd, speed_pwm, twist, status).
            motor_cmd — "F|B|L|R|S|SL|SR" or None (None = MANUAL pass-through
                        or twist-only AUTO_* command).
            speed_pwm — int (0-255) to emit as SPD:<n>, or None (unchanged).
            twist     — (vx, vy, wz, spd) for `MEC:` command, or None.
        """
        status = self._base_status()

        # Person-detection pause window: hold motors at zero, auto-resume.
        now = time.monotonic()
        if self._alert_pause_until > now:
            status["alert_pause_remaining_s"] = round(self._alert_pause_until - now, 2)
            return "S", None, (0, 0, 0, 0), status
        if self._alert_pause_until and now >= self._alert_pause_until:
            self._alert_pause_until = 0.0
            self._mode_before_pause = None

        if self._mode == Mode.MANUAL:
            return None, None, None, status

        if self._mode == Mode.EMERGENCY:
            return "S", None, (0, 0, 0, 0), status

        if self._mode == Mode.AUTO_LINE_FOLLOW:
            # AUTO_LINE_FOLLOW does not use the tick() path — callers must use
            # auto_line_follow_tick(frame) directly and build their own MEC:.
            status["line_follow"] = "active"
            return None, None, None, status

        if self._mode == Mode.AUTO_FREE_COVERAGE:
            fc = self._free_coverage.tick(
                int(tof_dict.get("front", 9999)),
                int(tof_dict.get("left",  9999)),
                int(tof_dict.get("right", 9999)),
                pose=pose,
                all_tofs=tof_dict,
            )
            status["coverage_state"] = fc.state
            status["coverage"]       = self._free_coverage.diagnostics()
            return None, None, (fc.vx, fc.vy, fc.wz, fc.spd), status

        if self._mode == Mode.AUTO_GPS_WAYPOINT:
            gnav = self._gps_navigator.tick(
                gps_data=gps_data,
                imu_yaw_deg=imu_yaw_deg,
                tof_dict=tof_dict,
                speed_pct=self._speed,
            )
            status["gps_state"] = gnav.state
            status["gps"]       = self._gps_navigator.diagnostics()
            if gnav.done:
                # Route complete — drop back to MANUAL.
                self._mode = Mode.MANUAL
            return None, None, (gnav.vx, gnav.vy, gnav.wz, gnav.spd), status

        return None, None, None, status

    def _base_status(self) -> Dict[str, Any]:
        return {
            "mode":  self._mode.value,
            "speed": self._speed,
        }


__all__ = ["NavController", "Mode", "LineFollowerConfig", "CoverageConfig"]
