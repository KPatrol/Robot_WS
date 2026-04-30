"""
nav_controller.py — Minimal navigation controller for K-Patrol Mecanum.

Modes
-----
    MANUAL         — operator drives via /motor, controller is pass-through
    SCRIPT_PATROL  — executes a declarative patrol script
    LINE_FOLLOW    — camera floor-line following (PD controller)
    FREE_COVERAGE  — autonomous random-walk coverage of open indoor area
    EMERGENCY      — motors stopped, waiting for operator clear
"""

from __future__ import annotations

import time
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

from .script_patrol import (
    ExecutorState,
    PatrolScript,
    ScriptConfig,
    ScriptExecutor,
    ScriptLibrary,
)
from .script_recorder import ScriptRecorder
from .line_follower import LineFollower, LineFollowerConfig
from .free_coverage import FreeCoverage, CoverageConfig


class Mode(str, Enum):
    MANUAL        = "MANUAL"
    SCRIPT_PATROL = "SCRIPT_PATROL"
    LINE_FOLLOW   = "LINE_FOLLOW"
    FREE_COVERAGE = "FREE_COVERAGE"
    EMERGENCY     = "EMERGENCY"


DEFAULT_SCRIPT_DIR = "data/scripts"


class NavController:
    """Minimal navigation controller: MANUAL + SCRIPT_PATROL + LINE_FOLLOW + FREE_COVERAGE + EMERGENCY."""

    def __init__(
        self,
        script_dir: str = DEFAULT_SCRIPT_DIR,
        config: Optional[ScriptConfig] = None,
        lf_config: Optional[LineFollowerConfig] = None,
        fc_config: Optional[CoverageConfig] = None,
    ):
        self._mode: Mode = Mode.MANUAL
        self._speed: int = 60  # percent — used as manual-mode default
        self._executor = ScriptExecutor(config or ScriptConfig())
        self._library  = ScriptLibrary(script_dir)
        self._recorder = ScriptRecorder()
        self._active_script_name: Optional[str] = None
        self._line_follower = LineFollower(lf_config)
        self._free_coverage = FreeCoverage(fc_config)
        # Detection→navigation safety bridge state
        self._alert_cooldowns:  Dict[str, float]    = {}
        self._alert_pause_until: float              = 0.0
        self._mode_before_pause: Optional[Mode]     = None
        self._last_alert: Optional[Dict[str, Any]]  = None

    # ── Mode control ──────────────────────────────────────────────
    def get_mode(self) -> str:
        return self._mode.value

    def set_mode(self, mode: str) -> bool:
        try:
            new_mode = Mode(str(mode).upper())
        except ValueError:
            return False
        if self._mode == Mode.SCRIPT_PATROL and new_mode != Mode.SCRIPT_PATROL:
            self._executor.stop()
        self._mode = new_mode
        return True

    def set_speed(self, speed_pct: int) -> None:
        self._speed = max(0, min(100, int(speed_pct)))

    # ── Line follower control ─────────────────────────────────────
    def line_follow_start(self, lf_config: Optional[LineFollowerConfig] = None) -> Dict[str, Any]:
        """Enter LINE_FOLLOW mode.  Optionally replace the follower config."""
        if self._mode not in (Mode.MANUAL, Mode.LINE_FOLLOW):
            return {"ok": False, "error": f"cannot start line-follow from mode {self._mode.value}"}
        if lf_config is not None:
            self._line_follower = LineFollower(lf_config)
        else:
            self._line_follower.reset_pid()
        self._mode = Mode.LINE_FOLLOW
        return {"ok": True}

    def line_follow_stop(self) -> Dict[str, Any]:
        if self._mode == Mode.LINE_FOLLOW:
            self._mode = Mode.MANUAL
        return {"ok": True}

    def line_follow_tick(
        self,
        frame,                          # numpy BGR frame from camera
        produce_overlay: bool = True,
    ):
        """Run one line-follower tick.  Returns LineResult (vx/vy/wz/spd/overlay)."""
        return self._line_follower.tick(frame, produce_overlay=produce_overlay)

    # ── Free coverage control ─────────────────────────────────────
    def free_coverage_start(self, fc_config: Optional[CoverageConfig] = None) -> Dict[str, Any]:
        """Enter FREE_COVERAGE mode.  Optionally replace the coverage config."""
        if self._mode not in (Mode.MANUAL, Mode.FREE_COVERAGE):
            return {"ok": False, "error": f"cannot start free-coverage from mode {self._mode.value}"}
        if fc_config is not None:
            self._free_coverage = FreeCoverage(fc_config)
        else:
            self._free_coverage.reset()
        self._mode = Mode.FREE_COVERAGE
        return {"ok": True}

    def free_coverage_stop(self) -> Dict[str, Any]:
        if self._mode == Mode.FREE_COVERAGE:
            self._mode = Mode.MANUAL
        return {"ok": True}

    def free_coverage_tick(
        self,
        front_mm: int,
        left_mm:  int,
        right_mm: int,
        pose:     Optional[Any] = None,
        all_tofs: Optional[Dict[str, float]] = None,
    ):
        """Run one coverage tick.  Returns CoverageResult(vx, vy, wz, spd, state).

        pose:     Optional Pose (from Odometry.get_pose()). Without it, the
                  coverage navigator falls back to memoryless random-walk +
                  wall-follow. With it, the OccupancyGrid records visits and
                  enables frontier exploration.
        all_tofs: Optional full ToF dict ({"front", "left", "right",
                  "front_left", "front_right", "back", ...}) for richer ray
                  casting into the map. Defaults to the 3 cardinals.
        """
        return self._free_coverage.tick(
            front_mm, left_mm, right_mm, pose=pose, all_tofs=all_tofs
        )

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
        """Bridge an AnomalyDetector event into the navigation FSM.

        Returns {ok, action, mode, cooldown_s, ...}. Safe to call from any
        thread; touches only simple scalars on the controller instance.
        """
        kind = str(kind).lower().strip()
        now  = time.monotonic()

        # Spam guard: same-kind alerts within cooldown are dropped.
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
            # Hard-stop. Operator must explicitly clear_emergency() to recover.
            self._mode_before_pause = None
            self._alert_pause_until = 0.0
            self._mode              = Mode.EMERGENCY
            self._executor.stop()
            return {
                "ok":         True,
                "action":     "emergency_stop",
                "kind":       kind,
                "mode":       self._mode.value,
                "confidence": float(confidence),
            }

        if kind == "person":
            # Soft-pause: idle motors for N seconds, then resume previous mode.
            # Skip if already paused (refresh window) or in EMERGENCY.
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

        # Unknown kind → log only.
        return {"ok": True, "action": "noop", "kind": kind}

    def get_last_alert(self) -> Optional[Dict[str, Any]]:
        return self._last_alert

    def clear_emergency(self) -> bool:
        cleared = self._executor.clear_emergency()
        if self._mode == Mode.EMERGENCY:
            self._mode = Mode.MANUAL
        # Drop any pending pause when operator clears the alarm explicitly.
        self._alert_pause_until = 0.0
        self._mode_before_pause = None
        return cleared

    # ── Script library ────────────────────────────────────────────
    def script_list(self) -> List[Dict[str, Any]]:
        out: List[Dict[str, Any]] = []
        for n in self._library.list_scripts():
            try:
                sc = self._library.load(n)
            except Exception:
                continue
            out.append({
                "name":              sc.name,
                "steps":             len(sc.steps),
                "loop":              sc.loop,
                "default_speed_pct": sc.default_speed_pct,
            })
        return out

    def script_save(self, data: Dict[str, Any]) -> Dict[str, Any]:
        try:
            script = PatrolScript.from_dict(data)
            path = self._library.save(script)
            return {"ok": True, "name": script.name, "path": path}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def script_delete(self, name: str) -> bool:
        try:
            return self._library.delete(name)
        except Exception:
            return False

    def script_load(self, name: str) -> Dict[str, Any]:
        try:
            script = self._library.load(name)
        except FileNotFoundError:
            return {"ok": False, "error": f"script not found: {name}"}
        except Exception as e:
            return {"ok": False, "error": str(e)}
        errors = self._executor.load(script)
        if errors:
            return {"ok": False, "errors": errors}
        self._active_script_name = script.name
        return {"ok": True, "name": script.name, "steps": len(script.steps)}

    def script_start(self, name: Optional[str] = None) -> Dict[str, Any]:
        if name:
            result = self.script_load(name)
            if not result.get("ok"):
                return result
        if not self._executor.start():
            return {"ok": False, "error": self._executor.status().get("error")}
        self._mode = Mode.SCRIPT_PATROL
        return {"ok": True, "name": self._active_script_name}

    def script_stop(self) -> Dict[str, Any]:
        self._executor.stop()
        if self._mode == Mode.SCRIPT_PATROL:
            self._mode = Mode.MANUAL
        return {"ok": True}

    # ── Recorder ─────────────────────────────────────────────────
    def record_start(self, name: str) -> Dict[str, Any]:
        if self._mode != Mode.MANUAL:
            return {"ok": False, "error": "recorder only works in MANUAL mode"}
        try:
            self._recorder.start(name)
            return {"ok": True, "name": name}
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def record_stop(self, imu_yaw_deg: float) -> Dict[str, Any]:
        if not self._recorder.active:
            return {"ok": False, "error": "recorder not active"}
        try:
            script = self._recorder.stop(float(imu_yaw_deg))
            if not script.steps:
                return {"ok": False, "error": "no steps recorded", "name": script.name}
            path = self._library.save(script)
            return {
                "ok":    True,
                "name":  script.name,
                "steps": len(script.steps),
                "path":  path,
            }
        except Exception as e:
            return {"ok": False, "error": str(e)}

    def record_cancel(self) -> Dict[str, Any]:
        self._recorder.cancel()
        return {"ok": True}

    def record_feed(
        self,
        cmd:         Optional[str],
        speed_pwm:   Optional[int],
        imu_yaw_deg: float,
    ) -> None:
        if not self._recorder.active or not cmd:
            return
        try:
            self._recorder.feed_motor_command(cmd, speed_pwm, float(imu_yaw_deg))
        except Exception:
            pass

    def record_status(self) -> Dict[str, Any]:
        return self._recorder.status()

    # ── Control-loop tick ────────────────────────────────────────
    def tick(
        self,
        tof_dict:       Dict[str, float],
        imu_yaw_deg:    float,
        encoder_counts: Optional[Dict[str, int]] = None,
        pose:           Optional[Any] = None,
    ) -> Tuple[Optional[str], Optional[int], Optional[Tuple[int, int, int, int]], Dict[str, Any]]:
        """One control-loop tick.

        Returns (motor_cmd, speed_pwm, twist, status).
            motor_cmd — "F|B|L|R|S|SL|SR" or None (None = MANUAL pass-through)
            speed_pwm — int (0-255) to emit as SPD:<n>, or None (unchanged)
            twist     — (vx, vy, wz, spd) for `MEC:` command, or None
                        (None = MEC disabled or MANUAL mode).
        """
        status = self._base_status()

        # Person-detection pause window: hold all motors at zero, status reports
        # remaining time. Auto-resume when window elapses (mode unchanged).
        now = time.monotonic()
        if self._alert_pause_until > now:
            status["alert_pause_remaining_s"] = round(self._alert_pause_until - now, 2)
            return "S", None, (0, 0, 0, 0), status
        if self._alert_pause_until and now >= self._alert_pause_until:
            # Pause just elapsed — clear bookkeeping; mode is already correct.
            self._alert_pause_until = 0.0
            self._mode_before_pause = None

        if self._mode == Mode.MANUAL:
            return None, None, None, status

        if self._mode == Mode.EMERGENCY:
            return "S", None, (0, 0, 0, 0), status

        if self._mode == Mode.LINE_FOLLOW:
            # LINE_FOLLOW does not use the tick() path — callers must use
            # line_follow_tick(frame) directly and build their own MEC: command.
            # Here we return a safe stop so the main loop idles the motors.
            status["line_follow"] = "active"
            return None, None, None, status

        if self._mode == Mode.FREE_COVERAGE:
            # FREE_COVERAGE uses the tick() path with current ToF readings.
            # Pose enables OccupancyGrid memory + frontier exploration; without
            # it the coverage navigator runs in memoryless random-walk mode.
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

        # SCRIPT_PATROL
        tick = self._executor.tick(imu_yaw_deg, tof_dict, encoder_counts=encoder_counts)
        status.update(tick.status)

        if tick.status.get("state") == ExecutorState.EMERGENCY.value:
            self._mode = Mode.EMERGENCY

        return tick.motor_cmd, tick.speed_pwm, tick.twist, status

    def _base_status(self) -> Dict[str, Any]:
        return {
            "mode":          self._mode.value,
            "speed":         self._speed,
            "active_script": self._active_script_name,
            "recorder":      self._recorder.status(),
        }


__all__ = ["NavController", "Mode", "LineFollowerConfig", "CoverageConfig"]
