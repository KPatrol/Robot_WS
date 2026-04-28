"""
script_recorder.py — Record operator manual driving into a PatrolScript
=======================================================================
While the robot is in MANUAL mode, observe motor commands and translate
each sustained command into a declarative script primitive.

Translation rules
-----------------
    F   → forward_time   (duration = time F sustained)
    B   → backward_time
    SL  → strafe_time direction=left
    SR  → strafe_time direction=right
    L   → rotate direction=left   (angle from IMU yaw delta)
    R   → rotate direction=right  (angle from IMU yaw delta)
    S   (sustained ≥ min_pause_duration_s) → pause step

Short bounces below `min_step_duration_s` are discarded, so tapping a
button briefly will not pollute the recorded script. Unknown commands
(DL, DR, LIGHT_*, SPD:*) are ignored.

Usage
-----
    rec = ScriptRecorder()
    rec.start("my_patrol")
    # on every motor command:
    rec.feed_motor_command(cmd, speed_pwm, imu_yaw_deg)
    # when operator hits stop:
    script = rec.stop(imu_yaw_deg)
    # script is a PatrolScript ready for ScriptLibrary.save()
"""

from __future__ import annotations

import time
from typing import Any, Dict, List, Optional

from .script_patrol import PatrolScript, ScriptStep, signed_angle_diff


class ScriptRecorder:
    _TRANSLATIONAL = {"F", "B", "SL", "SR"}
    _ROTATIONAL    = {"L", "R"}

    def __init__(
        self,
        min_step_duration_s:  float = 0.15,
        min_rotate_deg:       float = 5.0,
        min_pause_duration_s: float = 1.0,
    ):
        self.min_step_duration_s  = float(min_step_duration_s)
        self.min_rotate_deg       = float(min_rotate_deg)
        self.min_pause_duration_s = float(min_pause_duration_s)

        self.active: bool = False
        self.name:   Optional[str] = None
        self.steps:  List[ScriptStep] = []

        self._current_cmd:       Optional[str] = None
        self._current_start:     float = 0.0
        self._current_yaw:       float = 0.0
        self._current_speed_pct: int   = 60
        self._last_stop_at:      Optional[float] = None
        self._recording_start:   float = 0.0

    # ---- lifecycle ----
    def start(self, name: str) -> None:
        name = (name or "").strip()
        if not name or not all(c.isalnum() or c in ("-", "_") for c in name):
            raise ValueError(f"invalid recorder name: {name!r}")
        self.active            = True
        self.name              = name
        self.steps             = []
        self._current_cmd      = None
        self._last_stop_at     = None
        self._recording_start  = time.time()

    def stop(self, imu_yaw_deg: float, now: Optional[float] = None) -> PatrolScript:
        name = self.name or "recording"
        if not self.active:
            return PatrolScript(name=name, steps=[], loop=False)
        now = now if now is not None else time.time()
        self._flush_current(imu_yaw_deg, now)
        self.active       = False
        self._current_cmd = None
        return PatrolScript(
            name              = name,
            steps             = list(self.steps),
            loop              = False,
            default_speed_pct = 60,
        )

    def cancel(self) -> None:
        self.active       = False
        self.steps        = []
        self._current_cmd = None
        self._last_stop_at = None

    # ---- feed ----
    def feed_motor_command(
        self,
        cmd:         str,
        speed_pwm:   Optional[int],
        imu_yaw_deg: float,
        now:         Optional[float] = None,
    ) -> None:
        if not self.active or not cmd:
            return

        cmd_u = str(cmd).strip().upper()
        if cmd_u not in self._TRANSLATIONAL and cmd_u not in self._ROTATIONAL and cmd_u != "S":
            return

        now = now if now is not None else time.time()

        # Same command sustained → nothing to do
        if cmd_u == self._current_cmd:
            return

        # End any active motion
        if self._current_cmd is not None:
            self._flush_current(imu_yaw_deg, now)

        if cmd_u == "S":
            self._last_stop_at = now
            self._current_cmd  = None
            return

        # New motion starts — emit pending pause (if long enough)
        if self._last_stop_at is not None:
            pause_dur = now - self._last_stop_at
            if pause_dur >= self.min_pause_duration_s:
                self.steps.append(ScriptStep(
                    op         = "pause",
                    duration_s = round(pause_dur, 2),
                ))
            self._last_stop_at = None

        self._current_cmd       = cmd_u
        self._current_start     = now
        self._current_yaw       = float(imu_yaw_deg)
        if speed_pwm is not None:
            self._current_speed_pct = self._pwm_to_pct(int(speed_pwm))

    # ---- internals ----
    def _flush_current(self, imu_yaw_deg: float, now: float) -> None:
        cmd = self._current_cmd
        if cmd is None:
            return

        if cmd in self._ROTATIONAL:
            delta = abs(signed_angle_diff(float(imu_yaw_deg), self._current_yaw))
            if delta >= self.min_rotate_deg:
                self.steps.append(ScriptStep(
                    op        = "rotate",
                    angle_deg = round(min(delta, 180.0), 1),
                    direction = "left" if cmd == "L" else "right",
                    speed_pct = self._current_speed_pct,
                ))
        else:
            duration = now - self._current_start
            if duration >= self.min_step_duration_s:
                op, extra = self._translate_op(cmd)
                kwargs: Dict[str, Any] = {
                    "op":         op,
                    "duration_s": round(duration, 2),
                    "speed_pct":  self._current_speed_pct,
                }
                kwargs.update(extra)
                self.steps.append(ScriptStep(**kwargs))

        self._current_cmd = None

    @staticmethod
    def _translate_op(cmd: str):
        if cmd == "F":  return "forward_time",  {}
        if cmd == "B":  return "backward_time", {}
        if cmd == "SL": return "strafe_time",   {"direction": "left"}
        if cmd == "SR": return "strafe_time",   {"direction": "right"}
        return "pause", {}

    @staticmethod
    def _pwm_to_pct(pwm: int) -> int:
        try:
            pct = int(round(int(pwm) * 100 / 255))
        except (TypeError, ValueError):
            pct = 60
        return max(0, min(100, pct))

    def status(self) -> Dict[str, Any]:
        elapsed = (time.time() - self._recording_start) if self.active else 0.0
        return {
            "active":        self.active,
            "name":          self.name,
            "step_count":    len(self.steps),
            "elapsed_s":     round(elapsed, 1),
            "current_cmd":   self._current_cmd,
        }


__all__ = ["ScriptRecorder"]
