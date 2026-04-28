"""
nav_controller.py — Minimal navigation controller for K-Patrol Mecanum.

Modes
-----
    MANUAL         — operator drives via /motor, controller is pass-through
    SCRIPT_PATROL  — executes a declarative patrol script
    EMERGENCY      — motors stopped, waiting for operator clear

Everything else (mapping, odometry, line/marker, teach-replay) has been
removed in favor of deterministic scripted patrols.
"""

from __future__ import annotations

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


class Mode(str, Enum):
    MANUAL        = "MANUAL"
    SCRIPT_PATROL = "SCRIPT_PATROL"
    EMERGENCY     = "EMERGENCY"


DEFAULT_SCRIPT_DIR = "data/scripts"


class NavController:
    """Minimal navigation controller: MANUAL + SCRIPT_PATROL + EMERGENCY."""

    def __init__(
        self,
        script_dir: str = DEFAULT_SCRIPT_DIR,
        config: Optional[ScriptConfig] = None,
    ):
        self._mode: Mode = Mode.MANUAL
        self._speed: int = 60  # percent — used as manual-mode default
        self._executor = ScriptExecutor(config or ScriptConfig())
        self._library  = ScriptLibrary(script_dir)
        self._recorder = ScriptRecorder()
        self._active_script_name: Optional[str] = None

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

    def clear_emergency(self) -> bool:
        cleared = self._executor.clear_emergency()
        if self._mode == Mode.EMERGENCY:
            self._mode = Mode.MANUAL
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
    ) -> Tuple[Optional[str], Optional[int], Optional[Tuple[int, int, int, int]], Dict[str, Any]]:
        """One control-loop tick.

        Returns (motor_cmd, speed_pwm, twist, status).
            motor_cmd — "F|B|L|R|S|SL|SR" or None (None = MANUAL pass-through)
            speed_pwm — int (0-255) to emit as SPD:<n>, or None (unchanged)
            twist     — (vx, vy, wz, spd) for `MEC:` command, or None
                        (None = MEC disabled or MANUAL mode).
        """
        status = self._base_status()

        if self._mode == Mode.MANUAL:
            return None, None, None, status

        if self._mode == Mode.EMERGENCY:
            return "S", None, (0, 0, 0, 0), status

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


__all__ = ["NavController", "Mode"]
