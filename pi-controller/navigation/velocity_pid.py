"""
velocity_pid.py — Per-wheel velocity PID for closed-loop Mecanum drive.

Pi-side closed-loop velocity controller. Sits between NavController's body-twist
setpoint (vx, vy, wz) and the firmware's MOT:fr,fl,br,bl raw-PWM command. At
20 Hz it converts the body twist to per-wheel target RPM via Mecanum inverse
kinematics, compares against measured RPM from EncoderReader, and emits four
signed PWM values in [-255, 255].

Why Pi-side instead of firmware-side:
    Encoders live on a separate ESP32 from the motor controller, so a firmware
    PID would need a cross-ESP UART link. The Pi already has both the target
    (NavController) and the measurement (EncoderReader.get_encoder_counts()),
    so it is the natural place for the loop.

Architecture (per wheel):
    target_rpm  ─┐
                 ├─► error ──► PI ──► pwm_pi ──┐
    measured_rpm ┘                              ├─► clamp [-255, 255] ─► MOT:
                                                │
    target_rpm ──► Kff·target_rpm ──► pwm_ff ──┘

Feed-forward Kff is computed from the motor's no-load characteristic
(roughly PWM_at_max / RPM_at_max). The PI just trims residual error and
disturbance — the controller is responsive even with conservative gains.

Mecanum inverse kinematics (X-config, +vx forward, +vy left, +wz CCW):
    v_FL = vx - vy - wz·(lx + ly)
    v_FR = vx + vy + wz·(lx + ly)
    v_BL = vx + vy - wz·(lx + ly)
    v_BR = vx - vy + wz·(lx + ly)
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Dict, Tuple

from .odometry import ROBOT_LX, ROBOT_LY, WHEEL_RADIUS_M

WHEELS = ("FR", "FL", "BR", "BL")

# Convert wheel linear velocity (m/s) ↔ RPM. Wheel rad/s = v / R; RPM = rad/s · 60/(2π).
_MPS_TO_RPM = 60.0 / (2.0 * math.pi * WHEEL_RADIUS_M)
_LXLY = ROBOT_LX + ROBOT_LY


@dataclass
class VelocityPIDConfig:
    """Per-wheel PI gains + feed-forward."""
    kp: float = 0.4              # PWM per RPM error
    ki: float = 2.5              # PWM per (RPM·s)
    kff: float = 1.6             # PWM per RPM (open-loop characteristic)
    pwm_min: int = -255
    pwm_max: int = 255
    integral_clamp: float = 120.0   # anti-windup: |integral| ≤ this
    deadband_rpm: float = 1.0    # ignore tiny errors when target≈0
    max_target_rpm: float = 220.0   # safety: motor rating ~ JGA25-370 ≈ 230 rpm
    rpm_low_pass: float = 0.5    # 0=measured-only, 1=use last filtered (smoother)


@dataclass
class _WheelState:
    integral: float = 0.0
    last_error: float = 0.0
    rpm_filt: float = 0.0
    last_pwm: int = 0


class VelocityController:
    """Closed-loop per-wheel velocity controller for Mecanum drive.

    Usage (called at ~20 Hz):
        ctrl = VelocityController()
        pwm_fr, pwm_fl, pwm_br, pwm_bl, debug = ctrl.update(
            vx=0.3, vy=0.0, wz=0.0,
            measured_rpm={"FR": 95.2, "FL": 89.7, "BR": 92.1, "BL": 88.5},
        )
        ser.write(f"MOT:{pwm_fr},{pwm_fl},{pwm_br},{pwm_bl}\\n".encode())

    Reset state when re-entering AUTO mode or after a long pause to avoid
    integral kick.
    """

    def __init__(self, config: VelocityPIDConfig | None = None):
        self.cfg = config or VelocityPIDConfig()
        self._state: Dict[str, _WheelState] = {w: _WheelState() for w in WHEELS}
        self._last_t = 0.0

    # ------------------------------------------------------------------

    def reset(self) -> None:
        """Clear integrators and history. Call when entering an AUTO mode."""
        for w in WHEELS:
            self._state[w] = _WheelState()
        self._last_t = 0.0

    def body_twist_to_wheel_rpm(self, vx: float, vy: float, wz: float) -> Dict[str, float]:
        """Mecanum inverse kinematics. Returns target RPM per wheel.

        vx: m/s forward. vy: m/s left. wz: rad/s CCW.
        """
        v_fl = vx - vy - wz * _LXLY
        v_fr = vx + vy + wz * _LXLY
        v_bl = vx + vy - wz * _LXLY
        v_br = vx - vy + wz * _LXLY
        return {
            "FR": v_fr * _MPS_TO_RPM,
            "FL": v_fl * _MPS_TO_RPM,
            "BR": v_br * _MPS_TO_RPM,
            "BL": v_bl * _MPS_TO_RPM,
        }

    # ------------------------------------------------------------------

    def update(
        self,
        vx: float,
        vy: float,
        wz: float,
        measured_rpm: Dict[str, float],
        now: float | None = None,
    ) -> Tuple[int, int, int, int, Dict]:
        """Compute one PWM update. Returns (fr, fl, br, bl, debug_dict)."""
        now = now if now is not None else time.time()

        if self._last_t == 0.0:
            self._last_t = now
            dt = 0.05
        else:
            dt = max(1e-3, min(0.5, now - self._last_t))
            self._last_t = now

        targets = self.body_twist_to_wheel_rpm(vx, vy, wz)

        # Normalize if any wheel exceeds the motor's max RPM (preserve direction).
        max_abs = max(abs(t) for t in targets.values())
        if max_abs > self.cfg.max_target_rpm:
            scale = self.cfg.max_target_rpm / max_abs
            targets = {k: v * scale for k, v in targets.items()}

        pwms: Dict[str, int] = {}
        debug: Dict[str, Dict] = {}

        for wheel in WHEELS:
            tgt = targets[wheel]
            meas = float(measured_rpm.get(wheel, 0.0))
            st = self._state[wheel]

            # Optional smoothing on noisy encoder RPM (low-pass).
            a = self.cfg.rpm_low_pass
            st.rpm_filt = a * st.rpm_filt + (1.0 - a) * meas

            err = tgt - st.rpm_filt
            if abs(tgt) < self.cfg.deadband_rpm and abs(err) < self.cfg.deadband_rpm:
                # Settle to zero cleanly when commanded to stop.
                st.integral = 0.0
                err = 0.0

            # PI term
            st.integral = max(
                -self.cfg.integral_clamp,
                min(self.cfg.integral_clamp, st.integral + err * dt),
            )
            pwm_pi = self.cfg.kp * err + self.cfg.ki * st.integral

            # Feed-forward — open-loop estimate; PI corrects residual.
            pwm_ff = self.cfg.kff * tgt

            pwm = int(round(pwm_ff + pwm_pi))
            pwm = max(self.cfg.pwm_min, min(self.cfg.pwm_max, pwm))

            # Back-calculate integral on saturation to prevent further wind-up.
            if pwm == self.cfg.pwm_max or pwm == self.cfg.pwm_min:
                # Roll back the latest integration step if it pushed us into saturation.
                if (pwm == self.cfg.pwm_max and err > 0) or \
                   (pwm == self.cfg.pwm_min and err < 0):
                    st.integral -= err * dt
                    st.integral = max(
                        -self.cfg.integral_clamp,
                        min(self.cfg.integral_clamp, st.integral),
                    )

            st.last_error = err
            st.last_pwm   = pwm
            pwms[wheel]   = pwm
            debug[wheel]  = {
                "tgt":  round(tgt, 1),
                "meas": round(meas, 1),
                "err":  round(err, 1),
                "i":    round(st.integral, 2),
                "pwm":  pwm,
            }

        return (pwms["FR"], pwms["FL"], pwms["BR"], pwms["BL"],
                {"dt": round(dt, 3), "wheels": debug,
                 "tgt": {k: round(v, 1) for k, v in targets.items()}})

    # ------------------------------------------------------------------

    def diagnostics(self) -> Dict:
        return {
            "kp":  self.cfg.kp,
            "ki":  self.cfg.ki,
            "kff": self.cfg.kff,
            "wheels": {w: {
                "integral":   round(self._state[w].integral, 2),
                "last_error": round(self._state[w].last_error, 1),
                "last_pwm":   self._state[w].last_pwm,
            } for w in WHEELS},
        }


__all__ = ["VelocityController", "VelocityPIDConfig", "WHEELS"]
