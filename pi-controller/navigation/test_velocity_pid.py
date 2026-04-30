"""Unit tests for VelocityController — Mecanum inverse kinematics + per-wheel PI.

Run:  python -m pytest robots/pi-controller/navigation/test_velocity_pid.py
Or:   python -m unittest robots.pi-controller.navigation.test_velocity_pid
"""

from __future__ import annotations

import math
import unittest

from .velocity_pid import (
    VelocityController,
    VelocityPIDConfig,
    WHEELS,
)
from .odometry import ROBOT_LX, ROBOT_LY, WHEEL_RADIUS_M


class InverseKinematics(unittest.TestCase):
    def setUp(self):
        self.ctrl = VelocityController()

    def test_pure_forward_all_wheels_equal_positive(self):
        rpm = self.ctrl.body_twist_to_wheel_rpm(vx=0.3, vy=0.0, wz=0.0)
        # All four wheels should spin forward at the same rate.
        for w in WHEELS:
            self.assertGreater(rpm[w], 0)
        for w in WHEELS:
            self.assertAlmostEqual(rpm[w], rpm["FR"], places=4)

    def test_pure_strafe_left_diagonal_pairs_oppose(self):
        # vy>0 (left) → FL & BR negative; FR & BL positive.
        rpm = self.ctrl.body_twist_to_wheel_rpm(vx=0.0, vy=0.2, wz=0.0)
        self.assertLess(rpm["FL"], 0)
        self.assertLess(rpm["BR"], 0)
        self.assertGreater(rpm["FR"], 0)
        self.assertGreater(rpm["BL"], 0)

    def test_pure_rotation_ccw_left_side_negative(self):
        # wz>0 (CCW) → left wheels (FL, BL) reverse, right wheels (FR, BR) forward.
        rpm = self.ctrl.body_twist_to_wheel_rpm(vx=0.0, vy=0.0, wz=0.5)
        self.assertLess(rpm["FL"], 0)
        self.assertLess(rpm["BL"], 0)
        self.assertGreater(rpm["FR"], 0)
        self.assertGreater(rpm["BR"], 0)

    def test_zero_twist_returns_zero(self):
        rpm = self.ctrl.body_twist_to_wheel_rpm(0.0, 0.0, 0.0)
        for w in WHEELS:
            self.assertEqual(rpm[w], 0.0)


class TargetNormalization(unittest.TestCase):
    def test_targets_clamped_to_max_rpm(self):
        cfg = VelocityPIDConfig(max_target_rpm=100.0)
        ctrl = VelocityController(cfg)
        # Command something that would exceed max.
        fr, fl, br, bl, dbg = ctrl.update(
            vx=5.0, vy=0.0, wz=0.0,
            measured_rpm={w: 0.0 for w in WHEELS},
            now=1000.0,
        )
        # All wheel targets should be capped at ≤ max_target_rpm.
        for w in WHEELS:
            self.assertLessEqual(abs(dbg["tgt"][w]), cfg.max_target_rpm + 0.5)


class FeedforwardAndPI(unittest.TestCase):
    def test_pwm_positive_for_forward_target(self):
        ctrl = VelocityController()
        fr, fl, br, bl, _ = ctrl.update(
            vx=0.3, vy=0.0, wz=0.0,
            measured_rpm={w: 0.0 for w in WHEELS},
            now=1000.0,
        )
        for pwm in (fr, fl, br, bl):
            self.assertGreater(pwm, 0)

    def test_pwm_clamped_to_signed_byte(self):
        ctrl = VelocityController()
        fr, fl, br, bl, _ = ctrl.update(
            vx=10.0, vy=0.0, wz=0.0,
            measured_rpm={w: 0.0 for w in WHEELS},
            now=1000.0,
        )
        for pwm in (fr, fl, br, bl):
            self.assertGreaterEqual(pwm, -255)
            self.assertLessEqual(pwm, 255)

    def test_zero_target_zero_measured_yields_no_pwm(self):
        ctrl = VelocityController()
        # First call initializes dt; second call exercises the deadband.
        ctrl.update(vx=0.0, vy=0.0, wz=0.0,
                    measured_rpm={w: 0.0 for w in WHEELS}, now=1000.0)
        fr, fl, br, bl, _ = ctrl.update(
            vx=0.0, vy=0.0, wz=0.0,
            measured_rpm={w: 0.0 for w in WHEELS},
            now=1000.05,
        )
        self.assertEqual((fr, fl, br, bl), (0, 0, 0, 0))

    def test_measurement_matches_target_settles(self):
        # If the wheels are already at the commanded RPM, PI error → 0,
        # so PWM ≈ Kff·target. Drive a few ticks to let the low-pass settle.
        cfg = VelocityPIDConfig()
        ctrl = VelocityController(cfg)
        target_rpm = ctrl.body_twist_to_wheel_rpm(0.3, 0.0, 0.0)
        t = 1000.0
        for _ in range(10):
            t += 0.05
            ctrl.update(vx=0.3, vy=0.0, wz=0.0,
                        measured_rpm=target_rpm, now=t)
        fr, fl, br, bl, dbg = ctrl.update(
            vx=0.3, vy=0.0, wz=0.0,
            measured_rpm=target_rpm, now=t + 0.05,
        )
        # Errors should be small (< 1 RPM noise after settling).
        for w in WHEELS:
            self.assertLess(abs(dbg["wheels"][w]["err"]), 1.5)


class AntiWindup(unittest.TestCase):
    def test_integral_clamp_respected(self):
        cfg = VelocityPIDConfig(integral_clamp=50.0, ki=10.0)
        ctrl = VelocityController(cfg)
        t = 1000.0
        # Persistent large error → integral grows, but must clamp.
        for _ in range(50):
            t += 0.05
            ctrl.update(vx=0.5, vy=0.0, wz=0.0,
                        measured_rpm={w: 0.0 for w in WHEELS}, now=t)
        diag = ctrl.diagnostics()
        for w in WHEELS:
            self.assertLessEqual(abs(diag["wheels"][w]["integral"]), cfg.integral_clamp + 1e-6)


class ResetClearsState(unittest.TestCase):
    def test_reset_zeros_integral_and_history(self):
        ctrl = VelocityController()
        t = 1000.0
        # Wind up the integral.
        for _ in range(20):
            t += 0.05
            ctrl.update(vx=0.5, vy=0.0, wz=0.0,
                        measured_rpm={w: 0.0 for w in WHEELS}, now=t)
        ctrl.reset()
        diag = ctrl.diagnostics()
        for w in WHEELS:
            self.assertEqual(diag["wheels"][w]["integral"], 0.0)
            self.assertEqual(diag["wheels"][w]["last_pwm"], 0)


class DiagnosticsShape(unittest.TestCase):
    def test_diagnostics_keys(self):
        ctrl = VelocityController()
        d = ctrl.diagnostics()
        self.assertIn("kp", d)
        self.assertIn("ki", d)
        self.assertIn("kff", d)
        self.assertIn("wheels", d)
        for w in WHEELS:
            self.assertIn(w, d["wheels"])
            for k in ("integral", "last_error", "last_pwm"):
                self.assertIn(k, d["wheels"][w])


class UpdateOutputShape(unittest.TestCase):
    def test_update_returns_5_tuple(self):
        ctrl = VelocityController()
        out = ctrl.update(
            vx=0.0, vy=0.0, wz=0.0,
            measured_rpm={w: 0.0 for w in WHEELS}, now=1000.0,
        )
        self.assertEqual(len(out), 5)
        fr, fl, br, bl, dbg = out
        for v in (fr, fl, br, bl):
            self.assertIsInstance(v, int)
        self.assertIn("dt", dbg)
        self.assertIn("wheels", dbg)
        self.assertIn("tgt", dbg)


if __name__ == "__main__":
    unittest.main()
