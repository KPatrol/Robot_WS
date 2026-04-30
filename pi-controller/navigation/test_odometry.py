"""Unit tests for Odometry — Mecanum forward kinematics + IMU fusion.

Run:  python -m pytest robots/pi-controller/navigation/test_odometry.py
Or:   python -m unittest robots.pi-controller.navigation.test_odometry

Encoder counts are simulated by reasoning backwards through the inverse
kinematics: pick a desired body twist, compute the wheel angular velocity
that would produce it, and translate that into a per-tick count delta.
"""

from __future__ import annotations

import math
import unittest

from .odometry import (
    COUNTS_PER_REV,
    Odometry,
    Pose,
    ROBOT_LX,
    ROBOT_LY,
    WHEEL_RADIUS_M,
)


_LXLY = ROBOT_LX + ROBOT_LY


def _wheel_omega_from_twist(vx: float, vy: float, wz: float) -> dict:
    """Inverse kinematics — body twist → per-wheel ω (rad/s)."""
    v_fl = vx - vy - wz * _LXLY
    v_fr = vx + vy + wz * _LXLY
    v_bl = vx + vy - wz * _LXLY
    v_br = vx - vy + wz * _LXLY
    return {
        "FL": v_fl / WHEEL_RADIUS_M,
        "FR": v_fr / WHEEL_RADIUS_M,
        "BL": v_bl / WHEEL_RADIUS_M,
        "BR": v_br / WHEEL_RADIUS_M,
    }


def _counts_from_omega(omega_rps: dict, dt: float, prev: dict) -> dict:
    """Convert wheel ω + dt → cumulative counts delta added to prev."""
    out = {}
    for k, w in omega_rps.items():
        d_counts = int(round(w * dt * COUNTS_PER_REV / (2 * math.pi)))
        out[k] = prev[k] + d_counts
    return out


class InitAndReset(unittest.TestCase):
    def test_pose_starts_at_origin(self):
        odo = Odometry()
        p = odo.get_pose()
        self.assertEqual((p.x, p.y, p.theta), (0.0, 0.0, 0.0))
        self.assertEqual(p.source, "fused")

    def test_first_update_initializes_without_motion(self):
        odo = Odometry()
        # First call should anchor counts; pose should not move even with non-zero counts.
        p = odo.update({"FR": 500, "FL": 500, "BR": 500, "BL": 500},
                       imu_yaw_deg=0.0, now=1000.0)
        self.assertAlmostEqual(p.x, 0.0)
        self.assertAlmostEqual(p.y, 0.0)
        self.assertAlmostEqual(p.theta, 0.0)

    def test_reset_to_arbitrary_pose(self):
        odo = Odometry()
        odo.update({"FR": 0, "FL": 0, "BR": 0, "BL": 0}, now=1000.0)
        odo.reset(x=2.5, y=-1.5, theta_deg=90.0)
        p = odo.get_pose()
        self.assertAlmostEqual(p.x, 2.5)
        self.assertAlmostEqual(p.y, -1.5)
        self.assertAlmostEqual(p.theta, math.radians(90.0))


class ForwardKinematicsSanity(unittest.TestCase):
    """Drive synthetic encoder counts and check the recovered body twist + pose."""

    def setUp(self):
        # IMU disabled so encoders are the only signal — easier to assert exact values.
        self.odo = Odometry(imu_alpha=0.0)
        self.t = 1000.0
        self.dt = 0.05  # 20 Hz
        self.counts = {"FR": 0, "FL": 0, "BR": 0, "BL": 0}
        # Anchor counts.
        self.odo.update(self.counts, imu_yaw_deg=None, now=self.t)

    def _step(self, vx: float, vy: float, wz: float) -> Pose:
        omega = _wheel_omega_from_twist(vx, vy, wz)
        self.t += self.dt
        self.counts = _counts_from_omega(omega, self.dt, self.counts)
        return self.odo.update(self.counts, imu_yaw_deg=None, now=self.t)

    def test_pure_forward_drives_x_only(self):
        # 0.3 m/s for 1 s = ~0.3 m forward, no y, no theta.
        for _ in range(20):
            self._step(vx=0.3, vy=0.0, wz=0.0)
        p = self.odo.get_pose()
        self.assertAlmostEqual(p.x, 0.3, delta=0.02)
        self.assertAlmostEqual(p.y, 0.0, delta=0.01)
        self.assertAlmostEqual(p.theta, 0.0, delta=0.02)
        self.assertGreater(p.vx, 0.25)

    def test_pure_strafe_drives_y_only(self):
        for _ in range(20):
            self._step(vx=0.0, vy=0.2, wz=0.0)
        p = self.odo.get_pose()
        self.assertAlmostEqual(p.x, 0.0, delta=0.01)
        self.assertAlmostEqual(p.y, 0.2, delta=0.02)
        self.assertAlmostEqual(p.theta, 0.0, delta=0.02)
        self.assertGreater(p.vy, 0.15)

    def test_pure_rotation_drives_theta_only(self):
        # 0.5 rad/s for 1 s = 0.5 rad CCW, no translation.
        for _ in range(20):
            self._step(vx=0.0, vy=0.0, wz=0.5)
        p = self.odo.get_pose()
        self.assertAlmostEqual(p.x, 0.0, delta=0.02)
        self.assertAlmostEqual(p.y, 0.0, delta=0.02)
        self.assertAlmostEqual(p.theta, 0.5, delta=0.05)
        self.assertGreater(p.omega, 0.4)

    def test_combined_twist_recovers_diagonal_motion(self):
        # Drive forward + left strafe = diagonal.
        for _ in range(20):
            self._step(vx=0.2, vy=0.2, wz=0.0)
        p = self.odo.get_pose()
        self.assertAlmostEqual(p.x, 0.2, delta=0.03)
        self.assertAlmostEqual(p.y, 0.2, delta=0.03)


class IMUFusion(unittest.TestCase):
    def test_imu_complementary_filter_pulls_theta(self):
        # alpha=0.5 → IMU and encoder weight equally per step.
        odo = Odometry(imu_alpha=0.5)
        t = 1000.0
        # Anchor at 0 deg.
        odo.update({"FR": 0, "FL": 0, "BR": 0, "BL": 0}, imu_yaw_deg=0.0, now=t)
        # Encoders say 0 rotation, but IMU says 30° → fused should drift toward IMU.
        # The first call after init will use IMU-anchor logic so we step a few times.
        for _ in range(10):
            t += 0.05
            odo.update({"FR": 0, "FL": 0, "BR": 0, "BL": 0},
                       imu_yaw_deg=30.0, now=t)
        p = odo.get_pose()
        # Theta should have moved meaningfully toward 30° (the encoder said 0 but IMU
        # delta yaw is +30°, so fusion should converge above 0 over many steps).
        self.assertGreater(p.theta, math.radians(5.0))
        self.assertEqual(p.source, "fused")

    def test_encoder_only_fallback(self):
        odo = Odometry(imu_alpha=0.5)
        odo.update({"FR": 0, "FL": 0, "BR": 0, "BL": 0}, imu_yaw_deg=None, now=1000.0)
        p = odo.update({"FR": 100, "FL": 100, "BR": 100, "BL": 100},
                       imu_yaw_deg=None, now=1000.05)
        self.assertEqual(p.source, "encoder")


class WrapAndDiagnostics(unittest.TestCase):
    def test_theta_wrap_around_pi(self):
        odo = Odometry(imu_alpha=0.0)
        # Reset near +π so encoder rotation pushes us across the discontinuity.
        odo.reset(theta_deg=170.0)
        t = 1000.0
        counts = {"FR": 0, "FL": 0, "BR": 0, "BL": 0}
        odo.update(counts, now=t)
        # Drive +0.5 rad/s for 1 s → +28.6°, crosses +π → wraps to negative.
        for _ in range(20):
            t += 0.05
            omega = _wheel_omega_from_twist(0.0, 0.0, 0.5)
            counts = _counts_from_omega(omega, 0.05, counts)
            odo.update(counts, now=t)
        p = odo.get_pose()
        self.assertGreaterEqual(p.theta, -math.pi)
        self.assertLessEqual(p.theta, math.pi)

    def test_diagnostics_keys(self):
        odo = Odometry()
        d = odo.diagnostics()
        for k in ("updates", "last_dt", "imu_alpha", "imu_seen",
                  "last_dtheta_imu", "last_dtheta_enc",
                  "counts_per_m", "lxly_sum"):
            self.assertIn(k, d)

    def test_get_pose_returns_copy(self):
        odo = Odometry()
        odo.update({"FR": 0, "FL": 0, "BR": 0, "BL": 0}, now=1000.0)
        p1 = odo.get_pose()
        p1.x = 99.9   # mutating the copy must not corrupt internal state
        p2 = odo.get_pose()
        self.assertNotEqual(p2.x, 99.9)


class PoseSerialization(unittest.TestCase):
    def test_to_dict_includes_required_keys(self):
        p = Pose(x=1.0, y=-2.0, theta=math.pi / 4)
        d = p.to_dict()
        for k in ("x", "y", "theta", "theta_deg", "vx", "vy", "omega", "ts", "source"):
            self.assertIn(k, d)
        self.assertAlmostEqual(d["theta_deg"], 45.0, places=2)


if __name__ == "__main__":
    unittest.main()
