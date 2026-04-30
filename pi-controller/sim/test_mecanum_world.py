"""Smoke tests for the kinematic Mecanum simulator.

Run:  python -m pytest robots/pi-controller/sim/test_mecanum_world.py
Or:   python -m unittest robots.pi-controller.sim.test_mecanum_world
"""

from __future__ import annotations

import math
import unittest

from .mecanum_world import (
    RobotSim,
    World,
    WorldConfig,
    TOF_BEARINGS_RAD,
    make_demo_world,
)
from navigation.odometry import (
    COUNTS_PER_REV,
    ROBOT_LX,
    ROBOT_LY,
    WHEEL_RADIUS_M,
)


_LXLY = ROBOT_LX + ROBOT_LY


class WorldGeometry(unittest.TestCase):
    def test_world_to_cell_origin_centered(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05))
        ci, cj = w.world_to_cell(0.0, 0.0)
        # Origin sits in the middle cell of the grid.
        self.assertEqual((ci, cj), (w._cx, w._cy))

    def test_in_bounds_outside_grid(self):
        w = World(WorldConfig(size_m=4.0, cell_m=0.1))
        self.assertFalse(w.in_bounds(-1, 0))
        self.assertFalse(w.in_bounds(0, 9999))
        self.assertTrue(w.in_bounds(w._cx, w._cy))

    def test_outside_world_counts_as_occupied(self):
        w = World(WorldConfig(size_m=4.0, cell_m=0.1))
        self.assertTrue(w.is_occupied(100.0, 0.0))

    def test_add_box_marks_cells_occupied(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05))
        self.assertFalse(w.is_occupied(1.0, 1.0))
        w.add_box(0.5, 0.5, 1.5, 1.5)
        self.assertTrue(w.is_occupied(1.0, 1.0))
        # Edges only — centre still occupied, far point still free.
        self.assertFalse(w.is_occupied(2.5, 2.5))

    def test_add_wall_creates_thin_barrier(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05))
        w.add_wall(-2.0, 0.0, 2.0, 0.0, thickness_m=0.05)
        self.assertTrue(w.is_occupied(0.0, 0.0))
        self.assertTrue(w.is_occupied(1.0, 0.0))
        # 50 cm above the wall is still free.
        self.assertFalse(w.is_occupied(0.0, 0.5))

    def test_is_circle_blocked(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05))
        w.add_box(1.0, 1.0, 2.0, 2.0)
        # Disc of 0.3 m at (0.8, 0.8) overlaps the corner of the box.
        self.assertTrue(w.is_circle_blocked(0.8, 0.8, 0.3))
        # Far away → free.
        self.assertFalse(w.is_circle_blocked(-2.0, -2.0, 0.3))


class Raycast(unittest.TestCase):
    def test_raycast_empty_returns_max_range(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05, tof_max_m=2.0))
        # No obstacles → should hit max range.
        d = w.raycast(0.0, 0.0, 0.0)
        self.assertAlmostEqual(d, 2.0, delta=0.05)

    def test_raycast_hits_wall(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05, tof_max_m=2.0))
        # Wall 1 m ahead along +x.
        w.add_wall(1.0, -2.0, 1.0, 2.0, thickness_m=0.05)
        d = w.raycast(0.0, 0.0, 0.0)
        self.assertAlmostEqual(d, 1.0, delta=0.10)

    def test_raycast_respects_bearing(self):
        # Wall to the left (positive y), ray pointing left should hit; ray
        # pointing right should miss.
        w = World(WorldConfig(size_m=10.0, cell_m=0.05, tof_max_m=2.0))
        w.add_wall(-2.0, 1.0, 2.0, 1.0, thickness_m=0.05)
        d_left = w.raycast(0.0, 0.0, math.pi / 2)
        d_right = w.raycast(0.0, 0.0, -math.pi / 2)
        self.assertAlmostEqual(d_left, 1.0, delta=0.10)
        self.assertAlmostEqual(d_right, 2.0, delta=0.05)


class RobotMotion(unittest.TestCase):
    def test_forward_command_drives_x_forward(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05))
        r = RobotSim(w, x=0.0, y=0.0, theta_rad=0.0, ramp_tau_s=0.05)
        for _ in range(40):
            r.step(vx_cmd=0.3, vy_cmd=0.0, wz_cmd=0.0, dt=0.05)
        x, y, theta = r.pose()
        self.assertGreater(x, 0.4)
        self.assertAlmostEqual(y, 0.0, delta=0.02)
        self.assertAlmostEqual(theta, 0.0, delta=0.02)

    def test_strafe_left_drives_y_positive(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05))
        r = RobotSim(w, x=0.0, y=0.0, theta_rad=0.0, ramp_tau_s=0.05)
        for _ in range(40):
            r.step(vx_cmd=0.0, vy_cmd=0.2, wz_cmd=0.0, dt=0.05)
        x, y, _ = r.pose()
        self.assertAlmostEqual(x, 0.0, delta=0.02)
        self.assertGreater(y, 0.3)

    def test_pure_rotation_drives_theta_only(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05))
        r = RobotSim(w, ramp_tau_s=0.05)
        for _ in range(20):
            r.step(0.0, 0.0, 0.5, dt=0.05)
        x, y, theta = r.pose()
        self.assertAlmostEqual(x, 0.0, delta=0.02)
        self.assertAlmostEqual(y, 0.0, delta=0.02)
        self.assertAlmostEqual(theta, 0.5, delta=0.05)

    def test_collision_stops_translation_not_rotation(self):
        w = World(WorldConfig(size_m=6.0, cell_m=0.05))
        # Wall right in front of the spawn.
        w.add_wall(0.5, -2.0, 0.5, 2.0, thickness_m=0.05)
        r = RobotSim(w, x=0.0, y=0.0, theta_rad=0.0, ramp_tau_s=0.05)
        # Drive forward into wall + rotate.
        for _ in range(60):
            r.step(0.5, 0.0, 0.3, dt=0.05)
        x, y, theta = r.pose()
        # Translation must have been clipped by the collision.
        self.assertLess(x, 0.4)
        # Rotation should still have happened — collision doesn't kill wz.
        self.assertGreater(theta, 0.5)
        self.assertGreater(r.collisions, 0)


class EncoderConsistency(unittest.TestCase):
    """Counts emitted by RobotSim should round-trip through Mecanum forward
    kinematics back to the commanded body twist."""

    def test_forward_drive_yields_equal_positive_counts(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05))
        r = RobotSim(w, ramp_tau_s=0.05)
        # Long enough to ramp past the lag.
        for _ in range(40):
            r.step(0.3, 0.0, 0.0, dt=0.05)
        counts = r.encoder_counts
        # All four wheels should have advanced positively, and pairs should
        # match closely (forward drive = symmetric).
        for k, v in counts.items():
            self.assertGreater(v, 100, msg=f"{k} did not advance")
        diff_fl_fr = abs(counts["FL"] - counts["FR"])
        diff_bl_br = abs(counts["BL"] - counts["BR"])
        self.assertLess(diff_fl_fr, 50)
        self.assertLess(diff_bl_br, 50)

    def test_strafe_yields_anti_diagonal_counts(self):
        # vy>0 (left): FL & BR negative; FR & BL positive.
        w = World(WorldConfig(size_m=10.0, cell_m=0.05))
        r = RobotSim(w, ramp_tau_s=0.05)
        for _ in range(40):
            r.step(0.0, 0.2, 0.0, dt=0.05)
        c = r.encoder_counts
        self.assertLess(c["FL"], 0)
        self.assertLess(c["BR"], 0)
        self.assertGreater(c["FR"], 0)
        self.assertGreater(c["BL"], 0)


class Sensors(unittest.TestCase):
    def test_tofs_mm_returns_six_keys(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05))
        r = RobotSim(w)
        d = r.tofs_mm()
        for name in TOF_BEARINGS_RAD:
            self.assertIn(name, d)
            self.assertIsInstance(d[name], int)
            # No obstacles around → all readings should hit the max range cap.
            self.assertGreater(d[name], 1500)

    def test_front_tof_sees_wall_ahead(self):
        w = World(WorldConfig(size_m=10.0, cell_m=0.05, tof_max_m=2.0))
        w.add_wall(1.0, -2.0, 1.0, 2.0, thickness_m=0.05)
        r = RobotSim(w, x=0.0, y=0.0, theta_rad=0.0)
        d = r.tofs_mm()
        # Front beam should see the wall around 1 m.
        self.assertLess(d["front"], 1100)
        self.assertGreater(d["front"], 900)
        # Rear beam pointing the opposite direction should see open space.
        self.assertGreater(d["rear"], 1500)

    def test_imu_yaw_deg_matches_state(self):
        w = World()
        r = RobotSim(w, theta_rad=math.radians(45.0))
        self.assertAlmostEqual(r.imu_yaw_deg(), 45.0, delta=0.1)


class DemoWorld(unittest.TestCase):
    def test_make_demo_world_has_walls_and_obstacles(self):
        w = make_demo_world()
        # Centre should be free (robot spawn).
        self.assertFalse(w.is_occupied(0.0, 0.0))
        # The two interior obstacles should register.
        self.assertTrue(w.is_occupied(1.0, -0.6))
        self.assertTrue(w.is_occupied(-1.0, 1.2))
        # The room walls should be present.
        self.assertTrue(w.is_occupied(-3.0, 0.0))
        self.assertTrue(w.is_occupied(3.0, 0.0))


if __name__ == "__main__":
    unittest.main()
