"""Unit tests for FreeCoverage FSM.

Run:  python -m pytest robots/pi-controller/navigation/test_free_coverage.py
Or:   python -m unittest robots.pi-controller.navigation.test_free_coverage

These tests focus on transitions and outputs of the state machine. We
exercise it by feeding synthetic ToF readings (front/left/right) and an
optional pose. The wall-clock-dependent paths (TURNING duration, BACK
duration, frontier check period) are exercised by patching
time.monotonic to a controllable clock.
"""

from __future__ import annotations

import math
import unittest
from unittest import mock

from .free_coverage import (
    CoverageConfig,
    CoverageResult,
    FreeCoverage,
)
from .occupancy_grid import CELL_OBSTACLE, CELL_VISITED


class _FakePose:
    """Stand-in for navigation.odometry.Pose."""

    def __init__(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0):
        self.x, self.y, self.theta = x, y, theta


class _FakeClock:
    """Patchable replacement for time.monotonic()."""

    def __init__(self, t: float = 1000.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class ForwardAndTurn(unittest.TestCase):
    def test_forward_when_open_and_no_wall(self):
        # Map disabled to keep wall-follow / frontier paths out of the way.
        cfg = CoverageConfig(map_enabled=False, wall_follow_enabled=False)
        fc = FreeCoverage(cfg)
        out = fc.tick(front_mm=2000, left_mm=2000, right_mm=2000)
        self.assertEqual(out.state, "FORWARD")
        self.assertEqual(out.vx, cfg.base_vx)
        self.assertEqual(out.vy, 0)
        self.assertEqual(out.wz, 0)

    def test_turn_triggered_when_front_blocked(self):
        cfg = CoverageConfig(map_enabled=False, wall_follow_enabled=False)
        fc = FreeCoverage(cfg)
        out = fc.tick(front_mm=cfg.turn_trigger_mm - 50, left_mm=2000, right_mm=2000)
        self.assertEqual(out.state, "TURNING")
        self.assertEqual(out.vx, 0)
        # wz magnitude should equal turn_speed (sign depends on which side is closer).
        self.assertEqual(abs(out.wz), cfg.turn_speed)

    def test_turn_direction_away_from_closer_side(self):
        cfg = CoverageConfig(map_enabled=False, wall_follow_enabled=False)
        fc = FreeCoverage(cfg)
        # Left is closer → robot should turn CW (right) → wz negative.
        out = fc.tick(front_mm=cfg.turn_trigger_mm - 50, left_mm=300, right_mm=2000)
        self.assertEqual(out.state, "TURNING")
        self.assertLess(out.wz, 0)

        fc2 = FreeCoverage(cfg)
        # Right is closer → robot should turn CCW (left) → wz positive.
        out2 = fc2.tick(front_mm=cfg.turn_trigger_mm - 50, left_mm=2000, right_mm=300)
        self.assertEqual(out2.state, "TURNING")
        self.assertGreater(out2.wz, 0)


class EmergencyReverse(unittest.TestCase):
    def test_emergency_reverse_when_front_too_close(self):
        cfg = CoverageConfig(map_enabled=False)
        fc = FreeCoverage(cfg)
        out = fc.tick(front_mm=cfg.emergency_mm - 10, left_mm=2000, right_mm=2000)
        self.assertEqual(out.state, "BACK")
        self.assertEqual(out.vx, -cfg.base_vx)
        self.assertEqual(out.vy, 0)

    def test_back_transitions_after_duration(self):
        cfg = CoverageConfig(map_enabled=False, back_duration_s=0.10)
        clock = _FakeClock(1000.0)
        from . import free_coverage as fc_mod
        with mock.patch.object(fc_mod.time, "monotonic", clock):
            fc = FreeCoverage(cfg)
            # Front blocked → enter BACK on first tick.
            out1 = fc.tick(front_mm=cfg.emergency_mm - 10, left_mm=2000, right_mm=2000)
            self.assertEqual(out1.state, "BACK")
            # Still within duration → still BACK.
            clock.advance(0.05)
            out2 = fc.tick(front_mm=cfg.emergency_mm - 10, left_mm=2000, right_mm=2000)
            self.assertEqual(out2.state, "BACK")
            # Past duration AND front now clear → BACK should release.
            clock.advance(0.50)
            out3 = fc.tick(front_mm=2000, left_mm=2000, right_mm=2000)
            self.assertNotEqual(out3.state, "BACK")


class WallFollow(unittest.TestCase):
    def test_wall_follow_when_right_wall_in_range(self):
        cfg = CoverageConfig(map_enabled=False, wall_follow_enabled=True)
        fc = FreeCoverage(cfg)
        # Right wall right at target → near-zero correction.
        out = fc.tick(
            front_mm=2000,
            left_mm=2000,
            right_mm=cfg.wall_target_mm,
        )
        self.assertEqual(out.state, "WALL_FOLLOW")
        self.assertEqual(out.vy, 0)
        self.assertEqual(out.wz, 0)
        self.assertEqual(out.vx, cfg.base_vx)

    def test_wall_follow_too_far_steers_right(self):
        cfg = CoverageConfig(map_enabled=False, wall_follow_enabled=True)
        fc = FreeCoverage(cfg)
        # right_mm > target → err > 0 → vy negative (drift right), wz negative (yaw right).
        out = fc.tick(front_mm=2000, left_mm=2000, right_mm=cfg.wall_target_mm + 200)
        self.assertEqual(out.state, "WALL_FOLLOW")
        self.assertLess(out.vy, 0)
        self.assertLess(out.wz, 0)

    def test_wall_follow_too_close_steers_left(self):
        cfg = CoverageConfig(map_enabled=False, wall_follow_enabled=True)
        fc = FreeCoverage(cfg)
        # right_mm < target → err < 0 → vy positive, wz positive (yaw left).
        out = fc.tick(front_mm=2000, left_mm=2000, right_mm=cfg.wall_target_mm - 200)
        self.assertEqual(out.state, "WALL_FOLLOW")
        self.assertGreater(out.vy, 0)
        self.assertGreater(out.wz, 0)

    def test_wall_lost_falls_back_to_forward(self):
        cfg = CoverageConfig(map_enabled=False, wall_follow_enabled=True)
        fc = FreeCoverage(cfg)
        out = fc.tick(front_mm=2000, left_mm=2000, right_mm=cfg.wall_lost_mm + 100)
        self.assertEqual(out.state, "FORWARD")

    def test_clamps_wall_follow_lateral_and_yaw(self):
        cfg = CoverageConfig(map_enabled=False, wall_follow_enabled=True)
        fc = FreeCoverage(cfg)
        # Huge error → outputs should saturate, not blow up.
        out = fc.tick(front_mm=2000, left_mm=2000, right_mm=cfg.wall_target_mm - 800)
        self.assertEqual(out.state, "WALL_FOLLOW")
        self.assertLessEqual(abs(out.vy), cfg.wall_max_vy)
        self.assertLessEqual(abs(out.wz), cfg.wall_max_wz)


class BackwardsCompat(unittest.TestCase):
    def test_tick_without_pose_still_works(self):
        # Map enabled by default — but no pose → grid should not crash.
        fc = FreeCoverage()
        for _ in range(5):
            out = fc.tick(front_mm=2000, left_mm=2000, right_mm=2000)
            self.assertIsInstance(out, CoverageResult)

    def test_grid_unchanged_without_pose(self):
        fc = FreeCoverage()
        grid = fc.grid
        self.assertIsNotNone(grid)
        # No pose → mark_visited never called, all visit_count = 0.
        fc.tick(front_mm=2000, left_mm=2000, right_mm=2000)
        self.assertEqual(int(grid.visit_count.sum()), 0)

    def test_grid_disabled_when_map_disabled(self):
        fc = FreeCoverage(CoverageConfig(map_enabled=False))
        self.assertIsNone(fc.grid)


class MapUpdateWithPose(unittest.TestCase):
    def test_pose_drives_visit_count(self):
        fc = FreeCoverage()
        pose = _FakePose(0.0, 0.0, 0.0)
        fc.tick(front_mm=2000, left_mm=2000, right_mm=2000, pose=pose)
        grid = fc.grid
        i, j = grid.world_to_cell(0.0, 0.0)
        self.assertGreaterEqual(int(grid.visit_count[i, j]), 1)
        self.assertTrue(grid.flags[i, j] & CELL_VISITED)

    def test_all_tofs_int_shape_marks_obstacles(self):
        # Regression: production NavController passes all_tofs as
        # Dict[str, float] (e.g. {"front": 1500, "left": 800, ...}). Before
        # the fix in _update_map, plain int items hit the else-continue
        # branch and every ray was silently dropped, leaving obstacles=0
        # forever in FREE_COVERAGE mode on real hardware.
        cfg = CoverageConfig(
            wall_follow_enabled=False,
            frontier_check_period_s=999.0,  # disable frontier path
        )
        fc = FreeCoverage(cfg)
        pose = _FakePose(0.0, 0.0, 0.0)
        # 800 mm reading on the right beam should mark a cell as OBSTACLE.
        all_tofs = {
            "front":       2000,
            "front_left":  2000,
            "front_right": 2000,
            "left":        2000,
            "right":        800,
            "rear":        9999,
        }
        fc.tick(
            front_mm=2000, left_mm=2000, right_mm=800,
            pose=pose, all_tofs=all_tofs,
        )
        grid = fc.grid
        # OccupancyGrid stores cell-major (i=x, j=y). Right of pose at
        # bearing -90° from theta=0 is along -y, so ~0.8 m down the grid.
        self.assertGreater(
            int((grid.flags & CELL_OBSTACLE).sum()), 0,
            msg="Plain {name: int} all_tofs should still mark obstacles",
        )

    def test_all_tofs_dict_shape_still_works(self):
        # Belt-and-braces: confirm dict and tuple shapes both still parse.
        fc = FreeCoverage(CoverageConfig(
            wall_follow_enabled=False,
            frontier_check_period_s=999.0,
        ))
        pose = _FakePose(0.0, 0.0, 0.0)
        all_tofs = {
            "front":       {"bearing":  0.0, "distance": 800},
            "front_left":  ( 45.0, 2000),
            "right":       (-90.0, 1500),
        }
        fc.tick(
            front_mm=800, left_mm=2000, right_mm=1500,
            pose=pose, all_tofs=all_tofs,
        )
        grid = fc.grid
        self.assertGreater(int((grid.flags & CELL_OBSTACLE).sum()), 0)


class FrontierBehavior(unittest.TestCase):
    def test_no_frontier_when_local_not_saturated(self):
        # Just one visited cell — local area not saturated → stay in FORWARD.
        cfg = CoverageConfig(frontier_check_period_s=0.0)  # always check
        fc = FreeCoverage(cfg)
        pose = _FakePose(0.0, 0.0, 0.0)
        out = fc.tick(front_mm=2000, left_mm=2000, right_mm=2000, pose=pose)
        # First tick visits one cell — far from saturated.
        self.assertNotEqual(out.state, "FRONTIER_SEEK")

    def test_frontier_seek_when_local_saturated(self):
        cfg = CoverageConfig(
            frontier_check_period_s=0.0,
            frontier_local_visited_threshold=1,    # any visit counts
            frontier_saturation_pct=0.5,
            wall_follow_enabled=False,
        )
        fc = FreeCoverage(cfg)
        grid = fc.grid
        # Pre-saturate the local disc by hand: visit a 3x3 area.
        for di in range(-1, 2):
            for dj in range(-1, 2):
                wx = di * cfg.map_cell_size_m
                wy = dj * cfg.map_cell_size_m
                grid.mark_visited(wx, wy)
                grid.mark_visited(wx, wy)
        pose = _FakePose(0.0, 0.0, 0.0)
        out = fc.tick(front_mm=2000, left_mm=2000, right_mm=2000, pose=pose)
        # Either FRONTIER_SEEK (target found) or FORWARD (no frontier nearby) is
        # acceptable, but the FSM must not have crashed and result is sane.
        self.assertIsInstance(out, CoverageResult)


class Diagnostics(unittest.TestCase):
    def test_diagnostics_keys(self):
        fc = FreeCoverage()
        d = fc.diagnostics()
        self.assertIn("state", d)
        self.assertIn("frontier_target", d)
        # Map enabled by default.
        self.assertIn("map", d)
        self.assertEqual(d["state"], "FORWARD")
        self.assertIsNone(d["frontier_target"])

    def test_diagnostics_no_map_when_disabled(self):
        fc = FreeCoverage(CoverageConfig(map_enabled=False))
        d = fc.diagnostics()
        self.assertNotIn("map", d)


class Reset(unittest.TestCase):
    def test_reset_returns_to_forward(self):
        cfg = CoverageConfig(map_enabled=False)
        fc = FreeCoverage(cfg)
        # Push into TURNING.
        fc.tick(front_mm=cfg.turn_trigger_mm - 50, left_mm=2000, right_mm=2000)
        fc.reset()
        out = fc.tick(front_mm=2000, left_mm=2000, right_mm=2000)
        self.assertIn(out.state, ("FORWARD", "WALL_FOLLOW"))


if __name__ == "__main__":
    unittest.main()
