"""Unit tests for reactive_nav priority safety stack.

Run:  python -m pytest robots/pi-controller/test_reactive_nav.py

reactive_nav sits between any nav mode and the ESP32 UART, so it is the
last line of safety before motor commands leave the Pi. These tests pin
down the priority hierarchy (EMERGENCY > BRAKE > STEER > SIDE > CLEAR),
the back-pulse state machine, the back-safety reverse suppression, and
the clamping at the boundary.

The module reads `time.monotonic()` for the back-pulse timer, so we
monkey-patch it with a controllable clock — same pattern used in the
existing navigation/test_*.py files.
"""

from __future__ import annotations

import unittest
from unittest import mock

from reactive_nav import (
    Behavior,
    ReactiveConfig,
    ReactiveNav,
    ReactiveResult,
    ToFSnapshot,
)
import reactive_nav as rn_mod


class _Clock:
    """Hand-cranked monotonic clock for back-pulse timer tests."""

    def __init__(self, t0: float = 1000.0) -> None:
        self.t = float(t0)

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += float(dt)


def _far() -> ToFSnapshot:
    """Snapshot with all sensors out of any threshold range."""
    return ToFSnapshot(
        front=2000, front_left=2000, front_right=2000,
        left=2000, right=2000, back=2000,
    )


# ── ToFSnapshot helpers ────────────────────────────────────────────────────────

class ToFSnapshotTests(unittest.TestCase):
    def test_min_distance_picks_smallest(self):
        snap = ToFSnapshot(front=500, front_left=300, front_right=900,
                           left=80, right=600, back=1200)
        self.assertEqual(snap.min_distance(), 80)

    def test_from_sensor_reads_attrs(self):
        class Fake:
            front = 111
            front_left = 222
            front_right = 333
            left = 444
            right = 555
            back = 666

        snap = ToFSnapshot.from_sensor(Fake())
        self.assertEqual(snap.front, 111)
        self.assertEqual(snap.front_left, 222)
        self.assertEqual(snap.back, 666)

    def test_from_sensor_defaults_for_missing(self):
        class Empty:
            pass

        snap = ToFSnapshot.from_sensor(Empty())
        self.assertEqual(snap.front, 9999)
        self.assertEqual(snap.min_distance(), 9999)


# ── CLEAR pass-through ─────────────────────────────────────────────────────────

class ClearPassThrough(unittest.TestCase):
    def test_clear_returns_input_unchanged(self):
        rn = ReactiveNav()
        out = rn.process(_far(), vx=80, vy=10, wz=5, spd=60)
        self.assertEqual(out.behavior, Behavior.CLEAR)
        self.assertEqual(out.vx, 80)
        self.assertEqual(out.vy, 10)
        self.assertEqual(out.wz, 5)
        self.assertEqual(out.spd, 60)


# ── EMERGENCY (highest priority) ───────────────────────────────────────────────

class EmergencyBehavior(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = _Clock()
        self._patcher = mock.patch.object(rn_mod.time, "monotonic", self.clock)
        self._patcher.start()
        self.addCleanup(self._patcher.stop)

    def test_emergency_triggers_when_front_below_threshold(self):
        rn = ReactiveNav(ReactiveConfig(emergency_mm=150))
        snap = _far()
        snap.front = 100
        out = rn.process(snap, vx=80, vy=0, wz=0, spd=60)
        self.assertEqual(out.behavior, Behavior.EMERGENCY)
        # Reverse pulse: vx negative because back is clear
        self.assertLess(out.vx, 0)
        self.assertEqual(out.vy, 0)
        self.assertEqual(out.wz, 0)

    def test_emergency_back_pulse_suppressed_when_back_also_blocked(self):
        rn = ReactiveNav(ReactiveConfig(emergency_mm=150, back_mm=120))
        snap = _far()
        snap.front = 100
        snap.back = 80   # back is blocked too
        out = rn.process(snap, vx=80, vy=0, wz=0, spd=60)
        self.assertEqual(out.behavior, Behavior.EMERGENCY)
        # Should NOT reverse into the rear obstacle
        self.assertEqual(out.vx, 0)

    def test_emergency_clears_after_back_pulse_duration(self):
        cfg = ReactiveConfig(emergency_mm=150, back_pulse_ms=300)
        rn = ReactiveNav(cfg)

        # Trigger emergency
        snap = _far()
        snap.front = 100
        out1 = rn.process(snap, vx=80, vy=0, wz=0, spd=60)
        self.assertEqual(out1.behavior, Behavior.EMERGENCY)

        # Advance clock past the pulse window AND clear front obstacle
        self.clock.advance(0.5)  # 500 ms > 300 ms pulse
        out2 = rn.process(_far(), vx=80, vy=0, wz=0, spd=60)
        self.assertEqual(out2.behavior, Behavior.CLEAR)
        self.assertEqual(out2.vx, 80)

    def test_reset_clears_emergency_state(self):
        rn = ReactiveNav(ReactiveConfig(emergency_mm=150))
        snap = _far()
        snap.front = 100
        rn.process(snap, vx=80, vy=0, wz=0, spd=60)
        self.assertTrue(rn._in_emergency)
        rn.reset()
        self.assertFalse(rn._in_emergency)
        # Now CLEAR with no obstacle should pass through
        out = rn.process(_far(), vx=80, vy=0, wz=0, spd=60)
        self.assertEqual(out.behavior, Behavior.CLEAR)


# ── BRAKE progressive scaling ──────────────────────────────────────────────────

class BrakeBehavior(unittest.TestCase):
    def test_brake_at_threshold_keeps_full_speed(self):
        rn = ReactiveNav(ReactiveConfig(brake_mm=450, brake_min_mm=150,
                                        min_speed_fac=0.20))
        snap = _far()
        snap.front = 450  # exactly at threshold (boundary)
        out = rn.process(snap, vx=100, vy=0, wz=0, spd=100)
        # At threshold the inequality `front < brake_mm` is false
        self.assertEqual(out.behavior, Behavior.CLEAR)
        self.assertEqual(out.vx, 100)
        self.assertEqual(out.spd, 100)

    def test_brake_inside_band_scales_speed_down(self):
        rn = ReactiveNav(ReactiveConfig(brake_mm=450, brake_min_mm=150,
                                        min_speed_fac=0.20))
        snap = _far()
        snap.front = 300  # mid-band
        out = rn.process(snap, vx=100, vy=0, wz=0, spd=100)
        self.assertEqual(out.behavior, Behavior.BRAKE)
        # Speed should be reduced from 100
        self.assertLess(out.spd, 100)
        self.assertLess(out.vx, 100)
        self.assertGreater(out.vx, 0)

    def test_brake_at_min_distance_hits_floor(self):
        # Lower emergency_mm so the brake floor stays out of the EMERGENCY zone.
        rn = ReactiveNav(ReactiveConfig(emergency_mm=100,
                                        brake_mm=450, brake_min_mm=150,
                                        min_speed_fac=0.20))
        snap = _far()
        snap.front = 150  # at the brake floor, above emergency
        out = rn.process(snap, vx=100, vy=0, wz=0, spd=100)
        self.assertEqual(out.behavior, Behavior.BRAKE)
        self.assertEqual(out.spd, 20)  # 100 * 0.20

    def test_brake_does_not_fire_when_not_moving_forward(self):
        rn = ReactiveNav()
        snap = _far()
        snap.front = 300  # in brake band
        # vx=0 → brake should not engage
        out = rn.process(snap, vx=0, vy=0, wz=0, spd=60)
        self.assertEqual(out.behavior, Behavior.CLEAR)


# ── STEER (diagonal avoidance) ─────────────────────────────────────────────────

class SteerBehavior(unittest.TestCase):
    def test_steer_right_when_front_left_blocked(self):
        rn = ReactiveNav(ReactiveConfig(steer_mm=300, steer_vy=40, steer_wz=25))
        snap = _far()
        snap.front_left = 150  # blocked → veer right (vy+, wz-)
        out = rn.process(snap, vx=80, vy=0, wz=0, spd=60)
        self.assertEqual(out.behavior, Behavior.STEER_R)
        self.assertGreater(out.vy, 0)
        self.assertLess(out.wz, 0)

    def test_steer_left_when_front_right_blocked(self):
        rn = ReactiveNav(ReactiveConfig(steer_mm=300, steer_vy=40, steer_wz=25))
        snap = _far()
        snap.front_right = 150  # blocked → veer left (vy-, wz+)
        out = rn.process(snap, vx=80, vy=0, wz=0, spd=60)
        self.assertEqual(out.behavior, Behavior.STEER_L)
        self.assertLess(out.vy, 0)
        self.assertGreater(out.wz, 0)

    def test_both_diagonals_blocked_picks_wider_side(self):
        """When both front diagonals trigger, steer toward the more open one."""
        rn = ReactiveNav(ReactiveConfig(steer_mm=300))
        snap = _far()
        snap.front_left = 100   # tighter on the left
        snap.front_right = 250  # wider on the right
        out = rn.process(snap, vx=80, vy=0, wz=0, spd=60)
        # Closer obstacle on left → veer right (away from tight side)
        self.assertEqual(out.behavior, Behavior.STEER_R)


# ── SIDE (parallel strafe) ─────────────────────────────────────────────────────

class SideBehavior(unittest.TestCase):
    def test_side_left_blocks_triggers_strafe_right(self):
        rn = ReactiveNav(ReactiveConfig(side_mm=180, side_vy=35))
        snap = _far()
        snap.left = 100  # left wall close → strafe right (vy+)
        out = rn.process(snap, vx=80, vy=0, wz=0, spd=60)
        self.assertEqual(out.behavior, Behavior.SIDE_R)
        self.assertGreater(out.vy, 0)

    def test_side_right_blocks_triggers_strafe_left(self):
        rn = ReactiveNav(ReactiveConfig(side_mm=180, side_vy=35))
        snap = _far()
        snap.right = 100
        out = rn.process(snap, vx=80, vy=0, wz=0, spd=60)
        self.assertEqual(out.behavior, Behavior.SIDE_L)
        self.assertLess(out.vy, 0)

    def test_steer_dominates_side_in_behavior_label(self):
        """Side avoidance modifies vy but doesn't override STEER label."""
        rn = ReactiveNav()
        snap = _far()
        snap.front_left = 150  # STEER_R triggers
        snap.left = 100        # SIDE also wants to push right
        out = rn.process(snap, vx=80, vy=0, wz=0, spd=60)
        self.assertEqual(out.behavior, Behavior.STEER_R)
        # Both contributions push vy positive; magnitude should reflect both
        self.assertGreater(out.vy, 0)


# ── Back safety ────────────────────────────────────────────────────────────────

class BackSafety(unittest.TestCase):
    def test_reverse_suppressed_when_back_blocked(self):
        rn = ReactiveNav(ReactiveConfig(back_mm=120))
        snap = _far()
        snap.back = 80
        out = rn.process(snap, vx=-50, vy=0, wz=0, spd=60)
        self.assertEqual(out.vx, 0)

    def test_reverse_allowed_when_back_clear(self):
        rn = ReactiveNav(ReactiveConfig(back_mm=120))
        snap = _far()
        snap.back = 500
        out = rn.process(snap, vx=-50, vy=0, wz=0, spd=60)
        self.assertEqual(out.vx, -50)


# ── Output clamping ────────────────────────────────────────────────────────────

class OutputClamping(unittest.TestCase):
    def test_clamps_overlarge_inputs(self):
        rn = ReactiveNav()
        out = rn.process(_far(), vx=500, vy=-500, wz=900, spd=999)
        self.assertEqual(out.vx, 127)
        self.assertEqual(out.vy, -127)
        self.assertEqual(out.wz, 127)
        self.assertEqual(out.spd, 255)

    def test_clamps_negative_spd_to_zero(self):
        rn = ReactiveNav()
        out = rn.process(_far(), vx=0, vy=0, wz=0, spd=-100)
        self.assertEqual(out.spd, 0)

    def test_result_is_namedtuple_with_expected_fields(self):
        rn = ReactiveNav()
        out = rn.process(_far(), vx=0, vy=0, wz=0, spd=60)
        self.assertIsInstance(out, ReactiveResult)
        self.assertTrue(hasattr(out, "tof_min"))
        self.assertEqual(out.tof_min, 2000)


if __name__ == "__main__":
    unittest.main()
