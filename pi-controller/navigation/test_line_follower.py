"""Unit tests for LineFollower ToF emergency cut-off path.

These do not exercise the full vision pipeline (BEV warp + spine fit) — that
is covered by the offline synthetic harness in tools/test_line_follower_offline.py.
The focus here is the safety short-circuit added in V5.4.

Run:  python -m pytest robots/pi-controller/navigation/test_line_follower.py
"""

from __future__ import annotations

import unittest

import numpy as np

from .line_follower import LineFollower, LineFollowerConfig


def _blank_frame() -> np.ndarray:
    # 480p BGR all-black — the ToF gate must trigger before vision runs,
    # so the frame contents do not matter for these tests.
    return np.zeros((480, 640, 3), dtype=np.uint8)


class ToFEmergencyStop(unittest.TestCase):
    def test_no_provider_no_blocking(self):
        lf = LineFollower(LineFollowerConfig())
        result = lf.tick(_blank_frame(), produce_overlay=False)
        self.assertFalse(result.tof_blocked)
        self.assertIsNone(result.front_dist_mm)

    def test_provider_below_threshold_forces_full_stop(self):
        lf = LineFollower(LineFollowerConfig(tof_emergency_stop_mm=300))
        lf.set_front_distance_provider(lambda: 200.0)
        result = lf.tick(_blank_frame(), produce_overlay=False)
        self.assertTrue(result.tof_blocked)
        self.assertEqual((result.vx, result.vy, result.wz), (0, 0, 0))
        self.assertEqual(result.front_dist_mm, 200.0)

    def test_provider_at_threshold_does_not_block(self):
        # Strict `<` comparison: exactly equal to threshold is allowed.
        lf = LineFollower(LineFollowerConfig(tof_emergency_stop_mm=300))
        lf.set_front_distance_provider(lambda: 300.0)
        result = lf.tick(_blank_frame(), produce_overlay=False)
        self.assertFalse(result.tof_blocked)

    def test_zero_or_negative_distance_treated_as_no_reading(self):
        # VL53L1X drivers commonly report 0 on out-of-range / read failure;
        # the controller must NOT interpret that as "obstacle right in front".
        lf = LineFollower(LineFollowerConfig(tof_emergency_stop_mm=300))
        lf.set_front_distance_provider(lambda: 0)
        result = lf.tick(_blank_frame(), produce_overlay=False)
        self.assertFalse(result.tof_blocked)
        self.assertIsNone(result.front_dist_mm)

    def test_provider_exception_is_swallowed(self):
        def boom():
            raise RuntimeError("sensor gone")
        lf = LineFollower(LineFollowerConfig())
        lf.set_front_distance_provider(boom)
        # Must not raise — sensor glitch must not abort the control loop.
        result = lf.tick(_blank_frame(), produce_overlay=False)
        self.assertFalse(result.tof_blocked)

    def test_pd_state_preserved_during_block(self):
        # When the ToF gate fires, the PD state should NOT advance — the
        # next non-blocked tick should resume from the same lateral/heading
        # error history rather than treat the pause as a giant dt step.
        lf = LineFollower(LineFollowerConfig(tof_emergency_stop_mm=300))
        lf._prev_lat_err = 12.5
        lf._prev_hdg_err = 0.07
        lf.set_front_distance_provider(lambda: 100.0)
        lf.tick(_blank_frame(), produce_overlay=False)
        self.assertAlmostEqual(lf._prev_lat_err, 12.5)
        self.assertAlmostEqual(lf._prev_hdg_err, 0.07)


class ProviderClearedRestoresUnblocked(unittest.TestCase):
    def test_setting_provider_to_none_disables_gate(self):
        lf = LineFollower(LineFollowerConfig(tof_emergency_stop_mm=300))
        lf.set_front_distance_provider(lambda: 50.0)
        self.assertTrue(lf.tick(_blank_frame(), produce_overlay=False).tof_blocked)
        lf.set_front_distance_provider(None)
        self.assertFalse(lf.tick(_blank_frame(), produce_overlay=False).tof_blocked)


if __name__ == "__main__":
    unittest.main()
