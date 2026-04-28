"""Unit tests for TemporalSmoother.

Run:  python -m pytest robots/pi-controller/detection/test_temporal_smoother.py
Or:   python -m unittest robots.pi-controller.detection.test_temporal_smoother
"""

from __future__ import annotations

import unittest

from .temporal_smoother import TemporalSmoother


class SmootherCore(unittest.TestCase):
    def test_below_threshold_does_not_fire(self):
        s = TemporalSmoother(window=5, required=3)
        # 2 hits in 5 is not enough.
        for c in [0.8, 0.0, 0.7, 0.0, 0.0]:
            out = s.update({"person": c})
        self.assertEqual(out, {})

    def test_fires_on_consensus(self):
        s = TemporalSmoother(window=5, required=3)
        fires = []
        for c in [0.9, 0.8, 0.7]:
            fires.append(s.update({"person": c}))
        self.assertEqual(fires[0], {})
        self.assertEqual(fires[1], {})
        self.assertIn("person", fires[2])
        # Confidence is the mean of positive hits so far.
        self.assertAlmostEqual(fires[2]["person"], (0.9 + 0.8 + 0.7) / 3, places=5)

    def test_does_not_refire_while_latched(self):
        s = TemporalSmoother(window=5, required=3)
        for c in [0.9, 0.8, 0.7]:
            s.update({"person": c})
        # Still hitting — must not re-fire.
        out = s.update({"person": 0.85})
        self.assertEqual(out, {})

    def test_drops_then_refires(self):
        s = TemporalSmoother(window=4, required=3)
        for c in [0.9, 0.8, 0.7]:  # fires
            s.update({"person": c})
        # Enough empty frames to drop count below required.
        for _ in range(4):
            s.update({})
        self.assertFalse(s.state()["person"][2])  # unlatched
        # New consensus fires again.
        for c in [0.8, 0.7, 0.75]:
            out = s.update({"person": c})
        self.assertIn("person", out)

    def test_independent_kinds(self):
        s = TemporalSmoother(window=3, required=2, class_kinds=("person", "fire"))
        # person fires at frame 2, fire never fires.
        s.update({"person": 0.9})
        out2 = s.update({"person": 0.8, "fire": 0.02})
        self.assertIn("person", out2)
        self.assertNotIn("fire", out2)

    def test_reset(self):
        s = TemporalSmoother(window=3, required=2)
        s.update({"person": 0.9})
        s.update({"person": 0.8})
        s.reset("person")
        state = s.state()["person"]
        self.assertEqual(state, (0, 0, False))

    def test_validation(self):
        with self.assertRaises(ValueError):
            TemporalSmoother(window=2, required=3)
        with self.assertRaises(ValueError):
            TemporalSmoother(window=0, required=0)


if __name__ == "__main__":
    unittest.main()
