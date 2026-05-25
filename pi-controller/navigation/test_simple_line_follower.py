"""Unit tests for SimpleLineFollower steering modes.

Covers the V5.15c9 split where the simple follower can emit either
joystick verbs (legacy crab-walk) or Mecanum twist tuples (true
steering — the default).

We exercise the public surface only (`tick(frame, ...)`), not the
internal centroid math: the priority is locking in the *sign convention*
and *dead band* behaviour. A regression in either of those silently
sends the robot into a wall.

Run:
    python -m pytest robots/pi-controller/navigation/test_simple_line_follower.py -v
"""

from __future__ import annotations

import unittest

try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import cv2  # type: ignore
    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False

from .simple_line_follower import SimpleLineConfig, SimpleLineFollower


def _make_frame_with_line(h=480, w=960, line_cx=None, line_w=12, bg=(0, 0, 0)):
    """Synthesize a yellow-tape frame for the bottom 35% strip.

    line_cx=None → no line in frame (lost). Otherwise paint a vertical
    yellow stripe centred at line_cx in the BOTTOM strip only — that's
    exactly the slice the follower's ROI sees.
    """
    assert HAS_NUMPY and HAS_CV2, "test requires numpy + cv2"
    frame = np.zeros((h, w, 3), dtype=np.uint8)
    frame[:, :] = bg
    if line_cx is None:
        return frame
    # Yellow tape in BGR ≈ (0, 200, 220). Make it strong yellow so it
    # falls comfortably inside default HSV envelope.
    roi_top = int(h * 0.65)  # bottom 35%
    x0 = max(0, int(line_cx - line_w // 2))
    x1 = min(w, int(line_cx + line_w // 2))
    cv2.rectangle(frame, (x0, roi_top), (x1, h), (0, 200, 220), thickness=-1)
    return frame


@unittest.skipUnless(HAS_NUMPY and HAS_CV2,
                     "numpy + cv2 required for line-follower tests")
class MecanumSteering(unittest.TestCase):
    """Default V5.15c9 mode: emits MEC twist, never strafes."""

    def test_centred_line_emits_pure_forward(self):
        """Line at frame centre → wz=0, vx=base. No rotation jitter."""
        cfg = SimpleLineConfig(steering_mode="mecanum")
        lf = SimpleLineFollower(cfg)
        frame = _make_frame_with_line(line_cx=480)  # exact centre of 960
        r = lf.tick(frame)
        self.assertTrue(r.found)
        self.assertEqual(r.command, "MEC")
        self.assertIsNotNone(r.twist)
        vx, vy, wz, spd = r.twist
        self.assertEqual(vy, 0, "Mecanum mode must never strafe")
        self.assertEqual(wz, 0, "Centred line must produce zero rotation")
        self.assertEqual(vx, cfg.mec_base_vx)
        self.assertEqual(spd, cfg.mec_spd)

    def test_line_right_rotates_right(self):
        """Line to the right → positive wz (rotate CW to face it)."""
        cfg = SimpleLineConfig(steering_mode="mecanum")
        lf = SimpleLineFollower(cfg)
        # Line well outside dead band (96 px) — at 700 vs centre 480 → err=+220 px
        frame = _make_frame_with_line(line_cx=700)
        r = lf.tick(frame)
        self.assertTrue(r.found)
        self.assertEqual(r.command, "MEC")
        vx, vy, wz, _spd = r.twist
        self.assertEqual(vy, 0)
        self.assertGreater(wz, 0, "Line on right ⇒ wz must rotate right (positive)")
        self.assertGreater(vx, 0, "Forward motion must continue while steering")

    def test_line_left_rotates_left(self):
        cfg = SimpleLineConfig(steering_mode="mecanum")
        lf = SimpleLineFollower(cfg)
        # err = 260 - 480 = -220 px (well outside dead band)
        frame = _make_frame_with_line(line_cx=260)
        r = lf.tick(frame)
        self.assertTrue(r.found)
        vx, vy, wz, _spd = r.twist
        self.assertEqual(vy, 0)
        self.assertLess(wz, 0, "Line on left ⇒ wz must rotate left (negative)")
        self.assertGreater(vx, 0)

    def test_wz_clamped_to_max(self):
        """Extreme error must not blow past mec_max_wz."""
        cfg = SimpleLineConfig(steering_mode="mecanum",
                                mec_max_wz=20, mec_kp_steer=400.0)
        lf = SimpleLineFollower(cfg)
        # Line at far edge — gain×err would blow past 20 without clamp.
        frame = _make_frame_with_line(line_cx=900)
        r = lf.tick(frame)
        _vx, _vy, wz, _spd = r.twist
        self.assertLessEqual(abs(wz), 20, f"wz clamp violated: {wz}")

    def test_dead_band_within_tolerance(self):
        """Small wobble (<= straight_band) → wz=0 (no constant correction)."""
        cfg = SimpleLineConfig(steering_mode="mecanum",
                                mec_straight_band=96)
        lf = SimpleLineFollower(cfg)
        # err = 540 - 480 = 60 px — well inside 96 px dead band
        frame = _make_frame_with_line(line_cx=540)
        r = lf.tick(frame)
        _vx, _vy, wz, _spd = r.twist
        self.assertEqual(wz, 0, "Inside dead band must emit wz=0")

    def test_lost_line_stops(self):
        cfg = SimpleLineConfig(steering_mode="mecanum")
        lf = SimpleLineFollower(cfg)
        frame = _make_frame_with_line(line_cx=None)  # empty frame
        r = lf.tick(frame)
        self.assertFalse(r.found)
        self.assertEqual(r.command, "S")
        self.assertEqual(r.speed, 0)


@unittest.skipUnless(HAS_NUMPY and HAS_CV2,
                     "numpy + cv2 required for line-follower tests")
class VerbsModeRegression(unittest.TestCase):
    """Make sure legacy joystick verbs still work for venues where
    Mecanum kinematic split stalls at low PWM."""

    def test_verbs_mode_emits_forward_when_centred(self):
        cfg = SimpleLineConfig(steering_mode="verbs")
        lf = SimpleLineFollower(cfg)
        frame = _make_frame_with_line(line_cx=480)
        r = lf.tick(frame)
        self.assertTrue(r.found)
        self.assertEqual(r.command, "F")
        self.assertIsNone(r.twist, "Verb mode must NOT populate twist")

    def test_verbs_mode_emits_diagonal_on_medium_offset(self):
        cfg = SimpleLineConfig(steering_mode="verbs",
                                straight_band=80, diagonal_band=200)
        lf = SimpleLineFollower(cfg)
        # err = 600 - 480 = 120 px → inside diagonal band [80, 200]
        frame = _make_frame_with_line(line_cx=600)
        r = lf.tick(frame)
        self.assertEqual(r.command, "DR")


if __name__ == "__main__":
    unittest.main()
