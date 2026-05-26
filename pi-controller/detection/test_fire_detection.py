"""Tests for the V10.2 multi-stage fire detector.

The goal of these tests is to lock in the *false-positive* fixes and the
*recall* fix for small flames (lighters) so they don't regress when the
HSV thresholds get re-tuned for a new venue.

Each test builds a synthetic 640×480 BGR frame that targets one stage
of the pipeline:

  test_real_flame_*       - large + small flames with hot core → must fire
  test_red_shirt_*        - red rectangle, no hot core         → must NOT fire
  test_red_led_*          - high-V high-S spot                 → must NOT fire
  test_static_red_*       - same red blob every frame          → flicker reject
  test_horizontal_strip_* - long thin red bar                  → shape reject

Run:
    python -m pytest robots/pi-controller/detection/test_fire_detection.py -v
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

from .anomaly_detector import AnomalyDetector, DetectionConfig


# ─── Helpers ────────────────────────────────────────────────────────────


def _make_detector(**overrides) -> AnomalyDetector:
    """Build a detector for HSV pipeline tests.

    V11.0 default `fire_pipeline` is "yolo" — but these tests verify the
    HSV (V10.4) algorithm against synthetic frames, so we force HSV.
    The YOLO branch is tested separately via test_fire_model.py at the
    venue with a real camera + real model file.
    """
    overrides.setdefault("fire_pipeline", "hsv")
    cfg = DetectionConfig(dry_run=False, **overrides)
    det = AnomalyDetector(cfg)
    # Pin the mode so _resolve_fire_mode() doesn't log warnings every test.
    det._fire_mode_resolved = "hsv"
    return det


def _bgr_from_hsv(h: int, s: int, v: int):
    """Build a single BGR pixel from HSV values via cv2.cvtColor.

    Using cv2 round-tripping ensures the synthesized frame lands in the
    same HSV bin our detector will read it back at — no off-by-one in the
    color-space conversion.
    """
    px = np.uint8([[[h, s, v]]])
    return cv2.cvtColor(px, cv2.COLOR_HSV2BGR)[0, 0].tolist()


def _frame(h=480, w=640, bg=(0, 0, 0)):
    f = np.zeros((h, w, 3), dtype=np.uint8)
    f[:, :] = bg
    return f


def _paint_flame(frame, center_xy, radius=40, outer_only=False):
    """Paint a synthetic flame: yellow-white hot core + orange-red halo.

    center_xy: (cx, cy) — where the flame sits.
    radius:    overall flame radius. Core is ~30% of radius.
    outer_only: if True, only paint the orange halo (simulating a red
                shirt — same color, no bright center).
    """
    cx, cy = center_xy
    h, w = frame.shape[:2]
    # Outer halo — warm orange. HSV (20, 200, 220) ≈ rich orange.
    outer_bgr = _bgr_from_hsv(20, 200, 220)
    cv2.circle(frame, (cx, cy), radius, outer_bgr, thickness=-1)

    if not outer_only:
        # Hot core — near-white with a yellow tint. V high, S low-mid.
        core_bgr = _bgr_from_hsv(25, 80, 250)
        core_r = max(4, int(radius * 0.35))
        cv2.circle(frame, (cx, cy), core_r, core_bgr, thickness=-1)

    return frame


def _paint_red_led(frame, center_xy, radius=8):
    """Saturated red pinprick — like a status LED on an enclosure.

    H ≈ 0, S=255, V=255 — should fail Stage 2 (S > 200 in core test).
    """
    cx, cy = center_xy
    led_bgr = _bgr_from_hsv(0, 255, 255)
    cv2.circle(frame, (cx, cy), radius, led_bgr, thickness=-1)
    return frame


def _paint_red_strip(frame, x, y, w_strip, h_strip):
    """Long thin red rectangle — fails Stage 6 (aspect ratio < 0.4)."""
    h, w = frame.shape[:2]
    # Yellow-tinted strip so it passes Stages 1+2 but fails shape filter.
    # H=25 keeps the yellow/red ratio above the V10.4 skin gate.
    outer_bgr = _bgr_from_hsv(25, 200, 220)
    cv2.rectangle(frame, (x, y), (x + w_strip, y + h_strip),
                  outer_bgr, thickness=-1)
    core_bgr = _bgr_from_hsv(25, 80, 250)
    cv2.rectangle(frame, (x + 2, y + 2),
                  (x + w_strip - 2, y + h_strip - 2),
                  core_bgr, thickness=-1)
    return frame


def _paint_hand_under_neon(frame, top_left, size=(180, 240)):
    """Synthetic bare hand under warm-ish neon — V10.4 must reject this.

    Skin HSV: H ≈ 8–14 (orange), S ≈ 80–120 (moderately saturated),
    V ≈ 180–215 (bright under neon). Big enough (~43k px) to clear
    every area-based gate and bright enough to clear the V_min core.
    The discriminator must be Stage 1B (yellow/red ratio).
    """
    x, y = top_left
    w_, h_ = size
    skin_bgr = _bgr_from_hsv(10, 100, 200)
    cv2.rectangle(frame, (x, y), (x + w_, y + h_), skin_bgr, thickness=-1)
    # A few brighter knuckle highlights so the "core" S/V test sees them.
    highlight_bgr = _bgr_from_hsv(8, 90, 220)
    for cx in range(x + 20, x + w_ - 20, 35):
        cv2.circle(frame, (cx, y + 30), 12, highlight_bgr, thickness=-1)
    return frame


# ─── Test cases ─────────────────────────────────────────────────────────


@unittest.skipUnless(HAS_NUMPY and HAS_CV2,
                     "numpy + cv2 required for fire-detection tests")
class FireDetectionRecall(unittest.TestCase):
    """Make sure real flames — including small ones — get caught."""

    def test_large_real_flame_detected(self):
        det = _make_detector()
        frame = _frame()
        _paint_flame(frame, (320, 240), radius=60)

        # Push the same frame 4× so the temporal flicker filter has
        # history. We perturb each frame slightly so flicker passes.
        for i in range(4):
            f = frame.copy()
            _paint_flame(f, (320 + i, 240 - i), radius=60 + (i % 2))
            bbox, ratio = det._detect_fire(f)

        self.assertIsNotNone(bbox, "Large flame must be detected")
        self.assertGreater(ratio, 0.001)

    def test_small_lighter_flame_detected(self):
        """V10.2 specifically targets lighters at ~50 cm.

        Lighter flame at that distance is ~12-18 px tall, ~6-10 px wide.
        Old threshold (min_area_ratio=0.005, ~1500 px) silently rejected
        these. New threshold 0.0008 (~250 px) should pick them up.
        """
        det = _make_detector()
        # Lighter flame: ~15 px tall, ~8 px wide = ~120 px area. Slightly
        # under our threshold by area but our circle approximation gives
        # ~300 px which lands above 250.
        for i in range(4):
            f = _frame()
            _paint_flame(f, (320 + i, 240 - i), radius=10 + (i % 2))
            bbox, ratio = det._detect_fire(f)

        self.assertIsNotNone(bbox, "Small lighter flame must be detected")

    def test_flicker_history_required_for_motion_gate(self):
        """First frame goes through even without history (smoother handles)."""
        det = _make_detector()
        frame = _frame()
        _paint_flame(frame, (320, 240), radius=50)
        bbox, _ = det._detect_fire(frame)
        self.assertIsNotNone(bbox, "First frame must pass when flame is real")

    def test_v10_3_tiny_lighter_under_neon_indoor(self):
        """V10.3 regression guard: tiny lighter (~radius 6) on a bright grey
        background — the exact failure mode the operator reported for the
        indoor demo. Pre-V10.3, AGC drove core V to ~210 and 8-px halo
        eroded to nothing in stage 1's OPEN. Post-V10.3 must catch it."""
        det = _make_detector()
        # Bright grey background simulating neon-lit office walls reflecting
        # ambient light. Camera AGC would normally clamp these to V~200.
        bg_bgr = _bgr_from_hsv(0, 0, 200)
        for i in range(4):
            f = _frame(bg=tuple(int(v) for v in bg_bgr))
            # Paint a small flame with reduced core brightness (V=210 instead
            # of 250 — what AGC-clamped lighter actually looks like).
            cx, cy = 320 + i, 240 - i
            outer_bgr = _bgr_from_hsv(20, 200, 200)
            cv2.circle(f, (cx, cy), 8, outer_bgr, thickness=-1)
            core_bgr = _bgr_from_hsv(25, 80, 210)
            cv2.circle(f, (cx, cy), 3, core_bgr, thickness=-1)
            bbox, _ = det._detect_fire(f)

        self.assertIsNotNone(
            bbox,
            "V10.3 must catch a tiny lighter (~r=8 px, V_core=210) — "
            "the exact failure mode reported for the indoor demo.",
        )

    def test_v10_3_steady_flame_passes_relaxed_flicker(self):
        """Real lighter flame at 10 FPS shows ~3-7% area variation between
        frames from natural flame dance + hand tremor. Old config (0.92)
        let some through; V10.3 (0.97) lets all genuine flames through.
        Synthesize ~5 px center shift + radius perturbation."""
        det = _make_detector()
        # Centers + radii from a recorded lighter-flame trace (approximated).
        shapes = [(320, 240, 28), (325, 238, 30), (322, 242, 26), (327, 240, 29)]
        bbox = None
        for cx, cy, r in shapes:
            f = _frame()
            _paint_flame(f, (cx, cy), radius=r)
            bbox, _ = det._detect_fire(f)

        self.assertIsNotNone(
            bbox,
            "A real-world lighter flame (small position+radius variation) "
            "must pass V10.3 flicker gate",
        )


@unittest.skipUnless(HAS_NUMPY and HAS_CV2,
                     "numpy + cv2 required for fire-detection tests")
class FireDetectionPrecision(unittest.TestCase):
    """The harder problem — must NOT fire on common false positives."""

    def test_red_shirt_rejected(self):
        """Red rectangle without a hot core. Real red shirts have S~150,
        V~120 → Stage 2 (V>=230) excludes them."""
        det = _make_detector()
        for _ in range(5):
            f = _frame()
            _paint_flame(f, (320, 240), radius=80, outer_only=True)
            bbox, _ = det._detect_fire(f)
        self.assertIsNone(bbox, "Red shirt must NOT trigger fire")

    def test_static_red_object_rejected_by_flicker(self):
        """Identical mask every frame → IoU≈1 → flicker filter rejects."""
        det = _make_detector()
        frame = _frame()
        _paint_flame(frame, (320, 240), radius=60)

        last_bbox = None
        # 5 IDENTICAL frames — no perturbation. After enough history
        # accumulates, the flicker gate must reject.
        for i in range(8):
            bbox, _ = det._detect_fire(frame)
            last_bbox = bbox

        self.assertIsNone(
            last_bbox,
            "Perfectly-static flame-shaped mask must be rejected as 'not flickering'",
        )

    def test_red_led_rejected(self):
        """Tiny saturated red dot — passes outer mask but S=255 fails
        Stage 2 (core S<=200) AND area is below min_area_ratio."""
        det = _make_detector()
        for _ in range(5):
            f = _frame()
            _paint_red_led(f, (320, 240), radius=10)
            bbox, _ = det._detect_fire(f)
        self.assertIsNone(bbox, "Saturated red LED must NOT trigger fire")

    def test_horizontal_red_strip_rejected_by_shape(self):
        """Long thin horizontal red bar — aspect h/w = 0.1 → Stage 6 rejects."""
        det = _make_detector()
        for i in range(5):
            f = _frame()
            # Small jitter so flicker filter doesn't reject for static reason.
            _paint_red_strip(f, 100 + i, 240, 400, 30)
            bbox, _ = det._detect_fire(f)
        self.assertIsNone(
            bbox, "Horizontal red strip must NOT trigger fire (aspect ratio)"
        )

    def test_v10_4_human_hand_rejected_by_skin_gate(self):
        """V10.4 regression: bare hand under warm neon must NOT trigger fire.

        Synthesizes a hand-sized orange-pink rectangle (H=10, S=100, V=200)
        — the exact failure mode reported at the demo venue. Pre-V10.4
        every stage waved this through because skin HSV sits inside the
        outer envelopes and a hand is 40k+ px.
        """
        det = _make_detector()
        bbox = None
        for _ in range(5):
            f = _frame()
            _paint_hand_under_neon(f, top_left=(200, 100))
            bbox, _ = det._detect_fire(f)
        self.assertIsNone(
            bbox,
            "V10.4 must reject a bare hand — Stage 1B yellow/red ratio gate.",
        )

    def test_empty_frame_no_detection(self):
        det = _make_detector()
        f = _frame(bg=(20, 20, 20))  # dim grey, no flame-colored pixels
        bbox, ratio = det._detect_fire(f)
        self.assertIsNone(bbox)
        self.assertLess(ratio, 0.0001)

    def test_room_bathed_in_orange_light_rejected_by_area_cap(self):
        """Sunset glare on the whole frame — passes outer + core but
        ratio > fire_max_area_ratio (40%) → rejected."""
        det = _make_detector()
        f = _frame()
        # Fill 50% of frame with warm orange — sunset on a wall.
        warm_bgr = _bgr_from_hsv(20, 100, 240)
        cv2.rectangle(f, (0, 0), (640, 250), warm_bgr, thickness=-1)
        bbox, ratio = det._detect_fire(f)
        self.assertIsNone(
            bbox, "Frame-filling warm light must be rejected by area cap"
        )


@unittest.skipUnless(HAS_NUMPY and HAS_CV2,
                     "numpy + cv2 required for fire-detection tests")
class FireDetectionConfig(unittest.TestCase):
    """Config-surface sanity — make sure the new fields exist and have
    the documented defaults so anyone re-tuning sees what to touch."""

    def test_defaults_match_documentation(self):
        """V11.0 defaults — YOLO is primary, HSV is fallback.

        V11.0 (2026-05-25 PM): introduces `fire_pipeline` default "yolo" so
        a properly downloaded fire model is used preferentially.
        2026-05-26 update: HSV thresholds from V10.4 are explicitly kept as
        the *fallback* path (auto-engaged by `_resolve_fire_mode` when the
        YOLO model file is missing or ONNX runtime fails to load). On
        well-lit demo floors HSV produced too many false positives against
        bright orange skin tones, so HSV is no longer the default.
        """
        cfg = DetectionConfig()
        # Default = YOLO. The fire model lives at `models/fire_yolov8n.pt`
        # (auto-exported to ONNX on first inference). If that file is
        # absent at startup, `_resolve_fire_mode` logs a warning once and
        # silently falls back to HSV so detection still runs.
        self.assertEqual(cfg.fire_pipeline, "yolo")
        self.assertEqual(cfg.fire_yolo_model, "models/fire_yolov8n.pt")
        self.assertEqual(cfg.fire_yolo_confidence, 0.30)
        self.assertEqual(cfg.fire_yolo_imgsz, 416)
        # Default indices match the touati-kamel forest-fire model:
        # fire=3, fire-smoke=0. Edit DetectionConfig when downloading a
        # model with different class layout.
        self.assertEqual(cfg.fire_yolo_fire_class, 3)
        self.assertEqual(cfg.fire_yolo_smoke_class, 0)

        # V10.4 HSV fallback defaults preserved
        self.assertEqual(cfg.fire_min_area_ratio, 0.0003)
        self.assertEqual(cfg.fire_max_area_ratio, 0.40)
        self.assertEqual(cfg.fire_hot_core_v_min, 200)
        self.assertEqual(cfg.fire_hot_core_s_min, 30)
        self.assertEqual(cfg.fire_hot_core_s_max, 220)
        self.assertEqual(cfg.fire_hot_core_min_pixels, 4)
        self.assertEqual(cfg.fire_core_dilate_iter, 6)
        self.assertEqual(cfg.fire_aspect_ratio_min, 0.3)
        self.assertEqual(cfg.fire_max_static_iou, 0.97)
        self.assertEqual(cfg.fire_min_motion_frames, 1)
        self.assertEqual(cfg.fire_smoothing_required, 2)
        self.assertEqual(cfg.fire_min_yellow_over_red_ratio, 0.4)

    def test_v11_yolo_mode_falls_back_when_model_missing(self):
        """If `fire_pipeline=yolo` but the model file isn't on disk,
        the detector must auto-fall back to HSV without crashing.

        This is the safety property that lets operators ship the new
        default without first running the download script — the system
        still works, just at HSV accuracy.
        """
        cfg = DetectionConfig(dry_run=False, fire_pipeline="yolo",
                              fire_yolo_model="/nonexistent/path/fire.onnx")
        det = AnomalyDetector(cfg)
        # Resolve mode — should pick HSV because file is missing.
        mode = det._resolve_fire_mode()
        self.assertEqual(mode, "hsv",
                         "Missing YOLO model must fall back to HSV pipeline")

    def test_v11_env_override_forces_hsv(self):
        """KPATROL_FIRE_MODE=hsv in env must override the config setting.

        Operators use this to switch back to HSV at the venue without
        editing code (e.g. when the YOLO model is producing false
        positives on a specific lighting condition).
        """
        import os as _os
        prev = _os.environ.get("KPATROL_FIRE_MODE")
        _os.environ["KPATROL_FIRE_MODE"] = "hsv"
        try:
            cfg = DetectionConfig(dry_run=False, fire_pipeline="yolo")
            det = AnomalyDetector(cfg)
            mode = det._resolve_fire_mode()
            self.assertEqual(mode, "hsv",
                             "KPATROL_FIRE_MODE=hsv must force HSV mode")
        finally:
            if prev is None:
                _os.environ.pop("KPATROL_FIRE_MODE", None)
            else:
                _os.environ["KPATROL_FIRE_MODE"] = prev


if __name__ == "__main__":
    unittest.main()
