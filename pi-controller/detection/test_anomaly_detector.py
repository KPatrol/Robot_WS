"""Unit tests for AnomalyDetector snapshot inline base64 encoding.

Run:  python -m pytest robots/pi-controller/detection/test_anomaly_detector.py

These tests focus on the new V10.1 snapshot embedding feature:
- DetectionEvent has the snapshot_b64 field.
- _encode_snapshot_b64 returns valid base64 JPEG within a sane size budget.
- _emit_event populates the field end-to-end.
- Disabled / missing-cv2 paths return "" (so MQTT payload stays clean).

We deliberately bypass the camera + YOLO threads — those are integration
concerns. Here we drive the helpers directly with synthesized numpy frames.
"""

from __future__ import annotations

import base64
import os
import tempfile
import unittest
from unittest import mock

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

from .anomaly_detector import (
    AnomalyDetector,
    DetectionConfig,
    DetectionEvent,
)


def _make_detector(**overrides) -> AnomalyDetector:
    """Build a detector with snapshot_dir pointing at a temp folder."""
    cfg = DetectionConfig(
        dry_run=False,
        snapshot_dir=tempfile.mkdtemp(prefix="kpatrol_snap_"),
        **overrides,
    )
    det = AnomalyDetector(cfg)
    return det


class DetectionEventField(unittest.TestCase):
    def test_event_has_snapshot_b64_field(self):
        ev = DetectionEvent(
            kind="person",
            confidence=0.9,
            bbox=(10, 20, 30, 40),
            timestamp=123.0,
            snapshot_path="snapshots/foo.jpg",
            frame_width=640,
            frame_height=480,
        )
        self.assertEqual(ev.snapshot_b64, "")  # default empty

    def test_event_snapshot_b64_round_trips_through_json(self):
        ev = DetectionEvent(
            kind="fire",
            confidence=0.45,
            bbox=(0, 0, 10, 10),
            timestamp=1.0,
            snapshot_path="x.jpg",
            frame_width=320,
            frame_height=240,
            snapshot_b64="ABCDEF==",
        )
        import json as _json
        data = _json.loads(ev.to_json())
        self.assertEqual(data["snapshot_b64"], "ABCDEF==")


@unittest.skipUnless(HAS_NUMPY and HAS_CV2,
                     "numpy + cv2 required for image-encoding tests")
class EncodeSnapshotB64(unittest.TestCase):
    def test_encode_returns_valid_jpeg_base64(self):
        det = _make_detector()
        # synth 640x480 BGR frame with a colored gradient (compressible)
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        frame[:, :, 0] = np.tile(np.arange(640, dtype=np.uint8), (480, 1))
        frame[:, :, 1] = 100
        frame[:, :, 2] = np.tile(np.arange(480, dtype=np.uint8)[:, None], (1, 640))

        b64 = det._encode_snapshot_b64(frame)
        self.assertIsInstance(b64, str)
        self.assertGreater(len(b64), 100)  # non-empty
        # Base64 prefix of JPEG SOI marker FFD8 → "/9j/"
        self.assertTrue(b64.startswith("/9j/"),
                        f"expected JPEG b64 prefix, got: {b64[:8]}")
        # Decode and re-decode through cv2 to confirm integrity
        raw = base64.b64decode(b64)
        arr = np.frombuffer(raw, dtype=np.uint8)
        decoded = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        self.assertIsNotNone(decoded)
        # Resize cap should hold (max dim 320 by default)
        self.assertLessEqual(max(decoded.shape[:2]), 320)

    def test_encode_payload_size_under_emqx_default(self):
        """Round-trip must stay safely below EMQX default 256 KB ceiling.

        We deliberately use a high-entropy frame (random noise) — the worst
        case for JPEG. 320×240 q60 still must fit comfortably. We assert
        <100 KB to leave headroom for the rest of the JSON envelope.
        """
        det = _make_detector()
        rng = np.random.default_rng(seed=42)
        frame = rng.integers(0, 256, (480, 640, 3), dtype=np.uint8)
        b64 = det._encode_snapshot_b64(frame)
        self.assertGreater(len(b64), 0)
        self.assertLess(len(b64), 100 * 1024,
                        f"snapshot_b64 too large: {len(b64)} bytes")

    def test_encode_disabled_when_max_dim_zero(self):
        det = _make_detector(snapshot_b64_max_dim=0)
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        self.assertEqual(det._encode_snapshot_b64(frame), "")

    def test_encode_returns_empty_for_none_frame(self):
        det = _make_detector()
        self.assertEqual(det._encode_snapshot_b64(None), "")

    def test_encode_returns_empty_when_cv2_unavailable(self):
        det = _make_detector()
        det._cv2 = None  # simulate dry-run / missing cv2
        frame = np.zeros((100, 100, 3), dtype=np.uint8)
        self.assertEqual(det._encode_snapshot_b64(frame), "")

    def test_encode_preserves_aspect_ratio(self):
        """Tall frame should remain tall, wide frame should remain wide."""
        det = _make_detector(snapshot_b64_max_dim=200)
        wide = np.zeros((100, 800, 3), dtype=np.uint8)
        tall = np.zeros((800, 100, 3), dtype=np.uint8)

        for frame, expect_wider in [(wide, True), (tall, False)]:
            b64 = det._encode_snapshot_b64(frame)
            raw = base64.b64decode(b64)
            decoded = cv2.imdecode(np.frombuffer(raw, np.uint8),
                                   cv2.IMREAD_COLOR)
            h, w = decoded.shape[:2]
            self.assertLessEqual(max(h, w), 200)
            if expect_wider:
                self.assertGreater(w, h)
            else:
                self.assertGreater(h, w)


@unittest.skipUnless(HAS_NUMPY and HAS_CV2,
                     "numpy + cv2 required for emit-event tests")
class EmitEventEndToEnd(unittest.TestCase):
    def test_emit_event_populates_snapshot_b64(self):
        det = _make_detector()
        captured: list[DetectionEvent] = []
        det.on_event = captured.append

        frame = np.full((240, 320, 3), 50, dtype=np.uint8)  # solid grey
        det._emit_event("person", 0.88, (10, 10, 50, 80), frame, 1700000000.0)

        self.assertEqual(len(captured), 1)
        ev = captured[0]
        self.assertEqual(ev.kind, "person")
        self.assertGreater(len(ev.snapshot_b64), 0)
        self.assertTrue(ev.snapshot_b64.startswith("/9j/"))

    def test_emit_event_no_snapshot_b64_when_disabled(self):
        det = _make_detector(snapshot_b64_max_dim=0)
        captured: list[DetectionEvent] = []
        det.on_event = captured.append

        frame = np.full((240, 320, 3), 50, dtype=np.uint8)
        det._emit_event("fire", 0.5, (0, 0, 20, 20), frame, 1700000001.0)

        self.assertEqual(len(captured), 1)
        self.assertEqual(captured[0].snapshot_b64, "")

    def test_emit_event_no_snapshot_b64_in_dry_run(self):
        """Dry-run path returns (path, None) from _save_snapshot, so b64=''."""
        det = _make_detector()
        det._cv2 = None  # simulate camera-less environment
        captured: list[DetectionEvent] = []
        det.on_event = captured.append

        det._emit_event("person", 0.7, (1, 2, 3, 4), None, 1700000002.0)
        self.assertEqual(len(captured), 1)
        self.assertEqual(captured[0].snapshot_b64, "")


if __name__ == "__main__":
    unittest.main()
