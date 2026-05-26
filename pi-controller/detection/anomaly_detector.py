"""
K-Patrol Anomaly Detector
--------------------------

Edge-AI person + fire detection running on Raspberry Pi 4 Model B.

- YOLOv8n (ultralytics) for person detection (COCO class 0).
- Lightweight fire/smoke detector via HSV color segmentation + motion heuristic
  (cheaper than a fire-trained YOLO model and runs in realtime on Pi).
- Emits events via the on_event callback (typically AlertBridge.publish, which
  persists to SQLite then forwards to MQTT topic `kpatrol/{serial}/alert`).
- Saves annotated snapshots under ./snapshots/{ts}_{kind}.jpg with a rolling
  buffer (snapshot_max_keep). Inline base64-encoded thumbnails are also
  embedded in the event payload for fast preview on subscribers.

Designed to run as a sidecar process to the main controller. Imports are lazy
so the module can be imported on a dev laptop without cv2/ultralytics.

Usage:
    detector = AnomalyDetector(config, on_event=bridge.publish)
    detector.start(blocking=True)
"""

from __future__ import annotations

import base64
import json
import logging
import os
import threading
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from queue import Queue, Empty
from typing import Callable, Optional, Union

from .temporal_smoother import TemporalSmoother

log = logging.getLogger("kpatrol.detection")


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

@dataclass
class DetectionConfig:
    """Runtime configuration for the detector."""

    # Camera
    # Accepts either an integer device index (V4L2) or a URL string (e.g.
    # `http://127.0.0.1:8080/stream` for the MJPEG sidecar that owns
    # /dev/video0 on the Pi). cv2.VideoCapture handles both natively.
    camera_index: Union[int, str] = 0
    frame_width: int = 640
    frame_height: int = 480
    fps: int = 10

    # YOLO — V10 spec §5.5: 416×416 input for YOLOv8n int8.
    # Prefer ONNX on Pi (≈2× faster CPU inference vs the PyTorch backend
    # ultralytics ships with). If only the .pt is present at startup,
    # `_ensure_yolo` exports an .onnx beside it the first time, then uses
    # the .onnx for subsequent runs. Set `yolo_model_prefer_onnx=False`
    # to force the .pt path (debug / dev laptop).
    yolo_model: str = "yolov8n.pt"
    yolo_model_prefer_onnx: bool = True
    yolo_confidence: float = 0.55
    yolo_imgsz: int = 416

    # Person detection
    person_enabled: bool = True
    # Cooldown gates how fast the web sees a new alert. 8s was conservative
    # for early field testing; bring it down so the PWA refreshes promptly
    # while still throttling noise. Per-event cooldown still wins over
    # smoothing latency (~0.2s at fps=10).
    person_cooldown_sec: float = 3.0
    person_min_area_ratio: float = 0.02  # bbox must cover >=2% of frame

    # Fire detection — V10.4 (2026-05-25) tightens V10.3 to reject "tay
    # người = lửa" while keeping the indoor-neon lighter recall.
    #
    # V10.3 relaxed thresholds for tiny indoor flames. New failure mode:
    # bare human skin under warm neon lights up as fire because skin
    # HSV (H≈5–20, S≈60–150, V≈150–230) falls inside the old envelopes,
    # and a hand at arm's length is 6–15k px — well over the 92 px area
    # floor.
    #
    # V10.4 adds two cheap discriminators that genuine flames pass but
    # skin doesn't:
    #
    #   (a) Outer hue REGION split. We separate the outer mask into
    #       - "flame yellow"  H = 18–38   (real flame core hue)
    #       - "wrap red"      H = 0–12  + 160–179  (flame edges, ember)
    #       and require flame_yellow_px >= ratio × wrap_red_px. Skin is
    #       uniformly orange-pink with NO yellow centre → fails. Real
    #       flame has a yellow tip → passes.
    #
    #   (b) Hot-core SATURATION FLOOR (s_min = 30). Real flame cores
    #       carry a yellow tinge (S ≥ 30) even when bright. Bright skin
    #       and white specular highlights tend to S < 30. Combined with
    #       (a), this is the discriminator.
    #
    # Outer envelopes also tightened: S floor 60→80, yellow band V floor
    # 180→200 — both push skin out without losing real-flame recall.
    fire_enabled: bool = True
    fire_cooldown_sec: float = 5.0

    # V11.0 (2026-05-25): pipeline mode switch.
    #   "yolo" — DEFAULT (2026-05-26). Dedicated fire YOLO model gives
    #            far fewer false positives than HSV thresholding on
    #            indoor demo floors. HSV is preserved as automatic
    #            fallback (see `_resolve_fire_mode`) so the detector
    #            still runs if the .pt/.onnx file is missing.
    #   "hsv"  — Manual opt-in. V10.4 6-stage pipeline with skin gate.
    #            Useful for environments where the YOLO model has not
    #            been deployed yet or as a smoke test against a
    #            broken model file. False positive rate is materially
    #            higher than YOLO under bright neon / orange skin tones,
    #            which is why we no longer default to it.
    # Operator can switch modes via `KPATROL_FIRE_MODE=yolo|hsv` env var
    # without redeploying. AnomalyDetector auto-falls-back to HSV if
    # YOLO is selected but the model file is missing or fails to load.
    fire_pipeline: str = "yolo"
    # Path to a fire/smoke YOLO model. The download script drops .pt here;
    # ultralytics auto-exports an ONNX sibling on first inference for
    # ~1.5× speedup on Pi 4B ARM. `_ensure_fire_yolo()` looks for INT8
    # ONNX → ONNX → .pt in that order, so any of those formats works.
    fire_yolo_model: str = "models/fire_yolov8n.pt"
    # Person YOLO runs at 0.55; fire detection is rarer so we use a
    # lower threshold to catch faint flames, then rely on temporal
    # smoothing to suppress single-frame noise.
    fire_yolo_confidence: float = 0.30
    fire_yolo_imgsz: int = 416
    # Class index map for the fire model. Defaults match the touati-kamel
    # YOLOv8s forest-fire model on HuggingFace:
    #   0: fire-smoke   (combined)
    #   1: fog          (NOT a target)
    #   2: sol          (sun glare — NOT a target)
    #   3: fire         (pure flame)
    #   4: factory-smoke (industrial)
    # If your downloaded model uses different indices, edit these here.
    # Set smoke_class to -1 to ignore smoke alerts (lighter-only demos).
    fire_yolo_fire_class: int = 3
    fire_yolo_smoke_class: int = 0

    # 0.0003 ≈ 92 px on 640×480 — about the size of a candle flame at 1 m or
    # a lighter at 60 cm. Floor still well above sensor speckle (< 20 px).
    fire_min_area_ratio: float = 0.0003
    fire_max_area_ratio: float = 0.40

    # Outer flame HSV. V10.4 tightening:
    #   - Red wrap S floor 60→80 (skin S can dip to 60)
    #   - Yellow core V floor 180→200 (matches real flame; skin rarely V≥200)
    #   - Red wrap H upper 15→12 (12-18 is skin tone territory)
    #   - Yellow H lower 10→18 (10-18 is skin tone too)
    fire_hsv_low_red1: tuple = (0, 80, 160)
    fire_hsv_high_red1: tuple = (12, 255, 255)
    fire_hsv_low_red2: tuple = (160, 80, 160)
    fire_hsv_high_red2: tuple = (179, 255, 255)
    fire_hsv_low_yellow: tuple = (18, 80, 200)
    fire_hsv_high_yellow: tuple = (38, 255, 255)

    # V10.4 skin discriminator: require flame_yellow ≥ ratio × wrap_red.
    # Real candle: ratio > 1 (mostly yellow). Real wood-fire: ratio ~ 0.5.
    # Skin: ratio ~ 0 (no yellow centre). 0.4 splits cleanly.
    fire_min_yellow_over_red_ratio: float = 0.4

    # Hot-core requirement. V10.4 adds S_min = 30 (genuine flame core
    # carries yellow tinge; specular skin highlight does not).
    fire_hot_core_v_min: int = 200
    fire_hot_core_s_min: int = 30
    fire_hot_core_s_max: int = 220
    fire_hot_core_min_pixels: int = 4
    fire_core_dilate_iter: int = 6  # ~12 px tolerance for outer-near-core

    # Shape filter — accept tilted lighters down to ~h/w 0.3. Below that
    # we're looking at a horizontal strip (banner, marker line) which has
    # never been a real flame in the test set.
    fire_aspect_ratio_min: float = 0.3

    # Temporal flicker filter. The threshold semantics: a frame whose mask
    # has IoU > max_static_iou vs any history frame is treated as "static".
    # Lower threshold = STRICTER (more things classified static → rejected).
    #
    # V10.3 RAISED the threshold 0.92 → 0.97 (more lenient) — the previous
    # value rejected a steady bật lửa flame whose pixel-level dancing was
    # below 8% per frame at 10 FPS. Real-world IoU distribution:
    #   • dead-static poster on wall  → 0.99+ (only sensor noise)
    #   • steady lighter / candle     → 0.93–0.97 (subtle dancing)
    #   • dancing campfire flame      → 0.5–0.8
    # 0.97 lets all genuine flames through while still catching the poster.
    # min_motion_frames stays 1 so a single dissimilar frame in the buffer
    # is enough — robot operator doesn't have to wait for 2 frames of
    # disagreement before the alert fires.
    fire_flicker_window: int = 5
    fire_max_static_iou: float = 0.97
    fire_min_motion_frames: int = 1

    # Diagnostic: log every Nth rejection (avoid spamming at 10 FPS).
    # Set to 1 for full trace when debugging, default 30 = ~3 s at 10 FPS.
    fire_reject_log_every_n: int = 30

    # Snapshots — V10 spec §5.5: JPEG quality 70, base64-encoded inline payload
    snapshot_dir: str = "snapshots"
    snapshot_jpeg_quality: int = 70
    snapshot_max_keep: int = 500  # rolling buffer

    # Inline snapshot (base64-encoded JPEG embedded in MQTT alert payload).
    # Frame is resized so the longest side ≤ snapshot_b64_max_dim and re-
    # encoded at snapshot_b64_jpeg_quality. With 320×240 @ q70 a typical
    # JPEG is ~20–35 KB, ~30–45 KB after base64 — well under the EMQX
    # 256 KB default limit. Set max_dim=0 to disable inline snapshots.
    snapshot_b64_max_dim: int = 320
    snapshot_b64_jpeg_quality: int = 70

    # MQTT topic (prefix only, serial appended at runtime)
    mqtt_topic_prefix: str = "kpatrol"
    robot_serial: str = "KPATROL-001"

    # Temporal smoothing (N-of-M consensus before emitting an event).
    # Suppresses single-frame flickers from YOLO/HSV noise.
    # V10.3: dropped 3→2 frames because per-frame stages are still strict
    # (outer hue + core overlap + dilate), so 2-of-5 is enough to suppress
    # noise without making the operator wait 300 ms (3 frames @ 10 FPS) for
    # a real flame to register. At 10 FPS, 2 frames = 200 ms reaction.
    smoothing_window: int = 5
    fire_smoothing_required: int = 2
    person_smoothing_required: int = 2

    # Runtime
    dry_run: bool = False  # no camera, synthesize events (dev mode)
    log_level: str = "INFO"


# ---------------------------------------------------------------------------
# Event dataclass
# ---------------------------------------------------------------------------

@dataclass
class DetectionEvent:
    kind: str                  # "person" | "fire" | "motion"
    confidence: float
    bbox: tuple                # (x, y, w, h) in pixels
    timestamp: float           # unix seconds
    snapshot_path: str         # relative path
    frame_width: int
    frame_height: int
    extra: dict = field(default_factory=dict)
    # Inline snapshot: small JPEG re-encoded then base64 ascii. Empty string
    # when cv2 is unavailable or inline snapshots are disabled.
    snapshot_b64: str = ""

    def to_json(self) -> str:
        return json.dumps(asdict(self), separators=(",", ":"))


# ---------------------------------------------------------------------------
# Lazy imports for cv2 / ultralytics
# ---------------------------------------------------------------------------

def _lazy_import_cv2():
    try:
        import cv2  # type: ignore
        return cv2
    except ImportError:
        log.warning("cv2 not available — detector running in dry_run mode")
        return None


def _lazy_import_yolo():
    try:
        from ultralytics import YOLO  # type: ignore
        return YOLO
    except ImportError:
        log.warning("ultralytics not available — person detection disabled")
        return None


# ---------------------------------------------------------------------------
# Detector
# ---------------------------------------------------------------------------

class AnomalyDetector:
    """Main detector loop.

    Spawns:
      - capture thread (reads frames from camera into queue)
      - inference thread (runs YOLO + fire on each frame)
      - publisher callback fires on detected events
    """

    def __init__(
        self,
        config: Optional[DetectionConfig] = None,
        on_event: Optional[Callable[[DetectionEvent], None]] = None,
    ):
        self.config = config or DetectionConfig()
        self.on_event = on_event
        logging.basicConfig(level=getattr(logging, self.config.log_level))

        self._cv2 = None if self.config.dry_run else _lazy_import_cv2()
        self._yolo_cls = None if self.config.dry_run else _lazy_import_yolo()
        self._yolo_model = None  # lazy-loaded on first frame
        # V11.0: separate YOLO model for fire/smoke detection. Loaded
        # lazily on first frame; if the file is missing or ONNX runtime
        # complains, we fall back to the V10.4 HSV pipeline automatically.
        self._fire_yolo_model = None
        self._fire_yolo_load_attempted = False
        # Resolved pipeline mode after a successful load attempt — read by
        # _detect_fire() each frame to decide which branch to run.
        self._fire_mode_resolved: Optional[str] = None

        self._running = False
        self._frame_queue: Queue = Queue(maxsize=2)
        self._threads: list[threading.Thread] = []

        self._last_person_ts = 0.0
        self._last_fire_ts = 0.0

        self._smoother = TemporalSmoother(
            window=self.config.smoothing_window,
            required={
                "fire": self.config.fire_smoothing_required,
                "person": self.config.person_smoothing_required,
            },
            class_kinds=("person", "fire"),
        )
        # Hold the most recent raw bbox per kind so that when the smoother
        # finally fires, we still have a spatial reference for the snapshot.
        self._last_bbox: dict[str, tuple] = {}

        # Rolling buffer of fire masks for the per-pixel flicker filter
        # (stage 4 in _detect_fire). Stored as raw uint8 masks; lifetime
        # bound by deque maxlen so memory is fixed.
        from collections import deque
        self._fire_mask_history: deque = deque(
            maxlen=max(1, self.config.fire_flicker_window)
        )

        Path(self.config.snapshot_dir).mkdir(parents=True, exist_ok=True)

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self, blocking: bool = True) -> None:
        if self._running:
            return
        self._running = True
        log.info("[detector] starting (dry_run=%s)", self.config.dry_run)

        if self.config.dry_run:
            t = threading.Thread(target=self._dry_run_loop, daemon=True)
            t.start()
            self._threads.append(t)
        else:
            t_cap = threading.Thread(target=self._capture_loop, daemon=True)
            t_inf = threading.Thread(target=self._inference_loop, daemon=True)
            t_cap.start()
            t_inf.start()
            self._threads.extend([t_cap, t_inf])

        if blocking:
            try:
                while self._running:
                    time.sleep(0.5)
            except KeyboardInterrupt:
                self.stop()

    def stop(self) -> None:
        log.info("[detector] stopping")
        self._running = False
        for t in self._threads:
            t.join(timeout=2.0)
        self._threads.clear()

    # ------------------------------------------------------------------
    # Capture loop
    # ------------------------------------------------------------------

    def _capture_loop(self) -> None:
        cv2 = self._cv2
        if cv2 is None:
            log.error("[detector] no cv2, capture loop cannot start")
            return

        def _open():
            cap = cv2.VideoCapture(self.config.camera_index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)
            cap.set(cv2.CAP_PROP_FPS, self.config.fps)
            return cap

        cap = _open()
        interval = 1.0 / max(1, self.config.fps)
        # If the USB camera unplugs or the driver returns errors for too long,
        # re-open the device rather than burning CPU on a dead handle.
        consecutive_failures = 0
        failure_threshold = max(self.config.fps, 10)  # ~1s of bad reads

        while self._running:
            start = time.perf_counter()
            try:
                ok, frame = cap.read()
            except Exception as exc:
                log.warning("[detector] capture read raised: %s", exc)
                ok, frame = False, None

            if not ok or frame is None:
                consecutive_failures += 1
                if consecutive_failures >= failure_threshold:
                    log.warning("[detector] camera unresponsive — reopening")
                    try:
                        cap.release()
                    except Exception:
                        pass
                    cap = _open()
                    consecutive_failures = 0
                time.sleep(0.1)
                continue
            consecutive_failures = 0

            # drop oldest if queue full — always keep most recent
            if self._frame_queue.full():
                try:
                    self._frame_queue.get_nowait()
                except Empty:
                    pass
            self._frame_queue.put(frame)
            elapsed = time.perf_counter() - start
            time.sleep(max(0.0, interval - elapsed))

        try:
            cap.release()
        except Exception:
            pass

    # ------------------------------------------------------------------
    # Inference loop
    # ------------------------------------------------------------------

    def _inference_loop(self) -> None:
        while self._running:
            try:
                frame = self._frame_queue.get(timeout=0.5)
            except Empty:
                continue

            # Guard the entire per-frame pipeline so a single bad frame or
            # transient error in any sub-step never silently kills the thread.
            try:
                self._process_frame(frame)
            except Exception as exc:
                log.exception("[detector] inference iteration failed: %s", exc)

    def _process_frame(self, frame) -> None:
        h, w = frame.shape[:2]
        now = time.time()

        # Collect raw hits per class for this frame; the smoother decides
        # if any of them actually fire after N-of-M consensus.
        raw: dict[str, float] = {}

        if self.config.person_enabled and self._ensure_yolo():
            best_conf = 0.0
            best_bbox: Optional[tuple] = None
            for bbox, conf in self._detect_persons(frame):
                x, y, bw, bh = bbox
                area_ratio = (bw * bh) / (w * h)
                if area_ratio < self.config.person_min_area_ratio:
                    continue
                if conf > best_conf:
                    best_conf = float(conf)
                    best_bbox = bbox
            if best_bbox is not None:
                raw["person"] = best_conf
                self._last_bbox["person"] = best_bbox

        if self.config.fire_enabled:
            bbox, ratio = self._detect_fire(frame)
            if bbox is not None and ratio >= self.config.fire_min_area_ratio:
                raw["fire"] = float(ratio)
                self._last_bbox["fire"] = bbox

        stable = self._smoother.update(raw)
        for kind, conf in stable.items():
            # Per-kind cooldown still applies on top of smoothing.
            last_ts = self._last_person_ts if kind == "person" else self._last_fire_ts
            cooldown = (
                self.config.person_cooldown_sec
                if kind == "person"
                else self.config.fire_cooldown_sec
            )
            if now - last_ts < cooldown:
                continue
            bbox = self._last_bbox.get(kind, (0, 0, 0, 0))
            if kind == "person":
                self._last_person_ts = now
            else:
                self._last_fire_ts = now
            self._emit_event(kind, conf, bbox, frame, now)

    # ------------------------------------------------------------------
    # Detection primitives
    # ------------------------------------------------------------------

    def _ensure_yolo(self) -> bool:
        if self._yolo_model is not None:
            return True
        if self._yolo_cls is None:
            return False
        try:
            model_path = self._resolve_yolo_model_path()
            log.info("[detector] loading YOLO model: %s", model_path)
            self._yolo_model = self._yolo_cls(model_path)
            return True
        except Exception as exc:
            log.error("[detector] failed to load YOLO: %s", exc)
            return False

    def _resolve_yolo_model_path(self) -> str:
        """Pick the best available weights path.

        Preference order when `yolo_model_prefer_onnx` is True and the config
        points at a .pt:
            1. <stem>_int8.onnx — quantized via tools/quantize_yolo.py
               (~3× smaller, ~1.5–2× faster on Pi 4 CPU)
            2. <stem>.onnx — FP32 export (~2× faster than .pt)
            3. <stem>.pt — original PyTorch weights (slowest backend)

        The int8 variant is opt-in: it only exists after running
        `python -m tools.quantize_yolo --auto` once on the Pi. Falls back to
        the original path on any failure — the caller still gets a working
        model, just on the slower backend.
        """
        configured = self.config.yolo_model
        if not self.config.yolo_model_prefer_onnx:
            return configured
        if not configured.endswith(".pt"):
            return configured

        stem = configured[:-3]
        int8_path = f"{stem}_int8.onnx"
        if os.path.exists(int8_path):
            log.info("[detector] using INT8-quantized ONNX: %s", int8_path)
            return int8_path

        onnx_path = stem + ".onnx"
        if os.path.exists(onnx_path):
            return onnx_path
        if not os.path.exists(configured):
            # Ultralytics will auto-download well-known names like
            # "yolov8n.pt"; let it handle that on the .pt path. ONNX
            # export needs the local .pt anyway, so deferring is correct.
            return configured

        try:
            log.info("[detector] exporting %s → ONNX (one-time)", configured)
            tmp_model = self._yolo_cls(configured)
            exported = tmp_model.export(
                format="onnx",
                imgsz=self.config.yolo_imgsz,
                opset=12,
                simplify=True,
            )
            exported_path = str(exported) if exported else onnx_path
            if os.path.exists(exported_path):
                log.info("[detector] ONNX export ready: %s", exported_path)
                return exported_path
            log.warning("[detector] ONNX export reported success but file missing; using .pt")
        except Exception as exc:
            log.warning("[detector] ONNX export failed (%s); falling back to .pt", exc)
        return configured

    def _detect_persons(self, frame):
        """Run YOLO and yield (bbox, confidence) for class=person."""
        try:
            results = self._yolo_model.predict(
                frame,
                conf=self.config.yolo_confidence,
                imgsz=self.config.yolo_imgsz,
                classes=[0],  # person only
                verbose=False,
            )
        except Exception as exc:
            log.warning("[detector] YOLO inference error: %s", exc)
            return

        for r in results:
            boxes = getattr(r, "boxes", None)
            if boxes is None:
                continue
            for b in boxes:
                xyxy = b.xyxy[0].tolist()
                conf = float(b.conf[0])
                x1, y1, x2, y2 = map(int, xyxy)
                yield (x1, y1, x2 - x1, y2 - y1), conf

    def _detect_fire(self, frame):
        """Fire detection — V11.0 dual-mode (YOLO preferred, HSV fallback).

        Pipeline selection lives in `config.fire_pipeline`:
          * "yolo" → try to load `config.fire_yolo_model` ONNX. On success,
            run YOLO inference and return the highest-confidence fire/smoke
            bbox. On failure (file missing, ONNX load error, classes don't
            match), log once and fall through to HSV automatically.
          * "hsv"  → run the V10.4 6-stage HSV pipeline (see `_detect_fire_hsv`).
            Kept as a backup because it has no model dependency and is well
            understood after thesis tuning.

        Returns (bbox, area_ratio) on detection, (None, 0.0) on rejection.
        """
        cv2 = self._cv2
        if cv2 is None:
            return None, 0.0

        mode = self._fire_mode_resolved
        if mode is None:
            mode = self._resolve_fire_mode()
            self._fire_mode_resolved = mode

        if mode == "yolo":
            bbox, conf = self._detect_fire_yolo(frame)
            if bbox is not None:
                return bbox, conf
            # YOLO didn't fire — return cleanly. We do NOT fall through to
            # HSV per-frame, because HSV's false-positive rate against
            # bright skin makes the two pipelines disagree noisily.
            return None, 0.0

        # HSV branch (config="hsv" or YOLO load failed)
        return self._detect_fire_hsv(frame)

    def _resolve_fire_mode(self) -> str:
        """Decide once which pipeline to run for the lifetime of this detector.

        Honors `KPATROL_FIRE_MODE` env override (operator can force a mode
        without redeploying). If the operator picked YOLO but the model
        file is missing or ONNX runtime fails to load, falls back to HSV
        with a single warning line — better than logging once per frame.
        """
        want = (os.environ.get("KPATROL_FIRE_MODE") or self.config.fire_pipeline or "yolo").lower()
        if want == "hsv":
            log.info("[detector] fire pipeline = HSV (V10.4 6-stage)")
            return "hsv"

        # want == "yolo" — try to load
        if not self._ensure_fire_yolo():
            log.warning(
                "[detector] fire pipeline = HSV (YOLO model unavailable, "
                "see tools/download_fire_model.py)"
            )
            return "hsv"
        log.info(
            "[detector] fire pipeline = YOLO (%s · conf=%.2f · imgsz=%d)",
            self.config.fire_yolo_model,
            self.config.fire_yolo_confidence,
            self.config.fire_yolo_imgsz,
        )
        return "yolo"

    def _ensure_fire_yolo(self) -> bool:
        """Lazy-load the fire YOLO model. Returns True on success."""
        if self._fire_yolo_model is not None:
            return True
        if self._fire_yolo_load_attempted:
            # Already failed once — don't retry every frame.
            return False
        self._fire_yolo_load_attempted = True
        if self._yolo_cls is None:
            return False
        model_path = self.config.fire_yolo_model
        # Same lookup pattern as person YOLO: try INT8 → ONNX → .pt.
        candidates = []
        if model_path.endswith(".pt"):
            stem = model_path[:-3]
            candidates = [f"{stem}_int8.onnx", f"{stem}.onnx", model_path]
        elif model_path.endswith(".onnx"):
            stem = model_path[:-5]
            candidates = [f"{stem}_int8.onnx", model_path]
        else:
            candidates = [model_path]
        resolved = next((p for p in candidates if os.path.exists(p)), None)
        if resolved is None:
            log.warning(
                "[detector] fire YOLO model not found at any of: %s — "
                "run `python -m tools.download_fire_model` to fetch one",
                candidates,
            )
            return False
        try:
            log.info("[detector] loading fire YOLO: %s", resolved)
            self._fire_yolo_model = self._yolo_cls(resolved)
            return True
        except Exception as exc:
            log.error("[detector] fire YOLO load failed (%s): %s", resolved, exc)
            return False

    def _detect_fire_yolo(self, frame):
        """Run the fire YOLO model and return the best detection.

        Returns (bbox=(x,y,w,h), confidence) where confidence is the YOLO
        score normalised to [0, 1]. Returns (None, 0.0) when no class
        crosses `fire_yolo_confidence`. Smoke detections fire the same
        "fire" event kind so downstream consumers (actuator, MQTT alert,
        PWA) don't have to learn a new label.
        """
        c = self.config
        try:
            results = self._fire_yolo_model.predict(
                frame,
                conf=c.fire_yolo_confidence,
                imgsz=c.fire_yolo_imgsz,
                verbose=False,
            )
        except Exception as exc:
            log.warning("[detector] fire YOLO inference error: %s", exc)
            return None, 0.0

        target_classes = {c.fire_yolo_fire_class}
        if c.fire_yolo_smoke_class >= 0:
            target_classes.add(c.fire_yolo_smoke_class)

        best_conf = 0.0
        best_bbox: Optional[tuple] = None
        for r in results:
            boxes = getattr(r, "boxes", None)
            if boxes is None:
                continue
            for b in boxes:
                cls = int(b.cls[0])
                if cls not in target_classes:
                    continue
                conf = float(b.conf[0])
                if conf <= best_conf:
                    continue
                xyxy = b.xyxy[0].tolist()
                x1, y1, x2, y2 = map(int, xyxy)
                best_conf = conf
                best_bbox = (x1, y1, x2 - x1, y2 - y1)
        if best_bbox is None:
            return None, 0.0
        return best_bbox, best_conf

    def _detect_fire_hsv(self, frame):
        """V10.4 6-stage HSV pipeline — fallback when YOLO unavailable."""
        cv2 = self._cv2
        if cv2 is None:
            return None, 0.0
        c = self.config
        h, w = frame.shape[:2]
        total = h * w

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # ── Stage 1: outer flame mask ─────────────────────────────────────
        m_red1 = cv2.inRange(hsv, c.fire_hsv_low_red1, c.fire_hsv_high_red1)
        m_red2 = cv2.inRange(hsv, c.fire_hsv_low_red2, c.fire_hsv_high_red2)
        m_yel = cv2.inRange(hsv, c.fire_hsv_low_yellow, c.fire_hsv_high_yellow)
        mask_outer = cv2.bitwise_or(cv2.bitwise_or(m_red1, m_red2), m_yel)
        # V10.3: OPEN no longer applied here.
        outer_px = int(cv2.countNonZero(mask_outer))

        # ── Stage 1b: skin discriminator (V10.4) ──────────────────────────
        # Real flame has a yellow-hue centre; skin under warm light is
        # uniformly orange-pink in the wrap-red band with no yellow core.
        # We require flame_yellow_px >= ratio × wrap_red_px. Hand at arm's
        # length lights up wrap_red only → ratio → 0 → rejected here
        # before the more expensive stages run.
        wrap_red_px = int(cv2.countNonZero(cv2.bitwise_or(m_red1, m_red2)))
        yellow_px = int(cv2.countNonZero(m_yel))
        if wrap_red_px > 0:
            # Only enforce the ratio when there's meaningful red — pure
            # candle flames (almost all yellow, zero red) shouldn't trip
            # this gate.
            yr_ratio = yellow_px / float(wrap_red_px)
            if yr_ratio < c.fire_min_yellow_over_red_ratio:
                self._push_fire_history(None, h, w)
                self._log_fire_reject(
                    "STAGE1B_skin",
                    outer_px=outer_px, yellow_px=yellow_px,
                    wrap_red_px=wrap_red_px, ratio=round(yr_ratio, 2),
                )
                return None, 0.0

        # ── Stage 2: hot-core requirement ────────────────────────────────
        # H range left wide (0-180) on purpose — core is defined by
        # brightness + saturation, not hue.
        # V10.4 added S_min floor (S ≥ s_min) — kicks out pure-white
        # specular highlights and very bright skin which carry S < 30.
        mask_core = cv2.inRange(
            hsv,
            (0, c.fire_hot_core_s_min, c.fire_hot_core_v_min),
            (180, c.fire_hot_core_s_max, 255),
        )
        core_px = int(cv2.countNonZero(mask_core))
        if core_px < c.fire_hot_core_min_pixels:
            self._push_fire_history(None, h, w)
            self._log_fire_reject("STAGE2_no_core", outer_px=outer_px, core_px=core_px)
            return None, 0.0

        # ── Stage 3: outer must be spatially adjacent to core ────────────
        # Dilating the core by ~8 px and AND-ing with outer keeps only the
        # red/yellow pixels that wrap the bright center. A red shirt with
        # no bright spot fails; a white LED with no warm halo fails.
        core_dilated = cv2.dilate(
            mask_core,
            cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)),
            iterations=c.fire_core_dilate_iter,
        )
        mask_fire = cv2.bitwise_and(mask_outer, core_dilated)
        # Knit the flame back together after dilation morphology splits it.
        mask_fire = cv2.morphologyEx(
            mask_fire, cv2.MORPH_CLOSE, None, iterations=2
        )

        # ── Stage 4: area gate ───────────────────────────────────────────
        fire_px = int(cv2.countNonZero(mask_fire))
        ratio = fire_px / total
        if ratio < c.fire_min_area_ratio or ratio > c.fire_max_area_ratio:
            self._push_fire_history(mask_fire if fire_px else None, h, w)
            self._log_fire_reject(
                "STAGE4_area",
                outer_px=outer_px, core_px=core_px,
                fire_px=fire_px, ratio=ratio,
                min_ratio=c.fire_min_area_ratio,
                max_ratio=c.fire_max_area_ratio,
            )
            return None, ratio

        # ── Stage 5: temporal flicker check ──────────────────────────────
        # We push the current mask AFTER the check so the history reflects
        # observed flame frames only — a static red blob doesn't reset
        # back to "no fire" history on every rejection.
        motion_ok = self._fire_motion_ok(mask_fire)
        self._push_fire_history(mask_fire, h, w)
        if not motion_ok:
            self._log_fire_reject("STAGE5_static",
                                   core_px=core_px, fire_px=fire_px, ratio=ratio)
            return None, ratio

        # ── Stage 6: shape filter ────────────────────────────────────────
        contours, _ = cv2.findContours(
            mask_fire, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None, ratio
        biggest = max(contours, key=cv2.contourArea)
        x, y, bw, bh = cv2.boundingRect(biggest)
        if bw < 4 or bh < 4:
            self._log_fire_reject("STAGE6_tiny", bw=bw, bh=bh)
            return None, ratio
        aspect = bh / max(1, bw)
        if aspect < c.fire_aspect_ratio_min:
            self._log_fire_reject(
                "STAGE6_horizontal",
                bw=bw, bh=bh, aspect=round(aspect, 2),
                aspect_min=c.fire_aspect_ratio_min,
            )
            return None, ratio

        return (x, y, bw, bh), ratio

    def _log_fire_reject(self, stage: str, **fields) -> None:
        """Diagnostic log when a candidate is rejected by a fire stage.

        Throttled to one line per `fire_reject_log_every_n` frames so it
        doesn't drown the logs at 10 FPS. Operator can read these to know
        WHICH stage is dropping their lighter at the demo venue and tune
        the matching threshold without guessing. Defaults to every 30
        frames (~3 s) so it stays informative but quiet.
        """
        self._fire_reject_tick = getattr(self, "_fire_reject_tick", 0) + 1
        every = max(1, int(getattr(self.config, "fire_reject_log_every_n", 30)))
        if self._fire_reject_tick % every != 0:
            return
        details = " ".join(f"{k}={v}" for k, v in fields.items())
        log.info("[detector] fire reject %s %s", stage, details)

    def _fire_motion_ok(self, mask_now) -> bool:
        """True if mask differs enough from recent history.

        Real flame deforms each frame — IoU vs the previous mask drops
        well below 0.92. A static red poster produces near-perfect IoU.
        We require at least `fire_min_motion_frames` of the buffered
        masks to show motion vs the current one. An empty buffer (first
        valid frame) returns True; the upstream TemporalSmoother still
        enforces N-of-M frames before emitting an event, so we don't
        let single-frame flashes through.
        """
        cv2 = self._cv2
        if cv2 is None or not self._fire_mask_history:
            return True
        motion_count = 0
        seen = 0
        max_iou = float(self.config.fire_max_static_iou)
        for prev in self._fire_mask_history:
            if prev is None or prev.shape != mask_now.shape:
                continue
            seen += 1
            inter = int(cv2.countNonZero(cv2.bitwise_and(prev, mask_now)))
            union = int(cv2.countNonZero(cv2.bitwise_or(prev, mask_now)))
            if union == 0:
                continue
            if (inter / union) < max_iou:
                motion_count += 1
        if seen == 0:
            return True
        return motion_count >= int(self.config.fire_min_motion_frames)

    def _push_fire_history(self, mask, h: int, w: int) -> None:
        """Append a frame's fire mask to the rolling history.

        Passing `mask=None` records a "no fire seen" slot — we still
        advance the window so flicker comparisons against a returning
        flame use recent context, not stale frames from minutes ago.
        We can't allocate numpy zeros without numpy imported, but we
        also don't need to: None entries are simply skipped by the
        comparison loop in `_fire_motion_ok`.
        """
        if mask is None:
            self._fire_mask_history.append(None)
        else:
            self._fire_mask_history.append(mask.copy())

    # ------------------------------------------------------------------
    # Emit event + snapshot
    # ------------------------------------------------------------------

    def _emit_event(self, kind, confidence, bbox, frame, ts):
        snapshot_rel, annotated = self._save_snapshot(frame, bbox, kind, ts)
        h, w = frame.shape[:2] if frame is not None else (0, 0)
        snapshot_b64 = self._encode_snapshot_b64(annotated)
        event = DetectionEvent(
            kind=kind,
            confidence=float(confidence),
            bbox=bbox,
            timestamp=ts,
            snapshot_path=snapshot_rel,
            frame_width=w,
            frame_height=h,
            snapshot_b64=snapshot_b64,
        )
        log.info(
            "[detector] %s conf=%.2f bbox=%s -> %s (b64=%d B)",
            kind, confidence, bbox, snapshot_rel, len(snapshot_b64),
        )
        if self.on_event:
            try:
                self.on_event(event)
            except Exception as exc:
                log.exception("[detector] on_event callback error: %s", exc)

    def _save_snapshot(self, frame, bbox, kind, ts):
        """Write annotated JPEG to disk and return (path, annotated_frame).

        The annotated frame is reused by `_encode_snapshot_b64` so we don't
        re-draw the bbox twice. Returns (path, None) in dry-run mode.
        """
        if frame is None or self._cv2 is None:
            # dry_run: synthesize a path
            path = Path(self.config.snapshot_dir) / f"{int(ts)}_{kind}.jpg"
            path.write_bytes(b"")  # empty placeholder
            return str(path), None

        cv2 = self._cv2
        annotated = frame.copy()
        x, y, bw, bh = bbox
        color = (0, 0, 255) if kind == "fire" else (0, 255, 0)
        cv2.rectangle(annotated, (x, y), (x + bw, y + bh), color, 2)
        label = f"{kind.upper()}"
        cv2.putText(annotated, label, (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

        fname = f"{int(ts)}_{kind}.jpg"
        path = Path(self.config.snapshot_dir) / fname
        cv2.imwrite(
            str(path),
            annotated,
            [cv2.IMWRITE_JPEG_QUALITY, self.config.snapshot_jpeg_quality],
        )
        self._rotate_snapshots()
        return str(path), annotated

    def _encode_snapshot_b64(self, annotated) -> str:
        """Resize + JPEG-encode + base64 the annotated frame for inline MQTT.

        Returns "" in dry-run / when cv2 is unavailable / when disabled via
        snapshot_b64_max_dim <= 0. Aspect ratio is preserved; the longest
        side is clamped to snapshot_b64_max_dim.
        """
        if annotated is None or self._cv2 is None:
            return ""
        max_dim = int(self.config.snapshot_b64_max_dim)
        if max_dim <= 0:
            return ""
        cv2 = self._cv2
        try:
            h, w = annotated.shape[:2]
            longest = max(h, w)
            if longest > max_dim:
                scale = max_dim / float(longest)
                new_w = max(1, int(round(w * scale)))
                new_h = max(1, int(round(h * scale)))
                small = cv2.resize(annotated, (new_w, new_h),
                                    interpolation=cv2.INTER_AREA)
            else:
                small = annotated
            ok, buf = cv2.imencode(
                ".jpg",
                small,
                [cv2.IMWRITE_JPEG_QUALITY,
                 int(self.config.snapshot_b64_jpeg_quality)],
            )
            if not ok:
                return ""
            return base64.b64encode(buf.tobytes()).decode("ascii")
        except Exception as exc:
            log.warning("[detector] snapshot b64 encode failed: %s", exc)
            return ""

    def _rotate_snapshots(self) -> None:
        """Keep the snapshot dir below `snapshot_max_keep` files (oldest first)."""
        d = Path(self.config.snapshot_dir)
        files = sorted(d.glob("*.jpg"), key=lambda p: p.stat().st_mtime)
        excess = len(files) - self.config.snapshot_max_keep
        for f in files[:max(0, excess)]:
            try:
                f.unlink()
            except OSError:
                pass

    # ------------------------------------------------------------------
    # Dry run (dev without camera)
    # ------------------------------------------------------------------

    def _dry_run_loop(self) -> None:
        """Synthesize one person event every 15s and one fire event every 40s."""
        log.info("[detector] dry_run mode — synthesizing events")
        tick = 0
        while self._running:
            time.sleep(5)
            tick += 1
            now = time.time()
            if tick % 3 == 0:
                self._emit_event("person", 0.92, (120, 80, 140, 260), None, now)
            if tick % 8 == 0:
                self._emit_event("fire", 0.04, (400, 260, 80, 120), None, now)


# ---------------------------------------------------------------------------
# CLI entry (for testing)
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    import argparse

    ap = argparse.ArgumentParser(description="K-Patrol anomaly detector")
    ap.add_argument("--dry-run", action="store_true", help="no camera, synthesize events")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--model", type=str, default="yolov8n.pt")
    args = ap.parse_args()

    cfg = DetectionConfig(
        dry_run=args.dry_run,
        camera_index=args.camera,
        yolo_model=args.model,
    )

    def _print(ev: DetectionEvent):
        print(ev.to_json())

    AnomalyDetector(cfg, on_event=_print).start(blocking=True)
