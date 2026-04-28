"""
K-Patrol Anomaly Detector
--------------------------

Edge-AI person + fire detection running on Raspberry Pi 4 Model B.

- YOLOv8n (ultralytics) for person detection (COCO class 0).
- Lightweight fire/smoke detector via HSV color segmentation + motion heuristic
  (cheaper than a fire-trained YOLO model and runs in realtime on Pi).
- Publishes events to MQTT topic `kpatrol/{serial}/alert` and saves snapshots
  under ./snapshots/{ts}.jpg.

Designed to run as a sidecar process to the main controller. Imports are lazy
so the module can be imported on a dev laptop without cv2/ultralytics.

Usage:
    detector = AnomalyDetector(config)
    detector.start()   # blocking loop
"""

from __future__ import annotations

import json
import logging
import os
import threading
import time
from dataclasses import dataclass, field, asdict
from pathlib import Path
from queue import Queue, Empty
from typing import Callable, Optional

from .temporal_smoother import TemporalSmoother

log = logging.getLogger("kpatrol.detection")


# ---------------------------------------------------------------------------
# Config
# ---------------------------------------------------------------------------

@dataclass
class DetectionConfig:
    """Runtime configuration for the detector."""

    # Camera
    camera_index: int = 0
    frame_width: int = 640
    frame_height: int = 480
    fps: int = 10

    # YOLO
    yolo_model: str = "yolov8n.pt"
    yolo_confidence: float = 0.55
    yolo_imgsz: int = 320  # smaller = faster on Pi

    # Person detection
    person_enabled: bool = True
    person_cooldown_sec: float = 8.0     # min gap between person alerts
    person_min_area_ratio: float = 0.02  # bbox must cover >=2% of frame

    # Fire detection (HSV-based)
    fire_enabled: bool = True
    fire_cooldown_sec: float = 5.0
    fire_min_area_ratio: float = 0.015
    # HSV ranges for flame: low-red and high-red
    fire_hsv_low_red1: tuple = (0, 120, 180)
    fire_hsv_high_red1: tuple = (15, 255, 255)
    fire_hsv_low_red2: tuple = (160, 120, 180)
    fire_hsv_high_red2: tuple = (179, 255, 255)

    # Snapshots
    snapshot_dir: str = "snapshots"
    snapshot_jpeg_quality: int = 85
    snapshot_max_keep: int = 500  # rolling buffer

    # MQTT topic (prefix only, serial appended at runtime)
    mqtt_topic_prefix: str = "kpatrol"
    robot_serial: str = "KPATROL-001"

    # Temporal smoothing (N-of-M consensus before emitting an event).
    # Suppresses single-frame flickers from YOLO/HSV noise.
    smoothing_window: int = 5
    smoothing_required: int = 3

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

        self._running = False
        self._frame_queue: Queue = Queue(maxsize=2)
        self._threads: list[threading.Thread] = []

        self._last_person_ts = 0.0
        self._last_fire_ts = 0.0

        self._smoother = TemporalSmoother(
            window=self.config.smoothing_window,
            required=self.config.smoothing_required,
            class_kinds=("person", "fire"),
        )
        # Hold the most recent raw bbox per kind so that when the smoother
        # finally fires, we still have a spatial reference for the snapshot.
        self._last_bbox: dict[str, tuple] = {}

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

        cap = cv2.VideoCapture(self.config.camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.frame_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.frame_height)
        cap.set(cv2.CAP_PROP_FPS, self.config.fps)

        interval = 1.0 / max(1, self.config.fps)
        while self._running:
            start = time.perf_counter()
            ok, frame = cap.read()
            if not ok:
                time.sleep(0.1)
                continue
            # drop oldest if queue full — always keep most recent
            if self._frame_queue.full():
                try:
                    self._frame_queue.get_nowait()
                except Empty:
                    pass
            self._frame_queue.put(frame)
            elapsed = time.perf_counter() - start
            time.sleep(max(0.0, interval - elapsed))

        cap.release()

    # ------------------------------------------------------------------
    # Inference loop
    # ------------------------------------------------------------------

    def _inference_loop(self) -> None:
        while self._running:
            try:
                frame = self._frame_queue.get(timeout=0.5)
            except Empty:
                continue

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
            log.info("[detector] loading YOLO model: %s", self.config.yolo_model)
            self._yolo_model = self._yolo_cls(self.config.yolo_model)
            return True
        except Exception as exc:
            log.error("[detector] failed to load YOLO: %s", exc)
            return False

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
        """HSV-based flame segmentation. Returns (bbox, area_ratio) or (None, 0)."""
        cv2 = self._cv2
        if cv2 is None:
            return None, 0.0

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        c = self.config
        mask1 = cv2.inRange(hsv, c.fire_hsv_low_red1, c.fire_hsv_high_red1)
        mask2 = cv2.inRange(hsv, c.fire_hsv_low_red2, c.fire_hsv_high_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None, iterations=1)

        h, w = frame.shape[:2]
        total = h * w
        fire_px = int(cv2.countNonZero(mask))
        ratio = fire_px / total
        if ratio < c.fire_min_area_ratio:
            return None, ratio

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None, ratio
        biggest = max(contours, key=cv2.contourArea)
        x, y, bw, bh = cv2.boundingRect(biggest)
        return (x, y, bw, bh), ratio

    # ------------------------------------------------------------------
    # Emit event + snapshot
    # ------------------------------------------------------------------

    def _emit_event(self, kind, confidence, bbox, frame, ts):
        snapshot_rel = self._save_snapshot(frame, bbox, kind, ts)
        h, w = frame.shape[:2] if frame is not None else (0, 0)
        event = DetectionEvent(
            kind=kind,
            confidence=float(confidence),
            bbox=bbox,
            timestamp=ts,
            snapshot_path=snapshot_rel,
            frame_width=w,
            frame_height=h,
        )
        log.info(
            "[detector] %s conf=%.2f bbox=%s -> %s",
            kind, confidence, bbox, snapshot_rel,
        )
        if self.on_event:
            try:
                self.on_event(event)
            except Exception as exc:
                log.exception("[detector] on_event callback error: %s", exc)

    def _save_snapshot(self, frame, bbox, kind, ts) -> str:
        if frame is None or self._cv2 is None:
            # dry_run: synthesize a path
            path = Path(self.config.snapshot_dir) / f"{int(ts)}_{kind}.jpg"
            path.write_bytes(b"")  # empty placeholder
            return str(path)

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
        return str(path)

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
