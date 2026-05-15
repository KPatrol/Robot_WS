"""
End-to-end smoke test for the detection pipeline.

Runs the real `AnomalyDetector._process_frame` against frames coming from a
laptop webcam, a still image, an image folder, or a video file — then forwards
events through `AlertBridge` to the VPS MQTT broker so the web UI shows the
snapshot immediately. The point is to validate the full Pi pipeline on a Mac
before the robot is back online.

Why a separate test driver?
- `AnomalyDetector.start()` spawns a capture thread that opens a cv2 camera.
  That works for webcam but not for video files or single images, so we drive
  `_process_frame` directly and bypass the threading completely.
- The temporal smoother needs N-of-M consecutive hits. A one-shot image won't
  trigger an event — so `--loops` replays each image N times to satisfy
  consensus.

Run examples:
    # Single image, publish to VPS broker so the web shows it
    python -m tools.detection_smoke_test --source path/to/person.jpg

    # Webcam preview, no MQTT — just print events to the console
    python -m tools.detection_smoke_test --source 0 --no-mqtt

    # Video file, throttle to 10 FPS like the Pi
    python -m tools.detection_smoke_test --source clip.mp4 --fps 10

    # Synthesize a red frame to verify the fire HSV path without hardware
    python -m tools.detection_smoke_test --source synth:fire --loops 6
"""

from __future__ import annotations

import argparse
import logging
import sys
import time
from pathlib import Path
from typing import Iterator, Optional

# Make `detection` importable when invoked as `python -m tools.smoke_test`
# from the pi-controller root.
REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import cv2
import numpy as np

from detection.alert_bridge import AlertBridge, load_mqtt_env
from detection.anomaly_detector import AnomalyDetector, DetectionConfig, DetectionEvent

log = logging.getLogger("kpatrol.smoke")

IMAGE_EXTS = {".jpg", ".jpeg", ".png", ".bmp", ".webp"}
VIDEO_EXTS = {".mp4", ".mov", ".avi", ".mkv", ".webm"}


def _frames_from_image(path: Path, loops: int) -> Iterator[np.ndarray]:
    frame = cv2.imread(str(path))
    if frame is None:
        raise SystemExit(f"cv2 could not read image: {path}")
    for _ in range(max(1, loops)):
        yield frame.copy()


def _frames_from_dir(path: Path, loops: int) -> Iterator[np.ndarray]:
    images = sorted(p for p in path.iterdir() if p.suffix.lower() in IMAGE_EXTS)
    if not images:
        raise SystemExit(f"no images found in folder: {path}")
    # Each image needs `loops` repeats so the smoother latches per image, not
    # only when one image happens to repeat across the rolling window.
    for img_path in images:
        frame = cv2.imread(str(img_path))
        if frame is None:
            log.warning("skipping unreadable image: %s", img_path)
            continue
        for _ in range(max(1, loops)):
            yield frame.copy()


def _frames_from_video(path: Path, max_frames: Optional[int]) -> Iterator[np.ndarray]:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise SystemExit(f"cv2 could not open video: {path}")
    try:
        sent = 0
        while True:
            ok, frame = cap.read()
            if not ok or frame is None:
                break
            yield frame
            sent += 1
            if max_frames is not None and sent >= max_frames:
                break
    finally:
        cap.release()


def _frames_from_webcam(index: int, max_frames: int) -> Iterator[np.ndarray]:
    cap = cv2.VideoCapture(index)
    if not cap.isOpened():
        raise SystemExit(f"cv2 could not open webcam index {index}")
    try:
        sent = 0
        while sent < max_frames:
            ok, frame = cap.read()
            if not ok or frame is None:
                log.warning("webcam read failed at frame %d", sent)
                time.sleep(0.05)
                continue
            yield frame
            sent += 1
    finally:
        cap.release()


def _synthetic_fire_frame() -> np.ndarray:
    """A 640×480 frame with a large bright-orange blob the HSV detector accepts."""
    frame = np.full((480, 640, 3), 30, dtype=np.uint8)  # dark grey background
    # Bright orange / red disc — covers ~12% of the frame.
    cv2.circle(frame, (320, 240), 140, (40, 110, 240), -1)  # BGR
    return frame


def _synthetic_person_frame() -> np.ndarray:
    """A frame with a person-shaped silhouette. NOT a guaranteed YOLO hit — YOLO
    needs a real person image — but useful to verify the pipeline doesn't crash
    on synthetic input."""
    frame = np.full((480, 640, 3), 200, dtype=np.uint8)
    cv2.rectangle(frame, (260, 80), (380, 420), (40, 40, 40), -1)
    cv2.circle(frame, (320, 110), 50, (60, 60, 60), -1)
    return frame


def _build_source(args) -> Iterator[np.ndarray]:
    src = args.source

    # Synthetic generators -----------------------------------------------------
    if src.startswith("synth:"):
        kind = src.split(":", 1)[1]
        if kind == "fire":
            base = _synthetic_fire_frame()
        elif kind == "person":
            base = _synthetic_person_frame()
        else:
            raise SystemExit(f"unknown synth source: {kind} (try fire|person)")
        for _ in range(max(1, args.loops)):
            yield base.copy()
        return

    # Webcam (integer source) --------------------------------------------------
    if src.isdigit():
        yield from _frames_from_webcam(int(src), args.max_frames or 200)
        return

    path = Path(src).expanduser()
    if not path.exists():
        raise SystemExit(f"source path does not exist: {path}")

    if path.is_dir():
        yield from _frames_from_dir(path, args.loops)
        return

    suffix = path.suffix.lower()
    if suffix in IMAGE_EXTS:
        yield from _frames_from_image(path, args.loops)
        return
    if suffix in VIDEO_EXTS:
        yield from _frames_from_video(path, args.max_frames)
        return

    raise SystemExit(f"unsupported source type: {path}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        "--source", required=True,
        help="image / image-folder / video path, webcam index (0/1/…), "
             "or 'synth:fire' / 'synth:person'",
    )
    ap.add_argument("--no-mqtt", action="store_true",
                    help="skip MQTT — print events to stdout only")
    ap.add_argument("--env", default="mqtt.env",
                    help="path to mqtt.env (default: mqtt.env in cwd)")
    ap.add_argument("--db", default="alerts_smoke.db",
                    help="SQLite WAL path (default: alerts_smoke.db so it doesn't "
                         "collide with the bridge's main store)")
    ap.add_argument("--robot",
                    help="override ROBOT_SERIAL from mqtt.env")
    ap.add_argument("--loops", type=int, default=6,
                    help="repeat each still image N times so the temporal "
                         "smoother (default 2/5 person, 3/5 fire) latches (default 6)")
    ap.add_argument("--fps", type=float, default=10.0,
                    help="throttle frame feed to this rate (default 10)")
    ap.add_argument("--max-frames", type=int, default=None,
                    help="cap total frames pulled from video/webcam")
    ap.add_argument(
        "--lower-thresholds", action="store_true",
        help="loosen the smoother to fire on a single positive frame — useful "
             "when you want one image to trigger one alert immediately",
    )
    ap.add_argument("--snapshots", default="snapshots_smoke",
                    help="snapshot output dir (default: snapshots_smoke)")
    ap.add_argument("--log-level", default="INFO")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=getattr(logging, args.log_level.upper(), logging.INFO),
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    # ------------------------------------------------------------------
    # MQTT bridge (or stub)
    # ------------------------------------------------------------------
    env = load_mqtt_env(args.env)
    if args.robot:
        env["ROBOT_SERIAL"] = args.robot

    bridge: Optional[AlertBridge] = None
    counts = {"person": 0, "fire": 0, "motion": 0, "other": 0}

    def _on_event(ev: DetectionEvent) -> None:
        counts[ev.kind if ev.kind in counts else "other"] += 1
        log.info(
            "EVENT kind=%s conf=%.3f bbox=%s snap_b64=%dB",
            ev.kind, ev.confidence, ev.bbox, len(ev.snapshot_b64),
        )
        if bridge is not None:
            bridge.publish(ev)

    if not args.no_mqtt:
        bridge = AlertBridge(env, dry_run=False, db_path=args.db)
        bridge.connect()
        bridge.start_drainer(interval_sec=2.0)
        log.info("MQTT bridge wired — topic=%s", bridge.topic)
    else:
        log.info("--no-mqtt set — events stay local")

    # ------------------------------------------------------------------
    # Detector — instantiated but NOT started. We drive _process_frame
    # directly, which lets us pull frames from anywhere we like.
    # ------------------------------------------------------------------
    cfg = DetectionConfig(
        dry_run=False,
        snapshot_dir=args.snapshots,
        robot_serial=env.get("ROBOT_SERIAL", "KPATROL-001"),
        # Allow rapid back-to-back alerts during testing — production cooldowns
        # would suppress most events in a short smoke run.
        person_cooldown_sec=0.5,
        fire_cooldown_sec=0.5,
    )
    if args.lower_thresholds:
        cfg.smoothing_window = 2
        cfg.person_smoothing_required = 1
        cfg.fire_smoothing_required = 1

    detector = AnomalyDetector(cfg, on_event=_on_event)
    # Force YOLO to load up front so the first-frame latency doesn't bleed into
    # the per-frame loop timing.
    if detector._ensure_yolo():
        log.info("YOLO model ready")
    else:
        log.warning("YOLO not available — person detection will be a no-op")

    # ------------------------------------------------------------------
    # Drive frames through the pipeline
    # ------------------------------------------------------------------
    frame_interval = 1.0 / max(1.0, args.fps)
    frame_count = 0
    started = time.time()
    try:
        for frame in _build_source(args):
            tick = time.perf_counter()
            try:
                detector._process_frame(frame)
            except Exception as exc:
                log.exception("process_frame raised: %s", exc)
            frame_count += 1
            elapsed = time.perf_counter() - tick
            sleep_for = frame_interval - elapsed
            if sleep_for > 0:
                time.sleep(sleep_for)
    except KeyboardInterrupt:
        log.info("interrupted")
    finally:
        # Give the drainer a moment to push the last events before we tear it
        # down — otherwise unsynced alerts only ride the next run's drain tick.
        if bridge is not None:
            time.sleep(2.5)
            bridge.disconnect()

    duration = time.time() - started
    log.info(
        "DONE frames=%d duration=%.1fs events=%s",
        frame_count, duration, counts,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
