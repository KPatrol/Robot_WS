"""Live test harness for the fire YOLO model.

Opens the webcam (or an image file), runs `AnomalyDetector._detect_fire`,
draws detection overlay, and prints latency + FPS. Lets the operator
verify the model catches genuine flame and rejects bright skin BEFORE
deploying to the robot.

Usage
=====
    # Webcam test (default device 0)
    python3 -m tools.test_fire_model

    # Specific camera index (e.g. USB cam at /dev/video2)
    python3 -m tools.test_fire_model --camera 2

    # Single image
    python3 -m tools.test_fire_model --image /path/to/test.jpg

    # Side-by-side comparison: YOLO vs HSV pipeline
    python3 -m tools.test_fire_model --compare

Keyboard
========
    q    quit
    s    snapshot to /tmp/fire_test_<ts>.jpg
    m    toggle pipeline (YOLO ↔ HSV) live
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

try:
    import cv2  # type: ignore
    import numpy as np
except ImportError:
    print("[test_fire_model] requires cv2 + numpy. Install on the venv first.")
    sys.exit(1)

THIS = Path(__file__).resolve()
sys.path.insert(0, str(THIS.parent.parent))

from detection.anomaly_detector import AnomalyDetector, DetectionConfig


def _build_detector(mode: str) -> AnomalyDetector:
    """Construct detector with the requested fire pipeline mode."""
    cfg = DetectionConfig(dry_run=False)
    cfg.fire_pipeline = mode
    det = AnomalyDetector(cfg)
    return det


def _render_frame(frame, bbox, conf, mode: str, fps: float):
    out = frame.copy()
    h, w = out.shape[:2]

    # Detection overlay
    if bbox is not None:
        x, y, bw, bh = bbox
        color = (0, 0, 255)  # red
        cv2.rectangle(out, (x, y), (x + bw, y + bh), color, 2)
        label = f"FIRE {conf*100:.0f}%" if conf else "FIRE"
        cv2.putText(out, label, (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)

    # Status bar
    cv2.rectangle(out, (0, 0), (w, 32), (0, 0, 0), -1)
    state = f"FIRE!" if bbox is not None else "no fire"
    color = (0, 0, 255) if bbox is not None else (180, 180, 180)
    cv2.putText(
        out,
        f"[{state}]  mode={mode}  fps={fps:.1f}  conf={conf:.2f}  "
        "[m]=toggle  [s]=snap  [q]=quit",
        (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA,
    )
    return out


def _compare_frame(frame, det_yolo, det_hsv):
    """Render YOLO on left half, HSV on right half — side-by-side."""
    h, w = frame.shape[:2]
    half = cv2.resize(frame, (w // 2, h // 2))

    bbox_y, _ = det_yolo._detect_fire(frame)
    bbox_h, _ = det_hsv._detect_fire(frame)

    left = half.copy()
    right = half.copy()

    if bbox_y is not None:
        x, y, bw, bh = [v // 2 for v in bbox_y]
        cv2.rectangle(left, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
        cv2.putText(left, "YOLO FIRE", (x, max(15, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
    if bbox_h is not None:
        x, y, bw, bh = [v // 2 for v in bbox_h]
        cv2.rectangle(right, (x, y), (x + bw, y + bh), (0, 255, 255), 2)
        cv2.putText(right, "HSV FIRE", (x, max(15, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # Labels
    cv2.rectangle(left, (0, 0), (w // 2, 28), (0, 0, 0), -1)
    cv2.putText(left, "YOLO model (V11.0)", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)
    cv2.rectangle(right, (0, 0), (w // 2, 28), (0, 0, 0), -1)
    cv2.putText(right, "HSV pipeline (V10.4)", (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

    return np.hstack([left, right])


def main():
    ap = argparse.ArgumentParser(description="Live fire model test")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--camera", type=int, default=None,
                   help="V4L2 device index (default 0)")
    g.add_argument("--image", type=str, default=None,
                   help="single image — replay indefinitely")
    g.add_argument("--source", type=str, default=None,
                   help="video file or stream URL")
    ap.add_argument("--compare", action="store_true",
                    help="side-by-side YOLO vs HSV")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    ap.add_argument("--mode", choices=["yolo", "hsv"], default="yolo",
                    help="initial pipeline (toggle live with [m])")
    args = ap.parse_args()

    if args.compare:
        det_yolo = _build_detector("yolo")
        det_hsv = _build_detector("hsv")
        # Force resolve so warning prints once each.
        det_yolo._resolve_fire_mode()
        det_hsv._resolve_fire_mode()
    else:
        det = _build_detector(args.mode)

    # Open source
    static = None
    cap = None
    if args.image:
        static = cv2.imread(args.image)
        if static is None:
            print(f"Could not read {args.image}")
            sys.exit(2)
    else:
        src = args.source or (args.camera if args.camera is not None else 0)
        cap = cv2.VideoCapture(src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if not cap.isOpened():
            print(f"Could not open source {src}")
            sys.exit(2)

    win = "K-Patrol Fire Test"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1280, 720)

    fps = 0.0
    last = time.perf_counter()
    snap_count = 0

    try:
        while True:
            if static is not None:
                frame = static.copy()
            else:
                ok, frame = cap.read()
                if not ok:
                    time.sleep(0.05)
                    continue

            now = time.perf_counter()
            dt = now - last
            last = now
            if dt > 0:
                fps = 0.85 * fps + 0.15 * (1.0 / dt)

            if args.compare:
                vis = _compare_frame(frame, det_yolo, det_hsv)
                cv2.imshow(win, vis)
                state = "compare"
                bbox = None
                conf = 0.0
            else:
                bbox, conf = det._detect_fire(frame)
                mode = det._fire_mode_resolved or det.config.fire_pipeline
                cv2.imshow(win, _render_frame(frame, bbox, conf, mode, fps))
                state = mode

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                ts = int(time.time())
                path = f"/tmp/fire_test_{ts}.jpg"
                cv2.imwrite(path, frame)
                print(f"snap: {path}")
                snap_count += 1
            elif key == ord("m") and not args.compare:
                # Toggle pipeline live
                new_mode = "hsv" if det._fire_mode_resolved == "yolo" else "yolo"
                det.config.fire_pipeline = new_mode
                det._fire_mode_resolved = None  # force re-resolve
                det._fire_yolo_load_attempted = False
                print(f"toggle → {new_mode}")
    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()
        print(f"\nSnaps saved: {snap_count}")


if __name__ == "__main__":
    main()
