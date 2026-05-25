"""Realtime fire-detector calibration UI.

Open a webcam (or any V4L2 / URL source cv2 understands), run the V10.2
6-stage pipeline live, and let the operator tune every threshold via
trackbars while watching the masks update.

The goal is a 2-minute calibration session at the demo venue:
   1. Hold a lighter in front of the camera → confirm "FIRE" overlay appears.
   2. Hold a red shirt / red sign → confirm overlay stays "no fire".
   3. Adjust V_min / S_max / min_area / max_iou until both conditions hold.
   4. Hit `s` to dump the current values as a Python snippet you can paste
      into DetectionConfig.

Usage:
    python -m tools.calibrate_fire                        # default webcam 0
    python -m tools.calibrate_fire --camera 2             # USB cam idx 2
    python -m tools.calibrate_fire --source http://...    # MJPEG sidecar
    python -m tools.calibrate_fire --image lighter.jpg    # static image
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

try:
    import cv2  # type: ignore
    import numpy as np
except ImportError:
    print("[calibrate_fire] requires cv2 + numpy. Install on the Pi venv first.")
    sys.exit(1)

# Make the detection package importable when running as a script.
THIS = Path(__file__).resolve()
sys.path.insert(0, str(THIS.parent.parent))

from detection.anomaly_detector import AnomalyDetector, DetectionConfig


# ─── Trackbar wiring ────────────────────────────────────────────────────


WINDOW = "K-Patrol Fire Calibrate"
WINDOW_MASK = "Masks (outer | core | combined)"


def _make_trackbars(cfg: DetectionConfig) -> None:
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, 1280, 720)
    cv2.namedWindow(WINDOW_MASK, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_MASK, 1280, 240)

    def _noop(_: int) -> None:
        return None

    # Each trackbar carries a label that the redraw loop reads back via
    # cv2.getTrackbarPos. We don't bind on-change callbacks because cv2's
    # trackbar callbacks fire on the GUI thread and we'd have to mutex
    # against the capture loop.
    cv2.createTrackbar("OuterRed S_min",    WINDOW, cfg.fire_hsv_low_red1[1], 255, _noop)
    cv2.createTrackbar("OuterRed V_min",    WINDOW, cfg.fire_hsv_low_red1[2], 255, _noop)
    cv2.createTrackbar("OuterYel H_low",    WINDOW, cfg.fire_hsv_low_yellow[0], 60,  _noop)
    cv2.createTrackbar("OuterYel H_high",   WINDOW, cfg.fire_hsv_high_yellow[0], 60, _noop)
    cv2.createTrackbar("Core V_min",        WINDOW, cfg.fire_hot_core_v_min, 255, _noop)
    cv2.createTrackbar("Core S_max",        WINDOW, cfg.fire_hot_core_s_max, 255, _noop)
    cv2.createTrackbar("Core dilate",       WINDOW, cfg.fire_core_dilate_iter, 10,  _noop)
    cv2.createTrackbar("MinArea (x10000)",  WINDOW, int(cfg.fire_min_area_ratio * 10000), 200, _noop)
    cv2.createTrackbar("MaxArea (%)",       WINDOW, int(cfg.fire_max_area_ratio * 100), 100, _noop)
    cv2.createTrackbar("Aspect (x10)",      WINDOW, int(cfg.fire_aspect_ratio_min * 10), 30, _noop)
    cv2.createTrackbar("MaxIoU (x100)",     WINDOW, int(cfg.fire_max_static_iou * 100), 100, _noop)
    cv2.createTrackbar("MotionFrames",      WINDOW, cfg.fire_min_motion_frames, 5, _noop)


def _read_trackbars(cfg: DetectionConfig) -> DetectionConfig:
    """Pull current trackbar values into a fresh config (preserve outer H wraps)."""
    s_min   = cv2.getTrackbarPos("OuterRed S_min", WINDOW)
    v_min   = cv2.getTrackbarPos("OuterRed V_min", WINDOW)
    yh_low  = cv2.getTrackbarPos("OuterYel H_low", WINDOW)
    yh_high = max(yh_low + 1, cv2.getTrackbarPos("OuterYel H_high", WINDOW))

    cfg.fire_hsv_low_red1   = (0, s_min, v_min)
    cfg.fire_hsv_high_red1  = (15, 255, 255)
    cfg.fire_hsv_low_red2   = (160, s_min, v_min)
    cfg.fire_hsv_high_red2  = (179, 255, 255)
    cfg.fire_hsv_low_yellow = (yh_low, s_min, v_min)
    cfg.fire_hsv_high_yellow = (yh_high, 255, 255)

    cfg.fire_hot_core_v_min = cv2.getTrackbarPos("Core V_min", WINDOW)
    cfg.fire_hot_core_s_max = cv2.getTrackbarPos("Core S_max", WINDOW)
    cfg.fire_core_dilate_iter = max(1, cv2.getTrackbarPos("Core dilate", WINDOW))

    cfg.fire_min_area_ratio = max(0.0001, cv2.getTrackbarPos("MinArea (x10000)", WINDOW) / 10000.0)
    cfg.fire_max_area_ratio = max(cfg.fire_min_area_ratio + 0.01,
                                  cv2.getTrackbarPos("MaxArea (%)", WINDOW) / 100.0)
    cfg.fire_aspect_ratio_min = max(0.0, cv2.getTrackbarPos("Aspect (x10)", WINDOW) / 10.0)
    cfg.fire_max_static_iou = max(0.0, cv2.getTrackbarPos("MaxIoU (x100)", WINDOW) / 100.0)
    cfg.fire_min_motion_frames = max(0, cv2.getTrackbarPos("MotionFrames", WINDOW))
    return cfg


def _dump_python_snippet(cfg: DetectionConfig) -> str:
    """Print the current trackbar values as a paste-ready Python block."""
    lines = [
        "# Paste these into DetectionConfig defaults — calibrated for current venue:",
        f"    fire_hsv_low_red1: tuple = {cfg.fire_hsv_low_red1}",
        f"    fire_hsv_high_red1: tuple = {cfg.fire_hsv_high_red1}",
        f"    fire_hsv_low_red2: tuple = {cfg.fire_hsv_low_red2}",
        f"    fire_hsv_high_red2: tuple = {cfg.fire_hsv_high_red2}",
        f"    fire_hsv_low_yellow: tuple = {cfg.fire_hsv_low_yellow}",
        f"    fire_hsv_high_yellow: tuple = {cfg.fire_hsv_high_yellow}",
        f"    fire_hot_core_v_min: int = {cfg.fire_hot_core_v_min}",
        f"    fire_hot_core_s_max: int = {cfg.fire_hot_core_s_max}",
        f"    fire_core_dilate_iter: int = {cfg.fire_core_dilate_iter}",
        f"    fire_min_area_ratio: float = {cfg.fire_min_area_ratio:.4f}",
        f"    fire_max_area_ratio: float = {cfg.fire_max_area_ratio:.2f}",
        f"    fire_aspect_ratio_min: float = {cfg.fire_aspect_ratio_min:.2f}",
        f"    fire_max_static_iou: float = {cfg.fire_max_static_iou:.2f}",
        f"    fire_min_motion_frames: int = {cfg.fire_min_motion_frames}",
    ]
    return "\n".join(lines)


# ─── Per-frame analysis (mirrors AnomalyDetector._detect_fire internals) ──


def _analyze_frame(detector: AnomalyDetector, frame):
    """Re-run the pipeline manually to extract stage-by-stage masks.

    We can't piggy-back on AnomalyDetector._detect_fire directly because
    that only returns the final bbox. For visual debugging we want every
    intermediate mask so the operator can SEE which stage is gating.
    """
    cv2_ = detector._cv2
    cfg = detector.config
    h, w = frame.shape[:2]
    hsv = cv2_.cvtColor(frame, cv2_.COLOR_BGR2HSV)

    # Stage 1
    m_red1 = cv2_.inRange(hsv, cfg.fire_hsv_low_red1, cfg.fire_hsv_high_red1)
    m_red2 = cv2_.inRange(hsv, cfg.fire_hsv_low_red2, cfg.fire_hsv_high_red2)
    m_yel  = cv2_.inRange(hsv, cfg.fire_hsv_low_yellow, cfg.fire_hsv_high_yellow)
    mask_outer = cv2_.bitwise_or(cv2_.bitwise_or(m_red1, m_red2), m_yel)
    mask_outer = cv2_.morphologyEx(mask_outer, cv2_.MORPH_OPEN, None, iterations=1)

    # Stage 2
    mask_core = cv2_.inRange(
        hsv, (0, 0, cfg.fire_hot_core_v_min),
             (180, cfg.fire_hot_core_s_max, 255)
    )

    # Stage 3
    core_dilated = cv2_.dilate(
        mask_core,
        cv2_.getStructuringElement(cv2_.MORPH_RECT, (3, 3)),
        iterations=cfg.fire_core_dilate_iter,
    )
    mask_fire = cv2_.bitwise_and(mask_outer, core_dilated)
    mask_fire = cv2_.morphologyEx(mask_fire, cv2_.MORPH_CLOSE, None, iterations=2)

    fire_px = int(cv2_.countNonZero(mask_fire))
    ratio = fire_px / float(h * w)

    # Run the actual detector so flicker history advances naturally.
    bbox, det_ratio = detector._detect_fire(frame)

    return {
        "mask_outer": mask_outer,
        "mask_core": mask_core,
        "mask_fire": mask_fire,
        "ratio": ratio,
        "det_ratio": det_ratio,
        "bbox": bbox,
    }


def _render_overlay(frame, info: dict, fps: float):
    out = frame.copy()
    h, w = out.shape[:2]

    # Tint mask_fire onto frame in red so it's obvious what's flagged.
    overlay = out.copy()
    overlay[info["mask_fire"] > 0] = (0, 0, 255)
    out = cv2.addWeighted(overlay, 0.35, out, 0.65, 0)

    bbox = info["bbox"]
    if bbox is not None:
        x, y, bw, bh = bbox
        cv2.rectangle(out, (x, y), (x + bw, y + bh), (0, 0, 255), 2)
        cv2.putText(out, "FIRE", (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

    # Top status bar
    cv2.rectangle(out, (0, 0), (w, 34), (0, 0, 0), -1)
    state = "FIRE!" if bbox is not None else "no fire"
    color = (0, 0, 255) if bbox is not None else (180, 180, 180)
    cv2.putText(
        out,
        f"[{state}] ratio={info['ratio']:.4f}   fps={fps:.1f}   "
        "[ s ]=dump  [ q ]=quit  [ r ]=reset history",
        (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA,
    )
    return out


def _stack_masks(info: dict):
    """Build a horizontal triptych: outer | core | combined."""
    def _gray3(m):
        return cv2.cvtColor(m, cv2.COLOR_GRAY2BGR)
    pad = lambda img, label: cv2.putText(
        img.copy(), label, (8, 22),
        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 1, cv2.LINE_AA,
    )
    a = pad(_gray3(info["mask_outer"]), "outer")
    b = pad(_gray3(info["mask_core"]), "core (V/S gate)")
    c = pad(_gray3(info["mask_fire"]), "fire = outer & dilate(core)")
    return np.hstack([a, b, c])


# ─── Main loop ──────────────────────────────────────────────────────────


def main():
    ap = argparse.ArgumentParser(description="K-Patrol fire calibration UI")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--camera", type=int, default=None,
                   help="V4L2 device index (default: 0 if no --source/--image)")
    g.add_argument("--source", type=str, default=None,
                   help="Stream URL or video file path")
    g.add_argument("--image", type=str, default=None,
                   help="Single image — replay it indefinitely for static tuning")
    ap.add_argument("--width", type=int, default=640)
    ap.add_argument("--height", type=int, default=480)
    args = ap.parse_args()

    cfg = DetectionConfig(dry_run=False)
    detector = AnomalyDetector(cfg)
    _make_trackbars(cfg)

    static_image = None
    cap = None
    if args.image:
        static_image = cv2.imread(args.image)
        if static_image is None:
            print(f"[calibrate_fire] could not read {args.image}")
            sys.exit(2)
    else:
        src = args.source if args.source else (args.camera if args.camera is not None else 0)
        cap = cv2.VideoCapture(src)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
        if not cap.isOpened():
            print(f"[calibrate_fire] could not open source {src}")
            sys.exit(2)

    last_ts = time.perf_counter()
    fps = 0.0

    try:
        while True:
            if static_image is not None:
                frame = static_image.copy()
            else:
                ok, frame = cap.read()
                if not ok:
                    print("[calibrate_fire] frame read failed; retrying…")
                    time.sleep(0.1)
                    continue

            _read_trackbars(cfg)
            info = _analyze_frame(detector, frame)

            now = time.perf_counter()
            dt = now - last_ts
            last_ts = now
            if dt > 0:
                fps = 0.9 * fps + 0.1 * (1.0 / dt)

            cv2.imshow(WINDOW, _render_overlay(frame, info, fps))
            cv2.imshow(WINDOW_MASK, _stack_masks(info))

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord("s"):
                snippet = _dump_python_snippet(cfg)
                print("\n" + snippet + "\n", flush=True)
            elif key == ord("r"):
                detector._fire_mask_history.clear()
                print("[calibrate_fire] flicker history cleared")

    finally:
        if cap is not None:
            cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
