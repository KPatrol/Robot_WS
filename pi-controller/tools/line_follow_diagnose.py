#!/usr/bin/env python3
"""
line_follow_diagnose.py — Standalone hardware sanity tool for the floor-line
follower. Lets you verify camera + HSV + BEV + spine extraction + PD output
on the actual Pi without running the full MQTT stack.

Why this exists:
    The line-follower has many failure modes that look identical from the
    outside ("the robot doesn't move"). This tool isolates each stage so
    we can find which one broke:
        Stage 1: camera open + frame capture (picamera2 / cv2)
        Stage 2: HSV mask quality (white tape coverage)
        Stage 3: BEV warp geometry (lane is centred + roughly vertical)
        Stage 4: spine extraction + polynomial fit
        Stage 5: PD output (lateral / heading errors → vx, vy, wz)
    Each stage saves an artefact under --out so the operator can inspect
    visually and adjust HSV / BEV homography before re-running.

Modes
-----
    capture   — single-frame capture + full pipeline tick (default)
    stream    — N-frame stream, prints per-frame metrics + writes last frame
    hsv-pick  — interactive: click pixel in raw frame → print its HSV
    bev-pick  — interactive: click 4 corners (TL,TR,BR,BL) of lane patch
                in raw frame → prints suggested bev_src_pts array

Usage
-----
    # Quickest check: does the camera open and the pipeline run?
    python3 tools/line_follow_diagnose.py --out /tmp/lf_diag

    # Stream 200 frames and report timing + line_found rate
    python3 tools/line_follow_diagnose.py stream --frames 200 --out /tmp/lf_diag

    # Calibrate HSV — click anywhere on the white line in the raw image
    python3 tools/line_follow_diagnose.py hsv-pick --out /tmp/lf_diag

    # Calibrate BEV homography — click 4 lane-patch corners
    python3 tools/line_follow_diagnose.py bev-pick --out /tmp/lf_diag

The tool exits non-zero if a stage fails, so it's safe to use in a
shell `&&` chain on the Pi to gate deployment.
"""
from __future__ import annotations

import argparse
import json
import logging
import os
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np

# Make the navigation package importable when run from anywhere
HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

try:
    import cv2
except ImportError:
    print("FATAL: opencv-python not installed. `pip install opencv-python`", file=sys.stderr)
    sys.exit(2)

from navigation import (  # type: ignore
    LineFollower, LineFollowerConfig, HSVRange, open_camera,
)

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger("lf_diag")


# ── Helpers ──────────────────────────────────────────────────────────────────

def _ensure_out(out: str) -> Path:
    p = Path(out)
    p.mkdir(parents=True, exist_ok=True)
    return p


def _open_cam(args) -> Optional[object]:
    """Open camera using the same factory the production loop uses, so we
    test the SAME code path. Threaded=False here — single-frame mode wants
    a guaranteed read (ThreadedFrameGrabber returns None on duplicate)."""
    cam = open_camera(
        index=args.cam_index,
        width=args.width, height=args.height, framerate=args.fps,
        prefer=args.backend or None,
        threaded=False,
    )
    if cam is None:
        logger.error("FATAL: no camera backend opened. "
                     "Check that picamera2 is installed (`apt install python3-picamera2`) "
                     "or that a USB camera is on /dev/video%d.", args.cam_index)
        return None
    logger.info("Camera ready via %s", cam.backend)
    return cam


def _grab(cam, warmup: int = 3) -> Optional[np.ndarray]:
    """Discard `warmup` frames so we don't capture a black auto-exposure
    frame, then return the next frame. Returns None on persistent failure."""
    for _ in range(warmup):
        cam.read()
        time.sleep(0.05)
    for _ in range(20):
        f = cam.read()
        if f is not None:
            return f
        time.sleep(0.05)
    return None


def _hsv_mask(frame: np.ndarray, hsv_cfg: HSVRange) -> np.ndarray:
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    return cv2.inRange(hsv, np.array(hsv_cfg.low, dtype=np.uint8),
                              np.array(hsv_cfg.high, dtype=np.uint8))


# ── Mode: capture (default — single full-pipeline tick) ──────────────────────

def cmd_capture(args) -> int:
    out = _ensure_out(args.out)
    cam = _open_cam(args)
    if cam is None:
        return 2
    try:
        if not cam.start():
            # open_camera already started; this is a re-arm safety
            pass
        frame = _grab(cam)
        if frame is None:
            logger.error("FATAL: captured zero frames. Camera opened but read() never returned data.")
            return 3
        h, w = frame.shape[:2]
        logger.info("Captured frame %dx%d, mean=%.1f", w, h, float(frame.mean()))
        cv2.imwrite(str(out / "01_raw.png"), frame)

        if float(frame.mean()) < 5.0:
            logger.warning("Raw frame is nearly all-black (mean<5). Likely lens cap on, "
                           "exposure too low, or driver returning empty buffer.")

        # Stage 2: HSV mask
        cfg = LineFollowerConfig()
        mask = _hsv_mask(frame, cfg.hsv)
        cv2.imwrite(str(out / "02_hsv_mask.png"), mask)
        coverage = float(mask.mean()) / 255.0
        logger.info("HSV mask coverage = %.2f%% (low=%s high=%s)",
                    coverage * 100, cfg.hsv.low, cfg.hsv.high)
        if coverage < 0.005:
            logger.warning("Mask coverage <0.5%% — HSV thresholds likely wrong for this lighting. "
                           "Run `hsv-pick` to recalibrate.")
        elif coverage > 0.40:
            logger.warning("Mask coverage >40%% — HSV is too permissive (whole floor is white). "
                           "Tighten S/V bounds.")

        # Stage 3: BEV warp
        M = cv2.getPerspectiveTransform(cfg.bev_src_pts, cfg.bev_dst_pts)
        bev = cv2.warpPerspective(frame, M, (cfg.bev_w, cfg.bev_h))
        bev_mask = cv2.warpPerspective(mask, M, (cfg.bev_w, cfg.bev_h))
        cv2.imwrite(str(out / "03_bev.png"), bev)
        cv2.imwrite(str(out / "04_bev_mask.png"), bev_mask)

        # Stage 4 + 5: full pipeline tick via LineFollower itself
        lf = LineFollower(cfg)
        result = lf.tick(frame, produce_overlay=True)
        if result.overlay is not None:
            cv2.imwrite(str(out / "05_overlay.png"), result.overlay)

        summary = {
            "frame": {"width": w, "height": h, "mean": float(frame.mean())},
            "hsv": {
                "low": list(cfg.hsv.low), "high": list(cfg.hsv.high),
                "mask_coverage_pct": coverage * 100,
            },
            "bev": {
                "src_pts": cfg.bev_src_pts.tolist(),
                "dst_pts": cfg.bev_dst_pts.tolist(),
                "size": [cfg.bev_w, cfg.bev_h],
            },
            "result": {
                "vx": result.vx, "vy": result.vy, "wz": result.wz, "spd": result.spd,
                "line_found": result.line_found,
                "lateral_error_px": float(result.lateral_error),
                "heading_error_rad": float(result.heading_error),
                "tof_blocked": result.tof_blocked,
            },
            "verdict": "OK" if result.line_found else "LINE_NOT_FOUND",
        }
        with open(out / "summary.json", "w") as f:
            json.dump(summary, f, indent=2)
        logger.info("Pipeline tick: line_found=%s vx=%d vy=%d wz=%d  lat_err=%.1fpx hdg_err=%.3frad",
                    result.line_found, result.vx, result.vy, result.wz,
                    result.lateral_error, result.heading_error)
        logger.info("Artefacts written to %s", out)
        return 0 if result.line_found else 1
    finally:
        cam.stop()


# ── Mode: stream (timing + line_found rate over N frames) ────────────────────

def cmd_stream(args) -> int:
    out = _ensure_out(args.out)
    cam = _open_cam(args)
    if cam is None:
        return 2
    try:
        cfg = LineFollowerConfig()
        lf = LineFollower(cfg)

        # Warm up
        for _ in range(5):
            cam.read(); time.sleep(0.03)

        n = args.frames
        found = 0
        ticks = []
        last_frame = None
        last_overlay = None
        t_start = time.time()
        for i in range(n):
            frame = cam.read()
            if frame is None:
                continue
            t0 = time.time()
            r = lf.tick(frame, produce_overlay=(i == n - 1))
            ticks.append(time.time() - t0)
            if r.line_found:
                found += 1
            last_frame = frame
            if r.overlay is not None:
                last_overlay = r.overlay
        elapsed = time.time() - t_start

        if last_frame is not None:
            cv2.imwrite(str(out / "stream_last_raw.png"), last_frame)
        if last_overlay is not None:
            cv2.imwrite(str(out / "stream_last_overlay.png"), last_overlay)

        ticks_arr = np.array(ticks) * 1000.0 if ticks else np.array([0.0])
        report = {
            "frames_requested": n,
            "frames_processed": len(ticks),
            "elapsed_s": elapsed,
            "fps": len(ticks) / elapsed if elapsed > 0 else 0,
            "line_found_pct": (found / max(1, len(ticks))) * 100,
            "tick_ms": {
                "mean":  float(ticks_arr.mean()),
                "p50":   float(np.percentile(ticks_arr, 50)),
                "p95":   float(np.percentile(ticks_arr, 95)),
                "max":   float(ticks_arr.max()),
            },
        }
        with open(out / "stream_report.json", "w") as f:
            json.dump(report, f, indent=2)
        logger.info("Stream: %d frames in %.2fs (%.1f fps), line_found=%.1f%%, tick p95=%.1f ms",
                    len(ticks), elapsed, report["fps"], report["line_found_pct"], report["tick_ms"]["p95"])
        return 0
    finally:
        cam.stop()


# ── Mode: hsv-pick (click → HSV) ─────────────────────────────────────────────

def cmd_hsv_pick(args) -> int:
    out = _ensure_out(args.out)
    cam = _open_cam(args)
    if cam is None:
        return 2
    try:
        frame = _grab(cam)
        if frame is None:
            logger.error("Could not capture a frame")
            return 3
        cv2.imwrite(str(out / "hsvpick_raw.png"), frame)
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        samples: list[tuple[int, int, int]] = []

        def on_click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN:
                h, s, v = hsv[y, x].tolist()
                samples.append((h, s, v))
                logger.info("Clicked (%d,%d) → H=%d S=%d V=%d", x, y, h, s, v)

        win = "hsv-pick — click white line, ESC when done"
        cv2.namedWindow(win)
        cv2.setMouseCallback(win, on_click)
        while True:
            cv2.imshow(win, frame)
            k = cv2.waitKey(20) & 0xFF
            if k == 27:  # ESC
                break
        cv2.destroyAllWindows()
        if not samples:
            logger.warning("No samples collected.")
            return 1
        arr = np.array(samples)
        # Suggest a band: ±tolerance around the per-channel sample range
        h_lo, h_hi = max(0, int(arr[:, 0].min()) - 5),  min(180, int(arr[:, 0].max()) + 5)
        s_lo, s_hi = max(0, int(arr[:, 1].min()) - 30), min(255, int(arr[:, 1].max()) + 30)
        v_lo, v_hi = max(0, int(arr[:, 2].min()) - 30), min(255, int(arr[:, 2].max()) + 30)
        suggested = {"low":  (h_lo, s_lo, v_lo), "high": (h_hi, s_hi, v_hi)}
        logger.info("Suggested HSVRange: low=%s high=%s", suggested["low"], suggested["high"])
        with open(out / "hsv_suggestion.json", "w") as f:
            json.dump({"samples": samples, "suggested": suggested}, f, indent=2)
        return 0
    finally:
        cam.stop()


# ── Mode: bev-pick (click 4 corners → suggested src_pts) ─────────────────────

def cmd_bev_pick(args) -> int:
    out = _ensure_out(args.out)
    cam = _open_cam(args)
    if cam is None:
        return 2
    try:
        frame = _grab(cam)
        if frame is None:
            logger.error("Could not capture a frame")
            return 3
        clone = frame.copy()
        cv2.imwrite(str(out / "bevpick_raw.png"), frame)
        pts: list[tuple[int, int]] = []
        labels = ["TL (top-left)", "TR (top-right)", "BR (bottom-right)", "BL (bottom-left)"]

        def on_click(event, x, y, flags, param):
            if event == cv2.EVENT_LBUTTONDOWN and len(pts) < 4:
                pts.append((x, y))
                logger.info("Point %d %s = (%d,%d)", len(pts), labels[len(pts) - 1], x, y)
                cv2.circle(clone, (x, y), 6, (0, 0, 255), -1)
                cv2.putText(clone, str(len(pts)), (x + 8, y - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        win = "bev-pick — click TL, TR, BR, BL of lane patch (in order)"
        cv2.namedWindow(win)
        cv2.setMouseCallback(win, on_click)
        while True:
            cv2.imshow(win, clone)
            k = cv2.waitKey(20) & 0xFF
            if k == 27 or len(pts) == 4:
                break
        cv2.destroyAllWindows()
        if len(pts) != 4:
            logger.warning("Need exactly 4 points, got %d.", len(pts))
            return 1
        src = np.array(pts, dtype=np.float32)
        # Mirror line_follower defaults for dst (rectangular BEV patch)
        bev_w, bev_h = 300, 600
        dst = np.array([[0, 0], [bev_w, 0], [bev_w, bev_h], [0, bev_h]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        bev = cv2.warpPerspective(frame, M, (bev_w, bev_h))
        cv2.imwrite(str(out / "bevpick_warped.png"), bev)
        suggestion = {
            "bev_src_pts": src.tolist(),
            "bev_dst_pts": dst.tolist(),
            "bev_w": bev_w, "bev_h": bev_h,
        }
        with open(out / "bev_suggestion.json", "w") as f:
            json.dump(suggestion, f, indent=2)
        logger.info("Suggested bev_src_pts:\n%s", src)
        logger.info("Wrote warped preview to %s/bevpick_warped.png — verify the lane is "
                    "vertical and centred.", out)
        return 0
    finally:
        cam.stop()


# ── Entrypoint ───────────────────────────────────────────────────────────────

def main() -> int:
    p = argparse.ArgumentParser(description="Line-follower hardware diagnostic")
    p.add_argument("mode", nargs="?", default="capture",
                   choices=["capture", "stream", "hsv-pick", "bev-pick"])
    p.add_argument("--out", default="/tmp/lf_diag", help="output directory for artefacts")
    p.add_argument("--cam-index", type=int,
                   default=int(os.getenv("KPATROL_CAMERA", "0")),
                   help="cv2 camera index (picamera2 ignores this)")
    p.add_argument("--width",  type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps",    type=int, default=30)
    p.add_argument("--backend", choices=["picamera2", "opencv"], default=None,
                   help="force a backend (default: auto)")
    p.add_argument("--frames", type=int, default=100, help="(stream mode) frames to process")
    args = p.parse_args()

    handlers = {
        "capture":  cmd_capture,
        "stream":   cmd_stream,
        "hsv-pick": cmd_hsv_pick,
        "bev-pick": cmd_bev_pick,
    }
    return handlers[args.mode](args)


if __name__ == "__main__":
    sys.exit(main())
