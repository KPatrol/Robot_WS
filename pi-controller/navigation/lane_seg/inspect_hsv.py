"""Run the same line-follower HSV pipeline against a captured frame and
render a side-by-side preview so we can verify the colour detection +
BEV warp + column scan without sending the robot anywhere.

Useful when calibrating tape colour / BEV homography on the actual demo
location — much faster turnaround than restarting the service and
watching MQTT telemetry."""

from __future__ import annotations

import argparse
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from navigation.line_follower import HSVRange, LineFollower, LineFollowerConfig


def grab(stream: str, timeout_s: float = 5.0) -> np.ndarray:
    if os.path.exists(stream):
        img = cv2.imread(stream)
        if img is None:
            raise SystemExit(f"Can't read {stream}")
        return img
    cap = cv2.VideoCapture(stream)
    if not cap.isOpened():
        raise SystemExit(f"Cannot open stream {stream}")
    deadline = time.monotonic() + timeout_s
    last = None
    skip = 5
    while time.monotonic() < deadline:
        ok, frame = cap.read()
        if not ok:
            continue
        if skip > 0:
            skip -= 1
            continue
        last = frame
        break
    cap.release()
    if last is None:
        raise SystemExit("No frame received in time")
    return last


def label(img: np.ndarray, text: str) -> np.ndarray:
    out = img.copy()
    cv2.rectangle(out, (0, 0), (out.shape[1], 28), (0, 0, 0), -1)
    cv2.putText(out, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (255, 255, 255), 1, cv2.LINE_AA)
    return out


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--source", default="http://127.0.0.1:8080/stream",
                   help="MJPEG URL or path to a still image")
    p.add_argument("--out", default="/tmp/hsv_inspect.png")
    args = p.parse_args()

    frame = grab(args.source)
    h, w = frame.shape[:2]
    print(f"[hsv] Frame {h}x{w}")

    cfg = LineFollowerConfig()
    print(f"[hsv] Default HSV: low={cfg.hsv.low} high={cfg.hsv.high}")

    # Replicate the mask extraction step of line_follower.tick() exactly.
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.array(cfg.hsv.low, dtype=np.uint8),
                            np.array(cfg.hsv.high, dtype=np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    pct = 100.0 * (mask > 0).sum() / mask.size
    print(f"[hsv] Mask coverage: {pct:.2f}% of pixels")

    # Run a full tick to see what the PD pipeline decides.
    lf = LineFollower(cfg)
    t0 = time.perf_counter()
    result = lf.tick(frame)
    dt = (time.perf_counter() - t0) * 1000
    lost = not getattr(result, "line_found", False)
    lat = getattr(result, "lateral_error", getattr(result, "lateral_err", 0.0))
    hdg = getattr(result, "heading_error", getattr(result, "heading_err", 0.0))
    print(f"[hsv] tick(): vy={result.vy} wz={result.wz} spd={result.spd}  "
          f"lost={lost}  lat_err={lat:+.2f}  hdg_err={hdg:+.2f}  ({dt:.1f} ms)")

    blend = frame.copy()
    yellow = np.zeros_like(blend)
    yellow[mask > 0] = (0, 255, 255)
    blend = cv2.addWeighted(blend, 0.55, yellow, 0.45, 0)

    # Draw the BEV trapezoid on the overlay so we can see whether the line
    # actually falls inside the homography source region.
    pts = cfg.bev_src_pts.astype(int).reshape(-1, 1, 2)
    cv2.polylines(blend, [pts], True, (0, 255, 0), 2)

    # Apply the BEV warp to the mask + frame so we can see what the column
    # scan actually receives.
    M = cv2.getPerspectiveTransform(cfg.bev_src_pts, cfg.bev_dst_pts)
    bev_mask = cv2.warpPerspective(mask, M, (cfg.bev_w, cfg.bev_h))
    bev_frame = cv2.warpPerspective(frame, M, (cfg.bev_w, cfg.bev_h))
    bev_blend = bev_frame.copy()
    by = np.zeros_like(bev_blend)
    by[bev_mask > 0] = (0, 255, 255)
    bev_blend = cv2.addWeighted(bev_blend, 0.55, by, 0.45, 0)

    # Pad BEV (400×480) to match frame height (540) for neat hstack.
    pad_h = blend.shape[0] - bev_blend.shape[0]
    if pad_h > 0:
        bev_blend = cv2.copyMakeBorder(bev_blend, 0, pad_h, 0, 0,
                                        cv2.BORDER_CONSTANT, value=0)
        bev_mask_pad = cv2.copyMakeBorder(bev_mask, 0, pad_h, 0, 0,
                                          cv2.BORDER_CONSTANT, value=0)
    else:
        bev_mask_pad = bev_mask

    bev_mask_bgr = cv2.cvtColor(bev_mask_pad, cv2.COLOR_GRAY2BGR)
    bev_pct = 100.0 * (bev_mask > 0).sum() / max(1, bev_mask.size)

    # Top row in source-frame size (3 panels of 960×540 = 2880×540).
    top = np.hstack([
        label(frame, "1. Camera frame"),
        label(cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR), f"2. HSV mask ({pct:.2f}%)"),
        label(blend, f"3. Overlay + BEV box  ({dt:.0f} ms)"),
    ])
    # Bottom row in BEV size (3 × 400 = 1200 wide). Pad horizontally to
    # match the top row width so vstack works.
    bev_row = np.hstack([
        label(bev_blend, f"4. BEV view"),
        label(bev_mask_bgr, f"5. BEV mask ({bev_pct:.2f}%)"),
    ])
    if bev_row.shape[1] < top.shape[1]:
        pad = top.shape[1] - bev_row.shape[1]
        bev_row = cv2.copyMakeBorder(bev_row, 0, 0, 0, pad,
                                      cv2.BORDER_CONSTANT, value=0)
    grid = np.vstack([top, bev_row])
    cv2.imwrite(args.out, grid)
    print(f"[hsv] BEV mask coverage: {bev_pct:.2f}%")
    print(f"[hsv] Saved {args.out}")


if __name__ == "__main__":
    main()
