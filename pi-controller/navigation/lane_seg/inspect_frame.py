"""Snapshot 1 frame from the Pi MJPEG stream, run TwinLiteNet+ inference,
save an overlay PNG showing what the model sees.

Designed as a quick visual sanity-check before turning AUTO_LINE_FOLLOW
on for real. Run on the Pi so the snapshot comes from the actual robot
camera (no scp dance) and the inference timings reflect production
hardware.

Usage:
    python3 inspect_frame.py [--model PATH] [--out frame_overlay.png] \
        [--stream http://127.0.0.1:8080/stream] [--threshold 0.5]
"""

from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import cv2

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
from navigation.lane_seg import LaneSegConfig, LaneSegProvider


def grab_one_frame(stream_url: str, timeout_s: float = 5.0) -> np.ndarray:
    """Pull one frame off the MJPEG stream. cv2.VideoCapture handles the
    multipart boundary parsing for us; we just need to wait for the first
    decoded frame to arrive."""
    cap = cv2.VideoCapture(stream_url)
    if not cap.isOpened():
        raise SystemExit(f"[inspect] Cannot open stream {stream_url}")
    # Drop the first 5 frames — they're often partially-buffered and look
    # half-decoded, which would skew the mask we're about to render.
    deadline = time.monotonic() + timeout_s
    last_frame = None
    skip = 5
    while time.monotonic() < deadline:
        ok, frame = cap.read()
        if not ok:
            continue
        if skip > 0:
            skip -= 1
            continue
        last_frame = frame
        break
    cap.release()
    if last_frame is None:
        raise SystemExit("[inspect] Stream opened but no frame arrived within timeout")
    return last_frame


def make_overlay(frame: np.ndarray, mask: np.ndarray, prob: np.ndarray,
                 lat_ms: float) -> np.ndarray:
    """Stack four panels: raw frame, soft-prob heatmap, hard mask, blend.
    Resulting image is easy to eyeball at a glance to judge detection."""
    h, w = frame.shape[:2]

    # Soft probability — upsample to source res, apply COLORMAP_JET so the
    # gradient pops on the wood-grain floor.
    prob_up = cv2.resize((prob * 255).astype(np.uint8), (w, h),
                        interpolation=cv2.INTER_LINEAR)
    prob_color = cv2.applyColorMap(prob_up, cv2.COLORMAP_JET)

    # Hard binary mask painted yellow on a darkened copy of the frame.
    dim = (frame * 0.5).astype(np.uint8)
    blend = dim.copy()
    mask3 = cv2.merge([mask, mask, np.zeros_like(mask)])  # yellow (BGR-ish)
    yellow = np.zeros_like(blend)
    yellow[mask > 0] = (0, 255, 255)  # BGR yellow
    blend = cv2.addWeighted(blend, 1.0, yellow, 0.7, 0)

    # Mask alone as a 3-channel image for the grid.
    mask3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    # Build 2x2 grid with labels.
    def label(img: np.ndarray, text: str) -> np.ndarray:
        out = img.copy()
        cv2.rectangle(out, (0, 0), (out.shape[1], 28), (0, 0, 0), -1)
        cv2.putText(out, text, (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)
        return out

    pixels_total = mask.size
    pixels_lane = int((mask > 0).sum())
    pct = 100.0 * pixels_lane / pixels_total

    top = np.hstack([
        label(frame, "1. Source frame"),
        label(prob_color, f"2. Lane prob heatmap (max={prob.max():.2f})"),
    ])
    bottom = np.hstack([
        label(mask3, f"3. Binary mask ({pct:.2f}% lane px)"),
        label(blend, f"4. Overlay  ({lat_ms:.0f} ms inference)"),
    ])
    return np.vstack([top, bottom])


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--stream", default="http://127.0.0.1:8080/stream")
    p.add_argument("--model",
                   default="/home/khoavd/kpatrol/pi-controller/models/twinlitenet_nano_192x320.onnx")
    p.add_argument("--input-size", default="192x320")
    p.add_argument("--threshold", type=float, default=0.5)
    p.add_argument("--out", default="/tmp/lane_inspect.png")
    p.add_argument("--save-raw", default="/tmp/lane_raw.png",
                   help="Also save the un-annotated frame so we can re-run with new thresholds.")
    args = p.parse_args()

    h, w = (int(x) for x in args.input_size.split("x"))
    cfg = LaneSegConfig(
        model_path=args.model,
        input_size=(h, w),
        lane_threshold=args.threshold,
    )
    provider = LaneSegProvider(cfg)
    if not provider.available:
        raise SystemExit(f"[inspect] Provider not ready (model={args.model}, "
                        "onnxruntime installed?)")

    print(f"[inspect] Grabbing frame from {args.stream}…")
    frame = grab_one_frame(args.stream)
    print(f"[inspect] Got frame {frame.shape}, running inference…")
    cv2.imwrite(args.save_raw, frame)
    result = provider.infer(frame)
    if result is None:
        raise SystemExit("[inspect] infer returned None")
    print(f"[inspect] Latency: {result.latency_ms:.1f} ms")
    print(f"[inspect] Prob: min={result.prob.min():.3f} "
          f"max={result.prob.max():.3f} mean={result.prob.mean():.3f}")
    print(f"[inspect] Mask: lane pixels = {int((result.mask>0).sum())} / "
          f"{result.mask.size} ({100.0*(result.mask>0).sum()/result.mask.size:.2f}%)")

    overlay = make_overlay(frame, result.mask, result.prob, result.latency_ms)
    cv2.imwrite(args.out, overlay)
    print(f"[inspect] Saved overlay to {args.out}")


if __name__ == "__main__":
    main()
