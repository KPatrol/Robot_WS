"""Sample the HSV range of the line in a captured frame.

We pick the K largest connected components matching a colour seed
(default = saturated yellow) and report the H/S/V min/max envelope of
their pixels. The resulting numbers go straight into
LineFollowerConfig.hsv.low / .high.

Usage on the Pi:
    python3 sample_hsv.py /tmp/lane_raw.png            # auto-detect yellow
    python3 sample_hsv.py /tmp/lane_raw.png --hue 30   # seed any hue 0-179
"""

from __future__ import annotations

import argparse
import sys

import cv2
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument("frame", help="Path to a frame snapshot (BGR PNG / JPG)")
    p.add_argument("--hue", type=int, default=30,
                   help="Seed hue (0-179 in OpenCV space). 30 ≈ yellow.")
    p.add_argument("--hue-tol", type=int, default=12,
                   help="Hue tolerance around the seed for the coarse mask.")
    p.add_argument("--min-sat", type=int, default=80,
                   help="Minimum saturation to count a pixel as 'coloured'.")
    p.add_argument("--min-val", type=int, default=80,
                   help="Minimum value (brightness) so dark seams don't count.")
    p.add_argument("--quantile", type=float, default=0.05,
                   help="Trim this fraction of the H/S/V tails (per side) before "
                        "emitting the low/high envelope. 0.05 = 5%/95% range.")
    args = p.parse_args()

    bgr = cv2.imread(args.frame)
    if bgr is None:
        raise SystemExit(f"Cannot read {args.frame}")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

    h_low = max(0, args.hue - args.hue_tol)
    h_high = min(179, args.hue + args.hue_tol)
    coarse = cv2.inRange(hsv,
                          np.array([h_low, args.min_sat, args.min_val], dtype=np.uint8),
                          np.array([h_high, 255, 255], dtype=np.uint8))
    # Morphology cleanup to drop single-pixel speckle.
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    coarse = cv2.morphologyEx(coarse, cv2.MORPH_OPEN, kernel)
    coarse = cv2.morphologyEx(coarse, cv2.MORPH_CLOSE, kernel)

    pct = 100.0 * (coarse > 0).sum() / coarse.size
    print(f"[hsv] Coarse mask: {pct:.2f}% pixels match hue {args.hue}±{args.hue_tol}")
    if pct < 0.05:
        print("[hsv] WARNING: very few pixels matched. Try --hue / --min-sat lower.")

    ys, xs = np.where(coarse > 0)
    if len(ys) < 100:
        raise SystemExit("[hsv] Not enough sample pixels — adjust seed parameters.")

    sampled = hsv[ys, xs]    # (N, 3) in H, S, V order
    qlow, qhigh = args.quantile, 1.0 - args.quantile

    low = np.quantile(sampled, qlow, axis=0).astype(int)
    high = np.quantile(sampled, qhigh, axis=0).astype(int)

    # Clamp + widen slightly so a slightly-darker shadow on the same tape
    # still gets picked up (we add ±5 V tolerance below).
    low_h, low_s, low_v = low
    high_h, high_s, high_v = high
    low_h  = max(0,  low_h - 2)
    high_h = min(179, high_h + 2)
    low_s  = max(0,  low_s - 10)
    high_s = min(255, high_s + 10)
    low_v  = max(0,  low_v - 15)
    high_v = min(255, high_v + 15)

    print()
    print("─" * 60)
    print(f"  HSVRange(")
    print(f"      low =({low_h}, {low_s}, {low_v}),")
    print(f"      high=({high_h}, {high_s}, {high_v}),")
    print(f"  )")
    print("─" * 60)
    print("Sample stats on matched pixels:")
    print(f"  H: median={int(np.median(sampled[:, 0]))}  "
          f"range {int(sampled[:,0].min())}-{int(sampled[:,0].max())}")
    print(f"  S: median={int(np.median(sampled[:, 1]))}  "
          f"range {int(sampled[:,1].min())}-{int(sampled[:,1].max())}")
    print(f"  V: median={int(np.median(sampled[:, 2]))}  "
          f"range {int(sampled[:,2].min())}-{int(sampled[:,2].max())}")

    # Save a side-by-side preview so we can verify visually.
    refined = cv2.inRange(hsv,
                           np.array([low_h, low_s, low_v], dtype=np.uint8),
                           np.array([high_h, high_s, high_v], dtype=np.uint8))
    overlay = bgr.copy()
    overlay[refined > 0] = (0, 255, 255)  # yellow highlight
    blend = cv2.addWeighted(bgr, 0.6, overlay, 0.4, 0)
    grid = np.hstack([bgr, blend])
    out = args.frame.replace(".png", "_hsv.png").replace(".jpg", "_hsv.jpg")
    if out == args.frame:
        out = args.frame + ".hsv.png"
    cv2.imwrite(out, grid)
    print(f"\n[hsv] Preview saved to {out}")


if __name__ == "__main__":
    main()
