"""Quick latency + FPS benchmark for the TwinLiteNet+ ONNX export on Pi.

Run after setup_twinlite.sh + export_onnx.py:
    python3 benchmark_lane_seg.py [--frames 200] [--source <camera_index|video.mp4>]

Prints per-frame latency stats (min/median/p95/max) and steady-state FPS.
A dummy black-frame mode (--source none) runs without needing a camera.
"""

from __future__ import annotations

import argparse
import os
import statistics
import sys
import time
from typing import Optional

import numpy as np

# Make the parent package importable when this file is run directly.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))

from navigation.lane_seg import LaneSegConfig, LaneSegProvider


def _open_camera(source: str):
    """Return a frame-yielding generator for `source`.

    `source` is one of:
        "none"     → infinite stream of black frames (CPU bound, no I/O)
        "<int>"    → cv2.VideoCapture index (USB cam / Pi camera v4l2)
        "<path>"   → cv2.VideoCapture file path (mp4 / mjpg)
    """
    if source == "none":
        def gen():
            black = np.zeros((480, 640, 3), dtype=np.uint8)
            while True:
                yield black
        return gen()

    import cv2  # type: ignore
    cap = cv2.VideoCapture(int(source) if source.isdigit() else source)
    if not cap.isOpened():
        raise SystemExit(f"[bench] Failed to open camera source: {source}")
    def gen():
        try:
            while True:
                ok, frame = cap.read()
                if not ok:
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # loop file
                    continue
                yield frame
        finally:
            cap.release()
    return gen()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--frames", type=int, default=200,
                   help="Total frames to benchmark (after warmup)")
    p.add_argument("--warmup", type=int, default=20)
    p.add_argument("--source", default="none",
                   help="Camera index, video file, or 'none' for dummy black frames")
    p.add_argument("--model",
                   default="/home/khoavd/kpatrol/pi-controller/models/twinlitenet_nano_192x320.onnx")
    p.add_argument("--input-size", default="192x320")
    p.add_argument("--threads", type=int, default=2)
    args = p.parse_args()

    h, w = (int(x) for x in args.input_size.split("x"))
    cfg = LaneSegConfig(
        model_path=args.model,
        input_size=(h, w),
        intra_op_threads=args.threads,
    )
    provider = LaneSegProvider(cfg)
    if not provider.available:
        raise SystemExit("[bench] Provider not ready — check model_path and onnxruntime install")

    frames = _open_camera(args.source)
    print(f"[bench] Warming up {args.warmup} frames…")
    for _ in range(args.warmup):
        frame = next(frames)
        provider.infer(frame)

    print(f"[bench] Timing {args.frames} frames @ {h}x{w}, threads={args.threads}")
    latencies = []
    t_wall_start = time.perf_counter()
    for _ in range(args.frames):
        frame = next(frames)
        result = provider.infer(frame)
        if result is None:
            raise SystemExit("[bench] infer() returned None mid-bench — provider crashed?")
        latencies.append(result.latency_ms)
    wall = time.perf_counter() - t_wall_start

    latencies.sort()
    n = len(latencies)
    p50 = latencies[n // 2]
    p95 = latencies[int(n * 0.95)]
    p99 = latencies[int(n * 0.99)]
    print("─" * 60)
    print(f"  Frames        : {n}")
    print(f"  Wall time     : {wall*1000:.0f} ms")
    print(f"  Steady FPS    : {n / wall:.1f}")
    print(f"  Latency  min  : {min(latencies):.1f} ms")
    print(f"  Latency  p50  : {p50:.1f} ms")
    print(f"  Latency  p95  : {p95:.1f} ms")
    print(f"  Latency  p99  : {p99:.1f} ms")
    print(f"  Latency  max  : {max(latencies):.1f} ms")
    print(f"  Latency  mean : {statistics.mean(latencies):.1f} ms (stdev {statistics.pstdev(latencies):.1f})")


if __name__ == "__main__":
    main()
