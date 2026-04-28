#!/usr/bin/env python3
"""
record_route.py — Record raw route video for offline algorithm tuning
=====================================================================
Run on the Pi during MANUAL driving. Grabs frames from the local mjpeg_server
snapshot endpoint and writes a single raw MP4. No overlay, no telemetry —
you optimise the detector/controller against this clean footage offline.

Stops on Ctrl+C (SIGINT/SIGTERM) or when --seconds elapses.

Usage:
    python3 record_route.py                          # until Ctrl+C, /tmp/route/route.mp4
    python3 record_route.py --seconds 60
    python3 record_route.py --out-dir /tmp/route01
"""

import argparse
import os
import signal
import time
import urllib.request

import cv2
import numpy as np

SNAPSHOT_URL = "http://127.0.0.1:8080/snapshot"

STOP = False
def _handle_signal(*_):
    global STOP
    STOP = True
signal.signal(signal.SIGINT,  _handle_signal)
signal.signal(signal.SIGTERM, _handle_signal)


def fetch_frame():
    try:
        with urllib.request.urlopen(SNAPSHOT_URL, timeout=1.0) as r:
            data = r.read()
        arr = np.frombuffer(data, dtype=np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)
    except Exception:
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=int, default=0,
                    help="stop after N seconds (0 = until Ctrl+C)")
    ap.add_argument("--fps", type=int, default=15,
                    help="target recording rate")
    ap.add_argument("--out-dir", default="/tmp/route",
                    help="output directory (auto-created)")
    args = ap.parse_args()

    os.makedirs(args.out_dir, exist_ok=True)
    raw_path = os.path.join(args.out_dir, "route.mp4")

    first = None
    for _ in range(20):
        first = fetch_frame()
        if first is not None:
            break
        time.sleep(0.2)
    if first is None:
        print(f"ERROR: cannot fetch frame from {SNAPSHOT_URL}")
        print("Is mjpeg_server running? Check: systemctl status camera-stream")
        return

    h, w = first.shape[:2]
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(raw_path, fourcc, args.fps, (w, h))

    print(f"Recording @ {args.fps} fps  {w}x{h}  → {raw_path}")
    print(f"  stop: {f'{args.seconds}s' if args.seconds else 'Ctrl+C'}")
    print("Drive the route manually now.")

    t0       = time.time()
    interval = 1.0 / args.fps
    frames   = 0
    try:
        while not STOP:
            if args.seconds and (time.time() - t0 >= args.seconds):
                break
            loop  = time.time()
            frame = fetch_frame()
            if frame is not None:
                writer.write(frame)
                frames += 1
                if frames % (args.fps * 2) == 0:
                    print(f"  [{int(time.time()-t0):3d}s] frames={frames}")
            dt = time.time() - loop
            if dt < interval:
                time.sleep(interval - dt)
    finally:
        writer.release()

    dur = time.time() - t0
    size_mb = os.path.getsize(raw_path) / (1024 * 1024) if os.path.exists(raw_path) else 0.0
    print(f"\nDone: {frames} frames in {dur:.1f}s "
          f"({frames/max(dur,0.001):.1f} fps) → {raw_path} ({size_mb:.2f} MB)")


if __name__ == "__main__":
    main()
