#!/usr/bin/env python3
"""
carla_lane_test.py — Benchmark K-Patrol line-follower pipeline in CARLA.

What this does
--------------
1. Connects to CARLA server (default 10.8.0.7:2000).
2. Spawns a vehicle + two cameras:
     • RGB camera   — fed into the HSV+BEV line_follower pipeline
     • Semantic seg — used as ground truth for road-marking pixels
3. Drives the vehicle at low speed while both cameras capture frames.
4. On each frame pair:
     a. Run LineFollower.tick(rgb_frame)  → mask + (vy, wz) commands
     b. Extract road-line pixels from semantic frame (class 6)
     c. Compute IoU of HSV mask vs semantic ground truth
     d. Record lateral_error, heading_error, IoU per frame
5. After N seconds shows a matplotlib summary:
     • IoU distribution histogram
     • Lateral / heading error time-series
     • Sample overlaid frames (HSV mask vs semantic mask side-by-side)
6. Saves a JSON report to carla_lane_test_report.json.

Usage (run on the CARLA server machine)
---------------------------------------
    # Minimal
    python3 carla_lane_test.py

    # Custom server / duration / map
    python3 carla_lane_test.py --host 10.8.0.7 --port 2000 --duration 60 \
            --map Town04 --speed 15 --tape-color white

    # Headless (no display) — saves PNG summary only
    python3 carla_lane_test.py --headless

    # Show live overlay window while running
    python3 carla_lane_test.py --live

HSV presets (--tape-color)
--------------------------
    white   — white road lines (default)
    yellow  — yellow road lines
    dark    — dark floor tape on light floor (invert logic)

Notes
-----
- Requires: carla (pip install carla), opencv-python, numpy, matplotlib
- Run inside the same machine as CARLA server for lowest latency.
- CARLA must already be running with Traffic Manager on default port.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import time
import threading
from dataclasses import dataclass, field, asdict
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

# ── Try importing CARLA client library ────────────────────────────────────────
# Deferred to main() — keep module import side-effect-free so pytest can
# discover sibling test modules even on machines without CARLA installed.
try:
    import carla
    _CARLA_IMPORT_ERROR: Optional[str] = None
except ImportError:
    carla = None  # type: ignore
    _CARLA_IMPORT_ERROR = (
        "ERROR: 'carla' Python package not found.\n"
        "Install with: pip install carla\n"
        "Or add CARLA PythonAPI to PYTHONPATH:\n"
        "  export PYTHONPATH=$PYTHONPATH:/opt/carla/PythonAPI/carla/dist/carla-*.egg"
    )

# ── Add repo pi-controller to path so we can import line_follower ──────────────
_THIS_DIR = Path(__file__).resolve().parent
_PI_DIR   = _THIS_DIR.parent
sys.path.insert(0, str(_PI_DIR))

from navigation.line_follower import LineFollower, LineFollowerConfig, HSVRange  # noqa: E402

# ── CARLA semantic segmentation class IDs ─────────────────────────────────────
SEMANTIC_ROAD      = 1   # Road surface
SEMANTIC_ROADLINE  = 6   # Road lines (white/yellow markings)
SEMANTIC_SIDEWALK  = 2

# ── HSV presets ───────────────────────────────────────────────────────────────
HSV_PRESETS: Dict[str, HSVRange] = {
    "white":  HSVRange(low=(0, 0, 160),   high=(180, 50,  255)),   # white markings
    "yellow": HSVRange(low=(15, 80, 120),  high=(35, 255, 255)),   # yellow markings
    "dark":   HSVRange(low=(0, 0,   0),    high=(180, 60,  80)),   # dark tape on light floor
}

# ── BEV calibration for CARLA top-down camera (640×480, pitch -35°) ──────────
# These points correspond to a roughly 6 m × 4 m ground patch in front of vehicle.
CARLA_BEV_SRC = np.float32([
    [150, 220],  # TL (far left)
    [490, 220],  # TR (far right)
    [580, 430],  # BR (near right)
    [ 60, 430],  # BL (near left)
])
CARLA_BEV_DST = np.float32([
    [  0,   0],
    [400,   0],
    [400, 480],
    [  0, 480],
])


# ── Per-frame metrics ─────────────────────────────────────────────────────────

@dataclass
class FrameMetrics:
    t:             float
    iou:           float   # HSV mask vs semantic ground-truth
    lateral_err:   float   # pixels (from LineFollower)
    heading_err:   float   # radians (from LineFollower)
    line_found:    bool
    gt_px:         int     # ground-truth road-line pixels in frame
    det_px:        int     # detected pixels in frame


# ── Test runner ───────────────────────────────────────────────────────────────

class CarlaLaneTest:
    def __init__(self, args: argparse.Namespace):
        self.args        = args
        self.client      = None
        self.world       = None
        self.vehicle     = None
        self.cam_rgb     = None
        self.cam_seg     = None
        self._rgb_q: queue.Queue = queue.Queue(maxsize=2)
        self._seg_q: queue.Queue = queue.Queue(maxsize=2)
        self.metrics: List[FrameMetrics] = []
        self._stop = threading.Event()

        # Build line follower with CARLA-tuned BEV
        hsv_range = HSV_PRESETS.get(args.tape_color, HSV_PRESETS["white"])
        lf_cfg = LineFollowerConfig(
            hsv=hsv_range,
            bev_src_pts=CARLA_BEV_SRC.copy(),
            bev_dst_pts=CARLA_BEV_DST.copy(),
            bev_w=400, bev_h=480,
            bev_px_per_cm=6.5,
            wheel_half_track_cm=12.0,
            target_left_offset_cm=10.0,
            base_vx=50,
            base_spd=70,
        )
        self.lf = LineFollower(lf_cfg)

    # ── CARLA setup ───────────────────────────────────────────────────────────

    def connect(self) -> bool:
        print(f"Connecting to CARLA at {self.args.host}:{self.args.port} …")
        try:
            self.client = carla.Client(self.args.host, self.args.port)
            self.client.set_timeout(10.0)
            self.world   = self.client.get_world()
            sv = self.client.get_server_version()
            print(f"  CARLA server version : {sv}")
            print(f"  Current map          : {self.world.get_map().name}")
        except Exception as e:
            print(f"ERROR: cannot connect — {e}")
            return False
        return True

    def load_map(self) -> None:
        if self.args.map and self.args.map != self.world.get_map().name:
            print(f"Loading map {self.args.map} …")
            self.world = self.client.load_world(self.args.map)
            time.sleep(2.0)

    def spawn_vehicle(self) -> None:
        bp_lib = self.world.get_blueprint_library()
        vehicle_bp = bp_lib.filter("vehicle.tesla.model3")[0]
        spawn_points = self.world.get_map().get_spawn_points()
        sp = spawn_points[0]
        self.vehicle = self.world.spawn_actor(vehicle_bp, sp)
        print(f"  Vehicle spawned: {self.vehicle.type_id}  id={self.vehicle.id}")
        # Enable autopilot
        self.vehicle.set_autopilot(True)

    def attach_cameras(self) -> None:
        bp_lib = self.world.get_blueprint_library()

        # ── RGB camera ───────────────────────────────────────────────────────
        rgb_bp = bp_lib.find("sensor.camera.rgb")
        rgb_bp.set_attribute("image_size_x", "640")
        rgb_bp.set_attribute("image_size_y", "480")
        rgb_bp.set_attribute("fov",          "90")
        # Mount: 1.5 m above hood, tilted 35° down
        cam_transform = carla.Transform(
            carla.Location(x=1.6, z=1.5),
            carla.Rotation(pitch=-35.0)
        )
        self.cam_rgb = self.world.spawn_actor(rgb_bp, cam_transform,
                                               attach_to=self.vehicle)
        self.cam_rgb.listen(lambda img: self._on_rgb(img))

        # ── Semantic segmentation camera (same mount) ─────────────────────
        seg_bp = bp_lib.find("sensor.camera.semantic_segmentation")
        seg_bp.set_attribute("image_size_x", "640")
        seg_bp.set_attribute("image_size_y", "480")
        seg_bp.set_attribute("fov",          "90")
        self.cam_seg = self.world.spawn_actor(seg_bp, cam_transform,
                                               attach_to=self.vehicle)
        self.cam_seg.listen(lambda img: self._on_seg(img))
        print("  Cameras attached (RGB + SemanticSeg)")

    def _on_rgb(self, image) -> None:
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        bgr = arr[:, :, :3].copy()   # drop alpha, keep BGR order
        try:
            self._rgb_q.put_nowait((image.frame, bgr))
        except queue.Full:
            pass

    def _on_seg(self, image) -> None:
        # Semantic seg: R channel = semantic class ID
        arr = np.frombuffer(image.raw_data, dtype=np.uint8)
        arr = arr.reshape((image.height, image.width, 4))
        seg_r = arr[:, :, 2].copy()   # CARLA stores class in R channel
        try:
            self._seg_q.put_nowait((image.frame, seg_r))
        except queue.Full:
            pass

    # ── Main test loop ────────────────────────────────────────────────────────

    def run(self) -> None:
        print(f"\nRunning for {self.args.duration}s …  (Ctrl+C to stop early)\n")
        t_end   = time.monotonic() + self.args.duration
        n_frame = 0
        last_rgb_frame = None

        while time.monotonic() < t_end and not self._stop.is_set():
            # Drain queues — grab the latest matching pair
            try:
                frame_id_rgb, bgr = self._rgb_q.get(timeout=0.5)
                last_rgb_frame = bgr
            except queue.Empty:
                continue

            try:
                frame_id_seg, seg_r = self._seg_q.get_nowait()
            except queue.Empty:
                continue

            # ── Ground truth: road-line mask ─────────────────────────────
            gt_mask = (seg_r == SEMANTIC_ROADLINE).astype(np.uint8) * 255

            # ── Run line follower ─────────────────────────────────────────
            result = self.lf.tick(bgr, produce_overlay=self.args.live)

            # ── Convert HSV mask to binary (re-run to get the raw mask) ──
            hsv_img = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
            det_mask = cv2.inRange(
                hsv_img,
                np.array(self.lf.cfg.hsv.low,  dtype=np.uint8),
                np.array(self.lf.cfg.hsv.high, dtype=np.uint8),
            )
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            det_mask = cv2.morphologyEx(det_mask, cv2.MORPH_OPEN,  kernel)
            det_mask = cv2.morphologyEx(det_mask, cv2.MORPH_CLOSE, kernel)

            # ── IoU ───────────────────────────────────────────────────────
            inter = int(np.logical_and(det_mask > 0, gt_mask > 0).sum())
            union = int(np.logical_or (det_mask > 0, gt_mask > 0).sum())
            iou   = inter / union if union > 0 else 0.0

            m = FrameMetrics(
                t           = time.monotonic(),
                iou         = iou,
                lateral_err = result.lateral_error,
                heading_err = result.heading_error,
                line_found  = result.line_found,
                gt_px       = int((gt_mask > 0).sum()),
                det_px      = int((det_mask > 0).sum()),
            )
            self.metrics.append(m)
            n_frame += 1

            # ── Live window ───────────────────────────────────────────────
            if self.args.live and result.overlay is not None:
                gt_color = cv2.cvtColor(gt_mask, cv2.COLOR_GRAY2BGR)
                gt_color[:, :, 1] = gt_mask   # tint green for GT
                side = np.hstack([result.overlay, gt_color])
                cv2.imshow("K-Patrol Lane Test | Overlay  ←→  GT Semantic", side)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

            # ── Console progress ───────────────────────────────────────────
            if n_frame % 20 == 0:
                avg_iou = sum(x.iou for x in self.metrics[-20:]) / 20
                found   = sum(1 for x in self.metrics[-20:] if x.line_found)
                lat_avg = sum(abs(x.lateral_err) for x in self.metrics[-20:]) / 20
                print(f"  frame={n_frame:4d}  IoU={avg_iou:.3f}  "
                      f"found={found}/20  |lat|={lat_avg:.1f}px")

        if self.args.live:
            cv2.destroyAllWindows()

    # ── Cleanup ───────────────────────────────────────────────────────────────

    def cleanup(self) -> None:
        self._stop.set()
        for actor in [self.cam_rgb, self.cam_seg, self.vehicle]:
            try:
                if actor:
                    actor.destroy()
            except Exception:
                pass
        print("Actors destroyed.")

    # ── Report ────────────────────────────────────────────────────────────────

    def report(self) -> None:
        if not self.metrics:
            print("No metrics collected.")
            return

        ious   = [m.iou          for m in self.metrics]
        lats   = [m.lateral_err  for m in self.metrics]
        hdgs   = [m.heading_err  for m in self.metrics]
        founds = [m.line_found   for m in self.metrics]

        print("\n" + "═" * 58)
        print("  K-Patrol Line Follower — CARLA Benchmark Report")
        print("═" * 58)
        print(f"  Frames evaluated  : {len(self.metrics)}")
        print(f"  Detection rate    : {sum(founds)/len(founds)*100:.1f}%")
        print(f"  IoU mean          : {np.mean(ious):.4f}")
        print(f"  IoU median        : {np.median(ious):.4f}")
        print(f"  IoU p90           : {np.percentile(ious, 90):.4f}")
        print(f"  |Lateral err| mean: {np.mean(np.abs(lats)):.2f} px")
        print(f"  |Heading err| mean: {np.degrees(np.mean(np.abs(hdgs))):.2f}°")
        print("═" * 58)

        # ── Save JSON ─────────────────────────────────────────────────────
        report_path = Path(__file__).parent / "carla_lane_test_report.json"
        summary = {
            "host":            self.args.host,
            "map":             self.args.map or "default",
            "tape_color":      self.args.tape_color,
            "frames":          len(self.metrics),
            "detection_rate":  round(sum(founds)/len(founds), 4),
            "iou_mean":        round(float(np.mean(ious)),    4),
            "iou_median":      round(float(np.median(ious)),  4),
            "iou_p90":         round(float(np.percentile(ious, 90)), 4),
            "lat_err_mean_px": round(float(np.mean(np.abs(lats))), 2),
            "hdg_err_mean_deg":round(float(np.degrees(np.mean(np.abs(hdgs)))), 2),
        }
        with open(report_path, "w") as f:
            json.dump(summary, f, indent=2)
        print(f"\n  Report saved → {report_path}")

        # ── Plot ──────────────────────────────────────────────────────────
        if not self.args.headless:
            self._plot(ious, lats, hdgs, founds)

    def _plot(self, ious, lats, hdgs, founds) -> None:
        try:
            import matplotlib.pyplot as plt
            import matplotlib.gridspec as gridspec
        except ImportError:
            print("matplotlib not found — skipping plot.")
            return

        fig = plt.figure(figsize=(14, 8))
        fig.suptitle("K-Patrol Lane Follower — CARLA Benchmark", fontsize=14, fontweight="bold")
        gs  = gridspec.GridSpec(2, 3, figure=fig, hspace=0.40, wspace=0.35)

        ts = [m.t - self.metrics[0].t for m in self.metrics]

        # IoU time-series
        ax1 = fig.add_subplot(gs[0, :2])
        ax1.plot(ts, ious, color="#2196F3", linewidth=0.8, alpha=0.7)
        ax1.axhline(np.mean(ious), color="#F44336", linestyle="--", linewidth=1.2,
                    label=f"Mean IoU {np.mean(ious):.3f}")
        ax1.axhline(0.30, color="#FF9800", linestyle=":", linewidth=1.0, label="IoU 0.30 threshold")
        ax1.set_xlabel("Time (s)"); ax1.set_ylabel("HSV vs GT IoU")
        ax1.set_title("IoU over time"); ax1.legend(fontsize=8); ax1.set_ylim(0, 1)

        # IoU histogram
        ax2 = fig.add_subplot(gs[0, 2])
        ax2.hist(ious, bins=30, color="#2196F3", edgecolor="white", alpha=0.85)
        ax2.axvline(np.mean(ious), color="#F44336", linestyle="--", linewidth=1.5)
        ax2.set_xlabel("IoU"); ax2.set_title("IoU distribution")

        # Lateral error time-series
        ax3 = fig.add_subplot(gs[1, :2])
        ax3.plot(ts, lats, color="#4CAF50", linewidth=0.8, alpha=0.8)
        ax3.axhline(0, color="grey", linewidth=0.8, linestyle="--")
        ax3.fill_between(ts, lats, alpha=0.15, color="#4CAF50")
        ax3.set_xlabel("Time (s)"); ax3.set_ylabel("Lateral error (px)")
        ax3.set_title("Lateral error (+ = line right of target)")

        # Heading error
        ax4 = fig.add_subplot(gs[1, 2])
        hdg_deg = [math.degrees(h) for h in hdgs]
        ax4.hist(hdg_deg, bins=30, color="#9C27B0", edgecolor="white", alpha=0.85)
        ax4.set_xlabel("Heading error (°)"); ax4.set_title("Heading error distribution")

        out_png = Path(__file__).parent / "carla_lane_test_plot.png"
        fig.savefig(out_png, dpi=120, bbox_inches="tight")
        print(f"  Plot saved → {out_png}")
        plt.show()


# ── CLI ───────────────────────────────────────────────────────────────────────

def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="CARLA line-follower benchmark")
    p.add_argument("--host",       default="10.8.0.7", help="CARLA server IP")
    p.add_argument("--port",       type=int, default=2000)
    p.add_argument("--map",        default=None,
                   help="CARLA map to load (e.g. Town04). None = keep current.")
    p.add_argument("--duration",   type=int, default=60,
                   help="Test duration in seconds (default 60)")
    p.add_argument("--speed",      type=int, default=20,
                   help="Vehicle autopilot target speed km/h (default 20)")
    p.add_argument("--tape-color", default="white",
                   choices=list(HSV_PRESETS.keys()),
                   help="HSV preset for line colour (default white)")
    p.add_argument("--live",       action="store_true",
                   help="Show live OpenCV overlay window")
    p.add_argument("--headless",   action="store_true",
                   help="Skip matplotlib display (save PNG only)")
    return p.parse_args()


def main() -> None:
    if _CARLA_IMPORT_ERROR is not None:
        sys.exit(_CARLA_IMPORT_ERROR)
    args = parse_args()
    test = CarlaLaneTest(args)

    if not test.connect():
        sys.exit(1)

    try:
        test.load_map()
        test.spawn_vehicle()
        test.attach_cameras()

        # Give autopilot a moment to engage
        print("Warming up (3 s) …")
        time.sleep(3.0)

        test.run()

    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        test.cleanup()

    test.report()


if __name__ == "__main__":
    main()
