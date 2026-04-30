#!/usr/bin/env python3
"""
carla_capture_and_eval.py — Capture RGB + semantic frames from CARLA,
then run K-Patrol LineFollower pipeline and compute IoU vs ground truth.

Designed to run inside `carla-client:0.9.15` Docker container which has
Python 3.7, carla 0.9.15, numpy, and cv2 available.

Usage (inside container via docker run):
    python3 /test/carla_capture_and_eval.py \
        --host localhost --port 2000 \
        --frames 200 --map Town01 \
        --out /test/results

Output:
    /test/results/
        report.json           — per-scenario IoU + error summary
        overlays/             — sample overlay PNGs (every 10th frame)
        semantic/             — sample semantic mask PNGs
"""

from __future__ import annotations

import argparse
import json
import math
import os
import queue
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Optional, Tuple

import cv2
import numpy as np
import carla

# ── Add navigation module to path ────────────────────────────────────────────
_THIS = Path(__file__).resolve().parent
sys.path.insert(0, str(_THIS.parent))   # /test -> parent is also /test but mounted

# Try multiple possible locations for navigation package
for _nav_path in [
    _THIS.parent,
    Path("/test"),
    Path("/test/navigation").parent,
]:
    if (_nav_path / "navigation" / "line_follower.py").exists():
        sys.path.insert(0, str(_nav_path))
        break

from navigation.line_follower import LineFollower, LineFollowerConfig, HSVRange  # noqa

# ── CARLA semantic class IDs ───────────────────────────────────────────────────
SEMANTIC_ROADLINE = 6    # White/yellow road markings
SEMANTIC_ROAD     = 7    # Drivable road surface

# ── HSV presets ───────────────────────────────────────────────────────────────
HSV_PRESETS = {
    "white":  HSVRange(low=(0,   0, 160), high=(180,  60, 255)),
    "yellow": HSVRange(low=( 5, 50,  80), high=( 35, 255, 255)),
}

# ── Camera spec matching spawn_and_record.py (Bulldog04) ─────────────────────
CAM_W, CAM_H = 640, 360
CAM_FOV = 90.0       # Use wider FOV for better road coverage
CAM_X, CAM_Z = 1.6, 1.5
CAM_PITCH = -35.0    # Steeper downward tilt to see road markings clearly

# ── BEV calibration for 640×360 camera at pitch=-35°, FOV=90° ────────────────
# Wide trapezoid covering the full visible road patch (including centre line).
# Near edge at y=280 (just above hood) covers full road width x=0-640.
# Far edge at y=70 covers visible road horizon x≈80-560.
BEV_SRC = np.float32([
    [ 80,  70],  # far-left
    [560,  70],  # far-right
    [640, 280],  # near-right
    [  0, 280],  # near-left
])
BEV_DST = np.float32([
    [  0,   0],
    [400,   0],
    [400, 480],
    [  0, 480],
])


TURN_THRESH_DEG_S = 8.0   # |yaw_rate| above this → classified as a turn

@dataclass
class FrameResult:
    frame_idx:   int
    iou:         float
    line_found:  bool
    lateral_err: float
    heading_err: float
    gt_px:       int
    det_px:      int
    scenario:    str   # "STRAIGHT" | "TURN_LEFT" | "TURN_RIGHT"
    yaw_rate:    float # deg/s from vehicle physics


def carla_image_to_bgr(img: carla.Image) -> np.ndarray:
    arr = np.frombuffer(img.raw_data, dtype=np.uint8)
    arr = arr.reshape((img.height, img.width, 4))
    return arr[:, :, :3].copy()


def carla_semantic_to_mask(img: carla.Image, class_id: int) -> np.ndarray:
    arr = np.frombuffer(img.raw_data, dtype=np.uint8)
    arr = arr.reshape((img.height, img.width, 4))
    # CARLA semantic: Red channel = class ID (in BGRA order, that's index 2)
    return ((arr[:, :, 2] == class_id).astype(np.uint8) * 255)


def compute_iou(det: np.ndarray, gt: np.ndarray) -> float:
    inter = int(np.logical_and(det > 0, gt > 0).sum())
    union = int(np.logical_or (det > 0, gt > 0).sum())
    return inter / union if union > 0 else 0.0


def get_det_mask(bgr: np.ndarray, lf: LineFollower) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m   = cv2.inRange(hsv,
                      np.array(lf.cfg.hsv.low,  dtype=np.uint8),
                      np.array(lf.cfg.hsv.high, dtype=np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  kernel)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
    return m


class CarlaCapEval:
    def __init__(self, args: argparse.Namespace):
        self.args     = args
        self.client   = None
        self.world    = None
        self.vehicle  = None
        self.cam_rgb  = None
        self.cam_seg  = None
        self._rgb_q:  queue.Queue = queue.Queue(maxsize=200)
        self._seg_q:  queue.Queue = queue.Queue(maxsize=200)
        self.results: List[FrameResult] = []
        self._actors  = []

        hsv = HSV_PRESETS.get(args.tape_color, HSV_PRESETS["white"])
        lf_cfg = LineFollowerConfig(
            hsv=hsv,
            bev_src_pts=BEV_SRC.copy(),
            bev_dst_pts=BEV_DST.copy(),
            bev_w=400, bev_h=480,
            bev_px_per_cm=6.5,
            wheel_half_track_cm=12.0,
            target_left_offset_cm=10.0,
            base_vx=50, base_spd=70,
        )
        self.lf = LineFollower(lf_cfg)

    def connect(self) -> bool:
        print(f"[connect] {self.args.host}:{self.args.port}")
        try:
            self.client = carla.Client(self.args.host, self.args.port)
            self.client.set_timeout(15.0)
            sv = self.client.get_server_version()
            self.world = self.client.get_world()
            print(f"  Server: {sv}  Map: {self.world.get_map().name}")
        except Exception as e:
            print(f"ERROR: {e}")
            return False
        return True

    def load_map(self) -> None:
        cur = self.world.get_map().name.split("/")[-1]
        if self.args.map and cur != self.args.map:
            print(f"  Loading map: {self.args.map}")
            self.world = self.client.load_world(self.args.map)
            time.sleep(3.0)
        print(f"  Map: {self.world.get_map().name}")

    def setup(self) -> None:
        bp_lib = self.world.get_blueprint_library()

        # Async mode — simpler, more robust than synchronous
        settings = self.world.get_settings()
        if settings.synchronous_mode:
            settings.synchronous_mode    = False
            settings.fixed_delta_seconds = None
            self.world.apply_settings(settings)

        # Spawn vehicle — try each spawn point until one is free
        import random as _rnd
        veh_bp = bp_lib.filter("vehicle.tesla.model3")[0]
        spawn_pts = self.world.get_map().get_spawn_points()
        _rnd.shuffle(spawn_pts)
        ego = None
        for sp in spawn_pts:
            try:
                ego = self.world.spawn_actor(veh_bp, sp)
                break
            except RuntimeError:
                continue
        if ego is None:
            raise RuntimeError("All spawn points occupied — cannot spawn vehicle.")
        tm = self.client.get_trafficmanager(9050)
        ego.set_autopilot(True, 9050)
        self._actors.append(ego)
        self.vehicle = ego
        print(f"  Vehicle spawned: {ego.type_id}")

        # Camera mount
        cam_tf = carla.Transform(
            carla.Location(x=CAM_X, z=CAM_Z),
            carla.Rotation(pitch=CAM_PITCH),
        )

        # RGB camera
        rgb_bp = bp_lib.find("sensor.camera.rgb")
        rgb_bp.set_attribute("image_size_x", str(CAM_W))
        rgb_bp.set_attribute("image_size_y", str(CAM_H))
        rgb_bp.set_attribute("fov", str(CAM_FOV))
        self.cam_rgb = self.world.spawn_actor(rgb_bp, cam_tf, attach_to=ego)
        self.cam_rgb.listen(lambda img: self._on_rgb(img))
        self._actors.append(self.cam_rgb)

        # Semantic segmentation (same mount)
        seg_bp = bp_lib.find("sensor.camera.semantic_segmentation")
        seg_bp.set_attribute("image_size_x", str(CAM_W))
        seg_bp.set_attribute("image_size_y", str(CAM_H))
        seg_bp.set_attribute("fov", str(CAM_FOV))
        self.cam_seg = self.world.spawn_actor(seg_bp, cam_tf, attach_to=ego)
        self.cam_seg.listen(lambda img: self._on_seg(img))
        self._actors.append(self.cam_seg)

        print(f"  Cameras: {CAM_W}x{CAM_H} fov={CAM_FOV}° pitch={CAM_PITCH}°")

        # Warmup — wait for first frames to arrive
        print("  Warming up (3s) …")
        time.sleep(3.0)
        # Drain any queued frames
        for q in (self._rgb_q, self._seg_q):
            while not q.empty():
                try: q.get_nowait()
                except queue.Empty: break
        print("  Warmup done")

    def _on_rgb(self, img) -> None:
        try:
            self._rgb_q.put_nowait((img.frame, carla_image_to_bgr(img)))
        except queue.Full:
            # Drop oldest frame to make room
            try: self._rgb_q.get_nowait()
            except queue.Empty: pass
            try: self._rgb_q.put_nowait((img.frame, carla_image_to_bgr(img)))
            except queue.Full: pass

    def _on_seg(self, img) -> None:
        try:
            gt = carla_semantic_to_mask(img, SEMANTIC_ROADLINE)
            self._seg_q.put_nowait((img.frame, gt))
        except queue.Full:
            try: self._seg_q.get_nowait()
            except queue.Empty: pass
            try:
                gt = carla_semantic_to_mask(img, SEMANTIC_ROADLINE)
                self._seg_q.put_nowait((img.frame, gt))
            except queue.Full: pass

    def run(self, out_dir: Path) -> None:
        overlay_dir = out_dir / "overlays"
        sem_dir     = out_dir / "semantic"
        overlay_dir.mkdir(parents=True, exist_ok=True)
        sem_dir.mkdir(parents=True, exist_ok=True)

        target  = self.args.frames
        n       = 0
        print(f"\nCapturing {target} frames (frame-ID matched) …")

        # Dict-based buffers keyed by CARLA frame_id for proper synchronization
        rgb_buf: dict = {}
        seg_buf: dict = {}
        last_match_t = time.monotonic()
        stall_limit  = 15.0  # seconds before giving up

        while n < target:
            # Drain both queues into buffers
            for _ in range(50):
                try:
                    fid, bgr = self._rgb_q.get_nowait()
                    rgb_buf[fid] = bgr
                except queue.Empty:
                    break
            for _ in range(50):
                try:
                    fid, gt = self._seg_q.get_nowait()
                    seg_buf[fid] = gt
                except queue.Empty:
                    break

            # Find frame_ids present in both buffers (exact match)
            matched = sorted(set(rgb_buf.keys()) & set(seg_buf.keys()))

            if not matched:
                # No match yet — check for stall
                if time.monotonic() - last_match_t > stall_limit:
                    print(f"  WARN: no matched frames for {stall_limit:.0f}s — "
                          f"rgb_buf={len(rgb_buf)} seg_buf={len(seg_buf)}, stopping.")
                    break
                time.sleep(0.02)
                continue

            last_match_t = time.monotonic()

            # Process oldest matched frame
            fid     = matched[0]
            bgr     = rgb_buf.pop(fid)
            gt_mask = seg_buf.pop(fid)

            # Evict stale unmatched frames older than current fid
            for k in list(rgb_buf.keys()):
                if k < fid: del rgb_buf[k]
            for k in list(seg_buf.keys()):
                if k < fid: del seg_buf[k]

            # Classify driving scenario from vehicle angular velocity
            av       = self.vehicle.get_angular_velocity()
            yaw_rate = float(av.z)   # deg/s; + = CCW = turning left (CARLA coords)
            if abs(yaw_rate) < TURN_THRESH_DEG_S:
                scenario = "STRAIGHT"
            elif yaw_rate > 0:
                scenario = "TURN_LEFT"
            else:
                scenario = "TURN_RIGHT"

            # Run line follower
            save_this = (n % 10 == 0)
            res      = self.lf.tick(bgr, produce_overlay=save_this)
            det_mask = get_det_mask(bgr, self.lf)
            iou      = compute_iou(det_mask, gt_mask)

            fr = FrameResult(
                frame_idx  = n,
                iou        = round(iou, 4),
                line_found = res.line_found,
                lateral_err= round(res.lateral_error, 2),
                heading_err= round(res.heading_error, 4),
                gt_px      = int((gt_mask > 0).sum()),
                det_px     = int((det_mask > 0).sum()),
                scenario   = scenario,
                yaw_rate   = round(yaw_rate, 2),
            )
            self.results.append(fr)

            # Save overlay + semantic every 10 frames
            if save_this:
                tag = f"{n:04d}_{scenario[:3]}"
                if res.overlay is not None:
                    # Stamp scenario label on overlay
                    ov = res.overlay.copy()
                    scn_col = (0, 200, 80) if scenario == "STRAIGHT" else \
                              (0, 165, 255) if scenario == "TURN_LEFT" else (255, 100, 0)
                    cv2.putText(ov, scenario, (10, 56),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.7, scn_col, 2)
                    cv2.putText(ov, f"yaw={yaw_rate:+.1f}°/s", (10, 80),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, scn_col, 1)
                    cv2.imwrite(str(overlay_dir / f"{tag}_overlay.png"), ov)
                comp = np.zeros((CAM_H, CAM_W, 3), dtype=np.uint8)
                comp[:, :, 1] = gt_mask
                comp[:, :, 0] = det_mask
                cv2.imwrite(str(sem_dir / f"{tag}_comp.png"), comp)

            n += 1
            if n % 50 == 0:
                recent = self.results[-50:]
                avg_iou = sum(r.iou for r in recent) / len(recent)
                found   = sum(1 for r in recent if r.line_found)
                straight_n = sum(1 for r in recent if r.scenario == "STRAIGHT")
                turn_n     = 50 - straight_n
                print(f"  [{n:4d}/{target}]  IoU={avg_iou:.3f}  "
                      f"found={found}/50  straight={straight_n}  turns={turn_n}")

    def cleanup(self) -> None:
        # Stop listeners before destroying
        for a in reversed(self._actors):
            try:
                if hasattr(a, 'stop'):
                    a.stop()
            except Exception:
                pass
        time.sleep(0.5)
        for a in reversed(self._actors):
            try:
                if a.is_alive:
                    a.destroy()
            except Exception:
                pass
        print("Cleanup done.")

    def report(self, out_dir: Path) -> dict:
        if not self.results:
            print("No results.")
            return {}

        ious   = [r.iou          for r in self.results]
        lats   = [r.lateral_err  for r in self.results]
        hdgs   = [r.heading_err  for r in self.results]
        founds = [r.line_found   for r in self.results]

        # ── Per-scenario breakdown ────────────────────────────────────────────
        scenarios_all = ["STRAIGHT", "TURN_LEFT", "TURN_RIGHT"]
        scn_stats = {}
        for scn in scenarios_all:
            sub = [r for r in self.results if r.scenario == scn]
            if not sub:
                scn_stats[scn] = None
                continue
            scn_stats[scn] = {
                "n":           len(sub),
                "det_rate":    round(sum(1 for r in sub if r.line_found) / len(sub), 4),
                "iou_mean":    round(float(np.mean([r.iou for r in sub])),           4),
                "lat_mean_px": round(float(np.mean([abs(r.lateral_err) for r in sub])), 2),
                "hdg_mean_deg":round(float(np.degrees(np.mean([abs(r.heading_err) for r in sub]))), 2),
            }

        print("\n" + "═" * 60)
        print("  K-Patrol LineFollower — CARLA Benchmark")
        print("═" * 60)
        print(f"  Map              : {self.args.map or 'default'}")
        print(f"  Tape colour      : {self.args.tape_color}")
        print(f"  Frames evaluated : {len(self.results)}")
        print(f"  Detection rate   : {sum(founds)/len(founds)*100:.1f}%")
        print(f"  IoU mean         : {np.mean(ious):.4f}")
        print(f"  |Lat err| mean   : {np.mean(np.abs(lats)):.2f} px")
        print(f"  |Hdg err| mean   : {np.degrees(np.mean(np.abs(hdgs))):.2f}°")
        print()
        print(f"  {'Scenario':<14} {'Frames':>6}  {'DetRate':>7}  {'IoU':>6}  {'|Lat|px':>8}  {'|Hdg|°':>7}")
        print(f"  {'-'*14} {'-'*6}  {'-'*7}  {'-'*6}  {'-'*8}  {'-'*7}")
        for scn in scenarios_all:
            st = scn_stats[scn]
            if st is None:
                print(f"  {scn:<14} {'0':>6}  {'—':>7}  {'—':>6}  {'—':>8}  {'—':>7}")
            else:
                print(f"  {scn:<14} {st['n']:>6}  "
                      f"{st['det_rate']*100:>6.1f}%  "
                      f"{st['iou_mean']:>6.4f}  "
                      f"{st['lat_mean_px']:>8.1f}  "
                      f"{st['hdg_mean_deg']:>7.2f}°")
        print("═" * 60)

        summary = {
            "map":             self.args.map or "default",
            "tape_color":      self.args.tape_color,
            "cam_spec": {
                "w": CAM_W, "h": CAM_H,
                "fov": CAM_FOV, "pitch": CAM_PITCH,
            },
            "frames":          len(self.results),
            "detection_rate":  round(sum(founds) / len(founds), 4),
            "iou_mean":        round(float(np.mean(ious)),             4),
            "iou_median":      round(float(np.median(ious)),           4),
            "iou_p90":         round(float(np.percentile(ious, 90)),   4),
            "iou_p10":         round(float(np.percentile(ious, 10)),   4),
            "lat_err_mean_px": round(float(np.mean(np.abs(lats))),     2),
            "hdg_err_mean_deg":round(float(np.degrees(np.mean(np.abs(hdgs)))), 2),
            "by_scenario":     {k: v for k, v in scn_stats.items() if v is not None},
            "per_frame":       [asdict(r) for r in self.results],
        }

        rpt_path = out_dir / "report.json"
        rpt_path.write_text(json.dumps(summary, indent=2))
        print(f"\n  Report → {rpt_path}")
        return summary


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--host",       default="localhost")
    p.add_argument("--port",       type=int, default=2000)
    p.add_argument("--map",        default="Town01")
    p.add_argument("--frames",     type=int, default=400)
    p.add_argument("--tape-color", default="white",
                   choices=list(HSV_PRESETS.keys()))
    p.add_argument("--out",        default="/test/results")
    return p.parse_args()


def main():
    args = parse_args()
    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    test = CarlaCapEval(args)
    if not test.connect():
        sys.exit(1)

    try:
        test.load_map()
        test.setup()
        test.run(out_dir)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        test.cleanup()

    test.report(out_dir)


if __name__ == "__main__":
    main()
