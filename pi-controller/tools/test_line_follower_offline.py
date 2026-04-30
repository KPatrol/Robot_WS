#!/usr/bin/env python3
"""
test_line_follower_offline.py — Offline synthetic validation for LineFollower.

Generates synthetic 640×480 floor images with white lane markings, runs the
LineFollower pipeline, and reports IoU + error metrics — no CARLA/VPN needed.

Scenarios tested
----------------
  straight     — vertical white stripe, robot centred on line
  angled_left  — line angled 15° left
  angled_right — line angled 15° right
  offset_right — straight line offset 60px to the right of centre
  offset_left  — straight line offset 60px to the left of centre
  noisy_bg     — straight line with salt-and-pepper background noise
  double_line  — two parallel white lines (edge case)
  dim_line     — low-contrast line (near detection threshold)

Outputs
-------
  tools/offline_test_report.json   — per-scenario summary (IoU, errors, found%)
  tools/offline_test_plot.png      — 8-panel visualization
"""

from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import List, Tuple, Optional

import cv2
import numpy as np

# ── Add pi-controller root to path ────────────────────────────────────────────
_THIS = Path(__file__).resolve().parent
_PI   = _THIS.parent
sys.path.insert(0, str(_PI))

from navigation.line_follower import LineFollower, LineFollowerConfig, HSVRange  # noqa

# ── BEV calibration matching the 640×480 camera defaults ─────────────────────
BEV_SRC = np.float32([
    [160, 240], [480, 240],
    [580, 440], [ 60, 440],
])
BEV_DST = np.float32([
    [  0,   0], [400,   0],
    [400, 480], [  0, 480],
])


def _make_lf(hsv_preset: str = "white") -> LineFollower:
    presets = {
        "white":  HSVRange(low=(0,   0, 160), high=(180,  60, 255)),
        "yellow": HSVRange(low=(15, 80, 120), high=( 35, 255, 255)),
    }
    cfg = LineFollowerConfig(
        hsv              = presets[hsv_preset],
        bev_src_pts      = BEV_SRC.copy(),
        bev_dst_pts      = BEV_DST.copy(),
        bev_w            = 400,
        bev_h            = 480,
        bev_px_per_cm    = 5.0,
        wheel_half_track_cm  = 15.0,
        target_left_offset_cm= 10.0,
        base_vx          = 60,
        base_spd         = 80,
    )
    return LineFollower(cfg)


# ── Synthetic image generators ─────────────────────────────────────────────────

def _dark_floor(h: int = 480, w: int = 640) -> np.ndarray:
    """Create a realistic-looking dark floor (perlin-ish texture)."""
    base = np.random.randint(30, 60, (h, w, 3), dtype=np.uint8)
    # add subtle grain
    noise = np.random.randint(-10, 10, (h, w, 3), dtype=np.int16)
    floor = np.clip(base.astype(np.int16) + noise, 0, 255).astype(np.uint8)
    return floor


def _draw_line_mask(
    h: int, w: int,
    cx: float,      # x centre of line at image bottom (row=h-1)
    angle_deg: float = 0.0,   # clockwise from vertical
    thickness: int = 20,
    brightness: int = 230,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Returns (bgr_frame, gt_mask) for a single stripe on a dark floor.
    angle_deg: positive = line tilts right at the top.
    """
    floor = _dark_floor(h, w)
    gt    = np.zeros((h, w), dtype=np.uint8)

    angle_rad = math.radians(angle_deg)

    # For each row y, compute x-centre of line
    for y in range(h):
        dy = (h - 1) - y        # distance from bottom
        x  = int(round(cx + dy * math.tan(angle_rad)))
        x0 = max(0, x - thickness // 2)
        x1 = min(w - 1, x + thickness // 2)
        if x0 > w - 1 or x1 < 0:
            continue
        floor[y, x0:x1] = brightness
        gt[y, x0:x1]    = 255

    bgr = floor
    return bgr, gt


def _draw_double_line(
    h: int, w: int,
    cx1: float, cx2: float,
    thickness: int = 20,
    brightness: int = 230,
) -> Tuple[np.ndarray, np.ndarray]:
    f1, g1 = _draw_line_mask(h, w, cx1, thickness=thickness, brightness=brightness)
    _, g2  = _draw_line_mask(h, w, cx2, thickness=thickness, brightness=brightness)
    gt = np.clip(g1.astype(np.int32) + g2.astype(np.int32), 0, 255).astype(np.uint8)
    # Draw second line on same floor
    for y in range(h):
        x = int(round(cx2))
        x0 = max(0, x - thickness // 2)
        x1 = min(w - 1, x + thickness // 2)
        if 0 <= x0 < w and 0 <= x1 < w:
            f1[y, x0:x1] = brightness
    return f1, gt


# ── Scenario definitions ───────────────────────────────────────────────────────

@dataclass
class Scenario:
    name:        str
    bgr:         np.ndarray
    gt:          np.ndarray      # binary ground-truth lane mask (0 or 255)
    description: str


def build_scenarios() -> List[Scenario]:
    W, H = 640, 480
    cx   = W / 2

    scenarios = []

    # straight — line perfectly centred
    bgr, gt = _draw_line_mask(H, W, cx, angle_deg=0, thickness=22)
    scenarios.append(Scenario("straight", bgr, gt, "Vertical white line, centred"))

    # angled_left — line tilts 15° left at top
    bgr, gt = _draw_line_mask(H, W, cx, angle_deg=-15, thickness=22)
    scenarios.append(Scenario("angled_left", bgr, gt, "Line angled 15° left"))

    # angled_right — line tilts 15° right at top
    bgr, gt = _draw_line_mask(H, W, cx, angle_deg=15, thickness=22)
    scenarios.append(Scenario("angled_right", bgr, gt, "Line angled 15° right"))

    # offset right — line 60 px right of centre
    bgr, gt = _draw_line_mask(H, W, cx + 60, angle_deg=0, thickness=22)
    scenarios.append(Scenario("offset_right", bgr, gt, "Line 60px right of centre"))

    # offset left — line 60 px left of centre
    bgr, gt = _draw_line_mask(H, W, cx - 60, angle_deg=0, thickness=22)
    scenarios.append(Scenario("offset_left", bgr, gt, "Line 60px left of centre"))

    # noisy background — salt-and-pepper + low contrast
    bgr, gt = _draw_line_mask(H, W, cx, angle_deg=0, thickness=22, brightness=200)
    noise = np.zeros_like(bgr)
    sp_mask = np.random.rand(H, W) < 0.03   # 3% salt-pepper
    bgr[sp_mask] = 220
    scenarios.append(Scenario("noisy_bg", bgr, gt, "3% salt-pepper noise + low contrast"))

    # double line — two parallel white stripes
    bgr, gt = _draw_double_line(H, W, cx - 80, cx + 80, thickness=18, brightness=230)
    scenarios.append(Scenario("double_line", bgr, gt, "Two parallel lines"))

    # dim_line — very low brightness (near HSV threshold)
    bgr, gt = _draw_line_mask(H, W, cx, angle_deg=0, thickness=22, brightness=165)
    scenarios.append(Scenario("dim_line", bgr, gt, "Dim line (brightness=165, threshold edge)"))

    return scenarios


# ── Metric helpers ────────────────────────────────────────────────────────────

def compute_iou(pred: np.ndarray, gt: np.ndarray) -> float:
    inter = int(np.logical_and(pred > 0, gt > 0).sum())
    union = int(np.logical_or (pred > 0, gt > 0).sum())
    return inter / union if union > 0 else 0.0


def get_hsv_mask(bgr: np.ndarray, lf: LineFollower) -> np.ndarray:
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    m   = cv2.inRange(hsv,
                      np.array(lf.cfg.hsv.low,  dtype=np.uint8),
                      np.array(lf.cfg.hsv.high, dtype=np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    m = cv2.morphologyEx(m, cv2.MORPH_OPEN,  kernel)
    m = cv2.morphologyEx(m, cv2.MORPH_CLOSE, kernel)
    return m


# ── Main evaluation ───────────────────────────────────────────────────────────

@dataclass
class ScenarioResult:
    name:         str
    description:  str
    line_found:   bool
    iou:          float
    lateral_err:  float
    heading_err_deg: float
    det_px:       int
    gt_px:        int


def evaluate(scenarios: List[Scenario]) -> List[ScenarioResult]:
    lf = _make_lf("white")
    results = []

    for s in scenarios:
        lf.reset_pid()
        result = lf.tick(s.bgr.copy(), produce_overlay=True)
        det_mask = get_hsv_mask(s.bgr, lf)
        iou = compute_iou(det_mask, s.gt)

        sr = ScenarioResult(
            name         = s.name,
            description  = s.description,
            line_found   = result.line_found,
            iou          = round(iou, 4),
            lateral_err  = round(result.lateral_error, 2),
            heading_err_deg = round(math.degrees(result.heading_error), 2),
            det_px       = int((det_mask > 0).sum()),
            gt_px        = int((s.gt > 0).sum()),
        )
        results.append(sr)

        status = "FOUND" if result.line_found else "LOST"
        print(f"  [{status:5s}] {s.name:14s}  IoU={iou:.3f}  "
              f"lat={result.lateral_error:+7.1f}px  "
              f"hdg={math.degrees(result.heading_error):+6.1f}°")

    return results


# ── Report + plot ─────────────────────────────────────────────────────────────

def save_report(results: List[ScenarioResult]) -> None:
    out = [asdict(r) for r in results]
    found = sum(1 for r in results if r.line_found)
    ious  = [r.iou for r in results]
    summary = {
        "total_scenarios" : len(results),
        "found_rate"      : round(found / len(results), 4),
        "iou_mean"        : round(float(np.mean(ious)),       4),
        "iou_median"      : round(float(np.median(ious)),     4),
        "iou_min"         : round(float(np.min(ious)),        4),
        "scenarios"       : out,
    }
    path = _THIS / "offline_test_report.json"
    with open(path, "w") as f:
        json.dump(summary, f, indent=2)
    print(f"\n  Report  → {path}")
    return summary


def save_plot(scenarios: List[Scenario], results: List[ScenarioResult]) -> None:
    try:
        import matplotlib.pyplot as plt
        import matplotlib.gridspec as gridspec
    except ImportError:
        print("matplotlib not available — skipping plot.")
        return

    lf = _make_lf("white")
    n  = len(scenarios)
    fig = plt.figure(figsize=(16, 3.2 * n))
    fig.suptitle("LineFollower Offline Synthetic Test", fontsize=14, fontweight="bold")
    gs  = gridspec.GridSpec(n, 4, figure=fig, hspace=0.45, wspace=0.25)

    for i, (s, r) in enumerate(zip(scenarios, results)):
        lf.reset_pid()
        res = lf.tick(s.bgr.copy(), produce_overlay=True)
        det_mask = get_hsv_mask(s.bgr, lf)

        # Col 0: original image
        ax = fig.add_subplot(gs[i, 0])
        ax.imshow(cv2.cvtColor(s.bgr, cv2.COLOR_BGR2RGB))
        ax.set_title(f"{s.name}", fontsize=8, fontweight="bold")
        ax.axis("off")

        # Col 1: overlay from LineFollower
        ax = fig.add_subplot(gs[i, 1])
        if res.overlay is not None:
            ax.imshow(cv2.cvtColor(res.overlay, cv2.COLOR_BGR2RGB))
            ax.set_title("LF overlay", fontsize=8)
        else:
            ax.imshow(cv2.cvtColor(s.bgr, cv2.COLOR_BGR2RGB))
            ax.set_title("no overlay", fontsize=8)
        ax.axis("off")

        # Col 2: HSV detection mask
        ax = fig.add_subplot(gs[i, 2])
        ax.imshow(det_mask, cmap="Greens")
        ax.set_title(f"HSV mask  ({r.det_px}px)", fontsize=8)
        ax.axis("off")

        # Col 3: GT mask overlay comparison
        ax = fig.add_subplot(gs[i, 3])
        comp = np.zeros((*s.gt.shape, 3), dtype=np.uint8)
        comp[:, :, 1] = s.gt                       # GT = green
        comp[:, :, 0] = det_mask                   # detected = blue (overlap = cyan)
        ax.imshow(comp)
        status = "✓" if r.line_found else "✗"
        ax.set_title(f"{status} IoU={r.iou:.3f}  lat={r.lateral_err:+.0f}px  hdg={r.heading_err_deg:+.1f}°",
                     fontsize=7.5)
        ax.axis("off")

    out_png = _THIS / "offline_test_plot.png"
    fig.savefig(out_png, dpi=100, bbox_inches="tight")
    print(f"  Plot    → {out_png}")
    plt.close(fig)


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    print("=" * 60)
    print("  K-Patrol LineFollower — Offline Synthetic Test")
    print("=" * 60)

    scenarios = build_scenarios()
    print(f"\nRunning {len(scenarios)} scenarios …\n")
    results   = evaluate(scenarios)

    ious  = [r.iou for r in results]
    found = sum(1 for r in results if r.line_found)

    print("\n" + "═" * 60)
    print("  SUMMARY")
    print("═" * 60)
    print(f"  Scenarios    : {len(results)}")
    print(f"  Found rate   : {found}/{len(results)} ({found/len(results)*100:.0f}%)")
    print(f"  IoU mean     : {np.mean(ious):.4f}")
    print(f"  IoU median   : {np.median(ious):.4f}")
    print(f"  IoU min      : {np.min(ious):.4f}")
    print("═" * 60)

    # Pass/fail thresholds
    passed = all(r.iou >= 0.20 or not r.line_found for r in results)
    # dim_line intentionally near threshold; double line is edge case
    critical = [r for r in results
                if r.name not in ("dim_line", "double_line") and r.iou < 0.20]
    if critical:
        print("\n  WARN: low IoU on critical scenarios:")
        for r in critical:
            print(f"    {r.name}: IoU={r.iou:.3f}")
    else:
        print("\n  All critical scenarios passed IoU ≥ 0.20  ✓")

    save_report(results)
    save_plot(scenarios, results)


if __name__ == "__main__":
    main()
