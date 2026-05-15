#!/usr/bin/env python3
"""
line_follow_sim.py — closed-loop simulator for LineFollower.

Purpose
-------
Verify the camera-based line follower **without** the robot or a camera.
We synthesise a top-down world that contains a painted line, render the
exact view the on-board camera would see (inverse of the BEV homography),
feed that frame into the production LineFollower, then integrate the
returned (vx, vy, wz) command back into a kinematic robot pose.  The
resulting closed loop reveals every layer of the pipeline:

    ┌──────────┐  warp    ┌────────────┐ inv-perspect ┌────────────┐
    │ World map│ ───────► │ ground BEV │ ───────────► │ camera view│
    └──────────┘          └────────────┘              └─────┬──────┘
                                                            ▼
                                                    LineFollower.tick
                                                            │
                                              vx, vy, wz, overlay
                                                            ▼
                                                  pose ← integrate

Display (single OpenCV window, four panes):
    TL : top-down world + robot trail + heading triangle + grid
    TR : synthetic camera frame (what the Pi camera would record)
    BL : LineFollower overlay (HSV→BEV→spine→PD annotated)
    BR : telemetry HUD (pose, twist, errors, line state)

Scenarios:  --map straight | s_curve | oval | l_turn | zigzag

Modes:
    Default GUI mode             — interactive, q/r/space/s
    --headless --max-frames N    — log-only run (CI / regression)
    --record out.mp4             — write composite to MP4 (requires GUI)

Tuning levers (all cmdline):
    --v-max-cms / --w-max-dps    physical mapping of int8 firmware units
    --sim-hz                     simulation update rate
    --start-offset-cm            initial lateral error to test recovery
    --map-line-cm                painted line width (real tape ≈ 4 cm)

The ToF emergency cut-off is left disabled (no provider wired) so the
simulator focuses on the vision/PD pipeline.  Real robot tests should
verify that path separately — see `tools/line_follow_diagnose.py`.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from collections import deque
from typing import Deque, Optional, Tuple

import cv2
import numpy as np

# Allow `python3 tools/line_follow_sim.py` from inside pi-controller/
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from navigation import LineFollower, LineFollowerConfig, HSVRange  # noqa: E402


# ── World canvas (top-down ground truth) ─────────────────────────────────────
# 5 px / cm matches the LineFollower default bev_px_per_cm so a 1500×1500
# canvas represents a 300 cm × 300 cm patch of floor.  Plenty of room for the
# robot to wander without hitting the edge during a normal test.
WORLD_PX_PER_CM = 5.0
WORLD_W = 1500
WORLD_H = 1500


# ── Map generation ────────────────────────────────────────────────────────────

def _draw_grid(img: np.ndarray, step_px: int = 50, color=(35, 35, 35)) -> None:
    """Faint cm grid so the trail panel reads as physical distance."""
    h, w = img.shape[:2]
    for x in range(0, w, step_px):
        cv2.line(img, (x, 0), (x, h), color, 1)
    for y in range(0, h, step_px):
        cv2.line(img, (0, y), (w, y), color, 1)


def make_world(kind: str, line_width_cm: float) -> Tuple[np.ndarray, Tuple[float, float, float]]:
    """Generate a top-down BGR world image plus a sane initial robot pose
    (rx_world, ry_world, psi_rad). Heading 0 = robot facing image-up."""
    img = np.zeros((WORLD_H, WORLD_W, 3), dtype=np.uint8)
    _draw_grid(img)
    line_w_px = max(2, int(round(line_width_cm * WORLD_PX_PER_CM)))
    cx, cy = WORLD_W // 2, WORLD_H // 2

    # Small initial yaw tilt so a near-vertical line crosses ≥3 BEV scan
    # columns (col_w = 20 px). Without tilt, a thin straight line crosses
    # only 2 columns and the spine ≥3-points requirement fails on frame 1.
    tilt = float(np.deg2rad(4.0))

    # Robot's target-lane offset = (wheel_half_track + target_left_offset)
    # = 25 cm LEFT of the line, mirrored from LineFollowerConfig defaults.
    # We start the robot ON its target lane so a "fair" measurement of
    # tracking error reflects PD performance rather than initial-pose
    # mismatch. Use --start-offset-cm to perturb this and stress the loop.
    target_lane_px = 25 * WORLD_PX_PER_CM   # 125 px LEFT of the painted line

    if kind == "straight":
        cv2.line(img, (cx, 60), (cx, WORLD_H - 60), (255, 255, 255), line_w_px)
        start = (float(cx - target_lane_px), float(WORLD_H - 200), tilt)

    elif kind == "s_curve":
        # Period = 1100 px = WORLD_H - 400, exactly one full period between
        # the wrap-vertical re-entry rows (y=200 and y=WORLD_H-200=1300), so
        # the wrap discontinuity is invisible. Amplitude tuned so peak slope
        # arctan(2π·amp/period) ≈ 37° — a realistic curve the BEV-PD pipeline
        # can track without saturating.
        period = 1100
        ys = np.arange(60, WORLD_H - 60, 2)
        xs = (cx + 130 * np.sin(2 * np.pi * ys / period)).astype(np.int32)
        pts = np.stack([xs, ys.astype(np.int32)], axis=1)
        cv2.polylines(img, [pts], False, (255, 255, 255), line_w_px)
        line_x_at_start = cx + 130 * float(np.sin(2 * np.pi * (WORLD_H - 200) / period))
        start = (float(line_x_at_start - target_lane_px), float(WORLD_H - 200), tilt)

    elif kind == "oval":
        # Larger ellipse so the inner-track radius (semi-minor − target_lane)
        # is generous enough that the BEV polyfit sees a near-straight line
        # instead of a heavily curved one. Inner radius ≈ 600−125 = 95 cm.
        cv2.ellipse(img, (cx, cy), (600, 700), 0, 0, 360,
                    (255, 255, 255), line_w_px)
        # Robot at bottom-of-oval facing +x ("right"). Robot-frame LEFT
        # points to image-up (-y), so the target lane sits 25 cm above the
        # painted line, i.e. INSIDE the oval.
        start = (float(cx), float(cy + 700 - 20 - target_lane_px),
                 float(np.deg2rad(90.0)))

    elif kind == "l_turn":
        # Vertical leg from centre down to near-bottom, then horizontal leg
        # from centre out to right edge. Robot must start *on its target
        # lane* (25 cm LEFT of the vertical leg) and point up.
        cv2.line(img, (cx, cy), (cx, WORLD_H - 100), (255, 255, 255), line_w_px)
        cv2.line(img, (cx, cy), (WORLD_W - 100, cy), (255, 255, 255), line_w_px)
        start = (float(cx - target_lane_px), float(WORLD_H - 200), tilt)

    elif kind == "zigzag":
        # Triangle wave instead of square — square wave is uncatchable for
        # a smooth PD controller (infinite curvature).
        # Period = 550 (1100 / 2) to align wrap-vertical re-entry phase (see
        # s_curve). Amplitude sized so peak slope 4·amp/period ≈ tan(43°) — a
        # realistic zigzag the BEV pipeline can track.
        ys = np.arange(60, WORLD_H - 60, 2)
        period = 550
        amp = 130
        # triangle wave amplitude in [-1, 1]
        t_zz = (ys % period) / period
        tri = np.where(t_zz < 0.5, 4 * t_zz - 1, 3 - 4 * t_zz)
        xs = (cx + amp * tri).astype(np.int32)
        pts = np.stack([xs, ys.astype(np.int32)], axis=1)
        cv2.polylines(img, [pts], False, (255, 255, 255), line_w_px)
        # Line x at start: triangle wave evaluated at y=WORLD_H-200=1300
        t_start = ((WORLD_H - 200) % period) / period
        tri_start = (4 * t_start - 1) if t_start < 0.5 else (3 - 4 * t_start)
        line_x_at_start = cx + amp * tri_start
        start = (float(line_x_at_start - target_lane_px), float(WORLD_H - 200), tilt)

    else:
        raise ValueError(f"unknown map: {kind}")

    return img, start


# ── World ↔ BEV ↔ camera transforms ──────────────────────────────────────────

def world_to_bev(world: np.ndarray, rx: float, ry: float, psi_rad: float,
                 bev_w: int, bev_h: int) -> np.ndarray:
    """Extract the BEV view that the robot's downward-looking-virtual-camera
    would see.

    The 2×3 affine maps world pixels → BEV pixels such that:
        * (rx, ry)             → (bev_w/2, bev_h - 1)   robot at bottom-centre
        * forward unit (sin ψ, -cos ψ)  →  bev "up" = (0, -1)

    Heading convention:  ψ = 0 means robot points toward image-up (-y).
    Positive ψ rotates the heading clockwise (as seen on the world image).
    """
    cos_p = float(np.cos(psi_rad))
    sin_p = float(np.sin(psi_rad))
    tx = -rx * cos_p - ry * sin_p + bev_w * 0.5
    ty =  rx * sin_p - ry * cos_p + (bev_h - 1)
    M = np.array([
        [ cos_p, sin_p, tx],
        [-sin_p, cos_p, ty],
    ], dtype=np.float32)
    return cv2.warpAffine(world, M, (bev_w, bev_h),
                          flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))


def bev_to_camera(bev: np.ndarray, M_bev_to_cam: np.ndarray,
                  cam_w: int = 640, cam_h: int = 480,
                  blur_sigma: float = 1.5,
                  noise_std: float = 4.0) -> np.ndarray:
    """Re-apply forward perspective so the LineFollower sees what a real
    camera would see (un-warped trapezoidal floor patch).

    A small Gaussian blur + uniform noise emulates lens MTF and sensor
    noise.  Without it, painted-line edges in the synthetic frame are
    pixel-perfect — sharper than any physical camera — and the column
    scanner sees one or two spine points instead of the spread it would
    see in reality.  Tuned so HSV thresholding still recovers the line
    cleanly while edges bleed into ≥1 neighbouring scan column.
    """
    cam = cv2.warpPerspective(bev, M_bev_to_cam, (cam_w, cam_h),
                              flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))
    if blur_sigma > 0:
        # Kernel size auto-derived from sigma (rule-of-thumb 6σ+1, odd).
        k = max(3, int(round(blur_sigma * 6)) | 1)
        cam = cv2.GaussianBlur(cam, (k, k), blur_sigma)
    if noise_std > 0:
        noise = np.random.normal(0.0, noise_std, cam.shape).astype(np.float32)
        cam = np.clip(cam.astype(np.float32) + noise, 0, 255).astype(np.uint8)
    return cam


# ── Pose integration ─────────────────────────────────────────────────────────

def integrate_pose(rx: float, ry: float, psi_rad: float,
                   vx: int, vy: int, wz: int, dt: float,
                   v_max_cms: float, w_max_dps: float,
                   px_per_cm: float) -> Tuple[float, float, float]:
    """One Euler step using the firmware's int8 twist convention.

    Twist sign (matches navigation/line_follower.py docstring):
        vx > 0  forward
        vy > 0  strafe RIGHT
        wz > 0  rotate LEFT (CCW)

    Image convention used in this sim:
        ψ = 0 → forward = (0, -1)  i.e. up in the image
        ψ increasing CW → forward rotates toward image-+x
        Therefore ψ_dot = -wz_rad/s
    """
    fwd_cms = (vx / 127.0) * v_max_cms
    str_cms = (vy / 127.0) * v_max_cms
    yaw_dps = (wz / 127.0) * w_max_dps

    fwd_pxs = fwd_cms * px_per_cm
    str_pxs = str_cms * px_per_cm

    fx = float(np.sin(psi_rad))
    fy = -float(np.cos(psi_rad))
    rxu = float(np.cos(psi_rad))   # right unit (90° CW from forward)
    ryu = float(np.sin(psi_rad))

    rx_new = rx + (fx * fwd_pxs + rxu * str_pxs) * dt
    ry_new = ry + (fy * fwd_pxs + ryu * str_pxs) * dt
    psi_new = psi_rad - np.deg2rad(yaw_dps) * dt
    return rx_new, ry_new, psi_new


# ── Display helpers ──────────────────────────────────────────────────────────

def render_world_panel(world: np.ndarray, rx: float, ry: float, psi_rad: float,
                       trail: Deque[Tuple[float, float]],
                       size: int = 600) -> np.ndarray:
    scale = size / float(WORLD_W)
    view = cv2.resize(world, (size, int(WORLD_H * scale)),
                      interpolation=cv2.INTER_AREA)
    if view.shape[0] != size:  # square pad
        canvas = np.zeros((size, size, 3), dtype=np.uint8)
        canvas[:view.shape[0], :view.shape[1]] = view
        view = canvas

    if len(trail) > 1:
        pts = np.array([(int(x * scale), int(y * scale)) for x, y in trail],
                       dtype=np.int32)
        cv2.polylines(view, [pts], False, (0, 255, 255), 2)

    cx_img, cy_img = rx * scale, ry * scale
    s = 14
    fx, fy = np.sin(psi_rad), -np.cos(psi_rad)
    rxu, ryu = np.cos(psi_rad), np.sin(psi_rad)
    tip = (int(cx_img + fx * s * 1.2),
           int(cy_img + fy * s * 1.2))
    bl  = (int(cx_img - fx * s * 0.6 - rxu * s * 0.7),
           int(cy_img - fy * s * 0.6 - ryu * s * 0.7))
    br  = (int(cx_img - fx * s * 0.6 + rxu * s * 0.7),
           int(cy_img - fy * s * 0.6 + ryu * s * 0.7))
    cv2.fillPoly(view, [np.array([tip, bl, br], dtype=np.int32)], (0, 0, 255))
    cv2.circle(view, (int(cx_img), int(cy_img)), 3, (0, 200, 255), -1)
    cv2.putText(view, "WORLD (top-down)", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
    return view


def render_camera_panel(cam_frame: np.ndarray, size: int = 600) -> np.ndarray:
    h, w = cam_frame.shape[:2]
    scale = min(size / w, size / h)
    resized = cv2.resize(cam_frame, (int(w * scale), int(h * scale)),
                         interpolation=cv2.INTER_AREA)
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    y0 = (size - resized.shape[0]) // 2
    x0 = (size - resized.shape[1]) // 2
    canvas[y0:y0 + resized.shape[0], x0:x0 + resized.shape[1]] = resized
    cv2.putText(canvas, "SYNTH CAMERA (640x480)", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
    return canvas


def render_overlay_panel(overlay: Optional[np.ndarray],
                         size: int = 600) -> np.ndarray:
    canvas = np.zeros((size, size, 3), dtype=np.uint8)
    if overlay is None:
        cv2.putText(canvas, "no overlay", (size // 2 - 60, size // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (120, 120, 120), 1, cv2.LINE_AA)
        return canvas
    h, w = overlay.shape[:2]
    scale = min(size / w, size / h)
    resized = cv2.resize(overlay, (int(w * scale), int(h * scale)),
                         interpolation=cv2.INTER_AREA)
    y0 = (size - resized.shape[0]) // 2
    x0 = (size - resized.shape[1]) // 2
    canvas[y0:y0 + resized.shape[0], x0:x0 + resized.shape[1]] = resized
    cv2.putText(canvas, "LINE-FOLLOWER OVERLAY", (8, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1, cv2.LINE_AA)
    return canvas


def render_hud_panel(stats: dict, size: int = 600) -> np.ndarray:
    panel = np.full((size, size, 3), 18, dtype=np.uint8)
    cv2.putText(panel, "TELEMETRY", (12, 28),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.line(panel, (12, 38), (size - 12, 38), (60, 60, 60), 1)

    rows = [
        ("map",          stats.get("map", "-")),
        ("frame",        f"{stats.get('frame', 0)}"),
        ("sim_t",        f"{stats.get('sim_t', 0.0):6.2f} s"),
        ("rt_fps",       f"{stats.get('fps', 0.0):5.1f}"),
        ("",             ""),
        ("pose x (cm)",  f"{stats.get('rx_cm', 0.0):+7.1f}"),
        ("pose y (cm)",  f"{stats.get('ry_cm', 0.0):+7.1f}"),
        ("heading",      f"{stats.get('psi_deg', 0.0):+7.1f} deg"),
        ("",             ""),
        ("line_found",   "YES" if stats.get("line_found") else "no"),
        ("recovery",     stats.get("recovery_mode") or "-"),
        ("lat_err",      f"{stats.get('lat_err_px', 0.0):+7.1f} px"),
        ("hdg_err",      f"{stats.get('hdg_err_deg', 0.0):+7.1f} deg"),
        ("curvature",    f"{stats.get('curvature', 0.0):7.2f}"),
        ("",             ""),
        ("vx (cmd)",     f"{stats.get('vx', 0):+4d}"),
        ("vy (cmd)",     f"{stats.get('vy', 0):+4d}"),
        ("wz (cmd)",     f"{stats.get('wz', 0):+4d}"),
        ("spd",          f"{stats.get('spd', 0):4d}"),
        ("",             ""),
        ("dist_to_line", f"{stats.get('dist_to_line_cm', 0.0):6.1f} cm"),
        ("track_loss",   f"{stats.get('lost_count', 0):4d} fr"),
    ]
    y = 64
    line_h = 24
    for label, value in rows:
        if not label and not value:
            y += 6
            continue
        col = (160, 160, 160)
        if label == "line_found":
            col = (60, 220, 60) if stats.get("line_found") else (60, 60, 220)
        cv2.putText(panel, f"{label:<14}", (16, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
        cv2.putText(panel, str(value), (200, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (235, 235, 235), 1, cv2.LINE_AA)
        y += line_h

    if stats.get("paused"):
        cv2.putText(panel, "PAUSED — press <space> to resume",
                    (16, size - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (60, 220, 220), 1, cv2.LINE_AA)
    else:
        cv2.putText(panel, "q quit  r reset  space pause  s step",
                    (16, size - 24), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (130, 130, 130), 1, cv2.LINE_AA)
    return panel


def compose_dashboard(world_panel: np.ndarray, cam_panel: np.ndarray,
                      overlay_panel: np.ndarray, hud_panel: np.ndarray
                      ) -> np.ndarray:
    top = cv2.hconcat([world_panel, cam_panel])
    bot = cv2.hconcat([overlay_panel, hud_panel])
    return cv2.vconcat([top, bot])


# ── Ground-truth distance (sim-only metric) ─────────────────────────────────

def dist_to_line_world(world_gray: np.ndarray, rx: float, ry: float,
                       search_radius_px: int = 100) -> float:
    """Approximate signed distance from the robot to the painted line, using
    a small windowed nearest-pixel search.  Returns +cm; -1.0 if no line in
    window.  This is sim ground-truth, separate from the LineFollower's
    BEV-derived `lateral_error`."""
    h, w = world_gray.shape
    x0 = max(0, int(rx) - search_radius_px)
    x1 = min(w, int(rx) + search_radius_px)
    y0 = max(0, int(ry) - search_radius_px)
    y1 = min(h, int(ry) + search_radius_px)
    patch = world_gray[y0:y1, x0:x1]
    ys, xs = np.where(patch > 200)
    if len(xs) == 0:
        return -1.0
    dx = xs + x0 - rx
    dy = ys + y0 - ry
    d_px = float(np.sqrt(np.min(dx * dx + dy * dy)))
    return d_px / WORLD_PX_PER_CM


# ── Simulation core ─────────────────────────────────────────────────────────

def run_simulation(args: argparse.Namespace) -> int:
    cfg = LineFollowerConfig()
    # The sim's white-on-black map matches the default HSV (white tape) — no
    # tweak needed.  If the user passes --tape-color we'd recolour here.

    lf = LineFollower(cfg)

    # Pre-compute BEV → camera homography once: it's the inverse of what
    # LineFollower applies internally.
    M_bev_to_cam = cv2.getPerspectiveTransform(cfg.bev_dst_pts, cfg.bev_src_pts)

    world, start_pose = make_world(args.map, args.map_line_cm)
    world_gray = cv2.cvtColor(world, cv2.COLOR_BGR2GRAY)

    # Apply user-requested initial lateral offset (in robot's right direction)
    rx, ry, psi = start_pose
    if args.start_offset_cm:
        off_px = args.start_offset_cm * WORLD_PX_PER_CM
        rx += off_px * np.cos(psi)
        ry += off_px * np.sin(psi)

    trail: Deque[Tuple[float, float]] = deque(maxlen=4000)
    dt = 1.0 / args.sim_hz
    sim_t = 0.0
    frame_idx = 0
    paused = False
    step_once = False
    last_wall = time.monotonic()
    fps_smooth = 0.0
    lost_streak = 0
    summary = {"frames": 0, "found": 0, "lost": 0, "max_dist_cm": 0.0,
               "max_track_err_cm": 0.0, "off_track_frames": 0,
               "tof_blocked": 0,
               "vy_sat": 0, "wz_sat": 0,
               "abs_vy_sum": 0.0, "abs_wz_sum": 0.0}

    writer = None
    if args.record:
        # Lazy create on first composed frame to know the size.
        pass

    win = "K-Patrol Line-Follow Simulator"
    if not args.headless:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)

    while True:
        if frame_idx >= args.max_frames > 0:
            break

        active_step = (not paused) or step_once
        step_once = False

        if active_step:
            # 1. Render synthetic camera frame from current pose
            bev_synth = world_to_bev(world, rx, ry, psi, cfg.bev_w, cfg.bev_h)
            cam_frame = bev_to_camera(bev_synth, M_bev_to_cam)

            # 2. Tick the production line follower
            res = lf.tick(cam_frame, produce_overlay=not args.headless or bool(args.record))

            # 3. Integrate pose with returned twist
            rx, ry, psi = integrate_pose(
                rx, ry, psi,
                int(res.vx), int(res.vy), int(res.wz),
                dt, args.v_max_cms, args.w_max_dps, WORLD_PX_PER_CM,
            )

            # Wrap around to keep the robot inside the world canvas — useful
            # for endless oval / straight runs.
            rx = float(np.clip(rx, 10.0, WORLD_W - 10.0))
            if args.wrap_vertical and ry < 200.0:
                # Teleport back to the start row. Reset rx/psi to start_pose so
                # the robot lands aligned with the painted line at re-entry —
                # critical for s_curve/zigzag whose line_x at y=200 differs
                # from line_x at y=WORLD_H-200. Reset PD state too, otherwise
                # the previous-frame error carries through the teleport and
                # produces a one-tick derivative spike.
                rx = float(start_pose[0])
                psi = float(start_pose[2])
                ry = float(WORLD_H - 200.0)
                trail.clear()
                lf.reset_pid()
            else:
                ry = float(np.clip(ry, 10.0, WORLD_H - 10.0))

            trail.append((rx, ry))
            sim_t += dt
            frame_idx += 1

            # Stats. d_cm = raw world-space distance to nearest line pixel.
            # tracking_err_cm = how far the robot is from the *configured*
            # target lane, which sits (wheel_half_track + target_left_offset)
            # cm = 25 cm LEFT of the line. We can't use LineFollower's own
            # lateral_error here because polyfit extrapolation goes wild when
            # the line curls outside the BEV's far range — values reach
            # thousands of px. World-space (d_cm − target_offset_cm) is
            # bounded and physically meaningful.
            d_cm = dist_to_line_world(world_gray, rx, ry)
            target_offset_cm = (cfg.wheel_half_track_cm
                                + cfg.target_left_offset_cm)
            tracking_err_cm = (
                abs(d_cm - target_offset_cm) if d_cm >= 0 else -1.0
            )
            summary["frames"] += 1
            if res.line_found:
                summary["found"] += 1
                lost_streak = 0
            else:
                summary["lost"] += 1
                lost_streak += 1
            if res.tof_blocked:
                summary["tof_blocked"] += 1
            if tracking_err_cm >= 0:
                summary["max_track_err_cm"] = max(
                    summary["max_track_err_cm"], tracking_err_cm)
                if tracking_err_cm > args.off_track_cm:
                    summary["off_track_frames"] += 1
            if d_cm >= 0:
                summary["max_dist_cm"] = max(summary["max_dist_cm"], d_cm)
            # Saturation telemetry: count frames where |vy| or |wz| ≥ 120
            # (within 5% of int8 saturation). High rates → unstable PD.
            if abs(int(res.vy)) >= 120:
                summary["vy_sat"] += 1
            if abs(int(res.wz)) >= 120:
                summary["wz_sat"] += 1
            summary["abs_vy_sum"] += abs(int(res.vy))
            summary["abs_wz_sum"] += abs(int(res.wz))

        else:
            # Paused: still need a cheap camera render so the dashboard isn't
            # blank, but skip LF.tick to avoid PD time-step drift.
            cam_frame = bev_to_camera(
                world_to_bev(world, rx, ry, psi, cfg.bev_w, cfg.bev_h),
                M_bev_to_cam,
            )
            res = None

        # ── Display ──
        now = time.monotonic()
        elapsed = now - last_wall
        last_wall = now
        if elapsed > 0:
            inst_fps = 1.0 / elapsed
            fps_smooth = 0.9 * fps_smooth + 0.1 * inst_fps if fps_smooth else inst_fps

        if not args.headless or args.record:
            world_panel = render_world_panel(world, rx, ry, psi, trail)
            cam_panel = render_camera_panel(cam_frame)
            overlay = res.overlay if res is not None else None
            overlay_panel = render_overlay_panel(overlay)
            hud_panel = render_hud_panel({
                "map": args.map,
                "frame": frame_idx,
                "sim_t": sim_t,
                "fps": fps_smooth,
                "rx_cm": (rx - WORLD_W / 2) / WORLD_PX_PER_CM,
                "ry_cm": (WORLD_H / 2 - ry) / WORLD_PX_PER_CM,
                "psi_deg": float(np.rad2deg(psi)),
                "line_found": res.line_found if res is not None else False,
                "recovery_mode": res.recovery_mode if res is not None else "",
                "lat_err_px": res.lateral_error if res is not None else 0.0,
                "hdg_err_deg": float(np.rad2deg(res.heading_error)) if res is not None else 0.0,
                "curvature": res.curvature if res is not None else 0.0,
                "vx": int(res.vx) if res is not None else 0,
                "vy": int(res.vy) if res is not None else 0,
                "wz": int(res.wz) if res is not None else 0,
                "spd": int(res.spd) if res is not None else 0,
                "dist_to_line_cm": d_cm if active_step and res is not None else 0.0,
                "lost_count": lost_streak,
                "paused": paused,
            })
            dashboard = compose_dashboard(world_panel, cam_panel,
                                          overlay_panel, hud_panel)

            if args.record and writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                writer = cv2.VideoWriter(args.record, fourcc, args.sim_hz,
                                         (dashboard.shape[1], dashboard.shape[0]))
            if writer is not None:
                writer.write(dashboard)

            if not args.headless:
                cv2.imshow(win, dashboard)
                key = cv2.waitKey(max(1, int(1000 / args.sim_hz))) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("r"):
                    rx, ry, psi = start_pose
                    if args.start_offset_cm:
                        off_px = args.start_offset_cm * WORLD_PX_PER_CM
                        rx += off_px * np.cos(psi)
                        ry += off_px * np.sin(psi)
                    trail.clear()
                    sim_t = 0.0
                    frame_idx = 0
                    summary = {k: 0 if isinstance(v, int) else 0.0 for k, v in summary.items()}
                    lf.reset_pid()
                    lost_streak = 0
                elif key == ord(" "):
                    paused = not paused
                elif key == ord("s"):
                    paused = True
                    step_once = True

        # Headless logging cadence
        if args.headless and active_step and frame_idx % args.log_every == 0:
            d_disp = d_cm if d_cm >= 0 else float("nan")
            te_disp = tracking_err_cm if tracking_err_cm >= 0 else float("nan")
            print(f"[t={sim_t:6.2f}s f={frame_idx:5d}] "
                  f"line={'Y' if res.line_found else 'n'} "
                  f"vx={int(res.vx):+4d} vy={int(res.vy):+4d} wz={int(res.wz):+4d}  "
                  f"lat={res.lateral_error:+6.1f}px "
                  f"hdg={np.rad2deg(res.heading_error):+5.1f}deg  "
                  f"trk={te_disp:5.1f}cm dist={d_disp:5.1f}cm  "
                  f"rec={res.recovery_mode or '-'}",
                  flush=True)

    if writer is not None:
        writer.release()
    if not args.headless:
        cv2.destroyAllWindows()

    # ── Summary ──
    n = max(1, summary["frames"])
    found_pct = 100.0 * summary["found"] / n
    off_pct = 100.0 * summary["off_track_frames"] / n
    print()
    print("─── simulation summary " + "─" * 48)
    print(f"  map                : {args.map}")
    print(f"  frames simulated   : {summary['frames']}")
    print(f"  line_found rate    : {found_pct:5.1f} %")
    print(f"  max distance/line  : {summary['max_dist_cm']:5.1f} cm")
    print(f"  max tracking error : {summary['max_track_err_cm']:5.1f} cm "
          f"(deviation from configured target lane)")
    print(f"  off-track (>{args.off_track_cm:.0f}cm)   : {off_pct:5.1f} %")
    print(f"  ToF emergency hits : {summary['tof_blocked']}")
    vy_sat_pct = 100.0 * summary["vy_sat"] / n
    wz_sat_pct = 100.0 * summary["wz_sat"] / n
    avg_vy = summary["abs_vy_sum"] / n
    avg_wz = summary["abs_wz_sum"] / n
    print(f"  |vy|≥120 saturation: {vy_sat_pct:5.1f} %  avg|vy|={avg_vy:5.1f}")
    print(f"  |wz|≥120 saturation: {wz_sat_pct:5.1f} %  avg|wz|={avg_wz:5.1f}")
    print("─" * 70)

    # Exit code: 0 if found_pct >= threshold and off-track ratio low
    ok = found_pct >= args.pass_found_pct and off_pct <= args.pass_offtrack_pct
    return 0 if ok else 2


# ── CLI ─────────────────────────────────────────────────────────────────────

def parse_args(argv: Optional[list] = None) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="Closed-loop simulator for the K-Patrol LineFollower.")
    p.add_argument("--map", default="s_curve",
                   choices=["straight", "s_curve", "oval", "l_turn", "zigzag"],
                   help="Painted-line map shape (default: s_curve).")
    p.add_argument("--map-line-cm", type=float, default=8.0,
                   help="Painted line width in cm (real tape ≈ 4 cm).")
    p.add_argument("--start-offset-cm", type=float, default=8.0,
                   help="Initial lateral offset right of line (tests recovery).")
    p.add_argument("--sim-hz", type=float, default=30.0,
                   help="Simulation tick rate (Hz).")
    p.add_argument("--v-max-cms", type=float, default=30.0,
                   help="Physical forward speed at vx=127 (cm/s).")
    p.add_argument("--w-max-dps", type=float, default=90.0,
                   help="Physical yaw rate at wz=127 (deg/s).")
    p.add_argument("--max-frames", type=int, default=0,
                   help="Stop after N frames (0 = run until 'q'). "
                        "Required for --headless.")
    p.add_argument("--headless", action="store_true",
                   help="No GUI; print log lines and exit.")
    p.add_argument("--log-every", type=int, default=15,
                   help="Headless: print one line every N frames (default 15).")
    p.add_argument("--record", default="",
                   help="Optional MP4 path to record the dashboard.")
    p.add_argument("--off-track-cm", type=float, default=15.0,
                   help="Threshold for 'off-track' frame in summary.")
    p.add_argument("--pass-found-pct", type=float, default=85.0,
                   help="Min line_found%% for exit code 0 (default 85).")
    p.add_argument("--pass-offtrack-pct", type=float, default=20.0,
                   help="Max off-track%% for exit code 0 (default 20).")
    p.add_argument("--wrap-vertical", action="store_true",
                   help="Teleport robot from top back to bottom on periodic "
                        "maps (straight/s_curve/zigzag) for endless long-run "
                        "demos. Trail resets on wrap.")
    args = p.parse_args(argv)
    if args.headless and args.max_frames <= 0:
        p.error("--headless requires --max-frames > 0")
    return args


def main(argv: Optional[list] = None) -> int:
    args = parse_args(argv)
    return run_simulation(args)


if __name__ == "__main__":
    sys.exit(main())
