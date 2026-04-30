"""
line_follower.py — Camera-based floor line following for K-Patrol Mecanum.

Pipeline
--------
1. Capture BGR frame from camera (any OpenCV source).
2. Convert to HSV, threshold for floor tape colour.
3. Apply Bird's-Eye View (BEV) homography to remove perspective.
4. Find the line's spine via column-wise centroid scan.
5. Fit a 1-D polynomial to the spine → lateral_error, heading_error.
6. PD controller → (vy, wz) correction; forward speed vx held constant.
7. Annotate the frame with overlay (line, target path, error bar) and
   encode as JPEG for the MJPEG stream.

Coordinate system (matches reactive_nav / firmware MEC:)
    vx  +forward
    vy  +strafe-right (moves robot to the right, corrects left-drift)
    wz  +rotate-left  (CCW)

Target
------
Robot drives 10 cm to the left of the detected line.
  target_offset_px = (wheel_half_track_cm + 10) * bev_px_per_cm
Set wheel_half_track_cm to half your robot's wheel-to-wheel distance.

Calibration
-----------
BEV calibration points are defined in the config as four pixel
co-ordinates in the original camera frame that form a known ground
rectangle (measured physically with a ruler).  Tune bev_src_pts and
bev_dst_pts for your camera mounting.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import List, Optional, Tuple

import cv2
import numpy as np


# ── Colour thresholds (HSV) ────────────────────────────────────────────────────
# Default: white/light-grey tape on a dark floor.
# Override for coloured tape by changing LineFollowerConfig.

@dataclass
class HSVRange:
    low:  Tuple[int, int, int] = (0,   0, 160)   # white tape (any hue, low sat, high val)
    high: Tuple[int, int, int] = (180, 60, 255)


# ── BEV calibration ───────────────────────────────────────────────────────────
# src_pts: four corners of a known ground rectangle, in image pixels (TL,TR,BR,BL)
# dst_pts: where those corners map to in the BEV (unwarped) image.
# default assumes 640×480 camera, roughly 80×60 cm visible ground patch.

_DEFAULT_BEV_SRC = np.float32([
    [160, 240],   # top-left  in camera  (far-left)
    [480, 240],   # top-right in camera  (far-right)
    [580, 440],   # bottom-right (near-right)
    [ 60, 440],   # bottom-left  (near-left)
])
_DEFAULT_BEV_DST = np.float32([
    [  0,   0],   # top-left in BEV
    [400,   0],   # top-right
    [400, 480],   # bottom-right
    [  0, 480],   # bottom-left
])
_BEV_W, _BEV_H = 400, 480


# ── Config ─────────────────────────────────────────────────────────────────────

@dataclass
class LineFollowerConfig:
    # Colour
    hsv:               HSVRange    = field(default_factory=HSVRange)

    # BEV homography (pixel coordinates in raw camera frame)
    bev_src_pts:       np.ndarray  = field(default_factory=lambda: _DEFAULT_BEV_SRC.copy())
    bev_dst_pts:       np.ndarray  = field(default_factory=lambda: _DEFAULT_BEV_DST.copy())
    bev_w:             int         = _BEV_W
    bev_h:             int         = _BEV_H

    # Physical calibration
    bev_px_per_cm:     float       = 5.0    # pixels per centimetre in BEV output
    wheel_half_track_cm: float     = 15.0   # half the wheel-to-wheel distance (cm)
    target_left_offset_cm: float   = 10.0   # robot drives this far LEFT of the line

    # Controller gains
    kp_lateral:  float = 0.40   # P gain for lateral error → vy
    kd_lateral:  float = 0.08   # D gain for lateral error → vy
    kp_heading:  float = 0.30   # P gain for heading error → wz
    kd_heading:  float = 0.06   # D gain for heading error → wz

    # Speed
    base_vx:     int   = 60     # forward speed when line is found (0–127)
    lost_vx:     int   = 0      # forward speed when line is lost
    base_spd:    int   = 80     # absolute speed passed to ESP32 MEC: spd

    # Line detection
    min_line_pixels: int = 20   # minimum white pixels per column to count
    scan_cols:       int = 20   # number of vertical columns to scan
    min_blob_area:   int = 500  # minimum contiguous HSV blob area (px²) — rejects curb/vegetation noise

    # Heading-hold fallback: when line is lost hold last steering for N frames
    # before stopping (~0.5 s at 20 Hz). Set to 0 to disable.
    lost_grace_frames: int = 10

    # Overlay appearance
    overlay_alpha:   float = 0.55   # blend strength of annotation layer


# ── Result ─────────────────────────────────────────────────────────────────────

@dataclass
class LineResult:
    vx:            int
    vy:            int
    wz:            int
    spd:           int
    line_found:    bool
    lateral_error: float   # pixels, + = line is to the right of target
    heading_error: float   # radians, + = line angled left relative to robot
    overlay:       Optional[np.ndarray]   # BGR frame with annotations, or None


# ── Main class ─────────────────────────────────────────────────────────────────

class LineFollower:
    """
    Camera floor-line follower.

    Usage:
        cam = cv2.VideoCapture(0)
        lf  = LineFollower()
        ret, frame = cam.read()
        result = lf.tick(frame)
        # result.vx/vy/wz/spd for motor command
        # result.overlay for MJPEG stream
    """

    def __init__(self, config: Optional[LineFollowerConfig] = None):
        self.cfg = config or LineFollowerConfig()
        self._M: np.ndarray = cv2.getPerspectiveTransform(
            self.cfg.bev_src_pts, self.cfg.bev_dst_pts
        )
        # PD state
        self._prev_lat_err: float = 0.0
        self._prev_hdg_err: float = 0.0
        self._prev_t: float = time.monotonic()

        # Pre-compute target X in BEV image
        # Robot centre in BEV = horizontal centre (bev_w/2)
        # Target offset is "drive 10 cm LEFT of line"
        # In BEV, vy+ = robot moves right, so to be LEFT of line, the line
        # must be to the RIGHT of robot centre by (wheel_half_track + offset) px.
        self._target_offset_px: float = (
            (self.cfg.wheel_half_track_cm + self.cfg.target_left_offset_cm)
            * self.cfg.bev_px_per_cm
        )

        # Heading-hold state
        self._lost_count: int = 0
        self._last_vy:    int = 0
        self._last_wz:    int = 0

    def tick(
        self,
        frame: np.ndarray,
        produce_overlay: bool = True,
    ) -> LineResult:
        cfg = self.cfg
        now = time.monotonic()
        dt  = max(0.001, now - self._prev_t)
        self._prev_t = now

        # ── Step 1: HSV threshold ─────────────────────────────────────────────
        hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array(cfg.hsv.low,  dtype=np.uint8),
                           np.array(cfg.hsv.high, dtype=np.uint8))

        # Small morphology to clean noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Reject small fragmented blobs — curb stones / vegetation during turns
        if cfg.min_blob_area > 0:
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not any(cv2.contourArea(c) > cfg.min_blob_area for c in cnts):
                mask[:] = 0

        # ── Step 2: BEV warp ──────────────────────────────────────────────────
        bev = cv2.warpPerspective(mask, self._M, (cfg.bev_w, cfg.bev_h))

        # ── Step 3: Column-wise centroid scan ─────────────────────────────────
        col_w  = cfg.bev_w // cfg.scan_cols
        spine: List[Tuple[float, float]] = []   # (x_centre, y_centre) in BEV

        for i in range(cfg.scan_cols):
            x0, x1 = i * col_w, (i + 1) * col_w
            col_mask = bev[:, x0:x1]
            ys, xs = np.where(col_mask > 128)
            if len(ys) < cfg.min_line_pixels:
                continue
            cx = float(xs.mean()) + x0
            cy = float(ys.mean())
            spine.append((cx, cy))

        if len(spine) < 3:
            self._lost_count += 1
            overlay = self._draw_overlay(frame, bev, [], None, None) if produce_overlay else None
            # Heading-hold: keep last steering for grace period to bridge
            # brief occlusions during turns before stopping.
            if self._lost_count <= cfg.lost_grace_frames and (self._last_vy != 0 or self._last_wz != 0):
                hold_vx = max(0, cfg.base_vx // 2)
                return LineResult(hold_vx, self._last_vy, self._last_wz, cfg.base_spd,
                                  False, 0.0, 0.0, overlay)
            # Past grace period: full stop, reset PD state
            self._prev_lat_err = 0.0
            self._prev_hdg_err = 0.0
            return LineResult(cfg.lost_vx, 0, 0, cfg.base_spd,
                              False, 0.0, 0.0, overlay)

        spine_arr = np.array(spine)
        xs_arr    = spine_arr[:, 0]
        ys_arr    = spine_arr[:, 1]

        # ── Step 4: Line fit (1st-order polynomial x = a*y + b) ───────────────
        coeffs = np.polyfit(ys_arr, xs_arr, 1)   # x as function of y (vertical)
        a, b   = float(coeffs[0]), float(coeffs[1])

        # Robot is at bottom of BEV → evaluate line at y = bev_h
        line_x_at_robot = a * cfg.bev_h + b
        robot_x         = cfg.bev_w / 2.0

        # lateral_error: positive = line is to the right of the target position
        # We want line at (robot_x + target_offset_px)
        target_x        = robot_x + self._target_offset_px
        lateral_error   = line_x_at_robot - target_x   # + = line right of target

        # heading_error: slope of line in BEV; ideal = vertical (slope ~0)
        # angle in radians relative to vertical axis
        heading_error   = float(np.arctan(a))   # + = line leans left

        # ── Step 5: PD control ────────────────────────────────────────────────
        d_lat = (lateral_error   - self._prev_lat_err) / dt
        d_hdg = (heading_error   - self._prev_hdg_err) / dt

        self._prev_lat_err = lateral_error
        self._prev_hdg_err = heading_error

        # vy: + = strafe right to reduce left drift (when lateral_error > 0,
        # line is to the right of target → robot drifted left → steer right)
        vy_raw = cfg.kp_lateral * lateral_error + cfg.kd_lateral * d_lat
        # wz: + = rotate left; when heading_error > 0, line leans left → rotate left
        wz_raw = cfg.kp_heading * heading_error + cfg.kd_heading * d_hdg

        vy = int(max(-127, min(127, vy_raw)))
        wz = int(max(-127, min(127, wz_raw)))

        # Line found — reset grace state, save last known steering
        self._lost_count = 0
        self._last_vy    = vy
        self._last_wz    = wz

        overlay = (
            self._draw_overlay(frame, bev, spine, line_x_at_robot, target_x)
            if produce_overlay else None
        )

        return LineResult(cfg.base_vx, vy, wz, cfg.base_spd,
                          True, lateral_error, heading_error, overlay)

    # ── Overlay renderer ──────────────────────────────────────────────────────

    def _draw_overlay(
        self,
        raw_frame:       np.ndarray,
        bev:             np.ndarray,
        spine:           List[Tuple[float, float]],
        line_x_robot:    Optional[float],
        target_x:        Optional[float],
    ) -> np.ndarray:
        cfg   = self.cfg
        out   = raw_frame.copy().astype(np.float32)
        layer = np.zeros_like(out)

        # ── BEV thumbnail (top-right corner, 200×150 px) ──────────────────────
        bev_bgr   = cv2.cvtColor(bev, cv2.COLOR_GRAY2BGR).astype(np.float32)
        thumb_h, thumb_w = 150, 200
        bev_small = cv2.resize(bev_bgr, (thumb_w, thumb_h))
        h_frame, w_frame = raw_frame.shape[:2]
        tx0 = w_frame - thumb_w - 8
        ty0 = 8
        out[ty0:ty0+thumb_h, tx0:tx0+thumb_w] = bev_small

        # ── Draw detected spine on raw frame using inverse homography ─────────
        Minv = cv2.getPerspectiveTransform(self.cfg.bev_dst_pts, self.cfg.bev_src_pts)
        for (bx, by) in spine:
            pts_bev = np.array([[[bx, by]]], dtype=np.float32)
            pts_raw = cv2.perspectiveTransform(pts_bev, Minv)
            px, py  = int(pts_raw[0, 0, 0]), int(pts_raw[0, 0, 1])
            cv2.circle(layer, (px, py), 4, (0, 255, 120), -1)

        # ── Target path line (vertical stripe at target_x in BEV → raw) ─────
        if target_x is not None:
            for yy in range(0, cfg.bev_h, 20):
                pts_bev = np.array([[[target_x, float(yy)]]], dtype=np.float32)
                pts_raw = cv2.perspectiveTransform(pts_bev, Minv)
                px, py  = int(pts_raw[0, 0, 0]), int(pts_raw[0, 0, 1])
                if 0 <= px < w_frame and 0 <= py < h_frame:
                    cv2.circle(layer, (px, py), 3, (0, 140, 255), -1)

        # ── Error bar at bottom of raw frame ─────────────────────────────────
        if line_x_robot is not None and target_x is not None:
            bar_cx = w_frame // 2
            bar_y  = h_frame - 20
            lat_px = int((line_x_robot - target_x) * w_frame / cfg.bev_w * 0.5)
            colour = (0, 80, 255) if abs(lat_px) > 15 else (0, 200, 80)
            cv2.arrowedLine(layer,
                            (bar_cx, bar_y),
                            (bar_cx + lat_px, bar_y),
                            colour, 3, tipLength=0.3)
            cv2.line(layer, (bar_cx, bar_y - 8), (bar_cx, bar_y + 8), (255, 255, 255), 1)

        # ── Status text ───────────────────────────────────────────────────────
        label = "LINE OK" if spine else "LINE LOST"
        col   = (0, 220, 60) if spine else (0, 60, 255)
        cv2.putText(layer, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, col, 2)

        # ── Blend and return ──────────────────────────────────────────────────
        result = cv2.addWeighted(out, 1.0, layer, cfg.overlay_alpha, 0)
        return result.astype(np.uint8)

    def recalibrate_bev(
        self,
        src_pts: np.ndarray,
        dst_pts: np.ndarray,
        bev_w:   int,
        bev_h:   int,
    ) -> None:
        self.cfg.bev_src_pts = src_pts.copy()
        self.cfg.bev_dst_pts = dst_pts.copy()
        self.cfg.bev_w = bev_w
        self.cfg.bev_h = bev_h
        self._M = cv2.getPerspectiveTransform(src_pts, dst_pts)

    def reset_pid(self) -> None:
        self._prev_lat_err = 0.0
        self._prev_hdg_err = 0.0
        self._prev_t = time.monotonic()
        self._lost_count = 0
        self._last_vy    = 0
        self._last_wz    = 0


__all__ = ["LineFollower", "LineFollowerConfig", "LineResult", "HSVRange"]
