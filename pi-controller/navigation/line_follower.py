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
from typing import Callable, List, Optional, Tuple

import cv2
import numpy as np

from .nav_utils import clamp


# Firmware MEC: protocol uses signed-byte (int8) velocity fields, so any
# (vy, wz) command must saturate inside [-127, 127] before the serial write.
_TWIST_LIMIT = 127


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
    # heading_error is in RADIANS (arctan of slope), so kp_heading must be
    # large enough that a typical ±0.2 rad tilt produces a non-zero int8
    # wz. Earlier values (0.30) were sized for degrees and silently
    # truncated to zero rotation in real use — symptom: robot strafes but
    # never rotates into curves. Verified in tools/line_follow_sim.py.
    kp_lateral:  float = 0.40   # P gain for lateral error → vy
    kd_lateral:  float = 0.08   # D gain for lateral error → vy
    kp_heading:  float = 8.0    # P gain for heading error (rad) → wz
    kd_heading:  float = 0.0    # D gain off — derivative on polyfit-derived slope
                                # is too noisy and induces a rotational limit cycle
                                # on mecanum platforms. Lateral_error already drives
                                # strafe correction; wz handles slow heading drift
                                # and curve following.
    heading_deadband_rad: float = 0.02  # Ignore heading_error below ~1°. On straight
                                # lines small slope noise (from polyfit on slightly
                                # tilted spine) would otherwise make wz nudge robot,
                                # which couples back into vy and amplifies oscillation.

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

    # Active rotation-recovery: after lost_grace_frames expire, rotate in place
    # toward the last steering direction for up to recovery_max_frames so the
    # camera can re-acquire a line that left the FOV during a sharp turn.
    # Set recovery_max_frames=0 to disable (full stop after grace).
    recovery_wz:          int = 45
    recovery_max_frames:  int = 40

    # 2nd-order line fit: lets heading_error capture the start of a curve
    # (lookahead) instead of the instantaneous slope at the robot. Falls back
    # to 1st order if fewer than 4 spine points or RankWarning.
    polyfit_order:        int   = 2
    # Where in the BEV (0..1, 0=top/far, 1=bottom/near robot) to evaluate the
    # lookahead slope. 0.4 = mid-far, 1.0 = directly under the robot.
    lookahead_y_ratio:    float = 0.45
    # Blend weight of lookahead heading vs. robot heading (0=use robot only,
    # 1=use lookahead only). 0.6 favours anticipating the curve.
    lookahead_blend:      float = 0.6
    # Adaptive lookahead: at higher commanded vx the robot covers more ground
    # per control tick, so it must look further ahead (smaller y_ratio) to keep
    # the same response time. effective_y_ratio = lookahead_y_ratio
    #   - lookahead_speed_gain * (last_vx / 127). Tuned so vx=127 shaves the
    # ratio by ~0.20 → 0.45 default → 0.25 at top speed (≈ twice as far). Set
    # to 0.0 to disable (fixed-distance lookahead, original behaviour).
    lookahead_speed_gain: float = 0.20
    # Curvature-aware slowdown: when |Δslope between near/lookahead| exceeds
    # threshold, scale vx down to curvature_min_vx_factor * base_vx. Slope is
    # dx/dy in BEV pixel units → ~0.3 already corresponds to a ~17° lean.
    curvature_threshold:      float = 0.30
    curvature_min_vx_factor:  float = 0.45

    # Optional turn-marker detection: a coloured square placed left/right of
    # the line that biases lateral_error toward that side, helping the robot
    # commit to a turn before the line bends. Disabled when turn_marker=False.
    turn_marker:               bool = False
    turn_marker_left_hsv_low:  Tuple[int, int, int] = (100,  80,  60)   # blue
    turn_marker_left_hsv_high: Tuple[int, int, int] = (130, 255, 255)
    turn_marker_right_hsv_low: Tuple[int, int, int] = (  0, 120,  80)   # red
    turn_marker_right_hsv_high:Tuple[int, int, int] = ( 12, 255, 255)
    turn_marker_min_area:      int   = 800
    turn_marker_bias_px:       float = 60.0    # added to lateral_error toward turn side
    turn_marker_hold_frames:   int   = 8       # how long bias persists after marker exits FOV

    # ToF emergency cut-off — front distance (mm) below which the controller
    # forces a full stop regardless of line tracking. Independent of the
    # directional SafetyController gate at the command-bus level: this hard
    # short-circuits the line-follow output before it even leaves the module,
    # so the firmware never sees an unsafe MEC: command.
    tof_emergency_stop_mm: int   = 250
    # Distance below which forward speed is halved (early slow-down). Set to
    # 0 to disable; must be ≥ tof_emergency_stop_mm.
    tof_slow_distance_mm:  int   = 500

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
    # True if the front ToF distance dropped below the emergency threshold
    # and the controller short-circuited to a full stop.
    tof_blocked:   bool   = False
    # Latest front distance (mm) sampled from the provider, or None when no
    # provider is wired. Surfaced so the host can publish it for telemetry.
    front_dist_mm: Optional[float] = None
    # |slope_lookahead - slope_near|; >0 = curve detected. 0 when straight or
    # 2nd-order fit unavailable. Surfaced for telemetry/diagnostics.
    curvature:     float = 0.0
    # "MANUAL" | "RECOVER_LEFT" | "RECOVER_RIGHT" | "HOLD" | "STOP" — only
    # populated when line_found is False; helps the operator/log interpret
    # what the controller is doing during line-loss events.
    recovery_mode: str   = ""


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
        self._lost_count:     int = 0
        self._last_vx:        int = 0
        self._last_vy:        int = 0
        self._last_wz:        int = 0
        # Active recovery counter — separate from _lost_count so we can
        # distinguish passive grace (hold last steering) vs. active rotate.
        self._recovery_count: int = 0
        # Turn-marker bias: latched +/- pixel offset that decays over
        # turn_marker_hold_frames once the marker leaves the FOV.
        self._marker_bias_px: float = 0.0
        self._marker_hold:    int   = 0

        # Optional callable returning the latest front-facing ToF distance
        # in millimetres (None / negative ⇒ "no reading"). Wired by the host
        # after construction via set_front_distance_provider; default-None
        # keeps unit tests and offline tools working without sensor data.
        self._front_dist_provider: Optional[Callable[[], Optional[float]]] = None

    def set_front_distance_provider(
        self,
        provider: Optional[Callable[[], Optional[float]]],
    ) -> None:
        """Inject a callable returning the latest front-facing ToF distance
        in millimetres. The provider should be cheap (read-only snapshot of
        a shared sensor cache); it is invoked once per tick(). Pass None to
        clear and disable the emergency cut-off."""
        self._front_dist_provider = provider

    def _read_front_distance(self) -> Optional[float]:
        """Sample the wired ToF provider, swallowing any exception so a
        sensor glitch never aborts the control loop."""
        if self._front_dist_provider is None:
            return None
        try:
            d = self._front_dist_provider()
        except Exception:
            return None
        if d is None:
            return None
        try:
            d = float(d)
        except (TypeError, ValueError):
            return None
        # Treat 0 / negative as "no reading" — VL53L1X drivers commonly
        # report 0 on out-of-range or read failure.
        if d <= 0.0:
            return None
        return d

    def _detect_turn_marker(self, hsv: np.ndarray) -> int:
        """Detect a coloured turn marker in the HSV frame.

        Returns -1 if a left-marker (default blue) blob with area ≥
        turn_marker_min_area is present, +1 for right-marker (default red),
        0 if neither side has a sufficiently large blob. When both sides
        are above threshold the larger one wins — defensive against noise
        bands but in practice only one marker is placed per turn.
        """
        cfg = self.cfg
        def _blob_area(low: Tuple[int, int, int],
                       high: Tuple[int, int, int]) -> int:
            m = cv2.inRange(hsv,
                            np.array(low,  dtype=np.uint8),
                            np.array(high, dtype=np.uint8))
            cnts, _ = cv2.findContours(m, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
            if not cnts:
                return 0
            return int(max(cv2.contourArea(c) for c in cnts))

        left_area  = _blob_area(cfg.turn_marker_left_hsv_low,
                                cfg.turn_marker_left_hsv_high)
        right_area = _blob_area(cfg.turn_marker_right_hsv_low,
                                cfg.turn_marker_right_hsv_high)

        thresh = int(cfg.turn_marker_min_area)
        left_ok  = left_area  >= thresh
        right_ok = right_area >= thresh
        if left_ok and right_ok:
            return -1 if left_area > right_area else 1
        if left_ok:
            return -1
        if right_ok:
            return 1
        return 0

    def tick(
        self,
        frame: np.ndarray,
        produce_overlay: bool = True,
    ) -> LineResult:
        cfg = self.cfg
        now = time.monotonic()
        dt  = max(0.001, now - self._prev_t)
        self._prev_t = now

        # Camera stutter guard: a dropped or partially-decoded frame from
        # picamera2 / V4L2 can surface here as None or a zero-sized array.
        # Treat it as a "lost line" tick so the lost-grace passive-hold
        # bridge keeps applying instead of crashing the nav loop.
        if frame is None or not isinstance(frame, np.ndarray) or frame.size == 0:
            self._lost_count += 1
            if (self._lost_count <= cfg.lost_grace_frames
                    and (self._last_vy != 0 or self._last_wz != 0)):
                hold_vx = max(0, cfg.base_vx // 2)
                return LineResult(hold_vx, self._last_vy, self._last_wz, cfg.base_spd,
                                  False, 0.0, 0.0, None)
            self._last_vy = 0
            self._last_wz = 0
            return LineResult(0, 0, 0, cfg.base_spd, False, 0.0, 0.0, None)

        # ── Step 0: ToF emergency cut-off ─────────────────────────────────────
        # Sampled before any vision work so an obstacle cuts forward motion
        # even if the camera path is slow / stalled this frame.
        front_dist = self._read_front_distance()
        if (front_dist is not None
                and cfg.tof_emergency_stop_mm > 0
                and front_dist < cfg.tof_emergency_stop_mm):
            # Short-circuit: full stop. Do not advance PD state — preserves
            # last-known errors so resumption is smooth once the obstacle
            # clears, instead of treating the pause as a giant dt.
            self._lost_count = 0
            self._last_vy = 0
            self._last_wz = 0
            overlay = (
                self._draw_overlay(frame, np.zeros((cfg.bev_h, cfg.bev_w),
                                                   dtype=np.uint8),
                                   [], None, None)
                if produce_overlay else None
            )
            return LineResult(0, 0, 0, cfg.base_spd,
                              False, 0.0, 0.0, overlay,
                              tof_blocked=True, front_dist_mm=front_dist)

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

            # Phase 1 — passive heading-hold: bridge brief occlusions by
            # keeping last steering at half forward speed.
            if (self._lost_count <= cfg.lost_grace_frames
                    and (self._last_vy != 0 or self._last_wz != 0)):
                hold_vx = max(0, cfg.base_vx // 2)
                return LineResult(hold_vx, self._last_vy, self._last_wz, cfg.base_spd,
                                  False, 0.0, 0.0, overlay,
                                  tof_blocked=False, front_dist_mm=front_dist,
                                  recovery_mode="HOLD")

            # Phase 2 — active rotation-recovery: rotate in place toward the
            # last steering direction so the camera sweeps back over the line
            # that vanished off-side during a sharp turn. Capped to avoid
            # spinning forever on a genuine end-of-line.
            if (cfg.recovery_max_frames > 0
                    and self._recovery_count < cfg.recovery_max_frames
                    and self._last_wz != 0):
                self._recovery_count += 1
                wz_dir = 1 if self._last_wz > 0 else -1
                rec_wz = wz_dir * int(cfg.recovery_wz)
                mode   = "RECOVER_LEFT" if wz_dir > 0 else "RECOVER_RIGHT"
                return LineResult(0, 0, rec_wz, cfg.base_spd,
                                  False, 0.0, 0.0, overlay,
                                  tof_blocked=False, front_dist_mm=front_dist,
                                  recovery_mode=mode)

            # Phase 3 — full stop. Reset PD + recovery state so the next
            # successful detection starts clean instead of carrying stale
            # derivatives.
            self._prev_lat_err = 0.0
            self._prev_hdg_err = 0.0
            self._recovery_count = 0
            self._marker_bias_px = 0.0
            self._marker_hold    = 0
            return LineResult(cfg.lost_vx, 0, 0, cfg.base_spd,
                              False, 0.0, 0.0, overlay,
                              tof_blocked=False, front_dist_mm=front_dist,
                              recovery_mode="STOP")

        spine_arr = np.array(spine)
        xs_arr    = spine_arr[:, 0]
        ys_arr    = spine_arr[:, 1]

        # ── Step 4: Line fit (adaptive 1st/2nd order x = poly(y)) ─────────────
        # Try 2nd-order so heading_error can anticipate curves; fall back to
        # linear when too few points or rank-deficient (e.g. all spine points
        # at the same y after BEV warp). np.polyfit returns highest-order
        # coefficient first.
        order = max(1, min(int(cfg.polyfit_order), 2))
        if len(spine) < 4:
            order = 1
        try:
            coeffs = np.polyfit(ys_arr, xs_arr, order)
        except (np.linalg.LinAlgError, ValueError):
            coeffs = np.polyfit(ys_arr, xs_arr, 1)
            order  = 1

        # Evaluation helpers — derivative dx/dy at a given y so heading is
        # the line's tangent at that point, not the chord across all of it.
        def _x_at(y: float) -> float:
            return float(np.polyval(coeffs, y))
        def _slope_at(y: float) -> float:
            if order == 2:
                a2, a1, _ = (float(c) for c in coeffs)
                return 2.0 * a2 * y + a1
            return float(coeffs[0])

        # Robot is at bottom of BEV → evaluate line at y = bev_h (near) and
        # at y = bev_h * effective_y_ratio (far) so we can sense curvature.
        # Adaptive lookahead: at higher commanded vx, push the lookahead point
        # further ahead so the PD has the same time-to-target. Anchored on the
        # last commanded vx (not base_vx) so we automatically pull the lookahead
        # back in during curvature-induced slowdowns where reaction matters more.
        speed_norm = max(0.0, min(1.0, abs(self._last_vx) / 127.0))
        y_ratio_eff = float(cfg.lookahead_y_ratio) - float(cfg.lookahead_speed_gain) * speed_norm
        # Clamp eval points to spine's actual y-range — order-2 polyfit
        # extrapolated beyond the data fit produces wild line_x values
        # (thousands of px) when the line only occupies the upper portion
        # of BEV, which would saturate vy/wz every other tick.
        y_min_spine = float(ys_arr.min())
        y_max_spine = float(ys_arr.max())
        y_near = min(float(cfg.bev_h), y_max_spine)
        y_far_raw = float(cfg.bev_h) * float(max(0.15, min(0.95, y_ratio_eff)))
        y_far  = max(y_min_spine, min(y_far_raw, y_max_spine))

        line_x_at_robot = _x_at(y_near)
        # Slopes from a separate linear regression are far more stable than
        # the local derivative of an order-2 polyfit. Order-2 derivative
        # `2*a2*y + a1` swings wildly when the spine sample shifts even a
        # few pixels (small ψ rotations between ticks), causing heading_error
        # to jump 9° → 87° → saturate wz → induce more rotation → limit cycle.
        # Linear fit slope is the average tangent across the spine and stays
        # bounded.
        if order == 2 and len(spine) >= 4:
            # Full-spine linear fit as fallback / single-source slope
            try:
                lin_coeffs = np.polyfit(ys_arr, xs_arr, 1)
                lin_slope  = float(lin_coeffs[0])
            except (np.linalg.LinAlgError, ValueError):
                lin_slope = float(_slope_at(y_near))

            # Split spine in half along y to recover a curvature signal that
            # full-spine regression averages away. Per-half linear regression
            # is still bounded (Rule 1: never use polyfit-order-2 derivative,
            # which jitters 9°→87° in one tick); using regression on a subset
            # is at most as noisy as a pointwise spot, but in practice each
            # half has ≥3 samples so the slope is smoothed.
            y_mid = 0.5 * (float(y_min_spine) + float(y_max_spine))
            near_mask = ys_arr >= y_mid  # high y in BEV = closer to robot
            far_mask  = ys_arr <  y_mid  # low  y in BEV = further ahead

            def _half_slope(mask: np.ndarray) -> float:
                if int(mask.sum()) >= 3:
                    try:
                        c = np.polyfit(ys_arr[mask], xs_arr[mask], 1)
                        return float(c[0])
                    except (np.linalg.LinAlgError, ValueError):
                        pass
                return lin_slope

            slope_near = _half_slope(near_mask)
            slope_far  = _half_slope(far_mask)
        else:
            slope_near = _slope_at(y_near)
            slope_far  = _slope_at(y_far)

        # Curvature = how much the slope changes between near and far. Used
        # purely as a magnitude for vx slowdown — sign already lives in the
        # heading_error term.
        curvature = abs(slope_far - slope_near)

        robot_x   = cfg.bev_w / 2.0
        target_x  = robot_x + self._target_offset_px

        # ── Step 4b: Turn-marker bias ─────────────────────────────────────────
        # Optional coloured square placed on one side of the line. While in
        # FOV it adds a fixed pixel bias to lateral_error; once it exits, the
        # bias persists for turn_marker_hold_frames so the robot commits to
        # the turn even after the marker scrolls out of frame.
        marker_side = 0   # -1 = left, +1 = right, 0 = none
        if cfg.turn_marker:
            marker_side = self._detect_turn_marker(hsv)
        if marker_side != 0:
            # +bias_px pushes lateral_error positive → robot interprets as
            # "line drifted right" → strafes right (toward right marker).
            self._marker_bias_px = marker_side * float(cfg.turn_marker_bias_px)
            self._marker_hold    = max(1, int(cfg.turn_marker_hold_frames))
        elif self._marker_hold > 0:
            self._marker_hold -= 1
            if self._marker_hold == 0:
                self._marker_bias_px = 0.0

        # lateral_error: positive = line is to the right of the target position
        lateral_error = (line_x_at_robot - target_x) + self._marker_bias_px

        # heading_error: blend of robot-tangent and lookahead-tangent so the
        # robot starts rotating into a curve a beat before its centre crosses
        # the bend. arctan converts pixel slope (dx/dy) → radians vs. vertical.
        blend = max(0.0, min(1.0, float(cfg.lookahead_blend)))
        slope_eff    = (1.0 - blend) * slope_near + blend * slope_far
        heading_error_raw = float(np.arctan(slope_eff))
        # Deadband suppresses tiny slope noise on straight lines that would
        # otherwise cause wz to nudge the robot and re-couple into vy.
        if abs(heading_error_raw) < float(cfg.heading_deadband_rad):
            heading_error = 0.0
        else:
            sign = 1.0 if heading_error_raw > 0 else -1.0
            heading_error = sign * (abs(heading_error_raw) - float(cfg.heading_deadband_rad))

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

        vy = int(clamp(vy_raw, -_TWIST_LIMIT, _TWIST_LIMIT))
        wz = int(clamp(wz_raw, -_TWIST_LIMIT, _TWIST_LIMIT))

        # Line found — reset all loss/recovery state, save last steering
        self._lost_count     = 0
        self._recovery_count = 0
        self._last_vy        = vy
        self._last_wz        = wz

        # ── Step 6: vx selection ──────────────────────────────────────────────
        # Three independent slowdown sources, take the most restrictive:
        #   a) base_vx
        #   b) curvature scaling — faster bend ⇒ slower forward
        #   c) ToF proximity scaling
        vx_out = cfg.base_vx

        if cfg.curvature_threshold > 0.0 and curvature > 0.0:
            # Linear ramp from base_vx (at curvature=0) to curvature_min_vx
            # (at curvature=threshold), clamped beyond. min_vx_factor=0.45
            # means forward speed bottoms out at 45% of base_vx in tight bends.
            ratio = min(1.0, curvature / float(cfg.curvature_threshold))
            curve_factor = 1.0 - ratio * (1.0 - float(cfg.curvature_min_vx_factor))
            vx_out = min(vx_out, max(0, int(cfg.base_vx * curve_factor)))

        # ToF slow-zone: when the path ahead is closer than tof_slow_distance_mm
        # but still past the emergency cut-off, halve forward speed so the
        # PD controller still gets to steer without overshooting the obstacle.
        if (front_dist is not None
                and cfg.tof_slow_distance_mm > 0
                and front_dist < cfg.tof_slow_distance_mm):
            vx_out = min(vx_out, max(0, cfg.base_vx // 2))

        overlay = (
            self._draw_overlay(frame, bev, spine, line_x_at_robot, target_x)
            if produce_overlay else None
        )

        # Cache for adaptive lookahead next tick.
        self._last_vx = int(vx_out)

        return LineResult(vx_out, vy, wz, cfg.base_spd,
                          True, lateral_error, heading_error, overlay,
                          tof_blocked=False, front_dist_mm=front_dist,
                          curvature=curvature, recovery_mode="")

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
        self._prev_lat_err   = 0.0
        self._prev_hdg_err   = 0.0
        self._prev_t         = time.monotonic()
        self._lost_count     = 0
        self._recovery_count = 0
        self._last_vx        = 0
        self._last_vy        = 0
        self._last_wz        = 0
        self._marker_bias_px = 0.0
        self._marker_hold    = 0


__all__ = ["LineFollower", "LineFollowerConfig", "LineResult", "HSVRange"]
