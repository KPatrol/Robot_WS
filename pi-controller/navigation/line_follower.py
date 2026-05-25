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
from typing import Any, Callable, List, Optional, Tuple

import cv2
import logging
import numpy as np

logger = logging.getLogger(__name__)

from .nav_utils import clamp


# Firmware MEC: protocol uses signed-byte (int8) velocity fields, so any
# (vy, wz) command must saturate inside [-127, 127] before the serial write.
_TWIST_LIMIT = 127


# ── Colour thresholds (HSV) ────────────────────────────────────────────────────
# Default: white/light-grey tape on a dark floor.
# Override for coloured tape by changing LineFollowerConfig.

@dataclass
class HSVRange:
    # V5.15c3: broadened range AGAIN after the first tuning collapsed in
    # dim ambient light. Original (18,118,115)→(31,227,233) was sampled
    # at noon in a bright room; the same tape under evening warm-white
    # LED lit at <30% scores V≈70-110, so V_low=115 rejected nearly
    # every pixel (HSV mask coverage dropped from ~1% to 0.03%). The new
    # envelope covers:
    #   H 18-38 — yellow through yellow-orange
    #   S 80+   — saturated enough to reject wood-grain colour drift
    #   V 50+   — works in dim hallway lighting and bright sunlight alike
    # Trade-off: slightly higher chance of false-positive on warm-toned
    # wood-floor stripes; in practice the morphology + min_blob_area
    # filters strip those. Verified on /tmp/diag.png — coverage went
    # from 0.03% → 0.7%, full tape visible after refresh.
    low:  Tuple[int, int, int] = (18,  80,  50)   # yellow tape — wide envelopehigh val)
    high: Tuple[int, int, int] = (38, 255, 255)   # yellow tape — wide envelope


# ── BEV calibration ───────────────────────────────────────────────────────────
# src_pts: four corners of a known ground rectangle, in image pixels (TL,TR,BR,BL)
# dst_pts: where those corners map to in the BEV (unwarped) image.
# default assumes 640×480 camera, roughly 80×60 cm visible ground patch.

# V5.15: BEV homography source points calibrated for 960×540 Pi camera
# frames (the current camera-stream resolution). Old defaults targeted
# 640×480 — they remained valid for that resolution but on the higher-res
# stream they sampled a tiny inset that often missed the line entirely.
# Trapezoid covers from ~mid-frame down (skipping ceiling/distance) to
# the bottom edge. Width is symmetric around the robot's forward axis.
_DEFAULT_BEV_SRC = np.float32([
    # V5.15b: WIDER trapezoid — earlier 240/720/870/90 trapezoid was
    # narrow enough that any drift left the line outside the warp
    # region. Now spans 100→860 px at the horizon row and the full
    # bottom edge so the line stays in BEV even when the robot is
    # ~30 cm off-axis. Wider trapezoid trades a bit of edge distortion
    # for robustness — irrelevant when the downstream PD uses the
    # column centroid, not pixel-perfect angle.
    [100, 250],   # top-left  in camera (wider horizon)
    [860, 250],   # top-right in camera
    [960, 540],   # bottom-right (entire bottom-right corner)
    [  0, 540],   # bottom-left  (entire bottom-left corner)
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

    # V5.14: optional deep lane-segmentation provider. When set AND
    # `lane_seg_provider.available`, the HSV threshold step in tick() is
    # bypassed and the deep mask is fed straight into the BEV warp +
    # centroid scan. Drops in without disturbing any other config knob —
    # PD gains, scan-column count, BEV homography all keep working
    # because the deep model emits a binary mask in the same source-frame
    # coordinate space as the HSV branch.
    #
    # Typed as Any to keep the navigation package import-free of
    # onnxruntime — the actual type is navigation.lane_seg.LaneSegProvider
    # but importing it here would force every consumer to install the
    # ONNX wheel. Soft typing keeps the existing HSV pipeline a no-op
    # on Pis that don't ship the model file.
    lane_seg_provider: Optional[Any] = None

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
    # V5.15c: rolled back kp_lateral 0.80 → 0.45 and kd 0.10 → 0.06.
    # The aggressive doubled gain overshoots: robot strafes ±80 in one
    # tick, lands off the line on the OPPOSITE side, strafes back,
    # repeats. Operator reports "đi cực nhanh khiến out khỏi line".
    # 0.45 is a touch above the 0.40 baseline so we still react faster
    # than pre-V5.15b, but the response stays bounded by max_strafe and
    # the bigger help comes from the new vx slowdown + tighter
    # lat_err_slowdown_px below — robot eases forward as soon as it
    # drifts, giving the PD time to land softly back on centre.
    kp_lateral:  float = 0.45   # P gain for lateral error → vy
    kd_lateral:  float = 0.06   # D gain for lateral error → vy
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
    # V5.15c2: vx 100 → 80. The 100 setting drove forward fast enough
    # that any small lateral error compounded into "off the BEV" before
    # the PD could correct. 80 gives the controller more ticks per
    # metre to keep the robot on-line. Keep base_spd 220 so the MEC
    # kinematic split (some wheels at vx-vy) still has PWM above stall.
    base_vx:     int   = 80     # forward velocity component in MEC (0–127)
    lost_vx:     int   = 0      # forward speed when line is lost
    base_spd:    int   = 220    # absolute scale passed to ESP32 MEC: spd

    # Line detection
    # V5.15: relaxed from 20 → 5. The yellow floor tape produces a thin
    # ~4-6 px wide stripe in the BEV image (vs. 10-15 px for the older
    # white-tape default the original threshold was calibrated against).
    # Twenty pixels per column was too aggressive — the column scan kept
    # dropping every other column and the centroid line went sparse,
    # which then triggered the "line lost" branch. Five pixels still
    # rejects single-pixel speckle from morphology residue.
    min_line_pixels: int = 5    # minimum mask pixels per column to count
    scan_cols:       int = 20   # number of vertical columns to scan

    # V5.15: line-following target mode.
    #   "center" — robot drives DIRECTLY OVER the line (line stays in the
    #              middle of the camera frame). Simpler, more predictable,
    #              symmetric error space → operator just lines the robot
    #              up over the tape and presses GO.
    #   "side"   — robot keeps the line at a fixed lateral offset, set by
    #              `target_left_offset_cm`. The classical "drive 10 cm
    #              left of the line" mode. Mecanum-friendly but easy to
    #              get the setup distance wrong, in which case the PD
    #              lurches diagonally to correct.
    # Default 'center' because that's what users intuitively expect.
    target_mode: str = "center"
    # V5.15c2: further dialed back 0.8 → 0.45 after operator reported
    # the robot crabs sideways so fast it leaves the BEV window before
    # the next tick can correct. With base_vx=80 below, vy cap = 36 —
    # robot eases sideways gently while continuing forward, taking
    # multiple ticks to recentre instead of one violent sweep.
    max_strafe_ratio: float = 0.45

    # V5.15b: adaptive forward speed. When the lateral error exceeds
    # `lat_err_slowdown_px`, vx is scaled down so the PD has time to
    # bring the robot back on-line before more forward drift compounds
    # the offset. At |err| == threshold, vx hits `min_vx_factor` × base.
    # Critical for an unbalanced mecanum: pushing forward at full base
    # speed while drifting sideways turns a 5 cm offset into 20 cm in
    # one second.
    # V5.15c2: tighter slowdown threshold + lower floor. Robot eases
    # off forward as soon as it sees a 3 cm offset (was 6 cm) and can
    # drop to 10% base_vx (was 25%) when way off — almost stops moving
    # forward, devotes its motion budget to strafing back to centre.
    # Stable, "êm dịu" character the operator asked for.
    lat_err_slowdown_px: float = 15.0   # ~3 cm at bev_px_per_cm=5
    min_vx_factor: float = 0.10         # at threshold error, vx → 10% base

    # V5.15b: rolling mean of |vy| over the last N ticks. If the robot
    # has to crab the same way for a sustained period the simplest
    # explanation is a mechanical bias (uneven wheels, off-centre mass).
    # We feed-forward a constant counter-bias so the PD doesn't have to
    # fight the drift on its own; the bias decays when the error sign
    # flips, which prevents it from runaway during a real curve.
    drift_compensation: bool = True
    drift_window_ticks: int = 60        # 3 s at 20 Hz
    drift_bias_gain: float = 0.4        # fraction of running-mean to add back
    min_blob_area:   int = 500  # minimum contiguous HSV blob area (px²) — rejects curb/vegetation noise

    # Heading-hold fallback: when line is lost hold last steering for N frames
    # before stopping (~0.5 s at 20 Hz). Set to 0 to disable.
    lost_grace_frames: int = 10

    # Active rotation-recovery: after lost_grace_frames expire, rotate in place
    # toward the last steering direction for up to recovery_max_frames so the
    # camera can re-acquire a line that left the FOV during a sharp turn.
    # Set recovery_max_frames=0 to disable (full stop after grace).
    # Tuned 2026-05-20 from (45, 40) → (35, 25) after l_turn sim: original was
    # 40 frames × 45 °/s = 60° of rotation, blowing past 90° corners and forcing
    # vy saturation on re-acquire. 25 frames × 35 °/s ≈ 29° covers the L-turn
    # without overshooting, and the new _was_lost PD reset absorbs residual
    # offset without the strafe spike.
    recovery_wz:          int = 35
    recovery_max_frames:  int = 25

    # Post-recovery settle window: for the first `settle_frames` ticks after
    # the line is re-acquired, multiply the lateral PD output by
    # `settle_lat_gain` so a large residual offset does not produce a vy
    # saturation pulse. Heading PD stays at full gain (rotation alone cannot
    # destabilise tracking the way saturated strafe can). Set settle_frames=0
    # to disable.
    settle_frames:        int   = 8
    settle_lat_gain:      float = 0.4

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
    # Adaptive lookahead disabled by default (thresh=0). The earlier attempt
    # (thresh=0.08 with divergence detector) over-suppressed lookahead on
    # gentle oval / zigzag curves, ballooning off-track to 60%+. The slope
    # cap+EMA above is sufficient on its own. Re-enable via thresh>0 only
    # after careful per-map verification.
    lookahead_blend_straight_thresh: float = 0.0
    lookahead_blend_straight_factor: float = 0.20
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

    # Slope-jitter guard: cap |slope_far - slope_near| before blending into
    # heading_error. 0.60 (~31°) is large enough to keep real S-curve / oval /
    # zigzag curvature information intact (verified via tools/line_follow_sim.py
    # 2026-05-20: cap=0.20 destroyed oval+zigzag pass rates). Set to 0 to
    # disable the cap.
    slope_far_max_delta:      float = 0.60

    # Temporal EMA smoothing on slope_near / slope_far so polyfit jitter does
    # not propagate to heading_error frame-to-frame. alpha=0.60 still lets real
    # curves register within ~2 frames (0.1 s @ 20 Hz) while filtering high-
    # frequency spine sampling noise. Lower values (0.30) over-smoothed the
    # oval / zigzag tracking. Set to 1.0 to disable.
    slope_ema_alpha:          float = 0.60

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

    # ── Obstacle-avoidance sub-FSM ────────────────────────────────────────
    # When True, an obstacle inside tof_emergency_stop_mm triggers a timed
    # 3-phase detour (STRAFE → FORWARD → STRAFE_BACK) and then falls back to
    # the vision pipeline to re-acquire the line. When False, the controller
    # short-circuits to a full stop (legacy behaviour, used when no detour
    # space is available, e.g. inside narrow corridors).
    avoid_enabled:                 bool  = True
    # Phase 1 — strafe sideways to clear the obstacle. Direction is chosen
    # automatically per detour: opposite to the last vy command if possible
    # (so the robot moves AWAY from where the line was), or +1 (right) by
    # default.
    avoid_strafe_vy:               int   = 50
    avoid_strafe_duration_s:       float = 0.8
    # Phase 2 — drive forward past the obstacle.
    avoid_forward_vx:              int   = 70
    avoid_forward_duration_s:      float = 1.5
    # Phase 3 — strafe back toward the line, with a small forward component
    # so the robot finishes ahead of the obstacle rather than re-acquiring
    # behind it. After this phase ends, the vision pipeline runs again and
    # the existing line-loss recovery handles re-acquisition.
    avoid_strafe_back_vy:          int   = 35   # absolute magnitude (sign auto-flipped)
    avoid_strafe_back_vx:          int   = 25
    avoid_strafe_back_duration_s:  float = 1.0

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
    # "NONE" | "STRAFE" | "FORWARD" | "STRAFE_BACK" — populated when the
    # obstacle-avoidance sub-FSM is active. Empty/"NONE" otherwise.
    avoid_phase:   str   = ""


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

        # Pre-compute target X in BEV image.
        # V5.15: when `target_mode == "center"` the robot drives directly
        # over the line — target_x == robot_x (mid of BEV). When "side",
        # the line is held at a fixed lateral offset configured by
        # `wheel_half_track_cm + target_left_offset_cm`. "center" is the
        # default because users intuitively place the robot ON the tape,
        # not next to it, and the symmetric error space makes the PD
        # tuning simpler.
        if self.cfg.target_mode == "center":
            self._target_offset_px: float = 0.0
        else:
            self._target_offset_px = (
                (self.cfg.wheel_half_track_cm + self.cfg.target_left_offset_cm)
                * self.cfg.bev_px_per_cm
            )

        # V5.15b: rolling-mean lateral error window for drift compensation.
        # Refilled every successful tick; cleared on lost so the bias
        # doesn't persist across reacquisitions where the chassis may
        # have been moved.
        self._lat_err_history: List[float] = []

        # Heading-hold state
        self._lost_count:     int = 0
        self._last_vx:        int = 0
        self._last_vy:        int = 0
        self._last_wz:        int = 0
        # Active recovery counter — separate from _lost_count so we can
        # distinguish passive grace (hold last steering) vs. active rotate.
        self._recovery_count: int = 0
        # Latched while in any loss/recovery phase. On the first frame the
        # line is re-acquired, prev_lat/hdg_err are reset to current values so
        # the PD derivative does not spike from (post_recovery − pre_loss)/dt,
        # which would otherwise saturate vy and induce a limit-cycle after the
        # robot loses sight of the line at sharp corners.
        self._was_lost:       bool = False
        # Post-recovery settle window: for the first N frames after re-acquire
        # the lateral PD is attenuated to absorb residual offset without
        # commanding a vy saturation pulse.
        self._settle_count:   int = 0
        # EMA-filtered slope state to suppress polyfit / spine-sampling jitter.
        # None ⇒ uninitialised; seeded with the first observed slope so the
        # filter does not start at 0 and bias real curves toward straight.
        self._slope_near_ema: Optional[float] = None
        self._slope_far_ema:  Optional[float] = None
        # Turn-marker bias: latched +/- pixel offset that decays over
        # turn_marker_hold_frames once the marker leaves the FOV.
        self._marker_bias_px: float = 0.0
        self._marker_hold:    int   = 0

        # Optional callable returning the latest front-facing ToF distance
        # in millimetres (None / negative ⇒ "no reading"). Wired by the host
        # after construction via set_front_distance_provider; default-None
        # keeps unit tests and offline tools working without sensor data.
        self._front_dist_provider: Optional[Callable[[], Optional[float]]] = None

        # Obstacle-avoidance sub-FSM state. "NONE" when idle; otherwise one of
        # "STRAFE" / "FORWARD" / "STRAFE_BACK" with `_avoid_t0` capturing the
        # monotonic timestamp at which the phase started. `_avoid_side` is
        # +1 (right) or -1 (left) — direction chosen at detour-entry based on
        # the last lateral command so the robot moves AWAY from the line, not
        # back across it.
        self._avoid_phase: str   = "NONE"
        self._avoid_t0:    float = 0.0
        self._avoid_side:  int   = +1

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

    # ── Obstacle-avoidance sub-FSM ────────────────────────────────────────
    def _enter_avoid(self, now: float) -> None:
        """Latch the detour FSM into phase STRAFE. Direction is chosen so the
        robot moves AWAY from where the line currently sits: if the last vy
        was positive (the controller was strafing right toward the line),
        detour to the LEFT (-1); otherwise detour to the RIGHT (+1)."""
        # Choose side opposite to last command's strafe direction so we don't
        # crowd the line while detouring. Default right (+1) when no recent
        # lateral command (line straight ahead).
        if self._last_vy > 0:
            self._avoid_side = -1
        elif self._last_vy < 0:
            self._avoid_side = +1
        else:
            self._avoid_side = +1
        self._avoid_phase = "STRAFE"
        self._avoid_t0    = now
        # Reset line-loss bookkeeping so vision pipeline gets a clean slate
        # once the detour completes.
        self._lost_count     = 0
        self._recovery_count = 0
        self._was_lost       = True   # force PD re-seed on re-acquire

    def _step_avoid(self, now: float) -> Tuple[int, int, int, str]:
        """Advance the detour FSM by one tick and return (vx, vy, wz, phase).
        When the FSM exits, sets `_avoid_phase` back to "NONE" and returns
        phase="NONE" so the caller falls through to the vision pipeline."""
        cfg     = self.cfg
        phase   = self._avoid_phase
        elapsed = now - self._avoid_t0

        if phase == "STRAFE":
            if elapsed < cfg.avoid_strafe_duration_s:
                vy = int(cfg.avoid_strafe_vy) * self._avoid_side
                return 0, vy, 0, "STRAFE"
            # advance to FORWARD
            self._avoid_phase = "FORWARD"
            self._avoid_t0    = now
            phase, elapsed = "FORWARD", 0.0

        if phase == "FORWARD":
            if elapsed < cfg.avoid_forward_duration_s:
                return int(cfg.avoid_forward_vx), 0, 0, "FORWARD"
            # advance to STRAFE_BACK
            self._avoid_phase = "STRAFE_BACK"
            self._avoid_t0    = now
            phase, elapsed = "STRAFE_BACK", 0.0

        if phase == "STRAFE_BACK":
            if elapsed < cfg.avoid_strafe_back_duration_s:
                # Strafe back toward the line (opposite side) with small forward.
                vy = -int(cfg.avoid_strafe_back_vy) * self._avoid_side
                return int(cfg.avoid_strafe_back_vx), vy, 0, "STRAFE_BACK"
            # detour complete — exit FSM
            self._avoid_phase = "NONE"
            self._avoid_t0    = 0.0
            return 0, 0, 0, "NONE"

        # Should never reach here with phase != NONE / STRAFE / FORWARD /
        # STRAFE_BACK, but defensively reset.
        self._avoid_phase = "NONE"
        return 0, 0, 0, "NONE"

    def _make_avoid_result(
        self,
        vx: int, vy: int, wz: int, phase: str,
        front_dist: Optional[float],
        frame: np.ndarray,
        produce_overlay: bool,
    ) -> "LineResult":
        """Build a LineResult for a detour tick. Tracks last_v* so when the
        FSM finishes and vision resumes, recovery logic sees the detour as
        the most recent command — not stale pre-detour values."""
        cfg = self.cfg
        self._last_vy = vy
        self._last_wz = wz
        overlay = (
            self._draw_overlay(frame, np.zeros((cfg.bev_h, cfg.bev_w),
                                                dtype=np.uint8),
                               [], None, None)
            if produce_overlay else None
        )
        return LineResult(
            vx, vy, wz, cfg.base_spd,
            False, 0.0, 0.0, overlay,
            tof_blocked=True,
            front_dist_mm=front_dist,
            avoid_phase=phase,
        )

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

        # ── Step 0: ToF emergency cut-off + obstacle-avoidance sub-FSM ────────
        # Sampled before any vision work so an obstacle cuts forward motion
        # even if the camera path is slow / stalled this frame.
        front_dist = self._read_front_distance()

        # If the detour FSM is already running, keep stepping it regardless
        # of current front distance — the timed pattern is open-loop by
        # design so the robot commits to the manoeuvre instead of oscillating
        # at the boundary.
        if self._avoid_phase != "NONE":
            vx_a, vy_a, wz_a, phase_a = self._step_avoid(now)
            if phase_a != "NONE":
                return self._make_avoid_result(
                    vx_a, vy_a, wz_a, phase_a,
                    front_dist, frame, produce_overlay,
                )
            # FSM just exited this tick — fall through to vision pipeline so
            # the line is re-acquired in the same frame. Reset PD timing so
            # the dt step between pre-detour and now does not spike the
            # derivative term.
            self._prev_t = now

        elif (front_dist is not None
                and cfg.tof_emergency_stop_mm > 0
                and front_dist < cfg.tof_emergency_stop_mm):
            if cfg.avoid_enabled:
                # Enter detour FSM and emit phase-1 command this tick.
                self._enter_avoid(now)
                vx_a, vy_a, wz_a, phase_a = self._step_avoid(now)
                return self._make_avoid_result(
                    vx_a, vy_a, wz_a, phase_a,
                    front_dist, frame, produce_overlay,
                )
            # Legacy hard-stop path (avoidance disabled).
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

        # ── Step 1: Mask extraction ───────────────────────────────────────────
        # V5.14: prefer the deep lane-seg provider when one is wired in;
        # fall back to the classical HSV threshold otherwise. The deep
        # mask is already a uint8 0/255 binary in source-frame coords —
        # same shape, same dtype as the HSV branch — so steps 2+ are
        # unchanged. Morphology is still applied to both branches: the
        # deep model occasionally leaves single-pixel speckle along weak
        # edges, and the cleanup is cheap (<1 ms).
        mask: Optional[np.ndarray] = None
        if cfg.lane_seg_provider is not None and getattr(cfg.lane_seg_provider, "available", False):
            try:
                seg_result = cfg.lane_seg_provider.infer(frame)
                if seg_result is not None:
                    mask = seg_result.mask
            except Exception as exc:
                logger.warning(f"[LF] lane_seg infer failed, falling back to HSV: {exc}")
        if mask is None:
            hsv  = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv,
                               np.array(cfg.hsv.low,  dtype=np.uint8),
                               np.array(cfg.hsv.high, dtype=np.uint8))

        # Small morphology to clean noise — applied to both HSV and deep masks.
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

        # V5.15c3: relaxed AGAIN 2 → 1. The yellow tape is so thin in
        # BEV (~5 px wide on a 400 px canvas) that it routinely sits in
        # only one of the 20 scan columns — verified on the diagnostic
        # frame where BEV mask was 3.3% but the detection still flagged
        # lost. With min=1 a single column is enough to give a lateral
        # centroid; heading slope skips out gracefully when there's no
        # second point to regress against (handled in step 4 below).
        if len(spine) < 1:
            self._lost_count += 1
            overlay = self._draw_overlay(frame, bev, [], None, None) if produce_overlay else None

            # Phase 1 — passive heading-hold: bridge brief occlusions by
            # keeping last steering at half forward speed.
            if (self._lost_count <= cfg.lost_grace_frames
                    and (self._last_vy != 0 or self._last_wz != 0)):
                hold_vx = max(0, cfg.base_vx // 2)
                self._was_lost = True
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
                self._was_lost = True
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
            self._was_lost       = True
            self._marker_bias_px = 0.0
            self._marker_hold    = 0
            # Reset slope EMA so a totally-new line direction post-stop does
            # not carry stale slope memory from before the loss event.
            self._slope_near_ema = None
            self._slope_far_ema  = None
            return LineResult(cfg.lost_vx, 0, 0, cfg.base_spd,
                              False, 0.0, 0.0, overlay,
                              tof_blocked=False, front_dist_mm=front_dist,
                              recovery_mode="STOP")

        spine_arr = np.array(spine)
        xs_arr    = spine_arr[:, 0]
        ys_arr    = spine_arr[:, 1]

        # V5.15c3: single-centroid fast path. With min_spine=1 the
        # column scan may hand us exactly one point (the yellow tape only
        # registers in one of the 20 BEV scan columns when the line is
        # thin). polyfit needs ≥2 points, so we synthesize a constant
        # polynomial x = centroid.x — line treated as a perfect vertical
        # passing through that centroid, slope = 0. The PD then steers
        # entirely on lateral_error, which is exactly the right behaviour
        # at this detection density.
        if len(spine) < 2:
            coeffs = np.array([float(xs_arr[0])])     # constant: x = cx_only
            order = 0
        else:
            # ── Step 4: Line fit (adaptive 1st/2nd order x = poly(y)) ─────────
            # Try 2nd-order so heading_error can anticipate curves; fall back to
            # linear when too few points or rank-deficient (e.g. all spine
            # points at the same y after BEV warp). np.polyfit returns
            # highest-order coefficient first.
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
        # order == 0 is the single-centroid fast path: constant line, no slope.
        def _x_at(y: float) -> float:
            return float(np.polyval(coeffs, y))
        def _slope_at(y: float) -> float:
            if order == 0:
                return 0.0
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

        # Temporal EMA smoothing on slope_near / slope_far. Pure per-frame
        # polyfit output drifts ~0.05 between consecutive frames even on a
        # geometrically static straight line (the BEV view scrolls and the
        # spine sample set changes), which propagates into heading_error and
        # eventually drives a vy limit cycle. The EMA averages this jitter
        # out while real curvature still registers within ~3 frames.
        alpha = float(cfg.slope_ema_alpha)
        if 0.0 < alpha < 1.0:
            if self._slope_near_ema is None:
                self._slope_near_ema = slope_near
                self._slope_far_ema  = slope_far
            else:
                self._slope_near_ema = (alpha * slope_near
                                        + (1.0 - alpha) * self._slope_near_ema)
                self._slope_far_ema  = (alpha * slope_far
                                        + (1.0 - alpha) * self._slope_far_ema)
            slope_near = float(self._slope_near_ema)
            slope_far  = float(self._slope_far_ema)

        # Slope-jitter guard: cap slope_far relative to slope_near. On near-
        # straight regimes slope_near is stable (~0) but slope_far can swing
        # ±0.5+ across consecutive frames as the robot translates and the
        # order-2 polyfit far tail amplifies tiny spine wobbles. Without this
        # cap heading_error grows monotonically (+7° → +70° in ~10 frames)
        # until vy saturates into a ±127 limit cycle. Capping the cross-spine
        # delta preserves real curvature signal (L-turn / S-curve have
        # slope_near tilted too, so the cap floor is high enough) while
        # killing pure jitter on straight lines.
        if cfg.slope_far_max_delta > 0.0:
            max_delta = float(cfg.slope_far_max_delta)
            slope_far = slope_near + max(-max_delta,
                                         min(slope_far - slope_near, max_delta))

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
        # Adaptive blend: scale by |slope_far - slope_near|. If both halves
        # agree (≈ straight), suppress lookahead to keep slope_far jitter from
        # drifting hdg; if they diverge (real curve), restore full blend.
        blend = max(0.0, min(1.0, float(cfg.lookahead_blend)))
        thresh = float(cfg.lookahead_blend_straight_thresh)
        if thresh > 0.0:
            divergence = abs(slope_far - slope_near)
            if divergence < thresh:
                factor = float(cfg.lookahead_blend_straight_factor)
                # Linear ramp from factor (at div=0) to 1.0 (at div=thresh).
                t = divergence / thresh
                blend *= factor + (1.0 - factor) * t
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
        # Edge-trigger from any loss/recovery phase to MANUAL: seed the
        # derivative memory with the freshly observed error so d_lat does
        # not spike from (post_recovery − pre_loss)/dt. Without this the
        # first re-acquired frame after a sharp turn computes d_lat in the
        # hundreds, vy*kd_lateral saturates the strafe channel, and the
        # robot overshoots into a limit-cycle that fails l_turn.
        if self._was_lost:
            self._prev_lat_err = lateral_error
            self._prev_hdg_err = heading_error
            self._was_lost     = False
            self._settle_count = max(self._settle_count, cfg.settle_frames)

        d_lat = (lateral_error   - self._prev_lat_err) / dt
        d_hdg = (heading_error   - self._prev_hdg_err) / dt

        self._prev_lat_err = lateral_error
        self._prev_hdg_err = heading_error

        # During the settle window the lateral PD is attenuated. Heading PD
        # is left at full gain because rotation alone cannot drive the robot
        # off-track in the way a saturated vy can — and at corner exit we
        # actively want the heading channel to dominate.
        if self._settle_count > 0:
            lat_attenuation = float(cfg.settle_lat_gain)
            self._settle_count -= 1
        else:
            lat_attenuation = 1.0

        # vy: + = strafe right to reduce left drift (when lateral_error > 0,
        # line is to the right of target → robot drifted left → steer right)
        vy_raw = lat_attenuation * (cfg.kp_lateral * lateral_error
                                    + cfg.kd_lateral * d_lat)
        # wz: + = rotate left; when heading_error > 0, line leans left → rotate left
        wz_raw = cfg.kp_heading * heading_error + cfg.kd_heading * d_hdg

        # V5.15b: drift-bias compensation. An unbalanced chassis causes
        # the robot to drift the SAME direction every tick, forcing the
        # PD to spend most of its vy fighting the bias instead of
        # tracking the line. We track the rolling mean of recent
        # lateral_error samples; if it sustains one sign, that's the
        # mechanical drift signature and we add a constant counter-bias
        # so the PD only sees the residual real error. The window length
        # and gain are configurable; the bias decays automatically when
        # the rolling mean shrinks toward zero.
        if cfg.drift_compensation:
            self._lat_err_history.append(lateral_error)
            if len(self._lat_err_history) > cfg.drift_window_ticks:
                self._lat_err_history.pop(0)
            if len(self._lat_err_history) >= cfg.drift_window_ticks // 2:
                mean_err = sum(self._lat_err_history) / len(self._lat_err_history)
                # If sustained drift, PD adds extra strafe in opposite
                # direction. mean_err > 0 means robot consistently sees
                # line to the right of target → it has been drifting
                # left → bias vy positive (strafe right).
                vy_raw += cfg.drift_bias_gain * cfg.kp_lateral * mean_err

        vy = int(clamp(vy_raw, -_TWIST_LIMIT, _TWIST_LIMIT))
        wz = int(clamp(wz_raw, -_TWIST_LIMIT, _TWIST_LIMIT))
        # V5.15b: cap |vy| as a fraction of the forward base speed so the
        # robot can't crab sideways faster than it moves forward — keeps
        # the recovery diagonal shallow, robot returns to the line with
        # a smooth curve instead of a sharp 90° slide. With ratio 1.5
        # the strafe can briefly exceed base_vx when overpowering a
        # mechanical drift, then ease off as the robot recentres.
        vy_cap = int(cfg.base_vx * cfg.max_strafe_ratio)
        if vy_cap > 0:
            vy = int(clamp(vy, -vy_cap, vy_cap))

        # Line found — reset all loss/recovery state, save last steering
        self._lost_count     = 0
        self._recovery_count = 0
        self._last_vy        = vy
        self._last_wz        = wz

        # ── Step 6: vx selection ──────────────────────────────────────────────
        # Four independent slowdown sources, take the most restrictive:
        #   a) base_vx
        #   b) lateral-error slowdown — when off-line, ease forward so the
        #      PD has time to correct before drift compounds (V5.15b).
        #   c) curvature scaling — faster bend ⇒ slower forward
        #   d) ToF proximity scaling
        vx_out = cfg.base_vx

        # V5.15b: adaptive vx vs. lateral error. Robots with an unbalanced
        # chassis (e.g. operating without the battery so the centre of
        # mass is off) drift continuously toward one side; pushing forward
        # at full speed while drifting widens the gap. Scale vx down with
        # |lateral_error| so a hard offset bleeds off in mostly-strafe.
        if cfg.lat_err_slowdown_px > 0.0:
            err_norm = min(1.0, abs(lateral_error) / cfg.lat_err_slowdown_px)
            err_factor = 1.0 - err_norm * (1.0 - cfg.min_vx_factor)
            vx_out = min(vx_out, max(0, int(cfg.base_vx * err_factor)))

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
        self._was_lost       = False
        self._settle_count   = 0
        self._marker_bias_px = 0.0
        self._marker_hold    = 0
        # V5.15b: clear drift history on reset — fresh start, fresh bias.
        self._lat_err_history = []


__all__ = ["LineFollower", "LineFollowerConfig", "LineResult", "HSVRange"]
