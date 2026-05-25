"""Brutally-simple line follower with two steering modes.

This module exists alongside the full BEV / PD pipeline in
`line_follower.py` because that pipeline accumulated enough moving parts
— perspective warp, polynomial fit, drift compensation, heading hold —
that it's no longer easy to reason about on the demo floor. The thesis
write-up only needs "robot driving forward along a tape line"; this
module delivers exactly that with a couple of dozen lines of OpenCV
and three tunable numbers.

Two steering modes are supported via `SimpleLineConfig.steering_mode`:

* ``"mecanum"`` (default V5.15c9) — emits twist tuples ``(vx, vy=0, wz,
  spd)`` for the firmware MEC: command. This is **actual Mecanum
  steering**: the robot rotates about its centre toward the line while
  driving forward, never strafes. It's what graduation-defence judges
  expect when they hear "robot bám line". The earlier verb mode looked
  like "đi chéo" because diagonal commands (DL/DR) mix vx + vy, which
  translates Mecanum wheels into a crab-walk — the robot drifts
  sideways without turning to face the line.

* ``"verbs"`` — legacy joystick verb output ``F / DL / DR / SL / SR /
  S``. Kept as a fallback for venues where MEC: per-wheel PWM stalls
  at low speeds (motor stall threshold ~PWM 100). Joystick verbs use
  full-power H-bridge presets so the wheels always start cleanly.

Design choices and why:
  * **No BEV warp.** Calibrating the homography requires measuring four
    floor points against the mounted camera; if any of those numbers
    drift (camera tilted by mounting friction, frame resolution
    changes), the entire warp produces nonsense. The simple controller
    reads a *bottom strip* of the raw frame instead — same width, no
    perspective transform.
  * **Centroid only.** The full pipeline drives both `vy` (strafe) and
    `wz` (rotate) from a polyfit of the line spine. That needs ≥4
    spine points; with a 5-pixel-wide tape it routinely delivered 1-2
    → lost-flag → recovery rotate → robot froze. Here we take the
    centroid of every white pixel in the bottom strip and treat that
    as the line position, full stop.
  * **Proportional steering, no derivative.** With a Mecanum platform
    on a flat indoor floor there is no lag to compensate; a derivative
    term just amplifies camera shake. P-only with a generous dead
    band keeps the wheels quiet when the line is centred.

The result is "đi thẳng theo line + đánh lái khi lệch" — natural
steering, not crab-walking.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import cv2
import numpy as np


# ─── Config ──────────────────────────────────────────────────────────────
@dataclass
class SimpleLineConfig:
    """Tuning knobs for the simple centroid-based follower."""

    # HSV envelope for yellow demo tape. Wider than the full pipeline's
    # version because we don't have downstream BEV+spine filtering to
    # absorb false positives — but we do still apply morphology to clean
    # up speckle.
    hsv_low:  Tuple[int, int, int] = (18,  80,  50)
    hsv_high: Tuple[int, int, int] = (38, 255, 255)

    # Fraction of frame height to scan, measured from the bottom.
    # 0.35 = bottom third of the image, where the floor in front of the
    # robot lives in a normal mounting. Larger value sees further ahead
    # but mixes in distant noise.
    roi_bottom_frac: float = 0.35

    # V5.15c9: pick how the controller drives the wheels.
    #   "mecanum" — emit MEC: twist (vx, 0, wz, spd). True steering.
    #   "verbs"   — emit F/DL/DR/SL/SR/S joystick verbs. Crab-walks.
    steering_mode: str = "mecanum"

    # ── MECANUM mode params (steering_mode == "mecanum") ─────────────
    #
    # Twist tuple sent to firmware MEC: command. Scales match what the
    # full BEV pipeline emits so the per-wheel PWM split lands above
    # stall (~100) on every wheel even during a sharp correction:
    #     fl_pwm = vx - vy - wz
    #     fr_pwm = vx + vy + wz
    #     bl_pwm = vx + vy - wz
    #     br_pwm = vx - vy + wz
    # With vy=0, vx=80, max_wz=40 the worst-case wheel PWM is 120 or 40
    # — the latter is below stall, so we widen the dead band (mec_straight_band)
    # so we never command a tiny wz that the wheels can't act on.
    mec_base_vx: int = 80          # forward velocity (normalized 0-100)
    mec_max_wz: int = 45           # |wz| clamp (rotate ±45)
    mec_kp_steer: float = 90.0     # error_norm × kp → wz
    mec_spd: int = 200             # overall PWM scale forwarded to firmware
    # Dead band in pixels — when |err| < this, wz = 0 (pure forward).
    # On a 960×720 frame that's ±10% of width = ±96 px → plenty of
    # tolerance for camera/tape micro-jitter without freezing wheels.
    mec_straight_band: int = 96

    # ── VERB mode params (steering_mode == "verbs") ──────────────────
    #
    # Kept as fallback. Same threshold/speed schema as before V5.15c9.
    straight_band: int = 140
    diagonal_band: int = 280
    speed_forward:   int = 120
    speed_diagonal:  int = 110
    speed_strafe:    int = 100

    # Minimum mask coverage in the ROI to call the line "found". Below
    # this we treat the frame as lost and stop the robot. 0.1 % is
    # ~600 px on a 640×480 frame — enough to reject speckle, low enough
    # to catch a thin tape at a glance.
    min_mask_pct: float = 0.1

    # When line goes lost, freeze any motion immediately and report
    # back so the orchestrator can decide whether to keep the camera on
    # or switch back to MANUAL.
    stop_on_lost: bool = True


# ─── Result type ─────────────────────────────────────────────────────────
@dataclass
class SimpleLineResult:
    """One tick's decision, packaged for the orchestrator + telemetry."""

    # In "verbs" mode: 'F' / 'SL' / 'SR' / 'DL' / 'DR' / 'S'.
    # In "mecanum" mode: 'MEC' for an active twist or 'S' for stop —
    # the orchestrator dispatches based on this verb.
    command: str
    # Absolute speed (verb mode) OR PWM scale forwarded with MEC: (mecanum mode).
    speed: int
    # True if the line was visible this frame.
    found: bool
    # Pixel offset of line centroid from frame centre. Positive = line
    # to the right.
    error_px: float
    # Mask coverage percentage in the ROI strip.
    coverage_pct: float
    # Mecanum twist (vx, vy, wz, spd) when command == 'MEC'. None otherwise.
    # vy is always 0 here — Mecanum follower never strafes by design.
    twist: Optional[Tuple[int, int, int, int]] = None
    # Optional BGR overlay for the MJPEG stream.
    overlay: Optional[np.ndarray] = None


# ─── The follower ────────────────────────────────────────────────────────
class SimpleLineFollower:
    """Centroid-based line follower — no BEV, no PD, just go.

    Stateless across ticks (the orchestrator owns "in / out of mode")
    so reset / restart is just constructing a fresh instance — there's
    no PD derivative to clear, no recovery counter, no spine history."""

    def __init__(self, cfg: Optional[SimpleLineConfig] = None):
        self.cfg = cfg or SimpleLineConfig()
        # Pre-build morphology kernel — same shape every call.
        self._kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))

    def tick(self, frame: np.ndarray,
              produce_overlay: bool = False) -> SimpleLineResult:
        cfg = self.cfg
        h, w = frame.shape[:2]
        # ROI = bottom strip of the frame. We slice a view so the
        # downstream cv2 calls allocate a small array, not the full
        # frame — keeps per-tick cost <5 ms on Pi.
        roi_top = int(h * (1.0 - max(0.05, min(0.95, cfg.roi_bottom_frac))))
        roi = frame[roi_top:h, :, :]

        # HSV mask + cheap morphology cleanup.
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv,
                           np.array(cfg.hsv_low, dtype=np.uint8),
                           np.array(cfg.hsv_high, dtype=np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self._kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self._kernel)

        # Coverage health check.
        coverage_pct = 100.0 * float((mask > 0).sum()) / float(mask.size)

        # Centroid of all on-pixels in the ROI. `cv2.moments` is the
        # cheapest robust centroid we can get — O(N) sum, no contour
        # extraction needed.
        m = cv2.moments(mask, binaryImage=True)
        if coverage_pct < cfg.min_mask_pct or m["m00"] < 1.0:
            # Line not visible — stop unless config says otherwise.
            overlay = self._build_overlay(frame, mask, roi_top, None,
                                          "S", 0, "LOST") if produce_overlay else None
            return SimpleLineResult(
                command="S" if cfg.stop_on_lost else "F",
                speed=0 if cfg.stop_on_lost else cfg.speed_forward,
                found=False,
                error_px=0.0,
                coverage_pct=coverage_pct,
                overlay=overlay,
            )

        # Centroid in ROI coords → frame-centre error.
        cx = float(m["m10"] / m["m00"])
        error_px = cx - (w / 2.0)

        # Dispatch to the steering mode the operator picked.
        if cfg.steering_mode == "mecanum":
            cmd, spd, twist = self._steer_mecanum(error_px, w)
        else:
            cmd, spd, twist = self._steer_verbs(error_px)

        overlay = self._build_overlay(frame, mask, roi_top,
                                      (int(cx), 0), cmd, spd,
                                      "FOUND") if produce_overlay else None
        return SimpleLineResult(
            command=cmd,
            speed=spd,
            found=True,
            error_px=error_px,
            coverage_pct=coverage_pct,
            twist=twist,
            overlay=overlay,
        )

    def _steer_verbs(self, error_px: float):
        """Legacy joystick-verb steering. Returns (cmd, spd, twist=None)."""
        cfg = self.cfg
        abs_err = abs(error_px)
        if abs_err <= cfg.straight_band:
            return "F", cfg.speed_forward, None
        if abs_err <= cfg.diagonal_band:
            cmd = "DR" if error_px > 0 else "DL"
            return cmd, cfg.speed_diagonal, None
        cmd = "SR" if error_px > 0 else "SL"
        return cmd, cfg.speed_strafe, None

    def _steer_mecanum(self, error_px: float, frame_w: int):
        """True Mecanum steering: rotate toward the line, never strafe.

        Returns (cmd='MEC', spd, twist=(vx, vy=0, wz, spd)).

        Sign convention: error_px > 0 means line is to the RIGHT of the
        camera centre. The robot must rotate **right** (clockwise as
        seen from above) to face the line. Firmware convention: positive
        wz = rotate right (yaw CW). So `wz = kp * (error / (w/2))`
        directly works without sign flip.

        Dead band ⇒ pure forward (wz=0). Outside the band, wz scales
        with normalized error and is clamped to the configured |wz| cap.
        """
        cfg = self.cfg
        if abs(error_px) <= cfg.mec_straight_band:
            wz = 0
        else:
            err_norm = max(-1.0, min(1.0, error_px / (frame_w / 2.0)))
            wz_raw = cfg.mec_kp_steer * err_norm
            wz = int(max(-cfg.mec_max_wz, min(cfg.mec_max_wz, wz_raw)))
        twist = (int(cfg.mec_base_vx), 0, wz, int(cfg.mec_spd))
        return "MEC", int(cfg.mec_spd), twist

    def _build_overlay(self, frame, mask, roi_top, centroid_in_roi,
                        cmd, spd, status):
        """Lightweight overlay for MJPEG / debug viewing."""
        out = frame.copy()
        # Highlight the ROI strip.
        cv2.rectangle(out, (0, roi_top), (out.shape[1], out.shape[0]),
                      (0, 255, 0), 1)
        # Paint the mask in yellow on the overlay so the operator sees
        # what the controller saw.
        roi_view = out[roi_top:, :, :]
        roi_view[mask > 0] = (0, 255, 255)
        # Centre + centroid marker.
        cx_frame = out.shape[1] // 2
        cv2.line(out, (cx_frame, roi_top), (cx_frame, out.shape[0]),
                 (255, 0, 0), 1)
        if centroid_in_roi is not None:
            cx_roi = centroid_in_roi[0]
            cv2.circle(out, (cx_roi, roi_top + (out.shape[0] - roi_top) // 2),
                       6, (0, 0, 255), -1)
        # Status banner.
        cv2.rectangle(out, (0, 0), (out.shape[1], 28), (0, 0, 0), -1)
        cv2.putText(out,
                    f"SIMPLE LINE  {status}  cmd={cmd} spd={spd}",
                    (8, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 0) if status == "FOUND" else (0, 165, 255),
                    1, cv2.LINE_AA)
        return out
