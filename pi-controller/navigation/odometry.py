"""
odometry.py — Mecanum forward kinematics + complementary filter.

Fuses 4-wheel encoder counts (374 PPR × 4 quadrature edges = 1496 cnt/rev,
97 mm wheel ⇒ 4905.6 cnt/m) with BNO08x IMU yaw to estimate body pose
(x, y, θ) in the world frame at ~20 Hz.

Mecanum forward kinematics (X-config, wheels FL/FR/BL/BR):
    vx =  (R/4) * ( ω_FL + ω_FR + ω_BL + ω_BR )
    vy =  (R/4) * (-ω_FL + ω_FR + ω_BL - ω_BR )
    ωz =  (R / (4*(lx+ly))) * (-ω_FL + ω_FR - ω_BL + ω_BR )

Frame integration (world frame):
    x  ← x + (vx*cosθ - vy*sinθ)*dt
    y  ← y + (vx*sinθ + vy*cosθ)*dt
    θ  ← complementary_filter(θ_enc, θ_imu)

Complementary filter:
    θ_fused = (1-α) * (θ_prev + ωz*dt)  +  α * unwrap(θ_imu)
    α ≈ 0.02 — IMU pulls slowly to suppress encoder drift while filtering
    short-term IMU noise. Encoder dominates dynamics; IMU dominates DC.

Drift sources & mitigations:
    - Wheel slip on smooth tile  → IMU correction reins it in
    - IMU yaw bias ~0.5°/min     → encoder dominates per-tick
    - Encoder mismatch (mecanum) → tune wheel_radius / robot_lx_ly to
                                   measured straight-line and rotation
"""

from __future__ import annotations

import math
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Tuple


# ----- robot geometry constants (KPATROL V10) ------------------------
WHEEL_RADIUS_M       = 0.0485   # 97 mm dia ÷ 2
COUNTS_PER_REV       = 1496     # 374 PPR × 4 quadrature edges
COUNTS_PER_METER     = COUNTS_PER_REV / (2 * math.pi * WHEEL_RADIUS_M)
ROBOT_LX             = 0.115    # half wheelbase X (front↔back centerline)
ROBOT_LY             = 0.135    # half wheelbase Y (left↔right centerline)
LXLY_SUM             = ROBOT_LX + ROBOT_LY


def _wrap_pi(angle_rad: float) -> float:
    """Normalize an angle to [-π, π]."""
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad


@dataclass
class Pose:
    """Robot pose in world frame (meters, radians, unix seconds)."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0   # radians, +CCW
    vx: float = 0.0      # m/s, body frame
    vy: float = 0.0      # m/s, body frame
    omega: float = 0.0   # rad/s, body frame
    timestamp: float = field(default_factory=time.time)
    source: str = "fused"   # "fused" | "encoder" | "imu_only"

    def to_dict(self) -> Dict:
        return {
            "x":         round(self.x, 4),
            "y":         round(self.y, 4),
            "theta":     round(self.theta, 4),
            "theta_deg": round(math.degrees(self.theta), 2),
            "vx":        round(self.vx, 3),
            "vy":        round(self.vy, 3),
            "omega":     round(self.omega, 3),
            "ts":        round(self.timestamp, 3),
            "source":    self.source,
        }


class Odometry:
    """4-wheel Mecanum odometry with complementary IMU yaw fusion.

    Thread-safe — `update()` may be called from the main loop while
    `get_pose()` runs from MQTT publish threads.
    """

    def __init__(
        self,
        wheel_radius: float = WHEEL_RADIUS_M,
        counts_per_rev: int = COUNTS_PER_REV,
        lx: float = ROBOT_LX,
        ly: float = ROBOT_LY,
        imu_alpha: float = 0.02,    # 0=encoder only, 1=IMU only
        slip_max_dps: float = 720.0  # reject single-tick |dθ_enc| spikes
    ):
        self._lock = threading.Lock()
        self._wheel_r          = wheel_radius
        self._cpr              = counts_per_rev
        self._cpm              = counts_per_rev / (2 * math.pi * wheel_radius)
        self._lxly             = lx + ly
        self._alpha            = float(max(0.0, min(1.0, imu_alpha)))
        self._slip_max_rad     = math.radians(slip_max_dps)

        # State
        self._pose             = Pose(source="fused")
        self._initialized      = False
        self._last_t           = 0.0

        # Last raw inputs (for differencing)
        self._last_counts: Dict[str, int] = {"FR": 0, "FL": 0, "BR": 0, "BL": 0}
        self._imu_offset       = 0.0       # set on first IMU sample
        self._imu_seen         = False

        # Diagnostics
        self.update_count      = 0
        self.last_dt           = 0.0
        self.last_dtheta_imu   = 0.0
        self.last_dtheta_enc   = 0.0

    # ------------------------------------------------------------------

    def reset(self, x: float = 0.0, y: float = 0.0, theta_deg: float = 0.0) -> None:
        """Force-set the pose. Useful when entering a known waypoint."""
        with self._lock:
            self._pose = Pose(
                x=x, y=y, theta=math.radians(theta_deg),
                source="fused",
            )
            self._initialized = False
            self._last_t = 0.0
            self._imu_seen = False

    def get_pose(self) -> Pose:
        with self._lock:
            return Pose(
                x=self._pose.x, y=self._pose.y, theta=self._pose.theta,
                vx=self._pose.vx, vy=self._pose.vy, omega=self._pose.omega,
                timestamp=self._pose.timestamp, source=self._pose.source,
            )

    # ------------------------------------------------------------------

    def update(
        self,
        encoder_counts: Dict[str, int],
        imu_yaw_deg: Optional[float] = None,
        now: Optional[float] = None,
    ) -> Pose:
        """Advance the pose estimate by one tick.

        encoder_counts must contain "FR", "FL", "BR", "BL" cumulative counts.
        imu_yaw_deg is optional — when None, falls back to encoder-only (still
        publishable but accumulates yaw drift over time).
        """
        now = now if now is not None else time.time()

        with self._lock:
            if not self._initialized:
                self._last_counts = {k: int(encoder_counts.get(k, 0)) for k in ("FR", "FL", "BR", "BL")}
                self._last_t = now
                if imu_yaw_deg is not None:
                    self._imu_offset = math.radians(imu_yaw_deg)
                    self._imu_seen = True
                self._pose.timestamp = now
                self._initialized = True
                return self._pose_copy()

            dt = max(1e-3, now - self._last_t)

            # Per-wheel angular velocity (rad/s) — cumulative-count delta
            # ÷ counts_per_rev × 2π ÷ dt
            d = {}
            for k in ("FR", "FL", "BR", "BL"):
                cur  = int(encoder_counts.get(k, self._last_counts[k]))
                d[k] = (cur - self._last_counts[k]) * (2 * math.pi / self._cpr)
                self._last_counts[k] = cur

            wheel_omega = {k: d[k] / dt for k in d}   # rad/s

            # Mecanum forward kinematics — solve for body twist (vx, vy, ωz)
            # Sign convention: +vx forward, +vy left, +ωz CCW.
            # Wheel ω sign convention: positive = wheel spins forward.
            r = self._wheel_r
            vx = (r / 4.0) * ( wheel_omega["FL"] + wheel_omega["FR"]
                             + wheel_omega["BL"] + wheel_omega["BR"])
            vy = (r / 4.0) * (-wheel_omega["FL"] + wheel_omega["FR"]
                             + wheel_omega["BL"] - wheel_omega["BR"])
            wz = (r / (4.0 * self._lxly)) * (
                -wheel_omega["FL"] + wheel_omega["FR"]
                - wheel_omega["BL"] + wheel_omega["BR"]
            )

            # World-frame integration (use yaw mid-point to reduce error)
            theta_mid = self._pose.theta + 0.5 * wz * dt
            cos_t = math.cos(theta_mid)
            sin_t = math.sin(theta_mid)
            self._pose.x += (vx * cos_t - vy * sin_t) * dt
            self._pose.y += (vx * sin_t + vy * cos_t) * dt

            # Heading: complementary filter — encoder ωz integrates fast,
            # IMU yaw pulls slowly to cancel slip drift.
            theta_enc_pred = self._pose.theta + wz * dt

            theta_fused = theta_enc_pred
            source = "encoder"
            if imu_yaw_deg is not None:
                imu_rad = math.radians(imu_yaw_deg)
                if not self._imu_seen:
                    # First IMU sample — anchor offset so fused starts at
                    # current pose theta (avoids a discontinuity).
                    self._imu_offset = imu_rad - self._pose.theta
                    self._imu_seen = True
                theta_imu = _wrap_pi(imu_rad - self._imu_offset)

                # Reject implausible ENC step (slip spike) → trust IMU more
                d_enc = abs(_wrap_pi(theta_enc_pred - self._pose.theta))
                if d_enc > self._slip_max_rad * dt:
                    alpha = min(1.0, self._alpha * 5.0)  # spike → faster IMU pull
                else:
                    alpha = self._alpha

                # Wrap-aware blend
                err = _wrap_pi(theta_imu - theta_enc_pred)
                theta_fused = _wrap_pi(theta_enc_pred + alpha * err)
                self.last_dtheta_imu = err
                source = "fused"

            self.last_dtheta_enc = wz * dt
            self._pose.theta     = _wrap_pi(theta_fused)
            self._pose.vx        = vx
            self._pose.vy        = vy
            self._pose.omega     = wz
            self._pose.timestamp = now
            self._pose.source    = source
            self._last_t         = now
            self.update_count   += 1
            self.last_dt         = dt
            return self._pose_copy()

    # ------------------------------------------------------------------

    def _pose_copy(self) -> Pose:
        return Pose(
            x=self._pose.x, y=self._pose.y, theta=self._pose.theta,
            vx=self._pose.vx, vy=self._pose.vy, omega=self._pose.omega,
            timestamp=self._pose.timestamp, source=self._pose.source,
        )

    # ------------------------------------------------------------------

    def diagnostics(self) -> Dict:
        with self._lock:
            return {
                "updates":         self.update_count,
                "last_dt":         round(self.last_dt, 3),
                "imu_alpha":       self._alpha,
                "imu_seen":        self._imu_seen,
                "last_dtheta_imu": round(math.degrees(self.last_dtheta_imu), 2),
                "last_dtheta_enc": round(math.degrees(self.last_dtheta_enc), 2),
                "counts_per_m":    round(self._cpm, 1),
                "lxly_sum":        self._lxly,
            }


__all__ = ["Pose", "Odometry", "WHEEL_RADIUS_M", "COUNTS_PER_REV",
           "COUNTS_PER_METER", "ROBOT_LX", "ROBOT_LY"]
