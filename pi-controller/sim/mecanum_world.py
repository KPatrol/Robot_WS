"""
mecanum_world.py — Lightweight 2-D world + Mecanum robot simulator.

Built to validate the navigation stack (FreeCoverage / Odometry / VelocityPID)
end-to-end on a developer laptop without robot hardware. Deliberately minimal:
no rendering dependency, no rigid-body solver, no motor model — just enough
physics so that the FSM, occupancy grid, and pose estimator see realistic
inputs.

World
-----
Boolean occupancy grid (cell_size_m × cell_size_m). Supports add_box,
add_wall, and a fast 2D ray-march for ToF simulation.

Robot
-----
Body twist (vx, vy, ωz) is the command interface — the same one FreeCoverage
emits. A first-order velocity ramp models actuator lag (tau ≈ 50 ms). Pose
integrates in world frame using the mid-point yaw. Collisions clip the next
position to the previous valid one (no bounce, no slide — the FSM's emergency
reverse handles wall contact).

Sensors
-------
- ToFs: 6 rays in standard K-Patrol layout (front, FL, FR, left, right, rear).
        Returned in millimetres. Capped at max_range_m.
- Encoders: cumulative count per wheel via mecanum inverse kinematics.
- IMU yaw: world-frame θ in degrees with optional Gaussian noise.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import numpy as np

from navigation.odometry import (
    COUNTS_PER_REV,
    ROBOT_LX,
    ROBOT_LY,
    WHEEL_RADIUS_M,
)


_LXLY = ROBOT_LX + ROBOT_LY


# ToF ray bearings relative to robot's heading (radians).
# Order matches the typical K-Patrol mounting:
#   front (0°), front-left (+45°), front-right (-45°), left (+90°),
#   right (-90°), rear (180°).
TOF_BEARINGS_RAD: Dict[str, float] = {
    "front":       0.0,
    "front_left":  math.pi / 4,
    "front_right": -math.pi / 4,
    "left":        math.pi / 2,
    "right":       -math.pi / 2,
    "rear":        math.pi,
}


@dataclass
class WorldConfig:
    size_m:        float = 12.0   # square world side (m)
    cell_m:        float = 0.05   # 5 cm grid — fine enough for raycasting
    tof_max_m:     float = 2.0    # VL53L0X max usable range
    tof_step_m:    float = 0.04   # ray march step (≈ 0.8 cell)
    encoder_noise: float = 0.0    # std dev of per-tick count noise (counts)
    imu_noise_deg: float = 0.0    # std dev of yaw noise (degrees)
    robot_radius:  float = 0.20   # collision radius (m)


class World:
    """2-D occupancy grid world.

    Cells with True are blocked. Origin (0,0) sits at the centre of the grid
    so the robot can drive in any direction from spawn.
    """

    def __init__(self, config: Optional[WorldConfig] = None):
        cfg = config or WorldConfig()
        self.cfg = cfg
        n = int(round(cfg.size_m / cfg.cell_m))
        if n % 2 != 0:
            n += 1
        self.n = n
        self.occ = np.zeros((n, n), dtype=bool)
        self._cx = n // 2
        self._cy = n // 2

    # ── geometry ──────────────────────────────────────────────────────

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        i = int(round(x / self.cfg.cell_m)) + self._cx
        j = int(round(y / self.cfg.cell_m)) + self._cy
        return i, j

    def in_bounds(self, i: int, j: int) -> bool:
        return 0 <= i < self.n and 0 <= j < self.n

    def is_occupied(self, x: float, y: float) -> bool:
        i, j = self.world_to_cell(x, y)
        if not self.in_bounds(i, j):
            return True   # outside-world counts as wall
        return bool(self.occ[i, j])

    def is_circle_blocked(self, x: float, y: float, radius_m: float) -> bool:
        """True if a disk of radius_m at (x,y) overlaps any occupied cell."""
        r_cells = max(1, int(math.ceil(radius_m / self.cfg.cell_m)))
        ci, cj = self.world_to_cell(x, y)
        for di in range(-r_cells, r_cells + 1):
            for dj in range(-r_cells, r_cells + 1):
                if di * di + dj * dj > r_cells * r_cells:
                    continue
                i, j = ci + di, cj + dj
                if not self.in_bounds(i, j) or self.occ[i, j]:
                    return True
        return False

    # ── building blocks ───────────────────────────────────────────────

    def add_box(self, x_min: float, y_min: float,
                x_max: float, y_max: float, value: bool = True) -> None:
        """Fill an axis-aligned rectangle (world coords)."""
        i0, j0 = self.world_to_cell(min(x_min, x_max), min(y_min, y_max))
        i1, j1 = self.world_to_cell(max(x_min, x_max), max(y_min, y_max))
        i0, j0 = max(0, i0), max(0, j0)
        i1, j1 = min(self.n - 1, i1), min(self.n - 1, j1)
        self.occ[i0:i1 + 1, j0:j1 + 1] = value

    def add_wall(self, x0: float, y0: float, x1: float, y1: float,
                 thickness_m: float = 0.05) -> None:
        """Draw a thin wall segment."""
        steps = max(2, int(math.hypot(x1 - x0, y1 - y0) / self.cfg.cell_m) + 1)
        for k in range(steps + 1):
            t = k / steps
            x = x0 + t * (x1 - x0)
            y = y0 + t * (y1 - y0)
            self.add_box(x - thickness_m / 2, y - thickness_m / 2,
                         x + thickness_m / 2, y + thickness_m / 2)

    def add_room_walls(self, x_min: float, y_min: float,
                       x_max: float, y_max: float,
                       thickness_m: float = 0.05) -> None:
        """Hollow rectangular room (4 walls only, interior open)."""
        self.add_wall(x_min, y_min, x_max, y_min, thickness_m)
        self.add_wall(x_max, y_min, x_max, y_max, thickness_m)
        self.add_wall(x_max, y_max, x_min, y_max, thickness_m)
        self.add_wall(x_min, y_max, x_min, y_min, thickness_m)

    # ── sensing ────────────────────────────────────────────────────────

    def raycast(self, x: float, y: float, theta: float,
                max_range_m: Optional[float] = None) -> float:
        """March until occupied or max_range. Returns hit distance (m)."""
        max_range = max_range_m if max_range_m is not None else self.cfg.tof_max_m
        step = self.cfg.tof_step_m
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        dist = 0.0
        # Skip the cell the robot is sitting in.
        dist = step
        while dist < max_range:
            xk = x + dist * cos_t
            yk = y + dist * sin_t
            if self.is_occupied(xk, yk):
                return dist
            dist += step
        return max_range


# ─────────────────────────────────────────────────────────────────────


@dataclass
class _RobotState:
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    vx: float = 0.0
    vy: float = 0.0
    wz: float = 0.0


class RobotSim:
    """Mecanum kinematic robot in a World.

    Drive interface: ``step(vx_cmd, vy_cmd, wz_cmd, dt)``. The robot ramps
    its actual velocity toward the command with a first-order lag, integrates
    pose, accumulates encoder counts, and exposes ``tofs_mm()`` /
    ``imu_yaw_deg()`` / ``encoder_counts``.
    """

    def __init__(
        self,
        world: World,
        x: float = 0.0,
        y: float = 0.0,
        theta_rad: float = 0.0,
        ramp_tau_s: float = 0.05,
        rng: Optional[random.Random] = None,
    ):
        self.world = world
        self.state = _RobotState(x=x, y=y, theta=theta_rad)
        self.ramp_tau = max(1e-3, ramp_tau_s)
        self.encoder_counts: Dict[str, int] = {"FR": 0, "FL": 0, "BR": 0, "BL": 0}
        self._collisions = 0
        self._rng = rng or random.Random(42)

    # ------------------------------------------------------------------

    def reset(self, x: float = 0.0, y: float = 0.0, theta_rad: float = 0.0) -> None:
        self.state = _RobotState(x=x, y=y, theta=theta_rad)
        self.encoder_counts = {"FR": 0, "FL": 0, "BR": 0, "BL": 0}
        self._collisions = 0

    # ------------------------------------------------------------------

    def step(self, vx_cmd: float, vy_cmd: float, wz_cmd: float, dt: float) -> None:
        """Advance one tick. dt in seconds."""
        s = self.state
        # First-order velocity ramp — fakes motor + drivetrain lag.
        a = dt / (self.ramp_tau + dt)
        s.vx += a * (vx_cmd - s.vx)
        s.vy += a * (vy_cmd - s.vy)
        s.wz += a * (wz_cmd - s.wz)

        # Integrate pose using the mid-point yaw (matches Odometry's convention).
        theta_mid = s.theta + 0.5 * s.wz * dt
        cos_t = math.cos(theta_mid)
        sin_t = math.sin(theta_mid)
        new_x = s.x + (s.vx * cos_t - s.vy * sin_t) * dt
        new_y = s.y + (s.vx * sin_t + s.vy * cos_t) * dt
        new_theta = s.theta + s.wz * dt

        # Collision: if the proposed position would overlap an obstacle, stop
        # translation but allow the rotation. The FSM's emergency reverse will
        # back the robot off when ToF readings spike.
        if self.world.is_circle_blocked(new_x, new_y, self.world.cfg.robot_radius):
            self._collisions += 1
            s.vx = 0.0
            s.vy = 0.0
        else:
            s.x = new_x
            s.y = new_y
        s.theta = self._wrap(new_theta)

        # Update encoder counts (mecanum inverse kinematics, body frame).
        v_fl = s.vx - s.vy - s.wz * _LXLY
        v_fr = s.vx + s.vy + s.wz * _LXLY
        v_bl = s.vx + s.vy - s.wz * _LXLY
        v_br = s.vx - s.vy + s.wz * _LXLY
        wheel_omega = {
            "FL": v_fl / WHEEL_RADIUS_M,
            "FR": v_fr / WHEEL_RADIUS_M,
            "BL": v_bl / WHEEL_RADIUS_M,
            "BR": v_br / WHEEL_RADIUS_M,
        }
        scale = COUNTS_PER_REV / (2 * math.pi)
        for k, w in wheel_omega.items():
            d_counts = w * dt * scale
            if self.world.cfg.encoder_noise > 0:
                d_counts += self._rng.gauss(0.0, self.world.cfg.encoder_noise)
            self.encoder_counts[k] += int(round(d_counts))

    # ── sensing ────────────────────────────────────────────────────────

    def tofs_mm(self) -> Dict[str, int]:
        """Return all 6 ToF readings in mm (capped at world max range)."""
        s = self.state
        out = {}
        for name, bearing in TOF_BEARINGS_RAD.items():
            d = self.world.raycast(s.x, s.y, s.theta + bearing)
            out[name] = int(round(d * 1000))
        return out

    def imu_yaw_deg(self) -> float:
        deg = math.degrees(self.state.theta)
        if self.world.cfg.imu_noise_deg > 0:
            deg += self._rng.gauss(0.0, self.world.cfg.imu_noise_deg)
        return deg

    def pose(self) -> Tuple[float, float, float]:
        return self.state.x, self.state.y, self.state.theta

    # ── diagnostics ───────────────────────────────────────────────────

    @property
    def collisions(self) -> int:
        return self._collisions

    @staticmethod
    def _wrap(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


# ─────────────────────────────────────────────────────────────────────


def make_demo_world(cfg: Optional[WorldConfig] = None) -> World:
    """A 6×6 m room with two interior obstacles — good baseline for FreeCoverage."""
    world = World(cfg)
    world.add_room_walls(-3.0, -3.0, 3.0, 3.0, thickness_m=0.10)
    world.add_box(0.5, -1.0, 1.5, -0.3)   # box obstacle (e.g. a chair)
    world.add_box(-1.5, 0.8, -0.6, 1.6)   # second obstacle
    return world


__all__ = ["World", "WorldConfig", "RobotSim", "make_demo_world", "TOF_BEARINGS_RAD"]
