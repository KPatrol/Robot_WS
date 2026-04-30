"""
occupancy_grid.py — 2D coverage + obstacle map for free-space navigation.

Backs FreeCoverage's wall-follow + frontier-exploration logic. Records:
    - which world cells the robot has visited (incremented on every pose update)
    - which cells were last seen as obstacles by ToF rays
    - frontier cells (unvisited but adjacent to visited free space)

Geometry
--------
The map is a fixed-size grid centered on the robot's start pose. Default
60×60 cells × 0.25 m = 15 m × 15 m square — large enough for typical
indoor patrol areas but small enough to fit in RAM (≈ 11 KB total) and
to serialize to disk in milliseconds.

World → cell
    i = round(x / cell_size_m) + origin_cell_x
    j = round(y / cell_size_m) + origin_cell_y

The robot is free to drive outside the map; rays/visits beyond the
border are silently dropped (in_bounds check). Use a wider grid if the
patrol area is bigger than 15 m on a side.

Cell semantics
--------------
flags         uint8  bitfield:
    bit 0 = VISITED        — robot has occupied this cell
    bit 1 = OBSTACLE       — last ToF ray endpoint hit this cell
visit_count   uint16        — saturating count, how many times traversed
obstacle_age  int16  ticks  — countdown to obstacle expiry (-1 = never seen)

Obstacles decay on a ~60 s timescale (1200 ticks @ 20 Hz) so the map
self-heals when objects move (chairs, people, doors).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np


# Bitfield flags — packed into uint8 cell array.
CELL_VISITED  = 1 << 0
CELL_OBSTACLE = 1 << 1


@dataclass
class GridConfig:
    cell_size_m:           float = 0.25   # 25 cm cells — coarse enough for fast updates
    width:                 int   = 60     # cells (15 m wide)
    height:                int   = 60     # cells
    origin_cell_x:         int   = 30     # robot starts at this cell (world (0,0))
    origin_cell_y:         int   = 30
    obstacle_decay_ticks:  int   = 1200   # ~60 s @ 20 Hz before obstacle expires
    ray_max_range_m:       float = 1.5    # ToF readings ≥ this treated as "free"


class OccupancyGrid:
    """Visit + obstacle map. Not thread-safe — call from one thread (nav loop)."""

    def __init__(self, config: Optional[GridConfig] = None):
        cfg = config or GridConfig()
        self.cfg = cfg
        self.visit_count  = np.zeros((cfg.width, cfg.height), dtype=np.uint16)
        self.obstacle_age = np.full((cfg.width, cfg.height), -1, dtype=np.int16)
        self.flags        = np.zeros((cfg.width, cfg.height), dtype=np.uint8)
        self._tick: int = 0

    # ── Coordinate conversions ────────────────────────────────────────

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        i = int(round(x / self.cfg.cell_size_m)) + self.cfg.origin_cell_x
        j = int(round(y / self.cfg.cell_size_m)) + self.cfg.origin_cell_y
        return i, j

    def cell_to_world(self, i: int, j: int) -> Tuple[float, float]:
        x = (i - self.cfg.origin_cell_x) * self.cfg.cell_size_m
        y = (j - self.cfg.origin_cell_y) * self.cfg.cell_size_m
        return x, y

    def in_bounds(self, i: int, j: int) -> bool:
        return 0 <= i < self.cfg.width and 0 <= j < self.cfg.height

    # ── Marking ────────────────────────────────────────────────────────

    def mark_visited(self, x: float, y: float) -> None:
        i, j = self.world_to_cell(x, y)
        if not self.in_bounds(i, j):
            return
        if self.visit_count[i, j] < 65000:
            self.visit_count[i, j] += 1
        self.flags[i, j] |= CELL_VISITED
        # Visiting a cell overrides any prior obstacle reading.
        self.flags[i, j] &= np.uint8(0xFF ^ CELL_OBSTACLE)
        self.obstacle_age[i, j] = -1

    def cast_ray(self, x0: float, y0: float, theta_world: float, dist_m: float) -> None:
        """Mark obstacle at the ray endpoint (if dist < max_range) and
        clear obstacles encountered along the way.

        theta_world: ray bearing in world frame, radians.
        dist_m:      measured ToF distance.
        """
        if dist_m <= 0.05:
            return
        endpoint_obstacle = dist_m < self.cfg.ray_max_range_m
        # Sample free cells every half-cell along the ray, stopping just
        # before the endpoint (which gets obstacle treatment instead).
        step = self.cfg.cell_size_m * 0.5
        n = max(1, int(dist_m / step) - 1)
        cos_t, sin_t = math.cos(theta_world), math.sin(theta_world)
        for k in range(1, n):
            xk = x0 + k * step * cos_t
            yk = y0 + k * step * sin_t
            i, j = self.world_to_cell(xk, yk)
            if not self.in_bounds(i, j):
                continue
            if self.flags[i, j] & CELL_OBSTACLE:
                # Confirmed free — accelerate the obstacle's decay.
                age = int(self.obstacle_age[i, j])
                if age > 0:
                    self.obstacle_age[i, j] = max(0, age - 5)
                if self.obstacle_age[i, j] <= 0:
                    self.flags[i, j] &= np.uint8(0xFF ^ CELL_OBSTACLE)
        if endpoint_obstacle:
            xe = x0 + dist_m * cos_t
            ye = y0 + dist_m * sin_t
            i, j = self.world_to_cell(xe, ye)
            if self.in_bounds(i, j) and not (self.flags[i, j] & CELL_VISITED):
                self.flags[i, j] |= CELL_OBSTACLE
                self.obstacle_age[i, j] = self.cfg.obstacle_decay_ticks

    def tick(self) -> None:
        """Advance internal tick — decay stale obstacles every ~3 s."""
        self._tick += 1
        if self._tick % 60 == 0:
            mask = self.obstacle_age > 0
            self.obstacle_age[mask] -= 60
            stale = (self.obstacle_age <= 0) & ((self.flags & CELL_OBSTACLE) > 0)
            if stale.any():
                self.flags[stale] &= np.uint8(0xFF ^ CELL_OBSTACLE)

    # ── Frontier search ───────────────────────────────────────────────

    def nearest_frontier(
        self,
        x: float,
        y: float,
        theta_rad: float,
        max_radius_m: float = 3.0,
        min_radius_m: float = 0.5,
        heading_weight: float = 0.4,
    ) -> Optional[Tuple[float, float, float]]:
        """Find an unvisited cell adjacent to a visited one (a frontier).

        Returns (frontier_x, frontier_y, bearing_rad) — None if no
        frontier within range. Cost = euclidean distance + heading
        misalignment penalty (so the robot prefers frontiers it's
        already pointing at, reducing rotate-stop-rotate churn).
        """
        ix, iy = self.world_to_cell(x, y)
        if not self.in_bounds(ix, iy):
            return None
        max_cells = max(2, int(max_radius_m / self.cfg.cell_size_m))
        min_cells = max(1, int(min_radius_m / self.cfg.cell_size_m))

        best_cost: float = float("inf")
        best_ij: Optional[Tuple[int, int]] = None

        for di in range(-max_cells, max_cells + 1):
            for dj in range(-max_cells, max_cells + 1):
                d2 = di * di + dj * dj
                if d2 < min_cells * min_cells or d2 > max_cells * max_cells:
                    continue
                i, j = ix + di, iy + dj
                if not self.in_bounds(i, j):
                    continue
                f = int(self.flags[i, j])
                if f & (CELL_OBSTACLE | CELL_VISITED):
                    continue
                # Frontier definition: unvisited free cell with at least one
                # 8-connected visited neighbor (so it's reachable from known space).
                adj_visited = False
                for ddi in (-1, 0, 1):
                    if adj_visited:
                        break
                    for ddj in (-1, 0, 1):
                        if ddi == 0 and ddj == 0:
                            continue
                        ai, aj = i + ddi, j + ddj
                        if self.in_bounds(ai, aj) and (self.flags[ai, aj] & CELL_VISITED):
                            adj_visited = True
                            break
                if not adj_visited:
                    continue
                fx, fy = self.cell_to_world(i, j)
                bearing = math.atan2(fy - y, fx - x)
                heading_err = abs(_wrap_pi(bearing - theta_rad))
                cost = math.hypot(fx - x, fy - y) + heading_weight * heading_err
                if cost < best_cost:
                    best_cost = cost
                    best_ij = (i, j)

        if best_ij is None:
            return None
        i, j = best_ij
        fx, fy = self.cell_to_world(i, j)
        return fx, fy, math.atan2(fy - y, fx - x)

    # ── Diagnostics + persistence ─────────────────────────────────────

    def coverage_pct(self) -> float:
        total = self.cfg.width * self.cfg.height
        if total == 0:
            return 0.0
        visited = int(np.count_nonzero((self.flags & CELL_VISITED) > 0))
        return visited / total

    def diagnostics(self) -> dict:
        return {
            "tick":          self._tick,
            "coverage_pct":  round(self.coverage_pct() * 100, 1),
            "visited":       int(np.count_nonzero((self.flags & CELL_VISITED) > 0)),
            "obstacles":     int(np.count_nonzero((self.flags & CELL_OBSTACLE) > 0)),
            "size":          [self.cfg.width, self.cfg.height],
            "cell_size_m":   self.cfg.cell_size_m,
        }

    def save_npz(self, path: str) -> None:
        np.savez_compressed(
            path,
            visit_count=self.visit_count,
            obstacle_age=self.obstacle_age,
            flags=self.flags,
            origin=np.array([self.cfg.origin_cell_x, self.cfg.origin_cell_y], dtype=np.int32),
            cell_size=np.array([self.cfg.cell_size_m], dtype=np.float32),
        )

    def load_npz(self, path: str) -> bool:
        try:
            data = np.load(path)
        except Exception:
            return False
        try:
            vc = data["visit_count"]
            oa = data["obstacle_age"]
            fl = data["flags"]
        except KeyError:
            return False
        if vc.shape != (self.cfg.width, self.cfg.height):
            return False
        self.visit_count  = vc.astype(np.uint16, copy=False)
        self.obstacle_age = oa.astype(np.int16, copy=False)
        self.flags        = fl.astype(np.uint8, copy=False)
        return True


def _wrap_pi(angle_rad: float) -> float:
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad


__all__ = ["OccupancyGrid", "GridConfig", "CELL_VISITED", "CELL_OBSTACLE"]
