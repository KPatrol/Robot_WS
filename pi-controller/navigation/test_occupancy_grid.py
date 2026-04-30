"""Unit tests for OccupancyGrid.

Run:  python -m pytest robots/pi-controller/navigation/test_occupancy_grid.py
Or:   python -m unittest robots.pi-controller.navigation.test_occupancy_grid
"""

from __future__ import annotations

import math
import os
import tempfile
import unittest

import numpy as np

from .occupancy_grid import (
    CELL_OBSTACLE,
    CELL_VISITED,
    GridConfig,
    OccupancyGrid,
)


class GridGeometry(unittest.TestCase):
    def setUp(self):
        self.grid = OccupancyGrid(GridConfig(
            cell_size_m=0.25, width=20, height=20,
            origin_cell_x=10, origin_cell_y=10,
        ))

    def test_world_to_cell_origin(self):
        i, j = self.grid.world_to_cell(0.0, 0.0)
        self.assertEqual((i, j), (10, 10))

    def test_world_to_cell_quarter_meter(self):
        # 0.25 m = exactly 1 cell from origin
        self.assertEqual(self.grid.world_to_cell(0.25, 0.0), (11, 10))
        self.assertEqual(self.grid.world_to_cell(0.0, -0.25), (10, 9))

    def test_cell_to_world_roundtrip(self):
        for x, y in [(0.5, 1.25), (-0.75, 2.0), (0.0, 0.0)]:
            i, j = self.grid.world_to_cell(x, y)
            wx, wy = self.grid.cell_to_world(i, j)
            self.assertAlmostEqual(wx, round(x / 0.25) * 0.25, places=6)
            self.assertAlmostEqual(wy, round(y / 0.25) * 0.25, places=6)

    def test_in_bounds(self):
        self.assertTrue(self.grid.in_bounds(0, 0))
        self.assertTrue(self.grid.in_bounds(19, 19))
        self.assertFalse(self.grid.in_bounds(-1, 0))
        self.assertFalse(self.grid.in_bounds(20, 0))
        self.assertFalse(self.grid.in_bounds(10, 20))


class VisitMarking(unittest.TestCase):
    def test_mark_visited_increments_count(self):
        g = OccupancyGrid(GridConfig(width=10, height=10, origin_cell_x=5, origin_cell_y=5))
        for _ in range(3):
            g.mark_visited(0.0, 0.0)
        self.assertEqual(int(g.visit_count[5, 5]), 3)
        self.assertTrue(g.flags[5, 5] & CELL_VISITED)

    def test_mark_visited_clears_obstacle(self):
        g = OccupancyGrid(GridConfig(width=10, height=10, origin_cell_x=5, origin_cell_y=5))
        g.flags[5, 5] = CELL_OBSTACLE
        g.obstacle_age[5, 5] = 1000
        g.mark_visited(0.0, 0.0)
        self.assertFalse(g.flags[5, 5] & CELL_OBSTACLE)
        self.assertEqual(int(g.obstacle_age[5, 5]), -1)

    def test_mark_visited_outside_bounds_is_silent(self):
        g = OccupancyGrid(GridConfig(width=10, height=10, origin_cell_x=5, origin_cell_y=5))
        # 100 m way outside the 10×0.25 = 2.5 m grid — should not raise.
        g.mark_visited(100.0, 100.0)


class RayCasting(unittest.TestCase):
    def test_cast_ray_marks_endpoint_obstacle(self):
        g = OccupancyGrid(GridConfig(
            cell_size_m=0.25, width=20, height=20,
            origin_cell_x=10, origin_cell_y=10, ray_max_range_m=1.5,
        ))
        # Ray pointing east (theta=0) for 0.5 m → endpoint at (0.5, 0).
        g.cast_ray(0.0, 0.0, 0.0, 0.5)
        ei, ej = g.world_to_cell(0.5, 0.0)
        self.assertTrue(g.flags[ei, ej] & CELL_OBSTACLE)

    def test_cast_ray_long_distance_no_endpoint_obstacle(self):
        # Distance > max_range → "free" reading; endpoint should NOT be marked.
        g = OccupancyGrid(GridConfig(ray_max_range_m=1.5))
        g.cast_ray(0.0, 0.0, 0.0, 2.0)
        ei, ej = g.world_to_cell(2.0, 0.0)
        if g.in_bounds(ei, ej):
            self.assertFalse(g.flags[ei, ej] & CELL_OBSTACLE)

    def test_cast_ray_skips_visited_endpoint(self):
        g = OccupancyGrid(GridConfig(width=20, height=20, origin_cell_x=10, origin_cell_y=10))
        # Pre-visit the would-be endpoint.
        g.mark_visited(0.5, 0.0)
        g.cast_ray(0.0, 0.0, 0.0, 0.5)
        ei, ej = g.world_to_cell(0.5, 0.0)
        self.assertFalse(g.flags[ei, ej] & CELL_OBSTACLE)


class FrontierSearch(unittest.TestCase):
    def test_no_frontier_when_grid_empty(self):
        g = OccupancyGrid(GridConfig(width=20, height=20, origin_cell_x=10, origin_cell_y=10))
        self.assertIsNone(g.nearest_frontier(0.0, 0.0, 0.0, max_radius_m=2.0))

    def test_frontier_adjacent_to_visited_cell(self):
        g = OccupancyGrid(GridConfig(
            cell_size_m=0.25, width=20, height=20,
            origin_cell_x=10, origin_cell_y=10,
        ))
        # Visit a small cluster centered at the origin so a ring of unvisited
        # cells around it qualifies as frontier.
        for di in range(-2, 3):
            for dj in range(-2, 3):
                wx, wy = di * 0.25, dj * 0.25
                g.mark_visited(wx, wy)
        # Search from origin: nearest frontier should be a cell adjacent to the cluster.
        result = g.nearest_frontier(0.0, 0.0, 0.0, max_radius_m=2.0, min_radius_m=0.1)
        self.assertIsNotNone(result)
        fx, fy, _bearing = result
        # The frontier should sit in the ring just outside the visited cluster.
        ring_dist = math.hypot(fx, fy)
        self.assertGreater(ring_dist, 0.5)   # outside the central 2-cell cluster
        self.assertLess(ring_dist, 1.5)      # not far away

    def test_frontier_skips_obstacles(self):
        g = OccupancyGrid(GridConfig(width=20, height=20, origin_cell_x=10, origin_cell_y=10))
        # Mark a single visited cell + its 8 neighbors as obstacles → no frontier.
        g.mark_visited(0.0, 0.0)
        for di, dj in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:
            g.flags[10 + di, 10 + dj] = CELL_OBSTACLE
        self.assertIsNone(g.nearest_frontier(0.0, 0.0, 0.0, max_radius_m=1.0, min_radius_m=0.1))


class TickDecay(unittest.TestCase):
    def test_obstacle_age_decays_to_zero(self):
        g = OccupancyGrid(GridConfig(obstacle_decay_ticks=120))
        i, j = g.world_to_cell(0.5, 0.0)
        g.flags[i, j] = CELL_OBSTACLE
        g.obstacle_age[i, j] = 120
        # Tick 60 times triggers one decay step (60 ticks decrement).
        for _ in range(60):
            g.tick()
        self.assertEqual(int(g.obstacle_age[i, j]), 60)
        # Another 60 ticks: decay to 0 → flag cleared.
        for _ in range(60):
            g.tick()
        self.assertFalse(g.flags[i, j] & CELL_OBSTACLE)


class Persistence(unittest.TestCase):
    def test_save_and_load_roundtrip(self):
        g = OccupancyGrid(GridConfig(width=10, height=10, origin_cell_x=5, origin_cell_y=5))
        g.mark_visited(0.0, 0.0)
        g.mark_visited(0.25, 0.0)
        g.flags[3, 3] = CELL_OBSTACLE
        g.obstacle_age[3, 3] = 800

        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "map.npz")
            g.save_npz(path)
            g2 = OccupancyGrid(GridConfig(width=10, height=10, origin_cell_x=5, origin_cell_y=5))
            self.assertTrue(g2.load_npz(path))
            np.testing.assert_array_equal(g.flags, g2.flags)
            np.testing.assert_array_equal(g.visit_count, g2.visit_count)
            np.testing.assert_array_equal(g.obstacle_age, g2.obstacle_age)

    def test_load_rejects_shape_mismatch(self):
        g = OccupancyGrid(GridConfig(width=10, height=10))
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "wrong.npz")
            g.save_npz(path)
            g2 = OccupancyGrid(GridConfig(width=20, height=20))
            self.assertFalse(g2.load_npz(path))


class CoveragePct(unittest.TestCase):
    def test_coverage_pct_increases_with_visits(self):
        g = OccupancyGrid(GridConfig(width=10, height=10, origin_cell_x=5, origin_cell_y=5))
        self.assertEqual(g.coverage_pct(), 0.0)
        g.mark_visited(0.0, 0.0)
        self.assertGreater(g.coverage_pct(), 0.0)
        # 1 cell out of 100 = 1 %.
        self.assertAlmostEqual(g.coverage_pct(), 0.01, places=4)


if __name__ == "__main__":
    unittest.main()
