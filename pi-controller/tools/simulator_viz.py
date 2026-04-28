#!/usr/bin/env python3
"""
simulator_viz.py — Visual simulator for K-Patrol SMART_COVERAGE + marker boundary
===================================================================================
A top-down matplotlib visualization that drives the REAL navigation code
(MarkerBoundary + SmartCoveragePlanner + VFH) inside a simulated room.

What you see:
    • Room walls (black)
    • Obstacles (grey boxes)
    • Marker pillars   —  HOME red, CORNER green, labeled H/1/2/3/4
    • Robot triangle   —  red, pointing forward
    • Camera FOV cone  —  yellow wedge
    • Movement trail   —  blue line
    • Progressive polygon boundary — green dashed (disk → corridor → polygon)
    • Visited cells    —  light green overlay (from real MiniGrid)
    • Live status box  —  mode, phase, boundary, coverage, collisions

Robot runs 3 phases automatically:
    1. Discovery — slowly rotates to discover markers around it
    2. Coverage  — SMART_COVERAGE inside the polygon
    3. Return home — navigates back to marker ID 0 when coverage ≥ 60%

Keyboard:
    space   pause / resume
    r       reset current scenario
    1-3     switch scenario (1=square, 2=rectangle, 3=triangle)
    q       quit

Usage (from robots/pi-controller/):
    python3 tools/simulator_viz.py
    python3 tools/simulator_viz.py --scenario rectangle
    python3 tools/simulator_viz.py --speed 30         # faster animation
    python3 tools/simulator_viz.py --no-autohome       # don't auto return home
"""

import argparse
import math
import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from navigation import (  # noqa: E402
    Odometer, MiniGrid, VFH, NavController, Mode,
    MarkerBoundary, HOME_ID,
)
from navigation.mini_grid import UNKNOWN, FREE, OBSTACLE  # noqa: E402


# ═════════════════════════════════════════════════════════════════════
# Scenarios — edit here to add your own test layouts
# ═════════════════════════════════════════════════════════════════════

SCENARIOS: Dict[str, Dict] = {
    "square": {
        "name": "Square 2.6×2.6m (4 corners + HOME)",
        "room_w": 4.0, "room_h": 4.0,
        # robot starts at origin facing +x; markers in odom frame
        "markers": {
            0: (0.5, 0.0),     # HOME — 0.5 m ahead of start
            1: (-1.1, 1.3),    # CORNER 1 — top-left
            2: ( 1.3, 1.3),    # CORNER 2 — top-right
            3: ( 1.3, -1.3),   # CORNER 3 — bottom-right
            4: (-1.1, -1.3),   # CORNER 4 — bottom-left
        },
        "obstacles": [],
        "robot_start": (0.0, 0.0, 0.0),
    },
    "rectangle": {
        "name": "Square 3×3m (larger, same shape)",
        "room_w": 4.5, "room_h": 4.5,
        "markers": {
            0: (0.4, 0.0),
            1: (-1.5, 1.5),
            2: ( 1.5, 1.5),
            3: ( 1.5, -1.5),
            4: (-1.5, -1.5),
        },
        "obstacles": [],
        "robot_start": (0.0, 0.0, 0.0),
    },
    "triangle": {
        "name": "Triangle (3 corners only)",
        "room_w": 4.0, "room_h": 4.0,
        "markers": {
            0: (0.2, -0.2),
            1: (0.0, 1.5),
            2: (1.3, -0.8),
            3: (-1.3, -0.8),
        },
        "obstacles": [],
        "robot_start": (0.0, 0.0, math.pi / 2),  # facing +y
    },
}


# ═════════════════════════════════════════════════════════════════════
# Physics / sensor simulation
# ═════════════════════════════════════════════════════════════════════

SENSOR_ANGLES_DEG = {
    "front": 0.0, "front_left": 30.0, "front_right": -30.0,
    "left": 90.0, "right": -90.0, "back": 180.0,
}

# Match test_nav_safety.py
ROBOT_SPEED = 0.15
ROBOT_STRAFE = 0.12
ROBOT_ROT = 1.0

CMD_VELS = {
    "F":  (ROBOT_SPEED, 0, 0),
    "B":  (-ROBOT_SPEED, 0, 0),
    "L":  (0, 0, ROBOT_ROT),
    "R":  (0, 0, -ROBOT_ROT),
    "SL": (0, ROBOT_STRAFE, 0),
    "SR": (0, -ROBOT_STRAFE, 0),
    "DL": (ROBOT_SPEED * 0.7, ROBOT_STRAFE * 0.7, 0),
    "DR": (ROBOT_SPEED * 0.7, -ROBOT_STRAFE * 0.7, 0),
    "S":  (0, 0, 0),
    "":   (0, 0, 0),
}

WHEEL_CIRC = math.pi * 0.06
COUNTS_PER_M = (11 * 34 * 4) / WHEEL_CIRC

CAMERA_FOV_DEG = 54.0
MARKER_DETECT_RANGE = 3.0   # how far the simulated camera can see a marker
MARKER_MIN_RANGE = 0.15     # too close = out of focus

ROBOT_MARGIN_M = 0.12        # collision box half-size


def ray_wall(rx: float, ry: float, angle: float,
             room_w: float, room_h: float) -> float:
    dx, dy = math.cos(angle), math.sin(angle)
    hw, hh = room_w / 2, room_h / 2
    d = float("inf")
    for wd, dc in [(-hw - rx, dx), (hw - rx, dx), (-hh - ry, dy), (hh - ry, dy)]:
        if abs(dc) < 1e-9:
            continue
        t = wd / dc
        if t > 1e-4:
            hx = rx + t * dx
            hy = ry + t * dy
            if (-hw - 0.01 <= hx <= hw + 0.01 and
                -hh - 0.01 <= hy <= hh + 0.01):
                d = min(d, t)
    return d


def ray_rect(rx: float, ry: float, angle: float,
             rect: Tuple[float, float, float, float]) -> float:
    x0, y0, x1, y1 = rect
    dx, dy = math.cos(angle), math.sin(angle)
    tmin, tmax = 0.0, 1e9
    for r, dd, lo, hi in [(rx, dx, x0, x1), (ry, dy, y0, y1)]:
        if abs(dd) < 1e-9:
            if r < lo or r > hi:
                return 1e9
        else:
            t1 = (lo - r) / dd
            t2 = (hi - r) / dd
            if t1 > t2:
                t1, t2 = t2, t1
            tmin = max(tmin, t1)
            tmax = min(tmax, t2)
            if tmin > tmax:
                return 1e9
    return tmin if tmin > 1e-4 else tmax


def simulate_tof(rx: float, ry: float, theta: float,
                 room_w: float, room_h: float,
                 obstacles: List[Tuple[float, float, float, float]]) -> Dict[str, float]:
    readings = {}
    for name, ang_deg in SENSOR_ANGLES_DEG.items():
        a = theta + math.radians(ang_deg)
        d = ray_wall(rx, ry, a, room_w, room_h)
        for obs in obstacles:
            dr = ray_rect(rx, ry, a, obs)
            if dr < d:
                d = dr
        readings[name] = max(10.0, min(8000.0, d * 1000))
    return readings


def detect_markers(rx: float, ry: float, theta: float,
                   markers_truth: Dict[int, Tuple[float, float]]) -> List[Tuple[int, float, float]]:
    """Return [(id, rel_forward, rel_left), ...] for markers visible in camera FOV."""
    fov_half = math.radians(CAMERA_FOV_DEG / 2)
    result = []
    for mid, (mx, my) in markers_truth.items():
        dx, dy = mx - rx, my - ry
        dist = math.hypot(dx, dy)
        if dist > MARKER_DETECT_RANGE or dist < MARKER_MIN_RANGE:
            continue
        bearing_world = math.atan2(dy, dx)
        bearing_rel = bearing_world - theta
        while bearing_rel > math.pi:
            bearing_rel -= 2 * math.pi
        while bearing_rel < -math.pi:
            bearing_rel += 2 * math.pi
        if abs(bearing_rel) > fov_half:
            continue
        rel_fwd = dist * math.cos(bearing_rel)
        rel_left = dist * math.sin(bearing_rel)
        result.append((mid, rel_fwd, rel_left))
    return result


def sim_encoders(vx: float, vy: float, dt: float, prev: Dict[str, int]) -> Dict[str, int]:
    enc = dict(prev)
    for name, spd in [("FR", vx - vy), ("FL", vx + vy), ("BR", vx + vy), ("BL", vx - vy)]:
        enc[f"{name}_count"] = prev.get(f"{name}_count", 0) + int(spd * dt * COUNTS_PER_M)
        enc[f"{name}_rpm"] = spd / WHEEL_CIRC * 60
    return enc


# ═════════════════════════════════════════════════════════════════════
# Simulator state machine
# ═════════════════════════════════════════════════════════════════════

class KPatrolSim:
    """Drives the real navigation stack inside a simulated world."""

    DISCOVERY_ROTATE_CMD = "L"           # slow scan
    DISCOVERY_MIN_TICKS = 72             # full 360° rotation (ROBOT_ROT=1 rad/s, dt=0.1)
    DISCOVERY_MAX_TICKS = 220            # hard timeout (~3 full rotations)
    COVERAGE_TARGET_PCT = 45.0           # auto return home threshold
    COVERAGE_MAX_STEPS = 1500            # fallback: go home after this many coverage steps

    def __init__(self, scenario_name: str, auto_home: bool = True):
        if scenario_name not in SCENARIOS:
            raise KeyError(f"Unknown scenario: {scenario_name}")
        self.scenario_name = scenario_name
        self.auto_home = auto_home
        self.dt = 0.1
        self.reset()

    def reset(self):
        sc = SCENARIOS[self.scenario_name]
        self.room_w = sc["room_w"]
        self.room_h = sc["room_h"]
        self.markers_truth = dict(sc["markers"])
        self.obstacles = list(sc["obstacles"])

        # Fresh navigation stack so reset is clean
        self.odom = Odometer()
        self.grid = MiniGrid(
            width_m=max(self.room_w, self.room_h) + 1.0,
            height_m=max(self.room_w, self.room_h) + 1.0,
            cell_size_m=0.05,
        )
        self.vfh = VFH(safe_mm=500, caution_mm=350, stop_mm=250)
        self.nav = NavController(
            self.odom, self.grid, self.vfh,
            waypoints_path="/tmp/sim_viz_wp.json",
            grid_path="/tmp/sim_viz_grid.json",
        )

        self.rx, self.ry, self.rtheta = sc["robot_start"]
        self.odom.reset(imu_yaw_deg=math.degrees(self.rtheta))

        # Pre-mark known obstacles so the planner avoids them
        for obs in self.obstacles:
            x = obs[0]
            while x <= obs[2]:
                y = obs[1]
                while y <= obs[3]:
                    self.grid.mark_obstacle(x, y)
                    y += 0.05
                x += 0.05

        self.enc = {f"{m}_{t}": 0 for m in ["FR", "FL", "BR", "BL"] for t in ["count", "rpm"]}
        self.current_cmd = "S"
        self.trail: List[Tuple[float, float]] = [(self.rx, self.ry)]
        self.step_count = 0
        self.collision_count = 0
        self.phase = "discovery"
        self.phase_since_step = 0

        self.nav.set_mode("MANUAL")

    # ── Single simulation tick ─────────────────────────────────────

    def tick(self) -> Dict:
        # 1. Apply previous command to world
        vx, vy, w = CMD_VELS.get(self.current_cmd, (0, 0, 0))
        ct, st = math.cos(self.rtheta), math.sin(self.rtheta)
        nx = self.rx + (vx * ct - vy * st) * self.dt
        ny = self.ry + (vx * st + vy * ct) * self.dt
        ntheta = self.rtheta + w * self.dt

        # 2. Collision check against walls + obstacles
        collided = False
        hw, hh = self.room_w / 2, self.room_h / 2
        if (nx < -hw + ROBOT_MARGIN_M or nx > hw - ROBOT_MARGIN_M or
                ny < -hh + ROBOT_MARGIN_M or ny > hh - ROBOT_MARGIN_M):
            collided = True
        if not collided:
            for obs in self.obstacles:
                if (obs[0] - ROBOT_MARGIN_M <= nx <= obs[2] + ROBOT_MARGIN_M and
                        obs[1] - ROBOT_MARGIN_M <= ny <= obs[3] + ROBOT_MARGIN_M):
                    collided = True
                    break

        if not collided:
            self.rx, self.ry = nx, ny
        elif abs(vx) > 1e-3 or abs(vy) > 1e-3:
            self.collision_count += 1
        self.rtheta = ntheta

        self.trail.append((self.rx, self.ry))
        if len(self.trail) > 800:
            self.trail.pop(0)

        # 3. Generate sensor data
        tof = simulate_tof(self.rx, self.ry, self.rtheta,
                           self.room_w, self.room_h, self.obstacles)
        self.enc = sim_encoders(vx, vy, self.dt, self.enc)
        imu_deg = math.degrees(self.rtheta)

        # 4. Feed marker observations to the real MarkerBoundary
        visible = detect_markers(self.rx, self.ry, self.rtheta, self.markers_truth)
        for mid, rel_fwd, rel_left in visible:
            self.nav.observe_marker(mid, rel_fwd, rel_left)

        # 5. Auto phase transitions
        self._update_phase()

        # 6. Nav tick — produces next command
        raw_cmd, _status = self.nav.tick(tof, self.enc, imu_deg, dt=self.dt)

        if self.phase == "discovery":
            self.current_cmd = self.DISCOVERY_ROTATE_CMD
        else:
            self.current_cmd = raw_cmd or "S"

        self.step_count += 1
        return self.get_state()

    # ── Phase logic ────────────────────────────────────────────────

    def _enter_coverage(self):
        self.nav.set_mode("SMART_COVERAGE")
        self.phase = "coverage"
        self.phase_since_step = self.step_count

    def _update_phase(self):
        corners = self.nav.boundary.get_corner_positions()
        n_corners = len(corners)
        has_home = self.nav.boundary.has_home()
        coverage = self.grid.coverage_percent()

        if self.phase == "discovery":
            ticks_in_phase = self.step_count - self.phase_since_step
            all_markers_found = n_corners >= 4 and has_home

            # Ideal path: complete at least one full rotation AND see all markers
            if all_markers_found and ticks_in_phase >= self.DISCOVERY_MIN_TICKS:
                self._enter_coverage()
            # Fallback: after hard timeout, accept partial discovery (≥3 corners)
            elif ticks_in_phase >= self.DISCOVERY_MAX_TICKS and n_corners >= 3 and has_home:
                self._enter_coverage()

        elif self.phase == "coverage":
            steps_in_coverage = self.step_count - self.phase_since_step
            coverage_trigger = coverage >= self.COVERAGE_TARGET_PCT and has_home
            time_trigger = steps_in_coverage >= self.COVERAGE_MAX_STEPS and has_home
            if self.auto_home and (coverage_trigger or time_trigger):
                self.nav.set_mode("RETURN_HOME")
                self.phase = "return_home"
                self.phase_since_step = self.step_count
        elif self.phase == "return_home":
            # Check distance to home marker
            home = self.nav.boundary.get_home_position()
            if home is not None:
                pose = self.odom.get_pose()
                dist = math.hypot(pose.x - home[0], pose.y - home[1])
                if dist < 0.15:
                    self.nav.set_mode("MANUAL")
                    self.phase = "done"
                    self.phase_since_step = self.step_count

    # ── State snapshot for the visualizer ─────────────────────────

    def get_state(self) -> Dict:
        odom_pose = self.odom.get_pose()
        return {
            "step": self.step_count,
            "phase": self.phase,
            "rx": self.rx, "ry": self.ry, "rtheta": self.rtheta,
            "odom_x": odom_pose.x, "odom_y": odom_pose.y,
            "cmd": self.current_cmd,
            "mode": self.nav.get_mode(),
            "boundary_mode": self.nav.boundary.get_mode(),
            "corners_seen": len(self.nav.boundary.get_corner_positions()),
            "has_home": self.nav.boundary.has_home(),
            "coverage": self.grid.coverage_percent(),
            "collisions": self.collision_count,
            "trail": list(self.trail),
        }


# ═════════════════════════════════════════════════════════════════════
# Visualization
# ═════════════════════════════════════════════════════════════════════

class Visualizer:
    def __init__(self, sim: KPatrolSim, interval_ms: int = 50):
        self.sim = sim
        self.interval_ms = interval_ms
        self.paused = False
        self.fig, self.ax = plt.subplots(figsize=(10, 10))
        self.fig.canvas.manager.set_window_title("K-Patrol Marker Boundary Simulator")

        # Dynamic artists
        self.obstacle_patches: List = []
        self.marker_circles: List = []
        self.marker_labels: List = []
        self.polygon_artist = None
        self.disk_artist = None
        self.corridor_line = None
        self.robot_patch = None
        self.fov_patch = None
        self.heatmap_scatter = None
        self.text_box = None

        self.trail_line, = self.ax.plot([], [], "-",
                                        color="#3b82f6", linewidth=1.5, alpha=0.6)

        self._setup_static()
        self._bind_keys()

    # ── Static scene ──────────────────────────────────────────────

    def _setup_static(self):
        sc = SCENARIOS[self.sim.scenario_name]
        hw, hh = self.sim.room_w / 2, self.sim.room_h / 2

        self.ax.clear()
        self.ax.set_xlim(-hw - 0.4, hw + 0.4)
        self.ax.set_ylim(-hh - 0.4, hh + 0.4)
        self.ax.set_aspect("equal")
        self.ax.set_title(
            f"K-Patrol Simulator — {sc['name']}\n"
            f"[space] pause   [r] reset   [1-3] scenario   [q] quit",
            fontsize=11,
        )
        self.ax.grid(True, alpha=0.3)
        self.ax.set_xlabel("x (m)")
        self.ax.set_ylabel("y (m)")

        # Room walls
        self.ax.add_patch(plt.Rectangle(
            (-hw, -hh), self.sim.room_w, self.sim.room_h,
            fill=False, edgecolor="black", linewidth=2.5, zorder=1,
        ))

        # Obstacles
        for obs in self.sim.obstacles:
            x0, y0, x1, y1 = obs
            self.ax.add_patch(plt.Rectangle(
                (x0, y0), x1 - x0, y1 - y0,
                facecolor="#9ca3af", edgecolor="#4b5563", alpha=0.8, zorder=2,
            ))

        # Marker pillars (ground truth)
        for mid, (mx, my) in self.sim.markers_truth.items():
            face = "#dc2626" if mid == HOME_ID else "#16a34a"
            c = plt.Circle((mx, my), 0.10, facecolor=face,
                           edgecolor="black", linewidth=1.5, zorder=7)
            self.ax.add_patch(c)
            label = "H" if mid == HOME_ID else str(mid)
            self.ax.text(mx, my, label, ha="center", va="center",
                         color="white", fontsize=11, fontweight="bold", zorder=8)

        # Trail line
        self.trail_line, = self.ax.plot(
            [], [], "-", color="#3b82f6", linewidth=1.5, alpha=0.7, zorder=3,
        )

        # Status text
        self.text_box = self.ax.text(
            0.02, 0.98, "", transform=self.ax.transAxes,
            verticalalignment="top", fontsize=9, family="monospace",
            bbox=dict(boxstyle="round,pad=0.5", facecolor="white",
                      edgecolor="#9ca3af", alpha=0.9),
            zorder=20,
        )

        # Legend
        legend_elements = [
            mpatches.Patch(facecolor="#dc2626", edgecolor="k", label="HOME marker"),
            mpatches.Patch(facecolor="#16a34a", edgecolor="k", label="Corner markers"),
            mpatches.Patch(facecolor="red", label="Robot"),
            mpatches.Patch(facecolor="#fde047", alpha=0.3, label="Camera FOV"),
            mpatches.Patch(facecolor="none", edgecolor="#16a34a",
                           linestyle="--", linewidth=2, label="Boundary polygon"),
            mpatches.Patch(facecolor="#bbf7d0", alpha=0.4, label="Visited cells"),
        ]
        self.ax.legend(handles=legend_elements, loc="lower right", fontsize=8,
                       framealpha=0.9)

    def _bind_keys(self):
        def on_key(event):
            if event.key == " ":
                self.paused = not self.paused
            elif event.key == "r":
                self.sim.reset()
                self._setup_static()
            elif event.key == "q":
                plt.close(self.fig)
            elif event.key in ("1", "2", "3"):
                idx = int(event.key) - 1
                names = list(SCENARIOS.keys())
                if 0 <= idx < len(names):
                    self.sim.scenario_name = names[idx]
                    self.sim.reset()
                    self._setup_static()
        self.fig.canvas.mpl_connect("key_press_event", on_key)

    # ── Frame update ──────────────────────────────────────────────

    def update(self, frame):
        if not self.paused:
            self.sim.tick()
        state = self.sim.get_state()

        self._draw_robot_and_fov(state)
        self._draw_trail(state)
        self._draw_boundary()
        self._draw_heatmap()
        self._draw_status(state)
        return []

    def _draw_robot_and_fov(self, state):
        if self.robot_patch is not None:
            self.robot_patch.remove()
        if self.fov_patch is not None:
            self.fov_patch.remove()

        cx, cy = state["rx"], state["ry"]
        theta = state["rtheta"]

        # FOV wedge
        half = CAMERA_FOV_DEG / 2
        self.fov_patch = mpatches.Wedge(
            (cx, cy), MARKER_DETECT_RANGE,
            math.degrees(theta) - half,
            math.degrees(theta) + half,
            alpha=0.15, color="#fde047", zorder=2,
        )
        self.ax.add_patch(self.fov_patch)

        # Robot triangle
        size = 0.14
        pts = np.array([
            [size,        0.0],
            [-size * 0.5, size * 0.6],
            [-size * 0.5, -size * 0.6],
        ])
        rot = np.array([[math.cos(theta), -math.sin(theta)],
                        [math.sin(theta), math.cos(theta)]])
        pts_world = pts @ rot.T + np.array([cx, cy])
        self.robot_patch = plt.Polygon(
            pts_world, color="#dc2626", ec="black", lw=1.5, zorder=10,
        )
        self.ax.add_patch(self.robot_patch)

    def _draw_trail(self, state):
        if len(state["trail"]) > 1:
            xs = [p[0] for p in state["trail"]]
            ys = [p[1] for p in state["trail"]]
            self.trail_line.set_data(xs, ys)

    def _draw_boundary(self):
        # Remove previous
        for artist in (self.polygon_artist, self.disk_artist, self.corridor_line):
            if artist is not None:
                try:
                    artist.remove()
                except Exception:
                    pass
        self.polygon_artist = None
        self.disk_artist = None
        self.corridor_line = None

        corners = self.sim.nav.boundary.get_corner_positions()
        if len(corners) >= 3:
            poly_pts = [(x, y) for _, x, y in corners]
            # close polygon by repeating first
            poly_pts.append(poly_pts[0])
            xs = [p[0] for p in poly_pts]
            ys = [p[1] for p in poly_pts]
            self.polygon_artist, = self.ax.plot(
                xs, ys, "--", color="#16a34a", linewidth=2.5, zorder=4,
            )
        elif len(corners) == 2:
            (_, x1, y1), (_, x2, y2) = corners
            self.corridor_line, = self.ax.plot(
                [x1, x2], [y1, y2], "--", color="#16a34a", linewidth=2.5, zorder=4,
            )
        elif len(corners) == 1:
            _, mx, my = corners[0]
            self.disk_artist = plt.Circle(
                (mx, my), 1.5, fill=False, edgecolor="#16a34a",
                linewidth=2, linestyle="--", zorder=4,
            )
            self.ax.add_patch(self.disk_artist)

    def _draw_heatmap(self):
        if self.heatmap_scatter is not None:
            try:
                self.heatmap_scatter.remove()
            except Exception:
                pass
            self.heatmap_scatter = None

        # Sample FREE cells from the grid — every ~N steps to avoid clutter
        if self.sim.step_count % 5 != 0:
            return
        xs, ys = [], []
        g = self.sim.grid
        for r in range(0, g.rows, 2):
            for c in range(0, g.cols, 2):
                if g._grid[r][c] == FREE:
                    x, y = g._to_world(c, r)
                    xs.append(x)
                    ys.append(y)
        if xs:
            self.heatmap_scatter = self.ax.scatter(
                xs, ys, s=8, c="#bbf7d0", alpha=0.4, zorder=1,
            )

    def _draw_status(self, state):
        home_mark = "OK" if state["has_home"] else "--"
        phase_display = {
            "discovery":  "1. DISCOVERY (scanning)",
            "coverage":   "2. SMART_COVERAGE",
            "return_home": "3. RETURN HOME",
            "done":       "4. DONE (back home)",
        }.get(state["phase"], state["phase"])

        txt = (
            f"Step:      {state['step']}\n"
            f"Phase:     {phase_display}\n"
            f"Nav mode:  {state['mode']}\n"
            f"Cmd:       {state['cmd']}\n"
            f"Pose:      ({state['rx']:+.2f}, {state['ry']:+.2f})  yaw={math.degrees(state['rtheta']):+.0f}°\n"
            f"Boundary:  {state['boundary_mode']}\n"
            f"Corners:   {state['corners_seen']}/4\n"
            f"HOME:      {home_mark}\n"
            f"Coverage:  {state['coverage']:.1f}%\n"
            f"Collision: {state['collisions']}"
        )
        self.text_box.set_text(txt)

    # ── Run ───────────────────────────────────────────────────────

    def run(self):
        _ani = FuncAnimation(
            self.fig, self.update, interval=self.interval_ms,
            blit=False, cache_frame_data=False,
        )
        plt.show()


# ═════════════════════════════════════════════════════════════════════
# CLI
# ═════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="K-Patrol marker boundary simulator")
    parser.add_argument("--scenario", choices=list(SCENARIOS.keys()),
                        default="square", help="which scenario to run")
    parser.add_argument("--speed", type=int, default=50,
                        help="animation interval in ms (lower = faster, default 50)")
    parser.add_argument("--no-autohome", action="store_true",
                        help="disable auto RETURN_HOME at 60%% coverage")
    args = parser.parse_args()

    sim = KPatrolSim(args.scenario, auto_home=not args.no_autohome)
    viz = Visualizer(sim, interval_ms=args.speed)
    viz.run()


if __name__ == "__main__":
    main()
