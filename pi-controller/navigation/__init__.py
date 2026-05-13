"""
K-Patrol Navigation Package
============================
Autonomous navigation for the K-Patrol Mecanum robot.

Modes (NavController FSM, 5 total):
    MANUAL              — operator pass-through
    AUTO_FREE_COVERAGE  — indoor random-walk + frontier exploration
    AUTO_LINE_FOLLOW    — camera floor-line PD follower
    AUTO_GPS_WAYPOINT   — outdoor GPS route (Haversine + IMU + ToF safety)
    EMERGENCY           — motors stopped, waiting for operator clear

Quick Start:
    from navigation import NavController

    nav = NavController()
    nav.auto_free_coverage_start()
    nav.auto_line_follow_start()
    nav.auto_gps_waypoint_start([{"lat": 21.028, "lon": 105.804, "radius_m": 3}])

    # In main loop (~20 Hz):
    cmd, speed_pwm, twist, status = nav.tick(
        tof_dict, imu_yaw_deg,
        pose=pose,
        gps_data=gps_reader.get_data(),
    )
"""

from .nav_controller import NavController, Mode
from .line_follower import LineFollower, LineFollowerConfig, LineResult, HSVRange
from .free_coverage import FreeCoverage, CoverageConfig
from .gps_navigator import (
    GPSNavigator,
    GPSNavigatorConfig,
    GPSNavResult,
    GPSState,
    Waypoint,
    haversine_m,
    initial_bearing_deg,
    shortest_angle_deg,
)
from .occupancy_grid import OccupancyGrid, GridConfig, CELL_VISITED, CELL_OBSTACLE
from .odometry import Odometry, Pose
from .velocity_pid import VelocityController, VelocityPIDConfig
from .camera_source import (
    CameraSource,
    Picamera2Source,
    OpenCVSource,
    ThreadedFrameGrabber,
    open_camera,
)

__all__ = [
    "NavController",
    "Mode",
    "LineFollower",
    "LineFollowerConfig",
    "LineResult",
    "HSVRange",
    "FreeCoverage",
    "CoverageConfig",
    "GPSNavigator",
    "GPSNavigatorConfig",
    "GPSNavResult",
    "GPSState",
    "Waypoint",
    "haversine_m",
    "initial_bearing_deg",
    "shortest_angle_deg",
    "OccupancyGrid",
    "GridConfig",
    "CELL_VISITED",
    "CELL_OBSTACLE",
    "Odometry",
    "Pose",
    "VelocityController",
    "VelocityPIDConfig",
    "CameraSource",
    "Picamera2Source",
    "OpenCVSource",
    "ThreadedFrameGrabber",
    "open_camera",
]
