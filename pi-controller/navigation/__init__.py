"""
K-Patrol Navigation Package
============================
Autonomous navigation for the K-Patrol Mecanum robot.

Modules:
    script_patrol   — declarative script executor (IMU-closed-loop rotation +
                      timed translation + ToF emergency reflex)
    nav_controller  — controller wrapping ScriptExecutor + LineFollower with
                      MANUAL / SCRIPT_PATROL / LINE_FOLLOW / EMERGENCY modes
    line_follower   — camera floor-line PD follower with BEV overlay
    script_recorder — record manual drives as patrol scripts

Quick Start:
    from navigation import NavController

    nav = NavController(script_dir="data/scripts")
    nav.script_start("rectangle")           # scripted patrol
    nav.line_follow_start()                 # camera line following

    # In main loop (~20 Hz):
    cmd, speed_pwm, twist, status = nav.tick(tof_dict, imu_yaw_deg)
"""

from .nav_controller import NavController, Mode, LineFollowerConfig
from .line_follower import LineFollower, LineResult, HSVRange
from .free_coverage import FreeCoverage, CoverageConfig
from .occupancy_grid import OccupancyGrid, GridConfig, CELL_VISITED, CELL_OBSTACLE
from .odometry import Odometry, Pose
from .velocity_pid import VelocityController, VelocityPIDConfig
from .script_patrol import (
    ExecutorState,
    PatrolScript,
    ScriptConfig,
    ScriptExecutor,
    ScriptLibrary,
    ScriptStep,
    example_rectangle,
    validate_script,
    validate_step,
)
from .script_recorder import ScriptRecorder

__all__ = [
    "NavController",
    "Mode",
    "LineFollowerConfig",
    "LineFollower",
    "LineResult",
    "HSVRange",
    "FreeCoverage",
    "CoverageConfig",
    "OccupancyGrid",
    "GridConfig",
    "CELL_VISITED",
    "CELL_OBSTACLE",
    "Odometry",
    "Pose",
    "VelocityController",
    "VelocityPIDConfig",
    "ScriptExecutor",
    "ScriptLibrary",
    "ScriptConfig",
    "ScriptStep",
    "PatrolScript",
    "ExecutorState",
    "ScriptRecorder",
    "example_rectangle",
    "validate_script",
    "validate_step",
]
