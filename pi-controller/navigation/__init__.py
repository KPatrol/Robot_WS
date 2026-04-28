"""
K-Patrol Navigation Package
============================
Scripted autonomous patrol for the K-Patrol Mecanum robot.

Modules:
    script_patrol  — declarative script executor (IMU-closed-loop rotation +
                     timed translation + ToF emergency reflex)
    nav_controller — thin controller wrapping ScriptExecutor with
                     MANUAL / SCRIPT_PATROL / EMERGENCY modes

Quick Start:
    from navigation import NavController

    nav = NavController(script_dir="data/scripts")
    nav.script_save({"name": "rectangle", ...})
    nav.script_start("rectangle")

    # In main loop (~20 Hz):
    cmd, speed_pwm, status = nav.tick(tof_dict, imu_yaw_deg)
"""

from .nav_controller import NavController, Mode
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
