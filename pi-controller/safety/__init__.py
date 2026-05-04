"""
K-Patrol Safety Package
=======================
Cross-cutting safety helpers that bridge detection / IMU / battery events to
firmware actuators (warning light + buzzer) and to the navigation FSM.

Modules
-------
    alert_actuator   Maps DetectionEvent (person/fire) → MQTT light/buzzer
                     commands. Owns its own per-kind cooldown so a flickering
                     detector cannot drive the buzzer continuously.

    tipover_watcher  Polls IMU roll/pitch and triggers EMERGENCY when the
                     robot has fallen past a configurable threshold.

    battery_watcher  Consumes telemetry battery_pct and emits MQTT alerts
                     (low/critical) plus a BUZZ:BEEP at the critical step.
"""

from .alert_actuator import AlertActuator, ActuatorConfig
from .tipover_watcher import TipOverWatcher, TipOverConfig
from .battery_watcher import BatteryWatcher, BatteryConfig

__all__ = [
    "AlertActuator",
    "ActuatorConfig",
    "TipOverWatcher",
    "TipOverConfig",
    "BatteryWatcher",
    "BatteryConfig",
]
