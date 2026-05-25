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

    alarm_controller Operator-configurable rule engine. Each rule maps an
                     event-type (person / fire / battery_low / tipover /
                     schedule / system_error / any_safety) onto a light +
                     buzzer pattern when the event has been continuously
                     observed for `continuous_duration_s` inside an optional
                     time-of-day window. Rules are pushed via MQTT and the
                     fire events are reported back on `/alarm/triggered`.
"""

from .alert_actuator import AlertActuator, ActuatorConfig
from .tipover_watcher import TipOverWatcher, TipOverConfig
from .battery_watcher import BatteryWatcher, BatteryConfig
from .alarm_controller import (
    AlarmController,
    AlarmRule,
    TimeWindow,
    EVENT_PERSON,
    EVENT_FIRE,
    EVENT_BATTERY_LOW,
    EVENT_BATTERY_CRITICAL,
    EVENT_TIPOVER,
    EVENT_SCHEDULE,
    EVENT_SYSTEM_ERROR,
    EVENT_ANY_SAFETY,
)

__all__ = [
    "AlertActuator",
    "ActuatorConfig",
    "TipOverWatcher",
    "TipOverConfig",
    "BatteryWatcher",
    "BatteryConfig",
    "AlarmController",
    "AlarmRule",
    "TimeWindow",
    "EVENT_PERSON",
    "EVENT_FIRE",
    "EVENT_BATTERY_LOW",
    "EVENT_BATTERY_CRITICAL",
    "EVENT_TIPOVER",
    "EVENT_SCHEDULE",
    "EVENT_SYSTEM_ERROR",
    "EVENT_ANY_SAFETY",
]
