"""
Alert → Actuator bridge.

Translates DetectionEvent (person, fire) and tip-over / battery events into
firmware commands for the warning light + buzzer. The detector already does
N-of-M temporal smoothing, but the actuator owns a separate per-kind cooldown
because:

  * The FSM cooldown (NavController._ALERT_COOLDOWNS_S) gates *navigation*
    reactions (pause / e-stop). A repeat fire alert mid-cooldown is dropped
    there, but we still want the strobe + alarm to *stay on* as long as the
    operator hasn't cleared the EMERGENCY.
  * Without this layer, every detection callback would re-issue the same
    LP:WARN_STROBE / BUZZ:ALARM bytes on the serial link and crowd out
    motor commands. The actuator coalesces "same pattern still active"
    into a no-op.

Design
------
* Pure callable in / callable out — no MQTT or motor-controller imports.
  The host wires `send_cmd=motor_controller.send_command` at construction.
* Holds last-issued pattern per channel (light, buzzer) so repeat events
  while the same pattern is already running cost nothing.
* `clear()` is the explicit "operator cleared EMERGENCY" hook — only it
  resets light/buzzer to OFF.
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Dict, Optional, Tuple

log = logging.getLogger("kpatrol.safety.actuator")


@dataclass
class ActuatorConfig:
    """Per-kind mapping of event → (light pattern, buzzer pattern).

    Patterns must match firmware vocabulary (see controller_four_motors_mecanum.ino):
      Light:  OFF | WARN_BLINK | WARN_STROBE | BOTH_BLINK | SOS
      Buzzer: OFF | ON | BEEP | ALARM | SOS
    """

    person_light: str = "WARN_BLINK"
    person_buzzer: str = "BEEP"
    fire_light: str = "WARN_STROBE"
    fire_buzzer: str = "ALARM"
    tipover_light: str = "SOS"
    tipover_buzzer: str = "SOS"
    battery_low_light: str = "WARN_BLINK"
    battery_low_buzzer: str = "BEEP"
    battery_critical_light: str = "WARN_STROBE"
    battery_critical_buzzer: str = "ALARM"

    # How long a one-shot buzzer pattern (BEEP) is allowed to remain "active"
    # in our local state before the next event re-arms it. Must roughly match
    # the firmware BEEP duration (~250ms) plus margin.
    beep_repeat_min_sec: float = 1.0


# Continuous patterns latch on the firmware until OFF; one-shots auto-fade.
_CONTINUOUS_LIGHT = {"WARN_BLINK", "WARN_STROBE", "BOTH_BLINK", "SOS"}
_CONTINUOUS_BUZZER = {"ON", "ALARM", "SOS"}


class AlertActuator:
    """Bridges high-level safety events to firmware light/buzzer commands.

    The constructor takes a `send_cmd(line: str) -> bool` callable — typically
    `motor_controller.send_command` — so this module never imports the serial
    layer or paho. Tests pass a lambda that records the call.
    """

    def __init__(
        self,
        send_cmd: Callable[[str], bool],
        config: Optional[ActuatorConfig] = None,
    ):
        self.send_cmd = send_cmd
        self.config = config or ActuatorConfig()
        self._lock = threading.Lock()
        # Last pattern actually written to the firmware (None == unknown / OFF).
        self._light: Optional[str] = None
        self._buzzer: Optional[str] = None
        # Wall clock of the last write per channel; used to debounce one-shot
        # BEEP patterns so a 10-Hz event stream doesn't flood the serial link.
        self._light_ts: float = 0.0
        self._buzzer_ts: float = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def on_detection(self, kind: str, confidence: float = 0.0) -> Dict[str, str]:
        """Wire a DetectionEvent kind to actuator output.

        Returns the (light, buzzer) pair we ended up holding so the caller
        can log / publish telemetry without re-deriving it.
        """
        kind = (kind or "").lower().strip()
        if kind == "person":
            return self._apply(self.config.person_light, self.config.person_buzzer,
                               source=f"person@{confidence:.2f}")
        if kind == "fire":
            return self._apply(self.config.fire_light, self.config.fire_buzzer,
                               source=f"fire@{confidence:.2f}")
        log.debug("[actuator] ignoring unknown detection kind=%s", kind)
        return {"light": self._light or "OFF", "buzzer": self._buzzer or "OFF"}

    def on_tipover(self, axis: str, angle_deg: float) -> Dict[str, str]:
        """Tip-over → SOS pattern on both channels."""
        return self._apply(
            self.config.tipover_light, self.config.tipover_buzzer,
            source=f"tipover {axis}={angle_deg:.1f}°",
        )

    def on_battery(self, level: str, pct: float) -> Dict[str, str]:
        """Battery low / critical → escalating warning."""
        level = (level or "").lower().strip()
        if level == "critical":
            return self._apply(
                self.config.battery_critical_light,
                self.config.battery_critical_buzzer,
                source=f"battery_critical {pct:.0f}%",
            )
        if level == "low":
            return self._apply(
                self.config.battery_low_light,
                self.config.battery_low_buzzer,
                source=f"battery_low {pct:.0f}%",
            )
        return {"light": self._light or "OFF", "buzzer": self._buzzer or "OFF"}

    def clear(self) -> None:
        """Operator cleared EMERGENCY — turn everything off.

        Called from kpatrol_mqtt_v5._on_emergency_clear() (and equivalents).
        Idempotent: a clear when already clear is a no-op.
        """
        with self._lock:
            need_light = self._light not in (None, "OFF")
            need_buzz = self._buzzer not in (None, "OFF")
        if need_light:
            self._write_light("OFF")
        if need_buzz:
            self._write_buzzer("OFF")

    def state(self) -> Dict[str, str]:
        with self._lock:
            return {"light": self._light or "OFF", "buzzer": self._buzzer or "OFF"}

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _apply(self, light: str, buzzer: str, source: str) -> Tuple[Dict[str, str], None] | Dict[str, str]:
        """Send light + buzzer if they actually differ from current state.

        For one-shot buzzer patterns (BEEP) we re-arm if the cooldown has
        elapsed; otherwise we coalesce to no-op so flickery detectors don't
        flood the UART.
        """
        now = time.monotonic()
        with self._lock:
            cur_light, cur_buzz = self._light, self._buzzer
            light_ts, buzz_ts = self._light_ts, self._buzzer_ts

        # Light: continuous patterns latch — only re-issue on change.
        if light != cur_light:
            self._write_light(light)
        elif light not in _CONTINUOUS_LIGHT and now - light_ts > self.config.beep_repeat_min_sec:
            # One-shot light pattern that may have already faded — re-issue.
            self._write_light(light)

        # Buzzer: same logic, separate state.
        if buzzer != cur_buzz:
            self._write_buzzer(buzzer)
        elif buzzer not in _CONTINUOUS_BUZZER and now - buzz_ts > self.config.beep_repeat_min_sec:
            self._write_buzzer(buzzer)

        log.info("[actuator] %s → light=%s buzzer=%s", source, light, buzzer)
        return {"light": light, "buzzer": buzzer}

    def _write_light(self, pattern: str) -> None:
        try:
            ok = bool(self.send_cmd(f"LP:{pattern}"))
        except Exception as exc:
            log.warning("[actuator] light send raised: %s", exc)
            ok = False
        if ok:
            with self._lock:
                self._light = pattern
                self._light_ts = time.monotonic()

    def _write_buzzer(self, pattern: str) -> None:
        try:
            ok = bool(self.send_cmd(f"BUZZ:{pattern}"))
        except Exception as exc:
            log.warning("[actuator] buzzer send raised: %s", exc)
            ok = False
        if ok:
            with self._lock:
                self._buzzer = pattern
                self._buzzer_ts = time.monotonic()
