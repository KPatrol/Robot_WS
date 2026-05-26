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
import queue
import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, Optional, Tuple

log = logging.getLogger("kpatrol.safety.actuator")


@dataclass
class ActuatorConfig:
    """Per-kind mapping of event → (light pattern, buzzer pattern).

    Patterns must match firmware vocabulary (see controller_four_motors_mecanum.ino):
      Light:  OFF | WARN_BLINK | WARN_STROBE | BOTH_BLINK | SOS
      Buzzer: OFF | ON | BEEP | ALARM | SOS

    HARDWARE NOTE (2026-05-25): the warning light on this build is a 12VDC
    mechanical relay (GPIO 38 → RELAY_WARNING_LIGHT_PIN). Every pattern
    other than OFF makes the firmware toggle the relay at 1–5 Hz, which
    wears the relay contacts out — typical electromechanical relays rate
    ~100k–1M operations. Sustained blinking from a chronic alert (battery
    low repeats every poll) destroys the relay within hours.

    Two mitigations:
      1. battery_low/critical default to LIGHT=OFF (buzzer only). Battery
         status is already shown in the cockpit UI; the operator doesn't
         need a relay-driven physical lamp to know the pack is at 20%.
      2. Detection patterns (person/fire) keep WARN_BLINK but the actuator
         auto-clears them via `auto_clear_seconds` so the relay clicks for
         a brief alert burst (3 s default) instead of latching for hours.
    Long-term fix: add a LIGHTP_SOLID_ON pattern to the firmware that
    latches the relay ON without blinking, then point all light patterns
    here at it. Skipped for now to avoid a firmware reflash before demo.
    """

    person_light: str = "WARN_BLINK"
    person_buzzer: str = "BEEP"
    fire_light: str = "WARN_STROBE"
    fire_buzzer: str = "ALARM"
    tipover_light: str = "SOS"
    tipover_buzzer: str = "SOS"
    # V5.15c10: don't run the relay-driven warning light for battery alerts.
    # Was WARN_BLINK / WARN_STROBE; relay was clicking at 1–5 Hz for hours
    # whenever pack hovered near the low threshold.
    battery_low_light: str = "OFF"
    battery_low_buzzer: str = "BEEP"
    battery_critical_light: str = "OFF"
    battery_critical_buzzer: str = "ALARM"

    # How long a one-shot buzzer pattern (BEEP) is allowed to remain "active"
    # in our local state before the next event re-arms it. Must roughly match
    # the firmware BEEP duration (~250ms) plus margin.
    beep_repeat_min_sec: float = 1.0

    # V5.15c10: auto-clear a continuous light pattern after this many seconds.
    # Sends LP:OFF unprompted to make the firmware leave blink mode → relay
    # stops clicking. Zero/negative disables the timer. 4 s gives ~4 visible
    # blinks for an alert, then quiet — visible alert without wearing relay.
    auto_clear_light_after_sec: float = 4.0
    # Buzzer auto-clear is shorter — ALARM/SOS get distracting fast.
    auto_clear_buzzer_after_sec: float = 6.0


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
        periph_relay: Optional[Callable[[bool], Any]] = None,
    ):
        """
        send_cmd: UART writer for the S3 motor controller (LP:/BUZZ: lines).
            Drives the in-cluster status light pattern + the placeholder
            BUZZER_PIN echo loop.
        periph_relay: optional callable that toggles the D1 R32 peripheral
            hub relay (GPIO26 — drives the 12V automotive lamp + horn in
            parallel through a 5 A fuse). Passing in `periph_hub.relay_on/
            relay_off` here lets a tip-over/fire alert actually make
            audible noise + flash the main warning lamp; without it the
            actuator only animates the on-board status LED via the S3
            motor and the operator never hears anything.
        """
        self.send_cmd = send_cmd
        self.periph_relay = periph_relay
        self.config = config or ActuatorConfig()
        self._lock = threading.Lock()
        # Last pattern actually written to the firmware (None == unknown / OFF).
        self._light: Optional[str] = None
        self._buzzer: Optional[str] = None
        # Wall clock of the last write per channel; used to debounce one-shot
        # BEEP patterns so a 10-Hz event stream doesn't flood the serial link.
        self._light_ts: float = 0.0
        self._buzzer_ts: float = 0.0

        # Dispatch UART writes via a background worker so safety callbacks
        # (detection / tipover / battery) never block the main loop on a
        # slow serial port. Bounded queue drops oldest on overflow rather
        # than back-pressuring the safety thread.
        self._tx_queue: "queue.Queue[Optional[str]]" = queue.Queue(maxsize=32)
        self._tx_stop = threading.Event()
        self._tx_thread = threading.Thread(
            target=self._tx_loop, name="alert_actuator_tx", daemon=True,
        )
        self._tx_thread.start()

        # V5.15c10: auto-clear continuous patterns to spare the relay.
        # When a continuous pattern (WARN_BLINK/STROBE/SOS) is applied,
        # a timer is armed; on expiry we send LP:OFF/BUZZ:OFF so the
        # firmware exits blink mode → relay stops clicking. Re-applying
        # the same pattern within the window re-arms (operator sees
        # ongoing alert), but a single trigger plus N seconds of silence
        # ends with the relay quiet.
        self._light_clear_timer: Optional[threading.Timer] = None
        self._buzzer_clear_timer: Optional[threading.Timer] = None

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
            # V5.15c11: person/fire/tipover alerts latch until cleared so
            # the operator actually notices them on the floor. Battery
            # alerts keep the auto-clear path for relay protection.
            # Person alerts: skip the peripheral-hub relay — the lamp + horn
            # would be too loud for routine surveillance.
            return self._apply(self.config.person_light, self.config.person_buzzer,
                               source=f"person@{confidence:.2f}", auto_clear=False)
        if kind == "fire":
            # Fire is a genuine emergency — pull the lamp + horn through
            # the D1 R32 relay so the alert is audible across the room.
            self._fire_periph_relay(True)
            return self._apply(self.config.fire_light, self.config.fire_buzzer,
                               source=f"fire@{confidence:.2f}", auto_clear=False)
        log.debug("[actuator] ignoring unknown detection kind=%s", kind)
        return {"light": self._light or "OFF", "buzzer": self._buzzer or "OFF"}

    def on_tipover(self, axis: str, angle_deg: float) -> Dict[str, str]:
        """Tip-over → SOS pattern on both channels (latches until cleared).

        V5.15c11 (2026-05-26): also fires the D1 R32 peripheral-hub relay
        (PIN_RELAY = GPIO26 in `peripheral_hub_d1r32.ino`) which drives
        the actual 12 V automotive lamp + horn in parallel. Without
        this, the S3 motor's LP:SOS/BUZZ:SOS only animate the on-board
        status LED — the operator never hears anything because the
        BUZZER_PIN on the S3 board is a placeholder (no piezo wired).
        """
        self._fire_periph_relay(True)
        return self._apply(
            self.config.tipover_light, self.config.tipover_buzzer,
            source=f"tipover {axis}={angle_deg:.1f}°",
            auto_clear=False,
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
        pending: list[str] = []
        with self._lock:
            now = time.monotonic()
            # Cancel any pending auto-clear timers — we're clearing now.
            if self._light_clear_timer is not None:
                self._light_clear_timer.cancel()
                self._light_clear_timer = None
            if self._buzzer_clear_timer is not None:
                self._buzzer_clear_timer.cancel()
                self._buzzer_clear_timer = None
            if self._light not in (None, "OFF"):
                self._light = "OFF"
                self._light_ts = now
                pending.append("LP:OFF")
            if self._buzzer not in (None, "OFF"):
                self._buzzer = "OFF"
                self._buzzer_ts = now
                pending.append("BUZZ:OFF")
        for line in pending:
            self._enqueue(line)
        # Release the peripheral-hub relay (12 V lamp + horn). Safe to
        # call when no relay was ever fired — `_fire_periph_relay` itself
        # short-circuits when self.periph_relay is None.
        self._fire_periph_relay(False)

    def state(self) -> Dict[str, str]:
        with self._lock:
            return {"light": self._light or "OFF", "buzzer": self._buzzer or "OFF"}

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    def _fire_periph_relay(self, on: bool) -> None:
        """Drive the D1 R32 peripheral-hub relay (12V lamp + horn).

        Wired in parallel through the hub's PIN_RELAY = GPIO26. A single
        digitalWrite both flashes the warning lamp and sounds the horn,
        which is exactly what we want for an emergency alert — they share
        the same fuse and the operator only has one circuit to maintain.

        Safe-no-op when no callable was injected (`periph_relay=None`),
        so the existing pure-UART tests keep passing.
        """
        if self.periph_relay is None:
            return
        try:
            self.periph_relay(bool(on))
        except Exception as exc:  # pragma: no cover - log only
            log.warning("[actuator] periph_relay(%s) raised: %s", on, exc)

    def _apply(self, light: str, buzzer: str, source: str, *, auto_clear: bool = True) -> Dict[str, str]:
        """Send light + buzzer if they actually differ from current state.

        For one-shot buzzer patterns (BEEP) we re-arm if the cooldown has
        elapsed; otherwise we coalesce to no-op so flickery detectors don't
        flood the UART. The full decision-and-update path runs under a
        single lock so two concurrent callers cannot both observe a stale
        snapshot and double-emit the same pattern.

        `auto_clear` (V5.15c11 — 2026-05-26): when False, suppress the
        relay-protection auto-clear timer. Safety alerts that the operator
        needs to hear/see (tip-over, fire, person under SOS) latch until
        explicitly cleared; only the noisy/repeating battery alerts opt
        into the original 4-6 s auto-clear behaviour.
        """
        pending: list[str] = []
        # Continuous patterns that just got applied — schedule auto-clear
        # for these after we release the lock.
        arm_light_clear = False
        arm_buzzer_clear = False
        with self._lock:
            now = time.monotonic()

            # Light: continuous patterns latch — re-issue only on change or
            # when a one-shot pattern has aged past the cooldown.
            if light != self._light or (
                light not in _CONTINUOUS_LIGHT
                and now - self._light_ts > self.config.beep_repeat_min_sec
            ):
                self._light = light
                self._light_ts = now
                pending.append(f"LP:{light}")
            # Re-arm timer even when we coalesced (operator's alert is still
            # active) so a stable continuous pattern still expires after the
            # window. Skip when pattern is OFF — nothing to clear.
            if (auto_clear
                    and light in _CONTINUOUS_LIGHT
                    and self.config.auto_clear_light_after_sec > 0):
                arm_light_clear = True

            # Buzzer: same logic, separate state.
            if buzzer != self._buzzer or (
                buzzer not in _CONTINUOUS_BUZZER
                and now - self._buzzer_ts > self.config.beep_repeat_min_sec
            ):
                self._buzzer = buzzer
                self._buzzer_ts = now
                pending.append(f"BUZZ:{buzzer}")
            if (auto_clear
                    and buzzer in _CONTINUOUS_BUZZER
                    and self.config.auto_clear_buzzer_after_sec > 0):
                arm_buzzer_clear = True

        # Enqueue OUTSIDE the lock — the bounded queue's put_nowait is fast
        # but we don't want to hold the state lock across any I/O primitive.
        for line in pending:
            self._enqueue(line)

        # Arm auto-clear timers AFTER we release the lock + enqueue. The
        # timer thread will re-acquire the lock when it fires.
        if arm_light_clear:
            self._arm_light_clear()
        if arm_buzzer_clear:
            self._arm_buzzer_clear()

        log.info("[actuator] %s → light=%s buzzer=%s", source, light, buzzer)
        return {"light": light, "buzzer": buzzer}

    def _arm_light_clear(self) -> None:
        """(Re)schedule LP:OFF after auto_clear_light_after_sec seconds.

        Each call cancels any previously-armed timer so the firmware
        exit-blink lands at the right moment relative to the LAST trigger,
        not the first one in a burst.
        """
        delay = float(self.config.auto_clear_light_after_sec)
        with self._lock:
            if self._light_clear_timer is not None:
                self._light_clear_timer.cancel()
            t = threading.Timer(delay, self._auto_clear_light)
            t.name = "alert_actuator_light_clear"
            t.daemon = True
            self._light_clear_timer = t
        t.start()

    def _arm_buzzer_clear(self) -> None:
        delay = float(self.config.auto_clear_buzzer_after_sec)
        with self._lock:
            if self._buzzer_clear_timer is not None:
                self._buzzer_clear_timer.cancel()
            t = threading.Timer(delay, self._auto_clear_buzzer)
            t.name = "alert_actuator_buzzer_clear"
            t.daemon = True
            self._buzzer_clear_timer = t
        t.start()

    def _auto_clear_light(self) -> None:
        emit = False
        with self._lock:
            if self._light not in (None, "OFF"):
                self._light = "OFF"
                self._light_ts = time.monotonic()
                emit = True
            self._light_clear_timer = None
        if emit:
            self._enqueue("LP:OFF")
            log.info("[actuator] auto-clear light → LP:OFF (relay rest)")

    def _auto_clear_buzzer(self) -> None:
        emit = False
        with self._lock:
            if self._buzzer not in (None, "OFF"):
                self._buzzer = "OFF"
                self._buzzer_ts = time.monotonic()
                emit = True
            self._buzzer_clear_timer = None
        if emit:
            self._enqueue("BUZZ:OFF")
            log.info("[actuator] auto-clear buzzer → BUZZ:OFF")

    def _enqueue(self, line: str) -> None:
        try:
            self._tx_queue.put_nowait(line)
        except queue.Full:
            # Drop the oldest queued line to keep the most recent intent.
            # Safety patterns are idempotent — losing a stale frame is fine.
            try:
                _ = self._tx_queue.get_nowait()
                self._tx_queue.task_done()
            except queue.Empty:
                pass
            try:
                self._tx_queue.put_nowait(line)
            except queue.Full:
                log.warning("[actuator] tx queue still full after drop; dropping %s", line)

    def _tx_loop(self) -> None:
        while not self._tx_stop.is_set():
            try:
                line = self._tx_queue.get(timeout=0.5)
            except queue.Empty:
                continue
            if line is None:  # sentinel
                self._tx_queue.task_done()
                break
            try:
                self.send_cmd(line)
            except Exception as exc:
                log.warning("[actuator] send_cmd raised for %s: %s", line, exc)
            finally:
                self._tx_queue.task_done()

    def _drain_for_tests(self, timeout: float = 1.0) -> None:
        """Block until the TX queue has been fully drained. Test-only.

        Production code must never call this — the whole point of the worker
        thread is to keep safety callbacks off the UART critical path.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if self._tx_queue.unfinished_tasks == 0:
                return
            time.sleep(0.005)

    def shutdown(self, timeout: float = 1.0) -> None:
        """Stop the background TX worker. Safe to call multiple times."""
        if self._tx_stop.is_set():
            return
        self._tx_stop.set()
        # Cancel auto-clear timers so they don't fire after shutdown.
        with self._lock:
            for attr in ("_light_clear_timer", "_buzzer_clear_timer"):
                t = getattr(self, attr, None)
                if t is not None:
                    t.cancel()
                    setattr(self, attr, None)
        try:
            self._tx_queue.put_nowait(None)
        except queue.Full:
            pass
        self._tx_thread.join(timeout=timeout)
