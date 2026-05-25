#!/usr/bin/env python3
"""
peripheral_hub.py — driver for the K-Patrol D1 R32 peripheral hub (firmware v2.0).

Speaks the UART line protocol at 115200 bps documented in
`robots/firmware/peripheral_hub_d1r32/peripheral_hub_d1r32.ino`:

  MCU → Pi (telemetry, one line per row):
      BOOT:periph-hub-v2.0,sda=21,scl=22,dht=25,relay=26
      HB:t=<ms>                            every 1 s
      DHT:<t>,<h>                          every 5 s (or DHT:nan,nan / DHT:disabled)
      STATE:relay=<0|1>                    on change + every 5 s
      HEAP:<bytes>                         every 30 s
      META:fw=...,uptime=...,watchdog_armed=<0|1>,relay_pol=<low|high>,time=<HH:MM:SS|?>
      PONG:t=<ms>                          reply to PING
      WATCHDOG:fired,reason=<str>          when 5-s safety auto-OFF triggers
      TIME:set,<HH:MM:SS>                  ack after a TIME push
      ERR:<reason>                         parser / validation failures

  Pi → MCU (commands):
      PING / STATUS / KEEPALIVE
      RELAY:ON | RELAY:OFF | RELAY:T | RELAY:TEST
      RELAY:POL:LOW | RELAY:POL:HIGH
      TIME:HH:MM[:SS]   (Pi syncs wall-clock every 60 s)
      OLED:<text>        (≤20 chars shown under the smiley face)

Threading model:
  * A single read thread owns the serial port and parses incoming lines.
  * Reads + writes are guarded by `_io_lock` so command sends from the main
    thread don't race with the read loop's reconnect path.
  * State updates are guarded by `_state_lock` so callers calling
    `snapshot()` / `to_dict()` always see a self-consistent view.

The hub is non-essential for motion safety, so this driver never blocks the
caller: a missing /dev/ttyUSB1, a wrong baud, or a hung MCU just leaves
`state.connected = False` and the read thread keeps retrying every
`connect_retry_period_s` seconds.
"""

from __future__ import annotations

import logging
import os
import re
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Dict, Optional

import serial


_RE_HB    = re.compile(r"^HB:t=(\d+)$")
_RE_DHT   = re.compile(r"^DHT:(-?\d+(?:\.\d+)?),(-?\d+(?:\.\d+)?)$")
_RE_STATE = re.compile(r"^STATE:relay=([01])$")
_RE_HEAP  = re.compile(r"^HEAP:(\d+)$")
_RE_META  = re.compile(r"^META:(.+)$")
_RE_WDT   = re.compile(r"^WATCHDOG:fired,reason=(.+)$")


@dataclass
class PeripheralState:
    """Latest known state of the peripheral hub. Returned from `snapshot()`."""

    connected: bool = False
    relay: bool = False
    watchdog_armed: bool = False
    temperature_c: Optional[float] = None
    humidity_pct: Optional[float] = None
    dht_disabled: bool = False
    last_hb_ms: int = 0          # MCU-side millis at last HB
    last_hb_local_ts: float = 0.0  # local wall-clock at last HB (time.time())
    free_heap: int = 0
    fw_version: str = ""
    boot_count: int = 0
    relay_polarity: str = ""     # "active_low" | "active_high"
    time_known_mcu: bool = False
    last_err: str = ""


# Heartbeat is 1 Hz; anything older than this many seconds means the hub is
# offline (cable yanked, MCU rebooting, etc).
HB_STALE_S = 5.0


class PeripheralHub:
    """Manages the D1 R32 peripheral hub over a serial port.

    Lifecycle: `start()` to spin the read + time-sync threads, `stop()` to
    tear down. All command methods (`relay_on()`, `set_oled_text()`, …)
    return True iff the line was successfully written to the serial port.
    """

    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        logger: Optional[logging.Logger] = None,
        time_sync_period_s: float = 60.0,
        connect_retry_period_s: float = 5.0,
        port_rediscover_fn: Optional[Callable[[], Optional[str]]] = None,
    ):
        self.port = port
        self.baudrate = baudrate
        self.logger = logger or logging.getLogger("peripheral_hub")
        self.time_sync_period_s = time_sync_period_s
        self.connect_retry_period_s = connect_retry_period_s
        # Optional hook: if the current self.port disappears (USB unplug or
        # user-initiated cable swap), call this to ask the integrator for a
        # fresh port path. Returning None means "no candidate right now";
        # the hub will then keep retrying the old path. This is how Python-
        # side content-discovery (sniff for `STATE:`/`META:`) is wired in
        # without giving the hub a hard dependency on the bridge module.
        self.port_rediscover_fn = port_rediscover_fn

        self.serial: Optional[serial.Serial] = None
        self.state = PeripheralState()
        self._state_lock = threading.Lock()
        self._io_lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None
        self._time_thread: Optional[threading.Thread] = None
        self._last_reconnect_attempt = 0.0

        # Optional callbacks — integrators can hook these for MQTT fan-out,
        # logging, etc. All callbacks run on the read thread; keep them fast.
        self.on_state_change: Optional[Callable[[PeripheralState], None]] = None
        self.on_dht: Optional[Callable[[float, float], None]] = None
        self.on_watchdog_fired: Optional[Callable[[str], None]] = None
        self.on_boot: Optional[Callable[[str], None]] = None

    # ───── Lifecycle ────────────────────────────────────────────────────────
    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._read_thread = threading.Thread(
            target=self._read_loop, name="periph-read", daemon=True
        )
        self._read_thread.start()
        self._time_thread = threading.Thread(
            target=self._time_sync_loop, name="periph-time", daemon=True
        )
        self._time_thread.start()
        self.logger.info(f"[periph] driver started, port={self.port}")

    def stop(self) -> None:
        self._running = False
        with self._io_lock:
            if self.serial is not None:
                try:
                    self.serial.close()
                except Exception:
                    pass
                self.serial = None
        with self._state_lock:
            self.state.connected = False

    # ───── Connection ──────────────────────────────────────────────────────
    def _try_connect(self) -> bool:
        # If the currently-pinned path is gone, ask the rediscover hook for a
        # fresh one. This is what makes "cable swap between USB slots" survive
        # without restarting the service — the integrator's hook sniffs every
        # ttyUSB*/ttyACM* and returns the one that's emitting periph-hub
        # signatures right now.
        if self.port_rediscover_fn is not None and not os.path.exists(self.port):
            try:
                new_port = self.port_rediscover_fn()
            except Exception as exc:
                self.logger.warning(f"[periph] rediscover hook raised: {exc}")
                new_port = None
            if new_port and new_port != self.port:
                self.logger.info(f"[periph] port rediscovered: {self.port} -> {new_port}")
                self.port = new_port
        try:
            ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.5,
                write_timeout=1.0,
            )
            with self._io_lock:
                self.serial = ser
            with self._state_lock:
                self.state.connected = True
            self.logger.info(f"[periph] connected to {self.port}")
            # Give the MCU a moment to finish its boot banner, then ask for state.
            time.sleep(0.3)
            self.request_status()
            self._push_time()  # immediate sync so OLED shows real time fast
            return True
        except Exception as exc:
            self.logger.warning(f"[periph] connect failed: {exc}")
            with self._state_lock:
                self.state.connected = False
            return False

    # ───── Read loop ───────────────────────────────────────────────────────
    def _read_loop(self) -> None:
        buf = bytearray()
        while self._running:
            if self.serial is None:
                now = time.monotonic()
                if now - self._last_reconnect_attempt >= self.connect_retry_period_s:
                    self._last_reconnect_attempt = now
                    self._try_connect()
                if self.serial is None:
                    time.sleep(0.5)
                    continue

            try:
                chunk = self.serial.read(256)
                if not chunk:
                    # Read timeout — check for HB staleness then keep waiting.
                    self._check_hb_staleness()
                    continue
                buf.extend(chunk)
                while b"\n" in buf:
                    raw_line, _, rest = buf.partition(b"\n")
                    buf = bytearray(rest)
                    line = raw_line.strip(b"\r").decode("utf-8", errors="replace").strip()
                    if line:
                        try:
                            self._parse_line(line)
                        except Exception as exc:
                            self.logger.exception(f"[periph] parse error: {exc} on {line!r}")
            except (serial.SerialException, OSError) as exc:
                self.logger.warning(f"[periph] read err: {exc}")
                with self._io_lock:
                    try:
                        if self.serial is not None:
                            self.serial.close()
                    except Exception:
                        pass
                    self.serial = None
                with self._state_lock:
                    self.state.connected = False
                time.sleep(1.0)

    def _check_hb_staleness(self) -> None:
        """If we haven't seen HB for too long, flag connected=False even if
        the serial port itself looks fine — the MCU might be wedged."""
        with self._state_lock:
            if not self.state.connected:
                return
            if self.state.last_hb_local_ts <= 0:
                return
            if time.time() - self.state.last_hb_local_ts > HB_STALE_S * 3:
                self.state.connected = False
                self.logger.warning("[periph] HB stale, marking disconnected")

    def _parse_line(self, line: str) -> None:
        # ── BOOT — clear stale state, bump boot count ────────────────────
        if line.startswith("BOOT:"):
            with self._state_lock:
                self.state.boot_count += 1
                # Pick out fw= field for display.
                fw = ""
                for part in line[5:].split(","):
                    part = part.strip()
                    if part.startswith("periph-hub-"):
                        fw = part
                        break
                self.state.fw_version = fw or "unknown"
                # Don't reset relay here — the firmware will emit a fresh
                # STATE line within ~5 s. Leaving the cached value lets the
                # UI show "stale" rather than a flicker to OFF.
            self.logger.info(f"[periph] boot {line!r}")
            if self.on_boot:
                try:
                    self.on_boot(line)
                except Exception as exc:
                    self.logger.error(f"[periph] on_boot cb err: {exc}")
            return

        # ── HB:t=<ms> ────────────────────────────────────────────────────
        m = _RE_HB.match(line)
        if m:
            with self._state_lock:
                self.state.last_hb_ms = int(m.group(1))
                self.state.last_hb_local_ts = time.time()
                self.state.connected = True
            return

        # ── DHT: <t>,<h>  or  DHT:disabled  or  DHT:nan,nan ─────────────
        if line.startswith("DHT:"):
            payload = line[4:]
            if payload == "disabled":
                with self._state_lock:
                    self.state.dht_disabled = True
                    self.state.temperature_c = None
                    self.state.humidity_pct = None
                return
            if payload.startswith("disabled,"):
                # e.g. "DHT:disabled,fails=3,retry_in=30000ms"
                with self._state_lock:
                    self.state.dht_disabled = True
                self.logger.warning(f"[periph] dht disabled: {payload}")
                return
            if payload == "nan,nan":
                with self._state_lock:
                    self.state.temperature_c = None
                    self.state.humidity_pct = None
                return
            m = _RE_DHT.match(line)
            if m:
                try:
                    t = float(m.group(1))
                    h = float(m.group(2))
                    with self._state_lock:
                        self.state.temperature_c = t
                        self.state.humidity_pct = h
                        self.state.dht_disabled = False
                    if self.on_dht:
                        try:
                            self.on_dht(t, h)
                        except Exception as exc:
                            self.logger.error(f"[periph] on_dht cb err: {exc}")
                except ValueError:
                    pass
            return

        # ── STATE:relay=<0|1> ───────────────────────────────────────────
        m = _RE_STATE.match(line)
        if m:
            new_relay = bool(int(m.group(1)))
            changed = False
            with self._state_lock:
                changed = self.state.relay != new_relay
                self.state.relay = new_relay
            if changed and self.on_state_change:
                try:
                    self.on_state_change(self.snapshot())
                except Exception as exc:
                    self.logger.error(f"[periph] on_state_change cb err: {exc}")
            return

        # ── HEAP:<bytes> ────────────────────────────────────────────────
        m = _RE_HEAP.match(line)
        if m:
            with self._state_lock:
                self.state.free_heap = int(m.group(1))
            return

        # ── META:k=v,k=v,... ────────────────────────────────────────────
        m = _RE_META.match(line)
        if m:
            self._parse_meta(m.group(1))
            return

        # ── WATCHDOG:fired,reason=... ───────────────────────────────────
        m = _RE_WDT.match(line)
        if m:
            reason = m.group(1)
            with self._state_lock:
                self.state.relay = False
                self.state.watchdog_armed = False
            self.logger.warning(f"[periph] watchdog fired: {reason}")
            if self.on_watchdog_fired:
                try:
                    self.on_watchdog_fired(reason)
                except Exception as exc:
                    self.logger.error(f"[periph] on_watchdog cb err: {exc}")
            if self.on_state_change:
                try:
                    self.on_state_change(self.snapshot())
                except Exception as exc:
                    self.logger.error(f"[periph] on_state_change cb err: {exc}")
            return

        # ── PONG: / TIME:set, / ERR: ────────────────────────────────────
        if line.startswith("PONG:"):
            return
        if line.startswith("TIME:set,"):
            with self._state_lock:
                self.state.time_known_mcu = True
            return
        if line.startswith("ERR:"):
            with self._state_lock:
                self.state.last_err = line[4:]
            self.logger.warning(f"[periph] mcu err: {line}")
            return

        # Anything else — debug-log it so we can audit unknown frames.
        self.logger.debug(f"[periph] <- {line}")

    def _parse_meta(self, payload: str) -> None:
        kv: Dict[str, str] = {}
        for part in payload.split(","):
            if "=" in part:
                k, _, v = part.partition("=")
                kv[k.strip()] = v.strip()
        with self._state_lock:
            self.state.fw_version = kv.get("fw", self.state.fw_version)
            self.state.watchdog_armed = kv.get("watchdog_armed") == "1"
            self.state.relay_polarity = kv.get("relay_pol", self.state.relay_polarity)
            t = kv.get("time", "?")
            self.state.time_known_mcu = t != "?"

    # ───── Time sync ───────────────────────────────────────────────────────
    def _time_sync_loop(self) -> None:
        # Don't push immediately — `_try_connect()` already did one push as
        # part of the connect handshake. Sleep first, then go on cadence.
        while self._running:
            time.sleep(self.time_sync_period_s)
            self._push_time()

    def _push_time(self) -> bool:
        now = time.localtime()
        cmd = f"TIME:{now.tm_hour:02d}:{now.tm_min:02d}:{now.tm_sec:02d}"
        return self._send(cmd, quiet=True)

    # ───── Command API ─────────────────────────────────────────────────────
    def _send(self, cmd: str, quiet: bool = False) -> bool:
        with self._io_lock:
            ser = self.serial
            if ser is None or not ser.is_open:
                if not quiet:
                    self.logger.debug(f"[periph] skip send (offline): {cmd}")
                return False
            try:
                ser.write((cmd + "\n").encode("ascii", errors="replace"))
                ser.flush()
                return True
            except (serial.SerialException, OSError) as exc:
                self.logger.warning(f"[periph] write err: {exc} on {cmd!r}")
                try:
                    ser.close()
                except Exception:
                    pass
                self.serial = None
                with self._state_lock:
                    self.state.connected = False
                return False

    # — convenience wrappers —
    def ping(self) -> bool:           return self._send("PING")
    def request_status(self) -> bool: return self._send("STATUS")
    def keepalive(self) -> bool:      return self._send("KEEPALIVE", quiet=True)
    def relay_on(self) -> bool:       return self._send("RELAY:ON")
    def relay_off(self) -> bool:      return self._send("RELAY:OFF")
    def relay_toggle(self) -> bool:   return self._send("RELAY:T")
    def relay_click_test(self) -> bool: return self._send("RELAY:TEST")

    def set_relay_polarity(self, active_low: bool) -> bool:
        return self._send("RELAY:POL:LOW" if active_low else "RELAY:POL:HIGH")

    def push_time_now(self) -> bool:
        """Force an immediate TIME sync (in addition to the 60-s cadence)."""
        return self._push_time()

    def set_oled_text(self, text: str) -> bool:
        # Strip newlines / carriage returns so we don't accidentally inject
        # extra commands; truncate to the firmware's OLED_STATUS_MAX-1 (20).
        clean = text.replace("\n", " ").replace("\r", " ").strip()[:20]
        return self._send(f"OLED:{clean}")

    # ───── Snapshot ────────────────────────────────────────────────────────
    def snapshot(self) -> PeripheralState:
        with self._state_lock:
            return PeripheralState(
                connected=self.state.connected,
                relay=self.state.relay,
                watchdog_armed=self.state.watchdog_armed,
                temperature_c=self.state.temperature_c,
                humidity_pct=self.state.humidity_pct,
                dht_disabled=self.state.dht_disabled,
                last_hb_ms=self.state.last_hb_ms,
                last_hb_local_ts=self.state.last_hb_local_ts,
                free_heap=self.state.free_heap,
                fw_version=self.state.fw_version,
                boot_count=self.state.boot_count,
                relay_polarity=self.state.relay_polarity,
                time_known_mcu=self.state.time_known_mcu,
                last_err=self.state.last_err,
            )

    def to_dict(self) -> Dict:
        """JSON-safe dict for MQTT publish. Field names match the schema the
        backend's MqttIngestService expects on `kpatrol/<serial>/peripherals/state`."""
        s = self.snapshot()
        return {
            "connected": s.connected,
            "relay": s.relay,
            "watchdog_armed": s.watchdog_armed,
            "temperature_c": s.temperature_c,
            "humidity_pct": s.humidity_pct,
            "dht_disabled": s.dht_disabled,
            "free_heap": s.free_heap,
            "fw_version": s.fw_version,
            "relay_polarity": s.relay_polarity,
            "time_known_mcu": s.time_known_mcu,
            "last_seen_ms": int(s.last_hb_local_ts * 1000) if s.last_hb_local_ts else 0,
        }


# Allow running as a standalone smoke test:  python3 peripheral_hub.py /dev/ttyUSB0
if __name__ == "__main__":
    import sys

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s %(message)s",
    )
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    hub = PeripheralHub(port=port)

    def _on_dht(t: float, h: float) -> None:
        print(f"  DHT: {t:.1f}°C  {h:.0f}% RH")

    def _on_state(s: PeripheralState) -> None:
        print(f"  STATE: relay={int(s.relay)}  wdt={int(s.watchdog_armed)}")

    def _on_wdt(reason: str) -> None:
        print(f"  WATCHDOG: {reason}")

    hub.on_dht = _on_dht
    hub.on_state_change = _on_state
    hub.on_watchdog_fired = _on_wdt
    hub.start()

    print(f"Connected to {port}. Type commands: ON / OFF / T / TEST / OLED <text> / QUIT")
    try:
        while True:
            cmd = input("> ").strip().upper()
            if not cmd:
                continue
            if cmd == "QUIT":
                break
            elif cmd == "ON":
                hub.relay_on()
            elif cmd == "OFF":
                hub.relay_off()
            elif cmd == "T":
                hub.relay_toggle()
            elif cmd == "TEST":
                hub.relay_click_test()
            elif cmd == "STATUS":
                hub.request_status()
                time.sleep(0.3)
                print(hub.to_dict())
            elif cmd.startswith("OLED "):
                hub.set_oled_text(cmd[5:])
            else:
                print(f"unknown: {cmd}")
    except KeyboardInterrupt:
        pass
    finally:
        hub.stop()
        print("bye")
