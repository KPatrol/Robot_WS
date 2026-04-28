#!/usr/bin/env python3
"""
K-Patrol GPS Reader — NEO-6M Module via UART/USB-TTL
=====================================================

Reads NMEA 0183 sentences from a u-blox NEO-6M GPS module and exposes the
parsed fix as a thread-safe :class:`GPSData` snapshot. Designed for outdoor
patrol mode where the robot must report absolute geo-position to the
backend in addition to encoder/IMU dead-reckoning.

Hardware wiring (recommended: USB-TTL adapter — CP2102/CH340 → /dev/ttyUSB1)
    NEO-6M VCC  → 5V (module has on-board 3.3V LDO)
    NEO-6M GND  → GND
    NEO-6M TX   → USB-TTL RX
    NEO-6M RX   → USB-TTL TX  (optional, only for UBX config writes)

Why USB-TTL instead of Pi UART (GPIO 14/15)?
    1. Pi UART is shared with the kernel serial console — disabling it
       requires raspi-config + reboot, which is fragile in deployment.
    2. USB is hot-pluggable; if the antenna gets unplugged we just
       reconnect, no kernel module reload.
    3. Keeps GPIO bus free for future expansion (PWM, I2C peripherals).

NMEA sentences parsed:
    $GPGGA  — fix quality, lat/lon, altitude, # satellites, HDOP
    $GPRMC  — UTC date/time, lat/lon, ground speed (knots), course
    $GPVTG  — track over ground, speed in km/h
    $GPGSA  — DOP and active satellites (PDOP/HDOP/VDOP)

Output:
    Thread-safe :func:`get_data()` → :class:`GPSData` snapshot
    Public attributes are immutable per snapshot (returned by value).

Usage (standalone test):
    python3 gps_reader.py --port /dev/ttyUSB1
    python3 gps_reader.py --port /dev/ttyUSB1 --json

Author: K-Patrol Team
Date:   2026-04-25
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from dataclasses import dataclass, asdict, field
from enum import IntEnum
from typing import Optional, Dict, Any, Callable

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)

try:
    import pynmea2
except ImportError:
    print("ERROR: pynmea2 not installed. Run: pip3 install pynmea2")
    sys.exit(1)


# ============================================================================
# CONSTANTS
# ============================================================================

class FixQuality(IntEnum):
    """NMEA $GPGGA fix quality field."""
    NO_FIX      = 0
    GPS_FIX     = 1   # Standard GPS (SPS)
    DGPS_FIX    = 2   # Differential GPS
    PPS_FIX     = 3
    RTK_FIXED   = 4
    RTK_FLOAT   = 5
    ESTIMATED   = 6   # Dead reckoning
    MANUAL      = 7
    SIMULATION  = 8


# Fix is "usable" when we have at least 4 satellites and HDOP <= 5.0.
# Below that, position drifts >50 m in our tests with the bare ceramic patch.
MIN_SATELLITES_USABLE = 4
MAX_HDOP_USABLE       = 5.0

# Stale threshold: if no GGA seen in this many seconds, report has_fix=False
# even if the last cached fix was good (antenna disconnect, long indoor stay).
FIX_STALE_SECONDS = 5.0


# ============================================================================
# DATA CLASSES
# ============================================================================

@dataclass
class GPSData:
    """Immutable snapshot of the latest GPS fix.

    All numeric fields default to 0/0.0 when no fix is available; consumers
    should always check :attr:`has_fix` before trusting the position.
    """
    # Fix status
    has_fix: bool = False
    fix_quality: int = 0           # FixQuality enum value
    satellites: int = 0
    hdop: float = 99.9             # Horizontal dilution of precision

    # Position (WGS-84)
    latitude: float = 0.0          # Decimal degrees, +N / -S
    longitude: float = 0.0         # Decimal degrees, +E / -W
    altitude: float = 0.0          # Metres above MSL (geoid)

    # Velocity
    speed_kmh: float = 0.0
    heading: float = 0.0           # Course over ground, degrees true (0–360)

    # Time
    utc_time: str = ""             # ISO-8601 UTC, e.g. "2026-04-25T14:32:18Z"
    fix_timestamp: float = 0.0     # Unix epoch seconds when fix was received

    # Diagnostics
    last_sentence: str = ""        # Last NMEA sentence type (debug aid)
    rx_count: int = 0              # Total NMEA sentences received since start

    def to_dict(self) -> Dict[str, Any]:
        """Serialise for MQTT payload (matches backend Telemetry GPS schema)."""
        return {
            "hasFix":       self.has_fix,
            "fixQuality":   self.fix_quality,
            "satellites":   self.satellites,
            "hdop":         round(self.hdop, 2),
            "latitude":     round(self.latitude, 7),     # ~1cm precision
            "longitude":    round(self.longitude, 7),
            "altitude":     round(self.altitude, 1),
            "speedKmh":     round(self.speed_kmh, 2),
            "heading":      round(self.heading, 1),
            "utcTime":      self.utc_time,
            "fixTimestamp": int(self.fix_timestamp * 1000),
            "timestamp":    int(time.time() * 1000),
            "rxCount":      self.rx_count,
            "lastSentence": self.last_sentence,
        }


# ============================================================================
# GPS READER
# ============================================================================

class GPSReader:
    """Threaded NMEA reader for u-blox NEO-6M GPS.

    The read loop runs at the module's native rate (1 Hz default) — there is
    no value in polling faster, so we just block on :meth:`serial.readline`
    with a short timeout to keep shutdown responsive.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB1",
        baudrate: int = 9600,
        on_fix_acquired: Optional[Callable[[GPSData], None]] = None,
        on_fix_lost: Optional[Callable[[], None]] = None,
        mode: str = "dedicated",
    ):
        """
        :param mode: ``"dedicated"`` (default) opens its own serial port and
            spawns a reader thread. ``"shared"`` skips the serial layer
            entirely — caller must push raw NMEA sentences via
            :meth:`feed_nmea` (used when ESP32-S3 forwards GPS data over the
            motor UART with a "NMEA:" prefix).
        """
        if mode not in ("dedicated", "shared"):
            raise ValueError(f"Unknown mode: {mode!r}")
        self.mode = mode
        self.port = port
        self.baudrate = baudrate
        self._serial: Optional[serial.Serial] = None
        self._lock = threading.Lock()
        self._data = GPSData()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._connected = False
        self._last_reconnect = 0.0
        self._reconnect_interval = 3.0
        self._last_gga_at = 0.0
        self._rx_total = 0

        # Callbacks fire on rising/falling edge of has_fix
        self.on_fix_acquired = on_fix_acquired
        self.on_fix_lost = on_fix_lost

    # ── lifecycle ────────────────────────────────────────────────────

    @property
    def connected(self) -> bool:
        return self._connected

    def connect(self) -> bool:
        if self.mode == "shared":
            # No serial of our own — caller will push NMEA via feed_nmea().
            self._connected = True
            self._running = True
            print(f"[GPS] Shared mode active — awaiting NMEA frames via feed_nmea()")
            return True
        try:
            if self._serial and self._serial.is_open:
                self._serial.close()
            self._serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
            )
            self._connected = True
            print(f"[GPS] Connected to {self.port} @ {self.baudrate} baud")

            if not self._running:
                self._running = True
                self._thread = threading.Thread(
                    target=self._read_loop,
                    name="gps_reader",
                    daemon=True,
                )
                self._thread.start()
            return True
        except serial.SerialException as e:
            print(f"[GPS] Connection failed: {e}")
            self._connected = False
            return False

    def disconnect(self) -> None:
        self._running = False
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._serial and self._serial.is_open:
            try:
                self._serial.close()
            except serial.SerialException:
                pass
        self._connected = False

    # ── public: shared-mode ingestion ──────────────────────────────

    def feed_nmea(self, line: str) -> None:
        """Inject a raw NMEA sentence (without trailing CRLF) into the parser.

        Used by the MQTT bridge when the ESP32-S3 forwards GPS data over the
        motor UART with the "NMEA:" prefix. The line should start with '$'
        and contain a checksum '*XX' suffix.
        """
        if not line:
            return
        line = line.strip()
        if line.startswith("$") and "*" in line:
            self._parse_nmea(line)

    # ── public accessors ────────────────────────────────────────────

    def get_data(self) -> GPSData:
        """Return a snapshot of the latest fix (thread-safe)."""
        with self._lock:
            d = self._data
            stale = (time.time() - self._last_gga_at) > FIX_STALE_SECONDS
            return GPSData(
                has_fix       = d.has_fix and not stale,
                fix_quality   = d.fix_quality,
                satellites    = d.satellites,
                hdop          = d.hdop,
                latitude      = d.latitude,
                longitude     = d.longitude,
                altitude      = d.altitude,
                speed_kmh     = d.speed_kmh,
                heading       = d.heading,
                utc_time      = d.utc_time,
                fix_timestamp = d.fix_timestamp,
                last_sentence = d.last_sentence,
                rx_count      = self._rx_total,
            )

    # ── internal: read loop ─────────────────────────────────────────

    def _try_reconnect(self) -> bool:
        if time.time() - self._last_reconnect < self._reconnect_interval:
            return False
        self._last_reconnect = time.time()
        return self.connect()

    def _read_loop(self) -> None:
        while self._running:
            if not self._connected or not self._serial:
                self._try_reconnect()
                time.sleep(1.0)
                continue
            try:
                raw = self._serial.readline()
                if not raw:
                    # Timeout — also a chance to mark fix stale via getter
                    continue
                line = raw.decode("ascii", errors="ignore").strip()
                if line.startswith("$") and "*" in line:
                    self._parse_nmea(line)
            except (OSError, serial.SerialException) as e:
                if self._running:
                    print(f"[GPS] Read error: {e}")
                    self._connected = False
            except UnicodeDecodeError:
                # Garbage frame on cold start — ignore
                pass

    # ── internal: NMEA parsing ──────────────────────────────────────

    def _parse_nmea(self, sentence: str) -> None:
        try:
            msg = pynmea2.parse(sentence)
        except pynmea2.ParseError:
            return

        with self._lock:
            self._rx_total += 1
            self._data.last_sentence = msg.sentence_type

            stype = msg.sentence_type
            if stype == "GGA":
                self._apply_gga(msg)
            elif stype == "RMC":
                self._apply_rmc(msg)
            elif stype == "VTG":
                self._apply_vtg(msg)
            elif stype == "GSA":
                self._apply_gsa(msg)

    def _apply_gga(self, m: "pynmea2.GGA") -> None:
        was_fix = self._data.has_fix
        try:
            quality = int(m.gps_qual or 0)
            sats    = int(m.num_sats or 0)
            hdop    = float(m.horizontal_dil) if m.horizontal_dil else 99.9
        except (ValueError, TypeError):
            quality, sats, hdop = 0, 0, 99.9

        usable = (
            quality > 0
            and sats >= MIN_SATELLITES_USABLE
            and hdop <= MAX_HDOP_USABLE
            and m.latitude is not None
            and m.longitude is not None
        )

        self._data.fix_quality = quality
        self._data.satellites  = sats
        self._data.hdop        = hdop
        self._data.has_fix     = bool(usable)

        if usable:
            self._data.latitude  = float(m.latitude)
            self._data.longitude = float(m.longitude)
            try:
                self._data.altitude = float(m.altitude or 0.0)
            except (ValueError, TypeError):
                pass
            self._data.fix_timestamp = time.time()
            self._last_gga_at = time.time()

        # Edge-triggered callbacks
        if usable and not was_fix and self.on_fix_acquired:
            try:
                self.on_fix_acquired(self._data)
            except Exception as e:
                print(f"[GPS] on_fix_acquired callback error: {e}")
        elif (not usable) and was_fix and self.on_fix_lost:
            try:
                self.on_fix_lost()
            except Exception as e:
                print(f"[GPS] on_fix_lost callback error: {e}")

    def _apply_rmc(self, m: "pynmea2.RMC") -> None:
        # RMC also carries lat/lon/speed/course but GGA is preferred for
        # fix quality. We keep RMC for UTC time and heading fallback.
        try:
            if m.datestamp and m.timestamp:
                self._data.utc_time = (
                    f"{m.datestamp.isoformat()}T{m.timestamp.isoformat()}Z"
                )
        except (ValueError, AttributeError):
            pass

        try:
            if m.spd_over_grnd is not None:
                # RMC speed is in knots; convert to km/h.
                self._data.speed_kmh = float(m.spd_over_grnd) * 1.852
        except (ValueError, TypeError):
            pass

        try:
            if m.true_course is not None:
                self._data.heading = float(m.true_course)
        except (ValueError, TypeError):
            pass

    def _apply_vtg(self, m: "pynmea2.VTG") -> None:
        # VTG gives speed in km/h directly — preferred over RMC conversion.
        try:
            if m.spd_over_grnd_kmph is not None:
                self._data.speed_kmh = float(m.spd_over_grnd_kmph)
        except (ValueError, TypeError):
            pass

        try:
            if m.true_track is not None:
                self._data.heading = float(m.true_track)
        except (ValueError, TypeError):
            pass

    def _apply_gsa(self, m: "pynmea2.GSA") -> None:
        # GSA refines HDOP between GGA messages.
        try:
            if m.hdop is not None:
                self._data.hdop = float(m.hdop)
        except (ValueError, TypeError):
            pass


# ============================================================================
# STANDALONE MAIN — for bench testing without the MQTT bridge
# ============================================================================

def main() -> None:
    parser = argparse.ArgumentParser(description="K-Patrol NEO-6M GPS Reader")
    parser.add_argument("--port", "-p", default="/dev/ttyUSB1",
                        help="Serial port (default: /dev/ttyUSB1)")
    parser.add_argument("--baud", "-b", type=int, default=9600,
                        help="Baud rate (default: 9600)")
    parser.add_argument("--json", "-j", action="store_true",
                        help="Print JSON each update instead of human-readable")
    parser.add_argument("--interval", "-i", type=float, default=1.0,
                        help="Print interval in seconds (default: 1.0)")
    args = parser.parse_args()

    reader = GPSReader(port=args.port, baudrate=args.baud)

    def on_fix(data: GPSData) -> None:
        print(f"\n[GPS] Fix acquired: {data.satellites} sats, HDOP={data.hdop:.1f}")

    def on_lost() -> None:
        print("\n[GPS] Fix lost!")

    reader.on_fix_acquired = on_fix
    reader.on_fix_lost = on_lost

    if not reader.connect():
        sys.exit(1)

    print("Reading GPS data... (Ctrl+C to exit)\n")
    try:
        while True:
            data = reader.get_data()
            if args.json:
                print(json.dumps(data.to_dict()))
            else:
                if data.has_fix:
                    print(
                        f"\r[FIX] {data.latitude:.6f}, {data.longitude:.6f} | "
                        f"alt={data.altitude:5.1f}m | "
                        f"spd={data.speed_kmh:5.2f}km/h | "
                        f"hdg={data.heading:5.1f}° | "
                        f"sats={data.satellites:2d} | "
                        f"HDOP={data.hdop:.1f} | "
                        f"rx={data.rx_count}",
                        end="", flush=True,
                    )
                else:
                    print(
                        f"\r[ -- ] searching... "
                        f"sats={data.satellites:2d} HDOP={data.hdop:.1f} "
                        f"last={data.last_sentence:<4} rx={data.rx_count}",
                        end="", flush=True,
                    )
            time.sleep(args.interval)
    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        reader.disconnect()


if __name__ == "__main__":
    main()
