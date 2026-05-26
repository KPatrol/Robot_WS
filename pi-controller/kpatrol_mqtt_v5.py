#!/usr/bin/env python3
"""
kpatrol_mqtt_v5.py — K-Patrol MQTT Controller V5.3
===================================================
Directional safety + 3 autonomous navigation modes + safety hardware.

Navigation modes (NavController, 5 total):
  • MANUAL              — operator drives via kpatrol/{serial}/motor
  • AUTO_FREE_COVERAGE  — indoor random-walk + frontier exploration
  • AUTO_LINE_FOLLOW    — camera floor-line PD follower (BEV + HSV)
  • AUTO_GPS_WAYPOINT   — outdoor GPS route (Haversine + IMU + ToF safety)
  • EMERGENCY           — motors stopped, waiting for operator clear

MQTT topics (scoped to kpatrol/{serial}/…):
  Subscribe:
    nav_command       {"mode":"MANUAL|AUTO_FREE_COVERAGE|AUTO_LINE_FOLLOW|"
                       "AUTO_GPS_WAYPOINT|EMERGENCY",
                       "speed":60, "action":"clear_emergency"}
    gps_route         {"action":"set|start|stop",
                       "waypoints":[{"lat":..., "lon":..., "radius_m":3}],
                       "loop":false}
    buzzer            {"pattern":"OFF|ON|BEEP|ALARM|SOS"}
    light_pattern     {"pattern":"OFF|WARN_BLINK|WARN_STROBE|"
                       "BOTH_BLINK|SOS"}
  Publish:
    nav_status     FSM state, current waypoint, progress, errors
    gps_status     GPS route progress + nav state

Hardware (V5.3):
  ESP32-S3  → /dev/ttyKPATROL_MOTOR   (motors + BNO08x IMU + GPS UART2 +
                                       buzzer + e-stop button + remote relay)
  ESP32     → /dev/ttyKPATROL_ENCODER (4 encoders + 6x VL53L0X ToF)
"""

import glob
import json
import logging
import time
import sys
import os
import re
import threading
from typing import Optional, Dict, Any, Tuple, List, Callable, ClassVar, Union
from dataclasses import dataclass, asdict, field
from enum import Enum

logger = logging.getLogger("kpatrol.mqtt")

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("ERROR: paho-mqtt not installed. Run: pip3 install paho-mqtt")
    sys.exit(1)

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip3 install pyserial")
    sys.exit(1)

# Add parent directory to path so navigation package resolves
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from peripheral_hub import PeripheralHub, PeripheralState  # type: ignore
from navigation import (
    NavController, Mode, LineFollowerConfig, HSVRange, CoverageConfig,
    Odometry, VelocityController, VelocityPIDConfig,
    open_camera,
)

# Optional OpenCV — required for LINE_FOLLOW mode
try:
    import cv2 as _cv2
    _CV2_AVAILABLE = True
except ImportError:
    _cv2 = None  # type: ignore
    _CV2_AVAILABLE = False

# V5.2: NEO-6M GPS reader (optional — falls back gracefully if module missing)
try:
    from gps_reader import GPSReader, GPSData
    _GPS_AVAILABLE = True
except ImportError as _gps_err:
    logger.warning(f"[GPS] Module unavailable: {_gps_err} — outdoor mode disabled")
    GPSReader = None  # type: ignore
    GPSData = None    # type: ignore
    _GPS_AVAILABLE = False

# V5.3: Edge-AI anomaly detector (optional — graceful no-op if cv2/ultralytics
# missing on dev box). When present, the detector runs inline in this process
# so safety reactions (fire→EMERGENCY, person→pause) avoid an MQTT round-trip.
try:
    from detection.anomaly_detector import (
        AnomalyDetector, DetectionConfig, DetectionEvent,
    )
    from detection.alert_db import AlertStore
    _DETECTION_AVAILABLE = True
except ImportError as _det_err:
    logger.warning(f"[DETECT] Module unavailable: {_det_err} — anomaly detection disabled")
    AnomalyDetector = None    # type: ignore
    DetectionConfig = None    # type: ignore
    DetectionEvent = None     # type: ignore
    AlertStore = None         # type: ignore
    _DETECTION_AVAILABLE = False

# V5.4: Safety actuators / watchers — light + buzzer reaction, tip-over guard,
# battery low/critical hysteresis. Pure-Python, no extra deps; if the package
# is somehow missing we fall back to no-op stubs so the bridge keeps running.
try:
    from safety import (
        AlertActuator, ActuatorConfig,
        TipOverWatcher, TipOverConfig,
        BatteryWatcher, BatteryConfig,
        AlarmController, AlarmRule,
        EVENT_PERSON, EVENT_FIRE,
        EVENT_BATTERY_LOW, EVENT_BATTERY_CRITICAL,
        EVENT_TIPOVER, EVENT_SCHEDULE,
        EVENT_SYSTEM_ERROR, EVENT_ANY_SAFETY,
    )
    _SAFETY_AVAILABLE = True
except ImportError as _saf_err:
    logger.warning(f"[SAFETY] Module unavailable: {_saf_err} — actuator/watchers disabled")
    AlertActuator = None      # type: ignore
    ActuatorConfig = None     # type: ignore
    TipOverWatcher = None     # type: ignore
    TipOverConfig = None      # type: ignore
    BatteryWatcher = None     # type: ignore
    BatteryConfig = None      # type: ignore
    AlarmController = None    # type: ignore
    AlarmRule = None          # type: ignore
    EVENT_PERSON = "person"
    EVENT_FIRE = "fire"
    EVENT_BATTERY_LOW = "battery_low"
    EVENT_BATTERY_CRITICAL = "battery_critical"
    EVENT_TIPOVER = "tipover"
    EVENT_SCHEDULE = "schedule"
    EVENT_SYSTEM_ERROR = "system_error"
    EVENT_ANY_SAFETY = "any_safety"
    _SAFETY_AVAILABLE = False


# ==================== CONFIGURATION ====================

@dataclass
class MQTTConfig:
    host: str = os.environ.get("MQTT_HOST", "")
    port: int = int(os.environ.get("MQTT_PORT", "1883"))
    username: str = os.environ.get("MQTT_USERNAME", "")
    password: str = os.environ.get("MQTT_PASSWORD", "")
    client_id: str = f"kpatrol_pi_{os.uname().nodename}"
    keepalive: int = 60
    qos: int = 1
    robot_serial: str = os.environ.get("ROBOT_SERIAL", "KPATROL-001")

    def validate(self) -> None:
        missing = [k for k, v in {
            "MQTT_HOST": self.host,
            "MQTT_USERNAME": self.username,
            "MQTT_PASSWORD": self.password,
        }.items() if not v]
        if missing:
            raise RuntimeError(
                f"MQTT configuration missing: {', '.join(missing)}. "
                f"Source mqtt.env or export the variables before running."
            )


@dataclass
class SerialConfig:
    motor_port: str = "/dev/ttyKPATROL_MOTOR"
    encoder_port: str = "/dev/ttyKPATROL_ENCODER"
    # V5.6: D1 R32 peripheral hub (OLED + DHT11 + relay light/horn).
    # Non-essential — if the port is missing / disabled, the rest of the
    # stack runs unaffected.
    periph_port: Optional[str] = "/dev/kpatrol-periph"
    baudrate: int = 115200
    timeout: float = 1.0


@dataclass
class GPSConfig:
    """NEO-6M GPS module configuration.

    Two wiring modes are supported:

    * "shared"    — NEO-6M is wired to the motor ESP32-S3 (UART2 GPIO 41/42).
                    Firmware forwards every NMEA sentence to the Pi as
                    `NMEA:$GPxxx,...` lines on the motor USB serial.
                    GPSReader runs without owning a serial port; it is
                    fed via feed_nmea() from the motor read loop.
    * "dedicated" — NEO-6M is wired through a USB-TTL adapter on its own
                    /dev/ttyKPATROL_GPS port. GPSReader opens the port
                    and reads NMEA directly.

    "shared" is the default for V5.2 outdoor builds (one less USB cable).
    """
    port: str = "/dev/ttyKPATROL_GPS"
    baudrate: int = 9600
    enabled: bool = True
    publish_interval: float = 1.0   # seconds; NEO-6M emits at 1Hz natively
    mode: str = "shared"            # "shared" | "dedicated"


def make_topics(serial: str) -> "Topics":
    """Factory: create a Topics instance scoped to the given robot serial."""
    return Topics(serial)


class Topics:
    """
    Per-robot MQTT topic namespace.

    All topics follow the pattern: kpatrol/{serial}/{subtopic}

    Examples (serial = "KPATROL-001"):
        T.COMMAND   → "kpatrol/KPATROL-001/command"
        T.HEARTBEAT → "kpatrol/KPATROL-001/heartbeat"

    Subscribe with wildcard: kpatrol/{serial}/#
    """

    def __init__(self, serial: str):
        self._serial = serial
        p = f"kpatrol/{serial}"
        # Commands: Web → Pi (subscribed by Pi)
        self.COMMAND       = f"{p}/command"
        self.MOTOR         = f"{p}/motor"
        self.SPEED         = f"{p}/speed"
        self.MODE          = f"{p}/mode"
        self.EMERGENCY     = f"{p}/emergency"
        self.LIGHT         = f"{p}/light"
        self.MAIN_LIGHT    = f"{p}/main_light"
        self.SAFETY_CONFIG = f"{p}/safety_config"
        # Status: Pi → Web (published by Pi)
        self.STATUS        = f"{p}/status"
        self.POSITION      = f"{p}/position"
        self.SENSORS       = f"{p}/sensors"
        self.MOTORS        = f"{p}/motors"
        self.ENCODERS      = f"{p}/encoders"
        self.BATTERY       = f"{p}/battery"
        self.HEARTBEAT     = f"{p}/heartbeat"
        self.LOG           = f"{p}/log"
        self.ERROR         = f"{p}/error"
        self.SAFETY        = f"{p}/safety"
        self.IMU           = f"{p}/imu"
        # V5.3: Navigation FSM (5 modes — see header docstring)
        self.NAV_CMD       = f"{p}/nav_command"
        self.NAV_STATUS    = f"{p}/nav_status"
        # V5.3: GPS waypoint route management (AUTO_GPS_WAYPOINT mode)
        self.GPS_ROUTE     = f"{p}/gps_route"
        self.GPS_STATUS    = f"{p}/gps_status"
        # V5.3: Safety hardware (firmware-side state machines)
        self.BUZZ          = f"{p}/buzzer"
        self.LIGHT_PATTERN = f"{p}/light_pattern"
        # V5.1: MQTT link telemetry (bytes in/out, msg counts, drop rate)
        self.METRICS       = f"{p}/metrics"
        # V5.2: Outdoor GPS (NEO-6M) — absolute geo-position
        self.GPS           = f"{p}/gps"
        # LINE_FOLLOW: JPEG overlay frames for dashboard live view
        self.CAMERA        = f"{p}/camera"
        # Edge-AI anomaly detection (person + fire) — published by inline detector
        self.ALERT         = f"{p}/alert"
        # V5.3: Mecanum odometry (encoder + IMU complementary filter) at 20Hz
        self.ODOM          = f"{p}/odom"
        self.ODOM_RESET    = f"{p}/odom_reset"
        # V5.5: Operator-configured alarm rules (Web → Pi) + fire reports (Pi → Web)
        # Rule payload: JSON list of AlarmRule dicts (id/name/event_type/enabled/
        # windows[start,end,weekdays]/continuous_duration_s/cooldown_s/light_pattern/
        # buzzer_pattern). Pi pushes a trigger record back on /alarm/triggered when
        # the dwell threshold has been satisfied inside the rule's time window.
        self.ALARM_RULES     = f"{p}/alarm/rules"
        self.ALARM_TRIGGERED = f"{p}/alarm/triggered"
        # V5.6: D1 R32 peripheral hub (OLED + DHT11 + relay).
        # PERIPH_STATE: Pi publishes the latest hub snapshot (relay, temp,
        #               humidity, watchdog_armed, fw, heap) every 5 s + on change.
        # PERIPH_CMD:   Web publishes commands (relay on/off/toggle, oled text,
        #               polarity flip, manual time push) for the Pi to forward
        #               down the UART to the D1 R32.
        self.PERIPH_STATE = f"{p}/peripherals/state"
        self.PERIPH_CMD   = f"{p}/peripherals/cmd"
        # Wildcard for subscribing all topics of this robot
        self.WILDCARD      = f"{p}/#"

    @property
    def serial(self) -> str:
        return self._serial


# Motor positions
MOTOR_POSITIONS = ['FR', 'FL', 'BR', 'BL']
MOTOR_INVERTED = {'FR': False, 'FL': True, 'BR': False, 'BL': True}


# ==================== NETWORK GEOLOCATION ====================
# V5.13: hardware NEO-6M can't see satellites indoor, so we synthesize a
# position three different ways with steeply decreasing accuracy:
#
#   1. Manual override (env KPATROL_GPS_OVERRIDE_LAT/LON) — operator pins
#      the exact lat/lon of the demo location. ~0 m error, but requires
#      knowing the spot in advance.
#   2. WiFi triangulation via BeaconDB (https://beacondb.net) — open-data
#      successor of Mozilla Location Service. POST nearby BSSIDs + RSSI,
#      get back a fix accurate to ~20-200 m. No API key needed.
#   3. IP geolocation via ip-api.com — ISP exit point, ~5-50 km off. Last
#      resort so the dashboard at least shows a city.
#
# Sources are tried in that order at every refresh; the highest-accuracy
# result wins. If WiFi scan fails (no permission, no nmcli) BeaconDB is
# silently skipped — the operator only sees IP-level accuracy, same as
# V5.12. Refresh cadence stays at 5 min so we don't hammer the public
# beacondb instance.
class NetworkGeolocator:
    REFRESH_S = 300            # 5 min between successful refreshes
    BACKOFF_S = 30             # short retry when a fetch fails
    HTTP_TIMEOUT_S = 6
    BEACONDB_URL = "https://api.beacondb.net/v1/geolocate"
    IPAPI_URL    = "http://ip-api.com/json/?fields=status,country,countryCode,city,lat,lon,query,timezone"

    def __init__(self):
        self._cache: Optional[Dict[str, Any]] = None
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        # Manual override: parsed once at init so env tweaks need a
        # service restart — that matches the existing config pattern
        # and avoids a thread reading os.environ on every refresh.
        self._override: Optional[Tuple[float, float]] = None
        lat_s = os.environ.get("KPATROL_GPS_OVERRIDE_LAT", "").strip()
        lon_s = os.environ.get("KPATROL_GPS_OVERRIDE_LON", "").strip()
        if lat_s and lon_s:
            try:
                self._override = (float(lat_s), float(lon_s))
                logger.info(f"[NetGPS] Manual override active: {self._override[0]:.6f}, {self._override[1]:.6f}")
            except ValueError:
                logger.warning(f"[NetGPS] Bad override coords: lat={lat_s!r}, lon={lon_s!r}")

    def start(self) -> None:
        if self._running:
            return
        self._running = True
        self._fetch_once()
        self._thread = threading.Thread(target=self._loop, name="netgps", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._running = False

    def _loop(self) -> None:
        while self._running:
            ok = self._fetch_once()
            time.sleep(self.REFRESH_S if ok else self.BACKOFF_S)

    # ─── BSSID scan ────────────────────────────────────────────────────
    def _scan_wifi(self) -> List[Dict[str, Any]]:
        """Return a list of nearby APs in the BeaconDB request schema.

        Uses NetworkManager's cached scan via ``nmcli`` — no sudo, no
        ``CAP_NET_ADMIN`` needed. NetworkManager refreshes the scan
        every ~30 s on its own, so calling this every 5 min always
        gets a fresh list. Empty list on any failure (nmcli missing,
        no WiFi adapter, escape parsing edge case).
        """
        try:
            import subprocess
            # -t for terse, -e for escape backslashes (BSSID octets use ':'
            # which nmcli would otherwise treat as field separators). We
            # pull BSSID + SIGNAL (0-100) + FREQ; SIGNAL gets converted
            # to dBm with NetworkManager's documented mapping.
            out = subprocess.run(
                ["nmcli", "-t", "-e", "yes", "-f", "BSSID,SIGNAL,FREQ", "dev", "wifi", "list"],
                capture_output=True, text=True, timeout=4,
            )
            if out.returncode != 0:
                return []
            aps: List[Dict[str, Any]] = []
            for line in out.stdout.splitlines():
                # nmcli escapes ':' inside the BSSID as '\:' so we re-join
                # the first six hex pairs by splitting on UN-escaped ':'.
                parts = re.split(r"(?<!\\):", line.strip())
                if len(parts) < 3:
                    continue
                bssid = parts[0].replace("\\:", ":").upper()
                if not re.match(r"^[0-9A-F]{2}(:[0-9A-F]{2}){5}$", bssid):
                    continue
                try:
                    signal_pct = int(parts[1])
                    freq_mhz = int(parts[2].split()[0])
                except (ValueError, IndexError):
                    continue
                # NetworkManager: 100% ≈ -50 dBm, 0% ≈ -100 dBm linear.
                signal_dbm = (signal_pct // 2) - 100
                aps.append({
                    "macAddress": bssid,
                    "signalStrength": signal_dbm,
                    "frequency": freq_mhz,
                })
            return aps
        except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
            logger.debug(f"[NetGPS] WiFi scan unavailable: {exc}")
            return []
        except Exception as exc:
            logger.warning(f"[NetGPS] WiFi scan failed: {exc}")
            return []

    # ─── Resolver tier 2: BeaconDB ─────────────────────────────────────
    def _resolve_beacondb(self) -> Optional[Dict[str, Any]]:
        aps = self._scan_wifi()
        if len(aps) < 2:
            # BeaconDB needs at least two beacons for a meaningful fix —
            # one AP yields a useless circle the size of the AP's range.
            return None
        try:
            import urllib.request as _ur
            body = json.dumps({"wifiAccessPoints": aps}).encode("utf-8")
            req = _ur.Request(
                self.BEACONDB_URL,
                data=body,
                headers={
                    "Content-Type": "application/json",
                    "User-Agent": "kpatrol-pi/5.13",
                },
                method="POST",
            )
            with _ur.urlopen(req, timeout=self.HTTP_TIMEOUT_S) as resp:
                raw = resp.read().decode("utf-8", errors="ignore")
            data = json.loads(raw)
            loc = data.get("location") or {}
            if "lat" not in loc or "lng" not in loc:
                return None
            entry = {
                "latitude": float(loc["lat"]),
                "longitude": float(loc["lng"]),
                "accuracy_m": float(data.get("accuracy", 0) or 0),
                "city": "",      # BeaconDB doesn't reverse-geocode
                "country": "",
                "country_code": "",
                "timezone": "",
                "public_ip": "",
                "source_tier": "wifi",
                "ap_count": len(aps),
                "fetched_at_ms": int(time.time() * 1000),
            }
            logger.info(
                f"[NetGPS] BeaconDB fix ({entry['latitude']:.6f}, "
                f"{entry['longitude']:.6f}) ±{entry['accuracy_m']:.0f}m "
                f"from {len(aps)} APs"
            )
            return entry
        except Exception as exc:
            logger.warning(f"[NetGPS] BeaconDB lookup failed: {exc}")
            return None

    # ─── Resolver tier 3: IP geolocation ───────────────────────────────
    def _resolve_ipapi(self) -> Optional[Dict[str, Any]]:
        try:
            import urllib.request as _ur
            req = _ur.Request(self.IPAPI_URL, headers={"User-Agent": "kpatrol-pi/5.13"})
            with _ur.urlopen(req, timeout=self.HTTP_TIMEOUT_S) as resp:
                raw = resp.read().decode("utf-8", errors="ignore")
            data = json.loads(raw)
            if data.get("status") != "success":
                logger.warning(f"[NetGPS] ip-api error: {data!r}")
                return None
            entry = {
                "latitude": float(data["lat"]),
                "longitude": float(data["lon"]),
                "accuracy_m": 5000.0,   # ISP-exit point: city-block, not meters
                "city": data.get("city", ""),
                "country": data.get("country", ""),
                "country_code": data.get("countryCode", ""),
                "timezone": data.get("timezone", ""),
                "public_ip": data.get("query", ""),
                "source_tier": "ip",
                "ap_count": 0,
                "fetched_at_ms": int(time.time() * 1000),
            }
            logger.info(
                f"[NetGPS] IP fix {entry['city']}, {entry['country']} "
                f"({entry['latitude']:.4f}, {entry['longitude']:.4f}) "
                f"via {entry['public_ip']}"
            )
            return entry
        except Exception as exc:
            logger.warning(f"[NetGPS] ip-api fetch failed: {exc}")
            return None

    # ─── Resolver entry point ──────────────────────────────────────────
    def _fetch_once(self) -> bool:
        # Tier 1: manual override (no network round-trip needed). We
        # still refresh the cache so the timestamp stays fresh.
        if self._override is not None:
            entry = {
                "latitude": self._override[0],
                "longitude": self._override[1],
                "accuracy_m": 1.0,        # operator pinned it — assume sub-meter
                "city": "",
                "country": "",
                "country_code": "",
                "timezone": "",
                "public_ip": "",
                "source_tier": "manual",
                "ap_count": 0,
                "fetched_at_ms": int(time.time() * 1000),
            }
            with self._lock:
                self._cache = entry
            return True

        # Tier 2: BeaconDB WiFi triangulation.
        entry = self._resolve_beacondb()
        # Tier 3: IP geolocation as last resort.
        if entry is None:
            entry = self._resolve_ipapi()
        if entry is None:
            return False
        with self._lock:
            self._cache = entry
        return True

    def get_fix(self) -> Optional[Dict[str, Any]]:
        with self._lock:
            return dict(self._cache) if self._cache else None

# V5.10: front-right VL53L0X back online after the hardware replacement —
# 2026-05-23 raw serial showed valid distance readings on lane 2 (`TOF:.,.,
# <val>,...`). Mask is now all-bits-set so every lane the firmware emits is
# honoured. Keep the constants in place so re-disabling a faulty lane in
# the future is a one-line change rather than re-introducing the dead-code
# path through `_parse_tof_line`.
DISABLED_TOF_BIT_MASK = 0b111111  # 0 disabled lanes — all six honoured
DISABLED_TOF_LANES: tuple = ()


# ==================== SAFETY (Directional) ====================

class SafetyZone(Enum):
    SAFE = "safe"
    SLOW = "slow"
    CAUTION = "caution"
    DANGER = "danger"

@dataclass
class SafetyConfig:
    # Always-on by default. Only togglable via MQTT safety_config topic.
    # Thresholds tuned for max sensitivity: danger=350mm covers ~250mm worst-case
    # latency travel (200ms loop at 1m/s) + ~100mm BTS7960 active-brake distance.
    enabled: bool = True
    danger_distance: int = 350     # hard stop — covers latency + brake distance
    caution_distance: int = 550    # heavy slowdown — early brake margin
    slow_distance: int = 750       # gentle slowdown — earliest reaction
    speed_reduction_slow: float = 0.7
    speed_reduction_caution: float = 0.4

# Which movement directions each motor command requires to be clear
CMD_REQUIRED_DIRECTIONS: Dict[str, List[str]] = {
    'F':  ['forward'],
    'B':  ['backward'],
    'SL': ['left'],
    'SR': ['right'],
    'L':  ['forward'],
    'R':  ['forward'],
    'DL': ['forward', 'left'],
    'DR': ['forward', 'right'],
    'BL': ['backward', 'left'],
    'BR': ['backward', 'right'],
    'S':  [],
}


@dataclass
class ToFData:
    front: int = 0
    front_left: int = 0
    front_right: int = 0
    left: int = 0
    right: int = 0
    back: int = 0
    timestamp: int = 0
    # 6-bit mask from firmware v3.1+ telling us which lanes had a fresh,
    # status==0 reading on this frame. 0x3F = all six lanes fresh; missing
    # bit means firmware (or the I2C mux) couldn't read that sensor and the
    # corresponding distance is the 9999 sentinel rather than a real value.
    # Older firmware that doesn't emit field 7 keeps the default 0x3F so
    # consumers see the legacy "trust all distances" behaviour.
    valid_mask: int = 0x3F

    # ESP firmware contract:
    #   0     = pre-init / never written        → fail-safe (treated separately)
    #   1     = "min-range fail" sentinel       → object is RIGHT in front of the
    #                                             lens; preserve as DANGER trigger
    #   9999  = out of range / sensor failure   → treat as "far"
    # Real measurements arrive as median-of-3 distances in mm.
    VALID_MAX_MM: int = 2000  # >2m considered "no obstacle" for safety purposes
    # VL53L0X minimum reliable distance. Readings below this — except the
    # explicit min-range sentinel v==1 — are treated as noise. With a
    # partially-wired TCA9548A this prevents a single noisy lane (e.g.
    # front_left=3mm while every other sensor reports 9999) from forcing
    # the whole robot into DANGER and refusing manual control.
    MIN_VALID_MM: int = 30

    # bit index in firmware valid_mask, matching the field order in
    # TOF:front,front_left,front_right,left,right,back,valid_mask
    _MASK_BITS: ClassVar[Dict[str, int]] = {
        "front": 0, "front_left": 1, "front_right": 2,
        "left": 3, "right": 4, "back": 5,
    }

    def to_dict(self) -> Dict:
        # Publish sanitized lane values (9999 for invalid) so downstream
        # consumers (PWA radar, dashboards) don't see raw 1mm garbage when
        # the firmware spams uninitialized registers. The bitmask is also
        # rebuilt so a lane below MIN_VALID_MM has its valid bit cleared.
        sanitized: Dict[str, int] = {}
        rebuilt_mask = self.valid_mask
        for lane, raw in (
            ("front", self.front), ("front_left", self.front_left),
            ("front_right", self.front_right), ("left", self.left),
            ("right", self.right), ("back", self.back),
        ):
            bit = self._MASK_BITS[lane]
            clean = self._valid_or_far(raw, bit)
            sanitized[lane] = clean
            if clean == 9999:
                rebuilt_mask &= ~(1 << bit)
        sanitized["valid_mask"] = rebuilt_mask
        sanitized["timestamp"] = self.timestamp
        return sanitized

    def _valid_or_far(self, v: int, mask_bit: int = -1) -> int:
        """Return v if it's a real measurement; else 9999 (far).

        Honours the firmware valid_mask: if the bit for this lane is clear,
        the reading is rejected regardless of value (sensor on that mux
        channel never reported status==0). Anything below MIN_VALID_MM —
        including the legacy v==1 "min-range fail" sentinel — is treated
        as VL53L0X/firmware garbage rather than a real obstacle, because
        observed Pi traffic shows multiple lanes simultaneously latched on
        v==1 from uninitialized/stale registers (physically impossible).
        """
        if mask_bit >= 0 and not (self.valid_mask & (1 << mask_bit)):
            return 9999
        if v <= 0 or v >= 9999:
            return 9999
        if v < self.MIN_VALID_MM:
            return 9999
        if v > self.VALID_MAX_MM:
            return 9999
        return v

    def get_min_distance(self) -> int:
        # Pre-init guard: if no data has ever been written, force DANGER (0)
        # so the safety controller refuses motion until the ESP confirms a reading.
        if self.timestamp == 0:
            return 0
        vals = [
            self._valid_or_far(self.front,        self._MASK_BITS["front"]),
            self._valid_or_far(self.front_left,   self._MASK_BITS["front_left"]),
            self._valid_or_far(self.front_right,  self._MASK_BITS["front_right"]),
            self._valid_or_far(self.left,         self._MASK_BITS["left"]),
            self._valid_or_far(self.right,        self._MASK_BITS["right"]),
            self._valid_or_far(self.back,         self._MASK_BITS["back"]),
        ]
        return min(vals)

    def get_direction_distances(self) -> Dict[str, int]:
        # Pre-init: every direction is DANGER (0).
        if self.timestamp == 0:
            return {"forward": 0, "backward": 0, "left": 0, "right": 0}
        f  = self._valid_or_far(self.front,       self._MASK_BITS["front"])
        fl = self._valid_or_far(self.front_left,  self._MASK_BITS["front_left"])
        fr = self._valid_or_far(self.front_right, self._MASK_BITS["front_right"])
        l  = self._valid_or_far(self.left,        self._MASK_BITS["left"])
        r  = self._valid_or_far(self.right,       self._MASK_BITS["right"])
        b  = self._valid_or_far(self.back,        self._MASK_BITS["back"])
        return {
            "forward":  min(f, fl, fr),
            "backward": b,
            "left":     min(l, fl),
            "right":    min(r, fr),
        }

    def to_float_dict(self) -> Dict[str, float]:
        """Convert to float dict for navigation use.

        Returns sanitized distances (9999 for any lane flagged invalid by
        firmware mask, sub-MIN_VALID_MM noise, or pre-init zeros), so
        navigation map-builders never see physically impossible 1mm readings
        that would force endless DANGER stops.

        `timestamp` (ms since epoch) is included so consumers like FreeCoverage
        can detect stale ToF samples and throttle forward speed when the I2C
        mux read loop falls behind. Navigation map-builders ignore unknown
        keys.
        """
        return {
            "front":       float(self._valid_or_far(self.front,        self._MASK_BITS["front"])),
            "front_left":  float(self._valid_or_far(self.front_left,   self._MASK_BITS["front_left"])),
            "front_right": float(self._valid_or_far(self.front_right,  self._MASK_BITS["front_right"])),
            "left":        float(self._valid_or_far(self.left,         self._MASK_BITS["left"])),
            "right":       float(self._valid_or_far(self.right,        self._MASK_BITS["right"])),
            "back":        float(self._valid_or_far(self.back,         self._MASK_BITS["back"])),
            "timestamp":   float(self.timestamp),
        }


@dataclass
class IMUData:
    yaw: float = 0.0
    pitch: float = 0.0
    roll: float = 0.0
    accuracy: float = 0.0
    timestamp: int = 0

    def to_dict(self) -> Dict:
        return {"yaw": self.yaw, "pitch": self.pitch,
                "roll": self.roll, "accuracy": self.accuracy,
                "timestamp": self.timestamp}


@dataclass
class EncoderData:
    motor: str = ""
    count: int = 0
    revolutions: float = 0.0
    rpm: float = 0.0


@dataclass
class MotorData:
    position: str = ""
    speed: int = 0
    direction: str = "stopped"
    inverted: bool = False


class SafetyController:
    """Directional Safety Controller — per-direction zone evaluation.

    Fail-safe defaults: pre-init zones are DANGER, not SAFE — robot will not
    move until at least one valid ToF frame is received from the ESP. If the
    ESP stops sending frames (USB drop, firmware crash) for longer than
    `STALE_TOF_THRESHOLD_MS`, the controller reverts to DANGER on every
    direction so the watchdog cuts motion just as if obstacles appeared.
    """

    # V5.14: relaxed from 700 ms → 3000 ms. The encoder Dev firmware
    # occasionally drops to ~1 Hz emit rate when the TCA9548A I²C mux
    # is mid-recovery (tca_fail count climbing) — at the old threshold
    # the safety gate would briefly fail-closed and block every command
    # until the bus settled, even though the underlying ToF lanes were
    # still trustworthy from the previous read. 3 s tolerates two missed
    # frames while still latching DANGER when the firmware truly hangs.
    STALE_TOF_THRESHOLD_MS: int = 3000

    def __init__(self, config: SafetyConfig = None):
        self.config = config or SafetyConfig()
        self.tof_data = ToFData()
        # Default pre-init zone is DANGER so motion is refused until a valid
        # frame arrives. The first update_tof() flips this to real values.
        self.current_zone = SafetyZone.DANGER
        self.direction_zones: Dict[str, SafetyZone] = {
            "forward": SafetyZone.DANGER, "backward": SafetyZone.DANGER,
            "left": SafetyZone.DANGER, "right": SafetyZone.DANGER,
        }
        self.direction_distances: Dict[str, int] = {
            "forward": 0, "backward": 0, "left": 0, "right": 0,
        }
        self.lock = threading.Lock()
        # Optional callable returning True when the firmware link is healthy.
        # Wired by KPatrolMQTTV5 to EncoderReader.is_heartbeat_stale so that
        # `is_command_safe` can fail closed if ENC_HB stops arriving even
        # while a stale ToF cache might still look "fresh enough".
        self._link_alive_check: Optional[Callable[[], bool]] = None

    def set_link_alive_check(self, check: Callable[[], bool]) -> None:
        self._link_alive_check = check

    def _is_link_dead(self) -> bool:
        if self._link_alive_check is None:
            return False
        try:
            return not self._link_alive_check()
        except Exception:
            # Fail closed: any exception in the probe = link unsafe.
            return True

    def update_tof(self, front: int, front_left: int, front_right: int,
                   left: int, right: int, back: int,
                   valid_mask: int = 0x3F):
        with self.lock:
            self.tof_data.front = front
            self.tof_data.front_left = front_left
            self.tof_data.front_right = front_right
            self.tof_data.left = left
            self.tof_data.right = right
            self.tof_data.back = back
            self.tof_data.valid_mask = valid_mask
            self.tof_data.timestamp = int(time.time() * 1000)

            self.direction_distances = self.tof_data.get_direction_distances()
            for direction, dist in self.direction_distances.items():
                self.direction_zones[direction] = self._compute_zone(dist)
            self.current_zone = self._compute_zone(self.tof_data.get_min_distance())

    def _compute_zone(self, distance: int) -> SafetyZone:
        if distance < self.config.danger_distance:
            return SafetyZone.DANGER
        elif distance < self.config.caution_distance:
            return SafetyZone.CAUTION
        elif distance < self.config.slow_distance:
            return SafetyZone.SLOW
        return SafetyZone.SAFE

    def _is_data_stale(self) -> bool:
        """True if no ToF frame received within STALE_TOF_THRESHOLD_MS.

        Treat stale data as DANGER so the safety net fails closed when the
        ESP link drops mid-run (USB unplug, crash, firmware reset).
        """
        if self.tof_data.timestamp == 0:
            return True
        age_ms = int(time.time() * 1000) - self.tof_data.timestamp
        return age_ms > self.STALE_TOF_THRESHOLD_MS

    def is_command_safe(self, cmd: str) -> bool:
        if not self.config.enabled:
            return True
        # Fail-safe: stale ToF OR dead firmware link → reject motion.
        if self._is_data_stale() or self._is_link_dead():
            return False
        cmd_upper = cmd.strip().upper()
        for direction in CMD_REQUIRED_DIRECTIONS.get(cmd_upper, []):
            if self.direction_zones.get(direction) == SafetyZone.DANGER:
                return False
        return True

    def get_command_speed_multiplier(self, cmd: str) -> float:
        if not self.config.enabled:
            return 1.0
        if self._is_data_stale() or self._is_link_dead():
            return 0.0  # Stale data or dead link → block motion
        cmd_upper = cmd.strip().upper()
        required = CMD_REQUIRED_DIRECTIONS.get(cmd_upper, [])
        if not required:
            return 1.0
        return min(self._zone_multiplier(self.direction_zones.get(d, SafetyZone.SAFE)) for d in required)

    def get_safe_alternatives(self, blocked_cmd: str) -> List[str]:
        return [c for c in ['F', 'B', 'SL', 'SR', 'L', 'R']
                if c != blocked_cmd and self.is_command_safe(c)]

    def _zone_multiplier(self, zone: SafetyZone) -> float:
        if zone == SafetyZone.DANGER: return 0.0
        if zone == SafetyZone.CAUTION: return self.config.speed_reduction_caution
        if zone == SafetyZone.SLOW: return self.config.speed_reduction_slow
        return 1.0

    def get_speed_multiplier(self) -> float:
        if not self.config.enabled:
            return 1.0
        if self._is_data_stale():
            return 0.0
        return self._zone_multiplier(self.current_zone)

    def should_stop(self) -> bool:
        if not self.config.enabled:
            return False
        if self._is_data_stale():
            return True  # Fail-safe stop on link loss
        return self.current_zone == SafetyZone.DANGER

    def get_status(self) -> Dict:
        with self.lock:
            stale = self._is_data_stale()
            age_ms = (int(time.time() * 1000) - self.tof_data.timestamp
                      if self.tof_data.timestamp > 0 else -1)
            return {
                "enabled": self.config.enabled,
                "zone": self.current_zone.value if not stale else SafetyZone.DANGER.value,
                "speed_multiplier": self.get_speed_multiplier(),
                "min_distance": self.tof_data.get_min_distance(),
                "tof_stale": stale,
                "tof_age_ms": age_ms,
                "tof": self.tof_data.to_dict(),
                "thresholds": {
                    "danger": self.config.danger_distance,
                    "caution": self.config.caution_distance,
                    "slow": self.config.slow_distance,
                },
                "directions": {
                    d: {
                        "zone": self.direction_zones[d].value,
                        "distance": self.direction_distances[d],
                        "blocked": self.direction_zones[d] == SafetyZone.DANGER,
                    }
                    for d in ["forward", "backward", "left", "right"]
                },
                "timestamp": int(time.time() * 1000),
            }

    def set_enabled(self, enabled: bool):
        self.config.enabled = enabled
        logger.info(f"[Safety] {'Enabled' if enabled else 'Disabled'}")


# ==================== SERIAL MANAGERS (from V3) ====================

class MotorController:
    """Manages ESP32-S3 Motor Controller on /dev/ttyACM0"""

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        # V5.7: split the read/write critical sections so a main-loop
        # `drain_responses()` cannot block an inbound MQTT motor command.
        # Linux tty drivers handle concurrent read+write on the same fd
        # natively (full-duplex), so two locks are safe — they each only
        # need to serialise their own direction. Before this split a single
        # drain pass at 32 lines could hold the lock 20-100 ms, which is
        # why joystick frames felt sluggish while D1 R32 / peripheral hub
        # (separate UART, separate locks) felt instant.
        self.write_lock = threading.Lock()
        self.read_lock = threading.Lock()
        self.connected = False
        self.current_speed = 150
        self.light_state = False
        self.main_light_state = False
        self.motor_states: Dict[str, MotorData] = {
            pos: MotorData(position=pos, inverted=MOTOR_INVERTED[pos])
            for pos in MOTOR_POSITIONS
        }
        self._last_reconnect_attempt = 0
        self._reconnect_interval = 3
        self.imu_data = IMUData()
        # Dedicated IMU lock so consumers can snapshot yaw/pitch/roll without
        # contending with the serial-I/O critical sections (write_lock/read_lock).
        # Writes land in _parse_imu_line; reads must go through get_imu_snapshot().
        self._imu_lock = threading.Lock()
        # V5.2: optional sink for NMEA frames forwarded by ESP32 firmware
        # (lines prefixed with "NMEA:" arrive on the motor UART0).
        self._gps_sink = None
        # V5.4: latest battery percentage reported by firmware (BAT:<pct>).
        # None means firmware hasn't sent a reading yet — telemetry uses a
        # placeholder until the first valid sample arrives.
        # V5.5: also stash voltage in mV (firmware format `BAT:<pct>,<pack_mv>`)
        # so the alarm/socket payload can show LFP volts (more reliable than %
        # on the flat plateau of the discharge curve).
        self.battery_pct: Optional[float] = None
        self.battery_voltage_mv: Optional[int] = None
        self.battery_ts: int = 0

    def set_gps_sink(self, sink) -> None:
        """Register a callback (e.g. GPSReader.feed_nmea) to receive NMEA
        sentences forwarded by the motor firmware via shared-mode pass-through."""
        self._gps_sink = sink

    def _rediscover_port(self) -> str:
        """Pick the best motor port available right now. Re-evaluated on
        every (re)connect so the bridge survives the case where the
        ESP32-S3 brown-outs and re-enumerates *after* the service started.

        Order: udev symlink → content sniff (looks for ``IMU:`` / ``BAT:`` /
        ``GPS_HB:`` signatures on each free ttyACM*/ttyUSB*) → keep the old
        guess so the next attempt still has somewhere to try.
        """
        if os.path.exists("/dev/ttyKPATROL_MOTOR"):
            return "/dev/ttyKPATROL_MOTOR"
        for cand in sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")):
            if _classify_serial_port(cand) == "motor":
                return cand
        return self.port

    def connect(self) -> bool:
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            new_port = self._rediscover_port()
            if new_port != self.port:
                logger.info(f"[Motor] Port changed: {self.port} -> {new_port}")
                self.port = new_port
            # V5.10: open with DTR / RTS held LOW so we don't bounce-reset the
            # ESP32-S3. The K-Patrol motor board uses a WCH CH9102 USB-Serial
            # bridge whose DTR line is wired straight to the S3's EN pin —
            # pyserial's default `dtr=None` lets the kernel default kick in
            # (HIGH on Linux), which pulses on every Serial() open and resets
            # the firmware. Setting them False on the unopened serial object
            # makes the kernel ioctl set the lines BEFORE the first byte
            # crosses the wire, so the S3 keeps running uninterrupted across
            # Pi service restarts. timeout=0.1: short read deadline so
            # readline() cannot stall the main 20 Hz loop when a partial
            # frame is in the buffer.
            ser = serial.Serial()
            ser.port = self.port
            ser.baudrate = self.baudrate
            ser.timeout = 0.1
            ser.dtr = False
            ser.rts = False
            ser.open()
            self.serial = ser
            self.connected = True
            logger.info(f"[Motor] Connected to {self.port} (DTR/RTS held low)")
            # 0.5 s is plenty for the firmware's own boot drain; we used to
            # wait 2 s only because every open USED to reset the chip.
            time.sleep(0.5)
            return True
        except serial.SerialException as e:
            logger.error(f"[Motor] Connection failed: {e}")
            self.connected = False
            return False

    def _try_reconnect(self) -> bool:
        if time.time() - self._last_reconnect_attempt < self._reconnect_interval:
            return False
        self._last_reconnect_attempt = time.time()
        logger.info(f"[Motor] Attempting reconnection to {self.port}...")
        return self.connect()

    def disconnect(self):
        if self.serial and self.serial.is_open:
            self.send_command("S")
            self.serial.close()
        self.connected = False

    def send_command(self, command: str) -> bool:
        if not self.connected or not self.serial:
            self._try_reconnect()
            if not self.connected:
                return False
        # write_lock only — read paths use read_lock so a long drain pass
        # can't queue an inbound motor command.
        with self.write_lock:
            try:
                self.serial.write(f"{command}\n".encode())
                self.serial.flush()
                self._update_motor_states(command)
                return True
            except (OSError, serial.SerialException) as e:
                logger.error(f"[Motor] Send error: {e}")
                self.connected = False
                return False

    def set_speed(self, speed: int) -> bool:
        self.current_speed = max(0, min(255, speed))
        return self.send_command(f"SPD:{self.current_speed}")

    def _update_motor_states(self, command: str):
        movement_map = {
            'F': {'FR': 'forward', 'FL': 'forward', 'BR': 'forward', 'BL': 'forward'},
            'B': {'FR': 'backward', 'FL': 'backward', 'BR': 'backward', 'BL': 'backward'},
            'SR': {'FR': 'backward', 'FL': 'forward', 'BR': 'forward', 'BL': 'backward'},
            'SL': {'FR': 'forward', 'FL': 'backward', 'BR': 'backward', 'BL': 'forward'},
            'R': {'FR': 'backward', 'FL': 'forward', 'BR': 'backward', 'BL': 'forward'},
            'L': {'FR': 'forward', 'FL': 'backward', 'BR': 'forward', 'BL': 'backward'},
            'DR': {'FR': 'stopped', 'FL': 'forward', 'BR': 'forward', 'BL': 'stopped'},
            'DL': {'FR': 'forward', 'FL': 'stopped', 'BR': 'stopped', 'BL': 'forward'},
            'BR': {'FR': 'backward', 'FL': 'stopped', 'BR': 'stopped', 'BL': 'backward'},
            'BL': {'FR': 'stopped', 'FL': 'backward', 'BR': 'backward', 'BL': 'stopped'},
            'S': {'FR': 'stopped', 'FL': 'stopped', 'BR': 'stopped', 'BL': 'stopped'},
        }
        cmd_base = command.split()[0] if ' ' in command else command
        if cmd_base in movement_map:
            for pos, direction in movement_map[cmd_base].items():
                self.motor_states[pos].direction = direction
                self.motor_states[pos].speed = self.current_speed if direction != 'stopped' else 0

    def get_motor_status(self) -> Dict:
        return {pos: asdict(data) for pos, data in self.motor_states.items()}

    def read_response(self) -> Optional[str]:
        if not self.connected or not self.serial:
            self._try_reconnect()
            return None
        with self.read_lock:
            try:
                if self.serial.in_waiting:
                    line = self.serial.readline().decode(errors="ignore").strip()
                    # V5.2 shared-mode GPS: ESP32 forwards each NMEA frame
                    # prefixed with "NMEA:". Route these to the GPS parser
                    # and skip IMU/normal handling.
                    if line.startswith("NMEA:") and self._gps_sink is not None:
                        try:
                            self._gps_sink(line[5:])
                        except Exception as e:
                            logger.error(f"[Motor] GPS sink error: {e}")
                        return None
                    self._parse_imu_line(line)
                    self._parse_battery_line(line)
                    return line
            except (OSError, serial.SerialException) as e:
                logger.error(f"[Motor] Read error: {e}")
                self.connected = False
        return None

    def drain_responses(self, max_lines: int = 32) -> Optional[str]:
        """Read up to ``max_lines`` frames from the motor UART in one call.

        The main loop only ticks at 20Hz, but the ESP32 streams IMU/BAT/NMEA
        frames faster than that. Reading a single line per tick lets the
        kernel buffer back up and starves downstream consumers (telemetry,
        GPS). This drains everything currently buffered and returns the last
        non-IMU/NMEA line for log forwarding.
        """
        last_log_line: Optional[str] = None
        if not self.connected or not self.serial:
            self._try_reconnect()
            return None
        for _ in range(max_lines):
            with self.read_lock:
                if not self.serial or not self.serial.in_waiting:
                    break
                try:
                    line = self.serial.readline().decode(errors="ignore").strip()
                except (OSError, serial.SerialException) as e:
                    logger.error(f"[Motor] Read error: {e}")
                    self.connected = False
                    break
            if not line:
                continue
            if line.startswith("NMEA:") and self._gps_sink is not None:
                try:
                    self._gps_sink(line[5:])
                except Exception as e:
                    logger.error(f"[Motor] GPS sink error: {e}")
                continue
            self._parse_imu_line(line)
            self._parse_battery_line(line)
            if not line.startswith("IMU:") and not line.startswith("BAT:"):
                last_log_line = line
        return last_log_line

    def _parse_imu_line(self, line: str):
        if not line.startswith("IMU:"):
            return
        # V5.10: defensive parse. Earlier code only set locals inside the
        # `if len(parts) >= 4` branch but committed them unconditionally
        # in a later `with` block — a partial frame (USB CDC stalled
        # mid-line) would slip past the length check and crash the read
        # thread with UnboundLocalError, taking down the whole service.
        # Now: bail out of the function on any short/malformed payload.
        try:
            parts = line[4:].split(',')
            if len(parts) < 4:
                return
            yaw = float(parts[0])
            pitch = float(parts[1])
            roll = float(parts[2])
            accuracy = float(parts[3])
            ts = int(time.time() * 1000)
        except (ValueError, IndexError):
            return
        with self._imu_lock:
            self.imu_data.yaw = yaw
            self.imu_data.pitch = pitch
            self.imu_data.roll = roll
            self.imu_data.accuracy = accuracy
            self.imu_data.timestamp = ts

    def get_imu_snapshot(self) -> IMUData:
        """Return a consistent copy of the latest IMU sample.

        Callers from other threads MUST use this rather than touching
        ``self.imu_data.*`` directly, otherwise they can read a torn frame
        (yaw from sample N, pitch from sample N+1).
        """
        with self._imu_lock:
            return IMUData(
                yaw=self.imu_data.yaw,
                pitch=self.imu_data.pitch,
                roll=self.imu_data.roll,
                accuracy=self.imu_data.accuracy,
                timestamp=self.imu_data.timestamp,
            )

    def get_imu_status(self) -> Dict:
        return self.get_imu_snapshot().to_dict()

    def request_imu(self):
        self.send_command("IMU")

    def _parse_battery_line(self, line: str) -> None:
        """Parse `BAT:<pct>,<pack_mv>` telemetry from motor firmware.

        Older firmware sent only `BAT:<pct>`; we still accept that form. The
        voltage half is optional but lets the UI show LFP volts directly
        (more useful than % near the flat plateau of the LiFePO4 curve).
        """
        if not line.startswith("BAT:"):
            return
        parts = line[4:].split(",")
        try:
            pct = float(parts[0])
        except (ValueError, IndexError):
            return
        # Drop negative / pct>100 — uncalibrated ADC or no battery plugged.
        # We allow exactly 0.0% so the last warning before firmware cutoff is
        # still reported (LFP 4S below 10.0V is real, not a sensor glitch).
        # Older guard `pct <= 0.0` dropped this; that was unsafe on a real
        # near-empty pack.
        if pct < 0.0 or pct > 100.0:
            return
        self.battery_pct = pct
        if len(parts) >= 2:
            try:
                mv = int(parts[1])
                # LFP 4S sane envelope: 8.0V (deep cutoff w/ sag) .. 15.0V
                # (charger peak). Anything outside is sensor error — drop
                # rather than poison the UI with bogus volts.
                if 8000 <= mv <= 15000:
                    self.battery_voltage_mv = mv
            except ValueError:
                pass
        self.battery_ts = int(time.time() * 1000)

    def get_battery_pct(self) -> Optional[float]:
        return self.battery_pct

    def get_battery_voltage_mv(self) -> Optional[int]:
        return self.battery_voltage_mv


class EncoderReader:
    """Manages ESP32 Dev Encoder + ToF Reader on /dev/ttyUSB0"""

    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.serial: Optional[serial.Serial] = None
        self.lock = threading.Lock()
        self.connected = False
        self.encoder_data: Dict[str, EncoderData] = {
            pos: EncoderData(motor=pos) for pos in MOTOR_POSITIONS
        }
        self.read_thread: Optional[threading.Thread] = None
        self.running = False
        self._last_reconnect_attempt = 0
        self._reconnect_interval = 3
        self.tof_callback = None

        # Firmware v3.1+ heartbeat: ENC_HB:<uptime_ms>,<tca_fail_count>
        # arrives ~1 Hz. Used to detect ESP hang / USB-stall — independent
        # of ToF freshness because ENC_HB fires even if all 6 ToFs failed.
        self.last_heartbeat_ms: float = 0.0  # local monotonic, 0 == never seen
        self.heartbeat_uptime_ms: int = 0    # firmware-side uptime
        self.tca_fail_count: int = 0
        self._last_tca_fail_warned: int = 0  # avoid spamming logs

    # V5.14: relaxed from 3 → 8 s. Encoder Dev's main loop occasionally
    # stalls 2–4 s during VL53L0X cluster recovery on a stuck SDA, which
    # would otherwise trip `_is_link_dead` and force-close the safety
    # gate. 8 s still catches a genuine firmware hang within one screen
    # refresh while tolerating the worst-case I²C recovery latency.
    HEARTBEAT_STALE_SEC = 8.0

    def is_heartbeat_stale(self) -> bool:
        """True if firmware's 1-Hz heartbeat hasn't arrived in ~3 s.

        Returns False during the boot window (before any heartbeat) so a
        slow ESP boot doesn't get flagged as a hang.
        """
        if self.last_heartbeat_ms == 0.0:
            return False
        return (time.monotonic() - self.last_heartbeat_ms) > self.HEARTBEAT_STALE_SEC

    def set_tof_callback(self, callback):
        self.tof_callback = callback

    def _rediscover_port(self) -> str:
        """Re-evaluate encoder port at every (re)connect. udev symlink first,
        then content sniff (``ENC_HB:`` / ``ENC:`` / ``TOF:``) — that way the
        encoder is found even if the user swaps the encoder and the peripheral
        hub between USB-hub slots."""
        if os.path.exists("/dev/ttyKPATROL_ENCODER"):
            return "/dev/ttyKPATROL_ENCODER"
        for cand in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
            if _classify_serial_port(cand) == "encoder":
                return cand
        return self.port

    def connect(self) -> bool:
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            new_port = self._rediscover_port()
            if new_port != self.port:
                logger.info(f"[Encoder] Port changed: {self.port} -> {new_port}")
                self.port = new_port
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1.0)
            self.connected = True
            logger.info(f"[Encoder] Connected to {self.port}")
            time.sleep(2)
            # ESP32 Dev (CH340) does NOT reset on DTR toggle, so the firmware
            # may still be stuck in JSON mode from a previous session. Probe
            # and cycle output mode (`m`) until it reports MODE_COMPACT.
            self._ensure_compact_mode()
            if not self.running:
                self.running = True
                self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
                self.read_thread.start()
            return True
        except serial.SerialException as e:
            logger.error(f"[Encoder] Connection failed: {e}")
            self.connected = False
            return False

    def _ensure_compact_mode(self, max_cycles: int = 4) -> None:
        """Send `m` (cycle output mode) until firmware reports MODE_COMPACT.

        Default firmware boot is MODE_COMPACT, but the ESP32 Dev (CH340 USB)
        does not reset on DTR toggle — if a previous session toggled it to
        JSON or HUMAN, the chip stays there across `serial.Serial()` opens.
        We send `m` (cycle Human->Compact->JSON->Human), read the OK echo,
        and stop once we see `OK:MODE_COMPACT`. Bounded so a dead chip
        cannot hang the connect path.
        """
        if not self.serial:
            return
        try:
            self.serial.reset_input_buffer()
        except Exception:
            return
        for _ in range(max_cycles):
            try:
                self.serial.write(b'm\n')
                self.serial.flush()
            except (OSError, serial.SerialException):
                return
            deadline = time.monotonic() + 0.5
            while time.monotonic() < deadline:
                try:
                    raw = self.serial.readline()
                except (OSError, serial.SerialException):
                    return
                if not raw:
                    continue
                resp = raw.decode(errors="ignore").strip()
                if resp.startswith("OK:MODE_"):
                    if resp == "OK:MODE_COMPACT":
                        logger.info("[Encoder] Output mode locked to COMPACT")
                        return
                    break  # got a different mode echo; cycle again
        logger.warning("[Encoder] Could not confirm COMPACT mode after %d cycles", max_cycles)

    def _try_reconnect(self) -> bool:
        if time.time() - self._last_reconnect_attempt < self._reconnect_interval:
            return False
        self._last_reconnect_attempt = time.time()
        return self.connect()

    def disconnect(self):
        self.running = False
        if self.read_thread:
            self.read_thread.join(timeout=2)
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.connected = False

    def reset_counters(self) -> bool:
        if not self.connected or not self.serial:
            self._try_reconnect()
            if not self.connected:
                return False
        with self.lock:
            try:
                self.serial.write(b'R\n')
                self.serial.flush()
                return True
            except (OSError, serial.SerialException):
                self.connected = False
                return False

    def _read_loop(self):
        # V5.14: periodic mode re-lock. If the encoder firmware boots into
        # JSON / HUMAN (or some external monitor toggled it), the read loop
        # detects the wrong leading characters and queues a re-lock at most
        # once every 10 s. COMPACT lines start with `ENC:` / `TOF:` /
        # `ENC_HB:` / `STATUS:` / `OK:` / `ERR:` — anything else (e.g. `{`
        # for JSON, `===` banner for HUMAN) is a wrong-mode signal.
        last_relock_attempt = 0.0
        recent_wrong_mode = 0  # count consecutive wrong-mode lines
        while self.running:
            if not self.connected:
                self._try_reconnect()
                time.sleep(1)
                continue
            try:
                if self.serial and self.serial.in_waiting:
                    with self.lock:
                        line = self.serial.readline().decode().strip()
                        self._parse_line(line)
                    if line:
                        is_compact = (
                            line.startswith(("ENC:", "TOF:", "ENC_HB:",
                                              "STATUS:", "OK:", "ERR:", "MIN:")) )
                        if is_compact:
                            recent_wrong_mode = 0
                        else:
                            recent_wrong_mode += 1
                            now = time.monotonic()
                            if (recent_wrong_mode >= 3
                                    and now - last_relock_attempt > 10.0):
                                last_relock_attempt = now
                                logger.info(
                                    "[Encoder] Detected non-COMPACT output "
                                    f"({line[:24]!r}…) — re-locking mode"
                                )
                                self._ensure_compact_mode(max_cycles=6)
                                recent_wrong_mode = 0
            except (OSError, serial.SerialException) as e:
                if self.running:
                    logger.error(f"[Encoder] Read error: {e}")
                    self.connected = False
            time.sleep(0.05)

    def _parse_line(self, line: str):
        if not line:
            return
        # Firmware v3.0+ compact: ENC:fr,fl,br,bl,rpm_fr,rpm_fl,rpm_br,rpm_bl
        if line.startswith("ENC:"):
            self._parse_enc_compact_line(line)
            return
        if line.startswith("TOF:"):
            self._parse_tof_line(line)
            return
        if line.startswith("ENC_HB:"):
            self._parse_heartbeat_line(line)
            return
        # Firmware JSON mode (sticky if Pi previously toggled `j`). Parsing
        # the same payload here means the watchdog still sees ENC_HB / TOF,
        # the only difference is the line that carries the per-wheel data.
        if line.startswith("{"):
            self._parse_json_line(line)
            return
        # Legacy per-motor text: `<FR>: Count=N Rev=N.N RPM=N.N`
        match = re.match(
            r'(\w+):\s*Count=(-?\d+)\s+Rev=(-?[\d.]+)\s+RPM=(-?[\d.]+)', line)
        if match:
            motor = match.group(1)
            if motor in self.encoder_data:
                self.encoder_data[motor].count = int(match.group(2))
                self.encoder_data[motor].revolutions = float(match.group(3))
                self.encoder_data[motor].rpm = float(match.group(4))

    # Firmware compact: ENC:<fr_cnt>,<fl_cnt>,<br_cnt>,<bl_cnt>,<fr_rpm>,<fl_rpm>,<br_rpm>,<bl_rpm>
    _ENC_COMPACT_ORDER = ("FR", "FL", "BR", "BL")

    def _parse_enc_compact_line(self, line: str):
        try:
            parts = line[4:].split(',')
            if len(parts) < 8:
                return
            counts = [int(p) for p in parts[:4]]
            rpms = [float(p) for p in parts[4:8]]
            for motor, cnt, rpm in zip(self._ENC_COMPACT_ORDER, counts, rpms):
                if motor in self.encoder_data:
                    self.encoder_data[motor].count = cnt
                    self.encoder_data[motor].rpm = rpm
        except (ValueError, IndexError):
            pass

    def _parse_json_line(self, line: str):
        try:
            payload = json.loads(line)
        except (ValueError, TypeError):
            return
        enc = payload.get("enc") or {}
        for key, motor in (("fr", "FR"), ("fl", "FL"), ("br", "BR"), ("bl", "BL")):
            wheel = enc.get(key)
            if isinstance(wheel, dict) and motor in self.encoder_data:
                cnt = wheel.get("cnt")
                rpm = wheel.get("rpm")
                if cnt is not None:
                    try:
                        self.encoder_data[motor].count = int(cnt)
                    except (TypeError, ValueError):
                        pass
                if rpm is not None:
                    try:
                        self.encoder_data[motor].rpm = float(rpm)
                    except (TypeError, ValueError):
                        pass
        tof = payload.get("tof")
        if isinstance(tof, dict) and self.tof_callback:
            try:
                vals = (
                    int(tof.get("front", 0)),
                    int(tof.get("front_left", 0)),
                    int(tof.get("front_right", 0)),
                    int(tof.get("left", 0)),
                    int(tof.get("right", 0)),
                    int(tof.get("back", 0)),
                )
                self.tof_callback(*vals, valid_mask=0x3F)
            except (TypeError, ValueError):
                pass

    def _parse_heartbeat_line(self, line: str):
        try:
            parts = line[7:].split(',')
            if len(parts) >= 2:
                self.heartbeat_uptime_ms = int(parts[0])
                fail_count = int(parts[1])
                self.last_heartbeat_ms = time.monotonic()
                # Log only on transitions so a flaky mux doesn't flood at 1 Hz.
                if fail_count != self.tca_fail_count:
                    delta = fail_count - self._last_tca_fail_warned
                    if delta >= 5 or fail_count == 0:
                        logger.warning(
                            f"[Encoder] TCA9548A fail count: {fail_count} "
                            f"(+{delta} since last warn)"
                        )
                        self._last_tca_fail_warned = fail_count
                    self.tca_fail_count = fail_count
        except (ValueError, IndexError):
            pass

    def _parse_tof_line(self, line: str):
        try:
            parts = line[4:].split(',')
            if len(parts) >= 6:
                vals = [int(p) for p in parts[:6]]
                # Field 7 (valid_mask) was added in firmware v3.1 and is
                # appended to the existing positional payload. Default to
                # 0x3F (all lanes valid) when the firmware is older so
                # SafetyController.update_tof preserves prior behaviour.
                valid_mask = int(parts[6]) if len(parts) >= 7 else 0x3F
                # Apply the module-level disable mask. With V5.10 this is a
                # no-op (DISABLED_TOF_BIT_MASK = 0b111111) — left in place
                # so re-disabling a future faulty lane is a constant edit
                # rather than touching this parser.
                valid_mask &= DISABLED_TOF_BIT_MASK
                if self.tof_callback:
                    self.tof_callback(*vals, valid_mask=valid_mask)
        except (ValueError, IndexError):
            pass

    def get_encoder_status(self) -> Dict:
        return {pos: asdict(data) for pos, data in self.encoder_data.items()}

    def get_encoder_counts(self) -> Dict[str, int]:
        """Get raw encoder counts for odometry."""
        return {
            "FR_count": self.encoder_data["FR"].count,
            "FL_count": self.encoder_data["FL"].count,
            "BR_count": self.encoder_data["BR"].count,
            "BL_count": self.encoder_data["BL"].count,
            "FR_rpm": self.encoder_data["FR"].rpm,
            "FL_rpm": self.encoder_data["FL"].rpm,
            "BR_rpm": self.encoder_data["BR"].rpm,
            "BL_rpm": self.encoder_data["BL"].rpm,
        }


# ==================== MAIN MQTT CLIENT V5 ====================

class KPatrolMQTTV5:
    """K-Patrol MQTT Client V5 — V3 stable base + autonomous navigation."""

    def __init__(
        self,
        mqtt_config: MQTTConfig,
        serial_config: SerialConfig,
        gps_config: Optional[GPSConfig] = None,
    ):
        self.mqtt_config = mqtt_config
        self.serial_config = serial_config
        self.gps_config = gps_config or GPSConfig()

        # Per-robot scoped topics
        self.T = Topics(mqtt_config.robot_serial)
        logger.info(f"[MQTT] Robot serial: {mqtt_config.robot_serial}")
        logger.info(f"[MQTT] Topic namespace: kpatrol/{mqtt_config.robot_serial}/#")

        # Hardware managers (from V3)
        self.motor_controller = MotorController(serial_config.motor_port)
        self.encoder_reader = EncoderReader(serial_config.encoder_port)
        self.safety_controller = SafetyController()
        self.client: Optional[mqtt.Client] = None
        self.running = False

        # V5.6: D1 R32 peripheral hub (OLED + DHT11 + relay). Initialised here
        # but `.start()` is called from run() so the read thread doesn't spin
        # before the MQTT topics object exists. Disabled if periph_port is
        # falsy in SerialConfig (operator can override via env / CLI).
        self.periph_hub: Optional[PeripheralHub] = None
        self._periph_state_dirty = False
        self._t_last_periph_pub = 0.0
        if serial_config.periph_port:
            self.periph_hub = PeripheralHub(
                port=serial_config.periph_port,
                baudrate=serial_config.baudrate,
                logger=logger,
                port_rediscover_fn=self._rediscover_periph_port,
            )
            # Hub callbacks → mark snapshot dirty so the publish loop pushes
            # immediately on the next tick. Keep callbacks cheap; they run
            # on the hub's read thread.
            self.periph_hub.on_state_change = lambda _s: self._mark_periph_dirty()
            self.periph_hub.on_dht = lambda _t, _h: self._mark_periph_dirty()
            self.periph_hub.on_watchdog_fired = self._on_periph_watchdog
            self.periph_hub.on_boot = lambda _line: self._mark_periph_dirty()

        # V5.12: network geolocator. Polled in a background thread; used by
        # publish_gps() as the *primary* position source whenever the NEO-6M
        # has no satellite fix (indoor demo). Toggle off with
        # KPATROL_NET_GPS_ENABLED=0 if running fully outdoor and you only
        # want hardware GPS.
        self.network_geolocator: Optional[NetworkGeolocator] = None
        _net_gps_enabled = (
            os.environ.get("KPATROL_NET_GPS_ENABLED", "1") not in ("0", "false", "False")
        )
        if _net_gps_enabled:
            self.network_geolocator = NetworkGeolocator()
            logger.info("[NetGPS] Network geolocation provider enabled (fallback for indoor demo)")

        # V5.2: GPS reader (NEO-6M) for outdoor patrol
        self.gps_reader: Optional["GPSReader"] = None
        if _GPS_AVAILABLE and self.gps_config.enabled:
            self.gps_reader = GPSReader(
                port=self.gps_config.port,
                baudrate=self.gps_config.baudrate,
                on_fix_acquired=self._on_gps_fix_acquired,
                on_fix_lost=self._on_gps_fix_lost,
                mode=self.gps_config.mode,
            )
            # In shared mode the firmware forwards NMEA over the motor UART;
            # hook the parser into MotorController.read_response().
            if self.gps_config.mode == "shared":
                self.motor_controller.set_gps_sink(self.gps_reader.feed_nmea)
                logger.info(f"[GPS] Shared mode — NMEA piggybacks on motor UART")
            else:
                logger.info(f"[GPS] Dedicated mode — port {self.gps_config.port}")

        # Connect ToF callback
        self.encoder_reader.set_tof_callback(self.safety_controller.update_tof)

        # Wire firmware-link health into the safety gate so a dead ESP
        # (no ENC_HB heartbeat for ~3s) blocks motion even if a stale ToF
        # frame is still cached. lambda inverts the meaning: SafetyController
        # wants a "link is alive" predicate.
        self.safety_controller.set_link_alive_check(
            lambda: not self.encoder_reader.is_heartbeat_stale()
        )

        # Navigation: scripted patrol + LINE_FOLLOW + FREE_COVERAGE.
        # Map persistence is opt-in: set KPATROL_MAP_PERSIST_DIR=/path/to/dir
        # and the FREE_COVERAGE occupancy grid is loaded on start + auto-saved
        # every 30 s. Filename = map_<robot_serial>.npz so multiple robots can
        # share the same dir without clobbering each other.
        _map_dir = os.environ.get("KPATROL_MAP_PERSIST_DIR", "").strip()
        _fc_cfg: Optional[CoverageConfig] = None
        if _map_dir:
            try:
                os.makedirs(_map_dir, exist_ok=True)
                _map_path = os.path.join(_map_dir, f"map_{mqtt_config.robot_serial}.npz")
                _fc_cfg = CoverageConfig(save_path=_map_path)
                logger.info(f"[FC] map persistence enabled → {_map_path}")
            except OSError as exc:
                logger.error(f"[FC] map persistence init failed: {exc} — running without persistence")
                _fc_cfg = None
        # V5.14: optional TwinLiteNet+ deep lane segmentation, env-gated so
        # Pis without the ONNX model file (or the onnxruntime wheel) keep
        # using the rule-based HSV pipeline. Enable with:
        #   KPATROL_LANE_SEG_ENABLED=1
        #   KPATROL_LANE_SEG_MODEL=/abs/path/to/twinlitenet_nano_192x320.onnx
        # The provider is built once here and re-used by every
        # LineFollower instance (auto_line_follow_start rebuilds the LF
        # but we keep the same provider — ONNX session is heavy to
        # construct).
        _lf_cfg: Optional[LineFollowerConfig] = None
        if os.environ.get("KPATROL_LANE_SEG_ENABLED", "0") not in ("0", "false", "False"):
            try:
                from navigation.lane_seg import LaneSegConfig, LaneSegProvider
                _model_path = os.environ.get(
                    "KPATROL_LANE_SEG_MODEL",
                    "/home/khoavd/kpatrol/pi-controller/models/twinlitenet_nano_192x320.onnx",
                )
                _seg_size_env = os.environ.get("KPATROL_LANE_SEG_SIZE", "192x320")
                try:
                    _h, _w = (int(x) for x in _seg_size_env.lower().split("x"))
                except ValueError:
                    _h, _w = 192, 320
                _seg_cfg = LaneSegConfig(
                    model_path=_model_path,
                    input_size=(_h, _w),
                )
                _seg_provider = LaneSegProvider(_seg_cfg)
                if _seg_provider.available:
                    _lf_cfg = LineFollowerConfig(lane_seg_provider=_seg_provider)
                    logger.info("[LF] TwinLiteNet+ lane segmentation active — "
                                "HSV branch disabled while provider is up")
                else:
                    logger.warning("[LF] KPATROL_LANE_SEG_ENABLED=1 but provider "
                                   "failed to initialise — falling back to HSV")
            except ImportError as exc:
                logger.warning(f"[LF] navigation.lane_seg import failed ({exc}) — using HSV")

        self._nav = NavController(fc_config=_fc_cfg, lf_config=_lf_cfg)
        self._last_nav_cmd = ""
        self._nav_lock = threading.Lock()

        # V5.4: hook front-facing ToF into the line-follower's emergency
        # cut-off. The provider is read once per camera frame (~20 Hz); the
        # snapshot read is lock-free (single-int load) which is fine for our
        # purposes — the directional gate inside SafetyController remains
        # authoritative for command-level enforcement.
        try:
            self._nav.set_line_follow_front_distance_provider(
                lambda: float(self.safety_controller.tof_data.front)
                if self.safety_controller.tof_data.front else None
            )
        except AttributeError:
            # Older NavController without the wiring helper — line_follower
            # falls back to no ToF cut-off. SafetyController.is_command_safe
            # still gates the firmware command bus.
            logger.warning("[LF] NavController has no ToF provider hook — "
                           "line-follower running without inline emergency stop")

        # LINE_FOLLOW camera: index via env var KPATROL_CAMERA (default 0)
        # Set KPATROL_CAMERA=-1 to disable camera / LINE_FOLLOW.
        _cam_idx = int(os.environ.get("KPATROL_CAMERA", "0"))
        self._camera_enabled: bool = _CV2_AVAILABLE and _cam_idx >= 0
        self._camera_index:   int  = _cam_idx
        self._lf_last_twist: Optional[Tuple[int, int, int, int]] = None
        # V5.15c5: optional simple line follower. When KPATROL_SIMPLE_LF is
        # truthy (default ON because the full BEV pipeline keeps dropping
        # to LOST on demo tape), the LF camera loop dispatches via this
        # centroid-only controller — joystick verbs F/SL/SR/DL/DR/S, no
        # MEC kinematics, no PD. Stripped down for the thesis demo where
        # "robot driving forward along a line" is the only requirement.
        self._simple_lf = None
        if os.environ.get("KPATROL_SIMPLE_LF", "1") not in ("0", "false", "False"):
            try:
                from navigation.simple_line_follower import (
                    SimpleLineConfig, SimpleLineFollower,
                )
                self._simple_lf = SimpleLineFollower(SimpleLineConfig())
                logger.info("[LF] Simple centroid follower enabled (KPATROL_SIMPLE_LF=1)")
            except ImportError as exc:
                logger.warning(f"[LF] simple_line_follower import failed: {exc}")
        self._lf_last_simple: Optional[Tuple[str, int]] = None

        # FREE_COVERAGE dedup cache — avoids resending identical MEC: commands
        self._fc_last_twist: Optional[Tuple[int, int, int, int]] = None

        # V5.3: Inline anomaly detector (env-gated, default OFF to avoid camera conflict).
        # Set KPATROL_DETECTION_ENABLED=1 plus KPATROL_DETECTION_CAMERA=<idx>
        # (must differ from KPATROL_CAMERA used by LINE_FOLLOW).
        self._det_enabled: bool = (
            _DETECTION_AVAILABLE
            and os.environ.get("KPATROL_DETECTION_ENABLED", "0") not in ("0", "false", "False")
        )
        self._detector: Optional["AnomalyDetector"]   = None
        self._alert_store: Optional["AlertStore"]     = None
        self._alert_drain_thread: Optional[threading.Thread] = None
        self._alert_drain_stop = threading.Event()
        # Track long-running threads so shutdown() can wait for cv2 capture
        # releases (otherwise /dev/video0 stays locked across systemd restarts).
        self._nav_thread: Optional[threading.Thread] = None
        self._lf_thread: Optional[threading.Thread] = None
        if self._det_enabled:
            try:
                # Accept either an integer V4L2 index or a URL string
                # (e.g. http://127.0.0.1:8080/stream — needed on Pi where
                # kpatrol-mjpeg.service owns /dev/video0 exclusively).
                _det_cam_raw = os.environ.get("KPATROL_DETECTION_CAMERA", "1")
                try:
                    _det_cam_idx: Union[int, str] = int(_det_cam_raw)
                except ValueError:
                    _det_cam_idx = _det_cam_raw
                _conflicts = (
                    isinstance(_det_cam_idx, int)
                    and _det_cam_idx == self._camera_index
                    and self._camera_enabled
                )
                if _conflicts:
                    logger.warning(f"[DETECT] WARNING: detector camera idx ({_det_cam_idx}) "
                          f"clashes with LINE_FOLLOW camera — disabling detection")
                    self._det_enabled = False
                else:
                    os.makedirs("data/snapshots", exist_ok=True)
                    det_cfg = DetectionConfig(
                        camera_index=_det_cam_idx,
                        snapshot_dir="data/snapshots",
                        robot_serial=mqtt_config.robot_serial,
                        log_level="INFO",
                    )
                    self._alert_store = AlertStore("data/alerts.db")
                    self._detector = AnomalyDetector(det_cfg, on_event=self._on_detection_event)
                    logger.info(f"[DETECT] AnomalyDetector ready (camera={_det_cam_idx!r})")
            except Exception as exc:
                logger.error(f"[DETECT] init failed: {exc} — detection disabled")
                self._det_enabled = False
                self._detector = None
                self._alert_store = None

        # V5.3: Mecanum odometry (encoder + IMU complementary filter) — fuses
        # 4-wheel cumulative counts with BNO08x yaw to estimate (x, y, θ) in
        # world frame at 20 Hz. Published on T.ODOM; reset via T.ODOM_RESET.
        self._odom = Odometry()
        self._last_odom_pub = 0.0

        # V5.3: Pi-side closed-loop velocity PID. When KPATROL_VELOCITY_PID=1 and
        # an AUTO mode is active, the nav loop converts (vx,vy,wz,spd) MEC twist
        # to physical body twist (m/s, rad/s), runs per-wheel PI against measured
        # RPM, and dispatches MOT:fr,fl,br,bl. Falls back to MEC: when disabled.
        self._vpid_enabled: bool = (
            os.environ.get("KPATROL_VELOCITY_PID", "0") not in ("0", "false", "False")
        )
        self._vpid: Optional[VelocityController] = None
        self._vpid_max_mps: float = float(os.environ.get("KPATROL_VPID_MAX_MPS", "1.0"))
        self._vpid_max_wz:  float = float(os.environ.get("KPATROL_VPID_MAX_WZ",  "3.0"))
        if self._vpid_enabled:
            self._vpid = VelocityController()
            logger.info(f"[VPID] enabled — max {self._vpid_max_mps:.2f} m/s, "
                  f"{self._vpid_max_wz:.2f} rad/s")
        self._last_mot: Optional[Tuple[int, int, int, int]] = None

        # V5.1: Link telemetry (MQTT payload metrics).
        # Accumulates across _pub() calls; publish_metrics() snapshots them
        # into a periodic METRICS frame for the dashboard / thesis report.
        self._metrics_lock   = threading.Lock()
        self._msg_out_count  = 0
        self._msg_out_bytes  = 0
        self._msg_out_drops  = 0   # attempts while client disconnected
        self._msg_in_count   = 0
        self._msg_in_bytes   = 0
        self._per_topic_bytes: Dict[str, int] = {}
        self._metrics_started_at = time.time()

        # V10 §5.3: MQTT link watchdog. If broker connection drops for >3s while
        # the robot is in an autonomous mode, slam to EMERGENCY so a runaway
        # pursuit can't continue without operator oversight.
        self._mqtt_connected: bool = False
        self._last_connect_loss_ts: Optional[float] = None
        self._link_watchdog_grace_s: float = 3.0
        self._link_watchdog_fired: bool = False

        # V5.4: Safety actuator + watchers. The actuator turns logical alerts
        # (person/fire/tipover/battery) into LP:/BUZZ: commands sent to firmware;
        # the watchers convert raw IMU and battery samples into level changes.
        # Wire all three to share the same send_command sink so a single UART
        # outage takes them all down together rather than partial-failing.
        self._alert_actuator: Optional["AlertActuator"]   = None
        self._tipover_watcher: Optional["TipOverWatcher"] = None
        self._battery_watcher: Optional["BatteryWatcher"] = None
        # V5.5: Operator-configurable alarm rule engine. Sits next to the
        # legacy alert_actuator (which still owns the immediate person/fire
        # cooldown reflex) but applies higher-level policy: event-type X
        # inside time-window W continuously for D seconds → pattern P.
        # The MQTT trigger publish lambda is bound lazily because self._pub
        # exists already on the instance.
        self._alarm_controller: Optional["AlarmController"] = None
        if _SAFETY_AVAILABLE:
            # V5.15c11 (2026-05-26): inject the D1 R32 peripheral-hub
            # relay toggle so tip-over/fire alerts actually drive the
            # 12 V lamp + horn (GPIO26). Without this the buzzer is
            # silent because the S3 motor's BUZZER_PIN is a placeholder
            # and the real piezo lives on the peripheral hub.
            def _periph_relay_drive(on: bool) -> None:
                if self.periph_hub is None:
                    return
                if on:
                    self.periph_hub.relay_on()
                else:
                    self.periph_hub.relay_off()
            self._alert_actuator = AlertActuator(
                self.motor_controller.send_command,
                periph_relay=_periph_relay_drive,
            )
            self._tipover_watcher = TipOverWatcher(
                on_tipover=self._on_tipover,
                on_recover=self._on_tipover_recover,
            )
            self._battery_watcher = BatteryWatcher(on_event=self._on_battery_event)
            self._alarm_controller = AlarmController(
                send_cmd=self.motor_controller.send_command,
                publish_trigger=lambda rec: self._pub(self.T.ALARM_TRIGGERED, rec, qos=1),
            )

    def setup_mqtt(self):
        try:
            self.client = mqtt.Client(
                callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                client_id=self.mqtt_config.client_id,
                protocol=mqtt.MQTTv311
            )
        except (AttributeError, TypeError):
            self.client = mqtt.Client(
                client_id=self.mqtt_config.client_id,
                protocol=mqtt.MQTTv311
            )

        self.client.username_pw_set(self.mqtt_config.username, self.mqtt_config.password)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message

        will_payload = json.dumps({
            "clientId": self.mqtt_config.client_id,
            "status": "offline",
            "timestamp": int(time.time() * 1000)
        })
        self.client.will_set(self.T.HEARTBEAT, will_payload, qos=1, retain=True)

    def connect(self) -> bool:
        motor_ok = self.motor_controller.connect()
        encoder_ok = self.encoder_reader.connect()
        if not motor_ok and not encoder_ok:
            logger.warning("[MQTT] Warning: No serial devices connected!")

        # GPS is best-effort: a failed connect does not abort the bridge.
        # The reader thread will retry every 3s, so an antenna replug
        # mid-mission recovers automatically.
        if self.gps_reader is not None:
            if not self.gps_reader.connect():
                logger.warning("[GPS] Initial connect failed — will retry in background")

        # V5.10: in-process retry instead of letting the systemd unit recycle
        # us. Every full process exit triggered another DTR pulse on
        # /dev/ttyKPATROL_MOTOR (CH9102 wires DTR→EN on the S3 dev board), so
        # the operator saw the robot "come up but not respond" for ~90 s
        # while the broker was unreachable — five restart loops, five
        # forced ESP32-S3 reboots. Retrying inline keeps the serial port
        # open across broker outages so the firmware boots exactly once.
        #
        # Backoff schedule: 1, 2, 4, 8, 15, 15 … seconds (capped). Aggressive
        # at the start so a transient broker blip recovers in ~1 s, then
        # backs off so a longer outage doesn't burn CPU on tight retries.
        # No upper retry cap — systemd will SIGTERM us if it really wants
        # us gone, which `time.sleep()` honours.
        attempt = 0
        backoff_seconds = [1, 2, 4, 8, 15]
        while True:
            try:
                self.client.connect(self.mqtt_config.host, self.mqtt_config.port, self.mqtt_config.keepalive)
                if attempt > 0:
                    logger.info(f"[MQTT] Connected after {attempt + 1} attempt(s)")
                return True
            except Exception as e:
                delay = backoff_seconds[min(attempt, len(backoff_seconds) - 1)]
                logger.warning(f"[MQTT] Connection failed ({e}); retry in {delay}s")
                time.sleep(delay)
                attempt += 1

    # ── GPS callbacks ───────────────────────────────────────────────

    def _on_gps_fix_acquired(self, data) -> None:
        msg = (
            f"GPS fix acquired: {data.satellites} sats, "
            f"HDOP={data.hdop:.1f}, "
            f"({data.latitude:.6f}, {data.longitude:.6f})"
        )
        logger.info(f"[GPS] {msg}")
        # Defer publish_log until MQTT is connected — connect() is the path
        # that initialises everything, and the LWT may not be set yet.
        if self.client is not None and self.client.is_connected():
            self.publish_log(msg)

    def _on_gps_fix_lost(self) -> None:
        msg = "GPS fix lost — falling back to encoder/IMU dead-reckoning"
        logger.info(f"[GPS] {msg}")
        if self.client is not None and self.client.is_connected():
            self.publish_log(msg)

    # ── MQTT Callbacks ──────────────────────────────────────────────

    def on_connect(self, client, userdata, flags, rc, properties=None):
        reason_code = rc.value if hasattr(rc, 'value') else rc
        if reason_code == 0:
            logger.info("[MQTT] Connected successfully!")
            T = self.T
            # Subscribe using wildcard: kpatrol/{serial}/#
            # This covers ALL command topics in a single subscription.
            client.subscribe(T.WILDCARD, 1)
            logger.info(f"[MQTT] Subscribed to {T.WILDCARD}")
            # V10 §5.3 link watchdog: broker reachable again — reset state.
            self._mqtt_connected = True
            self._last_connect_loss_ts = None
            self._link_watchdog_fired = False
            self.send_heartbeat("online")
            self.publish_log(f"K-Patrol V5 connected | serial={T.serial}")
        else:
            logger.info(f"[MQTT] Connect failed with code: {reason_code}")
            self._mqtt_connected = False
            if self._last_connect_loss_ts is None:
                self._last_connect_loss_ts = time.time()

    def on_disconnect(self, client, userdata, disconnect_flags=None, rc=None, properties=None):
        reason = rc.value if hasattr(rc, 'value') else (rc if rc is not None else 0)
        logger.info(f"[MQTT] Disconnected (rc={reason})")
        # V10 §5.3 link watchdog: start the 3s grace timer; main loop will
        # promote to EMERGENCY if reconnect doesn't happen in time.
        self._mqtt_connected = False
        self._last_connect_loss_ts = time.time()
        self._link_watchdog_fired = False

    def on_message(self, client, userdata, msg):
        try:
            topic = msg.topic
            with self._metrics_lock:
                self._msg_in_count += 1
                self._msg_in_bytes += len(msg.payload) if msg.payload else 0
            # V5.15: skip our own binary publishes. We subscribe wildcard
            # `kpatrol/<serial>/#` so the camera JPEG topic gets echoed
            # back to us — its payload is raw bytes (0xff JPEG SOI), not
            # JSON, and trying to json.loads() it spams the log with
            # `utf-8 codec can't decode 0xff in position 0` once per
            # frame. Cheap check: JSON payloads always start with a
            # whitespace or `{`, `[`, `"`, `-`, `0-9`, `t/f/n` — bail
            # out fast on anything else.
            raw = msg.payload
            if not raw:
                return
            first = raw[0:1]
            if first not in (b"{", b"[", b'"', b"-", b" ", b"\t", b"\n",
                             b"\r", b"t", b"f", b"n") and not (b"0" <= first <= b"9"):
                return
            try:
                payload = json.loads(raw.decode())
            except (UnicodeDecodeError, json.JSONDecodeError):
                return
            T = self.T
            if topic == T.COMMAND:
                self.handle_command(payload)
            elif topic == T.MOTOR:
                self.handle_motor_command(payload)
            elif topic == T.SPEED:
                self.handle_speed(payload)
            elif topic == T.EMERGENCY:
                self.handle_emergency(payload)
            elif topic == T.LIGHT:
                self.handle_light(payload)
            elif topic == T.MAIN_LIGHT:
                self.handle_main_light(payload)
            elif topic == T.SAFETY_CONFIG:
                self.handle_safety_config(payload)
            elif topic == T.NAV_CMD:
                self.handle_nav_command(payload)
            elif topic == T.GPS_ROUTE:
                self.handle_gps_route(payload)
            elif topic == T.BUZZ:
                self.handle_buzzer(payload)
            elif topic == T.LIGHT_PATTERN:
                self.handle_light_pattern(payload)
            elif topic == T.ODOM_RESET:
                self.handle_odom_reset(payload)
            elif topic == T.ALARM_RULES:
                # Backend pushes the full rule set as a JSON array. The
                # controller does its own validation and drops malformed
                # entries silently — the operator sees that via the trigger
                # echo or the backend admin endpoint.
                if self._alarm_controller is not None:
                    self._alarm_controller.update_rules(payload)
                    logger.info(f"[ALARM] Updated rules: {len(payload) if isinstance(payload, list) else 1}")
            elif topic == T.ALERT:
                # V5.15c14 (2026-05-26): the camera detection pipeline lives in
                # the standalone `kpatrol-detection.service` (alert_bridge.py)
                # which publishes person/fire alerts to `kpatrol/<serial>/alert`.
                # The main controller subscribes wildcard so we already see the
                # message — wire it into the same actuator + alarm-controller
                # path that the inline detector would have used. Without this
                # bridge, operator-configured fire/person rules never fired
                # because alarm_controller.on_event() was never called.
                self.handle_alert(payload)
            elif topic == T.PERIPH_CMD:
                # V5.6: peripheral-hub command (relay / oled / polarity / time).
                self.handle_periph_command(payload)
        except json.JSONDecodeError:
            logger.info(f"[MQTT] Invalid JSON: {msg.payload}")
        except Exception as e:
            logger.error(f"[MQTT] Error: {e}")

    # ── Command Handlers (V3 base + directional safety) ────────────

    def handle_command(self, payload: Dict[str, Any]):
        """Handle movement command with DIRECTIONAL safety check."""
        cmd_type = payload.get("type", "")
        speed = payload.get("speed")

        if cmd_type == "RESET_ENCODERS":
            self.encoder_reader.reset_counters()
            return
        if cmd_type == "REQUEST_IMU":
            self.motor_controller.request_imu()
            return

        # Directional safety: only block if THIS command's direction is DANGER
        if not self.safety_controller.is_command_safe(cmd_type):
            alternatives = self.safety_controller.get_safe_alternatives(cmd_type)
            alt_str = ", ".join(alternatives) if alternatives else "none"
            logger.info(f"[Safety] Blocking {cmd_type} (safe: {alt_str})")
            self.publish_log(f"Safety blocked: {cmd_type} (try: {alt_str})")
            self.motor_controller.send_command("S")
            return

        if speed is not None:
            multiplier = self.safety_controller.get_command_speed_multiplier(cmd_type)
            self.motor_controller.set_speed(int(speed * multiplier))

        self.motor_controller.send_command(cmd_type)

    def handle_motor_command(self, payload: Dict[str, Any]):
        # Refuse manual control while any autonomous mode is active.
        mode = self._nav.get_mode()
        if mode in ("AUTO_FREE_COVERAGE", "AUTO_LINE_FOLLOW",
                    "AUTO_GPS_WAYPOINT", "EMERGENCY"):
            return
        cmd_type = payload.get("type", "")
        speed = payload.get("speed")
        if speed is not None:
            multiplier = self.safety_controller.get_speed_multiplier()
            self.motor_controller.set_speed(int(speed * multiplier))
        self.motor_controller.send_command(cmd_type)

    def handle_speed(self, payload: Dict[str, Any]):
        speed = payload.get("speed", 150)
        multiplier = self.safety_controller.get_speed_multiplier()
        self.motor_controller.set_speed(int(speed * multiplier))

    def handle_emergency(self, payload: Dict[str, Any]):
        logger.info("[EMERGENCY] Emergency stop!")
        self.motor_controller.send_command("S")
        # Also switch nav to MANUAL
        self._nav.set_mode("MANUAL")

    def handle_light(self, payload: Dict[str, Any]):
        cmd_type = payload.get("type", "")
        if cmd_type in ["LIGHT_ON", "LIGHT_OFF", "LIGHT_T"]:
            if self.motor_controller.send_command(cmd_type):
                if cmd_type == "LIGHT_ON": self.motor_controller.light_state = True
                elif cmd_type == "LIGHT_OFF": self.motor_controller.light_state = False
                elif cmd_type == "LIGHT_T": self.motor_controller.light_state = not self.motor_controller.light_state
                # V5.7: discrete-state event → push STATUS immediately so the
                # PWA button flips within ~100 ms instead of waiting for the
                # 2-s steady-state cadence (which is what made light toggles
                # feel laggy even though the UART write was instant).
                self.publish_status()

    def handle_main_light(self, payload: Dict[str, Any]):
        cmd_type = payload.get("type", "")
        if cmd_type in ["MAIN_ON", "MAIN_OFF", "MAIN_T"]:
            if self.motor_controller.send_command(cmd_type):
                if cmd_type == "MAIN_ON": self.motor_controller.main_light_state = True
                elif cmd_type == "MAIN_OFF": self.motor_controller.main_light_state = False
                elif cmd_type == "MAIN_T": self.motor_controller.main_light_state = not self.motor_controller.main_light_state
                self.publish_status()

    def handle_safety_config(self, payload: Dict[str, Any]):
        enabled = payload.get("enabled")
        if enabled is not None:
            self.safety_controller.set_enabled(enabled)
        if "danger_distance" in payload:
            self.safety_controller.config.danger_distance = payload["danger_distance"]
        if "caution_distance" in payload:
            self.safety_controller.config.caution_distance = payload["caution_distance"]
        if "slow_distance" in payload:
            self.safety_controller.config.slow_distance = payload["slow_distance"]
        # V5.7: discrete-state change → immediate STATUS push so the safety
        # toggle button in the PWA flips right away instead of waiting up
        # to 2 s for the next steady-state telemetry cycle.
        self.publish_status()
        try:
            self.publish_safety()
        except AttributeError:
            pass

    # ── Navigation command handler ─────────────────────────────────

    def handle_nav_command(self, payload: Dict[str, Any]):
        """Process nav_command payload.

        {"mode":"MANUAL|AUTO_FREE_COVERAGE|AUTO_LINE_FOLLOW|"
                "AUTO_GPS_WAYPOINT|EMERGENCY",
         "speed":60,
         "action":"clear_emergency"}

        Legacy aliases LINE_FOLLOW / FREE_COVERAGE / SCRIPT_PATROL are
        accepted for backwards compat with older dashboards.
        """
        if "speed" in payload:
            self._nav.set_speed(int(payload["speed"]))

        # Accept `action` as primary path, but also tolerate clients that
        # accidentally put the action verb in `mode` (older dashboards used
        # to send {"mode":"clear_emergency"}). Either lands here cleanly.
        action = str(payload.get("action") or payload.get("mode") or "").lower()
        if action == "clear_emergency":
            self._nav.clear_emergency()
            # V5.4: silence light + buzzer alongside nav-level clear so the
            # operator doesn't need a second button press to stop the alarm.
            if self._alert_actuator is not None:
                try:
                    self._alert_actuator.clear()
                except Exception as exc:
                    logger.error(f"[ALERT→ACT] clear error: {exc}")
            self.publish_log("Emergency cleared")

        if "mode" in payload:
            mode_str = str(payload["mode"]).upper()
            # Older dashboards sent verb-style strings like
            # "AUTO_LINE_FOLLOW_START" or "MANUAL_STOP". Strip the trailing
            # action suffix so they land in the alias map below.
            for suffix in ("_START", "_STOP", "_BEGIN", "_RESUME"):
                if mode_str.endswith(suffix):
                    mode_str = mode_str[: -len(suffix)]
                    break
            # Backwards-compat aliases for older dashboards
            mode_str = {
                "LINE_FOLLOW":   "AUTO_LINE_FOLLOW",
                "FREE_COVERAGE": "AUTO_FREE_COVERAGE",
                "SCRIPT_PATROL": "AUTO_FREE_COVERAGE",
                "GPS_WAYPOINT":  "AUTO_GPS_WAYPOINT",
            }.get(mode_str, mode_str)

            # Always stop motors FIRST before any mode switch
            self.motor_controller.send_command("S")
            self._last_nav_cmd = "S"
            self._lf_last_twist = None   # reset dedup cache for AUTO_LINE_FOLLOW
            self._fc_last_twist = None   # reset dedup cache for AUTO_FREE_COVERAGE

            if mode_str == "AUTO_LINE_FOLLOW":
                result = self._nav.auto_line_follow_start()
                if not result.get("ok"):
                    self.publish_log(f"AUTO_LINE_FOLLOW start failed: {result.get('error','?')}")
                    return
                if not self._camera_enabled:
                    self.publish_log("WARN: camera disabled — AUTO_LINE_FOLLOW active but robot will not move")

            elif mode_str == "AUTO_FREE_COVERAGE":
                result = self._nav.auto_free_coverage_start()
                if not result.get("ok"):
                    self.publish_log(f"AUTO_FREE_COVERAGE start failed: {result.get('error','?')}")
                    return

            elif mode_str == "AUTO_GPS_WAYPOINT":
                result = self._nav.auto_gps_waypoint_start()
                if not result.get("ok"):
                    self.publish_log(f"AUTO_GPS_WAYPOINT start failed: {result.get('error','?')}")
                    return

            else:
                if not self._nav.set_mode(mode_str):
                    self.publish_log(f"Unknown mode: {mode_str}")
                    return
            logger.info(f"[Nav] Mode → {mode_str}")

            # Set ESP32 speed to match nav _speed for autonomous modes
            if mode_str != "MANUAL":
                esp_speed = int(self._nav._speed * 255 / 100)
                self.motor_controller.set_speed(esp_speed)
            else:
                logger.info("[Nav] MANUAL — motors stopped")

    # ── GPS route command handler (AUTO_GPS_WAYPOINT) ──────────────

    def handle_gps_route(self, payload: Dict[str, Any]):
        """Process gps_route payload.

        {"action":"set|start|stop",
         "waypoints":[{"lat":21.028,"lon":105.804,"radius_m":3,"label":"Gate"}],
         "loop":false}
        """
        action = str(payload.get("action", "")).lower()

        if action == "set":
            wps = payload.get("waypoints") or []
            loop = bool(payload.get("loop", False))
            result = self._nav.auto_gps_waypoint_set_route(wps, loop=loop)
            logger.info(f"[GPS_ROUTE] Set {len(wps)} waypoint(s) → {result}")
            if result.get("ok"):
                self.publish_log(f"GPS route loaded: {result['count']} waypoints (loop={result['loop']})")
            else:
                self.publish_log(f"GPS route set failed: {result.get('error','?')}")
            self._pub(self.T.GPS_STATUS, {**result, "action": "set"})
            return

        if action == "start":
            wps = payload.get("waypoints")  # optional
            loop = bool(payload.get("loop", False))
            self.motor_controller.send_command("S")
            self._last_nav_cmd = "S"
            result = self._nav.auto_gps_waypoint_start(waypoints=wps, loop=loop)
            logger.info(f"[GPS_ROUTE] Start → {result}")
            self.publish_log(f"GPS waypoint nav: {result}")
            self._pub(self.T.GPS_STATUS, {**result, "action": "start"})
            return

        if action == "stop":
            result = self._nav.auto_gps_waypoint_stop()
            self.motor_controller.send_command("S")
            self._last_nav_cmd = "S"
            logger.info("[GPS_ROUTE] Stop")
            self.publish_log("GPS waypoint nav stopped")
            self._pub(self.T.GPS_STATUS, {**result, "action": "stop"})
            return

        self._pub(self.T.GPS_STATUS, {"ok": False, "error": f"unknown action {action}"})

    # ── Buzzer / Light pattern (firmware-side state machines) ──────

    def handle_alert(self, payload: Dict[str, Any]) -> None:
        """V5.15c14 (2026-05-26): Forward detection alerts from the standalone
        `kpatrol-detection.service` into the in-process actuator + alarm
        controller so operator rules actually fire.

        Payload shape (from alert_bridge.AlertBridge.publish):
            {"id":int, "kind":"person"|"fire", "confidence":float,
             "bbox":[x,y,w,h], "ts":float, ...}

        We ignore our own publishes (the MQTT wildcard echoes them back) by
        checking whether the alert was minted in this process — alert_bridge
        marks records with an `id` row from its sqlite database; this code
        only fires when `id` is present and `kind` is a known label. The
        same actuator coalescing + alarm-controller dwell logic the inline
        detector relies on remains unchanged.
        """
        kind = str(payload.get("kind", "")).lower().strip()
        if kind not in ("person", "fire"):
            return
        try:
            conf = float(payload.get("confidence") or 0.0)
        except (TypeError, ValueError):
            conf = 0.0
        bbox = payload.get("bbox") or []

        if self._alert_actuator is not None:
            try:
                self._alert_actuator.on_detection(kind, conf)
            except Exception as exc:
                logger.error(f"[ALERT→ACT] on_detection error: {exc}")

        if self._alarm_controller is not None:
            try:
                self._alarm_controller.on_event(
                    kind, {"confidence": conf, "bbox": list(bbox)},
                )
            except Exception as exc:
                logger.error(f"[ALARM] on_alert error: {exc}")

    def handle_buzzer(self, payload: Dict[str, Any]):
        """Forward buzzer command to firmware. {"pattern":"OFF|ON|BEEP|ALARM|SOS"}."""
        pattern = str(payload.get("pattern", "")).upper().strip()
        if pattern not in ("OFF", "ON", "BEEP", "ALARM", "SOS"):
            self.publish_log(f"Buzzer: invalid pattern '{pattern}'")
            return
        self.motor_controller.send_command(f"BUZZ:{pattern}")

    def handle_light_pattern(self, payload: Dict[str, Any]):
        """Forward warning-light pattern to firmware."""
        pattern = str(payload.get("pattern", "")).upper().strip()
        valid = ("OFF", "WARN_BLINK", "WARN_STROBE", "BOTH_BLINK", "SOS")
        if pattern not in valid:
            self.publish_log(f"LightPattern: invalid '{pattern}'")
            return
        self.motor_controller.send_command(f"LP:{pattern}")

    # ── Odometry handlers ──────────────────────────────────────────

    def handle_odom_reset(self, payload: Dict[str, Any]) -> None:
        """Force-reset world pose to a known waypoint.

        Payload: {"x": float, "y": float, "theta_deg": float}
        Used after entering a waypoint in line-follow mode or when
        the dashboard issues a "set as origin" command.
        """
        try:
            x = float(payload.get("x", 0.0))
            y = float(payload.get("y", 0.0))
            theta_deg = float(payload.get("theta_deg", 0.0))
            self._odom.reset(x=x, y=y, theta_deg=theta_deg)
            self.publish_log(f"Odom reset → x={x:.2f} y={y:.2f} θ={theta_deg:.1f}°")
        except (TypeError, ValueError) as exc:
            logger.error(f"[ODOM] reset payload error: {exc}")

    # ── Publishers ─────────────────────────────────────────────────

    def _pub(self, topic: str, payload: dict, qos: int = 0, retain: bool = False):
        data = json.dumps(payload)
        if self.client and self.client.is_connected():
            self.client.publish(topic, data, qos=qos, retain=retain)
            with self._metrics_lock:
                self._msg_out_count += 1
                self._msg_out_bytes += len(data)
                # Short topic label: strip the kpatrol/{serial}/ prefix.
                label = topic.rsplit("/", 1)[-1]
                self._per_topic_bytes[label] = self._per_topic_bytes.get(label, 0) + len(data)
        else:
            with self._metrics_lock:
                self._msg_out_drops += 1

    def _pub_raw(self, topic: str, payload: bytes, qos: int = 0) -> None:
        """Publish raw binary payload (e.g. JPEG bytes for camera topic)."""
        if self.client and self.client.is_connected():
            self.client.publish(topic, payload, qos=qos)
            with self._metrics_lock:
                self._msg_out_count += 1
                self._msg_out_bytes += len(payload)
                label = topic.rsplit("/", 1)[-1]
                self._per_topic_bytes[label] = self._per_topic_bytes.get(label, 0) + len(payload)
        else:
            with self._metrics_lock:
                self._msg_out_drops += 1

    def publish_metrics(self):
        """Emit link telemetry — bytes/msg counts since start, plus per-topic share.

        Retained so dashboards arriving mid-stream get the latest view.
        """
        with self._metrics_lock:
            uptime = max(1.0, time.time() - self._metrics_started_at)
            snapshot = {
                "msg_out_count":   self._msg_out_count,
                "msg_out_bytes":   self._msg_out_bytes,
                "msg_out_drops":   self._msg_out_drops,
                "msg_in_count":    self._msg_in_count,
                "msg_in_bytes":    self._msg_in_bytes,
                "bytes_per_sec":   round(self._msg_out_bytes / uptime, 1),
                "msgs_per_sec":    round(self._msg_out_count / uptime, 2),
                "per_topic_bytes": dict(self._per_topic_bytes),
                "uptime_s":        round(uptime, 1),
                "timestamp":       int(time.time() * 1000),
            }
        self._pub(self.T.METRICS, snapshot, qos=0, retain=True)

    def send_heartbeat(self, status: str = "online"):
        # Retain the online heartbeat so late-subscribing dashboards see
        # the latest status without waiting for the next interval. The LWT
        # (will_set, qos=1, retain=True) publishes "offline" with the same
        # topic when this client drops, so the retained flag is consistent.
        self._pub(self.T.HEARTBEAT, {
            "clientId": self.mqtt_config.client_id,
            "status": status,
            "esp32_motor": self.motor_controller.connected,
            "esp32_encoder": self.encoder_reader.connected,
            "safety_enabled": self.safety_controller.config.enabled,
            "nav_mode": self._nav.get_mode(),
            "serial": self.T.serial,
            "timestamp": int(time.time() * 1000),
        }, qos=1, retain=True)

    def _tick_battery_watcher(self):
        # Sampled at 5Hz from the main loop so the LFP cliff (≤20% → critical)
        # is detected within ~200ms instead of waiting for the 2s status cadence.
        if self._battery_watcher is None:
            return
        battery_pct = self.motor_controller.get_battery_pct()
        if battery_pct is None:
            return
        try:
            self._battery_watcher.tick(battery_pct)
        except Exception as exc:
            logger.error(f"[BATTERY] tick error: {exc}")

    def publish_status(self):
        # Retained: latest full snapshot is always available to new subscribers.
        # Battery: prefer firmware-reported pct; fall back to 85 only when no
        # reading has arrived yet (firmware older than V5.4 / sensor offline).
        # Watcher tick happens in its own 5Hz loop branch — keep this method
        # publish-only so MQTT cadence and safety cadence are independent.
        battery_pct = self.motor_controller.get_battery_pct()
        if battery_pct is None:
            battery_value = 85
        else:
            battery_value = round(float(battery_pct), 1)
        self._pub(self.T.STATUS, {
            "connected": True,
            "esp32_motor": self.motor_controller.connected,
            "esp32_encoder": self.encoder_reader.connected,
            "battery": battery_value,
            "speed": self.motor_controller.current_speed,
            "lightState": self.motor_controller.light_state,
            "mainLightState": self.motor_controller.main_light_state,
            "temperature": 45,
            "uptime": int(time.time()),
            "motors": self.motor_controller.get_motor_status(),
            "encoders": self.encoder_reader.get_encoder_status(),
            "safety": self.safety_controller.get_status(),
            "imu": self.motor_controller.get_imu_status(),
            "nav_mode": self._nav.get_mode(),
            "timestamp": int(time.time() * 1000),
        }, qos=1, retain=True)

    # ----- V5.3: Detection → Navigation safety bridge -----------------

    def _on_detection_event(self, event: "DetectionEvent") -> None:
        """Inline callback fired by AnomalyDetector when a person/fire is confirmed.

        Order matters: react FIRST (lowest latency for fire emergency-stop),
        persist SECOND (durable record), publish THIRD (best-effort to broker).
        """
        kind = str(event.kind).lower().strip()
        try:
            nav_result = self._nav.on_alert(kind, float(event.confidence), tuple(event.bbox))
            logger.info(f"[DETECT→NAV] {kind} conf={event.confidence:.2f} → {nav_result.get('action')}")
        except Exception as exc:
            logger.error(f"[DETECT→NAV] on_alert error: {exc}")

        # V5.4: physical alert (light + buzzer). Coalescing inside the actuator
        # prevents UART flooding when the detector fires repeatedly on the same
        # subject. Failures are non-fatal; nav + persist must still proceed.
        if self._alert_actuator is not None:
            try:
                self._alert_actuator.on_detection(kind, float(event.confidence))
            except Exception as exc:
                logger.error(f"[ALERT→ACT] on_detection error: {exc}")

        # V5.5: feed alarm controller. The controller will only act on rules
        # the operator has armed; detection on its own is a no-op here.
        if self._alarm_controller is not None:
            try:
                self._alarm_controller.on_event(
                    kind,
                    {"confidence": float(event.confidence), "bbox": list(event.bbox)},
                )
            except Exception as exc:
                logger.error(f"[ALARM] on_detection error: {exc}")

        alert_id: Optional[int] = None
        if self._alert_store is not None:
            try:
                alert_id = self._alert_store.insert(
                    kind=kind,
                    confidence=float(event.confidence),
                    bbox=tuple(event.bbox),
                    snapshot=event.snapshot_path,
                    robot=self.mqtt_config.robot_serial,
                    frame_w=event.frame_width,
                    frame_h=event.frame_height,
                    ts=event.timestamp,
                )
            except Exception as exc:
                logger.error(f"[DETECT] persist failed: {exc}")

        if self._publish_alert_row(
            alert_id=alert_id if alert_id is not None else -1,
            kind=kind,
            confidence=float(event.confidence),
            bbox=tuple(event.bbox),
            ts=float(event.timestamp),
            snapshot=str(event.snapshot_path),
            frame_w=int(event.frame_width),
            frame_h=int(event.frame_height),
            snapshot_b64=str(getattr(event, "snapshot_b64", "") or ""),
        ) and alert_id is not None and self._alert_store is not None:
            try:
                self._alert_store.mark_synced(alert_id)
            except Exception as exc:
                logger.error(f"[DETECT] mark_synced failed: {exc}")

    def _publish_alert_row(
        self,
        alert_id: int,
        kind: str,
        confidence: float,
        bbox: tuple,
        ts: float,
        snapshot: str,
        frame_w: int,
        frame_h: int,
        snapshot_b64: str = "",
    ) -> bool:
        payload = {
            "id": alert_id,
            "kind": kind,
            "confidence": round(confidence, 3),
            "bbox": list(bbox),
            "ts": ts,
            "snapshot": snapshot,
            "robot": self.mqtt_config.robot_serial,
            "frame_size": [frame_w, frame_h],
        }
        # Only attach b64 when present so AlertStore drain replays (which
        # don't carry the image) stay light, and EMQX retained-message size
        # stays low.
        if snapshot_b64:
            payload["snapshot_b64"] = snapshot_b64
        body = json.dumps(payload, separators=(",", ":"))
        if self.client is None or not self.client.is_connected():
            with self._metrics_lock:
                self._msg_out_drops += 1
            return False
        try:
            info = self.client.publish(self.T.ALERT, body, qos=1, retain=False)
            with self._metrics_lock:
                self._msg_out_count += 1
                self._msg_out_bytes += len(body)
                label = self.T.ALERT.rsplit("/", 1)[-1]
                self._per_topic_bytes[label] = self._per_topic_bytes.get(label, 0) + len(body)
            return getattr(info, "rc", 0) == 0
        except Exception as exc:
            logger.error(f"[DETECT] publish failed: {exc}")
            return False

    def _alert_drain_loop(self, interval_sec: float = 5.0) -> None:
        """Retries unsynced alert rows when broker reconnects."""
        if self._alert_store is None:
            return
        while not self._alert_drain_stop.wait(interval_sec):
            if self.client is None or not self.client.is_connected():
                continue
            try:
                rows = self._alert_store.unsynced(limit=50)
            except Exception as exc:
                logger.error(f"[DETECT] drain read failed: {exc}")
                continue
            for row in rows:
                try:
                    bbox = tuple(json.loads(row["bbox_json"]))
                except Exception:
                    bbox = (0, 0, 0, 0)
                if self._publish_alert_row(
                    alert_id=row["id"],
                    kind=row["kind"],
                    confidence=float(row["confidence"]),
                    bbox=bbox,
                    ts=float(row["ts"]),
                    snapshot=row.get("snapshot") or "",
                    frame_w=int(row["frame_w"]),
                    frame_h=int(row["frame_h"]),
                ):
                    try:
                        self._alert_store.mark_synced(row["id"])
                    except Exception as exc:
                        logger.error(f"[DETECT] mark_synced (drain) failed: {exc}")
                else:
                    break

    def publish_safety(self):
        self._pub(self.T.SAFETY, self.safety_controller.get_status())

    def publish_imu(self):
        # MQTT publish only — tipover tick runs in its own 20Hz branch so
        # dashboard cadence (2Hz) is decoupled from safety reaction time.
        status = self.motor_controller.get_imu_status()
        self._pub(self.T.IMU, status)

    def _tick_tipover_watcher(self):
        # 20Hz tipover sampling: roll/pitch crosses must be caught within
        # ~50ms to brake before the robot commits to falling. Firmware
        # timestamp gates ticks so a stale IMU (UART silence) doesn't spam
        # zero-angle into the watcher.
        if self._tipover_watcher is None:
            return
        snap = self.motor_controller.get_imu_snapshot()
        if not snap.timestamp:
            return
        try:
            self._tipover_watcher.tick(
                roll_deg=float(snap.roll),
                pitch_deg=float(snap.pitch),
            )
        except Exception as exc:
            logger.error(f"[TIPOVER] tick error: {exc}")

    # ── V5.4: Safety watcher callbacks ─────────────────────────────

    def _on_tipover(self, axis: str, angle_deg: float) -> None:
        """Fired when IMU shows sustained tilt past threshold."""
        logger.warning(f"[TIPOVER] FIRING axis={axis} angle={angle_deg:.1f}°")
        try:
            self._nav.trigger_emergency(f"tipover-{axis}")
        except AttributeError:
            # NavController may not expose trigger_emergency; fall back to a
            # forced motor stop so the wheels at least stop spinning.
            self.motor_controller.send_command("S")
        if self._alert_actuator is not None:
            try:
                self._alert_actuator.on_tipover(axis, float(angle_deg))
            except Exception as exc:
                logger.error(f"[TIPOVER→ACT] on_tipover error: {exc}")
        if self._alarm_controller is not None:
            try:
                self._alarm_controller.on_event(
                    EVENT_TIPOVER, {"axis": axis, "angle_deg": float(angle_deg)}
                )
            except Exception as exc:
                logger.error(f"[ALARM] on_tipover error: {exc}")
        self.publish_log(f"Tip-over detected: {axis}={angle_deg:.1f}°")

    def _on_tipover_recover(self) -> None:
        logger.info("[TIPOVER] Recovered upright")
        # V5.15c11 (2026-05-26): we now clear the actuator on recover so
        # the SOS light + buzzer stop the moment the robot is upright
        # again. Earlier comment said operators wanted the alert to
        # persist for inspection — but V5.15c10 had a 4-6 s auto-clear
        # timer hiding that intent. After we removed the timer (so the
        # alert could actually be heard while tipped), the lights kept
        # flashing forever once the robot was put back, which confused
        # operators on the demo floor. Recover-driven clear gives the
        # right UX: latch while tipped, silent again immediately after.
        if self._alert_actuator is not None:
            try:
                self._alert_actuator.clear()
            except Exception as exc:
                logger.error(f"[TIPOVER→ACT] clear error: {exc}")
        self.publish_log("Tip-over recovered")

    def _on_battery_event(self, level: str, pct: float) -> None:
        """Fired when battery crosses LOW or CRITICAL with hysteresis.

        Publishes a `BatterySnapshot`-shaped payload to T.BATTERY so the mobile
        app's AlarmRulesView can show LFP volts + state without needing a
        separate channel. Backend forwards the raw body via Socket.IO.
        """
        logger.info(f"[BATTERY] level={level} pct={pct:.1f}")
        if self._alert_actuator is not None:
            try:
                self._alert_actuator.on_battery(level, float(pct))
            except Exception as exc:
                logger.error(f"[BATTERY→ACT] on_battery error: {exc}")
        if self._alarm_controller is not None:
            kind = EVENT_BATTERY_CRITICAL if level == "critical" else EVENT_BATTERY_LOW
            try:
                self._alarm_controller.on_event(kind, {"pct": float(pct), "level": level})
            except Exception as exc:
                logger.error(f"[ALARM] on_battery error: {exc}")

        # Compose BatterySnapshot-shaped payload (matches mobile-app
        # useAlarmSocket.BatterySnapshot). `voltage` is included only when the
        # firmware reported a pack voltage (newer BAT:<pct>,<mv> format) —
        # frontend filter drops payloads without a numeric voltage.
        voltage_mv = self.motor_controller.get_battery_voltage_mv()
        payload: Dict[str, Any] = {
            "level": level,                         # backward-compat
            "pct": round(pct, 1),                   # backward-compat
            "percent": round(pct, 1),
            "state": level,                         # "low" | "critical"
            "source": "ina219",
            "robot": self.mqtt_config.robot_serial,
            "ts": int(time.time() * 1000),
        }
        if voltage_mv is not None:
            payload["voltage"] = round(voltage_mv / 1000.0, 2)
            payload["voltage_mv"] = voltage_mv
        # NOTE: retain=False — battery threshold events are transitions, not
        # steady state. With retain=True a recovered pack would still serve a
        # "critical" payload to any client that reconnects, firing false
        # alarms after the operator returns the robot to the dock. Steady-
        # state battery % lives in T.STATUS (already retained) instead.
        self._pub(self.T.BATTERY, payload, qos=1, retain=False)
        self.publish_log(f"Battery {level}: {pct:.1f}%")

    # ── V5.6 Peripheral hub bridge ─────────────────────────────────────────
    def _rediscover_periph_port(self) -> Optional[str]:
        """Hook the PeripheralHub calls when its pinned port disappears.
        Falls back to udev symlink, then content-sniff for the periph hub
        signature (``STATE:relay=`` / ``META:fw=periph-hub``). Returns None
        if nothing convincing is plugged in right now — the hub will keep
        retrying its current path."""
        if os.path.exists("/dev/kpatrol-periph"):
            return "/dev/kpatrol-periph"
        for cand in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
            if _classify_serial_port(cand) == "periph":
                return cand
        return None

    def _mark_periph_dirty(self) -> None:
        """Set the dirty flag so the next main-loop tick publishes immediately
        instead of waiting up to 5 s for the cadence-based push."""
        self._periph_state_dirty = True

    def _on_periph_watchdog(self, reason: str) -> None:
        """Called from the hub's read thread when the on-board 5-s watchdog
        forces relay OFF. Push a banner so the dashboard can highlight it,
        and log it locally."""
        logger.warning(f"[PERIPH] watchdog: {reason}")
        self.publish_log(f"Peripheral watchdog fired: {reason}")
        self._mark_periph_dirty()

    def handle_periph_command(self, payload: Dict[str, Any]) -> None:
        """Dispatch a Web → Pi peripheral command onto the UART. Schema:

            {"action": "relay",     "state": "on"|"off"|"toggle"|"test"}
            {"action": "polarity",  "active_low": <bool>}
            {"action": "oled",      "text": "<str>"}
            {"action": "time_sync"}                              (force immediate push)
            {"action": "request_status"}                         (request META + STATE)

        Silently no-ops if the hub isn't configured. Returns no value — the
        observable effect is the next STATE: / META: line streamed back from
        the MCU, which the read thread fans out via on_state_change.
        """
        if self.periph_hub is None:
            return
        if not isinstance(payload, dict):
            return
        action = str(payload.get("action", "")).lower()
        try:
            if action == "relay":
                state = str(payload.get("state", "")).lower()
                if state == "on":
                    self.periph_hub.relay_on()
                elif state == "off":
                    self.periph_hub.relay_off()
                elif state == "toggle":
                    self.periph_hub.relay_toggle()
                elif state == "test":
                    self.periph_hub.relay_click_test()
                else:
                    logger.warning(f"[PERIPH] bad relay state: {state!r}")
            elif action == "polarity":
                self.periph_hub.set_relay_polarity(bool(payload.get("active_low", False)))
            elif action == "oled":
                self.periph_hub.set_oled_text(str(payload.get("text", "")))
            elif action == "time_sync":
                self.periph_hub.push_time_now()
            elif action == "request_status":
                self.periph_hub.request_status()
            elif action == "keepalive":
                self.periph_hub.keepalive()
            else:
                logger.warning(f"[PERIPH] unknown action: {action!r}")
        except Exception as exc:
            logger.error(f"[PERIPH] handle_periph_command error: {exc}")

    def publish_peripherals_state(self) -> None:
        """Push the latest hub snapshot to MQTT (retained=True so a fresh
        dashboard connection sees the last-known state). Skipped if the hub
        isn't configured. The payload schema lines up with the backend's
        MqttIngestService expectation on `kpatrol/<serial>/peripherals/state`."""
        if self.periph_hub is None:
            return
        snap = self.periph_hub.to_dict()
        snap["timestamp"] = int(time.time() * 1000)
        snap["serial"] = self.mqtt_config.robot_serial
        self._pub(self.T.PERIPH_STATE, snap, qos=1, retain=True)

    def publish_encoders(self):
        data = self.encoder_reader.get_encoder_status()
        data["timestamp"] = int(time.time() * 1000)
        self._pub(self.T.ENCODERS, data)

    def publish_odom(self):
        """V5.3: fuse 4-wheel encoder counts + BNO08x yaw → world pose.

        Pulls cumulative counts from EncoderReader, current yaw from the
        IMU mirror on MotorController, and publishes the fused (x, y, θ)
        pose to T.ODOM at 20 Hz. Pose is the foundation for waypoint
        navigation, return-to-start, and coverage maps.
        """
        try:
            counts_raw = self.encoder_reader.get_encoder_counts()
            counts = {
                "FR": int(counts_raw.get("FR_count", 0)),
                "FL": int(counts_raw.get("FL_count", 0)),
                "BR": int(counts_raw.get("BR_count", 0)),
                "BL": int(counts_raw.get("BL_count", 0)),
            }
            imu_snap = self.motor_controller.get_imu_snapshot()
            # Skip IMU when stale/uninitialized (timestamp 0 → encoder-only fallback)
            yaw_arg = float(imu_snap.yaw) if imu_snap.timestamp else None
            pose = self._odom.update(counts, imu_yaw_deg=yaw_arg)
            self._pub(self.T.ODOM, pose.to_dict())
        except Exception as exc:
            # Never let odometry kill the publish loop — log and continue.
            logger.error(f"[ODOM] publish error: {exc}")

    def publish_motors(self):
        data = self.motor_controller.get_motor_status()
        data["timestamp"] = int(time.time() * 1000)
        self._pub(self.T.MOTORS, data)

    def publish_nav_status(self, status: dict):
        self._pub(self.T.NAV_STATUS, status)

    def publish_log(self, message: str):
        self._pub(self.T.LOG, {"message": message, "level": "info", "timestamp": int(time.time() * 1000)})

    def publish_error(self, message: str):
        self._pub(self.T.ERROR, {"message": message, "level": "error", "timestamp": int(time.time() * 1000)}, qos=1)

    def publish_sensors(self):
        """Publish raw ToF sensor distances on the dedicated SENSORS topic."""
        tof = self.safety_controller.tof_data
        self._pub(self.T.SENSORS, {
            "tof": tof.to_float_dict() if hasattr(tof, "to_float_dict") else {},
            "timestamp": int(time.time() * 1000),
        })

    def publish_gps(self):
        """Publish latest GPS fix with hardware → network fallback.

        Three-stage source resolution (V5.12):
          1. Hardware NEO-6M with satellite fix → publish as ``source=hardware``
             (best accuracy; activates the moment robot goes outdoor).
          2. Hardware module present but no fix + NetworkGeolocator has
             a cached IP-geolocation result → publish that as
             ``source=network`` (city-block accuracy, ~5-50 km). This is
             the indoor-demo path.
          3. Neither available → publish ``hasFix=false`` so dashboard
             can show "searching…".

        QoS 0 is intentional: GPS is steady 1 Hz telemetry, a missed
        sample is replaced by the next tick — retransmit cost not worth it.
        """
        payload: Dict[str, Any]
        if self.gps_reader is not None:
            data = self.gps_reader.get_data()
            payload = data.to_dict()
            payload["connected"] = self.gps_reader.connected
        else:
            # No hardware reader configured at all — synthesize an empty fix
            # so the network path below has a payload skeleton to fill in.
            payload = {
                "hasFix": False, "fixQuality": 0, "satellites": 0,
                "hdop": 99.99, "latitude": 0.0, "longitude": 0.0,
                "altitude": 0.0, "speedKmh": 0.0, "heading": 0.0,
                "utcTime": "", "fixTimestamp": 0,
                "timestamp": int(time.time() * 1000),
                "rxCount": 0, "lastSentence": "", "connected": False,
            }

        if payload.get("hasFix"):
            # Hardware wins — keep payload exactly as-is, just tag it.
            payload["source"] = "hardware"
        elif self.network_geolocator is not None:
            net_fix = self.network_geolocator.get_fix()
            if net_fix is not None:
                # Network fallback: mark the fix as valid and overwrite the
                # lat/lon while leaving the hardware fields (satellites,
                # hdop, fixQuality) at their "no signal" sentinels — that
                # way the operator UI can still surface "NEO-6M searching"
                # next to the network-derived position.
                payload["hasFix"] = True
                payload["latitude"] = net_fix["latitude"]
                payload["longitude"] = net_fix["longitude"]
                payload["source"] = "network"
                payload["network_tier"] = net_fix.get("source_tier", "ip")
                payload["network_accuracy_m"] = net_fix.get("accuracy_m", 0.0)
                payload["network_ap_count"] = net_fix.get("ap_count", 0)
                payload["network_city"] = net_fix.get("city", "")
                payload["network_country"] = net_fix.get("country", "")
                payload["network_country_code"] = net_fix.get("country_code", "")
                payload["network_public_ip"] = net_fix.get("public_ip", "")
                payload["network_fetched_at_ms"] = net_fix.get("fetched_at_ms", 0)
                # Network-side fix is not a real GPS quality; flag fixQuality=8
                # (NMEA "manual / network" value, well outside the 0-2 set
                # the hardware emits) so consumers don't confuse it with a
                # real satellite fix.
                payload["fixQuality"] = 8
            else:
                payload["source"] = "none"
        else:
            payload["source"] = "none"

        self._pub(self.T.GPS, payload, qos=0)

    # ── LINE_FOLLOW camera loop (separate thread) ─────────────────

    def _line_follow_loop(self):
        """Camera capture + line-follower tick at ~20 Hz.

        Only active when KPATROL_CAMERA env var ≥ 0 and cv2 is installed.
        Sends MEC: commands only while NavController is in AUTO_LINE_FOLLOW mode.
        Safety veto applies: if front distance is DANGER the robot stops.
        Publishes JPEG overlay to kpatrol/{serial}/camera for dashboard view.
        """
        if not _CV2_AVAILABLE:
            logger.info("[LF] cv2 not available — LINE_FOLLOW camera disabled")
            return

        # picamera2 first, cv2.VideoCapture fallback. Threaded latest-frame
        # grabber: PD always reads the freshest frame, never replays a stale
        # one (replaying inflates D-term spuriously).
        #
        # V5.15b: bumped capture resolution 640×480 → 960×540 to match the
        # MJPEG broadcast resolution used by the inspector scripts (and
        # what the BEV homography in line_follower.py is calibrated for).
        # Mismatched resolutions silently broke the warp: BEV src points
        # at x=860 / y=250 fell outside a 640×480 frame, so warpPerspective
        # produced a near-black BEV and tick() returned `lost` every cycle.
        cam = open_camera(
            index=self._camera_index,
            width=960, height=540, framerate=30,
            threaded=True,
        )
        if cam is None:
            logger.error(f"[LF] No camera backend opened (index={self._camera_index}) — LINE_FOLLOW disabled")
            return

        logger.info(f"[LF] Camera ready via {cam.backend} (960×540 @ 30fps)")
        interval      = 0.05  # 20 Hz control loop
        cam_pub_interval = 0.20  # 5 Hz JPEG stream (MQTT bandwidth friendly)
        last_cam_pub  = 0.0
        consecutive_misses = 0

        while self.running:
            t0 = time.time()

            # Only process when in AUTO_LINE_FOLLOW mode
            if self._nav.get_mode() != "AUTO_LINE_FOLLOW":
                self._lf_last_twist = None
                time.sleep(interval)
                continue

            frame = cam.read()
            if frame is None:
                # ThreadedFrameGrabber returns None when no fresh frame yet.
                # Brief wait then retry; only log if it persists so we don't
                # flood when PD ticks faster than camera FPS.
                consecutive_misses += 1
                if consecutive_misses == 40:  # ~2 s of dry reads
                    logger.warning("[LF] No fresh frames for ~2s — camera stalled?")
                time.sleep(0.01)
                continue
            consecutive_misses = 0

            # V5.15c5: simple line follower path. Bypasses the BEV / PD /
            # heading-hold stack and uses a centroid-only controller that
            # emits joystick verbs (F/SL/SR/DL/DR/S). Enabled by the
            # KPATROL_SIMPLE_LF env flag — fall back to the full pipeline
            # otherwise. The simple branch is what we ship for the thesis
            # demo because the full pipeline kept dropping into LOST
            # mode on a thin demo tape, while joystick-grade verbs are
            # already proven to drive the wheels on this build.
            if self._simple_lf is not None:
                try:
                    s_result = self._simple_lf.tick(frame, produce_overlay=True)
                except Exception as exc:
                    logger.exception(f"[LF-simple] tick error: {exc}")
                    time.sleep(interval)
                    continue
                cmd  = s_result.command
                spd  = s_result.speed
                found = s_result.found
                if getattr(self, "_lf_last_found", None) != found:
                    self._lf_last_found = found
                    logger.info(
                        f"[LF-simple] line {'FOUND' if found else 'LOST'}: "
                        f"cmd={cmd} spd={spd} err={s_result.error_px:.0f}px "
                        f"cov={s_result.coverage_pct:.2f}%"
                    )
                # V5.15c6: resend the current command EVERY tick (~50 ms).
                # The earlier dedup-only-on-change scheme starved the
                # firmware's 241 ms motor watchdog: when the controller
                # held the same verb for a few seconds (typical mid-track),
                # nothing was sent at all and S3 force-stopped the wheels.
                # V5.15c9: dispatch branches on cmd —
                #   "MEC" → Mecanum twist (vx,vy=0,wz,spd) — true steering.
                #   "S"   → hard stop, resend only on transition.
                #   verb  → legacy joystick F/DL/DR/SL/SR.
                # Safety veto identical to the full pipeline below.
                if cmd == "MEC" and s_result.twist is not None:
                    vx, vy, wz, spd_t = s_result.twist
                    # Safety: forward TOF in danger zone ⇒ stop forward.
                    if self.safety_controller.config.enabled and vx > 0:
                        fwd_dist = self.safety_controller.direction_distances.get("forward", 9999)
                        if fwd_dist < self.safety_controller.config.danger_distance:
                            vx, vy, wz = 0, 0, 0
                    self.motor_controller.send_command(f"MEC:{vx},{vy},{wz},{spd_t}")
                    self._lf_last_simple = "MEC"
                elif cmd == "S":
                    # Only resend STOP on transition — once the firmware
                    # is stopped it stays stopped, no WDT issue.
                    if getattr(self, "_lf_last_simple", None) != "S":
                        self.motor_controller.send_command("S")
                        self._lf_last_simple = "S"
                else:
                    # Verb mode: speed update only on change, verb sent every tick.
                    if spd != getattr(self, "_lf_last_simple_spd", None):
                        self.motor_controller.set_speed(int(spd))
                        self._lf_last_simple_spd = spd
                    self.motor_controller.send_command(cmd)
                    self._lf_last_simple = cmd
                # JPEG overlay publish (same path as full pipeline below).
                if s_result.overlay is not None and t0 - last_cam_pub >= cam_pub_interval:
                    ok, jpg = _cv2.imencode(".jpg", s_result.overlay, [_cv2.IMWRITE_JPEG_QUALITY, 70])
                    if ok:
                        self._pub_raw(self.T.CAMERA, jpg.tobytes())
                        last_cam_pub = t0
                elapsed = time.time() - t0
                time.sleep(max(0.0, interval - elapsed))
                continue

            # Per-tick try/except: any vision/PD bug logs once and resumes
                # next frame instead of silently killing the LF thread.
            try:
                result = self._nav.auto_line_follow_tick(frame, produce_overlay=True)
            except Exception as exc:
                logger.exception(f"[LF] tick error: {exc}")
                time.sleep(interval)
                continue

            vx, vy, wz, spd = result.vx, result.vy, result.wz, result.spd
            twist = (vx, vy, wz, spd)
            # V5.15b: log only when the lost / found state flips, so the
            # operator gets a single line per state change instead of a
            # 20 Hz flood. Useful for diagnosing "robot doesn't move"
            # post-mortem (was it stuck in lost=True the whole time?).
            line_found_now = getattr(result, "line_found", True)
            if getattr(self, "_lf_last_found", None) != line_found_now:
                self._lf_last_found = line_found_now
                logger.info(
                    f"[LF] line {'FOUND' if line_found_now else 'LOST'}: "
                    f"vx={vx} vy={vy} wz={wz} spd={spd}"
                )

            # Detection→nav safety bridge: when on_alert("person") opens a
            # pause window, NavController.tick() returns zero twist — but
            # line-follow runs in its own thread and never calls tick().
            # Mirror the gate here so the pause actually stops the robot.
            if hasattr(self._nav, "is_alert_paused") and self._nav.is_alert_paused():
                twist = (0, 0, 0, spd)

            # Safety veto: stop forward motion if obstacle in danger zone
            fwd_dist = self.safety_controller.direction_distances.get("forward", 9999)
            if self.safety_controller.config.enabled and vx > 0:
                if fwd_dist < self.safety_controller.config.danger_distance:
                    twist = (0, 0, 0, spd)

            # Send deduplicated MEC: command
            if twist != self._lf_last_twist:
                vx2, vy2, wz2, spd2 = twist
                self.motor_controller.send_command(f"MEC:{vx2},{vy2},{wz2},{spd2}")
                self._lf_last_twist = twist

            # Publish JPEG overlay at ≤5 Hz for dashboard live view
            if result.overlay is not None and t0 - last_cam_pub >= cam_pub_interval:
                ok, jpg = _cv2.imencode(".jpg", result.overlay, [_cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    self._pub_raw(self.T.CAMERA, jpg.tobytes())
                    last_cam_pub = t0

            elapsed = time.time() - t0
            time.sleep(max(0.0, interval - elapsed))

        cam.stop()
        logger.info("[LF] Camera released")

    # ── V5: Navigation Loop (separate thread) ─────────────────────

    def _nav_loop(self):
        """Navigation tick loop — runs at 20 Hz (50 ms).

        Drives AUTO_FREE_COVERAGE and AUTO_GPS_WAYPOINT modes by polling
        NavController.tick() with fresh ToF + IMU + GPS readings.
        MANUAL is a no-op (operator drives via /motor).
        EMERGENCY forces a STOP.
        AUTO_LINE_FOLLOW is handled by _line_follow_loop() instead.

        Transport: when NavController emits a twist (vx, vy, wz, spd) we
        send a `MEC:` command for per-wheel Mecanum control. Otherwise we
        fall back to the legacy single-letter command + SPD channel. Both
        paths are deduplicated to avoid flooding the ESP32 UART.
        """
        interval = 0.05  # 20 Hz
        last_status_pub = time.time()
        last_speed_pwm: Optional[int] = None
        last_twist: Optional[Tuple[int, int, int, int]] = None

        while self.running:
            t0 = time.time()

            tof_dict = self.safety_controller.tof_data.to_float_dict()
            imu_yaw  = self.motor_controller.get_imu_snapshot().yaw

            # Encoder closed-loop feedback for move_distance steps. The
            # encoder reader uses "FR_count"/"FL_count"/... keys; executor
            # expects bare wheel keys. Translate once here.
            enc_raw: Optional[Dict[str, float]] = None
            try:
                enc_raw = self.encoder_reader.get_encoder_counts()
                enc_counts: Optional[Dict[str, int]] = {
                    "FR": int(enc_raw.get("FR_count", 0)),
                    "FL": int(enc_raw.get("FL_count", 0)),
                    "BR": int(enc_raw.get("BR_count", 0)),
                    "BL": int(enc_raw.get("BL_count", 0)),
                }
            except Exception:
                enc_counts = None
                enc_raw = None

            # Snapshot the latest fused pose so FREE_COVERAGE can run
            # map-aware navigation (occupancy grid + frontier exploration).
            try:
                nav_pose = self._odom.get_pose()
            except Exception:
                nav_pose = None

            # GPS sample for AUTO_GPS_WAYPOINT mode (None when disabled).
            gps_sample = self.gps_reader.get_data() if self.gps_reader else None

            cmd, speed_pwm, twist, status = self._nav.tick(
                tof_dict, imu_yaw,
                encoder_counts=enc_counts,
                pose=nav_pose,
                gps_data=gps_sample,
            )
            current_mode = self._nav.get_mode()

            if current_mode == "MANUAL":
                # Pass-through — operator drives via /motor.
                pass
            elif twist is not None:
                # MEC mode: single combined vx/vy/wz/spd frame per change.
                if cmd is not None and cmd not in ("S",) and not self.safety_controller.is_command_safe(cmd):
                    # Safety veto → force stop both legacy and MEC.
                    self.motor_controller.send_command("S")
                    last_twist = (0, 0, 0, 0)
                    self._last_nav_cmd = "S"
                    if self._vpid is not None:
                        self._vpid.reset()
                        self._last_mot = (0, 0, 0, 0)
                elif self._vpid is not None and self._vpid_enabled:
                    # Closed-loop velocity PID: convert MEC normalized
                    # twist → physical body twist → per-wheel PWM via PI.
                    vx_n, vy_n, wz_n, spd = twist
                    scale = (spd / 255.0) if spd else 0.0
                    vx_mps = (vx_n / 127.0) * scale * self._vpid_max_mps
                    vy_mps = (vy_n / 127.0) * scale * self._vpid_max_mps
                    wz_rps = (wz_n / 127.0) * scale * self._vpid_max_wz
                    enc = enc_raw if enc_raw else self.encoder_reader.get_encoder_counts()
                    measured = {
                        "FR": float(enc.get("FR_rpm", 0.0)),
                        "FL": float(enc.get("FL_rpm", 0.0)),
                        "BR": float(enc.get("BR_rpm", 0.0)),
                        "BL": float(enc.get("BL_rpm", 0.0)),
                    }
                    fr, fl, br, bl, _dbg = self._vpid.update(
                        vx_mps, vy_mps, wz_rps, measured,
                    )
                    new_mot = (fr, fl, br, bl)
                    if new_mot != self._last_mot:
                        self.motor_controller.send_command(
                            f"MOT:{fr},{fl},{br},{bl}"
                        )
                        self._last_mot = new_mot
                    last_twist = twist
                    self._last_nav_cmd = cmd or ""
                    if spd:
                        last_speed_pwm = int(spd)
                elif twist != last_twist:
                    vx, vy, wz, spd = twist
                    self.motor_controller.send_command(f"MEC:{vx},{vy},{wz},{spd}")
                    last_twist = twist
                    # Keep the legacy _last_nav_cmd in sync so a later
                    # non-MEC tick won't re-send a stale letter.
                    self._last_nav_cmd = cmd or ""
                    if spd:
                        last_speed_pwm = int(spd)
            else:
                # Legacy transport: SPD:<n> + single-letter directional.
                if speed_pwm is not None and speed_pwm != last_speed_pwm:
                    self.motor_controller.set_speed(int(speed_pwm))
                    last_speed_pwm = int(speed_pwm)

                if cmd is not None and cmd != self._last_nav_cmd:
                    if cmd == "S" or self.safety_controller.is_command_safe(cmd):
                        self.motor_controller.send_command(cmd)
                    else:
                        self.motor_controller.send_command("S")
                        cmd = "S"
                    self._last_nav_cmd = cmd
                    last_twist = None

            if time.time() - last_status_pub > 0.5:
                self.publish_nav_status(status)
                last_status_pub = time.time()

            elapsed = time.time() - t0
            time.sleep(max(0.0, interval - elapsed))

    # ── Main Loop ─────────────────────────────────────────────────

    def run(self):
        self.running = True
        self.client.loop_start()

        # Start navigation loop thread
        self._nav_thread = threading.Thread(target=self._nav_loop, name="nav_loop", daemon=True)
        self._nav_thread.start()

        # V5.6: Start the D1 R32 peripheral hub. Non-blocking — the driver
        # spins its own read + time-sync threads and silently keeps retrying
        # the serial open if /dev/kpatrol-periph isn't present yet.
        if self.periph_hub is not None:
            self.periph_hub.start()

        # V5.12: kick the network geolocator. First fetch runs synchronously
        # inside start(), so by the time the first publish_gps() tick fires
        # the dashboard already has a coarse city-level fix.
        if self.network_geolocator is not None:
            self.network_geolocator.start()

        # Start LINE_FOLLOW camera thread (no-op if cv2 absent or KPATROL_CAMERA=-1)
        if self._camera_enabled:
            self._lf_thread = threading.Thread(target=self._line_follow_loop, name="lf_loop", daemon=True)
            self._lf_thread.start()
            logger.info(f"[LF] Camera thread started (device {self._camera_index})")
        else:
            logger.info("[LF] Camera disabled — LINE_FOLLOW mode will not move motors")

        # V5.3: Start AnomalyDetector + alert backlog drainer (env-gated)
        if self._det_enabled and self._detector is not None:
            try:
                self._detector.start(blocking=False)
                self._alert_drain_thread = threading.Thread(
                    target=self._alert_drain_loop, name="alert_drain", daemon=True,
                )
                self._alert_drain_thread.start()
                logger.info("[DETECT] anomaly detector + drainer started")
            except Exception as exc:
                logger.error(f"[DETECT] start failed: {exc}")
        else:
            logger.info("[DETECT] disabled (set KPATROL_DETECTION_ENABLED=1 to enable)")

        heartbeat_interval = 5
        status_interval = 2
        encoder_interval = 0.5
        safety_interval = 0.2
        sensors_interval = 0.5    # ToF sensor raw data at 2Hz
        imu_interval = 0.5
        metrics_interval = 10     # V5.1 link telemetry at 0.1Hz
        gps_interval = self.gps_config.publish_interval   # V5.2 default 1Hz
        odom_interval = 0.05      # V5.3 odometry at 20Hz
        battery_tick_interval = 0.2   # V10: 5Hz battery sampling for LFP cliff
        # V9: Pi no longer polls "REQUEST_IMU" — firmware v7+ runs the IMU
        # read on its own Core 0 task and auto-emits "IMU:" lines at 10 Hz.
        # Setting interval to 0 disables the poll branch below; left as a
        # variable in case we ever need to re-enable polling against an
        # older firmware build. Tipover watcher still ticks at 20 Hz
        # against the cached IMU snapshot.
        imu_request_interval = 0.0    # disabled — firmware streams IMU
        tipover_tick_interval = 0.05  # V10: 20Hz tipover watcher (cached IMU)
        alarm_tick_interval = 1.0     # V5.5: 1Hz alarm rule engine sweep

        last_heartbeat = 0
        last_status = 0
        last_encoder = 0
        last_safety = 0
        last_sensors = 0
        last_imu = 0
        last_imu_request = 0
        last_metrics = 0
        last_gps = 0
        last_odom = 0.0
        last_battery_tick = 0.0
        last_tipover_tick = 0.0
        last_alarm_tick = 0.0
        periph_pub_interval = 5.0  # V5.6: peripheral hub steady-state cadence
        # V5.7: send KEEPALIVE to D1 R32 every 2 s. Firmware arms a 5-s
        # safety watchdog the moment any output (relay / horn / light) goes
        # ON and force-clears them all if no UART traffic arrives within
        # that window. Without this scheduled ping the user sees lights /
        # horn switch off ~5 s after pressing ON in the web UI.
        periph_keepalive_interval = 2.0
        last_periph_keepalive = 0.0

        logger.info("\n" + "=" * 60)
        logger.info("    K-PATROL MQTT CLIENT V5.0")
        logger.info("    Directional Safety + Autonomous Navigation")
        logger.info("=" * 60)
        logger.info(f"MQTT Broker: {self.mqtt_config.host}:{self.mqtt_config.port}")
        logger.info(f"Motor Port:  {self.serial_config.motor_port} ({'OK' if self.motor_controller.connected else 'FAIL'})")
        logger.info(f"Encoder Port: {self.serial_config.encoder_port} ({'OK' if self.encoder_reader.connected else 'FAIL'})")
        if self.periph_hub is not None:
            logger.info(f"Periph Port: {self.serial_config.periph_port} (started, retries every 5s)")
        else:
            logger.info("Periph Port: disabled (set SerialConfig.periph_port to enable)")
        if self.gps_reader is not None:
            gps_state = "OK" if self.gps_reader.connected else "FAIL"
            if self.gps_config.mode == "shared":
                logger.info(f"GPS:         shared via motor UART ({gps_state}) — outdoor mode")
            else:
                logger.info(f"GPS Port:    {self.gps_config.port} ({gps_state}) — outdoor mode")
        else:
            logger.info("GPS:         disabled (indoor mode)")
        logger.info(f"Safety: {'Enabled' if self.safety_controller.config.enabled else 'Disabled'} (Directional)")
        logger.info(f"Nav Mode: {self._nav.get_mode()}")
        logger.info("=" * 60)
        logger.info("Press Ctrl+C to exit\n")

        try:
            while self.running:
                current_time = time.time()

                # V10 §5.3: MQTT link watchdog. Broker dropout >3s while in an
                # autonomous mode must force EMERGENCY — operator can't see or
                # override a runaway robot if telemetry is silent.
                if (
                    not self._mqtt_connected
                    and self._last_connect_loss_ts is not None
                    and not self._link_watchdog_fired
                    and current_time - self._last_connect_loss_ts > self._link_watchdog_grace_s
                ):
                    nav_mode = self._nav.get_mode()
                    if nav_mode in ("AUTO_FREE_COVERAGE", "AUTO_LINE_FOLLOW", "AUTO_GPS_WAYPOINT"):
                        logger.warning(
                            f"[LINK-WATCHDOG] MQTT down >{self._link_watchdog_grace_s:.0f}s "
                            f"in mode={nav_mode} — forcing EMERGENCY"
                        )
                        try:
                            self._nav.trigger_emergency("mqtt-link-lost-3s")
                        except AttributeError:
                            self._nav.set_mode("EMERGENCY")
                            self.motor_controller.send_command("S")
                    self._link_watchdog_fired = True

                if current_time - last_heartbeat >= heartbeat_interval:
                    self.send_heartbeat()
                    last_heartbeat = current_time

                if current_time - last_battery_tick >= battery_tick_interval:
                    self._tick_battery_watcher()
                    last_battery_tick = current_time

                if current_time - last_status >= status_interval:
                    self.publish_status()
                    last_status = current_time

                if current_time - last_encoder >= encoder_interval:
                    self.publish_encoders()
                    self.publish_motors()
                    last_encoder = current_time

                if current_time - last_safety >= safety_interval:
                    self.publish_safety()
                    last_safety = current_time

                if current_time - last_sensors >= sensors_interval:
                    self.publish_sensors()
                    last_sensors = current_time

                if current_time - last_imu >= imu_interval:
                    self.publish_imu()
                    last_imu = current_time

                # V9: poll branch disabled when imu_request_interval == 0
                # (firmware streams IMU). Kept guarded for backward compat
                # if older firmware revisions need explicit polling.
                if imu_request_interval > 0 and current_time - last_imu_request >= imu_request_interval:
                    self.motor_controller.request_imu()
                    last_imu_request = current_time

                if current_time - last_tipover_tick >= tipover_tick_interval:
                    self._tick_tipover_watcher()
                    last_tipover_tick = current_time

                # V5.5: alarm rule engine tick (1Hz). Drives schedule rules
                # and the stale-event self-clear; on_event() pushes for
                # detector / battery / tipover events as they arrive.
                if current_time - last_alarm_tick >= alarm_tick_interval:
                    if self._alarm_controller is not None:
                        try:
                            self._alarm_controller.tick()
                        except Exception as exc:
                            logger.error(f"[ALARM] tick error: {exc}")
                    last_alarm_tick = current_time

                if current_time - last_metrics >= metrics_interval:
                    self.publish_metrics()
                    last_metrics = current_time

                if current_time - last_gps >= gps_interval:
                    self.publish_gps()
                    last_gps = current_time

                # V5.3: 20Hz odometry — encoder + IMU complementary fusion
                if current_time - last_odom >= odom_interval:
                    self.publish_odom()
                    last_odom = current_time

                # V5.6: peripheral hub state. Push immediately on any change
                # (relay flip / DHT sample / boot / watchdog) and at the 5-s
                # cadence regardless so the dashboard knows the link is alive.
                if self.periph_hub is not None:
                    if (
                        self._periph_state_dirty
                        or current_time - self._t_last_periph_pub >= periph_pub_interval
                    ):
                        self.publish_peripherals_state()
                        self._t_last_periph_pub = current_time
                        self._periph_state_dirty = False
                    # V5.7: keep firmware safety watchdog at bay.
                    if current_time - last_periph_keepalive >= periph_keepalive_interval:
                        self.periph_hub.keepalive()
                        last_periph_keepalive = current_time

                # Drain everything queued on the motor UART so IMU/BAT
                # frames don't pile up between 20Hz ticks. Returns the last
                # non-IMU/BAT line for log forwarding.
                response = self.motor_controller.drain_responses()
                if response:
                    logger.info(f"[Motor] {response}")

                time.sleep(0.05)

        except KeyboardInterrupt:
            logger.info("\n[Main] Shutting down...")

        self.shutdown()

    def shutdown(self):
        self.running = False
        self.motor_controller.send_command("S")
        time.sleep(0.3)

        # V5.6: turn relay OFF before tearing down — same intent as the
        # safety watchdog on the MCU, but driven from the Pi so we don't
        # rely on UART silence to trigger it.
        if self.periph_hub is not None:
            try:
                self.periph_hub.relay_off()
                time.sleep(0.1)
                self.periph_hub.stop()
            except Exception as exc:
                logger.error(f"[PERIPH] shutdown error: {exc}")

        # V5.3: stop detector + drainer before tearing down MQTT
        self._alert_drain_stop.set()
        if self._detector is not None:
            try:
                self._detector.stop()
            except Exception as exc:
                logger.error(f"[DETECT] stop error: {exc}")

        # Wait for nav + line-follow loops to exit so cv2.VideoCapture.release()
        # actually runs. Without this, daemon threads die mid-loop and the
        # camera (/dev/video0) can remain locked across systemd restarts.
        for thread, label in (
            (self._lf_thread, "lf_loop"),
            (self._nav_thread, "nav_loop"),
            (self._alert_drain_thread, "alert_drain"),
        ):
            if thread is not None and thread.is_alive():
                thread.join(timeout=2.0)
                if thread.is_alive():
                    logger.warning(f"[Main] {label} did not exit within 2s")

        if self.client and self.client.is_connected():
            self.send_heartbeat("offline")
            time.sleep(0.5)
            self.client.loop_stop()
            self.client.disconnect()

        self.motor_controller.disconnect()
        self.encoder_reader.disconnect()
        if self.gps_reader is not None:
            self.gps_reader.disconnect()
        if self._alert_store is not None:
            try:
                self._alert_store.close()
            except Exception as exc:
                logger.error(f"[DETECT] alert_store close error: {exc}")
        logger.info("[Main] V5 shutdown complete")


# ==================== MAIN ====================

# Per-board UART signatures used by content-based discovery. Each board
# emits at least one of these substrings within ~1 s of being opened, even
# when the firmware is idle, because every sketch has a heartbeat / status
# emitter on its main loop. We probe a port by reading ~1.5 s of bytes and
# pattern-matching against these — that way two CH340 boards with identical
# VID/PID (encoder + peripheral hub) can be told apart without locking each
# one to a specific USB-hub slot.
_SIGNATURE_ENCODER = (b"ENC_HB:", b"ENC:", b"TOF:", b"STATUS:TOF_INIT")
_SIGNATURE_PERIPH  = (b"META:fw=periph-hub", b"STATE:relay=", b"DHT:", b"BOOT:periph-hub")
_SIGNATURE_MOTOR   = (b"IMU:", b"BAT:", b"GPS_HB:", b"NMEA:")


def _classify_serial_port(port: str, timeout_s: float = 1.5) -> str:
    """Sniff a port and return one of {'encoder','periph','motor','unknown'}.

    Opens the device at 115200 baud, optionally pokes it with a harmless
    line ('PING\\n' is recognised by the peripheral hub, ignored by every
    other board), then reads up to `timeout_s` seconds looking for any of
    the per-board signatures. Returns as soon as a match is found so the
    common case (encoder + periph live, motor cable unplugged) takes ~1 s,
    not 4.5 s. On any I/O failure returns 'unknown' so the caller can fall
    back to the udev symlink.
    """
    try:
        with serial.Serial(port, 115200, timeout=0.2) as s:
            try:
                s.write(b"PING\n")
                s.flush()
            except (OSError, serial.SerialException):
                pass
            end = time.monotonic() + timeout_s
            buf = b""
            while time.monotonic() < end:
                try:
                    chunk = s.read(256)
                except (OSError, serial.SerialException):
                    break
                if not chunk:
                    continue
                buf += chunk
                # Keep the window bounded so a long banner can't outpace the
                # signature scan — 4 KB is plenty for any one-frame pattern.
                if len(buf) > 4096:
                    buf = buf[-4096:]
                if any(sig in buf for sig in _SIGNATURE_ENCODER):
                    return "encoder"
                if any(sig in buf for sig in _SIGNATURE_PERIPH):
                    return "periph"
                if any(sig in buf for sig in _SIGNATURE_MOTOR):
                    return "motor"
    except (OSError, serial.SerialException):
        pass
    return "unknown"


def find_serial_ports():
    """Locate motor / encoder / peripheral-hub UARTs robustly.

    Strategy (in order):
      1. Honour stable udev symlinks (``/dev/ttyKPATROL_MOTOR`` etc.) — these
         are populated by 99-kpatrol-serial.rules and remain valid across
         reboots when wiring is unchanged.
      2. For any role still missing, sniff every unclaimed ``/dev/ttyUSB*``
         and ``/dev/ttyACM*`` and match the data stream against per-board
         signatures (``ENC_HB:`` ⇒ encoder, ``STATE:relay=`` ⇒ periph hub,
         ``IMU:`` / ``GPS_HB:`` ⇒ motor S3).
      3. Last resort: ttyACM0 for motor, the lowest unclaimed ttyUSB* for
         encoder / periph so the bridge can at least *try* to talk.

    This means the user can swap the two CH340 boards (encoder vs periph)
    between USB slots without editing the udev rule — Pi figures it out at
    every (re)connect.
    """
    motor_port: Optional[str]   = None
    encoder_port: Optional[str] = None
    periph_port: Optional[str]  = None

    # 1) Stable udev symlinks (the strongest hint we have).
    if os.path.exists("/dev/ttyKPATROL_MOTOR"):
        motor_port = "/dev/ttyKPATROL_MOTOR"
    if os.path.exists("/dev/ttyKPATROL_ENCODER"):
        encoder_port = "/dev/ttyKPATROL_ENCODER"
    if os.path.exists("/dev/kpatrol-periph"):
        periph_port = "/dev/kpatrol-periph"

    # 2) Content-based discovery for whatever's left. Build a set of port
    #    real-paths so we don't re-probe a port that a symlink already
    #    points at.
    def _real(p):
        try:
            return os.path.realpath(p)
        except OSError:
            return p

    claimed = {
        _real(p) for p in (motor_port, encoder_port, periph_port) if p
    }
    needs_motor   = motor_port   is None
    needs_encoder = encoder_port is None
    needs_periph  = periph_port  is None

    if needs_motor or needs_encoder or needs_periph:
        candidates = sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))
        for port in candidates:
            if not (needs_motor or needs_encoder or needs_periph):
                break
            if _real(port) in claimed:
                continue
            role = _classify_serial_port(port)
            logger.info(f"[Discover] {port} -> {role}")
            if role == "motor" and needs_motor:
                motor_port = port
                needs_motor = False
            elif role == "encoder" and needs_encoder:
                encoder_port = port
                needs_encoder = False
            elif role == "periph" and needs_periph:
                periph_port = port
                needs_periph = False
            claimed.add(_real(port))

    # 3) Fallbacks so MotorController.connect() has a path to retry against
    #    even when nothing was discovered. The reconnect loop will re-run
    #    _rediscover_port at the next attempt — same approach as before.
    if motor_port is None:
        motor_port = "/dev/ttyKPATROL_MOTOR"
    if encoder_port is None:
        encoder_port = "/dev/ttyKPATROL_ENCODER"
    if periph_port is None:
        periph_port = "/dev/kpatrol-periph"

    return motor_port, encoder_port, periph_port


def _configure_logging() -> None:
    """Init root logger once for daemon mode.

    Honors KPATROL_LOG_LEVEL (default INFO). When run under systemd the output
    goes to journald; locally it goes to stderr.
    """
    level_name = os.environ.get("KPATROL_LOG_LEVEL", "INFO").upper()
    level = getattr(logging, level_name, logging.INFO)
    logging.basicConfig(
        level=level,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )


def main():
    _configure_logging()
    logger.info("=" * 60)
    logger.info("K-PATROL MQTT CLIENT V5.0 FOR RASPBERRY PI")
    logger.info("Directional Safety + Autonomous Navigation")
    logger.info("=" * 60)

    os.makedirs("data", exist_ok=True)
    motor_port, encoder_port, periph_port = find_serial_ports()

    robot_serial = os.environ.get('ROBOT_SERIAL', 'KPATROL-001')
    logger.info(f"Robot Serial: {robot_serial}")
    mqtt_config = MQTTConfig(robot_serial=robot_serial)
    mqtt_config.validate()
    serial_config = SerialConfig(
        motor_port=motor_port,
        encoder_port=encoder_port,
        periph_port=periph_port,
    )

    # GPS toggle via env: KPATROL_GPS_ENABLED=0 to skip outdoor mode entirely.
    gps_enabled = os.environ.get("KPATROL_GPS_ENABLED", "1") not in ("0", "false", "False")
    # GPS wiring mode: "shared" (NEO-6M on ESP32-S3 UART2 → forwarded over
    # motor serial as `NMEA:...`) or "dedicated" (own USB-TTL adapter).
    gps_mode = os.environ.get("KPATROL_GPS_MODE", "shared").strip().lower()
    if gps_mode not in ("shared", "dedicated"):
        logger.warning(f"[GPS] Unknown KPATROL_GPS_MODE={gps_mode!r}, falling back to 'shared'")
        gps_mode = "shared"
    # GPS port: in 'shared' mode (default), NMEA is forwarded by the motor
    # ESP32-S3 over its USB serial — no separate device is opened. In
    # 'dedicated' mode we honour KPATROL_GPS_PORT or fall back to the udev
    # symlink. Either way the dataclass needs *some* string so leave it
    # empty in shared mode rather than reaching into find_serial_ports().
    if gps_mode == "shared":
        gps_port_path = ""
    else:
        gps_port_path = os.environ.get("KPATROL_GPS_PORT", "/dev/ttyKPATROL_GPS")
    gps_config = GPSConfig(port=gps_port_path, enabled=gps_enabled, mode=gps_mode)

    client = KPatrolMQTTV5(mqtt_config, serial_config, gps_config)
    client.setup_mqtt()

    if client.connect():
        client.run()
    else:
        logger.error("[Main] Failed to connect, exiting...")
        sys.exit(1)


if __name__ == "__main__":
    main()
