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
        # Wildcard for subscribing all topics of this robot
        self.WILDCARD      = f"{p}/#"

    @property
    def serial(self) -> str:
        return self._serial


# Motor positions
MOTOR_POSITIONS = ['FR', 'FL', 'BR', 'BL']
MOTOR_INVERTED = {'FR': False, 'FL': True, 'BR': False, 'BL': True}

# Hardware bypass: front-right VL53L0X (TCA9548A channel 2) is physically
# faulty on this build — its readings fluctuate between min-range and 9999
# regardless of obstacles, which would otherwise wedge the safety controller
# into permanent DANGER. We force-clear bit2 of the firmware valid_mask and
# zero its raw distance so downstream `_valid_or_far` treats the lane as
# "unknown" instead of "in your face". Remove this once the physical sensor
# is replaced.
DISABLED_TOF_BIT_MASK = 0b111011  # clear bit2 = front_right
DISABLED_TOF_LANES = ('front_right',)


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

    # Allow ~3× ESP ToF cycle (worst-case ~200ms × 3 + slack) before declaring
    # data stale. ESP currently runs ~5–20Hz depending on preset.
    STALE_TOF_THRESHOLD_MS: int = 700

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
        self.lock = threading.Lock()
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
        # contending with the serial-I/O critical section (self.lock). Writes
        # land in _parse_imu_line; reads must go through get_imu_snapshot().
        self._imu_lock = threading.Lock()
        # V5.2: optional sink for NMEA frames forwarded by ESP32 firmware
        # (lines prefixed with "NMEA:" arrive on the motor UART0).
        self._gps_sink = None
        # V5.4: latest battery percentage reported by firmware (BAT:<pct>).
        # None means firmware hasn't sent a reading yet — telemetry uses a
        # placeholder until the first valid sample arrives.
        self.battery_pct: Optional[float] = None
        self.battery_ts: int = 0

    def set_gps_sink(self, sink) -> None:
        """Register a callback (e.g. GPSReader.feed_nmea) to receive NMEA
        sentences forwarded by the motor firmware via shared-mode pass-through."""
        self._gps_sink = sink

    def connect(self) -> bool:
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            # timeout=0.1: short read deadline so readline() cannot stall the
            # main 20Hz loop when a partial frame is in the buffer. With the
            # previous 1.0s timeout, a fragmented IMU/BAT/NMEA line could
            # freeze the bridge for up to a full second.
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=0.1)
            self.connected = True
            logger.info(f"[Motor] Connected to {self.port}")
            time.sleep(2)
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
        with self.lock:
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
        with self.lock:
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
            with self.lock:
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
        try:
            parts = line[4:].split(',')
            if len(parts) >= 4:
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
        """Parse `BAT:<pct>` (or `BAT:<pct>,<voltage>`) telemetry from firmware."""
        if not line.startswith("BAT:"):
            return
        try:
            payload = line[4:].split(",", 1)[0]
            pct = float(payload)
        except (ValueError, IndexError):
            return
        # Drop pct<=0 / pct>100 — uncalibrated ADC or no battery plugged.
        # Without this guard, BatteryWatcher would fire CRITICAL on bench runs
        # where the firmware reports BAT:0,XXX before the ADC is calibrated.
        if pct <= 0.0 or pct > 100.0:
            return
        self.battery_pct = pct
        self.battery_ts = int(time.time() * 1000)

    def get_battery_pct(self) -> Optional[float]:
        return self.battery_pct


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

    HEARTBEAT_STALE_SEC = 3.0

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

    def connect(self) -> bool:
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
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
                # Mask out lanes flagged by DISABLED_TOF_BIT_MASK (currently
                # front_right / CH2 — see module-level comment).
                valid_mask &= DISABLED_TOF_BIT_MASK
                vals[2] = 9999  # front_right raw → "far" sentinel
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
        self._nav = NavController(fc_config=_fc_cfg)
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
        if _SAFETY_AVAILABLE:
            self._alert_actuator = AlertActuator(self.motor_controller.send_command)
            self._tipover_watcher = TipOverWatcher(
                on_tipover=self._on_tipover,
                on_recover=self._on_tipover_recover,
            )
            self._battery_watcher = BatteryWatcher(on_event=self._on_battery_event)

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

        try:
            self.client.connect(self.mqtt_config.host, self.mqtt_config.port, self.mqtt_config.keepalive)
            return True
        except Exception as e:
            logger.error(f"[MQTT] Connection failed: {e}")
            return False

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
            payload = json.loads(msg.payload.decode())
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

    def handle_main_light(self, payload: Dict[str, Any]):
        cmd_type = payload.get("type", "")
        if cmd_type in ["MAIN_ON", "MAIN_OFF", "MAIN_T"]:
            if self.motor_controller.send_command(cmd_type):
                if cmd_type == "MAIN_ON": self.motor_controller.main_light_state = True
                elif cmd_type == "MAIN_OFF": self.motor_controller.main_light_state = False
                elif cmd_type == "MAIN_T": self.motor_controller.main_light_state = not self.motor_controller.main_light_state

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

        action = payload.get("action", "")
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
        self.publish_log(f"Tip-over detected: {axis}={angle_deg:.1f}°")

    def _on_tipover_recover(self) -> None:
        logger.info("[TIPOVER] Recovered upright")
        # Note: actuator is intentionally NOT cleared here — operator must
        # explicitly clear EMERGENCY before lights/buzzer go silent, so the
        # alert remains visible while crew are inspecting the robot.
        self.publish_log("Tip-over recovered")

    def _on_battery_event(self, level: str, pct: float) -> None:
        """Fired when battery crosses LOW or CRITICAL with hysteresis."""
        logger.info(f"[BATTERY] level={level} pct={pct:.1f}")
        if self._alert_actuator is not None:
            try:
                self._alert_actuator.on_battery(level, float(pct))
            except Exception as exc:
                logger.error(f"[BATTERY→ACT] on_battery error: {exc}")
        # Best-effort publish so the dashboard / mobile-app can pop a banner
        # in addition to the actuator's local light + buzzer.
        self._pub(self.T.BATTERY, {
            "level": level,
            "pct": round(pct, 1),
            "robot": self.mqtt_config.robot_serial,
            "ts": int(time.time() * 1000),
        }, qos=1, retain=True)
        self.publish_log(f"Battery {level}: {pct:.1f}%")

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
        """Publish latest GPS fix (NEO-6M) — outdoor patrol mode.

        Skipped when the GPS module is disabled or unavailable. We always
        publish even without a fix so the dashboard can show "searching…"
        status; the `hasFix` flag distinguishes usable vs. stale data.

        QoS 0 is intentional: GPS is high-frequency telemetry and a missed
        sample is replaced by the next 1Hz update — retransmit cost is not
        worth the bandwidth.
        """
        if self.gps_reader is None:
            return
        data = self.gps_reader.get_data()
        payload = data.to_dict()
        payload["connected"] = self.gps_reader.connected
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
        cam = open_camera(
            index=self._camera_index,
            width=640, height=480, framerate=30,
            threaded=True,
        )
        if cam is None:
            logger.error(f"[LF] No camera backend opened (index={self._camera_index}) — LINE_FOLLOW disabled")
            return

        logger.info(f"[LF] Camera ready via {cam.backend} (640×480 @ 30fps)")
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
        imu_request_interval = 0.05   # V10: 20Hz IMU poll for tipover safety
        tipover_tick_interval = 0.05  # V10: 20Hz tipover watcher

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

        logger.info("\n" + "=" * 60)
        logger.info("    K-PATROL MQTT CLIENT V5.0")
        logger.info("    Directional Safety + Autonomous Navigation")
        logger.info("=" * 60)
        logger.info(f"MQTT Broker: {self.mqtt_config.host}:{self.mqtt_config.port}")
        logger.info(f"Motor Port:  {self.serial_config.motor_port} ({'OK' if self.motor_controller.connected else 'FAIL'})")
        logger.info(f"Encoder Port: {self.serial_config.encoder_port} ({'OK' if self.encoder_reader.connected else 'FAIL'})")
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

                if current_time - last_imu_request >= imu_request_interval:
                    self.motor_controller.request_imu()
                    last_imu_request = current_time

                if current_time - last_tipover_tick >= tipover_tick_interval:
                    self._tick_tipover_watcher()
                    last_tipover_tick = current_time

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

def find_serial_ports():
    """Locate the three USB-serial devices the Pi expects.

    Order of preference: udev symlink (stable) → numeric fallback (dev box).

    GPS is the trickiest because both the encoder ESP32 and the GPS adapter
    enumerate as /dev/ttyUSB*. Without the udev rule, we pick the *highest*
    numbered port for GPS so a freshly-plugged adapter does not steal the
    encoder's slot.
    """
    motor_port   = "/dev/ttyACM0"
    encoder_port = "/dev/ttyUSB0"
    gps_port     = "/dev/ttyUSB1"

    # Stable udev symlinks (preferred — install 99-kpatrol-serial.rules)
    if os.path.exists("/dev/ttyKPATROL_MOTOR"):
        motor_port = "/dev/ttyKPATROL_MOTOR"
    else:
        for port in ["/dev/ttyACM0", "/dev/ttyACM1"]:
            if os.path.exists(port):
                motor_port = port
                break

    if os.path.exists("/dev/ttyKPATROL_ENCODER"):
        encoder_port = "/dev/ttyKPATROL_ENCODER"
    else:
        for port in ["/dev/ttyUSB0", "/dev/ttyUSB1"]:
            if os.path.exists(port):
                encoder_port = port
                break

    if os.path.exists("/dev/ttyKPATROL_GPS"):
        gps_port = "/dev/ttyKPATROL_GPS"
    else:
        # Pick the first ttyUSB* not already claimed by the encoder.
        for port in ["/dev/ttyUSB1", "/dev/ttyUSB2", "/dev/ttyUSB0"]:
            if os.path.exists(port) and port != encoder_port:
                gps_port = port
                break

    return motor_port, encoder_port, gps_port


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
    motor_port, encoder_port, gps_port = find_serial_ports()

    robot_serial = os.environ.get('ROBOT_SERIAL', 'KPATROL-001')
    logger.info(f"Robot Serial: {robot_serial}")
    mqtt_config = MQTTConfig(robot_serial=robot_serial)
    mqtt_config.validate()
    serial_config = SerialConfig(motor_port=motor_port, encoder_port=encoder_port)

    # GPS toggle via env: KPATROL_GPS_ENABLED=0 to skip outdoor mode entirely.
    gps_enabled = os.environ.get("KPATROL_GPS_ENABLED", "1") not in ("0", "false", "False")
    # GPS wiring mode: "shared" (NEO-6M on ESP32-S3 UART2 → forwarded over
    # motor serial as `NMEA:...`) or "dedicated" (own USB-TTL adapter).
    gps_mode = os.environ.get("KPATROL_GPS_MODE", "shared").strip().lower()
    if gps_mode not in ("shared", "dedicated"):
        logger.warning(f"[GPS] Unknown KPATROL_GPS_MODE={gps_mode!r}, falling back to 'shared'")
        gps_mode = "shared"
    gps_config = GPSConfig(port=gps_port, enabled=gps_enabled, mode=gps_mode)

    client = KPatrolMQTTV5(mqtt_config, serial_config, gps_config)
    client.setup_mqtt()

    if client.connect():
        client.run()
    else:
        logger.error("[Main] Failed to connect, exiting...")
        sys.exit(1)


if __name__ == "__main__":
    main()
