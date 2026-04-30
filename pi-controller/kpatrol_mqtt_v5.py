#!/usr/bin/env python3
"""
kpatrol_mqtt_v5.py — K-Patrol MQTT Controller V5
=================================================
Directional safety + scripted autonomous patrol.

Navigation modes (NavController):
  • MANUAL        — operator drives via kpatrol/{serial}/motor
  • SCRIPT_PATROL — executes a declarative patrol script (rotate, forward_time,
                    strafe_time, pause, …) with IMU-closed-loop rotation and
                    ToF emergency reflex
  • LINE_FOLLOW   — camera floor-line PD following (BEV homography + HSV)
  • FREE_COVERAGE — autonomous random-walk coverage with ToF wall-bounce
  • EMERGENCY     — motors stopped, waiting for operator clear

MQTT topics (scoped to kpatrol/{serial}/…):
  Subscribe:
    nav_command    {"mode": "MANUAL|SCRIPT_PATROL|EMERGENCY",
                    "speed": 60,
                    "action": "clear_emergency"}
    script_command {"action": "list|save|delete|load|start|stop",
                    "name":   "<script_name>",
                    "script": { ...PatrolScript dict... }}
  Publish:
    nav_status     Executor state, current step, progress, errors
    script_list    Available scripts with metadata
    script_status  Result of last script_command action

Hardware:
  ESP32-S3  → /dev/ttyKPATROL_MOTOR   (motors + BNO08x IMU)
  ESP32     → /dev/ttyKPATROL_ENCODER (4 encoders + 6x VL53L0X ToF)
"""

import json
import time
import sys
import os
import re
import threading
from typing import Optional, Dict, Any, Tuple, List
from dataclasses import dataclass, asdict, field
from enum import Enum

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
    print(f"[GPS] Module unavailable: {_gps_err} — outdoor mode disabled")
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
    print(f"[DETECT] Module unavailable: {_det_err} — anomaly detection disabled")
    AnomalyDetector = None    # type: ignore
    DetectionConfig = None    # type: ignore
    DetectionEvent = None     # type: ignore
    AlertStore = None         # type: ignore
    _DETECTION_AVAILABLE = False


# ==================== CONFIGURATION ====================

@dataclass
class MQTTConfig:
    host: str = "103.81.84.43"
    port: int = 1883
    username: str = "alphaasimov2024"
    password: str = "gvB3DtGfus6U"
    client_id: str = f"kpatrol_pi_{os.uname().nodename}"
    keepalive: int = 60
    qos: int = 1
    # Robot serial number — used as MQTT topic namespace
    # Override via env var ROBOT_SERIAL or config.env KPATROL_ROBOT_SERIAL
    robot_serial: str = os.environ.get("ROBOT_SERIAL", "KPATROL-001")


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
        # V5: Navigation (MANUAL / SCRIPT_PATROL / EMERGENCY)
        self.NAV_CMD       = f"{p}/nav_command"
        self.NAV_STATUS    = f"{p}/nav_status"
        # Scripted patrol management
        self.SCRIPT_CMD    = f"{p}/script_command"
        self.SCRIPT_LIST   = f"{p}/script_list"
        self.SCRIPT_STATUS = f"{p}/script_status"
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


# ==================== SAFETY (Directional) ====================

class SafetyZone(Enum):
    SAFE = "safe"
    SLOW = "slow"
    CAUTION = "caution"
    DANGER = "danger"

@dataclass
class SafetyConfig:
    enabled: bool = True
    danger_distance: int = 300     # compensate for cheap sensor latency (~200ms)
    caution_distance: int = 450    # slow down earlier for safe braking
    slow_distance: int = 600
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

    def to_dict(self) -> Dict:
        return {
            "front": self.front, "front_left": self.front_left,
            "front_right": self.front_right, "left": self.left,
            "right": self.right, "back": self.back,
            "timestamp": self.timestamp
        }

    def get_min_distance(self) -> int:
        return min(self.front, self.front_left, self.front_right,
                   self.left, self.right, self.back) if self.front > 0 else 9999

    def get_direction_distances(self) -> Dict[str, int]:
        return {
            "forward":  min(self.front, self.front_left, self.front_right) if self.front > 0 else 9999,
            "backward": self.back if self.back > 0 else 9999,
            "left":     min(self.left, self.front_left) if self.left > 0 else 9999,
            "right":    min(self.right, self.front_right) if self.right > 0 else 9999,
        }

    def to_float_dict(self) -> Dict[str, float]:
        """Convert to float dict for navigation use."""
        return {
            "front": float(self.front), "front_left": float(self.front_left),
            "front_right": float(self.front_right), "left": float(self.left),
            "right": float(self.right), "back": float(self.back),
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
    """Directional Safety Controller — per-direction zone evaluation."""

    def __init__(self, config: SafetyConfig = None):
        self.config = config or SafetyConfig()
        self.tof_data = ToFData()
        self.current_zone = SafetyZone.SAFE
        self.direction_zones: Dict[str, SafetyZone] = {
            "forward": SafetyZone.SAFE, "backward": SafetyZone.SAFE,
            "left": SafetyZone.SAFE, "right": SafetyZone.SAFE,
        }
        self.direction_distances: Dict[str, int] = {
            "forward": 9999, "backward": 9999, "left": 9999, "right": 9999,
        }
        self.lock = threading.Lock()

    def update_tof(self, front: int, front_left: int, front_right: int,
                   left: int, right: int, back: int):
        with self.lock:
            self.tof_data.front = front
            self.tof_data.front_left = front_left
            self.tof_data.front_right = front_right
            self.tof_data.left = left
            self.tof_data.right = right
            self.tof_data.back = back
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

    def is_command_safe(self, cmd: str) -> bool:
        if not self.config.enabled:
            return True
        cmd_upper = cmd.strip().upper()
        for direction in CMD_REQUIRED_DIRECTIONS.get(cmd_upper, []):
            if self.direction_zones.get(direction) == SafetyZone.DANGER:
                return False
        return True

    def get_command_speed_multiplier(self, cmd: str) -> float:
        if not self.config.enabled:
            return 1.0
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
        return self._zone_multiplier(self.current_zone)

    def should_stop(self) -> bool:
        return self.config.enabled and self.current_zone == SafetyZone.DANGER

    def get_status(self) -> Dict:
        with self.lock:
            return {
                "enabled": self.config.enabled,
                "zone": self.current_zone.value,
                "speed_multiplier": self.get_speed_multiplier(),
                "min_distance": self.tof_data.get_min_distance(),
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
        print(f"[Safety] {'Enabled' if enabled else 'Disabled'}")


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
        # V5.2: optional sink for NMEA frames forwarded by ESP32 firmware
        # (lines prefixed with "NMEA:" arrive on the motor UART0).
        self._gps_sink = None

    def set_gps_sink(self, sink) -> None:
        """Register a callback (e.g. GPSReader.feed_nmea) to receive NMEA
        sentences forwarded by the motor firmware via shared-mode pass-through."""
        self._gps_sink = sink

    def connect(self) -> bool:
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1.0)
            self.connected = True
            print(f"[Motor] Connected to {self.port}")
            time.sleep(2)
            return True
        except serial.SerialException as e:
            print(f"[Motor] Connection failed: {e}")
            self.connected = False
            return False

    def _try_reconnect(self) -> bool:
        if time.time() - self._last_reconnect_attempt < self._reconnect_interval:
            return False
        self._last_reconnect_attempt = time.time()
        print(f"[Motor] Attempting reconnection to {self.port}...")
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
                print(f"[Motor] Send error: {e}")
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
                            print(f"[Motor] GPS sink error: {e}")
                        return None
                    self._parse_imu_line(line)
                    return line
            except (OSError, serial.SerialException) as e:
                print(f"[Motor] Read error: {e}")
                self.connected = False
        return None

    def _parse_imu_line(self, line: str):
        if line.startswith("IMU:"):
            try:
                parts = line[4:].split(',')
                if len(parts) >= 4:
                    self.imu_data.yaw = float(parts[0])
                    self.imu_data.pitch = float(parts[1])
                    self.imu_data.roll = float(parts[2])
                    self.imu_data.accuracy = float(parts[3])
                    self.imu_data.timestamp = int(time.time() * 1000)
            except (ValueError, IndexError):
                pass

    def get_imu_status(self) -> Dict:
        return self.imu_data.to_dict()

    def request_imu(self):
        self.send_command("IMU")


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

    def set_tof_callback(self, callback):
        self.tof_callback = callback

    def connect(self) -> bool:
        try:
            if self.serial and self.serial.is_open:
                self.serial.close()
            self.serial = serial.Serial(port=self.port, baudrate=self.baudrate, timeout=1.0)
            self.connected = True
            print(f"[Encoder] Connected to {self.port}")
            time.sleep(2)
            if not self.running:
                self.running = True
                self.read_thread = threading.Thread(target=self._read_loop, daemon=True)
                self.read_thread.start()
            return True
        except serial.SerialException as e:
            print(f"[Encoder] Connection failed: {e}")
            self.connected = False
            return False

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
                    print(f"[Encoder] Read error: {e}")
                    self.connected = False
            time.sleep(0.05)

    def _parse_line(self, line: str):
        pattern = r'(\w+):\s*Count=(-?\d+)\s+Rev=(-?[\d.]+)\s+RPM=(-?[\d.]+)'
        match = re.match(pattern, line)
        if match:
            motor = match.group(1)
            if motor in self.encoder_data:
                self.encoder_data[motor].count = int(match.group(2))
                self.encoder_data[motor].revolutions = float(match.group(3))
                self.encoder_data[motor].rpm = float(match.group(4))
        elif line.startswith("TOF:"):
            self._parse_tof_line(line)

    def _parse_tof_line(self, line: str):
        try:
            parts = line[4:].split(',')
            if len(parts) >= 6:
                vals = [int(p) for p in parts[:6]]
                if self.tof_callback:
                    self.tof_callback(*vals)
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
        print(f"[MQTT] Robot serial: {mqtt_config.robot_serial}")
        print(f"[MQTT] Topic namespace: kpatrol/{mqtt_config.robot_serial}/#")

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
                print(f"[GPS] Shared mode — NMEA piggybacks on motor UART")
            else:
                print(f"[GPS] Dedicated mode — port {self.gps_config.port}")

        # Connect ToF callback
        self.encoder_reader.set_tof_callback(self.safety_controller.update_tof)

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
                print(f"[FC] map persistence enabled → {_map_path}")
            except OSError as exc:
                print(f"[FC] map persistence init failed: {exc} — running without persistence")
                _fc_cfg = None
        self._nav = NavController(script_dir="data/scripts", fc_config=_fc_cfg)
        self._last_nav_cmd = ""
        self._nav_lock = threading.Lock()

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
        if self._det_enabled:
            try:
                _det_cam_idx = int(os.environ.get("KPATROL_DETECTION_CAMERA", "1"))
                if _det_cam_idx == self._camera_index and self._camera_enabled:
                    print(f"[DETECT] WARNING: detector camera idx ({_det_cam_idx}) "
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
                    print(f"[DETECT] AnomalyDetector ready (camera idx {_det_cam_idx})")
            except Exception as exc:
                print(f"[DETECT] init failed: {exc} — detection disabled")
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
            print(f"[VPID] enabled — max {self._vpid_max_mps:.2f} m/s, "
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
            print("[MQTT] Warning: No serial devices connected!")

        # GPS is best-effort: a failed connect does not abort the bridge.
        # The reader thread will retry every 3s, so an antenna replug
        # mid-mission recovers automatically.
        if self.gps_reader is not None:
            if not self.gps_reader.connect():
                print("[GPS] Initial connect failed — will retry in background")

        try:
            self.client.connect(self.mqtt_config.host, self.mqtt_config.port, self.mqtt_config.keepalive)
            return True
        except Exception as e:
            print(f"[MQTT] Connection failed: {e}")
            return False

    # ── GPS callbacks ───────────────────────────────────────────────

    def _on_gps_fix_acquired(self, data) -> None:
        msg = (
            f"GPS fix acquired: {data.satellites} sats, "
            f"HDOP={data.hdop:.1f}, "
            f"({data.latitude:.6f}, {data.longitude:.6f})"
        )
        print(f"[GPS] {msg}")
        # Defer publish_log until MQTT is connected — connect() is the path
        # that initialises everything, and the LWT may not be set yet.
        if self.client is not None and self.client.is_connected():
            self.publish_log(msg)

    def _on_gps_fix_lost(self) -> None:
        msg = "GPS fix lost — falling back to encoder/IMU dead-reckoning"
        print(f"[GPS] {msg}")
        if self.client is not None and self.client.is_connected():
            self.publish_log(msg)

    # ── MQTT Callbacks ──────────────────────────────────────────────

    def on_connect(self, client, userdata, flags, rc, properties=None):
        reason_code = rc.value if hasattr(rc, 'value') else rc
        if reason_code == 0:
            print("[MQTT] Connected successfully!")
            T = self.T
            # Subscribe using wildcard: kpatrol/{serial}/#
            # This covers ALL command topics in a single subscription.
            client.subscribe(T.WILDCARD, 1)
            print(f"[MQTT] Subscribed to {T.WILDCARD}")
            self.send_heartbeat("online")
            self.publish_log(f"K-Patrol V5 connected | serial={T.serial}")
        else:
            print(f"[MQTT] Connect failed with code: {reason_code}")

    def on_disconnect(self, client, userdata, disconnect_flags=None, rc=None, properties=None):
        reason = rc.value if hasattr(rc, 'value') else (rc if rc is not None else 0)
        print(f"[MQTT] Disconnected (rc={reason})")

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
            elif topic == T.SCRIPT_CMD:
                self.handle_script_command(payload)
            elif topic == T.ODOM_RESET:
                self.handle_odom_reset(payload)
        except json.JSONDecodeError:
            print(f"[MQTT] Invalid JSON: {msg.payload}")
        except Exception as e:
            print(f"[MQTT] Error: {e}")

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
            print(f"[Safety] Blocking {cmd_type} (safe: {alt_str})")
            self.publish_log(f"Safety blocked: {cmd_type} (try: {alt_str})")
            self.motor_controller.send_command("S")
            return

        if speed is not None:
            multiplier = self.safety_controller.get_command_speed_multiplier(cmd_type)
            self.motor_controller.set_speed(int(speed * multiplier))

        self.motor_controller.send_command(cmd_type)
        self._nav.record_feed(
            cmd_type,
            self.motor_controller.current_speed,
            self.motor_controller.imu_data.yaw,
        )

    def handle_motor_command(self, payload: Dict[str, Any]):
        # Refuse manual control while autonomous script is running
        if self._nav.get_mode() == "SCRIPT_PATROL":
            return
        cmd_type = payload.get("type", "")
        speed = payload.get("speed")
        if speed is not None:
            multiplier = self.safety_controller.get_speed_multiplier()
            self.motor_controller.set_speed(int(speed * multiplier))
        self.motor_controller.send_command(cmd_type)
        self._nav.record_feed(
            cmd_type,
            self.motor_controller.current_speed,
            self.motor_controller.imu_data.yaw,
        )

    def handle_speed(self, payload: Dict[str, Any]):
        speed = payload.get("speed", 150)
        multiplier = self.safety_controller.get_speed_multiplier()
        self.motor_controller.set_speed(int(speed * multiplier))

    def handle_emergency(self, payload: Dict[str, Any]):
        print("[EMERGENCY] Emergency stop!")
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

        {"mode": "MANUAL|SCRIPT_PATROL|EMERGENCY",
         "speed": 60,
         "action": "clear_emergency"}
        """
        if "speed" in payload:
            self._nav.set_speed(int(payload["speed"]))

        action = payload.get("action", "")
        if action == "clear_emergency":
            self._nav.clear_emergency()
            self.publish_log("Emergency cleared")

        if "mode" in payload:
            mode_str = str(payload["mode"]).upper()

            # Always stop motors FIRST before any mode switch
            self.motor_controller.send_command("S")
            self._last_nav_cmd = "S"
            self._lf_last_twist = None   # reset dedup cache for LINE_FOLLOW
            self._fc_last_twist = None   # reset dedup cache for FREE_COVERAGE

            if mode_str == "LINE_FOLLOW":
                # Ensure any running script is stopped before entering LINE_FOLLOW
                self._nav.script_stop()
                # Use dedicated start path that resets PID state
                result = self._nav.line_follow_start()
                if not result.get("ok"):
                    self.publish_log(f"LINE_FOLLOW start failed: {result.get('error','?')}")
                    return
                if not self._camera_enabled:
                    self.publish_log("WARN: camera disabled — LINE_FOLLOW mode is active but robot will not move")

            elif mode_str == "FREE_COVERAGE":
                # Stop any running script before entering FREE_COVERAGE
                self._nav.script_stop()
                result = self._nav.free_coverage_start()
                if not result.get("ok"):
                    self.publish_log(f"FREE_COVERAGE start failed: {result.get('error','?')}")
                    return

            else:
                if not self._nav.set_mode(mode_str):
                    self.publish_log(f"Unknown mode: {mode_str}")
                    return
            print(f"[Nav] Mode → {mode_str}")

            # Set ESP32 speed to match nav _speed for autonomous modes
            if mode_str != "MANUAL":
                esp_speed = int(self._nav._speed * 255 / 100)
                self.motor_controller.set_speed(esp_speed)
            else:
                print("[Nav] MANUAL — motors stopped")

    # ── Script command handler ─────────────────────────────────────

    def handle_script_command(self, payload: Dict[str, Any]):
        """Process script_command payload.

        {"action": "list|save|delete|load|start|stop",
         "name":   "<script_name>",
         "script": { ...PatrolScript dict... } }
        """
        action = payload.get("action", "")

        if action == "list":
            self._publish_script_list()
            return

        if action == "save":
            script_data = payload.get("script") or {}
            result = self._nav.script_save(script_data)
            print(f"[Script] Save → {result}")
            if result.get("ok"):
                self.publish_log(f"Script saved: {result['name']}")
                self._publish_script_list()
            else:
                self.publish_log(f"Script save failed: {result.get('error','?')}")
            self._pub(self.T.SCRIPT_STATUS, result)
            return

        if action == "delete":
            name = payload.get("name", "")
            ok = self._nav.script_delete(name)
            print(f"[Script] Delete {name} → {ok}")
            self._publish_script_list()
            self._pub(self.T.SCRIPT_STATUS, {"ok": ok, "action": "delete", "name": name})
            return

        if action == "load":
            name = payload.get("name", "")
            result = self._nav.script_load(name)
            print(f"[Script] Load {name} → {result}")
            self._pub(self.T.SCRIPT_STATUS, result)
            return

        if action == "start":
            name = payload.get("name")  # optional — use already-loaded otherwise
            # Ensure stop before starting
            self.motor_controller.send_command("S")
            self._last_nav_cmd = "S"
            result = self._nav.script_start(name)
            print(f"[Script] Start → {result}")
            self.publish_log(f"Script start: {result}")
            self._pub(self.T.SCRIPT_STATUS, result)
            return

        if action == "stop":
            self._nav.script_stop()
            self.motor_controller.send_command("S")
            self._last_nav_cmd = "S"
            print("[Script] Stop")
            self.publish_log("Script stopped")
            self._pub(self.T.SCRIPT_STATUS, {"ok": True, "action": "stop"})
            return

        if action == "record_start":
            name = payload.get("name", "")
            result = self._nav.record_start(name)
            result["action"] = "record_start"
            print(f"[Recorder] Start {name} → {result}")
            if result.get("ok"):
                self.publish_log(f"Recording started: {name}")
            self._pub(self.T.SCRIPT_STATUS, result)
            return

        if action == "record_stop":
            imu_yaw = self.motor_controller.imu_data.yaw
            result = self._nav.record_stop(imu_yaw)
            result["action"] = "record_stop"
            print(f"[Recorder] Stop → {result}")
            if result.get("ok"):
                self.publish_log(f"Recording saved: {result['name']} ({result['steps']} steps)")
                self._publish_script_list()
            else:
                self.publish_log(f"Recording failed: {result.get('error','?')}")
            self._pub(self.T.SCRIPT_STATUS, result)
            return

        if action == "record_cancel":
            result = self._nav.record_cancel()
            result["action"] = "record_cancel"
            print("[Recorder] Cancel")
            self.publish_log("Recording cancelled")
            self._pub(self.T.SCRIPT_STATUS, result)
            return

        if action == "record_status":
            status = self._nav.record_status()
            status["action"] = "record_status"
            self._pub(self.T.SCRIPT_STATUS, status)
            return

    def _publish_script_list(self):
        scripts = self._nav.script_list()
        self._pub(self.T.SCRIPT_LIST, {
            "scripts":   scripts,
            "timestamp": int(time.time() * 1000),
        })

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
            print(f"[ODOM] reset payload error: {exc}")

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

    def publish_status(self):
        # Retained: latest full snapshot is always available to new subscribers.
        self._pub(self.T.STATUS, {
            "connected": True,
            "esp32_motor": self.motor_controller.connected,
            "esp32_encoder": self.encoder_reader.connected,
            "battery": 85,
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
            print(f"[DETECT→NAV] {kind} conf={event.confidence:.2f} → {nav_result.get('action')}")
        except Exception as exc:
            print(f"[DETECT→NAV] on_alert error: {exc}")

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
                print(f"[DETECT] persist failed: {exc}")

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
                print(f"[DETECT] mark_synced failed: {exc}")

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
            print(f"[DETECT] publish failed: {exc}")
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
                print(f"[DETECT] drain read failed: {exc}")
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
                        print(f"[DETECT] mark_synced (drain) failed: {exc}")
                else:
                    break

    def publish_safety(self):
        self._pub(self.T.SAFETY, self.safety_controller.get_status())

    def publish_imu(self):
        self._pub(self.T.IMU, self.motor_controller.get_imu_status())

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
            imu_yaw = self.motor_controller.imu_data.yaw
            # Skip IMU when stale/uninitialized (timestamp 0 → encoder-only fallback)
            yaw_arg = float(imu_yaw) if self.motor_controller.imu_data.timestamp else None
            pose = self._odom.update(counts, imu_yaw_deg=yaw_arg)
            self._pub(self.T.ODOM, pose.to_dict())
        except Exception as exc:
            # Never let odometry kill the publish loop — log and continue.
            print(f"[ODOM] publish error: {exc}")

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
        Sends MEC: commands only while NavController is in LINE_FOLLOW mode.
        Safety veto applies: if front distance is DANGER the robot stops.
        Publishes JPEG overlay to kpatrol/{serial}/camera for dashboard view.
        """
        if not _CV2_AVAILABLE:
            print("[LF] cv2 not available — LINE_FOLLOW camera disabled")
            return

        cap = _cv2.VideoCapture(self._camera_index)
        cap.set(_cv2.CAP_PROP_FRAME_WIDTH,  640)
        cap.set(_cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(_cv2.CAP_PROP_FPS, 30)

        if not cap.isOpened():
            print(f"[LF] Camera {self._camera_index} failed to open — LINE_FOLLOW disabled")
            return

        print(f"[LF] Camera {self._camera_index} opened (640×480)")
        interval      = 0.05  # 20 Hz control loop
        cam_pub_interval = 0.20  # 5 Hz JPEG stream (MQTT bandwidth friendly)
        last_cam_pub  = 0.0

        while self.running:
            t0 = time.time()

            # Only process when in LINE_FOLLOW mode
            if self._nav.get_mode() != "LINE_FOLLOW":
                self._lf_last_twist = None
                time.sleep(interval)
                continue

            ret, frame = cap.read()
            if not ret:
                print("[LF] Frame capture failed")
                time.sleep(0.1)
                continue

            result = self._nav.line_follow_tick(frame, produce_overlay=True)

            vx, vy, wz, spd = result.vx, result.vy, result.wz, result.spd
            twist = (vx, vy, wz, spd)

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
                _, jpg = _cv2.imencode(".jpg", result.overlay, [_cv2.IMWRITE_JPEG_QUALITY, 70])
                self._pub_raw(self.T.CAMERA, jpg.tobytes())
                last_cam_pub = t0

            elapsed = time.time() - t0
            time.sleep(max(0.0, interval - elapsed))

        cap.release()
        print("[LF] Camera released")

    # ── V5: Navigation Loop (separate thread) ─────────────────────

    def _nav_loop(self):
        """Navigation tick loop — runs at 20 Hz (50 ms).

        Drives SCRIPT_PATROL and FREE_COVERAGE modes by polling
        NavController.tick() with fresh ToF + IMU readings.
        MANUAL is a no-op (operator drives via /motor).
        EMERGENCY forces a STOP.
        LINE_FOLLOW is handled by _line_follow_loop() instead.

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
            imu_yaw  = self.motor_controller.imu_data.yaw

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

            cmd, speed_pwm, twist, status = self._nav.tick(
                tof_dict, imu_yaw, encoder_counts=enc_counts, pose=nav_pose
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
        nav_thread = threading.Thread(target=self._nav_loop, name="nav_loop", daemon=True)
        nav_thread.start()

        # Start LINE_FOLLOW camera thread (no-op if cv2 absent or KPATROL_CAMERA=-1)
        if self._camera_enabled:
            lf_thread = threading.Thread(target=self._line_follow_loop, name="lf_loop", daemon=True)
            lf_thread.start()
            print(f"[LF] Camera thread started (device {self._camera_index})")
        else:
            print("[LF] Camera disabled — LINE_FOLLOW mode will not move motors")

        # V5.3: Start AnomalyDetector + alert backlog drainer (env-gated)
        if self._det_enabled and self._detector is not None:
            try:
                self._detector.start(blocking=False)
                self._alert_drain_thread = threading.Thread(
                    target=self._alert_drain_loop, name="alert_drain", daemon=True,
                )
                self._alert_drain_thread.start()
                print("[DETECT] anomaly detector + drainer started")
            except Exception as exc:
                print(f"[DETECT] start failed: {exc}")
        else:
            print("[DETECT] disabled (set KPATROL_DETECTION_ENABLED=1 to enable)")

        heartbeat_interval = 5
        status_interval = 2
        encoder_interval = 0.5
        safety_interval = 0.2
        sensors_interval = 0.5    # ToF sensor raw data at 2Hz
        imu_interval = 0.5
        metrics_interval = 10     # V5.1 link telemetry at 0.1Hz
        gps_interval = self.gps_config.publish_interval   # V5.2 default 1Hz
        odom_interval = 0.05      # V5.3 odometry at 20Hz

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

        print("\n" + "=" * 60)
        print("    K-PATROL MQTT CLIENT V5.0")
        print("    Directional Safety + Autonomous Navigation")
        print("=" * 60)
        print(f"MQTT Broker: {self.mqtt_config.host}:{self.mqtt_config.port}")
        print(f"Motor Port:  {self.serial_config.motor_port} ({'OK' if self.motor_controller.connected else 'FAIL'})")
        print(f"Encoder Port: {self.serial_config.encoder_port} ({'OK' if self.encoder_reader.connected else 'FAIL'})")
        if self.gps_reader is not None:
            gps_state = "OK" if self.gps_reader.connected else "FAIL"
            if self.gps_config.mode == "shared":
                print(f"GPS:         shared via motor UART ({gps_state}) — outdoor mode")
            else:
                print(f"GPS Port:    {self.gps_config.port} ({gps_state}) — outdoor mode")
        else:
            print("GPS:         disabled (indoor mode)")
        print(f"Safety: {'Enabled' if self.safety_controller.config.enabled else 'Disabled'} (Directional)")
        print(f"Nav Mode: {self._nav.get_mode()}")
        print("=" * 60)
        print("Press Ctrl+C to exit\n")

        try:
            while self.running:
                current_time = time.time()

                if current_time - last_heartbeat >= heartbeat_interval:
                    self.send_heartbeat()
                    last_heartbeat = current_time

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

                if current_time - last_imu_request >= 1.0:
                    self.motor_controller.request_imu()
                    last_imu_request = current_time

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

                response = self.motor_controller.read_response()
                if response and not response.startswith("IMU:"):
                    print(f"[Motor] {response}")

                time.sleep(0.05)

        except KeyboardInterrupt:
            print("\n[Main] Shutting down...")

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
                print(f"[DETECT] stop error: {exc}")

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
                print(f"[DETECT] alert_store close error: {exc}")
        print("[Main] V5 shutdown complete")


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


def main():
    print("\n" + "=" * 60)
    print("    K-PATROL MQTT CLIENT V5.0 FOR RASPBERRY PI")
    print("    Directional Safety + Autonomous Navigation")
    print("=" * 60 + "\n")

    os.makedirs("data", exist_ok=True)
    motor_port, encoder_port, gps_port = find_serial_ports()

    robot_serial = os.environ.get('ROBOT_SERIAL', 'KPATROL-001')
    print(f"Robot Serial: {robot_serial}")
    mqtt_config = MQTTConfig(robot_serial=robot_serial)
    serial_config = SerialConfig(motor_port=motor_port, encoder_port=encoder_port)

    # GPS toggle via env: KPATROL_GPS_ENABLED=0 to skip outdoor mode entirely.
    gps_enabled = os.environ.get("KPATROL_GPS_ENABLED", "1") not in ("0", "false", "False")
    # GPS wiring mode: "shared" (NEO-6M on ESP32-S3 UART2 → forwarded over
    # motor serial as `NMEA:...`) or "dedicated" (own USB-TTL adapter).
    gps_mode = os.environ.get("KPATROL_GPS_MODE", "shared").strip().lower()
    if gps_mode not in ("shared", "dedicated"):
        print(f"[GPS] Unknown KPATROL_GPS_MODE={gps_mode!r}, falling back to 'shared'")
        gps_mode = "shared"
    gps_config = GPSConfig(port=gps_port, enabled=gps_enabled, mode=gps_mode)

    client = KPatrolMQTTV5(mqtt_config, serial_config, gps_config)
    client.setup_mqtt()

    if client.connect():
        client.run()
    else:
        print("[Main] Failed to connect, exiting...")
        sys.exit(1)


if __name__ == "__main__":
    main()
