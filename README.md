<p align="center">
  <img src="docs/brand/logo.png" alt="K-Patrol" width="140" />
</p>

<h1 align="center">K-Patrol — Robot Workspace (Robot_WS)</h1>

<p align="center">
  <em>On-robot firmware, Raspberry Pi 4B controller, and supporting tools for the K-Patrol indoor patrol platform.</em><br/>
  <em>Mã firmware ESP32, controller chạy trên Raspberry Pi 4B, và các công cụ hỗ trợ cho robot tuần tra K-Patrol.</em>
</p>

<p align="center">
  <a href="https://www.espressif.com/en/products/socs/esp32-s3"><img alt="ESP32-S3" src="https://img.shields.io/badge/ESP32--S3-Arduino-e7352c?logo=espressif" /></a>
  <a href="https://www.raspberrypi.com/products/raspberry-pi-4-model-b/"><img alt="Raspberry Pi 4B" src="https://img.shields.io/badge/Raspberry%20Pi-4B-A22846?logo=raspberrypi" /></a>
  <a href="https://www.python.org"><img alt="Python" src="https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python" /></a>
  <a href="https://mqtt.org"><img alt="MQTT" src="https://img.shields.io/badge/MQTT-5-660066?logo=mqtt" /></a>
  <a href="https://opencv.org"><img alt="OpenCV" src="https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv" /></a>
  <a href="https://onnxruntime.ai"><img alt="ONNX Runtime" src="https://img.shields.io/badge/ONNX-Runtime-005CED?logo=onnx" /></a>
  <a href="#license"><img alt="License" src="https://img.shields.io/badge/license-MIT-green" /></a>
  <a href="CHANGELOG.md"><img alt="Release" src="https://img.shields.io/badge/release-v1.0.0-blue" /></a>
  <a href="https://github.com/KPatrol/Robot_WS"><img alt="Repo" src="https://img.shields.io/badge/repo-Robot_WS-181717?logo=github" /></a>
</p>

<p align="center">
  <a href="#overview--tổng-quan">Overview</a> ·
  <a href="#hardware-architecture">Hardware</a> ·
  <a href="#layout">Layout</a> ·
  <a href="#firmware-esp32-s3--d1-dev--d1-r32">Firmware</a> ·
  <a href="#pi-controller">Pi Controller</a> ·
  <a href="#detection--alarm">Detection</a> ·
  <a href="#getting-started">Getting Started</a> ·
  <a href="#roadmap">Roadmap</a> ·
  <a href="CHANGELOG.md">Changelog</a> ·
  <a href="#license">License</a>
</p>

---

## Overview / Tổng quan

The `robots/` workspace is the on-vehicle stack of the K-Patrol indoor patrol robot — everything that runs on the robot itself rather than on the cloud / operator side. It pairs:

- **Three coordinated ESP32 boards** that close the low-level control loops (motors, encoders, peripherals) and expose UART command protocols.
- **A Raspberry Pi 4B controller (Python)** that bridges UART ↔ MQTT v5, runs the patrol scripts, dual-pipeline anomaly detection (HSV + YOLO), an MJPEG camera streamer, and the hybrid GPS resolver.
- **Wiring docs, motor / IMU / GPS / ToF test sketches, and integration tests** used during the build of the K-Patrol V10 prototype.

This repository is published as **Robot_WS** so it can be flashed / deployed on the robot without pulling the rest of the stack.

> 🎓 **Academic context:** This stack is the on-robot half of the graduation thesis *"Phát triển hệ sinh thái tuần tra và giám sát thông minh tích hợp AIoT — K-Patrol"* by Vũ Đăng Khoa, Phenikaa University (MSSV 22010357, K16, CNTT4).

---

## Hardware Architecture

### Three-board topology (V10.3)

```
┌─────────────────────────────────────────────────────────────────┐
│  Raspberry Pi 4B (4 GB)                                         │
│  ├─ Python 3.10 controller stack                                │
│  ├─ MQTT v5 bridge · MJPEG streamer · YOLOv8n inference         │
│  └─ TwinLiteNet+ ONNX lane segmentation                         │
└──┬────────────────┬────────────────┬────────────────┬───────────┘
   │ USB serial     │ USB serial     │ USB serial     │ CSI/USB
   │ /dev/ttyKP_    │ /dev/ttyKP_    │ /dev/kpatrol-  │ camera
   │ MOTOR          │ ENCODER        │ periph         │
┌──▼──────────┐  ┌──▼──────────┐  ┌──▼──────────┐
│ ESP32-S3    │  │ ESP32 Dev   │  │ ESP32       │
│ (motor)     │  │ (encoder)   │  │ D1 R32 v2   │
│  Mecanum +  │  │  4× quad +  │  │ Relay +     │
│  BTS7960 ×4 │  │  6× VL53L0X │  │ buzzer +    │
│  + BNO08x   │  │  + INA219   │  │ lamp +      │
│  + NEO-6M   │  │  via TCA9548│  │ OLED eyes   │
└─────────────┘  └─────────────┘  └─────────────┘
```

### Power distribution (3-buck architecture)

| Rail | Source → Output | Loads |
|------|-----------------|-------|
| Buck #1 | 12 V → 5 V | Raspberry Pi 4B · 4G modem |
| Buck #2 | 12 V → 5 V | ESP32-S3 (motor) · ESP32 Dev (encoder) · BTS7960 logic |
| Buck #3 | 12 V regulated | Warning lamp · relay coil · buzzer · status LED |
| Direct  | 14.6 V LFP 4S (10 Ah) → 20 A fuse | DC motor power (BTS7960 power rail) |
| MCU headers | — | VL53L0X, IMU, GPS, DHT11 via on-board LDO/3V3 rail |

> ⚠️ **No 3V3 buck**: Sensors share the MCU header's regulated 3V3 rail. Avoids ground-loop issues common with isolated low-current rails.

### Bill of materials (key parts)

| Subsystem | Component | Notes |
|-----------|-----------|-------|
| Compute | Raspberry Pi 4B (4 GB) | Pi-side controller, MQTT bridge, camera, YOLO/ONNX |
| MCU motor | ESP32-S3 (CH9102 USB) | Mecanum controller, 5 × BTS7960 H-bridges (4 motors + brake) |
| MCU encoder | ESP32 Dev (CH340G) | Reads 4× quadrature encoders, broadcasts ENC_HB + TOF |
| MCU peripheral | ESP32 D1 R32 v2.0 (CH340G) | Relay 5V, buzzer, lamp, OLED SH1106 eyes |
| Motors | 4× DC + BTS7960 H-bridges | Mecanum drive (FL · FR · BL · BR) |
| Encoders | 4× quadrature, channel A only | ×2 counts per rev (corrected from ×4) |
| IMU | BNO08x | I²C primary, UART fallback |
| GPS | NEO-6M (UART) + ip-api.com (network fallback) | Hybrid resolver, indoor-friendly |
| ToF | 6× VL53L0X via TCA9548A I²C mux | Lane keeping + obstacle avoidance |
| Battery monitor | INA219 (I²C 0x40) | Primary; ADC1_CH2 fallback |
| Camera | Pi Camera Module v2 / USB | MJPEG streamer at 640×480 |
| Power | 14.6 V LFP 4S 10 Ah + 3× DC-DC buck | 12V→5V × 2, 12V regulated × 1 |

See [docs/HARDWARE_PERIPHERALS.md](docs/HARDWARE_PERIPHERALS.md) and [docs/HARDWARE_PERIPHERALS_QUICK.md](docs/HARDWARE_PERIPHERALS_QUICK.md) for the wiring, I²C address map, and MQTT topic table.

---

## Layout

```
robots/
├── firmware/                                # Arduino sketches (3 board families + tests)
│   ├── controller_four_motors_mecanum/        # PROD: ESP32-S3 mecanum + BNO08x + GPS + INA219
│   ├── encoder_reader_4motors_v5/             # PROD: ESP32 Dev — 4 encoders + ToF + heartbeat
│   ├── encoder_reader_4motors_v4/             # legacy (kept for diff baseline)
│   ├── peripheral_hub_d1r32/                  # PROD: D1 R32 — relay/buzzer/lamp/OLED eyes
│   ├── battery_monitor_s3/                    # INA219 bench validator
│   ├── test_ina219_s3/                        # INA219 unit test
│   ├── tof_debug/                             # TCA9548A + 6× VL53L0X scanner
│   ├── i2c_scanner_only/                      # Bus discovery
│   ├── test_*_motor*/                         # Motor bring-up (1/2/3 motor variants)
│   ├── test_bno08x_*/                         # IMU bring-up (I²C + UART)
│   └── test_gps_uart_sniffer/                 # NEO-6M sniffer
├── pi-controller/                           # Raspberry Pi 4B controller (Python 3.10)
│   ├── kpatrol_mqtt_v5.py                     # MAIN: MQTT bridge + serial fan-out + alarm engine
│   ├── kpatrol_controller.py                  # Legacy single-board controller (still used by CLI)
│   ├── peripheral_hub.py                      # D1 R32 serial ↔ MQTT bridge
│   ├── gps_reader.py                          # NEO-6M + ip-api hybrid
│   ├── sensor_reader.py                       # IMU + auxiliary
│   ├── navigation/                            # Patrol + line follower + lane seg
│   │   ├── simple_line_follower.py              # V5.15c9 Mecanum (vx+wz, vy=0)
│   │   ├── line_follower.py                     # V5.15c3 broad-envelope yellow tape
│   │   ├── nav_controller.py · script_patrol.py
│   │   └── lane_seg/                            # TwinLiteNet+ ONNX runtime
│   ├── detection/                             # Detection + alert bridge
│   │   ├── anomaly_detector.py                  # V10.4 HSV + V11.0 YOLO dual pipeline
│   │   ├── alert_bridge.py · test_fire_detection.py
│   │   └── service/ · datasets/
│   ├── safety/                                # Alert actuator + alarm rule engine
│   │   ├── alert_actuator.py                    # V5.15c10 relay protection (auto-clear)
│   │   ├── alarm_controller.py                  # Operator-configurable rule engine
│   │   ├── battery_watcher.py · tipover_watcher.py
│   ├── tools/                                 # Bench + calibration + diagnostics
│   │   ├── calibrate_fire.py                    # Realtime HSV trackbar UI
│   │   ├── download_fire_model.py               # Multi-source YOLO downloader
│   │   ├── test_fire_model.py                   # Webcam YOLO vs HSV compare
│   │   ├── detection_smoke_test.py · diag_ports.sh · test_avoid_fsm.py
│   ├── models/                                # ML weights (gitignored; README tracked)
│   ├── data/                                  # Grid + waypoints + maps
│   ├── *.service · 99-kpatrol-serial.rules    # systemd + udev
│   └── setup.sh · run.sh
├── camera-stream/                           # MJPEG server + route recorder
├── docs/                                    # Wiring + pinout + brand
└── images/                                  # Reference photos
```

---

## Firmware (ESP32-S3 + D1 Dev + D1 R32)

Each folder under [firmware/](firmware/) is a self-contained Arduino sketch.

| Sketch | Board | Role |
|--------|-------|------|
| `controller_four_motors_mecanum` | ESP32-S3 (CH9102) | Production mecanum motor controller + INA219 battery |
| `encoder_reader_4motors_v5` | ESP32 Dev (CH340G) | Production encoder reader + ToF + heartbeat |
| `peripheral_hub_d1r32` | ESP32 D1 R32 v2.0 | Production relay/buzzer/lamp/OLED hub |
| `battery_monitor_s3` · `test_ina219_s3` | ESP32-S3 | INA219 bench tests |
| `tof_debug` | any | 6× VL53L0X TCA9548A scanner |
| `test_*` | various | Motor / IMU / GPS bring-up |

### Flashing

```bash
# arduino-cli (or Arduino IDE 2.x)
arduino-cli compile  --fqbn esp32:esp32:esp32s3 firmware/controller_four_motors_mecanum
arduino-cli upload   --fqbn esp32:esp32:esp32s3 -p /dev/ttyKPATROL_MOTOR firmware/controller_four_motors_mecanum

arduino-cli compile  --fqbn esp32:esp32:esp32dev firmware/encoder_reader_4motors_v5
arduino-cli upload   --fqbn esp32:esp32:esp32dev -p /dev/ttyKPATROL_ENCODER firmware/encoder_reader_4motors_v5
```

The udev rule [pi-controller/99-kpatrol-serial.rules](pi-controller/99-kpatrol-serial.rules) gives stable `/dev/ttyKPATROL_*` symlinks (S3 CH9102 + Dev CH340G slot 1-1.3 + D1 R32 slot 1-1.2).

> ⚠️ **S3 DTR reset trap:** The CH9102 USB-Serial bridge ties DTR → EN on the S3 dev board. The Pi-side code sets `dtr=False, rts=False` before `Serial.open()` so the firmware is not reset on every reconnect.

---

## Pi Controller

The Raspberry Pi 4B hosts a Python 3.10 controller stack that:

1. Bridges UART (3 serial ports) ↔ **MQTT v5** (`kpatrol_mqtt_v5.py`).
2. Runs the dual-pipeline anomaly detector (YOLO + HSV) with snapshot fan-out.
3. Hosts the operator-configurable alarm rule engine + actuator with relay protection.
4. Resolves position via hybrid GPS (NEO-6M outdoor, ip-api.com indoor).
5. Streams MJPEG over HTTP for the operator console.
6. Optionally runs TwinLiteNet+ ONNX lane segmentation (~12.6 FPS on Pi 4B @ 192×320).

### Install on a Pi

```bash
cd pi-controller
chmod +x setup.sh
./setup.sh                       # installs system + Python deps, copies udev rules

# Edit env files:
cp config.env config.env.local && $EDITOR config.env.local
cp mqtt.env mqtt.env.local       && $EDITOR mqtt.env.local

./run.sh                         # one-shot run
sudo systemctl enable --now kpatrol-mqtt.service kpatrol-detection.service kpatrol-peripheral.service
```

See [pi-controller/DEPLOY_CHECKLIST.md](pi-controller/DEPLOY_CHECKLIST.md) for the full deploy + smoke-test runbook.

---

## Detection & Alarm

### Fire detection — V10.4 (default) + V11.0 (opt-in)

The 6+1 stage HSV pipeline (default `KPATROL_FIRE_MODE=hsv`) is tightened against indoor false positives:

| Stage | Purpose |
|-------|---------|
| 0 | Outer envelope: red wraps + yellow band, S ≥ 80, V ≥ 200 |
| 1a | Hot core: S ≥ 30 (rejects skin/specular highlights) |
| 1b | Yellow-vs-wrap-red ratio gate ≥ 0.4 (rejects orange-pink skin) |
| 2 | Spatial overlap of core ∩ outer |
| 3 | Area: ≥ 92 px (small/distant flames) |
| 4 | Temporal flicker: max_static_IoU = 0.97 (lenient) |
| 5 | Shape: elongation + solidity gate |

V11.0 swaps Stage 0-1 for a YOLOv8n detector when `KPATROL_FIRE_MODE=yolo` and a model exists in `pi-controller/models/`. Default falls back to V10.4 HSV because outdoor-trained models don't yet generalise to indoor demos.

```bash
# Calibrate live (trackbars)
python3 pi-controller/tools/calibrate_fire.py

# Download a YOLO model (optional)
python3 pi-controller/tools/download_fire_model.py --source dfire-v8n

# Webcam compare (YOLO vs HSV side-by-side)
python3 pi-controller/tools/test_fire_model.py
```

### Alarm engine — V5.15c9

[pi-controller/safety/alarm_controller.py](pi-controller/safety/alarm_controller.py) consumes 9 event types and emits MQTT `/alarm/triggered` with per-rule notification routing (handed to the backend NotificationService):

```
person · fire · battery_low · battery_critical · tipover ·
schedule · system_error · any_safety · manual_test
```

Each rule carries `continuousDuration` debounce + a time-of-day window. The actuator (V5.15c10) protects the mechanical relay with `auto_clear_light_after_sec = 4.0` and forces battery alerts to buzzer-only.

---

## Getting Started

```bash
git clone https://github.com/KPatrol/Robot_WS.git
cd Robot_WS

# 1. Flash 3 boards (see "Firmware" above):
#    - ESP32-S3 motor on /dev/ttyKPATROL_MOTOR
#    - ESP32 Dev encoder on /dev/ttyKPATROL_ENCODER
#    - D1 R32 peripheral hub on /dev/kpatrol-periph
# 2. Bring up the Pi:
cd pi-controller && ./setup.sh && ./run.sh
# 3. Stream camera:
cd ../camera-stream && python3 mjpeg_server.py
# 4. Tail logs:
journalctl -u kpatrol-mqtt -u kpatrol-detection -f
```

---

## Roadmap

Active development continues — this section tracks what's queued next so contributors can pick up.

### v1.1 (next minor)
- [ ] V11.1: HUST-Roboflow Vietnamese indoor-fire YOLO model (replacing forest-fire baseline)
- [ ] V5.16: Stuck-detection FSM (commanded-vs-actual delta + recovery routine)
- [ ] Two-wheel differential mode toggle (for non-mecanum chassis variants)
- [ ] Pi 5 8GB profile: bench TwinLiteNet+ Small at 256×416 @ 192×320 quantisation

### v1.2
- [ ] Hot-pluggable peripheral discovery (no fixed udev slot assignment)
- [ ] OTA firmware update over MQTT (signed payloads)
- [ ] On-robot ArUco-based docking + auto-charge return
- [ ] Local SQLite cache when broker is unreachable (replay on reconnect)

### Long term
- [ ] ROS 2 Humble bridge (telemetry passthrough to RViz)
- [ ] LIDAR option (2D RPLIDAR + cartographer SLAM)
- [ ] Voice intercom (push-to-talk via Opus + WebRTC)

Contributions welcome — open an issue first if the scope is non-trivial.

---

## Branch hygiene

- **`main`** — production trunk; current release tag `v1.0.0`.
- **`dev`** — integration branch tracked from `main`.
- Feature branches: `feat/<topic>`, hotfixes: `fix/<topic>`.

---

## Documentation

- [pi-controller/DEPLOY_CHECKLIST.md](pi-controller/DEPLOY_CHECKLIST.md) — production deploy + smoke test
- [docs/HARDWARE_PERIPHERALS.md](docs/HARDWARE_PERIPHERALS.md) — full peripheral hub wiring + I²C map
- [docs/HARDWARE_PERIPHERALS_QUICK.md](docs/HARDWARE_PERIPHERALS_QUICK.md) — bench bring-up cheat sheet
- [docs/MECANUM_SETUP_GUIDE.md](docs/MECANUM_SETUP_GUIDE.md)
- [docs/MOUNTING_GUIDE_IMU_TOF.md](docs/MOUNTING_GUIDE_IMU_TOF.md)
- [docs/RELAY_LIGHT_WIRING_GUIDE.md](docs/RELAY_LIGHT_WIRING_GUIDE.md)
- Pinout diagrams: [mecanum 4×](docs/PINOUT_DIAGRAM_MECANUM_FOUR_MOTORS.txt) · [encoder](docs/ENCODER_WIRING_4MOTORS.txt)
- [CHANGELOG.md](CHANGELOG.md) — release history

---

## Related repositories

| Component | Repo |
|-----------|------|
| Backend API + WebSocket | [KPatrol_Backend](https://github.com/KPatrol/KPatrol_Backend) |
| Operator PWA | [KPatrol_MobileApp](https://github.com/KPatrol/KPatrol_MobileApp) |
| Marketing site | [KPatrol_WebCommerce](https://github.com/KPatrol/KPatrol_WebCommerce) |

---

## Author

**Vũ Đăng Khoa** · MSSV 22010357 · K16, Lớp CNTT4
Khoa Hệ thống Thông tin · Trường Công nghệ Thông tin · Phenikaa University
✉️ khoa.vu@alphaasimov.com

---

## License

MIT License — © K-Patrol / Vu Dang Khoa, 2026.
