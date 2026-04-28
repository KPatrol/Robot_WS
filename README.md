<p align="center">
  <img src="docs/brand/logo.png" alt="K-Patrol" width="140" />
</p>

<h1 align="center">K-Patrol — Robot Workspace (Robot_WS)</h1>

<p align="center">
  <em>On-robot firmware, Raspberry Pi controller, and supporting tools for the K-Patrol indoor patrol platform.</em><br/>
  <em>Mã firmware ESP32, controller chạy trên Raspberry Pi 4, và các công cụ hỗ trợ cho robot tuần tra K-Patrol.</em>
</p>

<p align="center">
  <a href="https://www.espressif.com/en/products/socs/esp32-s3"><img alt="ESP32-S3" src="https://img.shields.io/badge/ESP32--S3-Arduino-e7352c?logo=espressif" /></a>
  <a href="https://www.raspberrypi.com/products/raspberry-pi-4-model-b/"><img alt="Raspberry Pi 4" src="https://img.shields.io/badge/Raspberry%20Pi-4B-A22846?logo=raspberrypi" /></a>
  <a href="https://www.python.org"><img alt="Python" src="https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python" /></a>
  <a href="https://mqtt.org"><img alt="MQTT" src="https://img.shields.io/badge/MQTT-5-660066?logo=mqtt" /></a>
  <a href="https://opencv.org"><img alt="OpenCV" src="https://img.shields.io/badge/OpenCV-ArUco-5C3EE8?logo=opencv" /></a>
  <a href="#license"><img alt="License" src="https://img.shields.io/badge/license-MIT-green" /></a>
  <a href="https://github.com/KPatrol/Robot_WS"><img alt="Repo" src="https://img.shields.io/badge/repo-Robot_WS-181717?logo=github" /></a>
</p>

<p align="center">
  <a href="#overview--tổng-quan">Overview</a> ·
  <a href="#hardware">Hardware</a> ·
  <a href="#layout">Layout</a> ·
  <a href="#firmware-esp32-s3">Firmware</a> ·
  <a href="#pi-controller">Pi Controller</a> ·
  <a href="#camera-stream">Camera</a> ·
  <a href="#getting-started">Getting Started</a> ·
  <a href="#license">License</a>
</p>

---

## Overview / Tổng quan

The `robots/` workspace is the on-vehicle stack of the K-Patrol indoor patrol robot — everything that runs on the robot itself rather than on the cloud / operator side. It pairs:

- **ESP32-S3 firmware** that closes the low-level motor + encoder + IMU + GPS loop and exposes a UART command protocol.
- **Raspberry Pi 4 controller (Python)** that bridges UART ↔ MQTT, runs the patrol scripts, ArUco / anomaly detection, and an MJPEG camera streamer.
- **Wiring docs, motor / IMU / GPS test sketches, and integration tests** used during the build of the K-Patrol V10 prototype.

This repository is mirrored from the [KPatrol monorepo](https://github.com/KPatrol/KPatrol) and is published standalone as **Robot_WS** so it can be flashed / deployed on the robot without pulling the rest of the stack.

---

## Hardware

| Subsystem | Component | Notes |
|-----------|-----------|-------|
| MCU | ESP32-S3 (DevKitC) | Arduino framework, runs the controller sketch |
| Motors | 4× DC + BTS7960 H-bridges | Mecanum drive (FL · FR · BL · BR) |
| Encoders | Quadrature, 4 channels | Closed-loop velocity control |
| IMU | BNO08x | I²C primary, UART fallback |
| GPS | NEO-M8N (UART) | Outdoor / lobby fixes |
| Compute | Raspberry Pi 4 (4 GB) | Pi-side controller, MQTT bridge, camera |
| Camera | USB / CSI | MJPEG streamer at 640×480 |
| Power | 12 V LiFePO4 + 5 V buck | Separate logic / motor rails |

See [docs/](docs/) for the wiring diagrams (TXT + Markdown). [docs/MOUNTING_GUIDE_IMU_TOF.md](docs/MOUNTING_GUIDE_IMU_TOF.md) covers the chassis layout.

---

## Layout

```
robots/
├── firmware/                          # ESP32-S3 Arduino sketches
│   ├── controller_four_motors_mecanum/   # Production mecanum controller
│   ├── encoder_reader_4motors/        # 4-channel encoder reader
│   ├── i2c_scanner_only/              # Bus discovery
│   ├── test_single_motor_bts7960/     # Single-motor + BTS7960 bring-up
│   ├── test_two_motors_FR_FL/         # Paired front motors
│   ├── test_three_motors_FR_FL_BR/    # 3-motor incremental test
│   ├── test_motor_encoder_esp32/      # Motor + encoder closed-loop test
│   ├── test_i2c_bno08x/               # BNO08x via I²C
│   ├── test_bno08x_uart/              # BNO08x via UART
│   ├── test_gps_uart_sniffer/         # NEO-M8N UART sniffer
│   └── backup/                        # Reference / archived sketches
├── pi-controller/                     # Raspberry Pi 4 controller (Python)
│   ├── kpatrol_controller.py          # Main controller orchestrator
│   ├── kpatrol_mqtt_v5.py             # MQTT v5 bridge
│   ├── kpatrol_diagnostics.py         # Self-test + bench tools
│   ├── kpatrol_cli.py                 # CLI for ad-hoc commands
│   ├── kpatrol_gamepad.py             # Gamepad teleop
│   ├── kpatrol_keyboard.py            # Keyboard teleop
│   ├── gps_reader.py                  # NEO-M8N reader
│   ├── sensor_reader.py               # IMU + auxiliary sensors
│   ├── navigation/                    # Patrol scripts + recorder
│   ├── detection/                     # ArUco + anomaly detector + alert bridge
│   ├── tools/                         # Marker generator, simulator, test harness
│   ├── data/                          # Grid + waypoints + maps
│   ├── *.service · 99-kpatrol-serial.rules · setup.sh · run.sh
│   └── README.md, README_SCRIPT_PATROL.md
├── camera-stream/                     # MJPEG server + route recorder
├── apps/control-center/               # (placeholder) on-robot control surface
├── test/                              # Integration / motor wizard scripts
├── docs/                              # Wiring + pinout diagrams + brand
└── images/                            # Reference photos (esp32, motor, …)
```

---

## Firmware (ESP32-S3)

Each folder under [firmware/](firmware/) is a self-contained Arduino sketch. The production sketch is [firmware/controller_four_motors_mecanum/](firmware/controller_four_motors_mecanum/); the others are bring-up tools used while integrating new hardware.

### Flashing

```bash
# Install arduino-cli or open the .ino in Arduino IDE 2.x
arduino-cli compile  --fqbn esp32:esp32:esp32s3 firmware/controller_four_motors_mecanum
arduino-cli upload   --fqbn esp32:esp32:esp32s3 -p /dev/ttyUSB0 firmware/controller_four_motors_mecanum
```

The pinout for the production sketch lives in [docs/PINOUT_DIAGRAM_MECANUM_FOUR_MOTORS.txt](docs/PINOUT_DIAGRAM_MECANUM_FOUR_MOTORS.txt).

---

## Pi Controller

The Raspberry Pi 4 hosts a Python 3.10 controller stack that:

1. Speaks the UART protocol exposed by the ESP32 sketch.
2. Bridges robot ↔ cloud over **MQTT v5** (`kpatrol_mqtt_v5.py`).
3. Runs autonomous patrol scripts (`navigation/script_patrol.py`).
4. Detects anomalies + ArUco markers (`detection/`).
5. Exposes a CLI / gamepad / keyboard teleop fallback.

### Install on a Pi

```bash
cd pi-controller
chmod +x setup.sh
./setup.sh                       # installs system + Python deps, copies udev rules

# Edit env files:
cp config.env config.env.local && $EDITOR config.env.local
cp mqtt.env mqtt.env.local       && $EDITOR mqtt.env.local

./run.sh                         # one-shot run
sudo systemctl enable --now kpatrol.service kpatrol-mqtt.service   # systemd
```

Service units in [pi-controller/kpatrol.service](pi-controller/kpatrol.service) and [pi-controller/kpatrol-mqtt.service](pi-controller/kpatrol-mqtt.service) wire the controller + MQTT bridge into systemd. The udev rule [pi-controller/99-kpatrol-serial.rules](pi-controller/99-kpatrol-serial.rules) gives stable names to the USB-serial adapters (ESP32, GPS, optional second IMU).

See [pi-controller/README.md](pi-controller/README.md) and [pi-controller/README_SCRIPT_PATROL.md](pi-controller/README_SCRIPT_PATROL.md) for the controller API + patrol-script DSL.

---

## Camera Stream

[camera-stream/mjpeg_server.py](camera-stream/mjpeg_server.py) exposes an MJPEG HTTP endpoint at `:8080/stream` consumed by the operator console. [camera-stream/record_route.py](camera-stream/record_route.py) records a synchronized telemetry + camera bundle while a patrol runs, used for offline review.

---

## Getting Started

```bash
git clone https://github.com/KPatrol/Robot_WS.git
cd Robot_WS

# 1. Flash the ESP32 with the production sketch (see "Firmware" above).
# 2. Bring up the Pi:
cd pi-controller && ./setup.sh && ./run.sh

# 3. Stream the camera:
cd ../camera-stream && python3 mjpeg_server.py

# 4. (Optional) Run the patrol DSL with a pre-recorded route:
python3 pi-controller/navigation/script_patrol.py --grid pi-controller/data/grid.json
```

---

## Branch hygiene

This repository keeps three persistent branches:

- **`main`** — the merged, production-ready trunk; every feature group lands here via squash-rebase from a per-feature branch.
- **`dev`** — an integration branch tracked from `main` for staging next-up work.
- **`feat/ci-and-quality`** — an open draft pull request that scaffolds the CI workflow (`.github/workflows/ci.yml`) + cross-editor config (`.editorconfig`).

The publish workflow that produces these branches lives in the parent monorepo's [DEPLOY.md](https://github.com/KPatrol/KPatrol/blob/main/DEPLOY.md).

---

## Documentation

- [docs/MECANUM_SETUP_GUIDE.md](docs/MECANUM_SETUP_GUIDE.md)
- [docs/MOUNTING_GUIDE_IMU_TOF.md](docs/MOUNTING_GUIDE_IMU_TOF.md)
- [docs/RELAY_LIGHT_WIRING_GUIDE.md](docs/RELAY_LIGHT_WIRING_GUIDE.md)
- [docs/SINGLE_MOTOR_WIRING_GUIDE.md](docs/SINGLE_MOTOR_WIRING_GUIDE.md)
- [docs/TWO_MOTORS_WIRING_GUIDE.md](docs/TWO_MOTORS_WIRING_GUIDE.md)
- Pinout diagrams: [single](docs/PINOUT_DIAGRAM_SINGLE_MOTOR.txt) · [two](docs/PINOUT_DIAGRAM_TWO_MOTORS.txt) · [three](docs/PINOUT_DIAGRAM_THREE_MOTORS.txt) · [mecanum 4×](docs/PINOUT_DIAGRAM_MECANUM_FOUR_MOTORS.txt)
- Encoder wiring: [docs/ENCODER_WIRING_4MOTORS.txt](docs/ENCODER_WIRING_4MOTORS.txt)

---

## License

MIT License — © K-Patrol / Vu Dang Khoa, 2026.
