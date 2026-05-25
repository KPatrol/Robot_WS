# Changelog — K-Patrol Robot_WS

All notable changes to this repository are documented here. The format is based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/) and this project follows
[Semantic Versioning](https://semver.org/).

## [Unreleased]

### Planned
- V11.1: HUST-Roboflow Vietnamese indoor-fire YOLO model (target indoor demo recall ≥ 90%)
- V5.16: Stuck-detection FSM (commanded vs. actual velocity delta + recovery)
- Two-wheel differential mode toggle for non-mecanum chassis
- Hot-pluggable peripheral discovery (no fixed udev slot)
- OTA firmware update over MQTT with signed payloads
- ROS 2 Humble bridge (telemetry passthrough to RViz)

## [1.0.0] — 2026-05-25

Graduation-thesis release: the on-robot stack used for the K-Patrol V10 prototype
demo at Phenikaa University.

### Added

#### Firmware — three-board peripheral architecture
- `encoder_reader_4motors_v5`: dedicated ESP32 Dev encoder reader emitting `ENC_HB`
  heartbeat + 6× VL53L0X ToF over UART. Replaces motor-board encoder polling.
- `peripheral_hub_d1r32`: ESP32 D1 R32 v2.0 firmware managing relay 5V, buzzer,
  warning lamp, and OLED SH1106 eye animation. Receives commands over USB serial.
- `battery_monitor_s3` / `test_ina219_s3`: standalone INA219 sketches for bench
  validation of the LFP 4S pack.
- `tof_debug`: TCA9548A scan-all-channels diagnostic for VL53L0X bring-up.
- INA219 primary path + ADC fallback in `controller_four_motors_mecanum` with LFP 4S
  thresholds (full 14.6 V / nominal 13.2 V / low 12.8 V / critical 12.0 V).

#### Pi controller — detection V10.4 + V11.0 infrastructure
- 6+1 stage HSV fire pipeline with skin discriminator:
  - Stage 1b yellow-vs-wrap-red ratio gate (≥ 0.4) rejects orange-pink skin.
  - Hot-core saturation floor `S_min = 30` rejects bright skin + specular highlights.
  - Outer envelope tightened: `S 60→80`, yellow `V 180→200`.
  - Temporal flicker IoU 0.97 (lenient) + area gate + shape gate.
- V11.0 dual-pipeline infrastructure: `KPATROL_FIRE_MODE=hsv|yolo` env switch.
  Default is HSV — outdoor-trained YOLO models don't yet generalise to indoor.
- New tools: `calibrate_fire.py` (realtime HSV trackbars), `download_fire_model.py`
  (D-Fire / Spacewalk / HuggingFace sources), `test_fire_model.py` (webcam compare).

#### Pi controller — safety V5.15c10 + alarm engine
- `alert_actuator`: relay protection. Battery low/critical lights default to OFF
  (the mechanical relay was clicking at 1 Hz blink). `auto_clear_light_after_sec`
  = 4.0 and `auto_clear_buzzer_after_sec` = 6.0 force alerts back to OFF.
- `alarm_controller`: operator-configurable rule engine. 9 event types
  (person, fire, battery_low, battery_critical, tipover, schedule, system_error,
  any_safety, manual_test), continuous-duration debounce, time-of-day window,
  per-rule light/buzzer pattern. Rules pushed via MQTT.

#### Pi controller — navigation V5.15c9 + lane segmentation
- `simple_line_follower`: Mecanum steering mode (vx + wz, vy=0) replacing
  crab-walk DL/DR diagonal. Emits MEC tuple `(vx=80, vy=0, wz, spd=200)`.
  Linear-regression slope (not polyfit derivative) for stable heading.
  Tuning: `kp_heading=8, kd=0, heading_deadband=0.02 rad`.
- `line_follower` V5.15c3: broadened yellow tape HSV envelope (H 18-38 /
  S 80+ / V 50+) for parity across dim and bright lighting. Coverage on
  `/tmp/diag.png` went from 0.03% → 0.7%.
- `lane_seg/`: TwinLiteNet+ Nano ONNX runtime — Pi 4B inference at 12.6 FPS,
  192×320, 2 threads. Soft-fail to HSV when ONNX missing.

#### Pi controller — MQTT bridge + peripheral hub Python
- `kpatrol_mqtt_v5.py`: MEC command dispatch (`cmd='MEC' → 'MEC:vx,vy,wz,spd'`),
  peripheral hub serial bridge, S3 DTR reset workaround
  (`dtr=False, rts=False` before `Serial.open()` — CH9102 ties DTR → EN),
  alarm rule MQTT subscription, hybrid GPS (NEO-6M + ip-api.com with `source`
  field auto-switch), `snapshot_b64` in MQTT payload for detection events.
- `peripheral_hub.py`: Python bridge for D1 R32 — MQTT ↔ serial commands +
  REST endpoint for diagnostics.

#### Documentation
- [docs/HARDWARE_PERIPHERALS.md](docs/HARDWARE_PERIPHERALS.md): full wiring map +
  I²C addresses + MQTT topic table for the D1 R32 hub.
- [docs/HARDWARE_PERIPHERALS_QUICK.md](docs/HARDWARE_PERIPHERALS_QUICK.md):
  bench bring-up cheat sheet.
- [pi-controller/DEPLOY_CHECKLIST.md](pi-controller/DEPLOY_CHECKLIST.md):
  refreshed for the 3-board layout + INA219 + TwinLiteNet+ option.

### Changed

- **Power architecture**: documented the 3-buck layout. There is no 3V3 buck —
  sensors share the MCU header's regulated 3V3 rail. Motor power bypasses bucks
  (direct 14.6 V via 20 A fuse to BTS7960 power rail).
- **udev rules** ([pi-controller/99-kpatrol-serial.rules](pi-controller/99-kpatrol-serial.rules)):
  added CH9102 (`1a86:55d3`) for S3 alongside native USB (`303a:1001`).
  Disambiguated CH340G encoder Dev (slot `1-1.3`) vs. periph hub (slot `1-1.2`)
  by USB hub path since CH340G has no programmable serial number.
- **Encoder counts**: corrected `COUNTS_PER_REV` from ×4 to ×2 (channel A ISR only).

### Fixed

- Detection: false-positive fire on bare human skin under warm neon lights
  (V10.3 → V10.4 skin discriminator).
- Detection: max_static_IoU semantic was reversed; raised to 0.97 (lenient).
- ToF: lane publisher was emitting 0/3 mm with `valid_mask` set; added defence
  (`MIN_VALID_TOF_MM=25`) at both publisher and PWA consumer.
- Detection: MJPEG sidecar stream now consumed via service sandbox.
- Encoder: fixed `COUNTS_PER_REV` from ×4 to ×2 (single-channel ISR).

### Security

- MQTT credentials moved from `pi-controller/mqtt.env` (gitignored) to
  `mqtt.env.local` (gitignored) — `.example` template tracked.
- Third-party reference repos (`3rdparty/`) and ML model weights
  (`pi-controller/models/*.pt`, `*.onnx`) excluded from git to avoid 22 MB+
  binaries in the repo history.

---

[Unreleased]: https://github.com/KPatrol/Robot_WS/compare/v1.0.0...HEAD
[1.0.0]: https://github.com/KPatrol/Robot_WS/releases/tag/v1.0.0
