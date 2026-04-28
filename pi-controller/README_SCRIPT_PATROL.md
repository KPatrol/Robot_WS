# K-Patrol Scripted Patrol — Deployment Guide

## Tổng quan

Hệ thống tuần tra tự động theo **script mệnh lệnh tuyến tính**: đi thẳng X giây,
xoay Y độ, đi ngang Z giây, v.v. Không phụ thuộc line, marker, mapping, odometry,
hay VFH — tập trung CPU cho AI detect người/lửa.

## Ba chế độ điều hướng

| Mode            | Hành vi                                                          |
|-----------------|------------------------------------------------------------------|
| `MANUAL`        | Operator lái qua topic `motor`, controller pass-through          |
| `SCRIPT_PATROL` | Executor chạy script, IMU-closed-loop rotation, ToF reflex       |
| `EMERGENCY`     | Motors dừng, chờ operator clear                                  |

## MQTT topics (scope `kpatrol/{serial}/…`)

### Subscribe
- `nav_command` — `{"mode": "MANUAL|SCRIPT_PATROL|EMERGENCY", "speed": 60, "action": "clear_emergency"}`
- `script_command` — `{"action": "list|save|delete|load|start|stop", "name": "...", "script": {...}}`

### Publish
- `nav_status` — executor state, current step, progress, errors
- `script_list` — scripts available với metadata
- `script_status` — kết quả action vừa thực hiện

## Primitive script steps

| Type             | Fields                                                |
|------------------|-------------------------------------------------------|
| `rotate`         | `angle_deg` (+trái, -phải), `speed_pct`               |
| `forward_time`   | `duration_s`, `speed_pct`                             |
| `backward_time`  | `duration_s`, `speed_pct`                             |
| `strafe_time`    | `direction` (`"left"`/`"right"`), `duration_s`, `speed_pct` |
| `forward_until`  | `min_distance_mm`, `timeout_s`, `speed_pct`           |
| `strafe_until`   | `direction`, `min_distance_mm`, `timeout_s`, `speed_pct` |
| `pause`          | `duration_s`                                          |

## Ví dụ script (rectangle)

```json
{
  "name": "rectangle",
  "default_speed_pct": 60,
  "loop": true,
  "steps": [
    {"type": "forward_time", "duration_s": 4.0},
    {"type": "rotate",       "angle_deg": 90},
    {"type": "forward_time", "duration_s": 2.0},
    {"type": "rotate",       "angle_deg": 90},
    {"type": "forward_time", "duration_s": 4.0},
    {"type": "strafe_time",  "direction": "left", "duration_s": 3.0}
  ]
}
```

## Bật máy & triển khai

### Trên máy dev (đã verify ✅)
- 40/40 unit tests pass (`python3 -m pytest tests/test_script_patrol.py`)
- Syntax check sạch cho `kpatrol_mqtt_v5.py`, `nav_controller.py`, `script_patrol.py`, `mjpeg_server.py`
- Integration smoke test: `save → list → start → tick → emergency → clear` đều đúng
- Speed dedup verified: tick đầu emit SPD+cmd, tick sau chỉ emit cmd khi đổi

### Trên Raspberry Pi
```bash
cd /home/pi/KPatrol/robots/pi-controller

# 1. Đảm bảo symlinks thiết bị
ls -l /dev/ttyKPATROL_MOTOR /dev/ttyKPATROL_ENCODER

# 2. Chạy controller
ROBOT_SERIAL=KPATROL-001 python3 kpatrol_mqtt_v5.py

# 3. (Tuỳ chọn) Stream camera
cd ../camera-stream && python3 mjpeg_server.py
```

### Systemd service
Cập nhật `kpatrol.service` / `kpatrol-mqtt.service` nếu cần — script path
giữ nguyên (`kpatrol_mqtt_v5.py`).

## Kiểm thử thủ công sau khi bật Pi

1. **MANUAL drive** — Gửi `{"type":"F"}` tới `kpatrol/{serial}/motor` → robot tiến.
2. **Save script** — Gửi payload `rectangle` tới `script_command` action `save`.
3. **Start** — `{"action":"start","name":"rectangle"}` → robot chạy chu trình.
4. **Stop** — `{"action":"stop"}` → dừng, mode về MANUAL.
5. **ToF reflex** — Đặt vật cản trước robot khi đang chạy → nav_status chuyển
   EMERGENCY, motors dừng, đèn đỏ.
6. **Clear** — `nav_command` với `{"action":"clear_emergency"}` → MANUAL.

## Các module đã xoá

`line_follower`, `teach_replay`, `marker_boundary`, `mini_grid`, `odometer`,
`path_planner`, `vfh`, `smart_coverage`, `patrol_spiral`, `patrol_scheduler`,
`battery_manager`, `speed_controller`, `map_manager`, toàn bộ `simulation/`,
`line_detector`, `marker_detector`, `kpatrol_mqtt{,_v2,_v3,_v4}`,
`benchmark_line_follow`, `simulate_line_follow`, `simulate_warehouse`,
`demo_map_result`, `test_teach_replay`.
