# K-PATROL MQTT INTEGRATION

## Tổng quan

Hệ thống MQTT cho phép điều khiển robot K-Patrol từ web app qua internet thông qua MQTT broker trên VPS.

```
┌─────────────────┐         ┌─────────────────┐         ┌─────────────────┐
│   Mobile/Web    │  MQTT   │   VPS Broker    │  MQTT   │  Raspberry Pi   │
│     App         │ ──────▶ │  <MQTT_BROKER_HOST>   │ ──────▶ │   + ESP32-S3    │
│  (Next.js)      │ ◀────── │    :1883        │ ◀────── │                 │
└─────────────────┘         └─────────────────┘         └─────────────────┘
```

## Cấu hình MQTT

### Broker
- **Host**: <MQTT_BROKER_HOST>
- **Port**: 1883 (TCP), 8083 (WebSocket)
- **Username**: <MQTT_USERNAME>
- **Password**: <MQTT_PASSWORD>

### Topics

| Topic | Hướng | Mô tả |
|-------|-------|-------|
| `kpatrol/command` | Web → Pi | Lệnh di chuyển |
| `kpatrol/speed` | Web → Pi | Cài đặt tốc độ |
| `kpatrol/emergency` | Web → Pi | Dừng khẩn cấp |
| `kpatrol/status` | Pi → Web | Trạng thái robot |
| `kpatrol/heartbeat` | Both | Heartbeat ping |
| `kpatrol/log` | Pi → Web | Log messages |
| `kpatrol/error` | Pi → Web | Error messages |

## Cài đặt

### 1. Raspberry Pi

```bash
# Đã cài sẵn paho-mqtt
pip3 install paho-mqtt

# Chạy thủ công
cd ~/robots/pi-controller
python3 kpatrol_mqtt.py

# Hoặc dùng alias
kp-mqtt
```

### 2. Chạy như Service

```bash
# Khởi động service
mqtt-start

# Dừng service
mqtt-stop

# Xem trạng thái
mqtt-status

# Xem log real-time
mqtt-log

# Bật auto-start
mqtt-enable
```

### 3. Web App (mobile-app)

```bash
cd mobile-app

# Cài dependencies
pnpm install

# Chạy dev server
pnpm dev
```

## Sử dụng

### Từ Terminal trên Pi

```bash
# Chạy MQTT client
kp-mqtt

# Output:
# [MQTT] Connected successfully!
# [MQTT] Subscribed to kpatrol/command
# ...
```

### Từ Web App

1. Mở http://localhost:3000 (hoặc URL deploy)
2. Vào tab "Điều khiển"
3. Xem trạng thái MQTT (xanh = connected)
4. Xem trạng thái Robot (xanh = online)
5. Sử dụng Joystick hoặc D-Pad để điều khiển

### Command Format

```json
// Command
{
  "type": "forward",  // forward|backward|left|right|strafe_left|strafe_right|stop|off
  "speed": 150,       // 0-255
  "timestamp": 1234567890123
}

// Status
{
  "connected": true,
  "battery": 85,
  "speed": 150,
  "motors": {"fl": 0, "fr": 0, "br": 0, "bl": 0},
  "timestamp": 1234567890123
}
```

## Files

### Pi (~/robots/pi-controller/)
- `kpatrol_mqtt.py` - MQTT client script
- `mqtt.env` - Cấu hình MQTT
- `kpatrol-mqtt.service` - Systemd service

### Web (mobile-app/)
- `src/lib/mqtt-config.ts` - MQTT configuration
- `src/providers/MQTTProvider.tsx` - MQTT React provider
- `src/components/ui/MQTTStatus.tsx` - Status component

## Troubleshooting

### MQTT không kết nối

```bash
# Kiểm tra mạng
ping <MQTT_BROKER_HOST>

# Test với mosquitto client
mosquitto_sub -h <MQTT_BROKER_HOST> -p 1883 -u <MQTT_USERNAME> -P <MQTT_PASSWORD> -t "kpatrol/#"
```

### Serial không hoạt động

```bash
# Kiểm tra port
ls /dev/tty* | grep -E '(ACM|USB)'

# Kiểm tra permission
sudo usermod -a -G dialout $USER
# Logout và login lại
```

### Web không nhận được status

- Kiểm tra Pi đang chạy kpatrol_mqtt.py
- Kiểm tra browser console cho MQTT errors
- Đảm bảo WebSocket port 8083 đang hoạt động

## Bảo mật

⚠️ **Lưu ý**: Credentials đang ở dạng plaintext. Trong production:
- Sử dụng environment variables
- Enable TLS/SSL
- Sử dụng authentication tokens
