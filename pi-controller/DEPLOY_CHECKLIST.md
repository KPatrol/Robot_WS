# K-Patrol Pi Deploy Checklist — 2026-05-13

Trạng thái firmware ESP32 đã verify trên Mac (5/6 TOF OK, CH2 FRONT_RIGHT hardware fault — accepted, fail-safe đã được kiểm chứng). Checklist này dùng khi rút ESP32 khỏi Mac và cắm lại vào Pi.

## 0. Pre-flight (làm trên Mac trước khi rút)

- [x] Firmware `encoder_reader_4motors.ino` v3.0 đã flash + verify (5/6 TOF, ENC, ENC_HB streaming OK).
- [x] Smoke test 30s `/dev/cu.usbserial-10`: 147 TOF / 48 ENC / 24 HB frames. CH2 luôn 9999 + bit2=0 (fail-safe đúng).
- [x] Parser `sensor_reader.py` xử lý `valid_mask` đúng, callbacks chạy ổn định.
- [x] `reactive_nav.py` dùng sentinel 9999 → KHÔNG false-steer khi CH2 chết.
- [x] `kpatrol_mqtt_v5.py` đọc `valid_mask` ở line 326–335 và check valid trước khi dùng (lines 372, 385).

→ Rút cable USB ESP32 khỏi Mac (đóng serial monitor trước nếu có: `lsof /dev/cu.usbserial-10` → kill PID).

## 1. Pi-side — physical & boot

1. Pi đã sạc / cắm nguồn → boot lên.
2. Cắm ESP32 Dev (encoder + TOF, CH340) vào USB Pi.
3. Cắm ESP32-S3 motor controller (`/dev/ttyACM0`) — nếu chưa cắm.
4. Cắm GPS NEO-6M (CH340) — sau encoder, để encoder lấy `ttyUSB0` trước.

## 2. Verify port enumeration

```bash
# Liệt kê tất cả tty hiện có
ls -la /dev/ttyUSB* /dev/ttyACM* 2>/dev/null

# Xem symlink udev đã tạo
ls -la /dev/ttyKPATROL_* 2>/dev/null

# Identify từng device (vendor/product/serial)
for d in /dev/ttyUSB* /dev/ttyACM*; do
  echo "=== $d ==="
  udevadm info -a -n "$d" 2>/dev/null | grep -E 'idVendor|idProduct|serial' | head -3
done
```

**Expected:**
- `/dev/ttyACM0` → ESP32-S3 motor (vendor 303a, product 1001).
- `/dev/ttyUSB0` → ESP32 Dev encoder (vendor 1a86, product 7523 — CH340).
- `/dev/ttyUSB1` → GPS NEO-6M (cùng CH340).

**Lưu ý udev rule conflict:** `99-kpatrol-serial.rules` hiện đang nhận encoder là CP2102 (`10c4:ea60`) — không khớp ESP32 CH340 thực tế. Symlink `/dev/ttyKPATROL_ENCODER` **sẽ KHÔNG được tạo**. Không sao: `kpatrol_mqtt_v5.py` (lines 2526–2530) auto-fallback sang `/dev/ttyUSB0` nên service vẫn chạy. Nếu muốn fix triệt để symlink, xem mục 6.

## 3. Verify firmware stream (sanity check before service)

```bash
# Đảm bảo service chưa giữ port
sudo systemctl stop kpatrol.service 2>/dev/null

# Cold-reset ESP32 và quan sát 5s output
python3 - <<'PY'
import serial, time
s = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
s.setDTR(False); s.setRTS(True); time.sleep(0.1); s.setRTS(False)
deadline = time.time() + 5
while time.time() < deadline:
    line = s.readline().decode('utf-8', errors='ignore').strip()
    if line: print(line)
s.close()
PY
```

**Expected output:**
- `STATUS:TOF_INIT,5,6` (5/6 sensor init OK)
- `ERR:TOF_INIT_CH2_FRONT_RIGHT` (CH2 fail — đã accepted)
- `TOF:9999,9999,9999,d3,d4,9999,<mask>` ~30 Hz
- `ENC_HB:<ms>,0` mỗi 1s (`tca_fail_count=0` ổn định)
- `ENC:<4 counts>,<4 speeds>` mỗi 500ms

Nếu thấy đúng → ESP32 + Pi handshake OK.

## 4. Start the service

```bash
# Reload nếu file service vừa cập nhật
sudo systemctl daemon-reload

# Enable + start
sudo systemctl enable kpatrol.service
sudo systemctl restart kpatrol.service

# Monitor logs realtime
sudo journalctl -u kpatrol.service -f -o cat
```

**Look for:**
- `[serial] Encoder connected on /dev/ttyUSB0`
- `[mqtt] Connected to 103.81.84.43:1883`
- TOF / ENC / HB messages publishing
- KHÔNG được thấy: `record_feed`, `script_dir`, `BAT:0,*` (3 fix đã apply trước đó).

## 5. End-to-end MQTT verify

Trên Mac hoặc máy khác:

```bash
# Subscribe robot alert + telemetry
mosquitto_sub -h 103.81.84.43 -p 1883 \
  -u alphaasimov2024 -P gvB3DtGfus6U \
  -t "kpatrol/KPATROL-001/#" -v
```

**Expected topics:**
- `kpatrol/KPATROL-001/telemetry` — TOF + ENC payload
- `kpatrol/KPATROL-001/heartbeat`
- `kpatrol/KPATROL-001/alert` (nếu detection trigger)

Visual confirm: https://kpatrol.khoavd.online

## 6. (Optional) Fix udev rule cho symlink `/dev/ttyKPATROL_ENCODER`

Encoder hiện là CH340 nên udev rule cần phân biệt với GPS qua **USB port path** (KERNELS) chứ không phải `serial` (CH340 không có serial unique).

```bash
# Tìm KERNELS path của encoder (cắm encoder, chưa cắm GPS)
udevadm info -a -n /dev/ttyUSB0 | grep -E 'KERNELS|idVendor|idProduct' | head -10
# Note: KERNELS=="1-1.2" (ví dụ)

# Sửa rule
sudo nano /etc/udev/rules.d/99-kpatrol-serial.rules
```

Thay khối "Encoder + ToF" bằng:

```
# Encoder + ToF — ESP32 Dev CH340 (phân biệt qua USB port path)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
    KERNELS=="<USB_PORT_PATH>", \
    SYMLINK+="ttyKPATROL_ENCODER", MODE="0666", GROUP="dialout"
```

Sau đó:
```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -la /dev/ttyKPATROL_ENCODER
```

Nếu không làm bước 6, fallback `/dev/ttyUSB0` vẫn hoạt động — KHÔNG ảnh hưởng deploy.

## 7. Smoke test detection (nếu Pi camera đã online)

```bash
cd /home/khoavd/kpatrol/pi-controller
python3 -m tools.detection_smoke_test --source 0 --no-mqtt --fps 5 --max-frames 30
```

→ Verify camera + YOLO + HSV pipeline chạy được trên Pi (kỳ vọng ~5–10 FPS với onnxruntime CPU).

## 8. Rollback nếu service không chạy

```bash
sudo systemctl stop kpatrol.service
sudo journalctl -u kpatrol.service -n 100 --no-pager
```

Common pitfalls:
- Port permission: user `khoavd` phải thuộc group `dialout` (`groups khoavd`).
- mqtt.env path: service load `EnvironmentFile=-/home/khoavd/kpatrol/pi-controller/mqtt.env` — verify file tồn tại.
- Camera busy: kill bất kỳ process nào đang giữ `/dev/video0`.

## 9. Khi nào CH2 cần fix

Đề xuất khi user rảnh: physical swap module CH2 ↔ CH5 → flash lại firmware (không đổi code).
- Nếu fault theo module → cảm biến VL53L0X CH2 hỏng (cần thay).
- Nếu fault theo lane → trace SDA/SCL CH2 ở TCA9548A.

Không patch firmware lần 4 cùng dạng — đây là hardware test, không phải software.

## 10. Cài đặt `kpatrol-detection.service` (detection bridge → MQTT)

Service publish person/fire alert lên `kpatrol/{serial}/alert`. Phải cài SAU khi `kpatrol.service` đã chạy ổn định (detection phụ thuộc camera, chia sẻ `mqtt.env`).

```bash
# Pi đã có file repo tại /home/khoavd/kpatrol (rsync từ Mac hoặc git pull)
cd /home/khoavd/kpatrol/pi-controller

# 1. Cài model + deps detection (nếu chưa)
pip3 install --user onnxruntime opencv-python-headless paho-mqtt

# 1a. Quantize YOLO sang INT8 (3.2 MB, ~1.5-2× nhanh hơn FP32 trên Pi 4 CPU)
#     Lần đầu cần yolov8n.pt → ultralytics auto-export ONNX → quantize_dynamic.
#     AnomalyDetector._resolve_yolo_model_path() tự pick <stem>_int8.onnx.
python3 -m tools.quantize_yolo --auto
ls yolov8n_int8.onnx   # phải tồn tại (~3.2 MB) bên cạnh pi-controller/

# 2. Install unit
sudo cp kpatrol-detection.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable kpatrol-detection.service
sudo systemctl start kpatrol-detection.service

# 3. Verify
sudo systemctl status kpatrol-detection.service --no-pager
sudo journalctl -u kpatrol-detection.service -f -o cat
```

**Expected logs:**
- `[bridge] connecting to 103.81.84.43:1883`
- `[bridge] mqtt connected`
- `[bridge] kpatrol/KPATROL-001/alert -> {…}` mỗi khi detect

**End-to-end smoke (từ Mac):**
```bash
mosquitto_sub -h 103.81.84.43 -p 1883 \
  -u alphaasimov2024 -P gvB3DtGfus6U \
  -t "kpatrol/+/alert" -v
```
→ Bật một vật thể người/lửa trước camera Pi → tin nhắn JSON với `kind=person|fire`, `confidence`, `bbox`, `snapshot_b64` xuất hiện.

**Backend verify (VPS):**
```bash
ssh khoavd@10.8.0.7 'docker compose -f /home/khoavd/kpatrol/docker-compose.yml logs --tail=50 backend | grep -i mqttingest'
```
→ Mỗi alert: `[MqttIngestService] Ingested alert from KPATROL-001: kind=person id=<DB-id>`.

**Web verify (mobile-app):**
- Đăng nhập https://monitor.khoavd.online với owner của KPATROL-001
- Trigger detection trên Pi → toast `robot:alert` event hiện tức thì
- Vào trang Alerts → hàng mới với snapshot inline.

**Resource budget:**
- `CPUQuota=80%` + `MemoryMax=512M` để không bóp nghẹt `kpatrol.service`.
- YOLOv8n INT8 + onnxruntime CPU: ~5–10 FPS trên Pi 4 (đã đo trên Mac, expect tương đương trên Pi 5).
- SQLite WAL drainer cap 10k row, vacuum 24h — không lo disk grow.

**Stop / rollback:**
```bash
sudo systemctl stop kpatrol-detection.service
sudo systemctl disable kpatrol-detection.service
```
`PartOf=kpatrol.service` đảm bảo: stop kpatrol → detection cũng stop. Restart kpatrol → detection KHÔNG auto-restart (chỉ `Wants`, không `Requires`); chạy lại bằng `systemctl restart kpatrol-detection`.
