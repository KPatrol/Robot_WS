# K-Patrol Bot - Hướng dẫn Thiết lập 4 Bánh Mecanum

## 📋 MỤC LỤC
1. [Kiểm tra phần cứng](#1-kiểm-tra-phần-cứng)
2. [Sơ đồ đấu nối](#2-sơ-đồ-đấu-nối)
3. [Nạp firmware](#3-nạp-firmware)
4. [Test từng motor](#4-test-từng-motor)
5. [Xử lý sự cố](#5-xử-lý-sự-cố)

---

## 1. KIỂM TRA PHẦN CỨNG

### ✅ Checklist trước khi bắt đầu:

```
□ 4 động cơ DC 12V có encoder
□ 4 driver BTS7960
□ 4 bánh xe Mecanum (2 trái + 2 phải)
□ ESP32-S3 Dev Module
□ Nguồn 12V (đủ dòng cho 4 motor, tối thiểu 10A)
□ Nguồn 5V cho logic (có thể dùng từ USB)
□ Dây nối đủ số lượng
```

### ⚠️ QUAN TRỌNG: Hướng lắp bánh Mecanum

Nhìn từ trên xuống, các roller phải tạo thành hình **X**:

```
        TRƯỚC (Front)
    ┌─────────────────┐
    │  \           /  │
    │   M1       M2   │
    │                 │
    │                 │
    │   M3       M4   │
    │  /           \  │
    └─────────────────┘
        SAU (Back)
```

- **M1, M4**: Bánh có roller hướng `\`
- **M2, M3**: Bánh có roller hướng `/`

---

## 2. SƠ ĐỒ ĐẤU NỐI

### 2.1. ESP32-S3 → BTS7960 (4 driver)

| Motor | Vị trí | RPWM | LPWM | R_EN | L_EN |
|-------|--------|------|------|------|------|
| M1 | Front Left | GPIO 4 | GPIO 5 | GPIO 6 | GPIO 7 |
| M2 | Front Right | GPIO 15 | GPIO 16 | GPIO 17 | GPIO 18 |
| M3 | Rear Left | GPIO 8 | GPIO 9 | GPIO 10 | GPIO 11 |
| M4 | Rear Right | GPIO 12 | GPIO 13 | GPIO 14 | GPIO 21 |

### 2.2. Đấu nối mỗi BTS7960

```
                    BTS7960 Module
    ┌──────────────────────────────────────────┐
    │                                          │
    │  [Logic Side - 7 pins]                   │
    │  ┌────┬────┬────┬────┬────┬────┬────┐   │
    │  │LPWM│RPWM│L_EN│R_EN│ IS │ IS │VCC │   │
    │  │ B-IB│B-IA│    │    │    │    │    │   │
    │  └──┬─┴──┬─┴──┬─┴──┬─┴────┴────┴──┬─┘   │
    │     │    │    │    │              │      │
    │     │    │    │    │              │      │
    │  ESP32   ESP32 ESP32 ESP32       5V      │
    │  GPIO    GPIO  GPIO  GPIO                │
    │                                          │
    │  [Power Side]                            │
    │  ┌────┬────┬────┬────┐                  │
    │  │ B- │ B+ │ M- │ M+ │                  │
    │  └──┬─┴──┬─┴──┬─┴──┬─┘                  │
    │     │    │    │    │                     │
    │    GND  12V  Motor Motor                 │
    │   (12V)     (White)(Red)                 │
    │                                          │
    └──────────────────────────────────────────┘
```

### 2.3. Kết nối nguồn

```
Nguồn 12V (Battery/Adapter)
    │
    ├──▶ [+12V] ──▶ Tất cả BTS7960 (B+ pin)
    │
    └──▶ [GND] ──┬──▶ Tất cả BTS7960 (B- pin)
                 │
                 └──▶ ESP32-S3 GND
                      (GND CHUNG RẤT QUAN TRỌNG!)

Nguồn 5V (USB hoặc regulator)
    │
    └──▶ [5V] ──▶ Tất cả BTS7960 (VCC pin)
```

### 2.4. Sơ đồ tổng thể

```
                         ┌─────────────┐
                         │  Nguồn 12V  │
                         │  (10A+)     │
                         └──────┬──────┘
                                │
        ┌───────────────────────┼───────────────────────┐
        │                       │                       │
        ▼                       ▼                       ▼
┌───────────────┐       ┌───────────────┐       ┌───────────────┐
│   BTS7960     │       │   BTS7960     │       │   BTS7960     │
│   Motor 1     │       │   Motor 2     │       │   Motor 3,4   │
│   (FL)        │       │   (FR)        │       │   (RL,RR)     │
└───────┬───────┘       └───────┬───────┘       └───────┬───────┘
        │                       │                       │
        ▼                       ▼                       ▼
   ┌────────┐              ┌────────┐              ┌────────┐
   │Motor 1 │              │Motor 2 │              │Motor 3,4│
   │  ⚙️    │              │  ⚙️    │              │  ⚙️⚙️   │
   └────────┘              └────────┘              └────────┘

                    ┌─────────────────┐
                    │    ESP32-S3     │
                    │                 │
                    │ GPIO 4,5,6,7    │──▶ BTS7960 #1
                    │ GPIO 15,16,17,18│──▶ BTS7960 #2
                    │ GPIO 8,9,10,11  │──▶ BTS7960 #3
                    │ GPIO 12,13,14,21│──▶ BTS7960 #4
                    │                 │
                    │ USB ◀──────────▶│ Raspberry Pi
                    └─────────────────┘
```

---

## 3. NẠP FIRMWARE

### 3.1. Cài đặt Arduino IDE

1. Tải Arduino IDE: https://www.arduino.cc/en/software
2. Thêm ESP32 board:
   - File → Preferences → Additional Board URLs:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Tools → Board → Boards Manager → Tìm "esp32" → Install

### 3.2. Cấu hình Board

```
Tools → Board: "ESP32S3 Dev Module"
Tools → USB CDC On Boot: "Enabled"
Tools → Upload Speed: "921600"
Tools → Port: (chọn port ESP32-S3)
```

### 3.3. Nạp code

1. Mở file: `firmware/esp32_s3_motor_controller/esp32_s3_motor_controller.ino`
2. Click "Upload" (→)
3. Đợi nạp xong, mở Serial Monitor (115200 baud)

---

## 4. TEST TỪNG MOTOR

### 4.1. Test qua Serial Monitor

Mở Serial Monitor (Tools → Serial Monitor, 115200 baud)

**Test từng motor:**
```
T1 100    → Test Motor 1 (Front Left) tốc độ 100
T2 100    → Test Motor 2 (Front Right)
T3 100    → Test Motor 3 (Rear Left)
T4 100    → Test Motor 4 (Rear Right)
S         → Dừng tất cả
```

**Test di chuyển:**
```
F 150     → Tiến
B 150     → Lùi
L 150     → Sang trái
R 150     → Sang phải
RL 150    → Xoay trái
RR 150    → Xoay phải
S         → Dừng
```

### 4.2. Test bằng Python script

Trên Raspberry Pi:

```bash
# Cài đặt pyserial
pip3 install pyserial

# Chạy test từng motor
python3 test/test_single_motor.py

# Chạy test di chuyển Mecanum
python3 test/test_mecanum_movement.py
```

### 4.3. Quy trình test chuẩn

```
BƯỚC 1: Test từng motor riêng lẻ
─────────────────────────────────
□ T1 100 → Motor 1 quay?
□ T2 100 → Motor 2 quay?
□ T3 100 → Motor 3 quay?
□ T4 100 → Motor 4 quay?

BƯỚC 2: Kiểm tra chiều quay
─────────────────────────────────
Với T1 100 (giá trị dương):
□ Motor phải quay theo chiều tiến
□ Nếu quay ngược → đảo dây M+ và M-

BƯỚC 3: Test di chuyển cơ bản
─────────────────────────────────
□ F 150 → Robot tiến thẳng?
□ B 150 → Robot lùi thẳng?
□ L 150 → Robot đi ngang trái?
□ R 150 → Robot đi ngang phải?
□ RL 150 → Robot xoay tại chỗ trái?
□ RR 150 → Robot xoay tại chỗ phải?
```

---

## 5. XỬ LÝ SỰ CỐ

### ❌ Vấn đề: Motor không quay

**Kiểm tra:**
1. Nguồn 12V có đủ không?
2. Dây motor (Đỏ/Trắng) đã nối đúng M+/M-?
3. VCC (5V) đã cấp cho BTS7960?
4. GND đã nối chung?

**Debug:**
```cpp
// Thêm vào code để debug
Serial.println("Testing Motor 1...");
digitalWrite(R_EN, HIGH);
digitalWrite(L_EN, HIGH);
ledcWrite(RPWM_channel, 200);  // Thử PWM cao
```

### ❌ Vấn đề: Motor quay ngược chiều

**Giải pháp:**
- Đảo 2 dây motor (M+ ↔ M-) tại driver BTS7960

### ❌ Vấn đề: Robot không đi thẳng

**Nguyên nhân có thể:**
1. Bánh Mecanum lắp sai hướng
2. Motor quay không đều (cần calibrate)
3. Mặt sàn không phẳng

**Giải pháp:**
- Kiểm tra lại hướng lắp bánh (roller tạo hình X)
- Điều chỉnh tốc độ từng motor để cân bằng

### ❌ Vấn đề: Robot đi ngang không đúng

**Nguyên nhân:**
- Bánh Mecanum lắp sai vị trí (trái/phải)

**Quy tắc:**
```
Bánh TRÁI (M1, M3): Roller nghiêng theo hướng \
Bánh PHẢI (M2, M4): Roller nghiêng theo hướng /
```

### ❌ Vấn đề: ESP32-S3 không nhận lệnh

**Kiểm tra:**
1. USB cable có hỗ trợ data không? (không phải cable sạc)
2. Driver USB đã cài đặt?
3. Baudrate đúng 115200?

**Trên Raspberry Pi:**
```bash
# Kiểm tra port
ls /dev/tty* | grep -E '(ACM|USB)'

# Test kết nối
screen /dev/ttyACM0 115200
# Gõ "H" để xem help
# Ctrl+A rồi K để thoát
```

---

## 📊 BẢNG TÓM TẮT GPIO

| Component | GPIO Pins |
|-----------|-----------|
| Motor 1 (FL) | 4, 5, 6, 7 |
| Motor 2 (FR) | 15, 16, 17, 18 |
| Motor 3 (RL) | 8, 9, 10, 11 |
| Motor 4 (RR) | 12, 13, 14, 21 |
| **UART TX** (to ESP32 Dev) | 17 |
| **UART RX** (from ESP32 Dev) | 18 |

---

## 🎯 NEXT STEPS

Sau khi 4 motor hoạt động ổn định:
1. [ ] Kết nối ESP32 Dev để đọc encoder
2. [ ] Implement PID control
3. [ ] Tích hợp ROS2 trên Raspberry Pi
4. [ ] Thêm điều khiển từ Web/Mobile

---

📅 Last updated: December 2024
🤖 K-Patrol Bot Project
