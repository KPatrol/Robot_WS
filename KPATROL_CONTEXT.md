# K-PATROL ROBOT - PROJECT CONTEXT & AI PROMPT

## 📋 TỔNG QUAN DỰ ÁN

**K-Patrol Bot** là robot tuần tra sử dụng 4 bánh xe Mecanum, cho phép di chuyển đa hướng (omnidirectional). Robot được điều khiển từ Raspberry Pi thông qua ESP32-S3 qua USB Serial.

---

## 🔧 KIẾN TRÚC PHẦN CỨNG

### Danh sách thiết bị:

| Thiết bị | Số lượng | Chức năng |
|----------|----------|-----------|
| **Raspberry Pi 4** | 1 | Điều khiển cấp cao, AI, Web server |
| **ESP32-S3 WROOM-1** | 1 | Vi điều khiển chính - điều khiển 4 driver motor |
| **ESP32 Dev Module** | 1 | Đọc encoder 4 động cơ (giảm tải GPIO) |
| **BTS7960 (IBT-2)** | 4 | Driver motor công suất cao (43A) |
| **JGA25-370 Motor** | 4 | Động cơ DC 12V + encoder + hộp số 1:34 |
| **Bánh Mecanum** | 4 | 2 trái + 2 phải (tạo hình X khi nhìn từ trên) |
| **Nguồn 12V** | 1 | Tối thiểu 10A cho 4 motor |

### Sơ đồ kết nối:

```
┌─────────────────┐    USB Serial    ┌─────────────────┐
│  Raspberry Pi 4 │◄────────────────►│   ESP32-S3      │
│  (Controller)   │   /dev/ttyACM0   │  Motor Control  │
└─────────────────┘                  └────────┬────────┘
                                              │ PWM + EN (GPIO)
                    ┌─────────────────────────┼─────────────────────────┐
                    ▼                         ▼                         ▼
              ┌──────────┐             ┌──────────┐             ┌──────────┐
              │ BTS7960  │             │ BTS7960  │             │ BTS7960  │
              │ Driver 1 │             │ Driver 2 │             │ Driver 3,4│
              └────┬─────┘             └────┬─────┘             └────┬─────┘
                   ▼                        ▼                        ▼
              [Motor FL]               [Motor FR]               [Motor BR, BL]
```

---

## 📍 GPIO ALLOCATION - ESP32-S3

### Motor Pin Mapping:

| Motor | Vị trí | RPWM | LPWM | R_EN | L_EN | PWM Channel |
|-------|--------|------|------|------|------|-------------|
| **M1** | Front-Left (FL) | GPIO 4 | GPIO 5 | GPIO 6 | GPIO 7 | Ch 0, 1 |
| **M2** | Front-Right (FR) | GPIO 15 | GPIO 16 | GPIO 17 | GPIO 18 | Ch 2, 3 |
| **M3** | Back-Right (BR) | GPIO 8 | GPIO 9 | GPIO 10 | GPIO 11 | Ch 4, 5 |
| **M4** | Back-Left (BL) | GPIO 12 | GPIO 13 | GPIO 14 | GPIO 21 | Ch 6, 7 |

### PWM Configuration:
- Frequency: **5000 Hz** (5 kHz)
- Resolution: **8-bit** (0-255)
- Library: LEDC (ESP32 built-in)

---

## 📍 GPIO ALLOCATION - ESP32 Dev Module (Encoder Reader)

| Motor | Encoder A (Yellow) | Encoder B (Green) |
|-------|-------------------|-------------------|
| **FR** | GPIO 34 | GPIO 35 |
| **FL** | GPIO 32 | GPIO 33 |
| **BR** | GPIO 25 | GPIO 26 |
| **BL** | GPIO 27 | GPIO 14 |

### Encoder Specifications (JGA25-370):
- PPR encoder disk: **11 pulses/revolution**
- Gear ratio: **1:34**
- Output shaft PPR: 11 × 34 = **374 pulses/revolution**
- With quadrature (×4): **1496 counts/revolution**

---

## 🔌 SƠ ĐỒ ĐẤU NỐI BTS7960

### Pinout:

```
┌──────────────────────────────────────────┐
│            BTS7960 MODULE                │
│                                          │
│  [Logic Side - 8 pins]                   │
│  RPWM  LPWM  R_EN  L_EN  R_IS  L_IS  VCC  GND │
│   │     │     │     │                │    │  │
│  ESP32 GPIO  GPIO  GPIO             5V  GND  │
│                                          │
│  [Power Side - 4 pins]                   │
│   B-    B+    M-    M+                   │
│   │     │     │     │                    │
│  GND   12V  Motor  Motor                 │
│ (12V)      (Black) (Red)                 │
└──────────────────────────────────────────┘
```

### Quan trọng:
- **GND CHUNG** bắt buộc giữa ESP32, BTS7960 và nguồn 12V
- Dây nguồn motor dùng **AWG 18** (dày)
- VCC logic dùng **5V** (khuyến nghị)

---

## 🔧 MOTOR ENCODER JGA25-370 (6 dây)

| Pin | Màu dây | Chức năng | Kết nối |
|-----|---------|-----------|---------|
| 1 | Đen (dày) | Motor - | BTS7960 M- |
| 2 | Đen (mảnh) | Encoder GND | ESP32 GND |
| 3 | Vàng | Encoder Phase A | ESP32 GPIO (interrupt) |
| 4 | Xanh lá | Encoder Phase B | ESP32 GPIO (interrupt) |
| 5 | Đỏ (mảnh) | Encoder VCC | ESP32 3.3V |
| 6 | Đỏ (dày) | Motor + | BTS7960 M+ |

---

## 🎮 GIAO THỨC SERIAL (115200 baud)

### Lệnh di chuyển:

```
F <speed>    - Forward (Tiến)
B <speed>    - Backward (Lùi)
SL <speed>   - Strafe Left (Sang trái)
SR <speed>   - Strafe Right (Sang phải)
L <speed>    - Rotate Left (Xoay trái)
R <speed>    - Rotate Right (Xoay phải)
DFL <speed>  - Diagonal Forward-Left
DFR <speed>  - Diagonal Forward-Right
DBL <speed>  - Diagonal Backward-Left
DBR <speed>  - Diagonal Backward-Right
```

### Lệnh dừng:

```
S            - Stop (Brake mode - giữ vị trí)
OFF          - Emergency OFF (disable tất cả driver)
```

### Lệnh test motor:

```
T1 <speed>   - Test Motor 1 (FL) độc lập
T2 <speed>   - Test Motor 2 (FR) độc lập
T3 <speed>   - Test Motor 3 (BR) độc lập
T4 <speed>   - Test Motor 4 (BL) độc lập
TA <speed>   - Test tất cả motor
```

### Lệnh điều khiển trực tiếp:

```
M <FL> <FR> <BR> <BL>  - Điều khiển từng bánh riêng biệt
```

### Parameters:
- **speed**: 0-255 (mặc định 150)
- Giá trị âm = quay ngược

---

## 🔄 LOGIC ĐIỀU KHIỂN BÁNH MECANUM

### Hướng lắp bánh (nhìn từ trên - roller tạo hình X):

```
       FRONT
     \       /
    FL[0]   FR[1]     (FL, BR: roller \)
                      (FR, BL: roller /)
    BL[3]   BR[2]
     /       \
       BACK
```

### Công thức di chuyển:

| Hướng | FL | FR | BR | BL |
|-------|----|----|----|----|
| **Forward** | + | + | + | + |
| **Backward** | - | - | - | - |
| **Strafe Right** | + | - | + | - |
| **Strafe Left** | - | + | - | + |
| **Rotate Right** | + | - | - | + |
| **Rotate Left** | - | + | + | - |
| **Diagonal FR** | + | 0 | + | 0 |
| **Diagonal FL** | 0 | + | 0 | + |
| **Diagonal BR** | 0 | - | 0 | - |
| **Diagonal BL** | - | 0 | - | 0 |

---

## 📁 CẤU TRÚC THƯ MỤC

```
robots/
├── pi-controller/              # 🖥️ Python scripts điều khiển từ Pi
│   ├── kpatrol_cli.py          # Command line interface
│   ├── kpatrol_keyboard.py     # Điều khiển bàn phím
│   ├── kpatrol_controller.py   # Full controller với UI
│   ├── kpatrol_diagnostics.py  # Công cụ chẩn đoán
│   ├── kpatrol_gamepad.py      # Điều khiển gamepad
│   ├── run.sh                  # Launcher script
│   └── README.md               # Hướng dẫn
│
├── firmware/                   # 🔧 Arduino firmware cho ESP32
│   ├── esp32_s3_motor_controller_pi/    # Firmware chính (Pi compatible)
│   ├── esp32_s3_motor_controller/       # Firmware cơ bản
│   ├── encoder_reader_4motors/          # Đọc encoder
│   ├── test_four_motors_mecanum/        # Test 4 motor Mecanum
│   ├── test_single_motor_bts7960/       # Test 1 motor
│   ├── test_two_motors_FR_FL/           # Test 2 motor
│   └── test_three_motors_FR_FL_BR/      # Test 3 motor
│
├── docs/                       # 📚 Tài liệu
│   ├── MECANUM_SETUP_GUIDE.md           # Hướng dẫn setup Mecanum
│   ├── SINGLE_MOTOR_WIRING_GUIDE.md     # Hướng dẫn đấu dây 1 motor
│   ├── TWO_MOTORS_WIRING_GUIDE.md       # Hướng dẫn đấu dây 2 motor
│   ├── PINOUT_DIAGRAM_*.txt             # Sơ đồ chân
│   └── ENCODER_WIRING_4MOTORS.txt       # Đấu dây encoder
│
├── test/                       # 🧪 Python test scripts
│   ├── setup_motor_wizard.py            # Wizard cấu hình motor
│   ├── test_single_motor.py             # Test 1 motor
│   └── test_mecanum_movement.py         # Test di chuyển Mecanum
│
└── KPATROL_CONTEXT.md          # 📋 File này - ngữ cảnh dự án
```

---

## 🚀 HƯỚNG DẪN SỬ DỤNG NHANH

### 1. Nạp firmware cho ESP32-S3:

```bash
# Mở Arduino IDE
# File: robots/firmware/esp32_s3_motor_controller_pi/esp32_s3_motor_controller_pi.ino
# Board: ESP32S3 Dev Module
# USB CDC On Boot: Enabled
# Upload Speed: 921600
```

### 2. Kết nối ESP32 với Pi:

```bash
# Cắm USB từ ESP32-S3 vào Raspberry Pi
# Kiểm tra cổng:
ls /dev/tty* | grep -E '(ACM|USB)'
# Thường là /dev/ttyACM0
```

### 3. Chạy controller trên Pi:

```bash
cd ~/robots/pi-controller

# Kiểm tra kết nối
kp-check

# Chạy diagnostics
python3 kpatrol_diagnostics.py

# Điều khiển CLI
python3 kpatrol_cli.py

# Điều khiển bàn phím
python3 kpatrol_keyboard.py
```

---

## ⚠️ LƯU Ý AN TOÀN

1. **Nguồn 12V** phải đủ dòng (tối thiểu 7A, khuyến nghị 10A+)
2. **GND chung** giữa ESP32, BTS7960 và nguồn 12V
3. **Nhấc robot lên** khi test motor
4. Sử dụng lệnh **OFF** trong trường hợp khẩn cấp
5. Kiểm tra **hướng lắp bánh Mecanum** (roller tạo hình X)

---

## 🔧 XỬ LÝ SỰ CỐ

### Motor không quay:
1. Kiểm tra nguồn 12V
2. Kiểm tra GND chung
3. Kiểm tra đèn LED trên BTS7960
4. Kiểm tra dây RPWM, LPWM, R_EN, L_EN

### Motor quay ngược:
- Đảo dây M+ và M- tại driver
- Hoặc set `*_INVERTED = true` trong firmware

### Không kết nối Serial:
```bash
# Kiểm tra quyền
sudo chmod 666 /dev/ttyACM0

# Thêm user vào group dialout
sudo usermod -a -G dialout $USER
# Logout và login lại
```

---

## 📡 API REFERENCE

### Python - Sử dụng SerialConnection:

```python
import serial

# Kết nối
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

# Gửi lệnh
ser.write(b'F 150\n')  # Tiến với speed 150

# Đọc response
response = ser.readline().decode()

# Đóng
ser.close()
```

### Direct Commands:

```python
# Di chuyển
ser.write(b'F 150\n')    # Forward
ser.write(b'B 150\n')    # Backward
ser.write(b'SL 150\n')   # Strafe Left
ser.write(b'SR 150\n')   # Strafe Right
ser.write(b'L 150\n')    # Rotate Left
ser.write(b'R 150\n')    # Rotate Right

# Dừng
ser.write(b'S\n')        # Stop (brake)
ser.write(b'OFF\n')      # Emergency OFF

# Test
ser.write(b'T1 100\n')   # Test Motor 1
ser.write(b'TA 100\n')   # Test All

# Direct control
ser.write(b'M 100 -100 100 -100\n')  # FL FR BR BL
```

---

## 🎯 MỤC TIÊU PHÁT TRIỂN

1. ✅ Điều khiển 4 motor Mecanum
2. ✅ Giao tiếp Serial với Pi
3. ✅ Python controller scripts
4. ⬜ PID control với encoder feedback
5. ⬜ Web/Mobile control interface
6. ⬜ Camera + AI vision
7. ⬜ RC controller (iBUS/SBUS)
8. ⬜ Autonomous navigation

---

## 📞 THÔNG TIN BỔ SUNG

- **Baud Rate**: 115200
- **Serial Port**: /dev/ttyACM0 (USB) hoặc /dev/ttyUSB0
- **Default Speed**: 150 (range: 0-255)
- **PWM Frequency**: 5000 Hz
- **PWM Resolution**: 8-bit

---

*Document Version: 2.0*
*Last Updated: January 2026*
