# 🔧 HƯỚNG DẪN GẮN IMU & TOF SENSORS LÊN K-PATROL ROBOT

> **Version**: 1.0 | **Date**: March 3, 2026
> **Purpose**: Hướng dẫn chi tiết mount IMU BNO08x và 6x ToF VL53L0X sensors

---

## 📋 MỤC LỤC

1. [Tổng quan vị trí sensors](#-tổng-quan-vị-trí-sensors)
2. [Gắn IMU BNO08x](#-gắn-imu-bno08x---quan-trọng-nhất)
3. [Gắn 6x ToF VL53L0X](#-gắn-6x-tof-vl53l0x)
4. [Đi dây và kết nối](#-đi-dây-và-kết-nối)
5. [Kiểm tra sau khi gắn](#-kiểm-tra-sau-khi-gắn)
6. [Troubleshooting](#-troubleshooting)

---

## 🗺️ TỔNG QUAN VỊ TRÍ SENSORS

### Robot Layout (Top View - Nhìn từ trên xuống)

```
                         FRONT (Hướng di chuyển chính)
                              ↑
                    ┌─────────┴─────────┐
                    │    FRONT ToF      │
                    │      (0°)         │
        ┌───────────┼───────────────────┼───────────────┐
        │           │                   │               │
        │  FL ToF   │                   │   FR ToF      │
        │  (30°)    │                   │   (-30°)      │
        │           │                   │               │
        ├───────────┤                   ├───────────────┤
        │           │                   │               │
        │  LEFT ToF │    ╔═══════╗      │  RIGHT ToF    │
        │  (90°)    │    ║  IMU  ║      │  (-90°)       │
        │           │    ║BNO08x ║      │               │
        ├───────────┤    ╚═══════╝      ├───────────────┤
        │           │                   │               │
        │    FL     │   [  ESP32-S3  ]  │    FR         │
        │   Motor   │   [Motor Ctrl  ]  │   Motor       │
        │           │                   │               │
        ├───────────┤                   ├───────────────┤
        │           │   [  ESP32 Dev ]  │               │
        │    BL     │   [ Sensor Hub ]  │    BR         │
        │   Motor   │                   │   Motor       │
        │           │                   │               │
        └───────────┼───────────────────┼───────────────┘
                    │                   │
                    │    BACK ToF       │
                    │     (180°)        │
                    └───────────────────┘
                              ↓
                          BACK

        Wheel Positions:
        FL ── FR    (Front wheels - Mecanum)
        │      │
        BL ── BR    (Back wheels - Mecanum)
```

---

## 🧭 GẮN IMU BNO08x - QUAN TRỌNG NHẤT

### ⚠️ NGUYÊN TẮC VÀNG CHO IMU

> **IMU PHẢI được gắn đúng cách để đọc YAW/PITCH/ROLL chính xác!**

### Vị trí gắn IMU

```
QUAN TRỌNG: Gắn IMU tại TRUNG TÂM của robot

Lý do:
1. Giảm thiểu ảnh hưởng của rung động từ motors
2. Center of rotation = center of IMU → đo chính xác
3. Cân bằng trọng lượng tốt hơn
4. Giảm sai số khi robot xoay

                    FRONT
                      ↑
        ┌─────────────────────────────┐
        │                             │
        │      [ESP32-S3 Board]       │
        │           ↓                 │
        │      ╔═════════╗            │
        │      ║   IMU   ║ ← Gắn      │
        │      ║ BNO08x  ║   tại đây! │
        │      ╚═════════╝            │
        │         │                   │
        │      [ESP32 DevKit]         │
        │                             │
        └─────────────────────────────┘
                      ↓
                    BACK
```

### Hướng gắn IMU (CRITICAL!)

```
BNO08x Module Orientation:
═══════════════════════════

Nhìn từ trên xuống, module BNO08x phải được gắn sao cho:

       ┌─────────────────┐
       │  ● ← Dấu chấm   │
       │   BNO08x        │  ← Mặt có chip quay LÊN TRÊN
       │                 │
       │  [Pin Headers]  │
       └────────┬────────┘
                │
                ↓
           Pins hướng ra SAU robot (về phía BACK)

QUAN TRỌNG - Quy ước trục:
═══════════════════════════
• X-axis: Hướng FRONT của robot (hướng tiến)
• Y-axis: Hướng LEFT của robot
• Z-axis: Hướng LÊN TRÊN (vuông góc mặt đất)

Kết quả đọc:
• YAW (0-360°): Góc xoay quanh trục Z (compass heading)
• PITCH: Nghiêng trước/sau (-90° đến 90°)
• ROLL: Nghiêng trái/phải (-180° đến 180°)
```

### Cách gắn vật lý IMU

```
VẬT LIỆU CẦN:
• Băng dính 2 mặt foam (chống rung)
• Hoặc: Standoff M2 + ốc + tấm đệm cao su

PHƯƠNG PHÁP 1: Băng dính foam (Khuyến khích)
═══════════════════════════════════════════
┌─────────────────────────────┐
│        BNO08x Module        │
├─────────────────────────────┤
│  ░░░░░ Foam tape ░░░░░░░░░ │  ← Foam 2-3mm chống rung
├─────────────────────────────┤
│       Robot Chassis         │
└─────────────────────────────┘

Bước thực hiện:
1. Làm sạch bề mặt chassis (alcohol)
2. Dán foam tape vào mặt dưới BNO08x
3. Căn chỉnh hướng (dấu chấm → FRONT)
4. Dán xuống TRUNG TÂM chassis
5. Đợi 30 phút trước khi test

PHƯƠNG PHÁP 2: Standoff M2
═══════════════════════════
    ┌─── BNO08x ───┐
    │   ┌─┐   ┌─┐  │
    │   │○│   │○│  │  ← Lỗ mounting M2
    └───┴─┴───┴─┴──┘
         │     │
    ┌────┴─────┴────┐
    │  Rubber pad   │  ← Đệm cao su chống rung
    ├───────────────┤
    │   Standoff    │  ← M2 x 10mm
    └───────┬───────┘
            │
    ════════╧════════  Chassis
```

### Khoảng cách từ motor

```
⚠️ QUAN TRỌNG: Giữ IMU CÁCH XA motors tối thiểu 5cm

Lý do:
• Motors tạo từ trường → ảnh hưởng magnetometer
• Rung động từ motors → nhiễu accelerometer
• Motor drivers (BTS7960) cũng tạo EMI

Khuyến nghị:
┌─────────────────────────────────────┐
│                                     │
│  [Motor]              [Motor]       │
│     │                    │          │
│     │    ≥ 5cm          │          │
│     │      │             │          │
│     └──────┼─────────────┘          │
│            │                        │
│       ╔════╧════╗                   │
│       ║   IMU   ║                   │
│       ╚═════════╝                   │
│                                     │
│  [Motor]              [Motor]       │
└─────────────────────────────────────┘
```

### Calibration IMU sau khi gắn

```bash
# Sau khi gắn IMU, cần calibrate:

1. Power on robot
2. Đặt robot trên mặt phẳng
3. Từ từ xoay robot theo hình số 8 (3-5 lần)
4. Kiểm tra accuracy qua serial:
   
   # Trên Pi
   echo "IMU" > /dev/ttyACM0
   cat /dev/ttyACM0
   
   # Output: IMU:yaw,pitch,roll,accuracy
   # accuracy < 10° = đã calibrate tốt
```

---

## 👁️ GẮN 6x TOF VL53L0X

### Thông số ToF Sensors

| Parameter | Value |
|-----------|-------|
| Model | VL53L0X |
| Range | 30mm - 2000mm |
| FOV | 25° cone |
| Interface | I2C via TCA9548A |
| Voltage | 3.3V - 5V |

### Vị trí gắn 6 sensors

```
TOP VIEW - Vị trí ToF sensors
══════════════════════════════

                    FRONT
                      ↑
              ┌───────┴───────┐
              │     ToF 0     │
              │   (FRONT)     │
              │      0°       │
    ┌─────────┼───────────────┼─────────┐
    │         │               │         │
    │  ToF 1  │               │  ToF 2  │
    │  (FL)   │               │  (FR)   │
    │  +30°   │               │  -30°   │
    │    ↖    │               │    ↗    │
    ├─────────┤               ├─────────┤
    │         │               │         │
    │  ToF 3  │    ROBOT      │  ToF 4  │
    │  (LEFT) │    BODY       │  (RIGHT)│
    │  +90°   │               │  -90°   │
    │    ←    │               │    →    │
    │         │               │         │
    ├─────────┤               ├─────────┤
    │         │               │         │
    │         │               │         │
    │         │               │         │
    └─────────┼───────────────┼─────────┘
              │     ToF 5     │
              │    (BACK)     │
              │    180°       │
              └───────┬───────┘
                      ↓
                    BACK
```

### Chi tiết từng ToF sensor

#### ToF 0 - FRONT (Channel 0)

```
Vị trí: Mặt trước robot, CHÍNH GIỮA
Góc: 0° (nhìn thẳng)
Mục đích: Phát hiện vật cản phía trước

Cách gắn:
┌─────────────────────────────┐
│                             │
│    ┌─────────┐              │
│    │ VL53L0X │ ← Mount      │
│    │   ■→    │   flush với  │
│    └────┬────┘   mặt trước  │
│         │                   │
│  ROBOT  │                   │
│  BODY   │                   │
└─────────┴───────────────────┘
          │
          ↓ Hướng laser

Độ cao: Cách mặt đất 8-12cm
        (để detect chân người, vật cản thấp)
```

#### ToF 1 - FRONT_LEFT (Channel 1)

```
Vị trí: Góc trước-trái
Góc: +30° so với trục FRONT
Mục đích: Phát hiện góc chéo trái

       ↖ 30°
        \
         \
    ┌─────\───────────────────┐
    │      \                  │
    │  ┌────\┐                │
    │  │ToF 1│                │
    │  └─────┘                │
    │                         │
    └─────────────────────────┘
```

#### ToF 2 - FRONT_RIGHT (Channel 2)

```
Vị trí: Góc trước-phải
Góc: -30° so với trục FRONT
Mục đích: Phát hiện góc chéo phải

                         30° ↗
                          /
                         /
    ┌───────────────────/─────┐
    │                  /      │
    │                ┌/───┐   │
    │                │ToF 2   │
    │                └────┘   │
    │                         │
    └─────────────────────────┘
```

#### ToF 3 - LEFT (Channel 3)

```
Vị trí: Mặt bên TRÁI
Góc: +90° (vuông góc, nhìn sang trái)
Mục đích: Phát hiện vật cản bên trái

    ← ToF 3 nhìn ra
    ┌───┐
    │   │
    │ToF│
    │ 3 │
    │   │
    ┌───┼───────────────────┐
    │   │                   │
    │   │   ROBOT BODY      │
    │   │                   │
    └───┴───────────────────┘
```

#### ToF 4 - RIGHT (Channel 4)

```
Vị trí: Mặt bên PHẢI
Góc: -90° (vuông góc, nhìn sang phải)
Mục đích: Phát hiện vật cản bên phải

                         ToF 4 nhìn ra →
                              ┌───┐
                              │   │
                              │ToF│
                              │ 4 │
                              │   │
    ┌─────────────────────────┼───┐
    │                         │   │
    │       ROBOT BODY        │   │
    │                         │   │
    └─────────────────────────┴───┘
```

#### ToF 5 - BACK (Channel 5)

```
Vị trí: Mặt sau robot, CHÍNH GIỮA
Góc: 180° (nhìn ra sau)
Mục đích: Phát hiện vật cản khi lùi

    ┌─────────────────────────┐
    │                         │
    │       ROBOT BODY        │
    │                         │
    └─────────┬───────────────┘
              │
         ┌────┴────┐
         │ VL53L0X │
         │    ■    │
         └────┬────┘
              │
              ↓ Hướng laser (ra sau)
```

### Độ cao mounting ToF sensors

```
SIDE VIEW - Đo chiều cao mounting
═══════════════════════════════════

            ToF sensor
               ■
               │ Laser beam
               │
   ─ ─ ─ ─ ─ ─ │─ ─ ─ ─ ─ ─ ─ ─ ─  ← 10cm từ mặt đất
               │
               │
   ════════════╧══════════════════  ← Mặt đất

KHUYẾN NGHỊ:
• FRONT, FL, FR: 10cm từ mặt đất
• LEFT, RIGHT: 10cm từ mặt đất  
• BACK: 8-10cm từ mặt đất

Lý do 10cm:
1. Detect được chân người (ankle level)
2. Detect được chân ghế, bàn
3. Tránh detect mặt đất khi robot nghiêng
```

### Bracket mounting ToF

```
CÁCH GẮN BRACKET CHO TOF:
═══════════════════════════

Có thể dùng:
1. In 3D bracket góc (recommended)
2. Bracket nhôm L-shape
3. Keo nóng trực tiếp (tạm thời)

Ví dụ bracket góc cho FRONT:

        ┌─────────────┐
        │   VL53L0X   │
        │      ■      │
        └──────┬──────┘
               │
        ┌──────┴──────┐
        │   Bracket   │ ← Góc 90°
        │      L      │
        └─────────────┘
               │
        ═══════╧═══════  Chassis
```

---

## 🔌 ĐI DÂY VÀ KẾT NỐI

### IMU BNO08x → ESP32-S3

```
BNO08x          ESP32-S3 Motor Controller
═══════════════════════════════════════════
  VIN    ────────→  3.3V
  GND    ────────→  GND
  SDA    ────────→  GPIO 35 (RX)  ← ESP32 nhận từ BNO08x
  SCL    ────────→  GPIO 36 (TX)  ← ESP32 gửi tới BNO08x
  RST    ────────→  GPIO 37

⚠️ CHÚ Ý: BNO08x ở UART mode (PS1=HIGH, PS0=LOW)
   - SDA pin = TX output của BNO08x
   - SCL pin = RX input của BNO08x
```

### ToF Sensors → TCA9548A → ESP32 DevKit

```
6x VL53L0X   →   TCA9548A   →   ESP32 DevKit
════════════════════════════════════════════

VL53L0X (mỗi cái)    TCA9548A            ESP32 DevKit
    VIN  ─────────→  VIN (3.3V)  ←────→  3.3V
    GND  ─────────→  GND         ←────→  GND
    SDA  ─────────→  SD0-SD5     
    SCL  ─────────→  SC0-SC5     
                     SDA         ←────→  GPIO 21
                     SCL         ←────→  GPIO 22

TCA9548A Channel Assignment:
═══════════════════════════════════
Channel 0 (SD0/SC0)  →  ToF FRONT
Channel 1 (SD1/SC1)  →  ToF FRONT_LEFT
Channel 2 (SD2/SC2)  →  ToF FRONT_RIGHT
Channel 3 (SD3/SC3)  →  ToF LEFT
Channel 4 (SD4/SC4)  →  ToF RIGHT
Channel 5 (SD5/SC5)  →  ToF BACK
```

### Sơ đồ dây tổng hợp

```
                    ┌──────────────────────────────────────┐
                    │          RASPBERRY PI 4B             │
                    │                                      │
                    │  USB-A ═══════╗     USB-A ═══════╗  │
                    └───────────────║─────────────────║───┘
                                    ║                 ║
                              USB Cable          USB Cable
                                    ║                 ║
                                    ↓                 ↓
┌───────────────────────────────────────┐    ┌───────────────────────┐
│           ESP32-S3 Motor Ctrl         │    │    ESP32 DevKit       │
│           /dev/ttyACM0                │    │    /dev/ttyUSB0       │
│                                       │    │                       │
│  GPIO 35 ←─── BNO08x SDA (TX)        │    │  GPIO 21 ←──→ TCA SDA │
│  GPIO 36 ───→ BNO08x SCL (RX)        │    │  GPIO 22 ←──→ TCA SCL │
│  GPIO 37 ───→ BNO08x RST             │    │                       │
│                                       │    │          ↓            │
│  GPIO 38 ───→ Relay Warning Light    │    │    ┌──────────┐       │
│  GPIO 39 ───→ Relay Main Light       │    │    │ TCA9548A │       │
│                                       │    │    │  0x70    │       │
│  Motor GPIO: 4-21 → BTS7960 drivers  │    │    └────┬─────┘       │
│                                       │    │         │             │
└───────────────────────────────────────┘    │    ┌────┴────┐        │
         │                                   │    ↓    ↓   ↓        │
         ↓                                   │  ToF  ToF  ...       │
    ┌─────────┐                              │  0-5 sensors         │
    │ BNO08x  │                              │                       │
    │   IMU   │                              │  Encoder pins:       │
    └─────────┘                              │  34-35, 32-33,       │
                                             │  25-26, 27-14        │
                                             └───────────────────────┘
```

### Chiều dài dây khuyến nghị

```
CHIỀU DÀI DÂY:
══════════════════════════════════════════
• IMU → ESP32-S3:        10-15cm (ngắn, tránh nhiễu)
• TCA9548A → ESP32 Dev:  10-15cm
• ToF → TCA9548A:        15-30cm (tùy vị trí mount)
• Pi → ESP32 (USB):      30-50cm

⚠️ QUAN TRỌNG:
• Dây I2C (SDA/SCL) nên ngắn nhất có thể
• Dùng dây đôi xoắn (twisted pair) cho I2C
• Tránh đi dây song song với dây nguồn motor
```

---

## ✅ KIỂM TRA SAU KHI GẮN

### 1. Kiểm tra IMU

```bash
# SSH vào Pi
ssh khoavd@192.168.199.108

# Test IMU trực tiếp
echo "IMU" > /dev/ttyACM0
cat /dev/ttyACM0

# Expected output:
# IMU:45.2,-1.3,0.8,5.0
#     yaw, pitch, roll, accuracy

# Kiểm tra:
✓ Yaw thay đổi khi xoay robot (0-360°)
✓ Pitch thay đổi khi nghiêng trước/sau
✓ Roll thay đổi khi nghiêng trái/phải
✓ Accuracy < 10° sau calibration
```

### 2. Kiểm tra ToF Sensors

```bash
# Test ToF từ ESP32 DevKit
# Qua serial monitor Arduino IDE hoặc:

echo "T" > /dev/ttyUSB0
cat /dev/ttyUSB0

# Expected output:
# TOF:250,300,280,450,520,800
#     front,fl,fr,left,right,back (mm)

# Kiểm tra từng sensor:
✓ Đưa tay trước FRONT → giá trị giảm
✓ Đưa tay bên LEFT → giá trị left giảm
✓ Tất cả 6 sensors phản hồi (không có 0 hoặc 8191)
```

### 3. Kiểm tra qua MQTT

```bash
# Subscribe để xem data
mosquitto_sub -h 103.81.84.43 \
  -u alphaasimov2024 -P gvB3DtGfus6U \
  -t 'kpatrol/sensors' -t 'kpatrol/imu'

# Kết quả mong đợi:
# {"tof":{"front":250,"front_left":300,...}}
# {"yaw":45.2,"pitch":-1.3,"roll":0.8,"accuracy":5.0}
```

### 4. Checklist xác nhận

```
IMU BNO08x:
□ Gắn tại TRUNG TÂM robot
□ Dấu chấm/X-axis hướng về FRONT
□ Cách motors ≥ 5cm
□ Có đệm chống rung (foam/rubber)
□ Yaw phản hồi khi xoay robot
□ Accuracy < 10° sau calibration

ToF Sensors:
□ FRONT (Ch 0): Chính giữa mặt trước, 0°
□ FL (Ch 1): Góc trước-trái, +30°
□ FR (Ch 2): Góc trước-phải, -30°
□ LEFT (Ch 3): Mặt trái, +90°
□ RIGHT (Ch 4): Mặt phải, -90°
□ BACK (Ch 5): Chính giữa mặt sau, 180°
□ Tất cả sensors cách mặt đất ~10cm
□ Không có sensor bị che khuất
□ I2C scan thấy TCA9548A (0x70)
□ All 6 sensors return valid values

Wiring:
□ IMU dây ≤ 15cm
□ I2C dây xoắn đôi
□ Không đi song song dây nguồn motor
□ Connectors chắc chắn
```

---

## 🔧 TROUBLESHOOTING

### IMU không đọc được

```
1. Kiểm tra mode UART:
   - PS1 = HIGH (đã solder sẵn trên Adafruit board)
   - PS0 = không nối (LOW)

2. Kiểm tra baudrate: 3,000,000 (3Mbps)

3. Kiểm tra GPIO:
   - ESP32 RX (GPIO 35) ← BNO08x TX (SDA pin)
   - ESP32 TX (GPIO 36) → BNO08x RX (SCL pin)

4. Reset IMU:
   - Pull RST (GPIO 37) LOW 100ms
   - Release to HIGH
```

### ToF sensors không detect

```
1. Chạy I2C Scanner:
   echo "I" > /dev/ttyUSB0
   # Phải thấy 0x70 (TCA9548A)

2. Kiểm tra từng channel:
   # Trong firmware có debug mode

3. Kiểm tra nguồn:
   - TCA9548A và VL53L0X cần 3.3V ổn định
   - Nếu dùng 5V, check regulator

4. Kiểm tra dây I2C:
   - Pull-up resistor 4.7K trên SDA/SCL
   - Dây ngắn < 30cm
```

### Giá trị IMU không ổn định

```
1. Rung động từ motors:
   - Thêm đệm foam dày hơn
   - Di chuyển IMU xa motors hơn

2. Từ trường nhiễu:
   - Tránh đặt gần motor drivers
   - Tránh gần dây nguồn DC

3. Cần re-calibrate:
   - Xoay robot theo hình số 8 chậm
   - 3-5 vòng cho đến khi accuracy < 10°
```

### ToF đọc giá trị 8191 (max range)

```
Nguyên nhân:
- Sensor không gắn đúng hướng
- Bề mặt đối diện hấp thụ IR (đen, nhung)
- Khoảng cách > 2m

Giải pháp:
- Kiểm tra hướng laser của sensor
- Test với vật thể trắng/sáng màu
```

---

## 📝 NOTES

1. **Làm tuần tự**: Gắn IMU trước, test xong mới gắn ToF
2. **Chụp ảnh**: Document vị trí gắn để reference sau
3. **Label dây**: Đánh dấu ToF channel trên dây
4. **Backup config**: Lưu firmware trước khi modify

---

## 📚 REFERENCES

- BNO08x Datasheet: Adafruit
- VL53L0X Datasheet: STMicroelectronics
- TCA9548A Datasheet: Texas Instruments
- K-Patrol PromptV8: `prompt/PromptV8/`

---

*Last updated: March 3, 2026*
