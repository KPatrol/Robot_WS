# 🔌 Test Đơn Giản 1 Motor - Quick Start

## 📋 Tóm Tắt Nhanh

Test 1 motor JGA25 với 1 driver BTS7960 trên ESP32-S3 để đảm bảo hoạt động chính xác.

## 🎯 Mục Đích

- ✅ Kiểm tra driver BTS7960 hoạt động
- ✅ Kiểm tra motor JGA25 quay đúng chiều
- ✅ Test PWM speed control (0-255)
- ✅ Làm quen với cấu hình ESP32-S3

## 📦 Cần Gì?

| Thiết bị | Số lượng |
|----------|----------|
| ESP32-S3 WROOM-1 | 1 |
| BTS7960 Driver (IBT-2) | 1 |
| Motor JGA25-370 12V | 1 |
| Nguồn 12V 3A | 1 |
| Dây nối Dupont | ~15 sợi |
| Cáp USB-C | 1 |

## 🔌 Đấu Dây Nhanh

### ESP32-S3 → BTS7960
```
GPIO 4  → RPWM  (PWM tiến)
GPIO 5  → LPWM  (PWM lùi)
GPIO 6  → R_EN  (Enable tiến)
GPIO 7  → L_EN  (Enable lùi)
5V      → VCC   (Logic 5V)
GND     → GND   (Mass)
```

### Nguồn 12V → BTS7960
```
+12V → B+
GND  → B-
```

### Motor → BTS7960
```
Đỏ Motor  → M+
Đen Motor → M-
```

**⚠️ QUAN TRỌNG:** Nối GND chung (ESP32 + BTS7960 + Nguồn 12V)

## 🚀 Nạp Code

### 1. Cài Arduino IDE
- Download: https://www.arduino.cc/en/software
- Cài board ESP32 (version 3.0.7):
  ```
  File → Preferences → Additional Board URLs:
  https://espressif.github.io/arduino-esp32/package_esp32_index.json
  ```

### 2. Cấu hình Board
```
Tools → Board → "ESP32S3 Dev Module"
Tools → USB CDC On Boot → "Enabled"
Tools → Upload Speed → "921600"
```

### 3. Upload Code
- Mở: `test_single_motor_bts7960.ino`
- Click Upload (→)
- Nếu lỗi: giữ BOOT → nhấn RESET → thả BOOT

### 4. Mở Serial Monitor
- Baud rate: **115200**
- Gõ `H` để xem help

## 🎮 Cách Dùng

Mở Serial Monitor và gõ lệnh:

```
F      → Tiến (Forward)
B      → Lùi (Backward)
S      → Dừng (Stop)
OFF    → Tắt driver
+      → Tăng tốc
-      → Giảm tốc
150    → Đặt tốc độ = 150
H      → Help
```

## ✅ Test Thành Công Khi:

- [ ] Motor quay khi gõ `F`
- [ ] Motor quay ngược khi gõ `B`
- [ ] Motor dừng khi gõ `S`
- [ ] Thay đổi tốc độ hoạt động
- [ ] Chiều quay đúng (nếu ngược → đảo M+/M-)

## 📚 Tài Liệu Đầy Đủ

Xem chi tiết: [`SINGLE_MOTOR_WIRING_GUIDE.md`](SINGLE_MOTOR_WIRING_GUIDE.md)

## ⚠️ Xử Lý Lỗi

### Motor không quay
1. Kiểm tra nguồn 12V
2. Kiểm tra GND chung
3. Kiểm tra 4 dây tín hiệu (GPIO 4,5,6,7)

### Motor quay ngược
- Đảo dây M+ ↔ M- ở BTS7960

### ESP32 không nhận lệnh
- Kiểm tra baud rate = 115200
- Thử cáp USB khác
- Kiểm tra port COM

## 📸 Hình Ảnh Tham Khảo

### BTS7960 Driver (IBT-2)
- Board màu xanh dương
- 8 pins logic + 4 pins power
- Tản nhiệt phía dưới

### ESP32-S3 WROOM-1
- Board màu đen
- USB-C ở giữa
- 2 nút BOOT và RESET

### Motor JGA25-370
- Động cơ DC có hộp số
- Board encoder màu đỏ
- 6 dây (2 motor + 4 encoder)

---

**📅 Created:** December 30, 2025  
**🤖 K-Patrol Project - Single Motor Test**
