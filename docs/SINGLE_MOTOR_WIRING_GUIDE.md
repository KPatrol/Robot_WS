# 🔌 HƯỚNG DẪN ĐẤU DÂY CHI TIẾT - TEST 1 MOTOR JGA25 VỚI BTS7960

> **Mục đích:** Test 1 motor + 1 driver để đảm bảo hoạt động chính xác trước khi mở rộng ra 4 motor

---

## 📦 THIẾT BỊ CẦN THIẾT

| STT | Thiết bị | Số lượng | Ghi chú |
|-----|----------|----------|---------|
| 1 | ESP32-S3 WROOM-1 Dev Board | 1 | Board màu đen với USB-C |
| 2 | BTS7960 Motor Driver | 1 | Board màu xanh dương (IBT-2) |
| 3 | Motor JGA25-370 12V | 1 | Động cơ có hộp số + encoder màu đỏ |
| 4 | Nguồn 12V | 1 | Tối thiểu 3A (adapter hoặc pin) |
| 5 | Nguồn 5V | 1 | Có thể dùng từ USB hoặc riêng |
| 6 | Dây nối Dupont | ~20 sợi | Đực-đực, đực-cái |
| 7 | Cáp USB-C | 1 | Để nạp code cho ESP32-S3 |

---

## 🎯 SƠ ĐỒ TỔNG QUAN

```
        ┌─────────────┐
        │  Nguồn 12V  │
        │  (3A+)      │
        └──────┬──────┘
               │
        ┏━━━━━━┻━━━━━━┓
        ▼              ▼
┌──────────────┐   ┌──────────────┐
│  BTS7960     │   │  ESP32-S3    │
│  Driver      │◄──┤  WROOM-1     │
│  (IBT-2)     │   │              │
└──────┬───────┘   └──────────────┘
       │                   ▲
       │                   │
       ▼                   │
┌──────────────┐          USB
│  Motor JGA25 │
│  + Encoder   │
└──────────────┘
```

---

## 🔧 PHẦN 1: ĐẤU DÂY BTS7960 DRIVER

### Mô tả BTS7960 (Board IBT-2)

BTS7960 có **2 hàng chân**:

#### A. HÀNG CHÂN LOGIC (Logic Side - 8 pins)
```
┌─────────────────────────────────────────────────────┐
│  [1]    [2]    [3]    [4]    [5]    [6]    [7]  [8] │
│ RPWM   LPWM   R_EN   L_EN   R_IS   L_IS   VCC   GND │
└──┬─────┬─────┬─────┬─────────────────────┬─────┬───┘
   │     │     │     │                     │     │
 Xanh  Vàng  Xanh  Vàng                  Đỏ   Đen
 Lục   Lá   Dương  Cam                   (5V)  (GND)
```

#### B. HÀNG CHÂN NGUỒN (Power Side - 4 pins)
```
┌───────────────────────────────────┐
│   [1]     [2]     [3]      [4]    │
│   B-      B+      M+       M-     │
└────┬──────┬───────┬────────┬──────┘
     │      │       │        │
    Đen    Đỏ    Đỏ Motor  Đen Motor
   (GND)  (12V)  (Dương)   (Âm)
```

### Chi Tiết Đấu Dây

#### 1. Dây từ ESP32-S3 → BTS7960 (Logic)

| Chân ESP32-S3 | Màu dây (gợi ý) | Chân BTS7960 | Chức năng |
|---------------|-----------------|--------------|-----------|
| **GPIO 4** | 🟦 Xanh dương | **RPWM** | PWM tiến (Forward) |
| **GPIO 5** | 🟨 Vàng | **LPWM** | PWM lùi (Backward) |
| **GPIO 6** | 🟩 Xanh lá | **R_EN** | Enable kênh tiến |
| **GPIO 7** | 🟧 Cam | **L_EN** | Enable kênh lùi |
| **5V** hoặc **3V3** | 🔴 Đỏ | **VCC** | Nguồn logic 5V |
| **GND** | ⚫ Đen | **GND** | Mass chung |

**⚠️ LƯU Ý:**
- VCC của BTS7960 nhận được cả 3.3V và 5V, nhưng **khuyến nghị dùng 5V** để tín hiệu ổn định hơn
- **GND phải nối chung** giữa ESP32-S3, BTS7960 và nguồn 12V

#### 2. Dây nguồn 12V → BTS7960 (Power)

| Nguồn 12V | Màu dây | Chân BTS7960 | Ghi chú |
|-----------|---------|--------------|---------|
| **+12V** (Dương) | 🔴 Đỏ dày | **B+** | Nguồn chính cho motor |
| **GND** (Âm) | ⚫ Đen dày | **B-** | Mass nguồn 12V |

**⚠️ QUAN TRỌNG:**
- Dùng dây **đủ dày** (tối thiểu AWG 18) cho dòng 12V
- Kiểm tra **cực tính** cẩn thận trước khi cấp điện
- Nối **B- (GND 12V)** với **GND của ESP32-S3** để tạo mass chung

#### 3. Dây motor JGA25 → BTS7960

Motor JGA25 có **6 dây**:

| Màu dây Motor | Chức năng | Kết nối đến |
|---------------|-----------|-------------|
| 🔴 **Đỏ** | Motor + (Dương) | **M+** của BTS7960 |
| ⚫ **Đen** | Motor - (Âm) | **M-** của BTS7960 |
| 🟡 **Vàng** | Encoder Phase A | *(Chưa dùng trong test này)* |
| 🟢 **Xanh lá** | Encoder Phase B | *(Chưa dùng trong test này)* |
| 🔴 **Đỏ** (mỏng) | Encoder VCC | *(Chưa dùng trong test này)* |
| ⚫ **Đen** (mỏng) | Encoder GND | *(Chưa dùng trong test này)* |

**Trong test này chỉ dùng 2 dây motor chính (Đỏ + Đen dày)**

---

## ⚡ PHẦN 2: SƠ ĐỒ KẾT NỐI ĐẦY ĐỦ

```
┌──────────────────────────────────────────────────────────────┐
│                    NGUỒN 12V (3A+)                           │
│                    ┌───┐  ┌───┐                              │
│                    │ + │  │ - │                              │
└────────────────────┴─┬─┴──┴─┬─┴──────────────────────────────┘
                       │      │
                       │      └──────────┐
                       │                 │
                  ┌────┴────┐       ┌────┴────┐
                  │   B+    │       │   B-    │
           ┌──────┴─────────┴───────┴─────────┴──────┐
           │         BTS7960 DRIVER (IBT-2)          │
           │  ┌──────────────────────────────────┐   │
           │  │  LOGIC PINS                      │   │
           │  │  RPWM LPWM R_EN L_EN VCC  GND    │   │
           │  └────┬───┬───┬───┬─────┬────┬─────┘   │
           │       │   │   │   │     │    │         │
           │  ┌────┴───┴───┴───┴─────┴────┴─────┐   │
           │  │  M+                         M-   │   │
           │  └──┬─────────────────────────┬────┘   │
           └─────┼─────────────────────────┼────────┘
                 │                         │
            ┌────┴────┐               ┌────┴────┐
            │  Đỏ     │               │  Đen    │
            │ Motor   │               │ Motor   │
            └─────────┴───────────────┴─────────┘
                   Motor JGA25-370
                   
                        ▲       ▲       ▲      ▲
                        │       │       │      │
                      GPIO4   GPIO5   GPIO6  GPIO7
                        │       │       │      │
            ┌───────────┴───────┴───────┴──────┴──────┐
            │           ESP32-S3 WROOM-1               │
            │  GPIO4  GPIO5  GPIO6  GPIO7   5V   GND  │
            │                                          │
            │              [USB-C]                     │
            │                 │                        │
            └─────────────────┼────────────────────────┘
                              │
                          Computer
                      (Nạp code + Serial)
```

---

## 🔌 PHẦN 3: BẢNG ĐẤU DÂY TỔNG HỢP

### Bảng kết nối đầy đủ

| Từ | Chân | Màu (gợi ý) | → Đến | Chân | Ghi chú |
|----|------|-------------|-------|------|---------|
| **ESP32-S3** | GPIO 4 | 🟦 Xanh | **BTS7960** | RPWM | PWM Forward |
| **ESP32-S3** | GPIO 5 | 🟨 Vàng | **BTS7960** | LPWM | PWM Backward |
| **ESP32-S3** | GPIO 6 | 🟩 Xanh lá | **BTS7960** | R_EN | Enable Forward |
| **ESP32-S3** | GPIO 7 | 🟧 Cam | **BTS7960** | L_EN | Enable Backward |
| **ESP32-S3** | 5V | 🔴 Đỏ | **BTS7960** | VCC | Logic 5V |
| **ESP32-S3** | GND | ⚫ Đen | **BTS7960** | GND | Mass logic |
| **Nguồn 12V** | + | 🔴 Đỏ dày | **BTS7960** | B+ | Nguồn 12V |
| **Nguồn 12V** | - | ⚫ Đen dày | **BTS7960** | B- | GND 12V |
| **Motor** | Đỏ | 🔴 | **BTS7960** | M+ | Motor dương |
| **Motor** | Đen | ⚫ | **BTS7960** | M- | Motor âm |

**⚠️ MASS CHUNG (Rất quan trọng):**
```
ESP32 GND ─┬─ BTS7960 GND ─┬─ Nguồn 12V (-)
           │                │
       Tất cả phải nối chung!
```

---

## ✅ CHECKLIST TRƯỚC KHI BẬT NGUỒN

Kiểm tra kỹ các bước sau:

- [ ] **1.** ESP32-S3 đã nối 4 dây tín hiệu (GPIO 4, 5, 6, 7) đến BTS7960
- [ ] **2.** ESP32-S3 GND đã nối với BTS7960 GND
- [ ] **3.** BTS7960 VCC đã nối với ESP32-S3 5V (hoặc nguồn 5V riêng)
- [ ] **4.** Nguồn 12V (+) đã nối đúng vào B+ của BTS7960
- [ ] **5.** Nguồn 12V (-) đã nối đúng vào B- của BTS7960
- [ ] **6.** B- (GND 12V) đã nối chung với GND của ESP32-S3
- [ ] **7.** Motor dây Đỏ đã nối vào M+ của BTS7960
- [ ] **8.** Motor dây Đen đã nối vào M- của BTS7960
- [ ] **9.** USB-C đã kết nối ESP32-S3 với máy tính
- [ ] **10.** Đã nạp code `test_single_motor_bts7960.ino` vào ESP32-S3

---

## 🚀 PHẦN 4: NẠP CODE VÀO ESP32-S3

### Bước 1: Cài đặt Arduino IDE

1. Tải **Arduino IDE 2.x**: https://www.arduino.cc/en/software
2. Cài đặt board ESP32:
   - Mở Arduino IDE
   - File → Preferences → **Additional Board Manager URLs**:
   ```
   https://espressif.github.io/arduino-esp32/package_esp32_index.json
   ```
   - Tools → Board → Boards Manager
   - Tìm "**esp32**" by Espressif Systems
   - Cài đặt phiên bản **3.0.0** trở lên (khuyến nghị 3.0.7)

### Bước 2: Cấu hình Board

Chọn đúng board và cấu hình:

```
Tools → Board → ESP32 Arduino → "ESP32S3 Dev Module"

Cấu hình chi tiết:
├─ USB CDC On Boot: "Enabled"
├─ CPU Frequency: "240MHz (WiFi)"
├─ Flash Mode: "QIO 80MHz"
├─ Flash Size: "16MB (128Mb)"
├─ Partition Scheme: "Default 4MB with spiffs"
├─ PSRAM: "Disabled"
├─ Upload Mode: "UART0 / Hardware CDC"
├─ Upload Speed: "921600"
└─ Port: (chọn cổng COM tương ứng, vd: COM3 hoặc /dev/ttyUSB0)
```

**Cách tìm Port:**
- **Windows:** Device Manager → Ports (COM & LPT) → tìm "USB Serial Port (COMx)"
- **macOS:** Terminal → `ls /dev/cu.*` → tìm `/dev/cu.usbserial-xxx`
- **Linux:** Terminal → `ls /dev/tty*` → tìm `/dev/ttyUSB0` hoặc `/dev/ttyACM0`

### Bước 3: Mở file code

1. Mở file: `robots/firmware/test_single_motor_bts7960/test_single_motor_bts7960.ino`
2. Kiểm tra code, đặc biệt phần GPIO pins:
   ```cpp
   #define RPWM_PIN    4
   #define LPWM_PIN    5
   #define R_EN_PIN    6
   #define L_EN_PIN    7
   ```

### Bước 4: Nạp code

1. Click nút **Upload** (→) hoặc Ctrl+U
2. Đợi quá trình compile và upload
3. Nếu lỗi "Failed to connect":
   - Giữ nút **BOOT** trên ESP32-S3
   - Nhấn nút **RESET** 1 lần
   - Thả nút **BOOT**
   - Upload lại

### Bước 5: Mở Serial Monitor

1. Tools → Serial Monitor
2. Chọn baud rate: **115200**
3. Line ending: **Newline**

Bạn sẽ thấy output:
```
========================================
  K-PATROL - TEST 1 MOTOR JGA25
  Driver: BTS7960
  Controller: ESP32-S3 WROOM-1
========================================

✅ System Ready!
⚠️  WARNING: Motor is DISABLED by default (Safe Mode)
```

---

## 🎮 PHẦN 5: TEST MOTOR

### Các lệnh có sẵn

| Lệnh | Mô tả | Ví dụ |
|------|-------|-------|
| `F` | Motor quay tiến | `F` |
| `B` | Motor quay lùi | `B` |
| `S` | Dừng motor (brake) | `S` |
| `OFF` | Tắt driver (thả trôi) | `OFF` |
| `+` | Tăng tốc độ (+10) | `+` |
| `-` | Giảm tốc độ (-10) | `-` |
| `100` | Đặt tốc độ cụ thể (0-255) | `150` |
| `H` | Hiển thị help | `H` |

### Quy trình test

1. **Kiểm tra an toàn:**
   - Nhấc robot lên khỏi mặt đất
   - Hoặc đặt trên giá đỡ

2. **Test tiến:**
   ```
   → F
   ```
   - Motor phải quay theo chiều tiến
   - Nếu quay ngược → đảo dây M+ và M-

3. **Test lùi:**
   ```
   → B
   ```
   - Motor phải quay ngược lại

4. **Test tốc độ:**
   ```
   → 50      (chậm)
   → F
   → 150     (trung bình)
   → F
   → 255     (tối đa)
   → F
   ```

5. **Dừng:**
   ```
   → S       (dừng nhưng vẫn giữ)
   → OFF     (thả trôi)
   ```

---

## ⚠️ XỬ LÝ SỰ CỐ

### ❌ Motor không quay

**Kiểm tra:**
1. ✓ Nguồn 12V đã cấp cho B+ và B-?
2. ✓ VCC (5V) đã cấp cho BTS7960?
3. ✓ GND đã nối chung?
4. ✓ 4 dây tín hiệu (GPIO 4,5,6,7) đã đấu đúng?
5. ✓ M+ và M- đã nối với motor?

**Debug:**
- Thêm vào `void setup()`:
  ```cpp
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  ledcWrite(RPWM_CHANNEL, 200);  // Test PWM cao
  ```

### ❌ Motor quay ngược chiều

**Giải pháp:**
- Đảo 2 dây motor: M+ ↔ M-

### ❌ Motor rung hoặc tiếng kêu lạ

**Nguyên nhân:**
- Tần số PWM không phù hợp
- Thay đổi:
  ```cpp
  #define PWM_FREQ  10000  // Thử 10kHz hoặc 20kHz
  ```

### ❌ ESP32-S3 không nhận lệnh Serial

**Kiểm tra:**
1. ✓ Baud rate đúng 115200?
2. ✓ Cáp USB có hỗ trợ data (không phải cable sạc)?
3. ✓ Driver USB đã cài đặt?

---

## 📚 THÔNG TIN KỸ THUẬT

### ESP32-S3 WROOM-1
- **Core:** Dual-core Xtensa LX7 @ 240MHz
- **RAM:** 512KB SRAM + 8MB PSRAM (optional)
- **Flash:** 16MB
- **GPIO:** 45 pins (có thể dùng PWM)
- **PWM:** LEDC (16 channels)

### BTS7960 Driver
- **Chip:** BTS7960 H-Bridge
- **Voltage:** 5.5V - 27V
- **Max Current:** 43A
- **PWM Frequency:** 25kHz max (khuyến nghị 5-20kHz)
- **Control:** PWM + Enable pins

### Motor JGA25-370
- **Voltage:** 12V DC
- **No-load Current:** ~200mA
- **Stall Current:** ~2.5A
- **Speed:** ~165 RPM (no load)
- **Encoder:** 11 PPR (44 CPR với quadrature)

---

## 📖 THAM KHẢO

- Arduino IDE: https://www.arduino.cc/en/software
- ESP32 Arduino Core: https://github.com/espressif/arduino-esp32
- BTS7960 Datasheet: [IBT-2 Documentation]
- LEDC PWM: https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/ledc.html

---

**📅 Last updated:** December 30, 2025  
**🤖 K-Patrol Project**
