# Hướng dẫn lắp đặt thiết bị ngoại vi bổ sung — K-Patrol

> **Phạm vi**: tích hợp 6 thiết bị mới vào hệ thống đã có (ESP32-S3 motor + ESP32 Dev encoder + Raspberry Pi compute).
> **Kiến trúc đã chọn**: **ESP32 D1 R32 WeMos làm peripheral hub mới** (form factor Arduino UNO R3, chip ESP32-WROOM-32 onboard), KHÔNG dùng Pi GPIO/I²C. Pi chỉ nhận data qua USB-Serial.
> **Mục tiêu**: không sửa firmware 2 board đang ổn (S3 motor + Dev encoder), tận dụng D1 R32 dư.
>
> ℹ️ **Pin I²C của D1 R32**: header có **label "SDA" và "SCL"** được nối **đúng vào GPIO 21 và GPIO 22** — chính là pin I²C mặc định của ESP32, dùng trực tiếp với `Wire.begin()` (không cần arg). Đây là 2 pin Digital I/O bình thường, OUTPUT được.
> ⚠️ **Cẩn thận pin Analog**: phía bên trái có header `GPIO36` và `GPIO39` là **INPUT-ONLY** — chỉ dùng đọc analog, KHÔNG output được. Tránh nhầm với SDA/SCL.

---

## 1. Tổng quan thiết bị cần lắp

| # | Thiết bị | Giao tiếp | Nguồn | Địa chỉ / chân |
|---|----------|-----------|-------|----------------|
| 1 | INA219 — đo dòng + áp pin | I²C | 3.3V | 0x40 (7-bit) |
| 2 | DHT11 — nhiệt độ + độ ẩm | 1-wire (single GPIO) | 3.3V | — |
| 3 | OLED 1.3" SH1106 (bán dưới tên SSD1306) | I²C | 3.3V | 0x3C (7-bit) ≡ 0x78 (8-bit) |
| 4 | Module Relay 5V kích (1 kênh) | GPIO digital | 5V coil | — |
| 5 | Đèn 12V + còi tích hợp | Qua tiếp điểm relay | 12V (pin chính) | — |
| 6 | 2× module Laser 5V | GPIO digital qua transistor | 5V | — |

---

## 2. Kiến trúc đã chọn: D1 R32 hub độc lập

### 2.1 Sơ đồ tổng thể

```
┌──────────────────────────────────────┐    UART /dev/ttyACM0       ┌────────────────────────────┐
│ ESP32-S3 (motor + IMU + INA219)     │◄──────────────────────────►│                            │
│   • 4× motor BTS7960                 │                            │                            │
│   • BNO08x IMU                       │                            │                            │
│   • GPS NEO-6M                       │                            │                            │
│   • INA219 ★ (đã có firmware)        │                            │                            │
└──────────────────────────────────────┘                            │                            │
                                                                     │                            │
┌──────────────────────────────────────┐    UART /dev/ttyACM1       │  Raspberry Pi 4            │
│ ESP32 Dev (encoder reader + ToF)    │◄──────────────────────────►│  (compute + MQTT + camera) │
│   • 4× encoder quadrature            │                            │                            │
│   • TCA9548A + 6× VL53L0X ToF        │                            │                            │
└──────────────────────────────────────┘                            │                            │
                                                                     │                            │
┌──────────────────────────────────────┐    UART /dev/ttyUSB? ★ NEW │                            │
│ ESP32 D1 R32 WeMos ★ (periph hub)    │◄──────────────────────────►│                            │
│   • I²C OLED 1.3" (0x3C) qua header  │                            │                            │
│     SDA=GPIO21, SCL=GPIO22 (default) │                            └────────────────────────────┘
│   • DHT11 (1-wire) qua GPIO 25 (D3)  │
│   • Relay 5V → đèn+còi 12V (GPIO 26) │
│   • 2× Laser 5V qua 2N2222           │
│     (GPIO 27 + GPIO 16)              │
└──────────────────────────────────────┘
```

### 2.2 Phân bổ thiết bị

| Thiết bị | Lắp ở | Cơ sở quyết định |
|----------|-------|------------------|
| INA219 | **ESP32-S3** (không đổi) | Firmware đã có sẵn (`Adafruit_INA219`, I²C SDA=19, SCL=20, addr 0x40). Chỉ đi dây vật lý. |
| OLED, DHT11, Relay, Lasers | **ESP32 D1 R32 mới** | Cô lập khỏi motor real-time + encoder loop. Tận dụng board D1 R32 dư. |

### 2.3 Tại sao chọn D1 R32 hub thay vì dồn vào S3/Dev?

- ✅ **Không sửa 2 firmware đang ổn** (S3 motor + Dev encoder) → giảm nguy cơ regression
- ✅ **Cô lập lỗi** — sensor hub crash không ảnh hưởng motor hay encoder
- ✅ **Tận dụng** ESP32 D1 R32 WeMos dư (không phải mua thêm)
- ✅ **D1 R32 có barrel jack 7-12V** → cấp nguồn trực tiếp từ pin LFP 12V qua cầu chì (KHÔNG cần buck riêng cho board, vẫn cần buck 5V cho relay+laser)
- ✅ **Pi 4 còn 2 cổng USB trống** sau khi đã cắm S3 + Dev → đủ cho D1 R32
- ✅ **Firmware mới ngắn** (~250 dòng C++) + giao thức text-line đơn giản (giống encoder reader)

---

## 3. Bảng vật tư bổ sung (BOM)

| Linh kiện | Số lượng | Giá thị trường VN | Ghi chú |
|-----------|----------|-------------------|---------|
| Transistor NPN 2N2222 (hoặc S8050) | 2 | ~2.000đ/cái | Drive laser |
| Điện trở 1 kΩ ¼W | 2 | ~500đ/cái | Base resistor 2N2222 |
| Điện trở 10 kΩ ¼W | 1 | ~500đ/cái | Pull-up DHT11 (nếu module chưa có) |
| Cầu chì 5A (cho đèn+còi 12V) | 1 | ~5.000đ | An toàn tải 12V |
| Cầu chì 1A (cho input 12V vào D1 R32 barrel jack) | 1 | ~3.000đ | An toàn nguồn vào board |
| Cáp USB Micro-B (cho flash D1 R32) | 1 | ~15.000đ | D1 R32 dùng cổng Micro-USB |
| **Jack DC 5.5×2.1mm đực** (cấp 12V vào barrel jack D1 R32) | 1 | ~5.000đ | Nếu cấp nguồn qua barrel jack thay vì Vin pin |
| Buck DC 12V→5V 2A (cho relay coil + laser) | 1 | ~25.000đ | Tách rail 5V để tránh inrush relay làm reset ESP32 |
| Dây bus đực-cái (DuPont) 20cm | 20 sợi | ~10.000đ | Đi dây |
| Diode 1N4007 flyback | 1 | ~500đ | Lắp ngược chiều song song coil relay nếu module chưa có |

**Tổng phụ kiện**: ~65.000đ (đã bao gồm cáp USB + buck nhỏ + jack DC).

> ⚠️ **Kiểm tra module relay trước khi đấu**: nếu module có **optocoupler** (PC817/EL817) → drive trực tiếp từ GPIO ESP32 3.3V. Nếu không opto → cũng OK vì transistor NPN trên board mở từ ~0.7V.
>
> ℹ️ **D1 R32 có thể bỏ qua buck riêng cho mình**: barrel jack 7-12V của nó dùng AMS1117 LDO onboard ra 5V → 3.3V. Cấp pin LFP 12V trực tiếp được (board chỉ tiêu ~80mA, AMS1117 không nóng nhiều). Nhưng **relay coil + laser PHẢI dùng buck 5V riêng** — không lấy từ pin 5V của D1 R32 (AMS1117 max 500mA, sẽ over-current khi relay inrush + laser ON).

---

## 4. Sơ đồ chân ESP32 D1 R32 WeMos

### 4.1 Layout vật lý (Arduino UNO R3 form factor)

```
              ESP32 D1 R32 WeMos (form factor Arduino UNO)
              ┌─────────────────────────────────────────────────────┐
              │  [BARREL JACK 7-12V]            [USB MICRO-B]       │
              │                                                      │
              │  Header phải-trên (I²C + SPI + GND/RST)              │
              │   ┌────────────────────────────────────────────────┐ │
              │   │ SCL = GPIO 22 ★ I²C SCL → OLED                 │ │
              │   │ SDA = GPIO 21 ★ I²C SDA → OLED                 │ │
              │   │ RESET (EN)                                      │ │
              │   │ GND                                             │ │
              │   │ IO18 = GPIO 18 = SCK   (FREE, SPI clock)        │ │
              │   │ IO19 = GPIO 19 = MISO  (FREE, SPI MISO)         │ │
              │   │ IO23 = GPIO 23 = MOSI  (FREE, SPI MOSI)         │ │
              │   │ IO5  = GPIO 5  = SS    ⚠ STRAPPING (boot HIGH)  │ │
              │   │ IO13 = GPIO 13         (FREE)                   │ │
              │   │ IO12 = GPIO 12         ⚠ STRAPPING (boot LOW)   │ │
              │   │ IO14 = GPIO 14         (FREE)                   │ │
              │   │ IO27 = GPIO 27 ★ LASER 1 (qua 2N2222)           │ │
              │   │ IO16 = GPIO 16 ★ LASER 2 (qua 2N2222)           │ │
              │   │ IO17 = GPIO 17         (FREE)                   │ │
              │   │ IO25 = GPIO 25 ★ DHT11 DATA                     │ │
              │   │ IO26 = GPIO 26 ★ RELAY IN (active LOW)          │ │
              │   │ TX0  = GPIO 1  ⚠ USB-Serial TX                  │ │
              │   │ RX0  = GPIO 3  ⚠ USB-Serial RX                  │ │
              │   └────────────────────────────────────────────────┘ │
              │                                                      │
              │  Header trái (Power + Analog input-only)             │
              │   ┌────────────────────────────────────────────────┐ │
              │   │ GPIO 0  ⚠ BOOT button                           │ │
              │   │ +5V                                              │ │
              │   │ RESET                                            │ │
              │   │ +3V3      ◄── 3.3V cho OLED, DHT11 VCC          │ │
              │   │ +5V                                              │ │
              │   │ GND                                              │ │
              │   │ GND                                              │ │
              │   │ Vin       ◄── nguồn 7-12V (thay barrel jack)    │ │
              │   │                                                  │ │
              │   │ GPIO 2  ⚠ on-board LED + strapping              │ │
              │   │ GPIO 4   (FREE, dùng làm spare)                 │ │
              │   │ GPIO 35  ❌ INPUT-ONLY (chỉ đọc analog)         │ │
              │   │ GPIO 34  ❌ INPUT-ONLY                          │ │
              │   │ GPIO 36  ❌ INPUT-ONLY                          │ │
              │   │ GPIO 39  ❌ INPUT-ONLY                          │ │
              │   └────────────────────────────────────────────────┘ │
              │                                                      │
              │  Phía dưới (analog, có thể chọn jumper)              │
              │   GPIO 15 ⚠ STRAPPING (boot HIGH) / GPIO 33 / GPIO 32│
              │                                                      │
              └─────────────────────────────────────────────────────┘
```

### 4.2 Chân được dùng

| Label trên board | ESP32 GPIO | Chức năng | Đi tới | Lý do chọn |
|------------------|------------|-----------|--------|------------|
| **SDA** (header) | **GPIO 21** | I²C SDA | OLED SDA | Pin I²C mặc định ESP32, OUTPUT được ✅ |
| **SCL** (header) | **GPIO 22** | I²C SCL | OLED SCL | Pin I²C mặc định ESP32, OUTPUT được ✅ |
| **IO25** | GPIO 25 | DHT11 DATA | DHT11 OUT | Free, DAC1 nhưng dùng digital OK |
| **IO26** | GPIO 26 | Relay IN | Relay module | Free, DAC2 nhưng dùng digital OK |
| **IO27** | GPIO 27 | Laser 1 | 2N2222 base | Free, không strapping |
| **IO16** | GPIO 16 | Laser 2 | 2N2222 base | Free, không strapping (UART2-RX cũng OK) |
| **+3V3** | — | Nguồn 3.3V | OLED VCC, DHT11 VCC | Từ AMS1117 onboard, max ~500mA |
| **GND** | — | Mass chung | GND tất cả | |
| **Vin** hoặc **barrel jack** | — | Nguồn vào | Pin LFP 12V qua cầu chì 1A | AMS1117 onboard hạ áp |
| **USB Micro-B** | — | UART qua chip **CH340G** | Pi USB port (→ `/dev/ttyUSB?`) | Giao tiếp với Pi |

### 4.3 ⚠ Chân CẤM dùng (lý do)

| Label | ESP32 GPIO | Lý do |
|-------|-----------|-------|
| (analog trái) | **GPIO 35** | ❌ INPUT-ONLY — chỉ đọc, không output |
| (analog trái) | **GPIO 34** | ❌ INPUT-ONLY |
| (analog trái) | **GPIO 36** | ❌ INPUT-ONLY |
| (analog trái) | **GPIO 39** | ❌ INPUT-ONLY |
| RX0 | GPIO 3 | USB-Serial RX (dùng cho debug print) |
| TX0 | GPIO 1 | USB-Serial TX |
| IO12 | GPIO 12 | STRAPPING — phải LOW khi boot, nếu lỡ HIGH thì board treo |
| IO5  | GPIO 5  | STRAPPING — phải HIGH khi boot (cũng là SS của SPI) |
| GPIO 2 (analog trái) | GPIO 2 | STRAPPING + on-board LED — tránh nếu không cần |
| (ẩn) | GPIO 6-11 | Nối flash SPI nội bộ — đụng vào treo board |
| (ẩn) | GPIO 0 | BOOT button strapping |
| (ẩn) | GPIO 15 | Strapping — phải HIGH khi boot |

### 4.4 Pin còn trống (spare) sau khi tích hợp 6 thiết bị

| Label | ESP32 GPIO | Ghi chú |
|-------|-----------|---------|
| IO13 | GPIO 13 | Spare digital |
| IO14 | GPIO 14 | Spare digital |
| IO17 | GPIO 17 | Spare (UART2-TX nếu cần) |
| IO18 = SCK | GPIO 18 | Spare (SPI clock + LED on-board, hơi nhấp nháy khi flash) |
| IO19 = MISO | GPIO 19 | Spare (SPI MISO) |
| IO23 = MOSI | GPIO 23 | Spare (SPI MOSI) |
| GPIO 4 (analog trái) | GPIO 4 | Spare digital + ADC2 |

→ Còn **7 GPIO trống** cho mở rộng sau (vd: buzzer extra, 2nd relay, button physical, SPI display, SD card).

---

## 5. Phân tích bus I²C — không xung đột

| Bus | Master | Slave | Địa chỉ |
|-----|--------|-------|---------|
| ESP32-S3 I²C (SDA=19, SCL=20) | ESP32-S3 | INA219 | 0x40 |
| ESP32 Dev I²C (SDA=21, SCL=22) | ESP32 Dev | TCA9548A → 6× VL53L0X | 0x70 + 0x29 |
| **ESP32 D1 R32 I²C** (SDA=**21**, SCL=**22**) ★ | **D1 R32 hub** | **OLED SH1106** | **0x3C** |

→ **3 bus I²C độc lập** chạy trên 3 MCU khác nhau, dùng GPIO khác nhau → không xung đột.

> ℹ️ Code Arduino: `Wire.begin();` — không cần arg vì đây là pin I²C mặc định của ESP32. Hoặc `Wire.begin(21, 22);` nếu muốn rõ ràng. Header `SDA`/`SCL` trên D1 R32 được nối SẴN vào GPIO 21/22.

---

## 6. Sơ đồ đấu dây từng thiết bị

### 6.1 INA219 — đo Voltage để tính % Pin (ESP32-S3)

**Mục đích**: chỉ đo điện áp pin (Vbus) qua I²C → tính % dung lượng còn lại. **KHÔNG** đo dòng (shunt 0.1Ω của module không chịu nổi dòng tải >3A của motor).

```
                          INA219 module (Vbus only mode)
                       ┌──────────────────┐
   Pin LFP+ ──────┬───►│ VIN+             │
                  └───►│ VIN-             │  (VIN+ short với VIN- = 0A qua shunt)
                       │                  │
   ESP32-S3 3V3 ──────►│ VCC          GND │──────► GND chung
   ESP32-S3 GPIO19 ───►│ SDA              │
   ESP32-S3 GPIO20 ───►│ SCL              │
                       └──────────────────┘

   Firmware S3 đọc Vbus (V) → tính % pin theo công thức LFP 4S:
       pct = clamp((Vbus - 10.0) / (14.4 - 10.0), 0, 1) * 100
       (LFP 4S empty ≈ 10.0V, full ≈ 14.4V)
```

| Chân INA219 | Nối với |
|-------------|---------|
| **VIN+** và **VIN−** | **Nối CHUNG** vào Pin LFP cực + (12V bus) — không qua tải |
| VCC | ESP32-S3 3V3 |
| GND | GND chung |
| SDA | ESP32-S3 GPIO 19 |
| SCL | ESP32-S3 GPIO 20 |

> ✅ **Vbus-only mode**: VIN+ và VIN- short với nhau → 0A qua shunt nội bộ → an toàn bất kể dòng motor. INA219 vẫn đọc Vbus đúng (so với GND) qua thanh ghi `Bus Voltage Register`, range 0-26V, độ phân giải 4mV.

### 6.2 Nguồn cấp D1 R32 + buck riêng cho relay/laser

D1 R32 tự cấp nguồn từ barrel jack 7-12V (hoặc Vin pin). Relay coil + laser dùng buck riêng để tránh quá tải AMS1117 onboard.

```
   Pin LFP+ (12V) ─── Cầu chì 1A ──► D1 R32 BARREL JACK (hoặc Vin)
                  │                  (AMS1117 → 5V → 3.3V onboard)
                  │
                  └── Cầu chì 3A ──► Buck DC-DC 12→5V 2A ──► Relay module VCC
                                                          ──► Laser 1 (+) qua 2N2222
                                                          ──► Laser 2 (+) qua 2N2222

   GND pin LFP ──► D1 R32 GND (chung)
              ──► Buck GND
              ──► Tất cả GND khác (chung)
```

| Tải trên D1 R32 onboard AMS1117 5V→3.3V | Dòng (mA) |
|----------------------------------------|-----------|
| ESP32 chip | 80 (idle) ÷ 240 (Wi-Fi nếu bật) |
| OLED VCC (3.3V) | 20 |
| DHT11 VCC (3.3V) | 2.5 |
| **Tổng tải 3V3 onboard** | **~100 mA** (an toàn, AMS1117 dư) |

| Tải trên buck 5V riêng | Dòng (mA) |
|------------------------|-----------|
| Relay coil (khi ON) | 80 |
| Laser 1 (khi ON) | 40 |
| Laser 2 (khi ON) | 40 |
| **Tổng đỉnh buck 5V** | **~160 mA** < buck 2A ⇒ rất dư |

### 6.3 OLED 1.3" SH1106 — màn hình trạng thái (D1 R32 I²C)

```
                       OLED 1.3" (4 chân I²C)
                   ┌─────────────────────┐
   D1 R32 +3V3 ───►│ VCC                 │
   D1 R32 GND ────►│ GND                 │
   D1 R32 SDA ────►│ SDA                 │  ◄── pin label "SDA" (= GPIO 21)
   D1 R32 SCL ────►│ SCL                 │  ◄── pin label "SCL" (= GPIO 22)
                   └─────────────────────┘
```

| Chân OLED | Nối với (label trên D1 R32) | GPIO ESP32 |
|-----------|----------------------------|------------|
| VCC | +3V3 pin | — |
| GND | GND pin | — |
| SDA | **SDA** (header riêng, gần RESET) | GPIO 21 |
| SCL | **SCL** (header riêng, gần RESET) | GPIO 22 |

> ✅ Nối thẳng OLED SDA/SCL vào 2 pin label **SDA/SCL** trên D1 R32 — chúng chính là GPIO 21/22 (pin I²C mặc định ESP32), code chỉ cần `Wire.begin();`.
>
> ℹ️ Address: 8-bit là **0x78**, sang code C++ (Adafruit_SH110X / U8g2) dùng **0x3C** (7-bit).

### 6.4 DHT11 — nhiệt độ + độ ẩm (D1 R32 IO25 = GPIO 25)

```
                       DHT11 (module 3 chân)
                   ┌─────────────────┐
   D1 R32 +3V3 ───►│ + (VCC)         │
                   │                 │
   D1 R32 IO25 ──►│ OUT  ◄── 10kΩ ──┤  ◄── (pull-up nếu module chưa có)
                   │                 │
   D1 R32 GND ────►│ - (GND)         │
                   └─────────────────┘
```

| Chân DHT11 | Nối với (label trên D1 R32) | GPIO ESP32 |
|------------|----------------------------|------------|
| `+` (VCC) | +3V3 pin | — |
| `OUT` | **IO25** | GPIO 25 |
| `-` (GND) | GND pin | — |

### 6.5 Relay 5V + Đèn 12V + Còi tích hợp (D1 R32 IO26 = GPIO 26)

#### Phần điều khiển (D1 R32 → Relay)

```
                       Module Relay 5V (1 kênh)
                   ┌──────────────────────────┐
   Buck 5V ───────►│ VCC (5V)                 │
   D1 R32 IO26 ──►│ IN  (active LOW thường)  │
   D1 R32 GND ────►│ GND                      │
                   │                          │
                   │   ┌─COIL─┐                │
                   │   │ ⇂⇂⇂  │                │
                   │   └──────┘                │
                   │           ┌──COM───►──┐   │
                   │           │            │   │  ──► Đi mạch tải 12V
                   │           │  NO ──►─┐  │   │
                   │           │         │  │   │
                   └───────────┴─────────┴──┴───┘
```

#### Phần tải (Relay → Đèn 12V + còi)

```
                                ┌───── Cầu chì 5A ─────┐
                                │                       │
   Bus 12V+ (từ pin LFP) ───────┘                       │
                                                         ▼
                                            ┌────────────────┐
                                            │  COM           │
                                  Relay ───►│                │
                                            │  NO  ─────────►├─── (đèn+còi) ─── 12V GND
                                            │                │
                                            │  NC  (không dùng)
                                            └────────────────┘
```

| Chân | Nối với (label trên D1 R32) | GPIO ESP32 |
|------|----------------------------|------------|
| Relay VCC | Buck 5V (riêng, KHÔNG lấy từ D1 R32) | — |
| Relay GND | GND chung | — |
| Relay IN | **IO26** | GPIO 26 |
| Relay COM | 12V+ qua cầu chì 5A | — |
| Relay NO | Dây dương đèn+còi | — |
| Relay NC | Không dùng | — |

> ⚠️ **An toàn**: Cầu chì 5A bắt buộc ở dây 12V+ trước COM. GND đèn 12V phải nối chung với GND D1 R32.
> 
> ℹ️ **Active level**: đa số module relay 5V opto-isolated 1 kênh là **active LOW** — IN = LOW → relay ON. Firmware có flag `RELAY_ACTIVE_LOW = true` để chỉnh nếu cần.

### 6.6 Laser 1 & Laser 2 — qua transistor NPN 2N2222

Laser ~50mA, vượt khả năng cấp dòng trực tiếp của GPIO ESP32 (max 12 mA an toàn) → dùng transistor low-side switch.

```
   Buck 5V ────────────┬────────────────┐
                       │                 │
                    ┌──┴──┐           ┌──┴──┐
                    │LASER│           │LASER│
                    │  1  │           │  2  │
                    └──┬──┘           └──┬──┘
                       │ (cathode)        │ (cathode)
                       ▼                  ▼
                    Collector          Collector
                    ┌──■──┐            ┌──■──┐
                    │2N2222            │2N2222
   D1 R32 IO27 ┬1k►│ Base │  ┌────────│ Base │ ◄1k─┬─ D1 R32 IO16
             │   │      │  │        │      │      │
             │   └──┬───┘  │        └──┬───┘      │
             │   Emitter   │         Emitter      │
             │      │      │            │         │
             │      └──────┴────────────┴─────────┤
             │              GND chung              │
             └─────────────────────────────────────┘
```

| Chân | Nối với (label trên D1 R32) | GPIO ESP32 |
|------|----------------------------|------------|
| Laser 1 (+) | Buck 5V | — |
| Laser 1 (-) | 2N2222 #1 Collector | — |
| 2N2222 #1 Base qua 1kΩ | **IO27** | GPIO 27 |
| 2N2222 #1 Emitter | GND chung | — |
| Laser 2 (+) | Buck 5V | — |
| Laser 2 (-) | 2N2222 #2 Collector | — |
| 2N2222 #2 Base qua 1kΩ | **IO16** | GPIO 16 |
| 2N2222 #2 Emitter | GND chung | — |

> ⚠️ **An toàn laser**: KHÔNG chiếu thẳng vào mắt. Lắp hướng xuống mặt đường. Mặc định OFF khi boot.

### 6.7 USB D1 R32 → Pi

```
   ESP32 D1 R32 WeMos
   ┌──────────────────┐
   │                  │
   │  [USB Micro-B]   │◄────── Cáp USB Micro-B ──────► Pi USB port
   │                  │                                  (→ /dev/ttyUSB?, qua chip CH340G)
   └──────────────────┘
```

> ℹ️ **Nhận diện trên Pi**: D1 R32 WeMos dùng chip USB-Serial **CH340G** → Pi nhận thành `/dev/ttyUSB0` (hoặc `ttyUSB1` tùy thứ tự cắm).
> - VID/PID: `1a86:7523` (Q1: kiểm tra bằng `lsusb` sau khi cắm).
> - Khuyến nghị: tạo **udev rule** giống `99-kpatrol-serial.rules` đã có để cố định tên thành `/dev/kpatrol-periph` theo serial chip CH340G.
> - Tải driver `ch341.ko` đã có sẵn trong kernel Raspberry Pi OS — không cần cài thêm.

---

## 7. Phân tích tải nguồn (power budget)

### 7.1 Phân chia 2 nguồn cho D1 R32 hub

**Nguồn 1 — D1 R32 board (qua barrel jack 12V trực tiếp)**: chỉ nuôi ESP32 + OLED + DHT11 qua AMS1117 onboard.

| Tải trên AMS1117 5V→3.3V onboard | Dòng (mA) |
|----------------------------------|-----------|
| ESP32 chip (Wi-Fi disabled) | 80 (idle) ÷ 160 (peak khi processing) |
| OLED 1.3" (qua 3V3 pin) | 20 |
| DHT11 (qua 3V3 pin) | 2.5 |
| **Tổng tải D1 R32 onboard** | **~100 mA** (an toàn cho AMS1117 500mA) |

**Nguồn 2 — Buck 12→5V riêng**: chỉ nuôi relay coil + laser (KHÔNG nối vào pin 5V của D1 R32).

| Tải trên buck 5V riêng | Dòng (mA) |
|------------------------|-----------|
| Relay coil 5V (khi ON) | 80 |
| Laser 1 (khi ON) | 40 |
| Laser 2 (khi ON) | 40 |
| **Tổng đỉnh buck 5V** | **~160 mA** < buck 2A ⇒ rất dư |

→ Buck 1-2A là đủ (chỉ dùng ~8% công suất buck 2A).

### 7.2 Pi 5V rail KHÔNG bị tải thêm

Với phương án D1 R32 hub, Pi chỉ cần cấp nguồn cho chính nó + camera. Không có thêm tải GPIO/I²C nào.

### 7.3 Bus 12V chính (từ pin LFP 10.000 mAh)

| Tải | Dòng đỉnh (A) |
|-----|---------------|
| 4 motor BTS7960 đầy tải | ~5.0 |
| Pi 4 (qua buck 12→5V cũ) | ~0.5 |
| 2 ESP32 cũ (S3 + Dev) | ~0.3 |
| D1 R32 (qua barrel jack 12V trực tiếp) | ~0.1 |
| Buck 12→5V cho relay+laser | ~0.2 (khi đỉnh) |
| Đèn + còi 12V (khi báo động) | ~1.0 |
| **Tổng đỉnh** | **~7.1 A** |

→ LFP 30C còn dư công suất.

---

## 8. Quy trình lắp đặt từng bước

### Bước 1: Chuẩn bị (offline)

1. Tắt nguồn robot, tháo cụm khoang điện
2. Kiểm tra pin LFP ≥ 11.5V
3. Pi shut down sạch (`sudo poweroff`)
4. Chuẩn bị BOM (mục 3) + đồng hồ đo + cáp USB Micro-B cho D1 R32

### Bước 2: Lắp INA219 vào ESP32-S3 (giữ nguyên kế hoạch)

1. Tháo dây nguồn motor PSU khỏi pin LFP+
2. Nối **pin LFP+ → INA219 VIN+**, **INA219 VIN- → motor PSU +**
3. Dây I²C: VCC↔3V3, GND↔GND, SDA↔S3 GPIO19, SCL↔S3 GPIO20
4. Cấp nguồn, kiểm tra LED INA219 sáng

### Bước 3: Lắp buck 12V→5V cho relay + laser (KHÔNG cho D1 R32)

1. Lấy nguồn 12V từ bus chính (sau cầu chì tổng), đưa vào IN của buck
2. Điều chỉnh trim-pot buck cho ra **đúng 5.00V** (đo bằng đồng hồ)
3. Output buck nối tới đường ray phân phối 5V cho relay coil + 2 laser (rail riêng, không nối với Pi 5V, không nối với D1 R32 5V pin)

### Bước 4: Cấp nguồn cho D1 R32 (qua barrel jack)

1. Lấy nguồn 12V từ bus chính qua **cầu chì 1A** → cắm vào **barrel jack 7-12V** của D1 R32 (jack DC 5.5×2.1mm, center positive)
2. Hoặc dùng Vin pin nếu không có jack DC: 12V+ → Vin pin của D1 R32
3. D1 R32 GND ↔ GND chung
4. **KHÔNG cắm USB cùng lúc với 12V** lần đầu — kiểm tra LED nguồn D1 R32 sáng trước
5. Đo điện áp tại pin **+5V** và **+3V3** của D1 R32: phải có ~5.0V và ~3.3V (AMS1117 onboard)

### Bước 5: Flash firmware D1 R32 (qua USB, không cắm 12V)

1. **Rút dây 12V** khỏi barrel jack/Vin trước khi cắm USB
2. Cắm cáp USB Micro-B → laptop
3. Trên laptop: kiểm tra cổng (`ls /dev/cu.usb*` trên macOS, `Device Manager` trên Windows)
4. Flash firmware D1 R32 qua Arduino IDE hoặc PlatformIO (board: **ESP32 Dev Module** hoặc **WEMOS D1 MINI ESP32**)
5. Mở Serial Monitor 115200 baud → thấy `BOOT:d1r32-hub-v1.0`

### Bước 6: Lắp OLED + DHT11 vào D1 R32

1. OLED 4 dây: VCC↔D1 R32 +3V3, GND↔D1 R32 GND, **SDA pin OLED ↔ pin label SDA trên D1 R32** (= GPIO 21), **SCL pin OLED ↔ pin label SCL trên D1 R32** (= GPIO 22)
2. DHT11 3 dây: VCC↔D1 R32 +3V3, OUT↔D1 R32 **IO25** (= GPIO 25), GND↔D1 R32 GND
3. Reset D1 R32, kiểm tra OLED hiển thị logo K-Patrol + DHT11 đọc nhiệt độ

### Bước 7: Lắp module relay + tải đèn 12V

1. Relay VCC↔buck 5V (Bước 3), GND↔GND chung, **IN↔D1 R32 IO26** (= GPIO 26)
2. Mạch tải: pin LFP+ → cầu chì 5A → COM ; NO → đèn+còi 12V (+) ; đèn+còi 12V (-) → GND chung
3. Test thủ công: short tay IN xuống GND → đèn sáng

### Bước 8: Lắp 2 laser qua 2N2222

1. Hàn 2 mạch transistor (mỗi cái: 1kΩ vào base, emitter→GND, collector→(-) laser)
2. Laser (+) → buck 5V (Bước 3)
3. Base #1 → D1 R32 **IO27** (= GPIO 27), Base #2 → D1 R32 **IO16** (= GPIO 16)
4. Test thủ công: nối tay IO27 lên 3V3 → laser 1 sáng

### Bước 9: Kết nối D1 R32 → Pi qua USB

1. Cắm cáp USB Micro-B D1 R32 → Pi (port USB còn trống)
2. Trên Pi: `ls /dev/ttyUSB*` để xem tên thiết bị mới (vd: `/dev/ttyUSB0`)
3. Test giao tiếp: `screen /dev/ttyUSB0 115200` → gõ `PING\n` → D1 R32 trả `PONG\n`
4. `lsusb` phải thấy dòng `1a86:7523 QinHeng Electronics CH340 serial converter`

### Bước 10: Cố định tên qua udev rule

Thêm vào `99-kpatrol-serial.rules`:

```
# ESP32 D1 R32 WeMos peripheral hub (CH340G chip)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
  ATTRS{serial}=="<điền_serial_chip>", SYMLINK+="kpatrol-periph", \
  MODE="0666", GROUP="dialout"
```

Lấy serial bằng: `udevadm info -a /dev/ttyUSB0 | grep serial`

Reload udev: `sudo udevadm control --reload && sudo udevadm trigger`

Sau đó device là **`/dev/kpatrol-periph`** bất kể thứ tự cắm.

> ⚠️ **Lưu ý chip CH340G**: nhiều board D1 R32 dùng cùng VID/PID `1a86:7523` (không có serial unique của ESP32 chip, mà là serial của CH340G — có khi rỗng). Nếu `ATTRS{serial}` rỗng, dùng **`ATTRS{devpath}`** (số cổng USB vật lý trên Pi) để nhận dạng:
> ```
> SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", \
>   KERNELS=="1-1.3", SYMLINK+="kpatrol-periph", \
>   MODE="0666", GROUP="dialout"
> ```
> Tìm KERNELS bằng `udevadm info -a /dev/ttyUSB0 | grep -m1 KERNELS`.

---

## 9. Giao thức UART D1 R32 ↔ Pi

Baud rate: **115200**. Định dạng text-line, mỗi dòng kết thúc `\n`. Giống encoder reader để dễ debug.

### 9.1 D1 R32 → Pi (telemetry, định kỳ)

| Format | Tần suất | Ý nghĩa |
|--------|----------|---------|
| `DHT:24.5,68.2\n` | 1 Hz | nhiệt độ °C, độ ẩm % |
| `STATE:RELAY=0,L1=1,L2=0\n` | 2 Hz | trạng thái relay + laser1 + laser2 (0/1) |
| `HB:1234567\n` | 1 Hz | heartbeat = millis() từ boot |
| `OK:<cmd>\n` | per-cmd | ACK lệnh đã nhận |
| `ERR:<reason>\n` | per-cmd | báo lỗi parsing/IO |
| `BOOT:d1r32-hub-v1.0\n` | 1 lần khi reset | version banner |

### 9.2 Pi → D1 R32 (commands)

| Lệnh | Tác dụng | Phản hồi |
|------|----------|----------|
| `PING\n` | Kiểm tra sống | `PONG\n` |
| `RELAY ON\n` / `RELAY OFF\n` / `RELAY TOGGLE\n` | Đèn+còi 12V | `OK:RELAY\n` |
| `LASER1 ON\n` / `LASER1 OFF\n` | Laser 1 | `OK:LASER1\n` |
| `LASER2 ON\n` / `LASER2 OFF\n` | Laser 2 | `OK:LASER2\n` |
| `LASERS ON\n` / `LASERS OFF\n` | Cả 2 laser | `OK:LASERS\n` |
| `DISP TEXT "<line1>\\n<line2>"\n` | Đẩy 2 dòng text lên OLED | `OK:DISP\n` |
| `DISP CLEAR\n` | Xóa OLED | `OK:DISP\n` |
| `STATUS\n` | Yêu cầu emit STATE ngay | `STATE:...\n` |
| `RESET\n` | Reset D1 R32 | (khởi động lại) |

### 9.3 Safety: auto-OFF khi mất kết nối

D1 R32 cài timer **5 giây**: nếu không có lệnh nào từ Pi trong 5s → **tắt cả 2 laser + relay** + hiện "DISCONNECT" lên OLED. Tránh laser hoặc đèn báo động bị treo ON khi Pi crash.

---

## 10. Firmware D1 R32 — sườn (sẽ viết chi tiết sau)

File mới: `robots/firmware/peripheral_hub_d1r32/peripheral_hub_d1r32.ino`

```cpp
// ESP32 D1 R32 WeMos Peripheral Hub for K-Patrol
// Tasks: OLED render, DHT11 read, relay drive, 2× laser drive
// UART text protocol with Pi @ 115200 baud
//
// Pin map (D1 R32 label → ESP32 GPIO):
//   SDA header  = GPIO 21 → OLED SDA       (Wire default)
//   SCL header  = GPIO 22 → OLED SCL       (Wire default)
//   IO25        = GPIO 25 → DHT11 DATA
//   IO26        = GPIO 26 → RELAY IN       (active LOW)
//   IO27        = GPIO 27 → LASER 1 base   (active HIGH via 2N2222)
//   IO16        = GPIO 16 → LASER 2 base   (active HIGH via 2N2222)

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>      // SH1106 1.3"
#include <DHT.h>

#define PIN_DHT      25
#define PIN_RELAY    26
#define PIN_LASER1   27
#define PIN_LASER2   16
#define RELAY_ACTIVE_LOW true

Adafruit_SH1106G oled(128, 64, &Wire, -1);
DHT dht(PIN_DHT, DHT11);

uint32_t last_dht_ms = 0;
uint32_t last_hb_ms  = 0;
uint32_t last_cmd_ms = 0;
bool laser1_on = false, laser2_on = false, relay_on = false;
float temp_c = NAN, humidity = NAN;

void setLaser(int idx, bool on) { /* digitalWrite + state */ }
void setRelay(bool on)          { /* digitalWrite with active-low support */ }
void renderOled()               { /* draw status box */ }
void readDht()                  { /* read with retry */ }
void handleCommand(const String& line) { /* parse PING / RELAY ON / ... */ }
void publishState()             { /* Serial.printf("STATE:...") */ }
void safetyCheck()              { /* if (now - last_cmd_ms > 5000) all-off */ }

void setup() {
  Serial.begin(115200);
  Wire.begin();              // SDA=GPIO21, SCL=GPIO22 (ESP32 default I²C)
  oled.begin(0x3C, true);
  dht.begin();
  pinMode(PIN_RELAY,  OUTPUT); setRelay(false);
  pinMode(PIN_LASER1, OUTPUT); setLaser(1, false);
  pinMode(PIN_LASER2, OUTPUT); setLaser(2, false);
  Serial.println("BOOT:d1r32-hub-v1.0");
}

void loop() {
  // 1. Receive UART command (line-buffered)
  // 2. Read DHT11 every 2s (it's slow)
  // 3. Render OLED every 500ms
  // 4. Emit HB every 1s
  // 5. Emit STATE every 500ms
  // 6. Safety check (auto-off after 5s no command)
}
```

→ Đầy đủ ~250 dòng. Sẽ viết chi tiết khi sang giai đoạn code.

---

## 11. Tổng kết I/O sau khi tích hợp

### ESP32-S3 (motor)
- Không thay đổi GPIO
- Chỉ thêm vật lý INA219 vào I²C 19/20

### ESP32 Dev (encoder)
- Không thay đổi gì cả

### ESP32 D1 R32 (peripheral hub) ★ NEW

| Label trên board | GPIO ESP32 | Chức năng | Tình trạng |
|------------------|------------|-----------|------------|
| SDA              | 21         | I²C → OLED SDA | ✅ Dùng |
| SCL              | 22         | I²C → OLED SCL | ✅ Dùng |
| IO25             | 25         | DHT11 1-wire   | ✅ Dùng |
| IO26             | 26         | Relay IN (active LOW) | ✅ Dùng |
| IO27             | 27         | Laser 1 base (qua 2N2222) | ✅ Dùng |
| IO16             | 16         | Laser 2 base (qua 2N2222) | ✅ Dùng |
| IO4, IO13, IO14, IO17, IO18, IO19, IO23 | 4/13/14/17/18/19/23 | Trống — header digital | 🟢 Spare |
| GPIO 32/33/34/35/36/39 (analog side) | 32-39 | Trống — analog input | 🟢 Spare |

### Raspberry Pi

| Cổng | Trạng thái |
|------|-----------|
| `/dev/ttyACM0` (hoặc symlink `kpatrol-motor`) | ESP32-S3 motor — không đổi |
| `/dev/ttyACM1` (hoặc `kpatrol-encoder`) | ESP32 Dev encoder — không đổi |
| `/dev/ttyUSB0` (hoặc `kpatrol-periph`) ★ | ESP32 D1 R32 hub — MỚI (CH340G) |
| GPIO + I²C-1 | Không dùng — giữ trống |

---

## 12. MQTT topics dự kiến

Tham khảo cho giai đoạn viết code:

| Topic | Hướng | Payload | Mô tả |
|-------|-------|---------|-------|
| `kpatrol/{serial}/periph/sensors` | Pi → broker | `{"temp_c":24.5,"humidity":68,"bat_mA":1250,"bat_V":12.4}` | DHT11 + INA219 (5s/lần) |
| `kpatrol/{serial}/periph/relay` | broker → Pi | `{"action":"on"\|"off"\|"toggle"}` | Bật/tắt đèn+còi 12V |
| `kpatrol/{serial}/periph/laser` | broker → Pi | `{"laser":1\|2\|"both","action":"on"\|"off"}` | Bật/tắt laser |
| `kpatrol/{serial}/periph/oled` | broker → Pi | `{"text":"<line1>\n<line2>"}` | Override text OLED (debug) |

Pi `kpatrol_mqtt_v5.py` sẽ:
- Subscribe topic relay/laser/oled → forward command qua UART tới D1 R32
- Publish topic sensors định kỳ từ data DHT11 (qua UART) + INA219 (qua S3 UART)

---

## 13. Checklist trước khi cấp nguồn

- [ ] Pin LFP đo được ≥ 11.5V
- [ ] Tất cả GND nối chung (Pi GND ↔ S3 GND ↔ Dev GND ↔ D1 R32 GND ↔ 12V GND)
- [ ] Cầu chì 1A đã lắp trên dây 12V+ vào barrel jack D1 R32
- [ ] Cầu chì 5A đã lắp trên dây 12V+ cho đèn+còi
- [ ] Buck 12→5V cho relay+laser đo output đúng 5.00V (±0.1V)
- [ ] D1 R32 đã flash firmware (test `PING` qua Serial Monitor laptop, baud 115200, board "ESP32 Dev Module")
- [ ] D1 R32 boot banner thấy `BOOT:d1r32-hub-v1.0`
- [ ] Relay đã thử bằng tay (short IN→GND) — đèn sáng
- [ ] Laser hướng xuống đất, KHÔNG hướng người/thú vật
- [ ] OLED hiển thị logo K-Patrol khi D1 R32 boot
- [ ] DHT11 đọc được nhiệt độ hợp lý (20-30°C trong phòng)
- [ ] D1 R32 USB → Pi: `lsusb` thấy `1a86:7523 QinHeng Electronics CH340 serial converter`
- [ ] `ls /dev/ttyUSB* /dev/ttyACM*` thấy device mới (thường `/dev/ttyUSB0`)
- [ ] udev rule đã tạo, `ls -l /dev/kpatrol-periph` thấy symlink

---

## 14. Câu hỏi cần xác nhận trước khi mua/lắp

1. **Module Relay**: có **opto-isolated** (PC817/EL817) không? Active LOW hay HIGH?
2. **Module Laser**: là module PCB 3 chân (S/+/-) hay diode laser trần?
3. **Đèn + còi 12V**: dòng tải khi BẬT là bao nhiêu A? (cần ≤ 5A để cầu chì OK)
4. **OLED 1.3"**: chip điều khiển là **SH1106** hay **SSD1306**? (xem nhãn module)
5. **DHT11**: module 3 chân (có pull-up) hay sensor trần 4 chân?
6. **Buck 12→5V**: đã có sẵn buck nào trong khoang điện chưa? Hay cần mua thêm?

> ✅ **Đã xác nhận**: Board peripheral hub là **ESP32 D1 R32 WeMos** với chip UART **CH340G** (VID/PID `1a86:7523`). Device sẽ nhận dạng dưới dạng `/dev/ttyUSB?` trên Pi (driver `ch341.ko` có sẵn trong kernel Raspberry Pi OS).

Trả lời các câu trên rồi đi vào bước 1 mục 8.

---

## 15. Sau khi đi dây xong → bắt tay code

Trình tự:

1. **Firmware D1 R32** — `robots/firmware/peripheral_hub_d1r32/peripheral_hub_d1r32.ino` (~250 dòng)
   - Arduino IDE: chọn board **"ESP32 Dev Module"** hoặc **"WEMOS D1 MINI ESP32"** (đều OK), baud upload 921600
   - Cài thư viện: `Adafruit_SH110X`, `Adafruit_GFX`, `DHT sensor library` (Adafruit)
   - Test qua Serial Monitor 115200 baud, ngắt 12V trước khi cắm USB
2. **Python driver** — `robots/pi-controller/peripheral_hub.py`
   - Wrapper UART tương tự `sensor_reader.py`
   - Parse `DHT:...`, `STATE:...`, `HB:...`, `BOOT:d1r32-hub-v1.0`
   - Expose API: `set_relay(on)`, `set_laser(idx, on)`, `display(text)`, `get_dht()`
   - Connect mặc định `/dev/kpatrol-periph` (symlink từ udev)
3. **Tích hợp** vào `kpatrol_mqtt_v5.py`:
   - Khởi tạo `PeripheralHub(port='/dev/kpatrol-periph')` trong `__init__`
   - MQTT subscribe topic mới + forward command
   - Telemetry publish định kỳ (5s/lần)
4. **MQTT topics** vào `Topics` class
5. **Update `requirements.txt`** — không cần thêm gì vì D1 R32 xử lý tất cả I²C/GPIO; Pi chỉ cần `pyserial` đã có
6. **Update PWA** (sau cùng) — UI nút bật/tắt laser + relay + hiển thị nhiệt độ

→ Sau khi user xác nhận xong wiring + 6 câu hỏi mục 14, mình bắt đầu Bước 1 (viết firmware D1 R32).
