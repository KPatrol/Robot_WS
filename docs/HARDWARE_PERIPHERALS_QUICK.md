# K-Patrol Peripheral — Quick Wiring Card (v2.0)

> **4 thiết bị active** · **3 trên D1 R32** (CH340G `1a86:7523`) · **1 trên ESP32-S3** (INA219) · UART 115200

Firmware: `peripheral_hub_d1r32.ino` v2.0 (laser đã loại bỏ — chỉ giữ OLED, DHT11, Relay).

## Sơ đồ tổng thể

```
  Pin LFP 12V ─┬─► INA219 VIN+ ──short── VIN-   (Vbus only, không qua shunt)
               │      VCC=S3·3V3, GND, SDA=S3·GPIO19, SCL=S3·GPIO20
               │      └─► ESP32-S3 (I²C 0x40) đọc Vbus → % pin ──► Pi /dev/ttyACM0
               │
               ├──[1A]──► D1 R32 barrel jack (AMS1117 → 5V/3V3 onboard)
               │              │
               │              ├─ +3V3 ──► OLED VCC, DHT11 VCC
               │              ├─ SDA(GPIO21) ─► OLED SDA   (I²C 0x3C)
               │              ├─ SCL(GPIO22) ─► OLED SCL
               │              ├─ IO25       ─► DHT11 OUT
               │              ├─ IO26       ─► Relay IN   (active HIGH — xem note)
               │              └─ USB Micro-B ─────────────► Pi /dev/ttyUSB0
               │
               ├──► Buck 12→5V ─► Relay VCC
               │
               └──[5A]──► Relay COM/NO ──► Đèn 12V + còi

  GND pin LFP ──► nối chung TẤT CẢ GND (S3, D1 R32, Buck, Relay, Pi)
                  BẮT BUỘC chung — nếu không relay sẽ không kích được
```

## Bảng đấu dây

| Thiết bị | Pin thiết bị | Pin MCU (label / GPIO) | Nguồn / Ghi chú |
|----------|--------------|------------------------|-----------------|
| **INA219** (S3) | VCC / GND / SDA / SCL / VIN+ = VIN− | S3 3V3 / GND / **GPIO19** / **GPIO20** / **chung Pin+** | I²C 0x40, **Vbus only** → đo V, tính % pin |
| OLED 1.3" SH1106 | VCC / GND / SDA / SCL | D1 R32 +3V3 / GND / **SDA**=21 / **SCL**=22 | I²C 0x3C, +3V3 onboard |
| DHT11           | VCC / OUT / GND       | D1 R32 +3V3 / **IO25** / GND | 1-wire, +3V3 onboard |
| Relay 5V        | VCC / IN / GND        | (buck 5V) / **IO26** / GND   | Active-HIGH (no opto) — xem note |
| Đèn 12V + còi   | + / −                 | Relay NO / GND                | 12V qua 5A fuse |

GPIO27 + GPIO16 trên D1 R32 hiện đang **trống** (đã loại bỏ laser ở v2.0). Có thể dùng cho thiết bị khác về sau.

## Quy trình (8 bước)

1. **Ngắt pin 12V**, đảm bảo an toàn
2. Đấu theo bảng trên, GND nối **chung 1 điểm**
3. Cắm USB Micro-B (laptop) → Arduino IDE: board *"ESP32 Dev Module"*, baud 115200
4. Flash `peripheral_hub_d1r32.ino` → Serial Monitor thấy `BOOT:periph-hub-v2.0`, OLED hiện mặt cười
5. Test tay: `RELAY:ON` → đèn sáng; `STATUS` → in META + STATE + DHT
6. Rút USB laptop, cấp 12V qua **barrel jack** (qua cầu chì 1A)
7. Cắm USB → Pi: `lsusb` thấy `1a86:7523`, `/dev/ttyUSB0` xuất hiện
8. Udev: tạo symlink `/dev/kpatrol-periph` (xem `HARDWARE_PERIPHERALS.md` Bước 10)

## Giao thức UART (v2.0)

**D1 R32 → Pi** (telemetry):
- `BOOT:periph-hub-v2.0,...` — boot banner
- `HB:t=<ms>` mỗi 1 s
- `DHT:<temp>,<humi>` mỗi 5 s (hoặc `DHT:nan,nan` / `DHT:disabled` khi lỗi)
- `STATE:relay=<0|1>` khi đổi + mỗi 5 s
- `HEAP:<bytes>` mỗi 30 s — phát hiện leak
- `META:fw=v2.0,uptime=...,watchdog_armed=...,relay_pol=...,time=HH:MM:SS|?` (response cho STATUS)
- `PONG:t=<ms>` — reply PING
- `WATCHDOG:fired,reason=...` — khi safety auto-OFF
- `TIME:set,HH:MM:SS` — ack cho TIME sync
- `ERR:<reason>` — parser/validate errors

**Pi → D1 R32** (commands):
- `PING` · `STATUS` · `KEEPALIVE`
- `RELAY:ON|OFF|T` · `RELAY:TEST` · `RELAY:POL:LOW|HIGH`
- `TIME:HH:MM` hoặc `TIME:HH:MM:SS` — Pi sync giờ mỗi phút
- `OLED:<text>` — text hiển thị dưới mặt cười (tối đa 20 ký tự)

> Safety: 5 s không nhận lệnh từ Pi → relay tự OFF. Task watchdog 8 s reset board nếu loop() hang.

## ⚠ Note — Module Relay 5V là ACTIVE-HIGH (không optocoupler)

Module relay hiện tại là **active-HIGH** (drive 3V3 vào IN để kích coil). Bench-confirmed 2026-05-21 sau 2 fix:

1. **Đảm bảo GND chung**: nối GND của module relay với GND của D1 R32 (BẮT BUỘC, vì 5V của relay thường lấy từ nguồn riêng). Không chung GND → GPIO không có điện thế tham chiếu → drive không có tác dụng.
2. **Firmware default**: `relay_active_low = false` — gõ `RELAY:ON` drive GPIO26 HIGH → relay kích. Watchdog fire/boot idle → GPIO LOW → relay tắt.

Nếu sau này đổi sang module relay có optocoupler PC817/EL817 (low-trigger): set `relay_active_low = true` hoặc gõ `RELAY:POL:LOW` runtime.

## OLED layout (v2.0)

```
┌────────────────────────────────────┐
│   ◉ ◉            T: 30.3°C         │ ← animated smiley
│   \___/          H:  95 %          │   + sensor + clock
│                  14:30:45          │
│ ─────────────────────────────────  │
│ K-Patrol ready                  ●  │ ← OLED:<text> from Pi + heart pulse
└────────────────────────────────────┘
```

Mặt cười animation: nhìn thẳng → phải → trái → lên → chớp mắt (chu kỳ ~12 s). Khi Pi chưa gửi `TIME:` thì dòng thứ 3 hiện uptime thay vì giờ thật.

---

📖 *Chi tiết đầy đủ (BOM, schematic, power budget)*: [`HARDWARE_PERIPHERALS.md`](./HARDWARE_PERIPHERALS.md)
