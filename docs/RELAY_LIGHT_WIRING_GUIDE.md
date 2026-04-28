# Hướng dẫn đấu nối Relay 12VDC-SL-C cho điều khiển đèn

## Thông tin Relay

| Thông số | Giá trị |
|----------|---------|
| Model | 12VDC-SL-C |
| Điện áp cuộn | 12V DC |
| Dòng tiếp điểm | 10A (250VAC) / 15A (120VAC) / 10A (28VDC) |

## Sơ đồ chân Relay

```
          ┌─────────────────────────────────────┐
          │         12VDC-SL-C RELAY            │
          │                                     │
          │  ┌─────┐               ┌─────────┐  │
          │  │COIL │               │CONTACTS │  │
          │  └──┬──┘               └────┬────┘  │
          │     │                       │       │
          │  ┌──┴──┬──────┐      ┌──────┼──────┐│
          │  │ IN  │ DC+  │ DC-  │  NO  │ COM  ││ NC │
          │  └──┬──┴──┬───┴──┬───┴──┬───┴──┬───┴┴──┬─┘
          └─────┼─────┼──────┼──────┼──────┼───────┼──
                │     │      │      │      │       │
```

## Sơ đồ đấu nối chi tiết

### Phần 1: Điều khiển Relay (Coil Side)

```
    ESP32-S3                     RELAY 12VDC-SL-C
    ┌────────┐                   ┌───────────────┐
    │        │                   │               │
    │  GPIO38├───────────────────►│ IN            │
    │        │                   │               │
    │    GND │◄──────────┐       │ DC-           │◄──┐
    │        │           │       │               │   │
    └────────┘           │       │ DC+           │◄──┼── 12V+
                         │       │               │   │
                         │       └───────────────┘   │
                         │                           │
                         └───────────────────────────┤
                                                     │
                                                   ──┴── GND CHUNG
```

### Phần 2: Tiếp điểm đèn (Contact Side)

```
    Nguồn đèn (12V/24V/220V)                    ĐÈN
         │                                        │
         │         RELAY                          │
         │    ┌────────────┐                      │
         │    │            │                      │
         └────►│ COM        │                      │
              │            │                      │
              │ NO  ───────┼──────────────────────┘
              │            │
              │ NC  ───────┼── (không dùng)
              │            │
              └────────────┘
```

## Bảng tóm tắt đấu nối

| Chân Relay | Kết nối với | Mô tả |
|------------|-------------|-------|
| **IN** | GPIO 38 (ESP32-S3) | Tín hiệu điều khiển |
| **DC+** | 12V+ | Nguồn dương 12V cho relay |
| **DC-** | GND chung | Nối chung với GND ESP32 |
| **COM** | Dây nguồn đèn | Từ nguồn (công tắc cũ) |
| **NO** | Dây đến đèn | Đi ra đèn (công tắc cũ) |
| **NC** | Không dùng | Để trống |

## Cách thay thế công tắc cơ học

### Bước 1: Xác định dây công tắc cũ
```
Trước khi thay:
    Nguồn đèn ──► [CÔNG TẮC] ──► Đèn
```

### Bước 2: Đấu vào Relay
```
Sau khi thay:
    Nguồn đèn ──► [COM] ──► [NO] ──► Đèn
```

### Lưu ý an toàn:
1. **NGẮT NGUỒN** trước khi đấu nối
2. Kiểm tra điện áp đèn (12V/24V/220V)
3. Đảm bảo relay chịu được dòng tải của đèn
4. Với đèn 220VAC, cần cách điện tốt

## Serial Commands điều khiển

| Lệnh | Mô tả |
|------|-------|
| `LIGHT_ON` | Bật đèn |
| `LIGHT_OFF` | Tắt đèn |
| `LIGHT_T` | Toggle (đổi trạng thái) |
| `LIGHT` | Xem trạng thái đèn |

## GPIO còn trống sau khi thêm Relay

| GPIO | Trạng thái | Gợi ý sử dụng |
|------|-----------|---------------|
| 38 | **RELAY ĐÈN** | ✓ Đã dùng |
| 39 | Trống | Relay phụ #2 |
| 40 | Trống | Sensor / Input |
| 41 | Trống | Sensor / Input |
| 42 | Trống | I2C SDA |
| 43 | Trống | UART TX |
| 44 | Trống | UART RX |
| 45 | Trống | I2C SCL |
| 46 | Trống | Spare |
| 47 | Trống | Spare |
| 48 | Trống | LED Status |

## Test thử

1. Upload code vào ESP32-S3
2. Mở Serial Monitor (115200 baud)
3. Gõ lệnh:
   ```
   LIGHT_ON   -> Đèn sáng
   LIGHT_OFF  -> Đèn tắt
   LIGHT_T    -> Toggle
   T          -> Chạy test sequence (bao gồm test đèn)
   ```
