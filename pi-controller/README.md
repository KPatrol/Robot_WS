# K-Patrol Pi Controller

Bộ công cụ điều khiển robot K-Patrol từ Raspberry Pi qua USB Serial.

## 📦 Yêu cầu

```bash
# Cài đặt thư viện Python
pip3 install pyserial

# (Tùy chọn) Cho gamepad controller
pip3 install inputs
```

## 🔌 Kết nối

- **USB Cable**: Từ Raspberry Pi USB đến ESP32-S3 USB-C
- **Port mặc định**: `/dev/ttyACM0`

Kiểm tra cổng:
```bash
ls /dev/tty* | grep -E '(ACM|USB)'
```

## 🛠️ Các công cụ

### 1. `kpatrol_controller.py` - Full Controller
Controller đầy đủ với giao diện đồ họa terminal và điều khiển bàn phím realtime.

```bash
python3 kpatrol_controller.py
```

**Điều khiển:**
- `W/↑` : Tiến
- `S/↓` : Lùi  
- `A/←` : Sang trái (Strafe)
- `D/→` : Sang phải (Strafe)
- `Q` : Xoay trái
- `E` : Xoay phải
- `7/9/1/3` : Di chuyển chéo
- `SPACE` : Dừng (Brake)
- `X` : Tắt khẩn cấp
- `+/-` : Tăng/giảm tốc độ
- `ESC` : Thoát

---

### 2. `kpatrol_cli.py` - Command Line Interface
Giao diện dòng lệnh tương tác, dễ sử dụng.

```bash
python3 kpatrol_cli.py
```

**Các lệnh:**
```
f [speed]    - Tiến
b [speed]    - Lùi
l [speed]    - Sang trái
r [speed]    - Sang phải
rl [speed]   - Xoay trái
rr [speed]   - Xoay phải
s            - Dừng
off          - Tắt driver
t1-t4        - Test motor 1-4
speed 200    - Đặt tốc độ
help         - Trợ giúp
quit         - Thoát
```

---

### 3. `kpatrol_keyboard.py` - Keyboard Control Mode
Điều khiển bàn phím đơn giản với giao diện trực quan.

```bash
python3 kpatrol_keyboard.py [port]

# Ví dụ
python3 kpatrol_keyboard.py /dev/ttyACM0
```

---

### 4. `kpatrol_gamepad.py` - Gamepad/Joystick Controller
Điều khiển bằng tay cầm game (Xbox, PS4, etc.)

```bash
# Cài đặt thư viện
pip3 install inputs

# Chạy
python3 kpatrol_gamepad.py [port]
```

**Điều khiển:**
- **Left Stick**: Di chuyển (tiến/lùi/ngang)
- **Right Stick**: Xoay
- **D-Pad**: Di chuyển theo hướng
- **LT**: Giảm tốc
- **RT**: Tăng tốc
- **A**: Dừng
- **B**: Emergency OFF
- **Start**: Thoát

---

### 5. `kpatrol_diagnostics.py` - System Diagnostics
Công cụ chẩn đoán và test hệ thống robot.

```bash
python3 kpatrol_diagnostics.py [port]
```

**Các test:**
1. Kiểm tra cổng Serial
2. Kiểm tra kết nối ESP32
3. Kiểm tra giao tiếp
4. Test từng motor (FL, FR, BR, BL)
5. Test di chuyển Mecanum
6. Test dừng khẩn cấp

---

## 📋 Quy trình khuyến nghị

```
1. Chạy Diagnostics để kiểm tra hệ thống
   python3 kpatrol_diagnostics.py

2. Sử dụng CLI để test các lệnh
   python3 kpatrol_cli.py

3. Chuyển sang Keyboard Control khi đã sẵn sàng
   python3 kpatrol_keyboard.py

4. (Tùy chọn) Dùng Gamepad cho điều khiển mượt mà
   python3 kpatrol_gamepad.py
```

## ⚠️ Lưu ý an toàn

1. **Luôn nhấc robot lên khi test motor**
2. **Nguồn 12V phải đủ dòng** (tối thiểu 7A, khuyến nghị 10A+)
3. **Kiểm tra GND chung** giữa ESP32, BTS7960 và nguồn 12V
4. **Sử dụng lệnh OFF** trong trường hợp khẩn cấp
5. **Không để robot chạy không giám sát**

## 🔧 Xử lý sự cố

### Không tìm thấy cổng Serial
```bash
# Liệt kê cổng
ls /dev/tty*

# Kiểm tra quyền
sudo chmod 666 /dev/ttyACM0

# Thêm user vào group dialout
sudo usermod -a -G dialout $USER
# Logout và login lại
```

### Motor không quay
1. Kiểm tra nguồn 12V
2. Kiểm tra GND chung
3. Kiểm tra đèn LED trên driver BTS7960
4. Kiểm tra dây tín hiệu RPWM, LPWM, R_EN, L_EN

### Motor quay ngược
- Đảo dây M+ và M- tại driver BTS7960
- Hoặc sửa cờ `*_INVERTED = true` trong firmware

## 📡 Giao thức lệnh ESP32

```
# Di chuyển
F <speed>    - Forward
B <speed>    - Backward
SL <speed>   - Strafe Left
SR <speed>   - Strafe Right
L <speed>    - Rotate Left
R <speed>    - Rotate Right
DFL <speed>  - Diagonal Forward-Left
DFR <speed>  - Diagonal Forward-Right
DBL <speed>  - Diagonal Backward-Left
DBR <speed>  - Diagonal Backward-Right

# Dừng
S            - Stop (Brake)
OFF          - Emergency OFF (disable drivers)

# Test motor
T1 <speed>   - Test Motor 1 (FL)
T2 <speed>   - Test Motor 2 (FR)
T3 <speed>   - Test Motor 3 (BR)
T4 <speed>   - Test Motor 4 (BL)
TA <speed>   - Test All motors

# Speed: 0-255 (mặc định 150)
```

## 📁 Cấu trúc thư mục

```
pi-controller/
├── README.md                 # File này
├── kpatrol_controller.py     # Full controller với UI
├── kpatrol_cli.py            # Command line interface
├── kpatrol_keyboard.py       # Keyboard control mode
├── kpatrol_gamepad.py        # Gamepad/joystick controller
└── kpatrol_diagnostics.py    # System diagnostics
```
