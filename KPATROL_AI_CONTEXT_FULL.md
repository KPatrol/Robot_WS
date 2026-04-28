# 🤖 K-PATROL ROBOT - COMPLETE AI CONTEXT PROMPT V1.0

**Document Type**: AI Context Prompt  
**Version**: 1.0  
**Last Updated**: February 17, 2026  
**Status**: ✅ Hardware Verified & Working  

---

## 📋 ROLE & INSTRUCTIONS

Bạn là trợ lý AI chuyên về điều khiển robot **K-Patrol** - một robot tuần tra sử dụng 4 bánh Mecanum cho phép di chuyển đa hướng (omnidirectional). Bạn đang làm việc trên **Raspberry Pi 4** - máy tính điều khiển cấp cao của robot.

**Workspace chính**: `~/robots/`

---

## 1. TỔNG QUAN HỆ THỐNG

### 1.1 Kiến trúc 2-Tier

```
┌─────────────────────────────────────────────────────────────────┐
│                      RASPBERRY PI 4                             │
│                    (Master Controller)                          │
│                                                                 │
│   /dev/ttyACM0              /dev/ttyUSB0                       │
│   (ESP32-S3)                (ESP32 Dev)                        │
│        │                         │                              │
└────────┼─────────────────────────┼──────────────────────────────┘
         │ USB Serial              │ USB Serial
         │ 115200 baud             │ 115200 baud
         ▼                         ▼
┌─────────────────┐       ┌─────────────────┐
│   ESP32-S3      │       │   ESP32 Dev     │
│   WROOM-1       │       │   Module        │
│                 │       │                 │
│ Motor Controller│       │ Encoder Reader  │
│ (PWM → BTS7960) │       │ (Interrupt ISR) │
└────────┬────────┘       └────────┬────────┘
         │                         │
         ▼                         ▼
┌─────────────────────────────────────────────────────────────────┐
│                    4x BTS7960 Motor Drivers                     │
│                           ↓                                     │
│                    4x JGA25-370 Motors                          │
│                    (12V DC + Encoder)                           │
│                           ↓                                     │
│                    4x Mecanum Wheels                            │
└─────────────────────────────────────────────────────────────────┘
```

### 1.2 Danh sách phần cứng

| Thiết bị | Model | Số lượng | Chức năng | Status |
|----------|-------|----------|-----------|--------|
| SBC | Raspberry Pi 4 | 1 | Master controller | ✅ |
| MCU | ESP32-S3 WROOM-1 | 1 | Motor controller | ✅ |
| MCU | ESP32 Dev Module | 1 | Encoder reader | ✅ |
| Driver | BTS7960 (IBT-2) | 4 | H-Bridge 43A | ✅ |
| Motor | JGA25-370 | 4 | DC 12V + Encoder | ✅ |
| Wheel | Mecanum | 4 | 2 Left + 2 Right | ✅ |
| Power | 12V 10A+ | 1 | Motor power | ✅ |

---

## 2. GPIO ALLOCATION

### 2.1 ESP32-S3 (Motor Controller)

| Motor | Position | RPWM | LPWM | R_EN | L_EN | PWM Ch | Inverted |
|-------|----------|------|------|------|------|--------|----------|
| M1 | Front-Right (FR) | GPIO 4 | GPIO 5 | GPIO 6 | GPIO 7 | 0,1 | No |
| M2 | Front-Left (FL) | GPIO 15 | GPIO 16 | GPIO 17 | GPIO 18 | 2,3 | **Yes** |
| M3 | Back-Right (BR) | GPIO 8 | GPIO 9 | GPIO 10 | GPIO 11 | 4,5 | No |
| M4 | Back-Left (BL) | GPIO 12 | GPIO 13 | GPIO 14 | GPIO 21 | 6,7 | **Yes** |

**PWM Config**: 5000 Hz, 8-bit resolution (0-255)

### 2.2 ESP32 Dev (Encoder Reader)

| Motor | Encoder A (Yellow) | Encoder B (Green) |
|-------|-------------------|-------------------|
| FR | GPIO 34 | GPIO 35 |
| FL | GPIO 32 | GPIO 33 |
| BR | GPIO 25 | GPIO 26 |
| BL | GPIO 27 | GPIO 14 |

**Encoder Specs (JGA25-370)**:
- PPR encoder disk: 11 pulses/rev
- Gear ratio: 1:34
- Effective PPR: 11 × 34 = 374 PPR
- With quadrature (×4): 1496 counts/rev

---

## 3. SERIAL PROTOCOL

### 3.1 Motor Controller (ESP32-S3) - `/dev/ttyACM0`

#### Movement Commands
| Command | Description | Motor Direction (FL, FR, BR, BL) |
|---------|-------------|----------------------------------|
| `F` | Forward | +, +, +, + |
| `B` | Backward | -, -, -, - |
| `SR` | Strafe Right | +, -, +, - |
| `SL` | Strafe Left | -, +, -, + |
| `R` | Rotate Right (CW) | +, -, -, + |
| `L` | Rotate Left (CCW) | -, +, +, - |
| `DR` | Diagonal Forward-Right | +, 0, +, 0 |
| `DL` | Diagonal Forward-Left | 0, +, 0, + |
| `S` | Stop (Brake) | 0, 0, 0, 0 |

#### Individual Motor Commands
| Command | Description |
|---------|-------------|
| `FR_F`, `FR_B`, `FR_S` | Front-Right Forward/Backward/Stop |
| `FL_F`, `FL_B`, `FL_S` | Front-Left Forward/Backward/Stop |
| `BR_F`, `BR_B`, `BR_S` | Back-Right Forward/Backward/Stop |
| `BL_F`, `BL_B`, `BL_S` | Back-Left Forward/Backward/Stop |

#### Control Commands
| Command | Description |
|---------|-------------|
| `SPD:xxx` | Set speed (0-255), e.g., `SPD:150` |
| `T` | Run automated test sequence |
| `I` | Show system information |
| `H` | Show help menu |

### 3.2 Encoder Reader (ESP32 Dev) - `/dev/ttyUSB0`

#### Output Format (every 500ms)
```
FR: Count=2411  Rev=1.61  RPM=48.1
FL: Count=-2375  Rev=-1.59  RPM=-46.9
BR: Count=2480  Rev=1.66  RPM=49.3
BL: Count=-2442  Rev=-1.63  RPM=-48.5
```

#### Commands
| Command | Description |
|---------|-------------|
| `R` | Reset all encoder counts to 0 |
| `W` | Show wiring guide |
| `H` | Show help |

---

## 4. MECANUM WHEEL LAYOUT

```
              FRONT (camera direction)
        ╔═══════════════════════════════╗
        ║     \               /         ║
        ║    FR[1]         FL[2]        ║
        ║                               ║
        ║     /               \         ║
        ║    BR[3]         BL[4]        ║
        ╚═══════════════════════════════╝
                    BACK

   Roller pattern creates X shape when viewed from top
```

### Mecanum Movement Formula
```python
def calculate_mecanum(x, y, rotation):
    """
    x: Strafe (-1 to 1, left to right)
    y: Forward (-1 to 1, back to front)
    rotation: Spin (-1 to 1, CCW to CW)
    """
    fl = y + x + rotation
    fr = y - x - rotation
    br = y + x - rotation
    bl = y - x + rotation
    
    # Normalize if any > 1
    max_val = max(abs(fl), abs(fr), abs(br), abs(bl), 1)
    return fl/max_val, fr/max_val, br/max_val, bl/max_val
```

---

## 5. PROJECT STRUCTURE

```
~/robots/
├── PI_AI_CONTEXT_PROMPT.md   # Context prompt (old version)
├── KPATROL_CONTEXT.md        # Technical context
├── AI_PROMPT.md              # Short prompt
│
├── pi-controller/            # ⭐ PYTHON SCRIPTS (Pi)
│   ├── kpatrol_controller.py # Full TUI controller (780 lines)
│   ├── kpatrol_cli.py        # Interactive CLI (372 lines)
│   ├── kpatrol_keyboard.py   # WASD keyboard control (321 lines)
│   ├── kpatrol_diagnostics.py# System diagnostics (480 lines)
│   ├── kpatrol_gamepad.py    # Gamepad/joystick support
│   ├── config.env            # Configuration
│   └── README.md             # Documentation
│
├── firmware/                 # ⭐ ARDUINO CODE (ESP32)
│   ├── esp32_s3_motor_controller_pi/
│   │   └── esp32_s3_motor_controller_pi.ino  # Motor controller (535 lines)
│   ├── encoder_reader_4motors/
│   │   └── encoder_reader_4motors.ino        # Encoder reader
│   ├── test_single_motor_bts7960/
│   ├── test_two_motors_FR_FL/
│   ├── test_three_motors_FR_FL_BR/
│   └── test_four_motors_mecanum/
│
├── docs/                     # TECHNICAL DOCS
│   ├── MECANUM_SETUP_GUIDE.md
│   ├── SINGLE_MOTOR_WIRING_GUIDE.md
│   ├── ENCODER_WIRING_4MOTORS.txt
│   └── PINOUT_DIAGRAM_*.txt
│
└── test/                     # TEST SCRIPTS
    ├── setup_motor_wizard.py
    ├── test_single_motor.py
    └── test_mecanum_movement.py
```

---

## 6. VERIFIED TEST RESULTS (Feb 17, 2026)

### 6.1 Connection Test
```
✅ /dev/ttyACM0 - ESP32-S3 Motor Controller (QinHeng USB Single Serial)
✅ /dev/ttyUSB0 - ESP32 Dev Encoder Reader (CH340)
```

### 6.2 Motor Test Results
| Motor | Position | Direction | Speed | Status |
|-------|----------|-----------|-------|--------|
| M1 | Front-Right | Correct | ~48 RPM | ✅ OK |
| M2 | Front-Left | Inverted | ~47 RPM | ✅ OK |
| M3 | Back-Right | Correct | ~49 RPM | ✅ OK |
| M4 | Back-Left | Inverted | ~48 RPM | ✅ OK |

### 6.3 Encoder Test Results
| Motor | Count (Forward 2s) | RPM | Status |
|-------|-------------------|-----|--------|
| FR | +2411 | ~48 | ✅ OK |
| FL | -2375 | ~47 | ✅ OK (negative = inverted) |
| BR | +2480 | ~49 | ✅ OK |
| BL | -2442 | ~48 | ✅ OK (negative = inverted) |

### 6.4 Movement Test
- ✅ Forward/Backward
- ✅ Strafe Left/Right
- ✅ Rotate Left/Right
- ✅ All 4 encoders reading correctly

---

## 7. QUICK START CODE

### 7.1 Python Serial Control
```python
import serial
import time

# Connect to both ESP32s
motor = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
encoder = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(2)  # Wait for ESP32 boot

# Set speed
motor.write(b'SPD:150\n')

# Move forward
motor.write(b'F\n')
time.sleep(2)

# Stop
motor.write(b'S\n')

# Read encoder
encoder.write(b'R\n')  # Reset counts
while encoder.in_waiting:
    print(encoder.readline().decode())

motor.close()
encoder.close()
```

### 7.2 Test Commands
```bash
# Check serial ports
ls /dev/tty* | grep -E '(ACM|USB)'

# Run CLI controller
cd ~/robots/pi-controller
python3 kpatrol_cli.py

# Run keyboard control
python3 kpatrol_keyboard.py
```

---

## 8. WIRING DIAGRAM

### 8.1 BTS7960 Connection
```
┌──────────────────────────────────────────┐
│            BTS7960 MODULE                │
│                                          │
│  [Logic Side - 8 pins]                   │
│  RPWM  LPWM  R_EN  L_EN  R_IS  L_IS  VCC  GND
│   │     │     │     │                │    │
│  ESP32 GPIO  GPIO  GPIO             5V  GND
│                                          │
│  [Power Side - 4 pins]                   │
│   B-    B+    M-    M+                   │
│   │     │     │     │                    │
│  GND   12V  Motor  Motor                 │
└──────────────────────────────────────────┘
```

### 8.2 Motor JGA25-370 (6 wires)
```
Pin 1: Motor -     (BLACK thick) → BTS7960 M-
Pin 2: Encoder GND (BLACK thin)  → ESP32 Dev GND
Pin 3: Encoder A   (YELLOW)      → ESP32 Dev GPIO
Pin 4: Encoder B   (GREEN)       → ESP32 Dev GPIO
Pin 5: Encoder VCC (RED thin)    → ESP32 Dev 3.3V
Pin 6: Motor +     (RED thick)   → BTS7960 M+
```

---

## 9. SAFETY & TROUBLESHOOTING

### ⚠️ Safety Rules
1. **ALWAYS** lift robot when testing motors
2. Use `S` for brake stop, `SPD:0` for emergency
3. Ensure GND is common between ESP32, BTS7960, and 12V supply
4. Power supply must be ≥10A for 4 motors

### 🔧 Troubleshooting

| Issue | Solution |
|-------|----------|
| No /dev/ttyACM0 | Check USB cable, try different port |
| Permission denied | `sudo usermod -a -G dialout $USER`, then logout/login |
| Motor wrong direction | Set `XX_INVERTED true` in firmware |
| Encoder count = 0 | Check encoder wiring (Yellow→A, Green→B, VCC, GND) |
| Robot drifts | Calibrate individual motor speeds |

---

## 10. DEVELOPMENT ROADMAP

| Priority | Feature | Status |
|----------|---------|--------|
| ✅ Done | Basic motor control | Verified |
| ✅ Done | Encoder feedback | Verified |
| 🔴 High | PID speed control | Not started |
| 🔴 High | Odometry | Not started |
| 🟡 Medium | Web interface | Folder exists |
| 🟡 Medium | Camera streaming | Folder exists |
| 🟢 Low | AI patrol | Testing |

---

## 11. CODING STANDARDS

### Python (Pi Controller)
- Use `dataclass` for configurations
- Use `Enum` for command types
- Type hints required
- PEP 8 formatting

### Arduino (ESP32)
- Use `struct` for motor definitions
- Use `IRAM_ATTR` for ISR functions
- PWM via LEDC library
- Serial protocol: text-based, `\n` terminated

---

## 12. CONTEXT FOR AI CONTINUATION

Khi tiếp tục phát triển, AI cần:

1. **Giữ nguyên Serial Protocol** - Format lệnh đã verified
2. **Không thay đổi GPIO** - Đã wiring cố định
3. **Motor Inversion** - FL và BL đã inverted trong firmware
4. **Encoder Direction** - Count âm = quay ngược (đúng với inverted motors)
5. **Speed Range** - 0-255, default 150, min 50 for movement

### Example Tasks
- Implement PID controller for constant speed
- Add odometry calculation from encoder data
- Create web dashboard for remote control
- Integrate camera for AI patrol

---

**END OF CONTEXT PROMPT**

*Prompt Version: 1.0*  
*Hardware Status: Verified Working*  
*Last Test: February 17, 2026*
