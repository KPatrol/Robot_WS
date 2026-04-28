# KPatrol Robot - Two Motors Wiring Guide (FR + FL)

**Date:** January 1, 2026  
**Hardware:** ESP32-S3 + 2x BTS7960 + 2x JGA25-370 Motors  
**Status:** Testing Configuration - Front-Right + Front-Left

---

## Table of Contents

1. [Hardware Requirements](#1-hardware-requirements)
2. [Component Overview](#2-component-overview)
3. [Wiring Diagram](#3-wiring-diagram)
4. [Step-by-Step Wiring Instructions](#4-step-by-step-wiring-instructions)
5. [Power Supply Setup](#5-power-supply-setup)
6. [Arduino IDE Configuration](#6-arduino-ide-configuration)
7. [Uploading and Testing](#7-uploading-and-testing)
8. [Serial Commands Reference](#8-serial-commands-reference)
9. [Troubleshooting](#9-troubleshooting)
10. [Safety Guidelines](#10-safety-guidelines)

---

## 1. Hardware Requirements

### Essential Components

| Component | Quantity | Specifications | Notes |
|-----------|----------|----------------|-------|
| ESP32-S3 WROOM-1 Dev Board | 1 | Dual-core, 240MHz, USB-C | DevKit version |
| BTS7960 Motor Driver (IBT-2) | 2 | 43A, 5.5-27V | H-bridge driver |
| JGA25-370 DC Motor | 2 | 12V, 1:34 gearbox, encoder | 6-wire version |
| 12V Power Supply | 1 | 3A minimum, 5A+ recommended | Sufficient current critical |
| USB-C Cable | 1 | Data-capable | For programming ESP32 |
| Jumper Wires | ~20 | Male-to-female, 20cm | For logic connections |
| Power Wires | 4m | 16-18 AWG, red/black | For motor power |
| Breadboard (optional) | 1 | Full-size | For prototyping |
| Ferrule Terminals (optional) | 16 | Various sizes | Secure screw connections |

### Recommended Tools

- Wire stripper & cutter
- Screwdriver set (small Phillips/flathead)
- Multimeter (for continuity & voltage checks)
- Heat-shrink tubing or electrical tape
- Label maker or masking tape (for wire marking)

---

## 2. Component Overview

### ESP32-S3 WROOM-1 Pinout (Relevant Pins)

```
┌─────────────────────────────────────┐
│          ESP32-S3 DEV BOARD         │
├─────────────────────────────────────┤
│ LEFT SIDE:        │ RIGHT SIDE:     │
│ GPIO 4  (FR_RPWM) │                 │
│ GPIO 5  (FR_LPWM) │                 │
│ GPIO 6  (FR_R_EN) │                 │
│ GPIO 7  (FR_L_EN) │                 │
│ GPIO 15 (FL_RPWM) │                 │
│ GPIO 16 (FL_LPWM) │                 │
│ GPIO 17 (FL_R_EN) │                 │
│ GPIO 18 (FL_L_EN) │                 │
│ 5V                │                 │
│ GND               │                 │
└─────────────────────────────────────┘
```

### BTS7960 Driver Pinout

**Logic Side (8 pins):**
- `RPWM` - PWM input for forward rotation
- `LPWM` - PWM input for backward rotation  
- `R_EN` - Enable forward (must be HIGH)
- `L_EN` - Enable backward (must be HIGH)
- `R_IS` - Current sense forward (optional)
- `L_IS` - Current sense backward (optional)
- `VCC` - Logic power (5V from ESP32)
- `GND` - Logic ground (to ESP32 GND)

**Power Side (4 screw terminals):**
- `B+` - Battery positive (12V+)
- `B-` - Battery negative (12V-)
- `M+` - Motor positive (red wire)
- `M-` - Motor negative (black wire)

### JGA25-370 Motor Wiring

**6-Wire Configuration:**
- **RED (thick)** - Motor power positive
- **BLACK (thick)** - Motor power negative
- **RED (thin)** - Encoder VCC (5V) - not used in basic test
- **BLACK (thin)** - Encoder GND - not used in basic test
- **GREEN** - Encoder C1 (Phase A) - not used in basic test
- **BLUE** - Encoder C2 (Phase B) - not used in basic test

⚠️ **CRITICAL:** Only thick RED and BLACK wires connect to BTS7960 M+/M-. Do NOT connect motor wires to common ground or power supply directly.

---

## 3. Wiring Diagram

### Complete System Connection

```
┌─────────────────────────────────────────────────────────────────┐
│                    POWER DISTRIBUTION                           │
└─────────────────────────────────────────────────────────────────┘

12V Power Supply
├── 12V+ ──┬── BTS7960 #1 (B+)
│          └── BTS7960 #2 (B+)
│
└── 12V- ──┬── BTS7960 #1 (B-)
           ├── BTS7960 #2 (B-)
           └── ESP32-S3 GND ⚠️ COMMON GROUND CRITICAL

ESP32-S3 USB-C or 5V
├── 5V ───┬── BTS7960 #1 (VCC)
│         └── BTS7960 #2 (VCC)
│
└── GND ──┬── BTS7960 #1 (GND)
          ├── BTS7960 #2 (GND)
          └── 12V- (Common Ground)

┌─────────────────────────────────────────────────────────────────┐
│              LOGIC CONNECTIONS - FRONT-RIGHT (FR)               │
└─────────────────────────────────────────────────────────────────┘

ESP32-S3          BTS7960 #1 (FR)
GPIO 4  ────────→ RPWM (PWM Ch 0 - Forward)
GPIO 5  ────────→ LPWM (PWM Ch 1 - Backward)
GPIO 6  ────────→ R_EN (Always HIGH)
GPIO 7  ────────→ L_EN (Always HIGH)
5V      ────────→ VCC
GND     ────────→ GND

BTS7960 #1        Motor FR
M+      ────────→ RED (thick wire)
M-      ────────→ BLACK (thick wire)

┌─────────────────────────────────────────────────────────────────┐
│              LOGIC CONNECTIONS - FRONT-LEFT (FL)                │
└─────────────────────────────────────────────────────────────────┘

ESP32-S3          BTS7960 #2 (FL)
GPIO 15 ────────→ RPWM (PWM Ch 2 - Forward)
GPIO 16 ────────→ LPWM (PWM Ch 3 - Backward)
GPIO 17 ────────→ R_EN (Always HIGH)
GPIO 18 ────────→ L_EN (Always HIGH)
5V      ────────→ VCC
GND     ────────→ GND

BTS7960 #2        Motor FL
M+      ────────→ RED (thick wire)
M-      ────────→ BLACK (thick wire)
```

---

## 4. Step-by-Step Wiring Instructions

### Phase 1: Preparation (Power OFF)

1. **Organize Components**
   - Place ESP32-S3 on breadboard (if using)
   - Position both BTS7960 drivers near motors
   - Keep 12V power supply UNPLUGGED

2. **Identify Wires**
   - Label BTS7960 #1 as "FR" (Front-Right)
   - Label BTS7960 #2 as "FL" (Front-Left)
   - Mark motor FR and FL with tape

3. **Prepare Jumper Wires**
   - Cut/prepare 8 wires for FR logic (~20cm each)
   - Cut/prepare 8 wires for FL logic (~20cm each)
   - Recommended colors:
     - RPWM: Yellow
     - LPWM: Blue
     - R_EN: Green
     - L_EN: White
     - VCC: Red
     - GND: Black

### Phase 2: Logic Connections - Front-Right (FR)

Connect ESP32-S3 to BTS7960 #1:

| ESP32-S3 Pin | Wire Color | BTS7960 #1 Pin | Function |
|--------------|------------|----------------|----------|
| GPIO 4 | Yellow | RPWM | Forward PWM |
| GPIO 5 | Blue | LPWM | Backward PWM |
| GPIO 6 | Green | R_EN | Enable Forward |
| GPIO 7 | White | L_EN | Enable Backward |
| 5V | Red (thick) | VCC | Logic Power |
| GND | Black (thick) | GND | Logic Ground |

### Phase 3: Logic Connections - Front-Left (FL)

Connect ESP32-S3 to BTS7960 #2:

| ESP32-S3 Pin | Wire Color | BTS7960 #2 Pin | Function |
|--------------|------------|----------------|----------|
| GPIO 15 | Yellow (labeled FL) | RPWM | Forward PWM |
| GPIO 16 | Blue (labeled FL) | LPWM | Backward PWM |
| GPIO 17 | Green (labeled FL) | R_EN | Enable Forward |
| GPIO 18 | White (labeled FL) | L_EN | Enable Backward |
| 5V | Red (thick) | VCC | Logic Power |
| GND | Black (thick) | GND | Logic Ground |

⚠️ **TIP:** Use heat-shrink tubing or labels to differentiate FL and FR wires

### Phase 4: Motor Connections

**BTS7960 #1 (FR) to Motor FR:**

| BTS7960 #1 | Motor FR Wire | Notes |
|------------|---------------|-------|
| M+ | RED (thick) | Motor power positive |
| M- | BLACK (thick) | Motor power negative |

**BTS7960 #2 (FL) to Motor FL:**

| BTS7960 #2 | Motor FL Wire | Notes |
|------------|---------------|-------|
| M+ | RED (thick) | Motor power positive |
| M- | BLACK (thick) | Motor power negative |

⚠️ **CRITICAL:** Do NOT connect motor wires to power supply ground or battery directly

### Phase 5: Power Supply Connections (LAST STEP)

1. **12V Power to BTS7960 Drivers:**

| Power Supply | BTS7960 #1 | BTS7960 #2 | Wire Gauge |
|--------------|------------|------------|------------|
| 12V+ | B+ | B+ | 16-18 AWG |
| 12V- | B- | B- | 16-18 AWG |

2. **Common Ground (CRITICAL):**
   - Connect 12V- (power supply GND) to ESP32-S3 GND
   - This creates common ground reference for logic and power

3. **Verify Connections:**
   - Use multimeter to check continuity
   - Verify no short circuits between 12V+ and 12V-
   - Confirm all logic connections secure

---

## 5. Power Supply Setup

### Power Requirements

**Per Motor (JGA25-370):**
- No-load current: ~200mA
- Stall current: ~2.5A
- Typical running: 500mA - 1A

**System Total:**
- 2 motors: 1A - 5A (depending on load)
- ESP32-S3: ~200mA
- BTS7960 logic (2x): ~50mA
- **Recommended:** 12V 5A power supply (or higher)

### Power Supply Checklist

- [ ] Voltage: 12V DC (±10%)
- [ ] Current capacity: 3A minimum, 5A+ recommended
- [ ] Barrel jack or screw terminals
- [ ] Verify polarity (center positive common)
- [ ] Add fuse: 5A slow-blow (recommended)

### Adding Capacitors (Optional but Recommended)

Place capacitors near BTS7960 B+/B- terminals:
- **Electrolytic:** 1000µF, 16V+ (one per driver)
- **Ceramic:** 0.1µF (one per driver)

Purpose: Filter voltage spikes, reduce motor noise

---

## 6. Arduino IDE Configuration

### Board Settings

1. **Install ESP32 Board Support:**
   - Arduino IDE → Preferences
   - Additional Boards Manager URLs:
     ```
     https://espressif.github.io/arduino-esp32/package_esp32_index.json
     ```
   - Tools → Board → Boards Manager → Search "ESP32" → Install v3.0.7

2. **Select Board:**
   - Tools → Board → ESP32 Arduino → **ESP32S3 Dev Module**

3. **Critical Settings:**

| Setting | Value | CRITICAL? |
|---------|-------|-----------|
| USB CDC On Boot | **Disabled** | ✅ YES! |
| Upload Mode | UART0 / Hardware CDC | ✅ YES! |
| USB Mode | Hardware CDC and JTAG | Recommended |
| CPU Frequency | 240MHz (WiFi/BT) | Default |
| Flash Size | 4MB (32Mb) | Default |
| Upload Speed | 921600 | Recommended |
| Partition Scheme | Default 4MB with spiffs | Default |
| PSRAM | Disabled | Default |

4. **Select Port:**
   - Tools → Port → `/dev/cu.usbserial-XXXX` (macOS/Linux)
   - Or: `COM3`, `COM4`, etc. (Windows)

### Serial Monitor Settings

- **Baud Rate:** 115200
- **Line Ending:** Both NL & CR (or Newline)
- **Timestamp:** Off (optional)

---

## 7. Uploading and Testing

### Upload Firmware

1. **Open Test File:**
   - File → Open → Navigate to: `robots/firmware/test_two_motors_FR_FL/test_two_motors_FR_FL.ino`

2. **Verify Code:**
   - Click ✓ (Verify) button
   - Check for compilation errors

3. **Upload:**
   - Click → (Upload) button
   - Wait for "Done uploading" message

4. **Open Serial Monitor:**
   - Tools → Serial Monitor
   - Set baud to **115200**
   - You should see:
     ```
     ========================================
       KPatrol - Two Motors Test (FR + FL)
     ========================================
     Initializing motors...
     ✓ Front-Right motor configured
     ✓ Front-Left motor configured
     
     ✓ System ready!
     ```

### Initial Testing (Power OFF)

**Before connecting 12V power:**

1. Upload code and verify serial output works
2. Type commands in Serial Monitor:
   - `I` - Should show system info
   - `H` - Should show help menu
3. Commands won't spin motors yet (no power)

### Powered Testing (Power ON)

**After connecting 12V power supply:**

1. **Power Supply Check:**
   - Verify BTS7960 power LEDs are ON (usually red)
   - Check voltage: 11-13V at B+/B- terminals

2. **Test Sequence (use commands):**

```
> SPD:100          (Set to slow speed first - safer)
> FR_F             (Front-Right should spin forward)
> FR_S             (Stop)
> FR_B             (Front-Right should spin backward)
> FR_S             (Stop)

> FL_F             (Front-Left should spin forward)
> FL_S             (Stop)
> FL_B             (Front-Left should spin backward)
> FL_S             (Stop)

> F                (Both motors forward - robot moves forward)
> S                (Stop both)

> R                (Rotate right - FR back, FL forward)
> S                (Stop)

> L                (Rotate left - FR forward, FL back)
> S                (Stop)

> T                (Run full automated test sequence)
```

3. **Automated Test:**
   - Type `T` and press Enter
   - System will run through all movement patterns
   - Duration: ~30 seconds
   - Observe both motors during test

---

## 8. Serial Commands Reference

### Individual Motor Control

| Command | Action | Notes |
|---------|--------|-------|
| `FR_F` | Front-Right Forward | At current speed |
| `FR_B` | Front-Right Backward | At current speed |
| `FR_S` | Front-Right Stop | Immediate brake |
| `FL_F` | Front-Left Forward | At current speed |
| `FL_B` | Front-Left Backward | At current speed |
| `FL_S` | Front-Left Stop | Immediate brake |

### Combined Movement

| Command | Action | Robot Behavior |
|---------|--------|----------------|
| `F` | Both Forward | Robot moves forward |
| `B` | Both Backward | Robot moves backward |
| `S` | Both Stop | Robot stops |
| `R` | Rotate Right | FR back, FL forward (in-place rotation) |
| `L` | Rotate Left | FR forward, FL back (in-place rotation) |

### Speed Control

| Command | Action | Range |
|---------|--------|-------|
| `SPD:xxx` | Set speed | 0-255 (e.g., `SPD:150`) |

**Speed Reference:**
- `SPD:50` - Very slow (10-20 RPM)
- `SPD:100` - Slow (40-50 RPM)
- `SPD:150` - Medium (default, ~80 RPM)
- `SPD:200` - Fast (~100 RPM)
- `SPD:255` - Maximum (~120 RPM)

### System Commands

| Command | Action | Output |
|---------|--------|--------|
| `T` | Run automated test sequence | ~30 seconds, all patterns |
| `I` | Show system information | GPIO, PWM, memory info |
| `H` | Show help menu | All commands listed |

---

## 9. Troubleshooting

### Serial Monitor Issues

**Problem:** No output or garbage characters

**Solutions:**
1. ✅ **Verify USB CDC On Boot = Disabled** (most common)
2. Check baud rate = 115200
3. Try different USB cable (must support data)
4. Select correct port in Tools → Port
5. Press EN/RESET button on ESP32 after opening Serial Monitor

---

**Problem:** "A fatal error occurred: Could not open port"

**Solutions:**
1. Close Serial Monitor before uploading
2. Check USB cable connection
3. Install CP210x or CH340 drivers (if needed)
4. Try different USB port on computer

---

### Motor Issues

**Problem:** One or both motors don't spin

**Checklist:**
1. **Power:**
   - [ ] 12V power supply connected and ON
   - [ ] BTS7960 power LEDs are ON (red)
   - [ ] Measure voltage at B+ and B-: should be ~12V
   
2. **Logic:**
   - [ ] Enable pins HIGH (check R_EN, L_EN in code)
   - [ ] PWM signals connected (RPWM, LPWM)
   - [ ] 5V and GND to BTS7960 VCC/GND
   
3. **Motor:**
   - [ ] M+ and M- connected to motor thick wires
   - [ ] Motor wires not connected to GND (common mistake!)
   - [ ] Try swapping M+ and M- (tests direction)

4. **Commands:**
   - [ ] Set speed: `SPD:150`
   - [ ] Try individual commands: `FR_F`, `FL_F`

---

**Problem:** Motor spins one direction only

**Cause:** M- wire connected to power supply GND (like your previous issue!)

**Solution:**
- Disconnect M- from any ground or negative terminal
- M- connects ONLY to BTS7960 M- screw terminal
- M+ and M- must be isolated from power supply

---

**Problem:** Only one motor works, other is silent

**Solutions:**
1. Test power: Swap motor connections (see if problem follows motor or driver)
2. Check second BTS7960 driver:
   - Power LED ON?
   - B+ and B- have voltage?
   - Logic wires connected?
3. Test with individual commands (`FR_F` vs `FL_F`)
4. Check power supply current capacity (might be maxing out)

---

**Problem:** Motors work individually but not together

**Cause:** Insufficient power supply current

**Solutions:**
- Upgrade to 5A or higher power supply
- Check for loose B+ or B- connections
- Add bulk capacitors (1000µF) near drivers
- Reduce speed when both motors run: `SPD:100`

---

**Problem:** Motor direction reversed

**Solutions:**
- **Hardware:** Swap M+ and M- wires on that motor
- **Software:** Swap RPWM/LPWM channels in code:
  ```cpp
  // If FR motor is backward:
  #define FR_RPWM 5  // Was 4
  #define FR_LPWM 4  // Was 5
  ```

---

### Power Supply Issues

**Problem:** System resets when motors start

**Cause:** Voltage drop / insufficient current

**Solutions:**
1. Use power supply with higher current rating (5A+)
2. Add electrolytic capacitor: 1000µF 16V near BTS7960 B+/B-
3. Use thicker power wires (16 AWG or lower gauge number)
4. Reduce motor speed: `SPD:100`
5. Power ESP32 from separate 5V source (USB)

---

**Problem:** BTS7960 power LED flickers

**Cause:** Loose connection or insufficient voltage

**Solutions:**
- Tighten B+ and B- screw terminals on BTS7960
- Measure voltage at terminals: should be steady 11-13V
- Check power supply output with multimeter
- Inspect power wire gauge (should be 16-18 AWG minimum)

---

### Software Issues

**Problem:** Code won't compile

**Common Errors:**
1. "LEDC not found" → Install ESP32 board support v3.0+
2. "GPIO not defined" → Check pin definitions at top of code
3. "Upload failed" → Close Serial Monitor before upload

**Solutions:**
- Update ESP32 board support to latest (v3.0.7+)
- Verify board selection: "ESP32S3 Dev Module"
- Check USB cable supports data transfer

---

**Problem:** Commands not recognized

**Example:** Type `F` but nothing happens

**Solutions:**
1. Check Serial Monitor line ending: "Both NL & CR" or "Newline"
2. Type command and press ENTER
3. Commands are case-insensitive (f or F both work)
4. Check serial output for error messages
5. Try `H` command to verify communication

---

## 10. Safety Guidelines

### Electrical Safety

⚠️ **DO:**
- ✅ Connect power LAST (after all logic wiring verified)
- ✅ Verify polarity with multimeter before powering on
- ✅ Use fuse or circuit breaker on 12V line (5A rating)
- ✅ Keep 12V power wiring separate from logic wiring
- ✅ Add capacitors near BTS7960 for voltage stability

⚠️ **DON'T:**
- ❌ Connect motor M- to power supply GND (causes unidirectional issue)
- ❌ Reverse 12V polarity (can destroy BTS7960)
- ❌ Hot-swap motor connections while powered
- ❌ Use undersized power supply (<3A)
- ❌ Touch motor terminals while running

### Mechanical Safety

⚠️ **DO:**
- ✅ Secure motors to chassis before testing
- ✅ Start with low speed (`SPD:50` or `SPD:100`)
- ✅ Keep fingers away from gears and wheels
- ✅ Test on blocks (wheels off ground) initially

⚠️ **DON'T:**
- ❌ Reverse motor direction at full speed without delay
- ❌ Stall motors for extended periods (overheating)
- ❌ Run motors without load for long duration (encoder shaft stress)

### BTS7960 Driver Protection

**Avoid H-Bridge Shoot-Through:**
- Code includes 200ms delay when changing direction
- Don't manually set both RPWM and LPWM HIGH simultaneously
- Let code handle direction changes safely

**Thermal Protection:**
- BTS7960 has thermal shutdown (usually 150°C)
- If driver gets very hot (>70°C), reduce duty cycle
- Add heatsink to BTS7960 chip (optional)
- Ensure adequate airflow in enclosure

**Current Monitoring (Optional):**
- Connect R_IS and L_IS pins to ESP32 ADC
- Read current consumption in real-time
- Implement software current limiting

---

## Testing Checklist

### Pre-Power Checks

- [ ] All logic wires connected per pinout (8 per driver)
- [ ] Enable pins (R_EN, L_EN) wired correctly
- [ ] 5V and GND shared between ESP32 and both BTS7960
- [ ] Motor M+/M- connected (thick wires only)
- [ ] Motor M- NOT connected to common ground
- [ ] 12V power supply polarity verified with multimeter
- [ ] B+ and B- terminals secure on both drivers
- [ ] Common ground: 12V- ↔ ESP32 GND
- [ ] No short circuits between 12V+ and 12V-
- [ ] Code uploaded successfully
- [ ] Serial Monitor shows "System ready!" at 115200 baud

### Post-Power Tests

- [ ] BTS7960 power LEDs ON (both drivers)
- [ ] Measure 12V at B+ terminals with multimeter
- [ ] `SPD:100` command sets low speed
- [ ] `FR_F` spins Front-Right motor forward
- [ ] `FR_B` spins Front-Right motor backward
- [ ] `FR_S` stops Front-Right motor
- [ ] `FL_F` spins Front-Left motor forward
- [ ] `FL_B` spins Front-Left motor backward
- [ ] `FL_S` stops Front-Left motor
- [ ] `F` command spins both motors forward
- [ ] `B` command spins both motors backward (robot backward)
- [ ] `R` command rotates right (FR back, FL forward)
- [ ] `L` command rotates left (FR forward, FL back)
- [ ] `T` command runs full automated test sequence
- [ ] Motor directions match expected robot movement
- [ ] No excessive heat on BTS7960 drivers
- [ ] No voltage sag when both motors start

---

## Next Steps

### After Successful Two-Motor Test:

1. **Add Rear Motors (RR + RL):**
   - Use GPIO 8, 9, 10, 11 for Rear-Right
   - Use GPIO 12, 13, 14, 21 for Rear-Left
   - Follow same wiring pattern as FR/FL

2. **Implement Mecanum Wheel Kinematics:**
   - Forward: All 4 wheels same direction
   - Backward: All 4 wheels reverse
   - Strafe Right: FR+RL forward, FL+RR backward
   - Strafe Left: FL+RR forward, FR+RL backward
   - Rotate: Left wheels forward, right wheels backward (or vice versa)
   - Diagonal: Speed differential between sides

3. **Add Encoder Feedback:**
   - Connect encoder thin wires (VCC, GND, C1, C2)
   - Implement interrupt-based encoder counting
   - Calculate speed (RPM) and distance (odometry)
   - Closed-loop PID speed control

4. **Integrate with Backend:**
   - WiFi connection to NestJS backend
   - WebSocket real-time control
   - Send telemetry data (speed, position, battery)
   - Receive movement commands from mobile app

---

## Reference Documents

- **Pinout Diagram:** `robots/docs/PINOUT_DIAGRAM_TWO_MOTORS.txt`
- **Single Motor Guide:** `robots/docs/SINGLE_MOTOR_WIRING_GUIDE.md`
- **Firmware:** `robots/firmware/test_two_motors_FR_FL/test_two_motors_FR_FL.ino`
- **Mecanum Setup:** `robots/docs/MECANUM_SETUP_GUIDE.md` (for 4-motor system)

---

## Support & Troubleshooting

If you encounter issues not covered in this guide:

1. **Check Serial Monitor Output:**
   - Error messages provide specific clues
   - `I` command shows system configuration

2. **Isolate Problem:**
   - Test components individually (one motor at a time)
   - Swap connections to identify faulty hardware

3. **Measure with Multimeter:**
   - Verify voltages at each stage (12V, 5V, GPIO)
   - Check continuity of all connections

4. **Review Previous Fix:**
   - Remember: Motor M- must NOT connect to GND!
   - This was your breakthrough discovery

---

**Document Version:** 1.0  
**Last Updated:** January 1, 2026  
**Tested Configuration:** ESP32-S3 + 2x BTS7960 + 2x JGA25-370

**Status:** ✅ Validated - Wiring pattern tested and working
