/*
 * ============================================================================
 * KPatrol Robot - Encoder Reader + ToF Sensors (6x VL53L0X)
 * ============================================================================
 * 
 * Hardware:
 *   - ESP32 Dev Module (standard, NOT ESP32-S3)
 *   - 4x JGA25-370 12V DC Motors with Encoders
 *   - TCA9548A I2C Multiplexer (address 0x70)
 *   - 6x VL53L0X ToF Sensors via TCA9548A
 * 
 * Date: February 25, 2026
 * Version: 3.0 - Raw sensor data for Pi (no local safety logic)
 * Purpose: Read encoders + ToF distance sensors, send raw data to Pi
 * 
 * Motor Encoder Pinout (6-pin connector):
 *   Pin 1: M1 Motor -     (BLACK - thick) → BTS7960 M-
 *   Pin 2: GND Encoder    (BLACK - thin)  → ESP32 GND
 *   Pin 3: Fase A Encoder (YELLOW)        → ESP32 GPIO (interrupt)
 *   Pin 4: Fase B Encoder (GREEN)         → ESP32 GPIO (interrupt)
 *   Pin 5: 3.3V/5V Encoder+ (RED - thin)  → ESP32 3.3V
 *   Pin 6: M1 Motor +     (RED - thick)   → BTS7960 M+
 * 
 * GPIO Allocation:
 *   Encoders:
 *     FR: GPIO 34 (A), GPIO 35 (B)
 *     FL: GPIO 32 (A), GPIO 33 (B)
 *     BR: GPIO 25 (A), GPIO 26 (B)
 *     BL: GPIO 27 (A), GPIO 14 (B)
 * 
 *   I2C (ToF Sensors via TCA9548A):
 *     GPIO 21 → SDA
 *     GPIO 22 → SCL
 * 
 * ToF Sensor Positions (TCA9548A channels):
 *     Channel 0: FRONT        (0°)
 *     Channel 1: FRONT_LEFT   (30°)
 *     Channel 2: FRONT_RIGHT  (-30°)
 *     Channel 3: LEFT         (90°)
 *     Channel 4: RIGHT        (-90°)
 *     Channel 5: BACK         (180°)
 * 
 * Encoder Specifications (JGA25-370):
 *   - PPR (Pulses Per Revolution): 11 PPR (encoder disk)
 *   - Gear Ratio: 1:34
 *   - Effective PPR at output shaft: 11 * 34 = 374 PPR
 *   - With quadrature (x4): 374 * 4 = 1496 counts per revolution
 * 
 * Serial Output Format (for Pi parsing):
 *   ENC:fr_cnt,fl_cnt,br_cnt,bl_cnt,fr_rpm,fl_rpm,br_rpm,bl_rpm
 *   TOF:front,front_left,front_right,left,right,back (mm)
 *
 * NOTE: Safety logic is handled by Pi, not here!
 * 
 * Arduino IDE Settings:
 *   - Board: "ESP32 Dev Module"
 *   - Upload Speed: 921600
 *   - Serial Baud: 115200
 * 
 * Required Libraries:
 *   - Adafruit VL53L0X (Install via Library Manager)
 *   - Wire (built-in)
 * 
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ============================================================================
// I2C CONFIGURATION (ToF Sensors)
// ============================================================================
#define I2C_SDA           21    // GPIO 21 - I2C Data
#define I2C_SCL           22    // GPIO 22 - I2C Clock
#define TCA9548A_ADDR     0x70  // TCA9548A I2C address (A0=A1=A2=GND)

// ============================================================================
// TOF SENSOR CONFIGURATION
// ============================================================================
#define TOF_COUNT         6     // Number of VL53L0X sensors

// ToF sensor positions (TCA9548A channel mapping)
enum ToFPosition {
  TOF_FRONT = 0,        // Channel 0 - Front center (0°)
  TOF_FRONT_LEFT = 1,   // Channel 1 - Front left (30°)
  TOF_FRONT_RIGHT = 2,  // Channel 2 - Front right (-30°)
  TOF_LEFT = 3,         // Channel 3 - Left side (90°)
  TOF_RIGHT = 4,        // Channel 4 - Right side (-90°)
  TOF_BACK = 5          // Channel 5 - Back center (180°)
};

// NOTE: Safety zones are now handled by Pi controller
// ESP32 Dev only sends raw ToF distances

// ToF sensor objects
Adafruit_VL53L0X tofSensors[TOF_COUNT];
bool tofInitialized[TOF_COUNT] = {false};

// ToF readings (mm)
uint16_t tofDistance[TOF_COUNT] = {9999, 9999, 9999, 9999, 9999, 9999};
bool tofValid[TOF_COUNT] = {false};

// ToF timing
unsigned long lastToFReadTime = 0;
const unsigned long TOF_READ_INTERVAL = 50;  // Read every 50ms (20Hz)

// Min distance tracking (for info only, Pi handles safety)
uint16_t minFrontDistance = 9999;

// ============================================================================
// ENCODER PIN DEFINITIONS
// ============================================================================

// Front-Right (FR) Encoder
#define FR_ENC_A  34  // GPIO 34 - Fase A (YELLOW wire)
#define FR_ENC_B  35  // GPIO 35 - Fase B (GREEN wire)

// Front-Left (FL) Encoder
#define FL_ENC_A  32  // GPIO 32 - Fase A (YELLOW wire)
#define FL_ENC_B  33  // GPIO 33 - Fase B (GREEN wire)

// Back-Right (BR) Encoder
#define BR_ENC_A  25  // GPIO 25 - Fase A (YELLOW wire)
#define BR_ENC_B  26  // GPIO 26 - Fase B (GREEN wire)

// Back-Left (BL) Encoder
#define BL_ENC_A  27  // GPIO 27 - Fase A (YELLOW wire)
#define BL_ENC_B  14  // GPIO 14 - Fase B (GREEN wire)

// ============================================================================
// ENCODER SPECIFICATIONS
// ============================================================================
#define ENCODER_PPR         11    // Pulses per revolution (encoder disk)
#define GEAR_RATIO          34    // Gear ratio (1:34)
#define COUNTS_PER_REV      (ENCODER_PPR * GEAR_RATIO * 4)  // 1496 with quadrature

// ============================================================================
// GLOBAL VARIABLES - Encoder Counts (volatile for ISR)
// ============================================================================
volatile long FR_count = 0;
volatile long FL_count = 0;
volatile long BR_count = 0;
volatile long BL_count = 0;

// Previous counts for speed calculation
long FR_prev_count = 0;
long FL_prev_count = 0;
long BR_prev_count = 0;
long BL_prev_count = 0;

// Timing
unsigned long lastPrintTime = 0;
unsigned long lastSpeedCalcTime = 0;
const unsigned long PRINT_INTERVAL = 500;      // Print every 500ms
const unsigned long SPEED_CALC_INTERVAL = 100; // Calculate speed every 100ms

// Speed (RPM)
float FR_rpm = 0;
float FL_rpm = 0;
float BR_rpm = 0;
float BL_rpm = 0;

// ============================================================================
// INTERRUPT SERVICE ROUTINES (ISR)
// ============================================================================

// Front-Right Encoder ISR
void IRAM_ATTR FR_encoderISR() {
  int a = digitalRead(FR_ENC_A);
  int b = digitalRead(FR_ENC_B);
  if (a == b) {
    FR_count++;
  } else {
    FR_count--;
  }
}

// Front-Left Encoder ISR
void IRAM_ATTR FL_encoderISR() {
  int a = digitalRead(FL_ENC_A);
  int b = digitalRead(FL_ENC_B);
  if (a == b) {
    FL_count++;
  } else {
    FL_count--;
  }
}

// Back-Right Encoder ISR
void IRAM_ATTR BR_encoderISR() {
  int a = digitalRead(BR_ENC_A);
  int b = digitalRead(BR_ENC_B);
  if (a == b) {
    BR_count++;
  } else {
    BR_count--;
  }
}

// Back-Left Encoder ISR
void IRAM_ATTR BL_encoderISR() {
  int a = digitalRead(BL_ENC_A);
  int b = digitalRead(BL_ENC_B);
  if (a == b) {
    BL_count++;
  } else {
    BL_count--;
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("================================================");
  Serial.println("  KPatrol - Encoder + ToF Sensors Reader v2.0");
  Serial.println("================================================");
  
  // Initialize I2C for ToF sensors
  Serial.println("\n[1/2] Initializing I2C & ToF sensors...");
  setupToFSensors();
  
  // Initialize encoders
  Serial.println("\n[2/2] Initializing encoders...");
  setupEncoders();
  
  printWiringGuide();
  
  Serial.println("\n================================================");
  Serial.println("  SYSTEM READY!");
  Serial.println("================================================");
  Serial.println("Commands: R=Reset, T=ToF, E=Encoder, J=JSON, H=Help\n");
  
  lastPrintTime = millis();
  lastSpeedCalcTime = millis();
  lastToFReadTime = millis();
}

// ============================================================================
// SETUP ENCODERS
// ============================================================================
void setupEncoders() {
  // Configure encoder pins as inputs with pullup
  // Note: GPIO 34, 35 don't have internal pullup, use external if needed
  pinMode(FR_ENC_A, INPUT);
  pinMode(FR_ENC_B, INPUT);
  pinMode(FL_ENC_A, INPUT_PULLUP);
  pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP);
  pinMode(BR_ENC_B, INPUT_PULLUP);
  pinMode(BL_ENC_A, INPUT_PULLUP);
  pinMode(BL_ENC_B, INPUT_PULLUP);
  
  // Attach interrupts for encoder A channels (CHANGE for quadrature)
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), FR_encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), FL_encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BR_ENC_A), BR_encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_ENC_A), BL_encoderISR, CHANGE);
  
  Serial.println("  ✓ FR encoder: GPIO 34 (A), GPIO 35 (B)");
  Serial.println("  ✓ FL encoder: GPIO 32 (A), GPIO 33 (B)");
  Serial.println("  ✓ BR encoder: GPIO 25 (A), GPIO 26 (B)");
  Serial.println("  ✓ BL encoder: GPIO 27 (A), GPIO 14 (B)");
}

// ============================================================================
// TCA9548A CHANNEL SELECT
// ============================================================================
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
  delayMicroseconds(100);  // Small delay for channel switch
}

// ============================================================================
// SETUP TOF SENSORS
// ============================================================================
void setupToFSensors() {
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400kHz I2C Fast Mode
  
  Serial.println("  I2C initialized: SDA=GPIO21, SCL=GPIO22");
  
  // Check if TCA9548A is present
  Wire.beginTransmission(TCA9548A_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("  ✗ ERROR: TCA9548A not found at 0x70!");
    Serial.println("    Check wiring: VCC→3.3V, GND→GND, SDA→GPIO21, SCL→GPIO22");
    return;
  }
  Serial.println("  ✓ TCA9548A found at 0x70");
  
  // Initialize each VL53L0X sensor
  const char* sensorNames[] = {"FRONT", "FRONT_LEFT", "FRONT_RIGHT", "LEFT", "RIGHT", "BACK"};
  
  for (int i = 0; i < TOF_COUNT; i++) {
    tcaSelect(i);
    delay(10);
    
    if (tofSensors[i].begin()) {
      tofInitialized[i] = true;
      tofValid[i] = true;
      
      // Configure for long range mode (up to 2m)
      tofSensors[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_LONG_RANGE);
      
      Serial.printf("  ✓ ToF #%d (%s) - OK\n", i, sensorNames[i]);
    } else {
      tofInitialized[i] = false;
      tofValid[i] = false;
      Serial.printf("  ✗ ToF #%d (%s) - FAILED\n", i, sensorNames[i]);
    }
  }
  
  // Count successful sensors
  int successCount = 0;
  for (int i = 0; i < TOF_COUNT; i++) {
    if (tofInitialized[i]) successCount++;
  }
  Serial.printf("  ToF sensors initialized: %d/%d\n", successCount, TOF_COUNT);
}

// ============================================================================
// READ ALL TOF SENSORS
// ============================================================================
void readToFSensors() {
  VL53L0X_RangingMeasurementData_t measure;
  
  for (int i = 0; i < TOF_COUNT; i++) {
    if (!tofInitialized[i]) {
      tofDistance[i] = 9999;
      tofValid[i] = false;
      continue;
    }
    
    tcaSelect(i);
    tofSensors[i].rangingTest(&measure, false);
    
    if (measure.RangeStatus != 4) {  // Phase failures = 4 means out of range
      tofDistance[i] = measure.RangeMilliMeter;
      tofValid[i] = true;
    } else {
      tofDistance[i] = 9999;  // Out of range
      tofValid[i] = false;
    }
  }
  
  // Update min front distance (for info, safety handled by Pi)
  updateMinDistance();
}

// ============================================================================
// UPDATE MIN DISTANCE (for monitoring, Pi handles safety)
// ============================================================================
void updateMinDistance() {
  // Get minimum distance from all sensors
  minFrontDistance = 9999;
  
  for (int i = 0; i < TOF_COUNT; i++) {
    if (tofValid[i] && tofDistance[i] < minFrontDistance) {
      minFrontDistance = tofDistance[i];
    }
  }
}

// ============================================================================
// PRINT WIRING GUIDE
// ============================================================================
void printWiringGuide() {
  Serial.println("\n================================================");
  Serial.println("  WIRING GUIDE");
  Serial.println("================================================");
  
  Serial.println("\n--- ENCODER WIRING ---");
  Serial.println("Motor Connector (6 pins):");
  Serial.println("  Pin 1: Motor -  (BLACK thick) → BTS7960 M-");
  Serial.println("  Pin 2: GND      (BLACK thin)  → ESP32 GND");
  Serial.println("  Pin 3: Fase A   (YELLOW)      → ESP32 GPIO");
  Serial.println("  Pin 4: Fase B   (GREEN)       → ESP32 GPIO");
  Serial.println("  Pin 5: VCC      (RED thin)    → ESP32 3.3V");
  Serial.println("  Pin 6: Motor +  (RED thick)   → BTS7960 M+");
  Serial.println("");
  Serial.println("┌─────────┬──────────┬──────────┐");
  Serial.println("│ Motor   │ Fase A   │ Fase B   │");
  Serial.println("├─────────┼──────────┼──────────┤");
  Serial.println("│ FR      │ GPIO 34  │ GPIO 35  │");
  Serial.println("│ FL      │ GPIO 32  │ GPIO 33  │");
  Serial.println("│ BR      │ GPIO 25  │ GPIO 26  │");
  Serial.println("│ BL      │ GPIO 27  │ GPIO 14  │");
  Serial.println("└─────────┴──────────┴──────────┘");
  
  Serial.println("\n--- ToF SENSORS WIRING (via TCA9548A) ---");
  Serial.println("ESP32 → TCA9548A:");
  Serial.println("  GPIO 21 → SDA");
  Serial.println("  GPIO 22 → SCL");
  Serial.println("  3.3V    → VCC");
  Serial.println("  GND     → GND, A0, A1, A2");
  Serial.println("");
  Serial.println("TCA9548A Address Pins:");
  Serial.println("  A0 → GND (0x70)");
  Serial.println("  A1 → GND");
  Serial.println("  A2 → GND");
  Serial.println("  RST → VCC or NC");
  Serial.println("");
  Serial.println("TCA9548A → VL53L0X:");
  Serial.println("┌─────────┬───────────────┬───────────┐");
  Serial.println("│ Channel │ Sensor        │ Position  │");
  Serial.println("├─────────┼───────────────┼───────────┤");
  Serial.println("│ SD0/SC0 │ VL53L0X #0    │ FRONT     │");
  Serial.println("│ SD1/SC1 │ VL53L0X #1    │ FRONT-L   │");
  Serial.println("│ SD2/SC2 │ VL53L0X #2    │ FRONT-R   │");
  Serial.println("│ SD3/SC3 │ VL53L0X #3    │ LEFT      │");
  Serial.println("│ SD4/SC4 │ VL53L0X #4    │ RIGHT     │");
  Serial.println("│ SD5/SC5 │ VL53L0X #5    │ BACK      │");
  Serial.println("└─────────┴───────────────┴───────────┘");
  Serial.println("");
  Serial.println("Each VL53L0X module:");
  Serial.println("  VIN → 3.3V (shared rail)");
  Serial.println("  GND → GND (shared rail)");
  Serial.println("  SDA → TCA SDx");
  Serial.println("  SCL → TCA SCx");
  Serial.println("");
}

// ============================================================================
// CALCULATE SPEED (RPM)
// ============================================================================
void calculateSpeed() {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastSpeedCalcTime) / 1000.0; // seconds
  
  if (deltaTime >= (SPEED_CALC_INTERVAL / 1000.0)) {
    // Get current counts (disable interrupts briefly for atomic read)
    noInterrupts();
    long fr_cnt = FR_count;
    long fl_cnt = FL_count;
    long br_cnt = BR_count;
    long bl_cnt = BL_count;
    interrupts();
    
    // Calculate count difference
    long fr_diff = fr_cnt - FR_prev_count;
    long fl_diff = fl_cnt - FL_prev_count;
    long br_diff = br_cnt - BR_prev_count;
    long bl_diff = bl_cnt - BL_prev_count;
    
    // Calculate RPM: (counts / counts_per_rev) * (60 / deltaTime)
    FR_rpm = (fr_diff / (float)COUNTS_PER_REV) * (60.0 / deltaTime);
    FL_rpm = (fl_diff / (float)COUNTS_PER_REV) * (60.0 / deltaTime);
    BR_rpm = (br_diff / (float)COUNTS_PER_REV) * (60.0 / deltaTime);
    BL_rpm = (bl_diff / (float)COUNTS_PER_REV) * (60.0 / deltaTime);
    
    // Save current counts
    FR_prev_count = fr_cnt;
    FL_prev_count = fl_cnt;
    BR_prev_count = br_cnt;
    BL_prev_count = bl_cnt;
    
    lastSpeedCalcTime = currentTime;
  }
}

// ============================================================================
// PRINT ENCODER DATA (Human readable)
// ============================================================================
void printEncoderData() {
  // Get current counts (atomic read)
  noInterrupts();
  long fr_cnt = FR_count;
  long fl_cnt = FL_count;
  long br_cnt = BR_count;
  long bl_cnt = BL_count;
  interrupts();
  
  // Calculate revolutions
  float fr_rev = fr_cnt / (float)COUNTS_PER_REV;
  float fl_rev = fl_cnt / (float)COUNTS_PER_REV;
  float br_rev = br_cnt / (float)COUNTS_PER_REV;
  float bl_rev = bl_cnt / (float)COUNTS_PER_REV;
  
  Serial.println("── ENCODER DATA ─────────────────────────────────");
  Serial.printf("FR: Count=%6ld  Rev=%6.2f  RPM=%6.1f\n", fr_cnt, fr_rev, FR_rpm);
  Serial.printf("FL: Count=%6ld  Rev=%6.2f  RPM=%6.1f\n", fl_cnt, fl_rev, FL_rpm);
  Serial.printf("BR: Count=%6ld  Rev=%6.2f  RPM=%6.1f\n", br_cnt, br_rev, BR_rpm);
  Serial.printf("BL: Count=%6ld  Rev=%6.2f  RPM=%6.1f\n", bl_cnt, bl_rev, BL_rpm);
}

// ============================================================================
// PRINT TOF DATA (Human readable with visual layout)
// ============================================================================
void printToFData() {
  Serial.println("\n══════════════ ToF DISTANCE (mm) ══════════════");
  
  // Robot top view with sensor positions
  Serial.println("                    FRONT");
  Serial.printf("              ┌────[%4d]────┐\n", tofDistance[TOF_FRONT]);
  Serial.printf("    FRONT_L  [%4d]      [%4d]  FRONT_R\n", 
    tofDistance[TOF_FRONT_LEFT], tofDistance[TOF_FRONT_RIGHT]);
  Serial.println("              │            │");
  Serial.printf("     LEFT   [%4d]  ◎   [%4d]   RIGHT\n", 
    tofDistance[TOF_LEFT], tofDistance[TOF_RIGHT]);
  Serial.println("              │            │");
  Serial.printf("              └────[%4d]────┘\n", tofDistance[TOF_BACK]);
  Serial.println("                    BACK");
  
  Serial.printf("  Min Distance: %d mm\n", minFrontDistance);
  Serial.println("════════════════════════════════════════════════");
}

// ============================================================================
// PRINT COMPACT DATA (for Pi parsing)
// Format: ENC:fr_cnt,fl_cnt,br_cnt,bl_cnt,fr_rpm,fl_rpm,br_rpm,bl_rpm
// ============================================================================
void printCompactEncoderData() {
  noInterrupts();
  long fr_cnt = FR_count;
  long fl_cnt = FL_count;
  long br_cnt = BR_count;
  long bl_cnt = BL_count;
  interrupts();
  
  Serial.printf("ENC:%ld,%ld,%ld,%ld,%.1f,%.1f,%.1f,%.1f\n",
    fr_cnt, fl_cnt, br_cnt, bl_cnt,
    FR_rpm, FL_rpm, BR_rpm, BL_rpm);
}

// ============================================================================
// PRINT COMPACT TOF DATA (for Pi parsing)
// Format: TOF:front,front_left,front_right,left,right,back
// ============================================================================
void printCompactToFData() {
  Serial.printf("TOF:%d,%d,%d,%d,%d,%d\n",
    tofDistance[TOF_FRONT],
    tofDistance[TOF_FRONT_LEFT],
    tofDistance[TOF_FRONT_RIGHT],
    tofDistance[TOF_LEFT],
    tofDistance[TOF_RIGHT],
    tofDistance[TOF_BACK]);
}

// ============================================================================
// PRINT JSON DATA (for easy parsing)
// ============================================================================
void printJSONData() {
  noInterrupts();
  long fr_cnt = FR_count;
  long fl_cnt = FL_count;
  long br_cnt = BR_count;
  long bl_cnt = BL_count;
  interrupts();
  
  Serial.print("{");
  
  // Encoders
  Serial.print("\"enc\":{");
  Serial.printf("\"fr\":{\"cnt\":%ld,\"rpm\":%.1f},", fr_cnt, FR_rpm);
  Serial.printf("\"fl\":{\"cnt\":%ld,\"rpm\":%.1f},", fl_cnt, FL_rpm);
  Serial.printf("\"br\":{\"cnt\":%ld,\"rpm\":%.1f},", br_cnt, BR_rpm);
  Serial.printf("\"bl\":{\"cnt\":%ld,\"rpm\":%.1f}", bl_cnt, BL_rpm);
  Serial.print("},");
  
  // ToF sensors
  Serial.print("\"tof\":{");
  Serial.printf("\"front\":%d,", tofDistance[TOF_FRONT]);
  Serial.printf("\"front_left\":%d,", tofDistance[TOF_FRONT_LEFT]);
  Serial.printf("\"front_right\":%d,", tofDistance[TOF_FRONT_RIGHT]);
  Serial.printf("\"left\":%d,", tofDistance[TOF_LEFT]);
  Serial.printf("\"right\":%d,", tofDistance[TOF_RIGHT]);
  Serial.printf("\"back\":%d", tofDistance[TOF_BACK]);
  Serial.print("},");
  
  // Min distance (Pi handles safety logic)
  Serial.printf("\"min_distance\":%d", minFrontDistance);
  
  Serial.println("}");
}

// Output mode
enum OutputMode {
  MODE_HUMAN,   // Human readable
  MODE_COMPACT, // Compact for Pi parsing (ENC:, TOF:)
  MODE_JSON     // JSON format
};

OutputMode outputMode = MODE_COMPACT;  // Default to COMPACT for Pi parsing

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  unsigned long currentTime = millis();
  
  // Calculate encoder speed periodically
  calculateSpeed();
  
  // Read ToF sensors periodically (50ms = 20Hz)
  if (currentTime - lastToFReadTime >= TOF_READ_INTERVAL) {
    readToFSensors();
    lastToFReadTime = currentTime;
    
    // Send compact ToF data immediately after reading
    if (outputMode == MODE_COMPACT) {
      printCompactToFData();
    }
  }
  
  // Print all data periodically (500ms)
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    switch (outputMode) {
      case MODE_HUMAN:
        Serial.println("\n════════════════════════════════════════════════");
        Serial.printf("Time: %.1f sec\n", millis() / 1000.0);
        printEncoderData();
        printToFData();
        Serial.println("");
        break;
        
      case MODE_COMPACT:
        printCompactEncoderData();
        // ToF already printed in 50ms loop
        break;
        
      case MODE_JSON:
        printJSONData();
        break;
    }
    lastPrintTime = currentTime;
  }
  
  // Check for serial commands
  processSerialCommands();
}

// ============================================================================
// PROCESS SERIAL COMMANDS
// ============================================================================
void processSerialCommands() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'r':
      case 'R':
        // Reset all encoder counters
        noInterrupts();
        FR_count = 0;
        FL_count = 0;
        BR_count = 0;
        BL_count = 0;
        interrupts();
        FR_prev_count = 0;
        FL_prev_count = 0;
        BR_prev_count = 0;
        BL_prev_count = 0;
        Serial.println("OK:RESET");
        break;
        
      case 't':
      case 'T':
        // Print ToF data once
        if (outputMode == MODE_HUMAN) {
          printToFData();
        } else {
          printCompactToFData();
        }
        break;
        
      case 'e':
      case 'E':
        // Print encoder data once
        if (outputMode == MODE_HUMAN) {
          printEncoderData();
        } else {
          printCompactEncoderData();
        }
        break;
        
      case 'j':
      case 'J':
        // Toggle JSON mode
        if (outputMode == MODE_JSON) {
          outputMode = MODE_COMPACT;
          Serial.println("OK:MODE_COMPACT");
        } else {
          outputMode = MODE_JSON;
          Serial.println("OK:MODE_JSON");
        }
        break;
        
      case 'm':
      case 'M':
        // Cycle through output modes
        if (outputMode == MODE_HUMAN) {
          outputMode = MODE_COMPACT;
          Serial.println("OK:MODE_COMPACT");
        } else if (outputMode == MODE_COMPACT) {
          outputMode = MODE_JSON;
          Serial.println("OK:MODE_JSON");
        } else {
          outputMode = MODE_HUMAN;
          Serial.println("OK:MODE_HUMAN");
        }
        break;
        
      case 's':
      case 'S':
        // Print min distance (safety handled by Pi)
        Serial.printf("MIN:%d\n", minFrontDistance);
        break;
        
      case 'i':
      case 'I':
        // I2C scan
        scanI2C();
        break;
        
      case 'w':
      case 'W':
        // Print wiring guide
        printWiringGuide();
        break;
        
      case 'h':
      case 'H':
      case '?':
        printHelp();
        break;
    }
  }
}

// ============================================================================
// I2C SCANNER
// ============================================================================
void scanI2C() {
  Serial.println("\n── I2C SCAN ─────────────────────────────────────");
  int devicesFound = 0;
  
  for (byte address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.printf("  Found device at 0x%02X", address);
      if (address == TCA9548A_ADDR) {
        Serial.print(" (TCA9548A)");
      } else if (address == 0x29) {
        Serial.print(" (VL53L0X)");
      }
      Serial.println();
      devicesFound++;
    }
  }
  
  Serial.printf("  Total devices found: %d\n", devicesFound);
  Serial.println("");
}

// ============================================================================
// PRINT HELP
// ============================================================================
void printHelp() {
  Serial.println("\n================================================");
  Serial.println("  COMMANDS (v3.0 - Raw Data Mode)");
  Serial.println("================================================");
  Serial.println("  R - Reset encoder counts to 0");
  Serial.println("  T - Print ToF sensor data");
  Serial.println("  E - Print encoder data");
  Serial.println("  S - Print min distance");
  Serial.println("  J - Toggle JSON mode");
  Serial.println("  M - Cycle output mode (Human/Compact/JSON)");
  Serial.println("  I - Scan I2C bus");
  Serial.println("  W - Show wiring guide");
  Serial.println("  H - Show this help");
  Serial.println("");
  Serial.println("OUTPUT FORMATS:");
  Serial.println("  Compact: ENC:fr,fl,br,bl,rpm_fr,rpm_fl,rpm_br,rpm_bl");
  Serial.println("           TOF:front,fl,fr,left,right,back (mm)");
  Serial.println("  JSON:    {\"enc\":{...},\"tof\":{...},\"min_distance\":xxx}");
  Serial.println("");
  Serial.println("NOTE: Safety logic is handled by Pi, not here!");
  Serial.printf("Current mode: %s\n", 
    outputMode == MODE_HUMAN ? "HUMAN" : 
    outputMode == MODE_COMPACT ? "COMPACT" : "JSON");
  Serial.println("================================================\n");
}
