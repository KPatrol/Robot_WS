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
#include <esp_task_wdt.h>
#include <esp_idf_version.h>

// ============================================================================
// DEBUG / BUILD FLAGS
// ============================================================================
// KPATROL_DEBUG = 1 → human-readable boot banner + wiring guide + ✓ logs.
// KPATROL_DEBUG = 0 → silent boot; only protocol-prefix lines (ENC:, TOF:,
// ENC_HB:, OK, ERR) and explicit user-invoked H/W output remain. Production
// builds for the Pi must use 0 to avoid corrupting the COMPACT line stream
// the Pi parser expects.
#ifndef KPATROL_DEBUG
#define KPATROL_DEBUG 0
#endif

#if KPATROL_DEBUG
  #define DBG_PRINT(...)    Serial.print(__VA_ARGS__)
  #define DBG_PRINTLN(...)  Serial.println(__VA_ARGS__)
  #define DBG_PRINTF(...)   Serial.printf(__VA_ARGS__)
#else
  #define DBG_PRINT(...)    do {} while (0)
  #define DBG_PRINTLN(...)  do {} while (0)
  #define DBG_PRINTF(...)   do {} while (0)
#endif

// Hardware watchdog: 2s timeout. Loop must reset within this window or the
// chip reboots. Longer than the worst-case ToF read (~140ms × 6 sensors at
// startup = ~1s) but short enough that a wedged loop produces a fast recovery
// rather than silent freeze.
#define WDT_TIMEOUT_SEC   2

// Heartbeat: emit `ENC_HB:<uptime_ms>` every 1s so the Pi detects a hung
// ESP32 Dev even when nothing else changes (e.g. encoders idle, all ToF
// sensors at 9999). Pi-side watchdog can flag stale heartbeat as a failure.
const unsigned long ENC_HB_INTERVAL = 1000;
unsigned long lastEncHbTime = 0;

// ============================================================================
// I2C CONFIGURATION (ToF Sensors)
// ============================================================================
#define I2C_SDA           21    // GPIO 21 - I2C Data
#define I2C_SCL           22    // GPIO 22 - I2C Clock
#define TCA9548A_ADDR     0x70  // TCA9548A I2C address (A0=A1=A2=GND)
#define VL53L0X_ADDR      0x29  // VL53L0X default I2C address (all 6 sensors share this on different mux channels)

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

// ToF readings (mm) — published value (post-median-filter)
uint16_t tofDistance[TOF_COUNT] = {9999, 9999, 9999, 9999, 9999, 9999};
bool tofValid[TOF_COUNT] = {false};

// Bitmask version of tofValid[], emitted in TOF: line so the Pi can
// distinguish "no obstacle (true 9999)" from "sensor failed (fake 9999)".
// Bit i = 1 → sensor i produced a trustworthy reading this cycle.
uint8_t tofValidMask = 0;

// Median-of-3 temporal filter: keep last 3 raw readings per sensor and
// publish the median. Rejects single-frame VL53L0X spikes/dropouts without
// adding latency beyond one extra cycle. 9999 is treated as "no reading"
// when filling the window so a transient out-of-range doesn't poison the
// median permanently.
uint16_t tofRaw[TOF_COUNT][3] = {{9999, 9999, 9999}, {9999, 9999, 9999},
                                 {9999, 9999, 9999}, {9999, 9999, 9999},
                                 {9999, 9999, 9999}, {9999, 9999, 9999}};
uint8_t  tofRawIdx[TOF_COUNT] = {0, 0, 0, 0, 0, 0};

// ToF timing.
// HIGH_SPEED preset → ~20ms timing budget per sensor. With 6 sensors on
// TCA9548A switched sequentially this gives a real loop time of
//   6 × (20ms + ~3ms overhead) ≈ 140ms (~7Hz).
// Setting the gate to 30ms means we always run as fast as the sensors allow
// and never block CPU on artificial waiting. Don't expect a true 33Hz —
// the bottleneck is sensor integration time, not the loop interval.
unsigned long lastToFReadTime = 0;
const unsigned long TOF_READ_INTERVAL = 30;

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

  // Hardware watchdog. Subscribed AFTER serial so a wedge during sensor
  // init still produces a reboot rather than a silent freeze.
#if ESP_IDF_VERSION_MAJOR >= 5
  // ESP-IDF 5.x (Arduino core 3.x): init takes a config struct.
  esp_task_wdt_config_t encWdtCfg = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true,
  };
  esp_task_wdt_init(&encWdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
#endif
  esp_task_wdt_add(NULL);

  DBG_PRINTLN("\n\n");
  DBG_PRINTLN("================================================");
  DBG_PRINTLN("  KPatrol - Encoder + ToF Sensors Reader v3.0");
  DBG_PRINTLN("================================================");

  // Initialize I2C for ToF sensors
  DBG_PRINTLN("\n[1/2] Initializing I2C & ToF sensors...");
  setupToFSensors();
  esp_task_wdt_reset();

  // Initialize encoders
  DBG_PRINTLN("\n[2/2] Initializing encoders...");
  setupEncoders();
  esp_task_wdt_reset();

#if KPATROL_DEBUG
  printWiringGuide();
#endif

  DBG_PRINTLN("\n================================================");
  DBG_PRINTLN("  SYSTEM READY!");
  DBG_PRINTLN("================================================");
  DBG_PRINTLN("Commands: R=Reset, T=ToF, E=Encoder, J=JSON, H=Help\n");

  lastPrintTime = millis();
  lastSpeedCalcTime = millis();
  lastToFReadTime = millis();
  lastEncHbTime = millis();
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
  
  DBG_PRINTLN("  ✓ FR encoder: GPIO 34 (A), GPIO 35 (B)");
  DBG_PRINTLN("  ✓ FL encoder: GPIO 32 (A), GPIO 33 (B)");
  DBG_PRINTLN("  ✓ BR encoder: GPIO 25 (A), GPIO 26 (B)");
  DBG_PRINTLN("  ✓ BL encoder: GPIO 27 (A), GPIO 14 (B)");
}

// ============================================================================
// TCA9548A CHANNEL SELECT
// ============================================================================
// Returns true on success, false if the mux NACKs / is unreachable. Callers
// must propagate the failure (mark sensor invalid) instead of silently
// reading whichever channel was previously selected — that mode produced
// "all distances 9999, no error" when the mux locked up, hiding hardware
// faults from the Pi-side safety layer.
unsigned long tcaFailCount = 0;
bool tcaSelect(uint8_t channel) {
  if (channel > 7) return false;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    tcaFailCount++;
    return false;
  }
  delayMicroseconds(100);  // Small delay for channel switch
  return true;
}

// ============================================================================
// SETUP TOF SENSORS
// ============================================================================
void setupToFSensors() {
  // I²C at 100kHz Standard Mode. Empirically: a pure address scan at 100kHz
  // finds all 6 VL53L0X ACK at 0x29, while 400kHz Fast Mode misses CH2 and
  // CH3 (NACK at probe). The mux lane wiring + sensor cabling can't always
  // satisfy 400kHz tSU/tHD margins; 100kHz gives the headroom needed.
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);

  // VDD warm-up before first transaction.
  // Standalone scanner saw all 6 ACK after Serial setup (~2s). CH0 fails probe
  // here unless we replicate that warm-up. Bump to 1500ms after Wire.begin.
  delay(1500);

  DBG_PRINTLN("  I2C initialized: SDA=GPIO21, SCL=GPIO22 @100kHz");

  // Check if TCA9548A is present, with retries.
  bool tcaOk = false;
  for (int retry = 0; retry < 5; retry++) {
    Wire.beginTransmission(TCA9548A_ADDR);
    if (Wire.endTransmission() == 0) {
      tcaOk = true;
      if (retry > 0) DBG_PRINTF("  ↻ TCA9548A found on retry %d\n", retry);
      break;
    }
    delay(100);
  }
  if (!tcaOk) {
    Serial.println("ERR:TCA9548A_NOT_FOUND");
    DBG_PRINTLN("    Check wiring: VCC→3.3V, GND→GND, SDA→GPIO21, SCL→GPIO22");
    return;
  }
  DBG_PRINTLN("  ✓ TCA9548A found at 0x70");

  // Extra power-on settle. VL53L0X internal boot calibration takes time;
  // some lanes (CH0) drop probe if we hit them too early after reset.
  delay(500);
  esp_task_wdt_reset();

  const char* sensorNames[] = {"FRONT", "FRONT_LEFT", "FRONT_RIGHT", "LEFT", "RIGHT", "BACK"};

  // Pass 1: probe presence on every channel BEFORE any begin() runs.
  // Empirically: a pure scan (no begin() between probes) finds all 6 sensors
  // ACK at 0x29, but interleaving begin()'s many register transactions before
  // probing a later channel makes that later channel NACK. Theory: TCA9548A
  // lane stub capacitance + slow VL53L0X internal state machine need a
  // settling window after sustained activity on a different channel.
  bool probePresent[TOF_COUNT];
  for (int i = 0; i < TOF_COUNT; i++) {
    probePresent[i] = false;
    if (!tcaSelect(i)) {
      Serial.printf("ERR:TOF_MUX_CH%d\n", i);
      continue;
    }
    delay(50);
    // Full-bus warm-up scan 0x01..0x77 (skip TCA 0x70). Standalone scanner
    // pattern: probing many addresses before 0x29 lets the lane's slow
    // VL53L0X / pull-up RC fully settle so 0x29 ACKs reliably. A direct
    // 0x29 probe (even with retries) NACKs on CH2/CH3 — the warm-up is what
    // tips the balance. We don't care which other address ACKs; we only
    // record whether 0x29 ACK'd anywhere in the sweep.
    for (uint8_t addr = 0x01; addr < 0x78; addr++) {
      if (addr == TCA9548A_ADDR) continue;
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      if (err == 0 && addr == VL53L0X_ADDR) {
        probePresent[i] = true;
      }
    }
    if (probePresent[i]) {
      DBG_PRINTF("  ✓ CH%d (%s) probe OK\n", i, sensorNames[i]);
    }
  }

  // Retry pass for any channel that didn't ACK first time. Give each failed
  // lane an extra 300ms settle plus a fresh full-bus sweep. CH0 in particular
  // is the FIRST lane scanned and needs the most warm-up after Wire.begin.
  for (int i = 0; i < TOF_COUNT; i++) {
    if (probePresent[i]) continue;
    if (!tcaSelect(i)) continue;
    delay(300);
    esp_task_wdt_reset();
    for (uint8_t addr = 0x01; addr < 0x78; addr++) {
      if (addr == TCA9548A_ADDR) continue;
      Wire.beginTransmission(addr);
      uint8_t err = Wire.endTransmission();
      if (err == 0 && addr == VL53L0X_ADDR) {
        probePresent[i] = true;
      }
    }
    if (probePresent[i]) {
      DBG_PRINTF("  ↻ CH%d (%s) probe OK on retry\n", i, sensorNames[i]);
    } else {
      Serial.printf("ERR:TOF_NO_ACK_CH%d_%s_err2\n", i, sensorNames[i]);
    }
  }

  // Give sensors extra settle time between probe sweep and begin() —
  // internal VL53L0X boot calibration takes time after power-on; the
  // probe sweep alone doesn't guarantee they're ready for begin().
  delay(300);

  // Pass 2: now init each present sensor, with soft-reset retry.
  for (int i = 0; i < TOF_COUNT; i++) {
    if (!probePresent[i]) {
      tofInitialized[i] = false;
      tofValid[i] = false;
      continue;
    }
    if (!tcaSelect(i)) {
      tofInitialized[i] = false;
      tofValid[i] = false;
      Serial.printf("ERR:TOF_MUX_CH%d\n", i);
      continue;
    }
    delay(80);

    bool initOk = tofSensors[i].begin();

    // Retry with VL53L0X soft-reset (register 0xBF) up to 3 times if begin()
    // fails. Some sensors need an explicit reset to recover from a half-baked
    // power-on calibration state — happens on CH2/CH3 empirically. CH2 in
    // particular sometimes needs multiple reset cycles + longer settle.
    for (int attempt = 0; attempt < 3 && !initOk; attempt++) {
      DBG_PRINTF("  ↻ CH%d (%s) begin failed, soft reset attempt %d\n",
                 i, sensorNames[i], attempt + 1);
      Wire.beginTransmission(VL53L0X_ADDR);
      Wire.write(0xBF);
      Wire.write(0x00);
      Wire.endTransmission();
      delay(10);
      Wire.beginTransmission(VL53L0X_ADDR);
      Wire.write(0xBF);
      Wire.write(0x01);
      Wire.endTransmission();
      delay(100);  // generous post-reset settle (datasheet says ≥1.2ms)
      esp_task_wdt_reset();
      initOk = tofSensors[i].begin();
      if (initOk) {
        DBG_PRINTF("  ✓ CH%d (%s) recovered after %d reset(s)\n",
                   i, sensorNames[i], attempt + 1);
        break;
      }
    }

    if (initOk) {
      tofInitialized[i] = true;
      tofValid[i] = true;

      // HIGH_SPEED: ~20ms timing budget — fastest meaningful refresh rate
      // for a ~1m/s indoor robot. LONG_RANGE (~33ms+ low signal threshold)
      // is overkill indoors and ~1.5× slower per-sensor; the safety net
      // only cares about objects within ~750mm, well inside HIGH_SPEED's
      // valid range (~1.2m typical) so we lose no useful information.
      tofSensors[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);

      DBG_PRINTF("  ✓ ToF #%d (%s) - OK\n", i, sensorNames[i]);
    } else {
      tofInitialized[i] = false;
      tofValid[i] = false;
      // Keep this in production: a missing sensor is a fault the Pi must
      // know about (lane is permanently masked-out below).
      Serial.printf("ERR:TOF_INIT_CH%d_%s\n", i, sensorNames[i]);
    }
  }

  // Count successful sensors
  int successCount = 0;
  for (int i = 0; i < TOF_COUNT; i++) {
    if (tofInitialized[i]) successCount++;
  }
  // Always emit init summary so Pi can log "started with N/6 sensors"
  // regardless of build flavor.
  Serial.printf("STATUS:TOF_INIT,%d,%d\n", successCount, TOF_COUNT);
}

// ============================================================================
// READ ALL TOF SENSORS — strict status filter + median-of-3 temporal filter
// ============================================================================
//   Range status meanings (Adafruit_VL53L0X / ST datasheet):
//     0 = valid
//     1 = sigma fail (low confidence)        → reject as noise
//     2 = signal fail (target too dark/far)  → reject as noise
//     3 = min-range fail (target too close)  → publish min-range sentinel (1)
//                                               so Pi sees "very close" not "far"
//     4 = phase fail (out of range)          → publish 9999 (far)
// Only status 0 is treated as a trustworthy distance value. Anything else
// gets a deterministic sentinel before the median filter sees it.
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

void readToFSensors() {
  VL53L0X_RangingMeasurementData_t measure;
  uint8_t validMask = 0;

  for (int i = 0; i < TOF_COUNT; i++) {
    if (!tofInitialized[i]) {
      tofDistance[i] = 9999;
      tofValid[i] = false;
      continue;
    }

    if (!tcaSelect(i)) {
      // Mux NACK → cannot trust whatever sensor was previously selected.
      // Mark this sensor failed for the cycle; do NOT poison the median
      // window with a fake 1mm — that would force a phantom DANGER on the
      // Pi side. 9999 + valid=false lets the Pi mask this lane out.
      tofDistance[i] = 9999;
      tofValid[i] = false;
      continue;
    }
    tofSensors[i].rangingTest(&measure, false);

    uint16_t raw;
    bool valid = false;
    switch (measure.RangeStatus) {
      case 0:  // valid
        raw = measure.RangeMilliMeter;
        valid = true;
        break;
      case 3:  // too close — collision-imminent signal, NOT far
        raw = 1;  // 1mm sentinel forces DANGER on Pi side
        valid = true;
        break;
      case 1:  // sigma fail
      case 2:  // signal fail
      case 4:  // phase / out of range
      default:
        raw = 9999;  // far / unknown
        valid = false;
        break;
    }

    // Push into median-of-3 ring buffer.
    tofRaw[i][tofRawIdx[i]] = raw;
    tofRawIdx[i] = (tofRawIdx[i] + 1) % 3;
    tofDistance[i] = median3(tofRaw[i][0], tofRaw[i][1], tofRaw[i][2]);
    tofValid[i] = valid;
    if (valid) validMask |= (1u << i);
  }

  tofValidMask = validMask;

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
// Format: TOF:front,front_left,front_right,left,right,back,valid_mask
// The 7th field (valid_mask, hex bitmap of which sensors produced a valid
// reading) is APPENDED rather than replacing the 6 distance fields, so old
// Pi parsers that slice `parts[:6]` keep working unchanged. New parsers can
// read parts[6] and AND with (1<<i) to ignore failed lanes.
// ============================================================================
void printCompactToFData() {
  Serial.printf("TOF:%d,%d,%d,%d,%d,%d,%u\n",
    tofDistance[TOF_FRONT],
    tofDistance[TOF_FRONT_LEFT],
    tofDistance[TOF_FRONT_RIGHT],
    tofDistance[TOF_LEFT],
    tofDistance[TOF_RIGHT],
    tofDistance[TOF_BACK],
    (unsigned)tofValidMask);
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

  // Pet the watchdog every loop iteration. If anything below blocks for
  // >WDT_TIMEOUT_SEC, the chip reboots and the Pi will see a reset banner
  // (or, in production builds, a stale heartbeat) instead of frozen data.
  esp_task_wdt_reset();

  // Calculate encoder speed periodically
  calculateSpeed();

  // Heartbeat: emit even when nothing else is happening so the Pi can
  // distinguish "ESP32 Dev alive but idle" from "ESP32 Dev hung". Includes
  // uptime + cumulative TCA9548A failure count so the Pi can alert on a
  // mux that's degrading mid-run.
  if (currentTime - lastEncHbTime >= ENC_HB_INTERVAL) {
    Serial.printf("ENC_HB:%lu,%lu\n", currentTime, tcaFailCount);
    lastEncHbTime = currentTime;
  }
  
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
