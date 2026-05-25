/**
 * KPatrol Encoder/TOF Reader Firmware v5.0 — FULL BUILD
 *
 * Same wire protocol as v4.1 (Pi-side parser compatible). Layers stacked
 * back together after Layer 1 (TOF only) proved stable in isolation:
 *   - Layer 1: VL53L0X x6 via TCA9548A, bare begin()+HIGH_SPEED, status filter,
 *              median-of-3, back-to-back sweep emitted as TOF:.
 *   - Layer 2: 4x quadrature encoders, IRAM_ATTR ISRs, x2 decoding on A only,
 *              RPM @ 50ms, ENC: emitted @ 20Hz.
 *   - Layer 3: ENC_HB heartbeat @ 1Hz (uptime,tca_fail), ESP32 task WDT 2s.
 *   - Layer 4: Serial command parser (R/T/E/J/M/S/I/W/H), 3 output modes
 *              (HUMAN/COMPACT/JSON), COMPACT default for Pi.
 *
 * Target: ESP32 Dev Module (encoder reader board)
 * I2C:    SDA=21, SCL=22 @ 100 kHz (400 kHz drops CH2/CH3 ACK on this bus)
 * Mux:    TCA9548A @ 0x70 with 6x VL53L0X behind it
 * Encoders:
 *   FR: GPIO 34 (A), GPIO 35 (B)   — NO internal pull-up on 34/35
 *   FL: GPIO 32 (A), GPIO 33 (B)
 *   BR: GPIO 25 (A), GPIO 26 (B)
 *   BL: GPIO 27 (A), GPIO 14 (B)
 *
 * Encoder math: 11 PPR x 34 gear x 2 (CHANGE on A only) = 748 counts/rev
 *
 * Wire protocol emitted (COMPACT mode = default):
 *   STATUS:TOF_INIT_OK,<i>,<name>           — once per sensor at boot
 *   ERR:TOF_INIT_CH<i>_<name>               — boot failure per lane
 *   STATUS:TOF_INIT,<up>,<total>            — boot summary
 *   TOF:front,FL,FR,left,right,back,valid_mask   — every full sweep (~8Hz)
 *   ENC:fr_cnt,fl_cnt,br_cnt,bl_cnt,fr_rpm,fl_rpm,br_rpm,bl_rpm   — 20Hz
 *   ENC_HB:<uptime_ms>,<tca_fail_count>     — 1Hz
 *   OK:<ack>                                — command acks
 */
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <esp_task_wdt.h>
#include <esp_idf_version.h>

// ─── BUILD FLAGS ─────────────────────────────────────────────────────────────
// 0 = silent boot, protocol-only lines (production Pi build). 1 = verbose banner
// and human notes (corrupts the Pi line stream — debug builds only).
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

// ─── PINS / I2C ──────────────────────────────────────────────────────────────
#define I2C_SDA           21
#define I2C_SCL           22
#define I2C_CLOCK_HZ      100000UL
#define TCA9548A_ADDR     0x70
#define VL53L0X_ADDR      0x29

#define FR_ENC_A  34
#define FR_ENC_B  35
#define FL_ENC_A  32
#define FL_ENC_B  33
#define BR_ENC_A  25
#define BR_ENC_B  26
#define BL_ENC_A  27
#define BL_ENC_B  14

#define ENCODER_PPR     11
#define GEAR_RATIO      34
#define COUNTS_PER_REV  (ENCODER_PPR * GEAR_RATIO * 2)  // 748

// ─── TIMING ──────────────────────────────────────────────────────────────────
#define WDT_TIMEOUT_SEC          2
#define VL53L0X_TIMING_BUDGET_MS 20

const unsigned long SPEED_CALC_INTERVAL_MS  = 50;    // 20Hz RPM recompute
const unsigned long ENC_EMIT_INTERVAL_MS    = 50;    // 20Hz ENC: emit
const unsigned long ENC_HB_INTERVAL_MS      = 1000;  // 1Hz heartbeat
const unsigned long HUMAN_PRINT_INTERVAL_MS = 500;   // human/JSON only
const unsigned long TOF_SWEEP_GAP_MS        = 0;     // back-to-back sweeps

// ─── TOF STATE ───────────────────────────────────────────────────────────────
#define TOF_COUNT 6

enum ToFPosition {
  TOF_FRONT = 0,
  TOF_FRONT_LEFT = 1,
  TOF_FRONT_RIGHT = 2,
  TOF_LEFT = 3,
  TOF_RIGHT = 4,
  TOF_BACK = 5
};

static const char* const SENSOR_NAMES[TOF_COUNT] = {
  "FRONT", "FRONT_LEFT", "FRONT_RIGHT", "LEFT", "RIGHT", "BACK"
};

Adafruit_VL53L0X tofSensors[TOF_COUNT];
bool     tofInitialized[TOF_COUNT] = {false, false, false, false, false, false};
uint16_t tofDistance   [TOF_COUNT] = {9999, 9999, 9999, 9999, 9999, 9999};
bool     tofValid      [TOF_COUNT] = {false, false, false, false, false, false};
uint8_t  tofLastStatus [TOF_COUNT] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Median-of-3 ring buffer — rejects single-frame VL53L0X spikes.
uint16_t tofRaw   [TOF_COUNT][3] = {
  {9999, 9999, 9999}, {9999, 9999, 9999}, {9999, 9999, 9999},
  {9999, 9999, 9999}, {9999, 9999, 9999}, {9999, 9999, 9999}
};
uint8_t  tofRawIdx[TOF_COUNT] = {0, 0, 0, 0, 0, 0};

uint8_t  tofValidMask = 0;
uint16_t minFrontDistance = 9999;
unsigned long tcaFailCount = 0;

static const uint16_t INVALID_MM = 9999;

// ─── ENCODER STATE ───────────────────────────────────────────────────────────
volatile long FR_count = 0;
volatile long FL_count = 0;
volatile long BR_count = 0;
volatile long BL_count = 0;

long FR_prev_count = 0;
long FL_prev_count = 0;
long BR_prev_count = 0;
long BL_prev_count = 0;

float FR_rpm = 0;
float FL_rpm = 0;
float BR_rpm = 0;
float BL_rpm = 0;

// ─── CADENCE TIMERS ──────────────────────────────────────────────────────────
unsigned long lastSpeedCalcMs  = 0;
unsigned long lastEncEmitMs    = 0;
unsigned long lastHumanPrintMs = 0;
unsigned long lastEncHbMs      = 0;

// ─── OUTPUT MODE ─────────────────────────────────────────────────────────────
enum OutputMode { MODE_HUMAN, MODE_COMPACT, MODE_JSON };
OutputMode outputMode = MODE_COMPACT;

// ─── ENCODER ISRs (x2 decoding on A only) ────────────────────────────────────
void IRAM_ATTR FR_encoderISR() {
  int a = digitalRead(FR_ENC_A);
  int b = digitalRead(FR_ENC_B);
  if (a == b) FR_count++; else FR_count--;
}
void IRAM_ATTR FL_encoderISR() {
  int a = digitalRead(FL_ENC_A);
  int b = digitalRead(FL_ENC_B);
  if (a == b) FL_count++; else FL_count--;
}
void IRAM_ATTR BR_encoderISR() {
  int a = digitalRead(BR_ENC_A);
  int b = digitalRead(BR_ENC_B);
  if (a == b) BR_count++; else BR_count--;
}
void IRAM_ATTR BL_encoderISR() {
  int a = digitalRead(BL_ENC_A);
  int b = digitalRead(BL_ENC_B);
  if (a == b) BL_count++; else BL_count--;
}

// ─── I2C HELPERS ─────────────────────────────────────────────────────────────
bool tcaSelect(uint8_t ch) {
  if (ch > 7) return false;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1u << ch);
  uint8_t err = Wire.endTransmission();
  if (err != 0) {
    tcaFailCount++;
    return false;
  }
  delayMicroseconds(100);
  return true;
}

// Soft-reset a VL53L0X via register 0xBF — same sequence the debug sketch uses
// between init retries when begin() reports failure.
void softResetVL53L0X() {
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(0xBF);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(10);
  Wire.beginTransmission(VL53L0X_ADDR);
  Wire.write(0xBF);
  Wire.write(0x01);
  Wire.endTransmission();
  delay(100);
}

// ─── TOF INIT ────────────────────────────────────────────────────────────────
bool initOneSensor(uint8_t i) {
  if (!tcaSelect(i)) return false;
  delay(80);  // post-select settle — debug sketch uses the same value

  for (uint8_t attempt = 0; attempt < 3; attempt++) {
    // Bare begin() (no explicit args) is what tof_debug.ino uses. Any
    // explicit-args overload returns 9999 on every channel even though
    // begin() itself reports OK.
    if (tofSensors[i].begin()) {
      tofSensors[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
      return true;
    }
    softResetVL53L0X();
  }
  return false;
}

void setupToFSensors() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(1500);  // VDD warm-up for the whole TOF stack

  bool tcaOk = false;
  for (int retry = 0; retry < 5; retry++) {
    Wire.beginTransmission(TCA9548A_ADDR);
    if (Wire.endTransmission() == 0) {
      tcaOk = true;
      break;
    }
    delay(100);
  }
  if (!tcaOk) {
    Serial.println("ERR:TCA9548A_NOT_FOUND");
    return;
  }

  esp_task_wdt_reset();

  uint8_t up = 0;
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    if (initOneSensor(i)) {
      tofInitialized[i] = true;
      up++;
      Serial.printf("STATUS:TOF_INIT_OK,%u,%s\n", i, SENSOR_NAMES[i]);
    } else {
      Serial.printf("ERR:TOF_INIT_CH%u_%s\n", i, SENSOR_NAMES[i]);
    }
    esp_task_wdt_reset();
  }
  Serial.printf("STATUS:TOF_INIT,%u,%u\n", up, (unsigned)TOF_COUNT);
}

// ─── ENCODER INIT ────────────────────────────────────────────────────────────
void setupEncoders() {
  // GPIO 34/35 have no internal pull-up — driver pushes rail-to-rail in this
  // hardware build, so INPUT is fine. Others get INPUT_PULLUP for noise margin.
  pinMode(FR_ENC_A, INPUT);
  pinMode(FR_ENC_B, INPUT);
  pinMode(FL_ENC_A, INPUT_PULLUP);
  pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP);
  pinMode(BR_ENC_B, INPUT_PULLUP);
  pinMode(BL_ENC_A, INPUT_PULLUP);
  pinMode(BL_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), FR_encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), FL_encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BR_ENC_A), BR_encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_ENC_A), BL_encoderISR, CHANGE);
}

// ─── MEDIAN-OF-3 ─────────────────────────────────────────────────────────────
static uint16_t median3(uint16_t a, uint16_t b, uint16_t c) {
  if (a > b) { uint16_t t = a; a = b; b = t; }
  if (b > c) { uint16_t t = b; b = c; c = t; }
  if (a > b) { uint16_t t = a; a = b; b = t; }
  return b;
}

// ─── TOF READ (blocking, single-shot) ────────────────────────────────────────
// Status filter:
//   0 = valid                                  → raw distance
//   1 = sigma fail   (low confidence)          → 9999 + invalid
//   2 = signal fail  (target too dark/far)     → 9999 + invalid
//   3 = min-range fail (target too close)      → 1mm sentinel + VALID (DANGER)
//   4 = phase fail   (out of range)            → 9999 + invalid
//
// Detach encoder ISRs around rangingTest: GPIO 34/35 noise produces ~22 kHz of
// spurious CHANGE interrupts that disrupt VL53L0X integration timing → every
// lane returns status 2/4. noInterrupts() can't be used because Wire on ESP32
// relies on FreeRTOS primitives and would deadlock; detachInterrupt() per ISR
// is microseconds and only silences the encoder lane.
void readOneToFBlocking(uint8_t i) {
  if (!tofInitialized[i]) {
    tofDistance[i] = INVALID_MM;
    tofValid[i] = false;
    return;
  }
  if (!tcaSelect(i)) {
    tofDistance[i] = INVALID_MM;
    tofValid[i] = false;
    return;
  }

  VL53L0X_RangingMeasurementData_t m;
  detachInterrupt(digitalPinToInterrupt(FR_ENC_A));
  detachInterrupt(digitalPinToInterrupt(FL_ENC_A));
  detachInterrupt(digitalPinToInterrupt(BR_ENC_A));
  detachInterrupt(digitalPinToInterrupt(BL_ENC_A));
  tofSensors[i].rangingTest(&m, false);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), FR_encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), FL_encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BR_ENC_A), BR_encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_ENC_A), BL_encoderISR, CHANGE);

  tofLastStatus[i] = m.RangeStatus;

  uint16_t raw;
  bool valid;
  switch (m.RangeStatus) {
    case 0:
      raw = m.RangeMilliMeter;
      valid = true;
      break;
    case 3:
      raw = 1;          // collision-imminent — Pi side raises DANGER
      valid = true;
      break;
    default:
      raw = INVALID_MM; // 1=sigma_fail, 2=signal_fail, 4=phase_fail
      valid = false;
      break;
  }

  tofRaw[i][tofRawIdx[i]] = raw;
  tofRawIdx[i] = (tofRawIdx[i] + 1) % 3;
  tofDistance[i] = median3(tofRaw[i][0], tofRaw[i][1], tofRaw[i][2]);
  tofValid[i] = valid;
}

void updateMinDistance() {
  minFrontDistance = INVALID_MM;
  for (int i = 0; i < TOF_COUNT; i++) {
    if (tofValid[i] && tofDistance[i] < minFrontDistance) {
      minFrontDistance = tofDistance[i];
    }
  }
}

// ─── COMPACT EMIT ────────────────────────────────────────────────────────────
void emitCompactToFData() {
  uint8_t mask = 0;
  for (int i = 0; i < TOF_COUNT; i++) {
    if (!tofInitialized[i]) {
      tofDistance[i] = INVALID_MM;
      tofValid[i] = false;
      continue;
    }
    if (tofValid[i]) mask |= (1u << i);
  }
  tofValidMask = mask;
  updateMinDistance();

  // Field order MUST match v4/v3 exactly: front, FL, FR, left, right, back,
  // valid_mask. The Pi parser splits on ',' and indexes by position.
  Serial.printf("TOF:%u,%u,%u,%u,%u,%u,%u\n",
    tofDistance[TOF_FRONT],
    tofDistance[TOF_FRONT_LEFT],
    tofDistance[TOF_FRONT_RIGHT],
    tofDistance[TOF_LEFT],
    tofDistance[TOF_RIGHT],
    tofDistance[TOF_BACK],
    (unsigned)tofValidMask);
}

void emitCompactEncoderData() {
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

void emitEncHeartbeat() {
  Serial.printf("ENC_HB:%lu,%lu\n", millis(), tcaFailCount);
}

// ─── RPM CALC ────────────────────────────────────────────────────────────────
void calculateSpeed() {
  unsigned long now = millis();
  float dt = (now - lastSpeedCalcMs) / 1000.0f;
  if (dt < (SPEED_CALC_INTERVAL_MS / 1000.0f)) return;

  noInterrupts();
  long fr_cnt = FR_count;
  long fl_cnt = FL_count;
  long br_cnt = BR_count;
  long bl_cnt = BL_count;
  interrupts();

  long fr_diff = fr_cnt - FR_prev_count;
  long fl_diff = fl_cnt - FL_prev_count;
  long br_diff = br_cnt - BR_prev_count;
  long bl_diff = bl_cnt - BL_prev_count;

  FR_rpm = (fr_diff / (float)COUNTS_PER_REV) * (60.0f / dt);
  FL_rpm = (fl_diff / (float)COUNTS_PER_REV) * (60.0f / dt);
  BR_rpm = (br_diff / (float)COUNTS_PER_REV) * (60.0f / dt);
  BL_rpm = (bl_diff / (float)COUNTS_PER_REV) * (60.0f / dt);

  FR_prev_count = fr_cnt;
  FL_prev_count = fl_cnt;
  BR_prev_count = br_cnt;
  BL_prev_count = bl_cnt;

  lastSpeedCalcMs = now;
}

// ─── HUMAN / JSON OUTPUT (debug modes only) ──────────────────────────────────
void emitHumanData() {
  noInterrupts();
  long fr_cnt = FR_count;
  long fl_cnt = FL_count;
  long br_cnt = BR_count;
  long bl_cnt = BL_count;
  interrupts();

  Serial.println("\n================================================");
  Serial.printf("Time: %.1f sec\n", millis() / 1000.0f);
  Serial.println("-- ENCODER DATA --------------------------------");
  Serial.printf("FR: Count=%6ld  Rev=%6.2f  RPM=%6.1f\n",
    fr_cnt, fr_cnt / (float)COUNTS_PER_REV, FR_rpm);
  Serial.printf("FL: Count=%6ld  Rev=%6.2f  RPM=%6.1f\n",
    fl_cnt, fl_cnt / (float)COUNTS_PER_REV, FL_rpm);
  Serial.printf("BR: Count=%6ld  Rev=%6.2f  RPM=%6.1f\n",
    br_cnt, br_cnt / (float)COUNTS_PER_REV, BR_rpm);
  Serial.printf("BL: Count=%6ld  Rev=%6.2f  RPM=%6.1f\n",
    bl_cnt, bl_cnt / (float)COUNTS_PER_REV, BL_rpm);
  Serial.println("-- ToF DISTANCE (mm) ---------------------------");
  Serial.printf("  FRONT      = %4u   FRONT_LEFT  = %4u   FRONT_RIGHT = %4u\n",
    tofDistance[TOF_FRONT], tofDistance[TOF_FRONT_LEFT], tofDistance[TOF_FRONT_RIGHT]);
  Serial.printf("  LEFT       = %4u   RIGHT       = %4u   BACK        = %4u\n",
    tofDistance[TOF_LEFT], tofDistance[TOF_RIGHT], tofDistance[TOF_BACK]);
  Serial.printf("  Min Distance: %u mm   Valid mask: 0x%02X   TCA fail: %lu\n",
    minFrontDistance, tofValidMask, tcaFailCount);
  Serial.println("================================================");
}

void emitJsonData() {
  noInterrupts();
  long fr_cnt = FR_count;
  long fl_cnt = FL_count;
  long br_cnt = BR_count;
  long bl_cnt = BL_count;
  interrupts();

  Serial.print("{\"enc\":{");
  Serial.printf("\"fr\":{\"cnt\":%ld,\"rpm\":%.1f},", fr_cnt, FR_rpm);
  Serial.printf("\"fl\":{\"cnt\":%ld,\"rpm\":%.1f},", fl_cnt, FL_rpm);
  Serial.printf("\"br\":{\"cnt\":%ld,\"rpm\":%.1f},", br_cnt, BR_rpm);
  Serial.printf("\"bl\":{\"cnt\":%ld,\"rpm\":%.1f}", bl_cnt, BL_rpm);
  Serial.print("},\"tof\":{");
  Serial.printf("\"front\":%u,",       tofDistance[TOF_FRONT]);
  Serial.printf("\"front_left\":%u,",  tofDistance[TOF_FRONT_LEFT]);
  Serial.printf("\"front_right\":%u,", tofDistance[TOF_FRONT_RIGHT]);
  Serial.printf("\"left\":%u,",        tofDistance[TOF_LEFT]);
  Serial.printf("\"right\":%u,",       tofDistance[TOF_RIGHT]);
  Serial.printf("\"back\":%u",         tofDistance[TOF_BACK]);
  Serial.printf("},\"valid_mask\":%u,\"min_distance\":%u,\"tca_fail\":%lu}\n",
    (unsigned)tofValidMask, minFrontDistance, tcaFailCount);
}

// ─── SERIAL COMMANDS ─────────────────────────────────────────────────────────
void resetEncoders() {
  noInterrupts();
  FR_count = 0; FL_count = 0; BR_count = 0; BL_count = 0;
  interrupts();
  FR_prev_count = 0; FL_prev_count = 0; BR_prev_count = 0; BL_prev_count = 0;
  Serial.println("OK:RESET");
}

void scanI2C() {
  Serial.println("-- I2C SCAN --");
  int found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X", addr);
      if (addr == TCA9548A_ADDR) Serial.print(" (TCA9548A)");
      else if (addr == VL53L0X_ADDR) Serial.print(" (VL53L0X)");
      Serial.println();
      found++;
    }
  }
  Serial.printf("  Total: %d\n", found);
}

void printWiringGuide() {
  Serial.println("-- WIRING --");
  Serial.println("  FR enc: GPIO 34 (A), GPIO 35 (B)");
  Serial.println("  FL enc: GPIO 32 (A), GPIO 33 (B)");
  Serial.println("  BR enc: GPIO 25 (A), GPIO 26 (B)");
  Serial.println("  BL enc: GPIO 27 (A), GPIO 14 (B)");
  Serial.println("  I2C: SDA=21, SCL=22 @ 100kHz");
  Serial.println("  TCA9548A @ 0x70, lanes 0..5 = front,FL,FR,left,right,back");
}

void printHelp() {
  Serial.println("-- COMMANDS (v5.0) --");
  Serial.println("  R - Reset encoder counts");
  Serial.println("  T - Emit TOF: line once");
  Serial.println("  E - Emit ENC: line once");
  Serial.println("  S - Print min distance");
  Serial.println("  J - Toggle JSON mode");
  Serial.println("  M - Cycle output mode (HUMAN/COMPACT/JSON)");
  Serial.println("  I - Scan I2C bus");
  Serial.println("  W - Wiring guide");
  Serial.println("  H - This help");
  Serial.println("  DEFAULT: COMPACT (Pi mode), ENC@20Hz, TOF~8Hz, HB@1Hz");
}

void processSerialCommands() {
  if (Serial.available() <= 0) return;
  char cmd = Serial.read();
  switch (cmd) {
    case 'r': case 'R':
      resetEncoders();
      break;
    case 't': case 'T':
      emitCompactToFData();
      break;
    case 'e': case 'E':
      emitCompactEncoderData();
      break;
    case 'j': case 'J':
      outputMode = (outputMode == MODE_JSON) ? MODE_COMPACT : MODE_JSON;
      Serial.println(outputMode == MODE_JSON ? "OK:MODE_JSON" : "OK:MODE_COMPACT");
      break;
    case 'm': case 'M':
      outputMode = (OutputMode)((outputMode + 1) % 3);
      Serial.println(
        outputMode == MODE_HUMAN   ? "OK:MODE_HUMAN" :
        outputMode == MODE_COMPACT ? "OK:MODE_COMPACT" :
                                     "OK:MODE_JSON");
      break;
    case 's': case 'S':
      Serial.printf("MIN:%u\n", minFrontDistance);
      break;
    case 'i': case 'I':
      scanI2C();
      break;
    case 'w': case 'W':
      printWiringGuide();
      break;
    case 'h': case 'H': case '?':
      printHelp();
      break;
  }
}

// ─── ARDUINO ENTRY ───────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t wdtCfg = {
    .timeout_ms = WDT_TIMEOUT_SEC * 1000,
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1,
    .trigger_panic = true,
  };
  esp_task_wdt_init(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_SEC, true);
#endif
  esp_task_wdt_add(NULL);

  DBG_PRINTLN();
  DBG_PRINTLN("================================================");
  DBG_PRINTLN("  KPatrol Firmware v5.0 - FULL BUILD");
  DBG_PRINTLN("  TOF + Encoder + HB + WDT + Commands");
  DBG_PRINTLN("================================================");

  setupToFSensors();
  esp_task_wdt_reset();

  setupEncoders();
  esp_task_wdt_reset();

  unsigned long now = millis();
  lastSpeedCalcMs  = now;
  lastEncEmitMs    = now;
  lastHumanPrintMs = now;
  lastEncHbMs      = now;
}

// ─── MAIN LOOP ───────────────────────────────────────────────────────────────
// Strategy:
//   - Pet WDT every iteration.
//   - Back-to-back 6-channel TOF sweep (matches tof_debug.ino pattern).
//     ~6 x 20ms = ~120ms → ~8 Hz TOF emit. Interleaving each lane with other
//     work was producing uniform status 2/4 in v4 — keep them tight.
//   - Encoder housekeeping runs OUTSIDE the sweep: calculateSpeed() + ENC: emit
//     every 50ms (20 Hz), ENC_HB every 1s.
//   - Serial commands drained every iter.
//   - WDT 2s — well above worst-case sweep (~150ms) + jitter.
void loop() {
  esp_task_wdt_reset();

  // TOF: full back-to-back sweep (~120-150ms).
  for (uint8_t i = 0; i < TOF_COUNT; i++) {
    readOneToFBlocking(i);
  }
  esp_task_wdt_reset();

  if (outputMode == MODE_COMPACT) {
    emitCompactToFData();
  } else {
    uint8_t mask = 0;
    for (int i = 0; i < TOF_COUNT; i++) {
      if (tofInitialized[i] && tofValid[i]) mask |= (1u << i);
    }
    tofValidMask = mask;
    updateMinDistance();
  }

  // Encoder housekeeping — independent of TOF sweep.
  unsigned long now = millis();
  calculateSpeed();

  if (now - lastEncEmitMs >= ENC_EMIT_INTERVAL_MS) {
    lastEncEmitMs = now;
    if (outputMode == MODE_COMPACT) emitCompactEncoderData();
  }

  if (now - lastEncHbMs >= ENC_HB_INTERVAL_MS) {
    lastEncHbMs = now;
    emitEncHeartbeat();
  }

  if (outputMode != MODE_COMPACT &&
      now - lastHumanPrintMs >= HUMAN_PRINT_INTERVAL_MS) {
    lastHumanPrintMs = now;
    if (outputMode == MODE_HUMAN) emitHumanData();
    else if (outputMode == MODE_JSON) emitJsonData();
  }

  processSerialCommands();

  if (TOF_SWEEP_GAP_MS > 0) delay(TOF_SWEEP_GAP_MS);
}
