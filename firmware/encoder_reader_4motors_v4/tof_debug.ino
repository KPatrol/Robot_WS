/*
 * KPatrol - ToF Debug Sketch (STANDALONE)
 * ============================================================================
 * Purpose:
 *   Isolate the 6x VL53L0X + TCA9548A subsystem from production firmware so we
 *   can diagnose:
 *     - per-sensor latency / jitter
 *     - per-channel failure rate
 *     - "delay & instability" regression vs. last-known-good state
 *
 * Hardware target: same ESP32 Dev board as encoder_reader_4motors.ino
 *   (NOT the ESP32-S3 motor controller — that board has no I2C ToF wiring).
 *
 * ──────────────────────────────────────────────────────────────────────────
 * WIRING — ESP32 Dev ↔ TCA9548A ↔ 6x VL53L0X
 * ──────────────────────────────────────────────────────────────────────────
 *   ESP32 Dev pin           TCA9548A pin
 *   ─────────────────────   ───────────────────
 *   GPIO 21 (SDA)       →   SDA   (master bus)
 *   GPIO 22 (SCL)       →   SCL   (master bus)
 *   3.3V                →   VCC
 *   GND                 →   GND, A0, A1, A2   (address pins all tied LOW → 0x70)
 *   3.3V (or NC)        →   RST   (active LOW; pull HIGH for normal operation)
 *
 *   TCA9548A channel        VL53L0X module        Position / Angle
 *   ─────────────────────   ───────────────────   ────────────────────────
 *   SD0 / SC0           ⇄   VL53L0X #0            FRONT          (  0°)
 *   SD1 / SC1           ⇄   VL53L0X #1            FRONT_LEFT     (+30°)
 *   SD2 / SC2           ⇄   VL53L0X #2            FRONT_RIGHT    (-30°)   *** HW BROKEN ***
 *   SD3 / SC3           ⇄   VL53L0X #3            LEFT           (+90°)
 *   SD4 / SC4           ⇄   VL53L0X #4            RIGHT          (-90°)
 *   SD5 / SC5           ⇄   VL53L0X #5            BACK           (180°)
 *
 *   Each VL53L0X module (shared 3.3V rail off TCA9548A or robot power board):
 *     VIN → 3.3V (DO NOT use 5V — module has on-board LDO but heat will drift readings)
 *     GND → GND
 *     SDA → corresponding TCA SDx
 *     SCL → corresponding TCA SCx
 *     XSHUT → NC (we DO NOT use XSHUT — TCA9548A handles channel isolation)
 *     GPIO1 (INT) → NC (polled mode; interrupt unused)
 *
 *   All six VL53L0X keep their factory I2C address (0x29). The mux is what
 *   isolates them — only one channel is selected at a time.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * KNOWN-GOOD CONFIG (carried over from production)
 * ──────────────────────────────────────────────────────────────────────────
 *   - I2C clock 100 kHz (NOT 400 kHz — Fast Mode misses CH2/CH3 ACK on this
 *     particular bus due to wire length + lane stub capacitance).
 *   - VL53L0X_SENSE_HIGH_SPEED ranging profile (~20 ms timing budget).
 *   - tcaSelect(ch) → write `1 << ch` to TCA9548A control register at 0x70.
 *   - Sequential polling (rangingTest blocking) — no continuous mode here.
 *
 * ──────────────────────────────────────────────────────────────────────────
 * SERIAL COMMANDS (115200 baud)
 * ──────────────────────────────────────────────────────────────────────────
 *   a       → cycle through all 6 channels each iteration (default)
 *   0..5    → isolate ONE channel; read it as fast as possible
 *   k       → toggle skip-CH2 (FR is physically broken; default ON)
 *   s       → print stats summary (success / fail / latency per channel)
 *   r       → reset stats
 *   i       → re-init the whole ToF stack (mux + all sensors)
 *   p       → toggle verbose per-frame print (default OFF after first 10 frames)
 *   h       → help
 *
 * Output format per cycle (when verbose ON):
 *   FRAME #<n>  cycle=<us>us
 *     CH0 FRONT       status=0 raw=  342 mm  read_us= 21340  ok
 *     CH1 FRONT_LEFT  status=0 raw=  680 mm  read_us= 21512  ok
 *     CH2 FRONT_RIGHT status=2 raw= 8191 mm  read_us= 21520  signal_fail
 *     ...
 *
 * CH2 (FR) policy:
 *   - CH2 is under calibration — `effectiveMm()` returns CH2's REAL lastMm
 *     unconditionally (no FL substitution). Status + raw are visible at all
 *     times so the operator can tune the sensor without hidden remapping.
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ─── Pin & address config ───────────────────────────────────────────────────
#define I2C_SDA           21
#define I2C_SCL           22
#define I2C_CLOCK_HZ      100000UL
#define TCA9548A_ADDR     0x70
#define VL53L0X_ADDR      0x29
#define TOF_COUNT         6
#define SERIAL_BAUD       115200

// FR (CH2) was previously marked broken. After re-seating the harness we want
// to re-validate it from boot — set false to test all 6 channels. Press 'k' at
// runtime to skip CH2 again if it spams noise.
bool skipBrokenFR = false;

enum ToFPosition {
  TOF_FRONT = 0,
  TOF_FRONT_LEFT = 1,
  TOF_FRONT_RIGHT = 2,
  TOF_LEFT = 3,
  TOF_RIGHT = 4,
  TOF_BACK = 5,
};
const char* const sensorNames[TOF_COUNT] = {
  "FRONT", "FRONT_LEFT", "FRONT_RIGHT", "LEFT", "RIGHT", "BACK"
};

// ─── Sensor state ───────────────────────────────────────────────────────────
Adafruit_VL53L0X tofSensors[TOF_COUNT];
bool tofInitialized[TOF_COUNT] = {false};

// ─── Stats (per channel) ────────────────────────────────────────────────────
struct ChannelStats {
  uint32_t okCount;
  uint32_t failCount;
  uint32_t totalReadUs;     // sum of successful read times (for avg)
  uint32_t maxReadUs;
  uint32_t minReadUs;
  uint16_t lastStatus;      // RangeStatus from last reading
  uint16_t lastMm;
};
ChannelStats stats[TOF_COUNT];

// ─── Runtime control ────────────────────────────────────────────────────────
enum Mode { MODE_ALL, MODE_SINGLE };
Mode mode = MODE_ALL;
int isolatedChannel = 0;
bool verbose = true;
uint32_t verboseFrameBudget = 10;   // print first 10 frames in verbose, then quiet until 'p'
uint32_t frameCount = 0;
uint32_t lastSummaryMs = 0;
const uint32_t SUMMARY_INTERVAL_MS = 2000;

// ────────────────────────────────────────────────────────────────────────────
// HELPERS
// ────────────────────────────────────────────────────────────────────────────
// Effective distance per channel. CH2 (FR) intentionally returns its REAL
// reading even when invalid — we are calibrating CH2 and want raw visibility,
// not a substituted FL value. Other channels return their own lastMm or 0.
uint16_t effectiveMm(int i) {
  if (i == TOF_FRONT_RIGHT) return stats[i].lastMm;  // always real CH2
  bool selfOk = (stats[i].lastStatus == 0 || stats[i].lastStatus == 3);
  return selfOk ? stats[i].lastMm : 0;
}

bool tcaSelect(uint8_t channel) {
  if (channel >= 8) return false;
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1u << channel);
  return Wire.endTransmission() == 0;
}

void resetStats() {
  for (int i = 0; i < TOF_COUNT; i++) {
    stats[i].okCount = 0;
    stats[i].failCount = 0;
    stats[i].totalReadUs = 0;
    stats[i].maxReadUs = 0;
    stats[i].minReadUs = UINT32_MAX;
    stats[i].lastStatus = 0xFF;
    stats[i].lastMm = 0;
  }
  frameCount = 0;
}

void printHelp() {
  Serial.println(F("\n────────────────────────────────────────────"));
  Serial.println(F("  ToF DEBUG — commands"));
  Serial.println(F("────────────────────────────────────────────"));
  Serial.println(F("  a    : read all 6 channels per cycle"));
  Serial.println(F("  0..5 : isolate one channel"));
  Serial.println(F("  k    : toggle skip-CH2 (FR broken)"));
  Serial.println(F("  s    : print stats summary"));
  Serial.println(F("  r    : reset stats"));
  Serial.println(F("  i    : re-init ToF stack"));
  Serial.println(F("  p    : toggle verbose per-frame print"));
  Serial.println(F("  h    : show this help"));
  Serial.println(F("────────────────────────────────────────────\n"));
}

void printSummary() {
  Serial.println(F("\n──── ToF STATS ─────────────────────────────────────────────────"));
  Serial.printf("  frames=%lu  mode=%s  skip_CH2=%s\n",
                (unsigned long)frameCount,
                mode == MODE_ALL ? "ALL" : "SINGLE",
                skipBrokenFR ? "ON" : "OFF");
  Serial.println(F("  CH  NAME         OK     FAIL   avg_us   min_us   max_us   last_status   last_mm   eff_mm"));
  for (int i = 0; i < TOF_COUNT; i++) {
    uint32_t avg = stats[i].okCount ? (stats[i].totalReadUs / stats[i].okCount) : 0;
    uint32_t mn  = stats[i].minReadUs == UINT32_MAX ? 0 : stats[i].minReadUs;
    Serial.printf("  %d   %-12s %6lu %6lu %8lu %8lu %8lu %12u %9u %8u\n",
                  i, sensorNames[i],
                  (unsigned long)stats[i].okCount,
                  (unsigned long)stats[i].failCount,
                  (unsigned long)avg,
                  (unsigned long)mn,
                  (unsigned long)stats[i].maxReadUs,
                  stats[i].lastStatus,
                  stats[i].lastMm,
                  effectiveMm(i));
  }
  Serial.println(F("────────────────────────────────────────────────────────────────\n"));
}

// ────────────────────────────────────────────────────────────────────────────
// INIT
// ────────────────────────────────────────────────────────────────────────────
bool initOneSensor(int i) {
  if (!tcaSelect(i)) {
    Serial.printf("  CH%d: MUX select failed\n", i);
    return false;
  }
  delay(80);

  bool ok = tofSensors[i].begin();

  // Soft-reset retry — same recovery path as production. Some boards need
  // 1-3 reset cycles after cold boot to come up cleanly on CH2/CH3.
  for (int attempt = 0; attempt < 3 && !ok; attempt++) {
    Serial.printf("  CH%d: begin() failed, soft reset #%d\n", i, attempt + 1);
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
    ok = tofSensors[i].begin();
  }

  if (ok) {
    tofSensors[i].configSensor(Adafruit_VL53L0X::VL53L0X_SENSE_HIGH_SPEED);
    Serial.printf("  CH%d (%s): OK\n", i, sensorNames[i]);
  } else {
    Serial.printf("  CH%d (%s): FAIL\n", i, sensorNames[i]);
  }
  return ok;
}

void initToFStack() {
  Serial.println(F("\n[init] Bringing up I2C + TCA9548A + 6x VL53L0X..."));
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(I2C_CLOCK_HZ);
  delay(1500);  // VL53L0X internal cal needs settle after rail comes up

  // Probe TCA9548A
  Wire.beginTransmission(TCA9548A_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("[init] ERR: TCA9548A NACK at 0x70 — check wiring!"));
    return;
  }
  Serial.println(F("[init] TCA9548A OK"));

  // Init each sensor (skip CH2 if broken flag set — won't waste 3 reset cycles)
  for (int i = 0; i < TOF_COUNT; i++) {
    if (skipBrokenFR && i == TOF_FRONT_RIGHT) {
      tofInitialized[i] = false;
      Serial.printf("  CH%d (%s): SKIPPED (hardware broken)\n", i, sensorNames[i]);
      continue;
    }
    tofInitialized[i] = initOneSensor(i);
  }

  int up = 0;
  for (int i = 0; i < TOF_COUNT; i++) if (tofInitialized[i]) up++;
  Serial.printf("[init] %d/%d sensors up\n\n", up, TOF_COUNT);
  resetStats();
}

// ────────────────────────────────────────────────────────────────────────────
// READ
// ────────────────────────────────────────────────────────────────────────────
// Reads ONE channel, updates its stats, optionally prints verbose line.
void readChannel(int i) {
  if (!tofInitialized[i]) return;

  uint32_t t0 = micros();
  if (!tcaSelect(i)) {
    stats[i].failCount++;
    stats[i].lastStatus = 0xFE;  // sentinel: mux fail
    if (verbose) Serial.printf("  CH%d %-12s MUX_FAIL\n", i, sensorNames[i]);
    return;
  }

  VL53L0X_RangingMeasurementData_t measure;
  tofSensors[i].rangingTest(&measure, false);
  uint32_t dt = micros() - t0;

  stats[i].lastStatus = measure.RangeStatus;
  stats[i].lastMm = measure.RangeMilliMeter;

  if (measure.RangeStatus == 0 || measure.RangeStatus == 3) {
    stats[i].okCount++;
    stats[i].totalReadUs += dt;
    if (dt > stats[i].maxReadUs) stats[i].maxReadUs = dt;
    if (dt < stats[i].minReadUs) stats[i].minReadUs = dt;
  } else {
    stats[i].failCount++;
  }

  if (verbose) {
    Serial.printf("  CH%d %-12s status=%u raw=%5u mm  read_us=%6lu  %s\n",
                  i, sensorNames[i],
                  measure.RangeStatus, measure.RangeMilliMeter,
                  (unsigned long)dt,
                  (measure.RangeStatus == 0) ? "ok" :
                  (measure.RangeStatus == 3) ? "too_close" :
                  (measure.RangeStatus == 1) ? "sigma_fail" :
                  (measure.RangeStatus == 2) ? "signal_fail" :
                  (measure.RangeStatus == 4) ? "phase_fail" : "unknown");
  }
}

void readCycle() {
  frameCount++;
  uint32_t cStart = micros();
  if (verbose) Serial.printf("\nFRAME #%lu\n", (unsigned long)frameCount);

  if (mode == MODE_SINGLE) {
    readChannel(isolatedChannel);
  } else {
    for (int i = 0; i < TOF_COUNT; i++) {
      if (skipBrokenFR && i == TOF_FRONT_RIGHT) continue;
      readChannel(i);
    }
  }

  uint32_t cycleUs = micros() - cStart;
  if (verbose) Serial.printf("  cycle_us=%lu\n", (unsigned long)cycleUs);

  if (frameCount >= verboseFrameBudget && verbose && verboseFrameBudget != UINT32_MAX) {
    verbose = false;
    Serial.println(F("[verbose OFF — press 'p' to re-enable, 's' for summary]"));
  }
}

// ────────────────────────────────────────────────────────────────────────────
// SERIAL CMDS
// ────────────────────────────────────────────────────────────────────────────
void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r' || c == ' ') continue;
    switch (c) {
      case 'a': case 'A':
        mode = MODE_ALL;
        Serial.println(F("[mode] ALL channels"));
        break;
      case '0': case '1': case '2': case '3': case '4': case '5':
        mode = MODE_SINGLE;
        isolatedChannel = c - '0';
        Serial.printf("[mode] SINGLE channel %d (%s)\n",
                      isolatedChannel, sensorNames[isolatedChannel]);
        break;
      case 'k': case 'K':
        skipBrokenFR = !skipBrokenFR;
        Serial.printf("[skip CH2] %s\n", skipBrokenFR ? "ON" : "OFF");
        break;
      case 's': case 'S':
        printSummary();
        break;
      case 'r': case 'R':
        resetStats();
        Serial.println(F("[stats reset]"));
        break;
      case 'i': case 'I':
        initToFStack();
        break;
      case 'p': case 'P':
        verbose = !verbose;
        verboseFrameBudget = verbose ? UINT32_MAX : 0;  // sticky when toggled by user
        Serial.printf("[verbose] %s\n", verbose ? "ON" : "OFF");
        break;
      case 'h': case 'H': case '?':
        printHelp();
        break;
      default:
        Serial.printf("[?] unknown cmd '%c' (h for help)\n", c);
        break;
    }
  }
}

// ────────────────────────────────────────────────────────────────────────────
// ARDUINO ENTRY
// ────────────────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(500);
  Serial.println();
  Serial.println(F("================================================"));
  Serial.println(F("  KPatrol - ToF Debug Sketch v1.0"));
  Serial.println(F("  Target: ESP32 Dev (encoder reader board)"));
  Serial.println(F("================================================"));
  printHelp();
  initToFStack();
  lastSummaryMs = millis();
}

void loop() {
  handleSerial();
  readCycle();

  // Periodic summary so you can leave it running without typing 's'
  uint32_t now = millis();
  if (now - lastSummaryMs > SUMMARY_INTERVAL_MS) {
    lastSummaryMs = now;
    if (!verbose) printSummary();
  }
}
