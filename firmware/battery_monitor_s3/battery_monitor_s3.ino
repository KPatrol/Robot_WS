/*
 * ============================================================================
 * KPatrol — Battery Monitor (ESP32-S3 + INA219, Vbus-only mode) — DEPRECATED
 * ============================================================================
 *
 * !!! DEPRECATED — kept only as a standalone bench/diagnostic sketch !!!
 *
 * Production now consolidates the INA219 read into the single motor-controller
 * sketch:  firmware/controller_four_motors_mecanum/controller_four_motors_mecanum.ino
 * (look for #define INA219_ADDR and printBatteryCompact()).
 *
 * Only one ESP32-S3 ships on the robot, and it runs the motor + IMU + battery
 * firmware as one binary. This sketch remains useful for bench testing the
 * INA219 module in isolation when diagnosing wiring/I²C issues — but do NOT
 * flash it to the production board.
 *
 * Purpose (historical)
 *   Standalone firmware for a dedicated battery-monitor ESP32-S3.
 *   Reads bus voltage from INA219, applies a moving-average filter, classifies
 *   pack state for an LFP 4S battery, and streams telemetry over native USB
 *   serial to the Raspberry Pi at 115200 bps.
 *
 * Wiring (Vbus-only mode — NO current measurement)
 *   INA219 VCC   → ESP32-S3 3V3
 *   INA219 GND   → ESP32-S3 GND  (also tied to LFP pack GND)
 *   INA219 SDA   → ESP32-S3 GPIO 19
 *   INA219 SCL   → ESP32-S3 GPIO 20
 *   INA219 VIN+  ──┐
 *                  ├── shorted, both tied to LFP+ (12V bus) — 0 A through shunt
 *   INA219 VIN−  ──┘
 *
 *   The 0.1Ω / 2W on-module shunt sees ~0 A so we only trust getBusVoltage_V().
 *   NEVER place the shunt in series with the motor load — it will burn at >3 A.
 *
 * Telemetry (out, ASCII lines @ 115200, terminated by '\n')
 *   BOOT:battery-mon-s3-v1.0,sda=19,scl=20,addr=0x40
 *   HB:t=<ms_since_boot>                                — every 1 s
 *   BAT:v=<vbus>,vavg=<filtered>,pct=<percent>,state=<S> — every 1 s
 *   I2C:scan=<list>                                     — on demand / on retry
 *
 *   State S ∈ { NO_PACK | CRITICAL | LOW | OK | HIGH }
 *
 * Commands (in, ASCII lines, terminated by '\n' or '\r\n')
 *   PING        → PONG:t=<ms>
 *   STATUS      → emits a BAT line immediately + a short META line
 *   SCAN        → re-runs I2C scan
 *
 * LFP 4S pack thresholds (volts)
 *   V_EMPTY      = 10.0   (≈ 2.50 V/cell — hard cutoff)
 *   V_CRITICAL   = 10.4   (≈ 2.60 V/cell — stop motors, RTH)
 *   V_LOW        = 11.0   (≈ 2.75 V/cell — warn the operator)
 *   V_FULL       = 14.4   (≈ 3.60 V/cell — rest voltage when full)
 *   V_OVER       = 15.0   (above this → wrong pack / wrong wiring)
 *   Hysteresis   = 0.2 V on every transition (avoid flapping near boundary)
 *
 * Arduino IDE settings
 *   Board:   "ESP32S3 Dev Module"
 *   USB CDC On Boot: Enabled
 *   Upload Mode: UART0 (USB CDC will work after first boot)
 *   Upload Speed: 921600
 *   Flash Size: 4MB (or 8MB depending on module)
 *   PSRAM: Disabled
 *   Baud:    115200
 *   Library: "Adafruit INA219" by Adafruit (via Library Manager)
 *
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_INA219.h>

// ─── Pin / bus config — MUST stay in sync with the main motor firmware ──────
#define I2C_SDA_PIN     19
#define I2C_SCL_PIN     20
#define I2C_FREQ_HZ     100000UL    // 100 kHz — safe for INA219
#define INA219_ADDR     0x40        // A0=A1=GND

// ─── LFP 4S pack thresholds (volts) ────────────────────────────────────────
static constexpr float V_EMPTY     = 10.0f;
static constexpr float V_CRITICAL  = 10.4f;
static constexpr float V_LOW       = 11.0f;
static constexpr float V_FULL      = 14.4f;
static constexpr float V_OVER      = 15.0f;
static constexpr float V_NO_PACK   = 1.0f;   // anything below = no battery
static constexpr float HYST_V      = 0.2f;

// ─── Cadence (ms) ──────────────────────────────────────────────────────────
static constexpr uint32_t SAMPLE_PERIOD_MS = 100;   // 10 Hz raw sample
static constexpr uint32_t EMIT_PERIOD_MS   = 1000;  // 1 Hz telemetry
static constexpr uint32_t HB_PERIOD_MS     = 1000;  // 1 Hz heartbeat
static constexpr uint32_t RETRY_PERIOD_MS  = 2000;  // recover after I2C failure

// ─── Moving-average filter ─────────────────────────────────────────────────
static constexpr uint8_t MA_WINDOW = 8;

enum PackState : uint8_t {
  STATE_NO_PACK  = 0,
  STATE_CRITICAL = 1,
  STATE_LOW      = 2,
  STATE_OK       = 3,
  STATE_HIGH     = 4,
};

static const char* state_name(PackState s) {
  switch (s) {
    case STATE_NO_PACK:  return "NO_PACK";
    case STATE_CRITICAL: return "CRITICAL";
    case STATE_LOW:      return "LOW";
    case STATE_OK:       return "OK";
    case STATE_HIGH:     return "HIGH";
  }
  return "?";
}

// ─── Globals ───────────────────────────────────────────────────────────────
Adafruit_INA219 ina219(INA219_ADDR);
bool      ina_ok          = false;
uint32_t  t_last_sample   = 0;
uint32_t  t_last_emit     = 0;
uint32_t  t_last_hb       = 0;
uint32_t  t_last_retry    = 0;

float     ma_buf[MA_WINDOW] = {0};
uint8_t   ma_idx            = 0;
uint8_t   ma_count          = 0;
float     vbus_last         = 0.0f;
float     vbus_avg          = 0.0f;
PackState state             = STATE_NO_PACK;

String    cmd_buf;

// ─── Battery % (linear approximation across V_EMPTY..V_FULL) ────────────────
//
// LFP discharge curve is famously flat (~13.2 V from 95% → 15%), so this
// linear mapping is intentionally conservative — it under-reports near the
// flat region. Acceptable for an "is the pack OK?" indicator; not a fuel gauge.
static float battery_pct_lfp4s(float vbus) {
  const float span = V_FULL - V_EMPTY;
  if (span <= 0.0f) return 0.0f;
  float p = (vbus - V_EMPTY) / span;
  if (p < 0.0f) p = 0.0f;
  if (p > 1.0f) p = 1.0f;
  return p * 100.0f;
}

// ─── State classifier with hysteresis ───────────────────────────────────────
//
// We require the voltage to cross the threshold by HYST_V before downgrading,
// and by HYST_V (the other way) before upgrading. Prevents flapping at the
// boundary when the pack sags under load and recovers.
static PackState classify(float v, PackState prev) {
  if (v < V_NO_PACK) return STATE_NO_PACK;

  // Use thresholds shifted by hysteresis depending on direction.
  switch (prev) {
    case STATE_NO_PACK:
      if (v >= V_NO_PACK + HYST_V) {
        if      (v >= V_OVER)     return STATE_HIGH;
        else if (v >= V_LOW)      return STATE_OK;
        else if (v >= V_CRITICAL) return STATE_LOW;
        else                      return STATE_CRITICAL;
      }
      return STATE_NO_PACK;

    case STATE_CRITICAL:
      if (v >= V_CRITICAL + HYST_V) return STATE_LOW;
      return STATE_CRITICAL;

    case STATE_LOW:
      if (v < V_CRITICAL - HYST_V) return STATE_CRITICAL;
      if (v >= V_LOW + HYST_V)     return STATE_OK;
      return STATE_LOW;

    case STATE_OK:
      if (v >= V_OVER)             return STATE_HIGH;
      if (v < V_LOW - HYST_V)      return STATE_LOW;
      return STATE_OK;

    case STATE_HIGH:
      if (v < V_OVER - HYST_V)     return STATE_OK;
      return STATE_HIGH;
  }
  return prev;
}

// ─── MA filter push ────────────────────────────────────────────────────────
static float ma_push(float v) {
  ma_buf[ma_idx] = v;
  ma_idx = (ma_idx + 1) % MA_WINDOW;
  if (ma_count < MA_WINDOW) ma_count++;
  float sum = 0.0f;
  for (uint8_t i = 0; i < ma_count; i++) sum += ma_buf[i];
  return sum / (float)ma_count;
}

// ─── I2C scan (emits a single I2C:scan=... line) ────────────────────────────
static void i2c_scan_emit() {
  String s = "I2C:scan=";
  uint8_t count = 0;
  for (uint8_t addr = 0x03; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      if (count > 0) s += ",";
      char buf[6];
      snprintf(buf, sizeof(buf), "0x%02X", addr);
      s += buf;
      count++;
    }
  }
  if (count == 0) s += "none";
  Serial.println(s);
}

// ─── Try to (re)init INA219 ────────────────────────────────────────────────
static bool init_ina219() {
  if (ina219.begin(&Wire)) {
    // 32V / 2A range — we don't use current readings but calibration is needed
    // for the library to compute getBusVoltage_V() correctly.
    ina219.setCalibration_32V_2A();
    return true;
  }
  return false;
}

// ─── Command handler ───────────────────────────────────────────────────────
static void emit_bat() {
  float pct = battery_pct_lfp4s(vbus_avg);
  Serial.printf("BAT:v=%.3f,vavg=%.3f,pct=%.1f,state=%s\n",
                vbus_last, vbus_avg, pct, state_name(state));
}

static void emit_status() {
  Serial.printf("META:fw=battery-mon-s3-v1.0,uptime=%lu,ina_ok=%d,ma=%d\n",
                (unsigned long)millis(), (int)ina_ok, (int)ma_count);
  emit_bat();
}

static void handle_cmd(const String& cmd) {
  if (cmd.length() == 0) return;
  if (cmd == "PING") {
    Serial.printf("PONG:t=%lu\n", (unsigned long)millis());
  } else if (cmd == "STATUS") {
    emit_status();
  } else if (cmd == "SCAN") {
    i2c_scan_emit();
  } else {
    Serial.printf("ERR:unknown_cmd=%s\n", cmd.c_str());
  }
}

static void poll_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      cmd_buf.trim();
      handle_cmd(cmd_buf);
      cmd_buf = "";
    } else {
      if (cmd_buf.length() < 64) cmd_buf += c;
      else cmd_buf = "";   // drop oversized junk
    }
  }
}

// ─── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  // USB-CDC needs a moment to enumerate; don't block forever.
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { delay(10); }

  Serial.println();
  Serial.printf("BOOT:battery-mon-s3-v1.0,sda=%d,scl=%d,addr=0x%02X\n",
                I2C_SDA_PIN, I2C_SCL_PIN, INA219_ADDR);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  delay(50);
  i2c_scan_emit();

  ina_ok = init_ina219();
  if (ina_ok) {
    Serial.printf("INA219:ok=1,addr=0x%02X\n", INA219_ADDR);
  } else {
    Serial.printf("INA219:ok=0,addr=0x%02X,reason=begin_failed\n", INA219_ADDR);
  }
}

// ─── Loop ──────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // 1) Always service commands first — keeps PING latency low.
  poll_serial();

  // 2) Retry INA219 if not initialised.
  if (!ina_ok && (now - t_last_retry >= RETRY_PERIOD_MS)) {
    t_last_retry = now;
    if (init_ina219()) {
      ina_ok = true;
      Serial.printf("INA219:recovered,addr=0x%02X\n", INA219_ADDR);
    }
  }

  // 3) Sample INA219 at 10 Hz.
  if (ina_ok && (now - t_last_sample >= SAMPLE_PERIOD_MS)) {
    t_last_sample = now;
    vbus_last = ina219.getBusVoltage_V();
    // INA219 can briefly return ridiculous values during I2C glitches; clamp.
    if (vbus_last < 0.0f)   vbus_last = 0.0f;
    if (vbus_last > 40.0f)  vbus_last = 40.0f;
    vbus_avg = ma_push(vbus_last);
    state = classify(vbus_avg, state);
  } else if (!ina_ok) {
    // No INA219 → keep state at NO_PACK so downstream sees the failure.
    state = STATE_NO_PACK;
  }

  // 4) Emit BAT telemetry at 1 Hz.
  if (now - t_last_emit >= EMIT_PERIOD_MS) {
    t_last_emit = now;
    if (ina_ok) emit_bat();
    else        Serial.println("BAT:v=0.000,vavg=0.000,pct=0.0,state=NO_PACK");
  }

  // 5) Heartbeat at 1 Hz (offset by half period so it never lines up with BAT).
  if (now - t_last_hb >= HB_PERIOD_MS) {
    t_last_hb = now;
    Serial.printf("HB:t=%lu\n", (unsigned long)now);
  }
}
