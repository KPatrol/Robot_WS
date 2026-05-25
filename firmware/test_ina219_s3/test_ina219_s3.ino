/*
 * ============================================================================
 * KPatrol — INA219 Battery Voltage Smoke Test (ESP32-S3, Vbus-only mode)
 * ============================================================================
 *
 * Purpose
 *   Standalone test for the INA219 module on the S3 peripheral bus, mirroring
 *   the exact pin / address / clock used by the production firmware
 *   (controller_four_motors_mecanum.ino). No motors, no WiFi, no watchdog —
 *   just confirm that the I2C wiring and Vbus-only configuration are correct
 *   and that battery percentage is computed cleanly for an LFP 4S pack.
 *
 * Wiring (Vbus-only mode, NO current measurement)
 *   INA219 VCC   → ESP32-S3 3V3
 *   INA219 GND   → ESP32-S3 GND  (also tied to LFP pack GND)
 *   INA219 SDA   → ESP32-S3 GPIO 19
 *   INA219 SCL   → ESP32-S3 GPIO 20
 *   INA219 VIN+  ──┐
 *                  ├──► both shorted to LFP+ (12V bus)   ← 0 A through shunt
 *   INA219 VIN−  ──┘
 *
 *   IMPORTANT: VIN+ and VIN− MUST be electrically tied together at the
 *   same point on LFP+. Do NOT put the shunt in series with the motor
 *   load — the 0.1Ω 2W on-module shunt will burn at >3 A.
 *
 * Battery percentage (LFP 4S, conservative empty/full thresholds)
 *   V_empty = 10.0 V   (≈ 2.50 V/cell, hard cutoff)
 *   V_full  = 14.4 V   (≈ 3.60 V/cell, fully charged at rest)
 *   pct = clamp((Vbus − V_empty) / (V_full − V_empty), 0..1) × 100
 *
 * Arduino IDE settings
 *   Board:   "ESP32S3 Dev Module"
 *   USB CDC On Boot: Enabled   (so Serial works over the native USB port)
 *   Upload Speed: 921600
 *   Flash Size: 4MB / 8MB depending on your module (4MB is safe default)
 *   Baud:    115200
 *   Library: "Adafruit INA219" by Adafruit (Library Manager)
 *
 * Expected serial output on a healthy bench setup (12.6 V LFP at rest):
 *   BOOT: ina219-smoke-v1.0
 *   I2C scan: 0x40
 *   INA219 OK @ 0x40
 *   Vbus=12.610 V  Vshunt= 0.012 mV  I=  0.12 mA   →  pct= 59.3 %  [OK]
 *
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_INA219.h>

// --- Pin / bus config — keep IN SYNC with controller_four_motors_mecanum.ino
#define I2C_SDA_PIN   19
#define I2C_SCL_PIN   20
#define I2C_FREQ_HZ   100000UL   // 100 kHz — INA219 supports up to 1 MHz
#define INA219_ADDR   0x40       // A0=A1=GND (default)

// --- LFP 4S pack thresholds (volts)
static constexpr float V_EMPTY = 10.0f;
static constexpr float V_FULL  = 14.4f;

// --- Sample period (ms)
static constexpr uint32_t PRINT_PERIOD_MS = 500;

Adafruit_INA219 ina219(INA219_ADDR);
bool ina_ok = false;
uint32_t t_last_print = 0;

static float battery_pct_lfp4s(float vbus) {
  const float span = V_FULL - V_EMPTY;
  if (span <= 0.0f) return 0.0f;
  float p = (vbus - V_EMPTY) / span;
  if (p < 0.0f) p = 0.0f;
  if (p > 1.0f) p = 1.0f;
  return p * 100.0f;
}

static void i2c_scan() {
  Serial.print("I2C scan:");
  uint8_t count = 0;
  for (uint8_t addr = 0x03; addr < 0x78; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      count++;
    }
  }
  if (count == 0) Serial.print(" (none — check SDA/SCL/GND/VCC)");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  // Give USB-CDC a moment to enumerate on Mac
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0) < 2000) { delay(10); }

  Serial.println();
  Serial.println("BOOT: ina219-smoke-v1.0");
  Serial.printf("I2C: SDA=GPIO%d  SCL=GPIO%d  @ %lu Hz  addr=0x%02X\n",
                I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ, INA219_ADDR);

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
  delay(50);

  i2c_scan();

  if (ina219.begin(&Wire)) {
    // Default calibration = 32V / 2A range. Vbus-only mode means the
    // 0.1Ω shunt sees ~0 A so reported current will hover near 0; we only
    // care about getBusVoltage_V().
    ina219.setCalibration_32V_2A();
    ina_ok = true;
    Serial.printf("INA219 OK @ 0x%02X\n", INA219_ADDR);
  } else {
    Serial.printf("INA219 FAIL @ 0x%02X — check wiring (VCC=3V3, GND, SDA=19, SCL=20)\n",
                  INA219_ADDR);
  }
  Serial.println();
}

void loop() {
  uint32_t now = millis();
  if (now - t_last_print < PRINT_PERIOD_MS) return;
  t_last_print = now;

  if (!ina_ok) {
    // Periodically retry — useful when you plug power after upload
    Serial.print("INA219 still not responding. ");
    i2c_scan();
    if (ina219.begin(&Wire)) {
      ina219.setCalibration_32V_2A();
      ina_ok = true;
      Serial.println("INA219 recovered.");
    }
    return;
  }

  float vshunt_mV = ina219.getShuntVoltage_mV();
  float vbus_V    = ina219.getBusVoltage_V();
  float current_mA = ina219.getCurrent_mA();
  float pct = battery_pct_lfp4s(vbus_V);

  // Plausibility check: LFP 4S at rest should be 10.0 .. 14.6 V.
  // USB-only power on the bench (no 12V applied) will report ~0 V.
  const char* status;
  if (vbus_V < 1.0f)        status = "NO-PACK";   // not connected
  else if (vbus_V < 9.5f)   status = "LOW";       // suspiciously low / wrong pack
  else if (vbus_V > 15.5f)  status = "HIGH";      // wrong pack / wrong wiring
  else                      status = "OK";

  Serial.printf("Vbus=%6.3f V  Vshunt=%6.3f mV  I=%7.2f mA  →  pct=%5.1f %%  [%s]\n",
                vbus_V, vshunt_mV, current_mA, pct, status);
}
