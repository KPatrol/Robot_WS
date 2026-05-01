/*
 * ============================================================================
 * KPatrol Robot - Four Motors Mecanum Wheel Control + Light Relay
 * ============================================================================
 * 
 * Hardware:
 *   - ESP32-S3 WROOM-1 Dev Board
 *   - 4x BTS7960 (IBT-2) Motor Drivers
 *   - 4x JGA25-370 12V DC Motors with Encoders
 *   - 1x 12VDC-SL-C Relay for Light Control
 *   - 12V Power Supply (7A minimum, 10A+ recommended)
 * 
 * Date: February 19, 2026
 * Status: Full Mecanum Configuration - 4 Motors + Light Relay
 * 
 * GPIO Allocation:
 *   Front-Right (FR):
 *     GPIO 4  → FR_RPWM  (PWM Ch 0 - Forward)
 *     GPIO 5  → FR_LPWM  (PWM Ch 1 - Backward)
 *     GPIO 6  → FR_R_EN  (Enable)
 *     GPIO 7  → FR_L_EN  (Enable)
 * 
 *   Front-Left (FL):
 *     GPIO 15 → FL_RPWM  (PWM Ch 2 - Forward)
 *     GPIO 16 → FL_LPWM  (PWM Ch 3 - Backward)
 *     GPIO 17 → FL_R_EN  (Enable)
 *     GPIO 18 → FL_L_EN  (Enable)
 * 
 *   Back-Right (BR):
 *     GPIO 8  → BR_RPWM  (PWM Ch 4 - Forward)
 *     GPIO 9  → BR_LPWM  (PWM Ch 5 - Backward)
 *     GPIO 10 → BR_R_EN  (Enable)
 *     GPIO 11 → BR_L_EN  (Enable)
 * 
 *   Back-Left (BL):
 *     GPIO 12 → BL_RPWM  (PWM Ch 6 - Forward)
 *     GPIO 13 → BL_LPWM  (PWM Ch 7 - Backward)
 *     GPIO 14 → BL_R_EN  (Enable)
 *     GPIO 21 → BL_L_EN  (Enable)
 * 
 *   Light Relays:
 *     GPIO 38 → RELAY_WARNING_LIGHT (Đèn cảnh báo - Warning Light)
 *     GPIO 39 → RELAY_MAIN_LIGHT (Đèn chính - Main Light)
 *
 *   BNO08x IMU (UART Mode - PS1=HIGH, PS0=LOW):
 *     GPIO 35 → BNO08x TX (SDA pin) - ESP32 RX
 *     GPIO 36 → BNO08x RX (SCL pin) - ESP32 TX
 *     GPIO 37 → BNO08x RST
 *
 *   GPS NEO-6M (V5.2 outdoor mode, UART2 - shared serial pass-through):
 *     GPIO 41 → GPS TX  - ESP32 RX (NMEA in)
 *     GPIO 42 → GPS RX  - ESP32 TX (UBX out, optional)
 *     5V/GND → NEO-6M VCC/GND
 *
 * PWM Configuration:
 *   - Frequency: 5000 Hz (5 kHz)
 *   - Resolution: 8-bit (0-255)
 *   - Library: LEDC (ESP32 built-in)
 * 
 * Serial Commands (115200 baud):
 *   Individual Motors:
 *     FR_F, FR_B, FR_S : Front-Right Forward/Backward/Stop
 *     FL_F, FL_B, FL_S : Front-Left Forward/Backward/Stop
 *     BR_F, BR_B, BR_S : Back-Right Forward/Backward/Stop
 *     BL_F, BL_B, BL_S : Back-Left Forward/Backward/Stop
 * 
 *   Mecanum Movement:
 *     F       : Forward (all wheels same direction)
 *     B       : Backward (all wheels reverse)
 *     SR      : Strafe Right (diagonal wheels)
 *     SL      : Strafe Left (diagonal wheels)
 *     R       : Rotate Right (left forward, right backward)
 *     L       : Rotate Left (right forward, left backward)
 *     DR      : Diagonal Forward-Right
 *     DL      : Diagonal Forward-Left
 *     S       : Stop all motors
 * 
 *   Light Control:
 *     LIGHT_ON  : Turn on warning light (Đèn cảnh báo)
 *     LIGHT_OFF : Turn off warning light
 *     LIGHT_T   : Toggle warning light state
 *     MAIN_ON   : Turn on main light (Đèn chính)
 *     MAIN_OFF  : Turn off main light
 *     MAIN_T    : Toggle main light state
 *     LIGHT     : Show all light states
 *
 *   IMU (BNO08x):
 *     IMU       : Read current IMU orientation (YAW, PITCH, ROLL)
 *     IMU_CAL   : Start magnetometer calibration
 *
 *   Speed & System:
 *     SPD:xxx           : Set speed (0-255)
 *     MEC:vx,vy,wz,spd  : Mecanum blended drive (vx/vy/wz ∈ [-127,127], spd 0-255)
 *     T       : Run test sequence
 *     I       : Show system info
 *     H       : Help menu
 * 
 * Arduino IDE Settings:
 *   - Board: "ESP32S3 Dev Module"
 *   - USB CDC On Boot: "Disabled" (CRITICAL!)
 *   - Upload Mode: "UART0 / Hardware CDC"
 *   - Upload Speed: 921600
 *   - Baud: 115200
 * 
 * ============================================================================
 */

#include <Adafruit_BNO08x.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

// ----------------------------------------------------------------------------
// Forward declarations (Arduino IDE auto-prototyping fix — must come before
// any function that takes these enum types as parameters)
// ----------------------------------------------------------------------------
enum BuzzerPattern : uint8_t;
enum LightPattern  : uint8_t;
void buzzer_start_pattern(BuzzerPattern p);
void light_pattern_set(LightPattern p);
void IRAM_ATTR estopISR();

// ============================================================================
// FRONT-RIGHT MOTOR (FR) - BTS7960 #1
// ============================================================================
#define FR_RPWM     4    // GPIO 4  - Forward PWM
#define FR_LPWM     5    // GPIO 5  - Backward PWM
#define FR_R_EN     6    // GPIO 6  - Enable Forward
#define FR_L_EN     7    // GPIO 7  - Enable Backward

#define FR_PWM_CH_R 0    // PWM Channel 0 for FR_RPWM
#define FR_PWM_CH_L 1    // PWM Channel 1 for FR_LPWM

// ============================================================================
// FRONT-LEFT MOTOR (FL) - BTS7960 #2
// ============================================================================
#define FL_RPWM     15   // GPIO 15 - Forward PWM
#define FL_LPWM     16   // GPIO 16 - Backward PWM
#define FL_R_EN     17   // GPIO 17 - Enable Forward
#define FL_L_EN     18   // GPIO 18 - Enable Backward

#define FL_PWM_CH_R 2    // PWM Channel 2 for FL_RPWM
#define FL_PWM_CH_L 3    // PWM Channel 3 for FL_LPWM

// ============================================================================
// BACK-RIGHT MOTOR (BR) - BTS7960 #3
// ============================================================================
#define BR_RPWM     8    // GPIO 8  - Forward PWM
#define BR_LPWM     9    // GPIO 9  - Backward PWM
#define BR_R_EN     10   // GPIO 10 - Enable Forward
#define BR_L_EN     11   // GPIO 11 - Enable Backward

#define BR_PWM_CH_R 4    // PWM Channel 4 for BR_RPWM
#define BR_PWM_CH_L 5    // PWM Channel 5 for BR_LPWM

// ============================================================================
// BACK-LEFT MOTOR (BL) - BTS7960 #4
// ============================================================================
#define BL_RPWM     12   // GPIO 12 - Forward PWM
#define BL_LPWM     13   // GPIO 13 - Backward PWM
#define BL_R_EN     14   // GPIO 14 - Enable Forward
#define BL_L_EN     21   // GPIO 21 - Enable Backward

#define BL_PWM_CH_R 6    // PWM Channel 6 for BL_RPWM
#define BL_PWM_CH_L 7    // PWM Channel 7 for BL_LPWM

// ============================================================================
// PWM CONFIGURATION
// ============================================================================
#define PWM_FREQ    5000 // 5 kHz frequency
#define PWM_RES     8    // 8-bit resolution (0-255)

// ---------------------------------------------------------------------------
// LEDC compat shim for ESP32 Arduino core 3.x
// Core 3.x removed ledcSetup / ledcAttachPin and made ledcWrite take a PIN
// instead of a channel. These wrappers keep the legacy channel-based call
// sites working by remembering pin<->channel mapping.
// ---------------------------------------------------------------------------
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  static uint8_t  _ledc_ch2pin[16]  = {0};
  static uint32_t _ledc_ch_freq[16] = {0};
  static uint8_t  _ledc_ch_res[16]  = {0};
  static inline void _ledcSetupCompat(uint8_t ch, uint32_t freq, uint8_t res) {
    if (ch < 16) { _ledc_ch_freq[ch] = freq; _ledc_ch_res[ch] = res; }
  }
  static inline void _ledcAttachPinCompat(uint8_t pin, uint8_t ch) {
    if (ch < 16) {
      _ledc_ch2pin[ch] = pin;
      ledcAttach(pin, _ledc_ch_freq[ch], _ledc_ch_res[ch]);
    }
  }
  static inline void _ledcWriteCompat(uint8_t ch, uint32_t duty) {
    if (ch < 16) ledcWrite(_ledc_ch2pin[ch], duty);
  }
  #define ledcSetup(ch, freq, res) _ledcSetupCompat((ch), (freq), (res))
  #define ledcAttachPin(pin, ch)   _ledcAttachPinCompat((pin), (ch))
  #define ledcWrite(ch, duty)      _ledcWriteCompat((ch), (duty))
#endif

// ============================================================================
// LIGHT RELAY CONFIGURATION (12VDC-SL-C)
// ============================================================================
#define RELAY_WARNING_LIGHT_PIN  38   // GPIO 38 - Warning Light (Đèn cảnh báo)
#define RELAY_MAIN_LIGHT_PIN     39   // GPIO 39 - Main Light (Đèn chính)
#define RELAY_ACTIVE_HIGH true        // Set false if relay activates on LOW

// ============================================================================
// BNO08x IMU CONFIGURATION (UART Mode - PS1=HIGH, PS0=LOW)
// ============================================================================
#define BNO08X_RX   35   // GPIO 35 - ESP32 RX ← BNO08x TX (SDA pin)
#define BNO08X_TX   36   // GPIO 36 - ESP32 TX → BNO08x RX (SCL pin)
#define BNO08X_RST  37   // GPIO 37 - Reset pin
#define IMU_REPORT_INTERVAL_US 50000  // 20Hz (50ms) for stability

// ============================================================================
// GPS NEO-6M CONFIGURATION (V5.2 — UART2, Core 0 task, NMEA pass-through)
// Wires: NEO-6M TX → GPIO 41, NEO-6M RX → GPIO 42, VCC=5V, GND=GND
// Forwarded to Pi on USB UART0 with "NMEA:" prefix; gps_reader.py parses it.
// ============================================================================
#define GPS_RX_PIN       41        // ESP32 RX2 ← NEO-6M TX
#define GPS_TX_PIN       42        // ESP32 TX2 → NEO-6M RX (optional, for UBX)
#define GPS_BAUD         9600
#define GPS_LINE_MAX     120       // NMEA sentence max safe length
#define GPS_TASK_CORE    0         // motor PID lives on Core 1 — keep apart

// ============================================================================
// SAFETY HARDWARE (V10.1) — Buzzer, Physical E-Stop, Remote-Control Relay
// Pinout chosen from free ESP32-S3-WROOM-1 GPIOs after motors/IMU/GPS/relays.
//   BUZZER_PIN       — drives an active 5V piezo buzzer (HIGH=tone)
//   ESTOP_PIN        — physical NC button to GND with INPUT_PULLUP + FALLING ISR
//   REMOTE_RELAY_PIN — placeholder for a remote-toggle relay that physically
//                      cuts motor power; controlled by command REMOTE:1 / REMOTE:0
// ============================================================================
#define BUZZER_PIN                  40   // free GPIO, no PWM needed for active buzzer
#define ESTOP_PIN                    1   // free GPIO, supports interrupts
#define REMOTE_RELAY_PIN             2   // free GPIO, drives relay coil/input
#define BUZZER_ACTIVE_HIGH        true   // set false if your buzzer is active-low
#define REMOTE_RELAY_ACTIVE_HIGH  true   // mirrors RELAY_ACTIVE_HIGH convention
#define ESTOP_DEBOUNCE_MS           30   // ignore re-triggers within this window

// ============================================================================
// MOTOR DIRECTION INVERSION
// Set to true to invert motor direction if wired backward
// ============================================================================
#define FR_INVERTED false  // Set true if FR motor rotates opposite
#define FL_INVERTED true   // Set true if FL motor rotates opposite
#define BR_INVERTED false  // Set true if BR motor rotates opposite
#define BL_INVERTED true   // Set true if BL motor rotates opposite (assume same as FL)

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
int currentSpeed = 150;  // Default speed (0-255)
unsigned long lastCommandTime = 0;
bool warningLightState = false; // Warning light relay state (Đèn cảnh báo)
bool mainLightState = false;    // Main light relay state (Đèn chính)

// ============================================================================
// WATCHDOG + OTA GLOBALS
// Watchdog: auto-stop motors if Pi stops sending commands for >3s.
// OTA: firmware update via WiFi (ArduinoOTA), enabled on demand to save RAM.
// ============================================================================
#define WATCHDOG_TIMEOUT_MS 3000UL      // stop motors if no command for this long
bool watchdogEnabled    = true;
bool watchdogTriggered  = false;
bool otaEnabled         = false;
char otaSsid[33]        = {0};
char otaPass[65]        = {0};
char otaHost[32]        = "kpatrol-esp32";

// ============================================================================
// SAFETY HARDWARE GLOBAL STATE (V10.1)
// E-stop is set by physical-button ISR; cleared by ESTOP_CLEAR command.
// While estopTriggered OR remoteRelayCutoff is true, motion commands are
// rejected at the parser top so a runaway Pi cannot bypass the safety lock.
// ============================================================================
volatile bool estopTriggered = false;
volatile uint32_t estopLastIsrMs = 0;
bool buzzerState = false;
bool remoteRelayCutoff = false;

enum BuzzerPattern : uint8_t { BUZZ_OFF, BUZZ_BEEP, BUZZ_ALARM, BUZZ_SOS };
BuzzerPattern buzzerPattern = BUZZ_OFF;
uint32_t buzzerPatternStartMs = 0;

enum LightPattern : uint8_t {
  LIGHTP_OFF,           // no pattern; manual control
  LIGHTP_WARN_BLINK,    // warning light slow blink (1Hz)
  LIGHTP_WARN_STROBE,   // warning light fast strobe (5Hz)
  LIGHTP_BOTH_BLINK,    // warning + main alternating (1Hz)
  LIGHTP_SOS            // SOS Morse on warning light (3s cycle)
};
LightPattern lightPattern = LIGHTP_OFF;
uint32_t lightPatternStartMs = 0;

// ============================================================================
// BNO08x IMU GLOBAL VARIABLES
// ============================================================================
Adafruit_BNO08x bno08x(BNO08X_RST);
sh2_SensorValue_t sensorValue;
bool imuInitialized = false;

// IMU Euler angles (continuously updated)
float imuYaw = 0, imuPitch = 0, imuRoll = 0;
float imuAccuracy = 0;
unsigned long lastIMUUpdate = 0;

// ============================================================================
// GPS NEO-6M GLOBAL STATE
// gpsEnabled is toggled at boot — if Serial2 init fails the task self-disables.
// gpsTaskHandle lets us inspect or recreate the task if needed.
// ============================================================================
HardwareSerial GPSSerial(2);              // UART2 — shares no pins with motor PWM
volatile bool gpsEnabled = false;
TaskHandle_t gpsTaskHandle = nullptr;
volatile uint32_t gpsLinesForwarded = 0;
volatile uint32_t gpsBytesRead = 0;       // total raw bytes from UART2 (debug)

// ============================================================================
// GPS TASK (Core 0) — read NMEA sentences from UART2 and forward to Pi
// Pi-side gps_reader.py recognises the "NMEA:" prefix and parses with pynmea2.
// We do NOT validate the checksum here; pynmea2 handles that on the Pi.
// ============================================================================
void gpsTask(void *param) {
  static char buf[GPS_LINE_MAX + 1];
  size_t idx = 0;
  uint32_t lastHbMs = 0;
  for (;;) {
    while (GPSSerial.available()) {
      char c = (char)GPSSerial.read();
      gpsBytesRead++;
      if (c == '\n') {
        buf[idx] = '\0';
        if (idx >= 6 && buf[0] == '$') {
          // Forward complete sentence to Pi (USB UART0)
          Serial.print("NMEA:");
          Serial.println(buf);
          gpsLinesForwarded++;
        }
        idx = 0;
      } else if (c != '\r') {
        if (idx < GPS_LINE_MAX) {
          buf[idx++] = c;
        } else {
          idx = 0;  // overflow — drop and resync on next '\n'
        }
      }
    }
    // Heartbeat every 5s — Pi-side parser ignores anything not starting with NMEA:.
    // bytes=0 → no UART2 data (power/wiring/baud); bytes>0 lines=0 → framing/baud.
    uint32_t nowMs = millis();
    if (nowMs - lastHbMs >= 5000) {
      lastHbMs = nowMs;
      Serial.printf("GPS_HB:bytes=%u lines=%u avail=%d\n",
                    (unsigned)gpsBytesRead, (unsigned)gpsLinesForwarded,
                    (int)GPSSerial.available());
    }
    vTaskDelay(pdMS_TO_TICKS(10));  // 100Hz poll, NMEA at 9600 ≈ 1KB/s
  }
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Serial initialization (UART mode - USB CDC must be OFF)
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("\n\n");
  Serial.println("========================================");
  Serial.println("  KPatrol - Mecanum Wheel Control");
  Serial.println("  4 Motors (FR + FL + BR + BL)");
  Serial.println("========================================");
  Serial.println("Initializing motors...");
  
  // ===== FRONT-RIGHT MOTOR SETUP =====
  ledcSetup(FR_PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(FR_PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(FR_RPWM, FR_PWM_CH_R);
  ledcAttachPin(FR_LPWM, FR_PWM_CH_L);
  pinMode(FR_R_EN, OUTPUT);
  pinMode(FR_L_EN, OUTPUT);
  digitalWrite(FR_R_EN, HIGH);
  digitalWrite(FR_L_EN, HIGH);
  ledcWrite(FR_PWM_CH_R, 0);
  ledcWrite(FR_PWM_CH_L, 0);
  Serial.println("✓ Front-Right motor configured");
  
  // ===== FRONT-LEFT MOTOR SETUP =====
  ledcSetup(FL_PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(FL_PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(FL_RPWM, FL_PWM_CH_R);
  ledcAttachPin(FL_LPWM, FL_PWM_CH_L);
  pinMode(FL_R_EN, OUTPUT);
  pinMode(FL_L_EN, OUTPUT);
  digitalWrite(FL_R_EN, HIGH);
  digitalWrite(FL_L_EN, HIGH);
  ledcWrite(FL_PWM_CH_R, 0);
  ledcWrite(FL_PWM_CH_L, 0);
  Serial.println("✓ Front-Left motor configured");
  
  // ===== BACK-RIGHT MOTOR SETUP =====
  ledcSetup(BR_PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(BR_PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(BR_RPWM, BR_PWM_CH_R);
  ledcAttachPin(BR_LPWM, BR_PWM_CH_L);
  pinMode(BR_R_EN, OUTPUT);
  pinMode(BR_L_EN, OUTPUT);
  digitalWrite(BR_R_EN, HIGH);
  digitalWrite(BR_L_EN, HIGH);
  ledcWrite(BR_PWM_CH_R, 0);
  ledcWrite(BR_PWM_CH_L, 0);
  Serial.println("✓ Back-Right motor configured");
  
  // ===== BACK-LEFT MOTOR SETUP =====
  ledcSetup(BL_PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(BL_PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcAttachPin(BL_RPWM, BL_PWM_CH_R);
  ledcAttachPin(BL_LPWM, BL_PWM_CH_L);
  pinMode(BL_R_EN, OUTPUT);
  pinMode(BL_L_EN, OUTPUT);
  digitalWrite(BL_R_EN, HIGH);
  digitalWrite(BL_L_EN, HIGH);
  ledcWrite(BL_PWM_CH_R, 0);
  ledcWrite(BL_PWM_CH_L, 0);
  Serial.println("✓ Back-Left motor configured");
  
  // ===== LIGHT RELAY SETUP =====
  pinMode(RELAY_WARNING_LIGHT_PIN, OUTPUT);
  pinMode(RELAY_MAIN_LIGHT_PIN, OUTPUT);
  warning_light_off();  // Start with warning light OFF
  main_light_off();     // Start with main light OFF
  Serial.println("✓ Warning Light relay configured (GPIO 38)");
  Serial.println("✓ Main Light relay configured (GPIO 39)");

  // ===== SAFETY HARDWARE SETUP (V10.1) =====
  pinMode(BUZZER_PIN, OUTPUT);
  buzzer_set(false);
  pinMode(REMOTE_RELAY_PIN, OUTPUT);
  remote_relay_cutoff_set(false);
  pinMode(ESTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estopISR, FALLING);
  Serial.printf("✓ Buzzer GPIO%d, Remote-relay GPIO%d, E-Stop GPIO%d (FALLING ISR)\n",
                BUZZER_PIN, REMOTE_RELAY_PIN, ESTOP_PIN);
  
  // ===== BNO08x IMU SETUP (UART Mode) =====
  Serial.println("\nInitializing BNO08x IMU...");
  Serial1.begin(3000000, SERIAL_8N1, BNO08X_RX, BNO08X_TX);
  
  if (bno08x.begin_UART(&Serial1)) {
    imuInitialized = true;
    Serial.println("✓ BNO08x IMU initialized (UART mode)");
    
    // Enable rotation vector report
    if (bno08x.enableReport(SH2_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US)) {
      Serial.println("  Rotation Vector: OK (20Hz)");
    } else {
      Serial.println("  Rotation Vector: FAILED");
    }
  } else {
    imuInitialized = false;
    Serial.println("✗ BNO08x not found! Check wiring:");
    Serial.println("    VIN → 3.3V");
    Serial.println("    GND → GND");
    Serial.println("    SDA → GPIO 35");
    Serial.println("    SCL → GPIO 36");
    Serial.println("    RST → GPIO 37 (or 3.3V)");
  }

  // ===== GPS NEO-6M SETUP (UART2 on Core 0) =====
  Serial.println("\nInitializing GPS NEO-6M (UART2)...");
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(50);
  gpsEnabled = true;
  BaseType_t taskOk = xTaskCreatePinnedToCore(
      gpsTask, "gpsTask", 4096, NULL, 1, &gpsTaskHandle, GPS_TASK_CORE);
  if (taskOk == pdPASS) {
    Serial.printf("✓ GPS UART2 ready (RX=GPIO%d, TX=GPIO%d, %d baud, Core %d)\n",
                  GPS_RX_PIN, GPS_TX_PIN, GPS_BAUD, GPS_TASK_CORE);
    Serial.println("  Forwarding NMEA sentences to Pi with prefix 'NMEA:'");
  } else {
    gpsEnabled = false;
    Serial.println("✗ GPS task creation FAILED — outdoor mode disabled");
  }

  // System ready
  Serial.println("\n✓ Mecanum system ready!");
  Serial.println("----------------------------------------");
  printHelp();
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

// ===== FRONT-RIGHT MOTOR FUNCTIONS =====
void FR_forward(int speed) {
  if (FR_INVERTED) {
    ledcWrite(FR_PWM_CH_R, 0);
    ledcWrite(FR_PWM_CH_L, speed);
  } else {
    ledcWrite(FR_PWM_CH_R, speed);
    ledcWrite(FR_PWM_CH_L, 0);
  }
}

void FR_backward(int speed) {
  if (FR_INVERTED) {
    ledcWrite(FR_PWM_CH_R, speed);
    ledcWrite(FR_PWM_CH_L, 0);
  } else {
    ledcWrite(FR_PWM_CH_R, 0);
    ledcWrite(FR_PWM_CH_L, speed);
  }
}

void FR_stop() {
  ledcWrite(FR_PWM_CH_R, 0);
  ledcWrite(FR_PWM_CH_L, 0);
}

// ===== FRONT-LEFT MOTOR FUNCTIONS =====
void FL_forward(int speed) {
  if (FL_INVERTED) {
    ledcWrite(FL_PWM_CH_R, 0);
    ledcWrite(FL_PWM_CH_L, speed);
  } else {
    ledcWrite(FL_PWM_CH_R, speed);
    ledcWrite(FL_PWM_CH_L, 0);
  }
}

void FL_backward(int speed) {
  if (FL_INVERTED) {
    ledcWrite(FL_PWM_CH_R, speed);
    ledcWrite(FL_PWM_CH_L, 0);
  } else {
    ledcWrite(FL_PWM_CH_R, 0);
    ledcWrite(FL_PWM_CH_L, speed);
  }
}

void FL_stop() {
  ledcWrite(FL_PWM_CH_R, 0);
  ledcWrite(FL_PWM_CH_L, 0);
}

// ===== BACK-RIGHT MOTOR FUNCTIONS =====
void BR_forward(int speed) {
  if (BR_INVERTED) {
    ledcWrite(BR_PWM_CH_R, 0);
    ledcWrite(BR_PWM_CH_L, speed);
  } else {
    ledcWrite(BR_PWM_CH_R, speed);
    ledcWrite(BR_PWM_CH_L, 0);
  }
}

void BR_backward(int speed) {
  if (BR_INVERTED) {
    ledcWrite(BR_PWM_CH_R, speed);
    ledcWrite(BR_PWM_CH_L, 0);
  } else {
    ledcWrite(BR_PWM_CH_R, 0);
    ledcWrite(BR_PWM_CH_L, speed);
  }
}

void BR_stop() {
  ledcWrite(BR_PWM_CH_R, 0);
  ledcWrite(BR_PWM_CH_L, 0);
}

// ===== BACK-LEFT MOTOR FUNCTIONS =====
void BL_forward(int speed) {
  if (BL_INVERTED) {
    ledcWrite(BL_PWM_CH_R, 0);
    ledcWrite(BL_PWM_CH_L, speed);
  } else {
    ledcWrite(BL_PWM_CH_R, speed);
    ledcWrite(BL_PWM_CH_L, 0);
  }
}

void BL_backward(int speed) {
  if (BL_INVERTED) {
    ledcWrite(BL_PWM_CH_R, speed);
    ledcWrite(BL_PWM_CH_L, 0);
  } else {
    ledcWrite(BL_PWM_CH_R, 0);
    ledcWrite(BL_PWM_CH_L, speed);
  }
}

void BL_stop() {
  ledcWrite(BL_PWM_CH_R, 0);
  ledcWrite(BL_PWM_CH_L, 0);
}

// ============================================================================
// MECANUM WHEEL MOVEMENT FUNCTIONS
// ============================================================================

void mecanum_forward(int speed) {
  Serial.println("\n>>> MECANUM: FORWARD <<<");
  FR_forward(speed);
  FL_forward(speed);
  BR_forward(speed);
  BL_forward(speed);
}

void mecanum_backward(int speed) {
  Serial.println("\n>>> MECANUM: BACKWARD <<<");
  FR_backward(speed);
  FL_backward(speed);
  BR_backward(speed);
  BL_backward(speed);
}

void mecanum_strafe_right(int speed) {
  Serial.println("\n>>> MECANUM: STRAFE RIGHT <<<");
  FR_backward(speed);  // Front-Right backward
  FL_forward(speed);   // Front-Left forward
  BR_forward(speed);   // Back-Right forward
  BL_backward(speed);  // Back-Left backward
}

void mecanum_strafe_left(int speed) {
  Serial.println("\n>>> MECANUM: STRAFE LEFT <<<");
  FR_forward(speed);   // Front-Right forward
  FL_backward(speed);  // Front-Left backward
  BR_backward(speed);  // Back-Right backward
  BL_forward(speed);   // Back-Left forward
}

void mecanum_rotate_right(int speed) {
  Serial.println("\n>>> MECANUM: ROTATE RIGHT <<<");
  FR_backward(speed);  // Right side backward
  FL_forward(speed);   // Left side forward
  BR_backward(speed);  // Right side backward
  BL_forward(speed);   // Left side forward
}

void mecanum_rotate_left(int speed) {
  Serial.println("\n>>> MECANUM: ROTATE LEFT <<<");
  FR_forward(speed);   // Right side forward
  FL_backward(speed);  // Left side backward
  BR_forward(speed);   // Right side forward
  BL_backward(speed);  // Left side backward
}

void mecanum_diagonal_forward_right(int speed) {
  Serial.println("\n>>> MECANUM: DIAGONAL FORWARD-RIGHT <<<");
  FR_stop();           // Front-Right stationary
  FL_forward(speed);   // Front-Left forward
  BR_forward(speed);   // Back-Right forward
  BL_stop();           // Back-Left stationary
}

void mecanum_diagonal_forward_left(int speed) {
  Serial.println("\n>>> MECANUM: DIAGONAL FORWARD-LEFT <<<");
  FR_forward(speed);   // Front-Right forward
  FL_stop();           // Front-Left stationary
  BR_stop();           // Back-Right stationary
  BL_forward(speed);   // Back-Left forward
}

void mecanum_stop() {
  Serial.println("\n>>> MECANUM: STOP ALL <<<");
  FR_stop();
  FL_stop();
  BR_stop();
  BL_stop();
}

// Active brake: both PWM channels HIGH simultaneously → BTS7960 short-brake.
// Stops in ~50 ms instead of ~200 ms coast.  brake_pwm 60–100 is a safe range.
void mecanum_active_brake(int brake_pwm = 80) {
  ledcWrite(FR_PWM_CH_R, brake_pwm); ledcWrite(FR_PWM_CH_L, brake_pwm);
  ledcWrite(FL_PWM_CH_R, brake_pwm); ledcWrite(FL_PWM_CH_L, brake_pwm);
  ledcWrite(BR_PWM_CH_R, brake_pwm); ledcWrite(BR_PWM_CH_L, brake_pwm);
  ledcWrite(BL_PWM_CH_R, brake_pwm); ledcWrite(BL_PWM_CH_L, brake_pwm);
  delay(40);
  FR_stop(); FL_stop(); BR_stop(); BL_stop();
  Serial.println(">>> MECANUM: ACTIVE BRAKE <<<");
}

// ============================================================================
// LIGHT RELAY CONTROL FUNCTIONS
// ============================================================================

// ===== WARNING LIGHT (Đèn cảnh báo) - GPIO 38 =====
void warning_light_on() {
  warningLightState = true;
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(RELAY_WARNING_LIGHT_PIN, HIGH);
  } else {
    digitalWrite(RELAY_WARNING_LIGHT_PIN, LOW);
  }
  Serial.println("⚠️ Warning Light: ON");
}

void warning_light_off() {
  warningLightState = false;
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(RELAY_WARNING_LIGHT_PIN, LOW);
  } else {
    digitalWrite(RELAY_WARNING_LIGHT_PIN, HIGH);
  }
  Serial.println("⚠️ Warning Light: OFF");
}

void warning_light_toggle() {
  if (warningLightState) {
    warning_light_off();
  } else {
    warning_light_on();
  }
}

// ===== MAIN LIGHT (Đèn chính) - GPIO 39 =====
void main_light_on() {
  mainLightState = true;
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(RELAY_MAIN_LIGHT_PIN, HIGH);
  } else {
    digitalWrite(RELAY_MAIN_LIGHT_PIN, LOW);
  }
  Serial.println("💡 Main Light: ON");
}

void main_light_off() {
  mainLightState = false;
  if (RELAY_ACTIVE_HIGH) {
    digitalWrite(RELAY_MAIN_LIGHT_PIN, LOW);
  } else {
    digitalWrite(RELAY_MAIN_LIGHT_PIN, HIGH);
  }
  Serial.println("💡 Main Light: OFF");
}

void main_light_toggle() {
  if (mainLightState) {
    main_light_off();
  } else {
    main_light_on();
  }
}

// ===== LIGHT STATUS =====
void light_status() {
  Serial.print("⚠️ Warning Light: ");
  Serial.println(warningLightState ? "ON" : "OFF");
  Serial.print("💡 Main Light: ");
  Serial.println(mainLightState ? "ON" : "OFF");
}

// Legacy support - map to warning light
void light_on() { warning_light_on(); }
void light_off() { warning_light_off(); }
void light_toggle() { warning_light_toggle(); }

// ============================================================================
// BUZZER CONTROL (V10.1) — active 5V piezo on BUZZER_PIN
// Patterns are non-blocking; tickBuzzerPattern() is called every loop().
// ============================================================================
void buzzer_set(bool on) {
  buzzerState = on;
  bool level = BUZZER_ACTIVE_HIGH ? on : !on;
  digitalWrite(BUZZER_PIN, level ? HIGH : LOW);
}
void buzzer_on()  { buzzer_set(true);  buzzerPattern = BUZZ_OFF; }
void buzzer_off() { buzzer_set(false); buzzerPattern = BUZZ_OFF; }
void buzzer_start_pattern(BuzzerPattern p) {
  buzzerPattern = p;
  buzzerPatternStartMs = millis();
}

void tickBuzzerPattern() {
  if (buzzerPattern == BUZZ_OFF) return;
  uint32_t t = millis() - buzzerPatternStartMs;
  switch (buzzerPattern) {
    case BUZZ_BEEP: {                    // single 200 ms beep then auto-off
      if (t < 200) buzzer_set(true);
      else { buzzer_set(false); buzzerPattern = BUZZ_OFF; }
      break;
    }
    case BUZZ_ALARM: {                   // 250 on / 250 off, continuous
      buzzer_set(((t / 250) % 2) == 0);
      break;
    }
    case BUZZ_SOS: {                     // ... --- ... over 3 s, repeating
      uint32_t pos = t % 3000;
      bool on = false;
      if      (pos <  200) on = true;
      else if (pos <  300) on = false;
      else if (pos <  500) on = true;
      else if (pos <  600) on = false;
      else if (pos <  800) on = true;
      else if (pos < 1000) on = false;
      else if (pos < 1400) on = true;
      else if (pos < 1500) on = false;
      else if (pos < 1900) on = true;
      else if (pos < 2000) on = false;
      else if (pos < 2400) on = true;
      else if (pos < 2500) on = false;
      else                 on = false;
      buzzer_set(on);
      break;
    }
    default: break;
  }
}

// ============================================================================
// LIGHT PATTERN STATE MACHINE (V10.1) — non-blocking, drives existing relays
// Called every loop(); writes only on transitions to avoid relay chatter.
// ============================================================================
void light_pattern_set(LightPattern p) {
  lightPattern = p;
  lightPatternStartMs = millis();
}

void tickLightPattern() {
  if (lightPattern == LIGHTP_OFF) return;
  uint32_t t = millis() - lightPatternStartMs;
  switch (lightPattern) {
    case LIGHTP_WARN_BLINK: {            // 1 Hz blink on warning light
      bool on = ((t / 500) % 2) == 0;
      if (on != warningLightState) on ? warning_light_on() : warning_light_off();
      break;
    }
    case LIGHTP_WARN_STROBE: {           // 5 Hz strobe on warning light
      bool on = ((t / 100) % 2) == 0;
      if (on != warningLightState) on ? warning_light_on() : warning_light_off();
      break;
    }
    case LIGHTP_BOTH_BLINK: {            // alternating warning vs main, 1 Hz
      bool warnOn = ((t / 500) % 2) == 0;
      if (warnOn  != warningLightState)  warnOn ? warning_light_on()  : warning_light_off();
      if (!warnOn != mainLightState)     warnOn ? main_light_off()    : main_light_on();
      break;
    }
    case LIGHTP_SOS: {                   // SOS Morse on warning light
      uint32_t pos = t % 3000;
      bool on = false;
      if      (pos <  200) on = true;
      else if (pos <  300) on = false;
      else if (pos <  500) on = true;
      else if (pos <  600) on = false;
      else if (pos <  800) on = true;
      else if (pos < 1000) on = false;
      else if (pos < 1400) on = true;
      else if (pos < 1500) on = false;
      else if (pos < 1900) on = true;
      else if (pos < 2000) on = false;
      else if (pos < 2400) on = true;
      else if (pos < 2500) on = false;
      else                 on = false;
      if (on != warningLightState) on ? warning_light_on() : warning_light_off();
      break;
    }
    default: break;
  }
}

// ============================================================================
// PHYSICAL E-STOP BUTTON (V10.1) — INPUT_PULLUP + FALLING ISR
// ISR is short: debounces and sets a flag. Real handling runs from main loop
// to avoid taking serial/relay locks inside an interrupt context.
// ============================================================================
void IRAM_ATTR estopISR() {
  uint32_t now = millis();
  if (now - estopLastIsrMs < ESTOP_DEBOUNCE_MS) return;
  estopLastIsrMs = now;
  estopTriggered = true;
}

void handleEstopIfTriggered() {
  static bool announced = false;
  if (!estopTriggered) { announced = false; return; }
  // Hold motors stopped; idempotent — safe to call every loop().
  FR_stop(); FL_stop(); BR_stop(); BL_stop();
  if (!announced) {
    warning_light_on();
    buzzer_start_pattern(BUZZ_ALARM);
    Serial.println("ESTOP:TRIGGERED source=physical_button");
    announced = true;
  }
}

// ============================================================================
// REMOTE-CONTROL RELAY (V10.1) — placeholder for wireless 12V cutoff relay
// User will install a remote-toggle relay (e.g. 433 MHz / WiFi) that hard-cuts
// motor power. While cutoff is true, motion commands are refused at parser top.
// ============================================================================
void remote_relay_cutoff_set(bool cutoff) {
  remoteRelayCutoff = cutoff;
  bool level = REMOTE_RELAY_ACTIVE_HIGH ? cutoff : !cutoff;
  digitalWrite(REMOTE_RELAY_PIN, level ? HIGH : LOW);
  if (cutoff) {
    FR_stop(); FL_stop(); BR_stop(); BL_stop();
    warning_light_on();
    Serial.println("REMOTE_RELAY:CUTOFF — motors disabled");
  } else {
    Serial.println("REMOTE_RELAY:RELEASED — motors re-enabled");
  }
}

// ============================================================================
// BNO08x IMU FUNCTIONS
// ============================================================================

void quaternionToEuler(float qr, float qi, float qj, float qk, float &yaw, float &pitch, float &roll) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  yaw *= RAD_TO_DEG;
  pitch *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;
  
  if (yaw < 0) yaw += 360;
}

// Read IMU and update global Euler angles (call this in loop)
void updateIMU() {
  if (!imuInitialized) return;
  
  // Check for reset
  if (bno08x.wasReset()) {
    Serial.println("IMU reset detected, re-enabling reports...");
    bno08x.enableReport(SH2_ROTATION_VECTOR, IMU_REPORT_INTERVAL_US);
  }
  
  // Read sensor data
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      quaternionToEuler(
        sensorValue.un.rotationVector.real,
        sensorValue.un.rotationVector.i,
        sensorValue.un.rotationVector.j,
        sensorValue.un.rotationVector.k,
        imuYaw, imuPitch, imuRoll
      );
      imuAccuracy = sensorValue.un.rotationVector.accuracy * RAD_TO_DEG;
      lastIMUUpdate = millis();
    }
  }
}

// Print current IMU values (human readable)
void printIMU() {
  if (!imuInitialized) {
    Serial.println("✗ IMU not initialized!");
    return;
  }
  
  // Update IMU first
  updateIMU();
  
  Serial.println("\n========================================");
  Serial.println("  BNO08x IMU Reading");
  Serial.println("========================================");
  Serial.printf("  YAW:   %7.2f°  (Heading/Compass)\n", imuYaw);
  Serial.printf("  PITCH: %7.2f°  (Tilt forward/back)\n", imuPitch);
  Serial.printf("  ROLL:  %7.2f°  (Tilt left/right)\n", imuRoll);
  Serial.println("----------------------------------------");
  Serial.printf("  Accuracy: %.1f°\n", imuAccuracy);
  if (imuAccuracy >= 10) {
    Serial.println("  ⚠️ Magnetometer needs calibration");
    Serial.println("  (Rotate sensor in figure-8 pattern)");
  } else {
    Serial.println("  ✓ Calibration OK");
  }
  Serial.println("========================================\n");
}

// Print IMU values in compact format for Pi parsing
// Format: IMU:yaw,pitch,roll,accuracy
void printIMUCompact() {
  if (!imuInitialized) {
    Serial.println("IMU:ERROR");
    return;
  }
  
  updateIMU();
  Serial.printf("IMU:%.2f,%.2f,%.2f,%.1f\n", imuYaw, imuPitch, imuRoll, imuAccuracy);
}

// Get heading for navigation (0-360 degrees)
float getHeading() {
  if (!imuInitialized) return -1;
  updateIMU();
  return imuYaw;
}

// ============================================================================
// HELP MENU
// ============================================================================
void printHelp() {
  Serial.println("Commands:");
  Serial.println("  Individual Motors:");
  Serial.println("    FR_F, FR_B, FR_S : Front-Right Forward/Backward/Stop");
  Serial.println("    FL_F, FL_B, FL_S : Front-Left Forward/Backward/Stop");
  Serial.println("    BR_F, BR_B, BR_S : Back-Right Forward/Backward/Stop");
  Serial.println("    BL_F, BL_B, BL_S : Back-Left Forward/Backward/Stop");
  Serial.println("");
  Serial.println("  Mecanum Movement:");
  Serial.println("    F      : Forward (all wheels same direction)");
  Serial.println("    B      : Backward (all wheels reverse)");
  Serial.println("    SR     : Strafe Right (sideways right)");
  Serial.println("    SL     : Strafe Left (sideways left)");
  Serial.println("    R      : Rotate Right (clockwise)");
  Serial.println("    L      : Rotate Left (counter-clockwise)");
  Serial.println("    DR     : Diagonal Forward-Right");
  Serial.println("    DL     : Diagonal Forward-Left");
  Serial.println("    S      : Stop all motors");
  Serial.println("");
  Serial.println("  Light Control:");
  Serial.println("    LIGHT_ON  : Warning Light on (Đèn cảnh báo)");
  Serial.println("    LIGHT_OFF : Warning Light off");
  Serial.println("    LIGHT_T   : Toggle Warning Light");
  Serial.println("    MAIN_ON   : Main Light on (Đèn chính)");
  Serial.println("    MAIN_OFF  : Main Light off");
  Serial.println("    MAIN_T    : Toggle Main Light");
  Serial.println("    LIGHT     : Show all light states");
  Serial.println("");
  Serial.println("  IMU (BNO08x):");
  Serial.println("    IMU      : Compact format (IMU:yaw,pitch,roll,accuracy)");
  Serial.println("    IMU_FULL : Human readable format");
  Serial.println("");
  Serial.println("  Speed Control:");
  Serial.println("    SPD:xxx          : Set speed (0-255), e.g. SPD:150");
  Serial.println("    MEC:vx,vy,wz,spd : Mecanum blended drive (line-follow)");
  Serial.println("                       vx/vy/wz: -127..127, spd: 0..255");
  Serial.println("");
  Serial.println("  Testing:");
  Serial.println("    T      : Run automated test sequence");
  Serial.println("    I      : Show system information");
  Serial.println("    H      : Show this help menu");
  Serial.println("");
  Serial.println("  Safety & Ops:");
  Serial.println("    PING             : reply PONG:<micros()>  (latency probe)");
  Serial.println("    WDT_ON / WDT_OFF : motor watchdog (auto-stop on silence)");
  Serial.println("    WDT?             : report watchdog state");
  Serial.println("    WIFI_ON:ssid,pw  : join WiFi + start ArduinoOTA");
  Serial.println("    OTA_OFF          : stop OTA and disable WiFi");
  Serial.println("    BENCH            : measure per-command motor-dispatch time");
  Serial.println("");
  Serial.println("  Light Patterns (V10.1, non-blocking):");
  Serial.println("    LP:OFF | LP:WARN_BLINK | LP:WARN_STROBE | LP:BOTH_BLINK | LP:SOS");
  Serial.println("");
  Serial.println("  Buzzer (V10.1):");
  Serial.println("    BUZZ:ON | BUZZ:OFF | BUZZ:BEEP | BUZZ:ALARM | BUZZ:SOS");
  Serial.println("");
  Serial.println("  Emergency Stop (V10.1):");
  Serial.println("    ESTOP         : software-trigger e-stop (mirrors physical button)");
  Serial.println("    ESTOP_CLEAR   : release e-stop after operator confirms safe");
  Serial.println("    ESTOP?        : report state (triggered, remote_cutoff, buzzer)");
  Serial.println("    REMOTE:1 / 0  : remote-relay cutoff (wireless motor power kill)");
  Serial.println("----------------------------------------");
  Serial.print("Current Speed: ");
  Serial.println(currentSpeed);
  Serial.print("Warning Light: ");
  Serial.println(warningLightState ? "ON" : "OFF");
  Serial.print("Main Light: ");
  Serial.println(mainLightState ? "ON" : "OFF");
  Serial.print("IMU: ");
  Serial.println(imuInitialized ? "OK" : "NOT CONNECTED");
  Serial.println("Ready for commands...\n");
}

// ============================================================================
// SYSTEM INFO
// ============================================================================
void printSystemInfo() {
  Serial.println("\n========================================");
  Serial.println("  MECANUM SYSTEM INFORMATION");
  Serial.println("========================================");
  Serial.println("Hardware:");
  Serial.println("  - ESP32-S3 WROOM-1");
  Serial.println("  - 4x BTS7960 Motor Drivers");
  Serial.println("  - 4x JGA25-370 12V Motors");
  Serial.println("  - 2x 12VDC-SL-C Relay (Warning + Main Light)");
  Serial.println("  - BNO08x IMU (9-DOF Orientation Sensor)");
  Serial.println("  - NEO-6M GPS (V5.2 outdoor mode)");
  Serial.println("  - Mecanum Wheel Configuration");
  Serial.println("");
  Serial.println("GPIO Mapping:");
  Serial.println("  Front-Right (FR): GPIO 4,5,6,7 (PWM Ch 0,1)");
  Serial.println("  Front-Left  (FL): GPIO 15,16,17,18 (PWM Ch 2,3)");
  Serial.println("  Back-Right  (BR): GPIO 8,9,10,11 (PWM Ch 4,5)");
  Serial.println("  Back-Left   (BL): GPIO 12,13,14,21 (PWM Ch 6,7)");
  Serial.println("  Warning Light  : GPIO 38");
  Serial.println("  Main Light     : GPIO 39");
  Serial.println("  BNO08x IMU     : GPIO 35,36,37 (UART1)");
  Serial.print  ("  NEO-6M GPS     : GPIO 41(RX),42(TX) (UART2) — ");
  Serial.print(gpsEnabled ? "ENABLED" : "DISABLED");
  Serial.print(", lines forwarded: ");
  Serial.println(gpsLinesForwarded);
  Serial.println("");
  Serial.println("Motor Direction Inversion:");
  Serial.print("  FR: ");
  Serial.print(FR_INVERTED ? "INVERTED" : "NORMAL");
  Serial.print("  FL: ");
  Serial.print(FL_INVERTED ? "INVERTED" : "NORMAL");
  Serial.print("  BR: ");
  Serial.print(BR_INVERTED ? "INVERTED" : "NORMAL");
  Serial.print("  BL: ");
  Serial.println(BL_INVERTED ? "INVERTED" : "NORMAL");
  Serial.println("");
  Serial.println("PWM Configuration:");
  Serial.print("  Frequency: ");
  Serial.print(PWM_FREQ);
  Serial.println(" Hz");
  Serial.print("  Resolution: ");
  Serial.print(PWM_RES);
  Serial.println("-bit (0-255)");
  Serial.println("");
  Serial.print("Current Speed: ");
  Serial.println(currentSpeed);
  Serial.print("Warning Light: ");
  Serial.println(warningLightState ? "ON" : "OFF");
  Serial.print("Main Light: ");
  Serial.println(mainLightState ? "ON" : "OFF");
  Serial.println("");
  Serial.println("BNO08x IMU Status:");
  Serial.print("  Initialized: ");
  Serial.println(imuInitialized ? "YES" : "NO");
  if (imuInitialized) {
    Serial.printf("  YAW: %.1f° | PITCH: %.1f° | ROLL: %.1f°\n", imuYaw, imuPitch, imuRoll);
    Serial.printf("  Accuracy: %.1f°\n", imuAccuracy);
  }
  Serial.println("");
  Serial.print("Free Heap: ");
  Serial.print(ESP.getFreeHeap());
  Serial.println(" bytes");
  Serial.println("========================================\n");
}

// ============================================================================
// AUTOMATED TEST SEQUENCE
// ============================================================================
void runTestSequence() {
  Serial.println("\n========================================");
  Serial.println("  MECANUM TEST SEQUENCE - 4 MOTORS");
  Serial.println("========================================");
  
  int testSpeed = 150;
  
  // Individual motor tests
  Serial.println("\n--- Individual Motor Tests ---");
  
  Serial.println("Test 1: FR Forward");
  FR_forward(testSpeed);
  delay(2000);
  FR_stop();
  delay(1000);
  
  Serial.println("Test 2: FL Forward");
  FL_forward(testSpeed);
  delay(2000);
  FL_stop();
  delay(1000);
  
  Serial.println("Test 3: BR Forward");
  BR_forward(testSpeed);
  delay(2000);
  BR_stop();
  delay(1000);
  
  Serial.println("Test 4: BL Forward");
  BL_forward(testSpeed);
  delay(2000);
  BL_stop();
  delay(1000);
  
  // Mecanum movement tests
  Serial.println("\n--- Mecanum Movement Tests ---");
  
  Serial.println("Test 5: Forward");
  mecanum_forward(testSpeed);
  delay(2000);
  mecanum_stop();
  delay(1000);
  
  Serial.println("Test 6: Backward");
  mecanum_backward(testSpeed);
  delay(2000);
  mecanum_stop();
  delay(1000);
  
  Serial.println("Test 7: Strafe Right");
  mecanum_strafe_right(testSpeed);
  delay(2000);
  mecanum_stop();
  delay(1000);
  
  Serial.println("Test 8: Strafe Left");
  mecanum_strafe_left(testSpeed);
  delay(2000);
  mecanum_stop();
  delay(1000);
  
  Serial.println("Test 9: Rotate Right");
  mecanum_rotate_right(testSpeed);
  delay(2000);
  mecanum_stop();
  delay(1000);
  
  Serial.println("Test 10: Rotate Left");
  mecanum_rotate_left(testSpeed);
  delay(2000);
  mecanum_stop();
  delay(1000);
  
  Serial.println("Test 11: Diagonal Forward-Right");
  mecanum_diagonal_forward_right(testSpeed);
  delay(2000);
  mecanum_stop();
  delay(1000);
  
  Serial.println("Test 12: Diagonal Forward-Left");
  mecanum_diagonal_forward_left(testSpeed);
  delay(2000);
  mecanum_stop();
  delay(1000);
  
  // Light relay test
  Serial.println("\n--- Light Relay Test ---");
  
  Serial.println("Test 13: Warning Light ON");
  warning_light_on();
  delay(2000);
  
  Serial.println("Test 14: Warning Light OFF");
  warning_light_off();
  delay(1000);
  
  Serial.println("Test 15: Main Light ON");
  main_light_on();
  delay(2000);
  
  Serial.println("Test 16: Main Light OFF");
  main_light_off();
  delay(1000);
  
  Serial.println("\n✓ Test sequence complete!");
  Serial.println("========================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Update IMU data continuously
  updateIMU();

  // Safety ticks (V10.1) — non-blocking, run every loop iteration.
  handleEstopIfTriggered();
  tickBuzzerPattern();
  tickLightPattern();

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();

    Serial.print("\n> Command: ");
    Serial.println(command);

    // Safety gate: while e-stop or remote cutoff is active, refuse all motion
    // commands so a runaway Pi cannot bypass the lock. Telemetry commands
    // (IMU, status, etc.) are still allowed.
    bool isMotion = (command == "F" || command == "B" || command == "SR" || command == "SL" ||
                     command == "R" || command == "L" || command == "DR" || command == "DL" ||
                     command == "BRAKE" ||
                     command == "FR_F" || command == "FR_B" ||
                     command == "FL_F" || command == "FL_B" ||
                     command == "BR_F" || command == "BR_B" ||
                     command == "BL_F" || command == "BL_B" ||
                     command.startsWith("MEC:") || command.startsWith("MOT:"));
    bool blocked = isMotion && (estopTriggered || remoteRelayCutoff);
    if (blocked) {
      Serial.print("✗ Motion BLOCKED — ");
      if (estopTriggered)     Serial.print("e-stop triggered (issue ESTOP_CLEAR). ");
      if (remoteRelayCutoff)  Serial.print("remote relay cutoff (issue REMOTE:0). ");
      Serial.println();
    }
    // Individual motor commands - FR
    else if (command == "FR_F") {
      FR_forward(currentSpeed);
      Serial.println("FR Forward");
    }
    else if (command == "FR_B") {
      FR_backward(currentSpeed);
      Serial.println("FR Backward");
    }
    else if (command == "FR_S") {
      FR_stop();
      Serial.println("FR Stopped");
    }
    
    // Individual motor commands - FL
    else if (command == "FL_F") {
      FL_forward(currentSpeed);
      Serial.println("FL Forward");
    }
    else if (command == "FL_B") {
      FL_backward(currentSpeed);
      Serial.println("FL Backward");
    }
    else if (command == "FL_S") {
      FL_stop();
      Serial.println("FL Stopped");
    }
    
    // Individual motor commands - BR
    else if (command == "BR_F") {
      BR_forward(currentSpeed);
      Serial.println("BR Forward");
    }
    else if (command == "BR_B") {
      BR_backward(currentSpeed);
      Serial.println("BR Backward");
    }
    else if (command == "BR_S") {
      BR_stop();
      Serial.println("BR Stopped");
    }
    
    // Individual motor commands - BL
    else if (command == "BL_F") {
      BL_forward(currentSpeed);
      Serial.println("BL Forward");
    }
    else if (command == "BL_B") {
      BL_backward(currentSpeed);
      Serial.println("BL Backward");
    }
    else if (command == "BL_S") {
      BL_stop();
      Serial.println("BL Stopped");
    }
    
    // Mecanum movement commands
    else if (command == "F") {
      mecanum_forward(currentSpeed);
    }
    else if (command == "B") {
      mecanum_backward(currentSpeed);
    }
    else if (command == "SR") {
      mecanum_strafe_right(currentSpeed);
    }
    else if (command == "SL") {
      mecanum_strafe_left(currentSpeed);
    }
    else if (command == "R") {
      mecanum_rotate_right(currentSpeed);
    }
    else if (command == "L") {
      mecanum_rotate_left(currentSpeed);
    }
    else if (command == "DR") {
      mecanum_diagonal_forward_right(currentSpeed);
    }
    else if (command == "DL") {
      mecanum_diagonal_forward_left(currentSpeed);
    }
    else if (command == "S") {
      mecanum_stop();
    }
    else if (command == "BRAKE") {
      mecanum_active_brake(80);
    }
    
    // Light control commands - Warning Light (Đèn cảnh báo)
    else if (command == "LIGHT_ON") {
      warning_light_on();
    }
    else if (command == "LIGHT_OFF") {
      warning_light_off();
    }
    else if (command == "LIGHT_T") {
      warning_light_toggle();
    }
    // Light control commands - Main Light (Đèn chính)
    else if (command == "MAIN_ON") {
      main_light_on();
    }
    else if (command == "MAIN_OFF") {
      main_light_off();
    }
    else if (command == "MAIN_T") {
      main_light_toggle();
    }
    // Light status
    else if (command == "LIGHT") {
      light_status();
    }
    
    // IMU commands
    else if (command == "IMU") {
      printIMUCompact();  // Compact format for Pi parsing
    }
    else if (command == "IMU_FULL") {
      printIMU();  // Human readable format
    }
    
    // Speed control
    else if (command.startsWith("SPD:")) {
      int newSpeed = command.substring(4).toInt();
      if (newSpeed >= 0 && newSpeed <= 255) {
        currentSpeed = newSpeed;
        Serial.print("✓ Speed set to: ");
        Serial.println(currentSpeed);
      } else {
        Serial.println("✗ Invalid speed! Use 0-255");
      }
    }

    // Mecanum blended drive: "MEC:vx,vy,wz,spd"
    // vx/vy/wz in [-127, 127] (unit-circle style), spd in [0, 255] is final magnitude
    else if (command.startsWith("MEC:")) {
      int vx = 0, vy = 0, wz = 0, spd = 0;
      if (sscanf(command.c_str(), "MEC:%d,%d,%d,%d", &vx, &vy, &wz, &spd) == 4) {
        vx  = constrain(vx,  -127, 127);
        vy  = constrain(vy,  -127, 127);
        wz  = constrain(wz,  -127, 127);
        spd = constrain(spd,    0, 255);

        // Inverse kinematics (X-wheels layout)
        int fl = vx - vy - wz;
        int fr = vx + vy + wz;
        int bl = vx + vy - wz;
        int br = vx - vy + wz;

        // Normalize if any wheel exceeds range
        int m = abs(fl);
        if (abs(fr) > m) m = abs(fr);
        if (abs(bl) > m) m = abs(bl);
        if (abs(br) > m) m = abs(br);
        if (m > 127) {
          fl = (int)((long)fl * 127 / m);
          fr = (int)((long)fr * 127 / m);
          bl = (int)((long)bl * 127 / m);
          br = (int)((long)br * 127 / m);
        }

        // Scale by target speed magnitude
        int pwm_fl = (int)((long)fl * spd / 127);
        int pwm_fr = (int)((long)fr * spd / 127);
        int pwm_bl = (int)((long)bl * spd / 127);
        int pwm_br = (int)((long)br * spd / 127);

        // Drive each wheel (FR/FL/BR/BL handle their own inversion flags)
        if (pwm_fl > 0)      FL_forward(pwm_fl);
        else if (pwm_fl < 0) FL_backward(-pwm_fl);
        else                 FL_stop();

        if (pwm_fr > 0)      FR_forward(pwm_fr);
        else if (pwm_fr < 0) FR_backward(-pwm_fr);
        else                 FR_stop();

        if (pwm_bl > 0)      BL_forward(pwm_bl);
        else if (pwm_bl < 0) BL_backward(-pwm_bl);
        else                 BL_stop();

        if (pwm_br > 0)      BR_forward(pwm_br);
        else if (pwm_br < 0) BR_backward(-pwm_br);
        else                 BR_stop();
      } else {
        Serial.println("✗ Invalid MEC! Use MEC:vx,vy,wz,spd");
      }
    }

    // Per-motor signed PWM: "MOT:fr,fl,br,bl"
    // Each value in [-255, 255]; positive = forward, negative = backward, 0 = stop.
    // Used by Pi-side velocity PID for closed-loop wheel speed control.
    else if (command.startsWith("MOT:")) {
      int fr = 0, fl = 0, br = 0, bl = 0;
      if (sscanf(command.c_str(), "MOT:%d,%d,%d,%d", &fr, &fl, &br, &bl) == 4) {
        fr = constrain(fr, -255, 255);
        fl = constrain(fl, -255, 255);
        br = constrain(br, -255, 255);
        bl = constrain(bl, -255, 255);

        if (fr > 0)      FR_forward(fr);
        else if (fr < 0) FR_backward(-fr);
        else             FR_stop();

        if (fl > 0)      FL_forward(fl);
        else if (fl < 0) FL_backward(-fl);
        else             FL_stop();

        if (br > 0)      BR_forward(br);
        else if (br < 0) BR_backward(-br);
        else             BR_stop();

        if (bl > 0)      BL_forward(bl);
        else if (bl < 0) BL_backward(-bl);
        else             BL_stop();
      } else {
        Serial.println("✗ Invalid MOT! Use MOT:fr,fl,br,bl");
      }
    }

    // Test & Info commands
    else if (command == "T") {
      runTestSequence();
    }
    else if (command == "I") {
      printSystemInfo();
    }
    else if (command == "H") {
      printHelp();
    }

    // ── Watchdog / OTA / Latency benchmark ──────────────────────────────
    // PING: low-level round-trip timing probe. Pi sends PING, firmware
    // replies PONG:<micros>. Pi uses this to measure serial→ESP latency.
    else if (command == "PING") {
      Serial.print("PONG:");
      Serial.println(micros());
    }
    // WDT_ON / WDT_OFF: enable/disable motor auto-stop-on-timeout.
    else if (command == "WDT_ON") {
      watchdogEnabled = true;
      watchdogTriggered = false;
      lastCommandTime = millis();
      Serial.println("WDT:ENABLED");
    }
    else if (command == "WDT_OFF") {
      watchdogEnabled = false;
      watchdogTriggered = false;
      Serial.println("WDT:DISABLED");
    }
    else if (command == "WDT?") {
      Serial.print("WDT:"); Serial.print(watchdogEnabled ? "ON" : "OFF");
      Serial.print(",trig="); Serial.print(watchdogTriggered ? 1 : 0);
      Serial.print(",idle_ms="); Serial.println(millis() - lastCommandTime);
    }
    // WIFI_ON:<ssid>,<pass> — bring WiFi up and start ArduinoOTA.
    // Credentials are only held in RAM; never logged in cleartext back.
    else if (command.startsWith("WIFI_ON:")) {
      String args = command.substring(8);
      int comma = args.indexOf(',');
      if (comma > 0) {
        String ssid = args.substring(0, comma);
        String pass = args.substring(comma + 1);
        ssid.trim(); pass.trim();
        ssid.toCharArray(otaSsid, sizeof(otaSsid));
        pass.toCharArray(otaPass, sizeof(otaPass));
        startOTA();
      } else {
        Serial.println("✗ WIFI_ON:<ssid>,<pass>");
      }
    }
    else if (command == "OTA_OFF" || command == "WIFI_OFF") {
      stopOTA();
    }
    // BENCH: time 100 MEC-stop dispatches to measure per-command overhead.
    else if (command == "BENCH") {
      const int N = 100;
      unsigned long t0 = micros();
      for (int i = 0; i < N; ++i) {
        // Same path as MEC:0,0,0,0 stop — avoids Serial I/O.
        FL_stop(); FR_stop(); BL_stop(); BR_stop();
      }
      unsigned long t1 = micros();
      unsigned long avg = (t1 - t0) / N;
      Serial.print("BENCH:N="); Serial.print(N);
      Serial.print(",total_us="); Serial.print(t1 - t0);
      Serial.print(",avg_us="); Serial.println(avg);
    }

    // ── Light patterns (V10.1) — non-blocking, drive the existing relays ─
    else if (command == "LP:OFF" || command == "LP:NONE") {
      lightPattern = LIGHTP_OFF;
      Serial.println("LIGHT_PATTERN:OFF");
    }
    else if (command == "LP:WARN_BLINK") {
      light_pattern_set(LIGHTP_WARN_BLINK);
      Serial.println("LIGHT_PATTERN:WARN_BLINK");
    }
    else if (command == "LP:WARN_STROBE") {
      light_pattern_set(LIGHTP_WARN_STROBE);
      Serial.println("LIGHT_PATTERN:WARN_STROBE");
    }
    else if (command == "LP:BOTH_BLINK") {
      light_pattern_set(LIGHTP_BOTH_BLINK);
      Serial.println("LIGHT_PATTERN:BOTH_BLINK");
    }
    else if (command == "LP:SOS") {
      light_pattern_set(LIGHTP_SOS);
      Serial.println("LIGHT_PATTERN:SOS");
    }

    // ── Buzzer (V10.1) — placeholder; physical buzzer to be installed later ─
    else if (command == "BUZZ:ON")    { buzzer_on();  Serial.println("BUZZ:ON"); }
    else if (command == "BUZZ:OFF")   { buzzer_off(); Serial.println("BUZZ:OFF"); }
    else if (command == "BUZZ:BEEP")  { buzzer_start_pattern(BUZZ_BEEP);  Serial.println("BUZZ:BEEP"); }
    else if (command == "BUZZ:ALARM") { buzzer_start_pattern(BUZZ_ALARM); Serial.println("BUZZ:ALARM"); }
    else if (command == "BUZZ:SOS")   { buzzer_start_pattern(BUZZ_SOS);   Serial.println("BUZZ:SOS"); }

    // ── E-Stop (V10.1) — software-trigger mirrors the physical button ─
    else if (command == "ESTOP" || command == "ESTOP_TRIGGER") {
      estopTriggered = true;
      handleEstopIfTriggered();
      Serial.println("ESTOP:TRIGGERED source=software");
    }
    else if (command == "ESTOP_CLEAR") {
      estopTriggered = false;
      buzzer_off();
      Serial.println("ESTOP:CLEARED");
    }
    else if (command == "ESTOP?" || command == "ESTOP_STATUS") {
      Serial.print("ESTOP:"); Serial.print(estopTriggered ? "TRIGGERED" : "CLEAR");
      Serial.print(",remote_cutoff="); Serial.print(remoteRelayCutoff ? 1 : 0);
      Serial.print(",buzzer="); Serial.println(buzzerState ? 1 : 0);
    }

    // ── Remote-control relay (V10.1) — placeholder for wireless cutoff ─
    else if (command == "REMOTE:1" || command == "REMOTE_CUTOFF") {
      remote_relay_cutoff_set(true);
    }
    else if (command == "REMOTE:0" || command == "REMOTE_RELEASE") {
      remote_relay_cutoff_set(false);
    }

    // Unknown command
    else {
      Serial.println("✗ Unknown command. Type 'H' for help.");
    }

    lastCommandTime = millis();
    // Any recognized command clears the watchdog trigger so motors can move
    // again after the Pi recovers from a brief silence.
    watchdogTriggered = false;
  }

  // Watchdog: auto-stop motors if Pi goes silent for > WATCHDOG_TIMEOUT_MS.
  // Runs every loop iteration, independent of Serial.available().
  if (watchdogEnabled && !watchdogTriggered &&
      (millis() - lastCommandTime) > WATCHDOG_TIMEOUT_MS) {
    mecanum_stop();
    watchdogTriggered = true;
    Serial.print("WDT:STOP timeout_ms=");
    Serial.println(millis() - lastCommandTime);
  }

  // OTA: service ArduinoOTA pump when enabled. Safe no-op when WiFi down.
  if (otaEnabled) {
    ArduinoOTA.handle();
  }
}

// ============================================================================
// OTA (WiFi firmware update) — opt-in, started via WIFI_ON:<ssid>,<pass>
// ============================================================================
void startOTA() {
  if (otaEnabled) {
    Serial.println("OTA:already enabled");
    return;
  }
  Serial.print("WIFI:connecting to ");
  Serial.println(otaSsid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(otaSsid, otaPass);
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - t0) < 15000UL) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WIFI:FAIL");
    WiFi.disconnect(true);
    return;
  }
  Serial.print("WIFI:OK ip=");
  Serial.println(WiFi.localIP());

  ArduinoOTA.setHostname(otaHost);
  ArduinoOTA.onStart([]() {
    // Stop motors before flashing to avoid runaway during reboot.
    mecanum_stop();
    watchdogEnabled = false;
    Serial.println("OTA:START");
  });
  ArduinoOTA.onEnd([]()    { Serial.println("\nOTA:END"); });
  ArduinoOTA.onError([](ota_error_t e) {
    Serial.print("OTA:ERR "); Serial.println((unsigned)e);
  });
  ArduinoOTA.onProgress([](unsigned int p, unsigned int t) {
    static unsigned int last = 0;
    unsigned int pct = (p * 100U) / t;
    if (pct != last && pct % 10 == 0) {
      Serial.print("OTA:"); Serial.print(pct); Serial.println("%");
      last = pct;
    }
  });
  ArduinoOTA.begin();
  otaEnabled = true;
  Serial.print("OTA:READY host=");
  Serial.println(otaHost);
}

void stopOTA() {
  if (!otaEnabled) {
    Serial.println("OTA:already disabled");
    return;
  }
  ArduinoOTA.end();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  otaEnabled = false;
  Serial.println("OTA:STOPPED");
}
