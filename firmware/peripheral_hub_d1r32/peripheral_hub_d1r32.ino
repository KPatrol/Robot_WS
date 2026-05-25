/*
 * ============================================================================
 * KPatrol — Peripheral Hub v2.0 (ESP32 D1 R32 / WeMos D1 R32)
 * ============================================================================
 *
 * v2.0 changes (was v1.x): lasers removed entirely; OLED redesigned into an
 * animated smiley + sensor panel; clock sync over UART added (TIME command).
 *
 * Purpose
 *   Single firmware for the D1 R32 peripheral controller. Drives 3 devices
 *   and streams telemetry to the Raspberry Pi over USB serial at 115200 bps.
 *   The Pi appears on /dev/ttyUSB0 (CH340G 1a86:7523) and is normally aliased
 *   to /dev/kpatrol-periph via udev.
 *
 * Devices managed
 *   ┌──────────────┬──────────────────┬─────────────────────────────────────┐
 *   │ Device       │ MCU pin (GPIO)   │ Notes                                │
 *   ├──────────────┼──────────────────┼─────────────────────────────────────┤
 *   │ OLED SH1106  │ SDA=21, SCL=22   │ I²C 0x3C, 1.3" 128×64                │
 *   │ DHT11        │ DATA=GPIO25      │ 1-wire, ≥2 s between reads            │
 *   │ Relay 5V     │ IN=GPIO26        │ Active-HIGH (no opto) — HIGH = ON    │
 *   └──────────────┴──────────────────┴─────────────────────────────────────┘
 *
 *   Relay drives a 12V automotive lamp + horn through NO/COM with a 5A fuse.
 *   GND of relay module MUST be tied to D1 R32 GND (common ground).
 *
 * Telemetry (Pi reads these — each line terminated by '\n', stable schema)
 *   BOOT:periph-hub-v2.0,sda=21,scl=22,dht=25,relay=26   — on power-up
 *   HB:t=<ms>                                            — every 1 s
 *   DHT:<temp_C>,<humi_%>                                — every 5 s
 *   DHT:disabled                                         — sensor offline/fault
 *   STATE:relay=<0|1>                                    — change + every 5 s
 *   HEAP:<bytes>                                         — every 30 s
 *   META:fw=...,uptime=...,watchdog_armed=<0|1>,relay_pol=<low|high>,time=<HH:MM:SS|?>
 *                                                        — on STATUS
 *   PONG:t=<ms>                                          — reply to PING
 *   WATCHDOG:fired,reason=<str>                          — when auto-OFF triggered
 *   TIME:set,<HH:MM:SS>                                  — ack after TIME command
 *   ERR:<reason>                                         — parser/validation errors
 *
 * Commands (Pi writes these — terminated by '\n' or '\r\n')
 *   PING                  → PONG line
 *   STATUS                → META + STATE + DHT lines
 *   KEEPALIVE             → just resets the 5 s safety watchdog timer
 *   RELAY:ON|OFF|T        → drive / toggle relay
 *   RELAY:TEST            → physical click test (LOW/HIGH/LOW/HIGH @ 600 ms)
 *   RELAY:POL:LOW|HIGH    → runtime polarity flip
 *   TIME:HH:MM[:SS]       → sync wall-clock (Pi sends every minute)
 *   OLED:<text>           → show <text> as a status line under the face
 *
 * Safety watchdog
 *   The relay is "armed" the moment a RELAY:ON / RELAY:T toggles it ON.
 *   While armed, the Pi MUST send a command (any command, or an explicit
 *   KEEPALIVE) every WATCHDOG_MS milliseconds — otherwise the hub forces
 *   the relay OFF and emits a WATCHDOG line. This prevents the 12 V lamp
 *   from being stuck ON if the Pi crashes or the USB link dies.
 *
 * Hard task watchdog (TASK_WDT_TIMEOUT_S) resets the chip if loop() hangs.
 *
 * Arduino IDE settings
 *   Board:   "ESP32 Dev Module"   (D1 R32 uses the same core as plain ESP32)
 *   Upload Speed: 921600 (drop to 460800 if upload fails on CH340G)
 *   Flash Size: 4MB
 *   CPU Frequency: 240 MHz
 *   Partition: Default
 *   Baud:    115200
 *   Libraries:
 *     - "Adafruit SH110X" (SH1106 controller variant)
 *     - "Adafruit GFX Library"
 *     - "DHT sensor library" by Adafruit
 *     - "Adafruit Unified Sensor" (DHT dependency)
 *
 * ============================================================================
 */

// ─── Compile-time peripheral enables ───────────────────────────────────────
//
// IMPORTANT: when a sensor is NOT physically wired and we still call its
// library, the library can spin with interrupts disabled long enough to trip
// the ESP32 Interrupt WDT (300 ms default) → Guru Meditation crash loop.
// Set these to 0 to compile that peripheral out entirely.
//
// Bisect order if you're getting crashes:
//   1. Both 0 → only HB stream. If still crashes, suspect power/USB cable.
//   2. ENABLE_OLED=1, ENABLE_DHT=0 → confirms OLED is fine.
//   3. ENABLE_DHT=1 ONLY AFTER you have a DHT11 physically connected and
//      reading good values on a separate bench test.
#define ENABLE_OLED 1
#define ENABLE_DHT  1

#include <Wire.h>
#include <driver/gpio.h>
#include <esp_task_wdt.h>
#include "eye_state.h"  // EyeState struct — must come before any function
                        // signature that uses it; isolating it in a .h
                        // avoids Arduino IDE's auto-prototype scanner.
#if ENABLE_OLED
  #include <Adafruit_GFX.h>
  #include <Adafruit_SH110X.h>
#endif
#if ENABLE_DHT
  #include <DHT.h>
#endif

// Hard task-watchdog timeout. If loop() hangs (e.g. I²C deadlock during an
// oled.display() while SDA is held LOW by a faulty slave), the watchdog
// resets the chip after this many seconds. Pick well above any legitimate
// blocking call (OLED full-frame = ~85 ms @ 100 kHz; DHT read = ~250 ms).
#define TASK_WDT_TIMEOUT_S  8

// How often to log free heap (helps spot fragmentation creep).
static constexpr uint32_t HEAP_PERIOD_MS = 30000;

// ─── Pin map ────────────────────────────────────────────────────────────────
//
// IMPORTANT: D1 R32 silkscreen labels do NOT match GPIO numbers in the obvious
// way. Use GPIO numbers (not "SDA"/"SCL" macros) so the mapping is explicit:
//   silkscreen SDA → GPIO21
//   silkscreen SCL → GPIO22
//   silkscreen IO25 → GPIO25, etc.
#define PIN_SDA       21
#define PIN_SCL       22
#define PIN_DHT       25
#define PIN_RELAY     26
// GPIO27 + GPIO16 are unused on v2.0 (laser feature removed).

#define I2C_FREQ_HZ   100000UL    // 100 kHz — safer for SH1106 on long wires
#define I2C_TIMEOUT_MS 50         // Wire bus timeout per transaction
#define OLED_ADDR     0x3C
#define OLED_W        128
#define OLED_H        64

#define DHT_TYPE      DHT11

// ─── Cadence (ms) ──────────────────────────────────────────────────────────
static constexpr uint32_t DHT_PERIOD_MS    = 5000;
static constexpr uint32_t HB_PERIOD_MS     = 1000;
static constexpr uint32_t OLED_PERIOD_MS   = 1000;
static constexpr uint32_t STATE_PERIOD_MS  = 5000;
static constexpr uint32_t WATCHDOG_MS      = 5000;

// After this many consecutive DHT failures, stop reading (avoids the
// noInterrupts() block inside Adafruit DHT library starving system tasks)
// and re-probe only once every DHT_RECOVER_PERIOD_MS.
static constexpr uint8_t  DHT_FAIL_THRESHOLD     = 3;
static constexpr uint32_t DHT_RECOVER_PERIOD_MS  = 30000;

// ─── Relay polarity (runtime-switchable) ────────────────────────────────────
//
// The specific 5V relay module on K-Patrol's bench is ACTIVE-HIGH (no
// optocoupler — IN driven HIGH triggers the coil). Most "5V Relay Module"
// boards with PC817 opto-input are active-LOW; if you swap to such a module
// just flip this default back to `true` or use `RELAY:POL:LOW` at runtime.
//
// We expose the polarity as a runtime variable so the operator can flip it
// from Serial Monitor with `RELAY:POL:HIGH` / `LOW` without re-flashing while
// bench-testing — handy when the silkscreen lies.
bool relay_active_low = false;   // K-Patrol module: active-HIGH (bench-confirmed)

static inline uint8_t relay_on_level()  { return relay_active_low ? LOW  : HIGH; }
static inline uint8_t relay_off_level() { return relay_active_low ? HIGH : LOW;  }

// ─── Globals ───────────────────────────────────────────────────────────────
#if ENABLE_OLED
Adafruit_SH1106G oled(OLED_W, OLED_H, &Wire, -1);
#endif
#if ENABLE_DHT
DHT              dht(PIN_DHT, DHT_TYPE);
#endif

bool      oled_ok            = false;
bool      relay_state        = false;
bool      watchdog_armed     = false;
uint32_t  t_last_cmd         = 0;
uint32_t  t_last_dht         = 0;
uint32_t  t_last_hb          = 0;
uint32_t  t_last_oled        = 0;
uint32_t  t_last_state_emit  = 0;
float     last_temp          = NAN;
float     last_humi          = NAN;

// Wall-clock sync from the Pi. The MCU has no RTC, so the Pi pushes the
// current time every minute via `TIME:HH:MM[:SS]`. We store the base and
// extrapolate using millis() until the next push.
bool      time_known         = false;
uint8_t   time_base_hour     = 0;
uint8_t   time_base_min      = 0;
uint8_t   time_base_sec      = 0;
uint32_t  time_base_ms       = 0;

// DHT failure tracking — when the sensor is unplugged or wired wrong, the
// Adafruit library spins inside a noInterrupts() block which can starve
// the system IDLE task and trip the Interrupt WDT. We bail out after a few
// consecutive failures and only re-probe every DHT_RECOVER_PERIOD_MS.
uint8_t   dht_fail_count     = 0;
bool      dht_disabled       = false;
uint32_t  t_dht_recover      = 0;

// Fixed-size buffers — Arduino String would re-allocate on every byte append
// in poll_serial() and on every OLED:<...> command, fragmenting the heap and
// eventually starving malloc after a few hours of UART traffic. Plain char[]
// gives constant memory and zero allocator pressure.
static constexpr uint8_t  CMD_BUF_MAX     = 96;
static constexpr uint8_t  OLED_STATUS_MAX = 21;   // 20 visible chars + NUL

char      cmd_buf[CMD_BUF_MAX + 1] = "";
uint8_t   cmd_len = 0;
// Status text shown below the smiley face — Pi can override via `OLED:<text>`.
char      oled_status[OLED_STATUS_MAX] = "K-Patrol ready";

uint32_t  t_last_heap        = 0;
uint32_t  t_last_dht_disabled_emit = 0;

// EyeState struct lives in eye_state.h (included near the top of the file).

// ─── Forward decls ─────────────────────────────────────────────────────────
static void emit_state();
static void draw_oled();

// ─── IO helpers ────────────────────────────────────────────────────────────
static void set_relay(bool on) {
  if (relay_state == on) return;
  relay_state = on;
  digitalWrite(PIN_RELAY, on ? relay_on_level() : relay_off_level());
  if (on) watchdog_armed = true;
  emit_state();
}

static void force_all_off(const char* reason) {
  // Skip the change-guard in set_relay() so we always pulse the pin to a
  // known safe level even if we think we're already there (defensive).
  digitalWrite(PIN_RELAY, relay_off_level());
  bool was_armed = watchdog_armed;
  bool any_was_on = relay_state;
  relay_state = false;
  watchdog_armed = false;
  if (any_was_on || was_armed) {
    Serial.printf("WATCHDOG:fired,reason=%s\n", reason);
    emit_state();
  }
}

// ─── Time helpers ─────────────────────────────────────────────────────────
// Project the wall-clock forward from the last TIME: sync using millis().
// Returns false if the Pi has never sent a TIME line.
static bool compute_now(uint8_t* h, uint8_t* m, uint8_t* s) {
  if (!time_known) {
    *h = *m = *s = 0;
    return false;
  }
  uint32_t elapsed_s = (millis() - time_base_ms) / 1000;
  uint32_t total_s = (uint32_t)time_base_sec + elapsed_s;
  *s = (uint8_t)(total_s % 60);
  uint32_t total_m = (uint32_t)time_base_min + total_s / 60;
  *m = (uint8_t)(total_m % 60);
  uint32_t total_h = (uint32_t)time_base_hour + total_m / 60;
  *h = (uint8_t)(total_h % 24);
  return true;
}

// ─── Telemetry emitters ────────────────────────────────────────────────────
static void emit_state() {
  Serial.printf("STATE:relay=%d\n", (int)relay_state);
}

static void emit_dht() {
  if (isnan(last_temp) || isnan(last_humi)) {
    Serial.println("DHT:nan,nan");
  } else {
    Serial.printf("DHT:%.1f,%.1f\n", last_temp, last_humi);
  }
}

static void emit_meta() {
  uint8_t h = 0, m = 0, s = 0;
  bool have_time = compute_now(&h, &m, &s);
  if (have_time) {
    Serial.printf("META:fw=periph-hub-v2.0,uptime=%lu,watchdog_armed=%d,oled=%d,relay_pol=%s,time=%02u:%02u:%02u\n",
                  (unsigned long)millis(), (int)watchdog_armed, (int)oled_ok,
                  relay_active_low ? "active_low" : "active_high",
                  h, m, s);
  } else {
    Serial.printf("META:fw=periph-hub-v2.0,uptime=%lu,watchdog_armed=%d,oled=%d,relay_pol=%s,time=?\n",
                  (unsigned long)millis(), (int)watchdog_armed, (int)oled_ok,
                  relay_active_low ? "active_low" : "active_high");
  }
}

// ─── OLED ──────────────────────────────────────────────────────────────────
//
// 128×64 SH1106 layout (default 6×8 font):
//
//   ┌────────────────────────────────────┐
//   │   ◉ ◉            T: 30.3°C         │ ← animated smiley on the left,
//   │   \___/          H:  95 %          │   sensor + clock on the right
//   │                  14:30:45          │
//   │ ─────────────────────────────────  │ ← divider
//   │ status text (from Pi)           ●  │ ← Pi-pushed status + heart pulse
//   └────────────────────────────────────┘
//
// Eye animation cycles every ~12 s (forward → right → forward → left → forward
// → up → forward → blink). The blink is two consecutive frames so it stays
// visible at the 1 Hz redraw rate.
// ─── Animated robot eyes (V2.2) ─────────────────────────────────────────
//
// Inspired by playfultechnology/esp32-eyes — two rounded-rectangle eyes
// side-by-side with darting pupils and natural blinks, instead of the
// single smiley face. The eye geometry is hand-tuned for 128×64 SH1106:
//   • Each eye is 22×16 px, rounded corners radius 4.
//   • Pupils are 6×6 squares (looks more "digital robot" than circular).
//   • Pupils dart along a small jitter pattern; a full blink (top + bottom
//     lids meeting in the middle) fires every ~6 s.
//   • Eyebrow angle changes randomly on a slow timer to give the face a
//     mood — neutral / friendly / surprised / concerned.
//
// Drawn into the left half of the screen so the temp/humidity/clock
// readout stays on the right. Eyes feel alive at the 1 Hz redraw cadence
// because the timing math is offset by `millis()` rather than a frame
// counter — when frames drop on a slow loop the animation still tracks
// wall clock.
static const int EYE_W = 22;
static const int EYE_H = 16;
static const int EYE_R = 4;
static const int EYE_TOP = 14;
static const int EYE_LX = 2;            // left eye x
static const int EYE_RX = EYE_LX + EYE_W + 4;  // right eye x, 4 px gap

// Convert a phase 0..15 (refreshed every ~400 ms) into a {dx,dy,blink,mood}
// tuple. Most phases are forward neutral; quick darts + one blink + one
// "surprise" eyebrow break the loop without spamming motion.
// EyeState struct itself is declared near the top of the file so Arduino's
// auto-prototype pass sees the type before any function signature using it.
static EyeState compute_eye_state(uint32_t now_ms) {
  uint32_t phase = (now_ms / 400u) % 28u;
  EyeState s = {0, 0, 0, 0, 0};
  switch (phase) {
    case  3:               s.dx =  3; break;
    case  6:               s.dx = -3; break;
    case  9:               s.dy = -2; break;
    case 11: case 12:      s.lid_top = EYE_H / 2; s.lid_bot = EYE_H / 2; break; // blink 0.8 s
    case 15:               s.dx =  2; s.dy = -1; break;
    case 18:               s.brow_dy =  2; break;  // happy ear-perk
    case 21:               s.dx = -2; s.dy =  1; break;
    case 25: case 26:      s.lid_top = EYE_H / 2; s.lid_bot = EYE_H / 2; break; // blink again
    default: /* neutral forward */                                       break;
  }
  return s;
}

// Draw one rounded-rectangle eye with a square pupil and optional lids.
// Eyebrow is a 3-pixel-tall slanted line above the eye whose slope hint
// comes from `brow_dy`.
static void draw_eye(int x, int y, const EyeState& s, bool tilt_inward) {
  // Eye whites — filled rounded rectangle.
  oled.fillRoundRect(x, y, EYE_W, EYE_H, EYE_R, SH110X_WHITE);
  // Pupil — square so the eyes look like a friendly tabletop robot, not a
  // human portrait. Centred then offset by gaze direction.
  int px = x + (EYE_W - 6) / 2 + s.dx;
  int py = y + (EYE_H - 6) / 2 + s.dy;
  oled.fillRect(px, py, 6, 6, SH110X_BLACK);
  // Eyelids — symmetric rectangles eating into the top + bottom of the
  // white. When lid_top == EYE_H/2 the eye is fully closed.
  if (s.lid_top > 0) {
    oled.fillRect(x, y, EYE_W, s.lid_top, SH110X_BLACK);
  }
  if (s.lid_bot > 0) {
    oled.fillRect(x, y + EYE_H - s.lid_bot, EYE_W, s.lid_bot, SH110X_BLACK);
  }
  // Eyebrow above. The two eyes tilt INWARD when happy/surprised — left
  // brow rises on its outer edge, right brow on its inner edge.
  int brow_y = y - 4;
  int slope = (tilt_inward ? +s.brow_dy : -s.brow_dy);
  oled.drawLine(x + 1, brow_y - slope, x + EYE_W - 2, brow_y + slope, SH110X_WHITE);
}

static void draw_oled() {
#if ENABLE_OLED
  if (!oled_ok) return;

  uint32_t now_ms = millis();
  oled.clearDisplay();
  oled.setTextColor(SH110X_WHITE);
  oled.setTextSize(1);

  // ── Header strip ─────────────────────────────────────────────────────
  // Inverted bar with brand on the left + small status pips on the right
  // (CLK = wall-clock sync from Pi, ON/-- = relay output state).
  oled.fillRect(0, 0, OLED_W, 9, SH110X_WHITE);
  oled.setTextColor(SH110X_BLACK, SH110X_WHITE);
  oled.setCursor(2, 1);
  oled.print(F("K-PATROL"));
  oled.setCursor(OLED_W - 39, 1);
  oled.print(time_known ? F("CLK") : F("---"));
  if (relay_state) {
    oled.fillRect(OLED_W - 16, 1, 14, 7, SH110X_BLACK);
    oled.setTextColor(SH110X_WHITE, SH110X_BLACK);
    oled.setCursor(OLED_W - 14, 1);
    oled.print(F("ON"));
  } else {
    oled.drawRect(OLED_W - 16, 1, 14, 7, SH110X_BLACK);
    oled.setTextColor(SH110X_BLACK, SH110X_WHITE);
    oled.setCursor(OLED_W - 13, 1);
    oled.print(F("--"));
  }
  oled.setTextColor(SH110X_WHITE);

  // ── Two animated eyes (left half) ────────────────────────────────────
  EyeState s = compute_eye_state(now_ms);
  draw_eye(EYE_LX, EYE_TOP, s, /*tilt_inward=*/false);
  draw_eye(EYE_RX, EYE_TOP, s, /*tilt_inward=*/true);

  // ── Right info column: temp / humidity / clock with mini icons ──────
  const int INFO_X = EYE_RX + EYE_W + 4;   // ~54
  const int INFO_TEXT_X = INFO_X + 10;
  oled.drawFastVLine(INFO_X - 2, 12, 36, SH110X_WHITE);  // soft separator

  // Thermometer
  oled.drawFastVLine(INFO_X + 2, 14, 5, SH110X_WHITE);
  oled.drawFastVLine(INFO_X + 4, 14, 5, SH110X_WHITE);
  oled.fillCircle(INFO_X + 3, 20, 2, SH110X_WHITE);
  oled.setCursor(INFO_TEXT_X, 15);
  if (isnan(last_temp)) {
    oled.print(F("--.- C"));
  } else {
    oled.printf("%4.1f", last_temp);
    oled.write(247);
    oled.print(F("C"));
  }

  // Humidity droplet
  oled.drawFastVLine(INFO_X + 3, 26, 2, SH110X_WHITE);
  oled.drawFastVLine(INFO_X + 2, 27, 2, SH110X_WHITE);
  oled.drawFastVLine(INFO_X + 4, 27, 2, SH110X_WHITE);
  oled.fillCircle(INFO_X + 3, 30, 2, SH110X_WHITE);
  oled.setCursor(INFO_TEXT_X, 27);
  if (isnan(last_humi)) {
    oled.print(F("--- %"));
  } else {
    oled.printf("%3.0f %%", last_humi);
  }

  // Clock dial
  oled.drawCircle(INFO_X + 3, 40, 3, SH110X_WHITE);
  oled.drawFastVLine(INFO_X + 3, 38, 3, SH110X_WHITE);
  oled.drawFastHLine(INFO_X + 3, 40, 3, SH110X_WHITE);
  oled.setCursor(INFO_TEXT_X, 39);
  uint8_t hh = 0, mm = 0, ss = 0;
  if (compute_now(&hh, &mm, &ss)) {
    oled.printf("%02u:%02u:%02u", hh, mm, ss);
  } else {
    uint32_t total_s = now_ms / 1000;
    uint32_t up_h = total_s / 3600;
    uint32_t up_m = (total_s / 60) % 60;
    oled.printf("up %lu:%02lu", (unsigned long)up_h, (unsigned long)up_m);
  }

  // ── Footer: divider + author / status rotate + heartbeat ─────────────
  oled.drawFastHLine(0, 48, OLED_W, SH110X_WHITE);

  // Cycle every 5 s so the attribution always appears within one screen
  // cycle, without permanently displacing the live status text.
  bool show_author = ((now_ms / 5000u) & 1u) != 0;
  oled.setCursor(0, 52);
  if (show_author || oled_status[0] == '\0') {
    oled.print(F("by Vu Dang Khoa"));
  } else {
    oled.print(oled_status);
  }

  // Heartbeat pulse — verifies the redraw loop is alive.
  static uint8_t pulse_phase = 0;
  pulse_phase = (pulse_phase + 1) & 0x03;
  oled.fillCircle(OLED_W - 5, 56, (pulse_phase < 2) ? 3 : 1, SH110X_WHITE);

  oled.display();
#endif
}

// ─── Command parser ────────────────────────────────────────────────────────
//
// Allocation-free parser: `const char*` + strcmp/strncmp. No String objects,
// no heap traffic, deterministic timing — important since this runs on every
// command and the Pi can flood several lines per second.
static void handle_cmd(const char* cmd) {
  if (cmd[0] == '\0') return;
  t_last_cmd = millis();

  if (strcmp(cmd, "PING") == 0) {
    Serial.printf("PONG:t=%lu\n", (unsigned long)millis());
    return;
  }
  if (strcmp(cmd, "STATUS") == 0) {
    emit_meta();
    emit_state();
    emit_dht();
    return;
  }
  if (strcmp(cmd, "KEEPALIVE") == 0) {
    // t_last_cmd already updated above — that is the whole point.
    return;
  }
  if (strcmp(cmd, "RELAY:ON")  == 0) { set_relay(true);  return; }
  if (strcmp(cmd, "RELAY:OFF") == 0) { set_relay(false); return; }
  if (strcmp(cmd, "RELAY:T")   == 0) { set_relay(!relay_state); return; }

  // Runtime polarity flip — for clone modules that ignore the silkscreen.
  if (strcmp(cmd, "RELAY:POL:LOW") == 0) {
    relay_active_low = true;
    digitalWrite(PIN_RELAY, relay_state ? relay_on_level() : relay_off_level());
    Serial.println("RELAY:polarity=active_low");
    return;
  }
  if (strcmp(cmd, "RELAY:POL:HIGH") == 0) {
    relay_active_low = false;
    digitalWrite(PIN_RELAY, relay_state ? relay_on_level() : relay_off_level());
    Serial.println("RELAY:polarity=active_high");
    return;
  }
  // Physical click test — drive GPIO HIGH/LOW twice with 600 ms gaps so the
  // operator can hear which level makes the relay coil engage. Watchdog is
  // reset on every iteration so it doesn't trip during the test.
  if (strcmp(cmd, "RELAY:TEST") == 0) {
    Serial.println("RELAY:test_begin,expect_4_clicks");
    for (int i = 0; i < 2; i++) {
      digitalWrite(PIN_RELAY, LOW);
      Serial.println("RELAY:drive=LOW");
      t_last_cmd = millis();
      esp_task_wdt_reset();
      delay(600);
      digitalWrite(PIN_RELAY, HIGH);
      Serial.println("RELAY:drive=HIGH");
      t_last_cmd = millis();
      esp_task_wdt_reset();
      delay(600);
    }
    // Restore to whatever the logical state says.
    digitalWrite(PIN_RELAY, relay_state ? relay_on_level() : relay_off_level());
    Serial.printf("RELAY:test_end,polarity=%s\n",
                  relay_active_low ? "active_low" : "active_high");
    return;
  }

  // Wall-clock sync from the Pi. Accepts `TIME:HH:MM` or `TIME:HH:MM:SS`.
  // Anything outside 0..23 / 0..59 ranges is rejected with ERR:bad_time.
  if (strncmp(cmd, "TIME:", 5) == 0) {
    int h = -1, m = -1, s = 0;
    int parsed = sscanf(cmd + 5, "%d:%d:%d", &h, &m, &s);
    if (parsed >= 2 &&
        h >= 0 && h < 24 && m >= 0 && m < 60 && s >= 0 && s < 60) {
      time_base_hour = (uint8_t)h;
      time_base_min  = (uint8_t)m;
      time_base_sec  = (uint8_t)s;
      time_base_ms   = millis();
      time_known     = true;
      Serial.printf("TIME:set,%02d:%02d:%02d\n", h, m, s);
      return;
    }
    Serial.println("ERR:bad_time");
    return;
  }

  // Status line shown below the smiley. Pi pushes things like "Patrol Lobby".
  // Empty payload clears the line.
  if (strncmp(cmd, "OLED:", 5) == 0) {
    strncpy(oled_status, cmd + 5, OLED_STATUS_MAX - 1);
    oled_status[OLED_STATUS_MAX - 1] = '\0';
    draw_oled();
    return;
  }

  Serial.printf("ERR:unknown_cmd=%s\n", cmd);
}

// Strip trailing whitespace / control chars in place. Mirrors String.trim()
// but on a char[] — caller passes the live buffer and we move the NUL back.
static void rstrip_inplace(char* buf, uint8_t* len) {
  while (*len > 0 && (buf[*len - 1] == ' ' || buf[*len - 1] == '\t')) {
    (*len)--;
  }
  buf[*len] = '\0';
}

static void poll_serial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rstrip_inplace(cmd_buf, &cmd_len);
      handle_cmd(cmd_buf);
      cmd_len = 0;
      cmd_buf[0] = '\0';
    } else if (cmd_len < CMD_BUF_MAX) {
      cmd_buf[cmd_len++] = c;
      cmd_buf[cmd_len] = '\0';
    } else {
      // Overflow — drop the line and emit a diagnostic so the operator
      // knows the Pi sent a runaway frame (or noise on USB).
      Serial.printf("ERR:cmd_overflow=%u\n", (unsigned)cmd_len);
      cmd_len = 0;
      cmd_buf[0] = '\0';
    }
  }
}

// ─── Setup ─────────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(100);

  // ── Boot-time GPIO safety ────────────────────────────────────────────────
  // The ESP32 boot ROM leaves most GPIO in input/high-Z for ~30 ms before
  // user code runs. With the relay attached, that means the IN line floats
  // somewhere between rails and the coil can tick momentarily, dumping 12 V
  // into the light + horn. Enable the internal pull resistor matching the
  // relay's "OFF" level BEFORE configuring the pin as OUTPUT.
  //
  //   Active-LOW relay  → OFF = HIGH → enable internal pull-UP
  //   Active-HIGH relay → OFF = LOW  → enable internal pull-DOWN
  if (relay_active_low) {
    gpio_pulldown_dis((gpio_num_t)PIN_RELAY);
    gpio_pullup_en((gpio_num_t)PIN_RELAY);
  } else {
    gpio_pullup_dis((gpio_num_t)PIN_RELAY);
    gpio_pulldown_en((gpio_num_t)PIN_RELAY);
  }

  // Drive to known-safe level then switch to OUTPUT. Order matters:
  // digitalWrite() on an INPUT pin only changes the latched output value;
  // pinMode(OUTPUT) then enables the driver with that value already set.
  digitalWrite(PIN_RELAY, relay_off_level());
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, relay_off_level());

  Serial.println();
  Serial.printf("BOOT:periph-hub-v2.0,sda=%d,scl=%d,dht=%d,relay=%d\n",
                PIN_SDA, PIN_SCL, PIN_DHT, PIN_RELAY);

  // I²C @ 100 kHz — conservative speed for SH1106 on the breadboard wiring.
  Wire.begin(PIN_SDA, PIN_SCL, I2C_FREQ_HZ);
  Wire.setTimeOut(I2C_TIMEOUT_MS);  // bail fast if SDA/SCL is stuck low
  delay(50);

#if ENABLE_OLED
  // OLED — Adafruit_SH1106G.begin() returns true on success.
  if (oled.begin(OLED_ADDR, true)) {
    oled_ok = true;
    Serial.printf("OLED:ok=1,addr=0x%02X\n", OLED_ADDR);
    // V2.2 boot splash. Three staged frames + a hold so the operator's eye
    // always catches the brand on a power-cycle:
    //
    //   F0 (0.5 s) ─ open with a peeking eye blink (the robot "wakes up")
    //   F1 (0.5 s) ─ K-PATROL banner drops in + chevron decoration on sides
    //   F2 (0.5 s) ─ subtitle + author credit fade in
    //   hold 1.5 s ─ static frame for reading
    for (uint8_t frame = 0; frame < 3; ++frame) {
      oled.clearDisplay();
      oled.setTextColor(SH110X_WHITE);

      if (frame == 0) {
        // Wake-up: a single rectangular eye opens in the centre. Top + bottom
        // lids retract over 3 sub-frames in this same display.
        const int eyeW = 64, eyeH = 40;
        const int eyeX = (OLED_W - eyeW) / 2;
        const int eyeY = (OLED_H - eyeH) / 2;
        oled.fillRoundRect(eyeX, eyeY, eyeW, eyeH, 8, SH110X_WHITE);
        oled.fillRect(eyeX + eyeW / 2 - 5, eyeY + eyeH / 2 - 5, 10, 10, SH110X_BLACK);
      } else {
        // Side chevrons — two pairs marching in from each edge.
        const int CY = 8;
        oled.fillTriangle(2,  CY + 6, 10, CY + 2, 10, CY + 10, SH110X_WHITE);
        oled.fillTriangle(8,  CY + 6, 16, CY + 2, 16, CY + 10, SH110X_WHITE);
        oled.fillTriangle(OLED_W - 4,  CY + 6, OLED_W - 12, CY + 2, OLED_W - 12, CY + 10, SH110X_WHITE);
        oled.fillTriangle(OLED_W - 10, CY + 6, OLED_W - 18, CY + 2, OLED_W - 18, CY + 10, SH110X_WHITE);

        // K-PATROL banner — text-size 2, 8 chars × 12 px = 96 px wide.
        oled.setTextSize(2);
        oled.setCursor(16, 6);
        oled.print(F("K-PATROL"));

        // Double rule below the banner.
        oled.drawFastHLine(8, 26, OLED_W - 16, SH110X_WHITE);
        oled.drawFastHLine(8, 28, OLED_W - 16, SH110X_WHITE);

        oled.setTextSize(1);
        oled.setCursor(13, 33);
        oled.print(F("PATROL ROBOT V2.2"));

        if (frame >= 2) {
          // Inner separator + author tag — final reveal.
          oled.drawFastHLine(20, 47, OLED_W - 40, SH110X_WHITE);
          oled.setCursor(19, 52);
          oled.print(F("by Vu Dang Khoa"));
        }
      }

      oled.display();
      delay(500);
    }
    delay(1500);
  } else {
    Serial.printf("OLED:ok=0,addr=0x%02X\n", OLED_ADDR);
  }
#else
  Serial.println("OLED:disabled,compile_time");
#endif

#if ENABLE_DHT
  dht.begin();
  Serial.println("DHT:init=ok");
#else
  Serial.println("DHT:disabled,compile_time");
  dht_disabled = true;   // ensure loop() never tries to read
#endif

  // Task watchdog — last line of defence. If any blocking call (OLED draw on
  // a stuck I²C bus, an Adafruit DHT read with the sensor unplugged, etc.)
  // takes longer than TASK_WDT_TIMEOUT_S, the chip resets cleanly instead of
  // sitting in a frozen state silently flashing the lights and laser.
  //
  // arduino-esp32 2.x → ESP-IDF 4.x signature: (timeout_s, panic)
  // arduino-esp32 3.x → ESP-IDF 5.x signature: (const esp_task_wdt_config_t*)
  // The IDF version macro lets us compile cleanly against both cores.
#if ESP_IDF_VERSION_MAJOR >= 5
  const esp_task_wdt_config_t twdt_cfg = {
    .timeout_ms     = TASK_WDT_TIMEOUT_S * 1000u,
    .idle_core_mask = 0,            // don't monitor IDLE tasks here
    .trigger_panic  = true,         // panic + reset on timeout
  };
  // reconfigure if it was already initialised by the Arduino runtime,
  // otherwise initialise from scratch.
  if (esp_task_wdt_reconfigure(&twdt_cfg) != ESP_OK) {
    esp_task_wdt_init(&twdt_cfg);
  }
#else
  esp_task_wdt_init(TASK_WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);                        // subscribe loopTask

  uint32_t now0 = millis();
  t_last_cmd        = now0;
  t_last_dht        = now0;   // first DHT read at t+DHT_PERIOD_MS — DHT11
                              // needs ≥1 s power-on settle anyway
  t_last_hb         = now0;
  t_last_oled       = now0;
  t_last_state_emit = now0;
  t_dht_recover     = now0;
  t_last_heap       = now0;
  t_last_dht_disabled_emit = now0;

  Serial.printf("HEAP:%u\n", (unsigned)ESP.getFreeHeap());
}

// ─── Loop ──────────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // 0) Pet the task watchdog. If anything below this line hangs longer than
  //    TASK_WDT_TIMEOUT_S the chip resets, dropping outputs to their safe
  //    boot state instead of leaving the light/horn/laser stuck ON.
  esp_task_wdt_reset();

  // 1) Always service commands first.
  poll_serial();

  // 2) Safety watchdog: if anything is armed and we haven't heard from the Pi
  //    in WATCHDOG_MS, force everything OFF.
  if (watchdog_armed && (now - t_last_cmd >= WATCHDOG_MS)) {
    force_all_off("uart_silent_5s");
  }

  // 3) DHT read every 5 s (don't read too fast — DHT11 needs ≥2 s).
  //    If the sensor is missing or wired wrong the Adafruit library will
  //    spin with interrupts disabled for ~250 ms per read; doing that on
  //    every cycle eventually trips the Interrupt WDT. Bail out after
  //    DHT_FAIL_THRESHOLD consecutive NaN reads and only re-probe once
  //    every DHT_RECOVER_PERIOD_MS.
#if ENABLE_DHT
  if (!dht_disabled) {
    if (now - t_last_dht >= DHT_PERIOD_MS) {
      t_last_dht = now;
      float t = dht.readTemperature();
      float h = dht.readHumidity();
      last_temp = t;
      last_humi = h;
      if (isnan(t) || isnan(h)) {
        if (dht_fail_count < 255) dht_fail_count++;
        if (dht_fail_count >= DHT_FAIL_THRESHOLD) {
          dht_disabled  = true;
          t_dht_recover = now;
          t_last_dht_disabled_emit = now;
          Serial.printf("DHT:disabled,fails=%u,retry_in=%lums\n",
                        (unsigned)dht_fail_count,
                        (unsigned long)DHT_RECOVER_PERIOD_MS);
        }
      } else {
        dht_fail_count = 0;
      }
      emit_dht();
    }
  } else if (now - t_dht_recover >= DHT_RECOVER_PERIOD_MS) {
    // Re-probe: one attempt, then back to disabled if it still fails.
    t_dht_recover = now;
    t_last_dht    = now;
    float t = dht.readTemperature();
    float h = dht.readHumidity();
    last_temp = t;
    last_humi = h;
    if (!isnan(t) && !isnan(h)) {
      dht_disabled   = false;
      dht_fail_count = 0;
      Serial.println("DHT:recovered");
    }
    emit_dht();
  }
#endif

  // 3b) Keep emitting a DHT line even while disabled (sensor fault OR
  //     compile-time off) so the Pi can distinguish "no temperature reading"
  //     from "UART link dead". Cadence matches the normal 5 s.
  if (dht_disabled && (now - t_last_dht_disabled_emit >= DHT_PERIOD_MS)) {
    t_last_dht_disabled_emit = now;
    Serial.println("DHT:disabled");
  }

  // 4) Heartbeat every 1 s.
  if (now - t_last_hb >= HB_PERIOD_MS) {
    t_last_hb = now;
    Serial.printf("HB:t=%lu\n", (unsigned long)now);
  }

  // 5) Periodic STATE re-emit (in case the Pi missed an edge).
  if (now - t_last_state_emit >= STATE_PERIOD_MS) {
    t_last_state_emit = now;
    emit_state();
  }

  // 6) Redraw OLED every 1 s.
  if (oled_ok && (now - t_last_oled >= OLED_PERIOD_MS)) {
    t_last_oled = now;
    draw_oled();
  }

  // 7) Free-heap probe every 30 s. Steady-state heap should be constant
  //    after boot; a downward trend means we're leaking (the original
  //    String-based parser did exactly that — keep watching).
  if (now - t_last_heap >= HEAP_PERIOD_MS) {
    t_last_heap = now;
    Serial.printf("HEAP:%u\n", (unsigned)ESP.getFreeHeap());
  }

  // 8) Yield to the FreeRTOS IDLE task. Without this, a busy loop() on a
  //    single-core slice can starve IDLE long enough to trip the Interrupt
  //    WDT (default 300 ms). 2 ms is plenty without blunting responsiveness.
  delay(2);
}
