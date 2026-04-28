/*
 * ============================================================================
 * KPatrol Robot - Encoder Reader for 4 Motors
 * ============================================================================
 * 
 * Hardware:
 *   - ESP32 Dev Module (standard, NOT ESP32-S3)
 *   - 4x JGA25-370 12V DC Motors with Encoders
 * 
 * Date: January 1, 2026
 * Purpose: Read encoder signals from all 4 motors continuously
 * 
 * Motor Encoder Pinout (6-pin connector):
 *   Pin 1: M1 Motor -     (BLACK - thick) → BTS7960 M-
 *   Pin 2: GND Encoder    (BLACK - thin)  → ESP32 GND
 *   Pin 3: Fase A Encoder (YELLOW)        → ESP32 GPIO (interrupt)
 *   Pin 4: Fase B Encoder (GREEN)         → ESP32 GPIO (interrupt)
 *   Pin 5: 3.3V/5V Encoder+ (RED - thin)  → ESP32 3.3V
 *   Pin 6: M1 Motor +     (RED - thick)   → BTS7960 M+
 * 
 * GPIO Allocation for Encoder Reading:
 *   Front-Right (FR):
 *     GPIO 34 → FR Encoder A (Fase A - YELLOW)
 *     GPIO 35 → FR Encoder B (Fase B - GREEN)
 * 
 *   Front-Left (FL):
 *     GPIO 32 → FL Encoder A (Fase A - YELLOW)
 *     GPIO 33 → FL Encoder B (Fase B - GREEN)
 * 
 *   Back-Right (BR):
 *     GPIO 25 → BR Encoder A (Fase A - YELLOW)
 *     GPIO 26 → BR Encoder B (Fase B - GREEN)
 * 
 *   Back-Left (BL):
 *     GPIO 27 → BL Encoder A (Fase A - YELLOW)
 *     GPIO 14 → BL Encoder B (Fase B - GREEN)
 * 
 * Encoder Specifications (JGA25-370):
 *   - PPR (Pulses Per Revolution): 11 PPR (encoder disk)
 *   - Gear Ratio: 1:34
 *   - Effective PPR at output shaft: 11 * 34 = 374 PPR
 *   - With quadrature (x4): 374 * 4 = 1496 counts per revolution
 * 
 * Arduino IDE Settings:
 *   - Board: "ESP32 Dev Module"
 *   - Upload Speed: 921600
 *   - Serial Baud: 115200
 * 
 * ============================================================================
 */

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
  Serial.println("  KPatrol - Encoder Reader (4 Motors)");
  Serial.println("================================================");
  Serial.println("Initializing encoders...\n");
  
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
  
  Serial.println("✓ Front-Right encoder: GPIO 34 (A), GPIO 35 (B)");
  Serial.println("✓ Front-Left encoder:  GPIO 32 (A), GPIO 33 (B)");
  Serial.println("✓ Back-Right encoder:  GPIO 25 (A), GPIO 26 (B)");
  Serial.println("✓ Back-Left encoder:   GPIO 27 (A), GPIO 14 (B)");
  Serial.println("");
  
  printWiringGuide();
  
  Serial.println("\n✓ Encoder reader ready!");
  Serial.println("================================================");
  Serial.println("Waiting for motor movement...\n");
  
  lastPrintTime = millis();
  lastSpeedCalcTime = millis();
}

// ============================================================================
// PRINT WIRING GUIDE
// ============================================================================
void printWiringGuide() {
  Serial.println("================================================");
  Serial.println("  ENCODER WIRING GUIDE");
  Serial.println("================================================");
  Serial.println("");
  Serial.println("Motor Connector (6 pins, top to bottom):");
  Serial.println("  Pin 1: M1 Motor -  (BLACK thick) → BTS7960 M-");
  Serial.println("  Pin 2: GND Encoder (BLACK thin)  → ESP32 GND");
  Serial.println("  Pin 3: Fase A      (YELLOW)      → ESP32 GPIO");
  Serial.println("  Pin 4: Fase B      (GREEN)       → ESP32 GPIO");
  Serial.println("  Pin 5: VCC Encoder (RED thin)    → ESP32 3.3V");
  Serial.println("  Pin 6: M1 Motor +  (RED thick)   → BTS7960 M+");
  Serial.println("");
  Serial.println("ESP32 Connections:");
  Serial.println("┌─────────┬──────────┬──────────┬─────────────┐");
  Serial.println("│ Motor   │ Fase A   │ Fase B   │ Wire Colors │");
  Serial.println("├─────────┼──────────┼──────────┼─────────────┤");
  Serial.println("│ FR      │ GPIO 34  │ GPIO 35  │ YEL, GRN    │");
  Serial.println("│ FL      │ GPIO 32  │ GPIO 33  │ YEL, GRN    │");
  Serial.println("│ BR      │ GPIO 25  │ GPIO 26  │ YEL, GRN    │");
  Serial.println("│ BL      │ GPIO 27  │ GPIO 14  │ YEL, GRN    │");
  Serial.println("└─────────┴──────────┴──────────┴─────────────┘");
  Serial.println("");
  Serial.println("Power Connections (ALL 4 encoders):");
  Serial.println("  VCC (RED thin)  → ESP32 3.3V (shared)");
  Serial.println("  GND (BLACK thin)→ ESP32 GND (shared)");
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
// PRINT ENCODER DATA
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
  
  Serial.println("────────────────────────────────────────────────");
  Serial.print("Time: ");
  Serial.print(millis() / 1000.0, 1);
  Serial.println(" sec");
  Serial.println("");
  
  // FR
  Serial.print("FR: Count=");
  Serial.print(fr_cnt);
  Serial.print("  Rev=");
  Serial.print(fr_rev, 2);
  Serial.print("  RPM=");
  Serial.println(FR_rpm, 1);
  
  // FL
  Serial.print("FL: Count=");
  Serial.print(fl_cnt);
  Serial.print("  Rev=");
  Serial.print(fl_rev, 2);
  Serial.print("  RPM=");
  Serial.println(FL_rpm, 1);
  
  // BR
  Serial.print("BR: Count=");
  Serial.print(br_cnt);
  Serial.print("  Rev=");
  Serial.print(br_rev, 2);
  Serial.print("  RPM=");
  Serial.println(BR_rpm, 1);
  
  // BL
  Serial.print("BL: Count=");
  Serial.print(bl_cnt);
  Serial.print("  Rev=");
  Serial.print(bl_rev, 2);
  Serial.print("  RPM=");
  Serial.println(BL_rpm, 1);
  
  Serial.println("");
}

// ============================================================================
// PRINT COMPACT DATA (for plotting)
// ============================================================================
void printCompactData() {
  noInterrupts();
  long fr_cnt = FR_count;
  long fl_cnt = FL_count;
  long br_cnt = BR_count;
  long bl_cnt = BL_count;
  interrupts();
  
  // Format: FR_cnt,FL_cnt,BR_cnt,BL_cnt,FR_rpm,FL_rpm,BR_rpm,BL_rpm
  Serial.print(fr_cnt);
  Serial.print(",");
  Serial.print(fl_cnt);
  Serial.print(",");
  Serial.print(br_cnt);
  Serial.print(",");
  Serial.print(bl_cnt);
  Serial.print(",");
  Serial.print(FR_rpm, 1);
  Serial.print(",");
  Serial.print(FL_rpm, 1);
  Serial.print(",");
  Serial.print(BR_rpm, 1);
  Serial.print(",");
  Serial.println(BL_rpm, 1);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  // Calculate speed periodically
  calculateSpeed();
  
  // Print data periodically
  unsigned long currentTime = millis();
  if (currentTime - lastPrintTime >= PRINT_INTERVAL) {
    printEncoderData();
    // Uncomment for compact output (Serial Plotter compatible):
    // printCompactData();
    lastPrintTime = currentTime;
  }
  
  // Check for serial commands
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    
    switch (cmd) {
      case 'r':
      case 'R':
        // Reset all counters
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
        Serial.println("\n✓ All encoder counts reset to 0\n");
        break;
        
      case 'w':
      case 'W':
        // Print wiring guide
        printWiringGuide();
        break;
        
      case 'h':
      case 'H':
        Serial.println("\n================================================");
        Serial.println("  COMMANDS");
        Serial.println("================================================");
        Serial.println("  R - Reset all encoder counts to 0");
        Serial.println("  W - Show wiring guide");
        Serial.println("  H - Show this help");
        Serial.println("================================================\n");
        break;
    }
  }
}
