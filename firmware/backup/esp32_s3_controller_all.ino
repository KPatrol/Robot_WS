/*
 * ============================================================================
 * KPatrol Robot - Four Motors Mecanum Wheel Control
 * ============================================================================
 * 
 * Hardware:
 *   - ESP32-S3 WROOM-1 Dev Board
 *   - 4x BTS7960 (IBT-2) Motor Drivers
 *   - 4x JGA25-370 12V DC Motors with Encoders
 *   - 12V Power Supply (7A minimum, 10A+ recommended)
 * 
 * Date: January 1, 2026
 * Status: Full Mecanum Configuration - 4 Motors
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
 *   Speed & System:
 *     SPD:xxx : Set speed (0-255)
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
  Serial.println("  Speed Control:");
  Serial.println("    SPD:xxx : Set speed (0-255), e.g. SPD:150");
  Serial.println("");
  Serial.println("  Testing:");
  Serial.println("    T      : Run automated test sequence");
  Serial.println("    I      : Show system information");
  Serial.println("    H      : Show this help menu");
  Serial.println("----------------------------------------");
  Serial.print("Current Speed: ");
  Serial.println(currentSpeed);
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
  Serial.println("  - Mecanum Wheel Configuration");
  Serial.println("");
  Serial.println("GPIO Mapping:");
  Serial.println("  Front-Right (FR): GPIO 4,5,6,7 (PWM Ch 0,1)");
  Serial.println("  Front-Left  (FL): GPIO 15,16,17,18 (PWM Ch 2,3)");
  Serial.println("  Back-Right  (BR): GPIO 8,9,10,11 (PWM Ch 4,5)");
  Serial.println("  Back-Left   (BL): GPIO 12,13,14,21 (PWM Ch 6,7)");
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
  
  Serial.println("\n✓ Test sequence complete!");
  Serial.println("========================================\n");
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    command.toUpperCase();
    
    Serial.print("\n> Command: ");
    Serial.println(command);
    
    // Individual motor commands - FR
    if (command == "FR_F") {
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
    
    // Unknown command
    else {
      Serial.println("✗ Unknown command. Type 'H' for help.");
    }
    
    lastCommandTime = millis();
  }
}
