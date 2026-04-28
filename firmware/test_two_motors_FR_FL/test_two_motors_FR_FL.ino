/*
 * ============================================================================
 * KPatrol Robot - Two Motors Test (Front-Right + Front-Left)
 * ============================================================================
 * 
 * Hardware:
 *   - ESP32-S3 WROOM-1 Dev Board
 *   - 2x BTS7960 (IBT-2) Motor Drivers
 *   - 2x JGA25-370 12V DC Motors with Encoders
 *   - 12V Power Supply (3A minimum, 5A+ recommended)
 * 
 * Date: January 1, 2026
 * Status: Testing Configuration - 2 Motors (FR + FL)
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
 * PWM Configuration:
 *   - Frequency: 5000 Hz (5 kHz)
 *   - Resolution: 8-bit (0-255)
 *   - Library: LEDC (ESP32 built-in)
 * 
 * Serial Commands (115200 baud):
 *   FR_F    : Front-Right Forward
 *   FR_B    : Front-Right Backward
 *   FR_S    : Front-Right Stop
 *   FL_F    : Front-Left Forward
 *   FL_B    : Front-Left Backward
 *   FL_S    : Front-Left Stop
 *   F       : Both Forward (robot forward)
 *   B       : Both Backward (robot backward)
 *   S       : Both Stop
 *   R       : Rotate Right (FR back, FL forward)
 *   L       : Rotate Left (FR forward, FL back)
 *   SPD:xxx : Set speed (0-255)
 *   T       : Run test sequence
 *   I       : Show system info
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
  Serial.println("  KPatrol - Two Motors Test (FR + FL)");
  Serial.println("========================================");
  Serial.println("Initializing motors...");
  
  // ===== FRONT-RIGHT MOTOR SETUP =====
  // Configure PWM channels for FR
  ledcSetup(FR_PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(FR_PWM_CH_L, PWM_FREQ, PWM_RES);
  
  // Attach PWM channels to GPIO pins
  ledcAttachPin(FR_RPWM, FR_PWM_CH_R);
  ledcAttachPin(FR_LPWM, FR_PWM_CH_L);
  
  // Configure enable pins
  pinMode(FR_R_EN, OUTPUT);
  pinMode(FR_L_EN, OUTPUT);
  
  // Enable both directions
  digitalWrite(FR_R_EN, HIGH);
  digitalWrite(FR_L_EN, HIGH);
  
  // Start with motors stopped
  ledcWrite(FR_PWM_CH_R, 0);
  ledcWrite(FR_PWM_CH_L, 0);
  
  Serial.println("✓ Front-Right motor configured");
  
  // ===== FRONT-LEFT MOTOR SETUP =====
  // Configure PWM channels for FL
  ledcSetup(FL_PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcSetup(FL_PWM_CH_L, PWM_FREQ, PWM_RES);
  
  // Attach PWM channels to GPIO pins
  ledcAttachPin(FL_RPWM, FL_PWM_CH_R);
  ledcAttachPin(FL_LPWM, FL_PWM_CH_L);
  
  // Configure enable pins
  pinMode(FL_R_EN, OUTPUT);
  pinMode(FL_L_EN, OUTPUT);
  
  // Enable both directions
  digitalWrite(FL_R_EN, HIGH);
  digitalWrite(FL_L_EN, HIGH);
  
  // Start with motors stopped
  ledcWrite(FL_PWM_CH_R, 0);
  ledcWrite(FL_PWM_CH_L, 0);
  
  Serial.println("✓ Front-Left motor configured");
  
  // System ready
  Serial.println("\n✓ System ready!");
  Serial.println("----------------------------------------");
  printHelp();
}

// ============================================================================
// MOTOR CONTROL FUNCTIONS
// ============================================================================

// ===== FRONT-RIGHT MOTOR FUNCTIONS =====
void FR_forward(int speed) {
  if (FR_INVERTED) {
    ledcWrite(FR_PWM_CH_R, 0);      // RPWM off
    ledcWrite(FR_PWM_CH_L, speed);  // LPWM active (inverted)
  } else {
    ledcWrite(FR_PWM_CH_R, speed);  // RPWM active
    ledcWrite(FR_PWM_CH_L, 0);      // LPWM off
  }
  Serial.print("FR Forward @ speed ");
  Serial.println(speed);
}

void FR_backward(int speed) {
  if (FR_INVERTED) {
    ledcWrite(FR_PWM_CH_R, speed);  // RPWM active (inverted)
    ledcWrite(FR_PWM_CH_L, 0);      // LPWM off
  } else {
    ledcWrite(FR_PWM_CH_R, 0);      // RPWM off
    ledcWrite(FR_PWM_CH_L, speed);  // LPWM active
  }
  Serial.print("FR Backward @ speed ");
  Serial.println(speed);
}

void FR_stop() {
  ledcWrite(FR_PWM_CH_R, 0);
  ledcWrite(FR_PWM_CH_L, 0);
  Serial.println("FR Stopped");
}

// ===== FRONT-LEFT MOTOR FUNCTIONS =====
void FL_forward(int speed) {
  if (FL_INVERTED) {
    ledcWrite(FL_PWM_CH_R, 0);      // RPWM off
    ledcWrite(FL_PWM_CH_L, speed);  // LPWM active (inverted)
  } else {
    ledcWrite(FL_PWM_CH_R, speed);  // RPWM active
    ledcWrite(FL_PWM_CH_L, 0);      // LPWM off
  }
  Serial.print("FL Forward @ speed ");
  Serial.println(speed);
}

void FL_backward(int speed) {
  if (FL_INVERTED) {
    ledcWrite(FL_PWM_CH_R, speed);  // RPWM active (inverted)
    ledcWrite(FL_PWM_CH_L, 0);      // LPWM off
  } else {
    ledcWrite(FL_PWM_CH_R, 0);      // RPWM off
    ledcWrite(FL_PWM_CH_L, speed);  // LPWM active
  }
  Serial.print("FL Backward @ speed ");
  Serial.println(speed);
}

void FL_stop() {
  ledcWrite(FL_PWM_CH_R, 0);
  ledcWrite(FL_PWM_CH_L, 0);
  Serial.println("FL Stopped");
}

// ===== COMBINED MOTOR FUNCTIONS =====
void both_forward(int speed) {
  Serial.println("\n>>> ROBOT FORWARD <<<");
  FR_forward(speed);
  FL_forward(speed);
}

void both_backward(int speed) {
  Serial.println("\n>>> ROBOT BACKWARD <<<");
  FR_backward(speed);
  FL_backward(speed);
}

void both_stop() {
  Serial.println("\n>>> ROBOT STOP <<<");
  FR_stop();
  FL_stop();
}

void rotate_right(int speed) {
  Serial.println("\n>>> ROTATE RIGHT <<<");
  FR_backward(speed);  // Right side backward
  FL_forward(speed);   // Left side forward
}

void rotate_left(int speed) {
  Serial.println("\n>>> ROTATE LEFT <<<");
  FR_forward(speed);   // Right side forward
  FL_backward(speed);  // Left side backward
}

// ============================================================================
// SAFE DIRECTION CHANGE (with delay to prevent H-bridge damage)
// ============================================================================
void safeDirectionChange() {
  both_stop();
  delay(200);  // 200ms brake before reversing
}

// ============================================================================
// HELP MENU
// ============================================================================
void printHelp() {
  Serial.println("Commands:");
  Serial.println("  Individual Motors:");
  Serial.println("    FR_F   : Front-Right Forward");
  Serial.println("    FR_B   : Front-Right Backward");
  Serial.println("    FR_S   : Front-Right Stop");
  Serial.println("    FL_F   : Front-Left Forward");
  Serial.println("    FL_B   : Front-Left Backward");
  Serial.println("    FL_S   : Front-Left Stop");
  Serial.println("");
  Serial.println("  Both Motors:");
  Serial.println("    F      : Both Forward (robot forward)");
  Serial.println("    B      : Both Backward (robot backward)");
  Serial.println("    S      : Both Stop");
  Serial.println("");
  Serial.println("  Rotation:");
  Serial.println("    R      : Rotate Right (FR back, FL forward)");
  Serial.println("    L      : Rotate Left (FR forward, FL back)");
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
  Serial.println("  SYSTEM INFORMATION");
  Serial.println("========================================");
  Serial.println("Hardware:");
  Serial.println("  - ESP32-S3 WROOM-1");
  Serial.println("  - 2x BTS7960 Motor Drivers");
  Serial.println("  - 2x JGA25-370 12V Motors");
  Serial.println("");
  Serial.println("GPIO Mapping:");
  Serial.println("  Front-Right (FR):");
  Serial.print("    RPWM: GPIO ");
  Serial.print(FR_RPWM);
  Serial.print(" (PWM Ch ");
  Serial.print(FR_PWM_CH_R);
  Serial.println(")");
  Serial.print("    LPWM: GPIO ");
  Serial.print(FR_LPWM);
  Serial.print(" (PWM Ch ");
  Serial.print(FR_PWM_CH_L);
  Serial.println(")");
  Serial.print("    R_EN: GPIO ");
  Serial.println(FR_R_EN);
  Serial.print("    L_EN: GPIO ");
  Serial.println(FR_L_EN);
  Serial.println("");
  Serial.println("  Front-Left (FL):");
  Serial.print("    RPWM: GPIO ");
  Serial.print(FL_RPWM);
  Serial.print(" (PWM Ch ");
  Serial.print(FL_PWM_CH_R);
  Serial.println(")");
  Serial.print("    LPWM: GPIO ");
  Serial.print(FL_LPWM);
  Serial.print(" (PWM Ch ");
  Serial.print(FL_PWM_CH_L);
  Serial.println(")");
  Serial.print("    R_EN: GPIO ");
  Serial.println(FL_R_EN);
  Serial.print("    L_EN: GPIO ");
  Serial.println(FL_L_EN);
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
  Serial.println("  AUTOMATED TEST SEQUENCE");
  Serial.println("========================================");
  
  int testSpeed = 150;  // Medium speed for testing
  
  // Test 1: Individual motor forward
  Serial.println("\nTest 1: FR Forward only");
  FR_forward(testSpeed);
  delay(2000);
  FR_stop();
  delay(1000);
  
  Serial.println("\nTest 2: FL Forward only");
  FL_forward(testSpeed);
  delay(2000);
  FL_stop();
  delay(1000);
  
  // Test 2: Individual motor backward
  Serial.println("\nTest 3: FR Backward only");
  FR_backward(testSpeed);
  delay(2000);
  FR_stop();
  delay(1000);
  
  Serial.println("\nTest 4: FL Backward only");
  FL_backward(testSpeed);
  delay(2000);
  FL_stop();
  delay(1000);
  
  // Test 3: Both motors forward
  Serial.println("\nTest 5: Both Forward (Robot Forward)");
  both_forward(testSpeed);
  delay(2000);
  both_stop();
  delay(1000);
  
  // Test 4: Both motors backward
  Serial.println("\nTest 6: Both Backward (Robot Backward)");
  both_backward(testSpeed);
  delay(2000);
  both_stop();
  delay(1000);
  
  // Test 5: Rotate right
  Serial.println("\nTest 7: Rotate Right");
  rotate_right(testSpeed);
  delay(2000);
  both_stop();
  delay(1000);
  
  // Test 6: Rotate left
  Serial.println("\nTest 8: Rotate Left");
  rotate_left(testSpeed);
  delay(2000);
  both_stop();
  delay(1000);
  
  // Test 7: Speed ramp
  Serial.println("\nTest 9: Speed Ramp (50 → 255)");
  for(int spd = 50; spd <= 255; spd += 50) {
    Serial.print("Speed: ");
    Serial.println(spd);
    both_forward(spd);
    delay(1000);
  }
  both_stop();
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
    
    // Individual motor commands
    if (command == "FR_F") {
      FR_forward(currentSpeed);
    }
    else if (command == "FR_B") {
      FR_backward(currentSpeed);
    }
    else if (command == "FR_S") {
      FR_stop();
    }
    else if (command == "FL_F") {
      FL_forward(currentSpeed);
    }
    else if (command == "FL_B") {
      FL_backward(currentSpeed);
    }
    else if (command == "FL_S") {
      FL_stop();
    }
    
    // Both motors commands
    else if (command == "F") {
      both_forward(currentSpeed);
    }
    else if (command == "B") {
      both_backward(currentSpeed);
    }
    else if (command == "S") {
      both_stop();
    }
    
    // Rotation commands
    else if (command == "R") {
      rotate_right(currentSpeed);
    }
    else if (command == "L") {
      rotate_left(currentSpeed);
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
  
  // Optional: Auto-stop after 30 seconds of no commands (safety feature)
  // Uncomment if needed:
  /*
  if (millis() - lastCommandTime > 30000) {
    both_stop();
    lastCommandTime = millis();
    Serial.println("⚠ Auto-stop: No commands for 30s");
  }
  */
}
