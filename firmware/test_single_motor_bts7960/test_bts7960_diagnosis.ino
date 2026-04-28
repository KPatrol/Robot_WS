/*
 * BTS7960 Diagnosis Tool - PWM Version
 * Kiểm tra từng chân với PWM đúng cách
 */

// Test GPIO 4 và GPIO 15
#define TEST_PIN_1    4
#define TEST_PIN_2    15
#define R_EN_PIN      6
#define L_EN_PIN      7

// PWM Config
#define PWM_FREQ      5000
#define PWM_RES       8
#define PWM_CH_1      0
#define PWM_CH_2      1
#define TEST_SPEED    200  // 0-255

void setup() {
  Serial.begin(115200);
  delay(2000);
  
  Serial.println("\n\n=== BTS7960 DIAGNOSIS TOOL (PWM) ===\n");
  
  // Setup Enable pins
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  
  // Setup PWM
  ledcSetup(PWM_CH_1, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_2, PWM_FREQ, PWM_RES);
  ledcAttachPin(TEST_PIN_1, PWM_CH_1);
  ledcAttachPin(TEST_PIN_2, PWM_CH_2);
  
  Serial.println("✓ Driver ENABLED (R_EN=HIGH, L_EN=HIGH)");
  Serial.printf("✓ PWM configured: %dHz, %d-bit, Speed=%d\n", PWM_FREQ, PWM_RES, TEST_SPEED);
  Serial.println("✓ Starting automatic tests...\n");
}

void loop() {
  // Test 1: PWM on GPIO 4, GPIO 15 = 0
  Serial.println("─────────────────────────────────");
  Serial.println("TEST 1: GPIO 4 PWM, GPIO 15 = 0");
  Serial.println("Expected: Motor rotates FORWARD");
  ledcWrite(PWM_CH_1, TEST_SPEED);
  ledcWrite(PWM_CH_2, 0);
  delay(3000);
  
  // Stop
  Serial.println("→ STOP");
  ledcWrite(PWM_CH_1, 0);
  ledcWrite(PWM_CH_2, 0);
  delay(2000);
  
  // Test 2: GPIO 4 = 0, PWM on GPIO 15
  Serial.println("─────────────────────────────────");
  Serial.println("TEST 2: GPIO 4 = 0, GPIO 15 PWM");
  Serial.println("Expected: Motor rotates BACKWARD (opposite)");
  ledcWrite(PWM_CH_1, 0);
  ledcWrite(PWM_CH_2, TEST_SPEED);
  delay(3000);
  
  // Stop
  Serial.println("→ STOP");
  ledcWrite(PWM_CH_1, 0);
  ledcWrite(PWM_CH_2, 0);
  delay(2000);
  
  // Test 3: Different speeds
  Serial.println("─────────────────────────────────");
  Serial.println("TEST 3: Speed ramp test (GPIO 4)");
  Serial.println("Expected: Motor speed increases");
  for(int spd = 50; spd <= 250; spd += 50) {
    Serial.printf("→ Speed: %d\n", spd);
    ledcWrite(PWM_CH_1, spd);
    ledcWrite(PWM_CH_2, 0);
    delay(1500);
  }
  
  // Stop
  Serial.println("→ STOP");
  ledcWrite(PWM_CH_1, 0);
  ledcWrite(PWM_CH_2, 0);
  delay(3000);
  
  Serial.println("\n=== CYCLE COMPLETE ===");
  Serial.println("Report results:");
  Serial.println("  - TEST 1: Motor rotated? (Y/N)");
  Serial.println("  - TEST 2: Motor rotated? Same/Opposite direction?");
  Serial.println("  - TEST 3: Speed increased gradually?");
  Serial.println("\nWaiting 5 seconds before next cycle...\n");
  delay(5000);
}
