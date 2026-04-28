/*
 * K-Patrol Bot - Test Đơn Giản 1 Motor JGA25 với BTS7960
 * ESP32-S3 WROOM-1 - UART Version
 * 
 * VERSION NÀY DÙNG UART THAY VÌ USB CDC
 * Nếu USB CDC không hoạt động, dùng version này
 */

#include <Arduino.h>

// ============== CẤU HÌNH GPIO CHO 1 MOTOR ==============
#define RPWM_PIN    4     // PWM tiến (Forward)
#define LPWM_PIN    15    // PWM lùi (Backward) - ĐỔI TỪ 5 SANG 15
#define R_EN_PIN    6     // Enable kênh tiến
#define L_EN_PIN    7     // Enable kênh lùi

// PWM Configuration
#define PWM_FREQ        5000    // 5kHz
#define PWM_RESOLUTION  8       // 8-bit (0-255)
#define RPWM_CHANNEL    0       // LEDC Channel cho RPWM
#define LPWM_CHANNEL    1       // LEDC Channel cho LPWM

// Tốc độ mặc định
int currentSpeed = 150;  // 0-255

// ============== KHAI BÁO HÀM ==============
void setupMotor();
void enableDriver();
void disableDriver();
void setMotorForward(int speed);
void setMotorBackward(int speed);
void stopMotor();
void printHelp();

// ============== SETUP ==============
void setup() {
  // Thử nhiều baud rate để debug
  Serial.begin(115200);
  delay(100);
  
  // Gửi nhiều loại ký tự để dễ nhận diện
  for(int i = 0; i < 5; i++) {
    Serial.println("START");
    Serial.println("KPATROL");
    Serial.println("115200");
    delay(200);
  }
  
  delay(1000);
  
  Serial.println("\n\n\n");
  Serial.println("========================================");
  Serial.println("  K-PATROL - TEST 1 MOTOR JGA25");
  Serial.println("  Driver: BTS7960");
  Serial.println("  Controller: ESP32-S3 WROOM-1");
  Serial.println("  Baud: 115200");
  Serial.println("========================================");
  
  setupMotor();
  
  Serial.println("\n✅ System Ready!");
  Serial.println("⚠️  WARNING: Motor is DISABLED by default (Safe Mode)");
  Serial.println("   Use 'F' to test Forward, 'B' for Backward");
  Serial.println("");
  
  printHelp();
}

// ============== KHỞI TẠO MOTOR ==============
void setupMotor() {
  Serial.println("\n[1/3] Configuring GPIO pins...");
  
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
  Serial.println("   ✓ Enable pins configured (DISABLED)");
  
  Serial.println("\n[2/3] Configuring PWM channels...");
  
  ledcSetup(RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  Serial.printf("   ✓ PWM Frequency: %d Hz\n", PWM_FREQ);
  Serial.printf("   ✓ PWM Resolution: %d-bit\n", PWM_RESOLUTION);
  
  Serial.println("\n[3/3] Attaching PWM to pins...");
  
  ledcAttachPin(RPWM_PIN, RPWM_CHANNEL);
  ledcAttachPin(LPWM_PIN, LPWM_CHANNEL);
  Serial.printf("   ✓ RPWM attached to GPIO %d\n", RPWM_PIN);
  Serial.printf("   ✓ LPWM attached to GPIO %d\n", LPWM_PIN);
  
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);
  
  Serial.println("\n✅ Motor driver initialized successfully!");
}

// ============== BẬT DRIVER ==============
void enableDriver() {
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  Serial.println("✓ Driver ENABLED");
}

// ============== TẮT DRIVER ==============
void disableDriver() {
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);
  Serial.println("✓ Driver DISABLED (Motor coasting)");
}

// ============== CHẠY TIẾN ==============
void setMotorForward(int speed) {
  speed = constrain(speed, 0, 255);
  
  enableDriver();
  ledcWrite(RPWM_CHANNEL, speed);
  ledcWrite(LPWM_CHANNEL, 0);
  
  Serial.printf("▶ FORWARD at speed %d/255\n", speed);
  Serial.printf("   RPWM=%d, LPWM=0, R_EN=HIGH, L_EN=HIGH\n", speed);
}

// ============== CHẠY LÙI ==============
void setMotorBackward(int speed) {
  speed = constrain(speed, 0, 255);
  
  // WORKAROUND: Vì LPWM không hoạt động
  // Tạm thời báo user cần đảo dây motor
  
  Serial.printf("◀ BACKWARD at speed %d/255\n", speed);
  Serial.println("⚠️  LPWM CHANNEL NOT WORKING!");
  Serial.println("   Possible causes:");
  Serial.println("   1. BTS7960 LPWM pin broken");
  Serial.println("   2. Wire not connected to LPWM");
  Serial.println("   3. Motor can only rotate one direction");
  Serial.println("");
  Serial.println("💡 SOLUTION:");
  Serial.println("   Option 1: Swap motor wires (M+ ↔ M-) at BTS7960");
  Serial.println("   Option 2: Try different BTS7960 driver");
  Serial.println("   Option 3: Check if motor is uni-directional");
}

// ============== DỪNG ==============
void stopMotor() {
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);
  Serial.println("⏹ STOPPED (Brake mode)");
}

// ============== HELP ==============
void printHelp() {
  Serial.println("┌─────────────────────────────────────────┐");
  Serial.println("│           AVAILABLE COMMANDS            │");
  Serial.println("├─────────────────────────────────────────┤");
  Serial.println("│  F       → Forward (Tiến)               │");
  Serial.println("│  B       → Backward (Lùi)               │");
  Serial.println("│  S       → Stop (Dừng - Brake)          │");
  Serial.println("│  OFF     → Disable driver (Thả trôi)    │");
  Serial.println("│  +       → Tăng tốc độ (+10)            │");
  Serial.println("│  -       → Giảm tốc độ (-10)            │");
  Serial.println("│  100     → Đặt tốc độ cụ thể (0-255)    │");
  Serial.println("│  TR      → Test RPWM only (Forward)     │");
  Serial.println("│  TL      → Test LPWM only (Backward)    │");
  Serial.println("│  H       → Hiển thị trợ giúp            │");
  Serial.println("└─────────────────────────────────────────┘");
  Serial.printf("Current speed: %d/255\n", currentSpeed);
  Serial.println("");
}

// ============== XỬ LÝ LỆNH ==============
void processCommand(String cmd) {
  cmd.trim();
  cmd.toUpperCase();
  
  if (cmd.length() == 0) return;
  
  Serial.println("──────────────────────────────────");
  Serial.printf("Command: %s\n", cmd.c_str());
  
  if (cmd == "F" || cmd == "FORWARD") {
    setMotorForward(currentSpeed);
  }
  else if (cmd == "B" || cmd == "BACKWARD") {
    setMotorBackward(currentSpeed);
  }
  else if (cmd == "S" || cmd == "STOP") {
    stopMotor();
  }
  else if (cmd == "OFF" || cmd == "DISABLE") {
    disableDriver();
  }
  else if (cmd == "H" || cmd == "HELP") {
    printHelp();
  }
  else if (cmd == "TR") {
    // Test RPWM only (Forward channel)
    enableDriver();
    ledcWrite(RPWM_CHANNEL, currentSpeed);
    ledcWrite(LPWM_CHANNEL, 0);
    Serial.printf("🔧 TEST RPWM: speed=%d, LPWM=0\n", currentSpeed);
    Serial.println("   (GPIO 4 should have PWM signal)");
  }
  else if (cmd == "TL") {
    // Test LPWM only (Backward channel)
    enableDriver();
    ledcWrite(RPWM_CHANNEL, 0);
    ledcWrite(LPWM_CHANNEL, currentSpeed);
    Serial.printf("🔧 TEST LPWM: speed=%d, RPWM=0\n", currentSpeed);
    Serial.printf("   GPIO %d should have PWM signal\n", LPWM_PIN);
    
    // Test blink to verify GPIO works
    Serial.println("   Testing GPIO with blink...");
    for(int i = 0; i < 5; i++) {
      digitalWrite(LPWM_PIN, HIGH);
      delay(100);
      digitalWrite(LPWM_PIN, LOW);
      delay(100);
    }
    Serial.println("   Blink test done. Restoring PWM...");
    ledcAttachPin(LPWM_PIN, LPWM_CHANNEL);
    ledcWrite(LPWM_CHANNEL, currentSpeed);
  }
  else if (cmd == "+") {
    currentSpeed = constrain(currentSpeed + 10, 0, 255);
    Serial.printf("Speed increased to %d\n", currentSpeed);
  }
  else if (cmd == "-") {
    currentSpeed = constrain(currentSpeed - 10, 0, 255);
    Serial.printf("Speed decreased to %d\n", currentSpeed);
  }
  else if (cmd.toInt() > 0 || cmd == "0") {
    int newSpeed = cmd.toInt();
    if (newSpeed >= 0 && newSpeed <= 255) {
      currentSpeed = newSpeed;
      Serial.printf("Speed set to %d\n", currentSpeed);
    } else {
      Serial.println("❌ Invalid speed! (0-255)");
    }
  }
  else {
    Serial.println("❌ Unknown command! Type 'H' for help");
  }
  
  Serial.println("──────────────────────────────────\n");
}

// ============== MAIN LOOP ==============
void loop() {
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    processCommand(cmd);
  }
}
