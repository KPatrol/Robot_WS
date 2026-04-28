/*
 * K-Patrol Bot - Test Đơn Giản 1 Motor JGA25 với BTS7960
 * ESP32-S3 WROOM-1
 * 
 * MỤC ĐÍCH: Test 1 motor + 1 driver để đảm bảo hoạt động trước khi đấu 4 motor
 * 
 * PHẦN CỨNG:
 * - ESP32-S3 WROOM-1 Dev Board
 * - BTS7960 Motor Driver (1 cái)
 * - Motor JGA25-370 12V với Encoder
 * - Nguồn 12V (tối thiểu 3A)
 * - Nguồn 5V cho logic
 * 
 * CÁCH SỬ DỤNG:
 * 1. Đấu dây theo sơ đồ bên dưới
 * 2. Nạp code vào ESP32-S3
 * 3. Mở Serial Monitor (115200 baud)
 * 4. Gõ lệnh: F, B, S, +, -, H
 */

#include <Arduino.h>

// ============== CẤU HÌNH GPIO CHO 1 MOTOR ==============
// Sử dụng GPIO tương tự Motor 1 trong hệ thống chính
#define RPWM_PIN    4     // PWM tiến (Forward)
#define LPWM_PIN    5     // PWM lùi (Backward)
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
  // Khởi tạo Serial
  Serial.begin(115200);
  delay(2000);  // Delay để Serial Monitor kịp kết nối
  
  Serial.println("\n\n\n");
  Serial.println("========================================");
  Serial.println("  K-PATROL - TEST 1 MOTOR JGA25");
  Serial.println("  Driver: BTS7960");
  Serial.println("  Controller: ESP32-S3 WROOM-1");
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
  
  // Cấu hình Enable pins
  pinMode(R_EN_PIN, OUTPUT);
  pinMode(L_EN_PIN, OUTPUT);
  
  // Mặc định DISABLE (LOW) - Safe Mode
  digitalWrite(R_EN_PIN, LOW);
  digitalWrite(L_EN_PIN, LOW);
  Serial.println("   ✓ Enable pins configured (DISABLED)");
  
  Serial.println("\n[2/3] Configuring PWM channels...");
  
  // Cấu hình PWM
  ledcSetup(RPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(LPWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  Serial.printf("   ✓ PWM Frequency: %d Hz\n", PWM_FREQ);
  Serial.printf("   ✓ PWM Resolution: %d-bit\n", PWM_RESOLUTION);
  
  Serial.println("\n[3/3] Attaching PWM to pins...");
  
  // Gắn PWM vào GPIO pins
  ledcAttachPin(RPWM_PIN, RPWM_CHANNEL);
  ledcAttachPin(LPWM_PIN, LPWM_CHANNEL);
  Serial.printf("   ✓ RPWM attached to GPIO %d\n", RPWM_PIN);
  Serial.printf("   ✓ LPWM attached to GPIO %d\n", LPWM_PIN);
  
  // Đặt PWM = 0
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);
  
  Serial.println("\n✅ Motor driver initialized successfully!");
}

// ============== BẬT DRIVER (Cấp điện cho H-Bridge) ==============
void enableDriver() {
  digitalWrite(R_EN_PIN, HIGH);
  digitalWrite(L_EN_PIN, HIGH);
  Serial.println("✓ Driver ENABLED");
}

// ============== TẮT DRIVER (Ngắt điện H-Bridge - Motor thả trôi) ==============
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
}

// ============== CHẠY LÙI ==============
void setMotorBackward(int speed) {
  speed = constrain(speed, 0, 255);
  
  enableDriver();
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, speed);
  
  Serial.printf("◀ BACKWARD at speed %d/255\n", speed);
}

// ============== DỪNG (BRAKE - Driver vẫn enable) ==============
void stopMotor() {
  ledcWrite(RPWM_CHANNEL, 0);
  ledcWrite(LPWM_CHANNEL, 0);
  Serial.println("⏹ STOPPED (Brake mode)");
}

// ============== HIỂN THỊ HELP ==============
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
  
  // Lệnh chữ
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
  else if (cmd == "+") {
    currentSpeed = constrain(currentSpeed + 10, 0, 255);
    Serial.printf("Speed increased to %d\n", currentSpeed);
  }
  else if (cmd == "-") {
    currentSpeed = constrain(currentSpeed - 10, 0, 255);
    Serial.printf("Speed decreased to %d\n", currentSpeed);
  }
  // Lệnh số
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
