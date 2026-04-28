/**
 * I2C Scanner - Test cơ bản nhất
 * Không cần thư viện ngoài
 * 
 * Nếu scan thấy 0x4A hoặc 0x4B → BNO08x OK
 * Nếu scan thấy 0 device → Kiểm tra dây hoặc module hỏng
 */

#include <Wire.h>

// Thử nhiều pin I2C khác nhau
#define TEST_SDA 21
#define TEST_SCL 22

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n========================================");
  Serial.println("   I2C SCANNER - DEBUG MODE");
  Serial.println("========================================\n");
  
  // Test với tốc độ thấp hơn
  Wire.begin(TEST_SDA, TEST_SCL);
  Wire.setClock(100000);  // 100kHz (chậm hơn, ổn định hơn)
  
  Serial.printf("Testing I2C on SDA=%d, SCL=%d\n", TEST_SDA, TEST_SCL);
  Serial.println("Clock: 100kHz\n");
}

void loop() {
  Serial.println("Scanning...");
  
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("  FOUND: 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      
      // Identify
      if (addr == 0x4A) Serial.print(" ← BNO08x (default address)");
      else if (addr == 0x4B) Serial.print(" ← BNO08x (alternate address)");
      else if (addr == 0x28) Serial.print(" ← BNO055");
      else if (addr == 0x29) Serial.print(" ← VL53L0X / BNO055");
      else if (addr == 0x68) Serial.print(" ← MPU6050 / DS3231");
      else if (addr == 0x69) Serial.print(" ← MPU6050 (alt)");
      else if (addr == 0x70) Serial.print(" ← TCA9548A");
      else if (addr == 0x76 || addr == 0x77) Serial.print(" ← BME280/BMP280");
      
      Serial.println();
      count++;
    }
    else if (error == 4) {
      Serial.print("  ERROR at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
  
  Serial.println();
  if (count == 0) {
    Serial.println("╔════════════════════════════════════════╗");
    Serial.println("║  NO I2C DEVICES FOUND!                 ║");
    Serial.println("║                                        ║");
    Serial.println("║  Check:                                ║");
    Serial.println("║  1. Power: 3.3V connected?             ║");
    Serial.println("║  2. GND: Common ground?                ║");
    Serial.println("║  3. SDA: Pin 21 on D1 R32              ║");
    Serial.println("║  4. SCL: Pin 22 on D1 R32              ║");
    Serial.println("║  5. Wires: Try different wires         ║");
    Serial.println("║  6. Module: May be defective           ║");
    Serial.println("╚════════════════════════════════════════╝");
  } else {
    Serial.printf("Found %d device(s)\n", count);
    Serial.println("I2C is working!");
  }
  
  Serial.println("\n--- Waiting 3 seconds ---\n");
  delay(3000);
}
