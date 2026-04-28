/**
 * ============================================================================
 * TEST BNO08x IMU - SIMPLE I2C MODE
 * ============================================================================
 * 
 * Hardware: Adafruit BNO08x + ESP32
 * 
 * WIRING (I2C - simplest):
 * ========================
 *   BNO08x    →    ESP32
 *   ------         -----
 *   VIN       →    3.3V (hoặc 5V nếu dùng Adafruit board)
 *   GND       →    GND
 *   SDA       →    GPIO21 (I2C SDA)
 *   SCL       →    GPIO22 (I2C SCL)
 *   RST       →    3.3V (hoặc GPIO để có thể reset)
 *   INT       →    (không cần nối, optional)
 *   P0        →    GND hoặc không nối (I2C mode)
 *   P1        →    GND hoặc không nối (I2C mode)
 * 
 * LƯUÝ:
 * - Adafruit BNO08x board có voltage regulator, dùng 5V OK
 * - P0=LOW, P1=LOW = I2C mode (default)
 * - I2C address mặc định: 0x4A (hoặc 0x4B nếu ADR jumper)
 * 
 * INSTALL LIBRARY:
 * - Arduino IDE → Tools → Manage Libraries
 * - Search: "Adafruit BNO08x"
 * - Install "Adafruit BNO08x" (sẽ tự cài dependencies)
 * 
 * ============================================================================
 */

#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ============================================================================
// CONFIGURATION
// ============================================================================

// I2C Pins for ESP32
#define I2C_SDA 21
#define I2C_SCL 22

// BNO08x Reset pin (optional - set to -1 if not connected)
#define BNO08X_RESET -1

// I2C Address (0x4A default, 0x4B if ADR jumper closed)
#define BNO08X_I2C_ADDR 0x4A

// Report interval in microseconds (10ms = 100Hz)
#define REPORT_INTERVAL_US 10000

// ============================================================================
// GLOBALS
// ============================================================================

Adafruit_BNO08x bno08x(BNO08X_RESET);
sh2_SensorValue_t sensorValue;

// Euler angles
float yaw = 0, pitch = 0, roll = 0;

// ============================================================================
// SETUP REPORTS
// ============================================================================
void setReports() {
  Serial.println("Setting desired reports...");
  
  // Rotation Vector (most accurate, uses all sensors)
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("Could not enable rotation vector");
  }
  
  // Accelerometer (optional)
  // if (!bno08x.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US)) {
  //   Serial.println("Could not enable accelerometer");
  // }
  
  // Gyroscope (optional)
  // if (!bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, REPORT_INTERVAL_US)) {
  //   Serial.println("Could not enable gyroscope");
  // }
  
  Serial.println("Reports enabled!");
}

// ============================================================================
// QUATERNION TO EULER CONVERSION
// ============================================================================
void quaternionToEuler(float qr, float qi, float qj, float qk) {
  float sqr = qr * qr;
  float sqi = qi * qi;
  float sqj = qj * qj;
  float sqk = qk * qk;

  // Yaw (Z axis rotation)
  yaw = atan2(2.0 * (qi * qj + qk * qr), (sqi - sqj - sqk + sqr));
  
  // Pitch (Y axis rotation)
  pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqi + sqj + sqk + sqr));
  
  // Roll (X axis rotation)
  roll = atan2(2.0 * (qj * qk + qi * qr), (-sqi - sqj + sqk + sqr));

  // Convert to degrees
  yaw *= RAD_TO_DEG;
  pitch *= RAD_TO_DEG;
  roll *= RAD_TO_DEG;
  
  // Normalize yaw to 0-360
  if (yaw < 0) yaw += 360;
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n============================================");
  Serial.println("   BNO08x IMU Test - I2C Mode");
  Serial.println("============================================\n");
  
  // Initialize I2C
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000); // 400kHz I2C
  
  Serial.println("Scanning I2C bus...");
  scanI2C();
  
  Serial.println("\nInitializing BNO08x...");
  
  // Try to initialize
  if (!bno08x.begin_I2C(BNO08X_I2C_ADDR, &Wire)) {
    Serial.println("ERROR: Failed to find BNO08x chip!");
    Serial.println("\nTroubleshooting:");
    Serial.println("1. Check wiring (SDA=21, SCL=22)");
    Serial.println("2. Check power (3.3V or 5V)");
    Serial.println("3. Try address 0x4B if ADR jumper is closed");
    Serial.println("4. Make sure P0 and P1 are LOW or floating (I2C mode)");
    while (1) { 
      delay(1000);
      Serial.print(".");
    }
  }
  
  Serial.println("BNO08x Found!");
  
  // Print product info
  for (int n = 0; n < bno08x.prodIds.numEntries; n++) {
    Serial.print("Part ");
    Serial.print(bno08x.prodIds.entry[n].swPartNumber);
    Serial.print(": Version ");
    Serial.print(bno08x.prodIds.entry[n].swVersionMajor);
    Serial.print(".");
    Serial.print(bno08x.prodIds.entry[n].swVersionMinor);
    Serial.print(".");
    Serial.println(bno08x.prodIds.entry[n].swVersionPatch);
  }
  
  setReports();
  
  Serial.println("\n============================================");
  Serial.println("   Starting readings... Rotate the sensor!");
  Serial.println("============================================\n");
}

// ============================================================================
// I2C SCANNER
// ============================================================================
void scanI2C() {
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("  Found device at 0x");
      if (addr < 16) Serial.print("0");
      Serial.print(addr, HEX);
      
      // Identify common devices
      if (addr == 0x4A) Serial.print(" <- BNO08x (default)");
      else if (addr == 0x4B) Serial.print(" <- BNO08x (ADR jumper)");
      else if (addr == 0x28 || addr == 0x29) Serial.print(" <- BNO055");
      else if (addr == 0x68 || addr == 0x69) Serial.print(" <- MPU6050/MPU9250");
      else if (addr == 0x70) Serial.print(" <- TCA9548A multiplexer");
      
      Serial.println();
      count++;
    }
  }
  Serial.printf("Found %d I2C device(s)\n", count);
}

// ============================================================================
// MAIN LOOP
// ============================================================================
void loop() {
  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset! Re-enabling reports...");
    setReports();
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      
      case SH2_ROTATION_VECTOR:
        // Convert quaternion to euler angles
        quaternionToEuler(
          sensorValue.un.rotationVector.real,
          sensorValue.un.rotationVector.i,
          sensorValue.un.rotationVector.j,
          sensorValue.un.rotationVector.k
        );
        
        // Print results
        Serial.printf("Yaw: %6.1f°  Pitch: %6.1f°  Roll: %6.1f°  ", yaw, pitch, roll);
        Serial.printf("(Accuracy: %.1f°)\n", 
          sensorValue.un.rotationVector.accuracy * RAD_TO_DEG);
        break;
        
      case SH2_ACCELEROMETER:
        Serial.printf("Accel: X=%6.2f Y=%6.2f Z=%6.2f m/s²\n",
          sensorValue.un.accelerometer.x,
          sensorValue.un.accelerometer.y,
          sensorValue.un.accelerometer.z);
        break;
        
      case SH2_GYROSCOPE_CALIBRATED:
        Serial.printf("Gyro: X=%6.2f Y=%6.2f Z=%6.2f rad/s\n",
          sensorValue.un.gyroscope.x,
          sensorValue.un.gyroscope.y,
          sensorValue.un.gyroscope.z);
        break;
    }
  }
}
