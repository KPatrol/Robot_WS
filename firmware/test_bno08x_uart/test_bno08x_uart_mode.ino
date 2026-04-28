/**
 * ============================================================================
 * TEST BNO08x - UART FULL MODE (PS1=HIGH, PS0=LOW)
 * ============================================================================
 * 
 * Board của bạn: PS1 hàn (HIGH), PS0 không hàn (LOW) = UART Full Mode
 * 
 * WIRING:
 * =======
 *   BNO08x     →    D1 R32 (ESP32)
 *   ------          ------
 *   VIN        →    3.3V
 *   GND        →    GND
 *   SDA (TX)   →    GPIO16 (RX2)  ← BNO gửi data ra pin này
 *   SCL (RX)   →    GPIO17 (TX2)  ← ESP32 gửi commands qua pin này
 *   RST        →    GPIO4 (hoặc 3.3V)
 *   INT        →    GPIO5 (optional)
 *   PS0        →    Không nối (đã LOW)
 *   PS1        →    Đã hàn HIGH sẵn
 * 
 * ============================================================================
 */

#include <Adafruit_BNO08x.h>

// UART pins
#define BNO08X_RX  16   // ESP32 RX ← BNO08x TX (SDA pin)
#define BNO08X_TX  17   // ESP32 TX → BNO08x RX (SCL pin)
#define BNO08X_RST 4    // Reset pin (optional, use -1 if not connected)

// Create BNO08x object
Adafruit_BNO08x bno08x(BNO08X_RST);
sh2_SensorValue_t sensorValue;

// Euler angles
float yaw = 0, pitch = 0, roll = 0;

// Report interval (microseconds)
#define REPORT_INTERVAL_US 10000  // 100Hz

void setReports() {
  Serial.println("Enabling sensor reports...");
  
  if (!bno08x.enableReport(SH2_ROTATION_VECTOR, REPORT_INTERVAL_US)) {
    Serial.println("  Could not enable rotation vector");
  } else {
    Serial.println("  Rotation vector: OK");
  }
  
  if (!bno08x.enableReport(SH2_ACCELEROMETER, REPORT_INTERVAL_US)) {
    Serial.println("  Could not enable accelerometer");
  } else {
    Serial.println("  Accelerometer: OK");
  }
}

void quaternionToEuler(float qr, float qi, float qj, float qk) {
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

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);
  
  Serial.println("\n============================================");
  Serial.println("   BNO08x UART Full Mode Test");
  Serial.println("   (PS1=HIGH, PS0=LOW)");
  Serial.println("============================================\n");
  
  Serial.println("WIRING:");
  Serial.println("  VIN → 3.3V");
  Serial.println("  GND → GND");
  Serial.println("  SDA → GPIO16 (RX2)");
  Serial.println("  SCL → GPIO17 (TX2)");
  Serial.println("  RST → GPIO4 (or 3.3V)");
  Serial.println();
  
  // Initialize Serial2 for BNO08x
  Serial2.begin(3000000, SERIAL_8N1, BNO08X_RX, BNO08X_TX);
  
  Serial.println("Initializing BNO08x via UART...");
  
  // Try to initialize with UART
  if (!bno08x.begin_UART(&Serial2)) {
    Serial.println("\n╔════════════════════════════════════════╗");
    Serial.println("║  ERROR: BNO08x not found via UART!     ║");
    Serial.println("╠════════════════════════════════════════╣");
    Serial.println("║  Try:                                  ║");
    Serial.println("║  1. Swap SDA↔SCL wires                 ║");
    Serial.println("║  2. Check power (3.3V)                 ║");
    Serial.println("║  3. Press RST button on BNO08x         ║");
    Serial.println("║  4. Connect RST pin to GPIO4           ║");
    Serial.println("╚════════════════════════════════════════╝");
    
    while(1) {
      delay(1000);
      Serial.print(".");
    }
  }
  
  Serial.println("BNO08x Found!\n");
  
  // Print product IDs
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
  Serial.println();
  
  setReports();
  
  Serial.println("\n============================================");
  Serial.println("   Reading data... Rotate the sensor!");
  Serial.println("============================================\n");
}

void loop() {
  if (bno08x.wasReset()) {
    Serial.println("Sensor was reset, re-enabling reports...");
    setReports();
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    switch (sensorValue.sensorId) {
      
      case SH2_ROTATION_VECTOR:
        quaternionToEuler(
          sensorValue.un.rotationVector.real,
          sensorValue.un.rotationVector.i,
          sensorValue.un.rotationVector.j,
          sensorValue.un.rotationVector.k
        );
        
        Serial.printf("YAW: %6.1f°  PITCH: %6.1f°  ROLL: %6.1f°  (Acc: %.1f°)\n",
          yaw, pitch, roll,
          sensorValue.un.rotationVector.accuracy * RAD_TO_DEG);
        break;
        
      case SH2_ACCELEROMETER:
        Serial.printf("ACCEL: X=%6.2f  Y=%6.2f  Z=%6.2f m/s²\n",
          sensorValue.un.accelerometer.x,
          sensorValue.un.accelerometer.y,
          sensorValue.un.accelerometer.z);
        break;
    }
  }
}
