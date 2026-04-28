// GPS UART sniffer for ESP32-S3 + NEO-6M.
// Scans common bauds on UART2 (GPIO41=RX, GPIO42=TX), dumps raw bytes as hex
// + ASCII for 5s per baud, then loops. Helps decide if NEO-6M is mute, on a
// different baud, or sending non-NMEA data.

#include <Arduino.h>

#define GPS_RX_PIN 41
#define GPS_TX_PIN 42

const uint32_t BAUDS[] = {9600, 4800, 19200, 38400, 57600, 115200};
const size_t   N_BAUDS = sizeof(BAUDS) / sizeof(BAUDS[0]);

HardwareSerial GPSSerial(2);

static void scanBaud(uint32_t baud) {
  GPSSerial.end();
  delay(50);
  GPSSerial.begin(baud, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);

  Serial.printf("\n=== BAUD %u ===\n", (unsigned)baud);
  uint32_t bytes = 0;
  uint32_t printable = 0;
  uint32_t end = millis() + 5000;
  while (millis() < end) {
    while (GPSSerial.available()) {
      int c = GPSSerial.read();
      bytes++;
      if (c >= 0x20 && c <= 0x7e) printable++;
      Serial.printf("%02X ", (unsigned)c);
      if (c == '\n') Serial.print('\n');
    }
    delay(2);
  }
  Serial.printf("\n--- baud %u: bytes=%u printable=%u ---\n",
                (unsigned)baud, (unsigned)bytes, (unsigned)printable);
}

void setup() {
  Serial.begin(115200);
  delay(800);
  Serial.println("\n\n========================================");
  Serial.println("GPS UART Sniffer (ESP32-S3, GPIO41/42)");
  Serial.println("========================================");
}

void loop() {
  for (size_t i = 0; i < N_BAUDS; ++i) {
    scanBaud(BAUDS[i]);
  }
  Serial.println("\n--- cycle complete; restarting in 2s ---");
  delay(2000);
}
