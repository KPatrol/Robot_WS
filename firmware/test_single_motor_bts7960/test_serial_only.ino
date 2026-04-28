/*
 * Test Serial Only - Debug Version
 * Chỉ test Serial communication, không có gì khác
 */

void setup() {
  Serial.begin(115200);
  delay(500);
}

void loop() {
  Serial.println("HELLO KPATROL 115200");
  Serial.print("Millis: ");
  Serial.println(millis());
  Serial.println("---");
  delay(1000);
}
