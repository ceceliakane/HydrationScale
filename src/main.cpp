#include <Arduino.h>
#include <Wire.h>
void setup() {
  Serial.begin(9600); Wire.begin(); delay(1000);
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { Serial.print("Found 0x"); Serial.println(a, HEX); }
  }
  Serial.println("Scan done");
}
void loop() {}
