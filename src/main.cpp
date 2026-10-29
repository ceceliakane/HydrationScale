// Holds D5 (mux select) at a steady level so a meter can read it.
// Built only in the [env:pintest] environment.
#include <Arduino.h>

#define MUX_SELECT_PIN 5

void setup() {
  Serial.begin(9600);
  uint32_t t = millis();
  while (!Serial && millis() - t < 5000) { }
  pinMode(MUX_SELECT_PIN, OUTPUT);
  Serial.println("D5 steady-level test: measure D5 to ground with a meter");
}

void loop() {
  digitalWrite(MUX_SELECT_PIN, LOW);
  Serial.println("D5 = LOW  for 6 s (expect about 0 V)");
  delay(6000);
  digitalWrite(MUX_SELECT_PIN, HIGH);
  Serial.println("D5 = HIGH for 6 s (expect about 4.7-5 V)");
  delay(6000);
}
