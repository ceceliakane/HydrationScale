// Standalone mux/ADC check. Built only in the [env:adctest] environment.
// Mux output -> A0, mux select -> D5 (LOW = thermistor, HIGH = FSR).
// Prints the voltage the ADC sees on each channel so you can compare each one
// with a meter on the matching op-amp output.
#include <Arduino.h>

#define MUX_SELECT_PIN 5
#define MUX_ADC_PIN    A0

static float readChannel(int level) {
  digitalWrite(MUX_SELECT_PIN, level);
  delay(20);                              // settle
  analogRead(MUX_ADC_PIN);                // discard conversions right after switching
  analogRead(MUX_ADC_PIN);
  long sum = 0;
  for (int k = 0; k < 32; k++) sum += analogRead(MUX_ADC_PIN);
  return (sum / 32.0f) * 5.0f / 4095.0f;  // 12-bit, 5 V reference
}

void setup() {
  Serial.begin(9600);
  uint32_t t = millis();
  while (!Serial && millis() - t < 5000) { }
  analogReadResolution(12);
  pinMode(MUX_SELECT_PIN, OUTPUT);
  Serial.println("Mux test: compare each ADC voltage to a meter on that op-amp output");
}

void loop() {
  float vTherm = readChannel(LOW);
  float vFsr   = readChannel(HIGH);
  Serial.print("D5=LOW (thermistor): "); Serial.print(vTherm, 3);
  Serial.print(" V   |   D5=HIGH (FSR): "); Serial.print(vFsr, 3);
  Serial.println(" V");
  delay(500);
}
