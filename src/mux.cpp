#include "mux.h"

static MuxChannel current = MUX_THERMISTOR;
static float dropV[2] = {0.0f, 0.0f};   // volts to ADD back per channel (measure these!)

void muxBegin() {
#if USE_MUX
  pinMode(MUX_SELECT_PIN, OUTPUT);
  digitalWrite(MUX_SELECT_PIN, LOW);     // default: thermistor
#endif
  current = MUX_THERMISTOR;
}

void muxSelect(MuxChannel ch, unsigned long settleMs) {
#if USE_MUX
  // Always re-drive the pin when settleMs is explicitly passed (test sketch);
  // otherwise skip if already on this channel.
  if (ch == current && settleMs == MUX_SETTLE_MS) return;

  digitalWrite(MUX_SELECT_PIN, ch == MUX_FSR ? HIGH : LOW);
  current = ch;
  if (settleMs) delay(settleMs);
  analogRead(MUX_ADC_PIN);               // discard the first two conversions after
  analogRead(MUX_ADC_PIN);               // a switch (sample cap still charging)
#else
  (void)settleMs;
  current = ch;
#endif
}

MuxChannel muxCurrent() { return current; }

float muxReadCounts(MuxChannel ch, int samples, int delayMs) {
  muxSelect(ch);
#if USE_MUX
  int pin = MUX_ADC_PIN;
#else
  int pin = (ch == MUX_THERMISTOR) ? THERM_PIN : FSR_PIN;
#endif
  long sum = 0;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(pin);
    if (delayMs) delay(delayMs);
  }
  return sum / (float)samples;
}

float muxCountsToVolts(MuxChannel ch, float counts) {
  return counts * (MUX_VREF / MUX_ADC_MAX) + dropV[ch];
}

float muxReadVolts(MuxChannel ch, int samples, int delayMs) {
  return muxCountsToVolts(ch, muxReadCounts(ch, samples, delayMs));
}

void  muxSetDropV(MuxChannel ch, float volts) { dropV[ch] = volts; }
float muxGetDropV(MuxChannel ch)              { return dropV[ch]; }

void muxCalibrateDrop(MuxChannel ch, float knownVolts) {
  dropV[ch] = 0.0f;                                   // measure uncorrected
  float measured = muxReadVolts(ch, 64, 2);
  dropV[ch] = knownVolts - measured;
}
