#include <Arduino.h>
#include "captouch.h"
#include <math.h>

/*
  Method (unchanged from capTouch.cpp):
    D2 rises -> start timer -> wait for comparator output to rise -> stop.
    td = R * C * ln( Vs / (Vs - Vref) )
  A cycle reads PRESS if the comparator is still LOW tthr after the rising edge.
  A press is declared when most of the last AVG_CYCLES cycles read PRESS,
  debounced over DEBOUNCE_AVGS consecutive averages.
  Timer = CPU cycle counter (DWT->CYCCNT). Set USE_CYCCNT 0 if it won't compile.
*/
#define USE_CYCCNT 1

// ---------------- pins ----------------
static const uint8_t PIN_DRIVE = 2;   // square wave out to R
static const uint8_t PIN_CMP   = 3;   // comparator output in

// ---------------- circuit values ----------------
static const float R_OHMS     = 470e3f;
static const float F_HZ       = 300.0f;
static const float VS_VOLTS   = 5.0f;
static const float VREF_VOLTS = 1.57f;
static const float LN_FACTOR  = logf(VS_VOLTS / (VS_VOLTS - VREF_VOLTS));
static const float WEAKEST_PRESS_PF = 50.0f;

static const float PRESS_DELTA_US = 0.5f * LN_FACTOR * R_OHMS * WEAKEST_PRESS_PF * 1e-12f * 1e6f;
static const float NOMINAL_C_PF = 36.3f;
static const float WEAKEST_C_PF = 59.3f;
static const float HARD_C_PF    = 195.33f;
static const uint16_t AVG_CYCLES    = 10;
static const uint8_t  DEBOUNCE_AVGS = 2;
static const uint32_t TTHR_UNSET    = 0xFFFFFFFF;

#if USE_CYCCNT
  static const float US_PER_TICK = 1e6f / (float)F_CPU;
  static inline uint32_t ticksNow() { return DWT->CYCCNT; }
#else
  static const float US_PER_TICK = 1.0f;
  static inline uint32_t ticksNow() { return micros(); }
#endif

static const uint32_t HALF_US       = (uint32_t)(500000.0f / F_HZ);
static const uint32_t TIMEOUT_TICKS = (uint32_t)((HALF_US - 20) / US_PER_TICK);

static float    baselineUs = 0;
static float    tthrUs = 0;
static uint32_t tthrTicks = TTHR_UNSET;
static bool     cmpAtThr = true;
static uint16_t pressCycles = 0;
static bool     pressed = false;
static uint8_t  count = 0;
static float    lastAvgUs = 0;

// ---------------- one square-wave cycle ----------------
static float runCycle() {
  float td = -1.0f;
  bool sampled = false;
  cmpAtThr = true;
  uint32_t m0 = micros();
  digitalWrite(PIN_DRIVE, HIGH);
  uint32_t t0 = ticksNow();

  while ((ticksNow() - t0) < TIMEOUT_TICKS) {
    uint32_t t = ticksNow();
    bool hi = (digitalRead(PIN_CMP) == HIGH);
    if (td < 0 && hi) td = (float)(t - t0) * US_PER_TICK;
    if (!sampled && (t - t0) >= tthrTicks) { cmpAtThr = hi; sampled = true; }
    if (td >= 0 && (sampled || tthrTicks == TTHR_UNSET)) break;
  }
  if (!sampled) cmpAtThr = (td >= 0);

  while ((micros() - m0) < HALF_US) { }
  digitalWrite(PIN_DRIVE, LOW);
  uint32_t m1 = micros();
  while ((micros() - m1) < HALF_US) { }
  return td;
}

static float averageCycles(uint16_t n) {
  float sum = 0; uint16_t good = 0;
  pressCycles = 0;
  for (uint16_t i = 0; i < n; i++) {
    float td = runCycle();
    if (td > 0) { sum += td; good++; }
    if (!cmpAtThr) pressCycles++;
  }
  return (good >= n / 2) ? sum / good : -1.0f;
}

static float tdToPF(float td_us) {
  return (td_us * 1e-6f) / (LN_FACTOR * R_OHMS) * 1e12f;
}

static float predictTdUs(float c_pF) {
  return LN_FACTOR * R_OHMS * c_pF * 1e-12f * 1e6f;
}

// ---------------- calibration ----------------
void capTouchRecalibrate() {
  tthrTicks = TTHR_UNSET;
  Serial.println(F("Calibrating touch baseline - hands OFF the pad and leads..."));
  for (int i = 0; i < 100; i++) runCycle();
  const int M = 30;
  float sum = 0, sumsq = 0; int good = 0;
  for (int i = 0; i < M; i++) {
    float a = averageCycles(AVG_CYCLES);
    if (a > 0) { sum += a; sumsq += a * a; good++; }
  }
  if (good < M / 2) {
    Serial.println(F("ERROR: comparator never rose. Check wiring, Vref, pull-up, R, f."));
    baselineUs = 0; return;
  }
  baselineUs = sum / good;
  float sd = sqrtf(max(0.0f, sumsq / good - baselineUs * baselineUs));
  Serial.print(F("Touch baseline td = ")); Serial.print(baselineUs, 3);
  Serial.print(F(" us (SD ")); Serial.print(sd, 3);
  Serial.print(F(" us)  C = ")); Serial.print(tdToPF(baselineUs), 1); Serial.println(F(" pF"));

  tthrUs = baselineUs + PRESS_DELTA_US;
  tthrTicks = (uint32_t)(tthrUs / US_PER_TICK);
  Serial.print(F("Touch tthr = ")); Serial.print(tthrUs, 2); Serial.println(F(" us"));
  Serial.print(F("Predicted td  untouched/weakest/hard: "));
  Serial.print(predictTdUs(NOMINAL_C_PF), 1); Serial.print(F(" / "));
  Serial.print(predictTdUs(WEAKEST_C_PF), 1); Serial.print(F(" / "));
  Serial.print(predictTdUs(HARD_C_PF), 1);    Serial.println(F(" us"));
  if (6.0f * sd > PRESS_DELTA_US)
    Serial.println(F("WARNING: touch baseline noise is large vs threshold. Tape down leads/pad, or raise R."));
  pressed = false; count = 0;
}

void capTouchBegin() {
  pinMode(PIN_DRIVE, OUTPUT);
  digitalWrite(PIN_DRIVE, LOW);
  pinMode(PIN_CMP, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
#if USE_CYCCNT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
#endif
  capTouchRecalibrate();
}

// ---------------- public read ----------------
bool readTouch() {
  if (baselineUs <= 0) return false;            // calibration failed

  float avg = averageCycles(AVG_CYCLES);
  if (avg < 0) return pressed;                  // bad window: keep previous state
  lastAvgUs = avg;

  bool wantPressed = (2 * pressCycles > AVG_CYCLES);
  if (wantPressed != pressed) {
    if (++count >= DEBOUNCE_AVGS) {
      pressed = wantPressed;
      count = 0;
#if CAPTOUCH_DEBUG
      Serial.print(F("Touch ")); Serial.print(pressed ? F("PRESSED") : F("released"));
      Serial.print(F("  td = ")); Serial.print(avg, 2);
      Serial.print(F(" us  C = ")); Serial.print(tdToPF(avg), 1); Serial.println(F(" pF"));
#endif
    }
  } else count = 0;

  digitalWrite(LED_BUILTIN, pressed ? HIGH : LOW);
  return pressed;
}

float capTouchDelayUs() { return lastAvgUs; }
