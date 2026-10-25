#include <Arduino.h>
/*
  Capacitive touch / press sensor - Arduino UNO R4 WiFi
  ELEE 4230/6230 Quiz 6 / CP4

  Circuit:   D2 (square wave out) --- R --- node VC --- sensor C to GND
             VC  -> comparator (+) input
             Vref (1.65 V, from the 3.3 V rail / 2) -> comparator (-) input
             Comparator output (open collector) -> 10k pull-up -> D3
             (pull the output up to 3.3 V or 5 V; both are fine for D3)

  D2 swings 0-5 V on the UNO R4, so Vs = 5 V even though Vref comes from 3.3 V.
  Change VS_VOLTS / VREF_VOLTS below if your hardware differs.

  Method:    D2 rises -> start timer -> wait for comparator output to rise -> stop.
             td = R * C * ln( Vs / (Vs - Vref) )
             so   C = td / ( R * ln( Vs / (Vs - Vref) ) )

  With R = 2M the delays are tens of microseconds (nominal ~29 us, weakest press +18 us),
  so the timer is the CPU cycle counter (48 MHz -> 20.8 ns per tick) instead of micros().
  If the sketch will not compile because of DWT/CoreDebug, set USE_CYCCNT to 0 (1 us ticks
  then give only ~18 ticks for the weakest press).

  Press rule (CP4): after each rising edge wait tthr, then read the comparator. HIGH =
  delay shorter than tthr = no press; LOW = press. A press is declared when most of the
  last AVG_CYCLES cycles read LOW (plus a 2-average debounce). tthr is set at startup
  midway between the untouched baseline td and the weakest-press td. td is also measured
  so you can compare it to the twin / hand calculation.

  Serial monitor / plotter: 115200 baud.  Send 'c' to re-measure the baseline.
*/

#define USE_CYCCNT 1

// ---------------- pins ----------------
const uint8_t PIN_DRIVE = 2;   // square wave out to R
const uint8_t PIN_CMP   = 3;   // comparator output in

// ---------------- circuit values (edit these) ----------------
const float R_OHMS     = 2.0e6f;     // series resistor (use your measured value)
const float F_HZ       = 300.0f;   // half period must be >= 3*R*Cmax (195 pF, 2M -> f <= ~427 Hz)
const float VS_VOLTS   = 5.0f;     // square-wave high level (UNO R4 pin)
const float VREF_VOLTS = 1.57f;    // comparator reference
const float LN_FACTOR  = logf(VS_VOLTS / (VS_VOLTS - VREF_VOLTS));   // 0.400 here
const float WEAKEST_PRESS_PF = 23.0f;   // smallest delta-C in your press map (light, edge)

// Threshold = half of the delay change of your weakest press.
const float   PRESS_DELTA_US = 0.5f * LN_FACTOR * R_OHMS * WEAKEST_PRESS_PF * 1e-12f * 1e6f;
// Bench-check predictions (hand calc / twin) for the three cases you compare in CP4.
const float   NOMINAL_C_PF   = 36.3f;     // untouched mean from your press map
const float   WEAKEST_C_PF   = 59.3f;     // light press at the edge (mean)
const float   HARD_C_PF      = 195.33f;   // hard press at the center (mean)
const uint16_t AVG_CYCLES    = 10;     // cycles averaged per decision
const uint8_t  DEBOUNCE_AVGS = 2;      // consecutive averages needed to change state
const uint32_t TTHR_UNSET    = 0xFFFFFFFF;
const uint32_t PRINT_INTERVAL_MS = 1000; // print once per interval (and instantly on press/release)

// ---------------- timing helpers ----------------
#if USE_CYCCNT
  const float US_PER_TICK = 1e6f / (float)F_CPU;
  static inline uint32_t ticksNow() { return DWT->CYCCNT; }
#else
  const float US_PER_TICK = 1.0f;
  static inline uint32_t ticksNow() { return micros(); }
#endif

const uint32_t HALF_US       = (uint32_t)(500000.0f / F_HZ);
const uint32_t TIMEOUT_TICKS = (uint32_t)((HALF_US - 20) / US_PER_TICK);

float    baselineUs = 0;
float    tthrUs = 0;                 // decision time after each rising edge
uint32_t tthrTicks = TTHR_UNSET;     // same, in timer ticks (UNSET during calibration)
bool     cmpAtThr = true;            // comparator level read at tthr on the last cycle
uint16_t pressCycles = 0;            // cycles in the last average that read LOW at tthr
bool     pressed = false;
uint8_t  count = 0;

// One full square-wave cycle (high half, then low half).
// Measures td (us, or -1 on timeout) AND reads the comparator at tthr after the rising
// edge (CP4 rule): HIGH at tthr = delay shorter than tthr = no press; LOW = press.
float runCycle() {
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
  if (!sampled) cmpAtThr = (td >= 0);        // never reached tthr: HIGH only if it rose

  while ((micros() - m0) < HALF_US) { }      // finish the high half
  digitalWrite(PIN_DRIVE, LOW);
  uint32_t m1 = micros();
  while ((micros() - m1) < HALF_US) { }      // low half lets C discharge
  return td;
}

// Average of n valid cycles. Returns -1 if most cycles timed out.
float averageCycles(uint16_t n) {
  float sum = 0; uint16_t good = 0;
  pressCycles = 0;
  for (uint16_t i = 0; i < n; i++) {
    float td = runCycle();
    if (td > 0) { sum += td; good++; }
    if (!cmpAtThr) pressCycles++;
  }
  return (good >= n / 2) ? sum / good : -1.0f;
}

float tdToPF(float td_us) {                  // C = td / (R * ln factor)
  return (td_us * 1e-6f) / (LN_FACTOR * R_OHMS) * 1e12f;
}

float predictTdUs(float c_pF) {              // td = R * C * ln[Vs/(Vs-Vref)]
  return LN_FACTOR * R_OHMS * c_pF * 1e-12f * 1e6f;
}

void calibrate() {
  tthrTicks = TTHR_UNSET;
  Serial.println(F("Calibrating baseline - hands OFF the sensor and leads..."));
  for (int i = 0; i < 100; i++) runCycle();            // settle
  const int M = 30;                                    // number of averages
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
  Serial.print(F("Baseline td = ")); Serial.print(baselineUs, 3);
  Serial.print(F(" us  (SD of averages ")); Serial.print(sd, 3);
  Serial.print(F(" us)  C = ")); Serial.print(tdToPF(baselineUs), 1);
  Serial.println(F(" pF  (includes fixed pin/loop latency)"));
  tthrUs = baselineUs + PRESS_DELTA_US;          // midpoint: untouched vs weakest press
  tthrTicks = (uint32_t)(tthrUs / US_PER_TICK);
  Serial.print(F("tthr (read comparator this long after each rising edge) = "));
  Serial.print(tthrUs, 2); Serial.println(F(" us"));
  Serial.println(F("--- Bench check for CP4: predicted td (hand calc/twin) vs measured ---"));
  Serial.print(F("Predicted td, untouched (36.3 pF):      ")); Serial.print(predictTdUs(NOMINAL_C_PF), 1); Serial.println(F(" us"));
  Serial.print(F("Predicted td, weakest press (59.3 pF):  ")); Serial.print(predictTdUs(WEAKEST_C_PF), 1); Serial.println(F(" us"));
  Serial.print(F("Predicted td, hard center (195.3 pF):   ")); Serial.print(predictTdUs(HARD_C_PF), 1); Serial.println(F(" us"));
  Serial.print(F("Measured td, untouched (this baseline): ")); Serial.print(baselineUs, 1); Serial.println(F(" us"));
  Serial.println(F("Now press the sensor; measured td is printed below. Type 'c' to re-baseline."));
  if (6.0f * sd > PRESS_DELTA_US)
    Serial.println(F("WARNING: baseline noise is large vs the press threshold. Tape down leads/pads, or raise R."));
  pressed = false; count = 0;
}

void setup() {
  pinMode(PIN_DRIVE, OUTPUT);
  digitalWrite(PIN_DRIVE, LOW);
  pinMode(PIN_CMP, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(9600);
  uint32_t t = millis();
  while (!Serial && millis() - t < 2000) { }
  Serial.println(F("=== NEW CODE: cap touch v5, R=2M, Vs=5V, Vref=1.65V ==="));
  // blink the LED 5 times so you can see the new code started
  for (int i = 0; i < 5; i++) { digitalWrite(LED_BUILTIN, HIGH); delay(100); digitalWrite(LED_BUILTIN, LOW); delay(100); }

#if USE_CYCCNT
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // enable the cycle counter
  DWT->CYCCNT = 0;
  DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
#endif

  // How fast can the loop poll the comparator pin? This sets the timing granularity.
  volatile int sink = 0;
  uint32_t a = ticksNow();
  for (int i = 0; i < 1000; i++) sink += digitalRead(PIN_CMP);
  uint32_t b = ticksNow();
  Serial.print(F("digitalRead poll time ~ ")); Serial.print((b - a) * US_PER_TICK / 1000.0f, 3);
  Serial.println(F(" us (timing granularity)"));

  Serial.print(F("ln factor = ")); Serial.print(LN_FACTOR, 4);
  Serial.print(F("  Vs = ")); Serial.print(VS_VOLTS, 2);
  Serial.print(F(" V  Vref = ")); Serial.print(VREF_VOLTS, 2); Serial.println(F(" V"));
  Serial.print(F("Half period = ")); Serial.print(HALF_US);
  Serial.print(F(" us.  Settling ratio at 195 pF = "));
  Serial.println(HALF_US * 1e-6f / (R_OHMS * 195e-12f), 2);   // want >= 3
  calibrate();
}

void loop() {
  if (Serial.available() && Serial.read() == 'c') calibrate();

  float avg = averageCycles(AVG_CYCLES);
  if (avg < 0 || baselineUs <= 0) return;

  // CP4 rule: a cycle reads PRESS if the comparator is still LOW at tthr.
  bool wantPressed = (2 * pressCycles > AVG_CYCLES);   // majority of the cycles
  if (wantPressed != pressed) {
    if (++count >= DEBOUNCE_AVGS) { pressed = wantPressed; count = 0; }
  } else count = 0;
  digitalWrite(LED_BUILTIN, pressed ? HIGH : LOW);

  static uint32_t lastPrint = 0;
  static bool lastPressed = false;
  if (pressed != lastPressed || millis() - lastPrint >= PRINT_INTERVAL_MS) {
    Serial.print(F("td = "));        Serial.print(avg, 2);
    Serial.print(F(" us | C = "));   Serial.print(tdToPF(avg), 1);
    Serial.print(F(" pF | baseline = ")); Serial.print(baselineUs, 2);
    Serial.print(F(" us | tthr = ")); Serial.print(tthrUs, 2);
    Serial.print(F(" us | cycles read as press: ")); Serial.print(pressCycles);
    Serial.print('/'); Serial.print(AVG_CYCLES);
    Serial.print(F(" | "));
    Serial.println(pressed ? F("PRESSED") : F("not pressed"));
    lastPrint = millis();
    lastPressed = pressed;
  }
}
