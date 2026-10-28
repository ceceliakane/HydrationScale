#include "hydration.h"
#include <math.h>

// ---------------- Tunable assumptions ----------------
static const float FL_OZ_PER_LB        = 0.5f;     // baseline: 0.5 fl oz per lb body weight
static const float HI_START_F          = 75.0f;    // heat index where extra fluid begins
static const float EXTRA_PER_F         = 0.015f;   // +1.5% per degF above HI_START_F
static const float EXTRA_CAP           = 0.50f;    // never more than +50%

static const float FLOZ_PER_WEIGHT_OZ  = 0.9586f;  // 1 oz of water by weight = 0.9586 fl oz
static const float STABLE_BAND_OZ      = 0.30f;    // readings within this count as "stable"
static const int   STABLE_CYCLES       = 5;        // ...for this many consecutive updates
static const float MIN_EVENT_OZ        = 1.0f;     // ignore changes smaller than FSR error
static const float MAX_DRINK_OZ        = 20.0f;
static const float MIN_BOTTLE_OZ       = 1.5f;     // below this the bottle counts as lifted off the scale    // bigger drop = different bottle / spill, ignore

// ---------------- State ----------------
static float bodyLb = 140.0f, exerciseOz = 0.0f;
static float goalOz = 0, intakeOz = 0, hi = 0, mult = 1.0f, lastDrinkOz = 0;

static float lastStable = 0;  static bool haveStable = false;
static float prevOz = 0;      static int  stableCount = 0;
static bool  wasOnScale = false, bottleAway = false;
static float refOz = 0;

// ================================================================
// Heat index (NWS Rothfusz regression, degF / %RH)
// ================================================================
float computeHeatIndexF(float T, float RH) {
  float simple = 0.5f * (T + 61.0f + ((T - 68.0f) * 1.2f) + (RH * 0.094f));
  if ((simple + T) / 2.0f < 80.0f) return simple;

  float H = -42.379f + 2.04901523f * T + 10.14333127f * RH
            - 0.22475541f * T * RH - 0.00683783f * T * T
            - 0.05481717f * RH * RH + 0.00122874f * T * T * RH
            + 0.00085282f * T * RH * RH - 0.00000199f * T * T * RH * RH;

  if (RH < 13.0f && T >= 80.0f && T <= 112.0f)
    H -= ((13.0f - RH) / 4.0f) * sqrtf((17.0f - fabsf(T - 95.0f)) / 17.0f);
  else if (RH > 85.0f && T >= 80.0f && T <= 87.0f)
    H += ((RH - 85.0f) / 10.0f) * ((87.0f - T) / 5.0f);
  return H;
}

float computeEnvMultiplier(float heatIndexF) {
  float extra = (heatIndexF - HI_START_F) * EXTRA_PER_F;
  if (extra < 0) extra = 0;
  if (extra > EXTRA_CAP) extra = EXTRA_CAP;
  return 1.0f + extra;
}

// ================================================================
void hydrationBegin(float bodyWeightLb, float exerciseAllowanceOz) {
  bodyLb = bodyWeightLb;
  exerciseOz = exerciseAllowanceOz;
  goalOz = bodyLb * FL_OZ_PER_LB + exerciseOz;
}

void hydrationResetDay() {
  intakeOz = 0;
  lastDrinkOz = 0;
  Serial.println("Hydration: new day, intake reset.");
}

void hydrationUpdate(float tempF, float rh, const ScaleReading& s, bool present) {
  // ---- 1. Daily goal from environment ----
  if (!isnan(tempF) && !isnan(rh)) {
    hi   = computeHeatIndexF(tempF, rh);
    mult = computeEnvMultiplier(hi);
  }
  goalOz = (bodyLb * FL_OZ_PER_LB) * mult + exerciseOz;

  // ---- 2. Is the bottle actually ON the scale? ----
  // Uses the FSR itself (weight above MIN_BOTTLE_OZ), not just the ultrasonic flag,
  // so a lift is caught even if the HC-SR04 still sees the bottle nearby.
  bool onScale = present && s.valid && s.weightOz > MIN_BOTTLE_OZ;

  if (wasOnScale && !onScale) {                 // bottle lifted off
    stableCount = 0;
    if (haveStable) {
      bottleAway = true;
      refOz = lastStable;
      Serial.print("Bottle lifted. Reference weight: ");
      Serial.print(refOz, 2); Serial.println(" oz");
    }
  }
  wasOnScale = onScale;

  if (!onScale) { stableCount = 0; return; }

  // ---- 3. Track stability while the bottle is on the scale ----
  if (fabsf(s.weightOz - prevOz) <= STABLE_BAND_OZ) stableCount++;
  else stableCount = 0;
  prevOz = s.weightOz;

  if (stableCount >= STABLE_CYCLES) {
    float now = s.weightOz;

    // Bottle just came back: compare against what it weighed when it left
    if (bottleAway) {
      float delta = refOz - now;                       // positive = water was drunk
      Serial.print("Bottle returned. Was "); Serial.print(refOz, 2);
      Serial.print(" oz, now "); Serial.print(now, 2);
      Serial.print(" oz, change "); Serial.print(delta, 2); Serial.println(" oz");

      if (delta >= MIN_EVENT_OZ && delta <= MAX_DRINK_OZ) {
        lastDrinkOz = delta * FLOZ_PER_WEIGHT_OZ;
        intakeOz += lastDrinkOz;
        Serial.print("Drink detected: "); Serial.print(lastDrinkOz, 1); Serial.println(" fl oz");
      } else if (delta <= -MIN_EVENT_OZ) {
        Serial.println("Refill detected (weight increased) - no intake counted.");
      } else {
        Serial.println("Change too small (under threshold) - not counted.");
      }
      bottleAway = false;
    }
    lastStable = now;
    haveStable = true;
  }
}

HydrationState hydrationGet() {
  HydrationState h;
  h.goalOz        = goalOz;
  h.intakeOz      = intakeOz;
  h.scorePct      = goalOz > 0 ? constrain(intakeOz / goalOz * 100.0f, 0.0f, 100.0f) : 0.0f;
  h.heatIndexF    = hi;
  h.envMultiplier = mult;
  h.lastDrinkOz   = lastDrinkOz;
  return h;
}
