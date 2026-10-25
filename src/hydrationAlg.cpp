#pragma once
#include <Arduino.h>
#include "sensors.h"     // ScaleReading

// ================================================================
// HYDRATION ALGORITHM (go-beyond)
//
//  Daily goal = baseline * environmental multiplier + exercise allowance
//    baseline   = body weight (lb) * 0.5 fl oz        (common rule of thumb)
//    multiplier = 1 + 1.5% per degF of heat index above 75F, capped at +50%
//    heat index = NWS Rothfusz regression from temperature + relative humidity
//                 (so hotter AND more humid -> higher goal)
//
//  Intake is measured from weight lost between a bottle leaving the scale
//  and returning to it (stable reading before vs. stable reading after).
//
//  ACSM's 2007 position stand gives no T/RH formula; it asks for individualized
//  plans and to limit water-deficit loss to <2% body mass. This is an
//  ACSM-informed design, with the constants below as tunable assumptions.
// ================================================================

struct HydrationState {
  float goalOz;        // daily goal (fl oz)
  float intakeOz;      // consumed so far today (fl oz)
  float scorePct;      // intake / goal * 100, clamped to 0-100
  float heatIndexF;
  float envMultiplier; // 1.00 = no adjustment
  float lastDrinkOz;   // size of most recent detected drink
};

void  hydrationBegin(float bodyWeightLb, float exerciseOz);
void  hydrationUpdate(float tempF, float rh, const ScaleReading& scale, bool bottlePresent);
HydrationState hydrationGet();
void  hydrationResetDay();

// Exposed for testing / your write-up
float computeHeatIndexF(float tempF, float rh);
float computeEnvMultiplier(float heatIndexF);
