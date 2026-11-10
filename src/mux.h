#pragma once
#include <Arduino.h>

// ================================================================
// 2-to-1 BJT ANALOG MUX (CP4)
//   GPIO LOW  -> Q1/Q3 off -> Q5 on  -> thermistor in-amp output (V1) to ADC
//   GPIO HIGH -> Q1/Q3 on  -> Q6 on  -> FSR op-amp output (V2) to ADC
// ================================================================

// Set to 1 once the mux is wired. With 0, each sensor is read directly
// on its own pin (THERM_PIN / FSR_PIN) and no select line is driven.
#define USE_MUX          1

#define MUX_ADC_PIN      A0    // shared ADC input (mux output)
#define MUX_SELECT_PIN   5     // D5
#define THERM_PIN        A0    // direct-wire pins used when USE_MUX is 0
#define FSR_PIN          A1

#define MUX_SETTLE_MS    10    // wait after switching, before the first conversion
#define MUX_ADC_MAX      4095.0f   // 12-bit
#define MUX_VREF         5.0f

enum MuxChannel { MUX_THERMISTOR = 0, MUX_FSR = 1 };

void       muxBegin();
void       muxSelect(MuxChannel ch, unsigned long settleMs = MUX_SETTLE_MS);
MuxChannel muxCurrent();

// Averaged ADC counts for a channel (selects the channel first).
float muxReadCounts(MuxChannel ch, int samples, int delayMs);

// Counts -> volts at the ADC pin, plus the per-channel correction below.
float muxCountsToVolts(MuxChannel ch, float counts);

// Convenience: select + average + convert.
float muxReadVolts(MuxChannel ch, int samples, int delayMs);

// The BJT pass transistors drop some voltage (Vce(sat) / Vbe effects), so the
// ADC sees slightly less than the op-amp output. Set a per-channel correction:
//   muxSetDropV(ch, v)               -> add v volts to every reading on ch
//   muxCalibrateDrop(ch, knownVolts) -> measure the op-amp output with a DMM,
//                                       pass it in, and the drop is computed
void  muxSetDropV(MuxChannel ch, float volts);
float muxGetDropV(MuxChannel ch);
void  muxCalibrateDrop(MuxChannel ch, float knownVolts);
