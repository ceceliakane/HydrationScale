#pragma once
#include <Arduino.h>

// ================================================================
// CONFIG
// ================================================================
#include "mux.h"   // USE_MUX, pins, and channel selection live there

#define TRIG_PIN         9
#define ECHO_PIN         10
#define SCALE_ENABLE_PIN 7

// ================================================================
// DATA TYPES
// ================================================================
struct ThermistorReading {
  float vRaw;      // averaged in-amp output (V)
  float vKalman;   // Kalman-filtered voltage (V)
  float tempF;     // calibrated temperature from vKalman (F)
};

struct ScaleReading {
  bool  valid;     // false if nothing is on the scale / still settling
  int   rawADC;
  float vRaw;      // V
  float vKalman;   // V (filtered)
  float weightOz;
  float forceN;
};

// ================================================================
// INIT
// ================================================================
bool sensorsBegin();           // init ADC, I2C, BMP280, SHT31, HC-SR04, then calibrate thermistor
                               // returns false if any I2C sensor was not found

// ================================================================
// BMP280 (I2C)  -- reference temp + pressure
// ================================================================
float readBMPTempF();
float readBMPTempC();
float readBMPPressure_hPa();

// ================================================================
// SHT31 (I2C)  -- humidity (+ its own temp)
// ================================================================
float readHumidityRH();        // %RH (raw; apply your CP2 calibration here if desired)
float readSHT31TempC();

// ================================================================
// THERMISTOR (RAW sensor: bridge -> INA121P -> ADC), Kalman filtered
// ================================================================
ThermistorReading readThermistor();   // call often (~every 200 ms) so the Kalman filter works
float readThermistorTempF();          // convenience wrapper: returns just tempF
void  calibrateThermistorToBMP();     // recomputes the offset against the BMP280

// ================================================================
// HC-SR04 presence detection
// ================================================================
float readDistanceCm();               // -1 if no echo
bool  presenceUpdate();               // non-blocking; call every loop. Returns objectPresent
bool  isObjectPresent();
bool  scaleReady();                   // present AND settled

// ================================================================
// FSR SCALE (RAW sensor: op-amp active linearization -> ADC), Kalman filtered
// ================================================================
ScaleReading readScale();             // valid=false unless scaleReady()
float readWeightOz();                 // convenience wrapper: returns 0 if not ready

// ================================================================
// Thermistor model helpers (demo / validation, kept from cp3demoFINAL)
// ================================================================
float calculateVoltageFromTempF(float tempF);
int   calculateADCFromTempF(float tempF);
float calculateOriginalVOutFromTempF(float tempF);
