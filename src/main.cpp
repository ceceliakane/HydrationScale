#include <Arduino.h>
// ================================================================
// Hydration Scale - main
// Every sensor is a function in sensors.cpp; this file only calls them.
// ================================================================
#include <WiFiS3.h>
#include "sensors.h"
#include "hydration.h"
#include "oled.h"
#include "captouch.h"
#include "iot.h"

// ---- USER SETTINGS ----
const float BODY_WEIGHT_LB   = 140.0;   // <-- set to yours
const float EXERCISE_OZ      = 0.0;     // extra fl oz allowance on workout days
const long  TZ_OFFSET_S      = -4L * 3600;  // US Eastern, EDT (use -5 after Nov 1, 2026)

// ---- ThingSpeak field map ----
#define F_BMP_TEMP_F   1
#define F_PRESSURE_HPA 2
#define F_THERM_TEMP_F 3   // Kalman-filtered, calibrated
#define F_HUMIDITY     4
#define F_WEIGHT_OZ    5
#define F_GOAL_OZ      6
#define F_INTAKE_OZ    7
#define F_HYDRATION    8   // score, % of goal

const unsigned long SENSE_INTERVAL_MS = 200;     // Kalman filters want frequent updates
const unsigned long ENV_INTERVAL_MS   = 2000;    // temp / humidity refresh
const unsigned long OLED_INTERVAL_MS  = 500;
const unsigned long DAY_CHECK_MS      = 60000;
const unsigned long IOT_INTERVAL_MS   = 15000;   // ThingSpeak free tier

unsigned long lastSense = 0, lastEnv = 0, lastOled = 0, lastDayCheck = 0, lastPost = 0;
long lastDay = -1;

ThermistorReading therm;
ScaleReading      scale;
float bmpTempF = NAN, humidity = NAN, envTempF = NAN;

void updateEnvironment() {
  bmpTempF = readBMPTempF();
  humidity = readHumidityRH();

  // Use both temperature sensors (thermistor + BMP280) when available
  bool t1 = !isnan(therm.tempF), t2 = !isnan(bmpTempF);
  if (t1 && t2)      envTempF = 0.5f * (therm.tempF + bmpTempF);
  else if (t1)       envTempF = therm.tempF;
  else if (t2)       envTempF = bmpTempF;
  else               envTempF = NAN;
}

void checkNewDay() {
  unsigned long epoch = WiFi.getTime();         // 0 if no time yet
  if (epoch == 0) return;
  long day = ((long)epoch + TZ_OFFSET_S) / 86400L;
  if (lastDay < 0) lastDay = day;
  else if (day != lastDay) { hydrationResetDay(); lastDay = day; }
}

void setup() {
  Serial.begin(9600);
  sensorsBegin();
  oledBegin();
  oledMessage("Calibrating touch", "Hands OFF the pad");
  capTouchBegin();
  oledWake();   // show the dashboard for the first 15 s
  hydrationBegin(BODY_WEIGHT_LB, EXERCISE_OZ);
  iotBegin();
  therm = readThermistor();
  updateEnvironment();
  Serial.println("Waiting for object...");
}

void loop() {
  presenceUpdate();                    // non-blocking, runs its own 60 ms timer

  if (readTouch()) oledWake();         // ~33 ms per call; a touch lights the OLED
  if (Serial.available() && Serial.read() == 'c') capTouchRecalibrate();
  unsigned long now = millis();

  if (now - lastSense >= SENSE_INTERVAL_MS) {
    lastSense = now;
    therm = readThermistor();          // keeps thermistor Kalman filter running
    scale = readScale();               // valid only when a bottle is present and settled
    hydrationUpdate(envTempF, humidity, scale, isObjectPresent());

  }

  if (now - lastEnv >= ENV_INTERVAL_MS) {
    lastEnv = now;
    updateEnvironment();
  }

  if (now - lastOled >= OLED_INTERVAL_MS) {
    lastOled = now;
    oledRender(hydrationGet(), envTempF, humidity, isObjectPresent());
  }

  if (now - lastDayCheck >= DAY_CHECK_MS) {
    lastDayCheck = now;
    checkNewDay();
  }

  if (now - lastPost >= IOT_INTERVAL_MS) {
    lastPost = now;
    HydrationState h = hydrationGet();

    Serial.print("Env "); Serial.print(envTempF, 1); Serial.print("F "); Serial.print(humidity, 0);
    Serial.print("%RH | HI "); Serial.print(h.heatIndexF, 1);
    Serial.print("F x"); Serial.print(h.envMultiplier, 2);
    Serial.print(" | Goal "); Serial.print(h.goalOz, 1);
    Serial.print(" oz | Intake "); Serial.print(h.intakeOz, 1);
    Serial.print(" oz | Score "); Serial.print(h.scorePct, 0); Serial.println("%");

    iotSetField(F_BMP_TEMP_F,   bmpTempF);
    iotSetField(F_PRESSURE_HPA, readBMPPressure_hPa());
    iotSetField(F_THERM_TEMP_F, therm.tempF);
    iotSetField(F_HUMIDITY,     humidity);
    if (scale.valid) iotSetField(F_WEIGHT_OZ, scale.weightOz);
    iotSetField(F_GOAL_OZ,      h.goalOz);
    iotSetField(F_INTAKE_OZ,    h.intakeOz);
    iotSetField(F_HYDRATION,    h.scorePct);
    iotPublish();
  }
}
