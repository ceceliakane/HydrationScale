#include "sensors.h"
#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_SHT31.h>
#include <math.h>
#include "Kalman.h"
#include "mux.h"

// ================================================================
// SHARED ADC SETTINGS (analogReadResolution is global, so one value for all)
// ================================================================
static const int   ADC_BITS = 12;
static const float ADC_MAX  = 4095.0;
static const float V_SUPPLY = 5.00;    // Uno R4 ADC reference

// ================================================================
// OBJECTS / STATE
// ================================================================
static Adafruit_BMP280 bmp;
static Adafruit_SHT31  sht31;
static bool bmpOk = false;
static bool shtOk = false;

// ---------- Thermistor constants ----------
static const float GAIN     = 3.00;    // INA121 gain (Rg = 25k)
static const float V_REF    = 2.50;
static const float R_BOTTOM = 448.0;
static const float R_P      = 821.0;
static const float R0       = 1000.0;
static const float T0_C     = 26.6;
static const float BETA     = 3636.0;
static float tempOffset = 0.0;
static KalmanFilter thermFilter(0.0001, 0.01, 1.0, 2.5);

// ---------- FSR calibration ----------
static const float V_ZERO        = 0.22;
static const float V_CAL         = 1.47;
static const float CAL_WEIGHT_OZ = 28.89;
static const float NEWTONS_PER_OZ = 0.2780139;
static const int   FSR_SAMPLES   = 16;
static KalmanFilter weightFilter(0.0001, 0.01, 1.0, 0.0);
static bool weightFilterPrimed = false;

// ---------- Presence ----------
static const float DETECT_CM  = 20.0;
static const float RELEASE_CM = 25.0;
static const uint8_t CONFIRM_COUNT = 3;
static const unsigned long PING_INTERVAL_MS = 60;
static const unsigned long ECHO_TIMEOUT_US  = 30000UL;
static const unsigned long SETTLE_MS        = 2000;
static bool objectPresent = false;
static uint8_t changeStreak = 0;
static float lastDist = -1;
static unsigned long lastPing = 0;
static unsigned long presentSince = 0;

// ================================================================
// INIT
// ================================================================
bool sensorsBegin() {
  analogReadResolution(ADC_BITS);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SCALE_ENABLE_PIN, OUTPUT);
  digitalWrite(SCALE_ENABLE_PIN, LOW);
muxBegin();

  Wire.begin();

  bmpOk = bmp.begin(0x76) || bmp.begin(0x77);
  if (bmpOk) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
  } else {
    Serial.println("BMP280 not found, check wiring!");
  }

  shtOk = sht31.begin(0x44);   // try 0x45 if your board has ADDR pulled high
  if (!shtOk) Serial.println("SHT31 not found, check wiring!");

  if (bmpOk) calibrateThermistorToBMP();

  return bmpOk && shtOk;
}

// ================================================================
// BMP280
// ================================================================
float readBMPTempC() {
  return bmpOk ? bmp.readTemperature() : NAN;
}

float readBMPTempF() {
  return bmpOk ? (bmp.readTemperature() * 1.8f) + 32.0f : NAN;
}

float readBMPPressure_hPa() {
  return bmpOk ? bmp.readPressure() / 100.0f : NAN;
}

// ================================================================
// SHT31
// ================================================================
// CP2 transfer function: SHT31 = SHT31_SLOPE * RH_ref + SHT31_INTERCEPT
// (fit against the Walmart hygrometer over 69-92 %RH). Inverting it converts an
// SHT31 reading back to the reference scale:  RH_ref = (SHT31 - intercept) / slope
#define SHT31_APPLY_CAL  1            // 0 = return the raw sensor value
static const float SHT31_SLOPE     = 1.32f;
static const float SHT31_INTERCEPT = -33.2f;

float readHumidityRH() {
  if (!shtOk) return NAN;
  float raw = sht31.readHumidity();
  if (isnan(raw)) return NAN;
#if SHT31_APPLY_CAL
  float rh = (raw - SHT31_INTERCEPT) / SHT31_SLOPE;
  return constrain(rh, 0.0f, 100.0f);
#else
  return raw;
#endif
}

float readHumidityRawRH() {          // uncorrected value, handy for logging/comparison
  return shtOk ? sht31.readHumidity() : NAN;
}

float readSHT31TempC() {
  if (!shtOk) return NAN;
  return sht31.readTemperature();
}

// ================================================================
// THERMISTOR
// ================================================================
static float voltageToTempF(float v) {
  return (17.43f * v) + 35.83f + tempOffset;
}

ThermistorReading readThermistor() {
  ThermistorReading r;
  r.vRaw    = muxReadVolts(MUX_THERMISTOR, 10, 5);
  r.vKalman = thermFilter.update(r.vRaw);
  r.tempF   = voltageToTempF(r.vKalman);
  return r;
}

float readThermistorTempF() {
  return readThermistor().tempF;
}

void calibrateThermistorToBMP() {
  Serial.println("Calibrating thermistor against BMP280...");
  delay(1000);

  float bmpSum = 0, thermSum = 0;
  const int samples = 20;

  for (int i = 0; i < samples; i++) {
    float bmpF = readBMPTempF();
    float vRaw = muxReadVolts(MUX_THERMISTOR, 10, 5);
    float vK   = thermFilter.update(vRaw);
    bmpSum   += bmpF;
    thermSum += (17.43f * vK) + 35.83f;   // uncalibrated
    delay(100);
  }

  tempOffset = (bmpSum / samples) - (thermSum / samples);
  Serial.print("Thermistor offset applied: ");
  Serial.print(tempOffset, 2);
  Serial.println(" F");
}

// ================================================================
// HC-SR04
// ================================================================
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1;
  return duration * 0.0343f / 2.0f;
}

static void onPresenceChanged(bool present) {
  digitalWrite(SCALE_ENABLE_PIN, present ? HIGH : LOW);
  if (present) {
    presentSince = millis();
    weightFilterPrimed = false;   // restart the weight filter for each new placement
    Serial.println("Object detected - scale ACTIVE (settling...)");
  } else {
    Serial.println("Object removed - scale IDLE");
  }
}

bool presenceUpdate() {
  unsigned long now = millis();
  if (now - lastPing < PING_INTERVAL_MS) return objectPresent;
  lastPing = now;

  float d = readDistanceCm();
  lastDist = d;
  bool wantPresent = objectPresent;

  if (d < 0) {
    wantPresent = false;
  } else if (!objectPresent && d <= DETECT_CM) {
    wantPresent = true;
  } else if (objectPresent && d >= RELEASE_CM) {
    wantPresent = false;
  }

  if (wantPresent != objectPresent) {
    if (++changeStreak >= CONFIRM_COUNT) {
      objectPresent = wantPresent;
      changeStreak = 0;
      onPresenceChanged(objectPresent);
    }
  } else {
    changeStreak = 0;
  }
  return objectPresent;
}

bool isObjectPresent() { return objectPresent; }
float lastDistanceCm() { return lastDist; }

bool scaleReady() {
  return objectPresent && (millis() - presentSince >= SETTLE_MS);
}

// ================================================================
// FSR SCALE
// ================================================================
ScaleReading readScale() {
  ScaleReading r = {false, 0, 0, 0, 0, 0};
  if (!scaleReady()) return r;

  float counts = muxReadCounts(MUX_FSR, FSR_SAMPLES, 0);
  r.rawADC = (int)counts;
  r.vRaw   = muxCountsToVolts(MUX_FSR, counts);

  if (!weightFilterPrimed) {
    weightFilter.reset(r.vRaw);
    weightFilterPrimed = true;
  }
  r.vKalman = weightFilter.update(r.vRaw);

  r.weightOz = 0.0;
  if (r.vKalman > V_ZERO) {
    float sensitivity = (V_CAL - V_ZERO) / CAL_WEIGHT_OZ;   // V per oz
    r.weightOz = (r.vKalman - V_ZERO) / sensitivity;
  }
  r.forceN = r.weightOz * NEWTONS_PER_OZ;
  r.valid  = true;
  return r;
}

float readWeightOz() {
  ScaleReading r = readScale();
  return r.valid ? r.weightOz : 0.0f;
}

// ================================================================
// THERMISTOR MODEL HELPERS (from cp3demoFINAL)
// ================================================================
float calculateVoltageFromTempF(float tempF) {
  return (tempF - 35.83f - tempOffset) / 17.43f;
}

int calculateADCFromTempF(float tempF) {
  float v = calculateVoltageFromTempF(tempF);
  int adc = round(v * (ADC_MAX / V_SUPPLY));
  return constrain(adc, 0, (int)ADC_MAX);
}

float calculateOriginalVOutFromTempF(float tempF) {
  float tK  = (tempF - 32.0f) / 1.8f + 273.15f;
  float t0K = T0_C + 273.15f;
  float rT  = R0 * exp(BETA * ((1.0f / tK) - (1.0f / t0K)));
  float rEq = (rT * R_P) / (rT + R_P);
  float vA  = V_SUPPLY * (R_BOTTOM / (rEq + R_BOTTOM));
  return V_REF + GAIN * (vA - V_REF);
}
