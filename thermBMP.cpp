#include <Wire.h>
#include <Adafruit_BMP280.h>
#include <WiFiS3.h>
#include "ThingSpeak.h"

// ================================================================
// HARDWARE INITIALIZATION
// ================================================================
Adafruit_BMP280 bmp; // I2C BMP280 Sensor Object

// ================================================================
// KALMAN FILTER CLASS
// ================================================================
class KalmanFilter {
  private:
    float q; // Process noise covariance
    float r; // Measurement noise covariance
    float x; // Estimated state value
    float p; // Estimation error covariance
    float k; // Kalman gain

  public:
    KalmanFilter(float processNoise, float measurementNoise, float initialError, float initialValue) {
      q = processNoise;
      r = measurementNoise;
      p = initialError;
      x = initialValue;
    }

    float update(float measurement) {
      p = p + q;
      k = p / (p + r);
      x = x + k * (measurement - x);
      p = (1.0 - k) * p; // Fixed subtraction syntax
      return x;
    }
};

// Initialize Kalman filter for A0 Voltage (Q=0.0001, R=0.01, P=1.0, Initial X=2.5V)
KalmanFilter voltageFilter(0.0001, 0.01, 1.0, 2.5);

// ================================================================
// NETWORK & THINGSPEAK CONFIGURATION
// ================================================================
const char SECRET_SSID[] = "thumbelina";
const char SECRET_PASS[] = "helloworld1234";

unsigned long myChannelNumber = 3493387;             // Your ThingSpeak Channel ID
const char myWriteAPIKey[]    = "BQV8M2I4V19LSHL0"; // Your ThingSpeak Write API Key

WiFiClient client;

// ================================================================
// CIRCUIT & CALIBRATION CONSTANTS
// ================================================================
const float V_SUPPLY = 5.00;     // System rail voltage (V)
const float V_REF    = 2.50;     // Buffered reference voltage (V)
const float GAIN     = 3.00;     // INA121 amplifier gain (Rg = 25k)

const float R_BOTTOM = 448.0;    // Bottom-Right fixed resistor (Ohms)
const float R_P      = 821.0;    // Parallel linearization resistor (Ohms)

// NTC Thermistor Specifications (1k @ 25°C)
const float R0       = 1000.0;   // Nominal resistance at 25°C (Ohms)
const float T0_C     = 26.6;     // Nominal temperature (°C)
const float BETA     = 3636.0;   // Thermistor Beta value (K)

// Timing
unsigned long lastPostTime = 0;
const unsigned long postInterval = 15000; // 15 seconds minimum for ThingSpeak free tier

// ================================================================
// TRANSFER FUNCTION: Signal Voltage -> Temperature (°F)
// ================================================================
float calculateTemperatureF(float vOut) {
  // 1. Calculate input differential voltage to amplifier
  float vDiff = (vOut - V_REF) / GAIN; 
  float vA = V_REF + vDiff; 

  if (vA <= 0.1 || vA >= 4.9) return -999.0; // Out-of-bounds guard

  // 2. Reverse-calculate thermistor resistance R_T
  float rEq = R_BOTTOM * ((V_SUPPLY / vA) - 1.0);
  float invRT = (1.0 / rEq) - (1.0 / R_P);
  if (invRT <= 0) return -999.0; 
  float rT = 1.0 / invRT;

  // 3. Steinhart-Hart / Beta equation calculation
  float steinhart = log(rT / R0) / BETA;
  steinhart += 1.0 / (T0_C + 273.15);
  float tempKelvin = 1.0 / steinhart;
  float tempCelsius = tempKelvin - 273.15;

  return (tempCelsius * 1.8) + 32.0;
}

void setup() {
  Serial.begin(9600);
  
  // Set ADC resolution matching hardware sampling (10-bit: 0-1023)
  analogReadResolution(10);

  // Initialize I2C Bus and BMP280 Reference Sensor
  Wire.begin();
  if (!bmp.begin(0x76)) { // Try 0x77 if 0x76 fails
    if (!bmp.begin(0x77)) {
      Serial.println("Could not find a valid BMP280 sensor, check wiring!");
    }
  }
  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);

  // Connect to Wi-Fi
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(SECRET_SSID);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");

  // Initialize ThingSpeak client
  ThingSpeak.begin(client);
}

void loop() {
  // 1. Read Real Temperature from BMP280 (°C to °F)
  float bmpTempC = bmp.readTemperature();
  float realTempF = (bmpTempC * 1.8) + 32.0;

  // 2. Read and average 10 ADC samples from circuit
  long adcSum = 0;
  for (int i = 0; i < 10; i++) {
    adcSum += analogRead(A0);
    delay(5);
  }
  int rawADC = adcSum / 10;

  // 3. Compute Unfiltered Voltage and Filtered Voltage
  float vRaw = rawADC * (V_SUPPLY / 1023.0);
  float vKalman = voltageFilter.update(vRaw);

  // 4. Compute Circuit-calculated Temperatures via Transfer Function
  float tempRaw = calculateTemperatureF(vRaw);
  float tempKalman = calculateTemperatureF(vKalman);

  // Output live debug data to Serial Monitor
  Serial.print("BMP Real Temp: "); Serial.print(realTempF, 2);
  Serial.print(" °F | vRaw: ");    Serial.print(vRaw, 3);
  Serial.print(" V | vKalman: ");  Serial.print(vKalman, 3);
  Serial.print(" V | tempKalman: "); Serial.print(tempKalman, 2);
  Serial.println(" °F");

  // 5. Post data to ThingSpeak every 15 seconds
  if (millis() - lastPostTime >= postInterval) {
    if (!isnan(realTempF) && realTempF > -40.0) {
      // Sensor & Pressure Fields
      ThingSpeak.setField(1, realTempF);                 // Field 1: BMP280 Real Temp (°F)
      ThingSpeak.setField(2, bmp.readPressure()/100.0F); // Field 2: Pressure (hPa)
      
      // Calculated Thermistor Temperatures
      ThingSpeak.setField(5, tempRaw);                   // Field 5: tempRaw (°F)
      ThingSpeak.setField(6, tempKalman);                // Field 6: tempKalman (°F)
      
      // Circuit Voltages
      ThingSpeak.setField(7, vRaw);                      // Field 7: vRaw (V)
      ThingSpeak.setField(8, vKalman);                   // Field 8: vKalman (V)
      
      int responseCode = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
      
      if (responseCode == 200) {
        Serial.println("ThingSpeak update successful.");
      } else {
        Serial.print("ThingSpeak update error. HTTP Code: ");
        Serial.println(responseCode);
      }
    } else {
      Serial.println("BMP280 reading invalid. Post skipped.");
    }
    lastPostTime = millis();
  }

  delay(200);
}
