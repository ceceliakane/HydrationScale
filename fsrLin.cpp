#include <WiFiS3.h>
#include "ThingSpeak.h"

// Wi-Fi Credentials
const char* WIFI_SSID = "thumbelina";
const char* WIFI_PASS = "helloworld1234";

// ThingSpeak Configuration
WiFiClient client;
unsigned long CHANNEL_ID = 3493387;             // Replace with your Channel ID
const char* WRITE_API_KEY = "BQV8M2I4V19LSHL0"; // Replace with your Write API Key

// Hardware & ADC Settings (Uno R4 defaults to 5V reference)
const float V_REF = 5.0;            
const int ADC_BITS = 12;            // Setting ADC to 12-bit mode
const float ADC_MAX = 4095.0;       // 2^12 - 1
const int FSR_PIN = A1;             // Analog input pin from op-amp output

// Two-Point Calibration Data
const float V_ZERO = 0.22;          // Voltage at 0 oz (baseline offset)
const float V_CAL = 1.47;           // Voltage with known calibration weight
const float CAL_WEIGHT_OZ = 28.89;   // Known calibration weight in ounces

// Unit Conversion
const float NEWTONS_PER_OZ = 0.2780139; // 1 oz-force ≈ 0.278014 N

void setup() {
  Serial.begin(9600);
  
  // Set Uno R4 ADC resolution from default 10-bit to 12-bit
  analogReadResolution(ADC_BITS);

  // Connect to Wi-Fi using standard Uno R4 WiFiS3 stack
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  ThingSpeak.begin(client);
}

void loop() {
  // 1. Read 12-bit raw ADC value
  int rawADC = analogRead(FSR_PIN);
  
  // 2. Convert ADC counts to Op-Amp output voltage
  float vOut = (rawADC * V_REF) / ADC_MAX;
  
  // 3. Linearized weight and force calculations
  float weightOz = 0.0;
  if (vOut > V_ZERO) {
    float sensitivity = (V_CAL - V_ZERO) / CAL_WEIGHT_OZ; // Volts per Ounce
    weightOz = (vOut - V_ZERO) / sensitivity;
  }
  float forceNewtons = weightOz * NEWTONS_PER_OZ;

  // Print debug values to Serial Monitor
  Serial.print("Raw ADC: ");
  Serial.print(rawADC);
  Serial.print(" | Voltage: ");
  Serial.print(vOut, 3);
  Serial.print(" V | Force: ");
  Serial.print(forceNewtons, 3);
  Serial.print(" N | Weight: ");
  Serial.print(weightOz, 2);
  Serial.println(" oz");

  // 4. Push fields to ThingSpeak
  ThingSpeak.setField(3, vOut);          // Field 1: Voltage
  ThingSpeak.setField(4, forceNewtons);  // Field 2: Force (N)
  ThingSpeak.setField(5, weightOz);      // Field 3: Weight (oz)

  // 5. Send data
  int httpCode = ThingSpeak.writeFields(CHANNEL_ID, WRITE_API_KEY);
  if (httpCode == 200) {
    Serial.println("ThingSpeak update successful.");
  } else {
    Serial.print("ThingSpeak error. HTTP Code: ");
    Serial.println(httpCode);
  }

  // Rate-limiting pause (ThingSpeak free accounts require at least 15s intervals)
  delay(15000);
}
