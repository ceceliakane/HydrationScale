#include <WiFiS3.h>
#include "ThingSpeak.h"

// Wi-Fi Credentials
const char* WIFI_SSID = "thumbelina";
const char* WIFI_PASS = "helloworld1234";

// ThingSpeak Configuration
WiFiClient client;
unsigned long CHANNEL_ID = 3493387;
const char* WRITE_API_KEY = "BQV8M2I4V19LSHL0";

// Hardware & ADC Settings (Uno R4 defaults to 5V reference)
const float V_REF = 5.0;
const int ADC_BITS = 12;
const float ADC_MAX = 4095.0;
const int FSR_PIN = A1;             // Analog input pin from op-amp output

// Two-Point Calibration Data
const float V_ZERO = 0.22;
const float V_CAL = 1.47;
const float CAL_WEIGHT_OZ = 28.89;

// Unit Conversion
const float NEWTONS_PER_OZ = 0.2780139;

// ---------- HC-SR04 presence sensor ----------
const uint8_t TRIG_PIN = 9;
const uint8_t ECHO_PIN = 10;
const uint8_t SCALE_ENABLE_PIN = 7;  // drives indicator LED or scale/op-amp power switch

const float DETECT_CM  = 20.0;       // "present" when closer than this
const float RELEASE_CM = 25.0;       // "gone" when farther than this (hysteresis)
const uint8_t CONFIRM_COUNT = 3;     // consecutive readings required to change state
const unsigned long PING_INTERVAL_MS = 60;
const unsigned long ECHO_TIMEOUT_US  = 30000UL;

// ---------- Scale timing ----------
const unsigned long SETTLE_MS          = 2000;   // wait after detection before measuring
const unsigned long LIVE_PRINT_MS      = 500;    // serial readout rate while present
const unsigned long UPLOAD_INTERVAL_MS = 15000;  // ThingSpeak free-tier minimum
const int ADC_SAMPLES = 16;                      // averaging for a stable reading

bool objectPresent = false;
uint8_t changeStreak = 0;
unsigned long lastPing = 0;
unsigned long presentSince = 0;
unsigned long lastLivePrint = 0;
unsigned long lastUpload = 0;
bool hasUploaded = false;

// ---------- Presence detection ----------
float readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return -1;            // no echo
  return duration * 0.0343f / 2.0f;
}

void onPresenceChanged(bool present) {
  digitalWrite(SCALE_ENABLE_PIN, present ? HIGH : LOW);
  if (present) {
    presentSince = millis();
    Serial.println("Object detected - scale ACTIVE (settling...)");
  } else {
    Serial.println("Object removed - scale IDLE");
  }
}

void updatePresence() {
  float d = readDistanceCm();
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
}

// ---------- Scale ----------
struct Reading {
  int rawADC;
  float vOut;
  float weightOz;
  float forceNewtons;
};

Reading readScale() {
  Reading r;

  // Average several samples to reduce noise
  long sum = 0;
  for (int i = 0; i < ADC_SAMPLES; i++) {
    sum += analogRead(FSR_PIN);
  }
  r.rawADC = sum / ADC_SAMPLES;

  r.vOut = (r.rawADC * V_REF) / ADC_MAX;

  r.weightOz = 0.0;
  if (r.vOut > V_ZERO) {
    float sensitivity = (V_CAL - V_ZERO) / CAL_WEIGHT_OZ;  // Volts per Ounce
    r.weightOz = (r.vOut - V_ZERO) / sensitivity;
  }
  r.forceNewtons = r.weightOz * NEWTONS_PER_OZ;
  return r;
}

void printReading(const Reading& r) {
  Serial.print("Raw ADC: ");
  Serial.print(r.rawADC);
  Serial.print(" | Voltage: ");
  Serial.print(r.vOut, 3);
  Serial.print(" V | Force: ");
  Serial.print(r.forceNewtons, 3);
  Serial.print(" N | Weight: ");
  Serial.print(r.weightOz, 2);
  Serial.println(" oz");
}

void uploadReading(const Reading& r) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi lost - reconnecting...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    return;  // try again next interval
  }

  ThingSpeak.setField(3, r.vOut);
  ThingSpeak.setField(4, r.forceNewtons);
  ThingSpeak.setField(5, r.weightOz);

  int httpCode = ThingSpeak.writeFields(CHANNEL_ID, WRITE_API_KEY);
  if (httpCode == 200) {
    Serial.println("ThingSpeak update successful.");
  } else {
    Serial.print("ThingSpeak error. HTTP Code: ");
    Serial.println(httpCode);
  }
}

// ---------- Arduino entry points ----------
void setup() {
  Serial.begin(9600);

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SCALE_ENABLE_PIN, OUTPUT);
  digitalWrite(SCALE_ENABLE_PIN, LOW);

  analogReadResolution(ADC_BITS);

  WiFi.begin(WIFI_SSID, WIFI_PASS);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");

  ThingSpeak.begin(client);
  Serial.println("Waiting for object...");
}

void loop() {
  unsigned long now = millis();

  // Non-blocking presence check
  if (now - lastPing >= PING_INTERVAL_MS) {
    lastPing = now;
    updatePresence();
  }

  // Scale only runs while an object is present and has settled
  if (objectPresent && (now - presentSince >= SETTLE_MS)) {

    if (now - lastLivePrint >= LIVE_PRINT_MS) {
      lastLivePrint = now;
      printReading(readScale());
    }

    // Upload on first settled reading, then every 15 s while present.
    // Also respects the 15 s limit if the object is removed and replaced quickly.
    if (!hasUploaded || (now - lastUpload >= UPLOAD_INTERVAL_MS)) {
      Reading r = readScale();
      uploadReading(r);
      lastUpload = millis();
      hasUploaded = true;
    }
  }
}
