#include "iot.h"
#include <WiFiS3.h>
#include "ThingSpeak.h"
#include "secrets.h"

static WiFiClient client;

void iotBegin() {
  Serial.print("Connecting to Wi-Fi: ");
  Serial.println(SECRET_SSID);
  WiFi.begin(SECRET_SSID, SECRET_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi!");
  ThingSpeak.begin(client);
}

void iotSetField(int field, float value) {
  if (!isnan(value)) ThingSpeak.setField(field, value);
}

int iotPublish() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi lost - reconnecting...");
    WiFi.begin(SECRET_SSID, SECRET_PASS);
    return -1;
  }
  int code = ThingSpeak.writeFields(TS_CHANNEL_ID, TS_WRITE_API_KEY);
  if (code == 200) Serial.println("ThingSpeak update successful.");
  else { Serial.print("ThingSpeak error. HTTP Code: "); Serial.println(code); }
  return code;
}
