#pragma once
#include <Arduino.h>

void iotBegin();                          // connect Wi-Fi + start ThingSpeak
void iotSetField(int field, float value); // ignores NaN so bad reads don't get logged
int  iotPublish();                        // returns HTTP code (200 = OK), reconnects if Wi-Fi dropped
