#pragma once
#include <Arduino.h>
#include "hydration.h"

// The "0x78" on your display is the 8-bit (write) address. Arduino/Adafruit
// libraries want the 7-bit address: 0x78 >> 1 = 0x3C.
#define OLED_I2C_ADDR     0x3C
#define OLED_WIDTH        128
#define OLED_HEIGHT       64          // change to 32 if yours is the 128x32 module
#define OLED_TIMEOUT_MS   15000

// 0 = screen turns on only after oledWake() (a confirmed touch), then times out.
// 1 = screen stays on (handy for debugging the display or the touch pad).
#define OLED_ALWAYS_ON    0

bool oledBegin();
void oledMessage(const char* line1, const char* line2);   // quick status text
void oledWake();      // call when a confirmed touch happens
void oledRender(const HydrationState& h, float tempF, float rh,
                bool bottlePresent, bool scaleReady, float weightOz);
