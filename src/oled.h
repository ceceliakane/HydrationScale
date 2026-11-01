#pragma once
#include <Arduino.h>
#include "hydration.h"

// The "0x78" on your display is the 8-bit (write) address. Arduino/Adafruit
// libraries want the 7-bit address: 0x78 >> 1 = 0x3C.
#define OLED_I2C_ADDR     0x3C
#define OLED_WIDTH        128
#define OLED_HEIGHT       64          // change to 32 if yours is the 128x32 module

bool oledBegin();
void oledMessage(const char* line1, const char* line2);   // quick status text
void oledToggle();            // tap: on -> off, off -> on
void oledSetOn(bool on);
bool oledIsOn();
void oledRender(const HydrationState& h, float tempF, float rh,
                bool bottlePresent, bool scaleReady, float weightOz);
