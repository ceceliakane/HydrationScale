#include "oled.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

static Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);
static bool screenOn = true;      // logical state, toggled by a tap
static bool physOn = true;

bool oledBegin() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_I2C_ADDR)) {
    Serial.println("OLED not found - check wiring/address (try 0x3D).");
    return false;
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Hydration Scale");
  display.println("starting...");
  display.display();
  return true;
}

void oledMessage(const char* line1, const char* line2) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 20); display.println(line1);
  display.setCursor(0, 34); display.println(line2);
  display.display();
}

void oledToggle()        { screenOn = !screenOn; }
void oledSetOn(bool on)  { screenOn = on; }
bool oledIsOn()          { return screenOn; }

void oledRender(const HydrationState& h, float tempF, float rh,
                bool bottlePresent, bool scaleReady, float weightOz) {
  bool wantOn = screenOn;
  if (!wantOn) {
    if (physOn) { display.ssd1306_command(SSD1306_DISPLAYOFF); physOn = false; }
    return;
  }
  if (!physOn) { display.ssd1306_command(SSD1306_DISPLAYON); physOn = true; }

  display.clearDisplay();
  display.setTextSize(1);

  display.setCursor(0, 0);
  display.print("Goal: ");  display.print(h.goalOz, 0);  display.print(" oz");

  display.setCursor(0, 11);
  display.print("Drank: "); display.print(h.intakeOz, 1); display.print(" oz");

  // progress bar
  display.drawRect(0, 24, 128, 12, SSD1306_WHITE);
  int fill = (int)(h.scorePct / 100.0f * 124.0f);
  if (fill < 2 && h.scorePct > 0.0f) fill = 2;          // make small progress visible
  display.fillRect(2, 26, fill, 8, SSD1306_WHITE);

  display.setCursor(0, 40);
  display.print(h.scorePct, 1); display.print("%");

  // live bottle status (right side of this line)
  display.setCursor(74, 40);
  if (!bottlePresent)      display.print("no bottle");
  else if (!scaleReady)    display.print("settling");
  else { display.print(weightOz, 1); display.print(" oz"); }

  display.setCursor(0, 52);
  if (isnan(tempF) || isnan(rh)) {
    display.print("Env: --");
  } else {
    display.print(tempF, 0); display.print("F  ");
    display.print(rh, 0);    display.print("%RH  x");
    display.print(h.envMultiplier, 2);
  }
  display.display();
}
