#include "leds.h"

Adafruit_NeoPixel pixels(NUM_LEDS, NEO_PIN, NEO_GRB + NEO_KHZ800);

void ledsInit() {
  pixels.begin();
  pixels.setBrightness(20);
}

void updateLEDs(int servoCount, bool torqueEnabled) {
  pixels.clear();
  pixels.setBrightness(20);
  
  if(servoCount >= 6 && torqueEnabled) {
    int b = 10 + (millis() % 100);
    pixels.fill(pixels.Color(0, b, 0));
  } else if(servoCount > 0) {
    pixels.fill(pixels.Color(30, 30, 0));
  } else {
    if(millis() % 1000 < 500) {
      pixels.fill(pixels.Color(30, 0, 0));
    }
  }
  pixels.show();
}