#ifndef LEDS_H
#define LEDS_H

#include <Adafruit_NeoPixel.h>

#define NEO_PIN     23
#define NUM_LEDS    2

extern Adafruit_NeoPixel pixels;

void ledsInit();
void updateLEDs(int servoCount, bool torqueEnabled);

#endif