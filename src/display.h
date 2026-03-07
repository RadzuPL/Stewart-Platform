#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SCSCL.h>

extern Adafruit_SSD1306 display;

void displayInit();
void oledShowBoot();
void updateOLED(int servoCount, int foundIDs[], bool torqueEnabled, int globalSpeed, SCSCL &sc);

#endif