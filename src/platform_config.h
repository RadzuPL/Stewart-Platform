#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <Arduino.h>

// === MODELE DANYCH ===
struct PlatformGeometry {
  float baseRadiusMm;
  float platformRadiusMm;
  float rodLengthMm;
  float hornLengthMm;
  bool configured;
};

struct ServoCalibration {
  int id;
  bool inverted;
  float offsetDeg;
  int minPos;
  int maxPos;
  int homePos;
  bool calibrated;
};

struct MotionLimits {
  float xMaxMm;
  float yMaxMm;
  float zMaxMm;
  float rollMaxDeg;
  float pitchMaxDeg;
  float yawMaxDeg;
};

// === DANE GLOBALNE ===
extern PlatformGeometry gPlatformGeometry;
extern ServoCalibration gServoCalibration[6];
extern MotionLimits gMotionLimits;

// === API ===
bool isPlatformGeometryReady();
bool areServoCalibrationsReady();
bool getServoIndexByID(int servoID, int &indexOut);
int clampServoPositionByIndex(int index, int position);

#endif