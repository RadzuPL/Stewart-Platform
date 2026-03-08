#include "platform_config.h"

// Geometria platformy na podstawie dokumentacji projektu referencyjnego.
// Wartości pozostają w float (bez zaokrąglania), bo są używane w obliczeniach.
PlatformGeometry gPlatformGeometry = {
  166.6f,   // baseRadiusMm (B_RAD)
  147.89f,  // platformRadiusMm (P_RAD)
  65.0f,    // rodLengthMm (ROD_LENGTH)
  19.0f,    // hornLengthMm (ARM_LENGTH)
  true      // configured
};

// Kalibracja serw:
// - 11/13/15 mają ruch odwrócony,
// - homePos ustawione na 512,
// - zakresy min/max ustawione wg aktualnie bezpiecznych pomiarów mechaniki.
// offsetDeg służy do precyzyjnego strojenia "zera" każdego serwa.
ServoCalibration gServoCalibration[6] = {
  {11, true,  0.0f, 174, 562, 512, true},
  {12, false, 0.0f, 462, 850, 512, true},
  {13, true,  0.0f, 174, 562, 512, true},
  {14, false, 0.0f, 462, 850, 512, true},
  {15, true,  0.0f, 174, 562, 512, true},
  {16, false, 0.0f, 462, 850, 512, true}
};

// Limity ruchu platformy: tymczasowe, do aktualizacji po testach mechanicznych.
MotionLimits gMotionLimits = {
  20.0f, // xMaxMm
  20.0f, // yMaxMm
  20.0f, // zMaxMm
  8.0f,  // rollMaxDeg
  8.0f,  // pitchMaxDeg
  8.0f   // yawMaxDeg
};

/**
 * Sprawdza, czy geometria platformy została skonfigurowana.
 * Wejście: brak.
 * Wyjście: true gdy geometria gotowa, false gdy brak danych.
 */
bool isPlatformGeometryReady() {
  return gPlatformGeometry.configured;
}

/**
 * Sprawdza, czy wszystkie serwa mają gotową kalibrację.
 * Wejście: brak.
 * Wyjście: true gdy każde serwo ma calibrated=true.
 */
bool areServoCalibrationsReady() {
  for (int i = 0; i < 6; i++) {
    if (!gServoCalibration[i].calibrated) {
      return false;
    }
  }
  return true;
}

/**
 * Zwraca indeks serwa w tablicy kalibracyjnej na podstawie ID.
 * Wejście: servoID - ID serwa.
 * Wyjście: true gdy znaleziono, indexOut ustawiony na indeks 0..5.
 */
bool getServoIndexByID(int servoID, int &indexOut) {
  for (int i = 0; i < 6; i++) {
    if (gServoCalibration[i].id == servoID) {
      indexOut = i;
      return true;
    }
  }
  return false;
}

/**
 * Ogranicza pozycję do bezpiecznego zakresu zdefiniowanego dla serwa.
 * Wejście: index - indeks serwa 0..5, position - pozycja docelowa.
 * Wyjście: pozycja przycięta do [minPos..maxPos].
 */
int clampServoPositionByIndex(int index, int position) {
  if (index < 0 || index >= 6) {
    return position;
  }

  int minPos = gServoCalibration[index].minPos;
  int maxPos = gServoCalibration[index].maxPos;

  if (position < minPos) return minPos;
  if (position > maxPos) return maxPos;
  return position;
}