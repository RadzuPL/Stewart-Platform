#ifndef MOTION_H
#define MOTION_H

#include <Arduino.h>
#include <SCSCL.h>

struct Pose6D {
  float xMm;
  float yMm;
  float zMm;
  float rollDeg;
  float pitchDeg;
  float yawDeg;
};

/**
 * Inicjalizuje stan modułu ruchu (arm/dry-run).
 * Wejście: brak.
 * Wyjście: brak.
 */
void motionInit();

/**
 * Obsługuje komendy ruchu z prefiksem M.
 * Wejście: params - tekst po literze 'M', sc - interfejs serw,
 *          globalSpeed - prędkość ruchu, torqueEnabled - aktualny stan torque.
 * Wyjście: true gdy komenda została poprawnie obsłużona.
 */
bool handleMotionCommand(const String &params, SCSCL &sc, int globalSpeed, bool torqueEnabled);

#endif