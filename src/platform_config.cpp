#include "platform_config.h"

// =============================================================================
// GEOMETRIA PLATFORMY
// Wszystkie wymiary w milimetrach, kąty w radianach.
// Źródło: pomiary fizyczne, docs/platform-parameters.md (archiwalne).
// =============================================================================

const double ARM_LENGTH = 19.0;          // Długość ramienia serwa (horn) [mm].
const double ROD_LENGTH = 65.0;          // Długość cięgna (push rod) [mm].
const double Z_HOME     = 64.0;          // Wysokość platformy w pozycji HOME [mm].
const double B_RAD      = 83.3;          // Promień bazy (odległość osi serwa od środka) [mm].
const double P_RAD      = 71.225;        // Promień platformy (odległość przegubu od środka) [mm].
const double THETA_P    = DEG2RAD(18.93);// Kąt offsetu przegubów platformy [rad].
const double THETA_B    = DEG2RAD(25.0); // Kąt offsetu osi serw na bazie [rad].

// =============================================================================
// PARAMETRY SERW SCS225
// Zakres kątowy: ~300° (0..1023 pozycji). Pozycja 512 = środek zakresu.
// =============================================================================

const int SERVO_MIN_PWM = 0;
const int SERVO_MAX_PWM = 1023;

const double SERVO_FULL_ANGULAR_RANGE = DEG2RAD(300.0);
const double SERVO_HALF_ANGULAR_RANGE = SERVO_FULL_ANGULAR_RANGE / 2.0;

// Gain IK: 1023 pozycji / 300° w radianach ≈ 195.38 pos/rad.
const double SERVO_GAIN = (double)(SERVO_MAX_PWM - SERVO_MIN_PWM) / SERVO_FULL_ANGULAR_RANGE;

// =============================================================================
// KALIBRACJA SERW
// Home, min, max — z pomiarów mechanicznych (serwa bez obciążenia platformą).
// Offset = homePos - 511 (środek zakresu SCS).
//
// Home:  P11:530, P12:540, P13:515, P14:520, P15:480, P16:540
// Min:   P11:400, P12:290, P13:385, P14:270, P15:350, P16:290
// Max:   P11:780, P12:670, P13:765, P14:650, P15:730, P16:670
// =============================================================================

ServoCalibration gServoCalibration[NB_SERVOS] = {
  // id, inverted, offset, minPos, maxPos, homePos, calibrated
  {11, true,   19,  400, 780, 530, true},
  {12, false,  29,  290, 670, 540, true},
  {13, true,    4,  385, 765, 515, true},
  {14, false,   9,  270, 650, 520, true},
  {15, true,  -31,  350, 730, 480, true},
  {16, false,  29,  290, 670, 540, true}
};

// =============================================================================
// KALIBRACJA IK PER SERWO
// Pozycja SCS = gain * kąt_rad + offset.
// Serwa nieparzyste (11,13,15): ruch odwrócony (gain ujemny).
// Serwa parzyste (12,14,16): gain dodatni.
// Offset = SERVO_MAX_PWM/2 + offset kalibracyjny danego serwa.
// =============================================================================

const ik_calibration_t SERVO_IK_CALIBRATION[NB_SERVOS] = {
  {-SERVO_GAIN, SERVO_MAX_PWM / 2 + 19},   // ID 11 (inverted)
  { SERVO_GAIN, SERVO_MAX_PWM / 2 + 29},   // ID 12
  {-SERVO_GAIN, SERVO_MAX_PWM / 2 +  4},   // ID 13 (inverted)
  { SERVO_GAIN, SERVO_MAX_PWM / 2 +  9},   // ID 14
  {-SERVO_GAIN, SERVO_MAX_PWM / 2 - 31},   // ID 15 (inverted)
  { SERVO_GAIN, SERVO_MAX_PWM / 2 + 29}    // ID 16
};

// =============================================================================
// LIMITY POZYCJI PER SERWO (walidacja w IK)
// Wartości dolne/górne w jednostkach SCS, niezależne od kierunku ruchu.
// =============================================================================

const uint16_t SERVO_PWM_LOWER_LIMIT[NB_SERVOS] = {
  400,  // ID 11
  290,  // ID 12
  385,  // ID 13
  270,  // ID 14
  350,  // ID 15
  290   // ID 16
};

const uint16_t SERVO_PWM_UPPER_LIMIT[NB_SERVOS] = {
  780,  // ID 11
  670,  // ID 12
  765,  // ID 13
  650,  // ID 14
  730,  // ID 15
  670   // ID 16
};

// =============================================================================
// ORIENTACJE RAMION SERW
// Kąty osi symetrii ramion w płaszczyźnie XY bazy (radiany).
// Zależne od fizycznego rozmieszczenia serw.
// =============================================================================

const double THETA_S[NB_SERVOS] = {
  DEG2RAD(-60),
  DEG2RAD(120),
  DEG2RAD(180),
  DEG2RAD(0),
  DEG2RAD(60),
  DEG2RAD(-120)
};

// =============================================================================
// LIMITY RUCHU PLATFORMY (IK)
// Wartości bezpiecznego zakresu ruchu. Do aktualizacji po testach mechanicznych.
// =============================================================================

const double HX_X_MIN  = -20.0;
const double HX_X_MAX  =  20.0;
const double HX_X_MID  = (HX_X_MAX + HX_X_MIN) / 2.0;
const double HX_X_BAND = HX_X_MAX - HX_X_MIN;

const double HX_Y_MIN  = -20.0;
const double HX_Y_MAX  =  20.0;
const double HX_Y_MID  = (HX_Y_MAX + HX_Y_MIN) / 2.0;
const double HX_Y_BAND = HX_Y_MAX - HX_Y_MIN;

const double HX_Z_MIN  = -20.0;
const double HX_Z_MAX  =  20.0;
const double HX_Z_MID  = (HX_Z_MAX + HX_Z_MIN) / 2.0;
const double HX_Z_BAND = HX_Z_MAX - HX_Z_MIN;

const double HX_A_MIN  = DEG2RAD(-10.0);
const double HX_A_MAX  = DEG2RAD(10.0);
const double HX_A_MID  = (HX_A_MAX + HX_A_MIN) / 2.0;
const double HX_A_BAND = HX_A_MAX - HX_A_MIN;

const double HX_B_MIN  = DEG2RAD(-10.0);
const double HX_B_MAX  = DEG2RAD(10.0);
const double HX_B_MID  = (HX_B_MAX + HX_B_MIN) / 2.0;
const double HX_B_BAND = HX_B_MAX - HX_B_MIN;

const double HX_C_MIN  = DEG2RAD(-10.0);
const double HX_C_MAX  = DEG2RAD(10.0);
const double HX_C_MID  = (HX_C_MAX + HX_C_MIN) / 2.0;
const double HX_C_BAND = HX_C_MAX - HX_C_MIN;

// =============================================================================
// LIMITY RUCHU (runtime, do clampowania komend M)
// =============================================================================

MotionLimits gMotionLimits = {
  20.0f,  // xMaxMm
  20.0f,  // yMaxMm
  20.0f,  // zMaxMm
  10.0f,   // rollMaxDeg
  10.0f,   // pitchMaxDeg
  10.0f    // yawMaxDeg
};

// =============================================================================
// API
// =============================================================================

/**
 * Sprawdza, czy geometria platformy jest skonfigurowana.
 * Wejście: brak.
 * Wyjście: true (geometria jest stała, zawsze gotowa).
 */
bool isPlatformGeometryReady() {
  return true;
}

/**
 * Sprawdza, czy wszystkie serwa mają gotową kalibrację.
 * Wejście: brak.
 * Wyjście: true gdy każde serwo ma calibrated=true.
 */
bool areServoCalibrationsReady() {
  for (int i = 0; i < NB_SERVOS; i++) {
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
  for (int i = 0; i < NB_SERVOS; i++) {
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
  if (index < 0 || index >= NB_SERVOS) {
    return position;
  }

  int minPos = gServoCalibration[index].minPos;
  int maxPos = gServoCalibration[index].maxPos;

  if (position < minPos) return minPos;
  if (position > maxPos) return maxPos;
  return position;
}