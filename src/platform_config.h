#ifndef PLATFORM_CONFIG_H
#define PLATFORM_CONFIG_H

#include <Arduino.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(x) ((x) * M_PI / 180.0)

// `POW` — szybsze niż pow() z cmath dla małych wykładników.
#define POW(base, exp)                              \
    (exp == 2   ? (base) * (base)                   \
     : exp == 3 ? (base) * (base) * (base)          \
     : exp == 4 ? (base) * (base) * (base) * (base) \
                : -1)

// === LICZBA SERW ===
#define NB_SERVOS 6

// === MODELE DANYCH ===

struct ServoCalibration {
  int id;
  bool inverted;
  int offset;       // Offset pozycji HOME względem 511 (środek zakresu SCS).
  int minPos;       // Dolny limit pozycji SCS (z pomiarów mechanicznych).
  int maxPos;       // Górny limit pozycji SCS (z pomiarów mechanicznych).
  int homePos;      // Zmierzona pozycja HOME w jednostkach SCS.
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

// Kalibracja liniowa IK: pozycja = gain * kąt_rad + offset.
typedef struct {
  double gain;
  int offset;
} ik_calibration_t;

// === STAŁE GEOMETRII PLATFORMY ===

extern const double ARM_LENGTH;
extern const double ROD_LENGTH;
extern const double Z_HOME;
extern const double B_RAD;
extern const double P_RAD;
extern const double THETA_P;
extern const double THETA_B;

// === STAŁE SERW ===

extern const int SERVO_MIN_PWM;
extern const int SERVO_MAX_PWM;
extern const double SERVO_FULL_ANGULAR_RANGE;
extern const double SERVO_HALF_ANGULAR_RANGE;

// Gain IK: jednostki pozycji SCS na radian.
extern const double SERVO_GAIN;

// Kalibracja liniowa IK per serwo (gain + offset z uwzględnieniem inwersji).
extern const ik_calibration_t SERVO_IK_CALIBRATION[NB_SERVOS];

// Limity pozycji per serwo (dolny/górny) — walidacja w IK.
extern const uint16_t SERVO_PWM_LOWER_LIMIT[NB_SERVOS];
extern const uint16_t SERVO_PWM_UPPER_LIMIT[NB_SERVOS];

// Orientacje ramion serw w płaszczyźnie XY bazy (radiany).
extern const double THETA_S[NB_SERVOS];

// === LIMITY RUCHU PLATFORMY (IK) ===

extern const double HX_X_MIN;
extern const double HX_X_MAX;
extern const double HX_X_MID;
extern const double HX_X_BAND;

extern const double HX_Y_MIN;
extern const double HX_Y_MAX;
extern const double HX_Y_MID;
extern const double HX_Y_BAND;

extern const double HX_Z_MIN;
extern const double HX_Z_MAX;
extern const double HX_Z_MID;
extern const double HX_Z_BAND;

extern const double HX_A_MIN;
extern const double HX_A_MAX;
extern const double HX_A_MID;
extern const double HX_A_BAND;

extern const double HX_B_MIN;
extern const double HX_B_MAX;
extern const double HX_B_MID;
extern const double HX_B_BAND;

extern const double HX_C_MIN;
extern const double HX_C_MAX;
extern const double HX_C_MID;
extern const double HX_C_BAND;

// === DANE GLOBALNE (runtime) ===

extern ServoCalibration gServoCalibration[NB_SERVOS];
extern MotionLimits gMotionLimits;

// === ALGORYTM IK ===
#define ALGO 3

// === API ===

bool isPlatformGeometryReady();
bool areServoCalibrationsReady();
bool getServoIndexByID(int servoID, int &indexOut);
int clampServoPositionByIndex(int index, int position);

#endif