/**
 * Konfiguracja geometrii Stewart Platform dopasowana do tego projektu.
 * Parametry z dokumentacji: docs/platform-parameters.md
 *
 * Serwa: SCS225 (Feetech), protokół SCS, pozycja 0..1023, bus szeregowy.
 * Ramię: 19 mm, cięgno: 65 mm.
 * Baza: R=166.6 mm, platforma: R=147.89 mm.
 *
 * UWAGA: BP2_MAX = (ARM+ROD)^2 = (19+65)^2 = 7056 mm^2.
 * Odległości baza-platforma (BP) muszą być mniejsze niż 84mm we wszystkich DOF.
 * Przy dużych radiusach (B_RAD, P_RAD) i małych ramionach jest to trudne do spełnienia.
 * Jeśli IK zwraca -1, sprawdź proporcje geometrii lub zmień Z_HOME.
 */

#pragma once

/*
 * ======== SERVO SETTINGS ========
 */

#define NB_SERVOS 6

/*
 * Zakres kątowy serwa SCS225: ~300° (0..1023 pozycji).
 * Jeden krok = ~0.2932°.
 * Pozycja 512 odpowiada środkowi zakresu mechanicznego.
 *
 * Makro M_PI_OVER_180 zastępuje radians() z Arduino.h, aby unikać
 * zależności od kolejności includowania nagłówków.
 */
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(x) ((x) * M_PI / 180.0)

const double SERVO_FULL_ANGULAR_RANGE = DEG2RAD(300.0);
const double SERVO_HALF_ANGULAR_RANGE = SERVO_FULL_ANGULAR_RANGE / 2;

/*
 * Limity pozycji w jednostkach SCS (0..1023).
 * Przeliczenie: pozycja = kąt_rad * (1023 / 300°_in_rad)
 * Te wartości są używane do walidacji w IK, nie do clampowania
 * (clampowanie odbywa się w platform_config).
 */
const int SERVO_MIN_PWM = 0;
const int SERVO_MAX_PWM = 1023;

/*
 * Offsety kompensacyjne per serwo (w jednostkach pozycji SCS).
 * Ustawione na podstawie zmierzonego HOME:
 * P11:530, P12:540, P13:515, P14:520, P15:480, P16:540.
 * Referencja środka: SERVO_MAX_PWM / 2 = 511.
 */
const int PW_OFFSET[] = {
    19,   // ID 11: 530 - 511
    29,   // ID 12: 540 - 511
    4,    // ID 13: 515 - 511
    9,    // ID 14: 520 - 511
    -31,  // ID 15: 480 - 511
    29    // ID 16: 540 - 511
};

/*
 * Limity bezpieczne per serwo (z pomiarów mechanicznych).
 * Uwaga: to limity numeryczne PWM (dolny/górny), niezależne od kierunku ruchu.
 */
const uint16_t SERVO_PWM_LOWER_LIMIT[NB_SERVOS] = {
    400, // ID 11
    290, // ID 12
    385, // ID 13
    270, // ID 14
    350, // ID 15
    290  // ID 16
};

const uint16_t SERVO_PWM_UPPER_LIMIT[NB_SERVOS] = {
    780, // ID 11
    670, // ID 12
    765, // ID 13
    650, // ID 14
    730, // ID 15
    670  // ID 16
};

/*
 * Gain: jednostki pozycji SCS na radian.
 * 1023 pozycji / 300° = 1023 / (300 * PI/180) rad = ~195.38 pos/rad
 */
const double gain = (SERVO_MAX_PWM - SERVO_MIN_PWM) /
                    (SERVO_FULL_ANGULAR_RANGE);

/*
 * calibration_t
 */
typedef struct
{
    double gain;
    int offset;
} calibration_t;

/*
 * Kalibracja liniowa: pozycja = gain * kąt_rad + offset.
 * Serwa nieparzyste (11,13,15) mają ruch odwrócony (gain ujemny),
 * serwa parzyste (12,14,16) mają gain dodatni.
 * Offset ustawia pozycję środkową (home = 512 = SERVO_MAX_PWM/2).
 */
const calibration_t SERVO_CALIBRATION[NB_SERVOS] = {
    {-gain, SERVO_MAX_PWM / 2 + PW_OFFSET[0]},  // ID 11 (inverted)
    { gain, SERVO_MAX_PWM / 2 + PW_OFFSET[1]},  // ID 12
    {-gain, SERVO_MAX_PWM / 2 + PW_OFFSET[2]},  // ID 13 (inverted)
    { gain, SERVO_MAX_PWM / 2 + PW_OFFSET[3]},  // ID 14
    {-gain, SERVO_MAX_PWM / 2 + PW_OFFSET[4]},  // ID 15 (inverted)
    { gain, SERVO_MAX_PWM / 2 + PW_OFFSET[5]}}; // ID 16

/*
 * ======== GEOMETRY SETTINGS ========
 */

/*
 * Orientacje ramion serw względem osi X (w radianach).
 * Konfiguracja zależy od fizycznego rozmieszczenia serw na bazie.
 * THETA_S[i] = kąt osi symetrii i-tego ramienia serwa w płaszczyźnie XY bazy.
 */
const double THETA_S[NB_SERVOS] = {
    DEG2RAD(-60),
    DEG2RAD(120),
    DEG2RAD(180),
    DEG2RAD(0),
    DEG2RAD(60),
    DEG2RAD(-120)};

/*
 * MIN/MAX COORDINATES
 * Limity bezpiecznego ruchu platformy. Wartości zależne od geometrii
 * i fizycznych ograniczeń (kolizje, zakres serw).
 * Do aktualizacji po testach mechanicznych.
 */
const double HX_X_MIN = -20;
const double HX_X_MAX = 20;
const double HX_X_MID = (HX_X_MAX + HX_X_MIN) / 2;
const double HX_X_BAND = HX_X_MAX - HX_X_MIN;

const double HX_Y_MIN = -20;
const double HX_Y_MAX = 20;
const double HX_Y_MID = (HX_Y_MAX + HX_Y_MIN) / 2;
const double HX_Y_BAND = HX_Y_MAX - HX_Y_MIN;

const double HX_Z_MIN = -20.0;
const double HX_Z_MAX = 20.0;
const double HX_Z_MID = (HX_Z_MAX + HX_Z_MIN) / 2;
const double HX_Z_BAND = HX_Z_MAX - HX_Z_MIN;

const double HX_A_MIN = DEG2RAD(-8.0);
const double HX_A_MAX = DEG2RAD(8.0);
const double HX_A_MID = (HX_A_MAX + HX_A_MIN) / 2;
const double HX_A_BAND = HX_A_MAX - HX_A_MIN;

const double HX_B_MIN = DEG2RAD(-8.0);
const double HX_B_MAX = DEG2RAD(8.0);
const double HX_B_MID = (HX_B_MAX + HX_B_MIN) / 2;
const double HX_B_BAND = HX_B_MAX - HX_B_MIN;

const double HX_C_MIN = DEG2RAD(-8.0);
const double HX_C_MAX = DEG2RAD(8.0);
const double HX_C_MID = (HX_C_MAX + HX_C_MIN) / 2;
const double HX_C_BAND = HX_C_MAX - HX_C_MIN;

/*
 * Parametry geometrii platformy z docs/platform-parameters.md.
 * THETA_P i THETA_B w radianach — kąty offsetu przegubów względem osi symetrii.
 */
const double THETA_P = DEG2RAD(18.93);  // Platform joint angle offset (rad)
const double THETA_B = DEG2RAD(25.0);   // Base servo pinion angle offset (rad)
const double P_RAD = 71.225;            // Platform radius (mm)
const double B_RAD = 83.3;              // Base radius (mm)
const double ARM_LENGTH = 19.0;         // Servo arm length (mm) — horn
const double ROD_LENGTH = 65.0;         // Push rod length (mm) — cięgno
const double Z_HOME = 64;             // Default Z height (mm), z docs/platform-parameters.md

/*
 * ======== ALGORITHM FOR SERVO ANGLE CALCULATIONS ========
 */
#define ALGO 3