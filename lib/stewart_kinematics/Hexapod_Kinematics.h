/**
 * S T E W A R T    P L A T F O R M    O N    E S P 3 2
 *
 * Copyright (C) 2019  Nicolas Jeanmonod, ouilogique.com
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

#pragma once

#ifdef PLATFORMIO
#include <Arduino.h>
#else
#include "../hexapod_desktop_app_cpp/hexapod_desktop_app.h"
#endif

#include "../../src/platform_config.h"

// angle_t
typedef struct
{
    double rad;      // Servo angle in radian.
    double deg;      // Servo angle in degrees.
    int us;          // Servo angle in µs (PWM).
    uint16_t pwm_us; // Servo angle in range 0 to 4096 (PWM).
    double debug;    // Used for debug.
} angle_t;

// Platform coordinates.
typedef struct
{
    double hx_x; // Surge, translation along X axis (mm)
    double hx_y; // Sway, translation along Y axis (mm)
    double hx_z; // Heave, translation along Z axis (mm)
    double hx_a; // Roll, rotation around X axis (rad)
    double hx_b; // Pitch, rotation around Y axis (rad)
    double hx_c; // Yaw, rotation around Z axis (rad)
} platform_t;

/**
 * Klasa kinematyki odwrotnej platformy Stewarta.
 * Wszystkie parametry geometrii i kalibracji pobierane z platform_config.
 */
class Hexapod_Kinematics
{
private:
    platform_t _coord;

public:
    Hexapod_Kinematics(){};
    int8_t home(angle_t *servo_angles);
    int8_t calcServoAngles(platform_t coord, angle_t *servo_angles);
    int8_t calcServoAnglesAlgo1(platform_t coord, angle_t *servo_angles);
    int8_t calcServoAnglesAlgo2(platform_t coord, angle_t *servo_angles);
    int8_t calcServoAnglesAlgo3(platform_t coord, angle_t *servo_angles);
    double getHX_X();
    double getHX_Y();
    double getHX_Z();
    double getHX_A();
    double getHX_B();
    double getHX_C();

    double mapDouble(double x, double in_min, double in_max, double out_min, double out_max);

    // === PREKALKOWANE WARTOŚCI GEOMETRII ===

    const double AXIS1 = PI / 6;
    const double AXIS2 = -PI / 2;
    const double AXIS3 = AXIS1;

    const double COS_THETA_S[NB_SERVOS] = {
        cos(THETA_S[0]), cos(THETA_S[1]), cos(THETA_S[2]),
        cos(THETA_S[3]), cos(THETA_S[4]), cos(THETA_S[5])
    };

    const double SIN_THETA_S[NB_SERVOS] = {
        sin(THETA_S[0]), sin(THETA_S[1]), sin(THETA_S[2]),
        sin(THETA_S[3]), sin(THETA_S[4]), sin(THETA_S[5])
    };

    const double M_THETA_S[NB_SERVOS] = {
        -THETA_S[0], -THETA_S[1], -THETA_S[2],
        -THETA_S[3], -THETA_S[4], -THETA_S[5]
    };

    const double sinD[NB_SERVOS] = {
        sin(M_THETA_S[0]), sin(M_THETA_S[1]), sin(M_THETA_S[2]),
        sin(M_THETA_S[3]), sin(M_THETA_S[4]), sin(M_THETA_S[5])
    };

    const double cosD[NB_SERVOS] = {
        cos(M_THETA_S[0]), cos(M_THETA_S[1]), cos(M_THETA_S[2]),
        cos(M_THETA_S[3]), cos(M_THETA_S[4]), cos(M_THETA_S[5])
    };

    const double P_COORDS[NB_SERVOS][2] = {
        { P_RAD * cos(AXIS1 + THETA_P),  P_RAD * sin(AXIS1 + THETA_P)},
        { P_RAD * cos(AXIS1 - THETA_P),  P_RAD * sin(AXIS1 - THETA_P)},
        { P_RAD * cos(AXIS2 + THETA_P),  P_RAD * sin(AXIS2 + THETA_P)},
        {-P_RAD * cos(AXIS2 + THETA_P),  P_RAD * sin(AXIS2 + THETA_P)},
        {-P_RAD * cos(AXIS3 - THETA_P),  P_RAD * sin(AXIS3 - THETA_P)},
        {-P_RAD * cos(AXIS3 + THETA_P),  P_RAD * sin(AXIS3 + THETA_P)}
    };

    const double B_COORDS[NB_SERVOS][2] = {
        { B_RAD * cos(AXIS1 + THETA_B),  B_RAD * sin(AXIS1 + THETA_B)},
        { B_RAD * cos(AXIS1 - THETA_B),  B_RAD * sin(AXIS1 - THETA_B)},
        { B_RAD * cos(AXIS2 + THETA_B),  B_RAD * sin(AXIS2 + THETA_B)},
        {-B_RAD * cos(AXIS2 + THETA_B),  B_RAD * sin(AXIS2 + THETA_B)},
        {-B_RAD * cos(AXIS3 - THETA_B),  B_RAD * sin(AXIS3 - THETA_B)},
        {-B_RAD * cos(AXIS3 + THETA_B),  B_RAD * sin(AXIS3 + THETA_B)}
    };

    const double BP2_MAX = POW((ARM_LENGTH + ROD_LENGTH), 2);

    const double ARM_LENGTH2 = POW(ARM_LENGTH, 2);
    const double ARM_LENGTH4 = POW(ARM_LENGTH, 4);
    const double ROD_LENGTH2 = POW(ROD_LENGTH, 2);
    const double ROD_LENGTH4 = POW(ROD_LENGTH, 4);

    const double BP2_PERP = ROD_LENGTH2 - ARM_LENGTH2;
};