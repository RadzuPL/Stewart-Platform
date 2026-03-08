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

/**
 * Oblicza kąty serw (radiany, stopnie, pozycja SCS) dla zadanej pozycji platformy.
 *
 * Wejście: coord - zadana pozycja platformy (translacje mm, rotacje rad).
 *          servo_angles - tablica wynikowa kątów serw.
 * Wyjście: 0 = OK, <0 = błąd (patrz kody poniżej).
 *
 * Kody błędów:
 *  -1: dystans BP przekracza fizyczne maksimum (ARM+ROD).
 *  -3: pozycja SCS przekracza górny limit serwa.
 *  -4: pozycja SCS poniżej dolnego limitu serwa.
 *  -5: wyrażenie pod pierwiastkiem ujemne (geometria nieosiągalna).
 */
int8_t Hexapod_Kinematics::calcServoAngles(
    platform_t coord,
    angle_t servo_angles[])
{
    static uint64_t nb_call = 0;
    ++nb_call;

    angle_t new_servo_angles[NB_SERVOS];

    double
        cosA = cos(coord.hx_a),
        cosB = cos(coord.hx_b),
        cosC = cos(coord.hx_c),
        sinA = sin(coord.hx_a),
        sinB = sin(coord.hx_b),
        sinC = sin(coord.hx_c);

    int8_t movOK = 0;

    for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
    {
        // Pozycja przegubu platformy względem pivota serwa.
        double BP_x = P_COORDS[sid][0] * cosB * cosC +
                      P_COORDS[sid][1] * (sinA * sinB * cosC - cosA * sinC) +
                      coord.hx_x -
                      B_COORDS[sid][0];
        double BP_y = P_COORDS[sid][0] * cosB * sinC +
                      P_COORDS[sid][1] * (sinA * sinB * sinC + cosA * cosC) +
                      coord.hx_y -
                      B_COORDS[sid][1];
        double BP_z = -P_COORDS[sid][0] * sinB +
                      P_COORDS[sid][1] * sinA * cosB +
                      coord.hx_z -
                      Z_HOME;

        double
            a = COS_THETA_S[sid] * BP_x + SIN_THETA_S[sid] * BP_y,
            b = -SIN_THETA_S[sid] * BP_x + COS_THETA_S[sid] * BP_y,
            c = BP_z,
            a2 = POW(a, 2),
            a4 = POW(a, 4),
            b2 = POW(b, 2),
            b4 = POW(b, 4),
            c2 = POW(c, 2),
            c4 = POW(c, 4);

        // Dystans^2 między pivotem serwa a przegubem platformy.
        double BP2 = a2 + b2 + c2;

        // Sprawdź, czy dystans nie przekracza fizycznego maksimum.
        if (BP2 > BP2_MAX)
        {
            movOK = -1;
            break;
        }

        double i1 = -ARM_LENGTH4 - ROD_LENGTH4 - a4 - b4 - c4 +
                    2 * (ARM_LENGTH2 * (ROD_LENGTH2 + a2 - b2 + c2) +
                         ROD_LENGTH2 * a2 + ROD_LENGTH2 * (b2 + c2) -
                         a2 * (b2 + c2) - b2 * c2);
        if (i1 < 0)
        {
            movOK = -5;
            break;
        }
        i1 = sqrt(i1);
        i1 = (2 * ARM_LENGTH * c - i1) /
             (ARM_LENGTH2 + 2 * ARM_LENGTH * a -
              ROD_LENGTH2 + BP2);
        i1 = 2 * atan(i1);
        new_servo_angles[sid].rad = i1;

        // Obrót kąta o połowę zakresu.
        new_servo_angles[sid].rad += SERVO_HALF_ANGULAR_RANGE;

        // Konwersja radiany → stopnie.
        new_servo_angles[sid].deg = degrees(new_servo_angles[sid].rad);

        // Konwersja radiany → pozycja SCS z uwzględnieniem kalibracji i inwersji.
        new_servo_angles[sid].pwm_us =
            round(SERVO_IK_CALIBRATION[sid].gain * new_servo_angles[sid].rad) +
            SERVO_IK_CALIBRATION[sid].offset;

        // Walidacja pozycji SCS względem limitów per serwo.
        if (new_servo_angles[sid].pwm_us > SERVO_PWM_UPPER_LIMIT[sid])
        {
            movOK = -3;
            break;
        }
        else if (new_servo_angles[sid].pwm_us < SERVO_PWM_LOWER_LIMIT[sid])
        {
            movOK = -4;
            break;
        }
    }

    // Zapisz wynikowe kąty tylko gdy wszystkie obliczenia poprawne.
    if (movOK == 0)
    {
        for (uint8_t sid = 0; sid < NB_SERVOS; sid++)
        {
            servo_angles[sid] = new_servo_angles[sid];
        }
        _coord.hx_x = coord.hx_x;
        _coord.hx_y = coord.hx_y;
        _coord.hx_z = coord.hx_z;
        _coord.hx_a = coord.hx_a;
        _coord.hx_b = coord.hx_b;
        _coord.hx_c = coord.hx_c;
    }

    return movOK;
}
