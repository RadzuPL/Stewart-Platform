#include "motion.h"
#include "platform_config.h"
#include "Hexapod_Kinematics.h"
#include <math.h>

// === STAN TESTÓW RUCHU ===
static bool gMotionArmed = false;
static bool gMotionDryRun = true;
static Hexapod_Kinematics gIK;

/**
 * Inicjalizuje stan modułu ruchu.
 * Wejście: brak.
 * Wyjście: brak.
 */
void motionInit() {
  gMotionArmed = false;
  gMotionDryRun = true;
}

/**
 * Parsuje 6 parametrów w formacie: x,y,z,roll,pitch,yaw.
 * Wejście: text - parametry tekstowe.
 * Wyjście: true gdy parsowanie poprawne, outPose wypełniony.
 */
static bool parsePose6D(const String &text, Pose6D &outPose) {
  float values[6];
  int count = 0;
  int start = 0;
  String s = text;
  s.trim();

  while (start <= (int)s.length() && count < 6) {
    int comma = s.indexOf(',', start);
    if (comma == -1) comma = s.length();

    String token = s.substring(start, comma);
    token.trim();
    if (token.length() == 0) return false;

    values[count++] = token.toFloat();
    start = comma + 1;
  }

  if (count != 6) return false;

  outPose.xMm = values[0];
  outPose.yMm = values[1];
  outPose.zMm = values[2];
  outPose.rollDeg = values[3];
  outPose.pitchDeg = values[4];
  outPose.yawDeg = values[5];
  return true;
}

/**
 * Sprawdza limity ruchu platformy.
 * Wejście: pose - zadana pozycja i orientacja.
 * Wyjście: true gdy wartości mieszczą się w limitach.
 */
static bool isPoseInsideLimits(const Pose6D &pose) {
  if (fabsf(pose.xMm) > gMotionLimits.xMaxMm) return false;
  if (fabsf(pose.yMm) > gMotionLimits.yMaxMm) return false;
  if (fabsf(pose.zMm) > gMotionLimits.zMaxMm) return false;
  if (fabsf(pose.rollDeg) > gMotionLimits.rollMaxDeg) return false;
  if (fabsf(pose.pitchDeg) > gMotionLimits.pitchMaxDeg) return false;
  if (fabsf(pose.yawDeg) > gMotionLimits.yawMaxDeg) return false;
  return true;
}

/**
 * Konwertuje wynik IK na bezpieczną pozycję SCS z clampowaniem.
 * Wejście: index - indeks serwa 0..5, ikPosition - pozycja z IK.
 * Wyjście: pozycja SCS przycięta do zakresu kalibracyjnego.
 */
static int ikPositionToSCS(int index, int ikPosition) {
  return clampServoPositionByIndex(index, ikPosition);
}

/**
 * Oblicza cele serw dla pozycji platformy z użyciem IK.
 * Wejście: pose - zadanie 6DoF (mm i stopnie).
 * Wyjście: true gdy outPos[6] wypełnione poprawnymi pozycjami SCS.
 */
static bool computeServoTargetsFromPose(const Pose6D &pose, int outPos[6]) {
  platform_t coord;
  coord.hx_x = pose.xMm;
  coord.hx_y = pose.yMm;
  coord.hx_z = pose.zMm;
  coord.hx_a = radians(pose.rollDeg);
  coord.hx_b = radians(pose.pitchDeg);
  coord.hx_c = radians(pose.yawDeg);

  angle_t servo_angles[NB_SERVOS];
  int8_t result = gIK.calcServoAngles(coord, servo_angles);

  if (result != 0) {
    Serial.print("[MOTION] IK error code: ");
    Serial.println(result);
    return false;
  }

  for (int i = 0; i < NB_SERVOS; i++) {
    int rawPos = (int)servo_angles[i].pwm_us;

    Serial.print("[MOTION] IK servo[");
    Serial.print(i);
    Serial.print("] rad=");
    Serial.print(servo_angles[i].rad, 4);
    Serial.print(" deg=");
    Serial.print(servo_angles[i].deg, 2);
    Serial.print(" pos_raw=");
    Serial.print(rawPos);

    outPos[i] = ikPositionToSCS(i, rawPos);

    Serial.print(" pos_clamped=");
    Serial.println(outPos[i]);
  }

  return true;
}

/**
 * Wysyła cele do serw z uwzględnieniem dry-run i clampowania.
 * Wejście: targets[6] - cele pozycji, sc - interfejs serw, globalSpeed - prędkość.
 * Wyjście: true gdy operacja zakończona.
 */
static bool sendTargetsToServos(const int targets[6], SCSCL &sc, int globalSpeed) {
  for (int i = 0; i < NB_SERVOS; i++) {
    int servoID = gServoCalibration[i].id;
    int safePos = clampServoPositionByIndex(i, targets[i]);

    Serial.print("[MOTION] ID ");
    Serial.print(servoID);
    Serial.print(" -> ");
    Serial.print(safePos);

    if (gMotionDryRun) {
      Serial.println(" [DRY]");
      continue;
    }

    int result = sc.WritePos(servoID, safePos, 1000, globalSpeed);
    Serial.print(" [result:");
    Serial.print(result);
    Serial.println("]");
    delay(20);
  }
  return true;
}

/**
 * Wykonuje ruch do pozycji HOME z kalibracji.
 * Wejście: sc - interfejs serw, globalSpeed - prędkość.
 * Wyjście: true gdy komenda wykonana.
 */
static bool moveHome(SCSCL &sc, int globalSpeed) {
  int targets[NB_SERVOS];
  for (int i = 0; i < NB_SERVOS; i++) {
    targets[i] = gServoCalibration[i].homePos;
  }
  return sendTargetsToServos(targets, sc, globalSpeed);
}

/**
 * Parsuje i wykonuje bezpośredni test pozycji serw: MT11:512,12:512,...
 * Wejście: params - tekst po "MT", sc - interfejs serw, globalSpeed - prędkość.
 * Wyjście: true gdy komenda wykonana.
 */
static bool handleDirectServoTest(const String &params, SCSCL &sc, int globalSpeed) {
  int targets[NB_SERVOS];
  for (int i = 0; i < NB_SERVOS; i++) {
    targets[i] = gServoCalibration[i].homePos;
  }

  int cursor = 0;
  String s = params;
  s.trim();

  while (cursor < (int)s.length()) {
    int comma = s.indexOf(',', cursor);
    if (comma == -1) comma = s.length();

    String pair = s.substring(cursor, comma);
    pair.trim();
    if (pair.length() == 0) {
      cursor = comma + 1;
      continue;
    }

    int colon = pair.indexOf(':');
    if (colon == -1) {
      Serial.println("[MOTION] ERR MT format: MT11:512,12:512,...");
      return false;
    }

    int id = pair.substring(0, colon).toInt();
    int pos = pair.substring(colon + 1).toInt();

    int idx = -1;
    if (!getServoIndexByID(id, idx)) {
      Serial.print("[MOTION] ERR unknown servo ID: ");
      Serial.println(id);
      return false;
    }

    targets[idx] = pos;
    cursor = comma + 1;
  }

  return sendTargetsToServos(targets, sc, globalSpeed);
}

/**
 * Drukuje status modułu ruchu i konfiguracji.
 * Wejście: torqueEnabled - aktualny stan torque.
 * Wyjście: brak.
 */
static void printMotionStatus(bool torqueEnabled) {
  Serial.println("=== MOTION STATUS ===");
  Serial.print("Armed: "); Serial.println(gMotionArmed ? "YES" : "NO");
  Serial.print("DryRun: "); Serial.println(gMotionDryRun ? "YES" : "NO");
  Serial.print("Torque: "); Serial.println(torqueEnabled ? "ON" : "OFF");
  Serial.print("Geometry: "); Serial.println(isPlatformGeometryReady() ? "READY" : "NOT READY");
  Serial.print("Calibration: "); Serial.println(areServoCalibrationsReady() ? "READY" : "NOT READY");

  Serial.println("--- Geometry ---");
  Serial.print("ARM="); Serial.print(ARM_LENGTH);
  Serial.print(" ROD="); Serial.print(ROD_LENGTH);
  Serial.print(" Z_HOME="); Serial.println(Z_HOME);
  Serial.print("B_RAD="); Serial.print(B_RAD);
  Serial.print(" P_RAD="); Serial.println(P_RAD);

  Serial.println("--- Motion Limits ---");
  Serial.print("X/Y/Z [mm]: ");
  Serial.print(gMotionLimits.xMaxMm); Serial.print(" / ");
  Serial.print(gMotionLimits.yMaxMm); Serial.print(" / ");
  Serial.println(gMotionLimits.zMaxMm);

  Serial.print("R/P/Y [deg]: ");
  Serial.print(gMotionLimits.rollMaxDeg); Serial.print(" / ");
  Serial.print(gMotionLimits.pitchMaxDeg); Serial.print(" / ");
  Serial.println(gMotionLimits.yawMaxDeg);

  Serial.println("--- Servo Calibration ---");
  for (int i = 0; i < NB_SERVOS; i++) {
    Serial.print("  ["); Serial.print(i); Serial.print("] ID=");
    Serial.print(gServoCalibration[i].id);
    Serial.print(" inv="); Serial.print(gServoCalibration[i].inverted ? "Y" : "N");
    Serial.print(" off="); Serial.print(gServoCalibration[i].offset);
    Serial.print(" home="); Serial.print(gServoCalibration[i].homePos);
    Serial.print(" min="); Serial.print(gServoCalibration[i].minPos);
    Serial.print(" max="); Serial.println(gServoCalibration[i].maxPos);
  }

  Serial.println("=====================");
}

/**
 * Drukuje pomoc dla komend M.
 * Wejście: brak.
 * Wyjście: brak.
 */
static void printMotionHelp() {
  Serial.println("=== MOTION HELP (M) ===");
  Serial.println("MS               - Motion status");
  Serial.println("MA1 / MA0        - Arm/Disarm motion");
  Serial.println("MD1 / MD0        - Dry-run ON/OFF");
  Serial.println("MH               - Move to HOME (safe)");
  Serial.println("MT<id:pos,...>    - Safe direct servo test with clamp");
  Serial.println("M<x,y,z,r,p,y>  - 6DoF command (IK)");
  Serial.println("M?               - This help");
  Serial.println("Example: M0,0,5,0,0,0  (heave +5mm)");
  Serial.println("Example: M0,0,0,3,0,0  (roll +3 deg)");
}

/**
 * Obsługuje komendy ruchu z prefiksem M.
 * Wejście: params - tekst po literze M, sc - interfejs serw,
 *          globalSpeed - prędkość ruchu, torqueEnabled - stan torque.
 * Wyjście: true gdy komenda obsłużona.
 */
bool handleMotionCommand(const String &params, SCSCL &sc, int globalSpeed, bool torqueEnabled) {
  String p = params;
  p.trim();

  if (p.length() == 0 || p == "?") {
    printMotionHelp();
    return true;
  }

  String up = p;
  up.toUpperCase();

  if (up == "S") {
    printMotionStatus(torqueEnabled);
    return true;
  }

  if (up == "A1") {
    gMotionArmed = true;
    Serial.println("[MOTION] ARM ON");
    return true;
  }

  if (up == "A0") {
    gMotionArmed = false;
    Serial.println("[MOTION] ARM OFF");
    return true;
  }

  if (up == "D1") {
    gMotionDryRun = true;
    Serial.println("[MOTION] DRY-RUN ON");
    return true;
  }

  if (up == "D0") {
    gMotionDryRun = false;
    Serial.println("[MOTION] DRY-RUN OFF");
    return true;
  }

  if (!gMotionArmed) {
    Serial.println("[MOTION] ERR not armed (use MA1)");
    return false;
  }

  if (!torqueEnabled) {
    Serial.println("[MOTION] ERR torque OFF (use T1)");
    return false;
  }

  if (!isPlatformGeometryReady()) {
    Serial.println("[MOTION] ERR geometry not configured");
    return false;
  }

  if (!areServoCalibrationsReady()) {
    Serial.println("[MOTION] ERR servo calibration not completed");
    return false;
  }

  if (up == "H") {
    bool ok = moveHome(sc, globalSpeed);
    Serial.println(ok ? "[MOTION] HOME OK" : "[MOTION] HOME ERR");
    return ok;
  }

  if (up.startsWith("T")) {
    bool ok = handleDirectServoTest(p.substring(1), sc, globalSpeed);
    Serial.println(ok ? "[MOTION] TEST OK" : "[MOTION] TEST ERR");
    return ok;
  }

  Pose6D pose;
  if (!parsePose6D(p, pose)) {
    Serial.println("[MOTION] ERR format: Mx,y,z,roll,pitch,yaw");
    return false;
  }

  Serial.print("[MOTION] Pose: x="); Serial.print(pose.xMm);
  Serial.print(" y="); Serial.print(pose.yMm);
  Serial.print(" z="); Serial.print(pose.zMm);
  Serial.print(" R="); Serial.print(pose.rollDeg);
  Serial.print(" P="); Serial.print(pose.pitchDeg);
  Serial.print(" Y="); Serial.println(pose.yawDeg);

  if (!isPoseInsideLimits(pose)) {
    Serial.println("[MOTION] ERR pose outside configured limits");
    return false;
  }

  int targets[NB_SERVOS];
  if (!computeServoTargetsFromPose(pose, targets)) {
    Serial.print("[IK_DBG] BP2_MAX=");
    Serial.println((ARM_LENGTH + ROD_LENGTH) * (ARM_LENGTH + ROD_LENGTH));
    Serial.println("[MOTION] ERR IK failed (pose unreachable or geometry error)");
    return false;
  }

  bool ok = sendTargetsToServos(targets, sc, globalSpeed);
  Serial.println(ok ? "[MOTION] POSE OK" : "[MOTION] POSE ERR");
  return ok;
}