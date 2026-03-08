#include <Arduino.h>
#include <SCSCL.h>
#include <Wire.h>
#include "display.h"
#include "leds.h"
#include "motion.h"

// === DEKLARACJE FORWARD ===
void processCommand(String cmd);
void scanServos();
void setTorque(bool enable);
void parsePositions(String cmd);
void printServos();
void resetServo(int servoID);
void setServoID(String params);
void printServoDiagnostic(int servoID);
void printAllDiagnostics();
bool isValidProtocolID(int id);
bool isManagedServoID(int id);
int readRegByteSafe(int servoID, uint8_t addr);
void printServoRegisterBlock(int servoID, uint8_t startAddr, uint8_t endAddr);

// === PINY ===
// UART2 do komunikacji z serwami magistralowymi.
#define SERVO_RX_PIN 18
#define SERVO_TX_PIN 19

// Magistrala I2C dla wyświetlacza OLED.
#define I2C_SDA     21
#define I2C_SCL     22

// === ZAKRESY ID ===
// ID fabryczne obserwowane diagnostycznie, przydatne np. po zmianie ID
// lub przy próbie resetu do ustawień fabrycznych.
#define SERVO_FACTORY_ID 1

// Zakres roboczych ID używanych przez platformę.
#define SERVO_ID_MIN    11
#define SERVO_ID_MAX    16

// Maksymalna liczba serw wykrywanych w projekcie:
// 6 serw roboczych + opcjonalnie 1 serwo na ID fabrycznym.
#define MAX_SCANNED_SERVOS 7

// === OBIEKTY ===
// Główny obiekt biblioteki do komunikacji z serwami SCS.
SCSCL sc;

// === SERVO STATUS ===
// Liczba aktualnie wykrytych serw.
int servoCount = 0;

// Tablica przechowująca ID serw znalezionych podczas ostatniego skanowania.
int foundIDs[MAX_SCANNED_SERVOS];

// Zapamiętany stan globalnego torque ustawianego z poziomu komend.
bool torqueEnabled = false;

// Globalna prędkość używana przez komendy ruchu.
int globalSpeed = 2000;

// === PROTOKÓŁ ===
// Jednoznakowe kody komend przyjmowanych z terminala przez UART0.
#define CMD_TORQUE  'T'   // Włączenie lub wyłączenie momentu serw.
#define CMD_SPEED   'S'   // Ustawienie globalnej prędkości ruchu.
#define CMD_POS     'P'   // Ruch jednego lub wielu serw do zadanych pozycji.
#define CMD_SCAN    'R'   // Skanowanie serw i szybki raport pozycji.
#define CMD_HELP    'H'   // Wyświetlenie pomocy.
#define CMD_RESET   'Z'   // Próba resetu fabrycznego serwa.
#define CMD_SETID   'I'   // Zmiana ID serwa przez zapis do EEPROM.
#define CMD_DIAG    'D'   // Diagnostyka jednego lub wszystkich serw.
#define CMD_MOVE6   'M'   // Ruch platformy: x,y,z,roll,pitch,yaw.

/**
 * Inicjalizacja mikrokontrolera, UART, I2C, OLED, LED i serw.
 * Wejście: brak.
 * Wyjście: brak (stan globalny i peryferia gotowe do pracy).
 */
void setup() {
  // Uruchom port debug/komendy na UART0.
  Serial.begin(115200);

  // Uruchom port serw na UART2 z prędkością 1 Mbps
  // i przypisz go do biblioteki obsługującej protokół SCS.
  Serial2.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  sc.pSerial = &Serial2;

  // Uruchom magistralę I2C oraz moduły interfejsu użytkownika.
  Wire.begin(I2C_SDA, I2C_SCL);
  displayInit();
  ledsInit();

  // Inicjalizuj moduł ruchu (arm/dry-run).
  motionInit();

  // Wyświetl komunikat startowy na porcie debug i na OLED.
  Serial.println("=== SERVO CONTROLLER v2.2 ===");
  Serial.println("Commands: T0/1 S<speed> P<id:pos> M(motion) R D[<id>] Z<id> I<old:new> H");
  oledShowBoot();

  // Wykryj serwa obecne na magistrali.
  scanServos();
  delay(100);

  // Ustaw wszystkie znalezione serwa w tryb pracy serwomechanizmu.
  // Rejestr 0x21 jest ustawiany na 0 zgodnie z używanym modelem i biblioteką.
  Serial.println("[INIT] Setting found servos to Mode 0 (servo mode)");
  for(int i = 0; i < servoCount; i++) {
    int id = foundIDs[i];
    int result = sc.writeByte(id, 0x21, 0);

    Serial.print("  ID "); Serial.print(id);
    Serial.print(" Mode write result: "); Serial.println(result);
    delay(50);
  }
}

/**
 * Główna pętla programu.
 * Wejście: brak.
 * Wyjście: brak (obsługa komend i okresowy monitoring).
 */
void loop() {
  // Odczytaj jedną linię komendy z portu debug.
  if(Serial.available()) {
    processCommand(Serial.readStringUntil('\n'));
  }

  // Okresowo odśwież listę serw, OLED, LED i raport tekstowy.
  static unsigned long lastUpdate = 0;
  if(millis() - lastUpdate > 2000) {
    scanServos();
    updateOLED(servoCount, foundIDs, torqueEnabled, globalSpeed, sc);
    updateLEDs(servoCount, torqueEnabled);
    printServos();
    lastUpdate = millis();
    delay(100);
  }

  delay(50);
}

/**
 * Parsuje i wykonuje komendę tekstową.
 * Wejście: cmd - pełna komenda, np. "T1", "P11:512", "D", "D11", "M0,0,5,0,0,0".
 * Wyjście: brak (odpowiedź przez Serial i akcja sprzętowa).
 */
void processCommand(String cmd) {
  // Usuń białe znaki i odrzuć puste wejście.
  cmd.trim();
  if(cmd.length() == 0) {
    return;
  }

  // Pierwszy znak określa typ komendy, reszta to parametry.
  char type = cmd.charAt(0);
  if(type >= 'a' && type <= 'z') {
    type = type - ('a' - 'A');
  }
  String params = cmd.substring(1);
  params.trim();

  // Zaloguj surowo odebraną komendę po normalizacji.
  Serial.println("CMD: " + String(type) + params);

  switch(type) {
    case CMD_TORQUE:
      // T0 / T1: ustaw globalny stan momentu i wyślij go do wszystkich
      // aktualnie znalezionych serw.
      if(params == "0" || params == "1") {
        torqueEnabled = (params == "1");
        setTorque(torqueEnabled);
        Serial.print("T"); Serial.println(torqueEnabled ? "1" : "0");
      } else {
        Serial.println("ERR T format: T0 or T1");
      }
      break;

    case CMD_SPEED:
      // S<speed>: ustaw globalną prędkość ruchu z ograniczeniem zakresu.
      globalSpeed = params.toInt();
      if(globalSpeed < 50) globalSpeed = 50;
      if(globalSpeed > 4000) globalSpeed = 4000;
      Serial.print("S"); Serial.println(globalSpeed);
      break;

    case CMD_POS:
      // P<id:pos[,id:pos...]>: wykonaj ruch jednego lub wielu serw.
      parsePositions(params);
      Serial.println("P OK");
      break;

    case CMD_SCAN:
      // R: przeskanuj magistralę w zakresie projektowym i wypisz skrócony raport.
      scanServos();
      Serial.print("R"); Serial.print(servoCount);
      for(int i = 0; i < servoCount; i++) {
        Serial.print(","); Serial.print(foundIDs[i]);
        Serial.print(":"); Serial.print(sc.ReadPos(foundIDs[i]));
      }
      Serial.println();
      break;

    case CMD_MOVE6:
      // M<subcommand>: obsługa komend ruchu platformy.
      // Delegowane do modułu motion.
      handleMotionCommand(params, sc, globalSpeed, torqueEnabled);
      break;

    case CMD_RESET:
      // Z<id>: wykonaj diagnostyczną próbę resetu fabrycznego serwa.
      // Zachowanie zależy od firmware modelu SCS225 i nie jest pewne.
      if(params.length() == 0) {
        Serial.println("ERR Z format: Z<id>");
      } else {
        resetServo(params.toInt());
      }
      break;

    case CMD_SETID:
      // I<old:new>: zmień ID serwa przez zapis do rejestru EEPROM.
      setServoID(params);
      break;

    case CMD_DIAG:
      // D: diagnostyka wszystkich znalezionych serw.
      // D<id>: diagnostyka pojedynczego wskazanego ID.
      if(params.length() == 0) {
        printAllDiagnostics();
      } else {
        printServoDiagnostic(params.toInt());
      }
      break;

    case CMD_HELP:
      // H: wypisz listę obsługiwanych komend.
      Serial.println("=== SERVO CONTROLLER HELP ===");
      Serial.println("T0/1       - Torque OFF/ON");
      Serial.println("S<speed>   - Set speed (50-4000)");
      Serial.println("P<id:pos>  - Move servo (P11:512,12:100)");
      Serial.println("M          - Motion commands (M? for help)");
      Serial.println("R          - Scan servos (1,11..16)");
      Serial.println("D          - Diagnostic all found servos");
      Serial.println("D<id>      - Diagnostic one servo (D11)");
      Serial.println("Z<id>      - Factory reset servo (target ID)");
      Serial.println("I<old:new> - Change servo ID (I1:11)");
      Serial.println("H          - This help");
      break;

    default:
      // Odrzuć nieobsługiwany typ komendy.
      Serial.println("ERR");
      break;
  }
}

/**
 * Skanuje serwa tylko w zakresie projektowym: ID 1 oraz ID 11..16.
 * Wejście: brak.
 * Wyjście: aktualizuje foundIDs[] i servoCount.
 */
void scanServos() {
  // Wyzeruj licznik wykrytych urządzeń przed nowym skanowaniem.
  servoCount = 0;

  // Najpierw sprawdź ID fabryczne, aby łatwo wykryć serwo po zmianie ID
  // albo po potencjalnym resecie.
  if(sc.Ping(SERVO_FACTORY_ID) != -1) {
    foundIDs[servoCount] = SERVO_FACTORY_ID;
    servoCount++;
  }

  // Następnie skanuj standardowe ID robocze używane przez platformę.
  for(int id = SERVO_ID_MIN; id <= SERVO_ID_MAX; id++) {
    if(sc.Ping(id) != -1) {
      if(servoCount < MAX_SCANNED_SERVOS) {
        foundIDs[servoCount] = id;
        servoCount++;
      }
    }
  }
}

/**
 * Ustawia moment (Torque Enable) na wszystkich znalezionych serwach.
 * Wejście: enable - true włącza moment, false wyłącza.
 * Wyjście: brak (raport przez Serial).
 */
void setTorque(bool enable) {
  Serial.println("\n[TORQUE] Setting all found servos to: " + String(enable ? "ON" : "OFF"));

  // Wyślij komendę momentu do każdego serwa z ostatniego skanowania.
  for(int i = 0; i < servoCount; i++) {
    int id = foundIDs[i];
    int result = sc.EnableTorque(id, enable ? 1 : 0);

    Serial.print("  ID "); Serial.print(id);
    Serial.print(" -> "); Serial.print(enable ? "ON" : "OFF");
    Serial.print(" [result: "); Serial.print(result); Serial.println("]");
    delay(50);
  }
}

/**
 * Parsuje parametry pozycji i wysyła komendy ruchu.
 * Wejście: cmd - lista par "id:pozycja" rozdzielonych przecinkami.
 * Wyjście: brak (wykonuje WritePos dla każdej poprawnej pary).
 */
void parsePositions(String cmd) {
  Serial.println("[PARSE] Input: '" + cmd + "'");

  int commaPos = 0;
  int pairCount = 0;

  // Przetwarzaj kolejne segmenty aż do końca tekstu.
  while(commaPos < (int)cmd.length()) {
    // Znajdź koniec bieżącej pary oddzielonej przecinkiem.
    int nextComma = cmd.indexOf(',', commaPos);
    if(nextComma == -1) {
      nextComma = cmd.length();
    }

    // Pobierz bieżący segment i usuń z niego białe znaki.
    String pair = cmd.substring(commaPos, nextComma);
    pair.trim();

    // Pomiń pusty segment, jeśli wystąpił np. po podwójnym przecinku.
    if(pair.length() == 0) {
      commaPos = nextComma + 1;
      continue;
    }

    // Każda para musi zawierać separator między ID a pozycją.
    int colonPos = pair.indexOf(':');
    if(colonPos == -1) {
      Serial.println("  [ERR] Pair '" + pair + "' has no ':'");
      commaPos = nextComma + 1;
      continue;
    }

    // Odczytaj ID serwa i pozycję docelową z bieżącej pary.
    int servoID = pair.substring(0, colonPos).toInt();
    int targetPos = pair.substring(colonPos + 1).toInt();

    Serial.print("  Pair#"); Serial.print(pairCount);
    Serial.print(" -> ID:"); Serial.print(servoID);
    Serial.print(" Pos:"); Serial.print(targetPos);

    // Zweryfikuj zakres ID i pozycji przed wysłaniem komendy ruchu.
    if(isValidProtocolID(servoID) && targetPos >= 0 && targetPos <= 1023) {
      int timeMs = 1000;
      int result = sc.WritePos(servoID, targetPos, timeMs, globalSpeed);
      Serial.print(" [OK, result:"); Serial.print(result); Serial.println("]");
    } else {
      Serial.print(" [ERR: invalid range (ID:0-253, Pos:0-1023)]");
      Serial.println();
    }

    pairCount++;
    commaPos = nextComma + 1;
  }

  Serial.print("[PARSE] Done: "); Serial.print(pairCount); Serial.println(" pairs");
}

/**
 * Drukuje skrócony status wszystkich znalezionych serw.
 * Wejście: brak.
 * Wyjście: brak (raport przez Serial).
 */
void printServos() {
  Serial.println();
  for(int i = 0; i < 40; i++) Serial.print("=");
  Serial.println();

  Serial.print("Servo: "); Serial.print(servoCount);
  Serial.print(" | Trq: "); Serial.print(torqueEnabled ? "ON" : "OFF");
  Serial.print(" | Speed: "); Serial.println(globalSpeed);

  // Wypisz podstawowe dane bieżące dla każdego wykrytego serwa.
  for(int i = 0; i < servoCount; i++) {
    int id = foundIDs[i];
    int pos = sc.ReadPos(id);
    int voltageRaw = sc.ReadVoltage(id);

    Serial.print("  #"); Serial.print(id);
    Serial.print(" Pos:"); Serial.print(pos);
    Serial.print(" V:"); Serial.print(voltageRaw / 10.0f);
    Serial.println("V");
  }

  for(int i = 0; i < 40; i++) Serial.print("=");
  Serial.println();
}

/**
 * Wykonuje próbę resetu fabrycznego pojedynczego serwa i raportuje wynik.
 * Wejście: servoID - ID serwa docelowego.
 * Wyjście: brak (raport przez Serial, aktualizacja listy serw).
 */
void resetServo(int servoID) {
  // Sprawdź poprawność ID w zakresie protokołu.
  if(!isValidProtocolID(servoID)) {
    Serial.println("[RESET] ERROR: ID must be in 0..253");
    return;
  }

  // Zweryfikuj, czy serwo odpowiada przed wysłaniem komendy resetu.
  if(sc.Ping(servoID) == -1) {
    Serial.println("[RESET] ERROR: Servo not responding before reset");
    return;
  }

  Serial.println("\n[RESET] Factory reset request");
  Serial.print("  Target ID: "); Serial.println(servoID);

  // Wyłącz moment przed operacją, aby ograniczyć ryzyko ruchu w trakcie.
  int tqResult = sc.EnableTorque(servoID, 0);
  Serial.print("  Torque OFF result: "); Serial.println(tqResult);
  delay(100);

  // Wyślij komendę resetu. Obsługa tej funkcji zależy od firmware serwa.
  int resetResult = sc.Reset(servoID);
  Serial.print("  Reset result: "); Serial.println(resetResult);

  // Daj serwu czas na restart i sprawdź odpowiedź na ID bieżącym oraz fabrycznym.
  delay(800);
  int pingCurrent = sc.Ping(servoID);
  int pingFactory = sc.Ping(SERVO_FACTORY_ID);

  // Zinterpretuj rezultat bez zakładania pełnej zgodności modelu SCS225.
  if(resetResult == 0) {
    if(pingFactory != -1) {
      Serial.println("  RESET OK: servo responds on ID=1");
    } else if(pingCurrent != -1) {
      Serial.println("  RESET SENT: servo still responds on previous ID");
      Serial.println("  NOTE: this model may ignore factory reset command");
    } else {
      Serial.println("  RESET SENT: no response yet (check power cycle)");
    }
  } else {
    Serial.println("  RESET NOT EXECUTED by servo");
    Serial.println("  NOTE: SCS225 may not support this command in firmware");
  }

  // Odśwież listę serw po operacji diagnostycznej.
  scanServos();
}

/**
 * Zmienia ID serwa przez zapis rejestru EEPROM ID (0x05).
 * Wejście: params - tekst w formacie "<current_id>:<new_id>".
 * Wyjście: brak (raport przez Serial i ponowny skan).
 */
void setServoID(String params) {
  // Znajdź separator pomiędzy aktualnym i nowym ID.
  int colonPos = params.indexOf(':');
  if(colonPos == -1) {
    Serial.println("[SETID] ERROR: Format should be I<current_id>:<new_id>");
    return;
  }

  // Odczytaj oba ID z parametru tekstowego.
  int currentID = params.substring(0, colonPos).toInt();
  int newID = params.substring(colonPos + 1).toInt();

  // Zweryfikuj zakresy dopuszczalne przez protokół.
  if(!isValidProtocolID(currentID) || !isValidProtocolID(newID)) {
    Serial.println("[SETID] ERROR: IDs must be in 0..253");
    return;
  }

  // Odrzuć zmianę na identyczną wartość.
  if(currentID == newID) {
    Serial.println("[SETID] ERROR: Current and new ID are the same");
    return;
  }

  // Sprawdź, czy serwo o aktualnym ID odpowiada.
  if(sc.Ping(currentID) == -1) {
    Serial.println("[SETID] ERROR: Current ID is not responding");
    return;
  }

  // Nowe ID nie może być już zajęte przez inne urządzenie.
  if(sc.Ping(newID) != -1) {
    Serial.println("[SETID] ERROR: New ID already in use");
    return;
  }

  Serial.println("\n[SETID] Changing servo ID...");
  Serial.print("  Current ID: "); Serial.println(currentID);
  Serial.print("  New ID: "); Serial.println(newID);

  // Dla bezpieczeństwa wyłącz moment przed zapisem do EEPROM.
  int tqResult = sc.EnableTorque(currentID, 0);
  Serial.print("  Torque OFF result: "); Serial.println(tqResult);
  delay(100);

  // Zapisz nowy ID do rejestru EEPROM odpowiedzialnego za adres urządzenia.
  int result = sc.writeByte(currentID, 0x05, newID);

  if(result == 0) {
    Serial.println("  SUCCESS - ID written to EEPROM");
    Serial.println("  Power cycle may be required by the servo model");
  } else {
    Serial.println("  FAILED (result: " + String(result) + ")");
  }

  // Odśwież listę serw po zmianie.
  delay(500);
  scanServos();
}

/**
 * Drukuje diagnostykę pojedynczego serwa.
 * Wejście: servoID - ID docelowe.
 * Wyjście: brak (raport przez Serial).
 */
void printServoDiagnostic(int servoID) {
  // Sprawdź poprawność ID przed próbą komunikacji.
  if(!isValidProtocolID(servoID)) {
    Serial.println("[DIAG] ERROR: ID must be in 0..253");
    return;
  }

  Serial.println();
  Serial.println("=========== DIAG ===========");
  Serial.print("ID: "); Serial.println(servoID);

  // Ping potwierdza obecność urządzenia na magistrali.
  int ping = sc.Ping(servoID);
  Serial.print("Ping: "); Serial.println(ping == -1 ? "NO RESPONSE" : "OK");
  Serial.print("Managed ID group: "); Serial.println(isManagedServoID(servoID) ? "YES" : "NO");

  // Jeżeli brak odpowiedzi, zakończ diagnostykę tego ID.
  if(ping == -1) {
    Serial.println("============================");
    return;
  }

  // Odczytaj podstawowe parametry runtime dostępne przez API biblioteki.
  int pos = sc.ReadPos(servoID);
  int voltageRaw = sc.ReadVoltage(servoID);

  Serial.print("Position: "); Serial.println(pos);
  Serial.print("Voltage: "); Serial.print(voltageRaw / 10.0f); Serial.println("V");

  // Wydrukuj pełny blok EEPROM używany przez serwa SCS.
  Serial.println("EEPROM [0x00..0x23]");
  printServoRegisterBlock(servoID, 0x00, 0x23);

  // Wydrukuj blok SRAM z parametrami bieżącymi i statusowymi.
  Serial.println("SRAM   [0x28..0x45]");
  printServoRegisterBlock(servoID, 0x28, 0x45);

  Serial.println("============================");
}

/**
 * Drukuje diagnostykę wszystkich aktualnie znalezionych serw.
 * Wejście: brak.
 * Wyjście: brak (raport przez Serial).
 */
void printAllDiagnostics() {
  // Odśwież listę urządzeń przed diagnostyką zbiorczą.
  scanServos();

  Serial.println("\n=== DIAGNOSTIC ALL START ===");
  Serial.print("Found servos: "); Serial.println(servoCount);

  // Wykonaj diagnostykę każdej znalezionej jednostki osobno.
  for(int i = 0; i < servoCount; i++) {
    printServoDiagnostic(foundIDs[i]);
    delay(20);
  }

  Serial.println("=== DIAGNOSTIC ALL END ===");
}

/**
 * Waliduje ID względem zakresu protokołu SCS/Feetech.
 * Wejście: id - badane ID.
 * Wyjście: true gdy 0..253, false poza zakresem.
 */
bool isValidProtocolID(int id) {
  return id >= 0 && id <= 253;
}

/**
 * Sprawdza, czy ID należy do grupy zarządzanej w tym projekcie.
 * Wejście: id - badane ID.
 * Wyjście: true dla ID 1 lub 11..16.
 */
bool isManagedServoID(int id) {
  return (id == SERVO_FACTORY_ID) || (id >= SERVO_ID_MIN && id <= SERVO_ID_MAX);
}

/**
 * Odczytuje pojedynczy rejestr 8-bit i waliduje zakres wartości.
 * Wejście: servoID - ID serwa, addr - adres rejestru.
 * Wyjście: 0..255 dla poprawnego odczytu, -1 gdy brak poprawnej odpowiedzi.
 */
int readRegByteSafe(int servoID, uint8_t addr) {
  // Odczytaj bajt z rejestru wskazanego adresem.
  int value = sc.readByte(servoID, addr);

  // Zwróć -1, jeśli biblioteka nie zwróciła poprawnej wartości bajtowej.
  if(value < 0 || value > 255) {
    return -1;
  }

  return value;
}

/**
 * Drukuje blok rejestrów w formie tabelarycznej po 8 adresów na linię.
 * Wejście: servoID - ID serwa, startAddr - początek zakresu, endAddr - koniec zakresu.
 * Wyjście: brak (raport przez Serial).
 */
void printServoRegisterBlock(int servoID, uint8_t startAddr, uint8_t endAddr) {
  // Iteruj po kolejnych liniach tabeli, po 8 adresów na wiersz.
  for(uint16_t base = startAddr; base <= endAddr; base += 8) {
    // Wydrukuj adres bazowy aktualnej linii.
    Serial.print("0x");
    if(base < 0x10) Serial.print("0");
    Serial.print(base, HEX);
    Serial.print(": ");

    // Wydrukuj kolejne wartości rejestrów w ramach bieżącej linii.
    for(uint16_t addr = base; addr < (base + 8) && addr <= endAddr; addr++) {
      int value = readRegByteSafe(servoID, (uint8_t)addr);

      // Wypisz "--", jeśli odczyt rejestru nie był poprawny.
      if(value < 0) {
        Serial.print("--");
      } else {
        if(value < 0x10) Serial.print("0");
        Serial.print(value, HEX);
      }

      // Dodaj separator między kolejnymi kolumnami tabeli.
      if(addr < (base + 7) && addr < endAddr) {
        Serial.print(" ");
      }
    }

    Serial.println();
  }
}
