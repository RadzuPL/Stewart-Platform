#include <Arduino.h>
#include <SCSCL.h>
#include <Wire.h>
#include "display.h"
#include "leds.h"

// === DEKLARACJE FORWARD ===
void processCommand(String cmd);    // Interpretuje polecenia z portu Serial (T/S/P/R/H)
void scanServos();                  // Skanuje serwa o ID 11-16 (magistrala SCSCL)
void setTorque(bool enable);        // Włącza/wyłącza zasilanie serw
void parsePositions(String cmd);    // Parsuje parametry pozycji (format: id:pos,id:pos,...)
void printServos();                 // Drukuje status wszystkich znalezionych serw
void resetServo(int servoID);
void setServoID(String params);

// === PINY ===
#define SERVO_RX_PIN 18             // UART2 RX – odbiór odpowiedzi od serw
#define SERVO_TX_PIN 19             // UART2 TX – wysyłanie komend do serw
#define I2C_SDA     21              // I2C linia danych – OLED SSD1306
#define I2C_SCL     22              // I2C linia zegara – OLED SSD1306
#define NEO_PIN     23              // GPIO – seryjna linka danych WS2812 (diody RGB)
#define NUM_LEDS    2               // Liczba diód RGB

// === OBIEKTY ===
SCSCL sc;                           // Sterownik serw (biblioteka SCSCL)

// === SERVO STATUS ===
int servoCount = 0;                 // Liczba znalezionych serw
int foundIDs[21];                   // Tablica ID znalezionych serw (max 21)
bool torqueEnabled = false;         // Globalny stan zasilania serw (true=ON, false=OFF)
int globalSpeed = 2000;             // Globalna prędkość ruchu (jednostki SCSCL, domyślnie 2000)

// === PROTOKÓŁ ===
#define CMD_TORQUE  'T'             // Komenda: T0 (wyłączy) / T1 (włączy) zasilanie
#define CMD_SPEED   'S'             // Komenda: S<wartość> – ustaw prędkość ruchu
#define CMD_POS     'P'             // Komenda: P<id>:<pos>,<id>:<pos> – przesuń serwą do pozycji
#define CMD_SCAN    'R'             // Komenda: R – skanuj serwa i wypisz status każdego
#define CMD_HELP    'H'             // Komenda: H – wyświetl pomoc
#define CMD_RESET   'Z'             // Komenda: Z<id> – zresetuj serwo do ustawień fabrycznych (ID=1) - NIE DZIAŁA !
#define CMD_SETID   'I'             // Komenda: I<old:new> – zmień ID serwa

void setup() {
  // Inicjalizacja portów szeregowych
  Serial.begin(115200);             // Debug/komendy (UART0, GPIO1/3)
  Serial2.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);  // Serwa (UART2, 1 Mbps)
  sc.pSerial = &Serial2;            // Wskaźnik do UART serw dla biblioteki SCSCL
  
  // Inicjalizacja peryferiów
  Wire.begin(I2C_SDA, I2C_SCL);     // I2C dla OLED
  displayInit();                    // Inicjalizacja ekranu SSD1306
  ledsInit();                       // Inicjalizacja diod RGB WS2812
  
  // Boot messages
  Serial.println("=== SERVO CONTROLLER v2.0 ===");
  Serial.println("Commands: T0/1 S<speed> P<id:pos> R Z<id> I<old:new> H");
  oledShowBoot();                   // Wyświetl splash screen na OLED
   
  // Po skanowaniu — ustaw wszystkie serwa na Mode 0 (servo)
  scanServos();
  delay(100);
  
  // Rejestr 0x21 = Mode; 0 = servo mode (pozycjonowanie), 2 = silnik (obrót)
  Serial.println("[INIT] Setting all servos to Mode 0 (servo)...");
  for(int i = 0; i < servoCount; i++) {
    int id = foundIDs[i];
    int result = sc.writeByte(id, 0x21, 0);  // 0x21 = Mode register, 0 = Mode 0
    Serial.print("  ID "); Serial.print(id);
    Serial.print(" Mode result: "); Serial.println(result);
    delay(50);
  }
}

void loop() {
  // Odbieranie i przetwarzanie komend z Serial (115200bps)
  if(Serial.available()) {
    processCommand(Serial.readStringUntil('\n'));  // Czyta do znaku nowej linii
  }
  
  // Co 2 sekundy: skanuj, odśwież ekran, wypisz status
  static unsigned long lastUpdate = 0;
  if(millis() - lastUpdate > 2000) {
    scanServos();                   // Sprawdź czy serwa są dostępne
    updateOLED(servoCount, foundIDs, torqueEnabled, globalSpeed, sc);  // Odśwież wyświetlacz
    updateLEDs(servoCount, torqueEnabled);  // Aktualizuj diody (stan zasilania)
    printServos();                  // Wypisz status do Serial
    lastUpdate = millis();
    delay(100);                     // Czekaj po princie aby komendy były czytelne
  }
  
  delay(50);                        // Zapobiegaj zalaniu CPU
}

void processCommand(String cmd) {
  // Odbierz komendę z portu Serial, podziel na typ (pierwszy znak) i parametry (reszta)
  // Format: <TYP><PARAMETRY> — T1, S2000, P11:512, R, H, Z11, I1:11
  
  cmd.trim();                       // Usuń białe znaki
  if(cmd.length() == 0) return;     // Ignoruj puste linie
  
  Serial.println("CMD: " + cmd);   // Echo komendy do debugowania
  char type = cmd.charAt(0);       // Pierwszy znak = typ komendy
  String params = cmd.substring(1);  // Reszta = parametry
  
  switch(type) {
    case CMD_TORQUE:
      // T0 = wyłącz zasilanie, T1 = włącz zasilanie
      torqueEnabled = (params == "1");
      setTorque(torqueEnabled);
      Serial.print("T"); Serial.println(torqueEnabled ? "1" : "0");
      break;
      
    case CMD_SPEED:
      // Ustaw globalną prędkość ruchu (wartość min 50, max 4000)
      globalSpeed = params.toInt();
      if(globalSpeed < 50) globalSpeed = 50;
      if(globalSpeed > 4000) globalSpeed = 4000;
      Serial.print("S"); Serial.println(globalSpeed);
      break;
      
    case CMD_POS:
      // Przesuń serwą/serwy: P11:512,12:100 (id:pozycja, rozdzielone przecinkami)
      parsePositions(params);
      Serial.println("P OK");
      break;
      
    case CMD_SCAN:
      // Skanuj serwa i wypisz każdy ID ze swoją pozycją
      scanServos();
      Serial.print("R"); Serial.print(servoCount);
      for(int i = 0; i < servoCount; i++) {
        Serial.print(","); Serial.print(foundIDs[i]);
        Serial.print(":"); Serial.print(sc.ReadPos(foundIDs[i]));
      }
      Serial.println();
      break;

    case CMD_RESET:
      // Zresetuj serwę do ustawień fabrycznych (zmieni ID na 1)
      resetServo(params.toInt());
      break;

    case CMD_SETID:
      // Zmień ID serwa — Format: I1:11 (zmiana z ID 1 na ID 11)
      setServoID(params);
      break;
      
    case CMD_HELP:
      // Wyświetl pomoc z listą dostępnych poleceń i ich formatem
      Serial.println("=== SERVO CONTROLLER HELP ===");
      Serial.println("T0/1      - Torque OFF/ON");
      Serial.println("S<speed>  - Set speed (50-4000)");
      Serial.println("P<id:pos> - Move servo (P11:512,12:100)");
      Serial.println("R         - Scan servos");
      Serial.println("Z<id>     - Reset servo to ID=1");
      Serial.println("I<old:new>- Change servo ID (I1:11)");
      Serial.println("D<id>     - Diagnostic for servo (D11)");
      Serial.println("H         - This help");
      break;
      
    default:
      // Nieznana komenda
      Serial.println("ERR");
  }
}

void scanServos() {
  // === SKANOWANIE SERW ===
  // Ping każde ID 0..20 i zbiera ID dostępnych serw do tablicy foundIDs[]
  // TODO: zoptymalizować skanowanie (11-16 + 1 zamiast 0-20)
  
  servoCount = 0;
  for(int id = 0; id <= 20; id++) {
    if(sc.Ping(id) != -1) {         // -1 = timeout/brak odpowiedzi, inaczej = OK
      foundIDs[servoCount] = id;
      servoCount++;
    }
  }
}

void setTorque(bool enable) {
  // === WŁĄCZ/WYŁĄCZ ZASILANIE SERW ===
  // Rejestr SCSCL_TORQUE_ENABLE (0x28/40): 1 = ON, 0 = OFF
  
  Serial.println("\n[TORQUE] Setting all servos to: " + String(enable ? "ON" : "OFF"));
  
  for(int i = 0; i < servoCount; i++) {
    int id = foundIDs[i];
    // EnableTorque() jest wrapperem dla sc.writeByte(id, 0x28, enable)
    int result = sc.EnableTorque(id, enable ? 1 : 0);
    
    Serial.print("  ID "); Serial.print(id);
    Serial.print(" → "); Serial.print(enable ? "ON" : "OFF");
    Serial.print(" [result: "); Serial.print(result); Serial.println("]");
    
    delay(50);  // Delay między wysłaniem do każdego serwa
  }
}

void parsePositions(String cmd) {
  // === PARSER POZYCJI ===
  // Wejście: "11:512" lub "11:512,12:100,13:512"
  // Mapowanie: id:pozycja dla KAŻDEGO serwa NIEZALEŻNIE
  // 
  // Stary parser były błędny — iterował po foundIDs zamiast po parach!
  
  Serial.println("[PARSE] Input: '" + cmd + "'");
  
  int commaPos = 0;
  int pairCount = 0;
  
  while(commaPos < cmd.length()) {
    // Szukaj następnego separatora (przecinek lub koniec)
    int nextComma = cmd.indexOf(',', commaPos);
    if(nextComma == -1) nextComma = cmd.length();
    
    // Wyciągnij parę "id:pos"
    String pair = cmd.substring(commaPos, nextComma);
    pair.trim();
    
    if(pair.length() == 0) {
      commaPos = nextComma + 1;
      continue;  // Pomiń puste pary
    }
    
    // Szukaj ':' w parze
    int colonPos = pair.indexOf(':');
    if(colonPos == -1) {
      Serial.println("  [ERR] Pair '" + pair + "' has no ':'");
      commaPos = nextComma + 1;
      continue;
    }
    
    // Parsuj ID i pozycję
    int servoID = pair.substring(0, colonPos).toInt();
    int targetPos = pair.substring(colonPos + 1).toInt();
    
    Serial.print("  Pair#"); Serial.print(pairCount);
    Serial.print(" → ID:"); Serial.print(servoID);
    Serial.print(" Pos:"); Serial.print(targetPos);
    
    // Sprawdzenie zakresu
    if(servoID >= 0 && servoID <= 253 && targetPos >= 0 && targetPos <= 1023) {
      int timeMs = 1000;  // Czas ruchu: 1s
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

void printServos() {
  // === WYPISZ STATUS WSZYSTKICH SERW ===
  // Pokazuje: liczba serw, stan zasilania, prędkość
  // Dla każdego serwa: ID, pozycja bieżąca, napięcie zasilania
  
  Serial.println();
  for(int i = 0; i < 40; i++) Serial.print("=");
  Serial.println();
  
  Serial.print("Servo: "); Serial.print(servoCount);
  Serial.print(" | Trq: "); Serial.print(torqueEnabled ? "ON" : "OFF");
  Serial.print(" | Speed: "); Serial.println(globalSpeed);
  
  for(int i = 0; i < servoCount; i++) {
    int id = foundIDs[i];
    Serial.print("  #"); Serial.print(id);
    Serial.print(" Pos:"); Serial.print(sc.ReadPos(id));
    Serial.print(" V:"); Serial.print(sc.ReadVoltage(id)/10.0); Serial.println("V");
  }
  
  for(int i = 0; i < 40; i++) Serial.print("=");
  Serial.println();
}

void resetServo(int servoID) {
  // Zresetuj serwę do ustawień fabrycznych — wymaża całą konfigurację
  // Po resecie serwo będzie mieć ID=1 i wymaga ponownej konfiguracji
  
  Serial.println("\n[RESET] WARNING: Restoring servo " + String(servoID) + " to factory defaults...");
  Serial.println("  After reset, servo will have ID=1");
  
  // Funkcja Reset() wysyła instrukcję 0x06 (RESET) do serwa i czeka na potwierdzenie
  int result = sc.Reset(servoID);
  
  Serial.print("  Reset result: ");
  if(result == 0) {
    Serial.println("OK - Servo now has ID=1");
  } else {
    Serial.println("FAILED (result: " + String(result) + ")");
    Serial.println("  Servo may not exist or communication failed");
  }
  
  delay(500);
  Serial.println("[RESET] Re-scanning servos...");
  scanServos();
}

void setServoID(String params) {
  // Zmień ID serwa na nowe — zmiana jest zapisywana w EEPROM
  // Format: "1:11" oznacza zmianę z ID=1 na ID=11
  // UWAGA: zmiany wejdą w życie dopiero po wyłączeniu i włączeniu zasilania
  
  int colonPos = params.indexOf(':');
  if(colonPos == -1) {
    Serial.println("[SETID] ERROR: Format should be I<current_id>:<new_id>");
    return;
  }
  
  int currentID = params.substring(0, colonPos).toInt();
  int newID = params.substring(colonPos + 1).toInt();
  
  Serial.println("\n[SETID] Changing servo ID...");
  Serial.print("  Current ID: "); Serial.println(currentID);
  Serial.print("  New ID: "); Serial.println(newID);
  
  // Sprawdzenie czy nowe ID jest w dozwolonym zakresie (0-253 dla protokołu Feetech)
  if(newID < 0 || newID > 253) {
    Serial.println("  ERROR: New ID must be 0-253");
    return;
  }
  
  // Sprawdzenie czy current i new ID nie są identyczne
  if(currentID == newID) {
    Serial.println("  ERROR: Current and new ID are the same");
    return;
  }
  
  // Rejestr 0x05 przechowuje ID serwa w EEPROM (standard protokołu Feetech)
  int result = sc.writeByte(currentID, 0x05, newID);
  
  if(result == 0) {
    Serial.println("  SUCCESS - ID changed (written to EEPROM)");
    Serial.println("  Servo must be power-cycled for change to take effect");
  } else {
    Serial.println("  FAILED (result: " + String(result) + ")");
    Serial.println("  Check if servo with this ID exists or communication error");
  }
  
  delay(500);
  Serial.println("[SETID] Re-scanning servos...");
  scanServos();
}
