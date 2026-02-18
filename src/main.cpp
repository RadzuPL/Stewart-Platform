#include <Arduino.h>
#include <SCSCL.h>
#include <Wire.h>
#include "display.h"
#include "leds.h"

// === DEKLARACJE FORWARD ===
void processCommand(String cmd);
void scanServos();
void setTorque(bool enable);
void parsePositions(String cmd);
void printServos();

// === PINY ===
#define SERVO_RX_PIN 18
#define SERVO_TX_PIN 19
#define I2C_SDA     21
#define I2C_SCL     22
#define NEO_PIN     23
#define NUM_LEDS    2

// === OBIEKTY ===
SCSCL sc;

// === SERVO STATUS ===
int servoCount = 0;
int foundIDs[21];
bool torqueEnabled = false;
int globalSpeed = 2000;

// === PROTOKÓŁ ===
#define CMD_TORQUE  'T'
#define CMD_SPEED   'S'
#define CMD_POS     'P'
#define CMD_SCAN    'R'
#define CMD_HELP    'H'

void setup() {
  Serial.begin(115200);
  Serial2.begin(1000000, SERIAL_8N1, SERVO_RX_PIN, SERVO_TX_PIN);
  sc.pSerial = &Serial2;
  
  Wire.begin(I2C_SDA, I2C_SCL);
  displayInit();
  ledsInit();
  
  Serial.println("=== SERVO CONTROLLER v2.0 ===");
  Serial.println("USB: T0/1 S<speed> P<id:pos> R H");
  oledShowBoot();
  
  scanServos();
}

void loop() {
  if(Serial.available()) {
    processCommand(Serial.readStringUntil('\n'));
  }
  
  static unsigned long lastUpdate = 0;
  if(millis() - lastUpdate > 2000) {
    scanServos();
    updateOLED(servoCount, foundIDs, torqueEnabled, sc);
    updateLEDs(servoCount, torqueEnabled);
    printServos();
    lastUpdate = millis();
  }
  
  delay(50);
}

void processCommand(String cmd) {
  cmd.trim();
  if(cmd.length() == 0) return;
  
  Serial.println("CMD: " + cmd);
  char type = cmd.charAt(0);
  String params = cmd.substring(1);
  
  switch(type) {
    case CMD_TORQUE:
      torqueEnabled = (params == "1");
      setTorque(torqueEnabled);
      Serial.print("T"); Serial.println(torqueEnabled ? "1" : "0");
      break;
      
    case CMD_SPEED:
      globalSpeed = params.toInt();
      if(globalSpeed < 50) globalSpeed = 50;
      if(globalSpeed > 4000) globalSpeed = 4000;
      Serial.print("S"); Serial.println(globalSpeed);
      break;
      
    case CMD_POS:
      parsePositions(params);
      Serial.println("P OK");
      break;
      
    case CMD_SCAN:
      scanServos();
      Serial.print("R"); Serial.print(servoCount);
      for(int i = 0; i < servoCount; i++) {
        Serial.print(","); Serial.print(foundIDs[i]);
        Serial.print(":"); Serial.print(sc.ReadPos(foundIDs[i]));
      }
      Serial.println();
      break;
      
    case CMD_HELP:
      Serial.println("H v2.0 | T0/1 S1000 P11:512,12:100 R H");
      break;
      
    default:
      Serial.println("ERR");
  }
}

void scanServos() {
  servoCount = 0;
  for(int id = 0; id <= 20; id++) {
    if(sc.Ping(id) != -1) {
      foundIDs[servoCount] = id;
      servoCount++;
    }
  }
}

void setTorque(bool enable) {
  for(int i = 0; i < servoCount; i++) {
    int id = foundIDs[i];
    // Wyłącz torque + ustaw tryb pozycjonowania (0x24 = torque, 0x25 = mode)
    sc.writeByte(id, 0x24, enable ? 0x20 : 0x00);  // Torque ON/OFF
    sc.writeByte(id, 0x25, 0x00);                  // TRYBY POZYCJONOWANIA
  }
}

void parsePositions(String cmd) {
  // P11:512,12:100 → jedź absolutnie
  int posIndex = 0;
  for(int servoIdx = 0; servoIdx < servoCount && posIndex < cmd.length(); servoIdx++) {
    int id = foundIDs[servoIdx];
    
    // Szukaj P<id>:
    int colonPos = cmd.indexOf(':', posIndex);
    if(colonPos == -1) break;
    
    // Wyciągnij pozycję
    int commaPos = cmd.indexOf(',', colonPos);
    if(commaPos == -1) commaPos = cmd.length();
    
    String posStr = cmd.substring(colonPos + 1, commaPos);
    int targetPos = posStr.toInt();
    
    if(targetPos >= 0 && targetPos <= 1023) {
      sc.WritePos(id, targetPos, globalSpeed);
      Serial.print("  #"); Serial.print(id); Serial.print(":"); Serial.println(targetPos);
    }
    
    posIndex = commaPos;
  }
}

void printServos() {
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
