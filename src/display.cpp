#include "display.h"

Adafruit_SSD1306 display(128, 32, &Wire, -1);

void displayInit() {
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
}

void oledShowBoot() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Servo Ctrl");
  display.println("v2.0 Ready");
  display.display();
}

void updateOLED(int servoCount, int foundIDs[], bool torqueEnabled, SCSCL &sc) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  display.setCursor(0, 0);
  display.print("S:"); display.print(servoCount); 
  display.print(" T:"); display.print(torqueEnabled ? "1" : "0");
  
  display.setCursor(0, 10);
  if(servoCount >= 1) {
    display.print("P1:"); display.print(sc.ReadPos(foundIDs[0]));
  }
  if(servoCount >= 2) {
    display.print(" P2:"); display.print(sc.ReadPos(foundIDs[1]));
  }
  
  display.setCursor(0, 20);
  if(servoCount >= 1) {
    display.print("V:"); display.print(sc.ReadVoltage(foundIDs[0])/10.0);
  }
  
  display.display();
}