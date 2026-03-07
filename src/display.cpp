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

void updateOLED(int servoCount, int foundIDs[], bool torqueEnabled, int globalSpeed, SCSCL &sc) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Linia 0: S:6 T:ON Sp:2000
  display.setCursor(0, 0);
  display.print("S:"); display.print(servoCount);
  display.print(" T:"); display.print(torqueEnabled ? "ON" : "OFF");
  display.print(" Sp:"); display.print(globalSpeed);
  
  // Linia 1: #11:277 #12:791 #13:5
  display.setCursor(0, 10);
  for(int i = 0; i < 3 && i < servoCount; i++) {
    int id = foundIDs[i];
    int pos = sc.ReadPos(id);
    if(i > 0) display.print(" ");
    //display.print("#"); 
    display.print(id); display.print(":"); display.print(pos);
  }
  
  // Linia 2: #14:786 #15:446 #16:797
  display.setCursor(0, 20);
  for(int i = 3; i < 6 && i < servoCount; i++) {
    int id = foundIDs[i];
    int pos = sc.ReadPos(id);
    if(i > 3) display.print(" ");
    //display.print("#"); 
    display.print(id); display.print(":"); display.print(pos);
  }
  
  display.display();
}