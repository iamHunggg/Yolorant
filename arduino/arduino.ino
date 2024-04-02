#include <Mouse.h>
#include <hiduniversal.h>
#include "HIDMouseRptParser.h"


USB Usb;
HIDUniversal Hid(&Usb);
HIDMouseEvents MouEvents;
HIDMouseReportParser Mou(&MouEvents);

void setup() {
  Mouse.begin();
  Serial.begin(1000000);
  Serial.println("Start");

  if (Usb.Init() == -1) {
    Serial.println("OSC did not start.");
    delay(200);
  }

  if (!Hid.SetReportParser(0, &Mou)) {
    ErrorMessage<uint8_t>(PSTR("SetReportParser"), 1);
  }
}


void moveMouseInLerpPixels(float xEnd, float yEnd, int pixelsPerMove = 2) {
  float xStart = 0, yStart = 0;
  
  float deltaX = xEnd - xStart;
  float deltaY = yEnd - yStart;
  
  float distance = sqrt(deltaX * deltaX + deltaY * deltaY);
  int steps = distance / pixelsPerMove;

  for (int i = 0; i <= steps; i++) {
    float t = (float)i / (float)steps;
    float easeT = t < 0.5 ? 2 * t * t : -1 + (4 - 2 * t) * t;
    float xLerp = lerp(xStart, xEnd, easeT);
    float yLerp = lerp(yStart, yEnd, easeT);
    Mouse.move(xLerp - xStart, yLerp - yStart);
    xStart = xLerp;
    yStart = yLerp;
    delay(0.05);
  }
}


float lerp(float start, float end, float t) {
  return start + t * (end - start);
}

void loop() {

  Usb.Task();
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    String command = data.substring(0, 1);
    int commaIndex = data.indexOf(',');
    int xValue = data.substring(1, commaIndex).toInt();
    int yValue = data.substring(commaIndex + 1).toInt();

    if (command == "a") {
      moveMouseInLerpPixels(xValue, yValue); // You can adjust the number of steps as needed
      }else if (command == "f") {
      moveMouseInSteps(xValue, yValue);
      Mouse.click();
      moveMouseInSteps(-xValue, -yValue);
}
}
}


void moveMouseInSteps(int x, int y) {
  while (x != 0 || y != 0) {
    // Choose the largest step size that stays within the -127 to 127 range
    char xStep = x > 127 ? 127 : (x < -127 ? -127 : x);
    char yStep = y > 127 ? 127 : (y < -127 ? -127 : y);
    Mouse.move(xStep, yStep);
    x -= xStep;
    y -= yStep;
  }
}

