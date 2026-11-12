#include "Tank.h"

// Keep tracks raised during upload and battery startup; either can reset the Mega.
void setup() {
  Tank.begin();
  Tank.turnOffMotors();
  delay(5000);
  Tank.setLeftMotorPWM(180);
  Tank.setRightMotorPWM(180);
  delay(600);
  Tank.turnOffMotors();
}

void loop() {}  // Power cycle or reset to repeat.
