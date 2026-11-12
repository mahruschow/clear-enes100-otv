#include "Tank.h"

void setup() {
  Serial.begin(115200);
  Tank.begin();
  Tank.turnOffMotors();
  Serial.println("READY: lift tracks, then send t");
}
void loop() {
  if (Serial.available() && Serial.read() == 't') {
    Serial.println("TEST: both motors");
    Tank.setLeftMotorPWM(180);
    Tank.setRightMotorPWM(180);
    delay(600);
    Tank.turnOffMotors();
    Serial.println("DONE: motors off");
  }
}
