#include <math.h>  // Simulator requires math before the other headers.
#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

// Route for the fixed obstacle layout shown in the team's screenshot.
// Start at X=0.55 m, Y=0.55 m, theta=0 (facing +X).
// Go above the first block, between the two blocks, then through the
// upper (limbo) side of the final barrier. The vehicle must physically
// fit under the limbo; adjust these points for other layouts.
const float targetX[] = {0.90, 1.90, 1.90, 2.85, 3.02, 3.75};
const float targetY[] = {1.55, 1.55, 0.85, 0.85, 1.40, 1.40};
const int waypointCount = sizeof(targetX) / sizeof(targetX[0]);
int waypoint = 0;

void stopDrive() {
  Tank.turnOffMotors();
}

float angleError(float target, float current) {
  return atan2(sin(target - current), cos(target - current));
}

void setup() {
  // Simulator Wi-Fi pins; replace marker ID, room, and pins for the actual OTV.
  Enes100.begin("CLEAR", WATER, 3, 1116, 8, 9);
  Tank.begin();
  stopDrive();
  Enes100.println("Starting waypoint route");
}

void loop() {
  if (waypoint >= waypointCount) {
    stopDrive();
    delay(100);
    return;
  }

  if (!Enes100.isVisible()) {
    stopDrive();
    Enes100.println("Pose lost; waiting");
    delay(200);
    return;
  }

  float x = Enes100.getX();
  float y = Enes100.getY();
  float heading = Enes100.getTheta();
  float dx = targetX[waypoint] - x;
  float dy = targetY[waypoint] - y;
  float distance = sqrt(dx * dx + dy * dy);

  if (distance < 0.12) {
    stopDrive();
    waypoint++;
    Enes100.print("Reached waypoint ");
    Enes100.println(waypoint);
    delay(100);
    return;
  }

  float error = angleError(atan2(dy, dx), heading);
  if (fabs(error) > 0.15) {
    const int turnPwm = 140;
    if (error > 0) {
      Tank.setLeftMotorPWM(-turnPwm);
      Tank.setRightMotorPWM(turnPwm);
    } else {
      Tank.setLeftMotorPWM(turnPwm);
      Tank.setRightMotorPWM(-turnPwm);
    }
  } else {
    // Ultrasonic 1 is enabled by default in the simulator. It senses the
    // white blocks, not walls or necessarily the log/limbo barrier.
    float front = Tank.readDistanceSensor(1);
    if (front >= 0 && front < 0.20) {
      stopDrive();
      Enes100.println("Unexpected obstacle ahead: stopped");
      delay(200);
      return;
    }

    int drivePwm = distance < 0.30 ? 95 : 140;
    Tank.setLeftMotorPWM(drivePwm);
    Tank.setRightMotorPWM(drivePwm);
  }

  delay(50);
}
