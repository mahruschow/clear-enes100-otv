// CLEAR practice tank navigation -- web simulator version.
// Paste this whole file into https://enes100.umd.edu/tools/simulator/
//
// Same logic as practice_tank/ (Navigation.h is inlined because the
// simulator only allows Arduino.h, Enes100.h, Tank.h and the C std lib).
// Simulator differences from the physical-tank sketch:
//   - no Serial; all output goes through Enes100.println
//   - readDistanceSensor returns -1 when nothing is within 1 m (treated as clear)
//   - no namespaces/structs/isfinite/NAN, which the simulator's interpreter lacks
#include <math.h>
#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

// Marker ID and room number are ignored by the simulator.
const int MARKER_ID = 3;
const int ROOM_NUMBER = 1116;

const bool ENABLE_MOTION = true;

// Arena is 4 m x 2 m. Drag the robot / change these to test other waypoints.
const float TARGET_X = 1.0;
const float TARGET_Y = 1.0;

#define MOVING 0
#define ARRIVED 1
#define BLOCKED 2
#define INVALID_POSE 3

// Output of navStep()
int cmdLeft = 0;
int cmdRight = 0;

bool halted = false;
int loopCount = 0;
bool havePose = false;
unsigned long lastPoseChangeMs = 0;
float lastX = 0;
float lastY = 0;
float lastTheta = 0;

float angleError(float desired, float current) {
  return atan2(sin(desired - current), cos(desired - current));
}

// Returns a status and sets cmdLeft / cmdRight.
int navStep(float x, float y, float theta, bool visible, float frontMeters, float targetX, float targetY) {
  cmdLeft = 0;
  cmdRight = 0;
  // Arena bounds in meters, with some margin for the marker position.
  if (!visible || x < 0.10 || x > 3.90 || y < 0.10 || y > 1.90) {
    return INVALID_POSE;
  }
  float dx = targetX - x;
  float dy = targetY - y;
  float distance = sqrt(dx * dx + dy * dy);
  if (distance < 0.12) return ARRIVED;
  // -1 means nothing detected within 1 m.
  if (frontMeters >= 0 && frontMeters < 0.20) return BLOCKED;

  float desired = atan2(dy, dx);
  float error = angleError(desired, theta);
  if (fabs(error) > 0.5) {
    // Far off heading: spin in place, slower as we get close to facing it.
    float mag = fabs(error);
    if (mag > 1.0) mag = 1.0;
    int turnPWM = (int)(50 + 30 * mag);
    if (error > 0) {
      cmdLeft = -turnPWM;
      cmdRight = turnPWM;
    } else {
      cmdLeft = turnPWM;
      cmdRight = -turnPWM;
    }
    return MOVING;
  }
  // Roughly facing target: drive forward and steer smoothly while moving.
  int drivePWM = 95;
  int steer = (int)(60 * error);
  cmdLeft = drivePWM - steer;
  cmdRight = drivePWM + steer;
  return MOVING;
}

void stopPermanently(const char *reason) {
  Tank.turnOffMotors();
  if (!halted) {
    Enes100.println(reason);
  }
  halted = true; // Press stop + run again for another attempt.
}

void setup() {
  Enes100.begin("CLEAR", WATER, MARKER_ID, ROOM_NUMBER, 52, 50);
  Tank.begin();
  Tank.turnOffMotors();
  lastPoseChangeMs = millis();
  Enes100.println("CLEAR tank navigation ready");
}

void loop() {
  if (halted) {
    Tank.turnOffMotors();
    delay(100);
    return;
  }

  float x = Enes100.getX();
  float y = Enes100.getY();
  float theta = Enes100.getTheta();
  bool visible = Enes100.isVisible();
  float frontMeters = Tank.readDistanceSensor(1);
  int status = navStep(x, y, theta, visible, frontMeters, TARGET_X, TARGET_Y);

  // Printing is slow in the simulator, so only print every 5th loop.
  loopCount++;
  if (loopCount % 5 == 0) {
  Enes100.print("x=");
  Enes100.print(x);
  Enes100.print(" y=");
  Enes100.print(y);
  Enes100.print(" theta=");
  Enes100.print(theta);
  Enes100.print(" front_m=");
  Enes100.print(frontMeters);
  Enes100.print(" status=");
  Enes100.println(status);
  }

  if (status == ARRIVED) {
    stopPermanently("Stopped: arrived at target");
    return;
  }
  if (status == BLOCKED) {
    stopPermanently("Stopped: obstacle within 0.20 m");
    return;
  }
  if (status != MOVING) {
    stopPermanently("Stopped: invalid pose");
    return;
  }

  // Stall / stale-pose guard: includes heading so a turn counts as progress.
  if (!havePose || fabs(x - lastX) > 0.02 || fabs(y - lastY) > 0.02 ||
      fabs(angleError(theta, lastTheta)) > 0.05) {
    havePose = true;
    lastX = x;
    lastY = y;
    lastTheta = theta;
    lastPoseChangeMs = millis();
  } else if (ENABLE_MOTION && millis() - lastPoseChangeMs > 2500) {
    stopPermanently("Stopped: no pose change for 2.5 seconds");
    return;
  }

  if (ENABLE_MOTION) {
    Tank.setLeftMotorPWM(cmdLeft);
    Tank.setRightMotorPWM(cmdRight);
  } else {
    Tank.turnOffMotors();
  }
  delay(20);
}
