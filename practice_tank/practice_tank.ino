#include "Enes100.h"
#include "Tank.h"
#include "Navigation.h"

// Set these from the marker on your assigned tank and the lab room.
const int MARKER_ID = -1;
const int ROOM_NUMBER = 1116; // Change if you are in 1120 or 1201.

// Start with motors disabled, then enable only in a clear test area.
const bool ENABLE_MOTION = false;

// EXAMPLE waypoint only; replace after checking the arena map and starting pose.
const float TARGET_X = 1.0f;
const float TARGET_Y = 1.0f;

bool halted = false;
unsigned long lastPoseChangeMs = 0;
float lastX = NAN, lastY = NAN, lastTheta = NAN;

void stopPermanently(const char *reason) {
  Tank.turnOffMotors();
  if (!halted) {
    Enes100.println(reason);
    Serial.println(reason);
  }
  halted = true; // Reset the board before another attempt.
}

void setup() {
  Serial.begin(115200);
  Tank.begin();
  Tank.turnOffMotors();
  if (MARKER_ID < 0) {
    Serial.println("Set MARKER_ID from the assigned tank before connecting.");
    halted = true;
    return;
  }
  // Physical course tank: Arduino Mega + installed Wi-Fi module (TX 52, RX 50).
  Enes100.begin("CLEAR", WATER, MARKER_ID, ROOM_NUMBER, 52, 50);
  lastPoseChangeMs = millis();
  Enes100.println("CLEAR tank navigation ready");
}

void loop() {
  if (halted) { Tank.turnOffMotors(); return; }

  Navigation::Pose pose = {
    Enes100.getX(), Enes100.getY(), Enes100.getTheta(), Enes100.isVisible()
  };
  const float frontMeters = Tank.readDistanceSensor(1);
  Navigation::Command cmd = Navigation::step(pose, frontMeters, TARGET_X, TARGET_Y);

  Serial.print("x="); Serial.print(pose.x, 2);
  Serial.print(" y="); Serial.print(pose.y, 2);
  Serial.print(" theta="); Serial.print(pose.theta, 2);
  Serial.print(" front_m="); Serial.print(frontMeters, 2);
  Serial.print(" status="); Serial.println(static_cast<int>(cmd.status));

  if (cmd.status != Navigation::Command::MOVING) {
    stopPermanently("Stopped: target, obstacle, or invalid sensor/pose");
    return;
  }

  // Stall / stale-pose guard: includes heading so a turn counts as progress.
  if (isnan(lastX) || fabsf(pose.x - lastX) > 0.02f ||
      fabsf(pose.y - lastY) > 0.02f ||
      fabsf(Navigation::angleError(pose.theta, lastTheta)) > 0.05f) {
    lastX = pose.x; lastY = pose.y; lastTheta = pose.theta;
    lastPoseChangeMs = millis();
  } else if (ENABLE_MOTION && millis() - lastPoseChangeMs > 2500UL) {
    stopPermanently("Stopped: no pose change for 2.5 seconds");
    return;
  }

  if (ENABLE_MOTION) {
    Tank.setLeftMotorPWM(cmd.left);
    Tank.setRightMotorPWM(cmd.right);
  } else {
    Tank.turnOffMotors();
  }
  delay(100);
}
