#pragma once

#include <math.h>

namespace Navigation {

struct Pose {
  float x;
  float y;
  float theta;
  bool visible;
};

struct Command {
  int left;
  int right;
  enum Status { MOVING, ARRIVED, BLOCKED, INVALID_POSE, INVALID_RANGE } status;
};

inline float angleError(float desired, float current) {
  return atan2f(sinf(desired - current), cosf(desired - current));
}

inline Command step(Pose pose, float frontMeters, float targetX, float targetY) {
  // Arena bounds in meters, with some margin for the marker position.
  if (!pose.visible || !isfinite(pose.x) || !isfinite(pose.y) ||
      !isfinite(pose.theta) || pose.x < 0.10f || pose.x > 3.90f ||
      pose.y < 0.10f || pose.y > 1.90f) {
    return {0, 0, Command::INVALID_POSE};
  }
  if (!isfinite(frontMeters) || frontMeters <= 0.0f || frontMeters > 1.0f) {
    return {0, 0, Command::INVALID_RANGE};
  }
  const float dx = targetX - pose.x;
  const float dy = targetY - pose.y;
  const float distance = sqrtf(dx * dx + dy * dy);
  if (distance < 0.12f) return {0, 0, Command::ARRIVED};
  if (frontMeters < 0.20f) return {0, 0, Command::BLOCKED};

  const float desired = atan2f(dy, dx);
  const float error = angleError(desired, pose.theta);
  if (fabsf(error) > 0.18f) {
    const int turnPWM = 85; // Tune on the physical tank.
    return error > 0.0f ? Command{-turnPWM, turnPWM, Command::MOVING}
                        : Command{turnPWM, -turnPWM, Command::MOVING};
  }
  const int drivePWM = 95; // Tune on the physical tank.
  return {drivePWM, drivePWM, Command::MOVING};
}

} // namespace Navigation
