// CLEAR obstacle navigation -- web simulator version.
// Paste this whole file into https://enes100.umd.edu/tools/simulator/
//
// Course layout (simulator and arena): 4 m x 2 m. Obstacles sit in two
// columns (x = 1.4-1.6 and x = 2.2-2.4), each split into three lanes
// centered at y = 0.5, 1.0, 1.5. Each column has at least one open lane.
//
// Strategy: pick a lane, face +x, look ahead with the front ultrasonic.
// If the lane is blocked, mark it, side-step to the nearest open lane and
// look again. Once past both columns, drive to the goal point.
#include <math.h>
#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

// Marker ID and room number are ignored by the simulator.
const int MARKER_ID = 3;
const int ROOM_NUMBER = 1116;

const bool ENABLE_MOTION = true;

// Lanes through the obstacle columns.
const float LANE_Y0 = 0.5;
const float LANE_SPACING = 0.5;
const int NUM_LANES = 3;

// Once x passes this, all obstacles are behind us.
const float CLEAR_X = 2.7;

// Goal point. y = 1.5 goes under the limbo, y = 0.5 goes over the log.
const float GOAL_X = 3.7;
const float GOAL_Y = 1.5;

// Ultrasonic thresholds (meters, measured from the front of the robot).
const float LOOK_DIST = 0.45;  // lane counts as blocked if something is this close
const float STOP_DIST = 0.25;  // stop driving if something gets this close

// Motor tuning (simulator values; the real tank needs more turn power).
const int DRIVE_PWM = 110;
const int TURN_MIN_PWM = 45;
const int TURN_MAX_PWM = 85;

#define ST_TO_LANE 0
#define ST_FACE_EAST 1
#define ST_CHECK 2
#define ST_DRIVE_EAST 3
#define ST_TO_GOAL 4
#define ST_DONE 5

int state = ST_TO_LANE;
int lane = 0;
int blockedMask = 0;      // bit i set = lane i blocked in the current column
float blockedAtX = 0;     // x where the current column's blocks were seen
float laneChangeX = 0;    // x to hold while side-stepping between lanes

// Motor command for this loop.
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

float laneY(int i) {
  return LANE_Y0 + LANE_SPACING * i;
}

int nearestLane(float y) {
  int best = 0;
  for (int i = 1; i < NUM_LANES; i++) {
    if (fabs(laneY(i) - y) < fabs(laneY(best) - y)) best = i;
  }
  return best;
}

// Distance to whatever is ahead, or 99 if nothing is in range.
// Simulator returns -1 for "nothing within 1 m"; real tank maxes out at 1.
float frontDistance() {
  float d = Tank.readDistanceSensor(1);
  if (d <= 0 || d >= 1.0) return 99;
  return d;
}

// Spin in place toward a heading. Returns true once facing it.
bool turnToward(float desired, float theta) {
  float error = angleError(desired, theta);
  if (fabs(error) < 0.08) {
    cmdLeft = 0;
    cmdRight = 0;
    return true;
  }
  float mag = fabs(error);
  if (mag > 1.0) mag = 1.0;
  int pwm = (int)(TURN_MIN_PWM + (TURN_MAX_PWM - TURN_MIN_PWM) * mag);
  if (error > 0) {
    cmdLeft = -pwm;
    cmdRight = pwm;
  } else {
    cmdLeft = pwm;
    cmdRight = -pwm;
  }
  return false;
}

// Drive while holding a heading; steers proportionally.
void driveHeading(float desired, float theta) {
  float error = angleError(desired, theta);
  if (fabs(error) > 0.5) {
    turnToward(desired, theta);
    return;
  }
  int steer = (int)(80 * error);
  cmdLeft = DRIVE_PWM - steer;
  cmdRight = DRIVE_PWM + steer;
}

// Drive to a point. Returns true when within tol.
bool driveToPoint(float tx, float ty, float tol, float x, float y, float theta) {
  float dx = tx - x;
  float dy = ty - y;
  if (sqrt(dx * dx + dy * dy) < tol) {
    cmdLeft = 0;
    cmdRight = 0;
    return true;
  }
  driveHeading(atan2(dy, dx), theta);
  return false;
}

// Choose the open lane closest to the current one (ties go toward the goal).
// Returns -1 if every lane is marked blocked.
int pickOpenLane() {
  int best = -1;
  for (int i = 0; i < NUM_LANES; i++) {
    if (blockedMask & (1 << i)) continue;
    if (best < 0) {
      best = i;
      continue;
    }
    int dNew = (int)abs(i - lane);
    int dBest = (int)abs(best - lane);
    if (dNew < dBest ||
        (dNew == dBest && fabs(laneY(i) - GOAL_Y) < fabs(laneY(best) - GOAL_Y))) {
      best = i;
    }
  }
  return best;
}

void setState(int s) {
  state = s;
  Enes100.print("state=");
  Enes100.print(s);
  Enes100.print(" lane=");
  Enes100.println(lane);
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
  Enes100.println("CLEAR obstacle navigation ready");

  lane = nearestLane(Enes100.getY());
  laneChangeX = Enes100.getX();
  setState(ST_TO_LANE);
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
  if (!Enes100.isVisible() || x < 0.05 || x > 3.95 || y < 0.05 || y > 1.95) {
    stopPermanently("Stopped: invalid pose");
    return;
  }
  float front = frontDistance();

  cmdLeft = 0;
  cmdRight = 0;

  // Forget old blocks once we are past the column they came from.
  if (blockedMask != 0 && x > blockedAtX + 0.5) {
    blockedMask = 0;
  }

  if (state == ST_TO_LANE) {
    // Side-step along y at a fixed x into the chosen lane.
    if (driveToPoint(laneChangeX, laneY(lane), 0.05, x, y, theta)) {
      setState(ST_FACE_EAST);
    }
  } else if (state == ST_FACE_EAST) {
    if (turnToward(0, theta)) {
      setState(ST_CHECK);
    }
  } else if (state == ST_CHECK) {
    if (front < LOOK_DIST) {
      if (blockedMask == 0) blockedAtX = x;
      blockedMask = blockedMask | (1 << lane);
      int next = pickOpenLane();
      if (next < 0) {
        // Every lane looked blocked (bad reading?). Start over in this column.
        blockedMask = 1 << lane;
        next = pickOpenLane();
      }
      Enes100.print("Lane blocked, front_m=");
      Enes100.println(front);
      lane = next;
      laneChangeX = x;
      setState(ST_TO_LANE);
    } else {
      setState(ST_DRIVE_EAST);
    }
  } else if (state == ST_DRIVE_EAST) {
    if (x > CLEAR_X) {
      setState(ST_TO_GOAL);
    } else if (front < STOP_DIST) {
      Tank.turnOffMotors();
      setState(ST_CHECK);
    } else {
      // Aim slightly ahead on the lane center so drift gets corrected.
      driveHeading(atan2(laneY(lane) - y, 0.3), theta);
    }
  } else if (state == ST_TO_GOAL) {
    if (driveToPoint(GOAL_X, GOAL_Y, 0.10, x, y, theta)) {
      setState(ST_DONE);
    }
  } else if (state == ST_DONE) {
    stopPermanently("Stopped: reached goal");
    return;
  }

  // Printing is slow in the simulator, so only print pose every 10th loop.
  loopCount++;
  if (loopCount % 10 == 0) {
    Enes100.print("x=");
    Enes100.print(x);
    Enes100.print(" y=");
    Enes100.print(y);
    Enes100.print(" theta=");
    Enes100.print(theta);
    Enes100.print(" front_m=");
    Enes100.println(front);
  }

  // Stall guard: only while motors are commanded; a turn counts as progress.
  bool commanded = cmdLeft != 0 || cmdRight != 0;
  if (!commanded || !havePose || fabs(x - lastX) > 0.02 || fabs(y - lastY) > 0.02 ||
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

  if (ENABLE_MOTION && commanded) {
    Tank.setLeftMotorPWM(cmdLeft);
    Tank.setRightMotorPWM(cmdRight);
  } else {
    Tank.turnOffMotors();
  }
  delay(20);
}
