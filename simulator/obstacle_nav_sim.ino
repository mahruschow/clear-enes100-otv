// CLEAR obstacle navigation -- web simulator version.
// Paste this whole file into https://enes100.umd.edu/tools/simulator/
//
// Course layout (simulator and arena): 4 m x 2 m. Obstacles sit in two
// columns (x = 1.4-1.6 and x = 2.2-2.4), each split into three lanes
// centered at y = 0.5, 1.0, 1.5. Each column has at least one open lane.
//
// Strategy: pick a lane, face +x, look ahead with the front ultrasonic.
// If the lane is blocked, mark it, side-step to the nearest open lane and
// look again. Once past both columns, line up with the limbo and drive
// straight under it to the goal.
//
// Every move is a timed turn or a timed straight drive (motors on, delay,
// motors off), then the pose is re-checked. delay() is real time, so moves
// are the same size no matter how slowly the simulator steps through code.
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

// The log / limbo spans x = 3.35 to 3.45. Log is y = 0 to 1 (orange); the
// limbo opening is y = 1.03 to 1.97 (white bar overhead). We go under the limbo.
const float WALL_X = 3.35;
const float HALF_LENGTH = 0.075;  // robot center to front, meters
const float LIMBO_Y = 1.5;        // center of the limbo opening
const float LIMBO_HALF_GAP = 0.3; // max allowed |y - LIMBO_Y| while passing under
// Hard limit for the robot's center unless lined up with the limbo.
const float MAX_X = WALL_X - HALF_LENGTH - 0.10;

// Final stop point past the limbo (right arena wall is at x = 4.0).
const float GOAL_X = 3.7;

// Ultrasonic thresholds (meters, measured from the front of the robot).
const float LOOK_DIST = 0.45;  // lane counts as blocked if something is this close
const float STOP_DIST = 0.25;  // never drive closer than this to an obstacle

// Position / heading tolerances.
const float Y_TOL = 0.06;        // meters
const float HEADING_TOL = 0.06;  // radians (~3.5 degrees)
const float MAX_STEP = 0.45;     // longest single straight move, meters

// Motor tuning (simulator values; the real tank needs its own numbers).
const int DRIVE_PWM = 220;
const int TURN_PWM = 150;
// Simulator motion model: speed = (L+R)/255/4 m/s, turn rate = (R-L)/255 rad/s.
// On the real tank, measure these two and replace them.
const float DRIVE_M_PER_SEC = 2.0 * DRIVE_PWM / 255.0 / 4.0;
const float TURN_RAD_PER_SEC = 2.0 * TURN_PWM / 255.0;

#define ST_TO_LANE 0
#define ST_FACE_EAST 1
#define ST_CHECK 2
#define ST_DRIVE_EAST 3
#define ST_TO_LIMBO_Y 4
#define ST_DONE 5
#define ST_FACE_LIMBO 6
#define ST_UNDER_LIMBO 7

int state = ST_TO_LANE;
int lane = 0;
int blockedMask = 0;      // bit i set = lane i blocked in the current column
float blockedAtX = 0;     // x where the current column's blocks were seen

bool halted = false;
int loopCount = 0;

// Stuck detection: a straight move that barely changes the pose.
float moveStartX = 0;
float moveStartY = 0;
float lastStep = 0;
int stuckCount = 0;

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

// Run the motors for a fixed time, then stop.
void timedMove(int left, int right, float seconds) {
  if (!ENABLE_MOTION) return;
  Tank.setLeftMotorPWM(left);
  Tank.setRightMotorPWM(right);
  delay((int)(seconds * 1000));
  Tank.turnOffMotors();
}

// One timed turn toward a heading. Returns true if already facing it.
bool turnTo(float desired, float theta) {
  float error = angleError(desired, theta);
  if (fabs(error) < HEADING_TOL) return true;
  float seconds = fabs(error) / TURN_RAD_PER_SEC;
  if (error > 0) {
    timedMove(-TURN_PWM, TURN_PWM, seconds);
  } else {
    timedMove(TURN_PWM, -TURN_PWM, seconds);
  }
  return false;
}

// One timed straight drive forward.
void driveForward(float meters, float x, float y) {
  moveStartX = x;
  moveStartY = y;
  lastStep = meters;
  timedMove(DRIVE_PWM, DRIVE_PWM, meters / DRIVE_M_PER_SEC);
}

// Side-step along y to targetY: face +y or -y, then drive straight.
// Returns true once within Y_TOL.
bool moveToY(float targetY, float y, float theta) {
  float dy = targetY - y;
  if (fabs(dy) < Y_TOL) return true;
  float facing = PI / 2;
  if (dy < 0) facing = -PI / 2;
  if (turnTo(facing, theta)) {
    float step = fabs(dy);
    if (step > MAX_STEP) step = MAX_STEP;
    driveForward(step, Enes100.getX(), y);
  }
  return false;
}

// Choose the open lane closest to the current one (ties go toward the limbo).
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
        (dNew == dBest && fabs(laneY(i) - LIMBO_Y) < fabs(laneY(best) - LIMBO_Y))) {
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
  Enes100.println("CLEAR obstacle navigation ready");

  lane = nearestLane(Enes100.getY());
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
  // Never touch the log/limbo frame: past MAX_X only while passing under the
  // limbo and lined up with its opening.
  bool passingLimbo = (state == ST_UNDER_LIMBO || state == ST_DONE);
  if (x > MAX_X && (!passingLimbo || fabs(y - LIMBO_Y) > LIMBO_HALF_GAP)) {
    stopPermanently("Stopped: too close to the log/limbo frame");
    return;
  }

  // Stuck check: did the last straight move actually move us?
  if (lastStep > 0.05) {
    float moved = sqrt((x - moveStartX) * (x - moveStartX) + (y - moveStartY) * (y - moveStartY));
    if (moved < 0.3 * lastStep) {
      stuckCount++;
      if (stuckCount >= 3) {
        stopPermanently("Stopped: stuck (drive moves are not changing the pose)");
        return;
      }
    } else {
      stuckCount = 0;
    }
  }
  lastStep = 0;

  float front = frontDistance();

  // Forget old blocks once we are past the column they came from.
  if (blockedMask != 0 && x > blockedAtX + 0.5) {
    blockedMask = 0;
  }

  if (state == ST_TO_LANE) {
    if (moveToY(laneY(lane), y, theta)) {
      setState(ST_FACE_EAST);
    }
  } else if (state == ST_FACE_EAST) {
    if (turnTo(0, theta)) {
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
      setState(ST_TO_LANE);
    } else {
      setState(ST_DRIVE_EAST);
    }
  } else if (state == ST_DRIVE_EAST) {
    if (x > CLEAR_X) {
      setState(ST_TO_LIMBO_Y);
    } else if (front < STOP_DIST + 0.03) {
      setState(ST_CHECK);
    } else {
      // Aim slightly ahead on the lane center so drift gets corrected.
      float desired = atan2(laneY(lane) - y, 0.4);
      if (fabs(angleError(desired, theta)) > 0.12) {
        turnTo(desired, theta);
      } else {
        float step = MAX_STEP;
        if (front - STOP_DIST < step) step = front - STOP_DIST;
        if (CLEAR_X + 0.05 - x < step) step = CLEAR_X + 0.05 - x;
        driveForward(step, x, y);
      }
    }
  } else if (state == ST_TO_LIMBO_Y) {
    // Past the obstacles: side-step onto the limbo center line.
    if (moveToY(LIMBO_Y, y, theta)) {
      setState(ST_FACE_LIMBO);
    }
  } else if (state == ST_FACE_LIMBO) {
    if (turnTo(0, theta)) {
      setState(ST_UNDER_LIMBO);
    }
  } else if (state == ST_UNDER_LIMBO) {
    // Straight under the limbo, holding y on its center line.
    float remaining = GOAL_X - x;
    if (remaining < 0.05) {
      setState(ST_DONE);
    } else {
      float desired = atan2(LIMBO_Y - y, 0.4);
      if (fabs(angleError(desired, theta)) > 0.12) {
        turnTo(desired, theta);
      } else {
        float step = MAX_STEP;
        if (remaining < step) step = remaining;
        driveForward(step, x, y);
      }
    }
  } else if (state == ST_DONE) {
    stopPermanently("Stopped: reached goal (under the limbo)");
    return;
  }

  // Printing is slow in the simulator, so only print pose every 5th loop.
  loopCount++;
  if (loopCount % 5 == 0) {
    Enes100.print("x=");
    Enes100.print(x);
    Enes100.print(" y=");
    Enes100.print(y);
    Enes100.print(" theta=");
    Enes100.print(theta);
    Enes100.print(" front_m=");
    Enes100.println(front);
  }
}
