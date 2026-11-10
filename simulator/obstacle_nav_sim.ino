// CLEAR obstacle navigation -- web simulator version.
// Paste this whole file into https://enes100.umd.edu/tools/simulator/
//
// Course layout: 4 m x 2 m. Three blocks sit in two columns (x = 1.4-1.6
// and x = 2.2-2.4), in three lanes centered at y = 0.5, 1.0, 1.5. Each lane
// has exactly one block: two are solid, one is striped. The tank must not
// touch any of them. The front sensor can see the solid blocks but NOT the
// striped one, so the code works out where the striped one is.
//
// Plan:
//   1. Survey: stop just in front of column 1 and point the sensor down each
//      lane. A reading of ~0.1 m means that lane's block is in column 1,
//      ~0.9 m means it's in column 2, and nothing means it's the striped
//      block's lane.
//   2. Through column 1, use a lane whose block is in column 2 (or the
//      striped lane if the striped block must be in column 2). Through
//      column 2, use a lane whose block is in column 1 (or the striped lane
//      if the striped block must be in column 1).
//   3. Past the obstacles, line up with the limbo and drive under it.
//
// Every move is "motors on, then a tight loop that watches position /
// sensor, then motors off" so the tank reacts within a few centimeters even
// though the simulator runs code slowly.
#include <math.h>
#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

// Marker ID and room number are ignored by the simulator.
const int MARKER_ID = 3;
const int ROOM_NUMBER = 1116;

// Lanes.
const float LANE_Y0 = 0.5;
const float LANE_SPACING = 0.5;

// Where the tank stops (robot center x) at each stage.
const float SURVEY_X = 1.2;   // in front of column 1 (face at 1.4)
const float GAP_X = 1.9;      // between column 1 (ends 1.6) and column 2 (starts 2.2)
const float CLEAR_X = 2.7;    // past column 2 (ends 2.4)

// Sensor readings from SURVEY_X (column 1 face ~0.13 m away, column 2 ~0.93 m).
const float COL1_MAX = 0.5;   // closer than this: block is in column 1
const float SAFETY_DIST = 0.08; // stop if anything gets this close while driving

// The log / limbo spans x = 3.35 to 3.45. Log is y = 0 to 1 (orange); the
// limbo opening is y = 1.03 to 1.97 (white bar overhead). We go under the limbo.
const float WALL_X = 3.35;
const float HALF_LENGTH = 0.075;  // robot center to front, meters
const float LIMBO_Y = 1.5;
const float LIMBO_HALF_GAP = 0.3; // max allowed |y - LIMBO_Y| while passing under
const float MAX_X = WALL_X - HALF_LENGTH - 0.10;
const float GOAL_X = 3.7;         // final stop (right arena wall is at x = 4.0)

// Tolerances.
const float HEADING_TOL = 0.04;   // radians (~2 degrees)
const float POS_TOL = 0.02;       // meters

// Motor power (simulator values; tune for the real tank).
const int DRIVE_PWM = 255;
const int SLOW_PWM = 70;          // last part of a move
// Full drive speed is about 0.5 m/s and full turn about 1.4 rad/s in the
// simulator. How early to slow down depends on how long one check takes,
// which the code measures as it goes (pollSec).
const float FAST_M_PER_SEC = 0.5;
const float FAST_RAD_PER_SEC = 1.4;
const int TURN_PWM = 180;
const int FINE_TURN_PWM = 45;     // last few degrees of a turn
const float STEER_GAIN = 120;     // PWM difference per radian of heading error

// Lane survey results.
#define UNKNOWN 0
#define IN_COL1 1
#define IN_COL2 2
#define STRIPED 3

int laneBlock[3] = {UNKNOWN, UNKNOWN, UNKNOWN};
bool done = false;
float pollSec = 0.05;  // measured time for one check of position/sensor
unsigned long lastPollMs = 0;

// Call once per check inside a motion loop to keep pollSec up to date.
void timePoll() {
  unsigned long now = millis();
  if (lastPollMs != 0 && now - lastPollMs < 1000) {
    pollSec = 0.8 * pollSec + 0.2 * (now - lastPollMs) / 1000.0;
  }
  lastPollMs = now;
}

// Slow down this far before a stop point so one late check can't overshoot.
float slowZone() {
  float z = FAST_M_PER_SEC * pollSec * 3;
  if (z < 0.05) z = 0.05;
  return z;
}

float fineTurnZone() {
  float z = FAST_RAD_PER_SEC * pollSec * 3;
  if (z < 0.12) z = 0.12;
  return z;
}
float holdY = 0;  // y line to hold while driving forward

float laneY(int i) {
  return LANE_Y0 + LANE_SPACING * i;
}

float angleError(float desired, float current) {
  return atan2(sin(desired - current), cos(desired - current));
}

// Distance to whatever is ahead, or 99 if nothing is in range.
// Simulator returns -1 for "nothing within 1 m"; real tank maxes out at 1.
float frontDistance() {
  float d = Tank.readDistanceSensor(1);
  if (d <= 0 || d >= 1.0) return 99;
  return d;
}

void setMotors(int left, int right) {
  if (left > 255) left = 255;
  if (left < -255) left = -255;
  if (right > 255) right = 255;
  if (right < -255) right = -255;
  Tank.setLeftMotorPWM(left);
  Tank.setRightMotorPWM(right);
}

void stopMotors() {
  Tank.turnOffMotors();
}

void finish(const char *reason) {
  stopMotors();
  Enes100.println(reason);
  done = true;
}

// Turn in place until facing `desired`. Slows down for the last few degrees.
void turnTo(float desired) {
  unsigned long start = millis();
  float err = angleError(desired, Enes100.getTheta());
  while (fabs(err) > HEADING_TOL && millis() - start < 8000) {
    int pwm = TURN_PWM;
    if (fabs(err) < fineTurnZone()) pwm = FINE_TURN_PWM;
    if (err > 0) {
      setMotors(-pwm, pwm);
    } else {
      setMotors(pwm, -pwm);
    }
    err = angleError(desired, Enes100.getTheta());
    timePoll();
  }
  stopMotors();
  lastPollMs = 0;
}

// Drive forward (+x) holding the line y = holdY until x reaches stopX.
// Returns false (and stops) if something unexpected is right in front, the
// tank stops making progress, or it would hit the log/limbo frame.
bool driveForwardTo(float stopX) {
  turnTo(0);
  lastPollMs = 0;
  unsigned long lastProgress = millis();
  float lastX = Enes100.getX();
  while (true) {
    float x = Enes100.getX();
    float y = Enes100.getY();
    if (x >= stopX - POS_TOL) {
      stopMotors();
      return true;
    }
    if (frontDistance() < SAFETY_DIST) {
      stopMotors();
      return false;
    }
    if (x > MAX_X && fabs(y - LIMBO_Y) > LIMBO_HALF_GAP) {
      stopMotors();
      return false;
    }
    if (x > lastX + 0.02) {
      lastX = x;
      lastProgress = millis();
    } else if (millis() - lastProgress > 1500) {
      stopMotors();
      return false;
    }
    int pwm = DRIVE_PWM;
    if (stopX - x < slowZone()) pwm = SLOW_PWM;
    float steer = STEER_GAIN * angleError(atan2(holdY - y, 0.4), Enes100.getTheta());
    setMotors((int)(pwm - steer), (int)(pwm + steer));
    timePoll();
  }
}

// Slide sideways: face +y or -y and drive until y reaches targetY,
// holding the current x line. Ends facing forward again.
void moveToY(float targetY) {
  float dy = targetY - Enes100.getY();
  if (fabs(dy) >= POS_TOL * 2) {
    float heading = PI / 2;
    float sign = 1;
    if (dy < 0) {
      heading = -PI / 2;
      sign = -1;
    }
    turnTo(heading);
    float holdX = Enes100.getX();
    unsigned long start = millis();
    while (millis() - start < 6000) {
      float y = Enes100.getY();
      float toGo = (targetY - y) * sign;
      if (toGo < POS_TOL) break;
      int pwm = DRIVE_PWM;
      if (toGo < slowZone()) pwm = SLOW_PWM;
      float lineErr = -sign * (holdX - Enes100.getX());
      float steer = STEER_GAIN * (angleError(heading, Enes100.getTheta()) + 2.5 * lineErr);
      setMotors((int)(pwm - steer), (int)(pwm + steer));
      timePoll();
    }
    stopMotors();
    lastPollMs = 0;
  }
  holdY = targetY;
  turnTo(0);
}

// Point the sensor down lane i (from SURVEY_X) and record where its block is.
void surveyLane(int i) {
  moveToY(laneY(i));
  float d = frontDistance();
  // Read twice in case one reading is off.
  float d2 = frontDistance();
  if (d2 < d) d = d2;
  if (d < COL1_MAX) laneBlock[i] = IN_COL1;
  else if (d < 1.0) laneBlock[i] = IN_COL2;
  else laneBlock[i] = STRIPED;
  Enes100.print("Lane ");
  Enes100.print(i);
  Enes100.print(" front_m=");
  Enes100.print(d);
  if (laneBlock[i] == IN_COL1) Enes100.println(" -> block in column 1");
  else if (laneBlock[i] == IN_COL2) Enes100.println(" -> block in column 2");
  else Enes100.println(" -> striped block lane");
}

int countLanes(int kind) {
  int n = 0;
  for (int i = 0; i < 3; i++) {
    if (laneBlock[i] == kind) n++;
  }
  return n;
}

// Of the lanes allowed by `kind` / `kind2`, the one closest to y.
int closestLane(int kind, int kind2, float y) {
  int best = -1;
  for (int i = 0; i < 3; i++) {
    if (laneBlock[i] != kind && laneBlock[i] != kind2) continue;
    if (best < 0 || fabs(laneY(i) - y) < fabs(laneY(best) - y)) best = i;
  }
  return best;
}

void setup() {
  Enes100.begin("CLEAR", WATER, MARKER_ID, ROOM_NUMBER, 52, 50);
  Tank.begin();
  Tank.turnOffMotors();
  Enes100.println("CLEAR obstacle navigation ready");
}

void loop() {
  if (done) {
    delay(100);
    return;
  }

  // 1. Go to the survey line in the nearest lane.
  float y0 = Enes100.getY();
  int first = 0;
  if (fabs(laneY(1) - y0) < fabs(laneY(first) - y0)) first = 1;
  if (fabs(laneY(2) - y0) < fabs(laneY(first) - y0)) first = 2;
  moveToY(laneY(first));
  if (!driveForwardTo(SURVEY_X)) {
    finish("Stopped: something in the way before column 1");
    return;
  }

  // 2. Survey lanes, nearest first, sweeping toward the far side.
  int dir = 1;
  if (first == 2 || (first == 1 && y0 > laneY(1))) dir = -1;
  int order[3];
  order[0] = first;
  if (first == 1) {
    order[1] = 1 + dir;
    order[2] = 1 - dir;
  } else {
    order[1] = first + dir;
    order[2] = first + 2 * dir;
  }
  surveyLane(order[0]);
  surveyLane(order[1]);
  // Two solid lanes found: the third must be the striped one.
  if (laneBlock[order[0]] != STRIPED && laneBlock[order[1]] != STRIPED) {
    laneBlock[order[2]] = STRIPED;
    Enes100.print("Lane ");
    Enes100.print(order[2]);
    Enes100.println(" -> striped block lane (deduced)");
  } else {
    surveyLane(order[2]);
  }
  if (countLanes(STRIPED) != 1 || countLanes(UNKNOWN) != 0) {
    finish("Stopped: survey didn't match the course layout");
    return;
  }

  // 3. Choose lanes. The striped block is in column 2 if both solids are in
  //    column 1, in column 1 if both solids are in column 2, else unknown.
  int stripedIn = 0;
  if (countLanes(IN_COL1) == 2) stripedIn = 2;
  if (countLanes(IN_COL2) == 2) stripedIn = 1;

  int col1Lane;
  if (stripedIn == 2) col1Lane = closestLane(IN_COL2, STRIPED, Enes100.getY());
  else col1Lane = closestLane(IN_COL2, IN_COL2, Enes100.getY());
  int col2Lane;
  if (stripedIn == 1) col2Lane = closestLane(IN_COL1, STRIPED, laneY(col1Lane));
  else col2Lane = closestLane(IN_COL1, IN_COL1, laneY(col1Lane));
  if (col1Lane < 0 || col2Lane < 0) {
    finish("Stopped: no clear lane found");
    return;
  }
  Enes100.print("Route: column 1 via lane ");
  Enes100.print(col1Lane);
  Enes100.print(", column 2 via lane ");
  Enes100.println(col2Lane);

  // 4. Through column 1, across the gap, through column 2.
  moveToY(laneY(col1Lane));
  if (!driveForwardTo(GAP_X)) {
    finish("Stopped: blocked in column 1");
    return;
  }
  moveToY(laneY(col2Lane));
  if (!driveForwardTo(CLEAR_X)) {
    finish("Stopped: blocked in column 2");
    return;
  }

  // 5. Line up with the limbo and drive under it.
  Enes100.println("Past obstacles, lining up with the limbo");
  moveToY(LIMBO_Y);
  if (driveForwardTo(GOAL_X)) {
    finish("Stopped: reached goal (under the limbo)");
  } else {
    finish("Stopped: could not get under the limbo");
  }
}
