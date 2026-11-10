// CLEAR obstacle navigation -- web simulator version.
// Paste this whole file into https://enes100.umd.edu/tools/simulator/
//
// THE TANK MUST NEVER TOUCH ANY BLOCK -- SOLID OR STRIPED.
//
// Course: 4 m x 2 m. Three blocks sit in two columns (x = 1.4-1.6 and
// x = 2.2-2.4) and three lanes (y = 0.5, 1.0, 1.5), one block per lane.
// Two are solid, one is striped. The sensor can't see the striped one.
//
// Plan (simple on purpose):
//   1. Drive up to column 1 and look down each lane with the sensor:
//      ~0.1 m = block in column 1, ~0.9 m = block in column 2,
//      nothing = the striped block's lane.
//   2. Go through column 1 in a lane whose block is in column 2, then
//      through column 2 in a lane whose block is in column 1.
//   3. Line up with the limbo and drive under it.
//
// Every move is: motors on, delay(), motors off, then check the position
// and fix any error. delay() is real time, so moves stay accurate no
// matter how slowly the simulator runs the code.
#include <math.h>
#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

// Marker ID and room number are ignored by the simulator.
const int MARKER_ID = 3;
const int ROOM_NUMBER = 1116;

// Where the tank stops (robot center x) at each stage.
const float SURVEY_X = 1.21;  // in front of column 1 (its face is at 1.4)
const float GAP_X = 1.9;      // between column 1 and column 2
const float CLEAR_X = 2.7;    // past column 2
const float LIMBO_Y = 1.5;    // center of the limbo opening
const float GOAL_X = 3.7;     // final stop, past the limbo

const float COL1_MAX = 0.5;   // survey reading closer than this = column 1

// Tolerances.
const float HEADING_TOL = 0.05;  // radians (~3 degrees)
const float POS_TOL = 0.03;      // meters

// Motor power and the speeds it gives in the simulator
// (speed = (L+R)/255/4 m/s, turn rate = (R-L)/255 rad/s).
// On the real tank, measure the two rates and replace them.
const int DRIVE_PWM = 255;
const int TURN_PWM = 200;
const float DRIVE_M_PER_SEC = 0.5;
const float TURN_RAD_PER_SEC = 1.569;

// What the survey found in each lane.
#define UNKNOWN 0
#define IN_COL1 1
#define IN_COL2 2
#define STRIPED 3
int lane0 = UNKNOWN;
int lane1 = UNKNOWN;
int lane2 = UNKNOWN;

bool done = false;
// Motors stay on a little longer than delay() because the simulator is slow
// to run the "motors off" line. Measured after every move.
float extraSec = 0.15;

float laneY(int i) {
  return 0.5 + 0.5 * i;
}

int getLane(int i) {
  if (i == 0) return lane0;
  if (i == 1) return lane1;
  return lane2;
}

void setLane(int i, int v) {
  if (i == 0) lane0 = v;
  else if (i == 1) lane1 = v;
  else lane2 = v;
}

int countLanes(int kind) {
  int n = 0;
  if (lane0 == kind) n++;
  if (lane1 == kind) n++;
  if (lane2 == kind) n++;
  return n;
}

float angleError(float desired, float current) {
  return atan2(sin(desired - current), cos(desired - current));
}

// Distance to whatever is ahead, or 99 if nothing is within 1 m.
float frontDistance() {
  float d = Tank.readDistanceSensor(1);
  if (d <= 0 || d >= 1.0) return 99;
  return d;
}

void finish(const char *reason) {
  Tank.turnOffMotors();
  Enes100.println(reason);
  done = true;
}

// One timed move of `amount` (meters or radians). leftSign / rightSign are
// +1 or -1. Returns the rate used, so the caller can measure the overrun.
float moveSec = 0;
float timedMove(float amount, int pwm, float fullRate, int leftSign, int rightSign) {
  float rate = fullRate;
  int fullPwm = pwm;
  moveSec = amount / fullRate - extraSec;
  if (moveSec < 0.05) {
    // Small move: lower the power so the extra motor-on time doesn't overshoot.
    moveSec = 0.05;
    rate = amount / (moveSec + extraSec);
    pwm = (int)(pwm * rate / fullRate);
    if (pwm < 10) pwm = 10;
    rate = fullRate * pwm / fullPwm;
  }
  Tank.setLeftMotorPWM(leftSign * pwm);
  Tank.setRightMotorPWM(rightSign * pwm);
  delay((int)(moveSec * 1000));
  Tank.turnOffMotors();
  return rate;
}

void learn(float actual, float rate) {
  float sample = actual / rate - moveSec;
  if (sample < 0) sample = 0;
  if (sample > 1.5) sample = 1.5;
  extraSec = 0.5 * extraSec + 0.5 * sample;
}

// Turn in place to face `desired`.
void turnTo(float desired) {
  float theta = Enes100.getTheta();
  for (int k = 0; k < 8; k++) {
    float err = angleError(desired, theta);
    if (fabs(err) < HEADING_TOL) return;
    float rate;
    if (err > 0) rate = timedMove(fabs(err), TURN_PWM, TURN_RAD_PER_SEC, -1, 1);
    else rate = timedMove(fabs(err), TURN_PWM, TURN_RAD_PER_SEC, 1, -1);
    float after = Enes100.getTheta();
    learn(fabs(angleError(after, theta)), rate);
    theta = after;
  }
}

// Drive straight until x (alongX) or y reaches `target`. Faces +x for x
// moves, and +y or -y for y moves. Returns false if something is in the way
// or the tank isn't moving.
bool driveTo(bool alongX, float target) {
  float pos;
  if (alongX) pos = Enes100.getX();
  else pos = Enes100.getY();
  float heading = 0;
  float sign = 1;
  if (!alongX) {
    heading = PI / 2;
    if (target < pos) {
      heading = -PI / 2;
      sign = -1;
    }
  }
  int stuck = 0;
  for (int k = 0; k < 8; k++) {
    float toGo = (target - pos) * sign;  // negative = overshot, back up
    if (fabs(toGo) < POS_TOL) return true;
    turnTo(heading);
    if (alongX && toGo > 0 && frontDistance() < toGo + 0.03) return false;
    float rate;
    if (toGo > 0) rate = timedMove(toGo, DRIVE_PWM, DRIVE_M_PER_SEC, 1, 1);
    else rate = timedMove(-toGo, DRIVE_PWM, DRIVE_M_PER_SEC, -1, -1);
    float after;
    if (alongX) after = Enes100.getX();
    else after = Enes100.getY();
    float moved = fabs(after - pos);
    learn(moved, rate);
    if (fabs(toGo) > 0.08 && moved < 0.3 * fabs(toGo)) {
      stuck++;
      if (stuck >= 2) return false;
    }
    pos = after;
  }
  return fabs(target - pos) < 2 * POS_TOL;
}

// Look down lane i from SURVEY_X and record where its block is.
void surveyLane(int i, bool slide) {
  if (slide) {
    driveTo(false, laneY(i));
    // Sliding can drift a few cm toward the column; get back on the line.
    driveTo(true, SURVEY_X);
  }
  turnTo(0);
  float d = frontDistance();
  if (d < COL1_MAX) setLane(i, IN_COL1);
  else if (d < 1.0) setLane(i, IN_COL2);
  else setLane(i, STRIPED);
  Enes100.print("Lane ");
  Enes100.print(i);
  Enes100.print(" front_m=");
  Enes100.print(d);
  if (getLane(i) == IN_COL1) Enes100.println(" -> block in column 1");
  else if (getLane(i) == IN_COL2) Enes100.println(" -> block in column 2");
  else Enes100.println(" -> striped block lane");
}

// Of the lanes of kind a or b, the one closest to y (or -1).
int closestLane(int a, int b, float y) {
  int best = -1;
  for (int i = 0; i < 3; i++) {
    int v = getLane(i);
    if (v != a && v != b) continue;
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

  // 1. Drive straight up to column 1.
  if (!driveTo(true, SURVEY_X)) {
    finish("Stopped: something in the way before column 1");
    return;
  }

  // 2. Survey: the lane we're in, then the middle, then the last if needed.
  float y = Enes100.getY();
  int first = 0;
  if (fabs(laneY(1) - y) < fabs(laneY(first) - y)) first = 1;
  if (fabs(laneY(2) - y) < fabs(laneY(first) - y)) first = 2;
  int second = 1;
  if (first == 1) second = 0;
  int third = 3 - first - second;
  surveyLane(first, fabs(laneY(first) - y) > 0.1);
  surveyLane(second, true);
  if (getLane(first) != STRIPED && getLane(second) != STRIPED) {
    setLane(third, STRIPED);  // two solid lanes found: the third is striped
    Enes100.print("Lane ");
    Enes100.print(third);
    Enes100.println(" -> striped block lane (deduced)");
  } else {
    surveyLane(third, true);
  }
  if (countLanes(STRIPED) != 1 || countLanes(UNKNOWN) != 0) {
    finish("Stopped: survey didn't match the course layout");
    return;
  }

  // 3. Choose lanes. The striped block is in column 2 if both solid blocks
  //    are in column 1, and in column 1 if both are in column 2. Otherwise
  //    we don't know its column, so we never use its lane.
  y = Enes100.getY();
  int col1Lane;
  if (countLanes(IN_COL1) == 2) col1Lane = closestLane(IN_COL2, STRIPED, y);
  else col1Lane = closestLane(IN_COL2, IN_COL2, y);
  int col2Lane;
  if (countLanes(IN_COL2) == 2) col2Lane = closestLane(IN_COL1, STRIPED, laneY(col1Lane));
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
  if (!driveTo(false, laneY(col1Lane)) || !driveTo(true, GAP_X)) {
    finish("Stopped: blocked at column 1");
    return;
  }
  if (!driveTo(false, laneY(col2Lane)) || !driveTo(true, CLEAR_X)) {
    finish("Stopped: blocked at column 2");
    return;
  }

  // 5. Line up with the limbo and drive straight under it.
  Enes100.println("Past obstacles, lining up with the limbo");
  if (driveTo(false, LIMBO_Y) && driveTo(true, GOAL_X)) {
    finish("Stopped: reached goal (under the limbo)");
  } else {
    finish("Stopped: could not get under the limbo");
  }
}
