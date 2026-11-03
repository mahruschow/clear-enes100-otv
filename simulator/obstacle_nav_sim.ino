// CLEAR obstacle navigation -- web simulator version.
// Paste this whole file into https://enes100.umd.edu/tools/simulator/
//
// Logic:
//   1. Face forward (+x) and drive forward, checking the front sensor
//      continuously while the motors run.
//   2. When an object is close ahead, stop and slowly turn while watching
//      the sensor until it sees an opening (nothing within 1 m) and then
//      where the opening ends. Try the side toward the middle first.
//   3. Slide sideways to the middle of the opening, face forward, repeat.
//   4. Once past the obstacles, line up with the limbo and drive under it.
//
// Striped blocks are rumble strips: the course expects you to drive over
// them, and the sensor can't see them.
//
// Every move is "motors on, then a tight loop that watches the sensor /
// position, then motors off" so the tank reacts within a few centimeters
// even though the simulator runs code slowly.
#include <math.h>
#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

// Marker ID and room number are ignored by the simulator.
const int MARKER_ID = 3;
const int ROOM_NUMBER = 1116;

// Arena is 4 m x 2 m. Once x passes this, all obstacles are behind us.
const float CLEAR_X = 2.7;
const float MID_Y = 1.0;

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

// Front sensor distances (meters, from the front of the robot).
const float DETECT_DIST = 0.42;  // object this close ahead -> stop and look for an opening
const float OPEN_DIST = 1.0;     // sensor sees nothing within this -> opening
const float CONFIRM_DIST = 0.6;  // still part of the opening while farther than this

// Scanning / side-stepping.
const float SCAN_MAX = 1.1;      // farthest we turn from forward (~63 degrees)
const float SIDE_MARGIN = 0.2;   // extra sideways distance past an opening's edge
const float BLIND_SIDESTEP = 0.5; // slide this far when no opening is in view
const float MIN_SIDE_Y = 0.35;   // stay this far from the side walls
const float MAX_SIDE_Y = 1.65;

// Tolerances.
const float HEADING_TOL = 0.05;  // radians (~3 degrees)
const float POS_TOL = 0.02;      // meters

// Motor power (simulator values; tune for the real tank).
const int DRIVE_PWM = 255;
const int SLOW_PWM = 90;         // last few cm of a move
const int TURN_PWM = 180;
const int FINE_TURN_PWM = 60;    // last few degrees of a turn
const int SCAN_PWM = 110;        // slow sweep while looking for an opening
const float STEER_GAIN = 120;    // PWM difference per radian of heading error

// Results from driveForward().
#define REACHED 0
#define BLOCKED 1
#define STUCK 2

bool done = false;
float holdY = 0;           // y line to hold while driving forward
int recoveries = 0;
const int MAX_RECOVERIES = 8;

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
    if (fabs(err) < 0.35) pwm = FINE_TURN_PWM;
    if (err > 0) {
      setMotors(-pwm, pwm);
    } else {
      setMotors(pwm, -pwm);
    }
    err = angleError(desired, Enes100.getTheta());
  }
  stopMotors();
}

// Drive forward (+x) holding the line y = holdY until x reaches stopX.
// If useSensor, stops early when something is within DETECT_DIST.
int driveForward(float stopX, bool useSensor) {
  turnTo(0);
  unsigned long lastProgress = millis();
  float lastX = Enes100.getX();
  while (true) {
    float x = Enes100.getX();
    float y = Enes100.getY();
    if (x >= stopX - POS_TOL) {
      stopMotors();
      return REACHED;
    }
    if (useSensor && frontDistance() < DETECT_DIST) {
      stopMotors();
      return BLOCKED;
    }
    if (x > MAX_X && fabs(y - LIMBO_Y) > LIMBO_HALF_GAP) {
      stopMotors();
      return STUCK;  // about to hit the log/limbo frame
    }
    // Progress check: if x hasn't moved for 1.5 s we're pushing on something.
    if (x > lastX + 0.02) {
      lastX = x;
      lastProgress = millis();
    } else if (millis() - lastProgress > 1500) {
      stopMotors();
      return STUCK;
    }
    int pwm = DRIVE_PWM;
    if (stopX - x < 0.1) pwm = SLOW_PWM;
    float steer = STEER_GAIN * angleError(atan2(holdY - y, 0.4), Enes100.getTheta());
    setMotors((int)(pwm - steer), (int)(pwm + steer));
  }
}

// Slide sideways: face +y or -y and drive until y reaches targetY,
// holding the current x line.
void moveToY(float targetY) {
  float dy = targetY - Enes100.getY();
  if (fabs(dy) < POS_TOL * 2) return;
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
    if (toGo < 0.1) pwm = SLOW_PWM;
    float lineErr = -sign * (holdX - Enes100.getX());
    float steer = STEER_GAIN * (angleError(heading, Enes100.getTheta()) + 2.5 * lineErr);
    setMotors((int)(pwm - steer), (int)(pwm + steer));
  }
  stopMotors();
}

// Back straight up a little (after bumping into something).
void backUp(float meters) {
  float x0 = Enes100.getX();
  float y0 = Enes100.getY();
  unsigned long start = millis();
  setMotors(-SLOW_PWM, -SLOW_PWM);
  while (millis() - start < 3000) {
    float dx = Enes100.getX() - x0;
    float dy = Enes100.getY() - y0;
    if (sqrt(dx * dx + dy * dy) >= meters) break;
  }
  stopMotors();
}

// Sweep slowly from forward toward side `dir` (+1 = +y, -1 = -y), watching
// the sensor. Returns true if an opening was found; openStart / openEnd are
// the headings where it started and ended, bounded says whether we saw the
// far edge too.
float openStart = 0;
float openEnd = 0;
bool bounded = false;
bool scanSide(int dir) {
  turnTo(0);
  bool inOpening = false;
  bounded = false;
  if (dir > 0) {
    setMotors(-SCAN_PWM, SCAN_PWM);
  } else {
    setMotors(SCAN_PWM, -SCAN_PWM);
  }
  unsigned long start = millis();
  while (millis() - start < 6000) {
    float heading = angleError(Enes100.getTheta(), 0);
    if (fabs(heading) > SCAN_MAX) break;
    float d = frontDistance();
    if (!inOpening && d > OPEN_DIST) {
      inOpening = true;
      openStart = heading;
      openEnd = heading;
    } else if (inOpening && d > CONFIRM_DIST) {
      openEnd = heading;
    } else if (inOpening) {
      if (fabs(openEnd - openStart) > 0.08) {
        bounded = true;
        break;
      }
      inOpening = false;  // too narrow to be a real opening; keep looking
    }
  }
  stopMotors();
  return inOpening;
}

// Work out how far to slide to line up with the opening just scanned.
// Returns the target y, or -1 if the opening isn't usable.
float openingTargetY(int dir, float objectDist, float y) {
  float d = objectDist + HALF_LENGTH;
  float side;
  if (bounded) {
    side = d * (tan(fabs(openStart)) + tan(fabs(openEnd))) / 2;
  } else {
    side = d * tan(fabs(openStart)) + SIDE_MARGIN;
  }
  float target = y + dir * side;
  if (target < MIN_SIDE_Y) target = MIN_SIDE_Y;
  if (target > MAX_SIDE_Y) target = MAX_SIDE_Y;
  if (fabs(target - y) < 0.15) return -1;  // just the gap along a wall
  return target;
}

// An object is ahead: find an opening and slide over to it.
void getAroundObject() {
  float objectDist = frontDistance();
  float y = Enes100.getY();
  Enes100.print("Object ahead at ");
  Enes100.println(objectDist);

  int firstDir = 1;
  if (y > MID_Y + 0.1) firstDir = -1;

  float target = -1;
  if (scanSide(firstDir)) target = openingTargetY(firstDir, objectDist, y);
  if (target < 0 && scanSide(-firstDir)) target = openingTargetY(-firstDir, objectDist, y);

  if (target < 0) {
    // Nothing usable in view: slide one obstacle-width and look again.
    target = y + firstDir * BLIND_SIDESTEP;
    if (target < MIN_SIDE_Y || target > MAX_SIDE_Y) target = y - firstDir * BLIND_SIDESTEP;
    if (target < MIN_SIDE_Y) target = MIN_SIDE_Y;
    if (target > MAX_SIDE_Y) target = MAX_SIDE_Y;
    recoveries++;
    Enes100.print("No opening in view, sliding to y=");
  } else {
    Enes100.print("Opening found, sliding to y=");
  }
  Enes100.println(target);
  moveToY(target);
  holdY = Enes100.getY();
}

void setup() {
  Enes100.begin("CLEAR", WATER, MARKER_ID, ROOM_NUMBER, 52, 50);
  Tank.begin();
  Tank.turnOffMotors();
  Enes100.println("CLEAR obstacle navigation ready");
  holdY = Enes100.getY();
}

void loop() {
  if (done) {
    delay(100);
    return;
  }

  // 1. Drive forward until something is in the way or we're past the obstacles.
  int result = driveForward(CLEAR_X, true);

  if (result == BLOCKED) {
    getAroundObject();
  } else if (result == STUCK) {
    recoveries++;
    Enes100.println("Bumped something, backing up");
    backUp(0.15);
    getAroundObject();
  } else {
    // 2. Past the obstacles: line up with the limbo and drive under it.
    Enes100.println("Past obstacles, lining up with the limbo");
    moveToY(LIMBO_Y);
    holdY = LIMBO_Y;
    if (driveForward(GOAL_X, false) == REACHED) {
      finish("Stopped: reached goal (under the limbo)");
    } else {
      finish("Stopped: could not get under the limbo");
    }
    return;
  }

  if (recoveries > MAX_RECOVERIES) {
    finish("Stopped: no way through found");
  }
}
