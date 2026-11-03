// CLEAR obstacle navigation -- web simulator version.
// Paste this whole file into https://enes100.umd.edu/tools/simulator/
//
// Logic:
//   1. Face forward (+x) and drive forward.
//   2. If the front sensor sees an object, stop and turn in small steps
//      until the sensor sees an opening (nothing within 1 m). Try the side
//      toward the middle of the arena first, then the other side.
//   3. Slide sideways to line up with the opening, turn back to face
//      forward, and keep driving.
//   4. Once past the obstacles, line up with the limbo and drive under it.
//
// Each move is motors on -> delay -> motors off. The simulator runs code
// slowly, so motors stay on a bit longer than asked. The code measures that
// extra time after every move and subtracts it from the next one, so it
// doesn't overshoot and wobble back and forth.
#include <math.h>
#include "Arduino.h"
#include "Enes100.h"
#include "Tank.h"

// Marker ID and room number are ignored by the simulator.
const int MARKER_ID = 3;
const int ROOM_NUMBER = 1116;

const bool ENABLE_MOTION = true;

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
const float STOP_DIST = 0.25;    // never drive closer than this

// Scanning for an opening.
const float SCAN_STEP = 0.2;     // radians per scan turn (~11 degrees)
const float SCAN_MAX = 1.1;      // farthest we turn from forward (~63 degrees)
const float CONFIRM_DIST = 0.6;  // still "open" while the sensor reads farther than this
const float SIDE_MARGIN = 0.2;   // extra sideways distance past the object's edge
const float BLIND_SIDESTEP = 0.5; // slide this far when no opening is in view
const float MIN_SIDE_Y = 0.35;   // stay this far from the side walls
const float MAX_SIDE_Y = 1.65;

// Tolerances / step size.
const float HEADING_TOL = 0.12;  // radians (~7 degrees)
const float Y_TOL = 0.08;        // meters
const float SIDE_TOL = 0.04;     // meters, when lining up with an opening
const float MAX_STEP = 0.45;     // longest single straight move, meters

// Motor tuning (simulator values; the real tank needs its own numbers).
const int DRIVE_PWM = 220;
const int TURN_PWM = 150;
// Simulator motion model: speed = (L+R)/255/4 m/s, turn rate = (R-L)/255 rad/s.
// On the real tank, measure these two and replace them.
const float DRIVE_M_PER_SEC = 2.0 * DRIVE_PWM / 255.0 / 4.0;
const float TURN_RAD_PER_SEC = 2.0 * TURN_PWM / 255.0;
const float MIN_MOVE_SEC = 0.05;

#define ST_FACE_FORWARD 0
#define ST_DRIVE 1
#define ST_SCAN 2
#define ST_SIDESTEP 3
#define ST_TO_LIMBO_Y 4
#define ST_FACE_LIMBO 5
#define ST_UNDER_LIMBO 6
#define ST_DONE 7

int state = ST_FACE_FORWARD;

// Scan state.
int scanDir = 1;           // +1 = turning toward +y, -1 = toward -y
int scanSidesTried = 0;
int scanFirstDir = 1;      // side tried first in the current scan
float scanAngle = 0;       // current scan heading relative to forward
bool scanSawOpen = false;  // currently turning through an opening

// Side-step state.
float objectDist = 0;      // front reading when the object was detected
float openAngle = 0;       // first scan heading that was clear
float openEndAngle = 0;    // last scan heading that was still clear
float sideTargetY = 0;     // y to side-step to so we line up with the opening
float holdY = 0;           // y line to hold while driving forward
int sideMoves = 0;         // drive moves made in the current side-step

bool halted = false;
int loopCount = 0;

// Learned extra motor-on time per move (seconds), and the last move made,
// so the next loop can measure how far it really went.
float extraSec = 0.15;
int lastMoveKind = 0;      // 0 none, 1 turn, 2 drive
float lastMoveSec = 0;
float lastMoveRate = 0;
float moveStartX = 0;
float moveStartY = 0;
float moveStartTheta = 0;
int stuckCount = 0;
int recoveries = 0;        // times we backed up to get unstuck
const int MAX_RECOVERIES = 6;

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

// Run the motors for a fixed time, then stop.
void timedMove(int left, int right, float seconds) {
  if (!ENABLE_MOTION) return;
  Tank.setLeftMotorPWM(left);
  Tank.setRightMotorPWM(right);
  delay((int)(seconds * 1000));
  Tank.turnOffMotors();
}

// Pick power and time for a move of `amount` (radians or meters) at full
// `pwm` / `fullRate`, accounting for the learned extra time. Small moves use
// lower power so the extra time doesn't overshoot them.
int planPwm = 0;
float planSec = 0;
float planRate = 0;
void planMove(float amount, int pwm, float fullRate) {
  float sec = amount / fullRate - extraSec;
  if (sec >= MIN_MOVE_SEC) {
    planPwm = pwm;
    planSec = sec;
    planRate = fullRate;
    return;
  }
  float rate = amount / (MIN_MOVE_SEC + extraSec);
  planPwm = (int)(pwm * rate / fullRate);
  if (planPwm < 20) planPwm = 20;
  planRate = fullRate * planPwm / pwm;
  planSec = MIN_MOVE_SEC;
}

void rememberMove(int kind, float x, float y, float theta) {
  lastMoveKind = kind;
  lastMoveSec = planSec;
  lastMoveRate = planRate;
  moveStartX = x;
  moveStartY = y;
  moveStartTheta = theta;
}

// One timed turn toward a heading. Returns true if already facing it.
bool turnTo(float desired, float x, float y, float theta) {
  float error = angleError(desired, theta);
  if (fabs(error) < HEADING_TOL) return true;
  planMove(fabs(error), TURN_PWM, TURN_RAD_PER_SEC);
  rememberMove(1, x, y, theta);
  if (error > 0) {
    timedMove(-planPwm, planPwm, planSec);
  } else {
    timedMove(planPwm, -planPwm, planSec);
  }
  return false;
}

// One timed straight drive forward.
void driveForward(float meters, float x, float y, float theta) {
  if (meters < 0.02) return;
  planMove(meters, DRIVE_PWM, DRIVE_M_PER_SEC);
  rememberMove(2, x, y, theta);
  timedMove(planPwm, planPwm, planSec);
}

// After a move, see how far it really went and update extraSec.
// Returns false if we look stuck.
bool learnFromLastMove(float x, float y, float theta) {
  if (lastMoveKind == 0) return true;
  float actual;
  if (lastMoveKind == 1) {
    actual = fabs(angleError(theta, moveStartTheta));
  } else {
    actual = sqrt((x - moveStartX) * (x - moveStartX) + (y - moveStartY) * (y - moveStartY));
  }
  float sample = actual / lastMoveRate - lastMoveSec;
  if (sample < 0) sample = 0;
  if (sample > 0.5) sample = 0.5;
  extraSec = 0.6 * extraSec + 0.4 * sample;

  bool ok = true;
  float minExpected = 0.05;          // meters
  if (lastMoveKind == 1) minExpected = 0.15;  // radians
  if (lastMoveRate * lastMoveSec > minExpected &&
      actual < 0.3 * lastMoveRate * lastMoveSec) {
    // Moved the motors but barely moved: we're pushing on something.
    stuckCount++;
    ok = false;
  } else if (lastMoveKind == 2) {
    stuckCount = 0;
  }
  lastMoveKind = 0;
  return ok;
}

// Back straight up (used to get unstuck or to get room to look again).
void backUp(float meters) {
  if (!ENABLE_MOTION) return;
  timedMove(-DRIVE_PWM, -DRIVE_PWM, meters / DRIVE_M_PER_SEC);
}

void setState(int s) {
  state = s;
  sideMoves = 0;
  Enes100.print("state=");
  Enes100.println(s);
}

void stopPermanently(const char *reason) {
  Tank.turnOffMotors();
  if (!halted) {
    Enes100.println(reason);
  }
  halted = true; // Press stop + run again for another attempt.
}

// No usable opening visible from here: slide one obstacle-width sideways
// (toward the middle first) and look again from there.
void blindSlide(float y) {
  recoveries++;
  if (recoveries > MAX_RECOVERIES) {
    stopPermanently("Stopped: no opening found");
    return;
  }
  sideTargetY = y + scanFirstDir * BLIND_SIDESTEP;
  if (sideTargetY < MIN_SIDE_Y || sideTargetY > MAX_SIDE_Y) {
    sideTargetY = y - scanFirstDir * BLIND_SIDESTEP;
  }
  if (sideTargetY < MIN_SIDE_Y) sideTargetY = MIN_SIDE_Y;
  if (sideTargetY > MAX_SIDE_Y) sideTargetY = MAX_SIDE_Y;
  Enes100.print("No opening in view, sliding to y=");
  Enes100.println(sideTargetY);
  setState(ST_SIDESTEP);
}

// Opening seen from openAngle to openEndAngle. If both edges were seen
// (bounded), slide to its middle; otherwise slide just past its near edge.
void finishOpening(bool bounded, float y) {
  float d = objectDist + HALF_LENGTH;
  float side;
  if (bounded) {
    side = d * (tan(fabs(openAngle)) + tan(fabs(openEndAngle))) / 2;
  } else {
    side = d * tan(fabs(openAngle)) + SIDE_MARGIN;
  }
  sideTargetY = y + scanDir * side;
  if (sideTargetY < MIN_SIDE_Y) sideTargetY = MIN_SIDE_Y;
  if (sideTargetY > MAX_SIDE_Y) sideTargetY = MAX_SIDE_Y;
  if (fabs(sideTargetY - y) < 0.15) {
    // The "opening" is just the gap along the wall; we can't fit there.
    blindSlide(y);
    return;
  }
  Enes100.print("Opening found, side-stepping to y=");
  Enes100.println(sideTargetY);
  setState(ST_SIDESTEP);
}

void startScan(float y) {
  // Try the side toward the middle of the arena first.
  scanDir = 1;
  if (y > MID_Y + 0.1) scanDir = -1;
  if (fabs(y - MID_Y) <= 0.1 && LIMBO_Y < MID_Y) scanDir = -1;
  scanFirstDir = scanDir;
  scanSidesTried = 1;
  scanAngle = 0;
  scanSawOpen = false;
  setState(ST_SCAN);
}

void setup() {
  Enes100.begin("CLEAR", WATER, MARKER_ID, ROOM_NUMBER, 52, 50);
  Tank.begin();
  Tank.turnOffMotors();
  Enes100.println("CLEAR obstacle navigation ready");
  holdY = Enes100.getY();
  setState(ST_FACE_FORWARD);
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
  if (!learnFromLastMove(x, y, theta)) {
    // Bumped into something: back up and look for an opening again.
    recoveries++;
    if (recoveries > MAX_RECOVERIES) {
      stopPermanently("Stopped: stuck too many times");
      return;
    }
    Enes100.println("Bumped something, backing up");
    backUp(0.15);
    setState(ST_FACE_FORWARD);
    return;
  }

  float front = frontDistance();

  if (state == ST_FACE_FORWARD) {
    if (turnTo(0, x, y, theta)) {
      setState(ST_DRIVE);
    }
  } else if (state == ST_DRIVE) {
    // Drive forward until something is in the way.
    if (x > CLEAR_X) {
      setState(ST_TO_LIMBO_Y);
    } else if (front < DETECT_DIST) {
      Enes100.print("Object ahead, front_m=");
      Enes100.println(front);
      objectDist = front;
      startScan(y);
    } else if (fabs(angleError(atan2(holdY - y, 0.5), theta)) > HEADING_TOL) {
      // Steer back onto the y line we're holding so drift doesn't add up.
      turnTo(atan2(holdY - y, 0.5), x, y, theta);
    } else {
      // Stop a little inside DETECT_DIST so there is room to look around.
      float step = MAX_STEP;
      if (front - (DETECT_DIST - 0.05) < step) step = front - (DETECT_DIST - 0.05);
      if (CLEAR_X + 0.05 - x < step) step = CLEAR_X + 0.05 - x;
      driveForward(step, x, y, theta);
    }
  } else if (state == ST_SCAN) {
    // Turn a little at a time. Find where an opening starts (nothing within
    // 1 m), keep turning to find where it ends, then slide to its middle.
    bool canStep = fabs(scanAngle) + SCAN_STEP <= SCAN_MAX + 0.01;
    if (!turnTo(scanAngle, x, y, theta)) {
      // still turning to the current scan angle
    } else if (!scanSawOpen && front > 1.0 && scanAngle != 0) {
      scanSawOpen = true;
      openAngle = scanAngle;
      openEndAngle = scanAngle;
      if (canStep) {
        scanAngle = scanAngle + scanDir * SCAN_STEP;
      } else {
        finishOpening(false, y);
      }
    } else if (scanSawOpen && front > CONFIRM_DIST) {
      openEndAngle = scanAngle;
      if (canStep) {
        scanAngle = scanAngle + scanDir * SCAN_STEP;
      } else {
        finishOpening(false, y);
      }
    } else if (scanSawOpen && openEndAngle != openAngle) {
      // Blocked again: we have both edges of the opening.
      finishOpening(true, y);
    } else if (canStep) {
      // Nothing yet (or the opening was too narrow): keep turning.
      scanSawOpen = false;
      scanAngle = scanAngle + scanDir * SCAN_STEP;
    } else if (scanSidesTried < 2) {
      // Nothing on this side; try the other side.
      scanDir = -scanDir;
      scanSidesTried = 2;
      scanAngle = scanDir * SCAN_STEP;
      scanSawOpen = false;
    } else {
      blindSlide(y);
    }
  } else if (state == ST_SIDESTEP) {
    // Slide sideways (turn to face +y or -y, drive) to line up with the
    // opening, then face forward and keep driving.
    float dy = sideTargetY - y;
    // Accept a looser line-up after a few tries so we don't wobble.
    float tol = SIDE_TOL;
    if (sideMoves >= 3) tol = Y_TOL;
    if (fabs(dy) < tol) {
      holdY = y;
      setState(ST_FACE_FORWARD);
    } else {
      float facing = PI / 2;
      if (dy < 0) facing = -PI / 2;
      if (turnTo(facing, x, y, theta)) {
        float step = fabs(dy);
        if (step > MAX_STEP) step = MAX_STEP;
        driveForward(step, x, y, theta);
        sideMoves++;
      }
    }
  } else if (state == ST_TO_LIMBO_Y) {
    // Past the obstacles: side-step onto the limbo center line.
    float dy = LIMBO_Y - y;
    if (fabs(dy) < Y_TOL) {
      setState(ST_FACE_LIMBO);
    } else {
      float facing = PI / 2;
      if (dy < 0) facing = -PI / 2;
      if (turnTo(facing, x, y, theta)) {
        float step = fabs(dy);
        if (step > MAX_STEP) step = MAX_STEP;
        driveForward(step, x, y, theta);
      }
    }
  } else if (state == ST_FACE_LIMBO) {
    if (turnTo(0, x, y, theta)) {
      setState(ST_UNDER_LIMBO);
    }
  } else if (state == ST_UNDER_LIMBO) {
    // Straight under the limbo, holding y on its center line.
    float remaining = GOAL_X - x;
    if (remaining < 0.06) {
      setState(ST_DONE);
    } else {
      float desired = atan2(LIMBO_Y - y, 0.4);
      if (turnTo(desired, x, y, theta)) {
        float step = MAX_STEP;
        if (remaining < step) step = remaining;
        driveForward(step, x, y, theta);
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
