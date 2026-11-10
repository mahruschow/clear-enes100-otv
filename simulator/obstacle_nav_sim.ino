// CLEAR obstacle navigation -- web simulator version.
// Paste this whole file into https://enes100.umd.edu/tools/simulator/
//
// RULE #1: THE TANK MUST NEVER TOUCH ANY BLOCK -- SOLID OR STRIPED.
//
// Course layout: 4 m x 2 m. Three blocks sit in two columns (x = 1.4-1.6
// and x = 2.2-2.4) and three lanes (centered at y = 0.5, 1.0, 1.5), exactly
// one block per lane. Two are solid, one is striped. The front sensor sees
// solid blocks but NOT striped ones, so we work out where the striped one is.
//
// Logic:
//   1. Drive forward to just in front of column 1, then point the sensor
//      down each lane: ~0.1 m = block in column 1, ~0.9 m = block in
//      column 2, nothing = the striped block's lane.
//   2. Every block cell (and both possible cells for the striped block when
//      we can't tell its column) becomes a KEEP-OUT zone. Every drive and
//      slide checks the keep-out zones continuously and stops before
//      entering one.
//   3. Pick the lane through each column that has no block there, drive
//      forward through it (still watching the sensor), and if anything
//      unexpected shows up, turn to scan for an opening -- but only slide
//      to lanes outside the keep-out zones.
//   4. Once past the obstacles, line up with the limbo and drive under it.
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

// Lanes and columns.
const float LANE_Y0 = 0.5;        // lane i is centered at LANE_Y0 + 0.5 * i
const float LANE_SPACING = 0.5;
const float COL_X0 = 1.4;         // column c spans COL_X0 + 0.8*c to +0.2
const float COL_SPACING = 0.8;
const float COL_WIDTH = 0.2;
const float KEEP_OUT_PAD = 0.11;  // robot half-diagonal (0.096) + margin

// Where the tank stops (robot center x) at each stage.
const float SURVEY_X = 1.2;   // in front of column 1 (face at 1.4)
const float GAP_X = 1.9;      // between column 1 (ends 1.6) and column 2 (starts 2.2)
const float CLEAR_X = 2.7;    // past column 2 (ends 2.4)
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
const float DETECT_DIST = 0.20;  // unexpected object this close ahead -> stop and look for an opening
const float COL1_MAX = 0.5;      // survey: closer than this = block in column 1
const float OPEN_DIST = 1.0;     // sensor sees nothing within this -> opening
const float CONFIRM_DIST = 0.6;  // still part of the opening while farther than this
const float LOOK_AHEAD = 0.06;   // keep-out check looks this far ahead of the center

// Scanning.
const float SCAN_MAX = 1.1;      // farthest we turn from forward (~63 degrees)

// Tolerances.
const float HEADING_TOL = 0.04;  // radians (~2 degrees)
const float POS_TOL = 0.02;      // meters

// Motor power (simulator values; tune for the real tank).
const int DRIVE_PWM = 255;
const int SLOW_PWM = 70;         // last part of a move
const int TURN_PWM = 180;
const int FINE_TURN_PWM = 45;    // last few degrees of a turn
const int SCAN_PWM = 110;        // sweep while looking for an opening
const float STEER_GAIN = 120;    // PWM difference per radian of heading error
// Full speed is about 0.5 m/s and 1.4 rad/s in the simulator. How early to
// slow down depends on how long one check takes, measured as we go.
const float FAST_M_PER_SEC = 0.5;
const float FAST_RAD_PER_SEC = 1.4;

// Results from driveForward().
#define REACHED 0
#define BLOCKED 1
#define STUCK 2
#define KEEP_OUT 3

// What the survey found in each lane.
#define UNKNOWN 0
#define IN_COL1 1
#define IN_COL2 2
#define STRIPED 3

// keepOut[c][i]: the cell in column c, lane i holds (or may hold) a block.
bool keepOut[2][3] = {{false, false, false}, {false, false, false}};
int laneBlock[3] = {UNKNOWN, UNKNOWN, UNKNOWN};
bool surveyed = false;

bool done = false;
float holdY = 0;           // y line to hold while driving forward
int recoveries = 0;
const int MAX_RECOVERIES = 8;
float pollSec = 0.05;      // measured time for one position/sensor check
unsigned long lastPollMs = 0;

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

// ---- KEEP-OUT: never let the tank's center get within KEEP_OUT_PAD of a
// ---- block cell that holds, or may hold, a block.
bool inKeepOut(float x, float y) {
  for (int c = 0; c < 2; c++) {
    float left = COL_X0 + COL_SPACING * c - KEEP_OUT_PAD;
    float right = COL_X0 + COL_SPACING * c + COL_WIDTH + KEEP_OUT_PAD;
    if (x < left || x > right) continue;
    for (int i = 0; i < 3; i++) {
      if (!keepOut[c][i]) continue;
      float bottom = laneY(i) - LANE_SPACING / 2 - KEEP_OUT_PAD;
      float top = laneY(i) + LANE_SPACING / 2 + KEEP_OUT_PAD;
      if (y > bottom && y < top) return true;
    }
  }
  return false;
}

// Is the spot just ahead of the tank (in its current heading) off limits?
bool keepOutAhead(float x, float y, float theta, float ahead) {
  return inKeepOut(x + ahead * cos(theta), y + ahead * sin(theta));
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

// Keep pollSec up to date; call once per check inside a motion loop.
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

// Turn in place until facing `desired`. Slows down for the last few degrees.
void turnTo(float desired) {
  lastPollMs = 0;
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
}

// Drive forward (+x) holding the line y = holdY until x reaches stopX.
// If useSensor, stops early when something is within DETECT_DIST.
// ALWAYS stops before entering a keep-out zone.
int driveForward(float stopX, bool useSensor) {
  turnTo(0);
  lastPollMs = 0;
  unsigned long lastProgress = millis();
  float lastX = Enes100.getX();
  while (true) {
    float x = Enes100.getX();
    float y = Enes100.getY();
    float theta = Enes100.getTheta();
    if (x >= stopX - POS_TOL) {
      stopMotors();
      return REACHED;
    }
    if (keepOutAhead(x, y, theta, LOOK_AHEAD)) {
      stopMotors();
      return KEEP_OUT;  // next few cm would enter a block's area
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
    if (stopX - x < slowZone() || keepOutAhead(x, y, theta, LOOK_AHEAD + slowZone())) {
      pwm = SLOW_PWM;
    }
    float steer = STEER_GAIN * angleError(atan2(holdY - y, 0.4), theta);
    setMotors((int)(pwm - steer), (int)(pwm + steer));
    timePoll();
  }
}

// Slide sideways: face +y or -y and drive until y reaches targetY,
// holding the current x line. Stops before entering a keep-out zone.
// Returns false if it had to stop early.
bool moveToY(float targetY) {
  float dy = targetY - Enes100.getY();
  if (fabs(dy) < POS_TOL * 2) return true;
  float heading = PI / 2;
  float sign = 1;
  if (dy < 0) {
    heading = -PI / 2;
    sign = -1;
  }
  turnTo(heading);
  lastPollMs = 0;
  float holdX = Enes100.getX();
  unsigned long start = millis();
  bool ok = true;
  while (millis() - start < 6000) {
    float x = Enes100.getX();
    float y = Enes100.getY();
    float theta = Enes100.getTheta();
    float toGo = (targetY - y) * sign;
    if (toGo < POS_TOL) break;
    if (keepOutAhead(x, y, theta, LOOK_AHEAD)) {
      ok = false;
      break;
    }
    int pwm = DRIVE_PWM;
    if (toGo < slowZone()) pwm = SLOW_PWM;
    float lineErr = -sign * (holdX - x);
    float steer = STEER_GAIN * (angleError(heading, theta) + 2.5 * lineErr);
    setMotors((int)(pwm - steer), (int)(pwm + steer));
    timePoll();
  }
  stopMotors();
  return ok;
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

// ---- Survey (once, in front of column 1) ----

void surveyLane(int i) {
  moveToY(laneY(i));
  turnTo(0);
  float d = frontDistance();
  float d2 = frontDistance();  // read twice in case one reading is off
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

// Survey the lanes and fill in keepOut. Returns false if the readings
// don't fit the course layout.
bool survey() {
  float y0 = Enes100.getY();
  int first = 0;
  if (fabs(laneY(1) - y0) < fabs(laneY(first) - y0)) first = 1;
  if (fabs(laneY(2) - y0) < fabs(laneY(first) - y0)) first = 2;
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
  if (laneBlock[order[0]] != STRIPED && laneBlock[order[1]] != STRIPED) {
    // Two solid lanes found: the third must be the striped one.
    laneBlock[order[2]] = STRIPED;
    Enes100.print("Lane ");
    Enes100.print(order[2]);
    Enes100.println(" -> striped block lane (deduced)");
  } else {
    surveyLane(order[2]);
  }
  if (countLanes(STRIPED) != 1 || countLanes(UNKNOWN) != 0) return false;

  // The striped block is in column 2 if both solids are in column 1, in
  // column 1 if both solids are in column 2, otherwise it could be in either.
  bool stripedCol1 = countLanes(IN_COL1) != 2;
  bool stripedCol2 = countLanes(IN_COL2) != 2;
  for (int i = 0; i < 3; i++) {
    keepOut[0][i] = laneBlock[i] == IN_COL1 || (laneBlock[i] == STRIPED && stripedCol1);
    keepOut[1][i] = laneBlock[i] == IN_COL2 || (laneBlock[i] == STRIPED && stripedCol2);
  }
  return true;
}

// The lane through column c that is clear and closest to y, or -1.
int clearLane(int c, float y) {
  int best = -1;
  for (int i = 0; i < 3; i++) {
    if (keepOut[c][i]) continue;
    if (best < 0 || fabs(laneY(i) - y) < fabs(laneY(best) - y)) best = i;
  }
  return best;
}

// Which column is the tank in front of (0 or 1), or -1 if past both.
int columnAhead(float x) {
  if (x < COL_X0 + COL_WIDTH) return 0;
  if (x < COL_X0 + COL_SPACING + COL_WIDTH) return 1;
  return -1;
}

// ---- Opening scan (used if something unexpected shows up) ----

float openStart = 0;
float openEnd = 0;
bool bounded = false;
bool scanSide(int dir) {
  turnTo(0);
  lastPollMs = 0;
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

// Something is in the way: scan for an opening, then slide to the clear
// lane nearest to it. Only lanes outside the keep-out zones are allowed.
void getAroundObject() {
  float objectDist = frontDistance();
  float x = Enes100.getX();
  float y = Enes100.getY();
  Enes100.print("Object ahead at ");
  Enes100.println(objectDist);

  int c = columnAhead(x);
  if (c < 0) c = 1;
  int firstDir = 1;
  if (y > MID_Y + 0.1) firstDir = -1;

  float target = -1;
  for (int k = 0; k < 2 && target < 0; k++) {
    int dir = firstDir;
    if (k == 1) dir = -firstDir;
    if (scanSide(dir)) {
      float d = objectDist + HALF_LENGTH;
      float side;
      if (bounded) side = d * (tan(fabs(openStart)) + tan(fabs(openEnd))) / 2;
      else side = d * tan(fabs(openStart)) + 0.2;
      int lane = clearLane(c, y + dir * side);
      if (lane >= 0 && fabs(laneY(lane) - y) > 0.15) target = laneY(lane);
    }
  }
  if (target < 0) {
    // No usable opening seen: try the nearest clear lane we're not already in.
    recoveries++;
    for (int i = 0; i < 3; i++) {
      if (!keepOut[c][i] && fabs(laneY(i) - y) > 0.15 &&
          (target < 0 || fabs(laneY(i) - y) < fabs(target - y))) {
        target = laneY(i);
      }
    }
    if (target < 0) target = y + firstDir * 0.5;
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

  // 1. First time through: drive up to column 1 and survey the lanes.
  if (!surveyed) {
    int first = 0;
    float y0 = Enes100.getY();
    if (fabs(laneY(1) - y0) < fabs(laneY(first) - y0)) first = 1;
    if (fabs(laneY(2) - y0) < fabs(laneY(first) - y0)) first = 2;
    moveToY(laneY(first));
    holdY = laneY(first);
    driveForward(SURVEY_X, false);
    if (!survey()) {
      finish("Stopped: survey didn't match the course layout");
      return;
    }
    surveyed = true;
  }

  // 2. Get into the clear lane for the column ahead.
  float x = Enes100.getX();
  float y = Enes100.getY();
  int c = columnAhead(x);
  if (c >= 0) {
    int lane = clearLane(c, y);
    if (lane < 0) {
      finish("Stopped: no clear lane");
      return;
    }
    if (fabs(laneY(lane) - y) > 0.05) {
      Enes100.print("Column ");
      Enes100.print(c + 1);
      Enes100.print(": using lane ");
      Enes100.println(lane);
      if (!moveToY(laneY(lane))) {
        finish("Stopped: slide would enter a block");
        return;
      }
    }
    holdY = laneY(lane);

    // 3. Drive forward through this column (to the gap, or past column 2).
    float stopX = GAP_X;
    if (c == 1) stopX = CLEAR_X;
    int result = driveForward(stopX, true);
    if (result == BLOCKED) {
      getAroundObject();
    } else if (result == STUCK || result == KEEP_OUT) {
      recoveries++;
      Enes100.println("Too close to a block, backing up");
      backUp(0.15);
      getAroundObject();
    }
    if (recoveries > MAX_RECOVERIES) {
      finish("Stopped: no way through found");
    }
    return;
  }

  // 4. Past the obstacles: line up with the limbo and drive under it.
  Enes100.println("Past obstacles, lining up with the limbo");
  moveToY(LIMBO_Y);
  holdY = LIMBO_Y;
  if (driveForward(GOAL_X, false) == REACHED) {
    finish("Stopped: reached goal (under the limbo)");
  } else {
    finish("Stopped: could not get under the limbo");
  }
}
