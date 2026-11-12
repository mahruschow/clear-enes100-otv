// CLEAR: UMD Mega practice tank, battery-start field route.
#include <Arduino.h>
#include <math.h>
#include "Tank.h"
#include "Enes100.h"

const int MARKER_ID=10, ROOM_NUMBER=1116; // EDIT to match tank and arena.
const float POOL_X=.55, INTAKE_REACH=.20; // practice docking targets, in meters
const float SURVEY=1.19, GAP=1.90, CLEAR_X=2.72;
const float LANE[]={.50,1.00,1.50};
const int POWER=180; // both wheels worked at this PWM in the bench test
// First upload: direction check only. Enable route only after a PASS.
const bool CHECK_ONLY=true;
bool done=false;
float px,py,th;
bool pose(){
  if(!Enes100.isVisible()) return false;
  px=Enes100.getX(); py=Enes100.getY(); th=Enes100.getTheta();
  return px>=0 && px<=4 && py>=0 && py<=2 && fabs(th)<=3.15;
}
bool fail(const char* why){
  Tank.turnOffMotors(); Enes100.println(why); return false;
}
bool recoverPose(){
  return fail("STOP: lost pose; reset in start square");
}
bool inside(){return px>.25 && px<3.78 && py>.25 && py<1.75;}
bool clearFront(){
  float r=Tank.readDistanceSensor(1);
  return isfinite(r) && r>=.15;
}
// Stop BEFORE asking Wi-Fi for another reading. No motor is left running
// if a vision request stalls. Each step is corrected from the measured pose.
void pulse(int left,int right,int ms){
  Tank.setLeftMotorPWM(left); Tank.setRightMotorPWM(right);
  delay(ms); Tank.turnOffMotors(); delay(100);
}
float wrap(float a){return atan2(sin(a),cos(a));}
bool steer(float x,float y,bool rotateOnly,float angle){
  unsigned long start=millis();
  float best=100; int stale=0,mode=-1;
  while(millis()-start<45000UL){
    if(!pose()){
      unsigned long paused=millis();
      if(!recoverPose()) return false;
      start+=millis()-paused;
      mode=-1;
    }
    if(!inside()) return fail("STOP: marker too near arena edge");
    float dx=x-px,dy=y-py,d=sqrt(dx*dx+dy*dy);
    if(!rotateOnly && d<.04) return true;
    float e=wrap((rotateOnly?angle:atan2(dy,dx))-th);
    int turning=fabs(e)>(rotateOnly?.10:.12);
    if(rotateOnly && !turning) return true;
    float err=turning?fabs(e):d;
    if(mode!=turning){mode=turning;best=100;stale=0;}
    if(err<best-(turning?.025:.008)){best=err;stale=0;}
    else if(++stale>=10) return fail("STOP: no progress; check marker heading and traction");
    if(turning){
      int p=e>0?POWER:-POWER;
      pulse(-p,p,fabs(e)>.4?110:55);
    }else{
      if(!clearFront()) return fail("STOP: front range too close or invalid");
      pulse(POWER,POWER,d>.22?140:65);
    }
  }
  return fail("STOP: waypoint timeout");
}
bool directionCheck(){
  if(!pose() || px<0.20f || px>0.70f || fabs(py-1)>.65)
    return fail("CHECK: start X must be 0.20-0.70 m; check starting Y");
  float x0=px,y0=py,h0=th;
  if(fabs(h0)>.25) return fail("CHECK: aim sensor toward +X; marker theta must be near zero");
  if(!clearFront()) return fail("CHECK: front range too close or invalid");
  for(int n=0;n<5;n++){
  if(!clearFront()) return fail("CHECK: front blocked");
  pulse(POWER,POWER,180);
  delay(400);
  if(!pose() || px<0.20f || px>1.0 || py<=.25 || py>=1.75) return fail("CHECK: lost pose or edge after forward pulse");
  float dx=px-x0,dy=py-y0;
  Enes100.print("Forward dx m: ");Enes100.println(dx);
  if(dx<.012 || fabs(dy)>dx || fabs(wrap(th-h0))>.22)
    return fail("CHECK: forward motion unconfirmed; inspect tracks and marker alignment");
  if(inside()) break; // Move inward before turning; keep route wall margin.
  x0=px;y0=py;
  }
  if(!inside()) return fail("CHECK: insufficient clearance to turn");
  x0=px;y0=py;h0=th;
  pulse(-POWER,POWER,150);
  delay(400);
  if(!pose() || !inside()) return fail("CHECK: lost pose or edge after turn");
  float dh=wrap(th-h0);
  Enes100.print("Left turn radians: ");Enes100.println(dh);
  if(dh<.04 || dh>.9 || hypot(px-x0,py-y0)>.06) return fail("CHECK: left turn unconfirmed");
  Enes100.println("CHECK PASS: forward and left verified");
  return true;
}
bool go(float x,float y){return steer(x,y,false,0);}
bool face(float a){return steer(0,0,true,a);}
// From the survey line, measured distances should cluster around 0.1 m
// and 0.9 m. Uncertain readings do not authorize a crossing.
int scan(){
  int nearVotes=0,farVotes=0;
  for(int i=0;i<5;i++){
    float r=Tank.readDistanceSensor(1);
    if(r>.015 && r<.38) nearVotes++;
    if(r>.62 && r<.99) farVotes++;
    delay(65);
  }
  return nearVotes>=4?1:(farVotes>=4?2:0);
}
bool route(){
  if(!pose() && !recoverPose()) return false;
  if(px>1 || fabs(py-1)<.15) return fail("STOP: place tank in a starting square and reset");
  bool upper=py>1;
  float dock=upper?.50+INTAKE_REACH:1.50-INTAKE_REACH;
  Enes100.println("Going to opposite mission site");
  if(!go(POOL_X,dock)||!face(upper?-PI/2:PI/2)) return false;
  Enes100.println("MISSION STOP: motors off for 3 s");
  delay(3000); // Practice tank only; no pump or arm.
  if(!go(POOL_X,1)) return false;
  int first=-1,second=-1;
  for(int i=0;i<3;i++){
    // Change lanes well back from column 1, then approach the survey line.
    if(!go(.95,LANE[i]) || !go(SURVEY,LANE[i]) || !face(0)) return false;
    int col=scan();
    Enes100.print("Lane "); Enes100.print(i+1); Enes100.print(" column (0=unknown): "); Enes100.println(col);
    if(col==2) first=i;  // Observed block in second column: pass first here.
    if(col==1) second=i;
    if(first>=0 && second>=0) break;
  }
  if(first<0 || second<0) return fail("STOP: could not confirm a lane for each crossing");
  if(!go(.95,LANE[first]) || !go(GAP,LANE[first]) || !go(GAP,LANE[second]) ||
     !go(CLEAR_X,LANE[second]) || !go(CLEAR_X,1.50) || !go(3.70,1.50)) return false;
  Tank.turnOffMotors(); Enes100.println("FINISHED: past limbo"); return true;
}
void setup(){
  Tank.begin(); Tank.turnOffMotors();
  pinMode(LED_BUILTIN,OUTPUT); digitalWrite(LED_BUILTIN,HIGH);
  delay(5000);
  Enes100.begin("CLEAR",WATER,MARKER_ID,ROOM_NUMBER,52,50);
  digitalWrite(LED_BUILTIN,LOW);
  Enes100.println("CLEAR FIELD: connected; waiting for tank marker");
}
void loop(){
  if(done) {delay(100);return;}
  if(!pose()) {delay(250);return;} // Wait for camera at startup; USB not required.
  done=true;
  if(directionCheck() && !CHECK_ONLY) route();
  Tank.turnOffMotors();
}
