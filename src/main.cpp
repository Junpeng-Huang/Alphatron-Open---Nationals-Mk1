#include <Arduino.h>
#include <math.h>
#include "Movement.h"
#include "pins.h"
#include "defines.h"
#include "Adafruit_BNO055.h"
#include "PID.h"
#include "camera.h"
#include "orbit.h"
#include "LightSensor.h"
#include "lineavoidance.h"
#include "Camera.h"
#include "time.h"
#include "kicker.h"
#include "Bluetooth.h"
#include <Wire.h>

using namespace bon;
sensors_event_t event;
Movement motors;
Orbit orbit;
LightSensor lightsensor;
oAvoidance outAvoidance;
Camera camera;
Kicker kicker;
Bluetooth bluetooth;
Timer surgeTimer = Timer((unsigned long)SURGE_MAX_TIME);
Timer visTimer = Timer((unsigned long)VIS_TIMER);

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);
PID pid = PID(COMPASS_P, COMPASS_I, COMPASS_D);
PID cameraPid = PID(CAMERA_P, CAMERA_I, CAMERA_D);
PID sidewayPid = PID(SIDEWAY_P, SIDEWAY_I, SIDEWAY_D);
PID forwardPid = PID(forward_P, forward_I, forward_D);
PID centrePid = PID(centre_P, centre_I, centre_D);
PID defendPid = PID(defend_P, defend_I, defend_D);

struct Move
{
	int speed;
	int dir;
	int rot;
};

int counter = 0;
int bnoCtr = 0;
bool surge = false;
int prevDir;
int prevSpeed;

int compass_correct(float targetHeading = 0)
{
	sensors_event_t event;
	bno.getEvent(&event);
	float orient = (float)event.orientation.x;
	if (orient > 180)
	{
		orient = orient - 360;
	}
	if (targetHeading > 180)
	{
		targetHeading = targetHeading - 360;
	}

	return pid.update(orient, 1.25 * targetHeading);
}

int sideways_correct(float orient = camera.ballDir, float targetHeading = 0)
{
	if (orient > 180)
	{
		orient -= 360;
	}
	orient *= -1;
	return sidewayPid.update(orient, targetHeading);
}

float centre_correct(float targetHeading = float(event.orientation.x))
{
	float orient = camera.defendDist*sin(DEG2RAD*camera.defendAngle);
	// Serial.println(orient);
	if (targetHeading > 180)
	{
		targetHeading = targetHeading - 360;
	}
	targetHeading = targetHeading*-1;
	return centrePid.update(orient, targetHeading);
}

int forwards_correct()
{
	float targetDist = DEFEND_DIST; //Maybe turn down the P...

	int currentDist = camera.defendDist;

	return forwardPid.update(currentDist, targetDist);
}

int defend_correct(float targetHeading = 180)
{
	float orient = camera.defendAngle;

	return defendPid.update(orient, targetHeading);
}

int camera_correct(float targetHeading = 0)
{
	float orient = camera.attackAngle;
	if (orient > 180)
	{
		orient = orient - 360;
	}
	if (targetHeading > 180)
	{
		targetHeading = targetHeading - 360;
	}

	return cameraPid.update(orient, targetHeading);
}

Move attack(double ballDir, double ballStr, bool ballVis, double outDir, double outSpd, double lineAngle){

	Move movement;

	if (lineAngle != -1) {
		 if (outAvoidance.botlocation >= 0 && camera.ballDirCW > lineAngle-90 && camera.ballDirCW < lineAngle+90 && camera.ballVisible){
			if ((lineAngle > 160 && lineAngle < 200) || (lineAngle > 330 || lineAngle < 30)){
				movement.speed = outSpd;
				movement.dir = outDir;
			} else {
				Orbit::OrbitData orbitData = orbit.update(camera.ballDir, camera.ballStr);
				movement.speed = orbitData.speed;
				movement.dir = -1*floatMod(camera.ballDir+camera.calculateAngleAddition(), 360);
				movement.rot = camera_correct();
				// if (camera.ballDir < 2 && camera.ballDir > -2 && camera.ballStr > KICK_BALL_STR && (camera.attackAngle < 2 || camera.attackAngle > 358)){
				// 	kicker.shouldKick = true;
				// 	kicker.kickDelay.resetTime();
				// 	// Serial.println("Kicking");
				// }
			}
		} else{
			movement.speed = outSpd;
			movement.dir = outDir;
		}
	} else {
		if (camera.ballVisible && outAvoidance.botlocation != -1) {
				Orbit::OrbitData orbitData = orbit.update(camera.ballDir, camera.ballStr);
				movement.speed = orbitData.speed;
				movement.dir = floatMod(camera.ballDir +camera.calculateAngleAddition(), 360);
				// if (camera.ballDir < 2 && camera.ballDir > -2 && camera.ballStr > KICK_BALL_STR && (camera.attackAngle < 2 || camera.attackAngle > 358)){
				// 	kicker.shouldKick = true;
				// 	kicker.kickDelay.resetTime();
				// 	// Serial.println("Kicking");
				// }
		} else{
			movement.speed = outSpd;
			movement.dir = outDir;
		}
	}
	return movement;
}

Move defend(double ballDir, double ballStr, bool ballDist, double outDir, double outSpd, bool defendVis, int defendDist, double lineAngle, double heading){
	Move move;
	bnoCtr++;
	if(bnoCtr % 5 == 0) {
		bno.getEvent(&event);
	}

	if (ballStr >= DEFENSE_SURGE_STRENGTH && (camera.ballDir < 10 || camera.ballDir > 350) && defendDist < DEFEND_DIST+2){
		surge = true;
		surgeTimer.resetTime();	
	} else if (surge && defendDist > DEFEND_DIST+40){
		surge = false;
	} else if (surgeTimer.timeHasPassedNoUpdate()){
		surge = false;
	}

	int fwdc = forwards_correct();
	int swdc = sideways_correct();
	int cdc = centre_correct();

	if(defendDist < DEFEND_DIST+10){ //Calibration 
		if(outDir == -1 && outAvoidance.botlocation != -1){
			if(ballDist != 0 && defendVis){
				if (surge){
					move.speed = DEFENSE_SUGRE_SPEED;
					move.dir = ballDir;
					move.rot = defend_correct();
					Serial.println("Surging");
				} else {
					// Serial.println(camera.defendAngle-heading);
					// Serial.println(ballDir);
					if(camera.defendAngle-heading > 225 && ballDir > 180 && ballDir < 230) { //CHECK
						move.dir = 270;
						move.speed = ORBIT_FAR_SPEED;
						move.rot = defend_correct();
						Serial.println("Moving Left");
					} else if (camera.defendAngle-heading < 135 && ballDir > 130 && ballDir < 180 ){ //CHECK
						move.dir = 90;
						move.speed = ORBIT_FAR_SPEED;
						move.rot = defend_correct();
						Serial.println("Moving Right");
					} else {
						move.speed = sqrt(((fwdc)*(fwdc))+((swdc)*(swdc)));
						move.dir = floatMod(RAD_TO_DEG * atan2(swdc, fwdc), 360);
						move.rot = defend_correct();
					}
				}
			} else{
				if (defendVis){
					move.speed = sqrt(((fwdc) * (fwdc)) + ((cdc) * (cdc)));
					move.dir = floatMod(RAD_TO_DEG * atan2(cdc, fwdc), 360);
					move.rot = defend_correct();
				} 
			}
		} else{
				move.dir = outDir;
				move.speed = outSpd;
				move.rot = defend_correct();
		}
    } else if (camera.ballVisible && ballDir > 90 && ballDir < 270){
        move.speed = ORBIT_FAR_SPEED;
        move.dir = floatMod(camera.ballDir +camera.calculateAngleAddition(), 360);
        move.rot = compass_correct();
	} else {
		move.speed = sqrt(((fwdc)*(fwdc))+((swdc)*(swdc)));
		move.dir = floatMod(RAD_TO_DEG * atan2(swdc, fwdc), 360);
		move.rot = defend_correct();
	}
	return move;
}


void setup()
{
	Serial.begin(9600);
	camera.init();
	delay(500);
 	lightsensor.init();
	bluetooth.init();
	Wire.begin();
	bno.begin();
	kicker.init();
	if (!bno.begin(bno.OPERATION_MODE_IMUPLUS)) {
		Serial.println("Error connecting to bno");
		while(1);
	}
	bno.setExtCrystalUse(true);
	delay(500);
	Serial.println("Done");
}

void loop()
{
	bnoCtr++;
	if(bnoCtr % 5 == 0) {
		bno.getEvent(&event);
	}
	float ol = lightsensor.update();
	float orient = -1 * ((float)event.orientation.x) + 360;
	if (orient > 180){
		orient = orient -360;
	}
  	float lineAngle = (ol != -1 ? floatMod(ol+orient, 360) : -1.00);
  	oAvoidance::Movement outavoidance = outAvoidance.moveDirection(lineAngle);
  	outavoidance.direction = (outavoidance.direction != -1 ? -1* (floatMod(outavoidance.direction-orient, 360)) + 360 : -1.00);
	camera.update(blueAttack == false);
	// bluetooth.update(camera.ballStr);

	if (ROBOT == 2){	// -- Attacking -- 
		
		// Serial.println("Attacking");
		Move att = attack(camera.ballDir, camera.ballStr, camera.ballVisible, outavoidance.direction, outavoidance.speed, lineAngle);
		if (camera.ballDist == 0 && lineAngle == -1 && outAvoidance.botlocation != -1) {
			// motors.move(prevSpeed-100, prevDir, camera_correct());
			if (!camera.defendVis){
				motors.move(HOMING_SPEED, 180, compass_correct());
			}
			else if (camera.defendDist > DEFEND_DIST+25){
				motors.move(HOMING_SPEED, 180, defend_correct());
			} else {
				motors.move(0, 0, defend_correct());
			}
		} else if (camera.attackVis) {
			motors.move(att.speed, att.dir, camera_correct());
			prevSpeed = att.speed;
			prevDir = att.dir;
		} else {
			motors.move(att.speed, att.dir, compass_correct());
			prevSpeed = att.speed;
			prevDir = att.dir;
		} 

	} else {	// -- Defending --
		
		// Serial.println("Defending");
		if (camera.defendDist != 0) {
			Move def = defend(camera.ballDir, camera.ballStr, camera.ballDist, outavoidance.direction, outavoidance.speed, camera.defendVis, camera.defendDist, lineAngle, orient);
			motors.move(def.speed, def.dir, def.rot);
			// Serial.println(def.dir);
		} 
		else if (camera.ballVisible && lineAngle == -1 && outAvoidance.botlocation != -1) {
			Orbit::OrbitData orbitData = orbit.update(camera.ballDir, camera.ballStr);
			motors.move(orbitData.speed-100, constrain(floatMod(camera.ballDir+camera.calculateAngleAddition(), 360), 90, 270), camera_correct());
			// Serial.println("Orbiting");
		} else {
			motors.move(outavoidance.speed, outavoidance.direction,compass_correct());
		}
	}

	// kicker.update();
	
	// if (lineAngle != -1 || outAvoidance.botlocation == -1){
	// 	motors.move(outavoidance.speed, outavoidance.direction, compass_correct());
	// } else {
	// 	Orbit::OrbitData orbitData = orbit.update(camera.ballDir, camera.ballStr);
	// 	motors.move(orbitData.speed, floatMod(camera.ballDir +camera.calculateAngleAddition(), 360), camera_correct());
	// }

	//- Testing -//

	// Serial.println(digitalRead(SUPERTEAM_PIN));
	// bno.getEvent(&event);
	// Serial.println(event.orientation.x);
	// motors.move(0,0,defend_correct());
	// if (camera.ballStrSurgeAvg > 180){
	// Serial.println(camera.ballDir);
	// }
	// if (camera.ballStrAvg > 160){
	// 	Serial.println(camera.ballStrAvg);
	// }
	// Serial.println(camera.ballDir);
	// if (camera.attackAngle != 0){
	// }
	// Serial.println(event.orientation.x-camera.defendAngle);
	// Serial.println(camera.defendDist); //, camera.ballStrAvg, camera.ballVisible, outavoidance.direction, outavoidance.speed, camera.defendVis, camera.defendDist, lineAngle, orient

	// lightsensor.test();
	// Serial.println(outAvoidance.botlocation);
	// Serial.println(lineAngle);
	// motors.move(outavoidance.speed, outavoidance.direction,compass_correct());
	// motors.move(0, 0, defend_correct());

	//- Kicker -//

	// digitalWrite(KICKER_PIN,HIGH);
	// delay(5000);
	// digitalWrite(KICKER_PIN,LOW);
	// Serial.println("LOWWWWWW");
	// delay(25);
	// digitalWrite(KICKER_PIN,HIGH);
	// delay(5000);
}