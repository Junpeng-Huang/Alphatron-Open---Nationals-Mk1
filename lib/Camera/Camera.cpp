#include "Camera.h"

Camera::Camera() {}

void Camera::init() {
    Serial7.begin(CAMERA_BAUD);
}

void Camera::update(bool attackBlue) {
    if(Serial7.available() >= 8) {
        uint8_t firstByte = Serial7.read();
        uint8_t secondByte = Serial7.peek();
        
        if(firstByte == 255 && secondByte == 255) {
            Serial7.read();
            int yellowX = Serial7.read();
            int yellowY = Serial7.read();
            yellowAngle = floatMod((RAD2DEG * atan2(240-yellowY-120, 320-yellowX-160)), 360);
            if(yellowAngle < 0){
                yellowAngle += 360;
            }
            yellowDist = sqrt(((320-yellowX-160)*(320-yellowX-160))+((240-yellowY-120)*(240-yellowY-120)));
            int blueX = Serial7.read();
            int blueY = Serial7.read();
            blueAngle = RAD2DEG * atan2(240-blueY-120, 320-blueX-160);
            if(blueAngle < 0){
                blueAngle += 360;
            }
            blueDist = sqrt(((320-blueX-160)*(320-blueX-160))+((240-blueY-120)*(240-blueY-120)));
            int ballX = Serial7.read();
            int ballY = Serial7.read();
            ballDir = RAD2DEG * atan2(240-ballY-120, 320-ballX-160);

            if(ballDir > 0){
                ballDir -= 360;
            }
            ballDir *= -1;
            ballDist = sqrt(((320-ballX-160)*(320-ballX-160))+((240-ballY-120)*(240-ballY-120)));

            
            ballDirCW = RAD2DEG * atan2(240-ballY-120, 320-ballX-160);
            if (ballDirCW < 0){
                ballDirCW += 360;
            }

            attackVis = (attackBlue ? (blueDist != 0 ? true : false) : (yellowDist != 0 ? true : false));
            defendVis = (attackBlue ? (yellowDist != 0 ? true : false) : (blueDist != 0 ? true : false));
            ballVisible = (ballDist != 0 ? true : false);
            
            if(yellowAngle != 225){
                prevAngY = yellowAngle;
            } else{
                yellowAngle = prevAngY;
            }
            if(yellowDist < 200){
                prevDistY = yellowDist;
            } else{
                yellowDist = prevDistY;
            }

            if(blueAngle != 225){
                prevAngB = blueAngle;
            } else{
                blueAngle = prevAngB;
            }
            if(blueDist < 200){
                prevDistB = blueDist;
            } else{
                blueDist = prevDistB;
            }
            
            if(ballDir != 225){
                prevAngBall = ballDir;
            } else{
                ballDir = prevAngBall;
            }
            if(ballDist < 200){
                prevDistBall = ballDist;
            } else{
                ballDist = prevDistBall;
            }

            attackAngle = (attackBlue ? blueAngle : yellowAngle);
            attackDist = (attackBlue ? blueDist : yellowDist);
            defendAngle = (attackBlue ? yellowAngle : blueAngle);
            defendDist = (attackBlue ? yellowDist : blueDist);

            if (ballDist != 0){
                ballStr = (10000/ballDist)-100;
                if (ballStr < 1) {
                    ballStr = 1;
                }
            } else {
                ballStr = 0;
            }
        }
    }
}

float Camera::calculateAngleAddition()
{
	float dir = ballDir > 180 ? ballDir - 360 : ballDir;
	float ballAngleDifference = findSign(dir) * fmin(90, 0.01 * expf(0.25 * smallestAngleBetween(ballDir, 0)));
	float strengthFactor = constrain(ballStr / BALL_CLOSE_STRENGTH, 0, 1);
	float distanceMultiplier = constrain((0.012 * expf(4.5 * strengthFactor)), 0, 1);
	angleAddition = ballAngleDifference * distanceMultiplier;
	return angleAddition;
}

float Camera::ballPixeltoCM(float dist){
    return (dist != 0) ? 0.0195649 * expf(0.0634054 * (dist + 30.1281)) + 21.2566 : 0;
}

float Camera::goalPixeltoCM(float dist){
    return (dist != 0) ? 5.7478f * powf(10.0f, -13.0f) * expf(0.0494379*(dist + 552.825f)) + 13.8327f : 0;
}