#include "motor.h"

Motor::Motor(int pwm, int inDir, bool rev){
    pwmPin = pwm;
    inDirPin = inDir;

    reversed = rev;

    pinMode(pwmPin, OUTPUT);
    pinMode(inDirPin, OUTPUT);
}

void Motor::move(int speed) {
    if(speed > 0){
        digitalWriteFast(inDirPin, HIGH);
    } else{
        digitalWriteFast(inDirPin, LOW);
    }
    analogWrite(pwmPin, abs(speed));
}

void Motor::brake(){
    // analogWrite(pwmPin, 0);
    digitalWriteFast(inDirPin, HIGH);
}   