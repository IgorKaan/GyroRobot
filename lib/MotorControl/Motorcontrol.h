#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include <AX12A.h> 

class MotorControl
{
    public:
        void setupMotor();

        void driveMotor();

        void goForward(short motorSpeed);

        void goBackward(short motorSpeed);

        void turnLeft(short motorSpeed);

        void turnRight(short motorSpeed);

        void stopMovement();

        void enableCentralMotor();

        void leftMotorDirection(short leftMotorDirectionValue, short leftMotorSpeed);

        void rightMotorDirection(short rightMotorDirectionValue, short rightMotorSpeed);

};

#endif