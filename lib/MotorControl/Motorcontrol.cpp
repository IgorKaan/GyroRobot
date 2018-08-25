#include "MotorControl.h"
#include <Arduino.h>
#include <AX12A.h> 

#define Direction_pin (10u) 
#define Baud_rate (1000000ul) 
#define ID_1 (1u) 
#define ID_2 (2u) 
#define ID_3 (3u)

//void MotorControl::MotorControl(){
//
//}


void MotorControl::enableCentralMotor()
{
    ax12a.torqueStatus(ID_2, ON);
}

void MotorControl::setupMotor() 
{
    ax12a.begin(Baud_rate, Direction_pin, &Serial); 
    ax12a.setEndless(ID_1, ON); 
    ax12a.setEndless(ID_3, ON); 
    ax12a.torqueStatus(ID_2, ON);
}


void MotorControl::rightMotorDirection(short rightMotorDirectionValue, short rightMotorSpeed)
{
    if (rightMotorDirectionValue >= 0) {
        ax12a.turn(ID_3, LEFT, rightMotorSpeed);
    }
    else {
        ax12a.turn(ID_3, RIGHT, rightMotorSpeed);
    }
}

void MotorControl::leftMotorDirection(short leftMotorDirectionValue, short leftMotorSpeed)
{
    if (leftMotorDirectionValue >= 0) {
        ax12a.turn(ID_1, RIGHT, leftMotorSpeed);
    }
    else {
        ax12a.turn(ID_1, LEFT, leftMotorSpeed);
    }
}
void MotorControl::goForward(short motorSpeed)
{
    ax12a.turn(ID_3, LEFT, motorSpeed); 
    ax12a.turn(ID_1, RIGHT, motorSpeed);
}

void MotorControl::goBackward(short motorSpeed)
{
    ax12a.turn(ID_3, RIGHT, motorSpeed); 
    ax12a.turn(ID_1, LEFT, motorSpeed);
}

void MotorControl::turnLeft(short motorSpeed)
{
    ax12a.turn(ID_3, RIGHT, motorSpeed); 
    ax12a.turn(ID_1, RIGHT, motorSpeed);
}

void MotorControl::turnRight(short motorSpeed)
{
    ax12a.turn(ID_3, LEFT, motorSpeed); 
    ax12a.turn(ID_1, LEFT, motorSpeed);
}

void MotorControl::stopMovement()
{
    ax12a.turn(ID_3, RIGHT, 0); 
    ax12a.turn(ID_1, RIGHT, 0);
}

void MotorControl::driveMotor()
{

}