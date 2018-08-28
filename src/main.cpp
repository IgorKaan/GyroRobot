#include <Arduino.h>
#include "Wire.h"
#include <AX12A.h> 
#include <TroykaIMU.h>
#include "math.h"
#include <I2Cdev.h>
#include <MotorControl.h>
#include <MqttClient.h>
#include <PubSubClient.h>
#include <stdio.h>

#define Kp  25  /*Too small Kp will cause the robot to fall, because the fix is ​​not enough. 
Too much Kp forces the robot to go wildly forward and backward.
A good Kp will make the robot move very little back and forth (or slightly oscillates).*/
#define Kd  0.02 /*A good Kd value will reduce the vibrations until the robot becomes almost steady. 
In addition, the correct Kd will hold the robot, even if it is pushed. */
#define Ki  5  /*The correct Ki value will shorten the time required to stabilize the robot.*/
#define sample_time  0.0005 
#define target_angle -1.5 
#define BETA 0.22f

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

Madgwick filter;
Accelerometer accel;
Gyroscope gyro;
MotorControl GyroRobot;

const char* ssid = "SPEECH_405";             
const char* password = "multimodal";         
const char* mqtt_server = "192.168.0.128";

mqttClient mqtt(ssid, password, mqtt_server);

float gx, gy, gz, ax, ay, az, roll;
float fps = 100;
short motorSpeed = 100;
short correctMotorSpeed = 200;
short motorDirectionValue = 0;
short xErrorValue;
short yErrorValue;

short correctValue = 0;

volatile int interruptCounter;
int totalInterruptCounter = 0;

float current_angle;
float prev_angle = 0; 
float value_of_error;
float prev_error = 0;
float error_sum = 0;

uint8_t topic_id = 5;
uint8_t splitindex;

std::string receivedData;
std::string valueX, valueY;
char outputData[10];

void IRAM_ATTR onTimer() 
{

    portENTER_CRITICAL_ISR(&timerMux);
    interruptCounter++;
    portEXIT_CRITICAL_ISR(&timerMux);
 
}

void init_Timer() 
{  
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 5000, true);
    timerAlarmEnable(timer);
}

void callback(char* topic, byte* message, unsigned int length)
{   
    for (int i = 0; i < length; i++) 
    {
        receivedData += (char)message[i];
        //correctValue = atoi(receivedData.c_str());
        splitindex = receivedData.find("/");
        valueX = receivedData.substr(0,splitindex);
        valueY = receivedData.substr(splitindex+1);
        xErrorValue = atoi(valueX.c_str());
        yErrorValue = atoi(valueY.c_str());
        Serial.println(xErrorValue);
        Serial.println(yErrorValue);

    } 
    //MESSAGE_IS_REC = true;
}

void setup()
{
    Serial.begin(115200);
    GyroRobot = MotorControl();
    GyroRobot.setupMotor();
    accel.begin();
    gyro.begin();
    init_Timer();
    mqtt.setupWifi();                   
    mqtt.setCallback(*callback);
    mqtt.subscribe(topic_id); 

}

void loop()
{
    GyroRobot.enableCentralMotor();
    
    if (interruptCounter > 0) {
 
        portENTER_CRITICAL(&timerMux);
        interruptCounter--;
        portEXIT_CRITICAL(&timerMux);

        totalInterruptCounter++;

        accel.readGXYZ(&ax, &ay, &az);
        gyro.readRadPerSecXYZ(&gx, &gy, &gz);
        filter.setKoeff(fps, BETA);
        filter.update(gx, gy, gz, ax, ay, az);
        roll = filter.getRollDeg();

        Serial.println(motorDirectionValue);
    
        current_angle = roll;  
        value_of_error = current_angle - target_angle;
        error_sum = error_sum + value_of_error;  
        error_sum = constrain(error_sum, -300, 300);
        motorDirectionValue = Kp*(value_of_error) + Ki*(error_sum)*sample_time - Kd*(current_angle-prev_angle)/sample_time;
        prev_angle = current_angle;

        if( -3.5 > roll || roll > 2 ) {
            GyroRobot.leftMotorDirection(motorDirectionValue ,correctMotorSpeed);
            GyroRobot.rightMotorDirection(motorDirectionValue ,correctMotorSpeed);
        } 
        else {
            GyroRobot.goForward(motorSpeed);
        }
    }
    mqtt.initClientLoop();
    itoa(motorDirectionValue, outputData, 10);  
    mqtt.pubFeedback(outputData,topic_id);
    
}
