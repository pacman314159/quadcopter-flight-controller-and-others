#include<Servo.h>
#include "libraries/mpu6050_functions.h"

#define MPU6050_CALIBRATE_PIN 2

//double gyroXCall, gyroYCall, gyroZCall, accXCall, accYCall, accZCall;
//double gyroX, gyroY, gyroZ, accX, accY, accZ;
//double accPitch, accRoll, gyroPitch, gyroRoll;
double pitch, roll, yaw;
double throttle = 50, desiredRollAngle = 0;
double defaultThrottle = 1140;

double currentTime, previousTime, elapsedTime;

double error, prevError;
double pwmLeft, pwmRight;

#define KP 2.9 //PID constants
#define KI 0.05
#define KD 1.76 //0.3
double p_term, i_term, d_term;
double pid; //sum of the 3 terms

Servo motorLeft, motorRight;
#define RIGHT_MOTOR_PIN 9
#define LEFT_MOTOR_PIN 10

void setup(){
   Serial.begin(19200);

   digitalWrite(13, HIGH);
   MPU6050_setup();
   while(digitalRead(MPU6050_CALIBRATE_PIN) != HIGH);
   MPU6050_calibrate();
   digitalWrite(13, LOW);

   motorLeft.attach(LEFT_MOTOR_PIN, 1000, 2000);
   motorRight.attach(RIGHT_MOTOR_PIN, 1000, 2000);
   motorLeft.writeMicroseconds(1000);
   motorRight.writeMicroseconds(1000);

   delay(3000);
}


void loop() {
   previousTime = currentTime;
   currentTime = millis();
   elapsedTime = (currentTime - previousTime) / 1000.0;

   getIMUData();
   applyComplementaryFilter();
   //_____1-AXIS PID (USE ROLL)_____
   error = roll - desiredRollAngle; 
   p_term = KP * error;
   i_term += KI * (error + prevError) * elapsedTime * 0.5;
   d_term = KD * ((error - prevError) / elapsedTime);
   pid = p_term + i_term + d_term;
   if(pid > 400) pid = 400;
   if(pid < -400) pid = -400;
   prevError = error;
   //________________________________
   pwmLeft = defaultThrottle + throttle - pid; 
   pwmRight = defaultThrottle + throttle + pid; 

   if(pwmLeft < 1000) pwmLeft = 1000;
   else if(pwmLeft > 2000) pwmLeft = 2000;
   if(pwmRight < 1000) pwmRight = 1000;
   else if(pwmRight > 2000) pwmRight = 2000;

   motorLeft.writeMicroseconds(pwmLeft);
   motorRight.writeMicroseconds(pwmRight);
}
