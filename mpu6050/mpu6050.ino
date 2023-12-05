#include "libraries/mpu6050_functions.h"

//double gyroXCall, gyroYCall, gyroZCall, accXCall, accYCall, accZCall;
//double gyroX, gyroY, gyroZ, accX, accY, accZ;
//double accPitch, accRoll, gyroPitch, gyroRoll;
double pitch, roll, yaw;
double currentTime, previousTime, elapsedTime;

void setup(){
   Serial.begin(19200);   
   MPU6050_setup();
   MPU6050_calibrate();
}

void loop(){
   currentTime = millis();
   elapsedTime = (currentTime - previousTime) / 1000.0;

   getIMUData();
   applyComplementaryFilter();
   Serial.print(pitch); Serial.print(" ");
   Serial.print(roll); Serial.print(" ");
   Serial.println("");

   previousTime = currentTime;
}
