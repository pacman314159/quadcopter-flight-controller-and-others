#ifndef MPU6050_FUNCTIONS_H
#define MPU6050_FUNCTIONS_H

#include <Wire.h>

#define ALPHA 0.95 //COMPLEMENTARY FILTER
#define RAD_TO_DEG 180.0 / 3.14159265
#define MPU6050_ADDRESS 0x68

// ----------DATASHEET----------
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

double gyroXCalibration, gyroYCalibration, gyroZCalibration;
double accXCalibration, accYCalibration, accZCalibration;
float accPitch, accRoll, gyroPitch, gyroRoll;
float gyroX, gyroY, gyroZ, accX, accY, accZ;
extern float pitch, roll, yaw;

extern float loopTime;

#define GYRO_LSB  65.5 //131, 65.5, 32.8, 16.4
#define ACC_LSB  8192.0 //16384, 8192, 4096, 2048

void getIMUData(bool afterCalibration = true){
   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x3B);
   Wire.endTransmission(false);
   Wire.requestFrom(MPU6050_ADDRESS,6,true);

   //Unit: g (1g = 9.8m/s^2)
   accX = (Wire.read()<<8|Wire.read()) / ACC_LSB;
   accY = (Wire.read()<<8|Wire.read()) / ACC_LSB;
   accZ = (Wire.read()<<8|Wire.read()) / ACC_LSB;

   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x43) ;
   Wire.endTransmission(false);
   Wire.requestFrom(MPU6050_ADDRESS,6,true);

   gyroX = (Wire.read()<<8|Wire.read()) / GYRO_LSB;
   gyroY = (Wire.read()<<8|Wire.read()) / GYRO_LSB;
   gyroZ = (Wire.read()<<8|Wire.read()) / GYRO_LSB;
   
   if(afterCalibration){
      gyroX -= gyroXCalibration; 
      gyroY -= gyroYCalibration; 
      gyroZ -= gyroZCalibration; 
      accX -= accXCalibration;
      accY -= accYCalibration;
      accZ -= accZCalibration;
   }
}

void applyComplementaryFilter(){
   accPitch = atan2(accX, sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG;
   accRoll = atan2(accY, accZ) * RAD_TO_DEG;

   /* accPitch = atan2(accX, sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG ; */
   /* accRoll = atan2(accY, sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG; */

   gyroPitch = pitch + gyroY * loopTime;
   gyroRoll = roll + gyroX * loopTime;

   pitch = ALPHA * gyroPitch + (1.0 - ALPHA) * accPitch;
   roll = ALPHA * gyroRoll + (1.0 - ALPHA) * accRoll;
}

void MPU6050_setup(){
   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x6B);
   Wire.write(0x00); //Disable temperature sensor
   Wire.endTransmission(true);

   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x1A); //DIGITAL_LOW_PASS_FILTER_CONFIG
   Wire.write(0x05);
   Wire.endTransmission(true);   

   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x1B); // GYRO_CONFIG register
   Wire.write(0x8); // 00001000 (500deg/s full scale)
   Wire.endTransmission(true);

   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x1C); //ACCEL_CONFIG register
   Wire.write(0x8); //00001000 (+/- 4g full scale range)
   Wire.endTransmission(true);   
}

void MPU6050_calibrate(){
   float calibrationIterations = 500.0;
   Serial.println("START CALIBRATING MPU6050");
   for(int i = 0; i < calibrationIterations; ++i){
      getIMUData(false);
      accXCalibration += accX;
      accYCalibration += accY;
      accZCalibration += accZ;
      gyroXCalibration += gyroX;
      gyroYCalibration += gyroY;
      gyroZCalibration += gyroZ;
      delay(10);
   }
   accXCalibration /= calibrationIterations;
   accYCalibration /= calibrationIterations;
   accZCalibration = (accZCalibration / calibrationIterations) - 1.0; 
   gyroXCalibration /= calibrationIterations;
   gyroYCalibration /= calibrationIterations;
   gyroZCalibration /= calibrationIterations;

   Serial.println("MPU6050 CALIBRATION DONE");
}

#endif
