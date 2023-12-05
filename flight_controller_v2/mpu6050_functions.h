#ifndef MPU6050_FUNCTIONS_H
#define MPU6050_FUNCTIONS_H

#include <Wire.h>

const float COMP_FILTER_COEF = 0.95; //Complementary Filter
const float EXP_FILTER_COEF_GYRO = 0.8; //Exponential Filter
const int MPU6050_ADDRESS = 0x68;

// ----------DATASHEET----------
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

double gyroXCalibration, gyroYCalibration, gyroZCalibration;
double accXCalibration, accYCalibration, accZCalibration;
float accPitch, accRoll, gyroPitch, gyroRoll;
float gyroX, gyroY, gyroZ;

extern float accX, accY, accZ;
extern float pitch, roll;
extern float ratePitch, rateRoll, rateYaw;

extern float loopTime;

const double GYRO_LSB = 65.5; //131, 65.5, 32.8, 16.4
const double ACC_LSB = 8192.0; //16384, 8192, 4096, 2048

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

void getCurrentAngle(){
   accPitch = degrees(atan2(accX, sqrt(accY*accY + accZ*accZ)));
   accRoll = degrees(atan2(accY, accZ));

   /* accPitch = atan2(accX, sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG ; */
   /* accRoll = atan2(accY, sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG; */

   gyroPitch = pitch + gyroY * loopTime;
   gyroRoll = roll + gyroX * loopTime;

   pitch = COMP_FILTER_COEF * gyroPitch + (1.0 - COMP_FILTER_COEF) * accPitch;
   roll = COMP_FILTER_COEF * gyroRoll + (1.0 - COMP_FILTER_COEF) * accRoll;
}

void getRotationRate(){
  ratePitch = (ratePitch * EXP_FILTER_COEF_GYRO) + (gyroY * (1.0 - EXP_FILTER_COEF_GYRO));
  rateRoll = (rateRoll * EXP_FILTER_COEF_GYRO) + (gyroX * (1.0 - EXP_FILTER_COEF_GYRO));
  rateYaw = (rateYaw * EXP_FILTER_COEF_GYRO) + (gyroZ * (1.0 - EXP_FILTER_COEF_GYRO));
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
   Wire.write(0x08); // 00001000 (500deg/s full scale)
   Wire.endTransmission(true);

   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x1C); //ACCEL_CONFIG register
   Wire.write(0x08); //00001000 (+/- 4g full scale range)
   Wire.endTransmission(true);   
}

void MPU6050_calibrate(){
   int calibrationIterations = 500;
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
