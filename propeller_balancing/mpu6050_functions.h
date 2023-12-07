#ifndef MPU6050_FUNCTIONS_H
#define MPU6050_FUNCTIONS_H

#include <Wire.h>
#include "objects.h"

const int MPU6050_ADDRESS = 0x68;
const float COMP_FILTER_COEF = 0.95, //Complementary Filter
            EXP_FILTER_COEF_GYRO = 0.8; //Exponential Filter

// ----------MPU6050 DATASHEET----------
const float GYRO_LSB = 65.5, //131, 65.5, 32.8, 16.4
            ACC_LSB = 8192.0; //16384, 8192, 4096, 2048
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

ThreeDimensions
  GYRO_CALIBRATE = {-1.8136743, -1.1240295, 0.3688241},
  ACC_CALIBRATE = {0.0439951, -0.0162146, 0.9741866};
ThreeDimensions acc, gyro; //imu

extern PrincipalAxes angle, rate;

extern float loopTime;

void getIMUData(){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS,6,true);

  //Unit: g (1g = 9.8m/s^2)
  acc.x = (Wire.read()<<8|Wire.read()) / ACC_LSB;
  acc.y = (Wire.read()<<8|Wire.read()) / ACC_LSB;
  acc.z = (Wire.read()<<8|Wire.read()) / ACC_LSB;

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x43) ;
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS,6,true);

  gyro.x = (Wire.read()<<8|Wire.read()) / GYRO_LSB;
  gyro.y = (Wire.read()<<8|Wire.read()) / GYRO_LSB;
  gyro.z = (Wire.read()<<8|Wire.read()) / GYRO_LSB;

  gyro.x -= GYRO_CALIBRATE.x;
  gyro.y -= GYRO_CALIBRATE.y; 
  gyro.z -= GYRO_CALIBRATE.z; 
  acc.x -= ACC_CALIBRATE.x;
  acc.y -= ACC_CALIBRATE.y;
  acc.z -= ACC_CALIBRATE.z;
}

void calculateAngle(){
  float accPitch, accRoll, gyroPitch, gyroRoll;

  accPitch = degrees(atan2(acc.x, sqrt(acc.y*acc.y + acc.z*acc.z)));
  accRoll = degrees(atan2(acc.y, sqrt(acc.x*acc.x + acc.z*acc.z)));

  // accRoll = degrees(atan2(acc.y, acc.z));
  // accPitch = atan2(acc.x, sqrt(acc.y*acc.y + acc.z*acc.z)) * RAD_TO_DEG ;

  gyroPitch = angle.pitch + gyro.y * loopTime;
  gyroRoll = angle.roll + gyro.x * loopTime;

  angle.pitch = COMP_FILTER_COEF * gyroPitch + (1.0 - COMP_FILTER_COEF) * accPitch;
  angle.roll = COMP_FILTER_COEF * gyroRoll + (1.0 - COMP_FILTER_COEF) * accRoll;
}

void calculateRotationRate(){
  rate.pitch = (rate.pitch * EXP_FILTER_COEF_GYRO) + (gyro.y * (1.0 - EXP_FILTER_COEF_GYRO));
  rate.roll = (rate.roll * EXP_FILTER_COEF_GYRO) + (gyro.x * (1.0 - EXP_FILTER_COEF_GYRO));
  rate.yaw = (rate.yaw * EXP_FILTER_COEF_GYRO) + (gyro.z * (1.0 - EXP_FILTER_COEF_GYRO));
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

#endif
