#ifndef MPU6050_FUNCTIONS_H
#define MPU6050_FUNCTIONS_H

#include <Wire.h>
#include "objects.h"

const int MPU6050_ADDRESS = 0x68,
          CALIBRATION_ITERATIONS = 500;
const float COMP_FILTER_COEF = 0.95, //Complementary Filter
            EXP_FILTER_COEF_GYRO = 0.8; //Exponential Filter

// ----------MPU6050 DATASHEET----------
const float GYRO_LSB = 65.5, //131, 65.5, 32.8, 16.4
            ACC_LSB = 8192.0; //16384, 8192, 4096, 2048
// https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf

ThreeDimensions
  GYRO_CALIBRATE = {-1.6513175, -1.1943113, 0.4746280},
  ACC_CALIBRATE = {0.0447114, -0.0068686, -0.0244645};
ThreeDimensions acc, gyro;

extern PrincipalAxes angle, rate;

extern float loopTime;

float temperature;

void getIMUData(bool afterCalibration = true){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 14, true);

  //Unit: g (1g = 9.8m/s^2)
  acc.x = (Wire.read()<<8|Wire.read()) / ACC_LSB;
  acc.y = (Wire.read()<<8|Wire.read()) / ACC_LSB;
  acc.z = (Wire.read()<<8|Wire.read()) / ACC_LSB;
  temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
  gyro.x = (Wire.read()<<8|Wire.read()) / GYRO_LSB;
  gyro.y = (Wire.read()<<8|Wire.read()) / GYRO_LSB;
  gyro.z = (Wire.read()<<8|Wire.read()) / GYRO_LSB;

   if(afterCalibration){
      gyro.x -= GYRO_CALIBRATE.x;
      gyro.y -= GYRO_CALIBRATE.y; 
      gyro.z -= GYRO_CALIBRATE.z; 
      acc.x -= ACC_CALIBRATE.x;
      acc.y -= ACC_CALIBRATE.y;
      acc.z -= ACC_CALIBRATE.z;
   }
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
  Wire.write(0x03); //44Hz
  Wire.endTransmission(true);   

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x1B); // GYRO_CONFIG register
  Wire.write(0x08); // 00001000 (500deg/s full scale)
  Wire.endTransmission(true);

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x1C); //ACCEL_CONFIG register
  Wire.write(0x10); //00010000 (+/- 8g full scale range)
  Wire.endTransmission(true);   
}

void MPU6050_calibrate(){
   Serial.println("START CALIBRATING MPU6050");

   for(int i = 0; i < CALIBRATION_ITERATIONS; ++i){
      getIMUData(false);
      ACC_CALIBRATE.x += acc.x;
      ACC_CALIBRATE.y += acc.y;
      ACC_CALIBRATE.z += acc.z;
      GYRO_CALIBRATE.x += gyro.x;
      GYRO_CALIBRATE.y += gyro.y;
      GYRO_CALIBRATE.z += gyro.z;
      delay(10);
   }
   ACC_CALIBRATE.x /= CALIBRATION_ITERATIONS;
   ACC_CALIBRATE.y /= CALIBRATION_ITERATIONS;
   ACC_CALIBRATE.z = (ACC_CALIBRATE.z / CALIBRATION_ITERATIONS) - 1.0; 
   GYRO_CALIBRATE.x /= CALIBRATION_ITERATIONS;
   GYRO_CALIBRATE.y /= CALIBRATION_ITERATIONS;
   GYRO_CALIBRATE.z /= CALIBRATION_ITERATIONS;

   Serial.println("MPU6050 CALIBRATION DONE");
   Serial.print("acc x: "); Serial.println(ACC_CALIBRATE.x, 7);
   Serial.print("acc y: "); Serial.println(ACC_CALIBRATE.y, 7);
   Serial.print("acc z: "); Serial.println(ACC_CALIBRATE.z, 7);
   Serial.print("gyro x: "); Serial.println(GYRO_CALIBRATE.x, 7);
   Serial.print("gyro y: "); Serial.println(GYRO_CALIBRATE.y, 7);
   Serial.print("gyro z: "); Serial.println(GYRO_CALIBRATE.z, 7);
}

#endif
