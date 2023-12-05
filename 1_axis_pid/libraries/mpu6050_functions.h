#ifndef MPU6050_FUNCTIONS_H
#define MPU6050_FUNCTIONS_H

//#include <Adafruit_MPU6050.h>
//#include <Adafruit_Sensor.h>
#include <Wire.h>

#define ALPHA 0.95 //COMPLEMENTARY FILTER
#define RAD_TO_DEG 180.0 / 3.14159265
#define MPU6050_ADDRESS 0x68

double gyroXCall, gyroYCall, gyroZCall, accXCall, accYCall, accZCall;
double gyroX, gyroY, gyroZ, accX, accY, accZ;
double accPitch, accRoll, gyroPitch, gyroRoll;
extern double pitch, roll, yaw;

extern double currentTime, previousTime, elapsedTime;

#define GYRO_LSB  65.5 //131 or 65.5 or 32.8 or 16.4
#define ACC_LSB  8192.0 //16384 or 8192 or 4096 or 2048

void getIMUData(bool afterCalibration = true){
   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x3B) ;
   Wire.endTransmission(false);
   Wire.requestFrom(MPU6050_ADDRESS,6,true);

   //Unit: g (1g = 9.8m/s^2)
   accX = (Wire.read()<<8|Wire.read()) / ACC_LSB;
   accY = (Wire.read()<<8|Wire.read()) / ACC_LSB;
   accZ = (Wire.read()<<8|Wire.read()) / ACC_LSB;

   Wire.beginTransmission(MPU6050_ADDRESS); Wire.write(0x43) ;
   Wire.endTransmission(false); Wire.requestFrom(MPU6050_ADDRESS,4,true);

   gyroX = (Wire.read()<<8|Wire.read()) / GYRO_LSB;
   gyroY = (Wire.read()<<8|Wire.read()) / GYRO_LSB;
   gyroZ = (Wire.read()<<8|Wire.read()) / GYRO_LSB;

   if(afterCalibration){
      gyroX -= gyroXCall; 
      gyroY -= gyroYCall; 
      gyroZ -= gyroZCall; 
      accX -= accXCall;
      accY -= accYCall;
      accZ -= accZCall - 1.0;
   }
}

void applyComplementaryFilter(){
   accPitch = atan2(accX, sqrt(accY*accY + accZ*accZ)) * RAD_TO_DEG ;
   accRoll = atan2(accY, sqrt(accX*accX + accZ*accZ)) * RAD_TO_DEG;

   gyroPitch = pitch + gyroY * elapsedTime;
   gyroRoll = roll + gyroX * elapsedTime;

   pitch = ALPHA * gyroPitch + (1.0 - ALPHA) * accPitch;
   roll = ALPHA * gyroRoll + (1.0 - ALPHA) * accRoll;
}

void MPU6050_setup(){
   //https://invensense.tdk.com/wp-content/uploads/2015/02/MPU-6000-Register-Map1.pdf
   Wire.begin();

   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x6B); //RESET_CONFIG
   Wire.write(0x0); //I don't know why but don't edit this line
   Wire.endTransmission(true);

   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x1A);
   Wire.write(0x05);
   Wire.endTransmission(true);

   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x1B); // GYRO_CONFIG register
   Wire.write(0x8); // 00001000 (500deg/s)
   Wire.endTransmission(true);

   Wire.beginTransmission(MPU6050_ADDRESS);
   Wire.write(0x1C); //ACCEL_CONFIG register
   Wire.write(0x8); //00001000 (+/- 4g)
   Wire.endTransmission(true);
}

void MPU6050_calibrate(){
   Serial.println("Start calibrating MPU6050");
   for(int i = 1; i <= 200; i++){
      if(!(i % 10)){
         Serial.print("Iteration: "); Serial.println(i);
      }    
      getIMUData(false);
      accXCall += accX;
      accYCall += accY;
      accZCall += accZ;
      gyroXCall += gyroX;
      gyroYCall += gyroY;
      gyroZCall += gyroZ;
   }
   accXCall /= 200.0;
   accYCall /= 200.0;
   accZCall /= 200.0; 
   gyroXCall /= 200.0;
   gyroYCall /= 200.0;
   gyroZCall /= 200.0;

   Serial.println("Calibrated values:");
   Serial.print("gyroXCall: "); Serial.println(gyroXCall);
   Serial.print("gyroYCall: "); Serial.println(gyroYCall);
   Serial.print("gyroZCall: "); Serial.println(gyroZCall);
   Serial.print("accXCall: "); Serial.println(accXCall);
   Serial.print("accYCall: "); Serial.println(accYCall);
   Serial.print("accZCall: "); Serial.println(accZCall);
   delay(1500);
}

#endif
