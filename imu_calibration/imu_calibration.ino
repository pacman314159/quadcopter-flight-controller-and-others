#include <Wire.h>

const int MPU6050_ADDRESS = 0x68;

const double GYRO_LSB = 65.5; //131, 65.5, 32.8, 16.4
const double ACC_LSB = 8192.0; //16384, 8192, 4096, 2048

const int CALIBRATION_ITERATIONS = 500;

double accX_calibrate, accY_calibrate, accZ_calibrate;
double gyroX_calibrate, gyroY_calibrate, gyroZ_calibrate;
double accX_raw, accY_raw, accZ_raw;
double gyroX_raw, gyroY_raw, gyroZ_raw;

void getIMUData(){
  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);

  accX_raw = (Wire.read()<<8 | Wire.read()) / ACC_LSB;
  accY_raw = (Wire.read()<<8 | Wire.read()) / ACC_LSB;
  accZ_raw = (Wire.read()<<8 | Wire.read()) / ACC_LSB;

  Wire.beginTransmission(MPU6050_ADDRESS);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDRESS, 6, true);

  gyroX_raw = (Wire.read()<<8 | Wire.read()) / GYRO_LSB;
  gyroY_raw = (Wire.read()<<8 | Wire.read()) / GYRO_LSB;
  gyroZ_raw = (Wire.read()<<8 | Wire.read()) / GYRO_LSB;
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
  Serial.println("START CALIBRATING MPU6050");
  for(int i = 0; i < CALIBRATION_ITERATIONS; ++i){
    getIMUData();
    accX_calibrate += accX_raw; 
    accY_calibrate += accY_raw; 
    accZ_calibrate += accZ_raw; 
    gyroX_calibrate += gyroX_raw;
    gyroY_calibrate += gyroY_raw;
    gyroZ_calibrate += gyroZ_raw;
    delay(10);
  }
  accX_calibrate /= CALIBRATION_ITERATIONS;
  accY_calibrate /= CALIBRATION_ITERATIONS;
  accZ_calibrate /= CALIBRATION_ITERATIONS;
  gyroX_calibrate /= CALIBRATION_ITERATIONS;
  gyroY_calibrate /= CALIBRATION_ITERATIONS;
  gyroZ_calibrate /= CALIBRATION_ITERATIONS;
  Serial.println("MPU6050 CALIBRATION DONE");
}

void printCalibrationData(){
  Serial.print("accX: "); Serial.println(accX_calibrate, 7);
  Serial.print("accY: "); Serial.println(accY_calibrate, 7);
  Serial.print("accZ: "); Serial.println(accZ_calibrate, 7);
  Serial.print("gyroX: "); Serial.println(gyroX_calibrate, 7);
  Serial.print("gyroY: "); Serial.println(gyroY_calibrate, 7);
  Serial.print("gyroZ: "); Serial.println(gyroZ_calibrate, 7);
}

void setup(){
  Serial.begin(19200);
  TWBR = 12;
  MPU6050_setup();
  MPU6050_calibrate();
  printCalibrationData();
}

void loop(){

}

