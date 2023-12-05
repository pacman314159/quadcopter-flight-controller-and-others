#ifndef BMP280_FUNCTIONS_H
#define BMP280_FUNCTIONS_H

#include<Wire.h>

const int BMP280_ADDRESS = 0x76;
const float EXP_FILTER_COEF_PRESSURE = 0.8;

uint16_t dig_T1, dig_P1;
int16_t dig_T2, dig_T3;
int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
//uint32_t press_msb, press_lsb, press_xlsb;
//uint32_t temp_msb, temp_lsb, temp_xlsb;

extern float pressureBMP;

const int NUM_DATA_SAMPLES = 20;
unsigned long int rawPressure, sumSensorData, averageDataSamples, sensorDatas[NUM_DATA_SAMPLES];
float fastPressure, slowPressure;
int memoryPosition;

void getBMPData(){
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(BMP280_ADDRESS, 6);
  /*
     press_msb = Wire.read(); press_lsb = Wire.read(); press_xlsb = Wire.read();
     temp_msb = Wire.read(); temp_lsb = Wire.read(); temp_xlsb = Wire.read();
     unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >> 4),
     adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >> 4);
  */
  unsigned long int adc_P = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4),
                adc_T = (Wire.read() << 12) | (Wire.read() << 4) | (Wire.read() >> 4);

  signed long int var1, var2;
  var1 = ((((adc_T>>3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
  var2 = (((((adc_T>>4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;

  unsigned long int p;
  var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11 ) * ((signed long int)dig_P6);
  var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
  var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
  var1 = (((dig_P3 * (((var1>>2) * (var1>>2)) >> 13 )) >> 3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
  var1 =((((32768+var1))*((signed long int)dig_P1))>>15);
  if (var1 == 0)
    return 0; // avoid exception caused by division by zero
  p = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
  if (p < 0x80000000) 
    p = (p << 1) / ((unsigned long int)var1);
  else
    p = (p / (unsigned long int)var1) * 2;
  var1 = (((signed long int)dig_P9) * ((signed long int)(((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((signed long int)(p>>2)) * ((signed long int)dig_P8))>>13;
  p = (unsigned long int)((signed long int)p + ((var1 + var2 + dig_P7) >> 4)); //Pressure in Pascal
  rawPressure = p;
}

void getBMPPressure(){
  if(memoryPosition == NUM_DATA_SAMPLES) memoryPosition = 0;
  sumSensorData -= sensorDatas[memoryPosition];
  sensorDatas[memoryPosition] = rawPressure;
  sumSensorData += sensorDatas[memoryPosition];
  ++memoryPosition;
  fastPressure = sumSensorData / NUM_DATA_SAMPLES;
  pressureBMP = fastPressure;
}

void BMP280_setup(){
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF4);
  Wire.write(0x2F); //0x57
  Wire.endTransmission(true);

  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0xF5);
  Wire.write(0x14); //0x14
  Wire.endTransmission(true);

  uint8_t data[24];
  Wire.beginTransmission(BMP280_ADDRESS);
  Wire.write(0x88);
  Wire.endTransmission(false);
  Wire.requestFrom(BMP280_ADDRESS, 24);
  for(int i = 0; i < 24; i++) data[i] = Wire.read();
  dig_T1 = (data[1] << 8) | data[0];
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6];
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11] << 8) | data[10];
  dig_P4 = (data[13] << 8) | data[12];
  dig_P5 = (data[15] << 8) | data[14];
  dig_P6 = (data[17] << 8) | data[16];
  dig_P7 = (data[19] << 8) | data[18];
  dig_P8 = (data[21] << 8) | data[20];
  dig_P9 = (data[23] << 8) | data[22];
}

void BMP280_calibrate(){
  Serial.println("START CALIBRATING BMP280");
  int numInterations = 30;
  for(int i = 0; i < numInterations; ++i){
    getBMPData(); getBMPPressure();
    delay(10);
  } 
  Serial.println("BMP280 CALIBRATION DONE");
}
#endif
