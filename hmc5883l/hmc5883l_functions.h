#ifndef HMC5883L_FUNCTION_H
#define HMC5883L_FUNCTION_H

#define RAD_TO_DEG 180.0 / 3.14159265
#define DEG_TO_RAD 3.14159265 / 180.0
#define PI 3.14159265
#define HCM_MAGNETIC_DECLINATION -0.7821

int xData, yData, zData;
double xHorizontal, yHorizontal, zHorizontal;
extern double azimuth;
extern double pitch, roll;

void HMC5883L_setup(){
  // https://cdn-shop.adafruit.com/datasheets/HMC5883L_3-Axis_Digital_Compass_IC.pdf
  Wire.beginTransmission(0x1E);
  Wire.write(0x00);
  Wire.write(0x70); //01110000
  Wire.endTransmission(true);

  Wire.beginTransmission(0x1E);
  Wire.write(0x01);
  Wire.write(0x20); //00100000
  // Wire.endTransmission(true);

  Wire.beginTransmission(0x1E);
  Wire.write(0x02);
  Wire.write(0x00);
  Wire.endTransmission(true);

  delay(40);
}

void getCompassData(){
  Wire.beginTransmission(0x1E);
  Wire.write(0x03);
  Wire.endTransmission(false);
  Wire.requestFrom(0x1E, 6);
  //For orientation purpose, x and y need to be swapped
  // xData = (Wire.read() << 8 | Wire.read());
  xData = Wire.read() << 8 | Wire.read();
  zData = Wire.read() << 8 | Wire.read();
  yData = Wire.read() << 8 | Wire.read();

  Serial.print(xData);
  Serial.print(" ");
  Serial.print(yData);
  Serial.print(" ");
  Serial.print(zData);
  Serial.println(" ");
  /* xHorizontal = xData * cos(pitch * DEG_TO_RAD) */
  /*               + yData * sin(pitch * DEG_TO_RAD) * sin (roll * DEG_TO_RAD) */
  /*               - zData * cos(roll * DEG_TO_RAD) * sin(pitch * DEG_TO_RAD); */
  /* yHorizontal = yData * cos(roll * DEG_TO_RAD) */
  /*               + zData * sin(roll * DEG_TO_RAD); */
  /* Serial.print(xHorizontal); Serial.print(" "); Serial.println(yHorizontal); */
  /* azimuth = (atan2(yHorizontal, xHorizontal) + PI) * RAD_TO_DEG + HCM_MAGNETIC_DECLINATION; */
  /* if(azimuth < 0) azimuth += 360.0; */
}

#endif
