#include "bmp280_functions.h"

float altitudeBMP; //meters
unsigned long int pressureBMP;
float slowPressure, fastPressure;

void setup(){
   digitalWrite(13, HIGH);

   Serial.begin(19200);
   Wire.begin();
   BMP280_setup();
   BMP280_calibrate();
   digitalWrite(13, LOW);
}

void loop(){
   getBMPData();
   getAltitude();
   /*
   Serial.print(slowPressure);
   Serial.print(" ");
   Serial.println(fastPressure);
   */
   Serial.println(altitudeBMP);
}
