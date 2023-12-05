#include<Wire.h>
#include "hmc5883l_functions.h";
#include<BasicLinearAlgebra.h>


double azimuth;
double pitch = 0.0, roll = 0.0;

void setup(){
  Serial.begin(19200);
  Wire.begin();
  digitalWrite(13, HIGH);

  HMC5883L_setup();

  digitalWrite(13, LOW);
}

void loop(){
  if(digitalRead(7)) Serial.println("quit");
  getCompassData();
  //Serial.println(azimuth);
  delay(50);
}
