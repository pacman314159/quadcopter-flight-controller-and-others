#ifndef VOLTAGE_DIVIDER_FUNCTIONS_H
#define VOLTAGE_DIVIDER_FUNCTIONS_H

#define VOLT_DIVIDER_COEFFICIENT (3900+2200)/2200
#define AVCC 5.04

double getBatteryVoltage(int input_pin){
   double inputVoltage = analogRead(input_pin) * (5.04 / 1023.0);
   return inputVoltage * VOLT_DIVIDER_COEFFICIENT;
}

#endif
