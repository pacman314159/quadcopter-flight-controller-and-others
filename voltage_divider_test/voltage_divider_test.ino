#include "voltage_divider_functions.h"
#define BATTERY_VOLT_PIN A7

void setup() {
   Serial.begin(19200);
}

void loop() {
   double batteryVolt;
   batteryVolt = getBatteryVoltage(BATTERY_VOLT_PIN);
   Serial.println(batteryVolt);

   delay(100);
}
