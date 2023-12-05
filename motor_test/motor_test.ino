#include<Servo.h>

#define MOTOR_PIN 9
#define BUTTON_PIN 2

double throttle = 1000;
Servo motor;

void setup(){
   pinMode(BUTTON_PIN, INPUT);
   motor.attach(MOTOR_PIN, 1000, 2000);
   motor.writeMicroseconds(throttle);
}

void loop(){
   if(digitalRead(BUTTON_PIN) == HIGH){
      throttle += 20.1;
      delay(500);
   }
   motor.writeMicroseconds(throttle);
}
