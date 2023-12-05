#ifndef MOTOR_MANIPULATION_H
#define MOTOR_MANIPULATION_H

#include<Servo.h>

extern const float ESC_REFRESH_RATE; //Hz
extern float pwmFL, pwmFR, pwmBL, pwmBR;
extern byte rxLastState[7]; //roll - pitch - throttle - yaw
extern volatile unsigned long rxCurrentTime;
extern volatile unsigned long rxRiseEdge[7];
extern int rxPulseLength[7];

Servo motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
const int BACK_RIGHT_MOTOR_PIN = 6;
const int FRONT_RIGHT_MOTOR_PIN = 9;
const int BACK_LEFT_MOTOR_PIN = 10;
const int FRONT_LEFT_MOTOR_PIN = 11;
unsigned long escLoopTime, riseEdgeTime;
unsigned long fallEdgeTimeFL, fallEdgeTimeFR, fallEdgeTimeBL, fallEdgeTimeBR;

void motor_setup(){
  DDRD |= B01000000; // Declare D6 output
  DDRB |= B00001110; // Declare D9 D10 D11 output
  int writingTime = 2;
  for(int loopCal = 0; loopCal < (ESC_REFRESH_RATE * writingTime); loopCal++){ //Write 1000µs pulse
      PORTD |= B01000000; PORTB |= B00001110; // set 4 pins HIGH
      delayMicroseconds(1000);
      PORTD &= B10111111; PORTB &= B11110001; // set 4 pins LOW
      delayMicroseconds(3000);
  }
}

void createPWMPulse(){
  PORTD |= B01000000; PORTB |= B00001110; //set 4 pins HIGH
  riseEdgeTime = micros();
  fallEdgeTimeFL = riseEdgeTime + pwmFL;
  fallEdgeTimeFR = riseEdgeTime + pwmFR;
  fallEdgeTimeBL = riseEdgeTime + pwmBL;
  fallEdgeTimeBR = riseEdgeTime + pwmBR;
 
  while(((PORTD & B01000000) != 0) || ((PORTB & B00001110) != 0)){ //check binary values at specified indexes if they all aren't zeros
    escLoopTime = micros();   
    //set these pins to low
    if(escLoopTime >= fallEdgeTimeBR) PORTD &= B10111111; //D6
    if(escLoopTime >= fallEdgeTimeFR) PORTB &= B11111101; //D9
    if(escLoopTime >= fallEdgeTimeBL) PORTB &= B11111011; //D10
    if(escLoopTime >= fallEdgeTimeFL) PORTB &= B11110111; //D11
  }
}

void esc_calibrate(){
  unsigned long startTime = micros();
  const unsigned long waitTime = 5e6;
  // Pull throttle stick to max for esc calibration
  while(rxPulseLength[3] <= 1800){
    rxCurrentTime = micros();
    if(PINC & B00000100){ //Channel 3
      if(rxLastState[3] == 0){ rxLastState[3] = 1; rxRiseEdge[3] = rxCurrentTime; }
    }
    else if(rxLastState[3] == 1){
      rxLastState[3] = 0;
      rxPulseLength[3] = rxCurrentTime - rxRiseEdge[3];
    }
    if(micros() - startTime > waitTime) return; // no calibration
  }
  Serial.println("START MOTOR CALIBRATION");
  motorFrontLeft.attach(FRONT_LEFT_MOTOR_PIN, 1000, 2000);
  motorFrontRight.attach(FRONT_RIGHT_MOTOR_PIN, 1000, 2000);
  motorBackLeft.attach(BACK_LEFT_MOTOR_PIN, 1000, 2000);
  motorBackRight.attach(BACK_RIGHT_MOTOR_PIN, 1000, 2000);
  motorFrontLeft.writeMicroseconds(2000);
  motorFrontRight.writeMicroseconds(2000);
  motorBackLeft.writeMicroseconds(2000); 
  motorBackRight.writeMicroseconds(2000);
  delay(2000);
  motorFrontLeft.writeMicroseconds(1000);
  motorFrontRight.writeMicroseconds(1000);
  motorBackLeft.writeMicroseconds(1000); 
  motorBackRight.writeMicroseconds(1000);
  delay(3000);
  motorFrontLeft.detach();
  motorFrontRight.detach();
  motorBackLeft.detach();
  motorBackRight.detach();
  Serial.println("CALIBRATION DONE");
}

#endif

