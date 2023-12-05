#include<Servo.h>
#include "mpu6050_functions.h"
#include "motor_manipulation.h"
#include "bmp280_functions.h"
#include "obj.h"

const float PWM_BALANCE_BOUNDARY = 15.0;
const float SIGNAL_LOSS_THRESHOLD = 930;
const float ESC_REFRESH_RATE = 250.0; //Hz

//-----(RECEIVER) PIN CHANGE INTERRUPT--------
byte rxLastState[7]; //roll - pitch - throttle - yaw - channel5 - channel6
volatile unsigned long int rxCurrentTime = 0;
volatile unsigned long int rxRiseEdge[7] = {0};
int rxPulseLength[7];
void rx_interrupt_setup(){
  PCICR |= B00000111; //enable PORT B, C, D
  PCMSK0 |= B00010000; //enable PCINT4 (D12) (channel 6) (PORT B)
  PCMSK1 |= B00001111; //enable PCINT8 -> PCINT11 (A0 -> A3) (channel 1 -> 4) (PORT C)
  PCMSK2 |= B00001000; //enable D3 (channl 5) (PORT D)
}

float loopTime;
int loopTimeMicros;
unsigned long previousTime;
float defaultThrottle = 1100, throttle, desiredThrottle;

bool altitudeHoldMode, prevAltitudeHoldState;
float pressureBMP;
float desiredPressure, errorPressure, prevErrorPressure;

PrincipalAxes angle, rate, desiredRate;
PrincipalAxes errorRate, prevErrorRate;
PrincipalAxes p_term, i_term, d_term, pid;

//                     pitch roll yaw 
PrincipalAxes P_gain = {1.0, 1.0, 1.0},
              I_gain = {0.0, 0.0, 0.0},
              D_gain = {0.5, 0.5, 0.5}; //constants here

float pwmFL, pwmFR, pwmBL, pwmBR;

void(* resetBoard) (void) = 0;

void handleElectromagnet(){
  if(rxPulseLength[5] < 1100) PORTD &= B11011111;
  else PORTD |= B00100000;
}

void precalculation(){
  if(rxPulseLength[3] < SIGNAL_LOSS_THRESHOLD) resetBoard();
  // for(int i = 1; i <= 6; i++){ Serial.print(rxPulseLength[i]); Serial.print(" "); } Serial.println();

  altitudeHoldMode = rxPulseLength[6] < 1500 ? true : false;
  desiredThrottle = 0.9 * rxPulseLength[3] - 900;

  if(rxPulseLength[2] > 1500 + PWM_BALANCE_BOUNDARY)
    desiredRate.pitch = rxPulseLength[2] - 1500 - PWM_BALANCE_BOUNDARY;
  else if(rxPulseLength[2] < 1500 - PWM_BALANCE_BOUNDARY)
    desiredRate.pitch = rxPulseLength[2] - 1500 + PWM_BALANCE_BOUNDARY;
  else desiredRate.pitch = 0.0;
  desiredRate.pitch -= angle.pitch * 15;
  desiredRate.pitch /= 3.0;

  if(rxPulseLength[1] > 1500 + PWM_BALANCE_BOUNDARY)
    desiredRate.roll = rxPulseLength[1] - 1500 - PWM_BALANCE_BOUNDARY;
  else if(rxPulseLength[1] < 1500 - PWM_BALANCE_BOUNDARY)
    desiredRate.roll = rxPulseLength[1] - 1500 + PWM_BALANCE_BOUNDARY;
  else desiredRate.roll = 0.0;
  desiredRate.roll -= angle.roll * 15;
  desiredRate.roll /= 3.0;

  if(rxPulseLength[4] > 1500 + PWM_BALANCE_BOUNDARY)
    desiredRate.yaw = (rxPulseLength[4] - 1500 - PWM_BALANCE_BOUNDARY) / 3.0;
  else if(rxPulseLength[4] < 1500 - PWM_BALANCE_BOUNDARY)
    desiredRate.yaw = (rxPulseLength[4] - 1500 + PWM_BALANCE_BOUNDARY) / 3.0;
  else desiredRate.yaw = 0.0;
}

void calculatePID(){
  errorRate.pitch = rate.pitch - desiredRate.pitch;
  p_term.pitch = P_gain.pitch * errorRate.pitch;
  i_term.pitch += I_gain.pitch * errorRate.pitch;
  d_term.pitch = D_gain.pitch * (errorRate.pitch - prevErrorRate.pitch);
  pid.pitch = p_term.pitch + i_term.pitch + d_term.pitch;
  prevErrorRate.pitch = errorRate.pitch;

  errorRate.roll = rate.roll - desiredRate.roll;
  p_term.roll = P_gain.roll * errorRate.roll;
  i_term.roll += I_gain.roll * errorRate.roll;
  d_term.roll = D_gain.roll * (errorRate.roll - prevErrorRate.roll);
  pid.roll = p_term.roll + i_term.roll + d_term.roll;
  prevErrorRate.roll = errorRate.roll;

  errorRate.yaw = rate.yaw - desiredRate.yaw;
  p_term.yaw = P_gain.yaw * errorRate.yaw;
  i_term.yaw += I_gain.yaw * errorRate.yaw;
  d_term.yaw = D_gain.yaw * (errorRate.yaw - prevErrorRate.yaw);
  pid.yaw = p_term.yaw + i_term.yaw + d_term.yaw;
  prevErrorRate.yaw = errorRate.yaw;
}

void calculatePWM(){
  pwmFL = pid.pitch - pid.roll - pid.yaw + throttle + defaultThrottle;
  pwmFR = pid.pitch + pid.roll + pid.yaw + throttle + defaultThrottle;
  pwmBL = -pid.pitch - pid.roll + pid.yaw + throttle + defaultThrottle;
  pwmBR = -pid.pitch + pid.roll -pid.yaw + throttle + defaultThrottle;
  pwmFL = max(pwmFL, defaultThrottle);
  pwmFR = max(pwmFR, defaultThrottle);
  pwmBL = max(pwmBL, defaultThrottle);
  pwmBR = max(pwmBR, defaultThrottle);
  // Serial.print(pwmFL); Serial.print(' '); Serial.print(pwmFR); Serial.print(' '); Serial.print(pwmBL); Serial.print(' '); Serial.println(pwmBR);
}

void finalPause(){
  while(not(rxPulseLength[4] < 1150 and rxPulseLength[3] < 1150)){ //Pull throttle stick down and yaw stick left
    if(PINC & B00000100){ //Channel 3
      if(rxLastState[3] == 0){ rxLastState[3] = 1; rxRiseEdge[3] = rxCurrentTime; }
    }else if(rxLastState[3] == 1){ rxLastState[3] = 0; rxPulseLength[3] = rxCurrentTime - rxRiseEdge[3]; }
    if(PINC & B00001000){ //Channel 4
      if(rxLastState[4] == 0){ rxLastState[4] = 1; rxRiseEdge[4] = rxCurrentTime; }
    }else if(rxLastState[4] == 1){ rxLastState[4] = 0; rxPulseLength[4] = rxCurrentTime - rxRiseEdge[4]; }
  }
}

void setup(){
  Serial.begin(19200);
  Wire.begin();
  TWBR = 12; //set 400kHz I2C clock frequency
  DDRB |= B00100000; PORTB |= B00100000; //Declare D13 output then set HIGH
  DDRD |= B00100000; PORTD |= B00100000; //Declare D5 output then set HIGH for magnet

  loopTime = 1.0 / ESC_REFRESH_RATE;
  loopTimeMicros = loopTime * 1e6;

  rx_interrupt_setup();
  MPU6050_setup();
  BMP280_setup();
  BMP280_calibrate();
  esc_calibrate();

  Serial.println("Last chance for safty checks, pull yaw stick left to proceed routine");
  finalPause();
  Serial.println("STARTING MOTORS, PLEASE STEP ASIDE");

  PORTB &= B11011111; //set D13 LOW
  delay(2500);
  motor_setup();
  previousTime = micros();
}


void loop(){
  handleElectromagnet();

  getBMPData();
  getBMPPressure();

  getIMUData();
  calculateAngle();
  calculateRotationRate();

  precalculation();
  calculatePID();

  calculatePWM();
  // createPWMPulse();

  if(micros() - previousTime >= loopTimeMicros) PORTB |= B00100000; //turn on indicator light D13 if 250Hz loop is overwhelmed
  while(micros() - previousTime < loopTimeMicros); // guarantee 250Hz loop time 
  previousTime = micros();
}

//Read PWM (channel 1 -> 4) from receiver
ISR(PCINT0_vect){
  rxCurrentTime = micros();
  if(PINB & B00010000){  //Channel 6
    if(rxLastState[6] == 0){
      rxLastState[6] = 1;
      rxRiseEdge[6] = rxCurrentTime;
    }
  }else if(rxLastState[6] == 1){
    rxLastState[6] = 0;
    rxPulseLength[6] = rxCurrentTime - rxRiseEdge[6];
  }
}
ISR(PCINT1_vect){
  rxCurrentTime = micros();
  if(PINC & B00000001){  //Channel 1
    if(rxLastState[1] == 0){
      rxLastState[1] = 1;
      rxRiseEdge[1] = rxCurrentTime;
    }
  }else if(rxLastState[1] == 1){
    rxLastState[1] = 0;
    rxPulseLength[1] = rxCurrentTime - rxRiseEdge[1];
  }
  if(PINC & B00000010){ //Channel 2
    if(rxLastState[2] == 0){
      rxLastState[2] = 1;
      rxRiseEdge[2] = rxCurrentTime;
    }
  }else if(rxLastState[2] == 1){
    rxLastState[2] = 0;  
    rxPulseLength[2] = rxCurrentTime - rxRiseEdge[2];
  }
  if(PINC & B00000100){ //Channel 3
    if(rxLastState[3] == 0){
      rxLastState[3] = 1;
      rxRiseEdge[3] = rxCurrentTime;
    }
  }
  else if(rxLastState[3] == 1){
    rxLastState[3] = 0;
    rxPulseLength[3] = rxCurrentTime - rxRiseEdge[3];
  }
  if(PINC & B00001000){ //Channel 4
    if(rxLastState[4] == 0){
      rxLastState[4] = 1;
      rxRiseEdge[4] = rxCurrentTime;
    }
  }else if(rxLastState[4] == 1){
    rxLastState[4] = 0;
    rxPulseLength[4] = rxCurrentTime - rxRiseEdge[4];
  }
}
ISR(PCINT2_vect){
  rxCurrentTime = micros();
  if(PIND & B00001000){  //Channel 5
    if(rxLastState[5] == 0){
      rxLastState[5] = 1;
      rxRiseEdge[5] = rxCurrentTime;
    }
  }else if(rxLastState[5] == 1){
    rxLastState[5] = 0;
    rxPulseLength[5] = rxCurrentTime - rxRiseEdge[5];
  }
}
//µ
