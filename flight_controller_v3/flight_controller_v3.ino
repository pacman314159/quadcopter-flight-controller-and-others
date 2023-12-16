#include "mpu6050_functions.h"
#include "motor_manipulation.h"
#include "bmp280_functions.h"
#include "objects.h"

const float PWM_BALANCE_BOUNDARY = 15.0;
const int ESC_REFRESH_RATE = 250, //Hz
          SIGNAL_LOSS_THRESHOLD = 930,
          PWM_SAFETY_START_THRESHOLD = 1400,
          DEFAULT_THROTTLE = 1100,
          MAX_PWM_LENGTH = 1900;

//-----(RECEIVER) PIN CHANGE INTERRUPT--------
byte rxLastState[7]; //roll - pitch - throttle - yaw - channel5 - channel6
volatile unsigned long rxCurrentTime = 0;
volatile unsigned long rxRiseEdge[7] = {0};
int rxPulseLength[7];
void receiver_setup(){ //pin change interruption
  PCICR |= B00000111; //enable PORT B, C, D
  PCMSK0 |= B00010000; //enable D12 (channel 6) (PORT B)
  PCMSK1 |= B00001111; //enable A0 -> A3 (channel 1 -> 4) (PORT C)
  PCMSK2 |= B00001000; //enable D3 (channel 5) (PORT D)
}
//---------------------------------------------

float loopTime;
int loopTimeMicros;
unsigned long previousTime;
float throttle, desiredThrottle;

bool altitudeHoldMode, prevAltitudeHoldState;
float pressureBMP;
float desiredPressure, errorPressure, prevErrorPressure;

PrincipalAxes angle, rate, desiredRate,
              errorRate, prevErrorRate,
              p_term, i_term, d_term, pid;
// PrincipalAxes P_gain = {2.5, 2.5, 3.0},
//               I_gain = {0.01, 0.01, 0.01},
//               D_gain = {25.0, 25.0, 0.0}; //constants here
PrincipalAxes P_gain = {2.0, 2.0 , 3.0},
              I_gain = {0.01, 0.01, 0.00},
              D_gain = {20.0, 20.0, 0.0}; //constants here

float pwmFL, pwmFR, pwmBL, pwmBR;

void(* resetBoard) (void) = 0;

void handleElectromagnet(){
  if(rxPulseLength[5] < 1100) PORTD &= B11011111;
  else PORTD |= B00100000;
}

void precalculation(){
  // altitudeHoldMode = rxPulseLength[6] < 1500 ? true : false;
  desiredThrottle = throttle = 0.9 * rxPulseLength[3] - 900;

  if(rxPulseLength[1] > 1500 + PWM_BALANCE_BOUNDARY)
    desiredRate.roll = rxPulseLength[1] - 1500 - PWM_BALANCE_BOUNDARY;
  else if(rxPulseLength[1] < 1500 - PWM_BALANCE_BOUNDARY)
    desiredRate.roll = rxPulseLength[1] - 1500 + PWM_BALANCE_BOUNDARY;
  else desiredRate.roll = 0.0;
  desiredRate.roll -= angle.roll * 15;
  desiredRate.roll /= 3.0;

  if(rxPulseLength[2] > 1500 + PWM_BALANCE_BOUNDARY)
    desiredRate.pitch = rxPulseLength[2] - 1500 - PWM_BALANCE_BOUNDARY;
  else if(rxPulseLength[2] < 1500 - PWM_BALANCE_BOUNDARY)
    desiredRate.pitch = rxPulseLength[2] - 1500 + PWM_BALANCE_BOUNDARY;
  else desiredRate.pitch = 0.0;
  desiredRate.pitch += angle.pitch * 15;
  desiredRate.pitch /= 3.0;

  if(rxPulseLength[4] > 1500 + PWM_BALANCE_BOUNDARY)
    desiredRate.yaw = (rxPulseLength[4] - 1500 - PWM_BALANCE_BOUNDARY) / 3.0;
  else if(rxPulseLength[4] < 1500 - PWM_BALANCE_BOUNDARY)
    desiredRate.yaw = (rxPulseLength[4] - 1500 + PWM_BALANCE_BOUNDARY) / 3.0;
  else desiredRate.yaw = 0.0;
}

void calculatePID(){
  errorRate.roll = rate.roll - desiredRate.roll;
  p_term.roll = P_gain.roll * errorRate.roll;
  i_term.roll += I_gain.roll * errorRate.roll;
  d_term.roll = D_gain.roll * (errorRate.roll - prevErrorRate.roll);
  pid.roll = p_term.roll + i_term.roll + d_term.roll;
  prevErrorRate.roll = errorRate.roll;

  errorRate.pitch = rate.pitch - desiredRate.pitch;
  p_term.pitch = P_gain.pitch * errorRate.pitch;
  i_term.pitch += I_gain.pitch * errorRate.pitch;
  d_term.pitch = D_gain.pitch * (errorRate.pitch - prevErrorRate.pitch);
  pid.pitch = p_term.pitch + i_term.pitch + d_term.pitch;
  prevErrorRate.pitch = errorRate.pitch;

  errorRate.yaw = rate.yaw - desiredRate.yaw;
  p_term.yaw = P_gain.yaw * errorRate.yaw;
  i_term.yaw += I_gain.yaw * errorRate.yaw;
  d_term.yaw = D_gain.yaw * (errorRate.yaw - prevErrorRate.yaw);
  pid.yaw = p_term.yaw + i_term.yaw + d_term.yaw;
  prevErrorRate.yaw = errorRate.yaw;
}

void calculatePWM(){
  pwmFL = throttle + pid.pitch - pid.roll + pid.yaw + DEFAULT_THROTTLE;
  pwmFR = throttle + pid.pitch + pid.roll - pid.yaw + DEFAULT_THROTTLE;
  pwmBL = throttle - pid.pitch - pid.roll - pid.yaw + DEFAULT_THROTTLE;
  pwmBR = throttle - pid.pitch + pid.roll + pid.yaw + DEFAULT_THROTTLE;
  pwmFL = constrain(pwmFL, DEFAULT_THROTTLE, MAX_PWM_LENGTH);
  pwmFR = constrain(pwmFR, DEFAULT_THROTTLE, MAX_PWM_LENGTH);
  pwmBL = constrain(pwmBL, DEFAULT_THROTTLE, MAX_PWM_LENGTH);
  pwmBR = constrain(pwmBR, DEFAULT_THROTTLE, MAX_PWM_LENGTH);
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

  loopTime = 1.0 / (float)ESC_REFRESH_RATE;
  loopTimeMicros = loopTime * 1e6;

  receiver_setup();
  MPU6050_setup();
  // BMP280_setup(); BMP280_calibrate();
  esc_calibrate();

  Serial.println("Last chance for safty checks, pull yaw stick left to proceed");
  finalPause();
  // MPU6050_calibrate();
  delay(1000);
  Serial.println("STARTING MOTORS, PLEASE STEP ASIDE");

  PORTB &= B11011111; //set D13 LOW
  delay(2800);
  motor_start();
  previousTime = micros();
}

void loop(){
  // for(int i = 1; i < 6; i++){ Serial.print(rxPulseLength[i]); Serial.print(" "); } Serial.println();
  if(rxPulseLength[3] < SIGNAL_LOSS_THRESHOLD) resetBoard();
  handleElectromagnet();

  // getBMPData(); getBMPPressure();

  getIMUData();
  calculateAngle();
  calculateRotationRate();

  precalculation();
  calculatePID();

  calculatePWM();
  createPWMPulse();
  // Serial.println(micros() - start);

  if(micros() - previousTime >= loopTimeMicros)
    PORTB |= B00100000; //turn on indicator light D13 if 200Hz loop is overwhelmed
  while(micros() - previousTime < loopTimeMicros); // guarantee 200Hz loop time 
  previousTime = micros();
}

//READ PWM OF 6 CHANNELS FROM RECEIVER
//=======================================================
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
//µ
