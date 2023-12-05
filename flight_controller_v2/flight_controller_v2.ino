//=======================================
// CASCADED PID LOOP MODE
//=======================================
#include<Servo.h>
#include "mpu6050_functions.h"
#include "motor_manipulation.h"
#include "bmp280_functions.h"

const float ANGLE_BALANCE_BOUNDARY = 15.0;
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

bool releaseMagnet;
float loopTime;
int loopTimeMicros;
unsigned long previousTime;
float defaultThrottle = 1100, throttle, desiredThrottle;

float accX, accY, accZ;

bool altitudeHoldMode, prevAltitudeHoldState;
float pressureBMP;
float desiredPressure, errorPressure, prevErrorPressure;
const float KP_altitudeHold = 1.0;
const float KI_altitudeHold = 0.0;
const float KD_altitudeHold = 5.0;
float p_term_pressure, i_term_pressure, d_term_pressure, pid_pressure;

float pitch, roll;
float desiredPitch, desiredRoll;
float errorPitch, errorRoll;
float prevErrorPitch, prevErrorRoll;
const float KP_outer_pitch = 0.8;
const float KI_outer_pitch = 0.0;
float p_term_outer_pitch, i_term_outer_pitch, pid_outer_pitch;
const float KP_outer_roll = KP_outer_pitch;
const float KI_outer_roll = KI_outer_pitch;
float p_term_outer_roll, i_term_outer_roll, pid_outer_roll;

float ratePitch, rateRoll, rateYaw;
float desiredRatePitch, desiredRateRoll, desiredRateYaw;
float errorRatePitch, errorRateRoll, errorRateYaw;
float prevErrorRatePitch, prevErrorRateRoll, prevErrorRateYaw;
const float KP_inner_pitch = 1.2;
const float KI_inner_pitch = 0.01;
const float KD_inner_pitch = 15.0;
float p_term_inner_pitch, i_term_inner_pitch, d_term_inner_pitch, pid_inner_pitch;
const float KP_inner_roll = KP_inner_pitch;
const float KI_inner_roll = KI_inner_pitch;
const float KD_inner_roll = KD_inner_pitch;
float p_term_inner_roll, i_term_inner_roll, d_term_inner_roll, pid_inner_roll;
const float KP_yaw = 3.0;
const float KI_yaw = 0.01;
const float KD_yaw = 0.0;
float p_term_inner_yaw, i_term_inner_yaw, d_term_inner_yaw, pid_inner_yaw;

float pwmFL, pwmFR, pwmBL, pwmBR;

void(* resetBoard) (void) = 0;

void getDesiredState(){
  if(rxPulseLength[3] < SIGNAL_LOSS_THRESHOLD) resetBoard();
  //for(int i = 1; i <= 6; i++){ Serial.print(rxPulseLength[i]); Serial.print(" "); } Serial.println();
  /*
  Boundaries:
   pitch, roll -> [-20; 20] degree
   yaw -> [-20; 20] degrees/second
   throtle -> [0; 900] µs
  */
  desiredPitch = abs(rxPulseLength[2] - 1500) < ANGLE_BALANCE_BOUNDARY ?
    0.0 : -0.04016 * rxPulseLength[2] + 60.31691;
  desiredRoll = abs(rxPulseLength[1] - 1500) < ANGLE_BALANCE_BOUNDARY ?
    0.0 : -0.04016 * rxPulseLength[1] + 60.31691;
  desiredRateYaw = abs(rxPulseLength[4] - 1500) < ANGLE_BALANCE_BOUNDARY ?
    0.0 : -0.04016 * rxPulseLength[4] + 60.31691;
  desiredThrottle = 0.9 * rxPulseLength[3] - 900;
  altitudeHoldMode = rxPulseLength[6] < 1500 ? true : false;
  releaseMagnet = rxPulseLength[5] < 1100 ? true : false;
}

void calculatePID(){ //Cascade pid controller
  //==========OUTER LOOP==========
  //----------PITCH----------
  errorPitch = pitch - desiredPitch;
  p_term_outer_pitch = KP_outer_pitch * errorPitch;
  i_term_outer_pitch += KI_outer_pitch * (errorPitch + prevErrorPitch) * 0.5;
  pid_outer_pitch = p_term_outer_pitch + i_term_outer_pitch;
  prevErrorPitch = errorPitch;
  //----------ROLL----------
  errorRoll = roll - desiredRoll;
  p_term_outer_roll = KP_outer_roll * errorRoll;
  i_term_outer_roll += KI_outer_roll * (errorRoll + prevErrorRoll) * 0.5;
  pid_outer_roll = p_term_outer_roll + i_term_outer_roll;
  prevErrorRoll = errorRoll;

  //==========INNER LOOP==========
  //----------PITCH----------
  desiredRatePitch =  pid_outer_pitch;
  errorRatePitch = ratePitch - desiredRatePitch;
  p_term_inner_pitch = KP_inner_pitch * errorRatePitch;
  i_term_inner_pitch += KI_inner_pitch * (errorRatePitch + prevErrorRatePitch) * 0.5;
  d_term_inner_pitch = KD_inner_pitch * (errorRatePitch - prevErrorRatePitch);
  pid_inner_pitch = p_term_inner_pitch + i_term_inner_pitch;
  pid_inner_pitch = constrain(pid_inner_pitch, -250, 250);
  prevErrorRatePitch = errorRatePitch;
  //----------ROLL----------
  desiredRateRoll =  pid_outer_roll;
  errorRateRoll = rateRoll - desiredRateRoll;
  p_term_inner_roll = KP_inner_roll * errorRateRoll;
  i_term_inner_roll += KI_inner_roll * (errorRateRoll + prevErrorRateRoll) * 0.5;
  d_term_inner_roll = KD_inner_roll * (errorRateRoll - prevErrorRateRoll);
  pid_inner_roll = p_term_inner_roll + i_term_inner_roll + d_term_inner_roll;
  pid_inner_roll = constrain(pid_inner_roll, -250, 250);
  prevErrorRateRoll = errorRateRoll;
  //----------YAW----------
  errorRateYaw = rateYaw - desiredRateYaw;
  p_term_inner_yaw = KP_yaw * errorRateYaw;
  i_term_inner_yaw += KI_yaw * (errorRateYaw + prevErrorRateYaw) * 0.5;
  d_term_inner_yaw = KD_yaw * (errorRateYaw - prevErrorRateYaw);
  pid_inner_yaw = p_term_inner_yaw + i_term_inner_yaw + d_term_inner_yaw;
  pid_inner_yaw = constrain(pid_inner_yaw, -250, 250);
  prevErrorRateYaw = errorRateYaw;

  //=======ALTITUDE HOLD==========
  throttle = desiredThrottle;
  if(altitudeHoldMode){
    if(prevAltitudeHoldState == 0){
      desiredPressure = pressureBMP;
      prevErrorPressure = 0;
    }
    errorPressure = pressureBMP - desiredPressure;
    p_term_pressure = KP_altitudeHold * errorPressure;
    i_term_pressure += KI_altitudeHold * (errorPressure + prevErrorPressure) * 0.5;
    d_term_pressure = KD_altitudeHold * (errorPressure - prevErrorPressure);
    pid_pressure = p_term_pressure + i_term_pressure + d_term_pressure;
    throttle = desiredThrottle + pid_pressure;
    prevErrorPressure = errorPressure;
  }
  prevAltitudeHoldState = altitudeHoldMode; 
}

void calculatePWM(){
  pwmFL = defaultThrottle + throttle + pid_inner_pitch - pid_inner_roll + pid_inner_yaw;
  pwmFR = defaultThrottle + throttle + pid_inner_pitch + pid_inner_roll - pid_inner_yaw;
  pwmBL = defaultThrottle + throttle - pid_inner_pitch - pid_inner_roll - pid_inner_yaw;
  pwmBR = defaultThrottle + throttle - pid_inner_pitch + pid_inner_roll + pid_inner_yaw;
  pwmFL = max(pwmFL, defaultThrottle);
  pwmFR = max(pwmFR, defaultThrottle);
  pwmBL = max(pwmBL, defaultThrottle);
  pwmBR = max(pwmBR, defaultThrottle);
  // Serial.print(pwmFL); Serial.print(' '); Serial.print(pwmFR); Serial.print(' '); Serial.print(pwmBL); Serial.print(' '); Serial.println(pwmBR);
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
  delay(100);
  esc_calibrate();
  while(not(rxPulseLength[4] < 1150 and rxPulseLength[3] < 1150)){ //Pull throttle stick down and yaw stick left
    if(PINC & B00000100){ //Channel 3
      if(rxLastState[3] == 0){ rxLastState[3] = 1; rxRiseEdge[3] = rxCurrentTime; }
    }else if(rxLastState[3] == 1){ rxLastState[3] = 0; rxPulseLength[3] = rxCurrentTime - rxRiseEdge[3]; }
    if(PINC & B00001000){ //Channel 4
      if(rxLastState[4] == 0){ rxLastState[4] = 1; rxRiseEdge[4] = rxCurrentTime; }
    }else if(rxLastState[4] == 1){ rxLastState[4] = 0; rxPulseLength[4] = rxCurrentTime - rxRiseEdge[4]; }
  }
  MPU6050_calibrate();
  BMP280_calibrate();

  PORTB &= B11011111; //set D13 LOW
  delay(3000);
  motor_setup();
  previousTime = micros();
}

void handleElectromagnet(){
  if(releaseMagnet) PORTD &= B11011111;
  else PORTD |= B00100000;
}

void loop(){
  getDesiredState();
  getIMUData(); getCurrentAngle(); getRotationRate();
  getBMPData(); getBMPPressure();
  calculatePID(); calculatePWM();
  createPWMPulse();
  handleElectromagnet();

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
