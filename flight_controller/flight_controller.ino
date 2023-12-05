#include<Servo.h>
#include "mpu6050_functions.h"
//#include "bmp280_functions.h"

//-----(RECEIVER) PIN CHANGE INTERRUPT--------
byte lastState[5]; //roll - pitch - throttle - yaw
volatile unsigned long rxCurrentTime = 0;
volatile unsigned long rxRiseEdge[5] = {0};
int rxPulseLength[5];
bool rxSignalLoss = false;
void rx_setup(){
  PCICR |= B00000010; //enable PCIE1 Bit 1 (PORT C)
  PCMSK1 |= B00001111; //enable PCINT8 -> PCINT11 (A0 -> A3)
}
//---------------------------------

#define BOARD2_ADDRESS 31 //slave
#define MPU6050_CALIBRATE_PIN 2
#define ANGLE_BALANCE_BOUNDARY 13
#define ALPHA_EXPONENTIAL_FILTER 0.7
#define SIGNAL_LOSS_THRESHOLD 930
#define ESC_REFRESH_RATE 250.0 //Hz

bool motorCalibrated;
float loopTime;
int loopTimeMicros, loopCal;
unsigned long previousTime;
unsigned long escLoopTime, riseEdgeTime;
unsigned long fallEdgeTimeFL, fallEdgeTimeFR, fallEdgeTimeBL, fallEdgeTimeBR;
float pitch, roll, yaw;
float defaultThrottle = 1100;
float desiredPitch, desiredRoll, desiredThrottle, desiredYaw;
float errorPitch, prevErrorPitch,
      errorRoll, prevErrorRoll,
      errorYaw, prevErrorYaw;

float p_adjust, i_adjust, d_adjust;
// #define KP_pitch 1.1//0.8
// #define KI_pitch 0.2//0.15
// #define KD_pitch 0.45//0.63
float KP_pitch = 0.2;
float KI_pitch = 0.05;
float KD_pitch = 0.2;
float p_pitch, i_pitch, d_pitch, pid_pitch;
// #define KP_roll 1.1//1.0
// #define KI_roll 0.2//0.15
// #define KD_roll 0.45//0.45
float KP_roll = KP_pitch;
float KI_roll = KI_pitch;
float KD_roll = KD_pitch;
float p_roll, i_roll, d_roll, pid_roll;

float KP_yaw = 1.0;
float KI_yaw = 0.05;
float KD_yaw = 0.0;
float p_yaw, i_yaw, d_yaw, pid_yaw;

Servo motorFrontLeft, motorFrontRight, motorBackLeft, motorBackRight;
float pwmFL, pwmFR, pwmBL, pwmBR;
#define FRONT_LEFT_MOTOR_PIN 10
#define FRONT_RIGHT_MOTOR_PIN 9
#define BACK_LEFT_MOTOR_PIN 11
#define BACK_RIGHT_MOTOR_PIN 6

void(* resetBoard) (void) = 0;

void getDesiredState(){
  if(rxPulseLength[3] < SIGNAL_LOSS_THRESHOLD) resetBoard();
  // for(int i = 1; i < 5; i++){ Serial.print(rxPulseLength[i]); Serial.print(" "); } Serial.println(" ");

  //Boundaries:
  // pitch, roll -> [-20 ; 20] degree
  // throtle -> [0; 900] µs
  if(abs(rxPulseLength[2] - 1500) < ANGLE_BALANCE_BOUNDARY)
    desiredPitch = 0.0;
  else{
    desiredPitch = -0.04016 * rxPulseLength[2] + 60.31691;
    //desiredPitch = ALPHA_EXPONENTIAL_FILTER * desiredPitch + (1.0 - ALPHA_EXPONENTIAL_FILTER) * pitch; //Exponential Filter
  }
  if(abs(rxPulseLength[1] - 1500) < ANGLE_BALANCE_BOUNDARY)
    desiredRoll = 0.0;
  else{
    desiredRoll = 0.04016 * rxPulseLength[1] - 60.31691;
    //desiredRoll = ALPHA_EXPONENTIAL_FILTER * desiredRoll + (1.0 - ALPHA_EXPONENTIAL_FILTER) * roll; //Exponential Filter
  }
  if(abs(rxPulseLength[4] - 1500) < ANGLE_BALANCE_BOUNDARY)
    desiredYaw = 0.0;
  else{
    desiredYaw = 0.04016 * rxPulseLength[4] - 60.31691;
    //desiredYaw = ALPHA_EXPONENTIAL_FILTER * desiredYaw + (1.0 - ALPHA_EXPONENTIAL_FILTER) * roll; //Exponential Filter
  }
  desiredThrottle = 0.66 * rxPulseLength[3] - 600.0;
}

void get_PID_Adjustment(){
  uint32_t receiveByte[12], sameBinary;
  Wire.requestFrom(BOARD2_ADDRESS, 12);
  for(uint32_t &i : receiveByte) i = Wire.read();
  sameBinary =
    (receiveByte[0] << 24)|
    (receiveByte[1] << 16)|
    (receiveByte[2] << 8)|
    receiveByte[3];
  p_adjust = *(float*)&sameBinary;
  sameBinary =
    (receiveByte[4] << 24)|
    (receiveByte[5] << 16)|
    (receiveByte[6] << 8)|
    receiveByte[7];
  i_adjust = *(float*)&sameBinary;
  sameBinary =
    (receiveByte[8] << 24)|
    (receiveByte[9] << 16)|
    (receiveByte[10] << 8)|
    receiveByte[11];
  d_adjust = *(float*)&sameBinary;

  // KP_pitch = p_adjust; KP_roll = p_adjust;
  // KI_pitch = i_adjust; KI_roll = i_adjust;
  // KD_pitch = d_adjust; KD_roll = d_adjust;
  //Serial.print(p_adjust); Serial.print(" "); Serial.print(i_adjust); Serial.print(" "); Serial.print(d_adjust); Serial.print(" ");
}

void calculatePID(){
  //------PITCH------
  errorPitch = pitch - desiredPitch;
  p_pitch = KP_pitch * errorPitch;
  i_pitch += KI_pitch * (errorPitch + prevErrorPitch) * loopTime * 0.5;
  d_pitch = KD_pitch * (errorPitch - prevErrorPitch) / loopTime;
  pid_pitch = p_pitch + i_pitch + d_pitch;
  if(pid_pitch > 400) pid_pitch = 400;
  else if(pid_pitch < -400) pid_pitch = -400;
  prevErrorPitch = errorPitch;

  //------ROLL------
  errorRoll = roll - desiredRoll;
  p_roll = KP_roll * errorRoll;
  i_roll += KI_roll * (errorRoll + prevErrorRoll) * loopTime * 0.5;
  d_roll = KD_roll * (errorRoll - prevErrorRoll) / loopTime;
  pid_roll = p_roll + i_roll + d_roll;
  if(pid_roll > 400) pid_roll = 400;
  if(pid_roll < -400) pid_roll = -400;
  prevErrorRoll = errorRoll;
  //-----YAW-----

}

void calculatePWM(){
  pwmFL = defaultThrottle + desiredThrottle - pid_pitch - pid_roll;
  pwmFR = defaultThrottle + desiredThrottle - pid_pitch + pid_roll;
  pwmBL = defaultThrottle + desiredThrottle + pid_pitch - pid_roll;
  pwmBR = defaultThrottle + desiredThrottle + pid_pitch + pid_roll;
  // Serial.print(pwmFL); Serial.print(' '); Serial.print(pwmFR); Serial.print(' '); Serial.print(pwmBL); Serial.print(' '); Serial.println(pwmBR);
}

void motor_setup(){
  //motorFrontLeft.attach(FRONT_LEFT_MOTOR_PIN, 1e3, 2e3); motorFrontRight.attach(FRONT_RIGHT_MOTOR_PIN, 1e3, 2e3); motorBackLeft.attach(BACK_LEFT_MOTOR_PIN, 1e3, 2e3); motorBackRight.attach(BACK_RIGHT_MOTOR_PIN, 1e3, 2e3);
  //motorFrontLeft.writeMicroseconds(1e3); motorFrontRight.writeMicroseconds(1e3); motorBackLeft.writeMicroseconds(1e3); motorBackRight.writeMicroseconds(1e3);

  DDRD |= B01000000; // Declare D6 output
  DDRB |= B00001110; // Declare D9 D10 D11 output
  int writingTime = 2;
  for(loopCal = 0; loopCal < (ESC_REFRESH_RATE * writingTime); loopCal++){ // Continuously write 1000µs
      PORTD |= B01000000; 
      PORTB |= B00001110;
      delayMicroseconds(1000);
      PORTD &= B10111111; 
      PORTB &= B11110001;
      delayMicroseconds(3000);
  }
}

void createPWMPulse(){
  // motorFrontLeft.writeMicroseconds(pwmFL); motorFrontRight.writeMicroseconds(pwmFR); motorBackLeft.writeMicroseconds(pwmBL); motorBackRight.writeMicroseconds(pwmBR);
  PORTD |= B01000000;
  PORTB |= B00001110;
  riseEdgeTime = micros(); 
  fallEdgeTimeFL = riseEdgeTime + pwmFL;
  fallEdgeTimeFR = riseEdgeTime + pwmFR;
  fallEdgeTimeBL = riseEdgeTime + pwmBL;
  fallEdgeTimeBR = riseEdgeTime + pwmBR;
 
  while(((PORTD & B01000000) != 0) || ((PORTB & B00001110) != 0)){ //check binary values at specified indexes if they all aren't zeros
    escLoopTime = micros();   
    if(escLoopTime >= fallEdgeTimeFL) PORTB &= B11111011; //set D10 LOW
    if(escLoopTime >= fallEdgeTimeFR) PORTB &= B11111101; //set D9 LOW
    if(escLoopTime >= fallEdgeTimeBL) PORTB &= B11110111; //set D11 LOW
    if(escLoopTime >= fallEdgeTimeBR) PORTD &= B10111111; //set D6 LOW
  }
}

void motor_optional_calibrate(){
  motorCalibrated = true;
  unsigned long startTime = micros();
  while(true){
    rxCurrentTime = micros();
    if(PINC & B00000100){ //Channel 3
      if(lastState[3] == 0){ lastState[3] = 1; rxRiseEdge[3] = rxCurrentTime; }
    }
    else if(lastState[3] == 1){
      lastState[3] = 0;
      rxPulseLength[3] = rxCurrentTime - rxRiseEdge[3];
    }
    if(rxPulseLength[3] > 1800) break; // Pull throttle stick to max for esc calibration
    if(micros() > startTime + 5e6) return; //if wait for longer than 5 seconds -> no calibration
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

void setup(){
  Serial.begin(19200);
  Wire.begin();
  TWBR = 12; //set 400kHz I2C clock frequency
  DDRB |= B00100000; //Declare D13 output
  PORTB |= B00100000; //set D13 HIGH
  
  loopTime = (1.0 / ESC_REFRESH_RATE);
  loopTimeMicros = loopTime * 1e6;

  rx_setup();
  motor_optional_calibrate();
  MPU6050_setup();
  delay(300);

  while(true){
    if(PINC & B00001000){
      if(lastState[4] == 0){ lastState[4] = 1; rxRiseEdge[4] = rxCurrentTime; }
    }else if(lastState[4] == 1){
      lastState[4] = 0;
      if(rxCurrentTime - rxRiseEdge[4] < 1200) break; //Pull yaw stick to the left to start IMU calibration
    }
  }
  MPU6050_calibrate();

  PORTB &= B11011111; //set D13 LOW
  delay(3000);
  motor_setup();
  previousTime = micros();
}

unsigned long something;

void loop(){
  getDesiredState();
  // get_PID_Adjustment();
  getIMUData();
  applyComplementaryFilter();
  calculatePID();
  calculatePWM();
  createPWMPulse();
  
  if(micros() - previousTime >= loopTimeMicros) // if 0.004s loop is overwhelmed
    PORTB |= B00100000; //turn on indicator light D13
  while(micros() - previousTime < loopTimeMicros); // guarantee 250Hz loop time 
  previousTime = micros();
}

//Read PWMs from receiver
ISR(PCINT1_vect){
  rxCurrentTime = micros();
  if(PINC & B00000001){  //Channel 1
    if(lastState[1] == 0){
      lastState[1] = 1;
      rxRiseEdge[1] = rxCurrentTime;
    }
  }else if(lastState[1] == 1){
    lastState[1] = 0;
    rxPulseLength[1] = rxCurrentTime - rxRiseEdge[1];
  }
  if(PINC & B00000010){ //Channel 2
    if(lastState[2] == 0){
      lastState[2] = 1;
      rxRiseEdge[2] = rxCurrentTime;
    }
  }else if(lastState[2] == 1){
    lastState[2] = 0;  
    rxPulseLength[2] = rxCurrentTime - rxRiseEdge[2];
  }
  if(PINC & B00000100){ //Channel 3
    if(lastState[3] == 0){
      lastState[3] = 1;
      rxRiseEdge[3] = rxCurrentTime;
    }
  }
  else if(lastState[3] == 1){
    lastState[3] = 0;
    rxPulseLength[3] = rxCurrentTime - rxRiseEdge[3];
  }
  if(PINC & B00001000){ //Channel 4
    if(lastState[4] == 0){
      lastState[4] = 1;
      rxRiseEdge[4] = rxCurrentTime;
    }
  }else if(lastState[4] == 1){
    lastState[4] = 0;
    rxPulseLength[4] = rxCurrentTime - rxRiseEdge[4];
  }
}
//µ
