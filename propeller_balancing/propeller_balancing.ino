#include "motor_manipulation.h"
#include "mpu6050_functions.h"

const int ESC_REFRESH_RATE = 250, //Hz
          SIGNAL_LOSS_THRESHOLD = 930,
          NUMBER_LOOP_SKIPS = 300; // when changing motors, a specified number of loop must be skipped

//-----(RECEIVER) PIN CHANGE INTERRUPT--------
byte rxLastState[7]; //roll - pitch - throttle - yaw - channel5 - channel6
volatile unsigned long rxCurrentTime = 0;
volatile unsigned long rxRiseEdge[7] = {0};
int rxPulseLength[7];
void receiver_setup(){ //pin change interruption
  PCICR |= B00000111; //enable PORT B, C, D
  PCMSK0 |= B00010000; //enable PCINT4 (D12) (channel 6) (PORT B)
  PCMSK1 |= B00001111; //enable PCINT8 -> PCINT11 (A0 -> A3) (channel 1 -> 4) (PORT C)
  PCMSK2 |= B00001000; //enable D3 (channl 5) (PORT D)
}
//-------------------------------------------

float loopTime;
int loopTimeMicros;
unsigned long previousTime;

float pwmFL, pwmFR, pwmBL, pwmBR;

byte data, rawData;

int loopSkip;

ThreeDimensions acc;
const int NUM_ACCELEROMETER_SAMPLES = 20;
float normAcc, normAcc_array[20];
int normAcc_index;
float normAcc_sum, normAcc_avg;

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

void(* resetBoard) (void) = 0;

void setup(){
  Serial.begin(19200);
  Wire.begin();
  TWBR = 12; //set 400kHz I2C clock frequency
  DDRB |= B00100000; PORTB |= B00100000; //Declare D13 output then set HIGH

  loopTime = 1.0 / (float)ESC_REFRESH_RATE;
  loopTimeMicros = loopTime * 1e6;

  receiver_setup();
  MPU6050_setup();

  Serial.println("Last chance for safty checks, pull yaw stick left to proceed");
  finalPause();
  Serial.println("STARTING MOTORS, PLEASE STEP ASIDE");

  PORTB &= B11011111; //set D13 LOW
  delay(2500);
  motor_start();
  previousTime = micros();
}

void loop(){
  if(rxPulseLength[3] < SIGNAL_LOSS_THRESHOLD) resetBoard();

  if(Serial.available() > 0){
    pwmFL = pwmFR = pwmBL = pwmBR = 1000;
    createPWMPulse();
    rawData = Serial.read();
    if(rawData != 10) data = rawData;
    switch(data){
      case '1': Serial.println("Test motor front left"); break;
      case '2': Serial.println("Test motor front right"); break;
      case '3': Serial.println("Test motor back left"); break;
      case '4': Serial.println("Test motor back right"); break;
    }
    //start loop skipping
    loopSkip = 1;
  }else if((not loopSkip) and data){
    getIMUData();

    normAcc = sqrt(acc.x*acc.x + acc.y*acc.y + acc.z*acc.z);
    normAcc_sum -= normAcc_array[normAcc_index];
    normAcc_array[normAcc_index] = normAcc;  
    normAcc_sum += normAcc_array[normAcc_index];
    normAcc_avg = normAcc_sum / NUM_ACCELEROMETER_SAMPLES;
    if(normAcc_index == NUM_ACCELEROMETER_SAMPLES - 1) normAcc_index = 0;
    else ++normAcc_index;

    Serial.println(normAcc_avg * 100);

    switch(data){
      case '1': pwmFL = rxPulseLength[3]; break;
      case '2': pwmFR = rxPulseLength[3]; break;
      case '3': pwmBL = rxPulseLength[3]; break;
      case '4': pwmBR = rxPulseLength[3]; break;
    } 
    createPWMPulse();
  }
  //increase until the value is reached
  if(loopSkip > 0) ++loopSkip;
  if(loopSkip == NUMBER_LOOP_SKIPS) loopSkip = 0;

  if(micros() - previousTime >= loopTimeMicros) PORTB |= B00100000; //turn on indicator light D13 if 250Hz loop is overwhelmed
  while(micros() - previousTime < loopTimeMicros); // guarantee 250Hz loop time 
  previousTime = micros();
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
