#include<Wire.h>

#define SLAVE_ADDRESS 31

#define DT 2
#define CLK 3
#define P_BUTTON 5
#define I_BUTTON 4
#define D_BUTTON 6

int currentStateCLK;
int lastStateCLK;

float pStep = 0.02, iStep = 0.02, dStep = 0.02;
float pCounter, iCounter, dCounter;
float p_adjust, i_adjust, d_adjust;

int mode = 0;

void requestEvent(){
  byte byteBreakDown[12];
  uint32_t p_adjust_binary = *(uint32_t*)&p_adjust,
           i_adjust_binary = *(uint32_t*)&i_adjust ,
           d_adjust_binary = *(uint32_t*)&d_adjust;

  byteBreakDown[0] = (p_adjust_binary >> 24) & 0xFF; //11111111 <- 0xFF
  byteBreakDown[1] = (p_adjust_binary >> 16) & 0xFF;
  byteBreakDown[2] = (p_adjust_binary >> 8) & 0xFF;
  byteBreakDown[3] = p_adjust_binary & 0xFF;

  byteBreakDown[4] = (i_adjust_binary >> 24) & 0xFF;
  byteBreakDown[5] = (i_adjust_binary >> 16) & 0xFF;
  byteBreakDown[6] = (i_adjust_binary >> 8) & 0xFF;
  byteBreakDown[7] = i_adjust_binary & 0xFF;

  byteBreakDown[8] = (d_adjust_binary >> 24) & 0xFF;
  byteBreakDown[9] = (d_adjust_binary >> 16) & 0xFF;
  byteBreakDown[10] = (d_adjust_binary >> 8) & 0xFF;
  byteBreakDown[11] = d_adjust_binary & 0xFF;

  Wire.write(byteBreakDown, 12);
}

void setup() {
  Serial.begin(19200);
  Wire.begin(SLAVE_ADDRESS);
  Wire.onRequest(requestEvent);

  pinMode(CLK,INPUT);
  pinMode(DT,INPUT);
  lastStateCLK = digitalRead(CLK);

  pinMode(P_BUTTON,INPUT);
  pinMode(I_BUTTON,INPUT);
  pinMode(D_BUTTON,INPUT);
}

void loop() {
  if(digitalRead(P_BUTTON) == HIGH) mode = 0;
  if(digitalRead(I_BUTTON) == HIGH) mode = 1;
  if(digitalRead(D_BUTTON) == HIGH) mode = 2;

  currentStateCLK = digitalRead(CLK);
  if (currentStateCLK != lastStateCLK  && currentStateCLK == 1){
    if (digitalRead(DT) != currentStateCLK) //CCW
      switch(mode){
        case 0: pCounter++; break;
        case 1: iCounter++; break;
        case 2: dCounter++; break;
      }
    else //CW
      switch(mode){
        case 0: pCounter--; break;
        case 1: iCounter--; break;
        case 2: dCounter--; break;
      }
    if(pStep < 0) pStep = 0;
    if(iStep < 0) iStep = 0;
    if(dStep < 0) dStep = 0;
    p_adjust = pStep * pCounter;
    i_adjust = iStep * iCounter;
    d_adjust = dStep * dCounter;
    Serial.print(p_adjust); Serial.print(" "); Serial.print(i_adjust); Serial.print(" "); Serial.print(d_adjust); Serial.print(" ");
  }
  lastStateCLK = currentStateCLK;
}
