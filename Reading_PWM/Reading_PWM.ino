byte last_channel[5];
volatile long current_time = 0;
volatile long timer[5] = {0};
int receiver_input[5];
void setup() {
  Serial.begin(19200);
  //enable PCIE0 Bit 1 (PORT C)
  PCICR |= (1 << PCIE1);
  //enable PCINT8 & PCINT9 & PCINT10 & PCINT11 (A0 -> A3)
  PCMSK1 |= B00001111;
}

void loop() {
  //delay(250);
  print_signals();
}

ISR (PCINT1_vect){
  current_time = micros();
  //Channel 1
  if(PINC & B00000001){
    if(last_channel[1] == 0){
      last_channel[1] = 1;
      timer[1] = current_time;
    }
  }else if(last_channel[1] == 1){
    last_channel[1] = 0;
    receiver_input[1] = current_time - timer[1];
  }
  //Channel 2
  if(PINC & B00000010){
    if(last_channel[2] == 0){
      last_channel[2] = 1;
      timer[2] = current_time;
    }
  }else if(last_channel[2] == 1){
    last_channel[2] = 0;  
    receiver_input[2] = current_time - timer[2];
  }
  //Channel 3
  if(PINC & B00000100){
    if(last_channel[3] == 0){
      last_channel[3] = 1;
      timer[3] = current_time;
    }
  }
  else if(last_channel[3] == 1){
    last_channel[3] = 0;
    receiver_input[3] = current_time - timer[3];
  }
  //Channel 4
  if(PINC & B00001000){
    if(last_channel[4] == 0){
      last_channel[4] = 1;
      timer[4] = current_time;
    }
  }else if(last_channel[4] == 1){
    last_channel[4] = 0;
    receiver_input[4] = current_time - timer[4];
  }
}
void print_signals(){
  //Serial.print("Throtte: ");
  Serial.print(receiver_input[1]);
  Serial.print(" ");
  Serial.print(receiver_input[2]);
  Serial.print(" ");
  Serial.print(receiver_input[3]);
  Serial.print(" ");
  Serial.println(receiver_input[4]);
}
