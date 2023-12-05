
volatile unsigned long currentTime = 0;
volatile unsigned long ultrasonicRiseEdge = 0;
byte lastState;
int ultrasonicPulseLength;
void ultrasonic_setup(){
  PCICR |= B00000100; //enable PCIE2 Bit 1 (PORT D)
  PCMSK2 |= B10000000;
}

const int trigPin = 8;
const int echoPin = 7;
// defines variables

float distance; // meters

void setup() {
  DDRB |= B00000001;
  ultrasonic_setup();
  Serial.begin(19200);
}

void loop() {
  //unsigned long int start = micros();
  PORTB |= B00000001;
  delayMicroseconds(10);
  PORTB &= B11111110;
  if(ultrasonicPulseLength != 0)
    distance = 0.5 * 340 * (float)(ultrasonicPulseLength / 1e6);
  
  Serial.print(ultrasonicPulseLength);
  Serial.print(" ");
  Serial.println(distance, 5);
}

ISR(PCINT2_vect){
  currentTime = micros();
  if(PIND & B10000000){
    if(lastState == 0){
      lastState = 1;
      ultrasonicRiseEdge = currentTime;
    }
  }else if(lastState == 1){
    lastState = 0;
    ultrasonicPulseLength = currentTime - ultrasonicRiseEdge;
  }
}
