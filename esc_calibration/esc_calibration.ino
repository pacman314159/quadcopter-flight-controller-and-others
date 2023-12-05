int start;

void setup(){
  Serial.begin(19200);
  Serial.println(start);  
}

void loop(){
  start = 1;
  Serial.println(start);  
  delay(500);
}
