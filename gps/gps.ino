#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

#define LCD_ADDRESS 0x27
#define PAGE_DURATION 1500
#define NUM_PAGES 4

#define RX_PIN 4
#define TX_PIN 3

LiquidCrystal_I2C lcd(LCD_ADDRESS, 16, 2);
int pageNumber;
unsigned long int prevTime;

SoftwareSerial gpsSerial(RX_PIN, TX_PIN);
TinyGPSPlus gps;

double latitude, longtitude, altitude;
int day, month, year, hour, minute, second;
int numSatellite;

void displayFirstPage(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("LONGTITUDE");
  lcd.setCursor(0,1);
  lcd.print(longtitude, 6);
}

void displaySecondPage(){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("LATITUDE");
  lcd.setCursor(0,1);
  lcd.print(latitude, 6);
}

void displayThirdPage(){
  int i = 5, j = 0;
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("UTC: ");
  lcd.setCursor(i,0); lcd.print(hour < 10 ? "0" : "");
  i += hour < 10 ? 1 : 0;
  lcd.setCursor(i,0); lcd.print(hour);
  lcd.setCursor(++i,0); lcd.print(":");
  lcd.setCursor(++i,0); lcd.print(minute < 10 ? "0" : "");
  i += minute < 10 ? 1 : 0;
  lcd.setCursor(i,0); lcd.print(minute);
  lcd.setCursor(++i,0); lcd.print(":");
  lcd.setCursor(++i,0); lcd.print(second < 10 ? "0" : "");
  i += second < 10 ? 1 : 0;
  lcd.setCursor(i,0); lcd.print(second);

  lcd.setCursor(j,1); lcd.print(day < 10 ? "0" : "");
  j += day < 10 ? 1 : 0;
  lcd.setCursor(j,1); lcd.print(day);
  lcd.setCursor(++j,1); lcd.print("/");
  lcd.setCursor(++j,1); lcd.print(month < 10 ? "0" : "");
  j += month < 10 ? 1 : 0;
  lcd.setCursor(j,1); lcd.print(month);
  lcd.setCursor(++j,1); lcd.print("/");
  lcd.setCursor(++j,1); lcd.print(year);
}

void displayFourthPage(){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("SATELLITES: ");
  lcd.setCursor(12,0); lcd.print(numSatellite);
  lcd.setCursor(0,1); lcd.print("ALTITUDE: ");
  lcd.setCursor(10,1); lcd.print(altitude, 1);
  lcd.setCursor(15,1); lcd.print("m");
}

void setup(){
  Serial.begin(19200);
  gpsSerial.begin(9600);
  lcd.init();                    
  lcd.backlight();
}

void loop(){
  if(gpsSerial.available() > 0){
    /*
    if(gps.encode(gpsSerial.read())){ //decode success
      latitude = gps.location.lat(); 
      longtitude = gps.location.lng(); 
      day = gps.date.day();
      month = gps.date.month();
      year = gps.date.year();
      hour = gps.time.hour();
      minute = gps.time.minute();
      second = gps.time.second();
      numSatellite = gps.satellites.value();
      altitude = gps.altitude.meters();
    }
    */
    Serial.write(gpsSerial.read());
  }else Serial.println("??");
  /*

  if(millis() - prevTime > PAGE_DURATION){ //execute code each 5s
    prevTime = millis();
    if(pageNumber == NUM_PAGES)
      pageNumber = 1;
    else pageNumber++;

    switch(pageNumber){
      case 1: displayFirstPage(); break;
      case 2: displaySecondPage(); break;
      case 3: displayThirdPage(); break;
      case 4: displayFourthPage(); break;
    }
  }
  */
}
