/*
 * Rui Santos 
 * Complete Project Details https://randomnerdtutorials.com
 */
 #include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

int t,dt;

// The serial connection to the GPS module
//SoftwareSerial gpsSerial(3,4);
//#define gpsSerial Serial;
TinyGPSPlus gps;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const double HOME_LAT = 46.64968;
const double HOME_LNG = 0.36235;

void setup(){
  Serial.begin(9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);} // Don't proceed, loop forever

}

void loop(){
  while (Serial.available() > 0){
    if(gps.encode(Serial.read())){
      dt = millis()-t;
      if(dt>110){
        t = millis();
        display.clearDisplay();
        display.setTextSize(1);             // Normal 1:1 pixel scale
        display.setTextColor(WHITE);        // Draw white text
        display.setCursor(0,0);             // Start at top-left corner
        display.print(F("lat : "));display.println(gps.location.lat(),8);
        display.print(F("lng : "));display.println(gps.location.lng(),8);
        display.print(F("cor : "));display.println(gps.course.deg(),8);
        display.print(F("gps : "));display.println(gps.satellites.value());
        display.print(F("tim : "));display.println(gps.time.value());
        display.print(F("spd : "));display.println(gps.speed.mps());
        display.print(F("hom : "));
        display.println(gps.courseTo(gps.location.lat(),gps.location.lng(),HOME_LAT,HOME_LNG));

        
        
        display.display();}
    }
  }
}
