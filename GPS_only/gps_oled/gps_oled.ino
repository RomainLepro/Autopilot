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
int i = 0;
float v,vmax;
// The serial connection to the GPS module
SoftwareSerial gpsSerial(3,4);
//#define gpsSerial Serial;
TinyGPSPlus gps;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

const double HOME_LAT = 46.64968;
const double HOME_LNG = 00.36235;

void setup(){
  gpsSerial.begin(9600);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);} // Don't proceed, loop forever

}

void loop(){
  while (gpsSerial.available() > 0){
    if(gps.encode(gpsSerial.read())){
      dt = millis()-t;
      if( gps.location.lat()!=0 && millis()>3000 && dt>10){
        t = millis();
        display.clearDisplay();
        display.setTextSize(1);             // Normal 1:1 pixel scale
        display.setTextColor(WHITE);        // Draw white text
        display.setCursor(0,0);             // Start at top-left corner
        display.print(F("lat: "));display.println(gps.location.lat(),6);
        display.print(F("lng: "));display.println(gps.location.lng(),6);
        display.print(F("cor: "));display.println(gps.course.deg(),6);
        display.print(F("hom: "));
        float O = gps.courseTo(gps.location.lat(),gps.location.lng(),HOME_LAT,HOME_LNG)-gps.course.deg();
        if (O>180){O-=360;}
        else if (O<-180){O+=360;}
        display.println(O);
        display.print(F("dist: "));
        display.println(gps.distanceBetween(gps.location.lat(),gps.location.lng(),HOME_LAT,HOME_LNG));
        display.print(F("gps: "));display.println(gps.satellites.value());//display.print(F("  "));display.println(st[i]);
        v = gps.speed.mps();
        if (v>vmax){vmax = v;}
        display.print(F("spd: "));display.print(v,1);
        display.print(F("   max: "));display.print(vmax,1);
        display.display();}
      else{
        display.clearDisplay();
        display.setTextSize(1);             // Normal 1:1 pixel scale
        display.setTextColor(WHITE);        // Draw white text
        display.setCursor(0,0);             // Start at top-left corner
        display.print(F("waiting: "));display.println(millis());
         display.display();
        }
     
    }
  }
}
