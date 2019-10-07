
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

#define screen true

#define analogYaw 10
#define analogPitch 11

int t,dt;
float vmax = 0;
float v = 1;
float Oz = 0.0;//yaw
float Oy = 0.0;//pitch
bool set = false;
float alt = 0.0;
float dist = 0.0;

// The serial connection to the GPS module
SoftwareSerial gpsSerial(3,4);
TinyGPSPlus gps;

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

double HOME_LAT = 46.64968;
double HOME_LNG = 00.36231;

void setup(){
  gpsSerial.begin(9600);

  if(screen && !display.begin(SSD1306_SWITCHCAPVCC, 0x3C) ) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);} // Don't proceed, loop forever
   t = millis();
}

void showScreen(){
   t = millis();
   display.clearDisplay();
   display.setTextSize(1);             // Normal 1:1 pixel scale
   display.setTextColor(WHITE);        // Draw white text
   display.setCursor(0,0);             // Start at top-left corner
   display.print(F("lat: "));display.println(gps.location.lat(),6);
   display.print(F("lng: "));display.println(gps.location.lng(),6);
   display.print(F("cor: "));display.println(gps.course.deg(),2);
   display.print(F("hom: "));display.println(Oz,2);
   display.print(F("dist: "));display.print(gps.distanceBetween(gps.location.lat(),gps.location.lng(),HOME_LAT,HOME_LNG));
   display.print(F(" alt: "));display.println(gps.altitude.meters()-alt,1);
   display.print(F("gps: "));display.print(gps.satellites.value());display.print(F("  "));display.println(set);
   if (v>vmax){vmax = v;}
   display.print(F("spd: "));display.print(v,1);
   display.print(F("   max: "));display.print(vmax,1);
   display.display();
}

void loop(){
  while (gpsSerial.available() > 0){
    gps.encode(gpsSerial.read());}
  dt = millis()-t;

  
  if(gps.location.isUpdated() && gps.location.lat()!= 0.0){
      v = gps.speed.mps();
      if(!set){
        if(v < 0.1  && millis()> 4000){ // if imobile, set home
          set = true;
          HOME_LAT = gps.location.lat();
          HOME_LNG = gps.location.lng();
          alt = gps.altitude.meters();
        }}
      else{
        dist = gps.distanceBetween(gps.location.lat(),gps.location.lng(),HOME_LAT,HOME_LNG);
        if(dist>20){ // dont turn if close to the waypoint
          Oz = gps.courseTo(gps.location.lat(),gps.location.lng(),HOME_LAT,HOME_LNG)-gps.course.deg();
          if (Oz>180){Oz-=360;}
          else if (Oz<-180){Oz+=360;}
          Oy = gps.altitude.meters()-alt+40; // fligh alt = 40 m
          if (Oy>10){Oy=10;}
          else if (Oy<-10){Oy = -10;}}
        
      }
      if(dt>10 && screen){
        t = millis();
         display.clearDisplay();
         display.setTextSize(1);             // Normal 1:1 pixel scale
         display.setTextColor(WHITE);        // Draw white text
         display.setCursor(0,0);             // Start at top-left corner
         display.print(F("lat: "));display.println(gps.location.lat(),6);
         display.print(F("lng: "));display.println(gps.location.lng(),6);
         display.print(F("cor: "));display.println(gps.course.deg(),2);
         display.print(F("hom: "));display.println(Oz,2);
         display.print(F("dist: "));display.print(gps.distanceBetween(gps.location.lat(),gps.location.lng(),HOME_LAT,HOME_LNG));
         display.print(F(" alt: "));display.println(gps.altitude.meters()-alt,1);
         display.print(F("gps: "));display.print(gps.satellites.value());display.print(F("  "));display.println(set);
         if (v>vmax){vmax = v;}
         display.print(F("spd: "));display.print(v,1);
         display.print(F("   max: "));display.print(vmax,1);
         display.display();
       }
      else if (dt>10 && !screen){
        analogWrite(analogYaw,Oz);
        analogWrite(analogPitch,Oy);
       }
    }

    
   else if(screen){
   display.clearDisplay();
   display.setTextSize(1);             // Normal 1:1 pixel scale
   display.setTextColor(WHITE);        // Draw white text
   display.setCursor(0,0);             // Start at top-left corner
   display.print(F("waiting: "));display.println(millis());
   display.display();
   }
}
