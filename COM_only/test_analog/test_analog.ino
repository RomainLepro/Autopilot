//#include <SPI.h>
//#include <Wire.h>
//#include <Adafruit_GFX.h>
//#include <Adafruit_SSD1306.h>
//#include <SoftwareSerial.h>

//SoftwareSerial gpsSerial(3,4);
//#define SCREEN_WIDTH 128 // OLED display width, in pixels
//#define SCREEN_HEIGHT 64 // OLED display height, in pixels
//#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
//Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define roll 10
int t,dt,val;


void setup(){
Serial.begin(9600);
//display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
}

void loop(){

  val += 1;
  if (val>2550)
  {val = 0;}
  Serial.println(val);
  analogWrite(roll,val/10);
  //display.clearDisplay();
  //display.setTextSize(1);             // Normal 1:1 pixel scale
  //display.setTextColor(WHITE);        // Draw white text
  //display.setCursor(0,0);             // Start at top-left corner
  //display.print(F("val : "));display.println(val);
  //display.display();
}
