//==============================================================
// Config
//==============================================================
#define PWM_PIN 21 //PWM output pin
#define INT_PIN 22 //interrupt pin
#define POLES 7
//==============================================================

#include "ESP32_PWM.h"
#include <M5Stack.h> //M5Core LCD 320x240

//fonts
#define LOAD_GFXFF
// Use these when printing or drawing text in GLCD and high rendering speed fonts
#define GLCD  0
#define GFXFF 1
#define FONT2 2
#define FONT4 4
#define FONT6 6
#define FONT7 7
#define FONT8 8
//usage
//M5.Lcd.setFreeFont(&FreeSans12pt7b);   // Select the font: see font directory 9pt 12pt 18pt 24pt
//M5.Lcd.drawString("abc123", 160, 120, GFXFF);// Print the string name of the font
/*  
#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. 15 lines x 40 chars - Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. 9 lines x 22 chars - Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
//#define LOAD_FONT8N // Font 8. Alternative to Font 8 above, slightly narrower, so 3 digits fit a 160 pixel TFT
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts
*/

PWM pwmObject; 

volatile uint32_t rpm_cnt = 0;
String rpmString; 
uint32_t interval = 500; //max display refresh rate
uint32_t interval_start = -9999999;
int lcd_header_y = 0;

int pwm = 950;
String pwmString;

void handleISR() {
  rpm_cnt++;
}

void setup_lcd() {
  M5.Lcd.fillScreen(TFT_BLACK);   
  M5.Lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  M5.Lcd.setTextSize(1);
    
  M5.Lcd.setTextDatum(TC_DATUM);
  M5.Lcd.setFreeFont(&FreeSans24pt7b);
  M5.Lcd.drawString("RPM Meter", M5.Lcd.width()/2, 0, GFXFF);   
  lcd_header_y = M5.Lcd.fontHeight();


  M5.Lcd.setFreeFont(&FreeSans18pt7b);
  int y = M5.Lcd.height();
  M5.Lcd.setTextDatum(BC_DATUM);
  M5.Lcd.drawString("DEC", M5.Lcd.width()/2, y, GFXFF); 
  M5.Lcd.setTextDatum(BR_DATUM);
  M5.Lcd.drawString("INC", M5.Lcd.width(), y, GFXFF); 
  M5.Lcd.setTextColor(TFT_RED, TFT_BLACK);   
  M5.Lcd.setTextDatum(BL_DATUM);  
  M5.Lcd.drawString("STOP", 0, y, GFXFF); 
}

void display_hw() {    
  M5.Lcd.setTextColor(TFT_WHITE, TFT_BLACK); 
  M5.Lcd.setFreeFont(&FreeSans24pt7b);
  M5.Lcd.setTextSize(2);

  String s;
  M5.Lcd.setTextDatum(MC_DATUM);  
  int x = M5.Lcd.width()/2;
  int y = 0;
 
  //RPM
  y += M5.Lcd.height()/3;
  //find max padding
  s = rpmString;
  while(M5.Lcd.textWidth(" " + s + " ") < M5.Lcd.width()) {
    s = " " + s + " ";
  }
  M5.Lcd.drawString(s, x, y); 

  //RPM
  M5.Lcd.setTextSize(1);  
  y += M5.Lcd.height()/3;
  //find max padding
  s = pwmString;
  while(M5.Lcd.textWidth(" " + s + " ") < M5.Lcd.width()) {
    s = " " + s + " ";
  }
  M5.Lcd.drawString(s, x, y);
}


void setup() {
  M5.begin();  
  M5.Power.begin(); //Init Power module.

  //LCD
  M5.Lcd.setRotation(1);  //1=buttons below screen 3=upside down 
  M5.Lcd.fillScreen(TFT_BLUE);  
  
  Serial.println("Sensor_RPM::setup");
  attachInterrupt(digitalPinToInterrupt(INT_PIN), handleISR, FALLING);
  setup_lcd();

  //setup_lcd(); 

  //pwmObject.begin(PWM_PIN,2000,125,250); //Oneshot motor ESC 2000Hz pulse 125-250 us
  pwmObject.begin(PWM_PIN,50,950,2050); //regular servo 50Hz pulse 950-2050 us
}

void loop() {
  //buttons
  M5.update();
  if(M5.BtnA.wasPressed()) {
      pwm = 950;
  }
  if(M5.BtnB.wasPressed()) {
      if(pwm>950) pwm -= 50;
  }
  if(M5.BtnC.wasPressed()) {
      if(pwm<2050) pwm += 50;
  }

  //execute every interval
  char buf[100];
  uint32_t now = millis();
  if(now - interval_start >  interval) {
    //long press buttons
    if(M5.BtnB.pressedFor(2*interval)) {
        if(pwm>950) pwm -= 50;
    }
    if(M5.BtnC.pressedFor(2*interval)) {
        if(pwm<2050) pwm += 50;
    }

     //display
    float rpm = (float)rpm_cnt / POLES * 60000 / (now - interval_start);
    interval_start = now;
    rpm_cnt = 0;
    sprintf(buf,"%.0f",rpm);
    rpmString = buf;

    sprintf(buf,"%d",pwm);
    pwmString = pwm;    

    Serial.printf("$MEAS,rpm,%s,pwm,%s\n", rpmString.c_str(), pwmString.c_str());
    display_hw();
  }

  //update pwm
  pwmObject.writeMicroseconds(pwm);
}
