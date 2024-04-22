#include <SPFD5408_Adafruit_GFX.h>    // Core graphics library
#include <SPFD5408_Adafruit_TFTLCD.h> // Hardware-specific library
#include <SPFD5408_TouchScreen.h>
#include <SoftwareSerial.h>
#include <SimpleTimer.h>
#include "DHT.h"
//----------------------------------------

//----------------------------------------
#if defined(__SAM3X8E__)
#undef __FlashStringHelper::F(string_literal)
#define F(string_literal) string_literal
#endif
//----------------------------------------

SoftwareSerial atmega(0, 1);

SimpleTimer timer, timer2, timer3;
String str;
// When using the BREAKOUT BOARD only, use these 8 data lines to the LCD:
// For the Arduino Uno, Duemilanove, Diecimila, etc.:
//   D0 connects to digital pin 8  (Notice these are
//   D1 connects to digital pin 9   NOT in order!)
//   D2 connects to digital pin 2
//   D3 connects to digital pin 3
//   D4 connects to digital pin 4
//   D5 connects to digital pin 5
//   D6 connects to digital pin 6
//   D7 connects to digital pin 7

// For the Arduino Mega, use digital pins 22 through 29
// (on the 2-row header at the end of the board).
//   D0 connects to digital pin 22
//   D1 connects to digital pin 23
//   D2 connects to digital pin 24
//   D3 connects to digital pin 25
//   D4 connects to digital pin 26
//   D5 connects to digital pin 27
//   D6 connects to digital pin 28
//   D7 connects to digital pin 29

// For the Arduino Due, use digital pins 33 through 40
// (on the 2-row header at the end of the board).
//   D0 connects to digital pin 33
//   D1 connects to digital pin 34
//   D2 connects to digital pin 35
//   D3 connects to digital pin 36
//   D4 connects to digital pin 37
//   D5 connects to digital pin 38
//   D6 connects to digital pin 39
//   D7 connects to digital pin 40

//----------------------------------------Pin configuration and initialization for Touchscreen
#define YP A1 //--> must be an analog pin, use "An" notation!
#define XM A2 //--> must be an analog pin, use "An" notation!
#define YM 7  //--> can be a digital pin
#define XP 6  //--> can be a digital pin

TouchScreen ts = TouchScreen(XP, YP, XM, YM, 300);
//----------------------------------------

//----------------------------------------Pin configuration and initialization for LCD TFT
#define LCD_CS A3
#define LCD_CD A2
#define LCD_WR A1
#define LCD_RD A0
// optional
#define LCD_RESET A4

Adafruit_TFTLCD tft(LCD_CS, LCD_CD, LCD_WR, LCD_RD, LCD_RESET);
//----------------------------------------

//----------------------------------------Defines colors
// Assign human-readable names to some common 16-bit color values:
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF
#define ORANGE      0xFD20
#define DARKORANGE  0xFB60
#define MAROON      0x7800
#define BLACKM      0x18E3
//----------------------------------------

//----------------------------------------The results of the x and y point calibration on the touch screen
#define TS_MINX 158
#define TS_MINY 127
#define TS_MAXX 895
#define TS_MAXY 872
//----------------------------------------

//----------------------------------------Variable for detecting touch screen when touched
#define MINPRESSURE 10
#define MAXPRESSURE 1000
//----------------------------------------

//----------------------------------------Defines an LED pin
#define LED 12
//----------------------------------------

//----------------------------------------Button location at point x and y
int BtnGreenX = 10;
int BtnGreenY = 60;
int BtnRedX = 100;
int BtnRedY = 60;
//----------------------------------------

//----------------------------------------DHT11 configuration and data variables
#define DHTPIN A5
#define DHTTYPE DHT11
DHT dht11(DHTPIN, DHTTYPE);
//----------------------------------------

//----------------------------------------The x and y points for the Temperature bar
int x_bar_t = 20;
int y_bar_t = 60;
//----------------------------------------
//----------------------------------------The variable to hold the conversion value from the temperature value to the value for the temperature bar
int T_to_Bar;
//----------------------------------------
//----------------------------------------The x and y points for the WaterLevel bar
int x_bar_w = 20;
int y_bar_w = 60;
//----------------------------------------
//----------------------------------------The variable to hold the conversion value from the WaterLevel value to the value for the WaterLevel bar
int W_to_Bar;
//----------------------------------------

//----------------------------------------The x and y points for the Soil bar
int x_bar_s = 20;
int y_bar_s = 60;
//----------------------------------------
//----------------------------------------The variable to hold the conversion value from the Soil value to the value for the Soil bar
//int S_to_Bar;
//----------------------------------------

//----------------------------------------Menu = 0 to display the Main Menu Display, Menu = 1 to control the LED and Menu = 2 to display DHT11 sensor data
int Menu = 0;
//----------------------------------------

//----------------------------------------Variable for the x, y and z points on the touch screen
int TSPointZ;
int x_set_location_135;
int y_set_location_135;
//----------------------------------------

//----------------------------------------Millis variable to update the temperature and humidity values
unsigned long previousMillis = 0; //--> will store last time updated
// constants won't change:
const long interval = 2000; //--> interval
//----------------------------------------
//----------------------------------------watersensor
int val = 0;
int analogPin = 0;
int led = 13;
//----------------------------------------


//========================================================================VOID SETUP()
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  atmega.begin(9600);
  tft.reset();

  tft.begin(0x9341); //--> SPFD5408/ILI9341

  tft.setRotation(135); //--> Need for the Mega, please changed for your choice or rotation initial

  tft.fillScreen(WHITE); //--> TFT LCD background color

  pinMode(13, OUTPUT);

  pinMode(LED, OUTPUT);

  dht11.begin();
  int Uart(int num);
  Menu_display();
  timer.setInterval(1000);
  timer2.setInterval(1000);
  timer3.setInterval(1000);
}
//========================================================================

//========================================================================VOID LOOP()
void loop() {
  // put your main code here, to run repeatedly:
  //----------------------------------------Main Menu Display
  if (Menu == 0) {
    GetTSPoint();

    //----------------------------------------Conditions for detecting touch screen when touched
    if (TSPointZ > MINPRESSURE && TSPointZ < MAXPRESSURE) {

      //----------------------------------------Conditions for detecting when the Button for controlling the LED is touched and its command (Enter the LED controlling menu)
      if (x_set_location_135 > 17 && x_set_location_135 < (17 + 280) && y_set_location_135 > 40 && y_set_location_135 < (40 + 40)) {
        Menu = 1;
        DrawButtonControlLEDPress();
        delay(100);
        DrawButtonControlLED();
        delay(100);
        tft.fillScreen(WHITE);
        delay(100);
        DrawButtonGreen(BtnGreenX, BtnGreenY);
        DrawButtonRed(BtnRedX, BtnRedY);

        tft.setTextSize(2);
        tft.setTextColor(BLUE);
        tft.setCursor(50, 15);
        tft.print("LED Level Sensor");

        if (digitalRead(LED) == 0) {
          tft.setTextSize(3);
          tft.setTextColor(BLUE, WHITE);
          tft.setCursor(40, 150);
          tft.print("LED OFF");
        } else if (digitalRead(LED) == 1) {
          tft.setTextSize(3);
          tft.setTextColor(BLUE, WHITE);
          tft.setCursor(40, 150);
          tft.print("LED ON");
        }
        DrawButtonBack(8, 6);
      }
      //----------------------------------------

      //----------------------------------------Condition to detect when the button to display DHT11 sensor data is touched and the command (Enter the menu displays DHT11 sensor data)
      if (x_set_location_135 > 17 && x_set_location_135 < (17 + 280) && y_set_location_135 > 90 && y_set_location_135 < (90 + 40)) {
        Menu = 2;
        DrawButtonTempHumPress();
        delay(100);
        DrawButtonTempHum();
        delay(100);
        tft.fillScreen(WHITE);
        delay(100);
        tft.drawLine(15, 40, 300, 40, MAROON);
        tft.drawLine(15, 39, 300, 39, MAROON);
        tft.setTextSize(2);
        tft.setTextColor(BLUE);
        tft.setCursor(40, 13);
        tft.print("Temperature & Humidity");
        tft.fillRect(170, 60, 130, 27, CYAN);
        tft.setTextSize(2);
        tft.setTextColor(WHITE);
        tft.setCursor(185, 65);
        DrawButtonBack(8, 6);
        delay(1000);
      }
      //----------------------------------------
      //----------------------------------------Condition to detect when the button to display SoilData Sensor data is touched and the command (Enter the menu displays SoilData sensor data)
      if (x_set_location_135 > 17 && x_set_location_135 < (17 + 280) && y_set_location_135 > 140 && y_set_location_135 < (140 + 40)) {
        Menu = 3;
        DrawButtonSoilDataPress();
        delay(100);
        DrawButtonSoilData();
        delay(100);
        tft.fillScreen(WHITE);
        delay(100);
        tft.drawLine(15, 40, 300, 40, MAROON);
        tft.drawLine(15, 39, 300, 39, MAROON);

        tft.setTextSize(2);
        tft.setTextColor(BLUE);
        tft.setCursor(40, 13);
        tft.print("SoilData Sensor");
        tft.fillRect(202, 60, 100, 27, CYAN);
        tft.setTextSize(2);
        tft.setTextColor(WHITE);
        tft.setCursor(205, 65);
        tft.print("Humidity");
        DrawButtonBack(8, 6);
        delay(50);
      }
      //----------------------------------------
      //----------------------------------------Condition to detect when the button to display WaterLevelData is touched and the command (Enter the menu displays SoilData sensor data)
      if (x_set_location_135 > 17 && x_set_location_135 < (17 + 280) && y_set_location_135 > 190 && y_set_location_135 < (190 + 40)) {
        Menu = 4;
        DrawButtonWaterLevelDataPress();
        delay(100);
        DrawButtonWaterLevelData();
        delay(100);
        tft.fillScreen(WHITE);
        delay(100);
        tft.drawLine(15, 40, 300, 40, MAROON);
        tft.drawLine(15, 39, 300, 39, MAROON);
        tft.setTextSize(2);
        tft.setTextColor(BLUE);
        tft.setCursor(65, 13);
        tft.print("Water Level Sensor");
        tft.fillRect(150, 60, 150, 27, CYAN);
        tft.setTextSize(2);
        tft.setTextColor(WHITE);
        tft.setCursor(200, 65);
        tft.print("Level");
        DrawButtonBack(20, 6);
        delay(50);
      }
      //----------------------------------------
    }
    //----------------------------------------
  }
  //----------------------------------------

  //----------------------------------------Menu or Mode to control the LED
  if (Menu == 1) {
    ControlTheLED();
  }
  //----------------------------------------

  //----------------------------------------Menu or Mode to display DHT11 sensor data
  if (Menu == 2) {
    //    randomSeed(analogRead(0));
    ShowDHT11Data();
  }
  //----------------------------------------
  //----------------------------------------Menu or Mode to display Soil sensor data
  if (Menu == 3) {
    SoilData();
  }
  //----------------------------------------
  //----------------------------------------Menu or Mode to display WaterLevelsensor data
  if (Menu == 4) {
    WaterLevelData();
  }
  //----------------------------------------
}
//========================================================================

//========================================================================GetTSPoint()
void GetTSPoint() {
  digitalWrite(13, HIGH);
  TSPoint p = ts.getPoint();
  digitalWrite(13, LOW);
  // if sharing pins, you'll need to fix the directions of the touchscreen pins
  pinMode(XM, OUTPUT);
  pinMode(YP, OUTPUT);
  p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.height());
  p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.width());
  y_set_location_135 = map(p.x, 240, 0, 0, tft.height());
  x_set_location_135 = map(tft.width() - p.y, 360, 0, 0, tft.width());
  TSPointZ = p.z;
}
//========================================================================

//========================================================================DrawButtonControlLED()
void DrawButtonControlLED() {
  tft.fillRoundRect(17, 40, 280, 40, 10, BLACKM);
  tft.fillRoundRect(19, 42, 276, 36, 10, GREEN);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(65, 53);
  tft.print("LED Level Sensor");
}
//========================================================================

//========================================================================DrawButtonControlLEDPress()
void DrawButtonControlLEDPress() {
  tft.fillRoundRect(17, 40, 280, 40, 10, BLACKM);
}
//========================================================================

//========================================================================DrawButtonTempHum()
void DrawButtonTempHum() {
  tft.fillRoundRect(17, 90, 280, 40, 10, BLACKM);
  tft.fillRoundRect(19, 92, 276, 36, 10, BLUE);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(25, 103);
  tft.print("Temperature & Humidity");
}
//========================================================================

//========================================================================DrawButtonTempHumPress()
void DrawButtonTempHumPress() {
  tft.fillRoundRect(17, 90, 280, 40, 10, BLACKM);
}
//========================================================================DrawButtonSoilData()
void DrawButtonSoilData() {
  tft.fillRoundRect(17, 140, 280, 40, 10, BLACKM);
  tft.fillRoundRect(19, 142, 276, 36, 10, ORANGE);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(90, 153);
  tft.print("Soil Sensor");
}
//========================================================================

//========================================================================DrawButtonSoilDataPress()
void DrawButtonSoilDataPress() {
  tft.fillRoundRect(17, 140, 280, 40, 10, BLACKM);
}
//========================================================================

//========================================================================DrawButtonWaterLevelData()
void DrawButtonWaterLevelData() {
  tft.fillRoundRect(17, 190, 280, 40, 10, BLACKM);
  tft.fillRoundRect(19, 192, 276, 36, 10, CYAN);
  tft.setTextSize(2);
  tft.setTextColor(WHITE);
  tft.setCursor(50, 203);
  tft.print("Water Level Sensor");
}
//========================================================================

//========================================================================DrawButtonWaterLevelDataPress()
void DrawButtonWaterLevelDataPress() {
  tft.fillRoundRect(17, 190, 280, 40, 10, BLACKM);
}
//========================================================================
//========================================================================DrawButtonGreen(x, y)
void DrawButtonGreen(int xp, int yp) {
  tft.fillRoundRect(xp, yp, 84, 84, 10, BLACKM);
  tft.fillRoundRect(xp + 2, yp + 2, 80, 80, 10, GREEN);
  tft.setCursor(xp + 22, yp + 32);
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.println("ON");
}
//========================================================================

//========================================================================DrawButtonGreenPress(x, y)
void DrawButtonGreenPress(int xp, int yp) {
  tft.fillRoundRect(xp, yp, 84, 84, 10, BLACKM);
}
//========================================================================

//========================================================================DrawButtonRed(x, y)
void DrawButtonRed(int xp, int yp) {
  tft.fillRoundRect(xp, yp, 84, 84, 10, BLACKM);
  tft.fillRoundRect(xp + 2, yp + 2, 80, 80, 10, RED);
  tft.setCursor(xp + 18, yp + 32);
  tft.setTextSize(3);
  tft.setTextColor(WHITE);
  tft.println("OFF");
}
//========================================================================

//========================================================================DrawButtonRedPress(x, y)
void DrawButtonRedPress(int xp, int yp) {
  tft.fillRoundRect(xp, yp, 84, 84, 10, BLACKM);
}
//========================================================================

//========================================================================DrawButtonBack(x, y)
void DrawButtonBack(int x_btn_back, int y_btn_back) {
  tft.fillRoundRect(x_btn_back, y_btn_back, 30, 30, 5, BLACKM);
  tft.fillRoundRect(x_btn_back + 2, y_btn_back + 2, 26, 26, 5, YELLOW);
  tft.setTextSize(2);
  tft.setTextColor(BLACKM);
  tft.setCursor(x_btn_back + 7, y_btn_back + 7);
  tft.print("<");
}
//========================================================================

//========================================================================DrawButtonBackPress(x, y)
void DrawButtonBackPress(int x_btn_back, int y_btn_back) {
  tft.fillRoundRect(x_btn_back, y_btn_back, 30, 30, 5, BLACKM);
}

//========================================================================draw_bar_water (waterlevel Bar)
void draw_bar_water(int x_bar2, int y_bar2) {
  tft.drawLine(x_bar2 + 20, y_bar2 + 30, x_bar2 + 20, y_bar2 + 70, BLACK);
  tft.drawLine(x_bar2 + 20, y_bar2 + 70, x_bar2 + 40, y_bar2 + 70, BLACK);
  tft.drawLine(x_bar2 + 40, y_bar2 + 70, x_bar2 + 40, y_bar2 + 30, BLACK);
}
//========================================================================

//========================================================================Menu_display()
void Menu_display() {
  tft.setTextSize(3);
  tft.setTextColor(BLACK);
  tft.setCursor(120, 7);
  tft.print("MENU");

  DrawButtonControlLED();
  DrawButtonTempHum();
  DrawButtonSoilData();
  DrawButtonWaterLevelData();
}
//========================================================================

//========================================================================ControlTheLED()
void ControlTheLED() {
  static int c;
  if (TSPointZ > MINPRESSURE && TSPointZ < MAXPRESSURE) {
    if (x_set_location_135 > BtnGreenX && x_set_location_135 < (BtnGreenX + 84) && y_set_location_135 > BtnGreenY && y_set_location_135 < (BtnGreenY + 84)) {
      tft.setTextSize(3);
      tft.setTextColor(BLUE, WHITE);
      tft.setCursor(40, 150);
      tft.print("LED ON ");

      digitalWrite(LED, HIGH);

      DrawButtonGreenPress(BtnGreenX, BtnGreenY);
      delay(100);
      DrawButtonGreen(BtnGreenX, BtnGreenY);
    }
    if (x_set_location_135 > BtnRedX && x_set_location_135 < (BtnRedX + 84) && y_set_location_135 > BtnRedY && y_set_location_135 < (BtnRedY + 84)) {
      tft.setTextSize(3);
      tft.setTextColor(BLUE, WHITE);
      tft.setCursor(40, 150);
      tft.print("LED OFF");
      digitalWrite(LED, LOW);
      DrawButtonRedPress(BtnRedX, BtnRedY);
      delay(100);
      DrawButtonRed(BtnRedX, BtnRedY);
    }
    unsigned long currentMillis = millis();
    c = Uart(0) / 10;
    tft.fillRect(195, 60, 120, 27, RED);
    tft.setTextSize(2);
    tft.setTextColor(WHITE);
    tft.setCursor(205, 65);
    tft.print("Lux Level");
    DrawButtonBack(8, 6);
    tft.setTextColor(ORANGE, WHITE);
    tft.setCursor(230, 110);
    tft.print(c);
    if (c < 10) tft.print(" ");
    tft.setCursor(250, 110);
    tft.print((char)247);
    tft.println("%");
    if (x_set_location_135 > 8 && x_set_location_135 < (8 + 30) && y_set_location_135 > 6 && y_set_location_135 < (6 + 30)) {
      Menu = 0;
      DrawButtonBackPress(8, 6);
      delay(100);
      DrawButtonBack(8, 6);
      delay(100);
      tft.fillScreen(WHITE);
      delay(100);
      Menu_display();
    }
  }
  GetTSPoint();
}
//========================================================================

//========================================================================ShowDHT11Data()
void ShowDHT11Data() {
  static int h;
  static int t;
  unsigned long currentMillis = millis();
  t = Uart(2);
  tft.setTextColor(ORANGE, WHITE);
  tft.setCursor(50, 100);
  tft.print(t);
  if (t < 10) tft.print(" ");
  tft.setCursor(80, 100);
  tft.print((char)247);
  tft.println("C");
  h = Uart(3);
  tft.setTextSize(2);
  tft.setTextColor(CYAN, WHITE);
  tft.setCursor(205, 95);
  tft.print(h);
  tft.print(" %");
  if (TSPointZ > MINPRESSURE && TSPointZ < MAXPRESSURE) {
    if (x_set_location_135 > 8 && x_set_location_135 < (8 + 30) && y_set_location_135 > 6 && y_set_location_135 < (6 + 30)) {
      Menu = 0;
      DrawButtonBackPress(8, 6);
      delay(100);
      DrawButtonBack(8, 6);
      delay(100);
      tft.fillScreen(WHITE);
      delay(100);
      Menu_display();
    }
  }
  GetTSPoint();
}
//========================================================================SoilData()
void SoilData() {
  static int s;
  GetTSPoint();
  if (timer2.isReady() == 1)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
    }
    s = Uart(1) / 5;
    tft.setTextSize(2);
    tft.setTextColor(CYAN, WHITE);
    tft.setCursor(205, 95);
    tft.print(s);
    tft.print(" %");
    timer2.reset();
  }
  if (TSPointZ > MINPRESSURE && TSPointZ < MAXPRESSURE)
  {
    if (x_set_location_135 > 8 && x_set_location_135 < (8 + 30) && y_set_location_135 > 6 && y_set_location_135 < (6 + 30))
    {
      Menu = 0;
      DrawButtonBackPress(8, 6);
      delay(100);

      DrawButtonBack(8, 6);
      delay(100);
      tft.fillScreen(WHITE);
      delay(100);
      Menu_display();
    }
  }
}

//========================================================================
void WaterLevelData() {
  static int w;
  w = Uart(4) / 10;
  W_to_Bar = map(w, 0, 100, 100, 0);//map(value, fromLow,fromHigh,toLow,toHigh) - 매핑값,현재범위 하한값, 현재범위 상한값, 매핑하려는 범위 하한값, 매핑하려는 범위 상한값
  tft.fillRect(x_bar_w + 20, y_bar_w + 20, 56, W_to_Bar, WHITE);
  tft.fillRect(x_bar_w + 20, (y_bar_w + 20) + W_to_Bar, 50, 100 - W_to_Bar , BLUE);
  GetTSPoint();
  timer2.setInterval(100);
  if (timer2.isReady() == 1)
  {
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval)
    {
      // save the last time you blinked the LED
      previousMillis = currentMillis;
    }
    tft.setTextSize(2);
    tft.setTextColor(CYAN, WHITE);
    tft.setCursor(205, 95);
    tft.print(w);
    tft.print(" %");
    if (w < 30) {
      tft.setCursor(160, 150);
      tft.setTextSize(3);
      tft.setTextColor(RED);
      tft.print("WARNING");
    }
    else {
      tft.setCursor(160, 150);
      tft.setTextSize(3);
      tft.setTextColor(WHITE);
      tft.print("WARNING");
    }
    timer2.reset();
  }
  if (TSPointZ > MINPRESSURE && TSPointZ < MAXPRESSURE)
  {
    if (x_set_location_135 > 20 && x_set_location_135 < (20 + 30) && y_set_location_135 > 6 && y_set_location_135 < (6 + 30))
    {
      Menu = 0;
      DrawButtonBackPress(20, 6);
      delay(100);
      DrawButtonBack(20, 6);
      delay(100);
      tft.fillScreen(WHITE);
      delay(100);
      Menu_display();
    }
  }
}
int Uart(int num)
{
  static String inString;
  int inString2;
  int R[5];
  inString2 = atmega.read();
  if (inString2 == 45)
  {
    inString = atmega.readStringUntil('_');
  }
  int index1 = inString.indexOf('/');
  int index2 = inString.indexOf('/', index1 + 1);
  int index3 = inString.indexOf('/', index2 + 1);
  int index4 = inString.indexOf('/', index3 + 1);

  R[0] = inString.substring(0, index1).toInt();
  R[1] = inString.substring(index1 + 1, index2).toInt();
  R[2] = inString.substring(index2 + 1, index3).toInt();
  R[3] = inString.substring(index3 + 1, index4).toInt();
  R[4] = inString.substring(index4 + 1, inString.length()).toInt();

  return R[num];
}
//========================================================================

//<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
