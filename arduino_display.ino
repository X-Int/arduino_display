#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"

#include <Scheduler.h>
#include <Servo.h>
#include <Arduino.h>

#define TFT_CS 23  // Chip select
#define TFT_DC 22  // Data/command
#define TFT_BL 24  // Backlight control
#define ODOR_SENSOR A8
#define POT1 A0
#define POT2 A1

// Display constructor for primary hardware SPI connection -- the specific
// pins used for writing to the display are unique to each board and are not
// negotiable. "Soft" SPI (using any pins) is an option but performance is
// reduced; it's rarely used, see header file for syntax if needed.
Adafruit_GC9A01A tft_left(TFT_CS, TFT_DC);

Servo servo1, servo2;

int odorVal = 0;
int angle = 90;
int count = 1;
int potVal1, potVal2;  // variable to read the value from the analog pin


//定义图片
extern const unsigned char gImage[];
#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 240
GFXcanvas16 demo8(240, 240);//创建画布
uint16_t imageBuffer[IMAGE_WIDTH * IMAGE_HEIGHT];

void setup() {
  Serial.begin(115200);
  Serial.println("GC9A01A Test!");
  tft_left.begin();

#if defined(TFT_BL)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);  // Backlight on
#endif                         // end TFT_BL
             
  servo1.attach(2);
  servo2.attach(3);

  Scheduler.startLoop(loop2);
}

void loop(void) {
  tft_left.setRotation(3);
  //testText();
  photoText();
  delay(5000);
}

void loop2() {
  if (angle >= 180) {
    count = -1;
  } else if (angle <= 90) {
    count = 1;
  }

  angle += 1 * count;

  servo1.write(angle);
  delay(20);  // waits for the servo to get there

  servo2.write(angle);
  delay(20);

  Serial.println(angle);
}

unsigned long testFillScreen() {
  unsigned long start = micros();
  tft_left.fillScreen(GC9A01A_BLACK);
  yield();
  tft_left.fillScreen(GC9A01A_RED);
  yield();
  tft_left.fillScreen(GC9A01A_GREEN);
  yield();
  tft_left.fillScreen(GC9A01A_BLUE);
  yield();
  tft_left.fillScreen(GC9A01A_BLACK);
  yield();
  return micros() - start;
}

unsigned long testText() {
  tft_left.fillScreen(GC9A01A_BLACK);
  unsigned long start = micros();
  tft_left.setCursor(50, 80);
  tft_left.setTextColor(GC9A01A_WHITE);
  tft_left.setTextSize(2);
  tft_left.println("Hello Idiot!");
  tft_left.println();

  tft_left.setCursor(60, 120);
  tft_left.setTextColor(GC9A01A_GREEN);
  tft_left.setTextSize(4);
  tft_left.println("Smell");

  odorVal = analogRead(ODOR_SENSOR);
  tft_left.setCursor(80, 160);
  tft_left.setTextColor(GC9A01A_RED);
  tft_left.setTextSize(4);
  tft_left.println(odorVal);

  
  return micros() - start;
}


unsigned long photoText(){
  tft_left.fillScreen(GC9A01A_BLACK);
  unsigned long start = micros();

  for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++) {
    imageBuffer[i] = (gImage[i * 2] << 8) | gImage[i * 2 + 1];  // 组合两个字节为一个 16 位的值
    }
  Serial.println(imageBuffer);
  demo8.fillScreen(0x0000);//初始化画布为黑色背景
  demo8.setRotation(1);

  
  demo8.drawBitmap(0,0,imageBuffer,240,240);  
  
  tft_left.drawRGBBitmap(0, 0, demo8.getBuffer(), demo8.width(), demo8.height()); 
  // 将画布内容绘制到屏幕
  return micros() - start;
  }
