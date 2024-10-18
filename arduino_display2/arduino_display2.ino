
#include <Adafruit_GFX.h>
//#include <Adafruit_GrayOLED.h>
//#include <Adafruit_SPITFT.h>
//#include <Adafruit_SPITFT_Macros.h>
//#include <gfxfont.h>
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
//#include "Arduino_GigaDisplayTouch.h" //import touchscreen library
#include <Wire.h>
//#include <Scheduler.h>
//#include <Servo.h>
#include "CST816S.h"



#define TOUCH 1
#define TFT_CS 23  // Chip select
#define TFT_DC 22  // Data/command
#define TFT_BL 24  // Backlight control
#define CTP_SDA 20
#define CTP_SCL 21
#define CTP_RST 27
#define CTP_INT 28
#define ODOR_SENSOR A8
#define POT1 A0
#define POT2 A1
#define UP 40  //五向按键
#define DOWN 42
#define LEFT 44
#define RIGHT 46
#define MID 48
#define SET 50
#define RST 52
#define TFT1_CS 31  // Chip select
#define TFT1_DC 33  // Data/command
//#define TFT1_RES 35

#define DELTA 10 // Step to move images


Adafruit_GC9A01A tft_left(TFT_CS, TFT_DC);
//Adafruit_GC9A01A tft_right(TFT1_CS, TFT1_DC);
//Arduino_GigaDisplayTouch touchDetector;//initialize touchcreen
CST816S touch(20,21,27,28);

//Servo servo1, servo2;

int odorVal = 0;
int angle = 90;
int count = 1;
int potVal1, potVal2;  // variable to read the value from the analog pin

//定义图片
extern const unsigned char gImage_eyes1[];
extern const unsigned char gImage_eyes2[];
extern const unsigned char gImage_eyes3[];
extern const unsigned char gImage_eyes4[];
extern const unsigned char gImage_eyes5[];
//extern const unsigned char gImage_blue[];
//extern const unsigned char gImage_green[];
//extern const unsigned char gImage_eye3[];
//定义图片指针
const unsigned char* images[]={
    gImage_eyes1,
    gImage_eyes2,
    gImage_eyes3,
    gImage_eyes4,
    gImage_eyes5,
  }; 
const int numImages = sizeof(images) / sizeof(images[0]);
int currentImageIndex = 0;

//创建画布
#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 240
GFXcanvas16 canvas(240, 240);
uint16_t imageBuffer[IMAGE_WIDTH * IMAGE_HEIGHT];

//定义图片的初始坐标位置
int image_x = 0;
int image_y = 0;
int detection;// select detection 2/1
int touch_x = 0;
int touch_y = 0;


void setup() {
  Serial.begin(115200);
  Serial.println("GC9A01A Test!");

  //open the screen
  tft_left.begin();
  touch.begin();
//  tft_right.begin();

  
  
#if defined(TFT_BL)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);  // Backlight on
#endif                         // end TFT_BL


  //设定五向按键所接串口为输出、上拉电阻模式
  for(int i=40;i<=52;i+=2){pinMode(i,INPUT_PULLUP);}   
//  servo1.attach(2);
//  servo2.attach(3);

  photoText();
  //Scheduler.startLoop(loop2);
}

void loop(void) {
//  tft_left.setRotation(0);
//  tft_right.setRotation(0);
  
  //testText();
  //photoText();
  //Serial.println(numImages);
  //remote_test();

    #if defined(TOUCH)//处理触摸事件
//        int touch_x ,touch_y;
//        uint8_t contacts;//check touch events
//        GDTpoint_t points[5];//save touchpoints(the maximum number of touchpoints is 5)
//        contacts = touchDetector.getTouchPoints(points);//get touch events
//        if(contacts > 0)
//        {
//          touch_x = points[0].x;//only record the first touchpoint
//          touch_y = points[0].y;
//        }
     if(touch.available()){
      touch_x=touch.data.x;
      touch_y=touch.data.y;
      }
        image_x = touch_x - (IMAGE_WIDTH / 2);//calculate the center point
        image_y = touch_y - (IMAGE_HEIGHT / 2);
        Serial.print("Touch X: ");
        Serial.print(touch_x);
        Serial.print(" Touch Y: ");
        Serial.println(touch_y);
        tft_left.fillScreen(GC9A01A_BLACK);
        tft_left.drawRGBBitmap(image_x, image_y, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
        
        delay(100);
  #endif
  
}

//void loop2() {  
//  #if defined(TOUCH)//处理触摸事件
////        int touch_x ,touch_y;
////        uint8_t contacts;//check touch events
////        GDTpoint_t points[5];//save touchpoints(the maximum number of touchpoints is 5)
////        contacts = touchDetector.getTouchPoints(points);//get touch events
////        if(contacts > 0)
////        {
////          touch_x = points[0].x;//only record the first touchpoint
////          touch_y = points[0].y;
////        }
//     if(touch.available()){
//      touch_x=touch.data.x;
//      touch_y=touch.data.y;
//      }
//        image_x = touch_x - (IMAGE_WIDTH / 2);//calculate the center point
//        image_y = touch_y - (IMAGE_HEIGHT / 2);
//        Serial.print("Touch X: ");
//        Serial.print(touch_x);
//        Serial.print(" Touch Y: ");
//        Serial.println(touch_y);
//        tft_left.fillScreen(GC9A01A_BLACK);
//        tft_left.drawRGBBitmap(image_x, image_y, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
//        
//        delay(100);
//  #endif
//}

//修改图片数组
uint16_t* change_uint16(const unsigned char image[]) {
  for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++) {
    uint16_t lowByte = image[i * 2];
    uint16_t highByte = image[i * 2 + 1];
    imageBuffer[i] = (highByte << 8) | lowByte;  // 组合两个字节为一个 16 位的值
  }
  return imageBuffer;
}

unsigned long photoText() {
  //tft_left.fillScreen(GC9A01A_BLACK);
  //tft_right.fillScreen(GC9A01A_BLACK);
  unsigned long start = micros();
  
  change_uint16(gImage_eyes1);
  //demo8.fillScreen(0x0000);//初始化画布为黑色背景
  canvas.fillScreen(0x0000);  //初始化画布为黑色背景
  
  tft_left.drawRGBBitmap(0, 0, imageBuffer, canvas.width(), canvas.height());
//tft_right.drawRGBBitmap(0, 0, imageBuffer, canvas.width(), canvas.height());

  return micros() - start;
}
