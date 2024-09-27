#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
#include "image.h"

#include <Scheduler.h>
#include <Servo.h>

#define TFT_CS 23  // Chip select
#define TFT_DC 22  // Data/command
#define TFT_BL 24  // Backlight control
#define ODOR_SENSOR A8
#define POT1 A0
#define POT2 A1
#define UP 40 //五向按键
#define DOWN 42
#define LEFT 44
#define RIGHT 46
#define MID 48
#define SET 50
#define RST 52

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
extern const unsigned char gImage_image[];
extern const unsigned char gImage_eye2[];
extern const unsigned char gImage_xiaoba[];
extern const unsigned char gImage_blue[];
extern const unsigned char gImage_green[];
extern const unsigned char gImage_eye3[];
//定义图片指针
const unsigned char* images[]={
    gImage_image,
    gImage_eye2,
    gImage_eye3,
    gImage_xiaoba,
    gImage_blue,
    gImage_green}; 
const int numImages = sizeof(images) / sizeof(images[0]);
 int currentImageIndex = 0;
//创建画布
#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 240
GFXcanvas16 demo8(240, 240);
uint16_t imageBuffer[IMAGE_WIDTH * IMAGE_HEIGHT];
//定义图片的初始坐标位置
int image_x=0;
int image_y=0;


void setup() {
  Serial.begin(115200);
  Serial.println("GC9A01A Test!");
  
  tft_left.begin();
  
#if defined(TFT_BL)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);  // Backlight on
#endif                         // end TFT_BL  

//设定五向按键所接串口为输出、上拉电阻模式
  for(int i=40;i<=52;i+=2){pinMode(i,INPUT_PULLUP);}   
  
  servo1.attach(2);
  servo2.attach(3);
  
  photoText();
  
  Scheduler.startLoop(loop2);
}

void loop(void) {
  tft_left.setRotation(3);
  //testText();
  //photoText();
  Serial.println(numImages);
  remote_test();
  //delay(5000);
 
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

  //Serial.println(angle);
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

//修改图片数组
uint16_t* change_uint16(const unsigned char image[]){
  for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++) {
    uint16_t lowByte = image[i * 2];
    uint16_t highByte = image[i * 2 + 1];
    imageBuffer[i] =  (highByte << 8) | lowByte;  // 组合两个字节为一个 16 位的值
    }
  return imageBuffer;
  }
  
unsigned long photoText(){
  tft_left.fillScreen(GC9A01A_BLACK);
  unsigned long start = micros();
  change_uint16(gImage_image);
  demo8.fillScreen(0x0000);//初始化画布为黑色背景
  tft_left.drawRGBBitmap(0, 0, imageBuffer, demo8.width(), demo8.height()); 
  // 将画布内容绘制到屏幕
  return micros() - start;
  }

  void remote_test(){
    int y=0;
    for(int i=40;i<=52;i+=2){
    if(!digitalRead(i))//被按下是低电平
    {
      if(i==52){Serial.println("RST");}
      if(i==50){Serial.println("SET");}
      if(i==48){Serial.println("MID");change_image();}
      if(i==46){Serial.println("RIGHT");y=4;move_image(y);}
      if(i==44){Serial.println("LEFT");y=3;move_image(y);}
      if(i==42){Serial.println("DOWN");y=2;move_image(y);}
      if(i==40){Serial.println("UP");y=1;move_image(y);}
    while(!digitalRead(i)){;}
  }
  delay(20);
  }
    }

  void change_image(){
    image_x=0;
    image_y=0;//清空图片位置
    tft_left.fillScreen(GC9A01A_BLACK);
    currentImageIndex=(currentImageIndex + 1)%numImages;//更新索引
    change_uint16(images[currentImageIndex]);
    tft_left.drawRGBBitmap(0, 0, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);  // 绘制图片
    }

  void move_image(int y){
    const int speed = 5; // 移动速度
    switch(y){
      case 0:break;
      case 1:image_y-= speed;break;
      case 2:image_y+= speed;break;
      case 3:image_x-= speed;break;
      case 4:image_x+= speed;break;
      }
    //tft_left.fillScreen(GC9A01A_BLACK);
    tft_left.drawRGBBitmap(image_x, image_y, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
    delay(100);
    }
