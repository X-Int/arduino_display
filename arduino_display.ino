#include <Adafruit_GFX.h>
#include <Adafruit_GrayOLED.h>
#include <Adafruit_SPITFT.h>
#include <Adafruit_SPITFT_Macros.h>
#include <gfxfont.h>
#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
#include <Wire.h>
#include <Scheduler.h>
//#include <Servo.h>
#include <cst816t.h>
#include <math.h>

#define TOUCH 1
#define TFT_CS 23  // 23 Chip select
#define TFT_DC 22  // 22 Data/command
#define TFT_BL 24  //24  Backlight control
#define TFT_RST 13
#define CTP_SDA 9
#define CTP_SCL 8
#define CTP_RST RESET //RESET
#define CTP_INT 25
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
#define TFT1_DC 22  // Data/command
//#define TFT1_RES 35
#define DELTA 10 // Step to move images

// Display constructor for primary hardware SPI connection -- the specific
// pins used for writing to the display are unique to each board and are not
// negotiable. "Soft" SPI (using any pins) is an option but performance is
// reduced; it's rarely used, see header file for syntax if needed.

Adafruit_GC9A01A tft_left(TFT_CS, TFT_DC);
Adafruit_GC9A01A tft_right(TFT1_CS, TFT_DC);
//两个显示屏除了cs引脚不同以外，其他的引脚均保持一致（dc rst都一致）
//触屏CTP_RST 与tft1/2_RST也可以接在一起 也可以不接一起--->接开发板的RESET

//Servo servo1, servo2;
int odorVal = 0;
int angle = 90;
int count = 1;
int potVal1, potVal2;  // variable to read the value from the analog pin

//定义图片
// extern const unsigned char gImage_eyes1[];
// extern const unsigned char gImage_eyes2[];
// extern const unsigned char gImage_eyes3[];
// extern const unsigned char gImage_eyes4[];
// extern const unsigned char gImage_eyes5[];
//extern const unsigned char gImage_blue[];
//extern const unsigned char gImage_green[];
//extern const unsigned char gImage_eye3[];
extern const unsigned char gImage_neweye[];
extern const unsigned char gImage_tears[];
//定义图片指针
const unsigned char* images[]={
    // gImage_eyes1,
    // gImage_eyes2,
    // gImage_eyes3,
    // gImage_eyes4,
    // gImage_eyes5,
    gImage_neweye,
    gImage_tears,
  }; 
const int numImages = sizeof(images) / sizeof(images[0]);
int currentImageIndex = 0;

//创建画布
#define IMAGE_WIDTH 240
#define IMAGE_HEIGHT 240
GFXcanvas16 canvas(240, 240);
uint16_t imageBuffer[IMAGE_WIDTH * IMAGE_HEIGHT];
uint16_t moveBuffer[IMAGE_HEIGHT * IMAGE_HEIGHT];

//定义图片的初始坐标位置
int image_x = 0;
int image_y = 0;
int detection;// select detection 2/1

//define touchpoints
#define MAX_DOUBLE_TOUCH 5
TwoWire myWire(CTP_SDA, CTP_SCL);
cst816t touchpad(myWire, CTP_RST, CTP_INT);
int touch_x,touch_y;
int doubleclick_count=0;
int double_touch_x[MAX_DOUBLE_TOUCH],double_touch_y[MAX_DOUBLE_TOUCH];

void setup() {
  Serial.begin(115200);
  Serial.println("GC9A01A Test!");

  //open the screen
  tft_left.begin();
  tft_right.begin();
  tft_left.setRotation(0);
  tft_right.setRotation(0);

  pinMode(TFT_RST,OUTPUT);
  digitalWrite(TFT_RST,LOW);
  delay(10);
  digitalWrite(TFT_RST,HIGH);

  // open the touchpad
  touchpad.begin(mode_motion);
  Serial.println(touchpad.version());

#if defined(TFT_BL)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);  // Backlight on
#endif                         // end TFT_BL

  //设定五向按键所接串口为输出、上拉电阻模式
  for(int i=40;i<=52;i+=2){pinMode(i,INPUT_PULLUP);}   
  //servo1.attach(2);
  //servo2.attach(3);

  //photoText();
  Scheduler.startLoop(loop2);
}

void loop(void) {
  //tft_left.setRotation(0);
  //tft_right.setRotation(0);
  //testText();
  //photoText();
  //remote_test();
  image_resize(gImage_neweye,0.9);
}

void loop2() {  
 #if defined(TOUCH)//处理触摸事件
    if(touchpad.available())  
    {
      touch_x = touchpad.x;
      touch_y = touchpad.y;
      Serial.println(touchpad.state());
      Serial.println("the touch point is:");
      Serial.println(touch_x);
      Serial.println(touch_y);
       
      if(touchpad.gesture_id==GESTURE_DOUBLE_CLICK){
        doubleclick_gesture();
      }
    }
 #endif
}

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
  change_uint16(gImage_neweye);
  //demo8.fillScreen(0x0000);//初始化画布为黑色背景
  canvas.fillScreen(0x0000);  //初始化画布为黑色背景
  tft_left.drawRGBBitmap(0, 0, imageBuffer, canvas.width(), canvas.height());
  tft_right.drawRGBBitmap(0, 0, imageBuffer, canvas.width(), canvas.height());
  return micros() - start;
}

void remote_test() {
  int direction = 0;
  int y=0;
  for (int i = 40; i <= 52; i += 2) {
    if (!digitalRead(i))  //被按下是低电平
    {
      //delay(50);//防抖动
      if(i==52){Serial.println("RST");}
      if(i==50){Serial.println("SET");}
      if(i==48){Serial.println("MID");change_image();}
      if(i==46){Serial.println("RIGHT");y=4;move_image(y,DELTA);}
      if(i==44){Serial.println("LEFT");y=3;move_image(y,DELTA);}
      if(i==42){Serial.println("DOWN");y=2;move_image(y,DELTA);}
      if(i==40){Serial.println("UP");y=1;move_image(y,DELTA);}
    while(!digitalRead(i)){;}
  }
  delay(5);
  }
    }

void change_image() {
  image_x = 0;
  image_y = 0;  //清空图片位置
  tft_left.fillScreen(GC9A01A_BLACK);
  currentImageIndex = (currentImageIndex + 1) % numImages;  //更新索引
  change_uint16(images[currentImageIndex]);
  tft_left.drawRGBBitmap(0, 0, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);  // 绘制图片
  //tft_right.drawRGBBitmap(0, 0, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT); 
}

void move_image(int direction, const int delta) {
  switch (direction) {
    case 0: break;
    case 1: image_y -= delta; break;
    case 2: image_y += delta; break;
    case 3: image_x -= delta; break;
    case 4: image_x += delta; break;
  }
  tft_left.fillScreen(GC9A01A_BLACK);
  tft_left.drawRGBBitmap(image_x, image_y, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
  //tft_right.drawRGBBitmap(image_x, image_y, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
  delay(100);
}

void doubleclick_gesture(){
  if(doubleclick_count < MAX_DOUBLE_TOUCH)
        {
          double_touch_x[doubleclick_count] = touch_x;
          double_touch_y[doubleclick_count] = touch_y;
          doubleclick_count++;
        }
        else{
          doubleclick_count=0;
          memset(double_touch_x,0,sizeof(double_touch_x));
          memset(double_touch_y,0,sizeof(double_touch_y));
        }
        if(doubleclick_count==1){
          image_x = double_touch_x[0] - (IMAGE_WIDTH / 2);
          image_y = double_touch_y[0] - (IMAGE_HEIGHT /2);
          //tft_left.fillScreen(GC9A01A_BLACK);
          tft_left.drawRGBBitmap(image_x, image_y, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
        }
}

void image_combination(unsigned char image1[], unsigned char image2[], int change_numrows, float rotate_angle, float alpha) {
    int totalpixels = IMAGE_WIDTH * IMAGE_HEIGHT; // 总像素数
    int numpixels = IMAGE_WIDTH * change_numrows; // 移动的像素数
    uint16_t* image3 = new uint16_t[totalpixels];
    uint16_t* image4 = new uint16_t[totalpixels];
    uint16_t* rotate_image = new uint16_t[totalpixels]; // 用于存储旋转后的图像
    float rad = rotate_angle * (M_PI / 180.0); // 将角度转换为弧度
    uint16_t lowByte, highByte;
    int center_x = IMAGE_WIDTH / 2;
    int center_y = IMAGE_HEIGHT / 2;

    // 将 image1 和 image2 转换为 16 位颜色值
    for (int i = 0; i < totalpixels; i++) {
        lowByte = image1[i * 2];
        highByte = image1[i * 2 + 1];
        image3[i] = (highByte << 8) | lowByte;  // 将两个字节组合为一个 16 位的值

        lowByte = image2[i * 2];
        highByte = image2[i * 2 + 1];
        image4[i] = (highByte << 8) | lowByte;  // 将两个字节组合为一个 16 位的值
    }

    // 初始化旋转后的图像为白色
    for (int i = 0; i < totalpixels; i++) {
        rotate_image[i] = 0xFFFF; // 默认旋转图像中的像素为白色
    }
    // 旋转 image2
    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            int newX = cos(rad) * (x - center_x) - sin(rad) * (y - center_y) + center_x;
            int newY = sin(rad) * (x - center_x) + cos(rad) * (y - center_y) + center_y;

            // 检查新坐标是否在图像范围内
            if (newX >= 0 && newX < IMAGE_WIDTH && newY >= 0 && newY < IMAGE_HEIGHT) {
                rotate_image[newY * IMAGE_WIDTH + newX] = image4[y * IMAGE_WIDTH + x]; // 将旋转后的像素映射到新位置
            }
        }
    }
    // 移动图像像素
    for (int i = totalpixels - 1; i >= numpixels; i--) {
        image4[i] = rotate_image[i - numpixels];
    }
    // 将移动后的图像前部设置为白色
    for (int i = 0; i < numpixels; i++) {
        image4[i] = 0xFFFF;  // 白色
    }
    // 将两个图像组合
    for (int i = 0; i < totalpixels; i++) {
        uint8_t r1 = (image3[i] >> 11) & 0x1F;
        uint8_t g1 = (image3[i] >> 5) & 0x3F;
        uint8_t b1 = image3[i] & 0x1F;
        uint8_t r2 = (image4[i] >> 11) & 0x1F;
        uint8_t g2 = (image4[i] >> 5) & 0x3F;
        uint8_t b2 = image4[i] & 0x1F;
        // 应用 alpha 混合
        uint8_t r = (uint8_t)((r1 * (1 - alpha)) + (r2 * alpha));
        uint8_t g = (uint8_t)((g1 * (1 - alpha)) + (g2 * alpha));
        uint8_t b = (uint8_t)((b1 * (1 - alpha)) + (b2 * alpha));
        moveBuffer[i] = (r << 11) | (g << 5) | b;
    }
    //串口显示
    Serial.print("moveBuffer:\n");
    for(int i = 0; i < IMAGE_HEIGHT;i++){
      for(int j = 0;j < IMAGE_WIDTH;j++)
      {
        Serial.print(moveBuffer[j],HEX);
        Serial.print(' ');
      }
      Serial.println();
    }
    // 绘制图像
    tft_left.drawRGBBitmap(0, 0, moveBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
    tft_right.drawRGBBitmap(0, 0, moveBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
}

void image_display(int x, int y,uint16_t image[],int height,int width){
  tft_left.drawRGBBitmap(x, y, image, height, width);
  tft_right.drawRGBBitmap(x, y, image, height, width);
}

void image_resize(const unsigned char origin_image[],float beta)
{
  int x=0,y=0;
  int centre_x = IMAGE_WIDTH / 2;
  int centre_y = IMAGE_HEIGHT / 2;
  int totalpixels = IMAGE_HEIGHT * IMAGE_WIDTH;
  uint16_t* resizeBuffer = new uint16_t[totalpixels];

  for(int i=0;i<totalpixels;i++){
    resizeBuffer[i]=0;
  }
  for(int i=0; i<IMAGE_HEIGHT * beta;i++){
    for(int j=0; j<IMAGE_WIDTH * beta;j++)
    {
      // float distance,new_distance;
      // distance = sqrt(pow(abs(x-centre_x),2)+pow(abs(y-centre_y),2));
      // new_distance = distance * beta;
      // new_x = round(x * beta);
      // new_y = round(y * beta);//四舍五入取整计算新的坐标值
      if(i<IMAGE_HEIGHT && j<IMAGE_WIDTH){
        resizeBuffer[i * IMAGE_WIDTH +j] = origin_image[IMAGE_WIDTH*(int)(i/beta) + (int)(j/beta)];
      }
    }
  }
  for(int i=0;i < totalpixels;i++){
    if(resizeBuffer[i]==0){resizeBuffer[i] = 0xFFFF;}
  }
  //image_display(0,0,resizeBuffer,240,240);
  //*resizeBuffer传递数组第一个元素 resizeBuffer传递整个数组
  Serial.print("放缩成功");
  Serial.print(sizeof(resizeBuffer));
}
