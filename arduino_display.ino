#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
#include <Wire.h>
#include <Scheduler.h>
#include <cst816t.h>


#define TOUCH 0
#define TFT_CS 23  // 23 Chip select
#define TFT_DC 22  // 22 Data/command
#define TFT_BL 24  //24  Backlight control
#define TFT_RST 13
#define CTP_SDA 9
#define CTP_SCL 8
#define CTP_RST RESET  //RESET
#define CTP_INT 25
#define TFT1_CS 31  // Chip select
#define TFT1_DC 22  // Data/command

Adafruit_GC9A01A tft_left(TFT_CS, TFT_DC);
Adafruit_GC9A01A tft_right(TFT1_CS, TFT_DC);
//两个显示屏除了cs引脚不同以外，其他的引脚均保持一致（dc rst都一致）
//触屏CTP_RST 与tft1/2_RST也可以接在一起 也可以不接一起--->接开发板的RESET

//定义图片
extern const unsigned char gImage_pupil[];
extern const unsigned char gImage_tear[];
//定义图片指针
const unsigned char* images[] = {
  gImage_pupil,
  gImage_tear,
};

//创建画布
#define IMAGE_HEIGHT 240
#define IMAGE_WIDTH 240
#define IMAGE_SIZE IMAGE_HEIGHT* IMAGE_WIDTH
uint16_t imageBuffer[IMAGE_SIZE], pupil[IMAGE_SIZE], tear[IMAGE_SIZE];
unsigned char Moveimage[IMAGE_SIZE *2];

//定义图片的初始坐标位置
int image_x = 0;
int image_y = 0;
int detection;

//define touchpoints
#define MAX_DOUBLE_TOUCH 5
TwoWire myWire(CTP_SDA, CTP_SCL);
//TwoWire myWire2(CTP_SDA, CTP_SCL);
cst816t touchpad(myWire, CTP_RST, CTP_INT);
//cst816t touchpad2(myWIre2,CTP_RST2,CTP_INT2);
int touch_x, touch_y;
uint16_t lowByte, highByte;
int doubleclick_count = 0;
int double_touch_x[MAX_DOUBLE_TOUCH], double_touch_y[MAX_DOUBLE_TOUCH];

//define function
void doubleclick_gesture() {
  if (doubleclick_count < MAX_DOUBLE_TOUCH) {
    double_touch_x[doubleclick_count] = touch_x;
    double_touch_y[doubleclick_count] = touch_y;
    doubleclick_count++;
  } else {
    doubleclick_count = 0;
    memset(double_touch_x, 0, sizeof(double_touch_x));
    memset(double_touch_y, 0, sizeof(double_touch_y));
  }
  if (doubleclick_count == 1) {
    image_x = double_touch_x[0] - (IMAGE_WIDTH / 2);
    image_y = double_touch_y[0] - (IMAGE_HEIGHT / 2);
    //tft_left.fillScreen(GC9A01A_BLACK);
    tft_left.drawRGBBitmap(image_x, image_y, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
  }
}

//修改图片数组
void gImage2image(const unsigned char gImage[], uint16_t* image) {
  for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++) {
    lowByte = gImage[i * 2];
    highByte = gImage[i * 2 + 1];
    image[i] = (highByte << 8) | lowByte;  // 组合两个字节为一个 16 位的值
  }
}

void Move1Image(const uint16_t img[],int changeNumrows){
  int total_pixels = IMAGE_SIZE;
  int num_pixels = IMAGE_WIDTH * changeNumrows;
   // 确保 num_pixels 不超过总像素数
  if (num_pixels > total_pixels) {
    num_pixels = total_pixels;
  }
  //方法一：
  //memmove(&Moveimage[num_pixels], &img2[0], (total_pixels - num_pixels) * sizeof(uint16_t));
  //memset(Moveimage, 0xFFFF, total_pixels * sizeof(uint16_t));
  //方法二：
  for(int i = num_pixels;i < total_pixels-num_pixels ;i++){
    Moveimage[i] = img[i - num_pixels];
  }
  for(i = 0;i < num_pixels;i++){
    Move1Image[i] = 65535;//white
  }

  Serial.print("Moveimage:");
  for(int i = 0; i < IMAGE_HEIGHT ; i++){
    for(int j = 0; j < IMAGE_WIDTH; j++){
      Serial.print(Moveimage[i * changeNumrows + j]);
      Serial.print(' ');
    }
    Serial.println();
  }
  tft_right.drawRGBBitmap(0, 0,tear, IMAGE_WIDTH, IMAGE_HEIGHT);
  tft_left.drawRGBBitmap(0, 0, pupil, IMAGE_WIDTH, IMAGE_HEIGHT);
}

void rotCombine2Images(const uint16_t img1[], const uint16_t img2[], float angle) {
  //int change_num = 5;//img2向下移动行数
  int centre_x = IMAGE_WIDTH / 2;
  int centre_y = IMAGE_HEIGHT /2;
  int total_pixels = IMAGE_SIZE;
  int changeNumrows = 5;
  int num_pixels = IMAGE_WIDTH * changeNumrows;
   // 确保 num_pixels 不超过总像素数
  if (num_pixels > total_pixels) {
    num_pixels = total_pixels;
  }

  // for(int i=0;i<total_pixels;i++){
  //   Moveimage[i] = 0xFFFF;
  // }
  // Moveimage[0] = 0XFFFF;
  // Serial.println(Moveimage[0]);
  tft_right.drawRGBBitmap(0, 0, tear, IMAGE_WIDTH, IMAGE_HEIGHT);
  tft_left.drawRGBBitmap(0, 0, pupil, IMAGE_WIDTH, IMAGE_HEIGHT);
}

void setup() {
  Serial.begin(115200);
  Serial.println("GC9A01A Test!");

  //open the screen
  tft_left.begin();
  tft_right.begin();
  tft_left.setRotation(0);
  tft_right.setRotation(0);

#if defined(TFT_BL)
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);  // Backlight on
#endif

  pinMode(TFT_RST, OUTPUT);
  digitalWrite(TFT_RST, LOW);
  delay(10);
  digitalWrite(TFT_RST, HIGH);

  // open the touchpad
  touchpad.begin(mode_motion);
  Serial.println(touchpad.version());

  gImage2image(gImage_tear, tear);
  gImage2image(gImage_pupil, pupil);

  Scheduler.startLoop(loop2);

  //test MoveImage
  //rotCombine2Images(pupil, tear, 5);
  Move1Image(tear,5);
}

void loop(void) {
  tft_left.setRotation(0);
  tft_right.setRotation(0);
  rotCombine2Images(pupil, tear, 5);
  Move1Image(tear,5);
}

void loop2() {
#if defined(TOUCH)  //处理触摸事件
  if (touchpad.available()) {
    touch_x = touchpad.x;
    touch_y = touchpad.y;
    // Serial.println(touchpad.state());
    // Serial.println("the touch point is:");
    // Serial.println(touch_x);
    // Serial.println(touch_y);

    if (touchpad.gesture_id == GESTURE_DOUBLE_CLICK) {
      doubleclick_gesture();
    }
  }
#endif
}
