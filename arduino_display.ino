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

uint16_t BilinearInterpolation(uint16_t* img,float img_x, float img_y){
  int img_x1 = int(img_x);
  int img_x2 = img_x1 + 1;
  int img_y1 = int(img_y);
  int img_y2 = img_y1 +1;

  if(img_x1 < 0 || img_y1 < 0 || img_x2 >= IMAGE_WIDTH || img_y2 >= IMAGE_WIDTH){return 0;}
  //计算插值权重
  float u = img_x - img_x1;
  float v = img_y - img_y1;
  float pm0 = (1 - u)*(1 - v);
  float pm1 = v * (1 - u);
  float pm2 = u * (1 - v);
  float pm3 = u * v;
  //获取四个像素点颜色
  uint16_t color_11 = img[img_y1 * IMAGE_WIDTH + img_x1];
  uint16_t color_12 = img[img_y1 * IMAGE_WIDTH + img_x2];
  uint16_t color_21 = img[img_y2 * IMAGE_WIDTH + img_x1];
  uint16_t color_22 = img[img_y2 * IMAGE_WIDTH + img_x2];
  //计算像素值
  // uint16_t R1 = color_11 * (1 - wx) + color_12 * wx;
  // uint16_t R2 = color_21 * (1 - wx) + color_22 * wx;
  // uint16_t color = R1 * (1 - wy) + R2 * wy;
  uint16_t R = uint16_t(((color_11 >> 11) & 0x1F)*pm0+((color_12 >> 11) & 0x1F)*pm1+((color_21 >> 11) & 0x1F)*pm2+((color_22 >> 11) & 0x1F)*pm3);
  uint16_t G = uint16_t(((color_11 >> 5) & 0x3F)*pm0+((color_12 >> 5) & 0x3F)*pm1+((color_21 >> 5) & 0x3F)*pm2+((color_22 >> 5) & 0x3F)*pm3);
  uint16_t B = uint16_t((color_11 & 0x1F)*pm0+(color_12 & 0x1F)*pm1+(color_21 & 0x1F)*pm2+(color_22 & 0x1F)*pm3);
  uint16_t color = (R << 11) | (G << 5) | B;
  return color;
}

void Rotate1Image(uint16_t img[], double Rotaryangle,double ZoomX, double ZoomY){
  /*平移、旋转和缩放图片：*/
  /*ZoomX、ZoomY为沿着X/Y轴的缩放系数*/
  /*rotaryangle为旋转角度 默认旋转方向是逆时针*/
  /*move_x move_y可以实现图片沿着x/y轴的平移(没有使用）*/
  if((fabs(ZoomX * IMAGE_WIDTH)<1.0e-4) || (fabs(ZoomY * IMAGE_HEIGHT)<1.0e-4)){return;}
  int move_x = 0;
  int move_y = 0;
  int center_x = IMAGE_WIDTH / 2;//旋转中心（x，y）
  int center_y = IMAGE_HEIGHT /2;
  int total_pixels = IMAGE_SIZE;
  float rad = Rotaryangle * (M_PI / 180.0);
  float sinA = sin(rad);
  float cosA = cos(rad);
  double rZoomX = 1.0 / ZoomX;
  double rZoomY = 1.0 / ZoomY;
  double Ax = (rZoomX * cosA); 
  double Ay = (rZoomX * sinA); 
  double Bx = (-rZoomY * sinA); 
  double By = (rZoomY * cosA);
  // 经过旋转和平移后的旋转中心
  double Cx = (-(center_x+move_x)*rZoomX*cosA+(center_y+move_y)*rZoomY*sinA+ center_x);
  double Cy = (-(center_x+move_x)*rZoomX*sinA-(center_y+move_y)*rZoomY*cosA+ center_y); 
  // 初始源图像的坐标
  double img0_x = Cx;
  double img0_y = Cy;
  // 获取旋转后的图像数组
  for (long y = 0; y < IMAGE_HEIGHT; y++) {
    double img_x_f = img0_x;
    double img_y_f = img0_y;
    for (long x = 0; x < IMAGE_WIDTH; x++) {
        int newX = int(img_x_f);//取整获得对应坐标值
        int newY = int(img_y_f);
        if (newX >= 0 && newX < IMAGE_WIDTH && newY >= 0 && newY < IMAGE_HEIGHT) {
          imageBuffer[y * IMAGE_WIDTH + x] = BilinearInterpolation(img,img_x_f,img_y_f);//二次插值获得像素值
          //imageBuffer[y * IMAGE_WIDTH + x] = img[newY * IMAGE_WIDTH + newX]; // 将旋转后的像素映射到新位置
        }
        img_x_f += Ax;
        img_y_f += Ay;
    }
    img0_x += Bx;
    img0_y += By;
  }
  tft_right.drawRGBBitmap(0, 0,imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
  tft_left.drawRGBBitmap(0, 0,imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
}

void Move1Image(uint16_t img[],int changeNumrows){
  int total_pixels = IMAGE_SIZE;
  int num_pixels = IMAGE_WIDTH * changeNumrows;
   // 确保 num_pixels 不超过总像素数
  if (num_pixels > total_pixels) {
    num_pixels = total_pixels;
  }
  memmove(&img[num_pixels], &img[0], (total_pixels - num_pixels) * sizeof(uint16_t));
  for(int i = 0;i < num_pixels;i++){
    img[i] = 0XFFFF;//white
  }
  // tft_right.drawRGBBitmap(0, 0,img, IMAGE_WIDTH, IMAGE_HEIGHT);
  // tft_left.drawRGBBitmap(0, 0, img, IMAGE_WIDTH, IMAGE_HEIGHT);
}

void rotCombine2Images(uint16_t img1[], uint16_t img2[], float angle) {
  Move1Image(img2,50);


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
  //Move1Image(tear,50);
  for(int i=5; i <= 50;i += 5){
    Rotate1Image(tear,i,0.8,0.8);
    delay(2000);
  }
}

void loop(void) {
  tft_left.setRotation(0);
  tft_right.setRotation(0);
  //rotCombine2Images(pupil, tear, 5);
  //Move1Image(tear,5);
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
