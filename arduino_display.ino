#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
#include <Wire.h>
#include <Scheduler.h>
#include <cst816t.h>

#define TOUCH 0
#define FILLCOLOR 1
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
extern const unsigned char gImage_tear[];//879F FFFF
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
int positions[240][2];
uint16_t fillcolour[] = {
  0XFFFF,//白色
  0XF800,//红色
  0X07E0,//绿色
  0X001F,//蓝色
  0XFFE0,//黄色
  0X0000,//黑色
};

//定义图片的初始坐标位置
int image_x = 0;
int image_y = 0;

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
void Serprint(uint16_t img[]){
  Serial.println("IMG:");
  for(int i=0;i<IMAGE_HEIGHT;i++){
    for(int j=0; j<IMAGE_WIDTH;j++){
      Serial.print(img[i*IMAGE_WIDTH+j],HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
  Serial.println();
}

//修改图片数组
void gImage2image(const unsigned char gImage[], uint16_t* image) {
  for (int i = 0; i < IMAGE_WIDTH * IMAGE_HEIGHT; i++) {
    lowByte = gImage[i * 2];
    highByte = gImage[i * 2 + 1];
    image[i] = (highByte << 8) | lowByte;  // 组合两个字节为一个 16 位的值
  }
}

void boundaryfill(uint16_t img[]){
  // //给图像外框填充rows数量的白色
  // int rows = 3;
  // for(int i = 0; i < rows; i++){
  //   for(int j = 0; j < IMAGE_WIDTH; j++){
  //     img[i * IMAGE_WIDTH + j] = 0XFFFF;
  //     img[IMAGE_WIDTH * (IMAGE_HEIGHT -1 -i) + j] = 0XFFFF;
  //   }
  // }
  // for(int j = 0; j < rows; j++){
  //   for(int i = 0;i <IMAGE_HEIGHT; i++){
  //     img[i * IMAGE_WIDTH +j] = 0XFFFF;
  //     img[(IMAGE_WIDTH-1-j) + i * IMAGE_WIDTH] = 0XFFFF;
  //   }
  // }
  //预处理tear数组，将像素值填充为0XFFFF或者0X879F
  for(int i = 0;i<IMAGE_SIZE;i++){
    if((img[i] >= 0XB7FF)||(img[1] <= 0X4FFF)){img[i] = 0XFFFF;}
    else{img[i] =0X879F; }
  }
}

double SinXDivX(double x){
  /*计算sinc值（插值权重）*/
  const float a = -1; // a 用于调节锐化或模糊的程度
  if (x < 0) x = -x; // 将 x 转为非负值
  double x2 = x * x;
  double x3 = x2 * x;
  if (x <= 1)
    return (a+2) * x3 - (a+3) * x2 + 1;
  else if (x <= 2) 
    return a * x3 - (5 * a) * x2 + (8 * a) * x - (4 * a);
  else
  return 0;
}

uint16_t PixelBound(uint16_t img[],int x, int y){
  //限制坐标在图像范围内
  x = (x >= 0) ? ((x < IMAGE_HEIGHT) ? x : IMAGE_HEIGHT - 1) : 0;
  y = (y >= 0) ? ((y < IMAGE_WIDTH) ? y : IMAGE_WIDTH - 1) : 0;
  return img[x * IMAGE_WIDTH + y];
}

uint16_t CubicConvInterpolation(uint16_t img[],float img_x,float img_y)
{
  /*三次卷积插值：计算一次4*4范围内的三次卷积插值*/
  /*一维空间插值计算:x/x-1/x+1/x+2四个点进行加权平均：f(x)= ∑f(x+k)⋅W(x−(x+k))
    其中w（x）是卷积核
    二维三次卷积插值（分离卷积）：水平方向插值-垂直方向插值-最终插值像素值*/
  int img_x0 = int(img_x);
  if(img_x0 > img_x){--img_x0;}
  int img_y0 = int(img_y);
  if(img_y0 > img_y){--img_y0;}
  float fu = img_x - img_x0; // x 方向的小数部分
  float fv = img_y - img_y0; // y 方向的小数部分
  uint16_t pixel[16];// 存储16个像素点
  int x,y;

  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 4;j++){
      x = img_x0 -1 +j;// 横坐标
      y = img_y0 -1 +i;// 纵坐标
      pixel[i * 4 + j] = PixelBound(img,x,y);// 存储对应像素值
    }
  }
  //  三次卷积插值计算
  uint16_t R=0,G=0,B=0;
  float afu[4], afv[4];
  float sR=0,sG=0,sB=0,aR=0,aB=0,aG=0;
  afu[0] = SinXDivX(1 + fu);
  afu[1] = SinXDivX(fu);
  afu[2] = SinXDivX(1 - fu);
  afu[3] = SinXDivX(2 - fu);
  afv[0] = SinXDivX(1 + fv);
  afv[1] = SinXDivX(fv);
  afv[2] = SinXDivX(1 - fv);
  afv[3] = SinXDivX(2 - fv);
  for(int i = 0; i < 4; i++){
    aR = aG = aB = 0; // 清空横向插值累加器
    for(int j = 0; j <4;j++){
      // 计算横着三次插值
      uint16_t color = pixel[i * 4 +j];
      R = uint16_t((color >> 11) & 0x1F);
      G = uint16_t((color >> 5) & 0x3F);
      B = uint16_t(color & 0x1F);
      aR += afu[j] * R;
      aG += afu[j] * G;
      aB += afu[j] * B;
    }
    // 计算竖着的插值得到最终结果
    sR += aR * afv[i];
    sG += aG * afv[i];
    sB += aB * afv[i];
  }
  //得到最终RGB颜色
  R = uint16_t(sR > 31 ? 31 : (sR < 0 ? 0 : sR));
  G = uint16_t(sG > 63 ? 63 : (sG < 0 ? 0 : sG));
  B = uint16_t(sB > 31 ? 31 : (sB < 0 ? 0 : sB));
  return  uint16_t((R << 11) | (G << 5) | B); 
}


uint16_t BilinearInterpolation(uint16_t* img,float img_x, float img_y){
  /*二次插值获得像素点RGB值*/
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
        int newX = int(img_x_f);//取整获得对应原图像中的坐标值
        int newY = int(img_y_f);
        if (newX >= 0 && newX < IMAGE_WIDTH && newY >= 0 && newY < IMAGE_HEIGHT) {
          //imageBuffer[y * IMAGE_WIDTH + x] = BilinearInterpolation(img,img_x_f,img_y_f);//二次插值获得像素值
          //imageBuffer[y * IMAGE_WIDTH + x] = img[newY * IMAGE_WIDTH + newX]; // 将旋转后的像素映射到新位置
          imageBuffer[y * IMAGE_WIDTH + x] = CubicConvInterpolation(img,img_x_f,img_y_f);
        }
        else if(FILLCOLOR){
            //图片缩小周围会出现一圈黑色，使用fillcolour填充不同的颜色进去
          imageBuffer[y * IMAGE_WIDTH + x] = fillcolour[0];//填充白色
        }
        img_x_f += Ax;
        img_y_f += Ay;
    }
    img0_x += Bx;
    img0_y += By;
  }
  //填补出现的0(由于缩小使得边界出现一串0)
  for(int i=0;i<total_pixels;i++){
    if(imageBuffer[i] == 0){
      imageBuffer[i] = fillcolour[0];
      }
  }
  // for(int i=0; i< total_pixels;i++){
  //   img[i] = imageBuffer[i];//把旋转后的图像数组重新放回源数组中
  // }
  // boundaryfill(img);

}

void Move1Image(uint16_t img[],int changeNumrows){
  /*将输入的图片向下移动changeNumrows行*/
  int total_pixels = IMAGE_SIZE;
  int num_pixels = IMAGE_WIDTH * abs(changeNumrows);
   // 确保 num_pixels 不超过总像素数
  if (num_pixels > total_pixels) {
    num_pixels = total_pixels;
  }
  if(changeNumrows >0){
    memmove(&imageBuffer[num_pixels], &img[0], (total_pixels - num_pixels) * sizeof(uint16_t));
    for(int i = 0;i < num_pixels;i++){
      imageBuffer[i] = 0XFFFF;//white
  }
  }else if(changeNumrows < 0){
    memmove(&imageBuffer[0], &img[num_pixels], (total_pixels - num_pixels) * sizeof(uint16_t));
    // 填充下方空白部分为白色
    for (int i = total_pixels - num_pixels; i < total_pixels; i++) {
      imageBuffer[i] = 0xFFFF; // white
    }
  }
  boundaryfill(imageBuffer);
}

void rotCombine2Images(uint16_t img1[], uint16_t img2[]) {
  /*图像融合*/
  for(int i=0; i < IMAGE_SIZE;i++){
    if(img2[i]== 0XFFFF || img2[i]== 0XFFDF ||img2[i]== 0XFF7F){
      imageBuffer[i] = img1[i];
    }
    else{
      imageBuffer[i] = img2[i];
    }
  }
  GaussBLur(imageBuffer,1.1,3);
  tft_right.drawRGBBitmap(0, 0, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
  tft_left.drawRGBBitmap(0, 0, imageBuffer, IMAGE_WIDTH, IMAGE_HEIGHT);
}

void GaussBLur(uint16_t img[],double sigma,int radius){
  /*高斯平滑*/
  /*sigma：高斯标准差,调整模糊程度 radius：高斯核半径图像质量调整*/
  /*radius=0则等于没做*/
  double *gaussMatrix,*pdbl;//  指向高斯核矩阵 使用pdb1遍历matrix
  double gaussSum = 0.0;//  存储高斯和
  double _2sigma2 = 2 * sigma * sigma;
  int x, y, xx, yy, xxx, yyy;
  double a,d;
  gaussMatrix =  new double[(radius * 2 + 1) * (radius * 2 + 1) * sizeof(double)];
  pdbl = gaussMatrix;
    if (!gaussMatrix) {
        return;
    }
  //  计算高斯核矩阵
  for (y = -radius; y <= radius; y++) {
    for (x = -radius; x <= radius; x++) {
      a = exp(-(double)(x * x + y * y) / _2sigma2);
      *pdbl++ = a;//  当前地址赋值并指向下一个地址
      gaussSum += a;
    }
  }
  //  归一化高斯核矩阵
  pdbl = gaussMatrix;
  for (y = -radius; y <= radius; y++) {
    for (x = -radius; x <= radius; x++) {
      *pdbl++ /= gaussSum;
    }
  }
  //  卷积计算
   for (y = 0; y < IMAGE_HEIGHT; y++) {   
        for (x = 0; x < IMAGE_WIDTH; x++) {    
            double r = 0.0, g = 0.0, b = 0.0;
            pdbl = gaussMatrix;   
            for (yy = -radius; yy <= radius; yy++) {   
                yyy = y + yy;// 当前y坐标位置计算   
                if (yyy >= 0 && yyy < IMAGE_HEIGHT) {   
                    for (xx = -radius; xx <= radius; xx++) {   
                        xxx = x + xx;// 当前x坐标计算   
                        if (xxx >= 0 && xxx < IMAGE_WIDTH) {   
                            uint16_t bbb = img[yyy * IMAGE_WIDTH + xxx];// 获得颜色值   
                            d = *pdbl;   
                            r += ((bbb >> 11) & 0x1F) * d;
                            g += ((bbb >> 5) & 0x3F) * d;
                            b += (bbb & 0x1F) * d;
                        }   
                        pdbl++;   
                    }   
                } else {   
                    pdbl += (radius * 2 + 1);   
                  }   
            }
            uint16_t R = uint16_t(r > 31 ? 31 : (r < 0 ? 0 : r));
            uint16_t G = uint16_t(g > 63 ? 63 : (g < 0 ? 0 : g));
            uint16_t B = uint16_t(b > 31 ? 31 : (b < 0 ? 0 : b));
            imageBuffer[y * IMAGE_WIDTH + x] = (R << 11) | (G << 5) | B;
          }
    }
  delete[] gaussMatrix; 
}

void Display2Image(uint16_t img1[],uint16_t img2[]){
  int move_rows = 140;
  Rotate1Image(img1,0,0.9,0.9);
  for(int i = 0;i < IMAGE_SIZE;i++){
    img1[i] = imageBuffer[i];
  }
  Rotate1Image(img2,0,0.8,0.8);// 缩小后的结果放在imagebufffer中
  for(int i = 0;i < IMAGE_SIZE;i++){
    img2[i] = imageBuffer[i];
      //  转移到image2上，每次旋转以img2作为旋转源图转不同的角度会比在imagebuffer上累加旋转
  }
  for(int i=10;i<=100;i+=10){
    Rotate1Image(img2,i,1,1);// 旋转 结果放入imagebuffer
    Move1Image(imageBuffer,move_rows);//  向下移动100行放入imagebuffer
    rotCombine2Images(img1,imageBuffer);//  叠加

  }
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
  boundaryfill(tear);//修改为只有两种颜色
  
  Scheduler.startLoop(loop2);
  //test
  Display2Image(pupil,tear);
}

void loop(void) {
  tft_left.setRotation(0);
  tft_right.setRotation(0);
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
