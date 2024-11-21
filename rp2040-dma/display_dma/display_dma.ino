#include <TFT_eSPI.h>//引入espi库

//定义屏幕大小
#define IMAGE_WIDTH  240
#define IMAGE_HEIGHT 240 //输入rotate半张图片
#define BUFFER_HEIGHT 60
//分成四部分传输
#define IMAGE_SIZE IMAGE_WIDTH*IMAGE_HEIGHT
#define BUFFER_SIZE BUFFER_HEIGHT * IMAGE_WIDTH
#define FILLCOLOR 1

//引入图片
#include "pupil_0.h"
uint16_t* imgPtr;
uint16_t fillcolour[] = {
  0XFFFF,  //白色
  // 0XF800,  //红色
  // 0X07E0,  //绿色
  // 0X001F,  //蓝色
  // 0XFFE0,  //黄色
  // 0X0000,  //黑色
};
//设定缩放比例
double resize = 1;

//初始化屏幕：定义两个屏幕 双屏显示
#define firstScreenCS 20
#define secondScreenCS 22
TFT_eSPI  tft = TFT_eSPI();
//创建DMA双缓冲区：
// TFT_eSprite spr[2] = {TFT_eSprite(&tft), TFT_eSprite(&tft)};
// uint16_t* sprPtr[2];//指针数组
// TFT_eSprite sprite[1] = {TFT_eSprite(&tft)};
// uint16_t* spr;
uint16_t renderbuf[2][60*240];
uint16_t* bufPtr;

//计算帧率：
uint16_t counter = 0;
int32_t startMillis = millis();
uint16_t interval = 100;
String fps = "xx.xx fps";
/*
initDMA():初始化DMA引擎使其附加到SPI总线上
deInitDMA():反初始化 DMA 引擎并将其从 SPI 总线上分离
pushImageDMA(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t* data, uint16_t* buffer = nullptr)
  :将图像数据传输到频幕中 data：所需传输图像指针 buffer：缓冲区确保传输过程中图像内容不被修改
*/

void setup(){
  Serial.begin(115200);
  //初始化两块屏幕
  tft_begin();
  //初始化DMA及bg
  digitalWrite(firstScreenCS, LOW);
  tft.setRotation(1); // 横屏模式Landscape orientation, USB at bottom right
  tft.setSwapBytes(true);//取消字节交换
  //tft.fillScreen(BGCOLOR);
  tft.initDMA();//初始化DMA
  tft.fillScreen(TFT_BLACK);
  digitalWrite(firstScreenCS, HIGH);

  digitalWrite(secondScreenCS, LOW);
  tft.setRotation(3); // 横屏模式Landscape orientation, USB at bottom right
  tft.setSwapBytes(true);//字节交换
  //tft.fillScreen(BGCOLOR);
  tft.initDMA();//初始化DMA
  tft.fillScreen(TFT_BLACK);
  digitalWrite(secondScreenCS, HIGH);


  // //sprPtr指针分别指向两个精灵
  // sprPtr[0] = (uint16_t*)spr[0].createSprite(WIDTH,85);//双缓冲填充tear
  // sprPtr[1] = (uint16_t*)spr[1].createSprite(WIDTH,85);//


  //pupil_0显示：
  digitalWrite(firstScreenCS, LOW);
  tft.startWrite();
  tft.pushImageDMA(0,0,240,240,pupil_0);
  tft.endWrite();
  digitalWrite(firstScreenCS, HIGH);

  digitalWrite(secondScreenCS, LOW);
  tft.startWrite();
  tft.pushImageDMA(0,0,240,240,pupil_0);
  tft.endWrite();
  digitalWrite(secondScreenCS, HIGH);



  startMillis = millis();
}

void loop(){
  digitalWrite(firstScreenCS, LOW);
  tft.startWrite();//开始绘制
  //tft.pushImageDMA(0,0,240,240,pupil_0);
  // newImageProcess(pupil_0,resize,0);
  // newImageProcess(pupil_0,resize,1);
  resize -= 0.02;
  if(resize == 0.02){resize = 0.98;}
  Rotate1Image(pupil_0,0,resize,resize,1);
  tft.endWrite();
  digitalWrite(firstScreenCS, HIGH);
  //if(resize == 0){resize = 0.9;}

  // digitalWrite(secondScreenCS, LOW);
  // tft.startWrite();//开始绘制
  // Image2Buffer(0,pupil_0,tear_1,240,85);
  // delay(3);
  // Image2Buffer(1,pupil_0,tear_0,240,85);
  // //Image2Buffer(0,pupil_0,tear_2,240,85);
  // tft.endWrite();
  // digitalWrite(secondScreenCS, HIGH);

//计算帧率
  counter++;  
  if (counter % interval == 0) {
    long millisSinceUpdate = millis() - startMillis;
    fps = String((interval * 1000.0 / (millisSinceUpdate))) + " fps";
    Serial.println(fps);
    startMillis = millis();
  }
}

double SinXDivX(double x) {
  /*计算sinc值（插值权重）*/
  const float a = -1;  // a 用于调节锐化或模糊的程度
  if (x < 0) x = -x;   // 将 x 转为非负值
  double x2 = x * x;
  double x3 = x2 * x;
  if (x <= 1)
    return (a + 2) * x3 - (a + 3) * x2 + 1;
  else if (x <= 2)
    return a * x3 - (5 * a) * x2 + (8 * a) * x - (4 * a);
  else
    return 0;
}

uint16_t PixelBound(uint16_t img[], int x, int y) {
  //限制坐标在图像范围内
  x = (x >= 0) ? ((x < IMAGE_HEIGHT) ? x : IMAGE_HEIGHT - 1) : 0;
  y = (y >= 0) ? ((y < IMAGE_WIDTH) ? y : IMAGE_WIDTH - 1) : 0;
  return img[x * IMAGE_WIDTH + y];
}

uint16_t CubicConvInterpolation(uint16_t img[], float img_x, float img_y) {
  /*三次卷积插值：计算一次4*4范围内的三次卷积插值*/
  /*一维空间插值计算:x/x-1/x+1/x+2四个点进行加权平均：f(x)= ∑f(x+k)⋅W(x−(x+k))
    其中w（x）是卷积核
    二维三次卷积插值（分离卷积）：水平方向插值-垂直方向插值-最终插值像素值*/
  int img_x0 = int(img_x);
  if (img_x0 > img_x) { --img_x0; }
  int img_y0 = int(img_y);
  if (img_y0 > img_y) { --img_y0; }
  float fu = img_x - img_x0;  // x 方向的小数部分
  float fv = img_y - img_y0;  // y 方向的小数部分
  uint16_t pixel[16];         // 存储16个像素点
  int x, y;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      x = img_x0 - 1 + j;                        // 横坐标
      y = img_y0 - 1 + i;                        // 纵坐标
      pixel[i * 4 + j] = PixelBound(img, x, y);  // 存储对应像素值
    }
  }
  //  三次卷积插值计算
  uint16_t R = 0, G = 0, B = 0;
  float afu[4], afv[4];
  float sR = 0, sG = 0, sB = 0, aR = 0, aB = 0, aG = 0;
  afu[0] = SinXDivX(1 + fu);
  afu[1] = SinXDivX(fu);
  afu[2] = SinXDivX(1 - fu);
  afu[3] = SinXDivX(2 - fu);
  afv[0] = SinXDivX(1 + fv);
  afv[1] = SinXDivX(fv);
  afv[2] = SinXDivX(1 - fv);
  afv[3] = SinXDivX(2 - fv);
  for (int i = 0; i < 4; i++) {
    aR = aG = aB = 0;  // 清空横向插值累加器
    for (int j = 0; j < 4; j++) {
      // 计算横着三次插值
      uint16_t color = pixel[i * 4 + j];
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
  return uint16_t((R << 11) | (G << 5) | B);
}

uint16_t BilinearInterpolation(uint16_t* img, float img_x, float img_y) {
  /*二次插值获得像素点RGB值*/
  int img_x1 = int(img_x);
  int img_x2 = img_x1 + 1;
  int img_y1 = int(img_y);
  int img_y2 = img_y1 + 1;

  if (img_x1 < 0 || img_y1 < 0 || img_x2 >= IMAGE_WIDTH || img_y2 >= IMAGE_WIDTH) { return 0; }
  //计算插值权重
  float u = img_x - img_x1;
  float v = img_y - img_y1;
  float pm0 = (1 - u) * (1 - v);
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
  uint16_t R = uint16_t(((color_11 >> 11) & 0x1F) * pm0 + ((color_12 >> 11) & 0x1F) * pm1 + ((color_21 >> 11) & 0x1F) * pm2 + ((color_22 >> 11) & 0x1F) * pm3);
  uint16_t G = uint16_t(((color_11 >> 5) & 0x3F) * pm0 + ((color_12 >> 5) & 0x3F) * pm1 + ((color_21 >> 5) & 0x3F) * pm2 + ((color_22 >> 5) & 0x3F) * pm3);
  uint16_t B = uint16_t((color_11 & 0x1F) * pm0 + (color_12 & 0x1F) * pm1 + (color_21 & 0x1F) * pm2 + (color_22 & 0x1F) * pm3);
  uint16_t color = (R << 11) | (G << 5) | B;
  return color;
}

void Rotate1Image(uint16_t img[], double Rotaryangle, double ZoomX, double ZoomY, int resize) {
  /*平移、旋转和缩放图片：*/
  /*ZoomX、ZoomY为沿着X/Y轴的缩放系数*/
  /*rotaryangle为旋转角度 默认旋转方向是逆时针*/
  /*move_x move_y可以实现图片沿着x/y轴的平移(没有使用）*/
  /*imgIndex:确定是图片的上半部分/下半部分 放入renderbuf中*/
  /*前半部分图片imgindex：0 后半部分为1*/
  if ((fabs(ZoomX * IMAGE_WIDTH) < 1.0e-4) || (fabs(ZoomY * IMAGE_HEIGHT) < 1.0e-4)) { return; }
  int move_x = 0;
  int move_y = 0;
  int center_x = IMAGE_WIDTH / 2;  //旋转中心（120，120）
  int center_y = (IMAGE_HEIGHT) / 2;
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
  double Cx = (-(center_x + move_x) * rZoomX * cosA + (center_y + move_y) * rZoomY * sinA + center_x);
  double Cy = (-(center_x + move_x) * rZoomX * sinA - (center_y + move_y) * rZoomY * cosA + center_y);
  // 初始源图像的坐标
  double img0_x = Cx;
  double img0_y = Cy;
  // 获取旋转后的图像数组
  int imgIndex=0;//图片四个部分的索引
  int bufIndex=0;//renderbuf的索引：放入renderbuf0/1
  //imgPtr图片索引
  uint16_t color;
  int lo_y =0;//放入renderbuf的索引

  for (long y = 0; y < IMAGE_HEIGHT; y++) {
    double img_x_f = img0_x;
    double img_y_f = img0_y;
    for (long x = 0; x < IMAGE_WIDTH; x++) {
      int newX = int(img_x_f);  //取整获得对应原图像中的坐标值
      int newY = int(img_y_f);

      lo_y = y % 60;//获取在buf中的y坐标值(余数)
      if(y / 60 == 1){imgIndex = 1;}//获取图片的索引
      else if (y / 60 == 2){imgIndex = 2;}
      else if(y / 60 == 3){imgIndex = 3;}
      bufIndex = imgIndex % 2;//获得renderbuf索引0/1

      if (newX >= 0 && newX < IMAGE_WIDTH && newY >= 0 && newY < IMAGE_HEIGHT) {
        //imageBuffer[y * IMAGE_WIDTH + x] = BilinearInterpolation(img,img_x_f,img_y_f);//二次插值获得像素值
        //imageBuffer[y * IMAGE_WIDTH + x] = img[newY * IMAGE_WIDTH + newX]; // 将旋转后的像素映射到新位置

        //img[y * IMAGE_WIDTH + x] = CubicConvInterpolation(img, img_x_f, img_y_f);
        //renderbuf[bufIndex][y * IMAGE_WIDTH + x] = CubicConvInterpolation(img, img_x_f, img_y_f);
        //if (img[y * IMAGE_WIDTH + x] == 0) { img[y * IMAGE_WIDTH + x] = fillcolour[0]; }
        //if (renderbuf[bufIndex][y * IMAGE_WIDTH + x] == 0) { renderbuf[bufIndex][y * IMAGE_WIDTH + x] = fillcolour[0]; }
        color = BilinearInterpolation(img, img_x_f, img_y_f);
        if(color == 0){color = fillcolour[0];}
        renderbuf[bufIndex][lo_y * IMAGE_WIDTH + x] = color;
        
      } else if (FILLCOLOR || resize) {
        //图片缩小周围会出现一圈黑色，使用fillcolour填充不同的颜色进去
        //img[y * IMAGE_WIDTH + x] = fillcolour[0];  //填充白色
        renderbuf[bufIndex][lo_y * IMAGE_WIDTH + x] = fillcolour[0];  //填充白色
        //if (img[y * IMAGE_WIDTH + x] == 0) { img[y * IMAGE_WIDTH + x] = fillcolour[0]; }
        if (renderbuf[bufIndex][lo_y * IMAGE_WIDTH + x] == 0) { renderbuf[bufIndex][lo_y * IMAGE_WIDTH + x] = fillcolour[0]; }
      }
      img_x_f += Ax;
      img_y_f += Ay;
      if((lo_y * IMAGE_WIDTH + x) == 14399){
        if(imgIndex == 0){tft.pushImageDMA(0,imgIndex *  60,240,60,&renderbuf[bufIndex][0]);}
        else if(imgIndex == 1){tft.pushImageDMA(0,imgIndex* 60,240,60,&renderbuf[bufIndex][0]);}
        else if(imgIndex == 2){tft.pushImageDMA(0,imgIndex* 60,240,60,&renderbuf[bufIndex][0]);}
        else if(imgIndex == 3){tft.pushImageDMA(0,imgIndex* 60,240,60,&renderbuf[bufIndex][0]);}
        }
    }
    img0_x += Bx;
    img0_y += By;
  }
  //传输数据
  //tft.pushImageDMA(0,imgIndex *  IMAGE_HEIGHT,240,60,&renderbuf[bufIndex][0]);
  //填补出现的0(由于缩小使得边界出现一串0)
  // for(int i=0;i<total_pixels;i++){
  //   if(imageBuffer[i] == 0){
  //     imageBuffer[i] = fillcolour[0];
  //     }
  // }
  // for(int i=0; i< total_pixels;i++){
  //   img[i] = imageBuffer[i];//把旋转后的图像数组重新放回源数组中
  // }
  // boundaryfill(img);
}


void ImageProcess(uint16_t* img,double scale){
  /*w:240 h:60*/
  /*整体缩放*/
  /*scale：缩放比例 part：确认是图像的上半部分or下半部分*/
  Rotate1Image(img,0,scale,scale,1);// 图片缩小
  int bufIndex;
  uint16_t color;
  for(int i = 0; i < 4;i++){
    imgPtr = img + IMAGE_SIZE * i;//图片指针（四个部分）
    bufIndex = i % 2;
    for(int j = 0; j < IMAGE_SIZE ; j ++){
      color = *imgPtr;
      imgPtr ++;
      renderbuf[bufIndex][j] = color;//填充数据到buf中
    }
    tft.pushImageDMA(0,i *  BUFFER_HEIGHT,240,60,&renderbuf[bufIndex][0]);
    //Rotate1Image(imgPtr,0,scale,scale,1);
  }
}

void newImageProcess(uint16_t* img,double scale,int part){
  /*w:240 h:60*/
  /*整体缩放*/
  /*scale：缩放比例 part：确认是图像的上半部分or下半部分*/
  imgPtr  = img + part * IMAGE_SIZE;//确认指针指向图片上半部分or下半部分
  if(part == 0){Rotate1Image(imgPtr,0,scale,scale,1);}//上半部分
  else if(part == 1){Rotate1Image(imgPtr,0,scale,scale,1);}//下半部分
}

void Image2Buffer(uint16_t sprIndex,uint16_t* img1,uint16_t* img2,int width,int height){
  /*img1:pupil img2:tear*/
  /*填充数组到spr中*/
  /*使用pushPixelsDMA实现一行一行的将数据发送到显示屏中*/
  //tft.startWrite();//开始绘制
  tft.setAddrWindow(0, 155, width, height);//在（0，155）做一个（240，85）的窗口
  int offset = width*(240-height);
  uint16_t color;
  for(int y=0;y < height;y++){
    //Serial.println("color:");
    bufPtr = &renderbuf[sprIndex][0];
    for(int x = 0; x <width;x++){
      if(img2[y*width + x] == 0xFFFF){
        color = img1[offset + y*width + x];
      }else{
        color = img2[y*width + x];
      }
      //Serial.print(color,HEX);
      //Serial.print(',');
      *bufPtr ++ = color;
    }
    //Serial.println(' ');
    tft.pushPixelsDMA(&renderbuf[sprIndex][0],width);
    //sprIndex = 1 - sprIndex;
  }
  //tft.endWrite();
}

void tft_begin(){
  pinMode(firstScreenCS,OUTPUT);
  pinMode(secondScreenCS,OUTPUT);

  digitalWrite(firstScreenCS,HIGH);
  digitalWrite(secondScreenCS,HIGH);
  
  digitalWrite(firstScreenCS, LOW);
  digitalWrite(secondScreenCS, LOW);
  tft.init();
  tft.setTextSize(2);//设定为横屏模式
  digitalWrite(firstScreenCS, HIGH);
  digitalWrite(secondScreenCS, HIGH);
}

void Serprint(uint16_t img[], int height, int width) {
  Serial.println("IMG:");
  for (int i = 0; i < height; i++) {
    for (int j = 0; j < width; j++) {
      Serial.print(img[i * width + j], HEX);
      Serial.print(' ');
    }
    Serial.println();
  }
  Serial.println();
}


