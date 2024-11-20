#include <TFT_eSPI.h>//引入espi库
//#include <TJpg_Decoder.h>//raspberry不支持

//定义屏幕大小
#define WIDTH  240
#define HEIGHT 240

//引入图片
#include "pupil_0.h"


//初始化屏幕：定义两个屏幕 双屏显示
#define firstScreenCS 20
#define secondScreenCS 22
TFT_eSPI  tft = TFT_eSPI();
//创建DMA双缓冲区：
// TFT_eSprite spr[2] = {TFT_eSprite(&tft), TFT_eSprite(&tft)};
// uint16_t* sprPtr[2];//指针数组
// TFT_eSprite sprite[1] = {TFT_eSprite(&tft)};
// uint16_t* spr;
uint16_t renderbuf[2][WIDTH];
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
 //spr = (uint16_t*)sprite[0].createSprite(WIDTH,85);

  //pupil_0显示：
  digitalWrite(firstScreenCS, LOW);
  tft.startWrite();
  tft.pushImageDMA(0,0,WIDTH,HEIGHT,pupil_0);
  tft.endWrite();
  digitalWrite(firstScreenCS, HIGH);

  digitalWrite(secondScreenCS, LOW);
  tft.startWrite();
  tft.pushImageDMA(0,0,WIDTH,HEIGHT,pupil_0);
  tft.endWrite();
  digitalWrite(secondScreenCS, HIGH);

  startMillis = millis();
}

void loop(){
  digitalWrite(firstScreenCS, LOW);
  tft.startWrite();//开始绘制
  Image2Buffer(0,pupil_0,tear_0,240,85);
  delay(3);
  Image2Buffer(1,pupil_0,tear_1,240,85);
  //Image2Buffer(0,pupil_0,tear_2,240,85);
  tft.endWrite();
  digitalWrite(firstScreenCS, HIGH);

  digitalWrite(secondScreenCS, LOW);
  tft.startWrite();//开始绘制
  Image2Buffer(0,pupil_0,tear_1,240,85);
  delay(3);
  Image2Buffer(1,pupil_0,tear_0,240,85);
  //Image2Buffer(0,pupil_0,tear_2,240,85);
  tft.endWrite();
  digitalWrite(secondScreenCS, HIGH);

  counter++;  
  if (counter % interval == 0) {
    long millisSinceUpdate = millis() - startMillis;
    fps = String((interval * 1000.0 / (millisSinceUpdate))) + " fps";
    Serial.println(fps);
    startMillis = millis();
  }
}


void Image2Display(uint16_t* img1,uint16_t* img2[]){
  /*w:240 h:85*/
}

void Image2Buffer(uint16_t sprIndex,uint16_t* img1,uint16_t* img2,int width,int height){
  /*img1:pupil img2:tear*/
  /*填充数组到spr中*/
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

// void Image2Spirit(int sprIndex,uint16_t* img1,uint16_t* img2,int width,int height){
//   tft.startWrite();//开始绘制
//   tft.setAddrWindow(0, 155, width, height);//在（0，155）做一个（240，85）的窗口
//   int offset = width*(240-height);
//   uint16_t color;
//   for(int y=0;y < height;y++){
//     //Serial.println("color:");
//     for(int x = 0; x <width;x++){
//       if(img2[y*width + x] == 0xFFFF){
//         color = img1[offset + y*width + x];
//       }else{
//         color = img2[y*width + x];
//       }
//       //Serial.print(color,HEX);
//       //Serial.print(',');
//       spr[sprIndex].drawPixel(x,y,color);
//     }
//     //Serial.println(' ');
//     //delay(3);
//   }
//   tft.pushImageDMA(0,155,240,85,sprPtr[sprIndex]);
//   tft.endWrite();
// }

// void newImage2Spirit (uint16_t* img1,uint16_t* img2,int width,int height){
//   tft.startWrite();//开始绘制
//   tft.setAddrWindow(0, 155, width, height);//在（0，155）做一个（240，85）的窗口
//   int offset = width*(240-height);
//   uint16_t color;
//   for(int y=0;y < height;y++){
//     //Serial.println("color:");
//     for(int x = 0; x <width;x++){
//       if(img2[y*width + x] == 0xFFFF){
//         color = img1[offset + y*width + x];
//       }else{
//         color = img2[y*width + x];
//       }
//       //Serial.print(color,HEX);
//       //Serial.print(',');
//       sprite[0].drawPixel(x,y,color);
//     }
//     //Serial.println(' ');
//     //delay(3);
//   }
//   tft.pushImageDMA(0,155,240,85,spr);
//   tft.endWrite();
// }

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


