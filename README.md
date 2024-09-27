# arduino_display

Arduino driver of two round displays to show eye emojis.

* Aspect ratio of images `1：1`
* Use the tool `Image2Lcd` to convert images into C arrays
* Pin connection:
```C
 TFT_CS 23  // Chip select
 TFT_DC 22  // Data/command
 TFT_BL 24  // Backlight control
 ODOR_SENSOR A8
 POT1 A0
 POT2 A1
 UP 40 //五向按键
 DOWN 42
 LEFT 44
 RIGHT 46
 MID 48
 SET 50
 RST 52
 servo1 2
 servo2 3
``` 
* Functions：
 1. 显示眼神图片 photoText()
 2. 切换图片 change_image()
 3. 移动图片 move_image()
