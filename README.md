# arduino_display


图片格式：1：1
获取图片像素对应数组：Image2Lcd 2.9.rar


Arduino driver of two round displays to show eye emojis.
接线：
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
 
实现功能：
 1.显示眼神图片photoText（）
 2.切换图片 change_image()
 3.移动图片 move_image()