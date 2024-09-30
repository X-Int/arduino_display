## -*- coding: utf-8 -*-
import cv2
from matplotlib import pyplot as plt
import numpy as np

# 读取图像，使用灰度模式
img = cv2.imread('22350-eyes4.jpg')

# ==================== 直方图均衡化 ============================
equ = cv2.equalizeHist(img)

# 显示原图和处理后的图像
plt.figure(figsize=(10, 5))

# 原图
plt.subplot(1, 2, 1)
plt.imshow(img)
plt.title('Original Image')
plt.axis('off')

# 处理后的图像
plt.subplot(1, 2, 2)
plt.imshow(equ)
plt.title('Equalized Image')
plt.axis('off')

# 显示图像
plt.show()
