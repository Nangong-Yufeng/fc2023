import numpy as np
import cv2

path = '../../opencv/shuzi.png'

img = cv2.imread(path)
img = cv2.resize(img,dsize=(56,28))
# the image height
sum_rows = img.shape[0]
# the image length
sum_cols = img.shape[1]
part1 = img[0:sum_rows, 0:sum_cols // 2]
part2 = img[0:sum_rows, sum_cols // 2:sum_cols]
img_1 = cv2.cvtColor(part1,cv2.COLOR_BGR2GRAY)
img_2 = cv2.cvtColor(part2,cv2.COLOR_BGR2GRAY)

cv2.imshow('img_1', img_1)
cv2.imshow('img_2', img_2)

cv2.waitKey(0)
cv2.imwrite('shuzi/7/1_1.jpg', img_1)
cv2.imwrite('shuzi/6/1_2.jpg', img_2)