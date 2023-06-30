import cv2
from rotate import rotate

img = cv2.imread("../playground/images/test05193.jpg")
img = rotate(img)
img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

_, imgb = cv2.threshold(img_gray, 220, 255, cv2.THRESH_BINARY)
contours, _ = cv2.findContours(imgb, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
rects = [cv2.minAreaRect(contour) for contour in contours]
rects.sort(key=lambda x: x[1][0]*x[1][1], reverse=True)

center_x = rects[2][0][0]
center_y = rects[2][0][1]
# 图片可能有微小角度，略微放大截取框的width height
wid = int(rects[2][1][0]) + 6
hei = int(rects[2][1][1]) + 6
x = int(center_x - wid // 2)
y = int(center_y - hei // 2)
img_crop = img[y:y+hei, x:x+wid]
cv2.imshow("crop", img_crop)
cv2.waitKey(0)