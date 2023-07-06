import crop
import cv2

img = cv2.imread('tmp.jpg')
img_crop = crop.crop(img)
cv2.imshow('img', img_crop)
cv2.waitKey(0)
cv2.destroyAllWindows()