import rotate
import cv2

img = cv2.imread('test2.png')
img_rotated = rotate.rotate(img)
cv2.imshow('img', img_rotated)
cv2.waitKey(0)
cv2.destroyAllWindows()