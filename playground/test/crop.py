import cv2
import numpy as np

def crop(img):
# img = cv2.imread("../playground/images/test05193.jpg")
# img = rotate(img)
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    _, imgb = cv2.threshold(img_gray, 180, 255, cv2.THRESH_BINARY)

    kernel = np.ones((3, 3), dtype=np.uint8)
    imgb = cv2.dilate(imgb, kernel, 1) # 1:迭代次数，也就是执行几次膨胀操作

    cv2.imshow('img', imgb)
    cv2.waitKey(0)
    cv2.destroyWindow('img')
    contours, _ = cv2.findContours(imgb, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rects = [cv2.minAreaRect(contour) for contour in contours]
    rects.sort(key=lambda x: x[1][0]*x[1][1], reverse=True)

    center_x = 0
    center_y = 0
    for rect in rects:
        if rect[1][0] / rect[1][1] < 1.2 and rect[1][0] / rect[1][1] > 0.8 and rect[1][0] * rect[1][1] > 63:
            # 获取中心点坐标
            center_x = rect[0][0]
            center_y = rect[0][1]
            # 图片可能有微小角度，略微放大截取框的width height
            wid = int(rect[1][0]) + 6
            hei = int(rect[1][1]) + 6
            break

    # 若找不到返回长宽均为0的照片
    if not center_x and not center_y:
        return img[0:0, 0:0]
    # 获取左上点坐标
    x = int(center_x - wid // 2)
    y = int(center_y - hei // 2)
    return img[y:y+hei, x:x+wid]
    # cv2.imshow("crop", img_crop)
    # cv2.waitKey(0)