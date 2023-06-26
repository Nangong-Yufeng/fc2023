import cv2
import numpy as np

rgb_img_path = './target0002.jpg'
# imagepath = 'rotateImg/mask_inv.png'
# img = cv2.imread(imagepath, -1)
img = cv2.imread(rgb_img_path, cv2.IMREAD_GRAYSCALE)
_, img = cv2.threshold(img, 200, 255, cv2.THRESH_BINARY)
cv2.imshow('img', img)
cv2.waitKey()
rgb_img = cv2.imread(rgb_img_path)
# imgflip = cv2.flip(img, 1)
# cv2.imwrite('fliped_img.png', imgflip)
contours, _ = cv2.findContours(img, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
# cv2.RETR_CCOMP 检测所有轮廓，建立两级层次结构
# cv2.CHAIN_APPROX_SIMPLE 压缩水平，垂直和对角线段，只留下端点

angle = 0.0

for cnt in contours:

    # 最小外界矩形的宽度和高度
    width, height = cv2.minAreaRect(cnt)[1]

    if width * height > 100:

        # 最小的外接矩形
        rect = cv2.minAreaRect(cnt)
        box = cv2.boxPoints(rect)  # 获取最小外接矩形的4个顶点
        box = np.intp(box)  # 转换为整数

        if 0 not in box.ravel():    # .ravel() 将数组整合为一个1维数组输出

            '''绘制最小外界矩形
            for i in range(4):
                cv2.line(image, tuple(box[i]), tuple(box[(i+1)%4]), 0)  # 5
            '''
            # 旋转角度
            for i in range(4):
                cv2.line(img, tuple(box[i]), tuple(box[(i + 1) % 4]), 0)  # 5
            cv2.imshow('img', img)
            cv2.waitKey()
            theta = cv2.minAreaRect(cnt)[2]
            print('thete = ', theta)
            print('width = ', width)
            print('height = ', height)
            if abs(theta) <= 45:
                print('图片的旋转角度为%s.' % theta)
            angle = theta
            if (height < width):
                angle = theta - 90

# 仿射变换,对图片旋转angle角度
h, w = img.shape
center = (h // 2 + 5, w // 2 + 5)
M = cv2.getRotationMatrix2D((h // 2  + 5, w // 2 + 5), angle, 1.0)
rotated = cv2.warpAffine(img, M, (h + 10, w + 10)), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)

cv2.imshow('img', rotated)
cv2.waitKey()
# 保存旋转后的图片
# cv2.imwrite('after_rotated_rgbtarget0002.jpg', rotated)