"""
将检测到的标靶照片转正
"""
import cv2
import numpy as np
import time
import math

def rotate(image:np.array, debug:bool=False)->np.array:
    """ 将标靶转正

    Args:
        image (np.array): 待处理图片，是openCV的BGR格式
        debug (bool, optional): 是否开启debug模式. 默认为否.
                
    Returns:
        np.array: 转正的图片，是openCV的BGR格式
    """
    if debug:
        debug_ret = {}
        time_start = time.time()
    
    # 转化为灰度图
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)   
    if debug:
        time_end = time.time()
        print(f"转化为灰度图：{1000*(time_end-time_start)}ms")
        debug_ret['gray'] = image_gray.copy()
        time_start = time.time()
    # 转化为二值图
    _, image_binary = cv2.threshold(image_gray, 180, 255, cv2.THRESH_BINARY)
    # cv2.imshow('tmp', image_binary)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    if debug:
        time_end = time.time()
        print(f"转化为二值图：{1000*(time_end-time_start)}ms")
        debug_ret['binary'] = image_binary
        time_start = time.time()

    # 对二值图进行轮廓检测
    # 轮廓的检索模式：cv2.RETR_CCOMP 只检测最外部轮廓
    # 轮廓的近似方法：cv2.CHAIN_APPROX_SIMPLE，只保留端点
    contours, _ = cv2.findContours(image_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if debug:
        time_end = time.time()
        print(f"对二值图轮廓检测：{1000*(time_end-time_start)}ms")
        for contour in contours:
            image_with_contour = image.copy()
            for i in range(len(contour)):
                cv2.line(image_with_contour, contour[i][0], contour[(i+1)%len(contour)][0], thickness=2, color=(0, 255, 255))
                debug_ret['with_contour'] = image_with_contour

            # cv2.imshow('tmp', image_with_contour)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()
        time_start = time.time()

    # 利用检测出来的边缘检测角度
    # 遍历每一个轮廓，避免标靶外面有小的噪点被识别
    angle = 0.
    rect = None
    used_contour = None
    used_rect_size = 0.
    for contour in contours:
        rect = cv2.minAreaRect(contour)  # 最小的外接矩形
        width, height = rect[1]
        if width * height > 100 and width * height > used_rect_size:  # 使用外接矩形的面积来筛选合适的
            # 旋转角度
            angle = cv2.minAreaRect(contour)[2]
            used_contour = contour
            used_rect_size = width * height
            if (height < width):
                angle -= 90

    if rect is None or used_contour is None:  # 如果一个轮廓都没检测出来就别旋转了直接返回得了
        return image[0:0, 0:0]

    if debug:
        time_end = time.time()
        print(f"利用轮廓获取角度：{1000*(time_end-time_start)}ms")
        image_with_rect = image.copy()
        rect = cv2.minAreaRect(used_contour)
        box = cv2.boxPoints(rect).astype(np.uint8)
        for i in range(4):
            cv2.circle(image_with_rect, center=tuple(box[i]), radius=3, color=(0, 0, 0), thickness=2)
            cv2.line(image_with_rect, tuple(box[i]), tuple(box[(i+1)%4]), color=(0, 0, 255), thickness=1)
        debug_ret['with_rect'] = image_with_rect
        cv2.imshow('haha', image_with_rect)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        time_start = time.time()

    # 检测图片正反
    # 得到仿射变换
    M = cv2.getRotationMatrix2D((0, 0), angle, 1.0)

    # 根据轮廓得到一个近似三角形
    _, tri = cv2.minEnclosingTriangle(used_contour.squeeze())
    tri = tri.squeeze().astype(int)

    # 先将三角形旋转angle角度，如果三角形旋转后尖尖朝下，则说明反了。
    rotated_tri = np.dot(np.append(tri, [[1], [1], [1]], axis=1), M.T).astype(int)
    ys = rotated_tri[:, 1].reshape(-1)
    if np.prod(np.mean(ys)-ys) < 0:
        angle += 180

    # 更新旋转矩阵并执行旋转
    h, w = image.shape[:2]
    a = max(h, w) * 1.5
    w_add = math.ceil((a - w) / 2)
    h_add = math.ceil((a - h) / 2)
    img = cv2.copyMakeBorder(image, h_add, h_add, w_add, w_add, cv2.BORDER_REPLICATE)
    h, w = img.shape[:2]
    center = (h >> 1, w >> 1)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    image_rotated = cv2.warpAffine(img, M, (h, w), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)

    if debug:
        time_end = time.time()
        print(f"得到旋转后图像：{1000*(time_end-time_start)}ms")
        for i in range(4):
            cv2.circle(image_rotated, center=tuple(box[i]), radius=3, color=(0, 0, 0), thickness=1)
            cv2.line(image_rotated, tuple(box[i]), tuple(box[(i+1)%4]), color=(0, 0, 255), thickness=1)
        debug_ret['rotated'] = image_rotated
        return debug_ret['rotated']
    
    return image_rotated