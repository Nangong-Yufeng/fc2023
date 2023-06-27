"""
将检测到的标靶照片转正
"""
import cv2
import numpy as np

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
    
    # 转化为灰度图
    image_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)   
    if debug:
        debug_ret['gray'] = image_gray.copy()
    # 转化为二值图
    _, image_binary = cv2.threshold(image_gray, 140, 255, cv2.THRESH_BINARY)
    if debug:
        debug_ret['binary'] = image_binary

    # 对二值图进行轮廓检测
    # 轮廓的检索模式：cv2.RETR_CCOMP 只检测最外部轮廓
    # 轮廓的近似方法：cv2.CHAIN_APPROX_SIMPLE，只保留端点
    contours, _ = cv2.findContours(image_binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if debug:
        for contour in contours:
            image_with_contour = image.copy()
            for i in range(len(contour)):
                cv2.line(image_with_contour, contour[i][0], contour[(i+1)%len(contour)][0], thickness=3, color=(0, 255, 255))
                debug_ret['with_contour'] = image_with_contour

    # 利用检测出来的边缘检测角度
    # 遍历每一个轮廓，避免标靶外面有小的噪点被识别
    angle = 0.
    rect = None
    for contour in contours:
        rect = cv2.minAreaRect(contour)  # 最小的外接矩形
        width, height = rect[1]
        if width * height > 100:  # 使用外接矩形的面积来筛选合适的
            # 旋转角度
            angle = cv2.minAreaRect(contour)[2]
            if (height < width):
                angle -= 90
                
    if debug:
        if rect is None:
            RuntimeWarning("一个轮廓都没识别出来")
        else:
            image_with_rect = image.copy()
            box = cv2.boxPoints(rect).astype(np.int)
            for i in range(4):
                cv2.circle(image_with_rect, center=tuple(box[i]), radius=3, color=(0, 0, 0), thickness=1)
                cv2.line(image_with_rect, tuple(box[i]), tuple(box[(i+1)%4]), color=(0, 0, 255), thickness=1)
            debug_ret['with_rect'] = image_with_rect
    # 仿射变换,对图片旋转angle角度
    h, w = image.shape[:2]
    center = (h // 2 + 5, w // 2 + 5)
    M = cv2.getRotationMatrix2D(center, angle, 1.0)
    image_rotated = cv2.warpAffine(image, M, (h + 10, w + 10), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)

    _, image_rotated = cv2.threshold(image_rotated, 200, 255, cv2.THRESH_BINARY)

    # # 检测图片正倒
    # contours, _ = cv2.findContours(image_rotated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    # image_external = contours[0]
    # # print(len(img_external))
    # _, cy = cv2.minAreaRect(image_external)[0]
    # _, hh = cv2.minAreaRect(image_external)[1]
    # tot = 0
    # for i in image_external:
    #     x, y = i[0]
    #     if (y < cy and (cy - y) > 0.4 * hh):
    #         tot += 1
    #     elif (y > cy and (y - cy) > 0.4 * hh):
    #         tot -= 1

    # if(tot > 0):
    #     angle += 180

    # M = cv2.getRotationMatrix2D((h // 2  + 5, w // 2 + 5), angle, 1.0)
    # ans = cv2.warpAffine(rgb_img, M, (h + 10, w + 10), flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
    if debug:
        debug_ret['rotate'] = image_rotated
        return debug_ret
    return image_rotated