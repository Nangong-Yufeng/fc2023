"""
提取转正标靶的数字矩形框
"""
import cv2
import numpy as np

def crop(img):
    """
    提取转正标靶的数字矩形框

    Args:
        img: 待处理图片，是openCV的BGR格式
    Returns:
        数字矩形框图片，是openCV的BGR格式; 若未检测到数字矩形框，返回长宽为0的图片，也是openCV的BGR格式
    """
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    imgb = cv2.Canny(img_gray, 60, 180, L2gradient=True)

    # 找到所有边界，按边界的外界最小矩形的面积大小排序
    contours, _ = cv2.findContours(imgb, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    rects = [(i, cv2.minAreaRect(contour)) for i, contour in enumerate(contours)]
    rects.sort(key=lambda x: x[1][1][0]*x[1][1][1], reverse=True)

    # 寻找近似为正方形的框，即为数字框
    # 限制正方形的最小面积，避免图中仍有小的黑色正方形框
    # 提取数字所在平行四边形，并变换为正方形
    for rect in rects:
        if rect[1][1][1] != 0 and rect[1][1][0] / rect[1][1][1] < 1.25 and rect[1][1][0] / rect[1][1][1] > 0.8 and rect[1][1][0] * rect[1][1][1] > 400:
            epsilon = 0.1 * cv2.arcLength(contours[rect[0]], True)
            approx = cv2.approxPolyDP(contours[rect[0]], epsilon, True)
            approx = np.squeeze(approx, 1)
            if len(approx) != 4:
                return img[0:0, 0:0]
            ind = 0
            # 找到平行四边形左上方的点
            for i in range(3):
                if approx[i+1][0] + approx[i+1][1] < approx[ind][0] + approx[ind][1]:
                    ind = i + 1
            # 透视变换的目标正方形，点与平行四边形顺序对应，逆时针方向
            tmp_list = []
            if not ind:
                tmp_list = [[0, 0], [0, 30], [30, 30], [30, 0]]
            elif ind == 1:
                tmp_list = [[30, 0], [0, 0], [0, 30], [30, 30]]
            elif ind == 2:
                tmp_list = [[30, 30], [30, 0], [0, 0], [0, 30]]
            elif ind == 3:
                tmp_list = [[0, 30], [30, 30], [30, 0], [0, 0]]
            src = np.array(approx, dtype=np.float32)
            dst = np.array(tmp_list, dtype=np.float32)
            # 透视变换
            M = cv2.getPerspectiveTransform(src, dst)
            ret = cv2.warpPerspective(img, M, [30, 30], flags=cv2.INTER_CUBIC, borderMode=cv2.BORDER_REPLICATE)
            return ret

    # 若找不到返回长宽均为0的照片
    return img[0:0, 0:0]