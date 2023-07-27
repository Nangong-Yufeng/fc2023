"""
代替 yolov5.utils.dataloaders.LoadImage
将原图转换为img_size的新图
"""
import numpy as np
from vision.yolov5.utils.augmentations import letterbox

def MyLoadIamge(im0:np.array, img_size=640, stride=32, auto=True):
    """将原图转换为img_size的新图

    Args:
        im0 (np.array): 原图
        img_size: 新图大小
        stride: model.stride
        auto: model.pt

    Returns:
        im (np.array): 新图
    """
    im = letterbox(im0, img_size, stride=stride, auto=auto)[0]  # padded resize
    im = im.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
    im = np.ascontiguousarray(im)  # contiguous
    return im