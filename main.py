"""
这是将在比赛时被运行的文件
"""
from utils import title
import os
import sys
import time
from pathlib import Path

import cv2

from vision.yolov5.models.common import DetectMultiBackend
from vision.yolov5.utils.torch_utils import select_device
from vision.yolov5.utils.general import check_img_size
from vision.MyLoadImage import MyLoadIamge

from vision.detect import detect


# 0. ~
title.printTitle()

# 1. 飞行前准备

# 2. 

"""
标靶识别
"""
FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

# 参数
device = 0  # 硬件 0：GPU
weights = ROOT / 'vision/best.pt'  # 权重文件
data = ROOT / 'vision/0515.yaml'  # 类别等信息
imgsz = (640, 640)  # 新图大小
half = False  # use FP16 half-precision inference
dnn = False  # use OpenCV DNN for ONNX inference
itv = 50  # 每隔多少毫秒做一次识别

device = select_device(device)
model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
stride, pt = model.stride, model.pt
imgsz = check_img_size(imgsz, s=stride)  # check image size

bs = 1
model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup

# 加载摄像头
cap = cv2.VideoCapture(0)

pre = int(time.time() * 1000)
while True:
    if int(time.time() * 1000) == pre + itv:
        ret, im0 = cap.read()  # 截图
        im = MyLoadIamge(im0=im0, img_size=imgsz, stride=stride, auto=pt)
        detect(im0=im0, im=im, model=model, itv=itv, conf_thres=0.7)
        pre = int(time.time() * 1000)
