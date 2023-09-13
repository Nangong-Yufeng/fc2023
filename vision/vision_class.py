"""
完成视觉工作的类
"""
import os
import sys
from pathlib import Path

import cv2

from vision.yolov5.models.common import DetectMultiBackend
from vision.yolov5.utils.general import check_img_size
from vision.yolov5.utils.torch_utils import select_device

from vision.MyLoadImage import MyLoadIamge
from anotherVision import NumberRecognizer
from vision.detect import detect

class Vision:
    """完成视觉工作的所有任务
    """

    def __init__(self, source=0, device='0', conf_thres=0.7):
        """构造函数

        :param source: 视频源 0: 外部摄像头，使用前请禁用电脑摄像头
        :param device: 硬件 '0'：GPU； 'CPU'：CPU
        :param conf_thres: 置信度阈值
        """
        FILE = Path(__file__).resolve()
        ROOT = FILE.parents[0]  # YOLOv5 root directory
        if str(ROOT) not in sys.path:
            sys.path.append(str(ROOT))  # add ROOT to PATH
        ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

        # 参数
        weights = ROOT / 'weights/m_low.pt'  # 权重文件
        data = ROOT / 'ngyf.yaml'  # 类别等信息
        imgsz = (640, 640)  # 新图大小
        half = False  # use FP16 half-precision inference
        dnn = False  # use OpenCV DNN for ONNX inference
        self.conf_thres = conf_thres

        print("视觉：加载权重文件")
        device = select_device(device)
        self.model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
        self.stride, self.pt = self.model.stride, self.model.pt
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size

        print("视觉：进行模型热身")
        bs = 1
        self.model.warmup(imgsz=(1 if self.pt or self.model.triton else bs, 3, *imgsz))  # warmup

        # 加载摄像头
        print("视觉：加载摄像头")
        self.im0 = None
        self.cap = cv2.VideoCapture(source)#, cv2.CAP_DSHOW
        print("视觉：设置视频格式")
        self.cap.set(cv2.CAP_PROP_FPS, 30)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        print("视觉：设置完成")
        print(f'帧率{self.cap.get(cv2.CAP_PROP_FPS)}')
        print(f'高度{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}')
        print(f'宽度{self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}')

        # 加载数字识别ocr
        print("视觉：加载数字识别ocr")
        self.numrec = NumberRecognizer()

    def shot(self):
        """截图
        """
        ret, self.im0 = self.cap.read()

    def run(self):
        """ 检测图像中的标靶，返回数值与坐标

        Return:
             list (vision_position): 标靶中心坐标、数值列表
        """
        # self.im0 = cv2.resize(self.im0, (1920, 1080))
        im = MyLoadIamge(im0=self.im0, img_size=self.imgsz, stride=self.stride, auto=self.pt)
        return detect(im0=self.im0, im=im, model=self.model, conf_thres=self.conf_thres, numrec=self.numrec)