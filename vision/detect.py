"""
检测标靶，返回数值与坐标
"""

import os
import sys
from pathlib import Path

import numpy as np
import torch
import cv2

from vision.yolov5.models.common import DetectMultiBackend
from vision.yolov5.utils.general import non_max_suppression, scale_boxes, Profile, check_img_size
from vision.yolov5.utils.plots import Annotator, colors
from vision.yolov5.utils.torch_utils import select_device, smart_inference_mode

from vision.MyLoadImage import MyLoadIamge
from vision.rotate import rotate
from vision.crop import crop

from anotherVision import NumberRecognizer
from navigation.class_list import vision_position

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
        weights = ROOT / 'best.pt'  # 权重文件
        data = ROOT / 'ngyf.yaml'  # 类别等信息
        imgsz = (640, 640)  # 新图大小
        half = False  # use FP16 half-precision inference
        dnn = False  # use OpenCV DNN for ONNX inference
        self.conf_thres = conf_thres

        device = select_device(device)
        self.model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
        self.stride, self.pt = self.model.stride, self.model.pt
        self.imgsz = check_img_size(imgsz, s=self.stride)  # check image size
        bs = 1
        self.model.warmup(imgsz=(1 if self.pt or self.model.triton else bs, 3, *imgsz))  # warmup

        # 加载摄像头
        self.im0 = None
        self.cap = cv2.VideoCapture(source)
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        print(f'帧率{self.cap.get(cv2.CAP_PROP_FPS)}')
        print(f'高度{self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)}')
        print(f'宽度{self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)}')

        fourcc = cv2.VideoWriter_fourcc(*'XVID')

        out = cv2.VideoWriter('testwrite.avi', fourcc, 60.0, (1920, 1080), True)

        self.numrec = NumberRecognizer('./anotherVision/weights/cnn2.pkl')

    @smart_inference_mode()
    def detect(
        self,
        im0: np.array,  # 原图
        im: np.array,  # 新图
        model,  # 模型文件
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        view_img=True,  # show results
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
    ):
        """ 检测标靶，返回数值与坐标

        Args:
            im0 (np.array): 原图
            im (np.array): 新图
            model: 模型文件
            itv: 图片显示时长
            conf_thres: 置信度阈值

        Returns:
            list (vision_position): 标靶中心坐标、数值列表
        """
        names = model.names
        dt = (Profile(), Profile(), Profile())

        with dt[0]:
            im = torch.from_numpy(im).to(model.device)  # Tensor
            im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            # 没有batch_size 时，在前面添加一个轴
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim

        # Inference
        with dt[1]:
            pred = model(im, augment=False, visualize=False)

        # NMS
        """
        pred 向前传播的输出
        conf_thres 置信度阈值
        iou_thres iou阈值
        classes 是否只保留特定的类别
        agnostic_nms 进行nms是否也去除不同类别之间的框
        返回值为list[torch.tensor],长度为batch_size
        每一个torch.tensor的shape为(num_boxes, 6),内容为box+conf+cls, box为xyxy(左上右下)
        """
        with dt[2]:
            pred = non_max_suppression(pred, conf_thres, iou_thres, classes, agnostic_nms, max_det=max_det)

        # Process predictions
        """
        对每一张图做处理
        循环次数等于batch_size
        """
        res = []
        for i, det in enumerate(pred):  # per image
            im0_copy = im0.copy()
            annotator = Annotator(im0, line_width=line_thickness, example=str(names))
            if len(det):
                # Rescale boxes from img_size to im0 size
                # 调整预测框坐标，将resize+pad后的img_size调整回im0的size
                # 此时坐标格式为xyxy
                det[:, :4] = scale_boxes(im.shape[2:], det[:, :4], im0.shape).round()

                # Print results
                # 统计检测到的每一个class的预测框数量
                for c in det[:, 5].unique():
                    n = (det[:, 5] == c).sum()  # detections per class

                # Write results
                for *xyxy, conf, cls in reversed(det):  # reversed反转列表顺序
                    tlbr = torch.tensor(xyxy).view(1, 4).view(-1).tolist()  # 框的左上右下坐标
                    img = im0_copy.copy()
                    hei = img.shape[0]
                    wid = img.shape[1]

                    top = int(tlbr[1]) - 5
                    down = int(tlbr[3]) + 5
                    left = int(tlbr[0]) - 5
                    right = int(tlbr[2]) + 5
                    if top < 0 or left < 0 or down > hei or right > wid:  # 舍弃边界图片，确保标靶完整
                        continue
                    img = img[top:down, left:right]  # 对原图切片，截取标靶
                    img = cv2.copyMakeBorder(img, 3, 3, 3, 3, cv2.BORDER_CONSTANT, 0)

                    img_rotated = rotate(img)
                    if img_rotated.shape[:2] == (0, 0):  # 未检测到数字正方形
                        continue

                    img_crop = crop(img_rotated)
                    if img_crop.shape[:2] == (0, 0):  # 未检测到数字正方形
                        continue

                    ret = self.numrec.recognize(img_crop)
                    print(f'检测到数字: {ret}')
                    res.append(vision_position(x=(left + right) / 2, y=(top + down) / 2, target_number=ret))
                    # 在原图上画框
                    if view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))

            im0 = annotator.result()
            # 若设置展示，则画出图片/视频
            if view_img:
                cv2.imshow('0', im0)
                cv2.waitKey(1)
        return res

    def shot(self):
        """截图
        """
        ret, self.im0 = self.cap.read()

    def run(self):
        """ 检测图像中的标靶，返回数值与坐标

        Return:
             list (vision_position): 标靶中心坐标、数值列表
        """
        im = MyLoadIamge(im0=self.im0, img_size=self.imgsz, stride=self.stride, auto=self.pt)
        return self.detect(im0=self.im0, im=im, model=self.model, conf_thres=self.conf_thres)