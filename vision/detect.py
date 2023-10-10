"""
检测标靶，返回数值与坐标
"""
import time

import cv2
import numpy as np
import torch

from .yolov5.utils.general import Profile, non_max_suppression, scale_boxes
from .yolov5.utils.plots import Annotator, colors
from .yolov5.utils.torch_utils import smart_inference_mode

from .rotate import rotate
from .crop import crop
from anotherVision import NumberRecognizer
from navigation.class_list import vision_position

@smart_inference_mode()
def detect(
    im0: np.array,  # 原图
    im: np.array,  # 新图
    model,  # 模型文件
    numrec: NumberRecognizer,  # 数字识别的类
    use_ocr=True, # 是否使用ocr
    conf_thres=0.25,  # confidence threshold
    iou_thres=0.45,  # NMS IOU threshold
    max_det=1000,  # maximum detections per image
    view_img=True,  # show results
    classes=None,  # filter by class: --class 0, or --class 0 2 3
    agnostic_nms=False,  # class-agnostic NMS
    line_thickness=3,  # bounding box thickness (pixels)
):
    """ 检测标靶，返回数值与坐标

    Args:
        im0 (np.array): 原图
        im (np.array): 新图
        model: 模型文件
        itv: 图片显示时长
        conf_thres: 置信度阈值
        use_ocr: 是否使用ocr

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
            # for c in det[:, 5].unique():
            #     n = (det[:, 5] == c).sum()  # detections per class

            # Write results
            for *xyxy, conf, cls in reversed(det):  # reversed反转列表顺序
                tlbr = torch.tensor(xyxy).view(1, 4).view(-1).tolist()  # 框的左上右下坐标
                img = im0_copy.copy()
                hei = img.shape[0]
                wid = img.shape[1]

                top = int(tlbr[1]) - 3
                down = int(tlbr[3]) + 3
                left = int(tlbr[0]) - 3
                right = int(tlbr[2]) + 3
                if top < 0 or left < 0 or down > hei or right > wid:  # 舍弃边界图片，确保标靶完整
                    continue

                if use_ocr:
                    img = img[top:down, left:right]  # 对原图切片，截取标靶

                    img_rotated = rotate(img)
                    if img_rotated.shape[:2] == (0, 0):  # 未检测到数字正方形
                        continue

                    # cv2.imwrite(f'D:/ngyf/crops/r{time.time()}.jpg', img_rotated)

                    img_crop = crop(img_rotated)
                    if img_crop.shape[:2] == (0, 0):  # 未检测到数字正方形
                        continue

                    # cv2.imwrite(f'D:/ngyf/crops/{time.time()}.jpg', img_crop)
                    # img_crop = cv2.cvtColor(img_crop, cv2.COLOR_BGR2GRAY)
                    # cv2.equalizeHist(img_crop, img_crop)

                    # tmp = int(time.time() * 1000)
                    ret = numrec.recognize(img_crop)
                    #print("数字识别时间： ", int(time.time() * 1000) - tmp)
                    if ret != -1:
                        print(f'检测到数字: {ret}')
                    res.append(vision_position(x=(left + right) / 2, y=(top + down) / 2, target_number=ret))
                else:
                    res.append(vision_position(x=(left + right) / 2, y=(top + down) / 2, target_number=-1))
                # 在原图上画框
                if view_img:  # Add bbox to image
                    c = int(cls)  # integer class
                    label = f'{names[c]} {conf:.2f}'
                    annotator.box_label(xyxy, label, color=colors(c, True))

        im0 = annotator.result()

        # 若设置展示，则画出图片/视频
        if view_img:
            ratio = 720 / im0.shape[0]
            im0 = cv2.resize(im0, None, fx=ratio, fy=ratio)
            cv2.imshow('0', im0)
            cv2.waitKey(1)
    return res