"""
程序修改自yolov5.detect.py

run为主要函数，含多个参数

main中调用run所传参数为主要参数，相关注释写在main中

"""

# YOLOv5 🚀 by Ultralytics, AGPL-3.0 license
"""
Run YOLOv5 detection inference on images, videos, directories, globs, YouTube, webcam, streams, etc.

Usage - sources:
    $ python detect.py --weights yolov5s.pt --source 0                               # webcam
                                                     img.jpg                         # image
                                                     vid.mp4                         # video
                                                     screen                          # screenshot
                                                     path/                           # directory
                                                     list.txt                        # list of images
                                                     list.streams                    # list of streams
                                                     'path/*.jpg'                    # glob
                                                     'https://youtu.be/Zgi9g1ksQHc'  # YouTube
                                                     'rtsp://example.com/media.mp4'  # RTSP, RTMP, HTTP stream

Usage - formats:
    $ python detect.py --weights yolov5s.pt                 # PyTorch
                                 yolov5s.torchscript        # TorchScript
                                 yolov5s.onnx               # ONNX Runtime or OpenCV DNN with --dnn
                                 yolov5s_openvino_model     # OpenVINO
                                 yolov5s.engine             # TensorRT
                                 yolov5s.mlmodel            # CoreML (macOS-only)
                                 yolov5s_saved_model        # TensorFlow SavedModel
                                 yolov5s.pb                 # TensorFlow GraphDef
                                 yolov5s.tflite             # TensorFlow Lite
                                 yolov5s_edgetpu.tflite     # TensorFlow Edge TPU
                                 yolov5s_paddle_model       # PaddlePaddle
"""

import os
import platform
import sys
from pathlib import Path

import torch

FILE = Path(__file__).resolve()
ROOT = FILE.parents[0]  # YOLOv5 root directory
if str(ROOT) not in sys.path:
    sys.path.append(str(ROOT))  # add ROOT to PATH
ROOT = Path(os.path.relpath(ROOT, Path.cwd()))  # relative

from models.common import DetectMultiBackend
from utils.dataloaders import IMG_FORMATS, VID_FORMATS, LoadImages, LoadScreenshots, LoadStreams
from utils.general import (LOGGER, Profile, check_file, check_img_size, check_imshow, check_requirements, colorstr, cv2,
                           increment_path, non_max_suppression, print_args, scale_boxes, strip_optimizer, xyxy2xywh)
from utils.plots import Annotator, colors, save_one_box
from utils.torch_utils import select_device, smart_inference_mode

import rotate
import crop

@smart_inference_mode()
def run(
        weights=ROOT / 'yolov5s.pt',  # model path or triton URL
        source=ROOT / 'data/images',  # file/dir/URL/glob/screen/0(webcam)
        data=ROOT / 'data/coco128.yaml',  # dataset.yaml path
        imgsz=(640, 640),  # inference size (height, width)
        conf_thres=0.25,  # confidence threshold
        iou_thres=0.45,  # NMS IOU threshold
        max_det=1000,  # maximum detections per image
        device='0',  # cuda device, i.e. 0 or 0,1,2,3 or cpu
        view_img=False,  # show results
        save_txt=False,  # save results to *.txt
        save_conf=False,  # save confidences in --save-txt labels
        save_crop=False,  # save cropped prediction boxes
        nosave=False,  # do not save images/videos
        classes=None,  # filter by class: --class 0, or --class 0 2 3
        agnostic_nms=False,  # class-agnostic NMS
        augment=False,  # augmented inference
        visualize=False,  # visualize features
        update=False,  # update all models
        project=ROOT / 'runs/detect',  # save results to project/name
        name='exp',  # save results to project/name
        exist_ok=False,  # existing project/name ok, do not increment
        line_thickness=3,  # bounding box thickness (pixels)
        hide_labels=False,  # hide labels
        hide_conf=False,  # hide confidences
        half=False,  # use FP16 half-precision inference
        dnn=False,  # use OpenCV DNN for ONNX inference
        vid_stride=1,  # video frame-rate stride
):
    source = str(source)
    save_img = not nosave and not source.endswith('.txt')  # save inference images
    is_file = Path(source).suffix[1:] in (IMG_FORMATS + VID_FORMATS)
    is_url = source.lower().startswith(('rtsp://', 'rtmp://', 'http://', 'https://'))
    webcam = source.isnumeric() or source.endswith('.streams') or (is_url and not is_file)
    screenshot = source.lower().startswith('screen')
    if is_url and is_file:
        source = check_file(source)  # download

    # Directories
    save_dir = increment_path(Path(project) / name, exist_ok=exist_ok)  # increment run
    (save_dir / 'labels' if save_txt else save_dir).mkdir(parents=True, exist_ok=True)  # make dir

    # Load model
    device = select_device(device)
    model = DetectMultiBackend(weights, device=device, dnn=dnn, data=data, fp16=half)
    stride, names, pt = model.stride, model.names, model.pt
    imgsz = check_img_size(imgsz, s=stride)  # check image size

    # Dataloader
    bs = 1  # batch_size
    if webcam:
        # 视频流的view_img = True
        view_img = check_imshow(warn=True)
        dataset = LoadStreams(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
        bs = len(dataset)
    elif screenshot:
        dataset = LoadScreenshots(source, img_size=imgsz, stride=stride, auto=pt)
    else:
        dataset = LoadImages(source, img_size=imgsz, stride=stride, auto=pt, vid_stride=vid_stride)
    vid_path, vid_writer = [None] * bs, [None] * bs

    # Run inference
    model.warmup(imgsz=(1 if pt or model.triton else bs, 3, *imgsz))  # warmup
    seen, windows, dt = 0, [], (Profile(), Profile(), Profile())

    """
    path 图片/视频路径
    im 进行resize+pad后的图片，格式(c, h, w)
    im0s 原size图片(h, w, c)
    vid_cap 读取图片时为None，读取视频时为视频源
    s 输出信息
    """
    for path, im, im0s, vid_cap, s in dataset:
        with dt[0]:
            im = torch.from_numpy(im).to(model.device) # Tensor
            im = im.half() if model.fp16 else im.float()  # uint8 to fp16/32
            im /= 255  # 0 - 255 to 0.0 - 1.0
            # 没有batch_size 时，在前面添加一个轴
            if len(im.shape) == 3:
                im = im[None]  # expand for batch dim

        # Inference
        with dt[1]:
            visualize = increment_path(save_dir / Path(path).stem, mkdir=True) if visualize else False
            pred = model(im, augment=augment, visualize=visualize)

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
        # Second-stage classifier (optional)
        # pred = utils.general.apply_classifier(pred, classifier_model, im, im0s)

        # Process predictions
        """
        对每一张图做处理
        循环次数等于batch_size
        """
        for i, det in enumerate(pred):  # per image
            seen += 1
            if webcam:  # batch_size >= 1
                p, im0, frame = path[i], im0s[i].copy(), dataset.count
                s += f'{i}: '
            else:
                p, im0, frame = path, im0s.copy(), getattr(dataset, 'frame', 0)

            p = Path(p)  # to Path
            save_path = str(save_dir / p.name)  # im.jpg
            txt_path = str(save_dir / 'labels' / p.stem) + ('' if dataset.mode == 'image' else f'_{frame}')  # im.txt

            s += '%gx%g ' % im.shape[2:]  # print string
            gn = torch.tensor(im0.shape)[[1, 0, 1, 0]]  # normalization gain whwh 获得每一张图像的whwh，存进矩阵，方便后续归一化
            imc = im0.copy() if save_crop else im0  # for save_crop
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
                    s += f"{n} {names[int(c)]}{'s' * (n > 1)}, "  # add to string

                # Write results
                # reco_val = []
                for *xyxy, conf, cls in reversed(det): # reversed反转列表顺序
                    tlbr = torch.tensor(xyxy).view(1, 4).view(-1).tolist()
                    img = im0_copy.copy()
                    hei = img.shape[0]
                    wid = img.shape[1]
                    img = img[max(0, int(tlbr[1])-5):min(int(tlbr[3])+5, hei), max(0, int(tlbr[0])-5):min(int(tlbr[2])+5, wid)]
                    img_rotated = rotate.rotate(img)
                    # cv2.imwrite('tmp.jpg', img_rotated)
                    img_crop = crop.crop(img_rotated)
                    if img_crop.shape[:2] == (0, 0):
                        continue
                    cv2.imshow('img_crop', img_crop)
                    cv2.waitKey(0)
                    cv2.destroyWindow('img_crop')
    #                 if save_txt:  # Write to file
    #                     # 将xyxy转换为xywh，并除以wh，做归一化，转换为列表保存
    #                     # view(-1): 将张量展平，如size[2, 3, 4] --> size[24]
    #                     # tolist(): 将张量转化为list
    #                     xywh = (xyxy2xywh(torch.tensor(xyxy).view(1, 4)) / gn).view(-1).tolist()  # normalized xywh
    #                     line = (cls, *xywh, conf) if save_conf else (cls, *xywh)  # label format
    #                     with open(f'{txt_path}.txt', 'a') as f:
    #                         f.write(('%g ' * len(line)).rstrip() % line + '\n')
    #
                    # 在原图上画框
                    if save_img or save_crop or view_img:  # Add bbox to image
                        c = int(cls)  # integer class
                        label = None if hide_labels else (names[c] if hide_conf else f'{names[c]} {conf:.2f}')
                        annotator.box_label(xyxy, label, color=colors(c, True))

    #
    #                 if save_crop:
    #                     tmp_path = save_one_box(xyxy, imc, file=save_dir / 'crops' / names[c] / f'{p.stem}.jpg', BGR=True)
    #                     # reco_val.append(threshold.thres(tmp_path))
    #             # print(reco_val.sort())
    #
            # Stream results
            im0 = annotator.result()
            # 若设置展示，则画出图片/视频
            if view_img:
                if platform.system() == 'Linux' and p not in windows:
                    windows.append(p)
                    cv2.namedWindow(str(p), cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)  # allow window resize (Linux)
                    cv2.resizeWindow(str(p), im0.shape[1], im0.shape[0])
                cv2.imshow(str(p), im0)
                cv2.waitKey(1)  # 1 millisecond
    #
    #         # Save results (image with detections)
    #         if save_img:
    #             if dataset.mode == 'image':
    #                 cv2.imwrite(save_path, im0)
    #             else:  # 'video' or 'stream'
    #                 if vid_path[i] != save_path:  # new video
    #                     vid_path[i] = save_path
    #                     if isinstance(vid_writer[i], cv2.VideoWriter):
    #                         vid_writer[i].release()  # release previous video writer
    #                     if vid_cap:  # video
    #                         fps = vid_cap.get(cv2.CAP_PROP_FPS)
    #                         w = int(vid_cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    #                         h = int(vid_cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    #                     else:  # stream
    #                         fps, w, h = 30, im0.shape[1], im0.shape[0]
    #                     save_path = str(Path(save_path).with_suffix('.mp4'))  # force *.mp4 suffix on results videos
    #                     vid_writer[i] = cv2.VideoWriter(save_path, cv2.VideoWriter_fourcc(*'mp4v'), fps, (w, h))
    #                 vid_writer[i].write(im0)
    #
    #     # Print time (inference-only)
    #     LOGGER.info(f"{s}{'' if len(det) else '(no detections), '}{dt[1].dt * 1E3:.1f}ms")
    #
    # # Print results
    # t = tuple(x.t / seen * 1E3 for x in dt)  # speeds per image
    # LOGGER.info(f'Speed: %.1fms pre-process, %.1fms inference, %.1fms NMS per image at shape {(1, 3, *imgsz)}' % t)
    # if save_txt or save_img:
    #     s = f"\n{len(list(save_dir.glob('labels/*.txt')))} labels saved to {save_dir / 'labels'}" if save_txt else ''
    #     LOGGER.info(f"Results saved to {colorstr('bold', save_dir)}{s}")
    # if update:
    #     strip_optimizer(weights[0])  # update model (to fix SourceChangeWarning)

if __name__ == '__main__':
    # weights 训练出来的权重文件
    # source 待检测的源文件，0为电脑自带摄像头
    # data 配置数据文件, 包括image/label/classes等信息
    # conf_thres 置信度阈值，小于该阈值的框不输出
    # device 设置设备CPU/CUDA
    # view_img 是否展示画框之后的视频/图片
    # vid_stride 视频每几帧做一次检测
    run(weights=ROOT / 'best.pt', source=0, data=ROOT / '0515.yaml', conf_thres=0.7, device='0', view_img=True, vid_stride=90)
