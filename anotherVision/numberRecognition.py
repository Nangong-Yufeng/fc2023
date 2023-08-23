import cv2
import torch
# from utils import getWeightPath
import models.mnist_model
import numpy as np
from PIL import Image
from torchvision import transforms
import pytesseract

class NumberRecognizer:
    """负责完成数字识别任务，由标靶识别模块使用
    """

    def __init__(self, weight_path: str) -> None:
        """
        Args:
            path (str): 要载入的模型的路径
        """
        pass


    def recognize(self, image) -> int:
        """对输入图像进行识别

        参数:
            image (np.ndarray): 要检测数字的图片

        返回值:
            int: 检测得到的图片中数字的结果
        """
        # 使用前请查看anotherVision下的README.md
        text = pytesseract.image_to_string(image, lang='digits', config='--psm 10 --oem 3 -c tessedit_char_whitelist=0123456789')
        # 去除识别结果中的空格和换行符等不可见字符
        text = ''.join(text.split())
        if len(text) == 0 or len(text) == 1:
            return -1
        # 三四位是误把边缘识别为1
        if len(text) == 3:
            text = text[-2:]
        if len(text) == 4:
            text = text[-3:-2]
        return int(text)


if __name__ == "__main__":
    nr = NumberRecognizer("C:/Users/lwy/Desktop/nangongyufeng/fc2023/anotherVision/weights/cnn2.pkl")
    image = cv2.imread("C:/Users/lwy/Desktop/20/DJI_004567.jpg")
    predicted_number = nr.recognize(image)
    print(predicted_number)
