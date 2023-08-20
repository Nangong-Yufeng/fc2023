import cv2
import torch
# from utils import getWeightPath
import models.mnist_model
import numpy as np
from PIL import Image
from torchvision import transforms


class NumberRecognizer:
    """负责完成数字识别任务，由标靶识别模块使用
    """

    def __init__(self, weight_path: str) -> None:
        """
        Args:
            path (str): 要载入的模型的路径
        """
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.model = models.mnist_model.CNN().to(self.device)
        self.model.load_state_dict(torch.load(weight_path))
        self.model.eval()
        self.image_transforms = transforms.ToTensor()
        pass

    def pretreatment(self, image) -> np.ndarray:
        image_pretreated = cv2.resize(image, (56, 28))
        image_tens_digit = image_pretreated[0: 28: 1, 0: 28: 1]
        image_units_digit = image_pretreated[0: 28: 1, 28: 56: 1]
        return image_tens_digit, image_units_digit

    def image_to_tensor(self, image):
        image_pil = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)).convert('L')
        # dim4
        image_tensor = self.image_transforms(image_pil).unsqueeze(1)
        image_tensor = image_tensor.to(self.device)
        return image_tensor

    def recognize(self, image) -> int:
        """对输入图像进行识别

        参数:
            image (np.ndarray): 要检测数字的图片

        返回值:
            int: 检测得到的图片中数字的结果
        """

        image_tens_digit, image_units_digit = self.pretreatment(image)
        tensor_tens_digit = self.image_to_tensor(image_tens_digit)
        tensor_units_digit = self.image_to_tensor(image_units_digit)
        outputs_tens_digit = self.model(tensor_tens_digit)
        outputs_units_digit = self.model(tensor_units_digit)
        predicted_tens_digit = torch.max(outputs_tens_digit.data, dim=1)[1]
        predicted_units_digit = torch.max(outputs_units_digit.data, dim=1)[1]
        predicted_number = int(predicted_tens_digit) * 10 + int(predicted_units_digit)

        return predicted_number


if __name__ == "__main__":
    nr = NumberRecognizer("C:/Users/lwy/Desktop/nangongyufeng/fc2023/anotherVision/weights/cnn2.pkl")
    image = cv2.imread("C:/Users/lwy/Desktop/fegfe.jpg", cv2.IMREAD_GRAYSCALE)
    predicted_number = nr.recognize(image)
    print(predicted_number)
