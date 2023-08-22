import cv2
import torch
# from utils import getWeightPath
import models.mnist_model
import numpy as np
from PIL import Image
from torchvision import transforms

target_width = 56
target_height = 28

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
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 将图像从BGR色彩空间转换为HSV色彩空间
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 定义红蓝黑白灰色的HSV范围
        lower_red1 = np.array([0, 43, 46])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([156, 43, 46])
        upper_red2 = np.array([180, 255, 255])

        lower_blue = np.array([100, 43, 46])
        upper_blue = np.array([124, 255, 255])

        lower_gray = np.array([0, 0, 46])
        upper_gray = np.array([180, 43, 225])

        lower_white = np.array([0, 0, 221])
        upper_white = np.array([180, 30, 255])

        lower_black = np.array([0, 0, 0])
        upper_black = np.array([180, 255, 46])

        # 创建一个掩码，将红蓝白色区域标记为白色，灰黑色区域标记为黑色
        mask_red1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask_red2 = cv2.inRange(hsv_image, lower_red2, upper_red2)
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
        mask_white = cv2.inRange(hsv_image, lower_white, upper_white)
        mask_gray = cv2.inRange(hsv_image, lower_gray, upper_gray)
        mask_black = cv2.inRange(hsv_image, lower_black, upper_black)

        # 将掩码应用于原始图像，将红蓝白色区域变为白色
        image[mask_red1 != 0] = [0, 0, 0]
        image[mask_red2 != 0] = [0, 0, 0]
        image[mask_blue != 0] = [0, 0, 0]
        image[mask_white != 0] = [0, 0, 0]
        image[mask_gray != 0] = [255, 255, 255]
        image[mask_black != 0] = [255, 255, 255]
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        scale_x = target_width / image.shape[1]
        scale_y = target_height / image.shape[0]
        scale = min(scale_x, scale_y)
        padded_image = np.zeros((target_height, target_width), dtype=np.uint8)
        image = cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
        # 计算缩放后图像在填充图像中的起始位置
        start_x = (padded_image.shape[1] // 2 - image.shape[1] // 2) // 2
        start_y = (padded_image.shape[0] - image.shape[0]) // 2

        # 将缩放后的图像放置在填充图像的中心
        padded_image[start_y:start_y + image.shape[0], start_x:start_x +image.shape[1] // 2] \
            = image[0: 28: 1, 0: image.shape[1] // 2: 1]
        padded_image[start_y:start_y + image.shape[0], padded_image.shape[1] // 2 + start_x: padded_image.shape[1] // 2 + start_x + image.shape[1] // 2] \
            = image[0: 28: 1, image.shape[1] // 2: image.shape[1] // 2 * 2: 1]
        image_pretreated = padded_image
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
    image = cv2.imread("C:/Users/lwy/Desktop/20/DJI_004567.jpg")
    predicted_number = nr.recognize(image)
    print(predicted_number)
