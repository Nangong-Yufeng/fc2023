import torch
from utils import getWeightPath

class NumberRecognizer:
    """负责完成数字识别任务，由标靶识别模块使用
    """
    def __init__(self, model_path:str) -> None:
        """
        Args:
            path (str): 要载入的模型的路径
        """
        self.model = torch.load(model_path)
        pass

    def recognize(self, image) -> int:
        """对输入图像进行识别

        参数:
            image (np.ndarray): 要检测数字的图片

        返回值:
            int: 检测得到的图片中数字的结果
        """

        # TODO: 做一些预处理（如果需要）
    
        return self.model(image)

if __name__ == "__main__":
    nr = NumberRecognizer(getWeightPath("a_fake_number_recognizer.pth"))
    nr.recognize("this is an image or a tensor")
    