本文件夹完成从视频读入，到计算中位数标靶在图像中的相对位置并返回的任务

主要包含标靶识别与数字识别两个部分

# 标靶识别

从视频中检测标靶，将标靶截取出来，并旋转至正向，再将标靶中的数字所在白色矩形截取，传递给数字识别

主要依靠yolov5实现

datasets/0515内为数据集

在yolov5/data/0515.yaml中配置数据集路径与识别类别

train.py中需要修改--data, --epochs, --batch-size, --device，具体含义见参考资料

## 参考资料

train.py注解 https://blog.csdn.net/CharmsLUO/article/details/123542598

detect.py注解 https://blog.csdn.net/CharmsLUO/article/details/123422822
