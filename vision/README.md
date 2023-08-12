本文件夹完成从视频读入，到计算中位数标靶在图像中的相对位置并返回的任务

主要包含标靶识别与数字识别两个部分

# 环境配置

建议使用带GPU的电脑

首先安装PyCharm与Conda，新建一个虚拟Conda环境，并配置PyCharm

对于带GPU的电脑，查询电脑信息，安装匹配的CUDA，CUDNN

根据具体情况，安装torch与torchvision，要求torch>=1.7.0，torchvision>=0.8.1

本目录下含有yolov5所需的包，用控制台进入当前文件夹，使用命令：

```commandline
pip install -r requirements.txt
```

进行安装

# 标靶识别

从视频中检测标靶，将标靶截取出来，并旋转至正向，再将标靶中的数字所在白色矩形截取，传递给数字识别

主要依靠yolov5实现

datasets/0515内为数据集

在yolov5/data/0515.yaml中配置数据集路径与识别类别

train.py中需要修改--data, --epochs, --batch-size, --device，具体含义见参考资料

./weights下含s_low,s_med,m_low三个文件夹，分别为用不同权重或网络结构的训练结果

## 参考资料

train.py注解 https://blog.csdn.net/CharmsLUO/article/details/123542598

detect.py注解 https://blog.csdn.net/CharmsLUO/article/details/123422822
