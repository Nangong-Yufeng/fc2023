"""
这是将在比赛时被运行的文件
"""
from utils import title
import time
from vision.detect import Vision

# 0. ~
title.printTitle()

# 1. 飞行前准备

# 2. 

"""
标靶识别
"""
vis = Vision(source=0, device='0', conf_thres=0.7)

itv = 50  # 每次检测的间隔时间，单位ms
pre = int(time.time() * 1000)  # 上次检测完的时间
while True:
    if int(time.time() * 1000) > pre + itv:
        vis.run()
        pre = int(time.time() * 1000)
