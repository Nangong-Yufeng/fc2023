"""
这是将在比赛时被运行的文件
"""

from utils import title
import time
from vision.detect import Vision
from navigation import Waypoint, set_home, mode_set, arm, yard_fly
from pymavlink import mavutil

'''
帅(😅)
'''
title.printTitle()

'''
飞行前准备
'''
# 连接飞行器
the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# 设置home点
home = Waypoint(22.5903516, 113.9755156, 0)
set_home(the_connection, home)

# 设置模式为纯手动
mode_set(the_connection, 0)

# 解锁飞机
arm(the_connection)

'''
开始自动飞行
'''
# 已知航点(操场的四个角)
wp1 = Waypoint(22.5899275, 113.9751526, 120)
wp2 = Waypoint(22.5899248, 113.9755938, 120)
wp3 = Waypoint(22.5909185, 113.9755938, 120)
wp4 = Waypoint(22.5909266, 113.9752198, 120)
wp = [wp1, wp2, wp3, wp4]

# 定义轨迹集
track_list = []

# 飞行环操场航线（会阻塞直到完成，还需要和视觉对接一下，改成都在main的一个循环里）
yard_fly(the_connection, wp, home, track_list)

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
