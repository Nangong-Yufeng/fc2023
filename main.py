"""
这是将在比赛时被运行的文件
"""

from utils import title
import time
from vision.detect import Vision
from navigation import Waypoint, set_home, mode_set, arm, wp_circle_course, wp_straight_course, mission_upload
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
home_position = Waypoint(22.5903516, 113.9755156, 0)
set_home(the_connection, home_position)

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

# 环操场航点
wp_line1 = [wp[3], wp[0]]
wp_circle1 = [wp[0], wp[1]]
wp_line2 = [wp[1], wp[2]]
wp_circle2 = [wp[2], wp[3]]

wp_list = (wp_circle_course(wp_circle1, 3, 180, 1))
wp_list.pop(-1)
wp_list.pop(-1)
wp_list.extend(wp_straight_course(wp_line2, 3))
wp_list.pop(-1)
wp_list.extend(wp_circle_course(wp_circle2, 3, 180, 1))
wp_list.pop(-1)
wp_list.extend(wp_straight_course(wp_line1, 3))

mission_upload(the_connection, wp_list, home_position)
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
