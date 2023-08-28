"""
这是将在比赛时被运行的文件
"""

from utils import title
import time
from vision.detect import Vision
from navigation import Waypoint, set_home, mode_set, arm, wp_circle_course, wp_straight_course, mission_upload, rec_match_received, gain_transform_frequency
from pymavlink import mavutil


def vision_test_court(the_connection):
    # 已知航点(操场的四个角)
    a = input("输入需要的环线高度（输入为空则默认为120米）： ")
    if a == '':
        alt = 120
    else:
        alt = int(a, base=10)
    print(alt)

    wp1 = Waypoint(22.5899275, 113.9751526, alt)
    wp2 = Waypoint(22.5899248, 113.9755938, alt)
    wp3 = Waypoint(22.5909185, 113.9755938, alt)
    wp4 = Waypoint(22.5909266, 113.9752198, alt)
    wp = [wp1, wp2, wp3, wp4]

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

    return wp_list

'''
帅(😅)
'''
title.printTitle()

'''
飞行前准备
'''
# 连接飞行器  device部分，可以在mission planner中成功连接后直接复制过来
# the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
the_connection = mavutil.mavlink_connection('/COM3', baud=57600)

# 测试投弹装置
if input("输入0测试投弹，输入其他跳过： ") == '0':
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1000, 0, 0, 0, 0, 0)
    time.sleep(1)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 2000, 0, 0, 0, 0, 0)
    print("投弹测试完成")

if input("输入0测试数传传输频率（大概需要10秒），输入其他跳过： ") == '0':
    frequency = gain_transform_frequency(the_connection)
    print("数传传输频率：", frequency, "Hz")

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

# 定义轨迹集
track_list = []


"""
标靶识别
"""
# 参数和初始化
vis = Vision(source=0, device='0', conf_thres=0.7)

# 循环侦察任务
while True:
    wp_list = vision_test_court(the_connection)

    if input("输入0切换自动模式开始任务（若已通过其他方式切换到自动，可输入其他跳过）： ") == '0':
        mode_set(the_connection, 10)

    while rec_match_received(the_connection, 'MISSION_CURREN T').seq < len(wp_list) - 1:
        cur = int(time.time() * 1000)
        vis.run()
        pre = int(time.time() * 1000)
        # print(pre - cur, 'ms')
    mode_set(the_connection, 11)
    print("circle completed, stand by at home")
