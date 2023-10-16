import numpy as np

from utils import title
import threading
import queue
import time
from geopy.distance import geodesic
import numpy
from navigation import (Waypoint, mode_set, mission_upload,
                        wp_detect_course, loiter_at_present, gain_track_point,
                        detect_completed, eliminate_error_target, command_retry,
                        gain_position_now, target_transfer,
                        wrong_number, wp_bombing_course, mission_current, bomb_drop,
                        loiter, return_to_launch, initiate_bomb_drop, preflight_command
                        ,wp_detect_course_HeBei, wp_detect_course_HeBei_2g, force_arm,
                        gain_transform_frequency, k_means_calculate, target_point, target_order,
                        target_match, coordinate_aver_cal, contest_detect_course, k_means_calculate, coordinate_transfer,
                        target_point, coordinate_aver_cal, target_match,
                        mission_upload_including_bomb_drop, mission_jump_to)
from pymavlink import mavutil
# 目标字典的目标存储个数
''''
key = [5,7,9]
print(np.median(key))

A = target_point(0.0001, 0.0002, 1)
B = target_point(0.0001, 0.0002, 2)
C = target_point(0.0002, 0.0001,2)

target_list = [Waypoint(-35.3603933, 149.1602209, 1),
               Waypoint(-35.3605727, 149.1600707, 2),
               Waypoint(-35.3605716, 149.1603376, 3)]
target_num = [Waypoint(-35.3605563, 149.1602477, 6),
              Waypoint(0, 0, 6),
              Waypoint(0, 0, 6)]
              # Waypoint(-35.3603507, 149.1602598, 5),
              # Waypoint(-35.3606438, 149.1599809, 4)]

points = [target_point(0.1,0.8,1),
          target_point(0.9,0.4,1),
          target_point(14,3,5),
          target_point(0.7, 0.5, 1),
          target_point(0.3, 0.7, 1)]
coordinate_aver_cal(points,1)


a = target_match(target_list, target_num, 0)
print(a.lat, a.lon)
a = k_means_calculate([A, B, C])
'''

# 侦察航线的转弯直径
DIAMETER = 0.00060
# 航线转向方向
DIRECTION = 1
# 直线航线拓展长度
LENGTH_EXTEND = 0.00010
# 靶标坐标侦察部分的航点数量
WP_NUMBER_OF_TARGET_DETECT = 20
# 数字侦察部分的航点数量
WP_NUMBER_OF_NUMBER_DETECT = 40
# 投弹点的位置
WP_SEQ_OF_BOMB_DROP = 10
# 设定的延迟时间
TIME_DELAY_MS = 250
# 侦察航向，指南针标准
DETECT_ANGLE = 340

# 投弹进场航向，指南针标准
if DETECT_ANGLE > 180:
    APPROACH_ANGLE = DETECT_ANGLE - 180
else:
    APPROACH_ANGLE = DETECT_ANGLE + 180
'''
需要测量的坐标
'''
# home点
wp_home = Waypoint(28.5928658, 113.1872269, 0)
# 靶标区的四个顶点，注意按照侦察航线顺序
wp_boarder = [Waypoint(28.5937116, 113.1869191, 0),
              Waypoint(28.5939566, 113.1868185, 0),
              Waypoint(28.5937976, 113.1871538, 0),
              Waypoint(28.5940625, 113.1870545, 0)]
target_coordinate = [target1, target2, target3] = [Waypoint(28.5937116, 113.1869191, 0),
                                                   Waypoint(28.5939566, 113.1868185, 0),
                                                   Waypoint(28.5937976, 113.1871538, 0)]  # 三个靶标的测量位置

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=921600)

detect_course = []
# 第一圈
detect_course.extend(contest_detect_course(detect_angle=DETECT_ANGLE, start_coordinate=wp_boarder[0],
                                           end_coordinate=wp_boarder[1], direction=DIRECTION,
                                           alt_detect=15, alt_circle=35, diameter=DIAMETER,
                                           length_expend=LENGTH_EXTEND))
# 第二圈
detect_course.extend(contest_detect_course(detect_angle=DETECT_ANGLE, start_coordinate=wp_boarder[2],
                                           end_coordinate=wp_boarder[3], direction=DIRECTION,
                                           alt_detect=15, alt_circle=35, diameter=DIAMETER,
                                           length_expend=LENGTH_EXTEND))
for i in range(0, 5):
    detect_course.pop(-1)

bomb_course = []

bomb_course.extend(wp_bombing_course(target1, approach_angle=APPROACH_ANGLE, length_bomb_lead=23,
                                     turn_direction=-DIRECTION))
bomb_course.extend(wp_bombing_course(target2, approach_angle=APPROACH_ANGLE, length_bomb_lead=23,
                                     turn_direction=-DIRECTION))
bomb_course.extend(wp_bombing_course(target3, approach_angle=APPROACH_ANGLE, length_bomb_lead=23,
                                     turn_direction=-DIRECTION))

mission_course = detect_course
mission_course.extend(bomb_course)

mission_upload_including_bomb_drop(the_connection, detect_course, [35, 57, 79])
