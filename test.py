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
                        target_match, coordinate_aver_cal)
from pymavlink import mavutil
# 目标字典的目标存储个数
LEN_OF_TARGET_LIST = 50
TIME_DELAY_MS = 150
DETECT_ANGLE = 231
APPROACH_ANGLE = 309  # 投弹时的进近航向，北起点逆时针
DETECT_TIME_LIMIT = int(3 * 60 * 1000)
DETECT_ACC = 6  # m
wp_home = Waypoint(38.543938, 115.04040769999999, 0)
wp_start = Waypoint(38.5590428, 115.1420812, 15)  # A组，顺时针侦察
final_target_position = Waypoint(38.5592552, 115.1421690, 0)
mission_start_time = 0
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
the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)#921600)

# force_arm(the_connection)

gain_transform_frequency(the_connection)

A_target1 = Waypoint(38.557288, 115.139136, 0)
A_target2 = Waypoint(38.557168, 115.138972, 0)
A_target3 = Waypoint(38.557443, 115.139024, 0)
A_target4 = Waypoint(38.557302, 115.138819, 0)


B_target1 = Waypoint(38.559180, 115.142050, 0)
B_target2 = Waypoint(38.559315, 115.142202,     0)
B_target3 = Waypoint(38.559314, 115.141904, 0)
B_target4 = Waypoint(38.559445, 115.142059, 0)


wp = [A_target1, A_target2, A_target3, A_target4, B_target1, B_target2, B_target3, B_target4]
mission_upload(the_connection, wp, wp_home)

