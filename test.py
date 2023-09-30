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
                        gain_transform_frequency)
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

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

# force_arm(the_connection)

gain_transform_frequency(the_connection)

A_target1 = Waypoint(38.557288, 115.139136, 0)
A_target2 = Waypoint(38.557168, 115.138972, 0)
A_target3 = Waypoint(38.557443, 115.139024, 0)
A_target4 = Waypoint(38.557302, 115.138819, 0)


B_target1 = Waypoint(38.559180, 115.142050, 0)
B_target2 = Waypoint(38.559315, 115.142202, 0)
B_target3 = Waypoint(38.559314, 115.141904, 0)
B_target4 = Waypoint(38.559445, 115.142059, 0)


wp = [A_target1, A_target2, A_target3, A_target4, B_target1, B_target2, B_target3, B_target4]
mission_upload(the_connection, wp, wp_home)


