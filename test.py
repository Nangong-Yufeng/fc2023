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
                        ,wp_detect_course_HeBei, wp_detect_course_HeBei_2g)
from pymavlink import mavutil
# 目标字典的目标存储个数
LEN_OF_TARGET_LIST = 50
TIME_DELAY_MS = 150
DETECT_ANGLE = 55
APPROACH_ANGLE = 129  # 投弹时的进近航向，北起点逆时针
DETECT_TIME_LIMIT = int(3 * 60 * 1000)
DETECT_ACC = 6  # m
wp_home = Waypoint(38.543938, 115.04040769999999, 0)
wp_start = Waypoint(38.5590444, 115.1420724, 15)  # B组，逆时针侦察
final_target_position = Waypoint(38.5592206, 115.1421583, 0)
mission_start_time = 0

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

detect_course = wp_detect_course_HeBei_2g(None, wp_start, approaching=DETECT_ANGLE, direction=1)
mission_upload(the_connection, detect_course, wp_home)

wp_list = wp_bombing_course(final_target_position, APPROACH_ANGLE, 'clock')
# mission_upload(the_connection, wp_list, wp_home)
