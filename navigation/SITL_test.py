import time

from pymavlink import mavutil
from .preflight import arm, mode_set, set_home
from .mission import clear_waypoint, rec_match_received, execute_bomb_course, loiter_at_present, mission_upload, wp_detect_course
from .class_list import Position_relative, Waypoint
from .get_para import gain_position_now, gain_ground_speed

# 连接飞行器
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

# arm飞行器
arm(the_connection)

# 用于清除当前的mission_list，不过好像有没有都一样
clear_waypoint(the_connection)

# 设置飞行器模式
mode_set(the_connection, 0)

# 设置飞行器home点
home_position = Position_relative(-35.3622066, 149.1651135, 0)
set_home(the_connection, home_position)

# 用于记录航点位置和到达该处的时间戳
track_list = []

# 设置航点
wp1 = Waypoint(-35.3598036, 149.1647555, 30)
wp2 = Waypoint(-35.3600394, 149.1604871, 30)
wp3 = Waypoint(-35.3654404, 149.1611205, 50)
wp4 = Waypoint(-35.3654516, 149.1654714, 80)
wp_target = Waypoint(-35.3598136, 149.1647655, 10)

wp = [wp4, wp2]
wp_detect = wp_detect_course([wp2, wp_target], 3, 100)

# 起飞并飞往侦察点
mode_set(the_connection, 13)

time.sleep(2)

mission_upload(the_connection, wp_detect, home_position)

mode_set(the_connection, 10)

while input("若视觉已返回坐标信息，输入零以继续；若识别不成功，输入检测高度: ") != '0':
    wp_detect = wp_detect_course([wp2, wp_target], 3, 30)
    mission_upload(the_connection, wp_detect, home_position)
    mode_set(the_connection, 10)

loiter_at_present(the_connection, 100)

#clear_waypoint(the_connection)

# 执行投弹航线
execute_bomb_course(the_connection, home_position, track_list, gain_position_now(the_connection), wp_target, precision=3, course_len=200, direction=1, radius=200)

mode_set(the_connection, 11)
