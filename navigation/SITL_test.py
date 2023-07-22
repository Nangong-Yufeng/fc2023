import time
import sys
from pymavlink import mavutil
from preflight import arm, mode_set, set_home
from mission import mission_upload, clear_waypoint, mission_current, wp_straight_course, wp_circle_course, wp_turn_course
from class_list import Position_relative, Waypoint
from set_para import set_speed
from get_para import gain_posture_para, gain_wp_now

#连接飞行器
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
#the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)


#arm飞行器；若arm失败，则程序中断
if(arm(the_connection) < -1):
    sys.exit(1)

#用于清除当前的mission_list，不过好像有没有都一样
clear_waypoint(the_connection)

#设置飞行器模式
if(mode_set(the_connection, 13) < -1):
    sys.exit(1)

#设置飞行器home点
home_position = Position_relative(-35.3622066, 149.1651135, 0)
if set_home(the_connection, 0, home_position) < -1:
    sys.exit(1)

time.sleep(1)

#设置航点
wp1 = Waypoint(-35.3598036, 149.1647555, 30)
wp2 = Waypoint(-35.3600394, 149.1604871, 20)
wp3 = Waypoint(-35.3654404, 149.1611205, 50)
wp4 = Waypoint(-35.3654516, 149.1654714, 80)
wp = [wp2, wp1]
wp_new = [wp1, wp4]

#上传航点任务
wp_line = wp_straight_course(wp, 30)
wp_circle = wp_circle_course(wp, 50, -1)
#wp_turn_course()

mission_upload(the_connection, wp_circle, home_position)


if(mode_set(the_connection, 10) < -1):
    sys.exit(1)

#set_speed(the_connection, 15)
#
#mission_upload(the_connection, wp_new, home_position)

while True:
    #mission_current(the_connection, wp_line)
    gain_wp_now(the_connection)