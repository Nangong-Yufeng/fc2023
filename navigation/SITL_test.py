import time
import sys
from pymavlink import mavutil
from preflight import arm, mode_set, set_home
from mission import clear_waypoint,upload_mission_till_completed, execute_bomb_course, loiter_at_present, wp_circle_course, wp_straight_course
from class_list import Position_relative, Waypoint
from set_para import set_speed
from get_para import gain_posture_para, position_now, gain_mission, waypoint_reached

#连接飞行器
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

#arm飞行器；若arm失败，则程序中断
arm(the_connection)

#用于清除当前的mission_list，不过好像有没有都一样
clear_waypoint(the_connection)

#设置飞行器模式
mode_set(the_connection, 13)

time.sleep(1)

#设置飞行器home点
home_position = Position_relative(-35.3622066, 149.1651135, 0)
set_home(the_connection, 0, home_position)


#设置航点
wp1 = Waypoint(-35.3598036, 149.1647555, 30)
wp2 = Waypoint(-35.3600394, 149.1604871, 30)
wp3 = Waypoint(-35.3654404, 149.1611205, 50)
wp4 = Waypoint(-35.3654516, 149.1654714, 80)
wp5 = Waypoint(-35.35941937, 149.16062729, 10)

wp = [wp4, wp2]

#飞往侦察点
mission1 = [wp1]
upload_mission_till_completed(the_connection, mission1, home_position)

loiter_at_present(the_connection, 50)


if input("假设视觉已返回坐标信息，输入零以继续： ") == '0':
    pass

#执行投弹航线
execute_bomb_course(the_connection, home_position, position_now(the_connection), wp1, precision=3, line_course=200, direction=1, radius=150)

mode_set(the_connection, 11)

