import time
import sys
from pymavlink import mavutil
from preflight import arm, mode_set, set_home
from mission import mission_upload, clear_waypoint, mission_current
from class_list import Position_relative, Waypoint

#连接飞行器
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

#arm飞行器；若arm失败，则程序中断
if(arm(the_connection) < -1):
    sys.exit(1)

#用于清除当前的mission_list，不过好像有没有都一样
clear_waypoint(the_connection)

#设置飞行器模式
if(mode_set(the_connection, 13) < -1):
    sys.exit(1)

#设置飞行器home点
position = Position_relative(-35.3622066, 149.1651135, 0)
if set_home(the_connection, 0, position) < -1:
    sys.exit(1)

time.sleep(2)

#设置航点
wp1 = Waypoint(-35.3598036, 149.1647555, 30)
wp2 = Waypoint(-35.3600394, 149.1604871, 20)
wp3 = Waypoint(-35.3654404, 149.1611205, 50)
wp4 = Waypoint(-35.3654516, 149.1654714, 80)
wp = [position, wp1, wp2, wp3, wp4]

#上传航点任务
mission_upload(the_connection, wp)

if(mode_set(the_connection, 10) < -1):
    sys.exit(1)

while True:
    mission_current(the_connection, wp)
    time.sleep(1)