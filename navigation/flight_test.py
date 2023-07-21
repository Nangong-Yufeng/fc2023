import time
import sys
from pymavlink import mavutil
from preflight import arm, mode_set, set_home
from mission import mission_upload, clear_waypoint, mission_current
from class_list import Position_relative, Waypoint

#the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

msg = the_connection.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
print(msg)

#设置飞行器home点
position = Position_relative(22.5903516, 113.9755156, 0)
if set_home(the_connection, 0, position) < -1:
    sys.exit(1)

time.sleep(2)

#设置航点
wp1 = Waypoint(22.5898966, 113.9750207, 80)
wp2 = Waypoint(22.5899275, 113.9756671, 80)
wp3 = Waypoint(22.5909185, 113.9755938, 80)
wp4 = Waypoint(22.5909266, 113.9752198, 80)
wp5 = Waypoint(22.5899248, 113.9751526, 80)
wp6 = Waypoint(22.5903161, 113.9750581, 80)
wp = [position, wp1, wp2, wp3, wp4, wp6, wp5]

#上传航点任务
mission_upload(the_connection, wp)

#while True:
#    mission_current(the_connection, wp)
#    time.sleep(1)