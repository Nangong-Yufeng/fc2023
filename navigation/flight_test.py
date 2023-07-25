import time
import sys
from pymavlink import mavutil
from preflight import arm, mode_set, set_home
from mission import mission_upload, clear_waypoint, mission_current, wp_straight_course, wp_circle_course, upload_mission_till_completed
from class_list import Position_relative, Waypoint

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

msg = the_connection.recv_match(type="GLOBAL_POSITION_INT", blocking=True)
print(msg)

#设置飞行器home点
home_position = Position_relative(22.5903516, 113.9755156, 0)
if set_home(the_connection, 0, home_position) < -1:
    sys.exit(1)

time.sleep(1)

#设置航点
#wp1 = Waypoint(22.5898966, 113.9750207, 100)
wp1 = Waypoint(22.5899248, 113.9755938, 100)
wp2 = Waypoint(22.5899275, 113.9756671, 100)
wp3 = Waypoint(22.5909185, 113.9755938, 100)
wp4 = Waypoint(22.5909266, 113.9752198, 100)

wp_line1 = [wp1, wp2]
wp_circle1 = [wp2, wp3]
wp_line2 = [wp3, wp4]
wp_circle2 = [wp4, wp1]

while True:
    upload_mission_till_completed(the_connection, wp_straight_course(wp_line1, 5), home_position)
    upload_mission_till_completed(the_connection, wp_circle_course(wp_circle1, 10, 240, -1), home_position)
    upload_mission_till_completed(the_connection, wp_straight_course(wp_line2, 5), home_position)
    upload_mission_till_completed(the_connection, wp_circle_course(wp_circle2, 10, 240, -1), home_position)

