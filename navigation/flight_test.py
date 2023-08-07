import sys
sys.path.append('../gui')
from pymavlink import mavutil
from preflight import arm, mode_set, set_home
from mission import wp_straight_course, wp_circle_course, upload_mission_till_completed, yard_fly
from class_list import Position_relative, Waypoint

track_list = []

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
#the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

mode_set(the_connection, 0)

# 设置飞行器home点
home_position = Position_relative(22.5903516, 113.9755156, 0)

arm(the_connection)

set_home(the_connection, home_position)

# 设置航点
wp2 = Waypoint(22.5899248, 113.9755938, 120)
wp1 = Waypoint(22.5899275, 113.9751526, 120)
wp3 = Waypoint(22.5909185, 113.9755938, 120)
wp4 = Waypoint(22.5909266, 113.9752198, 120)

wp_line1 = [wp4, wp1]
wp_circle1 = [wp1, wp2]
wp_line2 = [wp2, wp3]
wp_circle2 = [wp3, wp4]

wp_list = wp_straight_course(wp_line1, 3)
wp_list.extend(wp_circle_course(wp_circle1, 3, 180, 1))
wp_list.extend(wp_straight_course(wp_line2, 3))
wp_list.extend(wp_circle_course(wp_circle2, 3, 180, 1))

yard_fly(the_connection, wp_list, home_position, track_list)
