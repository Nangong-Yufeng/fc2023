from pymavlink import mavutil
from .preflight import arm, mode_set, set_home
from .mission import wp_straight_course, wp_circle_course, yard_fly, clear_waypoint
from .class_list import Position_relative, Waypoint
from .error_process import rec_match_received, retry_fuc_para1


track_list = []

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
#the_connection = mavutil.mavlink_connection('/dev/serial/by-id/usb-ArduPilot_CUAVv5_2F001F000850304E35313320-if00', baud=9600)
#the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
'''
time_list = []

while True:
  msg = rec_match_received(the_connection, 'GLOBAL_POSITION_INT')
  if len(time_list) <= 50:
      time_list.append(msg.time_boot_ms)
  else:
      frequency = 50 / (time_list[50] - time_list[0]) * 1000
      print("frequency: ", frequency)
      time_list = []
  print(msg.time_boot_ms, msg.relative_alt)
'''

retry_fuc_para1(the_connection, mode_set, 0)

# 设置飞行器home点
home_position = Position_relative(22.5903516, 113.9755156, 0)

set_home(the_connection, home_position)

arm(the_connection)

# 设置航点
wp2 = Waypoint(22.5899248, 113.9755938, 120)
wp1 = Waypoint(22.5899275, 113.9751526, 120)
wp3 = Waypoint(22.5909185, 113.9755938, 120)
wp4 = Waypoint(22.5909266, 113.9752198, 120)

wp = [wp1, wp2, wp3, wp4]

yard_fly(the_connection, wp, home_position, track_list)
