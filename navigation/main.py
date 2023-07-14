import time
from pymavlink import mavutil
from preflight import arm, mode_set, set_home
from class_list import Position_relative, Target_position

#连接飞行器
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')

while True:
    if(arm(the_connection) < -1):
        break
    if(mode_set(the_connection, 10) < -1):
        break
    position = Position_relative(-353622066, 1491651135, 10)
    if set_home(the_connection, 0, position) < -1:
        break
    target = Target_position(position)
    target.distance(the_connection)
    time.sleep(5)
