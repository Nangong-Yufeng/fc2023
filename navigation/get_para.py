from pymavlink import mavutil
from class_list import Position_relative
import math

def gain_posture_para(the_connection):
    msg = the_connection.recv_match(type='POSITION_TARGET_GLOBAL_INT', blocking=True)
    print(msg)

def gain_wp_now(the_connection):
    msg = the_connection.recv_match(type='POSITION_TARGET_GLOBAL_INT', blocking=True)
    if 1:
        wp_now = Position_relative(msg.lat_int, msg.lon_int, msg.alt)
        print(wp_now.lat, wp_now.lon, wp_now.alt)
    else:
        print("fuck")