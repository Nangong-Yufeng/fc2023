from navigation import Waypoint, set_home, mode_set, arm, wp_circle_course, wp_straight_course, mission_upload, rec_match_received, bomb_drop, gain_track_of_time, gain_ground_speed
from pymavlink import mavutil
import time


'''
飞行前准备
'''
# 连接飞行器  device部分，可以在mission planner中成功连接后直接复制过来
# the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
the_connection = mavutil.mavlink_connection('/COM3', baud=57600)

# 测试投弹装置
if input("输入0测试投弹，输入其他跳过： ") == '0':
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1000, 0, 0, 0, 0, 0)
    time.sleep(1)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 2000, 0, 0, 0, 0, 0)
    print("投弹装置测试完成")

if input("输入任意内容投弹： "):
    bomb_drop(the_connection)