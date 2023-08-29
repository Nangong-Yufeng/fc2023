from pymavlink import mavutil
from .preflight import arm, mode_set, set_home
from .mission import wp_straight_course, wp_circle_course, yard_fly, clear_waypoint
from .class_list import Position_relative, Waypoint
from .error_process import rec_match_received, retry_fuc_para1
from .get_para import gain_ground_speed, gain_transform_frequency


track_list = []

the_connection = mavutil.mavlink_connection('/dev/ttyUSB1', baud=57600)
#the_connection = mavutil.mavlink_connection('udpin:localhost:14550')
'''
SYSID_THISMAV = "SYSID_THISMAV"
parameter_set_message = mavutil.mavlink.MAVLink_param_set_message(
    target_system=the_connection.target_system,
    target_component=the_connection.target_component,
    param_id=SYSID_THISMAV.encode("utf-8"),
    param_value=20,
    param_type=mavutil.mavlink.MAV_PARAM_SR1_POSITION
)
the_connection.mav.send(parameter_set_message)

the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                    mavutil.mavlink.MAV_CMD_DO_SET_PARAMETER, mavutil.mavlink.SR1_POSITION, 20, 0, 0, 0, 0, 0)
msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
'''
if input("输入0测试数传传输频率（大概需要10秒），输入其他跳过： ") == '0':
    frequency = gain_transform_frequency(the_connection)
    print("数传传输频率：", frequency, "Hz")

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
