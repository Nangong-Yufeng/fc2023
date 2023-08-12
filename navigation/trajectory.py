from pymavlink import mavutil
from .get_para import position_now, gain_posture_para


# 为投弹位置估计准备的函数
def trajectory_cal(the_connection, time_delay, target):
    # 位置和姿态信息，暂未考虑延时问题
    position = position_now(the_connection)
    lat_now = position.lat
    lon_now = position.lon
    posture = gain_posture_para(the_connection)

    distance_to_target = 10
    return distance_to_target
