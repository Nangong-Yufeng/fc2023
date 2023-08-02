from pymavlink import mavutil
from get_para import gain_posture_para

#为视觉坐标转换准备的函数
def coordinate_transfer(the_connection, vision_target):
    pose = gain_posture_para(the_connection)