from navigation import gain_position_now, bomb_drop, Waypoint, set_home
from pymavlink import mavutil
import time

# 连接飞行器  device部分，可以在mission planner中成功连接后直接复制过来
the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
# the_connection = mavutil.mavlink_connection('/COM3', baud=57600)

# 设置home点
home_position = Waypoint(22.5903516, 113.9755156, 0)
set_home(the_connection, home_position)

# 测试投弹装置
if input("输入0测试投弹，输入其他跳过： ") == '0':
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1000, 0, 0, 0, 0, 0)
    time.sleep(1)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 2000, 0, 0, 0, 0, 0)
    print("投弹装置测试完成")

# 投弹
if input("输入任意内容投弹： "):
    bomb_drop(the_connection)

# 落点测量
if input("落地后，输入任意内容测量落点位置： "):
    wp = gain_position_now(the_connection)
    print("落点坐标 lat ", wp.lat, " lon ", wp.lon)
    with open(file='/home/bobo/fc2023/data.txt', mode='a') as f:
        f.write("落点坐标 lat ")
        f.write(str(wp.lat))
        f.write(" lon ")
        f.write(str(wp.lon))
        f.write('\n\n')
