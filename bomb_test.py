from navigation import (gain_position_now, bomb_drop, Waypoint, set_home, mode_set,
                        gain_ground_speed, gain_posture_para, force_arm, reboot, command_retry, gain_transform_frequency,
                        wp_bombing_course, mission_upload, wp_detect_course, wp_circle_course)
from pymavlink import mavutil
import time

# 连接飞行器  device部分，可以在mission planner中成功连接后直接复制过来
the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
# the_connection = mavutil.mavlink_connection('/COM3', baud=57600)

#gain_transform_frequency(the_connection)

mode_set(the_connection, 0)
'''
while input("gain position: ") != 0:
    print(input("position inform: "))
    wp = gain_position_now(the_connection)
    print(wp.lat, wp.lon)


reboot(the_connection)

command_retry(the_connection, 'mode_set', 0)

arm_check(the_connection)
'''

# 设置home点
home_position = Waypoint(22.7526209, 113.88290509999999, 0)
#command_retry(the_connection, 'set_home', home_position)
target2 = Waypoint(22.7525209,113.88280509999999, 0)
#gain_transform_frequency(the_connection)

target = home_position
course = wp_detect_course(target, 30, approach_angle='north')#wp_circle_course([target, target2], angle=270, precision=3)
mission_upload(the_connection, course, home_position)

#command_retry(the_connection, 'arm')

# 测试投弹装置
if input("输入0测试投弹，输入其他跳过： ") == '0':
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1000, 0, 0, 0, 0, 0)
    time.sleep(1)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 2000, 0, 0, 0, 0, 0)
    print("投弹装置测试完成")

# 投弹
input("输入任意内容投弹： ")
bomb_drop(the_connection)

position = gain_position_now(the_connection)
posture = gain_posture_para(the_connection)
speed_list = []
vx = vy = vz = direction = 0
alt = 0

# 由于瞬时读取的速度和高度值非常奇怪，使用连续读取几次的方式考察情况是否有改善
for count in range(0, 5):
    speed = gain_ground_speed(the_connection)
    speed_list.append(speed)
    vx += speed_list[count].vx
    vy += speed_list[count].vy
    vz += speed_list[count].vz
    direction += speed_list[count].direction
    alt += gain_position_now(the_connection).alt
length = len(speed_list)
vx /= length
vy /= length
vz /= length
direction /= length
alt /= length

print("bomb away!")
print("原始数据： ")
print("位置： lat ", position.lat, " lon ", position.lat, " alt ", position.alt)
print("速度:  north ", vx, 'east', vy, " down ", vz)
print("姿态： roll ", posture.roll, " pitch ", posture.pitch, " yaw ", posture.yaw)
print("方向角: direction ", direction)

# 将时间和投弹位资信息记录到文件中
localtime = time.localtime(time.time())
time_data = str(localtime.tm_year) + '.' + str(localtime.tm_mon) + '.' + str(localtime.tm_mday) + ' ' + str(
    localtime.tm_hour) + ':' + str(localtime.tm_min) + ':' + str(localtime.tm_sec)
with open(file='/home/bobo/fc2023/data.txt', mode='a') as f:
    f.write(time_data)
    f.write('\n')
    f.write("位置： lat " + str(position.lat) + " lon " + str(position.lat) + " alt " + str(position.alt))
    f.write('\n')
    f.write("速度:  north " + str(vx) + " east " + str(vy) + " down " + str(vz))
    f.write('\n')
    f.write("姿态： roll " + str(posture.roll) + " pitch " + str(posture.pitch) + " yaw " + str(posture.yaw))
    f.write('\n')
    f.write("方向（可用性未知） direction " + str(direction))
    f.write("\n")

# 落点测量
input("落地后，输入任意内容测量落点位置： ")
wp = gain_position_now(the_connection)
print("落点坐标 lat ", wp.lat, " lon ", wp.lon)
with open(file='/home/bobo/fc2023/data.txt', mode='a') as f:
    f.write("落点坐标 lat ")
    f.write(str(wp.lat))
    f.write(" lon ")
    f.write(str(wp.lon))
    f.write('\n\n')
