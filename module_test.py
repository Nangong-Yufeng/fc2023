'''
本程序用于各模块内容的分开测试
'''
from navigation import (Waypoint, set_home, mode_set, arm, wp_circle_course,wp_straight_course, mission_upload,
                        rec_match_received, gain_transform_frequency, gain_track_of_time, wp_detect_course,
                        loiter_at_present, delay_eliminate, coordinate_transfer, gain_position_now, gain_ground_speed,
                        gain_posture_para,bombing_course, mission_current, bomb_drop)
from pymavlink import mavutil
from vision.vision_class import Vision
import time


# 信息读取的测试
def test_gain_inform(the_connection):
  while not input("enter"):
    position = gain_position_now(the_connection)
    posture = gain_posture_para(the_connection)
    speed = gain_ground_speed(the_connection)
    vx = speed.vx
    vy = speed.vy
    vz = speed.vz
    direction = speed.direction
    print("原始数据： ")
    print("位置： lat ", position.lat, " lon ", position.lon, " alt ", position.alt)
    print("速度:  north ", vx, 'east', vy, " down ", vz)
    print("姿态： roll ", posture.roll, " pitch ", posture.pitch, " yaw ", posture.yaw)
    print("方向角: direction ", direction)

    # 将时间和投弹位资信息记录到文件中
    localtime = time.localtime(time.time())
    time_data = str(localtime.tm_year) + '.' + str(localtime.tm_mon) + '.' + str(localtime.tm_mday) + ' ' + str(
        localtime.tm_hour) + ':' + str(localtime.tm_min) + ':' + str(localtime.tm_sec)
    with open(file='/home/bobo/fc2023/gain_para_test_data.txt', mode='a') as f:
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


# 测试二分法选择延迟前航点是否正确
def test_time_selecting(the_connection):
   track_list = []
   while True:
     time_data = gain_track_of_time(the_connection, track_list)
     track = delay_eliminate(track_list, time_data)
     if track != None:
        print("now:", time_data)
        print("trace:", track.time)
        print("delay: ", time_data-track.time, "\n")


def test_location_transfer(the_connection, track_list):
  # 参数和初始化
  vis = Vision(source=0, device='0', conf_thres=0.7)
  while True:
    # 读取当前姿态和位置
    cur = int(time.time() * 1000)
    time_stamp = gain_track_of_time(the_connection, track_list)
    # 截图
    vis.shot()
    # 视觉处理
    vision_position_list = vis.run()
    pre = int(time.time() * 1000)
    # print(pre - cur, 'ms')
    # 进行坐标解算和靶标信息存储
    # 检测到靶标
    if len(vision_position_list) != 0:
        for n in range(len(vision_position_list)):
            track = delay_eliminate(track_list, time_stamp)
            target = coordinate_transfer(track.lat, track.lon, track.alt, track.yaw,
                                         track.pitch, track.roll, vision_position_list[n].x,
                                         vision_position_list[n].y, vision_position_list[n].num)
            print("标靶坐标：lat = ", target.lat,", lon = ", target.lon, ", num = ", target.number)
            with open(file='/home/bobo/fc2023/gain_para_test_data.txt', mode='a') as f:
                f.write("lat: " + str(target.lat) + " lon: " + str(target.lon) + " num: " + str(target.number))
                f.write('\n')
    else:
        print("none target detected")


def test_course_bombing(the_connection, home_position):
    target = Waypoint(22.5904647, 113.9623430,10)
    input("生成航线")
    position = gain_position_now(the_connection)
    wp_list = bombing_course(position, target, 2,30,10)
    mission_upload(the_connection, wp_list, home_position)

    if input("输入0切换自动模式开始任务（请检查目标点和home点已正确设置）（若已通过其他方式切换到自动，可输入其他跳过）： ") == '0':
        mode_set(the_connection, 10)

    while mission_current(the_connection) < len(wp_list) - 3:
        test_gain_inform(the_connection)
    bomb_drop(the_connection)



the_connection = mavutil.mavlink_connection('/COM3', baud=57600)

mode_set(the_connection, 0)

if input("输入O获取坐标, 输入其他跳过： ") == '0':
    wp = gain_position_now(the_connection)
    print("坐标 lat:", wp.lat, " lon:", wp.lon)

if input("输入0测试数传传输频率（大概需要10秒），输入其他跳过： ") == '0':
    frequency = gain_transform_frequency(the_connection)
    print("数传传输频率：", frequency, "Hz")

# 设置home点
home_position = Waypoint(22.5904647, 113.9623430, 0)
#set_home(the_connection, home_position)

track_list = []

test_location_transfer(the_connection,track_list)
# test_course_bombing(the_connection, home_position)
