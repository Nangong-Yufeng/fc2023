'''
本程序用于各模块内容的分开测试
'''
from navigation import (Waypoint, set_home, mode_set, arm, wp_circle_course,wp_straight_course, mission_upload,
                        rec_match_received, gain_transform_frequency, gain_track_of_time, wp_detect_course,
                        loiter_at_present, delay_eliminate, coordinate_transfer, gain_position_now, gain_ground_speed,
                        gain_posture_para,bombing_course, mission_current, bomb_drop, command_retry)
from pymavlink import mavutil
from vision.vision_class import Vision
from main import detect_completed, eliminate_error_target
import time
LEN_OF_TARGET_LIST = 200


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
    with open(file='C:/Users/35032/Desktop/gain_data_test.txt', mode='a') as f:
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
  pre_time = 0
  vis = Vision(source=0, device='0', conf_thres=0.7)
  while True:
    # 读取当前姿态和位置
    cur = int(time.time() * 1000)
    time_stamp = gain_track_of_time(the_connection, track_list)[0]
    print("循环时间： ", time_stamp - pre_time)
    pre_time = time_stamp
    # 截图
    time_vision = int(time.time() * 1000)
    vis.shot()
    # 视觉处理
    vision_position_list = vis.run()
    print("视觉时间： ", int(time.time() * 1000) - time_vision)
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
            with open(file='C:/Users/35032/Desktop/location.txt', mode='a') as f:
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



# 测试 过程淘汰 算法的正确性，需要vis参数设置为视频导入
def test_target_selection(the_connection):
    # 参数和初始化
    vis = Vision(source=0, device='0', conf_thres=0.7)
    track_list = []
    target_list = []
    target_dict = {}
    target_result = [-1, -1, -1]
    result = -1

    while result < 0:
        # 截图
        vis.shot()
        if vis.im0 is None:
            print("signal lost")
            continue

        inform = gain_track_of_time(the_connection, track_list)
        time_stamp = inform[0]

        # 视觉处理
        vision_position_list = vis.run()
        # pre = int(time.time() * 1000)
        # print(pre - cur, 'ms')

        # 进行坐标解算和靶标信息存储

        # 检测到靶标
        if len(vision_position_list) != 0:
            for n in range(len(vision_position_list)):
                track = delay_eliminate(track_list, time_stamp)
                target = coordinate_transfer(track.lat, track.lon, track.alt, track.yaw,
                                             track.pitch, track.roll, vision_position_list[n].x,
                                             vision_position_list[n].y, vision_position_list[n].number)
                # 视觉识别成功但数字识别失败
                if target.number < 0:
                    continue
                # 数字识别得到结果
                else:
                    target_list.append(target)
                    # 该目标是第一次出现
                    if target_dict.get(target.number, __default=-1) < 0:
                        target_dict[target.number] = 1
                    # 该目标不是第一次出现，且数量小于指定数量
                    elif target_dict.get(target.number, __default=-1) < 0.3 * LEN_OF_TARGET_LIST:
                        target_dict[target.number] += 1
                    # 该目标不是第一次出现，但存储数量已经达到指定上限
                    else:
                        continue
            # 如果超出设定范围，删除数量最少的一项
            eliminate_error_target(target_dict)

            # 判定侦察任务是否完成， 若得到探测结果，传入target列表，长度为3
            target_result = detect_completed(target_dict)
            result = target_result[0]

        # 没有检测到靶标
        else:
            result = -1
    print("detection completed!")


'''
测试进程
'''
the_connection = mavutil.mavlink_connection('/COM5', baud=57600)

command_retry(the_connection, 'mode_set', 0)

if input("输入O获取坐标, 输入其他跳过： ") == '0':
    wp = gain_position_now(the_connection)
    print("坐标 lat:", wp.lat, " lon:", wp.lon)

if input("输入0测试数传传输频率（大概需要10秒），输入其他跳过： ") == '0':
    frequency = gain_transform_frequency(the_connection)
    print("数传传输频率：", frequency, "Hz")

# 设置home点
home_position = Waypoint(22.5904647, 113.9623430, 0)
command_retry(the_connection, 'set_home', home_position)

# 图像参数和初始化
vis = Vision(source=0, device='0', conf_thres=0.7)

track_list = []

command_retry(the_connection, 'arm')

test_location_transfer(the_connection, track_list)
# test_course_bombing(the_connection, home_position)
