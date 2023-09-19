'''
本程序用于各模块内容的分开测试
'''
from navigation import (Waypoint, set_home, mode_set, arm, wp_circle_course,wp_straight_course, mission_upload,
                        rec_match_received, gain_transform_frequency, gain_track_of_time, wp_detect_course,
                        loiter_at_present, delay_eliminate, coordinate_transfer, gain_position_now, gain_ground_speed,
                        gain_posture_para,bombing_course, mission_current, bomb_drop, command_retry, loiter, wp_bombing_course)
from pymavlink import mavutil
from vision.vision_class import Vision
import time
LEN_OF_TARGET_LIST = 100


# 计算目标字典表中存储目标总数
def length_of_dict(dict):
    value = list(dict.values())
    length = 0
    for n in range(len(value)):
        length += value[n]

    # 调试用
    print("识别到目标总数： ", length)
    return length


# 判定是否完成了识别目标
def detect_completed(dict):
    key = list(dict.keys())
    key.sort(key=dict.get, reverse=True)
    if len(key) >= 3:
        target1, target2, target3 = key[0:3]
        if dict[target1] + dict[target2] + dict[target3] > 0.7 * LEN_OF_TARGET_LIST:
            print("vision detection result:   ", target1, "   ", target2, "   ", target3)
            for n in range(len(key)):
                print("result: ", key[n], "count: ", dict[key[n]])
            return [target1, target2, target3]
        else:
            return [-1, -1, -1]
    return [-1, -1, -1]


# 排除错误识别结果
def eliminate_error_target(dict):
    # 字典总数未达到设定目标
    if length_of_dict(dict) <= LEN_OF_TARGET_LIST:
        return -10
    # 字典总数量达到目标，删除出现次数最少的键值对
    else:
        key = list(dict.keys())
        key.sort(key=dict.get)
        last_target = key[0]
        result = dict.pop(last_target, -5)
        if result == -5:
            print("error in eliminate_error_target")
            return result
        else:
            # 测试用
            print("delete error result ", last_target)

            return result


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
            print("坐标和姿态：lat = ", track.lat, " lon: ", track.lon, " alt: ", track.alt, " pitch: ", track.pitch, " yaw: ", track.yaw, " roll: ", track.roll)
            with open(file='C:/Users/35032/Desktop/location.txt', mode='a') as f:
                f.write("lat: " + str(target.lat) + " lon: " + str(target.lon) + " num: " + str(target.number))
                f.write('\n')
    else:
        print("none target detected")


def test_course_bombing(the_connection, home_position):
    target = Waypoint(22.8025801, 114.2961667, 5)
    #position = gain_position_now(the_connection)
    position = Waypoint(22.8025382, 114.2961525, 50)

    #print("wp_now lat: ", position.lat, " lon: ", position.lon, " alt: ", position.alt)
    wp_list = wp_bombing_course(target, 180)
    mission_upload(the_connection, wp_list, home_position)

    command_retry(the_connection, 'arm')

    # input("盘旋")
    # loiter(the_connection, position)

    if input("输入0切换自动模式开始投弹任务（请检查目标点和home点已正确设置）（若已通过其他方式切换到自动，可输入其他跳过）： ") == '0':
        mode_set(the_connection, 10)

    while mission_current(the_connection) < len(wp_list) - 13:
        pass
        print(mission_current(the_connection))
    bomb_drop(the_connection)



# 测试 过程淘汰 算法的正确性
def test_target_selection(the_connection, home_position):
    wp1 = Waypoint(22.7528506, 113.88279299999999, 30)
    wp2 = Waypoint(22.8025382, 114.2961525, 30)
    wp = [wp1, wp2]
    detect_course = wp_detect_course(wp, 2, 30)
    mission_upload(the_connection, detect_course, home_position)

    if input("输入0切换自动模式开始任务（请检查目标点和home点已正确设置）（若已通过其他方式切换到自动，可输入其他跳过）： ") == '0':
        mode_set(the_connection, 10)

    # 参数和初始化
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
                # 视觉识别成功但数字识别失败
                if vision_position_list[n].num < 0:
                    continue
                # 数字识别得到结果
                else:
                    target = coordinate_transfer(track.lat, track.lon, track.alt, track.yaw,
                                                 track.pitch, track.roll, vision_position_list[n].x,
                                                 vision_position_list[n].y, vision_position_list[n].num)
                    print("检测到靶标数字： ", target.number)
                    print("标靶坐标：lat = ", target.lat, ", lon = ", target.lon, ", num = ", target.number)
                    print("坐标和姿态：lat = ", track.lat, " lon: ", track.lon, " alt: ", track.alt, " pitch: ",
                          track.pitch, " yaw: ", track.yaw, " roll: ", track.roll)
                    with open(file='C:/Users/35032/Desktop/location.txt', mode='a') as f:
                        f.write("lat: " + str(target.lat) + " lon: " + str(target.lon) + " num: " + str(target.number))
                        f.write('\n')
                    target_list.append(target)
                    # 该目标是第一次出现
                    if target_dict.get(target.number, -1) < 0:
                        target_dict[target.number] = 1
                    # 该目标不是第一次出现，且数量小于指定数量
                    elif target_dict.get(target.number, -1) < 0.3 * LEN_OF_TARGET_LIST:
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
print(the_connection.target_system, the_connection.target_component)

# command_retry(the_connection, 'mode_set', 0)

if input("输入O获取坐标, 输入其他跳过： ") == '0':
    print(input("位置信息： "))
    wp = gain_position_now(the_connection)
    print("坐标 lat:", wp.lat, " lon:", wp.lon, " alt: ", wp.alt)

if input("输入0测试数传传输频率（大概需要10秒），输入其他跳过： ") == '0':
    frequency = gain_transform_frequency(the_connection)
    print("数传传输频率：", frequency, "Hz")

# 设置home点
#home_position = Waypoint(22.590727599999997, 113.96202369999999, 0)
home_position = Waypoint(22.8027619, 114.2959589, 0)
#command_retry(the_connection, 'set_home', home_position)

# 图像参数和初始化
#vis = Vision(source=0, device='0', conf_thres=0.7)
#vis = Vision(source="D:/ngyf/videos/DJIG0007.mov", device='0', conf_thres=0.7)

track_list = []

command_retry(the_connection, 'arm')

# test_location_transfer(the_connection, track_list)
test_course_bombing(the_connection, home_position)
# test_target_selection(the_connection, home_position)

