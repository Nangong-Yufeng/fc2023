"""
初代main函数
"""

from utils import title
import time
from vision.vision_class import Vision
from navigation import (Waypoint, set_home, mode_set, arm, wp_circle_course,wp_straight_course, mission_upload,
                        rec_match_received, gain_transform_frequency, gain_track_of_time, wp_detect_course,
                        loiter_at_present, delay_eliminate, coordinate_transfer)
from pymavlink import mavutil
# 目标字典的目标存储个数
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


'''
帅(😅)
'''
title.printTitle()

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
    print("投弹测试完成")

if input("输入0测试数传传输频率（大概需要10秒），输入其他跳过： ") == '0':
    frequency = gain_transform_frequency(the_connection)
    print("数传传输频率：", frequency, "Hz")

# 设置home点
home_position = Waypoint(22.5903516, 113.9755156, 0)
set_home(the_connection, home_position)

# 设置模式为纯手动
mode_set(the_connection, 0)

# 解锁飞机
arm(the_connection)

'''
开始自动飞行
'''

"""
标靶识别
"""
# 参数和初始化
vis = Vision(source=0, device='0', conf_thres=0.7)

# 循环侦察任务（用于完整任务）
result = -1

# 侦察区坐标，使用环绕航线
wp1 = Waypoint(22.5899275, 113.9751526, 120)
wp2 = Waypoint(22.5899248, 113.9755938, 120)
wp_detect = [wp1, wp2]
alt = 120
track_list = []
target_list = []
target_dict = {}
target_result = [ -1, -1, -1]

# 开始侦察
while result == -1:
    # 生成下一圈侦察航线
    wp_detect_list = wp_detect_course(wp_detect, 3, alt=alt)
    # 上传下一圈任务
    mission_upload(the_connection, wp_detect, home_position)

    # 一圈侦察任务未完成时
    while rec_match_received(the_connection, 'MISSION_CURRENT').seq < len(wp_detect_list) - 1:
        # cur = int(time.time() * 1000)

        # 读取当前姿态和位置
        inform = gain_track_of_time(the_connection, track_list)
        time_stamp = inform[0]
        alt = inform[1]

        # 只在20米以下的高度进行视觉识别，避免生成过多错误结果
        if alt <= 20:

           # 截图
           vis.shot()
           if vis.im0 is None:
              print("signal lost")
              continue

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
        # 高度大于二十米，不进行检测
        else:
            continue

    # 若没有识别到数字，降低高度继续进行
    if alt > 15:
       alt -= 0.5

print("detection completed!")
# 侦察完成，进行标靶数据处理
target1_list = []
target2_list = []
target3_list = []
# 对三个靶标的所有坐标值进行筛选存储
for count in range(len(target_list)):
    if target_list[count].number == target_result[0]:
        target1_list.append(target_list[count])
    elif target_list[count].number == target_result[1]:
        target2_list.append(target_list[count])
    elif target_list[count].number == target_result[2]:
        target3_list.append(target_list[count])


'''
执行投弹
'''
loiter_at_present(the_connection, 50)


