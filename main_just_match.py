"""
已知坐标点仅进行数字识别和坐标匹配的main5.0
"""
import numpy as np
from utils import title
import threading
import queue
import time
from scipy.interpolate import UnivariateSpline
from vision.vision_class import Vision
from navigation import (Waypoint, mode_set,
                        gain_position_now, wp_bombing_course, mission_current,
                        preflight_command, contest_detect_course,
                        target_point, coordinate_aver_cal, target_match,
                        mission_upload_including_bomb_drop, mission_jump_to)
from pymavlink import mavutil
'''
需要设定的参数
'''
# 侦察航线的转弯直径
DIAMETER = 0.00060
# 航线转向方向
DIRECTION = 1
# 直线航线拓展长度
LENGTH_EXTEND = 0.00010
# 数字侦察部分的航点数量
WP_NUMBER_OF_NUMBER_DETECT = 40
# 投弹开始航点数
WP_BOMB_START_TARGET1 = 16
WP_BOMB_START_TARGET2 = 38
WP_BOMB_START_TARGET3 = 60
# 投弹点的位置
WP_SEQ_OF_BOMB_DROP = 10
# 设定的延迟时间
TIME_DELAY_MS = 250
# 侦察航向，指南针标准
DETECT_ANGLE = 340

# 投弹进场航向，指南针标准
if DETECT_ANGLE > 180:
    APPROACH_ANGLE = DETECT_ANGLE - 180
else:
    APPROACH_ANGLE = DETECT_ANGLE + 180
'''
需要测量的坐标
'''
# home点
wp_home = Waypoint(28.5928658, 113.1872269, 0)
# 靶标区的四个顶点，注意按照侦察航线顺序
wp_boarder = [Waypoint(28.5937116, 113.1869191, 0),
              Waypoint(28.5939566, 113.1868185, 0),
              Waypoint(28.5937976, 113.1871538, 0),
              Waypoint(28.5940625, 113.1870545, 0)]
# 三个靶标位置的坐标点
target_coordinate = [target1, target2, target3] = [Waypoint(0, 0, 0),
                                                   Waypoint(0, 0, 0),
                                                   Waypoint(0, 0, 0)]  # 三个靶标的测量位置
target_choose = 1  # 选择的靶标
final_target_coordinate = target_coordinate[0]  # 防止没有数字解算结果，直接运行设定目标


'''
线程1：获取姿态数据并生成带时间戳的姿态序列
'''


def get_attitude_data(track_queue, detect_result):
    # 初始化对轨
    time_start_plane = gain_position_now(the_connection).time
    time_start_computer = int(round(time.time() * 1000))
    time_gap = time_start_computer - time_start_plane

    # 循环存储侦查阶段所有位姿点
    while not detect_result.empty():
        # 获取位姿态数据
        track = gain_position_now(the_connection)

        # print("线程1运行： ", track.time)
        timestamp = track.time + time_gap

        # 将姿态数据和时间戳添加到姿态队列中
        track_queue.put((timestamp, track))

        # 清除过时的数据点C
        current_time = int(round(time.time() * 1000))
        while not track_queue.empty():
            data = track_queue.get()
            if current_time - data[0] <= 50000:  # 假设保留最近50秒的数据点
                track_queue.put(data)
                break


'''
线程2：处理图像、数字后处理和坐标解算
'''


def process_image_and_pose(track_queue, detect_result):
    target_list = []
    vis = Vision(source=0, device='0', conf_thres=0.7)  # D:/ngyf/videos/DJI_0001.MP4"
    '''
    进行数字识别和靶标匹配
    '''
    target_num_list = []  # 记录靶标坐标和数字的列表
    target_num_dict = {}  # 记录某个数字出现的数量

    # 侦察航线结束就停止各线程
    while mission_current(the_connection) < WP_NUMBER_OF_NUMBER_DETECT:
        # 图像处理
        vis.shot()
        if vis.im0 is None:
            print("图传无信号")
            continue

        # 视觉处理
        vision_position_list = vis.run()  # 识别数字

        # 检测到靶标
        if len(vision_position_list) != 0 and track_queue.qsize():

            for n in range(len(vision_position_list)):

                # 视觉识别成功但数字识别失败
                if vision_position_list[n].num < 0:
                    continue
                # 数字识别得到结果
                else:
                    current_time = int(round(time.time() * 1000)) - TIME_DELAY_MS

                    # 提取时间戳和对应的姿态数据
                    timestamps, tracks = zip(*list(track_queue.queue))

                    # 如果有足够的数据点，执行插值和解算
                    if len(tracks) >= 5:
                        # 根据时间戳排序数据点
                        sorted_indices = sorted(range(len(timestamps)), key=lambda i: -timestamps[i])

                        # 获取最接近指定时间的5个数据点的索引
                        selected_indices = sorted_indices[4::-1]

                        # 提取选定的数据点
                        selected_tracks = [tracks[i] for i in selected_indices]
                        selected_timestamps = [timestamps[i] for i in selected_indices]

                        # 强制数据递增
                        for i in range(len(selected_timestamps) - 1):
                            if selected_timestamps[i] == selected_timestamps[i + 1]:
                                selected_timestamps[i + 1] += 1

                        # 使用三次多项式插值
                        lat_interp = UnivariateSpline(selected_timestamps, [data.lat for data in selected_tracks], s=0)
                        lon_interp = UnivariateSpline(selected_timestamps, [data.lon for data in selected_tracks], s=0)

                        # 直接使用读到数字时的飞机坐标，不进行解算
                        target = target_point(lat_interp(current_time), lon_interp(current_time),
                                              vision_position_list[n].num)

                        # 存入坐标列表
                        target_num_list.append(target)

                        # 存入坐标字典
                        if target_num_dict.get(target.number, -1) < 0:
                            target_num_dict[target.number] = 1
                        # 该目标不是第一次出现
                        else:
                            target_num_dict[target.number] += 1

    # 数字侦察完成，关闭其他两个线程
    detect_result.get()
    print("数字侦察完成")

    # 根据字典进行排序
    key = list(target_num_dict.keys())
    key.sort(key=target_num_dict.get, reverse=True)

    global final_target_coordinate
    if len(key) == 0:
        median = -1
        final_target_coordinate = target_coordinate[1]
    elif len(key) == 1:
        coordinate = coordinate_aver_cal(target_list, key[0])
        list_num = [coordinate,
                    target_point(0, 0, 0),
                    target_point(0, 0, 0)]
        final_target_coordinate = target_match(target_coordinate, list_num, 0)
        median = key[0]
    elif len(key) == 2:
        coordinate1 = coordinate_aver_cal(target_list, key[0])
        coordinate2 = coordinate_aver_cal(target_list, key[1])
        list_num = [coordinate1,
                    coordinate2,
                    target_point(0, 0, 0)]
        final_target_coordinate = target_match(target_coordinate, list_num, 0)  # 只有两个数据的情况下，还是选第一个
        median = key[0]
    else:
        # 分别计算三个靶标的坐标
        coordinate1 = coordinate_aver_cal(target_list, key[0])
        coordinate2 = coordinate_aver_cal(target_list, key[1])
        coordinate3 = coordinate_aver_cal(target_list, key[2])
        list_num = [coordinate1, coordinate2, coordinate3]
        # 计算中位数，获取其在数组中的位置
        median = np.median(key)
        target_place = 0
        for i in range(len(key)):
            if key[i] == median:
                target_place = i
        # 最后与聚类坐标进行匹配选取
        final_target_coordinate = target_match(target_coordinate, list_num, target_place)
        global target_choose
        target_choose = final_target_coordinate.alt  # 借用alt参数把靶标选择数组坐标返回

    print("最终坐标结果 靶标纬度: ", final_target_coordinate.lat, " 靶标经度: ", final_target_coordinate.lon, " 靶标数字：", median)
    # 获取靶标坐标，线程二关闭


if __name__ == "__main__":
    '''                                         
    帅
    '''
    title.printTitle()

    '''
    连接并上传侦察任务
    '''
    the_connection = mavutil.mavlink_connection('/ttyUSB0', baud=921600)

    # 生成并上传任务，比赛时不需要
    detect_course = []
    # 第一圈
    detect_course.extend(contest_detect_course(detect_angle=DETECT_ANGLE, start_coordinate=wp_boarder[0],
                                               end_coordinate=wp_boarder[1], direction=DIRECTION,
                                               alt_detect=15, alt_circle=35, diameter=DIAMETER,
                                               length_expend=LENGTH_EXTEND))
    # 第二圈
    detect_course.extend(contest_detect_course(detect_angle=DETECT_ANGLE, start_coordinate=wp_boarder[2],
                                               end_coordinate=wp_boarder[3], direction=DIRECTION,
                                               alt_detect=15, alt_circle=35, diameter=DIAMETER,
                                               length_expend=LENGTH_EXTEND))
    for i in range(0, 5):
        detect_course.pop(-1)

    bomb_course = []

    bomb_course.extend(wp_bombing_course(target1, approach_angle=APPROACH_ANGLE, length_bomb_lead=23,
                                         turn_direction=-DIRECTION))
    bomb_course.extend(wp_bombing_course(target2, approach_angle=APPROACH_ANGLE, length_bomb_lead=23,
                                         turn_direction=-DIRECTION))
    bomb_course.extend(wp_bombing_course(target3, approach_angle=APPROACH_ANGLE, length_bomb_lead=23,
                                         turn_direction=-DIRECTION))
    mission_upload_including_bomb_drop(the_connection, detect_course, [35, 57, 79])

    # 起飞前准备
    preflight_command(the_connection, wp_home)
    '''
    侦察过程中多线程运行
    '''
    track_queue = queue.Queue(maxsize=0)  # 使用队列来存储姿态数据
    detect_result = queue.Queue()  # 通过判断队列中是否有数据判断是否关闭线程
    detect_result.put(-1)

    # 创建两个线程，一个用于获取姿态数据，另一个用于处理图像和位姿确定
    attitude_thread = threading.Thread(target=get_attitude_data, args=(track_queue, detect_result))
    image_thread = threading.Thread(target=process_image_and_pose, args=(track_queue, detect_result))

    attitude_thread.start()
    image_thread.start()

    attitude_thread.join()
    image_thread.join()

    '''
    进行投弹
    '''
    # 选择跳转到的航点
    if target_choose == 0:
        while mission_current(the_connection) < WP_BOMB_START_TARGET2:
            continue
        # 任务完成，设为手动
        while not mode_set(the_connection, 0):
            continue
    elif target_choose == 1:
        mission_jump_to(the_connection, WP_BOMB_START_TARGET2)
        while mission_current(the_connection) < WP_BOMB_START_TARGET3:
            continue
        # 任务完成，设为手动
        while not mode_set(the_connection, 0):
            continue
    else:
        mission_jump_to(the_connection, WP_BOMB_START_TARGET3)

    '''
    任务结束
    '''