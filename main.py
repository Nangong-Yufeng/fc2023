from utils import title
import threading
import queue
import time
from scipy.interpolate import interp1d, UnivariateSpline
import bisect
from vision.vision_class import Vision
from navigation import (Waypoint, set_home, mode_set, arm, wp_circle_course,wp_straight_course, mission_upload,
                        rec_match_received, gain_transform_frequency, gain_track_of_time, wp_detect_course,
                        loiter_at_present, delay_eliminate, coordinate_transfer, gain_track_point,
                        length_of_dict, detect_completed, eliminate_error_target, command_retry,
                        gain_position_now, set_ground_speed, target_transfer)
from pymavlink import mavutil
# 目标字典的目标存储个数
LEN_OF_TARGET_LIST = 100
TIME_DELAY_MS = 180
wp_detect = Waypoint(22.8027223, 114.2960957, 30)
wp_home = Waypoint(22.8027619, 114.2959589, 0)


'''
线程1：获取姿态数据并生成带时间戳的姿态序列
'''
def get_attitude_data(track_queue, detect_result):
    # 初始化对轨
    time_start_plane = gain_position_now(the_connection).time
    time_start_computer = int(round(time.time() * 1000))
    time_gap = time_start_computer - time_start_plane

    last_alt = 0
    last_time = time_start_computer
    # 循环存储侦查阶段所有位姿点
    while not detect_result.empty():
        # 获取位姿态数据
        track = gain_track_point(the_connection)

        # 记录下原始位姿数据，观察错误率
        with open(file='C:/Users/35032/Desktop/raw_posture_gps.txt', mode='a') as f:
            f.write("raw inform: lat " + str(track.lat) + " lon " + str(track.lon)
                    + " alt " + str(track.alt) + " time " + str(track.time)
                    + " pitch " + str(track.pitch) + " roll " + str(track.roll)
                    + "yaw " + str(track.yaw))

        '''
        对获取的位姿信息进行不正常值筛选
        '''
        # 小于零的错误高度值
        if track.alt < 0:
            track.alt = last_alt
        # 高度出现不正常的快速变化，取200ms内变化20m
        if abs((track.alt - last_alt) / (track.time + time_gap - last_time)) > 0.1:
            track.alt = last_alt

        last_alt = track.alt
        # print("线程1运行： ", track.time)
        timestamp = track.time + time_gap
        last_time = timestamp

        # 将姿态数据和时间戳添加到姿态队列中
        track_queue.put((timestamp, track))

        '''
        # 清除过时的数据点
        current_time = int(round(time.time() * 1000))
        while not track_queue.empty():
            data = track_queue.get()
            if current_time - data[0] <= 10000:  # 假设保留最近10秒的数据点
                track_queue.put(data)
                break
        '''


'''
线程2：处理图像、数字后处理和坐标解算
'''
def process_image_and_pose(track_queue, detect_result):
    # 侦察中记录侦察到靶标时靶标的数字、视觉位置和检测时的系统时间，以时间为key
    time_target_dict = {}
    # 目标数字与测试记录，以数字为key，用于过程淘汰算法
    target_dict = {}
    # 记录识别并测试得到的三个靶标数字
    target_result = [-1, -1, -1]

    # 图传信号初始化
    vis = Vision(source=0, device='0', conf_thres=0.7)# "D:/ngyf/videos/DJI_0001.MP4"

    result = -1
    while result < 0:
        # 图像处理
        vis.shot()
        if vis.im0 is None:
            print("signal lost")
            continue

        # 视觉处理
        vision_position_list = vis.run()

        # 检测到靶标
        if len(vision_position_list) != 0 and track_queue.qsize():

            for n in range(len(vision_position_list)):

                # 视觉识别成功但数字识别失败
                if vision_position_list[n].num < 0:
                    continue
                # 数字识别得到结果
                else:
                    print("检测到靶标数字： ", vision_position_list[n].number)
                    detect_time = int(round(time.time() * 1000)) - TIME_DELAY_MS

                    # 记录识别到靶标的时间和靶标数字
                    time_target_dict[detect_time] = [vision_position_list[n].num,
                                                     vision_position_list[n].x, vision_position_list.y]

                    # 对靶标数据进行过程淘汰
                    target_number = vision_position_list[n].num
                    # 该目标是第一次出现
                    if target_dict.get(target_number, -1) < 0:
                        target_dict[target_number] = 1
                    # 该目标不是第一次出现，且数量小于指定数量
                    elif target_dict.get(target_number, -1) < 0.3 * LEN_OF_TARGET_LIST:
                        target_dict[target_number] += 1
                    # 该目标不是第一次出现，但存储数量已经达到指定上限
                    else:
                        continue
                    # 如果超出设定范围，删除数量最少的一项
                    eliminate_error_target(target_dict)

                    # 判定侦察任务是否完成， 若得到探测结果，传入target列表，长度为3
                    target_result = detect_completed(target_dict)
                    result = target_result[0]

        # 未检测到靶标
        else:
            result = -1

    # 提取出该queue中的值，终止其他两个线程
    detect_result.get()

    print("侦察任务完成！")
    loiter_at_present(the_connection, 50)

    '''
    侦察完成 进行数据后处理
    '''
    [num1, num2, num3] = target_result

    # 提取时间戳和对应的姿态数据
    timestamps, tracks = zip(*list(track_queue.queue))
    target_time = list(time_target_dict.keys())

    # 存储的视觉信息
    vision_inform = list(time_target_dict.values())

    target1 = target_transfer(time_target_dict=time_target_dict, target_time=target_time, timestamps=timestamps,
                              tracks=tracks, vision_inform=vision_inform, num=num1, delay=TIME_DELAY_MS)
    '''
    基于坐标和数字进行错误目标判断，如有错误可以再跑接下来的点
    '''


'''
线程3：航点任务循环
'''
def detect_mission_circling(the_connection, detect_result):
    #set_home(the_connection, wp_home)

    # 设置模式为纯手动
    command_retry(the_connection, 'mode_set', 0)

    # 解锁飞机

    arm(the_connection)

    # 切换自动飞行
    if input("输入0切换自动模式开始任务（请检查目标点和home点已正确设置）（若已通过其他方式切换到自动，可输入其他跳过）： ") == '0':
        command_retry(the_connection, 'mode_set', 10)

    set_ground_speed(the_connection, 15)

    # 侦察部分航线
    while not detect_result.empty():
        '''if rec_match_received(the_connection, 'MISSION_CURRENT').seq < len(detect_course):
            continue
        '''
        command_retry(the_connection, 'mode_set', 10)
        print("next detect circle")


if __name__ == "__main__":
    '''
    帅
    '''
    title.printTitle()

    '''
    连接并上传侦察任务
    '''
    the_connection = mavutil.mavlink_connection('/COM5', baud=57600)
    print(the_connection.target_system, the_connection.target_component)

    # 生成并上传任务
    detect_course = wp_detect_course(wp_detect, 16, 'north')
    mission_upload(the_connection, detect_course, wp_home)

    '''
    侦察过程中多线程运行
    '''
    track_queue = queue.Queue(maxsize=0)  # 使用队列来存储姿态数据
    detect_result = queue.Queue()  # 通过判断队列中是否有数据判断是否关闭线程
    detect_result.put(-1)

    # 创建两个线程，一个用于获取姿态数据，另一个用于处理图像和位姿确定
    attitude_thread = threading.Thread(target=get_attitude_data, args=(track_queue, detect_result))
    image_thread = threading.Thread(target=process_image_and_pose, args=(track_queue, detect_result))
    mission_thread = threading.Thread(target=detect_mission_circling, args=(the_connection, detect_result))

    attitude_thread.start()
    image_thread.start()
    mission_thread.start()

    attitude_thread.join()
    image_thread.join()
    mission_thread.join()

    '''
    进行投弹
    '''



