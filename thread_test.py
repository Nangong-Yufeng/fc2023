from utils import title
import threading
import queue
import time
import numpy as np
from scipy.interpolate import interp1d, UnivariateSpline
from vision.vision_class import Vision
from navigation import (Waypoint, set_home, mode_set, arm, wp_circle_course,wp_straight_course, mission_upload,
                        rec_match_received, gain_transform_frequency, gain_track_of_time, wp_detect_course,
                        loiter_at_present, delay_eliminate, coordinate_transfer, gain_track_point,
                        length_of_dict, detect_completed, eliminate_error_target, command_retry,
                        gain_position_now,set_ground_speed)
from pymavlink import mavutil
# 目标字典的目标存储个数
LEN_OF_TARGET_LIST = 100
TIME_DELAY_MS = 300
wp_detect = Waypoint(22.8027223, 114.2960957, 30)
wp_home = Waypoint(22.8027619, 114.2959589, 0)

'''
线程1：获取姿态数据并生成带时间戳的姿态序列
'''
def get_attitude_data(track_queue, detect_result):
    time_start_plane = gain_position_now(the_connection).time
    time_start_computer = int(round(time.time() * 1000))
    time_gap = time_start_computer - time_start_plane
    while not detect_result.empty():
        # 获取位姿态数据
        track = gain_track_point(the_connection)
        # print("线程1运行： ", track.time)
        timestamp = track.time + time_gap

        # 将姿态数据和时间戳添加到姿态队列中
        track_queue.put((timestamp, track))

        # 清除过时的数据点
        current_time = int(round(time.time() * 1000))
        while not track_queue.empty():
            data = track_queue.get()
            if current_time - data[0] <= 10000:  # 假设保留最近10秒的数据点
                track_queue.put(data)
                break

'''
线程2：处理图像、数字后处理和坐标解算
'''
def process_image_and_pose(track_queue, detect_result):
    target_list = []
    target_dict = {}
    vis = Vision(source="D:/ngyf/videos/DJI_0001.MP4", device='0', conf_thres=0.7)#
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

            current_time = int(round(time.time() * 1000)) - TIME_DELAY_MS

            # 提取时间戳和对应的姿态数据
            timestamps, tracks = zip(*list(track_queue.queue))

            # 视觉识别成功但数字识别失败
            if vision_position_list[n].num < 0:
                continue
            # 数字识别得到结果
            else:
             # 如果有足够的数据点，执行插值
              if len(tracks) >= 5:
                # 根据时间戳排序数据点
                sorted_indices = sorted(range(len(timestamps)), key=lambda i: -timestamps[i])

                # 获取最接近指定时间的10个数据点的索引
                '''
                需要添加对错误gps信息的剔除
                '''
                selected_indices = sorted_indices[4::-1]

                # 提取选定的数据点
                selected_tracks = [tracks[i] for i in selected_indices]
                selected_timestamps = [timestamps[i] for i in selected_indices]

                # 强制数据递增
                for i in range(len(selected_timestamps)-1):
                    if selected_timestamps[i] == selected_timestamps[i+1]:
                        selected_timestamps[i+1] += 1

                print("time_stamps:", selected_timestamps)

                # 使用三次多项式插值
                roll_interp = UnivariateSpline(selected_timestamps, [data.roll for data in selected_tracks], s=0)
                pitch_interp = UnivariateSpline(selected_timestamps, [data.pitch for data in selected_tracks],s=0)
                yaw_interp = UnivariateSpline(selected_timestamps, [data.yaw for data in selected_tracks], s=0)
                lat_interp = UnivariateSpline(selected_timestamps, [data.lat for data in selected_tracks], s=0)
                lon_interp = UnivariateSpline(selected_timestamps, [data.lon for data in selected_tracks], s=0)
                alt_interp = UnivariateSpline(selected_timestamps, [data.alt for data in selected_tracks], s=0)

                print('type:', type(vision_position_list[n].x), '!!!!!!!')
                print(vision_position_list[n].x)
                # 获取插值后的姿态数据
                target = coordinate_transfer(lat_interp(current_time), lon_interp(current_time), alt_interp(current_time), yaw_interp(current_time),
                                                pitch_interp(current_time), roll_interp(current_time), vision_position_list[n].x,
                                                vision_position_list[n].y, vision_position_list[n].num)
                print("检测到靶标数字： ", target.number)
                print("标靶坐标：lat = ", target.lat, ", lon = ", target.lon, ", num = ", target.number)
                print("飞机位姿： ", lat_interp(current_time), lon_interp(current_time), alt_interp(current_time), yaw_interp(current_time),
                                                pitch_interp(current_time), roll_interp(current_time), vision_position_list[n].x,
                                                vision_position_list[n].y, vision_position_list[n].num)

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
              else:
                print("not enough point")
                continue

        # 未检测到靶标
        else:
            result = -1
    detect_result.get()


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

    the_connection = mavutil.mavlink_connection('/COM8', baud=57600)
    print(the_connection.target_system, the_connection.target_component)

    with open(file='C:/Users/35032/Desktop/transfer.txt', mode='a') as f:
        f.write("time_delay" + str(TIME_DELAY_MS))

    # 生成并上传任务
    detect_course = wp_detect_course(wp_detect, 16, 'north')
    mission_upload(the_connection, detect_course, wp_home)

    track_queue = queue.Queue()  # 使用队列来存储姿态数据
    detect_result = queue.Queue()
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

    if detect_result.empty():
        print("侦察任务完成！")
        loiter_at_present(the_connection, 50)
