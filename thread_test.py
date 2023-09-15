import threading
import queue
import time
from scipy.interpolate import interp1d
from vision.vision_class import Vision
from navigation import (Waypoint, set_home, mode_set, arm, wp_circle_course,wp_straight_course, mission_upload,
                        rec_match_received, gain_transform_frequency, gain_track_of_time, wp_detect_course,
                        loiter_at_present, delay_eliminate, coordinate_transfer, gain_track_point)
from pymavlink import mavutil
# 目标字典的目标存储个数
LEN_OF_TARGET_LIST = 100
TIME_DELAY_MS = 200

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


# 线程1：获取姿态数据并生成带时间戳的姿态序列
def get_attitude_data(track_queue):
    while True:
        # 模拟从传感器中获取姿态数据
        track = gain_track_point(the_connection)
        timestamp = int(round(time.time() * 1000))

        # 将姿态数据和时间戳添加到姿态队列中
        track_queue.put((track, timestamp))

        # 清除过时的数据点
        current_time = int(round(time.time() * 1000))
        while not track_queue.empty():
            data = track_queue.get()
            if current_time - data[1] <= 5000:  # 假设保留最近5秒的数据点
                track_queue.put(data)
                break



# 线程2：处理图像、数字后处理和坐标解算
def process_image_and_pose(track_queue):
    target_list = []
    target_dict = {}
    vis = Vision(source="D:/ngyf/videos/DJIG0007.mov", device='0', conf_thres=0.7)
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
        if len(vision_position_list) != 0:

          for n in range(len(vision_position_list)):

            current_time = int(round(time.time() * 1000)) - TIME_DELAY_MS

            # 提取时间戳和对应的姿态数据
            timestamps, tracks = zip(*list(track_queue.queue))

            # 如果有足够的数据点，执行插值
            if len(tracks) >= 4:
                # 根据时间戳排序数据点
                sorted_indices = sorted(range(len(timestamps)), key=lambda i: abs(timestamps[i] - current_time))

                # 获取最接近指定时间的10个数据点的索引
                '''
                需要添加对错误gps信息的剔除
                '''
                selected_indices = sorted_indices[:10]

                # 提取选定的数据点
                selected_tracks = [tracks[i] for i in selected_indices]
                selected_timestamps = [timestamps[i] for i in selected_indices]

                # 使用三次多项式插值
                roll_interp = interp1d(selected_timestamps, [data["roll"] for data in selected_tracks], kind='cubic')
                pitch_interp = interp1d(selected_timestamps, [data["pitch"] for data in selected_tracks],
                                        kind='cubic')
                yaw_interp = interp1d(selected_timestamps, [data["yaw"] for data in selected_tracks], kind='cubic')
                lat_interp = interp1d(selected_timestamps, [data["lat"] for data in selected_tracks], kind='cubic')
                lon_interp = interp1d(selected_timestamps, [data["lon"] for data in selected_tracks],
                                        kind='cubic')
                alt_interp = interp1d(selected_timestamps, [data["alt"] for data in selected_tracks], kind='cubic')

                # 获取插值后的姿态数据
                target = coordinate_transfer(lat_interp(current_time), lon_interp(current_time), alt_interp(current_time), yaw_interp(current_time),
                                             pitch_interp(current_time), roll_interp(current_time), vision_position_list[n].x,
                                             vision_position_list[n].y, vision_position_list[n].num)
                print("检测到靶标数字： ", target.number)
                print("标靶坐标：lat = ", target.lat, ", lon = ", target.lon, ", num = ", target.number)

                target_list.append(target)
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
                continue

        # 未检测到靶标
        else:
            result = -1


# 线程3：航点任务循环
def detect_mission_circling(the_connection):
    pass


if __name__ == "__main__":
    the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

    track_queue = queue.Queue()  # 使用队列来存储姿态数据

    # 创建两个线程，一个用于获取姿态数据，另一个用于处理图像和位姿确定
    attitude_thread = threading.Thread(target=get_attitude_data, args=(track_queue,))
    image_thread = threading.Thread(target=process_image_and_pose, args=(track_queue,))
    mission_thread = threading.Thread(target=detect_mission_circling(the_connection))

    attitude_thread.start()
    image_thread.start()
    mission_thread.start()

    attitude_thread.join()
    image_thread.join()
    mission_thread.join()
