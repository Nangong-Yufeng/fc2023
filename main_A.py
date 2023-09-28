from utils import title
import threading
import queue
import time
import numpy
from vision.vision_class import Vision
from navigation import (Waypoint, mode_set, mission_upload,
                        gain_track_point,
                        detect_completed, eliminate_error_target,
                        gain_position_now, target_transfer,
                        wp_bombing_course, mission_current, bomb_drop,
                        initiate_bomb_drop, preflight_command,
                        wp_detect_course_HeBei_2g)
from pymavlink import mavutil
# 目标字典的目标存储个数
LEN_OF_TARGET_LIST = 50
TIME_DELAY_MS = 150
DETECT_ANGLE = 231
APPROACH_ANGLE = 309  # 投弹时的进近航向，北起点逆时针
DETECT_TIME_LIMIT = int(3 * 60 * 1000)
DETECT_ACC = 6  # m
wp_home = Waypoint(38.543938, 115.04040769999999, 0)
wp_start = Waypoint(38.5569480, 115.1389195, 15)  # A组，顺时针侦察
final_target_position = Waypoint(38.5569207, 115.1385990, 0)
mission_start_time = 0


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

        '''
        对获取的位姿信息进行不正常值筛选
        '''
        # 小于零的错误高度值
        if track.alt < 0:
            track.alt = last_alt
        # 高度出现不正常的快速变化，取200ms内变化20m
        if ((track.time + time_gap - last_time) != 0 and
                abs((track.alt - last_alt) / (track.time + time_gap - last_time)) > 0.1):
            track.alt = 0.7 * last_alt + 0.3 * track.alt

        last_alt = track.alt
        # print("线程1运行： ", track.time)
        timestamp = track.time + time_gap
        last_time = timestamp

        # 将姿态数据和时间戳添加到姿态队列中
        track_queue.put((timestamp, track))

        '''
        # 清除过时的数据点C
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
    vis = Vision(source=0, device='0', conf_thres=0.7)#"D:/ngyf/videos/DJI_0001.MP4"

    result = -1
    last_show_time = 0
    while result < 0:
        current_time = int(round(time.time() * 1000))

        # 时间保护，若时间到3分半还为得到识别结果，则直接进入投弹
        if (current_time - mission_start_time) * 1e-3 - last_show_time > 60:
            print("运行时间：", (current_time - mission_start_time) * 1e-3, "秒")
            last_show_time = (current_time - mission_start_time) * 1e-3
        if current_time - mission_start_time > DETECT_TIME_LIMIT:
            print("time up!")
            target_result = detect_completed(target_dict, is_time_out=True)
            break

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

                # 视觉识别成功但数字识别失败
                if vision_position_list[n].num < 0:
                    continue
                # 数字识别得到结果
                else:
                    print("检测到靶标数字： ", vision_position_list[n].num)
                    detect_time = int(round(time.time() * 1000)) - TIME_DELAY_MS

                    # 记录识别到靶标的时间和靶标数字
                    time_target_dict[detect_time] = [vision_position_list[n].num,
                                                     vision_position_list[n].x, vision_position_list[n].y]

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
                    target_result = detect_completed(target_dict, is_time_out=False)
                    result = target_result[0]
                    break

        # 未检测到靶标
        else:
            result = -1

    # 提取出该queue中的值，终止其他两个线程
    detect_result.get()

    print("侦察任务完成！")
    while not mode_set(the_connection, 12):
        continue

    '''
    侦察完成 进行数据后处理
    '''
    # 提取时间戳和对应的姿态数据
    timestamps, tracks = zip(*list(track_queue.queue))
    # 存储的视觉识别世界戳
    target_time = list(time_target_dict.keys())
    # 存储的视觉信息
    vision_inform = list(time_target_dict.values())

    global final_target_position

    # 若没有检测到任何数字
    if len(time_target_dict) == 0:
        pass
    # 若视觉只检查到一个或两个数字
    elif len(target_result) == 1 or len(target_result) == 2:
        final_target_position = target_transfer(time_target_dict=time_target_dict, target_time=target_time,
                                                timestamps=timestamps,
                                                tracks=tracks, vision_inform=vision_inform,
                                                num=target_result[0], delay=TIME_DELAY_MS)
    else:
        [num1, num2, num3] = target_result[0:3]
        print(num1, num2, num3)

        # 解算出三个识别结果对应处理后的坐标
        target_list = [target1, target2, target3] = [target_transfer(time_target_dict=time_target_dict,
                                                                     target_time=target_time, timestamps=timestamps,
                                                                     tracks=tracks, vision_inform=vision_inform,
                                                                     num=num1, delay=TIME_DELAY_MS),
                                                     target_transfer(time_target_dict=time_target_dict,
                                                                     target_time=target_time, timestamps=timestamps,
                                                                     tracks=tracks, vision_inform=vision_inform,
                                                                     num=num2, delay=TIME_DELAY_MS),
                                                     target_transfer(time_target_dict=time_target_dict,
                                                                     target_time=target_time, timestamps=timestamps,
                                                                     tracks=tracks, vision_inform=vision_inform,
                                                                     num=num3, delay=TIME_DELAY_MS)]

        # 对确定后的三个靶标取中位数
        number_list = numpy.array([target_list[0].number, target_list[1].number, target_list[2].number])
        final_target_number = numpy.median(number_list)

        if target_list[0].number == final_target_number:
            target = target_list[0]
        elif target_list[1].number == final_target_number:
            target = target_list[1]
        else:
            target = target_list[2]

        print("靶标信息计算完成，结果为：", final_target_number)
        final_target_position = target
        print("靶标解算坐标", target.lat, target.lon)


'''
线程3：航点任务循环
'''
def detect_mission_circling(the_connection, detect_result):
    # 任务开始时间
    global mission_start_time
    mission_start_time = int(round(time.time() * 1000))

    # 切换自动飞行

    # 侦察部分航线
    while not detect_result.empty():
        if the_connection.recv_match(type='MISSION_CURRENT', blocking=True).seq < len(detect_course) - 2:
            continue
        while True:
            if mode_set(the_connection, 10):
                break
        print("next detect circle")


if __name__ == "__main__":
    '''                                         
    帅
    '''
    title.printTitle()

    '''
    连接并上传侦察任务
    '''
    the_connection = mavutil.mavlink_connection('/COM6', baud=57600)
    # 备用
    # the_connection = mavutil.mavlink_connection('/COM9', baud=57600)

    # 生成并上传任务，比赛时不需要
    detect_course = wp_detect_course_HeBei_2g(None, wp_start, approaching=DETECT_ANGLE, direction=-1)
    mission_upload(the_connection, detect_course, wp_home)

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
    # 盘旋等待任务上传
    wp_list = wp_bombing_course(final_target_position, APPROACH_ANGLE)
    mission_upload(the_connection, wp_list, wp_home)

    # 盘旋等待任务角度合适
    initiate_bomb_drop(the_connection, APPROACH_ANGLE)

    # 切换为自动模式，进入投弹航线
    while not mode_set(the_connection, 10):
        continue

    # 到达预设投弹位置前，设置时间检查防止因为信号中断不投弹
    last_wp = 0
    mission_current(the_connection)
    time.sleep(5)
    mission_current(the_connection)

    while True:
        msg = mission_current(the_connection)
        if (msg >= len(wp_list) - 16):
            print("reaching waypoint", msg)
            break
        if (int(round(time.time() * 1000)) - mission_start_time) > int(5.2 * 60 * 1000):
            print("time out!")
            break
        if msg != last_wp:
            print("reaching ", msg)
            last_wp = msg
            continue
        else:
            continue

    bomb_drop(the_connection)

    '''
    任务结束
    '''
    #return_to_launch(the_connection)
