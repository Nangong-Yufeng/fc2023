from pymavlink import mavutil
from math import *
import math
from .class_list import Waypoint
from .get_para import (gain_mission, gain_position_now,
                       gain_track_of_time, gain_heading)
from .preflight import mode_set
from .error_process import error_process, rec_match_received
from .transfer import coordinate_transfer
import bisect
from scipy.interpolate import UnivariateSpline
import numpy as np
CONSTANTS_RADIUS_OF_EARTH = 6371000
LEN_OF_TARGET_LIST = 100
DETECT_CONFIDENCE = 0.7

'''
通用任务函数
'''

def send_mission_list(the_connection, wp):
    print(the_connection.target_system, the_connection.target_component, len(wp), mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    args = (the_connection.target_system,
           the_connection.target_component,
           len(wp))#, mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    print(args)
    wp_list = mavutil.mavlink.MAVLink_mission_count_message(*args)
    the_connection.mav.send(wp_list)


def send_mission(the_connection, wp, seq):
    mission_message = mavutil.mavlink.MAVLink_mission_item_int_message(the_connection.target_system, the_connection.target_component, seq,
                                                                       mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                                                                       0, 0, 0, 0, 0, 0,
                                                                       int(wp[seq].lat * 1e7),
                                                                       int(wp[seq].lon * 1e7),
                                                                       wp[seq].alt,
                                                                       mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    the_connection.mav.send(mission_message)


# 在模拟器中可行，但实际飞控不支持
def mission_accomplished(the_connection, wp_list_len):
    msg = rec_match_received(the_connection, 'MISSION_CURRENT')
    if msg.seq == wp_list_len and msg.mission_state == 5:
        return 1
    else:
        #print("seq: ", msg.seq, "state: ", msg.mission_state)
        return -10


def mission_upload(the_connection, wp, home_position):

    wp.insert(0, home_position)

    # 上传航点数量信息
    send_mission_list(the_connection, wp)

    # 在地图中显示静态航点
    waypoint_print_list = []
    for count in range(len(wp)):
        waypoint_print_list.append((wp[count].lat, wp[count].lon))

    while True:
        message = the_connection.recv_match(blocking=True)
        message = message.to_dict()

        # 验证是否为MISSION_REQUEST
        if message["mavpackettype"] == mavutil.mavlink.MAVLink_mission_request_message.msgname:

            # 验证是否为mission items类型
            if message["mission_type"] == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
                seq = message["seq"]

                # 发送航点信息
                send_mission(the_connection, wp, seq)

        elif message["mavpackettype"] == mavutil.mavlink.MAVLink_mission_ack_message.msgname:

            # 若回传信息为任务被接受（mission_ack信息）
            if message["mission_type"] == mavutil.mavlink.MAV_MISSION_TYPE_MISSION and \
                    message["type"] == mavutil.mavlink.MAV_MISSION_ACCEPTED:

                print("Mission uploaded successfully")
                break


# 上传航点集并阻塞程序直到完成全部预定航点任务，并且打印动态参数
def upload_mission_till_completed(the_connection, wp, home_position, track_list):
    mission_upload(the_connection, wp, home_position)

    if input("任务上传完成，输入任意内容开始自动飞行： "):
        pass

    mode_set(the_connection, 10)

    # 注意其是此函数的局部变量，但因为记录的位置消息只需要在执行次程序的过程中使用，认为可以不用作为全局变量

    wp_list_len = gain_mission(the_connection)

    # 在到达最后航点前，实时显示动态轨迹，并记录track point
    while mission_accomplished(the_connection, wp_list_len) < 0:

        gain_track_of_time(the_connection, track_list)

    print("reaching last waypoint of NO.", wp_list_len, ", mission accomplished!")


# 似乎没用
def clear_waypoint(vehicle):
    message = mavutil.mavlink.MAVLink_mission_clear_all_message(target_system=vehicle.target_system,
                                                                target_component=vehicle.target_component,
                                                                mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    # send mission clear all command to the vehicle
    vehicle.mav.send(message)

    # create mission request list message
    message = mavutil.mavlink.MAVLink_mission_request_list_message(target_system=vehicle.target_system,
                                                                target_component=vehicle.target_component,
                                                                mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

    # send the message to the vehicle
    vehicle.mav.send(message)

    # wait mission count message
    message = vehicle.recv_match(type=mavutil.mavlink.MAVLink_mission_count_message.msgname,
                                 blocking=True)

    # convert this message to dictionary
    message = message.to_dict()

    # get the mission item count
    count = message["count"]
    print("Total mission item count:", count)


'''
对接视觉需要的函数
'''
# 手写方法，已弃用 对摄像头延时等影响造成延迟的消除，delay具体值需要使用实验测定
def delay_eliminate(track_list, time, delay=500):
    # 使用二分法查找离要求最近的时刻
    tail = len(track_list)-1
    head = 0
    while (tail - head) > 3 :
        mid = int((head+tail)*0.5)
        if track_list[mid].time - (time-delay) > 0:
            tail = mid
        elif track_list[mid].time - (time-delay) < 0:
            head = mid
        # 若表中恰好有所需时刻
        elif track_list[mid].time == (time-delay):
            return track_list[mid]

    # 对二分后结果进行处理，分奇数或偶数
    if tail-head == 3:
        if (track_list[head+1].time - (time-delay)) < (track_list[head+2].time - (time-delay)):
            return track_list[head+1]
        else:
            return track_list[head+2]
    elif tail-head == 2:
        return track_list[head+1]


'''
指定形状航线生成函数
'''
def XYtoGPS(x, y, ref_lat, ref_lon):
    # input GPS and Reference GPS in degrees
    # output XY in meters (m) X:North Y:East
    x_rad = float(x) / CONSTANTS_RADIUS_OF_EARTH
    y_rad = float(y) / CONSTANTS_RADIUS_OF_EARTH
    c = math.sqrt(x_rad * x_rad + y_rad * y_rad)

    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    ref_sin_lat = math.sin(ref_lat_rad)
    ref_cos_lat = math.cos(ref_lat_rad)

    if abs(c) > 0:
        sin_c = math.sin(c)
        cos_c = math.cos(c)

        lat_rad = math.asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c)
        lon_rad = (ref_lon_rad + math.atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c))

        lat = math.degrees(lat_rad)
        lon = math.degrees(lon_rad)

    else:
        lat = math.degrees(ref_lat)
        lon = math.degrees(ref_lon)

    return [lat, lon]

def GPStoXY(lat, lon, height, ref_lat, ref_lon):
    # input GPS and Reference GPS in degrees
    # output XY in meters (m) X:North Y:East
    lat_rad = radians(lat)
    lon_rad = radians(lon)
    ref_lat_rad = radians(ref_lat)
    ref_lon_rad = radians(ref_lon)

    sin_lat = sin(lat_rad)
    cos_lat = cos(lat_rad)
    ref_sin_lat = sin(ref_lat_rad)
    ref_cos_lat = cos(ref_lat_rad)

    cos_d_lon = cos(lon_rad - ref_lon_rad)

    arg = np.clip(ref_sin_lat * sin_lat + ref_cos_lat * cos_lat * cos_d_lon, -1.0, 1.0)
    c = acos(arg)

    k = 1.0
    if abs(c) > 0:
        k = (c / sin(c))

    x = float(k * (ref_cos_lat * sin_lat - ref_sin_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH)
    y = float(k * cos_lat * sin(lon_rad - ref_lon_rad) * CONSTANTS_RADIUS_OF_EARTH)

    return [x, y, height]


def GPStoDISTANCE(wp1, wp2):
    C = (math.sin(wp1.lat * math.pi / 180) * math.sin(wp2.lat * math.pi / 180) +
         math.cos(wp1.lat * math.pi / 180) * math.cos(wp2.lat * math.pi / 180) * math.cos((wp1.lon - wp2.lon) * math.pi / 180))
    Distance = CONSTANTS_RADIUS_OF_EARTH * math.acos(C) * math.pi / 180
    return Distance

# 根据上传的两个坐标点，通过自动设置更多的坐标点，生成一个两坐标之间的直线航线
def wp_straight_course(wp, precision):

    lat_len = (wp[1].lat - wp[0].lat)
    lon_len = (wp[1].lon - wp[0].lon)
    alt_len = (wp[1].alt - wp[0].alt)
    lat_len /= precision
    lon_len /= precision
    alt_len /= precision

    wp_list = [wp[0]]

    i = 0
    for i in range(0, precision):
        wp_new = Waypoint(wp_list[i].lat+lat_len, wp_list[i].lon+lon_len, wp_list[i].alt+alt_len)
        #wp_new.show()
        wp_list.append(wp_new)
        i+=1

    wp_list.append(wp[1])
    return wp_list


# 在两点之间形成近似圆弧航线（angle指定弧线的圆心角角度，direction取-1为顺时针，取1为逆时针，默认逆时针）
def wp_circle_course(wp, precision, angle, direction=1):
    lat_len = wp[1].lat - wp[0].lat
    lon_len = wp[1].lon - wp[0].lon
    alt_len = wp[1].alt - wp[0].alt

    # 半圆因为角度计算关系不能直接用下面的方法，但我懒得加一个专门的半圆航线了
    if angle == 180:
        angle = 179.9
    # 转为弧度制
    angle = angle * math.pi / 180

    # 精度差为零时需要单独计算
    # 注意使用弧度制
    if lon_len == 0 and lat_len > 0:
        theta_start = math.pi * 0.5
    elif lon_len == 0 and lat_len < 0:
        theta_start = math.pi * 1.5
    else:
        theta_start = math.atan(lat_len/lon_len)
    if lon_len >= 0: #一四象限
        pass
    else: #二三象限
        theta_start += math.pi

    # 计算半径和角度步长
    radius = math.sqrt(lat_len*lat_len + lon_len*lon_len*1.2) * 0.45 / math.sin(angle*0.5)#非常粗略
    #r_2 = math.asin(math.sqrt(pow(math.sin(lat_len / 2), 2) + math.cos(wp[1].lat * math.pi / 180) * math.cos(wp[0].lat * math.pi / 180)*pow(lon_len, 2))) * CONSTANTS_RADIUS_OF_EARTH
    #r_2 = GPStoDISTANCE(wp[0], wp[1])
    theta_step = angle / precision

    # 判断圆心位置
    if (direction >= 0 and angle < 180) or (direction < 0 and angle > 180):
        center = Waypoint(wp[0].lat+radius*math.sin(0.5*math.pi+theta_start-0.5*angle),
                          wp[0].lon+radius*math.cos(0.5*math.pi+theta_start-0.5*angle),
                          wp[0].alt+0.5*alt_len)

        '''
        # 新方法
        [lat, lon] = XYtoGPS(r_2*math.sin(0.5*math.pi+theta_start-0.5*angle),
                             r_2*math.cos(0.5*math.pi+theta_start-0.5*angle), wp[0].lat, wp[0].lon)
        center = Waypoint(lat, lon, wp[0].alt+0.5*alt_len)
        '''

    else:
        center = Waypoint(wp[0].lat+radius*math.sin(-0.5*math.pi+theta_start+0.5*angle),
                          wp[0].lon+radius*math.cos(-0.5*math.pi+theta_start+0.5*angle),
                          wp[0].alt+0.5*alt_len)

        '''
        # 新方法
        [lat, lon] = XYtoGPS(r_2 * math.sin(-0.5 * math.pi + theta_start + 0.5 * angle),
                             r_2 * math.cos(-0.5 * math.pi + theta_start + 0.5 * angle), wp[0].lat, wp[0].lon)
        center = Waypoint(lat, lon, wp[0].alt + 0.5 * alt_len)
        '''

    wp_list = [wp[0]]
    for i in range(0, precision):
        if direction >= 0:
           theta = (+theta_start - math.pi*0.5 - angle*0.5) + theta_step * (i+1)# 顺时针适用
        else:
           theta = (+theta_start + math.pi*0.5 + angle*0.5) - theta_step * (i+1)# 逆时针适用
        lat_new = center.lat + radius * math.sin(theta)
        lon_new = center.lon + radius * math.cos(theta)

        # 新方法
        # [lat_new, lon_new] = XYtoGPS(r_2*math.sin(theta), r_2*math.cos(theta), lat_new, lon_new)

        alt_new = wp[0].alt + alt_len / precision * (i+1)
        wp_new = Waypoint(lat_new, lon_new, alt_new)
        # wp_new.show()
        wp_list.append(wp_new)
        i += 1

    wp_list.pop(-1)
    wp_list.append(wp[1])
    return wp_list


# 根据两个航点，在其中生成四瓣型侦察航线（视解算正确率进行航线形状修改），根据靶标与起飞区的相对方向对航点顺序进行调整
def wp_detect_course(wp, alt, approach_angle='east'):
    # 生成侦察区四邻域顶点，距中心30米
    wp_north = Waypoint(wp.lat + 0.0004, wp.lon, alt)
    wp_west = Waypoint(wp.lat, wp.lon - 0.0004, alt)
    wp_south = Waypoint(wp.lat - 0.0004, wp.lon, alt)
    wp_east = Waypoint(wp.lat, wp.lon + 0.0004, alt)

    if approach_angle == 'east':
        [wp1, wp2, wp3, wp4] = [wp_west, wp_east, wp_south, wp_north]
    elif approach_angle == 'west':
        [wp1, wp2, wp3, wp4] = [wp_east, wp_west, wp_north, wp_south]
    elif approach_angle == 'south':
        [wp1, wp2, wp3, wp4] = [wp_north, wp_south, wp_west, wp_east]
    else:
        [wp1, wp2, wp3, wp4] = [wp_south, wp_north, wp_east, wp_west]

    line12 = wp_straight_course([wp1, wp2], 2)
    circle23 = wp_circle_course([wp2, wp3], 3, 270, direction=-1)
    line34 = wp_straight_course([wp3, wp4], 2)
    circle42 = wp_circle_course([wp4, wp2], 3, 270, direction=-1)
    line21 = wp_straight_course([wp2, wp1], 2)
    circle14 = wp_circle_course([wp1, wp4], 3, 270, direction=-1)
    line43 = wp_straight_course([wp4, wp3], 2)
    circle31 = wp_circle_course([wp3, wp1], 3, 270, direction=-1)

    detect_course = line12
    detect_course.pop(-1)
    detect_course.extend(circle23)
    detect_course.pop(-1)
    detect_course.extend(line34)
    detect_course.pop(-1)
    detect_course.extend(circle42)
    detect_course.pop(-1)
    detect_course.extend(line21)
    detect_course.pop(-1)
    detect_course.extend(circle14)
    detect_course.pop(-1)
    detect_course.extend(line43)
    detect_course.pop(-1)
    detect_course.extend(circle31)
    detect_course.pop(-1)

    return detect_course


'''
投弹相关函数
'''
# 已弃用，见wp_bombing_course
def bombing_course(wp_now, wp_target, precision, course_len, radius, theta, direction=1):

# 自动生成航路点集
    wp_now.alt = 35
    lat_len = wp_now.lat - wp_target.lat
    lon_len = wp_now.lon - wp_target.lon

    theta = math.atan(lat_len / lon_len)
    if lon_len >= 0: # 一四象限
        pass
    else: # 二三象限
        theta += math.pi

    #theta = theta * math.pi / 180

    # 暂时想不到根据距离推算经纬度关系的方法，姑且用0.001度约为一百米的方法估测
    # d_lat = 2 * radius * math.sin(theta) * 1e-5
    # d_lon = 2 * (radius / 1.1) * math.cos(theta) * 1e-5
    s_lat = course_len * math.sin(theta) * 1e-5
    s_lon = course_len * math.cos(theta) * 1e-5
    # 将任务结尾设定在越过目标点一定距离的地方
    flyby_len = 30
    flyby_lat = flyby_len*math.sin(theta+math.pi) * 1e-5
    flyby_lon = flyby_len*math.cos(theta+math.pi) * 1e-5

    # 生成两段弧线和一段直线组成的掉头航线
    wp_start = Waypoint(wp_now.lat+s_lat, wp_now.lon+s_lon, wp_now.alt)
    wp_mid = Waypoint(wp_start.lat+s_lat*0.5, wp_start.lon+s_lon*0.5, wp_now.alt)
    wp_end = Waypoint(wp_target.lat+flyby_lat, wp_target.lon+flyby_lon, 30)
    wp_far = Waypoint(wp_now.lat+2*s_lat, wp_now.lon+2*s_lon, wp_now.alt)

    half_chord_len = 0.6*radius*1e-5 # 转向处的半弦长

    wp1 = Waypoint(wp_mid.lat+half_chord_len*math.sin(-direction*math.pi*0.5+theta+math.pi*0.05),
                   wp_mid.lon+half_chord_len*math.cos(-direction*math.pi*0.5+theta+math.pi*0.05),
                   wp_now.alt)
    wp2 = Waypoint(wp_mid.lat+half_chord_len*math.sin(direction*math.pi*0.5+theta+math.pi*0.05),
                   wp_mid.lon+half_chord_len*math.cos(direction*math.pi*0.5+theta+math.pi*0.05),
                   wp_now.alt)

    wp_circle1 = [wp_now, wp_far]
    wp_circle2 = [wp_far, wp1]
    wp_circle3 = [wp1, wp_start]
    wp_line1 = [wp_start, wp_target]
    wp_line2 = [wp_target, wp_end]

    # 转过270度对准航线
    wp_circle_list = wp_circle_course(wp_circle1, 2*precision,180, -direction)
    wp_circle_list.pop(0)
    wp_circle_list.pop(-1) # 取出重复的航点
    wp_circle_list.extend(wp_circle_course(wp_circle2, precision-1, 150, -direction))
    wp_circle_list.pop(-1)
    wp_circle_list.extend(wp_circle_course(wp_circle3, precision, 30, direction))
    wp_circle_list.pop(-1)
    wp_circle_list.pop(-1)

    # 直线进入投弹航线
    wp_line_list = wp_straight_course(wp_line1, precision+3)
    wp_line_list.pop(-1)
    wp_line_list.extend(wp_straight_course(wp_line2, precision))

    wp_bomb_drop = wp_circle_list
    wp_bomb_drop.extend(wp_line_list)

    return wp_bomb_drop


# 投弹航线生成，以进场航线指向为主要参数, 角度以正北为零点，逆时针增加0-360
def wp_bombing_course(wp_target, approach_angle,
                      length_enter=30,  radius=50, length_approach=80, length_bomb=20, length_left=40,
                      precision_circle=4, precision_approach=3, precision_bomb=10, precision_enter=2,
                      alt_target=7, alt_bomb_start=10, alt_approach=14, alt_left=15, length_side_points=10):
    # 转为弧度制
    approach_angle = (approach_angle - 90) * math.pi / 180

    wp_target.alt = alt_target

    # 生成投弹航线
    len_north1 = length_bomb * math.sin(approach_angle)
    len_east1 = length_bomb * math.cos(approach_angle)
    [lat1, lon1] = XYtoGPS(len_north1, len_east1, ref_lat=wp_target.lat, ref_lon=wp_target.lon)
    wp_bomb = Waypoint(lat=lat1, lon=lon1, alt=alt_bomb_start)
    # 生成投弹部分航线
    bomb_line = wp_bombing_insert_course([wp_bomb, wp_target], precision_bomb, length_side_points)

    # 生成直线进近航线
    len_north2 = length_approach * math.sin(approach_angle)
    len_east2 = length_approach * math.cos(approach_angle)
    [lat2, lon2] = XYtoGPS(len_north2, len_east2, ref_lat=wp_bomb.lat, ref_lon=wp_bomb.lon)
    wp_approach = Waypoint(lat=lat2, lon=lon2, alt=alt_approach)
    approach_line = wp_straight_course([wp_approach, wp_bomb], precision_approach)

    # 生成掉头对准航线
    len_north3 = 2 * radius * math.sin(approach_angle - 0.5 * math.pi)
    len_east3 = 2 * radius * math.cos(approach_angle - 0.5 * math.pi)
    [lat3, lon3] = XYtoGPS(len_north3, len_east3, ref_lat=wp_approach.lat, ref_lon=wp_approach.lon)
    wp_turn_start = Waypoint(lat=lat3, lon=lon3, alt=alt_approach)
    turn_circle = wp_circle_course([wp_turn_start, wp_approach], precision_circle, 180)

    # 生成几个航点使飞机更好地进入掉头航线
    len_north4 = length_enter * math.sin(approach_angle + math.pi)
    len_east4 = length_enter * math.cos(approach_angle + math.pi)
    [lat4, lon4] = XYtoGPS(len_north4, len_east4, ref_lat=wp_turn_start.lat, ref_lon=wp_turn_start.lon)
    wp_enter = Waypoint(lat=lat4, lon=lon4, alt=alt_approach)
    enter_line = wp_straight_course([wp_enter, wp_turn_start], precision_enter)

    # 生成靶标后航点以保持飞机平飞
    len_north5 = length_left * math.sin(approach_angle + math.pi)
    len_east5 = length_left * math.cos(approach_angle + math.pi)
    [lat5, lon5] = XYtoGPS(len_north5, len_east5, ref_lat=wp_target.lat, ref_lon=wp_target.lon)
    wp_left = Waypoint(lat=lat5, lon=lon5, alt=alt_left)

    # 生成完整投弹航线
    bomb_course = enter_line
    bomb_course.pop(-1)
    bomb_course.extend(turn_circle)
    bomb_course.pop(-1)
    bomb_course.extend(approach_line)
    bomb_course.pop(-1)
    bomb_course.extend(bomb_line)
    bomb_course.append(wp_left)

    return bomb_course


# 投弹左右插点航线
def wp_bombing_insert_course(wp, numbers, distance):

    start_GPS = [wp[0].lat, wp[0].lon, wp[0].alt]
    end_GPS = [wp[1].lat, wp[1].lon, wp[1].alt]

    start = GPStoXY(start_GPS[0], start_GPS[1], start_GPS[2], start_GPS[0], start_GPS[1])
    # print(start)
    end = GPStoXY(end_GPS[0], end_GPS[1], end_GPS[2], start_GPS[0], start_GPS[1])
    # print(end)
    x = (end[0] - start[0]) / (numbers + 1)
    y = (end[1] - start[1]) / (numbers + 1)
    z = (end[2] - start[2]) / (numbers + 1)
    if y == 0:
        theta = pi / 2
    elif x == 0:
        theta = 0
    else:
        k = x / y

        # 求出斜率
        theta = atan(-1 / k)

    temp_result = np.zeros((numbers, 3))
    result = np.zeros((numbers, 3))

    for i in range(numbers):
        if i % 2 == 0:
            temp_result[i][0] = start[0] + (i + 1) * x - distance * cos(theta)
            temp_result[i][1] = start[1] + (i + 1) * y - distance * sin(theta)
        else:
            temp_result[i][0] = start[0] + (i + 1) * x + distance * cos(theta)
            temp_result[i][1] = start[1] + (i + 1) * y + distance * sin(theta)
        temp_result[i][2] = start[2] + (i + 1) * z

    wp_list = []

    alt_range = wp[1].alt - wp[0].alt
    alt_step = alt_range / numbers

    # print(temp_result)
    for i in range(numbers):
        result = XYtoGPS(temp_result[i][0], temp_result[i][1], start_GPS[0], start_GPS[1])
        alt = wp[0] + i * alt_step
        wp_list.append(Waypoint(result[0], result[1], alt))
    wp_list.append(wp[1])
    return wp_list


# 驱动电机执行投弹动作
def bomb_drop(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1000, 0, 0, 0, 0, 0)
    print("bomb away!")


def initiate_bomb_drop(the_connection, angle):
    # 因为顺逆不同进行转换
    theta = 360 - angle
    while True:
        heading = gain_heading(the_connection)
        # 在理想的航向范围内
        if theta - 45 < heading < theta + 100:
            break


'''
飞机动作控制函数
'''
def return_to_launch(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0, 0, 0, 0, 0, 0, 0, 0)
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                         0,  # confirmation
                                         77,  # param1
                                         0,  # param2
                                         0,  # param3
                                         0,  # param4
                                         0,  # param5
                                         0,  # param6
                                         0)  # param7
    msg = rec_match_received(the_connection, "COMMAND_ACK")
    if msg.result == 0:
        print("return to home")
    else:
        print("RTL failed")



def loiter_at_present(the_connection, alt):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, -30, 0, 0, 0, alt)
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                         0,  # confirmation
                                         77,  # param1
                                         0,  # param2
                                         0,  # param3
                                         0,  # param4
                                         0,  # param5
                                         0,  # param6
                                         0)  # param7
    msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    result = msg.result
    if result == 0:
        lat = gain_position_now(the_connection).lat
        lon = gain_position_now(the_connection).lon
        print("loiter at present of lat: ", lat, "lon: ", lon, "alt: ", alt)
    else:
       print("loiter failed")
       error_process(the_connection)


def loiter(the_connection, position):
    print(position.lat, position.lon)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, -30, 0, position.lat, position.lon, position.alt)
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                         0,  # confirmation
                                         77,  # param1
                                         0,  # param2
                                         0,  # param3
                                         0,  # param4
                                         0,  # param5
                                         0,  # param6
                                         0)  # param7
    msg = rec_match_received(the_connection, "COMMAND_ACK")
    if msg.result == 0:
        print("loiter")
    else:
        print("loiter failed")


# 环形飞行航线（仅测试用）
def yard_fly(the_connection, wp, home_position, track_list):
    wp_line1 = [wp[3], wp[0]]
    wp_circle1 = [wp[0], wp[1]]
    wp_line2 = [wp[1], wp[2]]
    wp_circle2 = [wp[2], wp[3]]

    wp_list = (wp_circle_course(wp_circle1, 3, 180, 1))
    wp_list.pop(-1)
    wp_list.pop(-1)
    wp_list.extend(wp_straight_course(wp_line2, 3))
    wp_list.pop(-1)
    wp_list.extend(wp_circle_course(wp_circle2, 3, 180, 1))
    wp_list.pop(-1)
    wp_list.extend(wp_straight_course(wp_line1, 3))
    #wp_list.append(home_position)

    print(len(wp_list))
    mission_upload(the_connection, wp_list, home_position)
    if input("输入0进行环操场航线： ") == '0':
        pass

    count = 0
    while True:#input("输入0进行环操场航线： ") == '0':

      mode_set(the_connection, 10)

      while rec_match_received(the_connection, 'MISSION_CURRENT').seq < len(wp_list)-1:
         gain_track_of_time(the_connection, track_list)
         continue
      count += 1
      print("circle NO.", count, " completed")
      loiter(the_connection, home_position)
      if input("输入0进行下一圈： ") == '0':
          pass


'''
数字后处理算法函数
'''
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
        if dict[target1] + dict[target2] + dict[target3] > DETECT_CONFIDENCE * LEN_OF_TARGET_LIST:
            print("vision detection result:   ", target1, "   ", target2, "   ", target3)
            for n in range(len(key)):
                print("result: ", key[n], "count: ", dict[key[n]])
            if len(key) >= 6:
                target4, target5, target6 = key[3:6]
                return [target1, target2, target3, target4, target5, target6]
            elif len(key) >= 5:
                target4, target5= key[3:5]
                return [target1, target2, target3, target4, target5]
            elif len(key) >= 4:
                target4 = key[3:4]
                return [target1, target2, target3, target4]
            else:
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


# 完整解算，传入靶标数字和各项记录消息，返回一个坐标
def target_transfer(time_target_dict, vision_inform, num, timestamps, target_time, tracks, delay):
    target_list = []
    # 记录到的每一个侦察数据
    for i in range(len(time_target_dict)):
        # 识别到的数字不是正确数字
        if vision_inform[i] != num:
            continue

        # 使用二分法查找检测目标在位姿点集中的位置
        order = bisect.bisect_left(timestamps, target_time[i])

        current_time = target_time[i]

        # 对目标附近的插值取点进行保护
        if order > 4:
            fit_start = order - 5
        else:
            fit_start = 0
        if order + 5 >= len(timestamps):
            fit_end = len(timestamps) - 1
        else:
            fit_end = order + 5
        if order == len(timestamps) or order == 0:
            print("mother fucker")
            continue

        # 筛选拟合点范围
        selected_indices = timestamps[fit_start:fit_end + 1]

        # 提取选定的数据点
        selected_tracks = [tracks[j] for j in selected_indices]
        selected_timestamps = [timestamps[j] for j in selected_indices]

        # 三次拟合插值
        roll_interp = UnivariateSpline(selected_timestamps, [data.roll for data in selected_tracks], s=0)
        pitch_interp = UnivariateSpline(selected_timestamps, [data.pitch for data in selected_tracks], s=0)
        yaw_interp = UnivariateSpline(selected_timestamps, [data.yaw for data in selected_tracks], s=0)
        lat_interp = UnivariateSpline(selected_timestamps, [data.lat for data in selected_tracks], s=0)
        lon_interp = UnivariateSpline(selected_timestamps, [data.lon for data in selected_tracks], s=0)
        alt_interp = UnivariateSpline(selected_timestamps, [data.alt for data in selected_tracks], s=0)

        target = coordinate_transfer(lat_interp(current_time), lon_interp(current_time), alt_interp(current_time),
                                     yaw_interp(current_time),
                                     pitch_interp(current_time), roll_interp(current_time), vision_inform[1],
                                     vision_inform[2], vision_inform[0])

        # 记录解算结果
        with open(file='C:/Users/35032/Desktop/transfer_result.txt', mode='a') as f:
            f.write("靶标坐标 lat: " + str(target.lat) + " lon: " + str(target.lon) + " num: " + str(target.num)
                    + " delay: " + str(delay))

        target_list.append(target)

    '''
    记录target并使用迭代算法得到最终结果
    '''
    point_list = []
    for j in range(len(target_list)):
        point_list[j] = (target_list[j].lat, target_list[j].lon)

    def distance(a, b):
        return math.sqrt((a[1] - b[1]) ** 2 + (a[0] - b[0]) ** 2)

    def mean(points, length):
        x = y = 0
        for k in range(length):
            x += points[i][0]
            y += points[i][1]
        return x / length, y / length

    def center(points, iteration=1000):
        """
            points (list of tuples)
        """
        if not points:
            return None
        old_center = points[0]
        length = math.ceil(len(points) * 2 / 3)
        while iteration > 0:
            iteration -= 1
            points.sort(key=lambda p: distance(p, old_center))
            new_center = mean(points, length)
            if old_center == new_center:
                return new_center
            old_center = new_center

    target_point = center(point_list)

    return Waypoint(target_point[0], target_point[1], 0)


# 从数据近似上识别是否可能有识别错误
def wrong_number(num_list):
    def is_same_or_similar(nums_lis):  # list[int]):
        confusing_digits = [[1, 7], [5, 6], [2, 7], [3, 8], [7, 9]]
        index_dict = {12: [False, False], 13: [False, False], 23: [False, False]}  # :dict[int, list[bool]]
        num_digits = [divmod(num, 10) for num in num_list]
        for i in range(3):
            for j in range(i + 1, 3):
                if num_digits[i][0] == num_digits[j][0] and num_digits[i][1] == num_digits[j][1]:
                    index_dict[(i + 1) * 10 + (j + 1)][0] = True
                tens_digits_pair = [num_digits[i][0], num_digits[j][0]]
                ones_digits_pair = [num_digits[i][1], num_digits[j][1]]
                # confusion表是升序
                tens_digits_pair.sort()
                ones_digits_pair.sort()
                if (tens_digits_pair in confusing_digits or tens_digits_pair[0] == tens_digits_pair[1]) \
                        and (ones_digits_pair in confusing_digits or ones_digits_pair[0] == ones_digits_pair[1]):
                    index_dict[(i + 1) * 10 + (j + 1)][1] = True
        # key为下标关系，第一个bool判是否相同， 第二个判是否相似
        return index_dict

    result1, result2, result3 = list(is_same_or_similar(num_list).values())

    if not result1[0] and result1[1]:
        return [0, 1]
    elif not result2[0] and result2[1]:
        return [0, 2]
    elif not result3[0] and result3[1]:
        return [1, 2]
    else:
        return [-1, -1]
