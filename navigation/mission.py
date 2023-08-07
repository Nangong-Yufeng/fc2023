import sys
sys.path.append('../gui')
from pymavlink import mavutil
import math
from class_list import Waypoint, track_point
from get_para import gain_mission, waypoint_reached, position_now, mission_current, gain_track_of_time
from preflight import mode_set
from error_process import error_process, rec_match_received
from gui import putPathPoint, setTargetPoints, runGui, setMapLocation
from threading import Thread
from trajectory import trajectory_cal


# 通用任务函数

def send_mission_list(the_connection, wp):
    wp_list = mavutil.mavlink.MAVLink_mission_count_message(the_connection.target_system,
                                                            the_connection.target_component, len(wp),
                                                            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
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


def mission_accomplished(the_connection, wp_list_len):
    msg = rec_match_received(the_connection, 'MISSION_CURRENT')
    if msg.seq == wp_list_len and msg.mission_state == 5:
        return 1
    else:
        return -10


def mission_upload(the_connection, wp, home_position):

    wp.insert(0, home_position)

    # 上传航点数量信息
    send_mission_list(the_connection, wp)

    # 在地图中显示静态航点
    waypoint_print_list = []
    for count in range(len(wp)):
        waypoint_print_list.append((wp[count].lat, wp[count].lon))
        #print(waypoint_print_list)
    #setTargetPoints(waypoint_print_list)

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
        print("loitering")

    mode_set(the_connection, 10)

    # 注意其是此函数的局部变量，但因为记录的位置消息只需要在执行次程序的过程中使用，认为可以不用作为全局变量

    wp_list_len = gain_mission(the_connection)

    # 在到达最后航点前，实时显示动态轨迹，并记录track point
    while mission_accomplished(the_connection, wp_list_len) < 0:
        gain_track_of_time(the_connection, track_list)
        po_now = position_now(the_connection)
        point = (po_now.lat, po_now.lon)
        #putPathPoint(point)

    print("reaching last waypoint of NO.", wp_list_len, ", mission accomplished!")


def clear_waypoint(the_connection):
    msg = mavutil.mavlink.MAVLink_mission_clear_all_message(the_connection.target_system,the_connection.target_component,
                                                            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    the_connection.mav.send(msg)


# 指定形状航线生成函数

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

    angle = angle * math.pi / 180 # 转为弧度制

    # 注意使用弧度制
    theta_start = math.atan(lat_len/lon_len)
    if lon_len >= 0: # 一四象限
        pass
    else: # 二三象限
        theta_start += math.pi

    # 计算半径和角度步长
    radius = math.sqrt(lat_len*lat_len + lon_len*lon_len*1.2) * 0.45 / math.sin(angle*0.5)#非常粗略
    theta_step = angle / precision

    # 判断圆心位置
    if direction >= 0:
        center = Waypoint(wp[0].lat+radius*math.sin(0.5*math.pi+theta_start-0.5*angle),
                          wp[0].lon+radius*math.cos(0.5*math.pi+theta_start-0.5*angle),
                          wp[0].alt+0.5*alt_len)

    else:
        center = Waypoint(wp[0].lat+radius*math.sin(-0.5*math.pi+theta_start+0.5*angle),
                          wp[0].lon+radius*math.cos(-0.5*math.pi+theta_start+0.5*angle),
                          wp[0].alt+0.5*alt_len)

    wp_list = [wp[0]]
    for i in range(0, precision):
        if direction >= 0:
           theta = (+theta_start - math.pi*0.5 - angle*0.5) + theta_step * (i+1) #顺时针适用
        else:
           theta = (+theta_start + math.pi*0.5 + angle*0.5) - theta_step * (i+1) #逆时针适用
        lat_new = center.lat + radius * math.sin(theta)
        lon_new = center.lon + radius * math.cos(theta)
        alt_new = wp[0].alt + alt_len / precision * (i+1)
        wp_new = Waypoint(lat_new, lon_new, alt_new)
        #wp_new.show()
        wp_list.append(wp_new)
        i += 1

    wp_list.append(wp[1])
    return wp_list


# 投弹相关函数

# 自动生成投弹航线并执行，采用反向飞离然后一字掉头后直线进场的方式，参数可决定转转弯方向（顺或逆）
# 修正，采用半圆航线飞至一定距离后，弧形航线进场投弹；后半段航线不变，前半段改变。此方法适用于盘旋侦察，如使用其他侦察航线则需要
def execute_bomb_course(the_connection, home_position, track_list, wp_now, wp_target, precision, course_len, direction, radius):

# 自动生成航路点集
    lat_len = wp_now.lat - wp_target.lat
    lon_len = wp_now.lon - wp_target.lon
    theta = math.atan(lat_len / lon_len)
    if lon_len >= 0: # 一四象限
        pass
    else: # 二三象限
        theta += math.pi

    # 暂时想不到根据距离推算经纬度关系的方法，姑且用0.001度约为一百米的方法估测
    #d_lat = 2 * radius * math.sin(theta) * 1e-5
    #d_lon = 2 * (radius / 1.1) * math.cos(theta) * 1e-5
    s_lat = course_len * math.sin(theta) * 1e-5
    s_lon = course_len * math.cos(theta) * 1e-5
    # 将任务结尾设定在越过目标点一定距离的地方
    flyby_len = 100
    flyby_lat = flyby_len*math.sin(theta+math.pi) * 1e-5
    flyby_lon = flyby_len*math.cos(theta+math.pi) * 1e-5

    # 生成两段弧线和一段直线组成的掉头航线
    wp_start = Waypoint(wp_now.lat+s_lat, wp_now.lon+s_lon, wp_now.alt)
    wp_mid = Waypoint(wp_start.lat+s_lat*0.5, wp_start.lon+s_lon*0.5, wp_now.alt)
    wp_end = Waypoint(wp_target.lat+flyby_lat, wp_target.lon+flyby_lon, 15)
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
    wp_line_list = wp_straight_course(wp_line1, precision)
    wp_line_list.pop(-1)
    wp_line_list.extend(wp_straight_course(wp_line2, precision))

    wp_bomb_drop = wp_circle_list
    wp_bomb_drop.extend(wp_line_list)
    '''
    wp_line1 = [wp_now, wp_start]
    wp_circle1 = [wp_start, wp1]
    wp_circle2 = [wp1, wp2]
    wp_circle3 = [wp2, wp_start]
    wp_line2 = [wp_start, wp_target]
    wp_line3 = [wp_target, wp_end]

    #直线开始进入掉头航线
    wp_line1_list = wp_straight_course(wp_line1, 3)
    wp_line1_list.pop(0) #起点航线有些奇怪，简单地去掉开头几个航点能让飞机跑的更好，后续加入飞机的方向可能会更好一些
    wp_line1_list.pop(0)
    #wp_line1_list = [wp_start]

    #掉头部分航路点
    wp_circle_list = wp_circle_course(wp_circle1, precision, 30, -direction)
    wp_circle_list.pop(-1) #取出重复的航点
    wp_circle_list.extend(wp_circle_course(wp_circle2, 2*precision, 270, direction))
    wp_circle_list.pop(-1)
    wp_circle_list.extend(wp_circle_course(wp_circle3, precision, 30, -direction))
    wp_circle_list.pop(-1)
    wp_circle_list.pop(-1)

    #完成掉头进入直线投弹航线
    wp_line2_list = wp_straight_course(wp_line2, 3)
    wp_line2_list.pop(-1)
    wp_line3_list = wp_straight_course(wp_line3, 3)

    wp_bomb_drop = wp_line1_list
    wp_bomb_drop.extend(wp_circle_list)
    wp_bomb_drop.extend(wp_line2_list)
    wp_bomb_drop.extend(wp_line3_list)
    '''
    # 开启地图脚本并在地图中显示静态航点
    Thread(target=runGui).start()
    setMapLocation((home_position.lat, home_position.lon))
    waypoint_print_list = []
    for count in range(len(wp_bomb_drop)):
        waypoint_print_list.append((wp_bomb_drop[count].lat, wp_bomb_drop[count].lon))
        # print(waypoint_print_list)
    setTargetPoints(waypoint_print_list)


# 上传并等待任务执行

    # 上传任务
    upload_mission_till_completed(the_connection, wp_bomb_drop, home_position, track_list)
    print("bombs away!")
    print(len(track_list))

def not_guilty_to_drop_the_bomb(the_connection, wp_target, time):
    wp_target = Waypoint(-35.35941937, 149.16062729, 0)
    # 经过投弹解算后认为的落点与目标的距离
    distance = trajectory_cal(the_connection, 0.5, wp_target)


# 驱动电机执行投弹动作
def bomb_drop(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, 5, 1000, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    if msg.result == 0:
        print("bomb away!")
        return 0
    else:
        print("failed to drop the bomb!")
        return -10


# 飞机动作控制函数

def loiter_at_present(the_connection, alt):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 30, 0, 0, 0, alt)
    msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    result = msg.result
    if result == 0:
        lat = position_now(the_connection).lat
        lon = position_now(the_connection).lon
        print("loiter at present of lat: ",lat, "lon: ", lon, "alt: ", alt)
    else:
       print("loiter failed")
       error_process(the_connection)

# 环形飞行航线（仅测试用）
def yard_fly(the_connection, wp, home_position, track_list):
    wp_line1 = [wp[3], wp[0]]
    wp_circle1 = [wp[0], wp[1]]
    wp_line2 = [wp[1], wp[2]]
    wp_circle2 = [wp[2], wp[3]]

    wp_list = wp_straight_course(wp_line1, 3)
    wp_list.extend(wp_circle_course(wp_circle1, 3, 180, 1))
    wp_list.extend(wp_straight_course(wp_line2, 3))
    wp_list.extend(wp_circle_course(wp_circle2, 3, 180, 1))

    Thread(target=runGui).start()
    setMapLocation((home_position.lat, home_position.lon))
    waypoint_print_list = []
    for count in range(len(wp)):
        waypoint_print_list.append((wp[count].lat, wp[count].lon))
        # print(waypoint_print_list)
    setTargetPoints(waypoint_print_list)

    upload_mission_till_completed(the_connection, wp_list, home_position, track_list)