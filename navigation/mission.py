import time
from pymavlink import mavutil
import math
from class_list import Waypoint
from get_para import gain_mission, waypoint_reached
from preflight import mode_set
import sys

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


def mission_upload(the_connection, wp, home_position):

    wp.insert(0, home_position)

    #上传航点数量信息
    send_mission_list(the_connection, wp)

    while True:
        message = the_connection.recv_match(blocking=True)
        message = message.to_dict()

        #验证是否为MISSION_REQUEST
        if message["mavpackettype"] == mavutil.mavlink.MAVLink_mission_request_message.msgname:

            #验证是否为mission items类型
            if message["mission_type"] == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:
                seq = message["seq"]

                #发送航点信息
                send_mission(the_connection, wp, seq)

        elif message["mavpackettype"] == mavutil.mavlink.MAVLink_mission_ack_message.msgname:

            #若回传信息为任务被接受（mission_ack信息）
            if message["mission_type"] == mavutil.mavlink.MAV_MISSION_TYPE_MISSION and \
                    message["type"] == mavutil.mavlink.MAV_MISSION_ACCEPTED:

                print("Mission uploaded successfully")
                break


def clear_waypoint(the_connection):
    msg = mavutil.mavlink.MAVLink_mission_clear_all_message(the_connection.target_system,the_connection.target_component,
                                                            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    the_connection.mav.send(msg)


def mission_current(the_connection,wp):
    mission_msg = the_connection.recv_match(type="MISSION_CURRENT", blocking=True)
    print(mission_msg.seq)
    wp[mission_msg.seq].distance(the_connection)
    time.sleep(2)


#根据上传的两个坐标点，通过自动设置更多的坐标点，生成一个两坐标之间的直线航线
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
        wp_list.append(wp_new)
        i+=1

    wp_list.append(wp[1])
    return wp_list


#在两点之间形成近似圆弧航线（angle指定弧线的圆心角角度，direction取1为顺时针，取-1为逆时针，默认顺时针）
def wp_circle_course(wp, precision, angle, direction=1):
    lat_len = wp[1].lat - wp[0].lat
    lon_len = wp[1].lon - wp[0].lon
    alt_len = wp[1].alt - wp[0].alt

    #半圆因为角度计算关系不能直接用下面的方法，但我懒得加一个专门的半圆航线了
    if angle == 180:
        angle = 179.9

    angle = angle * math.pi / 180 #转为弧度制

    #注意使用弧度制
    theta_start = math.atan(lat_len/lon_len)
    if lon_len >= 0: #一四象限
        pass
    else: #二三象限
        theta_start += math.pi

    radius = math.sqrt(lat_len*lat_len + lon_len*lon_len*1.2) * 0.45 / math.sin(angle*0.5)#非常粗略
    theta_step = angle / precision
    midpoint = Waypoint(wp[0].lat+0.5*lat_len, wp[0].lon+0.5*lon_len, wp[0].alt+0.5*alt_len)
    if direction >= 0:
        center = Waypoint(wp[0].lat+radius*math.sin(0.5*math.pi+theta_start-0.5*angle),
                          wp[0].lon+radius*math.cos(0.5*math.pi+theta_start-0.5*angle),
                          wp[0].alt+0.5*alt_len)

    else:
        center = Waypoint(wp[0].lat+radius*math.sin(-0.5*math.pi+theta_start+0.5*angle),
                          wp[0].lon+radius*math.cos(-0.5*math.pi+theta_start+0.5*angle),
                          wp[0].alt+0.5*alt_len)

    #print(center.lat, center.lon)
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
        wp_list.append(wp_new)
        i += 1

    wp_list.append(wp[1])
    return wp_list


#自动生成投弹航线并执行，采用反向飞离然后一字掉头后直线进场的方式，参数可决定转弯半径、转弯方向（顺或逆）
def execute_bomb_course(the_connection, home_position, wp_now, wp_target, precision, radius, line_course, direction):
    lat_len = wp_now.lat - wp_target.lat
    lon_len = wp_now.lon - wp_target.lon
    theta = math.atan(lat_len / lon_len)

    #暂时想不到根据距离推算经纬度关系的方法，姑且用0.001度约为一百米的方法估测
    d_lat = 2 * radius * math.sin(theta) * 1e-5
    d_lon = 2 * (radius / 1.1) * math.cos(theta) * 1e-5
    s_lat = line_course * math.sin(theta) * 1e-5
    s_lon = line_course * math.cos(theta) * 1e-5

    wp_near = Waypoint(wp_now.lat+s_lat, wp_now.lon+s_lon, wp_now.alt)
    wp_far = Waypoint(wp_now.lat++s_lat+d_lat, wp_now.lon++s_lon+d_lon, wp_now.alt)
    wp_line1 = [wp_now, wp_near]
    wp_circle1 = [wp_near, wp_far]
    wp_circle2 = [wp_far, wp_near]
    wp_line2 = [wp_near, wp_target]

    #直线开始进入掉头航线
    wp_line1_list = wp_straight_course(wp_line1, precision)

    #掉头部分航路点
    wp_circle_list = wp_circle_course(wp_circle1, precision, direction)
    wp_circle_list.extend(wp_circle_course(wp_circle2, precision, direction))

    #完成掉头进入直线投弹航线
    wp_line2_list = wp_straight_course(wp_line2, precision)

    wp_bomb_drop = wp_line1_list
    wp_bomb_drop.extend(wp_circle_list)
    wp_bomb_drop.extend(wp_line2_list)

    #逐个上传任务
    if upload_mission_till_completed(the_connection, wp_bomb_drop, home_position) == 0:
        print("bombs away!")


#上传航点集并阻塞程序直到完成全部预定航点任务
def upload_mission_till_completed(the_connection, wp, home_position):
    mission_upload(the_connection, wp, home_position)

    if (mode_set(the_connection, 10) < -1):
        sys.exit(1)

    wp_list_len = gain_mission(the_connection)
    while(waypoint_reached(the_connection) < wp_list_len):
        pass
    return 0


def loiter_at_present(the_connection, alt):
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_NAV_LOITER_UNLIM, 0, 0, 0, 30, 0, 0, 0, alt)
    msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    result = msg.result
    if result == 0:
        return 0
    else:
        print("loiter failed")
        return -10