import time
from pymavlink import mavutil
import math
from class_list import Waypoint

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


#在两点之间形成近似圆弧航线（direction取1为顺时针，取-1为逆时针，默认顺时针）
def wp_circle_course(wp, precision, direction=1):
    lat_len = wp[1].lat - wp[0].lat
    lon_len = wp[1].lon - wp[0].lon
    alt_len = wp[1].alt - wp[0].alt

    #注意使用弧度制
    theta_start = math.atan(lat_len/lon_len)
    if lon_len <= 0: #一四象限
        pass
    else: #二三象限
        theta_start += math.pi

    radius = math.sqrt(lat_len*lat_len + lon_len*lon_len*1.2) * 0.48 #非常粗略
    theta_step = math.pi / precision
    center = Waypoint(wp[0].lat+0.5*lat_len, wp[0].lon+0.5*lon_len, wp[0].alt+0.5*alt_len)

    wp_list = [wp[0]]
    for i in range(0, precision):
        theta = theta_start + direction * theta_step * (i+1)
        lat_new = center.lat + radius * math.sin(theta)
        lon_new = center.lon + radius * math.cos(theta)
        alt_new = wp[0].alt + alt_len / precision * (i+1)
        wp_new = Waypoint(lat_new, lon_new, alt_new)
        wp_list.append(wp_new)
        i += 1

    wp_list.append(wp[1])
    return wp_list

#生成一字航线型的掉头航线, 参数可决定转弯半径、转弯方向（顺或逆）
def wp_turn_course(wp_now, wp_target, precision, radius, direction):
    lat_len = wp_now.lat - wp_target.lat
    lon_len = wp_now.lon - wp_target.lon
    theta = math.atan(lat_len / lon_len)

    #暂时想不到根据距离推算经纬度关系的方法，姑且用0.001度约为一百米的方法估测
    d_lat = 2 * radius * math.sin(theta) * 1e-5
    d_lon = 2 * (radius / 1.1) * math.cos(theta) * 1e-5

    target = Waypoint(wp_now.lat+d_lat, wp_now.lon+d_lon, wp_now.alt)
    wp = [wp_now, wp_target]
    wp2 = [wp_target, wp_now]

    wp_list = wp_circle_course(wp, precision, direction)
    wp_list2 = wp_circle_course(wp2, precision, direction)

    wp_list.extend(wp_list2)
    return wp_list
