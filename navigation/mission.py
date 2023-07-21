import time

from pymavlink import mavutil
from math import fabs
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
def cruse_wp_planning(wp, num):

    lat_len = (wp[1].lat - wp[0].lat)
    lon_len = (wp[1].lon - wp[0].lon)
    alt_len = (wp[1].alt - wp[0].alt)
    lat_len /= num
    lon_len /= num
    alt_len /= num

    wp_list = [wp[0]]

    i = 0
    for i in range(0, num):
        wp_new = Waypoint(wp_list[i].lat+lat_len, wp_list[i].lon+lon_len, wp_list[i].alt+alt_len)
        wp_list.append(wp_new)
        i+=1

    wp_list.append(wp[1])
    return wp_list


