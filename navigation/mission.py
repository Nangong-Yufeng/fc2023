from pymavlink import mavutil
from math import fabs

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

def mission_upload(the_connection, wp):

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
    print(mission_msg)

#根据上传的两个坐标点，通过自动设置更多的坐标点，生成一个两坐标之间的直线航线
def cruse_wp_planning(vehicle, wp, num):
    lat_len = fabs(wp[0].lat - wp[1].lat)
    lon_len = fabs(wp[0].lon - wp[1].lon)

    pass