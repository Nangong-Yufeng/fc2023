from pymavlink import mavutil
from .class_list import Position_relative, posture_inform, track_point, speed_inform
from .error_process import rec_match_received
import time


def gain_posture_para(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                         the_connection.target_component,
                                         512,  # command
                                         0,  # confirmation
                                         30,# param1
                                         0,  # param2
                                         0,  # param3
                                         0,  # param4
                                         0,  # param5
                                         0,  # param6
                                         0)  # param7
    msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
    #msg = rec_match_received(the_connection, 'ATTITUDE')
    pose = posture_inform(msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw,
                          msg.rollspeed, msg.pitchspeed, msg.yawspeed)
    return pose


def gain_ground_speed(the_connection):
    msg = rec_match_received(the_connection, 'GLOBAL_POSITION_INT')
    speed = speed_inform(msg.vx, msg.vy, msg.vz, msg.hdg)
    return speed


# 不使用track_point类中姿态的部分
def gain_position_now(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                               the_connection.target_component,
                                               512,  # command
                                               0,  # confirmation
                                               33,  # param1
                                               0,  # param2
                                               0,  # param3
                                               0,  # param4
                                               0,  # param5
                                               0,  # param6
                                               0)  # param7
    msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    #msg = rec_match_received(the_connection, 'GLOBAL_POSITION_INT')
    wp_now = track_point(msg.lat*1e-7, msg.lon*1e-7, msg.relative_alt*1e-3, msg.time_boot_ms, 0, 0, 0)
    return wp_now


# 大概是弃用了
def waypoint_reached(the_connection):

    msg = the_connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
    print("reaching waypoint", msg.seq)
    return msg.seq


# 获取并打印任务航点列表
def gain_mission(vehicle):
    message = mavutil.mavlink.MAVLink_mission_request_list_message(target_system=vehicle.target_system,
                                                           target_component=vehicle.target_component,
                                                           mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    vehicle.mav.send(message)

    # message = rec_match_received(vehicle, mavutil.mavlink.MAVLink_mission_count_message.msgname)
    message = vehicle.recv_match(type=mavutil.mavlink.MAVLink_mission_count_message.msgname,blocking=True)
    message = message.to_dict()

    count = message["count"]
    print("Total mission item count:", count-1)

    # create mission item list
    mission_item_list = []

    # get the mission items
    for i in range(count):
        message = mavutil.mavlink.MAVLink_mission_request_int_message(target_system=vehicle.target_system,
                                                              target_component=vehicle.target_component,
                                                              seq=i,
                                                              mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        vehicle.mav.send(message)

        message = rec_match_received(vehicle, mavutil.mavlink.MAVLink_mission_item_int_message.msgname)
        message = message.to_dict()
        mission_item_list.append(message)
    '''for mission_item in mission_item_list:
        print("Seq", mission_item["seq"],"Latitude", mission_item["x"] * 1e-7,"Longitude", mission_item["y"] * 1e-7,"Altitude", mission_item["z"])
    '''
    return count-1


def mission_current(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                         the_connection.target_component,
                                         512,  # command
                                         0,  # confirmation
                                         42,  # param1
                                         0,  # param2
                                         0,  # param3
                                         0,  # param4
                                         0,  # param5
                                         0,  # param6
                                         0)  # param7
    mission_msg = rec_match_received(the_connection, "MISSION_CURRENT")
    return mission_msg.seq


# 记录一段时间内的航点信息 track_list是由track_point对象组成的list
def gain_track_of_time(the_connection, track_list, time_last=50):
    position = gain_position_now(the_connection)
    posture = gain_posture_para(the_connection)

    track = track_point(position.lat, position.lon, position.alt, position.time, posture.roll, posture.pitch, posture.yaw)
    track_list.append(track)

    # 默认保存过去500个位置信息，往前的消息删除
    if len(track_list) > time_last:
        track_list.pop(0)
    else:
        pass
    return [position.time, position.alt]
    #print("random track point ", len(track_list), ":", track_list[num].lat, track_list[num].lon, track_list[num].alt, track_list[num].time)


# 获取坐标和姿态信息，用于目标位置解算
def gain_track_point(the_connection):
    position = gain_position_now(the_connection)
    posture = gain_posture_para(the_connection)

    track = track_point(position.lat, position.lon, position.alt, position.time, posture.roll, posture.pitch,
                        posture.yaw)
    return track


def gain_transform_frequency(the_connection):
    # 数传频率测试
    time_list = []
    count = 0
    fre = 0
    while count < 10:
        # msg = gain_position_now(the_connection)
        # msg = gain_posture_para(the_connection)
        print(mission_current(the_connection))
        # gain_track_point(the_connection)
        # print(int(time.time()*1000))
        if len(time_list) <= 50:
            time_list.append(int(time.time()*1000))
        else:
            frequency = 50 / (time_list[50] - time_list[0]) * 1000
            print("frequency: ", frequency, "\n")
            time_list = []
            fre = (frequency + fre * count) / (count+1)
            count += 1
    return fre


def gain_heading(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                         the_connection.target_component,
                                         512,  # command
                                         0,  # confirmation
                                         33,  # param1
                                         0,  # param2
                                         0,  # param3
                                         0,  # param4
                                         0,  # param5
                                         0,  # param6
                                         0)  # param7
    msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    return msg.hdg
