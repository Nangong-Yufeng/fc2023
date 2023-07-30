import time

from pymavlink import mavutil
from class_list import Position_relative, posture_inform
from error_process import rec_match_received


def gain_posture_para(the_connection):
    msg = rec_match_received(the_connection, 'ATTITUDE')
    pose = posture_inform(msg.time_boot_ms, msg.roll, msg.pitch, msg.yaw,
                          msg.rollspeed, msg.pitchspeed, msg.yawspeed)
    return pose

def position_now(the_connection):
    msg = rec_match_received(the_connection, 'GLOBAL_POSITION_INT')
    wp_now = Position_relative(msg.lat*1e-7, msg.lon*1e-7, msg.relative_alt*1e-3)
    return wp_now

def waypoint_reached(the_connection, wp, wp_list_len):

    seq = mission_current(the_connection)
    print("seq ",seq)
    print("wp_len ", wp_list_len)
    if(seq <= wp_list_len):
        msg = the_connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
        return msg.seq
    else:
        distance_old = distance_new = 1000000
        count = 0
        while count < 10:
          distance_old = distance_new
          distance_new = wp[wp_list_len].distance(the_connection)

          if(distance_old < distance_new):
              count+=1
              time.sleep(0.1)
          else:
              continue
        return wp_list_len





#获取并打印任务航点列表
def gain_mission(vehicle):
    message = mavutil.mavlink.MAVLink_mission_request_list_message(target_system=vehicle.target_system,
                                                           target_component=vehicle.target_component,
                                                           mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    vehicle.mav.send(message)

    message = rec_match_received(vehicle, mavutil.mavlink.MAVLink_mission_count_message.msgname)
    #message = vehicle.recv_match(type=mavutil.mavlink.MAVLink_mission_count_message.msgname,blocking=True)
    message = message.to_dict()

    count = message["count"]
    print("Total mission item count:", count)

    # create mission item list
    mission_item_list = []

    # get the mission items
    for i in range(count):
        message = mavutil.mavlink.MAVLink_mission_request_int_message(target_system=vehicle.target_system,
                                                              target_component=vehicle.target_component,
                                                              seq=i,
                                                              mission_type=mavutil.mavlink.MAV_MISSION_TYPE_MISSION)

        vehicle.mav.send(message)

        message = vehicle.recv_match(type=mavutil.mavlink.MAVLink_mission_item_int_message.msgname,blocking=True)
        message = message.to_dict()
        mission_item_list.append(message)

    #for mission_item in mission_item_list:
        #print("Seq", mission_item["seq"],"Latitude", mission_item["x"] * 1e-7,"Longitude", mission_item["y"] * 1e-7,"Altitude", mission_item["z"])

    return count-1

def mission_current(the_connection):
    mission_msg = rec_match_received(the_connection, "MISSION_CURRENT")
    return mission_msg.seq

