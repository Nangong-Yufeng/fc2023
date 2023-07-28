from pymavlink import mavutil
from class_list import Position_relative
from error_process import rec_match_received

def gain_posture_para(the_connection):
    msg = the_connection.recv_match(type='ATTITUDE', blocking=True)
    #msg = the_connection.recv_match(type='SIMSTATE', blocking=True)
    print(msg)

def position_now(the_connection):
    msg = rec_match_received(the_connection, 'GLOBAL_POSITION_INT')
    #msg = the_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    wp_now = Position_relative(msg.lat*1e-7, msg.lon*1e-7, msg.relative_alt*1e-3)
    return wp_now

def waypoint_reached(the_connection):
    msg = the_connection.recv_match(type='MISSION_ITEM_REACHED', blocking=True)
    return msg.seq


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