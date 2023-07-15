from pymavlink import mavutil

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

    #print(the_connection.recv_match(type="COMMAND_ACK", blocking=True))
