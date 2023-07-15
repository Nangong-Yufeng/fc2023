import time
import sys
from pymavlink import mavutil
from preflight import arm, mode_set, set_home
from mission import send_mission, send_mission_list
from class_list import Position_relative, Waypoint

#连接飞行器
the_connection = mavutil.mavlink_connection('udpin:localhost:14550')


if(arm(the_connection) < -1):
    sys.exit(1)
if(mode_set(the_connection, 5) < -1):
    sys.exit(1)
position = Position_relative(-353622066, 1491651135, 10)
if set_home(the_connection, 0, position) < -1:
    sys.exit(1)
wp0 = Waypoint(-35.3598036, 149.1647555, 30)
wp1 = Waypoint(-35.3600394, 149.1604871, 20)
wp2 = Waypoint(-35.3654404, 149.1611205, 50)
wp3 = Waypoint(-35.3654516, 149.1654714, 80)
wp = [wp0, wp1, wp2, wp3]
send_mission_list(the_connection, wp)

while True:

    # catch a message
    message = the_connection.recv_match(blocking=True)

    # convert this message to dictionary
    message = message.to_dict()

    # check this message is MISSION_REQUEST
    if message["mavpackettype"] == mavutil.mavlink.MAVLink_mission_request_message.msgname:

        # check this request is for mission items
        if message["mission_type"] == mavutil.mavlink.MAV_MISSION_TYPE_MISSION:

            seq = message["seq"]
            print(seq)
            send_mission(the_connection, wp, seq)

    elif message["mavpackettype"] == mavutil.mavlink.MAVLink_mission_ack_message.msgname:

        # check this acknowledgement is for mission and it is accepted
        if message["mission_type"] == mavutil.mavlink.MAV_MISSION_TYPE_MISSION and \
                message["type"] == mavutil.mavlink.MAV_MISSION_ACCEPTED:
            # break the loop since the upload is successful
            print("Mission uploaded successfully")
            break

if(mode_set(the_connection, 10) < -1):
    sys.exit(1)