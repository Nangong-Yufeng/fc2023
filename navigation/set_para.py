import time
import sys
from pymavlink import mavutil

#设置空速大小
def set_ground_speed(vehicle, desired_flight_speed):
    message = mavutil.mavlink.MAVLink_command_long_message(target_system=vehicle.target_system,
                                                   target_component=vehicle.target_component,
                                                   command=mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,
                                                   confirmation=0,
                                                   param1=1,
                                                   param2=desired_flight_speed,
                                                   param3=0,
                                                   param4=0,
                                                   param5=0,
                                                   param6=0,
                                                   param7=0)

    vehicle.mav.send(message)
    msg = vehicle.recv_match(type="COMMAND_ACK", blocking=True)
    if msg.result == 0:
        print("successfully set ground speed as ", (desired_flight_speed), "m/s")
        return 0
    else:
        print("ground speed set failed")
        return -10
