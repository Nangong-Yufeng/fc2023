import time
from navigation import (Waypoint, set_home, mode_set, arm, mission_upload,
                        wp_detect_course, loiter_at_present, gain_track_point,
                        detect_completed, eliminate_error_target, command_retry,
                        gain_position_now, set_ground_speed, target_transfer,
                        wrong_number, wp_bombing_course, mission_current, bomb_drop,
                        loiter, return_to_launch, initiate_bomb_drop, reboot, system_check,
                        preflight_command, wp_detect_course_HeBei)
from pymavlink import mavutil

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

print("start")

wp_home = Waypoint(22.8027619, 114.2959589, 0)

mission_upload(the_connection, wp_detect_course_HeBei(wp_home, 20), wp_home)

# preflight_command(the_connection, wp_home)

while True:
    msg = gain_position_now(the_connection)
    print(msg.alt)
