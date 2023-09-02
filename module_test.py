from navigation import gain_position_now, bomb_drop, Waypoint, set_home, mode_set, gain_ground_speed, gain_posture_para, gain_track_of_time, delay_eliminate
from pymavlink import mavutil
import time

the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

mode_set(the_connection, 0)
'''
while not input("enter"):
    position = gain_position_now(the_connection)
    posture = gain_posture_para(the_connection)
    speed_list = []
    vx = vy = vz = direction = 0
    alt = 0

    # 由于瞬时读取的速度和高度值非常奇怪，使用连续读取几次的方式考察情况是否有改善
    for count in range(0, 3):
        speed = gain_ground_speed(the_connection)
        speed_list.append(speed)
        vx += speed_list[count].vx
        vy += speed_list[count].vy
        vz += speed_list[count].vz
        direction += speed_list[count].direction
        alt += gain_position_now(the_connection).alt
        print("vx single: ", speed_list[count].vx)
        print("altitude single: ", gain_position_now(the_connection).alt)
    length = len(speed_list)
    vx /= length
    vy /= length
    vz /= length
    direction /= length
    alt /= length

    print("原始数据： ")
    print("位置： lat ", position.lat, " lon ", position.lon, " alt ", position.alt)
    print("速度:  north ", vx, 'east', vy, " down ", vz)
    print("姿态： roll ", posture.roll, " pitch ", posture.pitch, " yaw ", posture.yaw)
    print("方向角: direction ", direction)

    # 将时间和投弹位资信息记录到文件中
    localtime = time.localtime(time.time())
    time_data = str(localtime.tm_year) + '.' + str(localtime.tm_mon) + '.' + str(localtime.tm_mday) + ' ' + str(
        localtime.tm_hour) + ':' + str(localtime.tm_min) + ':' + str(localtime.tm_sec)
    with open(file='/home/bobo/fc2023/test_data.txt', mode='a') as f:
        f.write(time_data)
        f.write('\n')
        f.write("位置： lat " + str(position.lat) + " lon " + str(position.lat) + " alt " + str(position.alt))
        f.write('\n')
        f.write("速度:  north " + str(vx) + " east " + str(vy) + " down " + str(vz))
        f.write('\n')
        f.write("姿态： roll " + str(posture.roll) + " pitch " + str(posture.pitch) + " yaw " + str(posture.yaw))
        f.write('\n')
        f.write("方向（可用性未知） direction " + str(direction))
        f.write("\n")
'''
track_list = []

while True:
    time_data = gain_track_of_time(the_connection, track_list)
    track = delay_eliminate(track_list, time_data)
    if track != None:
        print("now:", time_data)
        print("trace:", track.time)
        print("delay: ", time_data-track.time, "\n")
