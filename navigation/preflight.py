from pymavlink import mavutil

def arm(the_connection):
    the_connection.wait_heartbeat()

    # 使用long信息发送arm的command
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    # 读取返回的com_acknowledge消息
    msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    result = msg.result
    if result == 0:
        print("arm successfully")
        return 0
    else:
        print("arm failed")
        return -10

def mode_set(the_connection, mode):

    #使用long信息发送模式设置指令
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    result = msg.result

    #根据com_ack判断模式设置结果
    if result == 0:
        if mode == 0:
            print("set mode MANUAL successfully")
        elif mode == 2:
            print("set mode STABILIZE successfully")
        elif mode == 5:
            print("set mode FBWA successfully")
        elif mode == 6:
            print("set mode FBWB successfully")
        elif mode == 7:
            print("set mode CRUISE successfully")
        elif mode == 10:
            print("set mode AUTO successfully")
        elif mode == 11:
            print("set mode RTL successfully")
        elif mode == 13:
            print("set mode TAKEOFF successfully")
        elif mode == 15:
            print("set mode GUIDED successfully")
        elif mode == 19:
            print("set mode QLOITER successfully")
        else:
            print("other mode set successfully")
        return mode
    else:
        print("mode_set failed")
        return -10


def set_home(the_connection, mode, position_re):

    #设置home点
    the_connection.mav.command_int_send(the_connection.target_system, the_connection.target_component, 3,
                                         mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, mode, 0, 0, 0, int(position_re.lat * 1e7), int(position_re.lon * 1e7), position_re.alt)
    msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)

    #mode为1表示使用当前位置为home点，mode为0则为指定位置
    if msg.result == 0 and mode == 1:
        print("set current location as home successfully")
        return 0
    elif msg.result == 0 and mode == 0:
        print("successfully set home as lat = %s, lon = %s, re_alt = %s meters" %(position_re.lat, position_re.lon, position_re.alt))
        return 0
    else:
        print("home set failed")
        return -10

