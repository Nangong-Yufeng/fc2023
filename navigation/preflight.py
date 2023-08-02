import time
from pymavlink import mavutil
from error_process import error_process, is_none_return, rec_match_received

def arm(the_connection, times=5):
    the_connection.wait_heartbeat()

    count = 0
    while True:
      the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
      msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)

      if is_none_return(msg) == False:
            result = msg.result
            break
      elif count < times:
            time.sleep(2)
            count += 1
            print("receive None msg, retry No.", count)
            result = -1
            continue
      else:
            error_process(the_connection)



    # 使用long信息发送arm的command
    if result == 0:
        print("arm successfully")
    else:
        print("arm failed")
        msg = rec_match_received(the_connection, "GPS_RAW_INT")
        if msg.fix_type == 3 or msg.fix_type == 4:
            force_arm(the_connection)
        else:
            error_process(the_connection)




def mode_set(the_connection, mode, times=5):

    count = 0
    #使用long信息发送模式设置指令,进行信息判断(times可以指定重试的次数）
    while True:
      the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode, 0, 0, 0, 0, 0)
      msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)

      if is_none_return(msg) == False:
            result = msg.result
            break
      elif count < times:
            time.sleep(2)
            count += 1
            print("receive None msg, retry No.", count)
            result = -1
            continue
      else:
            error_process(the_connection)

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
        error_process(the_connection)


def set_home(the_connection, mode, position_re, times=5):

    #设置home点
    count = 0
    while True:
        the_connection.mav.command_int_send(the_connection.target_system, the_connection.target_component, 3,
                                            mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, mode, 0, 0, 0,
                                            int(position_re.lat * 1e7), int(position_re.lon * 1e7), position_re.alt)
        msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)

        if is_none_return(msg) == False:
            result = msg.result
            break
        elif count < times:
            time.sleep(2)
            count += 1
            print("receive None msg, retry No.", count)
            result = -1
            continue
        else:
            error_process(the_connection)

    #mode为1表示使用当前位置为home点，mode为0则为指定位置
    if msg.result == 0 and mode == 1:
        print("set current location as home successfully")
        return 0
    elif msg.result == 0 and mode == 0:
        print("successfully set home as lat = %s, lon = %s, re_alt = %s meters" %(position_re.lat, position_re.lon, position_re.alt))
    else:
        print("home set failed")
        error_process(the_connection)


def force_arm(the_connection, times=5):
    if input('解锁失败，但GPS连接正常，输入0以强制解锁 ') == '0':
        pass
    else:
        print("arm failed")
        error_process(the_connection)

    count = 0
    while True:
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 21196, 0, 0, 0, 0, 0)
        msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
        print(msg)

        if is_none_return(msg) == False:
            result = msg.result
            break
        elif count < times:
            time.sleep(2)
            count += 1
            print("receive None msg, retry No.", count)
            result = -1
            continue
        else:
            error_process(the_connection)

    if result == 0:
        print("force arm successfully")
    else:
        error_process(the_connection)
