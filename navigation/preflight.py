import time
import sys
from pymavlink import mavutil


# 有时由于信号干扰中断，脚本可能收不到需要的mavlink msg或者其反馈，需要进行错误处理
def is_none_return(msg):
    if msg is not None:
        return False
    else:
        return True


# 针对mav_recv_match做一个问题处理，如果没有收到消息，则等待一段时间后重复至一定次数
def rec_match_received(the_connection, type, times=5):
    count = 0
    while True:
        msg = the_connection.recv_match(type=type, blocking=True, timeout=5)

        if not is_none_return(msg):
            return msg
        elif count < times:
            time.sleep(2)
            count += 1
            print("None message match ", type, "retry No.", count)
            continue
        else:
            print("system error")
            sys.exit(1)

def arm(the_connection, times=5):
    the_connection.wait_heartbeat()

    if input("输入0以解锁飞机（若飞机在飞行过程中，输入其他任意内容跳过解锁)： ") == '0':
       print("arming")

       count = 0
       while True:
          the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
          the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                               the_connection.target_component,
                                               mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                               0,  # confirmation
                                               77,  # param1
                                               0,  # param2
                                               0,  # param3
                                               0,  # param4
                                               0,  # param5
                                               0,  # param6
                                               0)  # param7
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
            return -1

       if result == 0:
          print("arm successfully")
          return 1
       else:
          print("arm failed")
          the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                               the_connection.target_component,
                                               mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                               0,  # confirmation
                                               24,  # param1
                                               0,  # param2
                                               0,  # param3
                                               0,  # param4
                                               0,  # param5
                                               0,  # param6
                                               0)  # param7
          msg = rec_match_received(the_connection, "GPS_RAW_INT")
          if msg.fix_type == 3 or msg.fix_type == 4:
            if input('解锁失败，但GPS连接正常，输入0以强制解锁: ') == '0':
                pass
            else:
                print("arm failed")
                return -1
            return force_arm(the_connection)
          elif input("GPS未连接，若仍要强制解锁，输入0: ") == '0':
            return force_arm(the_connection)
          else:
            return -1
    else:
        print("skip arm")
        return 1


def force_arm(the_connection, times=5):
    count = 0
    while True:
        the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 2989, 0, 0, 0, 0, 0)
        the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                             the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                             0,  # confirmation
                                             77,  # param1
                                             0,  # param2
                                             0,  # param3
                                             0,  # param4
                                             0,  # param5
                                             0,  # param6
                                             0)  # param7
        msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)

        if is_none_return(msg) == False:
            result = msg.result
            break
        elif count < times:
            time.sleep(0.5)
            count += 1
            print("receive None msg, retry No.", count)
            result = -1
            continue
        else:
            return -1

    if result == 0:
        print("force arm successfully")
        return 1
    else:
        return -1


def mode_set(the_connection, mode, times=5):

    count = 0
    # 使用long信息发送模式设置指令,进行信息判断(times可以指定重试的次数）
    while True:
       the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, mode, 0, 0, 0, 0, 0)
       the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                            the_connection.target_component,
                                            mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                            0,  # confirmation
                                            77,  # param1
                                            0,  # param2
                                            0,  # param3
                                            0,  # param4
                                            0,  # param5
                                            0,  # param6
                                            0)  # param7
       msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)

       if not is_none_return(msg):
          result = msg.result
          break
       elif count < times:
          time.sleep(1)
          count += 1
          print("receive None msg, retry No.", count)
          result = -1
          continue
       else:
          return -1

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
        print("failed to set mode")
        return -1


def set_home(the_connection, position_re, times=5, mode=0):

    #设置home点
    count = 0
    while True:
        the_connection.mav.command_int_send(the_connection.target_system, the_connection.target_component, 3,
                                            mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0, 0, mode, 0, 0, 0,
                                            int(position_re.lat * 1e7), int(position_re.lon * 1e7), position_re.alt)
        the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                             the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                             0,  # confirmation
                                             77,  # param1
                                             0,  # param2
                                             0,  # param3
                                             0,  # param4
                                             0,  # param5
                                             0,  # param6
                                             0)  # param7
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
            return -1

    #mode为1表示使用当前位置为home点，mode为0则为指定位置
    if result == 0 and mode == 1:
        print("set current location as home successfully")
        return 1
    elif result == 0 and mode == 0:
        print("successfully set home as lat = %s, lon = %s, re_alt = %s meters" %(position_re.lat, position_re.lon, position_re.alt))
        return 1
    else:
        print("home set failed")
        return -1


def arm_check(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                the_connection.target_component,
                mavutil.mavlink.MAV_CMD_RUN_PREARM_CHECKS, # command
                0, # confirmation
                0, # param1
                0, # param2
                0, # param3
                0, # param4
                0, # param5
                0, # param6
                0) # param7
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                         0,  # confirmation
                                         1,  # param1
                                         0,  # param2
                                         0,  # param3
                                         0,  # param4
                                         0,  # param5
                                         0,  # param6
                                         0)  # param7
    msg = the_connection.recv_match(type="SYS_STATUS", blocking=True)
    print(msg)


def reboot(the_connection):
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # command
                                         0,  # confirmation
                                         1,  # param1
                                         0,  # param2
                                         0,  # param3
                                         0,  # param4
                                         0,  # param5
                                         0,  # param6
                                         0)  # param7
    the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                         the_connection.target_component,
                                         mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                         0,  # confirmation
                                         77,  # param1
                                         0,  # param2
                                         0,  # param3
                                         0,  # param4
                                         0,  # param5
                                         0,  # param6
                                         0)  # param7
    msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True)
    if msg.result == 0:
        print("reboot successfully")
    else:
        print("reboot failed")
