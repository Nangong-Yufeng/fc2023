import time
import sys
from pymavlink import mavutil


# 有时由于信号干扰中断，脚本可能收不到需要的mavlink msg或者其反馈，需要进行错误处理
def is_none_return(msg):
    if msg is not None:
        return False
    else:
        return True


# 系统部分检查，消息接收不设置time out
def system_check(the_connection):
    system_ok = False
    while not system_ok:
        time.sleep(2)
        the_connection.mav.command_long_send(the_connection.target_system,  # target_system
                                             the_connection.target_component,
                                             mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,  # command
                                             0,  # confirmation
                                             235,  # param1
                                             0,  # param2
                                             0,  # param3
                                             0,  # param4
                                             0,  # param5
                                             0,  # param6
                                             0)  # param7
        msg = the_connection.recv_match(type="HIGH_LATENCY2", blocking=True, timeout=5)
        if msg is None:
            print("未收到消息，等待初始化")
            continue
        failure_flags = msg.failure_flags

        if mavutil.mavlink.HL_FAILURE_FLAG_3D_GYRO & failure_flags != 0:
            print("陀螺仪不正常，重启飞机")
            reboot(the_connection)
        if mavutil.mavlink.HL_FAILURE_FLAG_3D_ACCEL & failure_flags != 0:
            print("加速度计不正常")
        if mavutil.mavlink.HL_FAILURE_FLAG_GPS & failure_flags != 0:
            print("GPS信号不正常")
        if mavutil.mavlink.HL_FAILURE_FLAG_3D_MAG & failure_flags != 0:
            print("磁场信号不正常")
        if mavutil.mavlink.HL_FAILURE_FLAG_ESTIMATOR & failure_flags != 0:
            print("初始化未完成")
        if failure_flags & 1 + failure_flags & 8 + failure_flags & 16 + failure_flags & 32 + failure_flags & 4096 == 0:
            system_ok = True



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


# 正常解锁飞机，指令反馈设置time out
def arm(the_connection):
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
   if msg is None:
       print("no message receive, failed to set mode")
       return False
   result = msg.result

   if result == 0:
      print("arm successfully")
      return True
   else:
      print("arm failed")
      return False

'''
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
            print("skip arm")
            return -1
'''


# 强制解锁程序，设置指令反馈等待time out
def force_arm(the_connection):
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
    msg = the_connection.recv_match(type="COMMAND_ACK", blocking=True, timeout=5)
    if msg is None:
        print("no message receive, failed force arm")
        return False
    result = msg.result

    if result == 0:
        print("force arm successfully")
        return True
    else:
        print("force arm failed")
        return False


# 模式设置，设置指令等待time out
def mode_set(the_connection, mode):
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
    if msg is None:
        print("no message receive, failed to set mode")
        return False
    result = msg.result

    # 根据com_ack判断模式设置结果
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
        elif mode == 12:
            print("set mode LOITER successfully")
        elif mode == 13:
            print("set mode TAKEOFF successfully")
        elif mode == 15:
            print("set mode GUIDED successfully")
        elif mode == 19:
            print("set mode QLOITER successfully")
        else:
            print("other mode set successfully")
        return True
    else:
        print("failed to set mode")
        return False


# 设置home点，指令等待time out
def set_home(the_connection, position_re, mode=0):
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
    if msg is None:
        print("no message receive, failed to set home")
        return False
    result = msg.result

    # mode为1表示使用当前位置为home点，mode为0则为指定位置
    if result == 0 and mode == 1:
        print("set current location as home successfully")
        return True
    elif result == 0 and mode == 0:
        print("successfully set home as lat = %s, lon = %s, re_alt = %s meters"
              % (position_re.lat, position_re.lon, position_re.alt))
        return True
    else:
        print("home set failed")
        return False


# 重启飞机，设置time out
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
    time.sleep(5)
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
    if msg is None:
        print("no message receive, failed to set home")
        return False
    if msg.result == 0:
        print("reboot successfully")
        return True
    else:
        print("reboot failed")
        return False


# 任务前所有指令综合
def preflight_command(the_connection, wp_home):
    system_check(the_connection)

    # 设置home点
    while input("输入0以设置home点：") == '0' and not set_home(the_connection, wp_home):
        continue

    # 设置模式为手动
    while input("输入0以设置模式为手动：") == '0' and not mode_set(the_connection, 0):
        continue

    # 解锁
    while True:
        choice = input("输入0解锁，输入1强制解锁，输入其他跳过解锁：")
        if choice == '0' and arm(the_connection):
            break
        elif choice == '1' and force_arm(the_connection):
            break
        else:
            break
