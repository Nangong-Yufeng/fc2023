# 故障处理函数
import time
import sys
from .preflight import arm, mode_set, set_home, reboot


def error_process(the_connection):
    print("system error")
    sys.exit(1)


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
            error_process(the_connection)


# 弃用（大概）
def retry_fuc_para1(the_connection, function, para):
    msg = function(the_connection, para)
    if msg > -1:
        pass
    elif msg == -2:
        function(the_connection, para)
    else:
        error_process(the_connection)


# 对指令进行重复进行
def command_retry(the_connection, command, para1=0):
    if command == 'arm':
        result = arm(the_connection)
        while result < 0:
            print("__retry arm command__")
            result = arm(the_connection)
            msg = input("输入任意内容重试；输入1取消重试并结束程序: ")
            if msg != 1:
                continue
            else:
                error_process(the_connection)
    elif command == 'mode_set':
        result = mode_set(the_connection, para1)
        while result < 0:
            print("__retry mode_set command__")
            result = mode_set(the_connection, para1)
    elif command == 'set_home':
        result = set_home(the_connection, para1)
        while result < 0:
            print("__retry mode_set command__")
            result = set_home(the_connection, para1)
    elif command == 'reboot':
        result = reboot(the_connection)
        while result < 0:
            print("__retry mode_set command__")
            result = reboot(the_connection)
    else:
        print("ERROR! command not found!")


