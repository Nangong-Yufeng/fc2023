# 故障处理函数
import time
import sys


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
