#故障处理函数
import time

from pymavlink import mavutil
import sys

def error_process(the_connection):
    print("system error")
    sys.exit(1)

def timeout_process(the_connection, time, num):
    pass


#有时由于信号干扰中断，脚本可能收不到需要的mavlink msg或者其反馈，需要进行错误处理
def is_none_return(msg):
    if msg is not None:
        return False
    else:
        return True