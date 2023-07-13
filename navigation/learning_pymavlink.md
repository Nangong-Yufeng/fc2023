本文档是pymavlink的一些学习笔记

推荐youtube上的视频教程入门（不过他是copter，会有点不一样）

    https://www.youtube.com/watch?v=kecnaxlUiTY&list=PLy9nLDKxDN68cwdt5EznyAul6R8mUSNou
    
关于MAV_CMD的一些接口，建议结合ardupilot的文档进行对比参考，可能会有不同

主体对象为the_connect，其是一个mavutil对象

关于mavutil的定义可以参考mavlink源代码

    https://github.com/ArduPilot/pymavlink/blob/e6ec7642d12c8c7b3c2b2ec38b38972314bdd895/mavutil.py#L505

# 信息获取 MAVLink Messages

例子

    print(the_connection.recv_match(type="HEARTBEAT", blocking=True, timeout = 5))

其中“HEARTBEAT”是一种MAVLink Messages，改为其他即可显示其他信息，目录参考

    https://mavlink.io/zh/messages/common.html#PROTOCOL_VERSION

其他参数blocking决定是否阻塞，timeout设定了阻塞时间等等，具体可参考编译器提示或翻阅源代码

## 遭遇问题

系统显示mavlink是2.4.39版本，但是仅mavlink2支持的各种信息都无法读取，原因不明！！！！
进一步测试，发现不止mavlink2，许多mavlink1支持的命令也不能读到，目前怀疑是ardupilot端并没有这些参数接口，建议阅读au文档
测试了多种消息获取的方式，结合ardupilot文档，基本认为是因为au端没有这些参数导致的错误

# 模式设置
对照表

0  Manual

1  CIRCLE

2  STABILIZE

3  TRAINING

4  ACRO

5

FBWA

6

FBWB

7

CRUISE

8

AUTOTUNE

10

Auto

11

RTL

12

Loiter

13

TAKEOFF

14

AVOID_ADSB

15

Guided

17

QSTABILIZE

18

QHOVER

19

QLOITER

20

QLAND

21

QRTL

22

QAUTOTUNE

23

QACRO

24

THERMAL

25

Loiter to QLand
