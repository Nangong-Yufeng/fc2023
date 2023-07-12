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
