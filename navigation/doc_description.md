# SITL_test.py
   用于在模拟环境中测试时使用的主函数

# flight_test.py
   用于在实际飞行中测试的主函数

# preflight.py
   定义了飞行前动作的函数原型
   
   ### arm函数：
   解锁飞机，打印并返回是否成功arm
   ### mode_set函数：
   设置飞机模式，打印成功设置为何种模式，并返回成功判断
   ### set_home函数：
   设置飞机home点，mode参数为1时表示使用当前位置为home点（不管position是多少）；mode为0时则将指定位置设为home点

# class_list.py
   定义了一些类的原型，便于参数传递

   ### Position_relative类：
   包含经纬度和相对高度三个量

   ### Waypoint类
   继承自Position_relative，增添了测量到该点距离的函数

# mission.py
   定义了与航点任务相关的函数

   ### send_mission_list(the_connection, wp)
   将任务列表上传到飞机，一般不需要在操作中单独调用

   ### send_mission(the_connection, wp, seq)
   将任务发送到飞机，一般也不需要单独调用

   ### mission_upload(the_connection, wp, home_position)
   上传任务，wp为使用python list方式记录的航点集，还要上传home点位置
   
  #### ps：上传home点是upload时不知为何第一个航点会被忽略，因此将第一个航点自动设置为home点，就算突然跑了第一个点一般也不会有什么大问题

   ### clear_waypoint(the_connection)
   清除已有的航点信息（待完善，似乎不太有用）

   ### mission_current(the_connection,wp)
   读取当前执行任务的信息，待完善

   ### wp_straight_course(wp, precision)
   传入一个只有两个航点的航点集，返回一个走直线的航点集，航点数量由precision决定 

   ### wp_circle_course(wp, precision, angle, direction=1)
   传入一个只有两个航点的航点集，返回一个走圆弧轨迹的航点集，precision决定航点数量，angle决定弧线的圆心角（支持大于180度的角），direction决定方向（1为顺时针，-1为逆时针，默认为顺时针）

   ### execute_bomb_course(the_connection, home_position, wp_now, wp_target, precision, radius, line_course, direction)
   根据当前位置和获取的靶标位置，直接自主执行飞离、掉头、投弹的过程。   

   ### upload_mission_till_completed(the_connection, wp, home_position):
   上传航点任务并执行，同时会阻塞至到达最后一个航点才继续其他代码

   ### loiter_at_present(the_connection, alt):
   在当前位置盘旋，直到模式切换或受到新的任务

# set_para.py
  设置一些参数
   
  ### set_speed
  设置空速，待完善

# get_para.py
  获取飞机信息
  
  ### gain_mission
  获取并打印航点列表，待完善