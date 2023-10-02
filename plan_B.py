from pymavlink import mavutil
from utils import title
from navigation import (Waypoint, mission_upload,
                        wp_bombing_course, mission_current, bomb_drop,
                        preflight_command,
                        wp_circle_course, wp_straight_course)
from math import *

'''                                         
坐标点集
'''
# 按照绕圈顺序排除1-4号靶标
A_target1 = Waypoint(38.557288, 115.139136, 0)
A_target2 = Waypoint(38.557168, 115.138972, 0)
A_target3 = Waypoint(38.557443, 115.139024, 0)
A_target4 = Waypoint(38.557302, 115.138819, 0)

B_target1 = Waypoint(38.559180, 115.142050, 0)
B_target2 = Waypoint(38.559315, 115.142202, 0)
B_target3 = Waypoint(38.559314, 115.141904, 0)
B_target4 = Waypoint(38.559445, 115.142059, 0)

T_target1 = Waypoint(38.543141, 115.041150, 0)
T_target3 = Waypoint(38.543231, 115.041392, 0)
T_target2 = Waypoint(38.542958, 115.041341, 0)
T_target4 = Waypoint(38.543087, 115.041528, 0)

wp_home = Waypoint(38.543087, 115.041528, 0)

'''                                         
参数表
'''
# 投票选择朝哪个靶标投弹
TARGET_CHOOSE_A = 0
TARGET_CHOOSE_B = 0
TARGET_CHOOSE_T = 0
# 侦察进近方向，指南针标准
DETECT_APPROACH_A = 231
DETECT_APPROACH_B = 51
DETECT_APPROACH_T = 155

# 投弹进近航向，北向起点逆时针标准
BOMB_APPROACH_A = 309
BOMB_APPROACH_B = 129
BOMB_APPROACH_T = 25
# 盘旋直径
Diameter = 0.0007
# 飞掠靶标区和绕圈处的高度设置
Alt_detect = 15
Alt_circle = 40
# 在靶标坐标前后拓展的距离
Length_expend = 0.00015
# 侦察旋转方向（投弹相反）
Direction_A = -1
Direction_B = 1
Direction_T = 1


# detect_angle为进入侦察区的进场角度，按照指南针标准
def course(wp_rectangle, detect_angle, target_choice, bomb_approach,
           diameter=Diameter, alt_detect=Alt_detect, alt_circle=Alt_circle, length_expend=Length_expend, direction=-1):
    # 转换为象限角
    angle = pi * ((360 - detect_angle) + 90) / 180
    right_angle = 0.5 * pi

    # 起始处和转向点

    # 第一圈 内侧靶标
    wp_start1 = Waypoint(wp_rectangle[0].lat + length_expend * sin(angle + pi),
                         wp_rectangle[0].lon + length_expend * cos(angle + pi), alt_detect)
    wp_end1 = Waypoint(wp_rectangle[1].lat + length_expend * sin(angle),
                       wp_rectangle[1].lon + length_expend * cos(angle), alt_detect)
    wp_turn11 = Waypoint(wp_end1.lat + diameter * sin(angle + direction * right_angle),
                         wp_end1.lon + diameter * cos(angle + direction * right_angle), alt_circle)
    wp_turn12 = Waypoint(wp_start1.lat + diameter * sin(angle + direction * right_angle),
                         wp_start1.lon + diameter * cos(angle + direction * right_angle), alt_circle)

    # 第二圈 外侧靶标
    wp_start2 = Waypoint(wp_rectangle[2].lat + length_expend * sin(angle + pi),
                         wp_rectangle[2].lon + length_expend * cos(angle + pi), alt_detect)
    wp_end2 = Waypoint(wp_rectangle[3].lat + length_expend * sin(angle),
                       wp_rectangle[3].lon + length_expend * cos(angle), alt_detect)
    wp_turn21 = Waypoint(wp_end2.lat + diameter * sin(angle + direction * right_angle),
                         wp_end2.lon + diameter * cos(angle + direction * right_angle), alt_circle)
    wp_turn22 = Waypoint(wp_start2.lat + diameter * sin(angle + direction * right_angle),
                         wp_start2.lon + diameter * cos(angle + direction * right_angle), alt_circle)

    # 第三圈 中间掠过补充
    wp_start3 = Waypoint(0.5 * wp_start1.lat + 0.5 * wp_start2.lat,
                         0.5 * wp_start1.lon + 0.5 * wp_start2.lon, alt_detect)
    wp_end3 = Waypoint(0.5 * wp_end1.lat + 0.5 * wp_end2.lat,
                       0.5 * wp_end1.lon + 0.5 * wp_end2.lon, alt_detect)

    line11 = wp_straight_course([wp_start1, wp_end1], 3)
    circle11 = wp_circle_course([wp_end1, wp_turn11], 3, 180, direction=direction)
    line12 = [wp_turn11, wp_turn12]
    circle12 = wp_circle_course([wp_turn12, wp_start2], 3, 180, direction=direction)

    line21 = wp_straight_course([wp_start2, wp_end2], 3)
    circle21 = wp_circle_course([wp_end2, wp_turn21], 3, 180, direction=direction)
    line22 = [wp_turn21, wp_turn22]
    circle22 = wp_circle_course([wp_turn22, wp_start3], 3, 180, direction=direction)

    line31 = wp_straight_course([wp_start3, wp_end3], 3)

    detect_course = line11
    detect_course.pop(-1)
    detect_course.extend(circle11)
    detect_course.pop(-1)
    detect_course.extend(line12)
    detect_course.pop(-1)
    detect_course.extend(circle12)

    detect_course.extend(line21)
    detect_course.pop(-1)
    detect_course.extend(circle21)
    detect_course.pop(-1)
    detect_course.extend(line22)
    detect_course.pop(-1)
    detect_course.extend(circle22)

    detect_course.extend(line31)
    detect_course.pop(-1)

    # 注意投弹航线的旋转方向和侦察时相反
    if direction == -1:
        bomb_course = wp_bombing_course(wp_rectangle[target_choice], bomb_approach, 'anti_clock')
    else:
        bomb_course = wp_bombing_course(wp_rectangle[target_choice], bomb_approach, 'clock')

    mission_course = detect_course
    mission_course.extend(bomb_course)

    return mission_course


if __name__ == "__main__":
    '''                                         
    帅
    '''
    title.printTitle()

    '''                                         
    初始化
    '''
    # the_connection = mavutil.mavlink_connection('/COM3', baud=57600)
    the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)

    course_final = []

    choice = input("输入A或B选择侦察任务：")
    if choice == 'A' or choice == 'a':
        course_final = course([A_target1, A_target2, A_target3, A_target4],
                              detect_angle=DETECT_APPROACH_A, target_choice=TARGET_CHOOSE_A,
                              bomb_approach=BOMB_APPROACH_A, direction=Direction_A)
        print("上传A组航线信息")
        mission_upload(the_connection, course_final, wp_home)
    elif choice == 'B' or choice == 'b':
        course_final = course([B_target1, B_target2, B_target3, B_target4],
                              detect_angle=DETECT_APPROACH_B, target_choice=TARGET_CHOOSE_B,
                              bomb_approach=BOMB_APPROACH_B, direction=Direction_B)
        print("上传B组航线信息")
        mission_upload(the_connection, course_final, wp_home)
    elif choice == 'test' or choice == 'TEST':
        course_final = course([T_target1, T_target2, T_target3, T_target4],
                              detect_angle=DETECT_APPROACH_T, target_choice=TARGET_CHOOSE_T,
                              bomb_approach=BOMB_APPROACH_T, direction=Direction_T)
        print("上传测试航线信息")
        mission_upload(the_connection, course_final, wp_home)
    else:
        print("输入错误 可以跳过或建议remake")
        course_final = course([T_target1, T_target2, T_target3, T_target4],
                              detect_angle=DETECT_APPROACH_T, target_choice=TARGET_CHOOSE_T,
                              bomb_approach=BOMB_APPROACH_T, direction=Direction_T)

    print("进行飞行前初始化和检测")
    preflight_command(the_connection, wp_home)

    last_wp = 0
    while True:
        msg = mission_current(the_connection)
        if msg >= len(course_final) - 14:
            print("到达投弹点", msg)
            break
        if msg != last_wp:
            print("到达航路点 ", msg)
            last_wp = msg
            continue

    bomb_drop(the_connection)
