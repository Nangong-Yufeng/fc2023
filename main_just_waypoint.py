from pymavlink import mavutil
from utils import title
from navigation import (Waypoint, wp_bombing_insert_course,
                        wp_circle_course, wp_straight_course, mission_upload_including_bomb_drop)
from math import *

'''                                         
坐标点集
'''
# 按照绕圈顺序排除1-4号范围点
B_edge1 = Waypoint(28.593853, 113.186879, 0)
B_edge4 = Waypoint(28.593993, 113.186996, 0)
B_edge2 = Waypoint(28.594014, 113.186821, 0)
B_edge3 = Waypoint(28.5938623, 113.1870680, 0)
wp_range_B = [B_edge1, B_edge2, B_edge3, B_edge4]
B_target = [B_edge1, B_edge2, B_edge4]

A_edge3 = Waypoint(28.591356, 113.188053, 0)
A_edge2 = Waypoint(28.591197, 113.187988, 0)
A_edge4 = Waypoint(28.591229, 113.188219, 0)
A_edge1 = Waypoint(28.5913153, 113.1879303, 0)
wp_range_A = [A_edge1, A_edge2, A_edge3, A_edge4]
A_target = [A_edge1, A_edge2, A_edge4]

wp_home = Waypoint(28.5926613, 113.1873804, 35)

'''                                         
参数表
'''
# 投票选择朝哪个靶标投弹
TARGET_CHOOSE_A = 0
TARGET_CHOOSE_B = 1
# 侦察进近方向，指南针标准
DETECT_APPROACH_B = 335
DETECT_APPROACH_A = 155
# 盘旋直径
Diameter = 0.00065
# 飞掠靶标区和绕圈处的高度设置，第一圈直接投弹的高度设置
Alt_detect = 18  # a组确定15，b组确定25
Alt_circle = 55  # a组确定55，b组确定65
Alt_bomb_start = 23  # a组确定23，b组确定25
Alt_bomb_drop = 15  # a组确定15，b组确定15
# 在靶标坐标前后拓展的距离
Length_expend = 0.00010
Length_start = 0.00090  # 约90米
Length_bomb_start = 0.00050  # 约50米
Length_bomb = 0.00018  # b组确定0.00045
# 侦察旋转方向（投弹相反）
Direction_A = -1  # -1为顺时针
Direction_B = 1


# detect_angle为进入侦察区的进场角度，按照指南针标准
def course(detect_angle, group='A',
           diameter=Diameter, alt_detect=Alt_detect, alt_circle=Alt_circle, length_expend=Length_expend, direction=-1):
    # 转换为象限角
    angle = pi * ((360 - detect_angle) + 90) / 180
    right_angle = 0.5 * pi

    # 起始处和转向点

    # 直接投弹起始点
    if group == 'A':
        wp_start = Waypoint(A_target[TARGET_CHOOSE_A].lat + Length_start * sin(angle + pi),
                            A_target[TARGET_CHOOSE_A].lon + Length_start * cos(angle + pi), Alt_bomb_start)
        wp_bomb_start = Waypoint(A_target[TARGET_CHOOSE_A].lat + Length_bomb_start * sin(angle + pi),
                                 A_target[TARGET_CHOOSE_A].lon + Length_bomb_start * cos(angle + pi), Alt_bomb_start)
        wp_bomb_drop = Waypoint(A_target[TARGET_CHOOSE_A].lat + Length_bomb * sin(angle + pi),
                                A_target[TARGET_CHOOSE_A].lon + Length_bomb * cos(angle + pi), Alt_bomb_drop)
        wp_target = Waypoint(A_target[TARGET_CHOOSE_A].lat,
                             A_target[TARGET_CHOOSE_A].lon, Alt_bomb_drop+5)
        # 第一圈 内侧靶标
        wp_start1 = Waypoint(wp_range_A[0].lat + length_expend * sin(angle + pi),
                             wp_range_A[0].lon + length_expend * cos(angle + pi), alt_detect)
        wp_end1 = Waypoint(wp_range_A[1].lat + length_expend * sin(angle),
                           wp_range_A[1].lon + length_expend * cos(angle), alt_detect)
        wp_turn11 = Waypoint(wp_end1.lat + diameter * sin(angle + direction * right_angle),
                             wp_end1.lon + diameter * cos(angle + direction * right_angle), alt_circle)
        wp_turn12 = Waypoint(wp_start1.lat + diameter * sin(angle + direction * right_angle),
                             wp_start1.lon + diameter * cos(angle + direction * right_angle), alt_circle)

        # 第二圈 外侧靶标
        wp_start2 = Waypoint(wp_range_A[2].lat + length_expend * sin(angle + pi),
                             wp_range_A[2].lon + length_expend * cos(angle + pi), alt_detect)
        wp_end2 = Waypoint(wp_range_A[3].lat + length_expend * sin(angle),
                           wp_range_A[3].lon + length_expend * cos(angle), alt_detect)
        wp_turn21 = Waypoint(wp_end2.lat + diameter * sin(angle + direction * right_angle),
                             wp_end2.lon + diameter * cos(angle + direction * right_angle), alt_circle)
        wp_turn22 = Waypoint(wp_start2.lat + diameter * sin(angle + direction * right_angle),
                             wp_start2.lon + diameter * cos(angle + direction * right_angle), alt_circle)
    else:
        wp_start = Waypoint(B_target[TARGET_CHOOSE_B].lat + Length_start * sin(angle + pi),
                            B_target[TARGET_CHOOSE_B].lon + Length_start * cos(angle + pi), Alt_bomb_start)
        wp_bomb_start = Waypoint(B_target[TARGET_CHOOSE_B].lat + Length_bomb_start * sin(angle + pi),
                                 B_target[TARGET_CHOOSE_B].lon + Length_bomb_start * cos(angle + pi), Alt_bomb_start)
        wp_bomb_drop = Waypoint(B_target[TARGET_CHOOSE_B].lat + Length_bomb * sin(angle + pi),
                                B_target[TARGET_CHOOSE_B].lon + Length_bomb * cos(angle + pi), Alt_bomb_drop)
        wp_target = Waypoint(B_target[TARGET_CHOOSE_B].lat,
                             B_target[TARGET_CHOOSE_B].lon, Alt_bomb_drop+5)

        # 第一圈 内侧靶标
        wp_start1 = Waypoint(wp_range_B[0].lat + length_expend * sin(angle + pi),
                             wp_range_B[0].lon + length_expend * cos(angle + pi), alt_detect)
        wp_end1 = Waypoint(wp_range_B[1].lat + length_expend * sin(angle),
                           wp_range_B[1].lon + length_expend * cos(angle), alt_detect)
        wp_turn11 = Waypoint(wp_end1.lat + diameter * sin(angle + direction * right_angle),
                             wp_end1.lon + diameter * cos(angle + direction * right_angle), alt_circle)
        wp_turn12 = Waypoint(wp_start1.lat + diameter * sin(angle + direction * right_angle),
                             wp_start1.lon + diameter * cos(angle + direction * right_angle), alt_circle)

        # 第二圈 外侧靶标
        wp_start2 = Waypoint(wp_range_B[2].lat + length_expend * sin(angle + pi),
                             wp_range_B[2].lon + length_expend * cos(angle + pi), alt_detect)
        wp_end2 = Waypoint(wp_range_B[3].lat + length_expend * sin(angle),
                           wp_range_B[3].lon + length_expend * cos(angle), alt_detect)
        wp_turn21 = Waypoint(wp_end2.lat + diameter * sin(angle + direction * right_angle),
                             wp_end2.lon + diameter * cos(angle + direction * right_angle), alt_circle)
        wp_turn22 = Waypoint(wp_start2.lat + diameter * sin(angle + direction * right_angle),
                             wp_start2.lon + diameter * cos(angle + direction * right_angle), alt_circle)

    wp_line_start = wp_straight_course([wp_start, wp_bomb_start], 2)
    wp_line_bomb = wp_bombing_insert_course([wp_bomb_start, wp_bomb_drop], 10, 3, angle)

    # line11 = wp_straight_course([wp_start1, wp_end1], 3)
    circle11 = wp_circle_course([wp_end1, wp_turn11], 3, 180, direction=direction)
    line12 = [wp_turn11, wp_turn12]
    circle12 = wp_circle_course([wp_turn12, wp_start2], 3, 180, direction=direction)

    line21 = wp_straight_course([wp_start2, wp_end2], 2)
    circle21 = wp_circle_course([wp_end2, wp_turn21], 3, 180, direction=direction)
    line22 = [wp_turn21, wp_home]

    mission_course = wp_line_start
    mission_course.pop(-1)
    mission_course.extend(wp_line_bomb)
    mission_course.append(wp_target)
    mission_course.extend(circle11)
    mission_course.pop(-1)
    mission_course.extend(line12)
    mission_course.pop(-1)
    mission_course.extend(circle12)

    mission_course.extend(line21)
    mission_course.pop(-1)
    mission_course.extend(circle21)
    mission_course.pop(-1)
    mission_course.extend(line22)

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
    the_connection = mavutil.mavlink_connection('/dev/ttyUSB0', baud=921600)

    course_final = []

    choice = input("输入A或B选择侦察任务：")
    if choice == 'A' or choice == 'a':
        course_final = course(group='A', detect_angle=DETECT_APPROACH_A,  direction=Direction_A)
        print("上传A组航线信息")
        mission_upload_including_bomb_drop(the_connection, course_final, [14])
    elif choice == 'B' or choice == 'b':
        course_final = course(group='B', detect_angle=DETECT_APPROACH_B,  direction=Direction_B)
        print("上传B组航线信息")
        mission_upload_including_bomb_drop(the_connection, course_final, [14])
    else:
        print("输入错误 可以跳过或建议remake")

    # print("进行飞行前初始化和检测")
    # preflight_command(the_connection, wp_home)
