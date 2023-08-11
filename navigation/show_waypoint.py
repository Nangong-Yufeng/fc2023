import sys
sys.path.append('../gui')
from gui import setTargetPoints, runGui, setMapLocation
from mission import wp_straight_course, wp_circle_course
from class_list import Waypoint, Position_relative
from threading import Thread


# 由于地图工具的稳定度不足，使用独立的程序，复制航点部分，在飞行前检查一下生成的航点是否合理，而不在飞行过程中使用地图
def show_waypoint_in_map(wp_list, home_position):
    Thread(target=runGui).start()
    setMapLocation((home_position.lat, home_position.lon))
    waypoint_print_list = []
    for count in range(len(wp_list)):
        waypoint_print_list.append((wp_list[count].lat, wp_list[count].lon))
    setTargetPoints(waypoint_print_list)

home_position = Position_relative(22.5903516, 113.9755156, 0)

wp2 = Waypoint(22.5899248, 113.9755938, 120)
wp1 = Waypoint(22.5899275, 113.9751526, 120)
wp3 = Waypoint(22.5909185, 113.9755938, 120)
wp4 = Waypoint(22.5909266, 113.9752198, 120)

wp = [wp1, wp2, wp3, wp4]

wp_line1 = [wp[3], wp[0]]
wp_circle1 = [wp[0], wp[1]]
wp_line2 = [wp[1], wp[2]]
wp_circle2 = [wp[2], wp[3]]

wp_list = (wp_circle_course(wp_circle1, 3, 180, 1))
wp_list.pop(-1)
wp_list.extend(wp_straight_course(wp_line2, 3))
wp_list.pop(-1)
wp_list.extend(wp_circle_course(wp_circle2, 3, 180, 1))
wp_list.pop(-1)
wp_list.extend(wp_straight_course(wp_line1, 3))

show_waypoint_in_map(wp_list, home_position)

