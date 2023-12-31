import geopy.distance
from .error_process import rec_match_received

class Position_relative:
    def __init__(self, lat=0, lon=0, alt=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt
    def show(self):
        print("(", self.lat, ",", self.lon, "),")
        #print("lat: %s, lon: %s, alt:%s" %(self.lat, self.lon, self.alt))

class Waypoint(Position_relative):
    def __init__(self, lat, lon, alt):
        super(Waypoint, self).__init__(lat, lon, alt)

    #计算当前位置与目标waypoint之间的距离
    def distance(self, the_connection):
        target = (self.lat, self.lon)
        local = (the_connection.location().lat , the_connection.location().lng )
        target_distance = geopy.distance.GeodesicDistance(target, local).meters
        return target_distance



# 记录飞机航点（含时间信息）
class track_point(Position_relative):
    def __init__(self, lat, lon, alt, time, roll, pitch, yaw):
        super(track_point, self).__init__(lat, lon, alt)
        self.time = time
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw


class posture_inform:
    def __init__(self, time, roll, pitch, yaw, roll_speed, pitch_speed, yaw_speed):
        self.time = time
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw
        self.roll_speed = roll_speed
        self.pitch_speed = pitch_speed
        self.yaw_speed = yaw_speed

class speed_inform:
    def __init__(self, vx, vy, vz, direction):
        # 分别指向北、东和下的速度分量（转为m/s）
        self.vx = vx * 1e-2
        self.vy = vy * 1e-2
        self.vz = vz * 1e-2
        # 方向角，正北为360
        self.direction = direction

# 视觉识别到靶标后返回的信息类
class vision_position:
    def __init__(self, x, y, target_number):
        self.x = x
        self.y = y
        # self.time = rec_match_received(the_connection, 'SYSTEM_TIME').time_boot_ms
        self.num = target_number

# 靶标信息
class target_point:
    def __init__(self, lat, lon, number):
        self.lat = lat
        self.lon = lon
        self.number = number
