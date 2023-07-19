import geopy.distance

class Position_relative:
    def __init__(self, lat=0, lon=0, alt=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class Waypoint(Position_relative):
    def __init__(self, lat, lon, alt):
        super(Waypoint, self).__init__(lat, lon, alt)

    #计算当前位置与目标waypoint之间的距离
    def distance(self, the_connection):
        target = (self.lat, self.lon)
        local = (the_connection.location().lat , the_connection.location().lng )
        target_distance = geopy.distance.GeodesicDistance(target, local).meters
        print(target_distance)