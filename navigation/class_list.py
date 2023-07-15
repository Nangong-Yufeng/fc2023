import geopy.distance

class Position_relative:
    def __init__(self, lat=0, lon=0, alt=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class Waypoint(Position_relative):
    def __init__(self, lat, lon, alt):
        super(Waypoint, self).__init__(lat, lon, alt)
    def distance(self, the_connection):
        target = (self.lat * 10e-8, self.lon * 10e-8)
        local = (the_connection.location().lat , the_connection.location().lng )
        target_distance = geopy.distance.GeodesicDistance(target, local).meters
        print(target_distance)