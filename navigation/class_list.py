import geopy.distance

class Position_relative:
    def __init__(self, lat=0, lon=0, alt=0):
        self.lat = lat
        self.lon = lon
        self.alt = alt

class Target_position:
    def __init__(self, position):
        self.lat = position.lat
        self.lon = position.lon
        self.alt = position.alt
    def distance(self, the_connection):
        target = (self.lat * 10e-8, self.lon * 10e-8)
        local = (the_connection.location().lat , the_connection.location().lng )
        target_distance = geopy.distance.GeodesicDistance(target, local).meters
        print(target_distance)