from threading import Thread
from map import MapWindow
from app import runGui

def static_map_show(static_wp):

    MapWindow.setTargetPoints(static_wp)

