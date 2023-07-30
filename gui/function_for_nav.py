from threading import Thread
from map import MapWindow
from PyQt5.QtWidgets import QMainWindow, QApplication, QSplitter
from PyQt5.QtCore import QTimer
import platformSetting, sys
from app import runGui

def static_map_show(static_wp):
    MapWindow.setTargetPoints(static_wp)
    Thread(target=runGui).start()
