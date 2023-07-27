import folium
import sys
import os
from PyQt5.QtWebEngineWidgets import QWebEngineView
from PyQt5.QtWidgets import QWidget, QApplication
from PyQt5 import QtCore

class MapWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        self.setGeometry(100, 100, 800, 600)
        self.setWindowTitle('Folium Map in PyQt5')

        # 创建一个Folium地图对象
        m = folium.Map(location=[34.0522, -118.2437], zoom_start=10)

        # 保存Folium地图为HTML文件
        m.save('.cache/map.html')

        # 创建一个QWebEngineView组件
        self.webView = QWebEngineView(self)
        self.webView.setGeometry(0, 0, 800, 600)

        # 在QWebEngineView组件中加载Folium地图
        self.webView.load(QtCore.QUrl.fromLocalFile(os.path.abspath('.cache/map.html')))
        
if __name__ == '__main__':
    app = QApplication(sys.argv)
    mapWindow = MapWindow()
    mapWindow.show()
    sys.exit(app.exec_())
