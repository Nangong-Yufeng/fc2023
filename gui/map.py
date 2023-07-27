import io
import folium
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView
import random 

class MapWindow(QWidget):
    """显示地图的QWidget, 实现参考 https://learndataanalysis.org/display-folium-maps-in-pyqt5/
    """
    TEST_LOCATION = (22.5903516, 113.9755356) # University Town Sports Center of Shenzhen
    TEST_PATH = [
        (22.5903516, 113.9755356),
        (22.5904516, 113.9755356),
        (22.5905516, 113.9755356),
        (22.5906516, 113.9755356),
        (22.5907516, 113.9755356),
        (22.5908516, 113.9755356),
    ]
    TEST_POINTS = [
        (22.5903516, 113.9755356),
        (22.5906516, 113.9755356),
    ]


    def __init__(self, parent=None, location:tuple[float, float]=TEST_LOCATION):
        super().__init__(parent)
        self.initUI()
        self.createMap(location)
        self.displayMap()

    
    def initUI(self):
        """初始化UI内容
        """
        # 创建一个QWebEngineView组件，并添加到layout里
        self.webView = QWebEngineView(self)
        self.layout = QVBoxLayout(self)
        self.setLayout(self.layout)
        self.layout.addWidget(self.webView)

    def createMap(self, location:tuple[float, float]=TEST_LOCATION):
        """创建一个地图并放到画面里

        Args:
            location (tuple[float, float]): 地图中心坐标
            zoom_start int: 地图初始缩放比例
        """
        
        # 创建一个Folium地图对象
        self.map = folium.Map(location=location, zoom_start=1000)
        self.map.add_child(folium.LatLngPopup())
        # self.path = PolyLine(MapWindow.TEST_PATH).add_to(self.map)
        for p in MapWindow.TEST_POINTS:
            folium.Marker(p).add_to(self.map)
        
    def displayMap(self):
        with io.BytesIO() as f:
            # 临时保存地图
            self.map.save(f, close_file=False)
            # 在QWebEngineView组件中加载Folium地图
            self.webView.setHtml(f.getvalue().decode())
        
    def addPoint(self):
        point = (22.5908516+random.random()/500, 113.9755356+random.random()/500)
        self.path.locations.append(point)
        self.displayMap()

# class MapController(QWidget):
#     """
#     放一些控件，用于控制地图
#     """
    
