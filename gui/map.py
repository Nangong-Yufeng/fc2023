import io
import folium
from queue import Queue, Empty
from PyQt5.QtWidgets import QWidget, QVBoxLayout
from PyQt5.QtWebEngineWidgets import QWebEngineView

class MapWindow(QWidget):
    """显示地图的QWidget, 实现参考 https://learndataanalysis.org/display-folium-maps-in-pyqt5/
    """
    DEFAULT_LOCATION = (22.5903516, 113.9755356) # University Town Sports Center of Shenzhen
    #DEFAULT_LOCATION = (-35.3622066, 149.1651135)
    pathQueue = Queue(maxsize=0)
    targetPoints = None

    def __init__(self, parent=None, location=DEFAULT_LOCATION):#tuple[float, float]=TEST_LOCATION):
        super().__init__(parent)
        self.initUI()
        self.path = None
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

    def createMap(self, location=DEFAULT_LOCATION):
        """创建一个地图, 并在上面画静态的目标点

        Args:
            location (tuple[float, float]): 地图中心坐标
            zoom_start int: 地图初始缩放比例
        """
        
        # 创建一个Folium地图对象
        self.map = folium.Map(location=location, zoom_start=17)
        self.map.add_child(folium.LatLngPopup())
        if MapWindow.targetPoints:
            for p in MapWindow.targetPoints:
                folium.Marker(p).add_to(self.map)
        
    def displayMap(self):
        """ 把当前的地图进行保存并展示
        """
        with io.BytesIO() as f:
            # 临时保存地图
            self.map.save(f, close_file=False)
            # 在QWebEngineView组件中加载Folium地图
            self.webView.setHtml(f.getvalue().decode())
        
    def addPathPoint(self):
        while True:
            try:
                point = self.pathQueue.get_nowait()
            except Empty:
                break
            else:
                if self.path:
                    self.path.locations.append(point)
                else:
                    self.path = folium.PolyLine([point]).add_to(self.map)
        self.displayMap()

    @classmethod
    def putPathPoint(cls, point):
        """向类变量pathQueue中添加点，用于在地图上绘制路径

        Args:
            point (tuple[float, float]): 添加下一个路径点
        """
        if not isinstance(point, tuple) and len(point) == 2:
            raise TypeError("input `point` should be a tuple (float, float)")
        cls.pathQueue.put_nowait(point)
    
    @classmethod
    def setTargetPoints(cls, points):
        """设置类变量targetPoints，用于在地图上标注任务点（需要在生成地图前调用）

        Args:
            points (list[tuple[int, int]]): 要设置为目标点集的列表
        """
        if not isinstance(points, list) or (points and not isinstance(points[0], tuple)):
            raise TypeError("input `points` should be a list of points(or an empty list), and a point should be a tuple (float, float).")
        cls.targetPoints = points

    @classmethod
    def setMapLocation(cls, point):
        """设置地图中心位置，默认为深圳大学城体育中心操场

        Args:
            point (tuple[float, float]): 要设置的中心点
        """
        if not isinstance(point, tuple) and len(point) == 2:
            raise TypeError("input `point` should be a tuple (float, float)")
        cls.DEFAULT_LOCATION = point
 

# class MapController(QWidget):
#     """
#     放一些控件，用于控制地图
#     """
    
