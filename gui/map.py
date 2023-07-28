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

        (22.59072678, 113.97520636),
        (22.590526960000002, 113.97519292),
        (22.590327140000003, 113.97517948),
        (22.590127320000004, 113.97516603999999),
        (22.589927500000005, 113.97515259999999),
        (22.589860272156944, 113.97516288054855),
        (22.58979950621155,  113.97519342187206),
        (22.58975114380504, 113.97524123766648),
        (22.5897199137714,  113.97530165254511),
        (22.589708869756123,  113.97536875919373),
        (22.58971909163341,  113.97543599598265),
        (22.58974957991699,  113.97549678855724),
        (22.58979735348906,  113.97554519267248),
        (22.58985774109134,  113.97557647541615),
        (22.589924838076676, 113.9755875779888),
        (22.59012354,  113.9755938),
        (22.590322280000002, 113.9755938),
        (22.590521020000004, 113.9755938),
        (22.590719760000006, 113.9755938),
        (22.590918500000008, 113.9755938),
        (22.59097562701364,  113.9755859702087),
        (22.591027542873597, 113.97556087947726),
        (22.591069171292297, 113.97552098115555),
        (22.591096441879216, 113.97547017646671),
        (22.591106688140112, 113.97541343304897),
        (22.59109890820452,  113.9753562992243),
        (22.591073862787763, 113.97530436148831),
        (22.591034000808886, 113.97526269826763),
        (22.590983219937463, 113.97523538335564),
        (22.59092648548288,  113.97522508758068)
    ]


    def __init__(self, parent=None, location = TEST_LOCATION):#tuple[float, float]=TEST_LOCATION):
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

    def createMap(self, location= TEST_LOCATION): #:tuple[float, float]=TEST_LOCATION):
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
    
