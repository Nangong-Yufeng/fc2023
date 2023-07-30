from threading import Thread
from map import MapWindow
from PyQt5.QtWidgets import QMainWindow, QApplication, QSplitter
from PyQt5.QtCore import QTimer
import platformSetting, sys

def runGui():
    """运行GUI程序

    Returns:
        app.exec_(): QApplication的返回值
    """
    platformSetting.setting()
    app = QApplication(sys.argv)
    window = MainWindow()
    return app.exec_()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.initUI()
        self.show()

    def initUI(self):
        self.setWindowTitle('Main')
        self.setGeometry(100, 100, 800, 600)

        self.splitter = QSplitter(self)
        self.map = MapWindow(self)
        self.splitter.addWidget(self.map)
        self.setCentralWidget(self.splitter)
    
        self.timer = QTimer(self)  # 初始化一个定时器
        self.timer.timeout.connect(self.map.addPathPoint)  # 计时器绑定map的添加航点刷新
        self.timer.start(1000)  # 设置计时间隔并启动        


if __name__ == '__main__':
    import time, math, tqdm
    # 设置静态target点
    MapWindow.setTargetPoints([
        (22.59072678, 113.97520636),
        (22.59052696, 113.97519292),
    ])

    # 新开一个线程运行GUI
    Thread(target=runGui).start()
    
    # 往路径点队列里 put 1000个点，每个点间隔0.01s
    c_x, c_y = MapWindow.DEFAULT_LOCATION
    for t in tqdm.tqdm(range(1000)):
        t = t/1000
        r = 5 * (0.1 + t)
        x = r * math.cos(2 * math.pi * t)/10000
        y = r * math.sin(2 * math.pi * t)/10000
        point = (c_x + x, c_y + y)
        MapWindow.putPathPoint(point)
        time.sleep(0.01)


        
