import sys
from map import MapWindow
from PyQt5.QtWidgets import QMainWindow, QApplication, QSplitter
from PyQt5.QtCore import QTimer


def runGui():
    """运行GUI程序

    Returns:
        app.exec_(): QApplication的返回值
    """
    app = QApplication([])
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
    
        # self.timer = QTimer(self) #初始化一个定时器
        # self.timer.timeout.connect(self.map.addPoint) #计时结束调用operate()方法
        # self.timer.start(2000) #设置计时间隔并启动        


if __name__ == '__main__':
    from queue import Queue
    runGui()
