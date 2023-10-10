from vision.vision_class import Vision
from time import time

vis = Vision(source=r'D:\ngyf\videos\DJI_0006.MP4')

while True:
    sta = int(1000 * time())
    vis.shot()
    vis.run(use_ocr=False)
    print(int(1000 * time()) - sta)
