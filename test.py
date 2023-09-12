from vision.vision_class import Vision

vis = Vision()

import time

print(time.time() * 1000)

while True:
    sta = int(time.time() * 1000)
    vis.shot()
    vis.run()
    print(int(time.time() * 1000) - sta)