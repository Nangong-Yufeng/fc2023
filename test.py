from vision.vision_class import Vision

vis = Vision(source="D:/ngyf/videos/DJI_0001.MP4")

while True:
    vis.shot()
    vis.run()
