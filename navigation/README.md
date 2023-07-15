# navigation文档说明

## 需要安装的环境

   ### pymavlink环境
       pip install pymavlink
   ### geopy插件
       pip install geopy
   用于根据经纬度计算距离

## main.py 
   主函数

## preflight.py
   定义了飞行前动作的函数原型，包括arm，模式设置，home点设置等
   
## class_list.py
   定义了一些类的原型，便于参数传递

## mission.py
   定义了航点任务的一些函数，包括航点集上传，清除现有任务等
   
### 存在问题：
其中mission_upload函数，由于不明原因，存在会遗漏第一个航点的情况，暂时通过设置一个无用航点来解决(建议设置home点)