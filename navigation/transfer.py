# yaw 偏航 pitch 俯仰 roll 滚转

import cv2
import numpy as np
import math
from .class_list import target_point
CONSTANTS_RADIUS_OF_EARTH = 6371000. 

def pixel_to_world(camera_intrinsics, camera_rotation, camera_position, img_point):   
    # 齐次坐标
    uv1 = np.asmatrix([img_point[0], img_point[1], 1.0]).T🔄  ❓   

    # 计算相机内参矩阵的逆矩阵
    K_inv = camera_intrinsics.I🔄  ❓  

    # 旋转矩阵的逆矩阵
    R_inv = np.asmatrix(camera_rotation)   

    # 由r和t求相机坐标系下世界原点的坐标x过程：
    # 设坐标系A是跟世界坐标系方向相同，跟相机坐标系原点重合的坐标系
    # (1) r：设x为相机坐标系下一个点的坐标，r*x表示该点在A系下的坐标
    # (2) t: 在世界坐标系中 A系原点 的位置向量
    # (3) 假设x是相机坐标系下世界原点的坐标，由(1)可得：r*x表示该点在A系下的坐标
    # (4) 由(2)可得，在A系中 世界坐标系原点 的位置向量是 -t，即 r*x = -t
    # 则 x = r.I * r * x = r.I * -t = - r.I * t 
    T = -R_inv.I.dot(camera_position)  

    Z_w = 0
    Mat1 = R_inv.dot(K_inv).dot(uv1)  
    Mat2 = R_inv.dot(T)
    Z_c = (Z_w + float(Mat2[2][0])) / float(Mat1[2][0])  # 计算尺度因子，用于缩放点到世界坐标系

    ## print(f'float(Mat1[2][0]: {float(Mat1[2][0])}')
    ## print(f'Z_c: {Z_c}') 
    ## print(f'coord in camera:{K_inv.dot(uv1)}')
    ## print(f'T: {T}')
    return R_inv.dot(Z_c * K_inv.dot(uv1) - T)   


# 定义相机位置和姿态
class Pose:
    def __init__(self, x, y, z, yaw, pitch, roll):  
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.pitch = pitch
        self.roll = roll 
        self.calculate_rotation_matrix()

    def calculate_rotation_matrix(self):
        """求旋转矩阵: 先偏航yaw, 再俯仰pitch, 最后滚转roll得到最终姿态"""
        rx = np.array([
            [1, 0, 0],
            [0, np.cos(self.roll), -np.sin(self.roll)],
            [0, np.sin(self.roll), np.cos(self.roll)]
        ])

        ry = np.array([
            [np.cos(self.pitch), 0, np.sin(self.pitch)],
            [0, 1, 0], 
            [-np.sin(self.pitch), 0, np.cos(self.pitch)]
        ])

        rz = np.array([ 
            [np.cos(self.yaw), -np.sin(self.yaw), 0], 
            [np.sin(self.yaw), np.cos(self.yaw), 0], 
            [0, 0, 1] 
        ])

        self.rotation_matrix = rz.dot(ry).dot(rx) 

    def get_rotation_matrix(self): 
        return self.rotation_matrix

def coordinate_transfer(lat, lon, alt, -yaw, pitch, roll, vision_x, vision_y, vision_num):   
    # 已经有以下参数
    # 将相机设为0，0，即坐标系是米级
    camera_pose = Pose(0, 0, alt, yaw, pitch, roll)  # gps坐标，高度，偏航yaw，俯仰pitch，滚转roll     

    pixel_coords = [vision_x, vision_y]  # 点在相机坐标系中的像素位置

    print(f'pixel_coords: {pixel_coords}')  
    # 相机原始内参矩阵cameraMatrix
    cameraMatrix = np.array(
        [
            [702.564636230469, 0, 960], 
            [0, 702.564636230469, 540],  
            [0, 0, 1] 
        ]
    )
    # 相机原始畸变参数k1，k2，k3，k4
    distCoeffs = np.array([0.2499176859855652, 0.0136057548224926, -0.0620835833251476, 0.0121930716559291])  

    # 计算旋转矩阵
    rc = camera_pose.get_rotation_matrix()  

    print(f'rc: {rc}')  

    # 对于地面坐标系xoy，若x指向东方，y指向北方
    # 对于飞机，x轴指向正北，y轴指向正东， z轴指向正上
    # 旋转方向符合右手定则

    # 去畸变
    # 获取变化后相机内参
    newCameraarray, _ = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (1920, 1080), 1.7, (1920, 1080), True)  

    # 假设要获取的像素坐标为 (x, y)
    distortedPoint = np.array([[pixel_coords]], dtype=np.float32)  # 输入点  

    # undistortedPoint就是去畸变后的点[x,y]，尺寸为1080*1920
    undistortedPoint = \
    cv2.fisheye.undistortPoints(distortedPoint, cameraMatrix, distCoeffs, np.eye(3), newCameraarray)[0][0]  
    print(f'undistortedPoint: {undistortedPoint}')  
    newCameraMatrix = np.mat(newCameraarray)  

    from math import cos, sin  

    # 计算点在实际中的世界坐标

    # 平移矩阵C，即相机中点在世界坐标的位置
    c = np.mat( 
        [camera_pose.x, camera_pose.y, camera_pose.z]   
    )

    c = np.asmatrix(c).T  # 转变成列矩阵

    matrix = pixel_to_world(newCameraMatrix, rc, c, undistortedPoint)  
    # 输出的前两个为相对与现在的坐标（单位是米级），最后一位应为0

    location = XYtoGPS(matrix[1], matrix[0], lat, lon)   
    target = target_point(location[0], location[1], vision_num)  
    return target 
    # 输出结果，输出类型为类target_point


# 相对坐标转为gps坐标（网上抄的），X向北，Y向东
def XYtoGPS(x, y, ref_lat, ref_lon):  
    # input GPS and Reference GPS in degrees
    # output XY in meters (m) X:North Y:East
    x_rad = float(x) / CONSTANTS_RADIUS_OF_EARTH
    y_rad = float(y) / CONSTANTS_RADIUS_OF_EARTH 
    c = math.sqrt(x_rad * x_rad + y_rad * y_rad)

    ref_lat_rad = math.radians(ref_lat)
    ref_lon_rad = math.radians(ref_lon)

    ref_sin_lat = math.sin(ref_lat_rad)
    ref_cos_lat = math.cos(ref_lat_rad)

    if abs(c) > 0:
        sin_c = math.sin(c)
        cos_c = math.cos(c)

        lat_rad = math.asin(cos_c * ref_sin_lat + (x_rad * sin_c * ref_cos_lat) / c)
        lon_rad = (ref_lon_rad + math.atan2(y_rad * sin_c, c * ref_cos_lat * cos_c - x_rad * ref_sin_lat * sin_c))

        lat = math.degrees(lat_rad)
        lon = math.degrees(lon_rad)

    else:
        lat = math.degrees(ref_lat)
        lon = math.degrees(ref_lon)

    return [lat, lon]
