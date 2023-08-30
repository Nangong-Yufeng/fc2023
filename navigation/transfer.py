from pymavlink import mavutil
from .get_para import gain_posture_para
from .class_list import track_point
import cv2
import numpy as np

# yaw 偏航 pitch 俯仰 roll 滚转

import cv2
import numpy as np
import math


def pixel_to_world(camera_intrinsics, camera_rotation, camera_position, img_point):
    # 齐次坐标
    uv1 = np.asmatrix([img_point[0], img_point[1], 1.0]).T

    # 计算相机内参矩阵的逆矩阵
    K_inv = camera_intrinsics.I

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

    print(f'float(Mat1[2][0]: {float(Mat1[2][0])}')
    print(f'Z_c: {Z_c}')
    print(f'coord in camera:{K_inv.dot(uv1)}')
    print(f'T: {T}')
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

def coordinate_transfer():
    # 已经有以下参数
    camera_pose = Pose(0, 0, 92, 0, 0, 0)  # gps坐标，高度，偏航yaw，俯仰pitch，滚转roll

    pixel_coords = [0, 0]  # 点在相机坐标系中的像素位置

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
    cv2.fisheye.undistortPoints(distortedPoint, cameraMatrix, distCoeffs, np.eye(3), newCameraarray)[0][
        0]

    newCameraMatrix = np.mat(newCameraarray)

    from math import cos, sin

    # 计算点在实际中的世界坐标

    # 平移矩阵C，即相机中点在世界坐标的位置
    c = np.mat(
        [camera_pose.x, camera_pose.y, camera_pose.z]
    )

    c = np.asmatrix(c).T  # 转变成列矩阵

    pixel_to_world(newCameraMatrix, rc, c, undistortedPoint)
    # 输出结果
    # 输出的前两个为相对与现在的坐标，最后一位应为0
'''
#为视觉坐标转换准备的函数
def coordinate_transfer(the_connection, vision_target, track_list):
    # 传入参数
    class Pose:
        def __init__(self, x, y, z, roll, pitch, yaw) -> None:
            self.x = x #相对飞机
            self.y = y
            self.z = z
            self.roll = roll
            self.pitch = pitch
            self.yaw = yaw


    camera_pose = Pose(1,2,3,4,5,6)  # gps坐标，相机事先与x,y,z夹角

    pixel_coords = [7,8]  # 点在相机坐标系中的像素位置

    # 内参矩阵cameraMatrix，需要根据实际情况定义
    cameraMatrix = np.array(
        [
            [702.564636230469, 0, 960],
            [0, 702.564636230469, 540],
            [0, 0, 1]
        ]
    )
    # 畸变参数k1，k2，k3，k4
    distCoeffs = np.array([0.2499176859855652, 0.0136057548224926, -0.0620835833251476, 0.0121930716559291])
    pose = gain_posture_para(the_connection)

    # 对目标坐标去畸变
    # 获取变化后相机内参
    newCameraMatrix, _ = cv2.getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, (1920, 1080), 1.7, (1920, 1080), True)

    # 假设要获取的像素坐标为 (x, y)
    distortedPoint = np.array([[pixel_coords]], dtype=np.float32)  # 输入点

    # undistortedPoint就是去畸变后的点[x,y]，尺寸为1080*1920
    undistortedPoint = \
    cv2.fisheye.undistortPoints(distortedPoint, cameraMatrix, distCoeffs, np.eye(3), newCameraMatrix)[0][0]

    from math import cos, sin
    # 计算点在实际中的世界坐标
    # 旋转矩阵
    theta, phi, psi = camera_pose.pitch, camera_pose.roll, camera_pose.yaw
    r = np.array(
        [
            [cos(theta) * cos(psi), cos(theta) * sin(psi), - sin(theta)],
            [sin(phi) * sin(theta) * cos(psi) - cos(phi) * sin(psi),
             sin(phi) * sin(theta) * sin(psi) + cos(phi) * cos(psi), sin(phi) * cos(theta)],
            [cos(phi) * sin(theta) * cos(psi) + sin(phi) * sin(psi),
             cos(phi) * sin(theta) * sin(psi) - sin(phi) * cos(psi), cos(phi) * cos(theta)]
        ]
    )

    # 平移矩阵
    t = np.array(
        [camera_pose.x, camera_pose.y, camera_pose.z]
    )
    pixel_to_world(newCameraMatrix, r, t, distortedPoint)
    # 输出结果


# 解算世界坐标
def pixel_to_world(camera_intrinsics, r, t, img_points):
    K_inv = camera_intrinsics.I
    R_inv = np.asmatrix(r).I
    R_inv_T = np.dot(R_inv, np.asmatrix(t))
    world_points = []
    coords = np.zeros((3, 1), dtype=np.float64)
    for img_point in img_points:
        coords[0] = img_point[0]
        coords[1] = img_point[1]
        coords[2] = 1.0
        cam_point = np.dot(K_inv, coords)
        cam_R_inv = np.dot(R_inv, cam_point)
        scale = R_inv_T[2][0] / cam_R_inv[2][0]
        scale_world = np.multiply(scale, cam_R_inv)
        world_point = np.asmatrix(scale_world) - np.asmatrix(R_inv_T)
        pt = np.zeros((3, 1), dtype=np.float64)
        pt[0] = world_point[0]
        pt[1] = world_point[1]
        pt[2] = 0
        world_points.append(pt.T.tolist())

    return world_points

def track_point_selecting(frequency=10):
    pass
    point = track_point(0,0,0,0,0,0,0)
    return point
'''
