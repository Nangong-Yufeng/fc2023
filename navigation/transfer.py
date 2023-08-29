from pymavlink import mavutil
from .get_para import gain_posture_para
from .class_list import track_point
import cv2
import numpy as np

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

