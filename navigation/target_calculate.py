import numpy as np
from sklearn.cluster import KMeans, DBSCAN
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from .class_list import Waypoint
from math import *
from geopy.distance import geodesic
from scipy.optimize import linear_sum_assignment
from .class_list import target_point


# 使用改进k_means算法计算三个坐标点
def k_means_calculate(target_list):
    import matplotlib.pyplot as plt
    # import data
    def parse_data(data):
        points = []
        for line in data.split('\n'):
            values = line.split()
            if len(values) == 2:
                lat, lng = values
                points.append([float(lat), float(lng)])
        return np.array(points)

    # 鲁棒性处理
    def robust_points(points):
        # 数据缩放
        scaler = StandardScaler()
        scaled_points = scaler.fit_transform(points)
        # DBSCAN去除离群点
        dbscan = DBSCAN(eps=0.2, min_samples=2)
        dbscan.fit(scaled_points)

        # 仅保留核心点
        core_mask = np.zeros_like(dbscan.labels_, dtype=bool)
        core_mask[dbscan.core_sample_indices_] = True
        clustered_points = scaled_points[core_mask]

        # 返回 scaler
        return clustered_points, scaled_points, scaler
        # 如有需要可以对坐标# 逆缩放 points
        # points = scaler.inverse_transform(post_data)

    # PCA聚类图
    def pca_center(post_data):
        pca_points = kmeans.predict(post_data)
        # 绘制散点图
        plt.scatter(post_data[:, 0], post_data[:, 1], c=pca_points)
        # 绘制聚类中心点
        plt.scatter(centroids[:, 0], centroids[:, 1], s=100, c='red', marker='*')
        # 添加文本标注
        for i in range(3):
            plt.text(centroids[i, 0], centroids[i, 1], 'pca' + str(i + 1))
        return plt, pca_points

    # 原始空间映射绘图
    def origin_grid(centroids, points):
        # 将points数据进行KMeans预测,得到分类信息
        data_points = kmeans.predict(points)
        # 将聚类中心点映射回原始坐标
        pca_centroids = scaler.inverse_transform(centroids)
        orig_centroids = pca.inverse_transform(pca_centroids)
        plt.scatter(points[:, 0], points[:, 1], c=data_points)
        plt.scatter(orig_centroids[:, 0], orig_centroids[:, 1], s=100, c='red', marker='*')
        for i in range(3):
            plt.text(orig_centroids[i, 0], orig_centroids[i, 1], 'targ' + str(i + 1))
        return orig_centroids, plt

    list = []
    for i in range(len(target_list)):
        list.append([target_list[i].lat, target_list[i].lon])
    points = np.array(list)
    # print(points)

    # PCA
    pca = PCA(n_components=2)
    post_data = pca.fit_transform(points)
    # robust
    post_data, scaled_points, scaler = robust_points(post_data)
    if len(post_data) < 10:
        print('K-means算法数据点不足')
        return None
    # # 两层逆映射
    # tags = scaler.inverse_transform(post_data)
    # res = pca.inverse_transform(tags)

    # KMeans聚类
    kmeans = KMeans(n_clusters=3)
    kmeans.fit(post_data)
    # 获取聚类中心点
    centroids = kmeans.cluster_centers_
    # print(centroids)

    plt.figure()
    plt, pcap = pca_center(post_data)

    plt.figure(figsize=(8, 8))
    plt.show()
    targets, plt = origin_grid(centroids, points)
    target1 = Waypoint(targets[0][0], targets[0][1], 0)
    target2 = Waypoint(targets[1][0], targets[1][1], 0)
    target3 = Waypoint(targets[2][0], targets[2][1], 0)
    return [target1, target2, target3]




# 弃用 在零附近会出现问题
def target_order(target_list):
    # 计算三角形重心
    center = Waypoint(0.3333*(target_list[0].lat + target_list[1].lat + target_list[2].lat),
                      0.3333*(target_list[0].lon + target_list[1].lon + target_list[2].lon), 0)
    print(center.lat, center.lon)
    # 计算三个点相对角度
    target_dict = {}


def angle_cal(target, center):
    if target.lon > center.lon:
        angle = atan((target.lat-center.lat) / (target.lon-center.lon))
    else:
        angle = atan((target.lat - center.lat) / (target.lon - center.lon))+pi
    return angle


# 将数字和靶标进行匹配，传入三个数字的坐标、解算坐标和中位数数字在数组中的位置
def target_match(list_k, list_num, target_num):
    # 聚类结果坐标点
    points2 = np.array([[list_k[0].lat, list_k[0].lon], [list_k[1].lat, list_k[1].lon], [list_k[2].lat, list_k[2].lon]])

    # 带数字的坐标点
    points1 = np.array([[list_num[0].lat, list_num[0].lon], [list_num[1].lat,
                        list_num[1].lon], [list_num[2].lat, list_num[2].lon]])

    # 计算两组点之间的欧式距离
    dist_matrix = np.zeros((len(points1), len(points2)))
    for i in range(len(points1)):
        for j in range(len(points2)):
            dist_matrix[i][j] = np.linalg.norm(points1[i] - points2[j])

    # 利用匈牙利算法求解最佳匹配
    row_ind, col_ind = linear_sum_assignment(dist_matrix)

    # 返回匹配结果
    for i in range(len(row_ind)):
        if row_ind[i] == target_num:
            return Waypoint(list_k[col_ind[i]].lat, list_k[col_ind[i]].lon, 0)

    # 防止没有返回值
    return Waypoint(list_k[col_ind[1]].lat, list_k[col_ind[1]].lon, 0)


# 判断某个坐标是否在指定的矩形区域内（粗暴的南北正向矩形）
def is_inside_target_area(target, target_known):
    max_lat = max(target_known[0].lat, target_known[1].lat, target_known[2].lat, target_known[3].lat)
    min_lat = min(target_known[0].lat, target_known[1].lat, target_known[2].lat, target_known[3].lat)
    max_lon = max(target_known[0].lon, target_known[1].lon, target_known[2].lon, target_known[3].lon)
    min_lon = min(target_known[0].lon, target_known[1].lon, target_known[2].lon, target_known[3].lon)
    if target.lat > max_lat or target.lat < min_lat or target.lon > max_lon or target.lon < min_lon:
        return False
    else:
        return True


# 使用直接平均的方法粗暴地计算靶标坐标，以供匹配使用
def coordinate_aver_cal(target_list, num):
    lat = 0
    lon = 0
    count = 0
    for i in range(len(target_list)):
        if target_list[i].number == num:
            lat = (lat * count + target_list[i].lat) / (count + 1)
            lon = (lon * count + target_list[i].lon) / (count + 1)
            count += 1
    return target_point(lat=lat, lon=lon, number=num)


# 对坐标进行最后的检查，防止解算坐标在目标区外，因此按照方向给一个预设点赋值。坐标区已知为四个顶点
def match_if_out_of_area(target, target_boarder, target_known):
    # 超出靶标区
    if not is_inside_target_area(target, target_boarder):
        center = Waypoint(0.25*(target_boarder[0].lat + target_boarder[1].lat +
                                target_boarder[2].lat + target_boarder[3].lat),
                          0.25*(target_boarder[0].lon + target_boarder[1].lon +
                                target_boarder[2].lon + target_boarder[3].lon), 0)
        angle = angle_cal(target, center)

        # 根据现场状况修改吧
        if angle < 0.5*pi or angle > 1.5*pi:
            return target_known[0]
        elif 0.5*pi < angle < pi:
            return target_known[1]
        else:
            return target_known[2]
    # 没有超出靶标区
    else:
        return target



