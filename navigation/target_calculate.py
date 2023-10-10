import numpy as np
from sklearn.cluster import KMeans, DBSCAN
from sklearn.preprocessing import StandardScaler
from sklearn.decomposition import PCA
from .class_list import Waypoint


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
        print('not enough data for k-means')
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
    targets, plt = origin_grid(centroids, points)
    target1 = Waypoint(targets[0][0], targets[0][1], 0)
    target2 = Waypoint(targets[1][0], targets[1][1], 0)
    target3 = Waypoint(targets[2][0], targets[2][1], 0)
    return [target1, target2, target3]

    # plt.show()

