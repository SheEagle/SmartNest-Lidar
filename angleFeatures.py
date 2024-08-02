import copy
import math
import os
import open3d as o3d
import numpy as np

from preProcess import pre_process

# 获取雷达传来的数据中每个角度出现最多的点，认为是准确的地图点
def clear(filename):
    lidar_all = []
    n = 0
    # angle_increment = 0.010613488964736462
    while n < 592:
        i = 1
        lidar_lists = []
        with open(filename, "r")as f:
            lines = f.readlines()
            while i < len(lines):
                s = lines[i]
                s_new = s.split(',')
                lidar_lists.append(int(round(float(s_new[n]), 2) * 100))
                i = i + 2
        lidar_lists = np.array(lidar_lists)

        """
        np.bincount(a)
        返回一个数组，其长度等于a中元素最大值加1，每个元素值则是它当前索引值在a中出现的次数。

        np.argmax(a)
        返回的是a中元素最大值所对应的索引值
        """
        bincount = np.bincount(lidar_lists)
        max_index = np.argmax(bincount)
        if max_index != 0:
            lidar_all.append(max_index)
        else:
            bincount[0] = 0
            lidar_all.append(np.argmax(bincount))
        n = n + 1
    return lidar_all


# 将雷达数据转换为直角坐标形式
def get_mapPoints(s1: object, s2: object):  # s1：雷达传来的数据结构(_after.txt文件)，s2：txt点云要写进的文件(_cloud.txt文件)
    with open(s2, "w+") as f0:
        """构建准确的地图点"""
        lidar_all = clear(s1)
        angle = 0.0
        angle_increment = 0.010613488964736462
        for r in lidar_all:
            """
             math.trunc(x)
             返回 x 截断整数的部分，即返回整数部分，忽略小数部分。
            """
            x = math.trunc(r * math.cos(angle + (-90.0 * 3.14159 / 180.0)))
            y = math.trunc(r * math.sin(angle + (-90.0 * 3.14159 / 180.0)))
            s = str(x) + " " + str(y) + " " + str(0) + "\n"
            f0.write(s)
            angle = angle + angle_increment


def remove_Point(cloud):
    # 去除地图点云中的离群点
    """
    pc,idx=pcd.remove_radius_outlier(nb_points, radius, print_progress=False)
    nb_points：邻域球内的最少点个数，小于该个数为噪声点
    radius： 邻域半径大小
    pc：去噪后的点云
    idx：去噪保留的点索引
    :param cloud:
    :return:去除离群点之后的cloud
    """
    cl, ind = cloud.remove_radius_outlier(nb_points=4, radius=15)  # 半径滤波，移除给定球体中没有邻居的点，第一个是领域球类最少的点，低于该数是噪声点，第二个是领域半径
    inlier_source = cloud.select_by_index(ind)
    return inlier_source


def draw_registration_result(source, target, transformation):
    source_temp = copy.deepcopy(source)
    target_temp = copy.deepcopy(target)
    source_temp.paint_uniform_color([1, 0.706, 0])
    target_temp.paint_uniform_color([0, 0.651, 0.929])
    source_temp.transform(transformation)
    o3d.visualization.draw_geometries([source_temp, target_temp])


def preprocess_point_cloud(pcd, voxel_size):
    print(":: 使用大小为为{}的体素下采样点云.".format(voxel_size))
    pcd_down = pcd.voxel_down_sample(voxel_size)

    radius_normal = voxel_size * 2
    print(":: 使用搜索半径为{}估计法线".format(radius_normal))
    pcd_down.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    print(":: 使用搜索半径为{}计算FPFH特征".format(radius_feature))
    pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(pcd_down, o3d.geometry.KDTreeSearchParamHybrid(
        radius=radius_feature, max_nn=100))
    return pcd_down, pcd_fpfh


def prepare_dataset(voxel_size, smap_cloud_path, xmap_cloud_path):
    print(":: 加载点云并转换点云的位姿.")
    source = o3d.io.read_point_cloud(smap_cloud_path, format='xyz')
    target = o3d.io.read_point_cloud(xmap_cloud_path, format='xyz')
    trans_init = np.asarray([[1.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0],
                             [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
    source.transform(trans_init)
    draw_registration_result(source, target, np.identity(4))

    source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
    target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
    return source, target, source_down, target_down, source_fpfh, target_fpfh


def execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    print(":: 对下采样的点云进行RANSAC配准.")
    print("   下采样体素的大小为： %.3f," % voxel_size)
    print("   使用宽松的距离阈值： %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True, distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 3,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)
         ], o3d.pipelines.registration.RANSACConvergenceCriteria(100000, 0.999))
    return result


def refine_registration(source, target, transformation, voxel_size):
    distance_threshold = voxel_size * 0.4
    print(":: 对原始点云进行点对面ICP配准精细对齐， 这次使用严格的距离阈值： %.3f." % distance_threshold)
    result = o3d.pipelines.registration.registration_icp(source, target, distance_threshold, transformation,
                                                         o3d.pipelines.registration.TransformationEstimationPointToPoint())
    return result


def angleFeatures(smap_after_path, xmap_after_path, smap_cloud_path, xmap_cloud_path):
    # 点写入txt中
    get_mapPoints(smap_after_path, smap_cloud_path)
    get_mapPoints(xmap_after_path, xmap_cloud_path)

    voxel_size = 5
    source, target, source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(voxel_size, smap_cloud_path,
                                                                                         xmap_cloud_path)

    result_ransac = execute_global_registration(source_down, target_down, source_fpfh, target_fpfh, voxel_size)
    draw_registration_result(source_down, target_down, result_ransac.transformation)

    result_icp = refine_registration(source, target, result_ransac.transformation, voxel_size)
    draw_registration_result(source, target, result_icp.transformation)
    return result_icp.transformation


if __name__ == "__main__":
    prefix = os.getcwd() + "/lidar_data"
    smap_before_path = prefix + "/S/2024-03-17_before.txt"
    xmap_before_path = prefix + "/X/2024-03-17_before.txt"
    pre_process(smap_before_path, xmap_before_path)
    smap_after_path = prefix + '/S/2024-03-17_after.txt'
    xmap_after_path = prefix + '/X/2024-03-17_after.txt'
    smap_cloud_path = prefix + '/S/2024-03-17_cloud.txt'  # S雷达地图txt点云
    xmap_cloud_path = prefix + '/X/2024-03-17_cloud.txt'
    tp = angleFeatures(smap_after_path, xmap_after_path, smap_cloud_path, xmap_cloud_path)
    print(tp)
