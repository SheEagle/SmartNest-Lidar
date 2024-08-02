import os
import math
import time
from sklearn.cluster import DBSCAN
import open3d as o3d
from pynput import keyboard
import threading
import matplotlib.pyplot as plt
from itertools import combinations
import rospy
import message_filters
from sensor_msgs.msg import LaserScan
from sort import *
from abc import abstractmethod, ABCMeta
from socket import *
import smtplib
from email.mime.text import MIMEText
import requests


prefix = os.getcwd() + "/lidar_data/"

smap_after_path = prefix + '/S/2024-03-17_after.txt'
smap_cloud_path = prefix + '/S/2024-03-17_cloud.txt'  # S雷达地图txt点云
sall_cloud_path = prefix + '/S/detect_cloud.txt'  # S雷达包含脚步的地图txt点云
sfoot_cloud_path = prefix + '/result/cloud.txt'  # S雷达分离出的人脚txt点云

xmap_after_path = prefix + '/X/2024-03-17_after.txt'
xmap_cloud_path = prefix + '/X/2024-03-17_cloud.txt'
xall_cloud_path = prefix + '/X/detect_cloud.txt'
xfoot_cloud_path = prefix + '/result/xcloud.txt'

# 全局变量
client_num = 0
isDark = True  # 判断室内是否处于黑暗状态
socket_list = []
ad_sock = []
gas_sensor = 0.0

mail_host = 'smtp.163.com'
mail_user = 'czxivy'
mail_pass = 'UGXPFUOYSXPREKJU'
sender = 'czxivy@163.com'
receivers = ['454259453@qq.com']


# 处理客户端函数
def handle_sensor(recv_data):
    global isDark
    global gas_sensor
    # 确定是哪个传感器发送的信息
    sensor_flag = recv_data.split(":")[0].strip()

    # 处理温湿度传感器发送的信息
    if sensor_flag == "1":
        tem_sensor = float(recv_data.split("temperature: ")[1].split(" ")[0].strip())
        hum_sensor = float(recv_data.split("humidity: ")[1].split(" ")[0].strip())
        gas_sensor = float(recv_data.split("gas: ")[1].strip())
        params = {'tem_hum': str(tem_sensor) + ',' + str(hum_sensor)}
        response = requests.get('http://47.100.224.55:7001/put_tem_hum', params=params)

    # 处理光敏电阻传感器发送的信息
    elif sensor_flag == "2":
        light_sensor = int(recv_data.split("LIGHT_SENSOR: ")[1].strip())
        print(light_sensor)
        if light_sensor >= 1000:
            isDark = True
        else:
            isDark = False


def handle_control(client_socket):
    while True:
        data = client_socket.recv(1024).decode()
        if data != '':
            if data[:2] != 'ad':
                light_num = int(data.split('-')[0])
                ope = data.split('-')[1]
                print(light_num, ope)
                if ope == 'on':
                    socket_list[light_num].send(b'1')
                else:
                    socket_list[light_num].send(b'0')

            else:
                ad_control = data.split('-')[1]
                if ad_control[:2] == 'on':
                    ad_tem = ad_control.split('-')[1]
                    send_txt = '1-'+ad_tem
                    ad_sock[0].send(send_txt.encode())
                else:
                    ad_sock[0].send(b'0')


def tcplink():
    global client_num
    while True:
        client_socket, clientAddr = tcp_server_socket.accept()
        print(clientAddr)
        data = client_socket.recv(1024).decode()
        if data == "0":
            socket_temp.append(client_socket)
            socket_seq.append(0)
            client_socket.send(b'0')
            client_num += 1
        elif data == "1":
            socket_temp.append(client_socket)
            socket_seq.append(1)
            client_socket.send(b'0')
            client_num += 1
        elif data == "2":
            socket_temp.append(client_socket)
            socket_seq.append(2)
            client_socket.send(b'0')
            client_num += 1

        elif data == 'ad':
            ad_sock.append(client_socket)
            client_num += 1

        elif data == 'light_ad_control':
            sub_thread = threading.Thread(target=handle_control, args=(client_socket,))
            sub_thread.start()

        else:
            # 使用多线程去接收多个客户端的请求
            sub_thread = threading.Thread(target=handle_sensor, args=(data,))
            sub_thread.start()


def get_footPoints(source, sink):  # source：雷达传来的数据结构 sink：txt点云要写进的文件
    with open(sink, "w+") as f0:
        angle = 0.0
        angle_increment = 0.010613488964736462
        for r in source:
            r = float(r)
            x = math.trunc(r * math.cos(angle + (-90.0 * 3.14159 / 180.0)))
            y = math.trunc(r * math.sin(angle + (-90.0 * 3.14159 / 180.0)))
            angle = angle + angle_increment
            s = str(x) + " " + str(y) + " " + str(0) + "\n"
            f0.write(s)

def remove_mapPoint(cloud):
    # 去除地图点云中的离群点
    cl, ind = cloud.remove_radius_outlier(nb_points=3, radius=18)
    inlier_source = cloud.select_by_index(ind)
    return inlier_source


def footstep_cluster(pointList):  # 聚类
    db = DBSCAN(eps=8, min_samples=6)
    db.fit(pointList)
    label = db.labels_
    return label


def remove_point(cloud):
    # 去除脚步点云中的离群点
    cl, ind = cloud.remove_radius_outlier(nb_points=3, radius=18)
    inlier_source = cloud.select_by_index(ind)
    return inlier_source


def filter(mapTxt, footTxt, newFootTxt):
    '''
    mapTxt：地图点点云
    footTxt：包含地图点、脚步点的点云
    newFootTxt：要写的点云文件
    均为 xy 坐标系下。

    作用：从 footTxt 中提取纯脚步点云保存到 newFootTxt。
    '''
    n = 0
    f0 = open(newFootTxt, "w+")
    fMap = open(mapTxt, "r")
    fFoot = open(footTxt, "r")
    mapLine = fMap.readlines()
    footLine = fFoot.readlines()
    sumLine = len(footLine)
    while n < sumLine:
        mapPoint = mapLine[n].split(' ')
        footPoint = footLine[n].split(' ')
        mapX = float(mapPoint[0])
        mapY = float(mapPoint[1])
        footX = float(footPoint[0])
        footY = float(footPoint[1])
        # 去除0,0点
        if footX == 0 and footY == 0:
            n = n + 1
            continue
        if ((mapX - footX) ** 2 + (mapY - footY) ** 2) >= 800:  # 差别很大判定为脚步点
            # 判断为脚步点
            s = footPoint[0] + " " + footPoint[1] + " " + str(0) + "\n"
            f0.write(s)
        n = n + 1


def calculate_distance(point1, point2):
    return np.linalg.norm(np.array(point1) - np.array(point2))


def pair_points(foot_center):
    pairs = []
    for pair in combinations(foot_center, 2):
        distance = calculate_distance(pair[0], pair[1])
        if distance < 45:
            pairs.append((pair[0], pair[1], distance))

    # 根据距离升序排序
    pairs.sort(key=lambda x: x[2])

    # 根据排序后的距离依次配对
    paired_points = []
    paired_indices = set()
    for pair in pairs:
        index1 = foot_center.index(pair[0])
        index2 = foot_center.index(pair[1])

        if index1 not in paired_indices and index2 not in paired_indices:
            paired_points.append((pair[0], pair[1]))
            paired_indices.add(index1)
            paired_indices.add(index2)

    # 添加未配对的点
    unmatched_points = [point for i, point in enumerate(foot_center) if i not in paired_indices]
    return paired_points, unmatched_points


def getLinearPara(point):
    p1, p2 = point[0], point[1]
    x1, x2, y1, y2 = p1[0], p2[0], p1[1], p2[1]
    k = (y2 - y1) / (x2 - x1)
    b = y2 - k * x2
    return [round(k, 2), round(b, 2)]


def h():
    print("rospy has shutdown")


class footsteps_detection(object):
    def __init__(self, tp):
        self.tp = tp
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name="脚步分离",
                          width=1700, height=900,
                          left=50, top=50)
        self.smapCloud = o3d.io.read_point_cloud(smap_cloud_path, format='xyz')
        """去除地图中的离群点"""
        self.smapCloud = remove_mapPoint(self.smapCloud)
        self.xmapCloud = o3d.io.read_point_cloud(xmap_cloud_path, format='xyz')
        self.smapCloud.transform(tp)
        self.xmapCloud = remove_mapPoint(self.xmapCloud)
        self.pcd = o3d.geometry.PointCloud()
        self.pcd.points = o3d.utility.Vector3dVector()
        self.track_pcd = o3d.geometry.PointCloud()
        self.track_pcd.points = o3d.utility.Vector3dVector()
        self.mot_tracker = Sort(max_age=5, min_hits=2, dis_threshold=800)

        # 求解各区域边界线
        boundary_points = [[[200, 84], [180, -180]], [[-92, 56], [-67, -220]], [[360, -18], [-410, -140]]]
        self.boundary_line = [getLinearPara(point) for point in boundary_points]

        # 本类作为observable添加注册observer
        self.lc = light_control()
        self.dt = danger_detect()

        t = threading.Thread(target=self.run)
        t.setDaemon(True)
        t.start()
        self.subscribe()

    def subscribe(self):
        rospy.init_node("subscriber")
        sub1 = message_filters.Subscriber('/scan1', LaserScan)
        sub2 = message_filters.Subscriber('/scan2', LaserScan)

        ats = message_filters.ApproximateTimeSynchronizer([sub1, sub2], 10, 0.1, allow_headerless=True)
        ats.registerCallback(self.callback)
        rospy.spin()

    def callback(self, msg1, msg2):
        rounded_ranges1 = [int(round(float(x), 2) * 100) for x in msg1.ranges]
        rounded_ranges2 = [int(round(float(x), 2) * 100) for x in msg2.ranges]

        self.vis.remove_geometry(self.smapCloud)
        # self.vis.remove_geometry(self.xmapCloud)
        self.vis.remove_geometry(self.pcd)
        self.vis.remove_geometry(self.track_pcd)

        get_footPoints(rounded_ranges1, sall_cloud_path)
        get_footPoints(rounded_ranges2, xall_cloud_path)

        # 从 sall_cloud_path 中提取纯脚步点，保存到 sfoot_cloud_path。
        """sfoot_cloud_path: 只包含脚步点的cloud数据"""
        filter(smap_cloud_path, sall_cloud_path, sfoot_cloud_path)
        filter(xmap_cloud_path, xall_cloud_path, xfoot_cloud_path)

        # sfoot_cloud_path 转换为 o3d 的点云文件 footCloud
        sfootCloud = o3d.io.read_point_cloud(sfoot_cloud_path, format='xyz')
        xfootCloud = o3d.io.read_point_cloud(xfoot_cloud_path, format='xyz')
        sfootCloud.transform(tp)
        sfootArray = np.array(sfootCloud.points)
        xfootArray = np.array(xfootCloud.points)

        footArray = np.concatenate((sfootArray, xfootArray))

        # 从纯脚步点聚类区分不同脚步
        foot_labels = footstep_cluster(footArray)

        num_clusters = max(foot_labels) + 1
        colors = plt.cm.jet(np.linspace(0, 1, num_clusters))
        colors = colors[:, :3]

        cluster_indices = [i for i, label in enumerate(foot_labels) if label != -1]
        clustered_points = footArray[cluster_indices]

        dets = []
        for i in range(num_clusters):
            label_indices = np.where(foot_labels == i)[0]
            det = np.round(np.mean(footArray[label_indices], axis=0)[:2], 2).tolist()
            dets.append(det)

        # 匹配不同脚步
        paired_points, unmatched_points = pair_points(dets)
        if paired_points and unmatched_points:
            dets = np.concatenate((np.mean(paired_points, axis=1), np.array(unmatched_points)), axis=0)
        elif (not paired_points) and unmatched_points:
            dets = unmatched_points
        elif paired_points and (not unmatched_points):
            dets = np.mean(paired_points, axis=1)
        else:
            dets = []

        trackers = self.mot_tracker.update(np.asarray(dets))
        trackers_array = np.array(trackers)

        # 为检测点划分区域
        target_area = set()
        for tracker in trackers_array:
            if tracker[1] < self.boundary_line[0][0] * tracker[0] + self.boundary_line[0][1]:
                if tracker[1] < self.boundary_line[2][0] * tracker[0] + self.boundary_line[2][1]:
                    target_area.add(2)
                else:
                    target_area.add(1)
            else:
                if tracker[1] > self.boundary_line[1][0] * tracker[0] + self.boundary_line[1][1]:
                    if tracker[1] > self.boundary_line[2][0] * tracker[0] + self.boundary_line[2][1]:
                        target_area.add(3)
                    else:
                        target_area.add(4)
                else:
                    if tracker[1] > self.boundary_line[2][0] * tracker[0] + self.boundary_line[2][1]:
                        target_area.add(5)
                    else:
                        target_area.add(6)
        print(target_area)

        if isDark:
            self.lc.update(target_area)
        self.dt.update(target_area)

        trackers_with_zeros = np.column_stack([trackers_array[:, :2], np.zeros(len(trackers_array))])
        self.track_pcd.points = o3d.utility.Vector3dVector(trackers_with_zeros)
        self.track_pcd.colors = o3d.utility.Vector3dVector([np.array([1, 0, 0])] * len(trackers))

        self.pcd.points = o3d.utility.Vector3dVector(clustered_points)
        if num_clusters != 0:
            self.pcd.colors = o3d.utility.Vector3dVector(colors[foot_labels[cluster_indices]])

        self.vis.add_geometry(self.track_pcd)
        self.vis.add_geometry(self.pcd)
        self.vis.add_geometry(self.smapCloud)
        # self.vis.add_geometry(self.xmapCloud)
        self.vis.poll_events()  # 注册更新操作
        self.vis.update_renderer()  # 渲染图窗

    def on_press(self, key):
        try:
            if key.char == 'q':
                rospy.on_shutdown(h)
        except AttributeError:
            print('special key {0} pressed'.format(key))

    def run(self):
        while (True):
            with keyboard.Listener(on_press=self.on_press) as listener:
                listener.join()


class area_func(metaclass=ABCMeta):
    @abstractmethod
    def update(self, target_area):
        pass


class light_control(area_func):
    def __init__(self):
        self.target_light = set()
        self.light_state = [False, False, False]

    def update(self, target_area):
        new_target = set()
        lights_location = [{1, 2}, {3, 4}, {5, 6}]
        for i in range(len(lights_location)):
            if not target_area.isdisjoint(lights_location[i]):
                new_target.add(i)
        turn_off = self.target_light.difference(new_target)
        turn_on = new_target.difference(self.target_light)
        self.target_light = new_target
        if turn_off:
            for i in turn_off:
                socket_list[i].send(b'0')
        if turn_on:
            for i in turn_on:
                socket_list[i].send(b'1')


class danger_detect(area_func):
    def __init__(self):
        self.dangerous_area = {2}
        self.last_send = time.time() - 60

    def update(self, target_area):
        if not target_area.isdisjoint(self.dangerous_area):
            send_time = time.time()
            if send_time - self.last_send > 60:
                message = MIMEText('您的家人在危险区域，请及时采取应急措施!', 'plain', 'utf-8')
                message['Subject'] = '危险区域通知'
                message['From'] = sender
                message['To'] = receivers[0]
                try:
                    smtpObj = smtplib.SMTP()
                    smtpObj.connect(mail_host, 25)
                    smtpObj.login(mail_user, mail_pass)
                    smtpObj.sendmail(
                        sender, receivers, message.as_string())
                    smtpObj.quit()
                    print('success')
                except smtplib.SMTPException as e:
                    print('error', e)
            self.last_send = send_time


if __name__ == "__main__":
    # 初始化tcp server
    tcp_server_socket = socket(AF_INET, SOCK_STREAM)
    tcp_server_socket.setsockopt(SOL_SOCKET, SO_REUSEADDR, True)
    address = ('192.168.119.130', int(5678))
    print(gethostbyname(gethostname()))
    tcp_server_socket.bind(address)
    tcp_server_socket.listen(8)
    socket_temp = []
    socket_seq = []
    new_tcp = threading.Thread(target=tcplink)
    new_tcp.start()

    # 旋转矩阵 ICP配准后获得
    tp = [[-0.93009428, 0.36732089, 0., -32.88565462],
          [-0.36732089, -0.93009428, 0., -140.56781704],
          [0., 0., 1., 0.],
          [0., 0., 0., 1.]]
    while client_num != 4:
        time.sleep(5)
    for i in range(3):
        for j in range(3):
            if socket_seq[j] == i:
                socket_list.append(socket_temp[j])
    detection = footsteps_detection(tp)
