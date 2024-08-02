import numpy as np


class KalmanFilter(object):
    # stateVariance 状态方差
    # measurementVariance 测量方差
    def __init__(self, dt=0.25, stateVariance=1, measurementVariance=0.05,
                 method="none"):
        super(KalmanFilter, self).__init__()
        self.method = method
        self.stateVariance = stateVariance
        self.measurementVariance = measurementVariance
        self.dt = dt
        self.initModel()

    def initModel(self):
        # [x,x_v,y,y_v]x,x方向的速度，y，y方向的速度
        # 有加速度的情况 A，B都有用
        if self.method == "Accerelation":
            # 加速度值
            self.U = 1
        # 匀速直线运动的时候，B没有用
        else:
            self.U = 0
        # 状态转移矩阵
        self.A = np.matrix([[1, self.dt, 0, 0], [0, 1, 0, 0],
                            [0, 0, 1, self.dt], [0, 0, 0, 1]])
        # 控制增益
        self.B = np.matrix([[self.dt ** 2 / 2], [self.dt], [self.dt ** 2 / 2],
                            [self.dt]])
        # 量测矩阵
        self.H = np.matrix([[1, 0, 0, 0], [0, 0, 1, 0]])
        # 初始预测协方差矩阵
        self.P = np.matrix(self.stateVariance * np.identity(self.A.shape[0]))  # np.identity:输出对角线为1，其余为0的n*n矩阵
        # 测量噪声协方差
        self.R = np.matrix(self.measurementVariance * np.identity(  # 2*2
            self.H.shape[0]))
        # 过程激励噪声协方差（系统噪声）
        self.Q = np.matrix([[self.dt ** 4 / 4, self.dt ** 3 / 2, 0, 0],
                            [self.dt ** 3 / 2, self.dt ** 2, 0, 0],
                            [0, 0, self.dt ** 4 / 4, self.dt ** 3 / 2],
                            [0, 0, self.dt ** 3 / 2, self.dt ** 2]])
        # print(self.A.shape)

        # 初始值设置
        self.erroCov = self.P
        self.state = np.array([[300], [0], [30], [0]])

    def predict(self):
        # 第一步，第二步，估计当前状态和当前协方差
        self.predictedState = self.A * self.state + self.U * self.B  # U为控制量，因为是匀速直线运动，所以U为0，U和B都没有用
        self.predictedErrorCov = self.A * self.erroCov * self.A.T + self.Q
        temp = np.asarray(self.predictedState).ravel()
        return np.array([temp[0], temp[2]])


    def correct(self, currentMeasurement):
        # 三四五步，计算卡尔曼增益，预测最优当前状态和当前协方差
        self.kalmanGain = self.predictedErrorCov * self.H.T * np.linalg.pinv(
            self.H * self.predictedErrorCov * self.H.T + self.R)
        self.state = self.predictedState + self.kalmanGain * (currentMeasurement
                                                              - (self.H * self.predictedState))
        self.erroCov = (np.identity(self.P.shape[0]) -
                        self.kalmanGain * self.H) * self.predictedErrorCov
        temp = np.asarray(self.state).ravel()
        return np.array([temp[0], temp[2]])
