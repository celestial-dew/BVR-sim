import numpy as np
from functools import cache

const = lambda x, y=np.pi: np.mod(x + y, 2 * y) - y  # x约束在[-y,y]
# 20km距离(m),相对方位,俯仰角(rad)噪声标准差
std0 = np.array((80, *np.deg2rad((0.8, 1))))
# 随距离平方缩放
sigma = lambda x: np.clip((x / 2e4) ** 2, 1e-3, 5) * std0


@cache
def fac(n):  # n!
    return n * fac(n - 1) if 1 < n else 1


def expm(A, order=9):  # order阶Pade近似求矩阵指数
    P = np.eye(len(A))
    j = np.linalg.norm(A, 1)
    if j:
        j = max(0, 1 + np.frexp(j)[1])
        A1 = A / (1 << j)
        k = fac(order) / fac(order << 1)
        N = D = 0
        for i in range(1 + order):
            c = k * fac(2 * order - i) / fac(i) / fac(order - i)
            N += c * P
            D += (-(i & 1) | 1) * c * P
            P @= A1
        P = np.linalg.solve(D, N)
        for _ in range(j):
            P @= P
    return P


def psd(P):  # 确保正定
    lamda, alpha = np.linalg.eigh((P + P.T) / 2)
    return alpha @ np.diag(np.maximum(1e-9, lamda)) @ alpha.T


def xyz2los(xyz):  # 体轴向量->视线角/NED->方位,俯仰角
    r = np.linalg.norm(xyz, axis=0)
    yaw = np.arctan2(xyz[1], xyz[0])  # 相对方位角
    pitch = np.arcsin(-xyz[2] / np.maximum(1e-3, r))  # 相对俯仰角
    if np.ndim(r):
        i = r < 1e-3  # 近端奇异
        r[i] = 1e-3
        yaw[i] = pitch[i] = 0
    elif r < 1e-3:
        r = 1e-3
        yaw = pitch = 0
    return r, yaw, pitch


def los2xyz(r, yaw, pitch):
    sy, sp = np.sin((yaw, pitch))
    cy, cp = np.cos((yaw, pitch))
    return r * np.array((cp * cy, cp * sy, -sp))


class kalman:
    H = np.eye(3, 9)  # 观测矩阵

    def __init__(self, dt, pos, vel=3 * [0], acc=3 * [0], a=1.1, q=1e2):
        """
        dt:float,滤波周期(s)
        pos:np.ndarray,目标NED初始绝对坐标(m)
        vel:目标NED初始绝对速度(m/s)
        acc:目标NED初始绝对加速度(m/s^2)
        a:float,机动频率(1/s)
        q:float,连续过程噪声功率谱密度(m^2/s^5)
        """
        self.dt = dt
        self.X = np.hstack((pos, vel, acc))  # 状态空间
        self.P = np.diag(np.repeat((200, 20, 20), 3) ** 2)  # 状态估计误差协方差矩阵
        self.t = 0  # 丢失时间
        A = np.array(((0, 1, 0), (0, 0, 1), (0, 0, -a)))  # 单轴Singer模型
        # Van Loan方法
        M = np.block([[A, np.diag((0, 0, q))], [np.zeros((3, 3)), -A.T]])
        phi, Qp = np.hsplit(expm(M * dt)[:3], 2)
        self.F = np.kron(phi, np.eye(3))  # 状态转移矩阵
        self.Q = np.kron(Qp @ phi.T, np.eye(3))  # 过程噪声协方差矩阵

    def get(self):
        return np.split(self.X.copy(), 3)

    def predict(self):
        self.t += self.dt
        self.X = self.F @ self.X  # 状态预测
        self.P = psd(self.F @ self.P @ self.F.T + self.Q)  # 协方差预测

    @staticmethod
    def unscent(RM, ned, pos, std):  # 无迹变换生成协方差矩阵
        """
        RM:np.ndarray,自身NED->体轴旋转矩阵
        ned:np.ndarray,自身NED坐标
        pos:目标NED测量坐标
        std:np.ndarray,距离(m),相对方位,俯仰角(rad)标准差
        """
        lamda = 1
        d = np.sqrt(lamda + len(std)) * np.linalg.cholesky(psd(np.diag(std) ** 2))
        Z = np.reshape(xyz2los(RM @ (pos - ned)), (-1, 1)) + np.hstack(
            (np.zeros((len(std), 1)), d, -d)
        )  # 生成sigma点
        hat = ned.reshape(-1, 1) + RM.T @ los2xyz(*Z)
        wm = np.full(len(std) << 1 | 1, 0.5 / (lamda + len(std)))
        wm[0] *= 2 * lamda
        wc = wm.copy()
        wc[0] += 2  # 标准alpha=1,beta=2修正
        dZ = hat - np.average(hat, 1, wm).reshape(-1, 1)  # 加权中心化
        return psd((wc * dZ) @ dZ.T)  # 加权外积等效

    def update(self, pos, R):
        # R:np.ndarray,协方差矩阵
        self.t = 0
        # 卡尔曼增益
        K = np.linalg.solve(self.H @ self.P @ self.H.T + R, self.H @ self.P).T
        self.X += K @ (pos - self.H @ self.X)  # 状态更新
        # Joseph形式协方差更新
        I_KH = np.eye(9) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T


class radar:
    def __init__(self, dt, r, deg, cls=kalman, **kwargs):
        """
        r:float,探测距离(m)
        deg:float,探测半角(deg)
        cls:type,滤波类
        kwargs:cls构造函数额外参数
        """
        self.dt = dt
        self.r = r
        self.rad = np.deg2rad(deg)
        self.cls = cls
        self.kwargs = kwargs
        self.RM = np.zeros((3, 3))
        self.ned = np.zeros(3)
        self.data = {}  # 目标NED测量坐标,测量者协方差矩阵
        self.filter: dict[int, kalman] = {}

    def update(self, RM, ned):
        self.RM = RM
        self.ned = ned

    def get(self, ID):
        # ID,int,目标ID
        return self.filter[ID].get() if ID in self.filter else np.split(np.zeros(9), 3)

    def recv(self, ID, pos, R):  # 信息接受函数
        self.data[ID] = pos, R

    def detect(self, ID, pos):  # 模拟雷达探测
        # pos:np.ndarray,目标NED真实坐标
        r, yaw, pitch = xyz2los(self.RM @ (pos - self.ned))
        std = sigma(r)
        r, yaw, pitch = np.random.normal((r, yaw, pitch), std)  # 噪声注入
        r = max(1e-3, r)
        yaw = const(yaw)
        pitch = np.clip(pitch, -np.pi / 2, np.pi / 2)
        # 非线性噪声NED坐标
        pos = self.ned + self.RM.T @ los2xyz(r, yaw, pitch)
        if r <= self.r and np.cos(self.rad) <= np.cos((yaw, pitch)).prod():
            self.recv(ID, pos, self.cls.unscent(self.RM, self.ned, pos, std))
            return True
        return False

    def step(self):
        dead = set()
        for ID, filtering in self.filter.items():
            filtering.predict()
            if ID in self.data:
                filtering.update(*self.data[ID])
            elif 180 < filtering.t:  # 丢失3min
                dead.add(ID)
        for ID in dead:
            self.filter.pop(ID)
        for ID, (pos, _) in self.data.items():
            if not ID in self.filter:
                self.filter[ID] = self.cls(self.dt, pos, **self.kwargs)
        self.data.clear()
