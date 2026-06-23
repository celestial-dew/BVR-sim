import numpy as np
from . import EnemyInfo
from .base import pid, base, const, los2xyz, xyz2los
from os.path import dirname
from glob import glob


def Jinv(yaw, pitch):  # 视线角速度->体轴角速度的雅可比逆矩阵
    sy = np.sin(yaw)
    cy, cp = np.cos((yaw, pitch))
    return np.array(((cy, sy), (-sy, cy))) @ np.diag((1, cp))


class ISM:  # 积分滑模控制
    def __init__(self, yaw, pitch, ki, rho, ka):
        """
        yaw:float,初始相对方位角(rad)
        pitch:初始相对俯仰角
        ki:float,积分增益,影响收敛速度与鲁棒性(1/s)
        rho:float,线性趋近项增益(1/s)
        ka:float,非线性切换项增益,克服扰动(rad/s)
        """
        self.yaw, self.pitch = np.clip((yaw, pitch), -np.pi / 3, np.pi / 3)
        self.ki = ki
        self.rho = rho
        self.ka = ka
        self.dt = base.dt
        self.summa = 0

    def update(self, RM, ned, pos, eps=0.05):  # 体轴y,z角速率指令
        # eps:float,tanh边界层厚度(rad)
        yaw, pitch = xyz2los(RM @ (pos - ned))[1:]
        # 角度误差
        error = np.array((const(yaw - self.yaw), pitch - self.pitch))
        if 175 < np.rad2deg(abs(error[0])):
            error[0] = abs(error[0])
        self.summa += error * self.dt
        sigma = error + self.ki * self.summa  # 积分滑模面
        # 连续化趋近律
        dot = -self.ki * error - self.rho * sigma - self.ka * np.tanh(sigma / eps)
        return Jinv(yaw, pitch) @ dot


class HOSM:  # 高阶滑模控制
    def __init__(self, yaw, pitch, alpha, beta):
        """
        alpha:float,Super-twisting第1增益,影响收敛速度(sqrt(rad)/s)
        beta:Super-twisting第2增益,要大于扰动导数上界(rad/s^2)
        """
        self.yaw, self.pitch = np.clip((yaw, pitch), -np.pi / 3, np.pi / 3)
        self.alpha = alpha
        self.beta = beta
        self.dt = base.dt
        self.summa = 0

    def update(self, RM, ned, pos):
        yaw, pitch = xyz2los(RM @ (pos - ned))[1:]
        # 角度误差
        error = np.array((const(yaw - self.yaw), pitch - self.pitch))
        if 175 < np.rad2deg(abs(error[0])):
            error[0] = abs(error[0])
        sign = np.sign(error)
        u = self.summa - self.alpha * sign * np.sqrt(np.abs(error))  # 辅助控制量
        self.summa -= self.beta * sign * self.dt
        return Jinv(yaw, pitch) @ u


class missile(base):
    def __init__(self, drone: base, info: EnemyInfo):
        """
        drone:载机实例
        info:目标信息
        """
        super().__init__("." if glob("aircraft/*") else dirname(dirname(__file__)))
        self.state = 0  # 0=中制导,1=末制导,2=已击中
        self.TargetID = info.EnemyID  # 0则游离
        self.q2ele = pid(0, 0, 0)  # -俯仰角速度->升降舵
        self.radar.update(drone.RM, drone.radar.ned)
        pos = drone.radar.ned + drone.RM.T @ los2xyz(
            info.TargetDis, info.TargetYaw, info.TargetPitch
        )  # 目标估计坐标
        self.radar.recv(info.EnemyID, pos, None)  # 初始化无需R

    def APN(self):  # 增广比例引导法生成NED惯性加速度指令
        pos, vel, acc = self.radar.get(self.TargetID)
        if not any(vel):
            return np.zeros(3)
        pos -= self.radar.ned  # 转相对坐标
        r = float(np.linalg.norm(pos)) or 1e-3
        if not self.state and r <= self.radar.r:  # 进入末制导
            self.state = 1
        pos /= r  # 转单位向量
        vel -= self.vel  # 转相对速度
        tgo = max(0.5, r / max(1e-3, np.dot(-vel, pos)))  # 避免近端奇异
        N = 4  # 比例基准值
        acmd = N * (vel - np.dot(vel, pos) * pos) / tgo  # PN中制导
        if self.state:  # 法向加速度补偿
            acmd += N / 2 * min(1, tgo / 2) * (acc - np.dot(acc, pos) * pos)
        return acmd

    def step(self):
        w = np.cross(self.vel, self.APN()) / np.dot(self.vel, self.vel)
        aileron = self.BTT(w[2])
        elevator = self.q2ele.update(self.q - (self.RM @ w)[1])
        super().step(aileron, elevator, 1)


class aim120c(missile):
    def __init__(self, drone: base, info: EnemyInfo):
        super().__init__(drone, info)
        self.fdm.load_model(type(self).__name__, False)
        self.radar.r = 2e4
        self.radar.rad = np.pi / 9  # 20deg
        self.roll2ail = pid(3, 0.05, 0.1)
        self.q2ele = pid(3, 1.5, 0.1)
        # 前方20m,避免炸到载机
        lat, lon, alt = drone.los2geo(20, 0, 0, True)
        lat, lon, head, pitch, roll = np.rad2deg(
            (lat, lon, drone.Yaw, drone.Pitch, drone.Roll)
        )  # 小仰角模拟高抛弹道
        super().ready(lat, lon, alt, drone.Mach_M, head, min(90, 5 + pitch), roll, 113)
