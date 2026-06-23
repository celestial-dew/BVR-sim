import numpy as np
from . import EnemyInfo
from .aero import g, aero
from .radar import const, radar, los2xyz, xyz2los
from itertools import count

R0 = 6371e3  # 地球半径(m)


class pid:  # 离散PID控制
    def __init__(self, kp, ki, kd, limit=(-1, 1)):
        """
        kp:float,比例增益,纠正强度
        ki:float,积分增益,惩罚累积误差
        kd:float,微分增益,提前刹车
        limit:tuple,输出限幅
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.limit = limit
        self.error = self.summa = self.diff = 0
        self.dt = aero.dt

    def update(self, error, N=1.0):
        """
        error:float,当前误差
        N:float,等效平滑拍数
        """
        m, M = self.limit
        if self.kd:  # 1阶低通滤波
            self.diff = (N * self.diff + (error - self.error) / self.dt) / (1 + N)
            # 微分限幅
            self.diff = np.clip(self.diff, m / self.kd, M / self.kd)
        self.error = error
        out = self.kp * error + self.ki * self.summa + self.kd * self.diff
        clip = np.clip(out, m, M)
        if self.ki and np.sign(out - clip) != np.sign(error):  # 未饱和/误差试图退出饱和
            # 积分抗饱和
            self.summa = np.clip(self.summa + error * self.dt, m / self.ki, M / self.ki)
        return clip


class base(aero):
    auto = count(1).__next__  # 避免子类ID副本

    def __init__(self, root):
        super().__init__(root)
        self.ID = type(self).auto()
        self.radar = radar(type(self).dt, 0.0, 0.0)
        self.roll2ail = pid(0, 0, 0)  # 滚转角->副翼

    def BTT(self, r, M=np.pi / 2):  # 滚转转弯
        """
        r:float,NED系期望偏航角速度(rad/s)
        M:float,滚转角绝对值限幅(rad)
        """
        aileron = np.pi / 2 != abs(self.Pitch)
        if aileron:
            M = min(np.pi / 2, abs(M))
            # 期望滚转角
            roll = np.clip(np.arctan2(np.linalg.norm(self.vel[:2]) * r, g), -M, M)
            aileron = self.roll2ail.update(roll - self.Roll)
        return aileron

    def geo2ned(self, lat, lon, alt):  # 经纬高->NED向量
        R = R0 + self.Altitude
        r = R * np.cos(self.Latitude)
        lat -= self.Latitude
        lon -= self.Longitude
        return np.array((R * lat, r * lon, self.Altitude - alt))

    def ned2geo(self, ned):
        R = R0 + self.Altitude
        r = R * np.cos(self.Latitude)
        lat = np.clip(self.Latitude + ned[0] / R, -np.pi / 2, np.pi / 2)
        lon = const(self.Longitude + (ned[1] / r if r else 0))
        return lat, lon, self.Altitude - ned[2]

    def geo2los(self, lat, lon, alt, RM=False):  # 经纬高->方位,俯仰角
        ned = self.geo2ned(lat, lon, alt)
        return xyz2los(self.RM @ ned if RM else ned)

    def los2geo(self, r, yaw, pitch, RM=False):
        xyz = los2xyz(r, yaw, pitch)
        return self.ned2geo(self.RM.T @ xyz if RM else xyz)

    def detect(self, target):
        if self.radar.detect(target.ID, target.radar.ned):
            pos, vel = self.radar.get(target.ID)[:2]
            pos -= self.radar.ned
            info = EnemyInfo()
            info.EnemyID = target.ID
            info.TargetDis, info.TargetYaw, info.TargetPitch = xyz2los(self.RM @ pos)
            info.DisRate = np.dot(vel - self.vel, pos) / info.TargetDis
            info.vel = vel
            info.TargetMach_M = float(np.linalg.norm(vel)) / 303.77  # 9km声速(m/s)
            return info
