import numpy as np
from . import SendData, DroneInfo
from .base import pid, base, const
from .missile import missile
from collections import defaultdict

table = (
    (np.linspace(0, 18e3, 7), (0.35, 0.5, 0.65, 0.8, 1, 1.15, 1.25)),  # 高度(m)
    (np.linspace(0.5, 1.5, 5), (0.7, 0.9, 1, 1.15, 1.25)),  # 马赫
    (np.linspace(0, np.pi, 7), (1.3, 1.2, 1, 0.8, 0.6, 0.45, 0.35)),  # 相对方位角(rad)
)
MissileTrackList = defaultdict(list[missile])  # 已射导弹列表


class drone(base):
    def __init__(self, root):
        super().__init__(root)
        self.mislist = []  # 剩余导弹列表(missile子类)
        self.AlarmList = []  # 告警列表(辐射源,相对方位,类型)
        self.FoundEnemyList = []  # 发现敌机列表(EnemyInfo)
        self.atrim = 0.0  # 配平攻角(rad)
        self.engage = 0  # -1=火控锁定,1=发射导弹,从0变化有效
        self.pitch2ele = self.mach2thr = pid(0, 0, 0)  # -俯仰角->升降舵,马赫->油门

    def step(self, cmd: SendData, dt1=0.5, dt2=3.0):
        """
        dt1:float,转弯时间(s)
        dt2:俯仰时间
        """
        info = next((x for x in self.FoundEnemyList if x.EnemyID == cmd.EnemyID), None)
        if not self.engage and info:
            if -1 == cmd.engage:
                info.isNTS = True  # 火控锁定
            elif (
                1 == cmd.engage  # 发射指令
                and self.mislist  # 有导弹
                and info.isNTS  # 已锁定
                and info.TargetDis <= info.MissileMaxDis  # 射程内
            ):
                MissileTrackList[self.ID].append(self.mislist.pop(0)(self, info))
        self.engage = cmd.engage
        dyaw = const(np.deg2rad(cmd.CmdHeadingDeg) - self.Yaw)  # 航向误差
        if np.sign(cmd.TurnDirection * dyaw) < 0:
            dyaw = cmd.TurnDirection * (2 * np.pi - abs(dyaw))  # 指定转向
        elif 175 < np.rad2deg(abs(dyaw)):
            dyaw = abs(dyaw)  # 正后方附近强制右转
        aileron = self.BTT(dyaw / dt1, np.deg2rad(cmd.CmdPhi))
        pitch = min(np.pi / 2, np.deg2rad(abs(cmd.CmdPitchDeg)))
        cr = np.cos(self.Roll)
        if cr:
            V = np.linalg.norm(self.vel)
            Vu = np.clip((cmd.CmdAlt - self.Altitude) / dt2, -V, V)  # 期望爬升速度
            pitch = np.clip(np.arcsin(Vu / V) + self.atrim / cr, -pitch, pitch)
        else:  # 只能转弯
            pitch = np.clip(np.sign(self.Roll) * dyaw, -pitch, pitch)
        elevator = self.pitch2ele.update(bool(cr) * self.Pitch - pitch)
        # 油门
        self.mach2thr.limit = 0.1, np.clip(cmd.ThrustLimit / 129, 0.1, 1)
        throttle = self.mach2thr.update(cmd.CmdSpd - self.Mach_M)
        super().step(aileron, elevator, throttle)

    def alarm(self, AlarmID, MisAzi, AlarmType):
        """
        AlarmID:int,辐射源ID
        MisAzi:float,相对方位角(rad)
        AlarmType:str,告警类型
        """
        self.AlarmList.append((AlarmID, MisAzi, AlarmType))

    def detect(self, target: base, isNTS, team):
        # team:bool,是否同队
        info = super().detect(target)
        if info:
            if not team and isinstance(target, drone):
                info.isNTS = isNTS
                for (xp, fp), x in zip(table[:2], (self.Altitude, self.Mach_M)):
                    x = np.interp(x, xp, fp)
                    info.MissilePowerfulDis *= x
                    info.MissileMaxDis *= x
                # 马赫反比于NEZ
                info.MissilePowerfulDis *= max(0.5, 1 - info.TargetMach_M / 10)
                # 攻角影响射程
                info.MissileMaxDis *= np.interp(abs(info.TargetYaw), *table[2])
                self.FoundEnemyList.append(info)
                # 导弹数据链
                for misl in MissileTrackList[self.ID]:
                    if info.EnemyID == misl.TargetID and misl.state < 2:
                        misl.radar.recv(info.EnemyID, *self.radar.data[info.EnemyID])
            if info.TargetDis < 2e4:
                self.alarm(info.EnemyID, info.TargetYaw, type(target).__name__)

    def getinfo(self):
        info = DroneInfo(self)
        info.strike = [x.TargetID for x in MissileTrackList[self.ID] if x.state == 2]
        return info


class f16(drone):
    def __init__(self, lat, lon, alt, mach, mislist, head, pitch, roll, fuel):
        super().__init__(None)
        self.fdm.load_model(type(self).__name__)
        self.mislist = list.copy(mislist)
        self.atrim = np.deg2rad(5)
        self.radar.r = 14e4
        self.radar.rad = np.pi / 3
        self.roll2ail = pid(3, 0.05, 0.1)
        self.pitch2ele = pid(3, 0.02, 0.25)
        self.mach2thr = pid(2, 0.08, 0.08)
        super().ready(lat, lon, alt, mach, head, pitch, roll, fuel)
