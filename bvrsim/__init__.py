from os import system
from sys import executable
from importlib.util import find_spec

if not find_spec("jsbsim"):  # jsbsim依赖numpy
    system(
        f"{executable} -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple jsbsim"
    )
from numpy import zeros


class EnemyInfo:  # 敌机带噪信息
    def __init__(self):
        self.EnemyID = 0  # 敌机ID
        self.isNTS = False  # 火控是否锁定
        self.TargetDis = 0.0  # 敌机距离(m)
        self.DisRate = 0.0  # 径向速度(m/s)
        self.TargetYaw = self.TargetPitch = 0.0  # 水平,纵向视线角(rad)
        self.vel = zeros(3, float)  # NED速度向量(m/s)
        self.TargetMach_M = 0.0  # 马赫
        self.MissilePowerfulDis = 4e4  # 不可逃逸区(m)
        self.MissileMaxDis = 8e4  # 射程(m)


class DroneInfo:  # 本机精确信息
    def __init__(self, drone):
        self.DroneID = drone.ID  # 本机ID
        self.Latitude = drone.Latitude  # 纬度(rad)
        self.Longitude = drone.Longitude  # 经度(rad)
        self.Altitude = drone.Altitude  # 高度(m)
        self.Yaw = drone.Yaw  # 航向角(rad)
        self.Pitch = drone.Pitch  # 俯仰角(rad)
        self.Roll = drone.Roll  # 滚转角(rad)
        self.V_N, self.V_E, self.V_D = drone.vel  # 北,东,地向速度(m/s)
        self.A_N, self.A_E, self.A_D = drone.acc  # 北,东,地向惯性加速度(m/s^2)
        self.Mach_M = drone.Mach_M  # 马赫
        self.fuel = drone.fuel  # 当前燃油(lbs)
        self.AlarmList = drone.AlarmList  # 告警列表(辐射源,相对方位,类型)
        self.FoundEnemyList = drone.FoundEnemyList  # 发现敌机列表(EnemyInfo)
        self.strike = []  # 导弹击中实例ID列表
        self.MissileNowNum = len(drone.mislist)  # 当前导弹数量


class SendData:  # 无人机控制信息
    def __init__(self):
        self.CmdSpd = 0.0  # 期望马赫
        self.CmdAlt = self.CmdHeadingDeg = 0.0  # 期望高度(m),绝对方位角(deg)
        self.CmdPitchDeg = self.CmdPhi = 20.0  # 最大俯仰,滚转角(deg)
        self.TurnDirection = 0  # 0=就近转,1=右转,-1=左转
        self.ThrustLimit = 129.0  # 推力限制(kN)
        self.engage = 0  # -1=火控锁定,1=发射导弹,从0变化有效
        self.EnemyID = 0  # 目标敌机ID


from .bvrsim import bvrsim  # 必须末尾
