import numpy as np
from jsbsim import FGFDMExec

g = 9.80665  # 重力加速度
m2ft = lambda x: x / 0.3048  # 米->英尺
ft2m = lambda x: 0.3048 * x  # 英尺->米


class aero:
    dt = 0.01

    def __init__(self, root):
        # root:str,模型根目录,None则jsbsim
        self.fdm = FGFDMExec(root)
        self.fdm.set_debug_level(0)
        self.fdm.set_dt(type(self).dt)
        self.Latitude = self.Longitude = 0.0  # 纬,经度(rad)
        self.Altitude = 0.0  # 高度(m)
        self.Yaw = self.Pitch = self.Roll = 0.0  # 方位,俯仰,滚转角(rad)
        self.alpha = 0.0  # 攻角(rad)
        self.p = self.q = self.r = 0.0  # 滚转,俯仰,偏航角速度(rad/s)
        self.vel = np.zeros(3)  # NED速度向量(m/s)
        self.Mach_M = 0.0  # 马赫
        self.axyz = np.zeros(3)  # 体轴比力加速度向量(m/s^2)
        self.acc = np.zeros(3)  # NED惯性加速度向量(m/s^2)
        self.RM = np.zeros((3, 3))  # 体轴->NED的旋转矩阵
        self.fuel = 0.0  # 燃油(lbs)

    def rotate(self, flush):  # NED->体轴的旋转矩阵
        if flush:
            sy, sp, sr = np.sin((self.Yaw, self.Pitch, self.Roll))
            cy, cp, cr = np.cos((self.Yaw, self.Pitch, self.Roll))
            # 绕x轴转Roll
            Rx = np.array(((1, 0, 0), (0, cr, sr), (0, -sr, cr)))
            # 绕y轴转Pitch
            Ry = np.array(((cp, 0, -sp), (0, 1, 0), (sp, 0, cp)))
            # 绕z轴转Yaw
            Rz = np.array(((cy, sy, 0), (-sy, cy, 0), (0, 0, 1)))
            self.RM = Rx @ Ry @ Rz
        return self.RM

    def update(self):
        # 位置
        self.Latitude = self.fdm["position/lat-gc-rad"]
        self.Longitude = self.fdm["position/long-gc-rad"]
        self.Altitude = ft2m(self.fdm["position/h-sl-ft"])
        # 姿态
        self.Yaw = self.fdm["attitude/psi-rad"]
        self.Pitch = self.fdm["attitude/theta-rad"]
        self.Roll = self.fdm["attitude/phi-rad"]
        self.alpha = self.fdm["aero/alpha-rad"]
        self.p = self.fdm["velocities/p-aero-rad_sec"]
        self.q = self.fdm["velocities/q-aero-rad_sec"]
        self.r = self.fdm["velocities/r-aero-rad_sec"]
        # 速度
        self.vel[0] = ft2m(self.fdm["velocities/v-north-fps"])
        self.vel[1] = ft2m(self.fdm["velocities/v-east-fps"])
        self.vel[2] = ft2m(self.fdm["velocities/v-down-fps"])
        self.Mach_M = self.fdm["velocities/mach"]
        # 加速度
        self.axyz[0] = ft2m(self.fdm["accelerations/a-pilot-x-ft_sec2"])
        self.axyz[1] = ft2m(self.fdm["accelerations/a-pilot-y-ft_sec2"])
        self.axyz[2] = ft2m(self.fdm["accelerations/a-pilot-z-ft_sec2"])
        self.acc = self.rotate(True).T @ self.axyz
        self.acc[2] += g  # 转惯性加速度
        self.fuel = self.fdm["propulsion/total-fuel-lbs"]

    def ready(self, lat, lon, alt, mach, head, pitch, roll, fuel):
        """
        lat:float,初始纬度(deg)
        lon:初始经度
        alt:float,初始高度(m)
        mach:float,初始马赫
        head:float,初始方位角(deg)
        pitch:初始俯仰角
        roll:初始滚转角
        fuel:float,初始燃油(lbs)
        """
        if not self.fdm.get_model_name():
            raise AttributeError(f"{type(self)}未加载模型")
        self.fdm["ic/lat-gc-deg"] = lat
        self.fdm["ic/long-gc-deg"] = lon
        self.fdm["ic/h-sl-ft"] = m2ft(alt)
        self.fdm["ic/mach"] = mach
        self.fdm["ic/psi-true-deg"] = head
        self.fdm["ic/theta-deg"] = pitch
        self.fdm["ic/phi-deg"] = roll
        self.fdm["propulsion/tank/contents-lbs"] = fuel
        self.fdm.run_ic()
        self.fdm["gear/gear-cmd-norm"] = 0  # 收起起落架
        self.fdm["propulsion/engine/set-running"] = 1  # 启动引擎
        self.update()

    def step(self, aileron, elevator, throttle):
        """
        aileron:float,归一化副翼(-1~1)
        elevator:归一化升降舵,+下-上
        throttle:float,归一化油门(0~1)
        """
        aileron, elevator, throttle = np.clip((aileron, elevator, throttle), -1, 1)
        self.fdm["fcs/aileron-cmd-norm"] = aileron
        self.fdm["fcs/elevator-cmd-norm"] = elevator
        self.fdm["fcs/throttle-cmd-norm"] = max(0, throttle)  # 禁止反推
        self.fdm["fcs/rudder-cmd-norm"] = 0
        self.fdm.run()
        self.update()
