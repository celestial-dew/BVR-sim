from bvrsim import bvrsim, DroneInfo, SendData
from bvrsim.drone import f16
from bvrsim.missile import aim120c
from multiprocessing import Lock

# 战场空间：[(纬度范围 deg), (经度范围 deg), (高度范围 m)]
field = [(23.0, 26.0), (118.0, 120.0), (2000, 15000)]
# 红方防空威胁区列表：[(中心纬度 deg, 中心经度 deg, 高度 m, 半径 m)]
threat = [(24.5, 119.0, 0, 50000)]


def redstrategy(info: DroneInfo, step_num: int) -> SendData:  # 红方示例策略
    cmd = SendData()
    cmd.CmdSpd = 2
    cmd.CmdAlt = 12000
    cmd.CmdHeadingDeg = 180
    cmd.EnemyID = 3  # 攻击/锁定ID=3的敌机
    # 目标信息
    enemy = next((x for x in info.FoundEnemyList if x.EnemyID == cmd.EnemyID), None)
    if not step_num % 200:
        cmd.engage = -1
    elif step_num % 200 == 100 and enemy and enemy.TargetDis < 5e4:  # 保证发射距离
        cmd.engage = 1
    return cmd


def bluestrategy(info: DroneInfo, step_num: int) -> SendData:  # 蓝方示例策略
    cmd = SendData()
    cmd.CmdSpd = 1.2
    cmd.CmdAlt = 11000
    cmd.CmdHeadingDeg = 0
    cmd.EnemyID = 1
    enemy = next((x for x in info.FoundEnemyList if x.EnemyID == cmd.EnemyID), None)
    if not step_num % 100:
        cmd.engage = -1
    elif step_num % 100 == 50 and enemy and enemy.TargetDis < 55000:
        cmd.engage = 1
    return cmd


if __name__ == "__main__":
    com = dict(drone=f16, alt=1e4, mach=0.8, pitch=0, roll=0, fuel=5e3)  # 共同参数
    red = []  # 红方初始参数
    blue = []  # 蓝方初始参数
    for i in range(2):
        red.append(
            com | dict(lat=24, lon=118.2 + i / 5, head=180, mislist=4 * [aim120c])
        )
        blue.append(
            com | dict(lat=23.2, lon=118.2 + i / 5, head=0, mislist=6 * [aim120c])
        )
    lock = Lock()
    sim = bvrsim(lock, field, threat, 10)  # 超实时仿真 10 分钟
    sim.run(redstrategy, bluestrategy, red, blue)
    pro = []
    for _ in range(2):
        pro.append(sim.start(redstrategy, bluestrategy, red, blue))  # 并行仿真
    for p in pro:
        p.join()
