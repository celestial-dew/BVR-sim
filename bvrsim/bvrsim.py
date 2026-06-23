import numpy as np
import multiprocessing as mu
from . import SendData, DroneInfo
from .base import base
from .drone import f16, drone, MissileTrackList
from .visual import tacview
from .missile import aim120c, missile
from os.path import splitext
from itertools import count
from collections import defaultdict
from collections.abc import Callable
from multiprocessing.synchronize import Lock

com = dict(drone=f16, alt=1e4, mach=0.8, pitch=0, roll=0, fuel=5e3)
red = []
blue = []
for i in range(4):  # 基于SH-11蓝方策略1
    red.append(com | dict(lat=25.8, lon=118.2 + i / 5, head=180, mislist=4 * [aim120c]))
    blue.append(com | dict(lat=23.2, lon=118.2 + i / 5, head=0, mislist=6 * [aim120c]))


def consumer(strategy, get: mu.Queue, put: mu.Queue):  # 消费者
    while True:
        data = get.get()
        put.put((data[0].DroneID, strategy(*data)))


class spatialgrid:  # 均匀网格优化邻近点查找
    def __init__(self, size):
        self.size = size
        self.grid = defaultdict(list)

    def clear(self):
        self.grid.clear()

    def hash(self, pos: np.ndarray):
        return tuple((pos // self.size).astype(int))

    def add(self, ID, pos):
        self.grid[self.hash(pos)].append(ID)

    def dfs(self, key, dim, near: list):
        if len(key) == dim:
            return near.extend(self.grid[tuple(key)])
        for d in range(-1, 2):
            key[dim] += d
            self.dfs(key, 1 + dim, near)
            key[dim] -= d  # 恢复现场

    def getnear(self, pos):
        near = []
        self.dfs(list(self.hash(pos)), 0, near)
        return near


class bvrsim:
    def __init__(
        self,
        lock: Lock,  # tacview实例化互斥锁
        field=[(23, 26), (118, 120), (2e3, 15e3)],  # list,战场纬,经,高度范围(deg,m)
        threat=[(24.5, 119, 0, 5e4)],  # list,红方威胁区列表(纬,经度deg,高度,半径m)
        time=30.0,  # float,超实时仿真最长用时(min)
    ):
        self.lock = lock
        self.field = np.vstack((np.deg2rad(field[:2]), field[2]))
        self.O = np.mean(self.field, 1)  # 战场中心(原点)
        self.threat = [np.hstack((np.deg2rad(x[:2]), x[2:])) for x in threat]
        self.time = 60 * time  # 转秒
        self.entity: dict[int, tuple[base, str]] = {}

    def restrict(self):  # 检查是否有效
        dead = set()
        for ID, (entity, color) in self.entity.items():
            if isinstance(entity, drone) and not entity.fuel:  # 飞机无燃料
                dead.add(ID)
                continue
            for x, (m, M) in zip(
                (entity.Latitude, entity.Longitude, entity.Altitude), self.field
            ):
                if x < m or M < x:  # 飞出战场
                    dead.add(ID)
                    break
            if "Red" == color and not ID in dead:
                for lat, lon, alt, r in self.threat:
                    if np.linalg.norm(entity.geo2ned(lat, lon, alt)) < r:  # 进入威胁区
                        dead.add(ID)
                        break
            elif isinstance(entity, missile) and ID in dead:  # 导弹失效
                entity.TargetID = 0
        return dead

    def strike(self):  # 检查是否碰撞
        grid = spatialgrid(15)
        for ID, (entity, _) in self.entity.items():
            entity.radar.update(entity.RM, -entity.geo2ned(*self.O))
            grid.add(ID, entity.radar.ned)
        near = {}  # 邻近点缓存
        r = defaultdict(dict)
        dead = set()
        for ID, (entity, _) in self.entity.items():  # 考虑连锁爆炸
            ned = entity.radar.ned
            key = grid.hash(ned)
            if not key in near:
                near[key] = grid.getnear(ned)
            for x in near[key]:
                if ID != x:
                    if not x in r[ID]:
                        pos = self.entity[x][0].radar.ned
                        r[ID][x] = r[x][ID] = np.linalg.norm(pos - ned)
                    if r[ID][x] < grid.size:
                        dead.add(ID)
                        if isinstance(entity, missile):  # 可能误截获,只记录1个目标
                            entity.state = 2
                            entity.TargetID = x
                        break
        return dead

    def detect(self):  # 感知更新
        for ID1, (entity, color1) in self.entity.items():
            if isinstance(entity, drone):
                isNTS = {x.EnemyID: x.isNTS for x in entity.FoundEnemyList}  # 上步锁定
                entity.FoundEnemyList.clear()
                entity.AlarmList.clear()
                for ID2, (target, color2) in self.entity.items():
                    if ID1 != ID2:
                        entity.detect(target, isNTS.get(ID2, False), color1 == color2)
            # 目标失效时导弹游离
            elif isinstance(entity, missile) and entity.TargetID in self.entity:
                entity.detect(self.entity[entity.TargetID][0])

    def update(self, queue: mu.Queue, launch, log: tacview, file, time):
        mislist = []  # 新发射导弹列表
        for entity, _ in self.entity.values():  # 与策略进程并行
            if isinstance(entity, missile):
                old = entity.state
                entity.step()
                if not old and entity.state:
                    log.loginit(entity, "White", "AIM-120C")  # 主动雷达开启
        if 500 < len(log.buffer):
            log.flush()
        for entity, _ in self.entity.values():
            if isinstance(entity, drone):
                ID, cmd = queue.get()  # 空则阻塞
                entity, color = self.entity[ID]  # 实际飞机
                if isinstance(entity, drone):
                    entity.step(cmd)
                    for enemy in entity.FoundEnemyList:
                        if enemy.isNTS:
                            log.logNTS(ID, enemy.EnemyID)  # 持续显示锁定
                if 1 == cmd.engage and MissileTrackList[ID]:
                    misl = MissileTrackList[ID][-1]
                    if not misl.ID in self.entity and misl.TargetID and misl.state < 2:
                        mislist.append(misl)
                        launch[color] += 1
                        print(time, ":", vars(entity), "发射", vars(misl), file=file)
                        log.loginit(misl, "White", "AIM-120C", 0)
            log.logstep(entity)
        for misl in mislist:
            self.entity[misl.ID] = misl, "White"

    def run(
        self,
        redstrategy: Callable[[DroneInfo, int], SendData],  # 红方策略函数
        bluestrategy=None,  # 蓝方策略函数,None则红方
        red=red,  # list,红方飞机初始参数列表
        blue=blue,  # 蓝方飞机初始参数列表
        exist_ok=False,  # bool,是否覆盖最近acmi文件,True可能覆盖并行acmi文件
        file=None,  # True|None,是否print到文件,None则终端
    ):
        base.auto = count(1).__next__
        self.entity.clear()
        MissileTrackList.clear()
        queue = [mu.Queue() for _ in range(3)]
        mu.Process(
            target=consumer, args=[redstrategy] + queue[::2], daemon=True
        ).start()  # 红方沙盒
        mu.Process(
            target=consumer, args=[bluestrategy or redstrategy] + queue[1:], daemon=True
        ).start()  # 蓝方沙盒
        with self.lock:
            log = tacview("output", exist_ok)
        if file:
            file = open(splitext(log.file.name)[0] + ".txt", "w")
        log.logtime(0)
        for color, lst in (("Red", red), ("Blue", blue)):  # 双方初始化
            for kwargs in lst:
                cls = kwargs.pop("drone")
                entity = cls(**kwargs)
                kwargs["drone"] = cls
                if "Blue" == color:
                    entity.radar.r *= 4 / 3  # 蓝方雷达距离优势
                self.entity[entity.ID] = entity, color
                log.loginit(entity, color)
        exist = {"Red": len(red), "Blue": len(blue)}
        launch = {x: 0 for x in exist}
        for step_num, time in ((x, base.dt * x) for x in count()):  # 主循环,aero.dt
            log.logtime(time)
            for event, dead in (("失效", self.restrict()), ("受撞", self.strike())):
                for ID in dead:
                    entity, color = self.entity.pop(ID)
                    if color in exist:
                        exist[color] -= 1
                    print(time, ":", vars(entity), event, file=file)
                    log.logdestroy(ID)
            if not all(exist.values()) or self.time < time:
                break
            if not step_num % 3:  # 雷达扫描周期
                self.detect()
            for entity, color in self.entity.values():  # 飞机信息发送
                entity.radar.step()
                if isinstance(entity, drone):
                    i = next(i for i, x in enumerate(exist) if x == color)
                    queue[i].put((entity.getinfo(), step_num))
            self.update(queue[2], launch, log, file, time)
        print(
            f"博弈用时:{time//60:.0f} min {time%60:.3f} s\n无人机剩余:{exist}\n发射导弹数:{launch}\n数据文件:{log.file.name}",  # type:ignore
            file=file,
        )
        if file:
            file.close()

    def start(self, redstrategy, bluestrategy=None, red=red, blue=blue):  # 并行仿真
        self.entity.clear()  # 避免fdm序列化报错
        pro = mu.Process(
            target=self.run, args=(redstrategy, bluestrategy, red, blue, False, True)
        )
        pro.start()
        return pro
