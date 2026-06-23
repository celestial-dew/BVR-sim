# BVR-sim（天穹 $`\cdot`$ 空战演武）

*—— 高拟真蜂群无人机超视距空战仿真平台*

[![](https://img.shields.io/badge/Python-3.10+-blue?logo=python&logoColor=white)](https://www.python.org)
[![](https://img.shields.io/badge/Flight%20Dynamics-JSBSim-orange)](https://jsbsim.sourceforge.net)
[![](https://img.shields.io/badge/Reinforcement%20Learning-Validation-mediumpurple)](bvrsim/bvrsim.py)
[![](https://img.shields.io/badge/License-BSD%203--Clause-green)](LICENSE)

## 📜 项目简介

BVR-sim 是一个基于开源飞行动力学库 [JSBSim](https://github.com/JSBSim-Team/jsbsim) 构建，面向蜂群无人机超视距空战 **策略验证** 的高拟真仿真平台。其设计目标并非直接用于强化学习训练，而是检验已训练策略在复杂感知条件下的 **泛化鲁棒性**，以缩小仿真与现实的差距（**Sim‑to‑Real Gap**）。特别是经参数校准的 AIM-120C-5 空空导弹动力学模型，保留了高速度与有限转弯率之间的物理耦合，避免了强化学习训练环境中常见的过度理想化机动假设。

平台在物理层依托 JSBSim 保证气动与运动学可信；在感知层引入符合统计规律的测量噪声与感知受限，从而揭示理想仿真中不易察觉的 **性能退化**。同时集成 Tacview 兼容的 ACMI 日志，便于对交战全过程进行三维复盘。

本平台源于 **第十九届“挑战杯”全国大学生课外学术科技作品竞赛“揭榜挂帅”擂台赛** —— 中国航空工业集团沈阳飞机设计研究所发布的《复杂任务下无人机智能协同对抗算法》赛题。作者团队凭借稳定高效的算法代码，和严谨规范的设计文档，荣获该赛题 **一等奖**。赛后，作者基于开源飞行仿真框架与公开资料，独立重构了全部仿真代码，**不包含任何赛事涉密代码或内部数据**，将其开放为可扩展的空战仿真框架。

## 📦 主要特性

### 高拟真飞行动力学

- F-16 战斗机：调用 JSBSim 官方气动数据库，六自由度非线性求解；控制回路将期望速度、高度、航向等 **高层战术指令**，经稳态运动学公式映射为期望姿态角（协调转弯滚转角与爬升俯仰角），再由 PID 控制器驱动舵面与油门，控制增益经人工整定以适应超视距机动需求。
- AIM-120C-5 导弹：基于公开资料重构 `aircraft/aim120c.xml` 配置文件，包含无量纲气动系数、减面燃烧推力表、静不稳定系数、动压调度的角速度阻尼器，导弹可用过载受气动外形、推力与实时动压共同约束；制导指令生成与姿态控制均在 Python 端实现。

### 导弹制导与状态估计

- 高抛弹道：发射时在载机俯仰角上叠加额外爬升角，提升初始弹道高度，减小稠密大气层内的高动压速度损失，延长有效射程。
- 中制导：比例导引（PN），利用剩余飞行时间平滑指令，兼顾能量管理。
- 末制导：增广比例导引（APN），在 PN 基础上补偿目标法向加速度，以在末端对抗高机动目标。
- 绝对状态卡尔曼滤波器（9 状态 Singer 模型）：采用 9 阶 Padé 矩阵指数进行精确离散化，Joseph 形式协方差更新；在 NED 绝对坐标系下直接估计目标坐标、速度、加速度，消除本机机动与滤波耦合；当目标信息中断时，滤波器仅执行预测步，协方差随时间自然增长，航迹丢失超过 180 秒则自动移除，避免协方差矩阵数值爆炸。

### 非理想感知与数据链建模

- 雷达测量噪声呈现 **各向异性**，其标准差随目标距离平方缩放，并利用饱和限幅函数抑制异常发散，迫使策略在非理想信息下完成目标跟踪与决策。
- 每 3 个仿真步（约 0.03 s）周期性执行雷达扫描与数据链更新，模拟实际传感器刷新与链路延迟；载机可通过数据链向已发射导弹实时注入目标测量向量与协方差矩阵，提高中制导精度。

详细实现见后文《飞行器真实性设计》章节。

### 并行仿真与友伤判定

- 双方决策代码分别运行于子进程沙盒，既绕过 Python 全局解释器锁（GIL），支持并行计算，又严格隔离策略与主仿真的内存空间，防止数据窥探或作弊。
- 所有飞机及导弹作为独立实体推进动力学解算；采用空间均匀哈希网格加速邻近点检测，任意实体间距离小于 15 m 即判定为碰撞摧毁（含友伤），模拟战场意外碰撞与误击风险（仿真步长较大时有穿透风险）。
- 实体飞出战场经纬高范围即刻销毁；可配置红方飞机威胁区（禁区），模拟敌方地空导弹拦截范围，进入即摧毁。

## 🔥 核心定位

与商业平台对比：

| 平台类型 | 典型代表 | 优势 | 面向策略验证的局限 |
| --- | --- | --- | --- |
| 战役级仿真系统 | AFSIM、NGTS | 内建高置信度交战模型与权威武器数据库，支持多域联合作战方案的脚本仿真，时间管理灵活，可超实时运行 | 专有架构限制了自定义感知模型的深度嵌入；虽能批量运行，但难以注入测量噪声、数据链延迟及随机故障等不确定性，无法构建全要素的策略鲁棒性验证闭环 |
| 高保真操作模拟器 | DCS World、X-Plane | 提供工程级精度的气动、航电与传感器操作模拟，具备强大的三维实时可视化与事后复盘能力，单机或小编队任务逼真度极高 | 仿真时序与人类操作紧密耦合，具有“人在回路”的特点；实验复现性与边界条件控制不足；其高保真特性集中于飞行操作层，缺乏高层战术指令解析与自主任务执行架构 |
| 飞控在环测试平台 | AirSim、RflySim | 物理与视觉渲染精细，深度集成PX4、ArduPilot飞控栈，适于底层控制律开发与硬件在环测试 | 设计重心在飞行控制与导航层，缺乏超视距空战专用的火控级传感器退化模型、制导数据链延迟仿真、高层战术指令抽象，无法为决策层策略提供符合空战实际的信息流闭环 |
| 通用机器人研究框架 | Gazebo、Webots | 高度模块化，传感器与物理引擎丰富，ROS、ROS2生态完善，适用于通用机器人算法快速验证 | 缺乏固定翼飞行器的高保真气动模型与空战武器系统仿真，需大量底层重构方能构建具备物理可信度的战术级对抗验证环境 |

与类似项目的差异：

| 维度 | BVR-sim | 常见空战仿真环境 |
| --- | --- | --- |
| 感知模型 | 距离依赖的噪声注入、视场限制、绝对状态卡尔曼滤波 | 多假设真值直接暴露，或仅添加简单高斯噪声 |
| 滤波器 | Singer 模型 + Padé 近似 + 无迹变换 + Joseph 更新 | 通常无目标跟踪，或使用简单 $`\alpha`$ - $`\beta`$ 滤波器 |
| 飞控接口 | 高层战术指令，解析映射为协调转弯和爬升姿态 | 直接控制舵面、过载或角速度，动作空间庞大 |
| 导弹物理 | 六自由度动力学，过载与舵效随动压变化，严格耦合 | 常简化为运动学质点或仅限制最大过载 |
| 制导律 | 三阶段制导：高抛弹道 $`\to`$ PN $`\to`$ APN | 多为纯比例引导，无末制导增强 |
| 仿真架构 | 红蓝策略运行于独立子进程，突破 GIL 限制；策略与仿真内存严格隔离，防止信息窥探与作弊 | 多为单进程顺序执行，并行加速需额外适配层，且缺乏进程级安全隔离 |

BVR-sim 的定位：提供 **Python 可控接口**，集成 **JSBSim 高保真飞行动力学框架**，并刻意引入 **非理想感知特征**（噪声、延迟、友伤判定），旨在回答：“*在训练中表现优异的策略，置于更真实的环境中是否依然可靠？*”

## 📁 文件结构
```
BVR-sim/
├── .gitignore
├── LICENSE
├── README.md
├── mySim.py         # 应用示例：定义红蓝示例策略函数，实例化 bvrsim 类并调用 run 与 start 函数仿真
├── output/          # 存放 ACMI 日志文件，并行仿真终端输出文件（自动在工作区生成）
├── aircraft/        # JSBSim 飞行器配置目录
│   └── aim120c.xml  # AIM‑120C‑5 的 JSBSim 配置文件：几何尺寸、质量特性、推力曲线、三通道飞控及全包线气动系数
└── bvrsim/          # 核心仿真代码库
    ├── __init__.py  # 包入口：定义 EnemyInfo、DroneInfo、SendData 等核心信息结构，自动安装依赖
    ├── aero.py      # JSBSim 气动模型封装，提供六自由度状态更新、坐标变换、单步动力学积分
    ├── base.py      # 飞行器基类：通用 PID 控制器、BTT 协调转弯、经纬高与 NED 互转、视线角计算、探测接口
    ├── bvrsim.py    # 仿真主引擎：并行策略调度、空间哈希网格碰撞检测、边界和威胁区检查、事件日志与仿真推进
    ├── drone.py     # 无人机飞控逻辑：将高层战术指令映射为底层飞控指令，武器管理、射程修正；F‑16 实例
    ├── missile.py   # 导弹基类与 AIM‑120C‑5 具体实现，包含比例引导（PN）与增广比例引导（APN）的多阶段制导律
    ├── radar.py     # 受限雷达模型、绝对状态卡尔曼滤波器：噪声注入、Singer 机动模型、9 阶 Padé 矩阵指数、无迹变换
    └── visual.py    # Tacview 遥测记录，输出 .acmi 文件用于事后可视化复盘
```
## 🚀 快速开始
```bash
git clone https://github.com/celestial-dew/BVR-sim.git
cd BVR-sim
python mySim.py
```
用户只需安装 [Python](https://www.python.org/downloads) 3.10 以上。（推荐用虚拟环境）运行时导入 `bvrsim` 会自动检测并配置 `jsbsim`、`numpy` 等必需依赖。

（可选但推荐）安装 [Tacview (OneDrive mirror)](https://www.tacview.net/download/license/en?file=TacviewSetup.exe&mirror=1)，加载 `output/*.acmi` 日志，查看实体详细参数、雷达锁定与损毁事件的三维回放。

外部视角：

https://github.com/user-attachments/assets/229882fd-ec26-4a78-843f-c3b679c3f333

座舱视角：

https://github.com/user-attachments/assets/64b6bab2-3f71-4066-aa90-4d180435f69f

应用示例（`mySim.py`）：
```python
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
```
## 🧠 核心接口说明

本接口基于赛题的信息结构与飞控方式，移除冗余指令，保留必要的感知受限与飞控难度，而非遵循 `reset`、`step`、`act`、`update` 等 RL 接口。

### DroneInfo（本机精确状态）

仿真每步会将本机状态打包为 `DroneInfo` 对象和步骤序号 `step_num` 传递给策略函数。
```python
class DroneInfo:
    DroneID: int          # 本机ID
    Latitude: float       # 纬度 (rad)
    Longitude: float      # 经度 (rad)
    Altitude: float       # 高度 (m)
    Yaw: float            # 航向角 (rad)
    Pitch: float          # 俯仰角 (rad)
    Roll: float           # 滚转角 (rad)
    V_N, V_E, V_D: float  # NED速度分量 (m/s)
    A_N, A_E, A_D: float  # NED运动加速度分量 (m/s²)
    Mach_M: float         # 马赫数
    fuel: float           # 剩余燃油 (lbs)
    AlarmList: list       # 告警列表 [(辐射源ID, 相对方位角, 类型)]
    FoundEnemyList: list  # 发现敌机列表 (EnemyInfo对象)
    strike: list          # 本机导弹已击中实体的ID列表（含敌机、友机或导弹，体现全体友伤设计）
    MissileNowNum: int    # 剩余导弹数量
```
### EnemyInfo（敌机带噪声信息）
```python
class EnemyInfo:
    EnemyID: int               # 敌机ID
    isNTS: bool                # 是否已被本机火控锁定
    TargetDis: float           # 距离 (m)
    DisRate: float             # 径向相对速度 (m/s)
    TargetYaw: float           # 水平视线角 (rad)
    TargetPitch: float         # 垂直视线角 (rad)
    vel: np.ndarray            # NED速度向量(m/s)
    TargetMach_M: float        # 估计马赫数
    MissilePowerfulDis: float  # 不可逃逸区距离 (动态计算)
    MissileMaxDis: float       # 最大射程 (动态计算)
```
### SendData（控制指令）

策略函数需返回 `SendData` 对象，指定期望航向、速度、高度及攻击指令等。
```python
class SendData:
    CmdSpd: float         # 期望马赫数
    CmdAlt: float         # 期望高度 (m)
    CmdHeadingDeg: float  # 绝对方位角 (deg)
    CmdPitchDeg: float    # 最大允许俯仰角 (deg)，较小值用于平稳飞行包线保护，较大值允许高G机动
    CmdPhi: float         # 最大允许滚转角 (deg)，同上
    TurnDirection: int    # 0=就近转, 1=右转, -1=左转
    ThrustLimit: float    # 推力限制 (kN)
    engage: int           # -1=火控锁定, 1=发射导弹 (仅在从0变为非0时触发一次)
    EnemyID: int          # 目标敌机ID
```
## ✈️ 飞行器真实性设计

本平台在 Python 代码层面严格遵循现代空战仿真对物理真实性的要求，飞控模型依据飞行动力学原理实现，感知模型依据雷达探测的宏观逻辑设计，以增强仿真仿真结果的工程可信度。

### F-16 战斗机

JSBSim 内置的高保真 F-16 六自由度模型，包含完整的非线性气动系数表（升力、阻力、侧力及三轴力矩随马赫数、攻角、侧滑角变化）、真实涡喷发动机推力曲线、燃油消耗及惯性耦合效应。与常见环境中采用的质点运动学或线性化小扰动模型不同，本平台直接利用该 F-16 的完整气动数据库，使飞机动态自然呈现短周期模态、荷兰滚模态及跨音速非线性特性，保证机动响应的物理合理性。

为便于用户控制，本平台无人机不直接使用舵面或姿态角速度等底层飞控指令，而是接收期望速度、高度、航向等高层战术指令，通过稳态运动学关系映射：

- 由 BTT 协调转弯公式生成滚转角指令：

$$
\phi_{cmd}=arctan2(V_{NE}\cdot r_{cmd},g)
$$

- 由航迹角 - 配平攻角公式生成俯仰角指令：

$$
\theta_{cmd}=arcsin(\frac{V_{Ucmd}}{V})+\frac{\alpha_{trim}}{cos\phi}
$$

其中，期望偏航角速度 $`r_{cmd}=\frac{\Delta\psi}{\Delta t_1}`$，$`\Delta\psi`$ 为航向误差；$`V_{NE}`$ 为当前水平速度，$`V`$ 为当前速度，期望爬升速度 $`V_{Ucmd}=clip(\frac{h_{cmd}-h}{\Delta t_2},-V,V)`$；$`\alpha_{trim}`$ 为配平攻角，$`\phi`$ 为当前滚转角。

而当 $`\theta=\pm\frac{\pi}{2}`$ 时，副翼无法产生有效偏航力矩，其指令置 0；当 $`\phi=\pm\frac{\pi}{2}`$ 时，升降舵无法实现爬升或俯冲，则俯仰角误差改为 $`-sgn(\phi)\cdot\Delta\psi`$，通过升降舵跟踪该指令实现强制转弯。

上述姿态角指令先经过用户限幅，再限制在 $`[-\frac{\pi}{2},\frac{\pi}{2}]`$，由 PID 控制器生成归一化副翼、升降舵指令，最终作用于 JSBSim 内置的 F-16 六自由度模型。基于座舱视角，注意归一化升降舵指令为正数时俯冲，负数时爬升。

### AIM-120-C 导弹

物理参数与六自由度动力学模型由 `aim120c.xml` 定义，该文件综合多份公开技术报告与社区分析数据校准形成，**严格保留速度与转弯率的物理耦合** —— 高超声速下有限可用过载与舵效随动压变化，杜绝强化学习训练中常见的“瞬间指向”过理想化假设。

- **几何参数**

  弹体全长 3.655 m，弹径 0.180 m，翼展 0.482 m；参考面积取基于弹径的圆截面 0.0254 $`m^2`$（0.274 $`ft^2`$），参考弦长 0.241 m。力矩参考点位于弹头后方 1.727 m 处，与初始重心位置重合，简化动力学耦合项配置。

- **质量与惯性**

  发射总质量 161.5 kg，其中推进剂质量 51 kg，空重 110.5 kg。绕纵轴转动惯量 $`Ixx=0.43\ slug\cdot ft^2`$（约 0.583 $`kg\cdot m^2`$），横轴与竖轴惯量 $`Iyy=Izz=270\ slug\cdot ft^2`$（约 366 $`kg\cdot m^2`$）。$`Ixx`$ 与 $`Iyy`$、$`Izz`$ 的巨大差异源于细长体质量分布特征：滚转惯量仅依赖弹体半径，而俯仰、偏航惯量由弹体长度主导，符合十字形尾舵空空导弹的惯量属性。仿真中重心视为固定，忽略推进剂消耗引起的微小移动。

- **推进系统**

  采用单推力固体火箭发动机，比冲 265 s。推力曲线以已消耗推进剂质量为自变量进行表格插值，满足 JSBSim 火箭推力表对自变量单调递增的要求。点火后 0.15 s 内迅速建压，约 1 s 内维持恒定峰值推力 16.77 kN（3770 lbf），此后逐渐衰减，总燃烧时间约 7.75 s，总冲与公开评估报告一致。推力作用点位于弹体尾部（弹头后 3.632 m），喷管出口面积 0.0113 $`m^2`$。

- **气动系数**

  零升阻力系数 CD0 覆盖马赫 0 ~ 4.0 包线，跨声速峰值 CD0 = 0.45（以参考面积计算），与 CFD 分析中湿面积折算后的趋势一致。法向力系数 $`CN\alpha`$ 以马赫数和攻角为双变量表格给出，体现压缩性对升力线斜率的影响，高亚声速区升力效率较超声速区有所增强。

  纵向静稳定度设计为 $`Cm\alpha`$ = +0.25（放宽静稳定），依赖主动控制系统增稳；舵面俯仰力矩系数 $`Cm\delta e`$ 随马赫数调度，以匹配不同速域下的舵效变化。偏航与滚转力矩系数分别取 $`Cn\delta r`$ = -6.5 和 $`Cl\delta a`$ = 10.0（常值），阻尼力矩系数 Cmq = -20.0、Cnr = -15.0、Clp = -30.0，均为无量纲角速度导数，用于捕获短周期阻尼特性。诱导阻力以 $`0.18\cdot\alpha^2`$ 建模，舵面偏转产生的附加阻力系数为 $`0.25\cdot\delta e^2`$。

- **舵回路与阻尼增稳**

  四片十字形布局的全动舵面偏度极限 $`\pm 30^\circ`$，作动器时间常数 0.0023 s，对应约 $`300^\circ /s`$ 的偏转度。XML 内部建基于动压 q 调度的角速度阻尼回路：俯仰与偏航阻尼增益由 0.15（q = 1000 psf）递减至 0.06（q = 12000 psf），滚转阻尼增益略高以抑制螺旋模态。该回路仅起短周期增稳作用，不影响外环制导指令的主导地位。

但导弹核心数据属于军事机密，上述参数的选取力求在可用信息范围内，具备同类武器的宏观表现，仅作为策略验证的 **拟真参考模型**，不应视为任何真实系统的精确复制。

制导律分三阶段：

- **发射初期**：赋予额外小仰角（$`\theta = \min(90^\circ, 5^\circ + \theta_{carrier}`$），模拟高抛弹道，提升射程。
- **中制导**：比例引导（PN），加速度指令：

$$
\mathbf{a}_{PNcmd}=N \frac{\mathbf{v}_{rel} - (\mathbf{v}_{rel}\cdot\mathbf{u})\mathbf{u}}{t_{go}}
$$

  其中，$`\mathbf{u}`$ 为视线单位向量，$`t_{go}`$ 为估计剩余飞行时间。

- **末制导**：当目标进入弹载雷达锁定距离后，切换为增广比例引导（APN），增加目标加速度补偿项：

$$
\mathbf{a}_{APNcmd}=\mathbf{a}_{PNcmd}+\frac{N}{2}min\left(1,\frac{t_{go}}{2}\right)(\mathbf{a}_T-(\mathbf{a}_T\cdot\mathbf{u})\mathbf{u})
$$

制导律生成 NED 惯性加速度指令，再由刚体运动学关系求得 NED 期望角速度：

$$
\mathbf{\omega}_{NEDcmd}=\frac{\mathbf{v}\times\mathbf{a}_{cmd}}{|\mathbf{v}|^2}
$$

取出 NED 偏航角速度 $`\mathbf{\omega}_{NEDcmd,D}`$，复用 BTT 协调转弯公式，生成滚转角指令；而俯仰通道需先变换至体轴：

$$
\mathbf{\omega}_{cmd}=\mathbf{R}_{NED\to xyz}\cdot\mathbf{\omega}_{NEDcmd}
$$

其中，$`\mathbf{R}_{NED\to xyz}`$ 为 NED 到导弹体轴系的旋转矩阵。

将滚转角指令、期望俯仰角速度 $`\omega_{cmd,y}`$ 分别由 PID 控制器生成归一化副翼、升降舵指令，驱动 `aim120c.xml` 定义的三通道执行机构。

导弹物理模型与制导律共同决定了“不可逃逸区”与“最大射程”的实际含义，载机需依据滤波器提供的目标状态，判断发射窗口并维持 **单向数据链**，直至末制导锁定。

### 感知受限与噪声注入

雷达探测受 **距离 ‑ 方位 ‑ 俯仰** 三维限制，最大探测距离与波束宽度均为合理值。

测量噪声标准差随目标距离平方缩放：

$$
\sigma(r) = clip\left(\big(\frac{r}{20000}\big)^2, 10^{-3}, 5\right)\cdot\sigma_0
$$

其中，基准噪声 $`\sigma_0 = (80\ m, 0.8^\circ, 1^\circ)`$。

且蓝方雷达探测距离放大至 $`\frac{4}{3}`$ 倍，构建感知非对称条件，以检验红方策略在雷达探测劣势下对目标截获、航迹维持与战术决策的鲁棒性。

## 📈 绝对状态卡尔曼滤波器

### 创新动机

传统机载火控雷达的跟踪滤波器多建立在 **弹体或载机相对坐标系** 下，其缺陷明显：
- 相对状态随本机剧烈机动而快速变化，线性假设极易被破坏。
- 相对加速度难以精确计算和预测，非线性增强，容易发散。
- 预测步与本机姿态、速度高度耦合，小半径转弯时稳定性显著下降。

本滤波器直接在 **NED 绝对地理坐标系** 下估计目标的绝对坐标、绝对速度和绝对加速度，具有以下优点：
- 状态转移矩阵仅取决于目标自身的运动模型，与导弹或载机机动 **完全解耦**，线性度更好。
- 即使导弹进行急转弯等高过载机动，滤波器的预测步也不受影响，鲁棒性高。
- 绝对状态便于融合与分发，可无缝支持数据链共享目标信息。

### 单轴 Singer 模型

Singer 模型将目标运动加速度建模为一阶零均值马尔可夫过程。对于任一坐标轴，取状态 $`\mathbf{x} = [x,\frac{dx}{dt},\frac{d^{2}x}{dt^{2}}]^T`$，连续状态方程为：

$$
d\mathbf{x} = \mathbf{A}\ \mathbf{x}\ dt + \mathbf{G}\ d\mathbf{w},
\quad
\mathbf{A} = \begin{bmatrix}
0 & 1 & 0\\
0 & 0 & 1\\
0 & 0 & -\alpha
\end{bmatrix}
$$

其中，$`\alpha`$ 为机动频率（单位 $`s^{-1}`$），$`\alpha`$ 越大，加速度时间相关性越弱；$`d\mathbf{w}`$ 为单位强度的 Wiener 过程，$`\mathbf{G}=[0,0,\sqrt{q}]^T`$，$`q`$ 为加速度导数的功率谱密度（单位 $`m^{2}/s^{5}`$），决定过程噪声强度。

### 三维扩展

使用 **9 阶 Padé 近似** 配合 **缩放 - 平方**（Scaling & Squaring）技术计算矩阵指数，兼顾高精度与数值稳定性：

$$
e^{\mathbf{A}} \approx \mathbf{D}_9(\mathbf{A})^{-1}\ \mathbf{N}_9(\mathbf{A})
$$

其中，$`\mathbf{D}_9`$ 和 $`\mathbf{N}_9`$ 为 9 阶矩阵多项式，其系数绝对值 $`c_{i}=\frac{9!\ (18-i)!}{18!\ i!\ (9-i)!}`$；通过 1-范数 自动确定缩放因子 $`2^j`$，计算缩比矩阵的 Padé 近似后连续平方 $`j`$ 次恢复。

由 **Van Loan** 定理，构造增广矩阵：

$$
\mathbf{M} = \begin{bmatrix}
\mathbf{A} & \mathbf{G}\ \mathbf{G}^T \\
\mathbf{0} & -\mathbf{A}^T
\end{bmatrix} = \begin{bmatrix}
\mathbf{A} & diag(0,0,q) \\
\mathbf{0} & -\mathbf{A}^T
\end{bmatrix}
$$

计算矩阵指数：

$$
e^{\mathbf{M}\ dt} =
\begin{bmatrix}
\mathbf{\Phi} & \mathbf{Q}_{d}\ \mathbf{\Phi}^{-T}\\
\mathbf{0} & \mathbf{\Phi}^{-T}
\end{bmatrix}
$$

取出 $`\mathbf{\Phi}`$ 和 $`\mathbf{Q}_{d}\ \mathbf{\Phi}^{-T}`$。由于三个坐标轴在 Singer 模型下相互独立，完整 9 维状态转移矩阵与过程噪声协方差矩阵可通过 **Kronecker 积** 直接扩展：

$$
\begin{cases}
\mathbf{F} = \mathbf{\Phi} \otimes \mathbf{I}_3 \\
\mathbf{Q} = \mathbf{Q}_{d} \otimes \mathbf{I}_3 = \mathbf{Q}_{d}\ \mathbf{\Phi}^{-T}\ \mathbf{\Phi}^T\otimes \mathbf{I}_3
\end{cases}
$$

### 预测步

预测步完全独立于导弹运动，无需控制矩阵：

$$
\begin{cases}
\hat{\mathbf{X}}_{k|k-1} = \mathbf{F}\ \hat{\mathbf{X}}_{k-1|k-1}\\
\mathbf{P}_{k|k-1} = \mathbf{F}\ \mathbf{P}_{k-1|k-1}\ \mathbf{F}^T + \mathbf{Q}
\end{cases}
$$

后调用 `psd(P)` 函数，保证 $`\mathbf{P}_{k|k-1}`$ 严格半正定，防止数值崩溃。

若测量丢失超过 180 秒，则自动删除该目标滤波器，模拟航迹丢失，并避免 $`\mathbf{P}_{k|k-1}`$ 数值爆炸问题。

### 更新步

非线性测量通过 **无迹变换**（Unscented Transform）生成 NED 系的测量协方差矩阵 $`\mathbf{R}`$，至少保留非线性变换后分布的前两阶矩（均值与协方差），相较于线性化方法能更真实地反映测量不确定性：

对测量向量 $`\mathbf{z} = [r,\psi,\theta]^T`$ 生成 $`2L+1`$ 个 Sigma 点：

$$
\begin{cases}
\mathbf{Z}^{(i)} = \mathbf{z} + \Delta\mathbf{Z}^{(i)}&i\in[1,2L+1]\\
\Delta\mathbf{Z}^{(i)} = \pm\sqrt{(L+\lambda)\mathbf{P}_{zz}}
\end{cases}
$$

其中，$`\mathbf{P}_{zz} = diag(\sigma_r^2,\sigma_\psi^2,\sigma_\theta^2)`$，$`\lambda = \alpha^2(L+\kappa)-L`$，取 $`\alpha=1,\beta=2,\lambda=1`$（等价于 $`\kappa=1`$）。

将各 Sigma 点转回 NED 坐标：

$$
\mathbf{p}^{(i)} = \mathbf{p}_{xyz} + \mathbf{R}_{NED\to xyz}^T \cdot los2xyz(\mathbf{Z}^{(i)})
$$

计算加权均值与协方差：

$$
\begin{cases}
\hat{\mathbf{p}} = \sum\limits_{i=1}^{2L+1} w_m^{(i)} \mathbf{p}^{(i)}\\
\mathbf{R} = \sum\limits_{i=1}^{2L+1} w_c^{(i)} (\mathbf{p}^{(i)}-\hat{\mathbf{p}})(\mathbf{p}^{(i)}-\hat{\mathbf{p}})^T
\end{cases}
$$

其中，权重 $`w_m^{(1)} = \frac{\lambda}{L+\lambda}`$，$`w_c^{(1)} = w_m^{(1)}+(1-\alpha^2+\beta)`$，其余 $`w_m^{(i)} = w_c^{(i)} = \frac{1}{2(L+\lambda)}`$。

上述带噪声的探测结果与不确定性估计直接馈入卡尔曼滤波器，构成感知 - 估计闭环，任何探测缺失或精度下降都将直接体现在目标状态协方差矩阵中。

求卡尔曼增益，并更新后验状态：

$$
\begin{cases}
\mathbf{K} = \mathbf{P}_{k|k-1}\mathbf{H}^T(\mathbf{H}\ \mathbf{P}_{k|k-1}\ \mathbf{H}^T + \mathbf{R})^{-1}\\
\hat{\mathbf{X}}_{k|k} = \hat{\mathbf{X}}_{k|k-1} + \mathbf{K}(\mathbf{Z} - \mathbf{H}\ \hat{\mathbf{X}}_{k|k-1})
\end{cases}
$$

其中，$`\mathbf{Z}`$ 为目标绝对坐标观测向量。

**Joseph 形式** 协方差更新：

$$
\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}\ \mathbf{H})\ \mathbf{P}_{k|k-1}\ (\mathbf{I} - \mathbf{K}\ \mathbf{H})^T + \mathbf{K}\ \mathbf{R}\ \mathbf{K}^T
$$

而非简化形式：$`\mathbf{P}_{k|k} = (\mathbf{I} - \mathbf{K}\ \mathbf{H})\ \mathbf{P}_{k|k-1}`$。前者具有更好的对称性和正定性保持能力，可有效抵抗计算机舍入误差导致的滤波器发散。

### 对制导性能的提升

采用绝对状态卡尔曼滤波器后，制导系统可直接使用滤波器输出的目标平滑速度与加速度，带来以下增益：
- 基于平滑的速度信息，剩余飞行时间估计更准确。
- APN 的目标法向加速度补偿项更精确，对高 G 机动目标的拦截能力增强。
- 制导指令更平滑，避免噪声导致的导弹运动振荡，减少能量损耗。

综上，该滤波器在解耦性、数值精度、鲁棒性和制导协同性上均较传统相对坐标方案有本质提升。

## 🎯 应用场景

- **策略验证**：检测策略是否过拟合于特定训练环境的“捷径特征”（shortcut features），环境非平稳性对博弈算法的影响分析。
- **飞控与滤波算法**：对比不同制导律或滤波算法在高拟真环境下的性能差异，飞行控制律的鲁棒性边界测试。
- **RL 训练改造**：可改造为考虑观测不确定性的 **高拟真 RL 训练** 环境，在训练中学习应对非理想感知，提升策略鲁棒性。
- **教学演示**：直观展示超视距空战中信息流传递与延迟的影响。
- **二次开发**：可修改气动与传感器模型，丰富飞行器选择，作为算法验证或空战游戏的底层基座。

## ⚖️ 协议与声明

本平台采用 **BSD 3‑Clause License**。在满足以下条件的前提下，允许自由使用、修改和分发：
- 保留原始版权声明、许可条款及免责声明。
- 禁止使用本项目的作者或贡献者名称进行商业推广或背书。

本平台仅为学术研究与教育目的而开发，**不包含任何国家秘密或受控数据**。所有飞行器参数均来源于公开文献与开源社区推导，与真实装备可能存在差异，用户可自行校准。

## 🙏 致谢

- JSBSim 开发团队提供的高保真飞行动力学框架。
- 第十九届“挑战杯”竞赛组委会与中国航空工业集团沈阳飞机设计研究所提供的赛题启发。
- 社区公开的 AIM-120C-5 性能评估报告（AIM-120C-5 Performance Assessment for Digital Combat Simulation Enhancement, Rev 2）与 DCS World 相关气动研究，为本模型参数校准提供了关键参考。
- 矩阵指数 Padé 近似算法参考（Moler & Van Loan, Nineteen Dubious Ways to Compute the Exponential of a Matrix, SIAM Review, 2003）。

## 📬 联系方式

如有问题或合作意向，欢迎通过 GitHub Issues 或邮件（15323122984@163.com）联系作者。