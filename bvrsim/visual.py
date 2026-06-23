import os
import time
from .base import base
from sys import argv
from glob import glob
from numpy import rad2deg


class tacview:
    def __init__(self, root, exist_ok):
        """
        root:str,acmi文件目录
        exist_ok:bool,是否覆盖最近文件
        """
        os.makedirs(root, exist_ok=True)
        file = f"{root}/{os.path.splitext(os.path.basename(argv[0]))[0]}_"
        file += f"{len(glob(file+'*.acmi'))+(not exist_ok)or 1}.acmi"  # 序号
        self.file = open(file, "w", encoding="utf-8")
        self.buffer = [
            "FileType=text/acmi/tacview\n",
            "FileVersion=2.2\n",
            f"0,ReferenceTime={time.strftime('%Y-%m-%dT%H:%M:%SZ',time.gmtime())}\n",
        ]

    def flush(self):
        self.file.writelines(self.buffer)
        self.buffer.clear()

    def __del__(self):
        self.flush()
        self.file.close()

    def logtime(self, time):
        self.buffer.append(f"#{time:.3f}\n")

    def loginit(self, entity: base, color, name="F-16C", radar=1):
        deg = rad2deg(entity.radar.rad)
        self.buffer.append(
            f"{entity.ID:X},Color={color},Name={name},RadarMode={radar},RadarAzimuth={deg},RadarElevation={deg},RadarRange={entity.radar.r},Radius=15\n"
        )

    def logstep(self, entity: base):
        lon, lat, roll, pitch, yaw = rad2deg(
            (entity.Longitude, entity.Latitude, entity.Roll, entity.Pitch, entity.Yaw)
        )
        self.buffer.append(
            f"{entity.ID:X},T={lon:.6f}|{lat:.6f}|{entity.Altitude:.2f}|{roll:.2f}|{pitch:.2f}|{yaw:.2f}\n"
        )

    def logNTS(self, entityID, TargetID):
        self.buffer.append(
            f"{entityID:X},LockedTargetMode=1,LockedTarget={TargetID:X}\n"
        )

    def logdestroy(self, TargetID):
        self.buffer.append(f"-{TargetID:X}\n")
