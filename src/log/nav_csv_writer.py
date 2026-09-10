"""100 Hz TC 导航状态 CSV 输出器 (campus01 实验契约)。

固定列: ``week,sow,x,y,z,vx,vy,vz,roll,pitch,yaw``。

契约:
- 每次成功机械编排提交写一行 (由 ``TcIntegration._emit_propagation`` 触发),
  不做整数秒门控, 不复制边界状态凑行数;
- ``sow`` 为 GPST 周内秒, ``x/y/z`` 为 ECEF (m), ``vx/vy/vz`` 为 ECEF (m/s),
  ``roll/pitch/yaw`` 为 FRD (deg, 与 RSLTWriter 一致);
- 未初始化阶段的纯 GNSS 解不属于机械编排状态, 不写入本文件。
"""
import csv
from pathlib import Path

import numpy as np

from src.core.time_utils import unix_to_gpst


class NavCsvWriter:
    """TC 机械编排状态 CSV 输出器。"""

    COLUMNS = ("week", "sow", "x", "y", "z",
               "vx", "vy", "vz", "roll", "pitch", "yaw")
    _SOW_FORMAT = "%.9f"
    _RAD2DEG = 180.0 / np.pi

    def __init__(self, output_dir: str, filename: str = "nav-100hz.csv"):
        self.output_dir = str(output_dir)
        self.filename = filename
        self.path = Path(self.output_dir) / filename
        self.row_count = 0
        self._fp = None

    def open(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = open(self.path, "w", encoding="utf-8", newline="")
        writer = csv.writer(self._fp)
        writer.writerow(self.COLUMNS)

    def write(self, state, P, si, q: int, qins: int, num_sv: int) -> None:
        """写一行机械编排状态 (P/si/q/qins/num_sv 保留接口兼容)。"""
        if self._fp is None:
            raise RuntimeError("NavCsvWriter not opened")
        week, sow = unix_to_gpst(state.timestamp)
        pos = np.asarray(state.pos_e, dtype=np.float64)
        vel = np.asarray(state.vel_e, dtype=np.float64)
        rpy = np.asarray(state.att_rpy, dtype=np.float64) * self._RAD2DEG
        row = [
            f"{int(week)}",
            self._SOW_FORMAT % float(sow),
            "%.6f" % pos[0], "%.6f" % pos[1], "%.6f" % pos[2],
            "%.6f" % vel[0], "%.6f" % vel[1], "%.6f" % vel[2],
            "%.6f" % rpy[0], "%.6f" % rpy[1], "%.6f" % rpy[2],
        ]
        self._fp.write(",".join(row) + "\n")
        self.row_count += 1

    def write_gnss_only(self, timestamp: float, pos_e: np.ndarray,
                        q: int, num_sv: int, pos_sd: np.ndarray = None) -> None:
        """忽略未初始化 GNSS 解 (非机械编排状态, 不属于本文件契约)。"""
        return

    def close(self) -> None:
        if self._fp is not None:
            self._fp.close()
            self._fp = None
