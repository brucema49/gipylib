"""机械编排 trace 输出器。

每历元输出一行 CSV, 字段:
  timestamp, pos_ecef[3], vel_ecef[3], att_rpy_deg[3], P1_diag[15]
"""
import csv
from pathlib import Path
from typing import Optional

import numpy as np

from src.core.data_types import InsState


class TraceWriter:
    """机械编排 trace CSV 输出器。"""

    HEADER = [
        "timestamp",
        "pos_x", "pos_y", "pos_z",
        "vel_x", "vel_y", "vel_z",
        "roll_deg", "pitch_deg", "yaw_deg",
        "P1_d0", "P1_d1", "P1_d2", "P1_d3", "P1_d4",
        "P1_d5", "P1_d6", "P1_d7", "P1_d8", "P1_d9",
        "P1_d10", "P1_d11", "P1_d12", "P1_d13", "P1_d14",
    ]

    def __init__(self, path: str):
        self.path = Path(path)
        self._fp: Optional[object] = None
        self._writer: Optional[csv.writer] = None
        self._row_count = 0

    def open(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = open(self.path, "w", newline="", encoding="utf-8")
        self._writer = csv.writer(self._fp)
        self._writer.writerow(self.HEADER)
        self._row_count = 0

    def write(self, state: InsState, P1_diag: np.ndarray) -> None:
        if self._writer is None:
            raise RuntimeError("TraceWriter 未 open")
        rpy_deg = np.degrees(state.att_rpy)
        row = [
            f"{state.timestamp:.6f}",
            f"{state.pos_e[0]:.6f}", f"{state.pos_e[1]:.6f}", f"{state.pos_e[2]:.6f}",
            f"{state.vel_e[0]:.6f}", f"{state.vel_e[1]:.6f}", f"{state.vel_e[2]:.6f}",
            f"{rpy_deg[0]:.6f}", f"{rpy_deg[1]:.6f}", f"{rpy_deg[2]:.6f}",
        ]
        for i in range(15):
            row.append(f"{float(P1_diag[i]):.6e}")
        self._writer.writerow(row)
        self._row_count += 1

    @property
    def row_count(self) -> int:
        return self._row_count

    def close(self) -> None:
        if self._fp is not None:
            self._fp.close()
            self._fp = None
            self._writer = None

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, *args):
        self.close()
