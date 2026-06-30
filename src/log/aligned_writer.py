"""对齐数据 CSV 输出器。"""
import csv
from pathlib import Path
from typing import List

from src.core.data_types import AlignedRow
from src.log.writer_base import WriterBase


CSV_HEADER: List[str] = [
    "gps_week", "gps_sow", "imu_count",
    "imu_first_sow", "imu_last_sow",
    "gnss_x", "gnss_y", "gnss_z",
    "gnss_q", "gnss_ns",
    "gnss_sdx", "gnss_sdy", "gnss_sdz",
    "imu_avg_ax", "imu_avg_ay", "imu_avg_az",
    "imu_avg_gx", "imu_avg_gy", "imu_avg_gz",
]


class AlignedWriter(WriterBase):
    """对齐数据 CSV 输出器。"""

    def __init__(self, output_dir: str, filename: str = "aligned.csv"):
        self.output_dir = Path(output_dir)
        self.filename = filename
        self._file = None
        self._writer = None

    def open(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        path = self.output_dir / self.filename
        self._file = open(path, "w", encoding="utf-8", newline="")
        self._writer = csv.writer(self._file)
        self._writer.writerow(CSV_HEADER)

    def write(self, row: AlignedRow) -> None:
        if self._writer is None:
            raise RuntimeError("AlignedWriter not opened")
        self._writer.writerow([
            row.week, f"{row.sow:.6f}", row.imu_count,
            f"{row.imu_first_sow:.6f}", f"{row.imu_last_sow:.6f}",
            f"{row.gnss_pos[0]:.4f}", f"{row.gnss_pos[1]:.4f}", f"{row.gnss_pos[2]:.4f}",
            row.gnss_q, row.gnss_ns,
            f"{row.gnss_sd[0]:.4f}", f"{row.gnss_sd[1]:.4f}", f"{row.gnss_sd[2]:.4f}",
            f"{row.imu_avg_accel[0]:.6f}", f"{row.imu_avg_accel[1]:.6f}", f"{row.imu_avg_accel[2]:.6f}",
            f"{row.imu_avg_gyro[0]:.6f}", f"{row.imu_avg_gyro[1]:.6f}", f"{row.imu_avg_gyro[2]:.6f}",
        ])

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
