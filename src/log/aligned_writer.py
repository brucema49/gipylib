"""对齐数据块状 CSV 输出器。"""
import csv
from pathlib import Path

from src.core.data_types import AlignedBlock
from src.core.time_utils import unix_to_gpst
from src.log.writer_base import WriterBase


class AlignedWriter(WriterBase):
    """对齐数据块状 CSV 输出器。

    输出格式（无表头）：
        G,week,sow,x,y,z,q,ns,sdx,sdy,sdz        (11 列)
        I,week,sow,gx,gy,gz,ax,ay,az              (9 列)
        I,week,sow,gx,gy,gz,ax,ay,az
        ...（N 行 I）
        G,week,sow,...
        ...
    """

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
        # 不写表头：G 行 11 列、I 行 9 列，列数不同

    def write(self, block: AlignedBlock) -> None:
        if self._writer is None:
            raise RuntimeError("AlignedWriter not opened")
        g = block.gnss
        # timestamp 是 Unix 时间戳，输出时转回 (week, sow)
        g_week, g_sow = unix_to_gpst(g.timestamp)
        # G 行: G, week, sow, x, y, z, q, ns, sdx, sdy, sdz (11 列)
        self._writer.writerow([
            "G", g_week, f"{g_sow:.6f}",
            f"{g.position[0]:.4f}", f"{g.position[1]:.4f}", f"{g.position[2]:.4f}",
            g.quality, g.num_sv,
            f"{g.sd[0]:.4f}", f"{g.sd[1]:.4f}", f"{g.sd[2]:.4f}",
        ])
        # I 行: I, week, sow, gx, gy, gz, ax, ay, az (9 列)
        for imu in block.imu_list:
            i_week, i_sow = unix_to_gpst(imu.timestamp)
            self._writer.writerow([
                "I", i_week, f"{i_sow:.6f}",
                f"{imu.gyro[0]:.6f}", f"{imu.gyro[1]:.6f}", f"{imu.gyro[2]:.6f}",
                f"{imu.accel[0]:.6f}", f"{imu.accel[1]:.6f}", f"{imu.accel[2]:.6f}",
            ])

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
