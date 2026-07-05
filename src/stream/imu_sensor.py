"""IMU 传感器线程。"""
from queue import Queue
from typing import Optional

import numpy as np

from src.core.data_types import ImuMeasurement, SensorData
from src.core.thread_control import ThreadControl
from src.stream.base import StreamerBase
from src.stream.formators import ImuFormator


class ImuSensor(StreamerBase):
    """IMU 文本读取（GPST 格式）。

    支持 IMU 坐标系转换：若 imu_coordinate_system != "FRD"，
    在读取时将原始坐标系转换到 FRD（项目标准 b 系）。
    """

    def __init__(self, file_path: str, output_queue: Queue,
                 control: ThreadControl,
                 imu_coordinate_system: str = "FRD"):
        super().__init__(
            file_path=file_path,
            formator=ImuFormator(),
            output_queue=output_queue,
            control=control,
            tag="imu",
        )
        self.coordinate_system = imu_coordinate_system.upper()

    def run(self):
        """线程入口：逐行读取 → 解码 → 坐标系转换 → 入队 → EOF sentinel。"""
        try:
            with open(self.file_path, "r", encoding="utf-8") as f:
                for line in f:
                    if not self.control.is_running():
                        break
                    data = self.formator.decode(line)
                    if data is not None:
                        if data.imu is not None:
                            data.imu = self._convert_to_frd(data.imu)
                        self.output_queue.put(data)
        finally:
            self.output_queue.put(None)

    def _convert_to_frd(self, imu: ImuMeasurement) -> ImuMeasurement:
        """将 IMU 数据从原始坐标系转换到 FRD（前-右-下）。"""
        if self.coordinate_system == "FRD":
            return imu
        elif self.coordinate_system == "RFU":
            # RFU (Right-Front-Up) → FRD (Front-Right-Down)
            # FRD_x = RFU_y, FRD_y = RFU_x, FRD_z = -RFU_z
            new_gyro = np.array([imu.gyro[1], imu.gyro[0], -imu.gyro[2]],
                                dtype=np.float64)
            new_accel = np.array([imu.accel[1], imu.accel[0], -imu.accel[2]],
                                 dtype=np.float64)
            return ImuMeasurement(
                timestamp=imu.timestamp,
                week=imu.week,
                accel=new_accel,
                gyro=new_gyro,
            )
        else:
            raise ValueError(
                f"Unsupported imu_coordinate_system: {self.coordinate_system}"
            )
