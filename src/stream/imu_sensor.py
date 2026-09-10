"""IMU 传感器线程。"""
from queue import Queue

import numpy as np

from src.core.data_types import ImuMeasurement, IncrementImuData, RateImuData
from src.core.thread_control import ThreadControl
from src.stream.base import StreamerBase
from src.stream.formators import (
    ImuFormator,
    EuRoCImuFormator,
    GreatMsfRateImuFormator,
    rfu_axes_to_frd,
)


class ImuSensor(StreamerBase):
    """IMU 文本读取，支持 GPST、EuRoC 和 GREAT 七列速率格式。

    支持 IMU 坐标系转换：若 imu_coordinate_system != "FRD"，
    在读取时将原始坐标系转换到 FRD（项目标准 b 系）。

    Args:
        file_path: IMU 数据文件路径
        output_queue: 输出队列
        control: 线程控制
        imu_coordinate_system: IMU 原始坐标系（FRD / RFU）
        imu_format: 数据格式（gpst / euroc / great_msf_7col_rate），默认 gpst
        imu_options: 格式专属选项（GREAT: week/axis_order/gyro_unit/accel_unit）
    """

    def __init__(self, file_path: str, output_queue: Queue,
                 control: ThreadControl,
                 imu_coordinate_system: str = "FRD",
                 imu_format: str = "gpst",
                 imu_options: dict | None = None):
        formator = self._create_formator(imu_format, imu_options)
        super().__init__(
            file_path=file_path,
            formator=formator,
            output_queue=output_queue,
            control=control,
            tag="imu",
        )
        self.coordinate_system = imu_coordinate_system.upper()

    @staticmethod
    def _create_formator(imu_format: str, imu_options: dict | None = None):
        """根据格式名称创建对应的解码器。"""
        fmt = imu_format.lower()
        options = dict(imu_options or {})
        if fmt == "gpst":
            return ImuFormator()
        elif fmt == "euroc":
            return EuRoCImuFormator()
        elif fmt == "great_msf_7col_rate":
            # GREAT 七列格式自带轴序/单位处理, 输出已是 FRD + rad/s + m/s²。
            unknown = set(options) - {
                "week", "axis_order", "gyro_unit", "accel_unit"}
            if unknown:
                raise ValueError(
                    f"Unsupported great_msf_7col_rate options: {sorted(unknown)}"
                )
            return GreatMsfRateImuFormator(**options)
        else:
            raise ValueError(
                f"Unsupported imu_format: {imu_format} "
                f"(expected 'gpst', 'euroc' or 'great_msf_7col_rate')"
            )

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
            # RFU (Right-Front-Up) → FRD (Front-Right-Down), 复用 formators 的
            # 唯一轴序映射表 (与 GREAT ``garfu`` 完全一致)
            if isinstance(imu.payload, RateImuData):
                payload = RateImuData(
                    gyro=rfu_axes_to_frd(imu.payload.gyro),
                    accel=rfu_axes_to_frd(imu.payload.accel),
                )
            elif isinstance(imu.payload, IncrementImuData):
                payload = IncrementImuData(
                    dtheta=rfu_axes_to_frd(imu.payload.dtheta),
                    dvel=rfu_axes_to_frd(imu.payload.dvel),
                    dt=imu.payload.dt,
                    sow=imu.payload.sow,
                )
            else:  # pragma: no cover - ImuMeasurement validates payload type.
                raise TypeError("unsupported IMU payload")
            return ImuMeasurement(
                timestamp=imu.timestamp,
                week=imu.week,
                payload=payload,
            )
        else:
            raise ValueError(
                f"Unsupported imu_coordinate_system: {self.coordinate_system}"
            )
