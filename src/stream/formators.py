"""Formator 解码层。

- ImuFormator: 解析 GPST 格式 IMU CSV
  列: GPS week, GPS sow, gx, gy, gz, ax, ay, az
- PosSolFormator: 解析 rtklib POS 格式 GNSS 结果
  数据行: yyyy/mm/dd hh:mm:ss.s  x  y  z  Q  ns  sdx  sdy  sdz  sdxy  sdyz  sdzx  age  ratio

输入的 GPS 周+周内秒在解码时转换为 Unix 时间戳（与 rtklib-py gtime_t 一致）。
"""
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from src.core.data_types import ImuMeasurement, GnssSolution, SensorData
from src.core.time_utils import ymdhms_to_gpst, gpst_to_unix


class FormatorBase(ABC):
    """解码层抽象基类。"""

    @abstractmethod
    def decode(self, line: str) -> Optional[SensorData]:
        """解码一行文本。

        Returns:
            SensorData 或 None（注释/空行）
        """
        ...


class ImuFormator(FormatorBase):
    """IMU 文本解码（GPST 格式）。"""

    def decode(self, line: str) -> Optional[SensorData]:
        line = line.strip()
        if not line or line.startswith("#"):
            return None
        parts = line.split(",")
        if len(parts) < 8:
            return None
        try:
            week = int(parts[0])
            sow = float(parts[1])
            gx = float(parts[2])
            gy = float(parts[3])
            gz = float(parts[4])
            ax = float(parts[5])
            ay = float(parts[6])
            az = float(parts[7])
        except (ValueError, IndexError):
            return None
        imu = ImuMeasurement(
            timestamp=gpst_to_unix(week, sow),
            week=week,
            accel=np.array([ax, ay, az], dtype=np.float64),
            gyro=np.array([gx, gy, gz], dtype=np.float64),
        )
        return SensorData(tag="imu", imu=imu)


class PosSolFormator(FormatorBase):
    """rtklib POS 格式解码。"""

    def decode(self, line: str) -> Optional[SensorData]:
        line = line.strip()
        if not line or line.startswith("%"):
            return None
        # 时间字段格式: yyyy/mm/dd hh:mm:ss.s
        # 用空格分割后，前两段是日期和时间
        parts = line.split()
        if len(parts) < 8:
            return None
        try:
            date_str = parts[0]  # yyyy/mm/dd
            time_str = parts[1]  # hh:mm:ss.s
            y, mo, d = (int(x) for x in date_str.split("/"))
            h, mi, s = time_str.split(":")
            h, mi = int(h), int(mi)
            s = float(s)
            week, sow = ymdhms_to_gpst(y, mo, d, h, mi, s)

            x = float(parts[2])
            y_pos = float(parts[3])
            z = float(parts[4])
            q = int(float(parts[5]))
            ns = int(float(parts[6]))
            sdx = float(parts[7])
            sdy = float(parts[8])
            sdz = float(parts[9])
        except (ValueError, IndexError):
            return None
        sol = GnssSolution(
            timestamp=gpst_to_unix(week, sow),
            week=week,
            position=np.array([x, y_pos, z], dtype=np.float64),
            quality=q,
            num_sv=ns,
            sd=np.array([sdx, sdy, sdz], dtype=np.float64),
        )
        return SensorData(tag="gnss_solution", gnss_solution=sol)
