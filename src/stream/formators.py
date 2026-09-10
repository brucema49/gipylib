"""Formator 解码层。

- ImuFormator: 解析 GPST 格式 IMU CSV
  列: GPS week, GPS sow, gx, gy, gz, ax, ay, az
- EuRoCImuFormator: 解析 EuRoC 格式 IMU CSV
  列: timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z
  时间戳为 Unix 纳秒，坐标系为 RFU (Right-Front-Up)
- PosSolFormator: 解析 rtklib POS 格式 GNSS 结果
  数据行: yyyy/mm/dd hh:mm:ss.s  x  y  z  Q  ns  sdx  sdy  sdz  sdxy  sdyz  sdzx  age  ratio

输入的 GPS 周+周内秒在解码时转换为 Unix 时间戳（与 rtklib-py gtime_t 一致）。
EuRoC 格式的纳秒时间戳除以 1e9 转换为 Unix 秒时间戳。
"""
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from src.core.data_types import (
    GnssSolution,
    ImuMeasurement,
    RateImuData,
    SensorData,
)
from src.core.time_utils import ymdhms_to_gpst, gpst_to_unix, unix_to_gpst


# GREAT ``DataFormat/AxisOrder`` 值 → 输出到项目 FRD 的三轴 (源索引, 符号)。
# ``garfu`` 与 ImuSensor 的历史 RFU→FRD 约定完全一致, 这里保留唯一一份映射表。
_AXIS_ORDER_TO_FRD = {
    "garfu": ((1, 1.0), (0, 1.0), (2, -1.0)),   # Right-Front-Up → FRD
    "gaflu": ((1, 1.0), (0, -1.0), (2, -1.0)),  # Left-Front-Up  → FRD
    "gafrd": ((0, 1.0), (1, 1.0), (2, 1.0)),    # Forward-Right-Down 保持不变
}


def axes_to_frd(values: np.ndarray, axis_order: str) -> np.ndarray:
    """按 GREAT 声明的轴序把三轴向量转换到项目 FRD。"""
    order = str(axis_order).lower()
    try:
        mapping = _AXIS_ORDER_TO_FRD[order]
    except KeyError:
        raise ValueError(
            f"Unsupported AxisOrder '{axis_order}', expected one of "
            f"{sorted(_AXIS_ORDER_TO_FRD)}"
        ) from None
    values = np.asarray(values, dtype=np.float64)
    if values.shape != (3,):
        raise ValueError("axis conversion requires exactly three values")
    return np.array([sign * values[src] for src, sign in mapping],
                    dtype=np.float64)


def rfu_axes_to_frd(values: np.ndarray) -> np.ndarray:
    """RFU (Right-Front-Up) → FRD (Front-Right-Down), 与历史实现一致。"""
    return axes_to_frd(values, "garfu")


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
    """IMU 文本解码（GPST 格式）。

    列: GPS week, GPS sow, gx, gy, gz, ax, ay, az
    """

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
            payload=RateImuData(
                accel=np.array([ax, ay, az], dtype=np.float64),
                gyro=np.array([gx, gy, gz], dtype=np.float64),
            ),
        )
        return SensorData(tag="imu", imu=imu)


class EuRoCImuFormator(FormatorBase):
    """IMU 文本解码（EuRoC 格式）。

    列: timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z
    时间戳为 Unix 纳秒，除以 1e9 转换为 Unix 秒。
    GPS 周号由 Unix 时间戳派生。
    坐标系默认为 RFU (Right-Front-Up)，由 ImuSensor 负责转换为 FRD。
    """

    def decode(self, line: str) -> Optional[SensorData]:
        line = line.strip()
        if not line or line.startswith("#"):
            return None
        parts = line.split(",")
        if len(parts) < 7:
            return None
        try:
            timestamp_ns = int(parts[0])
            wx = float(parts[1])
            wy = float(parts[2])
            wz = float(parts[3])
            ax = float(parts[4])
            ay = float(parts[5])
            az = float(parts[6])
        except (ValueError, IndexError):
            return None
        timestamp = timestamp_ns / 1e9
        week, sow = unix_to_gpst(timestamp)
        # 与 rtklib-py 的 epoch2time 一致: 把 GPS 时间当作 UTC 处理 (伪 Unix GPS)
        # 这样 imu.timestamp 与 obsr.t.time + obsr.t.sec 在同一时间系
        # rtklib-py 内部 epoch2time 把 RINEX GPS 时间当作 UTC, 比真实 Unix UTC 多 18s leap
        # unix_to_gpst/gpst_to_unix 互为逆运算 (未考虑 leap), 故直接 +18s 对齐
        LEAP_SECONDS = 18  # 2025年 GPS-UTC = 18s
        timestamp = gpst_to_unix(week, sow) + LEAP_SECONDS
        week, sow = unix_to_gpst(timestamp)
        imu = ImuMeasurement(
            timestamp=timestamp,
            week=week,
            payload=RateImuData(
                accel=np.array([ax, ay, az], dtype=np.float64),
                gyro=np.array([wx, wy, wz], dtype=np.float64),
            ),
        )
        return SensorData(tag="imu", imu=imu)


class GreatMsfRateImuFormator(FormatorBase):
    """GREAT-MSF 七列速率 IMU 流式解码 (campus01 ``-MEMS`` 文件)。

    列: ``sow g(3) a(3)`` (空格分隔, 无表头/单位行), 例如::

        180589.800000  -0.13146  0.04577 -0.03983  -0.06376 -0.30255 9.87304

    - ``axis_order`` 复用 ``axes_to_frd`` 的唯一映射表 (campus01 为 ``garfu``,
      即 Right-Front-Up; 静止时第三列加计 ≈ +9.87 印证 Up 轴);
    - ``gyro_unit``/``accel_unit`` 为 GREAT XML ``DataFormat`` 声明值;
    - 只产出 ``RateImuData``: sensor 层不做积分, 不做重采样。
    """

    # 只接受语义明确的单位 (DPS=deg/s, RAD=rad/s, MPS2/MPS=m/s²)。
    _GYRO_UNIT_SCALE = {
        "DPS": np.pi / 180.0,
        "RAD": 1.0,
    }
    _ACCEL_UNIT_SCALE = {
        "MPS2": 1.0,
        "MPS": 1.0,
    }

    def __init__(self, week: int = 0, axis_order: str = "garfu",
                 gyro_unit: str = "DPS", accel_unit: str = "MPS2"):
        self.week = int(week)
        self.axis_order = str(axis_order)
        # 提前校验轴序, 未支持的值在构造时即报错 (而不是每行解析时)
        axes_to_frd(np.zeros(3), self.axis_order)
        gyro_key = str(gyro_unit).upper()
        if gyro_key not in self._GYRO_UNIT_SCALE:
            raise ValueError(
                f"Unsupported GREAT gyro unit '{gyro_unit}', expected one of "
                f"{sorted(self._GYRO_UNIT_SCALE)}"
            )
        accel_key = str(accel_unit).upper()
        if accel_key not in self._ACCEL_UNIT_SCALE:
            raise ValueError(
                f"Unsupported GREAT accel unit '{accel_unit}', expected one of "
                f"{sorted(self._ACCEL_UNIT_SCALE)}"
            )
        self._gyro_scale = self._GYRO_UNIT_SCALE[gyro_key]
        self._accel_scale = self._ACCEL_UNIT_SCALE[accel_key]
        self._previous_sow: Optional[float] = None

    def decode(self, line: str) -> Optional[SensorData]:
        text = line.strip()
        if not text or text.startswith("#"):
            return None
        fields = text.split()
        if len(fields) != 7:
            raise ValueError(
                f"GREAT rate IMU row must have exactly 7 columns, "
                f"got {len(fields)}"
            )
        values = np.array([float(field) for field in fields], dtype=np.float64)
        if not np.all(np.isfinite(values)):
            raise ValueError("GREAT rate IMU row must contain finite values")
        sow = float(values[0])
        if self._previous_sow is not None and sow <= self._previous_sow:
            raise ValueError(
                "GREAT rate IMU timestamps must strictly increase, "
                f"got {sow} after {self._previous_sow}"
            )
        self._previous_sow = sow
        gyro = axes_to_frd(values[1:4], self.axis_order) * self._gyro_scale
        accel = axes_to_frd(values[4:7], self.axis_order) * self._accel_scale
        return SensorData(tag="imu", imu=ImuMeasurement(
            timestamp=gpst_to_unix(self.week, sow),
            week=self.week,
            payload=RateImuData(gyro=gyro, accel=accel),
        ))


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
