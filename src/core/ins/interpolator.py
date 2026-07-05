"""IMU 插值工具。

参考 KF-GINS gi_engine.h imuInterpolate / isToUpdate。
速率式 IMU: gyro/accel 不变, 仅修改 timestamp。
"""
from typing import Optional

from src.core.data_types import ImuMeasurement


def is_to_update(t0: float, t2: float, t_gnss: float,
                 threshold: float = 1e-3) -> int:
    """判断 GNSS 时间戳与 IMU 时间区间的关系。

    Args:
        t0: imu_pre.timestamp
        t2: imu_cur.timestamp
        t_gnss: GNSS 时间戳
        threshold: 时间对齐阈值 (s), 默认 1ms

    Returns:
        0: t_gnss 不在 [t0, t2] 之间
        1: t_gnss 靠近 t0 (|t0 - t_gnss| < threshold)
        2: t_gnss 靠近 t2 (|t2 - t_gnss| <= threshold)
        3: t0 < t_gnss < t2 (需要增量切分)
    """
    if abs(t0 - t_gnss) < threshold:
        return 1
    elif abs(t2 - t_gnss) <= threshold:
        return 2
    elif t0 < t_gnss < t2:
        return 3
    else:
        return 0


def imu_interpolate(imu_pre: ImuMeasurement, imu_cur: ImuMeasurement,
                    t_gnss: float) -> Optional[ImuMeasurement]:
    """增量切分插值, 返回 t_gnss 时刻的 IMU 历元。

    速率式 IMU: gyro/accel 不变, 仅创建 timestamp=t_gnss 的新历元。

    Args:
        imu_pre: 前一个 IMU 历元 (timestamp < t_gnss)
        imu_cur: 当前 IMU 历元 (timestamp >= t_gnss)
        t_gnss: GNSS 时间戳

    Returns:
        t_gnss 时刻的 ImuMeasurement, 或 None (时间戳不在区间内)
    """
    case = is_to_update(imu_pre.timestamp, imu_cur.timestamp, t_gnss)
    if case == 0:
        return None
    elif case == 1:
        return ImuMeasurement(
            timestamp=t_gnss,
            week=imu_pre.week,
            accel=imu_pre.accel.copy(),
            gyro=imu_pre.gyro.copy(),
        )
    elif case == 2:
        return ImuMeasurement(
            timestamp=t_gnss,
            week=imu_cur.week,
            accel=imu_cur.accel.copy(),
            gyro=imu_cur.gyro.copy(),
        )
    else:  # case == 3
        # 增量切分: 速率式 IMU 的 gyro/accel 不变, 仅修改 timestamp
        return ImuMeasurement(
            timestamp=t_gnss,
            week=imu_cur.week,
            accel=imu_cur.accel.copy(),
            gyro=imu_cur.gyro.copy(),
        )


def find_bracket_imus(imu_list, t_gnss: float
                      ) -> Optional[tuple]:
    """在 IMU 列表中寻找包夹 t_gnss 的两个历元。

    满足初始化.md 第 4.5.1 节"插值前后包夹规则"。

    Returns:
        (imu_pre, imu_cur, case) 或 None (不满足包夹条件)
    """
    if len(imu_list) < 2:
        return None

    for i in range(len(imu_list) - 1):
        t0 = imu_list[i].timestamp
        t2 = imu_list[i + 1].timestamp
        case = is_to_update(t0, t2, t_gnss)
        if case in (1, 2, 3):
            return (imu_list[i], imu_list[i + 1], case)

    # 检查是否 t_gnss 在所有 IMU 之前或之后
    if t_gnss < imu_list[0].timestamp or t_gnss > imu_list[-1].timestamp:
        return None

    return None
