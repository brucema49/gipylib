"""IMU 时间对齐工具（最近邻匹配策略）。

原线性插值策略参考 gnss_ins_lc_nhc navdataque.cc SortData，但经调查发现：
- gnss_ins_lc_nhc 使用增量式 IMU（gyro_/acce_ 实为 dtheta/dvel），SortData
  做的是增量切分（按比例分割增量），不适用于速率式 IMU
- tools/GINav 也使用增量式 IMU（imu.dw/dv），初始化时无时间插值

因此改为 GNSS 时间最近邻匹配 IMU 数据，不进行精细化插值。
时间对齐误差（最大半个 IMU 采样周期 ≈ 5ms @100Hz）后续由 KF 在线估计。
"""
from typing import Optional

from src.core.data_types import ImuMeasurement


def is_to_update(t0: float, t2: float, t_gnss: float,
                 threshold: float = 1e-3) -> int:
    """判断 GNSS 时间戳与 IMU 时间区间的关系（最近邻版）。

    Args:
        t0: imu_pre.timestamp
        t2: imu_cur.timestamp
        t_gnss: GNSS 时间戳
        threshold: 保留参数（最近邻策略不使用，兼容接口）

    Returns:
        0: t_gnss 不在 [t0, t2] 之间
        1: 最近邻为 imu_pre（t_gnss 靠近 t0）
        2: 最近邻为 imu_cur（t_gnss 靠近 t2）
    """
    if t0 <= t_gnss <= t2:
        dt0 = abs(t0 - t_gnss)
        dt2 = abs(t2 - t_gnss)
        return 1 if dt0 <= dt2 else 2
    return 0


def imu_interpolate(imu_pre: ImuMeasurement, imu_cur: ImuMeasurement,
                    t_gnss: float) -> Optional[ImuMeasurement]:
    """最近邻匹配：返回时间戳最接近 t_gnss 的 IMU 历元（不插值）。

    速率式 IMU 最近邻策略：
    - 在包夹区间 [imu_pre, imu_cur] 内，选时间戳最接近 t_gnss 的历元
    - 时间对齐误差（最大半个 IMU 采样周期）后续由 KF 估计

    Args:
        imu_pre: 前一个 IMU 历元 (timestamp <= t_gnss)
        imu_cur: 当前 IMU 历元 (timestamp >= t_gnss)
        t_gnss: GNSS 时间戳

    Returns:
        t_gnss 时刻标记的 ImuMeasurement（数据来自最近邻历元），或 None
    """
    case = is_to_update(imu_pre.timestamp, imu_cur.timestamp, t_gnss)
    if case == 0:
        return None

    dt_pre = abs(imu_pre.timestamp - t_gnss)
    dt_cur = abs(imu_cur.timestamp - t_gnss)
    nearest = imu_pre if dt_pre <= dt_cur else imu_cur

    return ImuMeasurement(
        timestamp=t_gnss,
        week=nearest.week,
        accel=nearest.accel.copy(),
        gyro=nearest.gyro.copy(),
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
        if case in (1, 2):
            return (imu_list[i], imu_list[i + 1], case)

    # 检查是否 t_gnss 在所有 IMU 之前或之后
    if t_gnss < imu_list[0].timestamp or t_gnss > imu_list[-1].timestamp:
        return None

    return None
