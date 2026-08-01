"""IMU 时间对齐工具（GVINS 风格线性插值 + 最近邻匹配）。

GVINS 风格线性插值（imu_interpolate_linear）：
- 项目统一时间对齐策略，初始化与主循环机械编排共用
- 参考 tools/GVINS/estimator/src/estimator_node.cpp process() lines 361-374
- 当 IMU 时间戳跨越 GNSS 时间戳时，在 GNSS 时刻线性插值 IMU 数据
- 反距离权重: w1=dt_2/(dt_1+dt_2), w2=dt_1/(dt_1+dt_2)

最近邻策略（imu_interpolate）：
- 历史保留接口，不再被项目代码使用
- 原 GINav 增量切分不适用于速率式 IMU
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

    Deprecated: 项目统一使用 imu_interpolate_linear 线性插值策略。
    本函数仅为兼容历史接口保留，新代码请使用 imu_interpolate_linear。

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


def imu_interpolate_linear(imu_pre: ImuMeasurement,
                           imu_cur: ImuMeasurement,
                           t_target: float) -> Optional[ImuMeasurement]:
    """GVINS 风格线性插值：在 t_target 处线性插值 IMU 数据。

    参考 tools/GVINS/estimator/src/estimator_node.cpp process() lines 361-374:
        dt_1 = t_target - imu_pre.t   (前一 IMU 到目标时刻)
        dt_2 = imu_cur.t - t_target   (目标时刻到当前 IMU)
        w1 = dt_2 / (dt_1 + dt_2)     # imu_pre 权重 (反距离)
        w2 = dt_1 / (dt_1 + dt_2)     # imu_cur 权重 (反距离)
        accel = w1 * imu_pre.accel + w2 * imu_cur.accel
        gyro  = w1 * imu_pre.gyro  + w2 * imu_cur.gyro

    Args:
        imu_pre: 前一 IMU 历元 (timestamp <= t_target)
        imu_cur: 当前 IMU 历元 (timestamp >= t_target)
        t_target: 插值目标时刻（通常为 GNSS 时间戳）

    Returns:
        t_target 时刻的 ImuMeasurement（线性插值数据），或 None（区间不包含 t_target）
    """
    t0 = imu_pre.timestamp
    t2 = imu_cur.timestamp
    if not (t0 <= t_target <= t2) or t2 <= t0:
        return None
    dt_1 = t_target - t0
    dt_2 = t2 - t_target
    w1 = dt_2 / (dt_1 + dt_2)
    w2 = dt_1 / (dt_1 + dt_2)
    return ImuMeasurement(
        timestamp=t_target,
        week=imu_pre.week,
        accel=w1 * imu_pre.accel + w2 * imu_cur.accel,
        gyro=w1 * imu_pre.gyro + w2 * imu_cur.gyro,
    )
