# IMU 机械编排方案

> 实现 INS 机械编排和初始化，支持增量式和速率式两种 IMU 数据格式，
> 以及 IMU/GNSS 时间对齐插值。
> 参考 GREAT-MSF 的 t_gsins、t_gimu、t_gbase、t_ginterp 类设计。
>
> **时间系统约定**：全框架统一使用 GPS 秒（GPST，since 1980-01-06），不使用 Unix epoch 或本地时间。
>
> **框架设计模式集成**：
> - **策略模式（仅前端）**：IMU 机械编排封装为 `ImuMechStrategy(OdometryStrategy)`，作为前端里程计策略，IMU 不可用时可降级为 `GnssPositioningStrategy`
> - **纯队列流水线**：IMU 传感器 `ImuSensor(BaseSensor)` **不作为 Subject**，无 `notify()`，数据通过 `output_queue.put()` 推入 imu_queue → Scheduler 转发到 estimate_queue → `LcIntegration.process_epoch()` 处理
> - **依赖注入**：`ImuMechStrategy` 通过构造函数接收 `InsCore` 与 `ImuPreprocessor` 实例
> - **OOP 三大特性**：封装（机械编排状态封装在 InsCore）、继承（ImuPreprocessor ABC → 速率/增量子类）、多态（框架持有 OdometryStrategy 抽象引用）

---

## 目录

- [1. 概述](#1-概述)
- [2. 坐标系定义](#2-坐标系定义)
- [3. ABC 类层次结构](#3-abc-类层次结构)
- [4. IMU 数据预处理](#4-imu-数据预处理)
- [5. IMU/GNSS 插值与时间对齐](#5-imugnss-插值与时间对齐)
- [6. INS 机械编排](#6-ins-机械编排)
- [7. 状态转移矩阵 F](#7-状态转移矩阵-f)
- [8. INS 初始化](#8-ins-初始化)
- [9. 地球参数](#9-地球参数)
- [10. 姿态表示与转换](#10-姿态表示与转换)
- [11. 参考代码映射](#11-参考代码映射)
- [12. 队列流水线集成](#12-队列流水线集成)

---

## 1. 概述

### 1.1 模块文件

| 文件 | 职责 | 参考 |
|------|------|------|
| `ins_core.py` | INS 核心：姿态/速度/位置更新、粗对准 | GREAT-MSF t_gsins |
| `imu_preprocess.py` | IMU 数据预处理（增量/速率转换、异常检测） | GREAT-MSF t_gimu |
| `interpolator.py` | IMU/GNSS 时间对齐插值 | GREAT-MSF t_ginterp / t_gpoly |
| `earth_param.py` | 地球参数（重力、自转角速度、曲率半径） | GREAT-MSF t_gbase |
| `attitude.py` | 姿态表示与转换（四元数/欧拉角/DCM） | GREAT-MSF t_gbase |

### 1.2 与 Estimator 的关系

```
IMU/GNSS 数据流（纯队列流水线，时间对齐在 Estimator 内部）:
  Stream → imu_queue → Scheduler(仅转发) → estimate_queue
                                                  ↓
                                  LcIntegration.process_epoch()
                                       ↓                ↓
                                  ImuPreprocessor   Interpolator
                                  (数据格式转换)    (时间对齐插值,在 Estimator 内部)
                                       ↓                ↓
                                    InsCore ←─────── 时间对齐后的 IMU
                                    (正向递推)
                                       ↓
                                  双滤波 EKF (P1 主滤波 + P2 NHC 子滤波)
                                       ↓
                                  双滤波独立反馈
```

- `InsCore` 负责 INS 正向递推（姿态、速度、位置更新）
- `Interpolator` 负责 IMU/GNSS 时间对齐（**在 Estimator 内部调用**，Scheduler 仅转发）
- `InsKf`/`LcEstimator` 负责构造双滤波状态转移矩阵 F1/F2（误差传播）
- InsCore 是正向递推，InsKf/LcEstimator 是误差传播，两者共享相同的物理模型
- IMU 数据通过 `queue.put()` 流转，**无观察者回调、无 notify()**

---

## 2. 坐标系定义

### 2.1 坐标系约定

| 坐标系 | 符号 | 定义 | 轴向 |
|--------|------|------|------|
| **ECEF 坐标系** | e 系 | 地心地固坐标系 | X→格林威治, Z→北极 |
| **体坐标系** | b 系 | IMU 坐标系 | FRD（前-右-下） |
| **车体坐标系** | v 系 | 车辆本体坐标系 | FRD（前-右-下） |
| **导航坐标系** | n 系 | 当地水平坐标系 | ENU（东-北-天），仅输出用 |
| **惯性坐标系** | i 系 | ECI 惯性坐标系 | — |

**本项目约定（参考 gnss_ins_lc_nhc）**：
- **机械编排在 E 系（ECEF）下进行**，不在 n 系下
- 体坐标系：**FRD**（前-右-下），与航空/惯导常用约定一致
- 车体坐标系：**FRD**（前-右-下），与 b 系通过安装角旋转矩阵 R_b^v 关联
- 导航系 ENU：仅用于输出显示，不参与内部计算
- 姿态角顺序：**航向(yaw) → 俯仰(pitch) → 横滚(roll)**

### 2.2 姿态矩阵 C_b^e

```
E 系下姿态矩阵 C_b^e（b 系到 e 系），参考 gnss_ins_lc_nhc navmech.cc

姿态更新在 E 系下进行:
  C_b^e(k+1) = C_b^e(k) * ΔC_b(k→k+1)

其中 ΔC_b 由 IMU 角增量构造（Rodrigues 公式）
```

### 2.3 安装角旋转矩阵 R_b^v

```
b 系（IMU 本体）与 v 系（车体）之间通过安装角旋转矩阵 R_b^v 关联

R_b^v 由安装角 [roll=0, pitch, yaw] 构造:
  R_b^v = R_z(yaw) * R_y(pitch)  （roll 假设为 0）

安装角只有 2 维（pitch, yaw），参考 gnss_ins_lc_nhc
安装角误差 δθ_imu = [δpitch, δyaw]
```

---

## 3. ABC 类层次结构

### 3.1 类继承关系总览

```
ImuPreprocessor (ABC)              Interpolator (ABC)              AttitudeUtil (静态)
├── RateImuPreprocessor            ├── LinearInterpolator          EarthParam (静态)
└── DeltaImuPreprocessor           └── PolyInterpolator

InsCore (参考 t_gsins)
  ├── attitude_update()
  ├── velocity_update()
  ├── position_update()
  ├── coarse_align_static()
  └── coarse_align_dynamic()
```

### 3.2 InsCore（参考 GREAT-MSF t_gsins）

```python
class InsCore:
    """INS 核心类，参考 GREAT-MSF t_gsins

    职责:
      - INS 正向递推（姿态、速度、位置更新，E 系下）
      - 粗对准（静态/动态）
      - 维护 INS 状态

    属性:
      state: InsState          # 当前 INS 状态
      preprocessor: ImuPreprocessor  # IMU 预处理器
      earth: EarthParam        # 地球参数
    """

    def __init__(self, preprocessor: ImuPreprocessor, earth: EarthParam): ...

    def update(self, imu_data: ImuMeasurement) -> InsState:
        """一步 INS 递推：姿态→速度→位置（E 系下）"""

    def attitude_update(self, C_be: np.ndarray, omega_ib_b: np.ndarray,
                        dt: float) -> np.ndarray:
        """姿态更新（E 系下）"""

    def velocity_update(self, v_e: np.ndarray, f_b: np.ndarray,
                        C_be: np.ndarray, r_e: np.ndarray,
                        dt: float) -> np.ndarray:
        """速度更新（E 系下）"""

    def position_update(self, r_e: np.ndarray, v_e: np.ndarray,
                        dt: float) -> np.ndarray:
        """位置更新（E 系下，ECEF 递推）"""

    def coarse_align_static(self, imu_buffer: list,
                            duration: float = 30.0) -> np.ndarray:
        """静态粗对准（E 系下）"""

    def coarse_align_dynamic(self, gnss_velocity: np.ndarray) -> np.ndarray:
        """动态粗对准（E 系下）"""
```

### 3.3 ImuPreprocessor（ABC，参考 GREAT-MSF t_gimu）

```python
from abc import ABC, abstractmethod

class ImuPreprocessor(ABC):
    """IMU 数据预处理器抽象基类，参考 GREAT-MSF t_gimu

    职责:
      - IMU 数据格式转换（速率式 ↔ 增量式）
      - 异常检测（NaN/Inf、幅值范围、时间戳单调性）
      - 零偏补偿
    """

    def __init__(self, imu_rate: float = 200.0): ...

    @abstractmethod
    def preprocess(self, imu_data: ImuMeasurement) -> ImuMeasurement:
        """预处理 IMU 数据（异常检测 + 零偏补偿）"""

    @abstractmethod
    def to_rate(self, imu_data: ImuMeasurement) -> tuple[np.ndarray, np.ndarray]:
        """转换为速率式 (ω, f)，返回 (angular_velocity, acceleration)"""

    @abstractmethod
    def to_delta(self, imu_data: ImuMeasurement) -> tuple[np.ndarray, np.ndarray]:
        """转换为增量式 (Δθ, Δv)，返回 (delta_theta, delta_v)"""

    def check_imu_data(self, imu_data: ImuMeasurement) -> bool:
        """IMU 数据异常检测（具体实现）"""


class RateImuPreprocessor(ImuPreprocessor):
    """速率式 IMU 预处理器

    适用于输出角速度 ω (rad/s) 和比力 f (m/s²) 的 IMU（如 MEMS IMU）
    """

    def preprocess(self, imu_data: ImuMeasurement) -> ImuMeasurement: ...
    def to_rate(self, imu_data: ImuMeasurement) -> tuple[np.ndarray, np.ndarray]: ...
    def to_delta(self, imu_data: ImuMeasurement) -> tuple[np.ndarray, np.ndarray]: ...


class DeltaImuPreprocessor(ImuPreprocessor):
    """增量式 IMU 预处理器

    适用于输出角增量 Δθ (rad) 和速度增量 Δv (m/s) 的 IMU（如光纤陀螺 IMU）
    """

    def preprocess(self, imu_data: ImuMeasurement) -> ImuMeasurement: ...
    def to_rate(self, imu_data: ImuMeasurement) -> tuple[np.ndarray, np.ndarray]: ...
    def to_delta(self, imu_data: ImuMeasurement) -> tuple[np.ndarray, np.ndarray]: ...
```

### 3.4 Interpolator（ABC，参考 GREAT-MSF t_ginterp / t_gpoly）

```python
from abc import ABC, abstractmethod

class Interpolator(ABC):
    """插值器抽象基类，参考 GREAT-MSF t_ginterp

    职责:
      - IMU/GNSS 时间对齐插值
      - 当 GNSS 历元到达时，将 IMU 数据插值到 GNSS 时间戳
      - 支持线性、多项式等插值方法
    """

    @abstractmethod
    def interpolate(self, data_before: np.ndarray, t_before: float,
                    data_after: np.ndarray, t_after: float,
                    t_target: float) -> np.ndarray:
        """在两个相邻历元之间插值

        参数:
            data_before: 前一历元数据 [N]
            t_before:    前一历元时间戳
            data_after:  后一历元数据 [N]
            t_after:     后一历元时间戳
            t_target:    目标插值时间戳（通常为 GNSS 时间戳）

        返回:
            插值结果 [N]
        """


class LinearInterpolator(Interpolator):
    """线性插值器，参考 GREAT-MSF ginterp 线性插值

    公式:
      x(t) = x(t1) + (x(t2) - x(t1)) * (t - t1) / (t2 - t1)

    适用场景:
      - IMU 比力/角速度插值（高采样率下线性近似足够精确）
      - IMU 时间戳对齐到 GNSS 历元
    """

    def interpolate(self, data_before: np.ndarray, t_before: float,
                    data_after: np.ndarray, t_after: float,
                    t_target: float) -> np.ndarray: ...


class PolyInterpolator(Interpolator):
    """多项式插值器，参考 GREAT-MSF t_gpoly

    支持任意阶多项式拟合，使用多个历元数据进行多项式回归。

    公式:
      x(t) = a0 + a1*t + a2*t² + ... + an*t^n
      系数通过最小二乘拟合历史数据确定

    适用场景:
      - 需要更高精度插值的场景
      - 低采样率 IMU 数据插值
      - 位置/速度插值（运动轨迹更平滑）

    属性:
        order: int  # 多项式阶数（默认3）
        window: int # 拟合窗口大小（默认 order+1）
    """

    def __init__(self, order: int = 3, window: int = 4): ...

    def interpolate(self, data_before: np.ndarray, t_before: float,
                    data_after: np.ndarray, t_after: float,
                    t_target: float) -> np.ndarray: ...

    def fit(self, data_list: list[np.ndarray], t_list: list[float]) -> None:
        """使用多个历元数据拟合多项式系数"""
```

### 3.5 AttitudeUtil（参考 GREAT-MSF t_gbase）

```python
class AttitudeUtil:
    """姿态工具类（静态方法），参考 GREAT-MSF t_gbase

    提供姿态表示之间的转换和基本运算。
    所有方法为静态方法，不持有状态。
    """

    @staticmethod
    def quat2dcm(q: np.ndarray) -> np.ndarray: ...

    @staticmethod
    def dcm2quat(C: np.ndarray) -> np.ndarray: ...

    @staticmethod
    def euler2dcm(yaw: float, pitch: float, roll: float) -> np.ndarray: ...

    @staticmethod
    def dcm2euler(C: np.ndarray) -> tuple[float, float, float]: ...

    @staticmethod
    def quat_mult(q1: np.ndarray, q2: np.ndarray) -> np.ndarray: ...

    @staticmethod
    def skew_symmetric(v: np.ndarray) -> np.ndarray: ...

    @staticmethod
    def orthogonalize(C: np.ndarray) -> np.ndarray: ...
```

### 3.6 EarthParam（参考 GREAT-MSF t_gbase）

```python
class EarthParam:
    """地球参数类（静态方法），参考 GREAT-MSF t_gbase

    提供 WGS84 地球参数和导航相关计算。
    所有方法为静态方法，不持有状态。
    """

    @staticmethod
    def rn(lat: float) -> float: ...

    @staticmethod
    def rm(lat: float) -> float: ...

    @staticmethod
    def earth_rate(lat: float) -> np.ndarray:
        """n 系下地球自转角速度（仅输出用）"""
        ...

    @staticmethod
    def earth_rate_ecef() -> np.ndarray:
        """E 系下地球自转角速度 [0, 0, ω_e]（常数）"""
        ...

    @staticmethod
    def gravity(lat: float, alt: float = 0.0) -> np.ndarray:
        """n 系下正常重力 [0, 0, -g]（仅输出用）"""
        ...

    @staticmethod
    def gravity_ecef(r_e: np.ndarray) -> np.ndarray:
        """E 系下正常重力向量（含离心力项，主计算用）

        参考 gnss_ins_lc_nhc navmech.cc
        """
        ...

    @staticmethod
    def navigation_rate(v_n: np.ndarray, pos: np.ndarray) -> np.ndarray:
        """n 系下导航系旋转角速度（仅输出用）"""
        ...
```

---

## 4. IMU 数据预处理

### 4.1 两种 IMU 数据格式

| 格式 | 输出量 | 物理含义 | 典型传感器 | 对应预处理器 |
|------|--------|---------|-----------|-------------|
| **速率式** | ω (rad/s), f (m/s²) | 角速度、比力 | MEMS IMU, 战术级 IMU | RateImuPreprocessor |
| **增量式** | Δθ (rad), Δv (m/s) | 角增量、速度增量 | 光纤陀螺 IMU, 高精度 IMU | DeltaImuPreprocessor |

### 4.2 增量式 ↔ 速率式转换

```
增量式 → 速率式:
  ω = Δθ / dt
  f = Δv / dt

速率式 → 增量式:
  Δθ = ω * dt
  Δv = f * dt
```

- `RateImuPreprocessor.to_delta()`: ω*dt → Δθ, f*dt → Δv
- `DeltaImuPreprocessor.to_rate()`: Δθ/dt → ω, Δv/dt → f
- 各自的 `to_rate()` / `to_delta()` 直接返回自身格式数据

### 4.3 异常检测

ImuPreprocessor.check_imu_data() 检测内容：

1. 时间戳单调递增
2. 加速度幅值在合理范围 (0 < |f| < 100 m/s²)
3. 角速度幅值在合理范围 (|ω| < 5 rad/s)
4. NaN/Inf 检测

---

## 5. IMU/GNSS 插值与时间对齐

### 5.1 问题背景

IMU 和 GNSS 具有不同的采样率（IMU 通常 100~200Hz，GNSS 通常 1~10Hz），
且两者的时间戳通常不对齐。在松组合融合中，需要在 GNSS 历元时刻获取
对应的 INS 预测状态，因此需要将 IMU 数据插值到 GNSS 时间戳。

**核心问题**：当 GNSS 量测时刻 t1 落在两个 IMU 时刻 t0 和 t2 之间时，
如何在不丢失 t2 数据的前提下进行精确插值？

**两种 GNSS 数据源模式下的插值需求**：

| 模式 | GNSS 数据来源 | 插值需求 |
|------|-------------|---------|
| 内部解算模式 | GnssProcessor 内部解算 | IMU 插值到 GNSS 观测历元 |
| 外部结果模式 | GnssExternalProvider 读取外部文件 | IMU 插值到外部结果时间戳 |

两种模式的插值逻辑完全一致，区别仅在于 GNSS 时间戳的来源。

### 5.2 插值策略（参考 KF-GINS 增量切分）

> 参考 KF-GINS 的 `GIEngine::newImuProcess()` 和 `imuInterpolate()` 实现。

**KF-GINS 的核心思想**：**增量切分而非弹出**。

```
IMU 时间线:  t0 ----------- t2 ---- t3
                  ↑
GNSS 历元:   t1 (t0 < t1 < t2)

错误做法（会丢失 t2 数据）:
  弹出 t2 → 插值到 t1 → 机械编排 t0→t1
  下一次缺少 t2 数据，无法编排 t2→t3

正确做法（KF-GINS 增量切分）:
  切分 t2 的增量:
    midimu = t0→t1 增量（前半段）
    imucur 保留 t1→t2 剩余增量（后半段）
  机械编排: t0→t1 → GNSS更新 → t1→t2
  下一次: imupre=imucur(t2剩余), imucur=t3, 正常编排 t2→t3
```

### 5.3 ImuMeasurement 数据结构

```python
@dataclass
class ImuMeasurement:
    """IMU 测量数据

    使用角速度/加速度形式（非增量形式）
    与 KF-GINS 的 dtheta/dvel 增量形式不同
    """
    timestamp: float                    # 秒 (Unix epoch 或 GPST)
    angular_velocity: np.ndarray        # [3] 角速度 (rad/s), 机体坐标系
    acceleration: np.ndarray            # [3] 加速度 (m/s^2), 机体坐标系
    dt: float = 0.0                     # 距上一时刻的时间间隔 (秒)

    # dt 字段说明:
    #   - 正常情况下 dt = timestamp - prev.timestamp
    #   - 增量切分后 dt 会被修改（参考 5.4 节）
    #   - 机械编排时通过 dt 计算增量: dtheta = omega * dt, dvel = f * dt
```

### 5.4 IMU 增量切分（参考 KF-GINS imuInterpolate）

```python
def imu_interpolate(imu_pre: ImuMeasurement,
                    imu_cur: ImuMeasurement,
                    timestamp: float) -> ImuMeasurement:
    """IMU 增量切分（参考 KF-GINS imuInterpolate）

    关键：imu_cur 被原地修改，保留剩余增量！

    与 KF-GINS 的差异:
      - KF-GINS 使用增量形式 (dtheta/dvel)，按比例切分增量
      - 本项目使用角速度/加速度形式，切分时只修改 dt
      - 机械编排时通过 dt 计算增量: dtheta = omega * dt
    """
    dt_total = imu_cur.timestamp - imu_pre.timestamp
    lamda = (timestamp - imu_pre.timestamp) / dt_total

    # 创建中间时刻 IMU（前半段）
    midimu = ImuMeasurement(
        timestamp=timestamp,
        angular_velocity=imu_cur.angular_velocity,  # 角速度不变
        acceleration=imu_cur.acceleration,          # 加速度不变
        dt=timestamp - imu_pre.timestamp            # 前半段 dt
    )

    # 关键：imu_cur 原地保留剩余增量（只修改 dt）
    imu_cur.dt = imu_cur.timestamp - timestamp

    return midimu
```

### 5.5 四种时间对齐情况

参考 KF-GINS 的 `isToUpdate()` 函数，判断 GNSS 更新时机：

```python
def is_to_update(t0: float, t2: float, t_gnss: float,
                 threshold: float = 1e-3) -> int:
    """判断 GNSS 更新时机（参考 KF-GINS isToUpdate）

    返回值:
        0: GNSS 时间不在 [t0, t2] 之间，只做 INS 传播
        1: GNSS 时间靠近 t0，先 GNSS 更新再 INS 传播
        2: GNSS 时间靠近 t2，先 INS 传播再 GNSS 更新
        3: GNSS 时间在 (t0, t2) 之间但不靠近任一，需增量切分
    """
    if abs(t0 - t_gnss) < threshold:
        return 1  # 靠近 t0
    elif abs(t2 - t_gnss) <= threshold:
        return 2  # 靠近 t2
    elif t0 < t_gnss < t2:
        return 3  # 在中间
    else:
        return 0  # 不在区间内
```

### 5.6 四种情况处理流程图

```
GNSS(t1) 到达，IMU 缓冲区有 t0, t2
─────────────────────────────────────────────────────────────

情况1 (res=1): |t1 - t0| < 阈值 (GNSS 靠近 t0)
  ┌─────────────────────────────────────┐
  │ 1. GNSS 更新 (使用 t0 时刻状态)     │
  │ 2. 状态反馈                          │
  │ 3. INS 传播: t0 → t2 (完整增量)     │
  └─────────────────────────────────────┘

情况2 (res=2): |t1 - t2| < 阈值 (GNSS 靠近 t2)
  ┌─────────────────────────────────────┐
  │ 1. INS 传播: t0 → t2 (完整增量)     │
  │ 2. GNSS 更新 (使用 t2 时刻状态)     │
  │ 3. 状态反馈                          │
  └─────────────────────────────────────┘

情况3 (res=3): t0 < t1 < t2 且不靠近任一 (GNSS 在中间)
  ┌─────────────────────────────────────┐
  │ 1. 增量切分:                         │
  │    midimu = t0→t1 增量               │
  │    imucur 保留 t1→t2 剩余增量        │
  │                                      │
  │ 2. INS 传播: t0 → t1 (前半段)        │
  │ 3. GNSS 更新 (t1 时刻状态)           │
  │ 4. 状态反馈                          │
  │ 5. INS 传播: t1 → t2 (后半段)        │
  │    (imucur 已切分，dt 变小)          │
  └─────────────────────────────────────┘

情况0 (res=0): GNSS 时间不在 [t0, t2] 之间
  ┌─────────────────────────────────────┐
  │ 只做 INS 传播: t0 → t2 (完整增量)   │
  └─────────────────────────────────────┘

下一次 IMU 数据 t3 到达:
  ┌─────────────────────────────────────┐
  │ imupre = imucur (t2, 剩余增量)      │
  │ imucur = 新数据 t3                   │
  │ 正常处理 t2 → t3                     │
  └─────────────────────────────────────┘
```

### 5.7 与 KF-GINS 的对比

| 维度 | KF-GINS | 本项目 |
|------|---------|--------|
| **IMU 数据形式** | 增量 (dtheta/dvel) | 角速度/加速度 |
| **切分方式** | 按比例切分增量 | 只修改 dt |
| **坐标系** | n 系 | E 系 |
| **架构** | 单线程顺序处理 | 多线程流式 |
| **GNSS 暂存** | gnssdata_ 成员变量 | pending_gnss 成员变量 |
| **时间对齐阈值** | TIME_ALIGN_ERR | time_align_threshold |
| **4 种情况处理** | isToUpdate() | is_to_update() |

### 5.8 详细实现位置

时间同步与 IMU 插值的完整实现位于估计器中，详见 [estimator.md 第 9 节](file:///e:/program_project/python/GInsStream/skills/estimator.md#9-时间同步与-imu-插值)。

### 5.9 插值注意事项

- 插值应在原始 IMU 测量域进行（比力、角速度），而非在导航结果域
- 本项目采用增量切分方式，角速度/加速度本身不变，只修改 dt
- 增量切分后采样间隔会变化，严格上不满足 INSMech 的等间隔假设，但影响较小（参考 KF-GINS 注释）
- 插值边界处理：当 GNSS 历元超出 IMU 缓冲区范围时，使用最近邻外推
- 参考 GREAT-MSF ginterp：还支持样条插值（SplineInterpolator），可后续扩展

---

## 6. INS 机械编排

### 6.1 机械编排方程

参考 gnss_ins_lc_nhc navmech.cc，INS 机械编排在 **E 系（ECEF）** 下进行：

```
1. 姿态更新:  C_b^e(k+1) = C_b^e(k) * ΔC_b(k→k+1)
2. 速度更新:  v^e(k+1) = v^e(k) + (f^e - 2*ω_ie^e × v^e + g^e) * dt
3. 位置更新:  r^e(k+1) = r^e(k) + v^e(k+1) * dt
```

**E 系 vs n 系机械编排差异**：
- E 系无需计算导航系旋转角速度 ω_en^n（不存在）
- E 系地球自转 ω_ie^e = [0, 0, ω_e] 为常数
- E 系重力 g^e 为位置函数（通过重力模型计算）
- E 系 Coriolis 项简化为 -2*ω_ie^e × v^e

### 6.2 姿态更新

```
公式（E 系下，参考 gnss_ins_lc_nhc navmech.cc）:
  C_b^e(k+1) = C_b^e(k) * ΔC_b(k→k+1)

其中:
  ΔC_b(k→k+1) 为体坐标系下 b 系从 k 到 k+1 的旋转矩阵

计算步骤:
  1. E 系角速度在体坐标系下的投影:
     ω_ie_b = C_e^b * ω_ie^e = (C_b^e)^T * [0, 0, ω_e]

  2. 体坐标系下 b 系相对 e 系角速度:
     ω_eb_b = ω_ib_b - ω_ie_b

  3. 旋转矢量:
     φ = ω_eb_b * dt

  4. 构造旋转矩阵（Rodrigues 公式）:
     |φ| < ε:  ΔC = I + [φ×]
     |φ| ≥ ε:  ΔC = I + sin(|φ|)/|φ| * [φ×] + (1-cos(|φ|))/|φ|² * [φ×]²

  5. 姿态更新:
     C_b^e(k+1) = C_b^e(k) * ΔC

  6. 正交化（防止数值漂移）:
     C_b^e(k+1) = U * V^T  (SVD 分解)
```

### 6.3 速度更新

```
公式（E 系下，参考 gnss_ins_lc_nhc navmech.cc）:
  v^e(k+1) = v^e(k) + (f^e - 2*ω_ie^e × v^e + g^e) * dt

其中:
  f^e = C_b^e * f^b                              比力在 E 系投影
  ω_ie^e = [0, 0, ω_e]^T                          地球自转角速度（E 系常数）
  g^e = EarthParam.gravity_ecef(r^e)               E 系下正常重力向量
  -2*ω_ie^e × v^e                                  Coriolis 加速度

计算步骤:
  1. f^e = C_b^e * f_b_corrected
  2. ω_ie^e = [0, 0, ω_e]^T    (常数，ω_e = 7.2921151467e-5 rad/s)
  3. g^e = EarthParam.gravity_ecef(state.position)    E 系重力
  4. coriolis = -2 * skew(ω_ie^e) @ v^e
  5. dv = (f^e + coriolis + g^e) * dt
  6. v^e(k+1) = v^e(k) + dv

E 系 vs n 系速度更新差异:
  - E 系无导航系旋转角速度 ω_en^n 项（不存在）
  - E 系 Coriolis 简化为 -2*ω_ie^e × v^e（ω_ie^e 为常数）
  - E 系重力 g^e 为位置函数，需通过重力模型计算（含离心力项）
```

### 6.4 位置更新

```
公式（E 系下，参考 gnss_ins_lc_nhc navmech.cc）:
  r^e(k+1) = r^e(k) + v^e(k+1) * dt

其中:
  r^e 为 ECEF 位置向量 [x, y, z] (m)
  v^e(k+1) 为更新后的 ECEF 速度 (m/s)

E 系 vs n 系位置更新差异:
  - E 系直接在 ECEF 坐标下递推，无需曲率半径 R_M/R_N
  - E 系无需经纬度递推（φ, λ, h），直接更新 ECEF 坐标
  - E 系位置更新公式更简洁，无奇异性问题（n 系在极点有 cos(φ) 奇异）
```

### 6.5 完整递推流程（InsCore.update）

```
InsCore.update(imu_data):

  1. IMU 数据预处理
     omega_ib_b, f_b = preprocessor.to_rate(imu_data)
     omega_ib_b_corrected = omega_ib_b - state.gyro_bias
     f_b_corrected = f_b - state.accel_bias

  2. 姿态更新（E 系下）
     omega_ie_b = state.rotation.T @ [0, 0, omega_e]    # E系地球自转在b系投影
     omega_eb_b = omega_ib_b_corrected - omega_ie_b      # b系相对e系角速度
     phi = omega_eb_b * dt                                # 旋转矢量
     delta_C = rodrigues(phi)                             # 旋转矩阵
     C_be_new = state.rotation @ delta_C                  # 姿态更新
     C_be_new = orthogonalize(C_be_new)                   # 正交化

  3. 速度更新（E 系下）
     f_e = C_be_new @ f_b_corrected                       # 比力在E系投影
     omega_ie_e = [0, 0, omega_e]                         # E系地球自转（常数）
     g_e = EarthParam.gravity_ecef(state.position)        # E系重力
     coriolis = -2 * skew(omega_ie_e) @ state.velocity    # Coriolis
     v_e_new = state.velocity + (f_e + coriolis + g_e) * dt

  4. 位置更新（E 系下）
     r_e_new = state.position + v_e_new * dt

  5. 构建新状态
     state_new = InsState(timestamp, r_e_new, v_e_new, dcm2quat(C_be_new), C_be_new, ...)
```

---

## 7. 状态转移矩阵 F

### 7.1 可配置维度误差模型

```
状态向量（可配置维度，参考 gnss_ins_lc_nhc，E 系下）:
  δx = [δr^e, δv^e, δφ^e, δb_g, δb_a, δθ_imu, δl_imu, δl_gnss]^T
        0-2   3-5   6-8   9-11  12-14 15-16   17-19   20-22

基础 15 维始终估计（E 系主滤波）。
安装角(2维)和IMU杆臂(3维)可选（NHC 子滤波，通过配置开关控制）。
GNSS杆臂(3维)可选（主滤波，E系下估计，默认关闭）。
不含时间同步参数（本项目通过精确插值对齐，无需在线估计）。

配置组合:
  - 最小配置（15维）: 仅主滤波 15 状态
  - 默认配置（20维）: 15 + IMU安装角(2) + IMU杆臂(3)
  - 全配置（23维）:   15 + IMU安装角(2) + IMU杆臂(3) + GNSS杆臂(3)
```

### 7.2 F 矩阵结构

```
F = [F_rr  F_rv  0     0     0     0     0     0    ]   位置误差方程（E 系）
    [F_vr  F_vv  F_vφ  F_vb  F_va  0     0     0    ]   速度误差方程（E 系）
    [0     0     F_φφ  F_pb  0     0     0     0    ]   姿态误差方程（E 系）
    [0     0     0     F_bb  0     0     0     0    ]   陀螺零偏方程
    [0     0     0     0     F_aa  0     0     0    ]   加计零偏方程
    [0     0     0     0     0     0     0     0    ]   安装角方程（常数）
    [0     0     0     0     0     0     0     0    ]   IMU杆臂方程（常数）
    [0     0     0     0     0     0     0     0    ]   GNSS杆臂方程（常数，可选）

安装角、IMU杆臂和GNSS杆臂均假设为常数，F 中对应行为 0。
E 系下 F 矩阵比 n 系更简洁（无 ω_en^n 相关项）。
```

### 7.3 F 矩阵各子块

#### 位置误差方程 (0:3, :)

```
F_rr: 位置对位置的偏导（E 系下）
  F_rr = 0    （E 系下位置对位置的偏导为零）

F_rv: 位置对速度的偏导（E 系下）
  F_rv = I_3  （E 系下位置对速度的偏导为单位阵）
```

#### 速度误差方程 (3:6, :)

```
F_vr: 速度对位置的偏导（E 系下，重力梯度项）
  F_vr = ∂g^e/∂r^e    （E 系重力对位置的偏导，通常可忽略或简化）
  参考 gnss_ins_lc_nhc navmech.cc

F_vv: 速度对速度的偏导（E 系 Coriolis 效应）
  F_vv = -[2*ω_ie^e ×]   （E 系下地球自转为常数 [0,0,ω_e]）

F_vφ: 速度对姿态的偏导（E 系下）
  F_vφ = [f^e ×]   或 C_b^e * [f^b ×] * C_e^b
  E 系下比力反对称矩阵

F_vb: 速度对陀螺零偏的偏导（间接耦合）
  F_vb = 0   （E 系下速度对陀螺零偏无直接耦合，通过姿态间接影响）

F_va: 速度对加计零偏的偏导
  F_va = -C_b^e
```

#### 姿态误差方程 (6:9, :)

```
F_φφ: 姿态对姿态的偏导（E 系下）
  F_φφ = -[ω_ie^e ×]
  E 系下无 ω_en^n 项（n 系下此项为 -[ω_in^n ×]）

F_pb: 姿态对陀螺零偏的偏导
  F_pb = -C_b^e
```

#### 传感器零偏方程

```
F_bb: 陀螺零偏（一阶马尔可夫）
  F_bb = -I / τ_g    τ_g: 相关时间

F_aa: 加计零偏（一阶马尔可夫）
  F_aa = -I / τ_a    τ_a: 相关时间
```

#### 安装角与杆臂方程

```
安装角 (15:17): 假设为常数，F[15:17, :] = 0
  参考 gnss_ins_lc_nhc: 安装角只有2维(pitch, yaw)，roll假设为0

IMU杆臂 (17:20): 假设为常数，F[17:20, :] = 0

GNSS杆臂 (20:23): 假设为常数，F[20:23, :] = 0（可选，estimate_gnss_leverarm=true）
  GNSS杆臂在主滤波中估计，与IMU杆臂独立
```

#### E 系 vs n 系 F 矩阵关键差异

```
| 子块 | n 系 | E 系 |
|------|------|------|
| F_rr | 含曲率半径项 | 0 |
| F_rv | 含曲率半径项 | I_3 |
| F_vv | -[2ω_ie^n + ω_en^n]× | -[2ω_ie^e ×]（常数） |
| F_vφ | [f^n ×] | [f^e ×] |
| F_φφ | -[ω_in^n ×] | -[ω_ie^e ×]（无ω_en^n项） |
| F_va | -C_b^n | -C_b^e |
| F_pb | -C_b^n | -C_b^e |

E 系优势:
  - ω_ie^e = [0,0,ω_e] 为常数，无需每步计算
  - 无 ω_en^n 项，F 矩阵更简洁
  - F_rv = I_3，计算简单
```

### 7.4 过程噪声矩阵 Q

```
Q 仅有以下非零子块:
  Q_gyro (3×3):  陀螺白噪声     → 姿态误差
  Q_accel (3×3): 加计白噪声     → 速度误差
  Q_bg (3×3):    陀螺零偏驱动噪声
  Q_ba (3×3):    加计零偏驱动噪声
  Q_angle (2×2): 安装角驱动噪声（可选，小量）
  Q_l_imu (3×3): IMU杆臂驱动噪声（可选，小量）
  Q_l_gnss (3×3): GNSS杆臂驱动噪声（可选，小量）

  Q[6:9, 6:9]   = (σ_g * dt)² * I₃
  Q[3:6, 3:6]   = (σ_a * dt)² * I₃
  Q[9:12, 9:12] = (σ_bg * √dt)² * I₃
  Q[12:15,12:15]= (σ_ba * √dt)² * I₃
  Q[15:17,15:17]= (σ_angle * √dt)² * I₂   (可选)
  Q[17:20,17:20]= (σ_l_imu * √dt)² * I₃   (可选)
  Q[20:23,20:23]= (σ_l_gnss * √dt)² * I₃  (可选，estimate_gnss_leverarm=true)
```

---

## 8. INS 初始化

### 8.1 初始化流程

```
INS 初始化
  │
  ├── 阶段1: 粗对准 (InsCore.coarse_align_*)
  │     ├── 静态粗对准（车辆静止）
  │     │   ├── 水平角: 由加速度计感知重力方向确定
  │     │   └── 方位角: 由陀螺仪感知地球自转确定（高精度IMU）
  │     │                或由 GNSS 速度方向确定（MEMS IMU）
  │     │
  │     └── 动态粗对准（车辆运动）
  │         ├── 水平角: 由加速度计确定
  │         └── 方位角: 由 GNSS 速度方向确定
  │
  ├── 阶段2: EKF 精对准
  │     ├── 初始化状态向量 x = 0
  │     ├── 初始化协方差 P
  │     └── 运行若干历元 EKF 更新
  │
  └── 阶段3: 初始化完成
        └── 进入正常松组合融合
```

### 8.2 静态粗对准

```
利用加速度计感知重力方向确定水平，利用陀螺仪感知地球自转确定方位。
在 E 系下进行粗对准，输出 C_b^e。

步骤:
  1. 累积加速度和角速度均值:
     f_mean = Σ f / N
     ω_mean = Σ ω / N

  2. 重力方向 → 确定 E 系下天向（通过初始位置计算 g^e 方向）:
     g_e = EarthParam.gravity_ecef(r0_e)    # E 系下重力方向
     g_dir = g_e / |g_e|                    # 天向（E 系下）

  3. 地球自转方向 → 确定 E 系下北向:
     omega_ie_e = [0, 0, omega_e]           # E 系下地球自转（常数）
     north_dir = omega_ie_e - (omega_ie_e · g_dir) * g_dir
     若 |north_dir| < ε (MEMS IMU): 使用初始位置计算北向
     否则: north_dir = north_dir / |north_dir|

  4. 东向 = 北向 × 天向:
     east_dir = north_dir × g_dir
     east_dir = east_dir / |east_dir|

  5. 重新正交化北向:
     north_dir = g_dir × east_dir

  6. 构造 C_b^e:
     C_e^b = [east_dir, north_dir, g_dir]^T
     C_b^e = (C_e^b)^T
```

### 8.3 动态粗对准

```
利用 GNSS 速度方向确定航向角。
在 E 系下进行粗对准，输出 C_b^e。

步骤:
  1. 将 GNSS 速度转换到 n 系下确定航向:
     v_n = C_e^n @ v_gnss^e    (ECEF速度转ENU)
     yaw = atan2(v_E, v_N)
  2. 假设水平: pitch = 0, roll = 0
  3. 构造 C_b^n: C_b^n = AttitudeUtil.euler2dcm(yaw, pitch, roll)
  4. 转换为 E 系: C_b^e = C_e^n^T @ C_b^n
     其中 C_e^n 由初始位置计算
```

### 8.4 初始化协方差设置

```
P[N,N] 初始协方差矩阵 (N=15/17/18/20/23, 由配置决定):

  位置 (0:3):       P[0:3,0:3]     = diag(σ_pos²)       σ_pos ~ [10, 10, 10] m
  速度 (3:6):       P[3:6,3:6]     = diag(σ_vel²)       σ_vel ~ [1, 1, 1] m/s
  姿态 (6:9):       P[6:9,6:9]     = diag(σ_att²)       σ_att ~ [5, 5, 30] °
  陀螺零偏 (9:12):  P[9:12,9:12]   = diag(σ_bg²)       σ_bg ~ 0.1 rad/s
  加计零偏 (12:15): P[12:15,12:15] = diag(σ_ba²)       σ_ba ~ 0.1 m/s²
  安装角 (15:17):   P[15:17,15:17] = diag(σ_angle²)    σ_angle ~ [5, 5] ° (可选)
  IMU杆臂 (17:20):  P[17:20,17:20] = diag(σ_l_imu²)    σ_l_imu ~ [0.5, 0.5, 0.5] m (可选)
  GNSS杆臂 (20:23): P[20:23,20:23] = diag(σ_l_gnss²)   σ_l_gnss ~ [0.5, 0.5, 0.5] m (可选)
```

---

## 9. 地球参数

### 9.1 WGS84 参数

```
WGS84_RE  = 6378137.0              地球长半轴 (m)
WGS84_F   = 1 / 298.257223563      扁率
WGS84_E2  = F * (2 - F)            第一偏心率的平方
WGS84_B   = 7.2921151467e-5        地球自转角速度 (rad/s)
WGS84_GM  = 3.986004418e14         引力常数 (m³/s²)
WGS84_G0  = 9.7803267715           赤道重力加速度 (m/s²)
```

### 9.2 曲率半径

```
卯酉圈曲率半径:
  R_N = a / √(1 - e² * sin²(φ))

子午圈曲率半径:
  R_M = a * (1 - e²) / (1 - e² * sin²(φ))^(3/2)
```

### 9.3 地球自转角速度

```
E 系下（本项目主计算系）:
  ω_ie^e = [0, 0, ω_e]^T    常数，无需每步计算
  ω_e = 7.2921151467e-5 rad/s

n 系下（仅输出用）:
  ω_ie^n = [0,
           ω_ie * cos(φ),
           ω_ie * sin(φ)]^T

ENU 坐标系: [东, 北, 天]
```

### 9.4 正常重力

```
E 系下（本项目主计算系，参考 gnss_ins_lc_nhc navmech.cc）:
  g^e = EarthParam.gravity_ecef(r^e)

  E 系重力向量包含引力+离心力:
  1. 计算 LLH: [lat, lon, h] = ecef2pos(r^e)
  2. 计算正常重力 g0(φ, h)（Somigliana + 自由空气改正）
  3. 计算 n 系下重力: g^n = [0, 0, -g0]^T
  4. 转换到 E 系: g^e = C_n^e @ g^n + 离心力项
     其中 C_n^e = [C_e^n]^T，由 lat, lon 计算

  简化计算（参考 gnss_ins_lc_nhc）:
  g^e = -g0 * [cos(lat)*cos(lon), cos(lat)*sin(lon), sin(lat)]^T + 离心力

n 系下（仅输出用）:
  g^n = [0, 0, -g(φ, h)]^T

Somigliana 公式（椭球面上的重力）:
  g0 = g_e * (1 + k * sin²(φ)) / √(1 - e² * sin²(φ))
  其中 k = 0.00193185265241

自由空气改正:
  g = g0 * (1 - 2h / a)
```

---

## 10. 姿态表示与转换

### 10.1 姿态表示方式

本项目内部使用**四元数**作为主要姿态表示，DCM（方向余弦矩阵）用于计算。

| 表示 | 优点 | 缺点 | 使用场景 |
|------|------|------|---------|
| **四元数** | 无奇异性，4参数，插值方便 | 不直观 | 状态存储、姿态更新 |
| **DCM** | 矩阵运算方便，直接变换向量 | 9参数，需正交化 | 向量变换、F矩阵构造 |
| **欧拉角** | 直观，3参数 | 奇异性（万向锁） | 输出显示、初始化 |

### 10.2 转换关系

```
四元数 → DCM (AttitudeUtil.quat2dcm):
  q = [q0, q1, q2, q3] (标量在前)
  C_b^e = (q0²-q1²-q2²-q3²)I + 2*q*q^T - 2*q0*[q×]

DCM → 四元数 (AttitudeUtil.dcm2quat):
  使用 Shepperd 方法避免数值不稳定

欧拉角 → DCM (AttitudeUtil.euler2dcm):
  旋转顺序: Z(yaw) → Y(pitch) → X(roll)
  坐标系: ENU 导航系, FRD 体坐标系
  注意: 欧拉角仅用于 n 系下的输出显示

DCM → 欧拉角 (AttitudeUtil.dcm2euler):
  先将 C_b^e 转换为 C_b^n: C_b^n = C_e^n @ C_b^e
  然后:
  pitch = arcsin(-C_b^n[2,0])
  yaw   = atan2(C_b^n[1,0], C_b^n[0,0])
  roll  = atan2(C_b^n[2,1], C_b^n[2,2])

四元数乘法 (AttitudeUtil.quat_mult):
  q1 ⊗ q2

反对称矩阵 (AttitudeUtil.skew_symmetric):
  [v×] = [[0, -v3, v2], [v3, 0, -v1], [-v2, v1, 0]]

正交化 (AttitudeUtil.orthogonalize):
  C = U * V^T  (SVD 分解强制正交)

E 系 ↔ n 系转换:
  C_b^n = C_e^n @ C_b^e    (E系姿态 → n系姿态，用于输出)
  C_b^e = C_n^e @ C_b^n    (n系姿态 → E系姿态，用于初始化)
  C_e^n 由位置 [lat, lon] 计算
```

---

## 11. 参考代码映射

### 11.1 GREAT-MSF → 本项目

| GREAT-MSF | 本项目 | 对应关系 |
|-----------|--------|---------|
| `t_gsins` | `InsCore` | INS 核心类：姿态/速度/位置更新、粗对准 |
| `t_gimu` | `ImuPreprocessor` (ABC) | IMU 数据预处理抽象基类 |
| — | `RateImuPreprocessor` | 速率式 IMU 预处理器 |
| — | `DeltaImuPreprocessor` | 增量式 IMU 预处理器 |
| `t_ginterp` | `Interpolator` (ABC) | 插值器抽象基类 |
| `ginterp` (线性) | `LinearInterpolator` | 线性插值 |
| `t_gpoly` | `PolyInterpolator` | 多项式插值 |
| `t_gbase` | `AttitudeUtil` | 姿态转换工具（静态方法） |
| `t_gbase` | `EarthParam` | 地球参数（静态方法） |

### 11.2 gnss_ins_lc_nhc → 本项目

| gnss_ins_lc_nhc | 本项目 | 对应关系 |
|-----------------|--------|---------|
| `navmech.cc` 姿态更新 | `InsCore.attitude_update()` | E 系下四元数/DCM 姿态递推 |
| `navmech.cc` 速度更新 | `InsCore.velocity_update()` | E 系下比力+Coriolis+重力 |
| `navmech.cc` 位置更新 | `InsCore.position_update()` | ECEF 位置递推 |
| `navmech.cc` F矩阵构造 | `InsKf.set_Ft()` / `LcEstimator.set_Ft()` | E 系下可配置维度状态转移矩阵 |
| `navinitalized.cc` | `InsCore.coarse_align_*()` | E 系下粗对准+精对准 |

### 11.3 GINav → 本项目

| GINav | 本项目 | 对应关系 |
|-------|--------|---------|
| `ins_mech.m` | `InsCore` | INS 机械编排 |
| `ins_init.m` | `InsCore` + 协方差设置 | INS 初始化 |
| `ins_align.m` | `InsCore.coarse_align_*()` | INS 对准 |
| `earth_update.m` | `EarthParam` | 地球参数更新 |
| `update_trans_mat.m` | `InsKf.set_Ft()` / `LcEstimator.set_Ft()` | 状态转移矩阵更新 |
| `Cnb2att.m` / `att2Cnb.m` | `AttitudeUtil` | 姿态转换 |

---

## 12. 队列流水线集成

### 12.1 IMU 在框架中的角色

IMU 机械编排在 GInsStream 框架中承担前端里程计角色，通过策略模式封装，通过**纯队列流水线**接收数据（**无观察者模式、无 notify()**）：

| 角色 | 设计模式 | 实现类 | 作用 |
|------|---------|--------|------|
| **前端里程计** | 策略模式 | `ImuMechStrategy(OdometryStrategy)` | 封装 IMU 机械编排，与 `GnssPositioningStrategy` 可热切换 |
| **数据源** | 纯队列流水线 | `ImuSensor(BaseSensor)` | 读取 IMU 数据，`output_queue.put()` 推入 imu_queue |
| **高频数据处理** | 纯 threading | `ImuSensor` 阻塞式读取 | 独立线程处理高频 IMU 数据（100~200Hz），不使用 asyncio |

### 12.2 前端策略类 — ImuMechStrategy

将 IMU 机械编排封装为前端里程计策略，实现与 GNSS 策略相同的接口：

```python
from abc import ABC, abstractmethod

class OdometryStrategy(ABC):
    """前端里程计算法策略基类（定义于 estimator.md）"""

    @abstractmethod
    def execute(self, measurement) -> 'InsState':
        """执行前端里程计解算，返回当前 INS 状态"""
        ...


class ImuMechStrategy(OdometryStrategy):
    """IMU 机械编排前端策略

    封装 InsCore 的姿态/速度/位置更新，
    作为前端里程计策略，与 GnssPositioningStrategy 可热切换。

    依赖注入: 构造函数接收 InsCore 与 ImuPreprocessor 实例。
    """

    def __init__(self, ins_core: 'InsCore', preprocessor: 'ImuPreprocessor'):
        self._ins_core = ins_core        # 依赖注入 INS 核心
        self._preprocessor = preprocessor  # 依赖注入 IMU 预处理器

    def execute(self, imu: 'ImuMeasurement') -> 'InsState':
        """执行 IMU 机械编排

        流程:
        1. 预处理（异常检测 + 零偏补偿）
        2. 姿态更新（E 系）
        3. 速度更新（E 系）
        4. 位置更新（E 系，ECEF 递推）
        """
        imu_clean = self._preprocessor.preprocess(imu)
        return self._ins_core.update(imu_clean)
```

### 12.3 纯队列流水线 — IMU 传感器（无 Subject 角色）

IMU 传感器继承 `BaseSensor`（**仅作为数据读取抽象，无 Subject 角色、无 `_observers`、无 `attach()`、无 `notify()`**），实现 `get_data()` 接口，数据通过 `output_queue.put()` 推入 imu_queue：

```python
class BaseSensor(ABC):
    """传感器抽象基类（数据读取抽象，定义于 StreamDesign.md）

    ※ 不作为 Subject，无观察者列表，无 attach/notify 方法。
       仅定义数据读取接口和共享线程框架。
    """

    def __init__(self, name: str = "", output_queue: 'Queue' = None):
        self._name = name               # 传感器标识
        self._output_queue = output_queue  # 输出队列（imu_queue / sensor_queue）

    @property
    def name(self) -> str:
        return self._name

    @abstractmethod
    def get_data(self) -> 'SensorData':
        """强制实现的传感器数据读取接口"""
        ...

    def run(self) -> None:
        """传感器线程主循环（纯 threading，由 StreamerBase 驱动）

        循环: get_data() → output_queue.put(data) → 直到 stop
        """
        ...


class ImuSensor(BaseSensor):
    """IMU 传感器（数据源，非 Subject）

    读取 IMU 原始数据（角速度/加速度 或 角增量/速度增量），
    由 SensorFactory 动态创建，实现 get_data() 接口。
    数据通过 output_queue.put() 推入 imu_queue，**无 notify() 调用**。
    """

    def get_data(self) -> 'SensorData':
        """读取一条 IMU 测量数据（不通知任何观察者）"""
        imu_meas = self._reader.read_measurement()
        data = SensorData(tag="imu", imu=imu_meas)
        # 不调用 notify()，由上层 run() 负责 output_queue.put(data)
        return data
```

### 12.4 纯 threading 处理高频 IMU 数据

IMU 采样率高（100~200Hz），采用**纯 threading** 阻塞式读取，**不使用 asyncio、不使用共享内存**（参考 StreamDesign.md）：

```python
import threading
from queue import Queue, Empty

class ImuSensorThread:
    """IMU 传感器线程包装（纯 threading）

    在独立线程中循环调用 ImuSensor.get_data()，
    将数据通过 imu_queue.put() 推入流水线。

    ※ 不使用 asyncio，不使用 multiprocessing.shared_memory。
       高频 IMU 的并发处理依赖 Python threading + Queue 实现。
    """

    def __init__(self, sensor: 'ImuSensor', imu_queue: 'Queue',
                 stop_event: threading.Event):
        self._sensor = sensor
        self._queue = imu_queue
        self._stop = stop_event
        self._thread: threading.Thread = None

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self) -> None:
        """线程主循环：get_data() → imu_queue.put()"""
        while not self._stop.is_set():
            try:
                data = self._sensor.get_data()
                if data is not None:
                    self._queue.put(data)  # 推入 imu_queue
            except Exception:
                # 异常时记录日志并继续（不中断线程）
                continue
```

### 12.5 工厂模式创建 IMU 传感器

`SensorFactory` 根据配置动态创建 IMU 传感器实例（**无 async_io 参数，无 AsyncImuSensor 分支**，参考 StreamDesign.md）：

```python
class SensorFactory:
    """传感器工厂（参考 StreamDesign.md）"""

    @staticmethod
    def create(sensor_type: str, config: dict,
               output_queue: 'Queue' = None) -> BaseSensor:
        if sensor_type == "imu":
            # 仅创建 ImuSensor，统一由 ImuSensorThread 线程驱动
            return ImuSensor(config, output_queue=output_queue)
        elif sensor_type == "gnss":
            return GnssRoverSensor(config, output_queue=output_queue)
        # ...
```

### 12.6 数据流与队列流水线时序

```
IMU 数据到达（纯队列流水线，无观察者回调）:
  ImuSensorThread._run()
    → ImuSensor.get_data()               (读取 IMU 测量：角速度/加速度)
    → imu_queue.put(SensorData(tag="imu"))  (推入 imu_queue)
        ↓
  Scheduler（仅转发，不做时间对齐）
    → estimate_queue.put(SensorData)
        ↓
  LcIntegration.process_epoch()          (从 estimate_queue.get())
    → _time_align()                       (时间对齐：增量切分，在 Estimator 内部)
    → ImuMechStrategy.execute()           (前端策略：机械编排 E 系递推)
    → InsKf.time_update()                 (双滤波 EKF 预测：P1 + P2 独立)
    → [GNSS 到达时] meas_update()         (双滤波量测更新：H1 作用于 P1, H2 作用于 P2)
    → feedback()                          (双滤波独立反馈)
    → solution_queue.put(Solution)        (输出解)
```

### 12.7 策略热切换示例

```python
# 正常模式：IMU 机械编排前端
frontend = ImuMechStrategy(ins_core, preprocessor)

# IMU 故障：热切换为 GNSS 纯解算前端
frontend = SppStrategy(options)

# 框架持有 OdometryStrategy 抽象引用，切换无需改动 LcIntegration
# （_frontend 为 Integration 基类通过依赖注入持有的策略引用）
integration._frontend = frontend
```

### 12.8 OOP 三大特性体现

| 特性 | 体现 |
|------|------|
| **封装** | 机械编排状态（姿态/速度/位置）封装在 `InsCore` 内部，双滤波状态分别封装在 P1/P2 矩阵中，外部仅通过策略接口访问 |
| **继承** | `ImuPreprocessor(ABC)` → `RateImuPreprocessor` / `DeltaImuPreprocessor`；`OdometryStrategy(ABC)` → `ImuMechStrategy`；`BaseSensor(ABC)` → `ImuSensor` |
| **多态** | 框架持有 `OdometryStrategy` 抽象引用（`_frontend`），运行时调用 `ImuMechStrategy` / `SppStrategy` / `RtkStrategy`；传感器层多态通过 `BaseSensor` 抽象引用 |
