# GNSS 算法架构设计

> 基于 rtklib-py 算法参考 + GREAT-MSF 架构模式，设计 GInsStream 流式 GNSS 处理框架。
> 实现 SPP 和 RTK（含 RTD 退化模式）功能，采用 ABC 抽象类继承体系。
>
> **时间系统约定**：全框架统一使用 GPS 秒（GPST，since 1980-01-06），不使用 Unix epoch 或本地时间。
>
> 在框架中，GNSS 解算承担两种角色：
> - **前端策略（仅 IMU 不可用降级时启用）**：作为 `GnssPositioningStrategy(OdometryStrategy)`，
>   当无 IMU 或 IMU 不可用时，直接用 SPP/RTK/RTD 产生里程计（位置/速度），与 `ImuMechStrategy` 可互换
> - **后端量测源**：松组合后端 EKF 的量测输入（GnssSolution），由 GnssSolutionProvider 统一提供，
>   仅作用于主滤波 P1（NHC 子滤波 P2 不直接使用 GNSS 量测）
>
> GNSS 传感器通过 `GnssRoverSensor(BaseSensor)` 读取原始观测值，`GnssSolSensor(BaseSensor)` 读取外部结果，
> 均实现 `get_data()` 接口，由 `SensorFactory` 动态创建。**采用纯队列流水线**：数据通过 `queue.put()` 推入
> sensor_queue → Scheduler 转发到 estimate_queue → `LcIntegration.process_epoch()` 处理，**无观察者模式、无 notify()**。

---

## 目录

- [1. 架构总览](#1-架构总览)
- [2. ABC 类继承体系](#2-abc-类继承体系)
- [3. 数据结构适配（rtklib-py 映射）](#3-数据结构适配rtklib-py-映射)
- [4. SPP 算法流程](#4-spp-算法流程)
- [5. RTK/RTD 算法流程](#5-rtkrtd-算法流程)
- [6. CombModel 层次体系](#6-combmodel-层次体系)
- [7. 坐标变换工具提取计划](#7-坐标变换工具提取计划)
- [8. 测试验证计划](#8-测试验证计划)
- [9. 队列流水线集成](#9-队列流水线集成)

---

## 1. 架构总览

### 1.1 设计原则

| 设计维度 | 决策 |
|---------|------|
| 算法参考 | rtklib-py（SPP/卫星位置/大气改正/观测方程） |
| 架构参考 | GREAT-MSF（t_gpvtflt 处理器模式、t_gbasemodel/t_gcombmodel/t_gcombDD 组合模型层次） |
| 处理模式 | 流式逐历元，无全局状态 |
| 类设计 | ABC 抽象基类 + 具体子类，面向接口编程 |
| RTD 定位 | RTK 的退化模式（码双差，无模糊度解算），不作为独立模块 |
| 外部结果 | 支持 GnssExternalProvider 直接读取外部 GNSS 定位结果文件，跳过内部解算 |

### 1.2 处理器层次

```
GnssProcessor (ABC)
├── SppProcessor          # 单点定位
└── RtkProcessor          # RTK 定位（含 RTD 退化模式）

GnssSolutionProvider (ABC)    # GNSS 结果提供者（统一内部/外部数据源）
├── GnssInternalProvider      # 内部解算（组合 GnssProcessor）
└── GnssExternalProvider      # 外部结果文件（读取 POS/NMEA/CSV）
```

### 1.3 组合模型层次（参考 GREAT-MSF）

```
BaseModel (ABC)           # 参考 t_gbasemodel
└── CombDD (BaseModel)    # 双差组合模型，参考 t_gcombDD
```

### 1.4 不包含的功能

- **PPP**：本框架面向低成本导航，不实现精密单点定位
- **RTD 独立模块**：RTD 是 RTK 的退化模式，统一在 RtkProcessor 中

---

## 2. ABC 类继承体系

### 2.1 GnssProcessor 抽象基类

```python
from abc import ABC, abstractmethod

class GnssProcessor(ABC):
    """GNSS 处理器抽象基类

    参考 GREAT-MSF t_gpvtflt 的处理器模式：
    每个处理器负责一种定位模式，按历元驱动处理。
    """

    @abstractmethod
    def process_epoch(self, gnss_meas, eph_buffer, options) -> Optional[GnssSolution]:
        """处理单个历元

        参数:
            gnss_meas: GnssMeasurement，一个历元的观测值
            eph_buffer: EphemerisBuffer，星历缓冲区
            options: dict，配置选项

        返回:
            GnssSolution 或 None（解算失败）
        """
        ...

    @abstractmethod
    def get_solution(self) -> Optional[GnssSolution]:
        """获取最新解算结果

        返回:
            最近一次成功解算的 GnssSolution
        """
        ...

    @abstractmethod
    def reset(self) -> None:
        """重置处理器状态（模糊度重置、位置初始化等）"""
        ...
```

### 2.2 SppProcessor

```python
class SppProcessor(GnssProcessor):
    """SPP 单点定位处理器

    算法参考 rtklib-py pntpos()。
    伪距观测方程 + 最小二乘迭代求解。
    """

    def __init__(self, options: dict):
        self._last_pos: Optional[np.ndarray] = None
        self._solution: Optional[GnssSolution] = None
        self._options: dict = options

    def process_epoch(self, gnss_meas, eph_buffer, options) -> Optional[GnssSolution]:
        """SPP 逐历元处理，详见第4节算法流程"""
        ...

    def get_solution(self) -> Optional[GnssSolution]:
        ...

    def reset(self) -> None:
        self._last_pos = None
        self._solution = None
```

### 2.3 RtkProcessor（含 RTD 退化模式）

```python
class RtkProcessor(GnssProcessor):
    """RTK 定位处理器，RTD 为其退化模式

    架构参考 GREAT-MSF t_gpvtflt + t_gcombDD：
    - RTK 模式：载波双差 + 模糊度解算
    - RTD 模式：码双差，无模糊度解算（RTK 的退化）

    RTD 退化条件：
    - 载波相位观测值不足或质量差
    - 模糊度解算失败（ratio 检验不通过）
    - 配置强制 RTD 模式
    """

    def __init__(self, options: dict):
        self._last_pos: Optional[np.ndarray] = None
        self._solution: Optional[GnssSolution] = None
        self._ambiguities: dict = {}          # 模糊度状态
        self._comb_model: CombDD = None       # 双差组合模型
        self._options: dict = options
        self._mode: str = "RTK"               # "RTK" 或 "RTD"（退化）

    def process_epoch(self, gnss_meas, eph_buffer, options) -> Optional[GnssSolution]:
        """RTK/RTD 逐历元处理，详见第5节算法流程"""
        ...

    def get_solution(self) -> Optional[GnssSolution]:
        ...

    def reset(self) -> None:
        """重置处理器状态，包括模糊度"""
        self._last_pos = None
        self._solution = None
        self._ambiguities = {}
        self._mode = "RTK"

    def _try_ambiguity_resolution(self, ...) -> tuple:
        """尝试模糊度解算

        返回:
            (resolved: bool, ambiguities: dict)
            resolved=False 时退化为 RTD 模式
        """
        ...

    def _degrade_to_rtd(self) -> None:
        """退化为 RTD 模式

        同一框架内切换：
        - 保留双差结构
        - 仅使用码观测值
        - 跳过模糊度解算
        """
        self._mode = "RTD"
```

### 2.4 GnssSolutionProvider — GNSS 结果提供者

> 统一内部解算和外部结果文件两种 GNSS 数据源，对 Integration 层提供统一接口。

```python
class GnssSolutionProvider(ABC):
    """GNSS 结果提供者抽象基类

    对 Integration 层屏蔽 GNSS 结果来源差异：
    - 内部模式：通过 GnssProcessor 解算得到
    - 外部模式：从 GNSS 定位结果文件流式读取
    """

    @abstractmethod
    def get_solution(self, timestamp: float) -> Optional[GnssSolution]:
        """获取指定时刻的 GNSS 解算结果

        参数:
            timestamp: 目标时间戳 (s)

        返回:
            GnssSolution 或 None
        """
        ...

    @abstractmethod
    def is_available(self) -> bool:
        """当前 GNSS 结果是否可用"""
        ...


class GnssInternalProvider(GnssSolutionProvider):
    """内部解算提供者

    组合 GnssProcessor（SppProcessor/RtkProcessor），
    从原始观测值解算得到 GnssSolution。
    gnss_source="internal" 时使用。
    """

    def __init__(self, gnss_processor: GnssProcessor, config: dict):
        self._processor = gnss_processor
        self._mode = config.get("mode", "differential")  # "differential" / "spp"

    def get_solution(self, timestamp: float) -> Optional[GnssSolution]:
        """调用 GnssProcessor 解算"""
        ...

    def is_available(self) -> bool:
        ...


class GnssExternalProvider(GnssSolutionProvider):
    """外部结果文件提供者

    从 GNSS 定位结果文件（POS/NMEA/CSV）流式读取，
    直接返回 GnssSolution，无需内部 GNSS 解算。
    gnss_source="external" 时使用。

    支持的输入格式：
    - POS 格式（参考 rtklib 输出格式）
    - NMEA 格式（$GPGGA/$GPRMC）
    - CSV 格式（自定义列）

    外部结果模式下，不需要 Rover/Ref/Eph 数据流。
    """

    def __init__(self, config: dict):
        self._solution_format = config.get("gnss_solution_format", "pos")
        self._solution_buffer = []       # 缓冲最近结果用于时间匹配/插值
        self._default_pos_std = config.get("default_pos_std", 1.0)
        self._default_vel_std = config.get("default_vel_std", 0.5)
        self._pos_cov_available = config.get("pos_covariance_available", False)

    def get_solution(self, timestamp: float) -> Optional[GnssSolution]:
        """从缓冲区查找/插值获取最近时刻的 GNSS 结果

        策略：
        1. 精确时间匹配（|dt| < 0.01s）
        2. 最近邻匹配（|dt| < max_age，默认1s）
        3. 线性插值（两个相邻结果之间）
        """
        ...

    def is_available(self) -> bool:
        ...

    def feed_solution(self, sol: GnssSolution) -> None:
        """从 GnssSolStreamer 接收新的定位结果"""
        ...
```

### 2.5 外部 GNSS 结果文件格式

外部模式下，GnssSolStreamer 通过 Formator 解析 GNSS 定位结果文件：

| 格式 | 文件扩展名 | 解析 Formator | 说明 |
|------|-----------|--------------|------|
| POS | `.pos` | `PosSolFormator` | 参考 rtklib 输出格式，含 LLH + 精度 |
| NMEA | `.nmea` | `NmeaSolFormator` | 标准 NMEA-0183，$GPGGA/$GPRMC |
| CSV | `.csv` | `CsvSolFormator` | 自定义列格式，需配置列映射 |

**POS 格式示例**（输入，与输出格式一致）：
```
%  GPST   lat(deg)    lon(deg)     height(m)  Q  ns  sdn  sde  sdu ...
2023/01/15 08:00:00.0  30.12345678  120.12345678  50.123  5  12  0.5  0.5  1.0
```

**NMEA 格式示例**（输入）：
```
$GPGGA,080000.0,3012.345678,N,12012.345678,E,1,12,1.2,50.1,M,...
$GPRMC,080000.0,A,3012.345678,N,12012.345678,E,5.0,45.0,150123,...
```

**CSV 格式示例**（输入）：
```
timestamp,pos_e,pos_n,pos_u,vel_e,vel_n,vel_u,status,num_sat
```

---

## 3. 数据结构适配（rtklib-py 映射）

### 3.1 观测值映射

| rtklib-py | 本项目 | 转换说明 |
|-----------|--------|---------|
| `Obs.time` = [tows, week] | `GnssMeasurement.timestamp` (float) | `tows + week * 604800` |
| `Obs.sat[i]` (int PRN) | `GnssObservation.satellite_id` (str "G01") | `f"{sys_char}{prn:02d}"` |
| `Obs.P[i]` | `GnssObservation.pseudorange` | 直接映射 |
| `Obs.L[i]` | `GnssObservation.phaserange` | 直接映射（周 → 米需乘波长） |
| `Obs.S[i]` | `GnssObservation.snr` | 直接映射 |
| `Obs.D[i]` | `GnssObservation.doppler` | 直接映射 |

### 3.2 星历映射

| rtklib-py | 本项目 | 转换说明 |
|-----------|--------|---------|
| `Eph.sat` (int) | `EphemerisData` + satellite_id | `prn + sys_offset[sys_char]` |
| `Eph.sqrtA, e, i0, OMG0, omg` | `EphemerisData.orbit_params` dict | 按键名映射 |
| `Eph.M0, deln, OMGd, idot` | `EphemerisData.orbit_params` dict | 按键名映射 |
| `Eph.cuc, cus, crc, crs, cic, cis` | `EphemerisData.orbit_params` dict | 按键名映射 |
| `Eph.toe` = [tows, week] | `EphemerisData.toe_seconds` + `week` | 拆分映射 |
| `Eph.clk` = [a0, a1, a2] | `EphemerisData.clock_bias` | 直接映射 |
| `Eph.tgd` | `EphemerisData.orbit_params['tgd']` | 按键名映射 |
| `Geph` (GLONASS) | `EphemerisData` (GLONASS 类型) | pos/vel/acc 字段映射 |
| `Nav` (全局容器) | `EphemerisBuffer` (流式管理) | 按卫星ID+时间查询 |

### 3.3 解算结果映射

| rtklib-py | 本项目 | 转换说明 |
|-----------|--------|---------|
| `Sol.rr[0:3]` | `GnssSolution.position` | ECEF 位置 |
| `Sol.dtr` | `GnssSolution.clock_bias` | 接收机钟差 |
| `Sol.qr` | `GnssSolution.pos_covariance` | 协方差矩阵 |
| `Sol.ns` | `GnssSolution.num_satellites` | 使用卫星数 |
| `Sol.stat` | `GnssSolution.status` | 解算状态 |

---

## 4. SPP 算法流程

### 4.1 SppProcessor.process_epoch() 流程

```
SppProcessor.process_epoch(gnss_meas, eph_buffer, options)
  │
  ├── 1. 卫星位置计算
  │     对每颗卫星:
  │       ├── 从 eph_buffer 选择星历（参考 rtklib-py seleph）
  │       ├── eph2pos() 计算卫星位置（参考 rtklib-py eph2pos）
  │       └── 卫星钟差改正
  │
  ├── 2. 初始位置
  │     ├── 有上次解 → 使用上次位置
  │     ├── 有 approx_position → 使用近似位置
  │     └── 否则 → 零向量
  │
  ├── 3. 迭代最小二乘（最多 max_iter 次）
  │     │
  │     ├── 3a. 构建观测方程（对每颗合格卫星）
  │     │     ├── 计算仰角/方位角（参考 rtklib-py satazel）
  │     │     ├── 仰角/信噪比筛选
  │     │     ├── 对流层改正（参考 rtklib-py tropmodel）
  │     │     ├── 电离层改正 Klobuchar（参考 rtklib-py ionmodel）
  │     │     ├── 卫星钟差改正
  │     │     ├── 地球自转改正（Sagnac 效应）
  │     │     ├── 计算几何距离
  │     │     └── 构建观测方程行: H_i, Z_i, W_i
  │     │         观测模型: P_i = ρ_i + c·dt_r - c·dt_s + T_i + I_i + ε_i
  │     │         线性化:   Z_i = P_i - ρ_i - c·dt_s + T_i + I_i
  │     │                   H_i = [-e_i^T, 1]
  │     │                   W_i = sin²(el) / σ_code²
  │     │
  │     ├── 3b. 加权最小二乘求解
  │     │     dx = (H^T W H)^{-1} H^T W Z
  │     │
  │     ├── 3c. 更新位置: pos += dx[:3]
  │     │
  │     └── 3d. 收敛判断: ‖dx[:3]‖ < 1e-4 → 退出迭代
  │
  └── 4. 构建解算结果
        ├── 位置 (ECEF)
        ├── 钟差
        ├── DOP 值
        ├── 协方差 = (H^T W H)^{-1}
        └── 精度估计
```

### 4.2 SPP 观测方程要点

- **观测值**：仅伪距（码观测值）
- **待估参数**：[x, y, z, c·dt_r]（3 位置 + 1 钟差）
- **最小卫星数**：4 颗
- **权重模型**：仰角加权 σ = σ_code / sin(el)
- **大气改正**：模型改正（Saastamoinen 对流层 + Klobuchar 电离层）

---

## 5. RTK/RTD 算法流程

### 5.1 RTK 与 RTD 的关系

```
┌─────────────────────────────────────────────────────┐
│                  RtkProcessor                        │
│                                                     │
│  ┌─────────────────────────────────────────────┐   │
│  │           双差框架（CombDD）                  │   │
│  │                                             │   │
│  │  RTK 模式:                                   │   │
│  │    观测值 = 码双差 + 载波双差                 │   │
│  │    待估参数 = 位置 + 模糊度                   │   │
│  │    模糊度解算 = LAMBDA / 部分 AR              │   │
│  │                                             │   │
│  │  RTD 模式（退化）:                            │   │
│  │    观测值 = 码双差（仅码）                    │   │
│  │    待估参数 = 位置（无模糊度）                 │   │
│  │    模糊度解算 = 跳过                          │   │
│  │                                             │   │
│  └─────────────────────────────────────────────┘   │
│                                                     │
│  退化条件:                                          │
│    1. 载波相位质量不足                              │
│    2. 模糊度 ratio 检验失败                         │
│    3. 周跳检测后模糊度重置期间                       │
│    4. 配置强制 RTD 模式                             │
└─────────────────────────────────────────────────────┘
```

### 5.2 RtkProcessor.process_epoch() 流程

```
RtkProcessor.process_epoch(rover_meas, ref_meas, eph_buffer, options)
  │
  ├── 1. 卫星位置计算（同 SPP，参考 rtklib-py satposs）
  │
  ├── 2. Rover-Ref 时间匹配
  │     ├── 找到时间戳最接近的 Ref 历元
  │     └── 差分龄期 > max_differential_age → 降级为 SPP 或放弃
  │
  ├── 3. 共视卫星筛选
  │     ├── Rover 与 Ref 的卫星交集
  │     ├── 仰角/信噪比筛选
  │     └── 选择参考卫星（最高仰角）
  │
  ├── 4. 构建双差观测方程（CombDD.cmb_equ()）
  │     │
  │     ├── 4a. 码双差（RTD/RTK 共用）
  │     │     Δ∇P = P_rover,i - P_rover,j - P_ref,i + P_ref,j
  │     │
  │     ├── 4b. 载波双差（仅 RTK）
  │     │     Δ∇L = L_rover,i - L_rover,j - L_ref,i + L_ref,j
  │     │
  │     └── 4c. 线性化观测方程
  │           ├── 码双差: Z_code = Δ∇P - Δ∇ρ,  H_code = [-Δ∇e^T]
  │           └── 载波双差: Z_phase = Δ∇L - Δ∇ρ + λ·N,  H_phase = [-Δ∇e^T, λ·I]
  │
  ├── 5. 模糊度处理
  │     ├── 检查载波质量 → 质量不足 → 跳到步骤6（RTD 退化）
  │     ├── 已有固定模糊度 → 使用固定解
  │     ├── 尝试模糊度解算（LAMBDA）
  │     │   ├── ratio 检验通过 → 固定解
  │     │   └── ratio 检验失败 → 浮点解 / 退化 RTD
  │     └── 周跳检测 → 重置相关模糊度 → 退化 RTD
  │
  ├── 6. 最小二乘 / Kalman 滤波求解
  │     ├── RTK: 位置 + 模糊度（Kalman 滤波）
  │     └── RTD: 仅位置（最小二乘，无模糊度参数）
  │
  └── 7. 构建解算结果
        ├── 位置 (ECEF)
        ├── 模式标记: "RTK-Fixed" / "RTK-Float" / "RTD"
        ├── 模糊度状态
        └── 精度估计
```

### 5.3 RTD 退化详解

RTD 不是独立模块，而是 RTK 框架内的退化模式：

| 维度 | RTK 模式 | RTD 退化模式 |
|------|---------|-------------|
| 双差类型 | 码双差 + 载波双差 | 仅码双差 |
| 观测值 | 伪距 + 载波相位 | 仅伪距 |
| 待估参数 | 位置 + 模糊度向量 | 仅位置 |
| 模糊度解算 | LAMBDA / 部分 AR | 跳过 |
| 解算方法 | Kalman 滤波（状态含模糊度） | 最小二乘（无模糊度状态） |
| 精度 | 厘米~分米级 | 分米~米级 |
| 退化触发 | — | 载波质量差 / ratio 失败 / 周跳重置 / 配置强制 |

**退化机制**：RtkProcessor 在每个历元处理时，先尝试 RTK（载波双差 + 模糊度解算），若条件不满足则自动退化为 RTD（仅码双差）。退化是动态的——当载波条件恢复时，可重新初始化模糊度回到 RTK 模式。

---

## 6. CombModel 层次体系

### 6.1 设计参考

参考 GREAT-MSF 的组合模型层次：
- `t_gbasemodel`：基础模型抽象基类，定义组合方程接口
- `t_gcombmodel`：组合模型中间层
- `t_gcombDD`：双差组合模型具体实现

### 6.2 BaseModel 抽象基类

```python
class BaseModel(ABC):
    """组合模型抽象基类

    参考 GREAT-MSF t_gbasemodel：
    定义 GNSS 组合观测方程的统一接口。
    所有组合模型（单差、双差、非差等）均继承此类。
    """

    @abstractmethod
    def cmb_equ(self, obs_data, sat_positions, ref_pos, options) -> CmbEquation:
        """构建组合观测方程

        参数:
            obs_data: 观测数据（已做数据匹配和筛选）
            sat_positions: 卫星位置字典 {sat_id: (rs, dts, var)}
            ref_pos: 参考位置 ECEF
            options: 配置选项

        返回:
            CmbEquation 包含:
                H: 观测矩阵 (n_obs, n_param)
                Z: 残差向量 (n_obs,)
                W: 权重矩阵 (n_obs, n_obs)
                param_info: 参数信息列表
        """
        ...
```

### 6.3 CombDD 双差组合模型

```python
class CombDD(BaseModel):
    """双差组合模型

    参考 GREAT-MSF t_gcombDD：
    实现站间-星间双差观测方程构建。

    双差消除:
      - 卫星钟差
      - 接收机钟差
      - 电离层延迟（短基线）
      - 对流层延迟（短基线）

    支持:
      - 码双差（RTD 模式使用）
      - 载波双差 + 模糊度（RTK 模式使用）
    """

    def __init__(self):
        self._ref_sat: Optional[str] = None     # 参考卫星
        self._ambiguities: dict = {}             # 双差模糊度状态

    def cmb_equ(self, obs_data, sat_positions, ref_pos, options) -> CmbEquation:
        """构建双差观测方程

        流程:
        1. 选择参考卫星（最高仰角）
        2. 对每对 (参考星, 流动星) 构建双差
        3. 码双差: Δ∇P（RTD/RTK 均使用）
        4. 载波双差: Δ∇L（仅 RTK 使用，含模糊度参数）
        5. 组装 H, Z, W 矩阵
        """
        ...

    def select_ref_sat(self, obs_data, sat_positions, rx_pos) -> str:
        """选择参考卫星（最高仰角）"""
        ...

    def build_code_dd(self, rover_obs, ref_obs, rs_i, rs_j, ref_pos, rx_pos) -> tuple:
        """构建码双差观测方程行

        返回: (H_i, Z_i, W_i)
        """
        ...

    def build_phase_dd(self, rover_obs, ref_obs, rs_i, rs_j, ref_pos, rx_pos, amb) -> tuple:
        """构建载波双差观测方程行

        返回: (H_i, Z_i, W_i, amb_param_index)
        """
        ...
```

### 6.4 CmbEquation 数据结构

```python
@dataclass
class CmbEquation:
    """组合观测方程

    由 BaseModel.cmb_equ() 返回，供处理器求解使用。
    """
    H: np.ndarray           # 观测矩阵 (n_obs, n_param)
    Z: np.ndarray           # 残差向量 (n_obs,)
    W: np.ndarray           # 权重矩阵 (n_obs, n_obs)
    param_info: list        # 参数信息 [{'name': str, 'type': str, 'sat_id': str}, ...]
    n_code_obs: int         # 码观测方程数
    n_phase_obs: int        # 载波观测方程数
```

---

## 7. 坐标变换工具提取计划

### 7.1 提取来源

从 rtklib-py `rtkcmn.py` 提取坐标变换函数，独立为 `coord_transform.py` 模块。

### 7.2 函数清单

| rtklib-py 函数 | 本项目函数 | 说明 |
|---------------|-----------|------|
| `ecef2pos()` | `ecef2pos()` | ECEF → LLH [lat, lon, h] (rad, rad, m) |
| `pos2ecef()` | `pos2ecef()` | LLH → ECEF |
| `ecef2enu()` | `ecef2enu()` | ECEF 差值 → ENU |
| `enu2ecef()` | `enu2ecef()` | ENU → ECEF 差值 |
| — | `xyz2enu_mat()` | ECEF→ENU 旋转矩阵 |
| — | `enu2xyz_mat()` | ENU→ECEF 旋转矩阵 |
| `gpst2time()` | `gpst2time()` | GPS 周+周内秒 → 时间戳 |
| `time2gpst()` | `time2gpst()` | 时间戳 → (week, tow) |
| `epoch2time()` | `epoch2time()` | [y,m,d,h,min,sec] → 时间戳 |

### 7.3 改造要点

- rtklib-py 使用列表传参 → 改为 numpy 数组
- 去除全局状态依赖
- 确保纯函数化（无副作用）
- 所有函数无状态，可直接被 SppProcessor / RtkProcessor 调用

---

## 8. 测试验证计划

### 8.1 单元测试

| 模块 | 测试内容 | 验证方法 |
|------|---------|---------|
| `coord_transform` | ECEF↔LLH 往返转换 | 往返误差 < 1e-6 |
| `coord_transform` | ECEF↔ENU 转换 | 与已知值对比 |
| `satpos` | 卫星位置计算 | 与 rtklib-py eph2pos 结果对比 |
| `SppProcessor` | SPP 定位 | 与 rtklib-py pntpos 结果对比 |
| `RtkProcessor (RTD)` | 码双差定位 | 与 rtklib-py / GINav 差分结果对比 |
| `RtkProcessor (RTK)` | 载波双差定位 | 与已知基线对比 |
| `CombDD` | 双差方程构建 | 与手工计算对比 |
| `troposphere` | 对流层改正 | 与 rtklib-py tropmodel 对比 |
| `ionosphere` | 电离层改正 | 与 rtklib-py ionmodel 对比 |

### 8.2 RTD 退化测试

| 测试场景 | 预期行为 |
|---------|---------|
| 正常载波观测 | RTK 模式，尝试模糊度解算 |
| 载波质量差（低 SNR） | 自动退化为 RTD |
| 模糊度 ratio 检验失败 | 退化为 RTD，输出浮点解或码差分解 |
| 周跳检测触发 | 重置相关模糊度，退化 RTD |
| 载波恢复后 | 重新初始化模糊度，恢复 RTK |
| 配置强制 RTD | 始终 RTD，不尝试模糊度解算 |

### 8.3 对比验证策略

1. **rtklib-py 对比**：用相同数据分别运行 rtklib-py 和本项目，逐历元对比位置差异
2. **GREAT-MSF 对比**：对比双差观测方程结构是否一致
3. **已知精确坐标**：验证 SPP 米级、RTD 分米级、RTK 厘米级精度

### 8.4 测试数据

- rtklib-py 自带数据：`library/rtklib-py/data/u-blox/`、`library/rtklib-py/data/phone/`
- 短基线 RTK 数据：验证双差和模糊度解算
- 低成本接收机数据：验证 RTD 退化场景

---

## 9. 队列流水线集成

### 9.1 GNSS 在框架中的双重角色

GNSS 解算在 GInsStream 框架中承担两种角色：

| 角色 | 集成方式 | 实现类 | 触发场景 |
|------|---------|--------|---------|
| **前端里程计（仅 IMU 不可用降级时）** | 策略模式（仅前端） | `GnssPositioningStrategy(OdometryStrategy)` | 无 IMU 或 IMU 故障时，直接用 SPP/RTK/RTD 产生里程计 |
| **后端量测源（仅作用于主滤波 P1）** | 纯队列流水线 | `GnssRoverSensor(BaseSensor)` / `GnssSolSensor(BaseSensor)` | 正常松组合模式下，GNSS 结果作为主滤波 P1 的 EKF 量测输入 |

> **不使用观察者模式**：GNSS 传感器不持有观察者列表，无 `attach/detach/notify`，无 `on_data()` 回调。
> 数据通过 `queue.put()` 推入 sensor_queue → Scheduler 转发到 estimate_queue → `LcIntegration.process_epoch()` 处理。
> **不使用 FusionStrategy 层**：后端融合由 `LcIntegration` 直接承担，无 `LcFusionStrategy` 类。

### 9.2 前端策略类 — GnssPositioningStrategy

将 GNSS 处理器封装为前端里程计策略（仅 IMU 不可用降级时启用），与 `ImuMechStrategy` 实现相同接口，可热切换：

```python
from abc import ABC, abstractmethod

class OdometryStrategy(ABC):
    """前端里程计算法策略基类（定义于 estimator.md）"""

    @abstractmethod
    def execute(self, measurement) -> 'InsState':
        """执行前端里程计解算，返回当前 INS 状态"""
        ...


class GnssPositioningStrategy(OdometryStrategy):
    """GNSS 纯解算前端策略（IMU 不可用降级时启用）

    当无 IMU 或 IMU 不可用时，直接用 GNSS 解算结果产生里程计。
    内部组合 GnssSolutionProvider，委托具体解算。
    与 ImuMechStrategy 实现相同接口，框架可热切换。
    """

    def __init__(self, gnss_provider: 'GnssSolutionProvider'):
        self._provider = gnss_provider  # 依赖注入

    def execute(self, measurement) -> 'InsState':
        """执行 GNSS 纯解算，将定位结果转为 INS 状态"""
        solution = self._provider.get_solution(measurement.timestamp)
        return self._solution_to_state(solution)


class SppStrategy(GnssPositioningStrategy):
    """SPP 单点定位前端策略

    封装 SppProcessor，适用于无基站、低成本场景。
    精度：米级（~10m）
    """

    def __init__(self, options: dict):
        super().__init__(GnssInternalProvider(SppProcessor(options), options))


class RtkStrategy(GnssPositioningStrategy):
    """RTK/RTD 差分定位前端策略

    封装 RtkProcessor，RTD 为其退化模式。
    精度：RTK 厘米级（~0.02m），RTD 分米级（~1m）
    """

    def __init__(self, options: dict):
        super().__init__(GnssInternalProvider(RtkProcessor(options), options))
```

### 9.3 前端策略热切换示例

```python
# 正常模式：IMU 机械编排前端（后端融合由 LcIntegration 直接承担）
odometry = ImuMechStrategy(ins_core, preprocessor)

# IMU 故障：热切换为 GNSS 纯解算前端
odometry = SppStrategy(options)
# 或差分模式
odometry = RtkStrategy(options)

# 框架持有 OdometryStrategy 抽象引用，切换无需改动 LcIntegration
integration._odometry_strategy = odometry
```

### 9.4 纯队列流水线 — GNSS 传感器（无 Subject 角色）

GNSS 传感器继承 `BaseSensor`（**无 Subject 角色**），实现 `get_data()` 接口。
**不持有观察者列表，无 attach/notify**；数据通过 `output_queue.put()` 推入下一级队列：

```python
class BaseSensor(ABC):
    """传感器抽象基类（无 Subject 角色，定义于 StreamDesign.md）

    仅强制 get_data() 接口，不维护观察者列表。
    数据通过 output_queue.put() 推入下一级队列。
    """

    def __init__(self, name: str = ""):
        self._name = name               # 传感器标识
        # 注：不持有 _observers 列表，无 attach/detach/notify 方法

    @property
    def name(self) -> str:
        return self._name

    @abstractmethod
    def get_data(self) -> 'SensorData':
        """强制实现的传感器数据读取接口"""
        ...


class GnssRoverSensor(BaseSensor):
    """GNSS 流动站原始观测值传感器（内部解算模式）

    读取 Rover 原始观测值（伪距/载波/多普勒），
    由 SensorFactory 动态创建，实现 get_data() 接口。
    数据通过 output_queue.put() 推入 rover_queue。
    """

    def get_data(self) -> 'SensorData':
        """读取一历元 GNSS 原始观测值，推入 output_queue"""
        raw_obs = self._reader.read_epoch()
        data = SensorData(tag="gnss_raw", gnss_meas=raw_obs)
        self.output_queue.put(data)   # 推入队列，无 notify
        return data


class GnssSolSensor(BaseSensor):
    """GNSS 外部结果传感器（外部结果模式）

    读取外部 GNSS 定位结果文件（POS/NMEA/CSV），
    由 SensorFactory 动态创建，实现 get_data() 接口。
    数据通过 output_queue.put() 推入 gnss_sol_queue。
    """

    def get_data(self) -> 'SensorData':
        """读取一条外部 GNSS 定位结果，推入 output_queue"""
        sol = self._reader.read_solution()
        data = SensorData(tag="gnss_solution", gnss_solution=sol)
        self.output_queue.put(data)   # 推入队列，无 notify
        return data
```

### 9.5 工厂模式创建 GNSS 传感器

`SensorFactory` 根据配置动态创建 GNSS 传感器实例（参考 StreamDesign.md）：

```python
class SensorFactory:
    """传感器工厂（参考 StreamDesign.md）"""

    @staticmethod
    def create(sensor_type: str, config: dict) -> BaseSensor:
        if sensor_type == "gnss":
            if config.get("gnss_source") == "external":
                return GnssSolSensor(config)   # 外部结果模式
            else:
                return GnssRoverSensor(config) # 内部解算模式
        elif sensor_type == "imu":
            return ImuSensor(config)
        # ...
```

### 9.6 数据流与队列流水线时序

```
内部解算模式:
  GnssRoverSensor.run() 线程
    → 读取原始观测值
    → output_queue.put(SensorData(tag="gnss_raw"))   (推入 rover_queue)
    → Scheduler 从 rover_queue 取数据
    → estimate_queue.put(SensorData)                  (仅转发，不做时间对齐)
    → LcIntegration.process_epoch(epoch_data)
        → GnssInternalProvider.get_solution()        (SPP/RTD/RTK 解算)
        → _gnss_update()                             (EKF 量测更新，仅作用 P1)
        → 双滤波独立反馈

外部结果模式:
  GnssSolSensor.run() 线程
    → 读取外部定位结果
    → output_queue.put(SensorData(tag="gnss_solution"))   (推入 gnss_sol_queue)
    → Scheduler 从 gnss_sol_queue 取数据
    → estimate_queue.put(SensorData)                      (仅转发，不做时间对齐)
    → LcIntegration.process_epoch(epoch_data)
        → GnssExternalProvider 直接返回外部结果
        → _gnss_update()                                  (EKF 量测更新，仅作用 P1)
        → 双滤波独立反馈
```

**关键点**：
- GNSS 数据通过 `queue.put()` 流转，**无 `notify()` 调用**
- `LcIntegration` 从 `estimate_queue` 取数据，**无 `on_data()` 回调**
- 后端融合（EKF + NHC + ZUPT）由 `LcIntegration` 直接承担，**无 `LcFusionStrategy` 中间层**
- GNSS 量测仅作用于主滤波 P1，NHC 子滤波 P2 不直接使用 GNSS 量测

### 9.7 OOP 三大特性体现

| 特性 | 体现 |
|------|------|
| **封装** | GNSS 解算细节封装在 `GnssProcessor` 子类内部，外部仅通过策略接口或队列接口访问 |
| **继承** | `GnssProcessor(ABC)` → `SppProcessor` / `RtkProcessor`；`GnssPositioningStrategy` → `SppStrategy` / `RtkStrategy`；`BaseModel(ABC)` → `CombDD` |
| **多态** | 框架持有 `OdometryStrategy` 抽象引用，运行时调用 `SppStrategy` / `RtkStrategy` / `ImuMechStrategy` |
