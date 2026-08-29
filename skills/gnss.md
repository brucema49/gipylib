# GNSS 算法架构设计

> **当前状态索引（2026-08-29）**：BDS GEO、BDS ISB、BDT→GPST 14 s、RINEX 基站列映射和未初始化相位行保护已完成验证。RTK 当前使用相位/伪距分离门限；BDS-only AR 仍未闭环。详见 [项目当前状态](项目当前状态.md)。

> 基于 rtklib-py 算法参考 + GREAT-MSF 架构模式，设计 GInsStream 流式 GNSS 处理框架。
> 实现 SPP 和 RTK（含 RTD 退化模式）功能，采用 ABC 抽象类继承体系。
>
> **时间系统约定**：全框架内部统一使用 **Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）**。
> rtklib-py 的 `gtime_t.time` 即 Unix 整数秒，`gtime_t.sec` 为不足秒的小数部分；
> 本项目 `GnssSolution.timestamp = sol.t.time + sol.t.sec`，`week` 由 `unix_to_gpst(timestamp)` 派生。
> 时间转换工具：`src/core/time_utils.py`（`gpst_to_unix` / `unix_to_gpst`，`GPST_EPOCH_UNIX = 315964800`）。
>
> **当前实现状态**：
> - ✅ 已实现：rtklib-py 已吸收到 `src/core/gnss/rtklib/`（7 个核心模块：`config.py` / `ephemeris.py` / `mlambda.py` / `pntpos.py` / `postpos.py` / `rinex.py` / `rtkcmn.py` / `rtkpos.py`）
> - ✅ 已实现：`src/core/gnss/gnss_processor.py::GnssProcessor(ABC)` 抽象基类
> - ✅ 已实现：`src/core/gnss/spp_processor.py::SppProcessor`（薄封装 rtklib-py `pntpos`）
> - ✅ 已实现：`src/core/gnss/rtk_processor.py::RtkProcessor`（薄封装 rtklib-py `relpos`）
> - ✅ 已实现：`src/core/gnss/solution_converter.py::sol_to_gnss_solution`（rtklib-py `Sol` → `GnssSolution`）
> - ✅ 已实现：`src/core/gnss/rtklib_config_adapter.py::RtklibEnv` + `build_params`（YAML → rtklib-py 配置注入）
> - ✅ 已实现：`src/stream/internal_gnss_sensor.py::InternalGnssSensor`（内部模式传感器线程，逐历元调用 pntpos/relpos）
> - ✅ 已实现：`src/stream/gnss_sol_sensor.py::GnssSolSensor`（外部模式传感器线程，读取 .pos 文件）
> - 🚧 预留：`GnssSolutionProvider` / `GnssInternalProvider` / `GnssExternalProvider` / `CombDD` / `GnssPositioningStrategy` 等（INS 启用后需要）
>
> **rtklib-py 吸收架构**：
> rtklib-py 原为外部 `library/rtklib-py`，已吸收为 `src/core/gnss/rtklib/` 子包，
> 通过 `config.py` 的 `_CfgProxy` 单例管理配置（由 `RtklibEnv.setup()` 调用 `config.set_params()` 注入），
> 不再修改 `sys.path` 或 `sys.modules`。子包内模块使用相对导入（如 `from .rtkcmn import ...`）。
>
> 在框架中，GNSS 解算承担两种角色：
> - **前端策略（仅 IMU 不可用降级时启用，当前未实现）**：作为 `GnssPositioningStrategy(OdometryStrategy)`，
>   当无 IMU 或 IMU 不可用时，直接用 SPP/RTK/RTD 产生里程计（位置/速度），与 `ImuMechStrategy` 可互换
> - **后端量测源**：松组合后端 EKF 的量测输入（GnssSolution），由 GnssSolutionProvider 统一提供，
>   作用于单滤波 P 矩阵（StateIndex 参数块）
>
> 当前内部模式下，`InternalGnssSensor` 直接调用 `SppProcessor` / `RtkProcessor`，把 `GnssSolution` 通过 `gnss_queue` 推入下游 `SolutionLogger` 输出 `.pos` 文件。
> 外部模式下，`GnssSolSensor` 通过 `PosSolFormator` 解析 `.pos` 文件得到 `GnssSolution`。

---

## 北斗与 BDS-3 处理基线（Data19 已验证）

北斗不是 GPS 频点或 GPS 时标的别名。启用 `BDS` 时，RINEX 解析、广播星历、卫星
坐标/钟差、SPP 星间钟偏和相对定位必须同时处于 BDS 路径；只看到 `Cxx` 卫星进入
观测数组并不足以证明北斗或 BDS-3 可用。

| 项目 | 当前约定/实现 | 不能省略的检查 |
| --- | --- | --- |
| RINEX 观测 | `C1I/L1I` -> 槽位 0，`C7I/L7I` -> 槽位 1；按信号频带号映射 | 分别核对 rover/base 的 P/L/S/LLI；BASE 的 `C,C,L,L,S,S` 排列曾导致 C7I 丢失 |
| BDS 双频 | B1I=`1561.098 MHz`、B2I/B2b=`1207.14 MHz` | `freq_ix0[BDS]=6`、`freq_ix1[BDS]=3`，且 `freq_table[6]` 必须存在；不得误用 GPS L1/L2 |
| 时间 | BDT 转 GPST：`week+1356` 且 `toc/toe/tot+14 s` | 比较选中 TOE/TOC；漏 14 s 会产生数十公里级卫星沿迹误差 |
| GEO 轨道 | PRN `1..5`、`59+` 使用 GEO 专用 5 deg 倾角旋转 | 按 ECEF 三分量和钟差与 RTKLIB 对照，不能只看轨道半径 |
| SPP/TC 时钟 | GPS 公共钟差之外，GLO/GAL/BDS 各有 ISB；SPP 为位置 + 4 个钟差参数 | GPS+BDS 必须估计 BDS ISB，不能把 BDT-GPST 偏差留在伪距残差中 |

Data19 HG4930 端到端验证得到 GPS+BDS RTK 固定率 `99.7%`，且与 RTKLIB 双固定
历元解差 RMSE 为 `2.7 cm`。这验证了 B1I/B2I 和已见 BDS-3 卫星的当前处理链，
并不表示任意 BDS RINEX/B-CNAV2 均已覆盖。BDS-only RTK 仍可能没有 FIX；应以浮点
误差、有效历元和同配置 RTKLIB 对照判断，不能仅以 FIX 率判定失败。

诊断顺序固定为：原始 RINEX 列与信号 -> 频率/波长 -> BDT/GPST 和 TOE/TOC ->
卫星 ECEF/钟差/TGD -> SPP 残差 -> rover/base 公共卫星与 DD 残差 -> AR/最终解。
完整 9-case GPS/BDS/GPS+BDS 对照见
`Data19_20201214_HG4930_CAR_Opensky/bds_diagnosis/README.md` 与
`issue/8-25北斗卫星处理修复.md`。

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

> rtklib-py 已吸收到 `src/core/gnss/rtklib/`，本项目不重新定义观测值/星历数据结构，
> 直接复用 rtklib-py 的 `Obs` / `Eph` / `Nav` / `Sol` / `gtime_t` 对象。
> 仅在边界处通过 `solution_converter.py::sol_to_gnss_solution` 把 `Sol` 转换为本项目 `GnssSolution`。

### 3.1 观测值/星历映射（直接复用 rtklib-py 对象）

| rtklib-py | 本项目使用方式 | 转换说明 |
|-----------|---------------|---------|
| `Obs` (obsr/obsb) | 直接传入 `SppProcessor.process_epoch(obsr)` / `RtkProcessor.process_epoch(obsr, obsb)` | 不转换，rtklib-py 原生对象 |
| `Eph` / `Nav` | 由 `RtklibEnv.init_nav()` 创建，存于 `nav` 对象，处理器持有 `nav` 引用 | 不转换，rtklib-py 原生对象 |
| `gtime_t` | `gtime_t.time`（Unix 整数秒）+ `gtime_t.sec`（小数秒） | 直接用 `sol.t.time + sol.t.sec` 得到 Unix 时间戳 |

### 3.2 解算结果映射（Sol → GnssSolution）

`src/core/gnss/solution_converter.py::sol_to_gnss_solution` 完成转换：

| rtklib-py `Sol` 字段 | 本项目 `GnssSolution` 字段 | 转换说明 |
|-----------|--------|---------|
| `Sol.t.time + Sol.t.sec` | `timestamp` (float) | Unix 时间戳（= gtime_t.time + gtime_t.sec） |
| `unix_to_gpst(timestamp)[0]` | `week` (int) | GPS 周号（由 timestamp 派生） |
| `Sol.rr[0:3]` | `position` (np.ndarray [3]) | ECEF 位置 |
| `Sol.rr[3:6]` | `velocity` (np.ndarray [3]) | ECEF 速度（由 `pntpos::estvel` 多普勒测速或 `relpos` 卡尔曼滤波速度填入） |
| `Sol.stat` | `quality` (int) | 1=SPP, 2=RTD, 4=浮点解, 5=LC（与 rtklib `SOLQ_*` 一致） |
| `Sol.ns` | `num_sv` (int) | 使用卫星数（rtklib-py `pntpos`/`relpos` 不写 `sol.ns`，由处理器回填） |
| `sqrt(diag(Sol.qr[0:3,0:3]))` | `sd` (np.ndarray [3]) | ECEF 位置标准差 (sdx, sdy, sdz) |
| `sqrt(diag(Sol.qv[0:3,0:3]))`（若可用） | `vel_sd` (np.ndarray [3], 可选) | ECEF 速度标准差 |
| `Sol.qr[0:3, 0:3]` | `cov` (np.ndarray [3,3], 可选) | ECEF 协方差矩阵（含非对角项） |

> 解算状态常量（`src/core/gnss/solution_converter.py`）：
> `SOLQ_NONE=0`, `SOLQ_FIX=1`, `SOLQ_FLOAT=2`, `SOLQ_DGPS=4`, `SOLQ_SINGLE=5`。
>
> **速度字段说明**：
> - SPP 模式下，`pntpos()` 内部调用 `estvel()`（基于多普勒观测值的最小二乘测速）将速度填入 `sol.rr[3:6]`
> - RTK 模式下，`relpos()` 卡尔曼滤波状态向量含速度分量，直接填入 `sol.rr[3:6]`
> - 速度用于 INS 动态初始化（速度矢量法，详见 [初始化.md 第 8 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#8-动态初始化---速度矢量初始化)）

---

## 4. SPP 算法流程

> **实现说明**：本项目 `SppProcessor` 是 rtklib-py `pntpos` 的薄封装（参考 `src/core/gnss/spp_processor.py`），
> 不重新实现 SPP 算法。以下流程描述 rtklib-py `pntpos` 内部逻辑，供参考。

### 4.1 SppProcessor.process_epoch() 流程（实际实现）

```python
# src/core/gnss/spp_processor.py
class SppProcessor(GnssProcessor):
    def __init__(self, nav):
        self.nav = nav
        from .rtklib.pntpos import pntpos
        self._pntpos = pntpos

    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]:
        """调用 pntpos 解算单历元 SPP。"""
        sol = self._pntpos(obsr, self.nav)
        # rtklib-py 的 pntpos 不写 sol.ns，用本历元观测卫星数近似
        if sol.stat != SOLQ_NONE and sol.ns == 0:
            sol.ns = len(obsr.sat)
        return sol_to_gnss_solution(sol)
```

**调用链**：`InternalGnssSensor._run_spp_loop` → 遍历 `rov.obslist` → `SppProcessor.process_epoch(obsr)` → `pntpos(obsr, nav)` → `sol_to_gnss_solution(sol)` → 推入 `gnss_queue`。

### 4.2 rtklib-py pntpos 内部算法（参考）

```
pntpos(obs, nav)  # rtklib-py 内部实现
  │
  ├── 1. 卫星位置计算
  │     对每颗卫星:
  │       ├── 从 nav 选择星历（seleph）
  │       ├── eph2pos() 计算卫星位置
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
  │     │     ├── 计算仰角/方位角（satazel）
  │     │     ├── 仰角/信噪比筛选
  │     │     ├── 对流层改正（tropmodel）
  │     │     ├── 电离层改正 Klobuchar（ionmodel）
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
  └── 4. 构建解算结果 Sol
        ├── Sol.rr[0:3] = 位置 (ECEF)
        ├── Sol.rr[3:6] = 速度 (ECEF) ← estvel() 多普勒测速填入
        ├── Sol.dtr = 钟差
        ├── Sol.qr = 位置协方差 = (H^T W H)^{-1}
        ├── Sol.qv = 速度协方差（若 estvel 计算）
        ├── Sol.stat = SOLQ_SINGLE (5)
        └── Sol.t = obs.t (gtime_t, Unix 时间戳)
```

### 4.2.1 多普勒测速（estvel / resdop）

> **实现说明**：本项目在 `src/core/gnss/rtklib/pntpos.py` 中实现了 `estvel()` 和 `resdop()`，
> 由 `pntpos()` 在位置解算完成后调用，将 ECEF 速度填入 `sol.rr[3:6]`。
> 速度用于 INS 动态初始化（速度矢量法，详见 [初始化.md 第 8 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#8-动态初始化---速度矢量初始化)）。

```
estvel(obs, nav, rs, dts, svh, rr)   # 多普勒测速主函数
  │
  ├── 1. 调用 resdop() 构建多普勒观测方程
  │     对每颗卫星（有有效多普勒观测 obs.D[i,0] != 0）:
  │       ├── 卫星速度 rs[i, 3:6]（来自星历）
  │       ├── 视线向量 e = (rs[i, 0:3] - rr) / |·|
  │       ├── 多普勒残差 v = -λ * D + (v_sat - v_rcv) · e + 钟漂项
  │       └── 设计矩阵 H[i, 0:3] = -e, H[i, 3] = 1（钟漂）
  │
  ├── 2. 加权最小二乘求解
  │     dx = (H^T W H)^{-1} H^T W v
  │     其中 W 由 varerr() 仰角加权
  │
  └── 3. 填入 Sol
        ├── sol.rr[3:6] = vel (ECEF 速度)
        └── sol.dtr[1]  = 钟漂（如有）
```

**多普勒测速要点**：
- 观测值：多普勒观测 `obs.D[i, 0]`（L1 频点）
- 待估参数：`[vx, vy, vz, c·dt_r_dot]`（3 速度 + 1 钟漂）
- 最小卫星数：4 颗（与位置解算一致）
- 载波波长：`λ = c / f`（由 `nav.freq[0]` 计算）
- 卫星速度：由星历计算，存于 `rs[i, 3:6]`

### 4.3 SPP 观测方程要点

- **观测值**：仅伪距（码观测值）
- **待估参数**：[x, y, z, c·dt_r]（3 位置 + 1 钟差）
- **最小卫星数**：4 颗
- **权重模型**：仰角加权 σ = σ_code / sin(el)
- **大气改正**：模型改正（Saastamoinen 对流层 + Klobuchar 电离层）

---

## 5. RTK/RTD 算法流程

> **实现说明**：本项目 `RtkProcessor` 是 rtklib-py `relpos` 的薄封装（参考 `src/core/gnss/rtk_processor.py`），
> 不重新实现 RTK 算法。RTD 不是独立模块，而是 rtklib-py `relpos` 内部的退化分支
> （`armode=0` 或 ratio 检验失败时返回 `SOLQ_DGPS=4` 即 RTD 浮点解）。
> 以下流程描述 rtklib-py `relpos` 内部逻辑，供参考。

### 5.1 RTK 与 RTD 的关系（rtklib-py 内部）

```
rtklib-py relpos(nav, obsr, obsb, sol)   # 单历元相对定位
  │
  ├── 默认尝试 RTK（载波双差 + 模糊度解算）
  │     观测值 = 码双差 + 载波双差
  │     待估参数 = 位置 + 速度 + 模糊度
  │     模糊度解算 = LAMBDA（受 armode / thresar 控制）
  │
  └── 退化分支（自动）：
        载波质量不足 / ratio 检验失败 / 周跳重置 / armode=0
        → 返回 SOLQ_DGPS (4) 即 RTD 浮点解（仅码双差）
```

| 维度 | RTK 模式（`SOLQ_FIX=1` / `SOLQ_FLOAT=2`） | RTD 退化（`SOLQ_DGPS=4`） |
|------|---------|-------------|
| 双差类型 | 码双差 + 载波双差 | 仅码双差 |
| 观测值 | 伪距 + 载波相位 | 仅伪距 |
| 待估参数 | 位置 + 模糊度向量 | 仅位置 |
| 模糊度解算 | LAMBDA / fix-and-hold | 跳过 |
| 精度 | 厘米~分米级 | 分米~米级 |
| 退化触发 | — | 载波质量差 / ratio 失败 / 周跳重置 / `armode=0` |

> **退化由 rtklib-py 内部自动处理**，本项目 `RtkProcessor` 不实现退化逻辑，
> 仅通过 `sol.stat` 字段透传解算类型（`SOLQ_FIX`/`SOLQ_FLOAT`/`SOLQ_DGPS`）。

### 5.2 RtkProcessor.process_epoch() 流程（实际实现）

```python
# src/core/gnss/rtk_processor.py
class RtkProcessor(GnssProcessor):
    def __init__(self, nav):
        self.nav = nav
        from .rtklib.pntpos import pntpos
        from .rtklib.rtkpos import relpos, timediff
        from .rtklib.rtkcmn import Sol, gtime_t
        self._pntpos = pntpos
        self._relpos = relpos
        self._timediff = timediff
        self._Sol = Sol
        self._gtime_t = gtime_t
        self.sol = self._Sol()              # 初始 sol，rr[0]==0 触发首历元 pntpos
        self._prev_t = self._gtime_t()      # 初始为 0，首历元不计算 nav.tt

    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]:
        """调用 relpos 解算单历元 RTK。"""
        # 1. 首历元或 sol.rr[0]==0 时先 pntpos 取初值
        if self.nav.use_sing_pos or self.sol.stat == SOLQ_NONE or self.sol.rr[0] == 0.0:
            self.sol = self._pntpos(obsr, self.nav)
            # 用 SPP 解初始化 nav.x，供 relpos 的 zdres 计算流动站位置
            self.nav.x[0:6] = self.sol.rr[0:6]
            self.nav.x[6:9] = 1e-6  # match RTKLIB
        else:
            self.sol = self._Sol()

        # 2. 确保时间戳正确
        if self.sol.t.time == 0:
            self.sol.t = obsr.t

        # 3. 计算 nav.tt（当前历元与上历元的时间差），对应 rtkpos.py:1106-1107
        # relpos 内的 udpos/udbias 依赖 nav.tt 做状态传播与过程噪声注入
        if self._prev_t.time != 0:
            self.nav.tt = self._timediff(self.sol.t, self._prev_t)

        # 4. 相对定位（修改 self.sol 与 self.nav 状态）
        self._relpos(self.nav, obsr, obsb, self.sol)

        # 5. 记录本历元时间，供下一历元计算 nav.tt
        self._prev_t.time = self.sol.t.time
        self._prev_t.sec = self.sol.t.sec

        # 6. rtklib-py 的 relpos 不写 sol.ns，用 nav.ns 回退
        if self.sol.stat != SOLQ_NONE and self.sol.ns == 0:
            self.sol.ns = self.nav.ns if self.nav.ns > 0 else len(obsr.sat)

        return sol_to_gnss_solution(self.sol)
```

**调用链**：`InternalGnssSensor._run_rtk_loop` → `first_obs`/`next_obs` 时间同步 → `RtkProcessor.process_epoch(obsr, obsb)` → `relpos(nav, obsr, obsb, sol)` → `sol_to_gnss_solution(sol)` → 推入 `gnss_queue`。

**跨历元状态管理**：
- `self.sol`: 上历元解算结果（用于判断是否需要重新 SPP 取初值）
- `self._prev_t`: 上历元解算时间（用于计算 `nav.tt`，TimeDiff）
- `nav`: 由调用方管理，持有 `x`/`P`/`azel`/`lock` 等状态

### 5.3 rtklib-py relpos 内部算法（参考）

```
relpos(nav, obsr, obsb, sol)  # rtklib-py 内部实现
  │
  ├── 1. 卫星位置计算（satposs）
  │
  ├── 2. Rover-Ref 时间匹配（首历元 first_obs，后续 next_obs）
  │     ├── rnx_decodefirst_obs(nav, rov, base, dir) 做初始时间对齐
  │     └── rnx_decodenext_obs(nav, rov, base, dir) 推进到下一历元
  │
  ├── 3. 共视卫星筛选 + 参考卫星选择（最高仰角）
  │
  ├── 4. 构建双差观测方程（zdres + ddres）
  │     ├── 码双差（RTD/RTK 共用）
  │     │     Δ∇P = P_rover,i - P_rover,j - P_ref,i + P_ref,j
  │     └── 载波双差（仅 RTK）
  │           Δ∇L = L_rover,i - L_rover,j - L_ref,i + L_ref,j
  │
  ├── 5. 状态更新（udpos / udion / udbias）
  │     ├── 位置/速度预测（Kalman 状态传播，依赖 nav.tt）
  │     ├── 电离层状态传播
  │     └── 模糊度状态传播（周跳检测 → 重置）
  │
  ├── 6. Kalman 滤波量测更新（holdamb / relpos_filter）
  │     ├── 待估参数 = 位置 + 速度 + 电离层 + 模糊度
  │     └── 状态协方差更新
  │
  ├── 7. 模糊度解算（resamb_LAMBDA）
  │     ├── armode != 0 → 尝试 LAMBDA
  │     ├── ratio 检验通过 → SOLQ_FIX (1)
  │     ├── ratio 检验失败 → SOLQ_FLOAT (2)
  │     └── armode == 0 或载波不足 → SOLQ_DGPS (4) 即 RTD 退化
  │
  └── 8. 构建解算结果 Sol
        ├── Sol.rr[0:3] = 位置 (ECEF)
        ├── Sol.stat = SOLQ_FIX / SOLQ_FLOAT / SOLQ_DGPS
        ├── Sol.qr = 协方差矩阵
        └── Sol.t = obsr.t (gtime_t, Unix 时间戳)
```

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

## 7. 坐标变换与时间工具

### 7.1 实际实现策略

**坐标变换函数不独立提取**：rtklib-py 已吸收到 `src/core/gnss/rtklib/`，
本项目 `SppProcessor` / `RtkProcessor` 直接调用 rtklib-py `rtkcmn.py` 中的
`ecef2pos` / `pos2ecef` / `ecef2enu` / `enu2ecef` 等函数，无需重复实现。

仅在边界处（如 `solution_writer.py` 输出 LLH/ENU 时）通过 rtklib-py 函数完成坐标变换。

**时间转换工具已独立提取**：`src/core/time_utils.py` 提供 Unix ↔ GPST 转换，
供 `formators.py`（输入解码）/ `solution_writer.py` / `aligned_writer.py`（输出）使用。

### 7.2 时间工具（src/core/time_utils.py）

| 函数 | 签名 | 说明 |
|------|------|------|
| `gpst_to_unix` | `(week: int, sow: float) -> float` | GPS 周+周内秒 → Unix 时间戳 |
| `unix_to_gpst` | `(unix_ts: float) -> Tuple[int, float]` | Unix 时间戳 → (GPS 周, 周内秒) |

**常量**：
- `GPST_EPOCH_UNIX = 315964800`（1980-01-06 00:00:00 UTC 的 Unix 时间戳）
- `SECONDS_PER_WEEK = 604800`

### 7.3 rtklib-py 坐标变换函数（直接复用）

| rtklib-py 函数 | 说明 | 调用位置 |
|---------------|------|---------|
| `ecef2pos()` | ECEF → LLH [lat, lon, h] (rad, rad, m) | `solution_writer.py` 输出 |
| `pos2ecef()` | LLH → ECEF | 外部结果输入解析 |
| `ecef2enu()` | ECEF 差值 → ENU | `solution_writer.py` 协方差旋转 |
| `enu2ecef()` | ENU → ECEF 差值 | 外部结果输入 |
| `xyz2enu()` | ECEF→ENU 旋转矩阵 | `solution_writer.py` 协方差旋转 |

### 7.4 调用示例

```python
# 输入端：formators.py 把 GPS 周+周内秒转为 Unix 时间戳
from src.core.time_utils import gpst_to_unix
timestamp = gpst_to_unix(week, sow)

# 解算端：直接复用 rtklib-py 函数，无需转换
from src.core.gnss.rtklib.rtkcmn import ecef2pos, ecef2enu
lat, lon, h = ecef2pos(sol.rr[0:3])

# 输出端：solution_writer.py 把 Unix 时间戳转回 (week, sow) 写文件
from src.core.time_utils import unix_to_gpst
week, sow = unix_to_gpst(sol.timestamp)
```

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

| 角色 | 集成方式 | 实现类 | 触发场景 | 实现状态 |
|------|---------|--------|---------|---------|
| **后端量测源（作用于单滤波 P）** | 纯队列流水线 | `InternalGnssSensor` / `GnssSolSensor` | 当前实现：internal/external 两种模式 | ✅ 已实现 |
| **前端里程计（仅 IMU 不可用降级时）** | 策略模式（仅前端） | `GnssPositioningStrategy(OdometryStrategy)` | IMU 不可用时，直接用 SPP/RTK 产生里程计 | 🚧 预留 |

> **不使用观察者模式**：GNSS 传感器不持有观察者列表，无 `attach/detach/notify`，无 `on_data()` 回调。
> 数据通过 `queue.put()` 推入 gnss_queue → Scheduler 转发到 estimate_queue → `LcIntegration.process_epoch()` 处理。
> **不使用 FusionStrategy 层**：后端融合由 `LcIntegration` 直接承担，无 `LcFusionStrategy` 类。

### 9.2 实际传感器实现

#### 9.2.1 InternalGnssSensor（内部解算模式，✅ 已实现）

`src/stream/internal_gnss_sensor.py::InternalGnssSensor` 是一个 `Thread` 子类，
根据 `positioning_mode` 创建 `SppProcessor` 或 `RtkProcessor`，逐历元解算并推入 `gnss_queue`：

```python
# src/stream/internal_gnss_sensor.py
class InternalGnssSensor(Thread):
    """内部 GNSS 解算传感器线程。"""

    def __init__(self, config: dict, output_queue: Queue, control: ThreadControl):
        Thread.__init__(self, name="InternalGnssSensor", daemon=True)
        self.gnss_cfg = config["gnss"]
        self.output_queue = output_queue
        self.control = control

    def _run_impl(self):
        # 1. 初始化 rtklib 环境 + nav
        env = RtklibEnv(self.gnss_cfg)
        env.setup()
        nav = env.init_nav()

        # 2. 准备 RINEX 文件（必要时简化）
        rover_path = self._prepare_rinex(self.gnss_cfg["rover_path"])

        # 3. 加载流动站观测值 + 星历
        from src.core.gnss.rtklib import rinex as rn
        rov = rn.rnx_decode(env.get_cfg())
        rov.decode_obsfile(nav, rover_path, None)
        rov.decode_nav(self.gnss_cfg["eph_path"], nav)

        # 4. RTK 模式加载基站
        #    rb_format 配置项指定基站坐标格式：
        #      "xyz"（默认，ECEF 米）或 "llh"（lat_deg, lon_deg, h_m）
        base = None
        if self.gnss_cfg.get("positioning_mode") == "rtk":
            base_path = self._prepare_rinex(self.gnss_cfg["base_path"])
            base = rn.rnx_decode(env.get_cfg())
            base.decode_obsfile(nav, base_path, None)
            if nav.rb[0] == 0:
                nav_rb = base.pos

        # 5. 创建处理器并运行
        mode = self.gnss_cfg["positioning_mode"]
        if mode == "spp":
            processor = SppProcessor(nav)
            self._run_spp_loop(processor, rov)
        elif mode == "rtk":
            processor = RtkProcessor(nav)
            self._run_rtk_loop(processor, rov, base, nav, rn)

    def _run_spp_loop(self, processor, rov):
        """SPP 模式: 直接遍历 rover.obslist。"""
        for obsr in rov.obslist:
            if not self.control.is_running():
                break
            sol = processor.process_epoch(obsr)
            if sol is not None:
                self.output_queue.put(SensorData(tag="gnss_solution", gnss_solution=sol))

    def _run_rtk_loop(self, processor, rov, base, nav, rn):
        """RTK 模式: 用 first_obs/next_obs 做时间同步。"""
        dir = 1  # forward
        obsr, obsb = rn.first_obs(nav, rov, base, dir)
        while True:
            if not self.control.is_running():
                break
            if obsr == []:
                break
            sol = processor.process_epoch(obsr, obsb)
            if sol is not None:
                self.output_queue.put(SensorData(tag="gnss_solution", gnss_solution=sol))
            obsr, obsb = rn.next_obs(nav, rov, base, dir)
```

**关键点**：
- 文件读完后推入 `None` 作为 EOF sentinel（在 `run()` 的 `finally` 块中）
- 支持临时简化 RINEX 文件（`_prepare_rinex`），处理完成后自动清理
- SPP 模式直接遍历 `rov.obslist`，RTK 模式用 `first_obs`/`next_obs` 做时间同步

#### 9.2.2 GnssSolSensor（外部结果模式，✅ 已实现）

`src/stream/gnss_sol_sensor.py::GnssSolSensor` 是一个 `Thread` 子类，
通过 `PosSolFormator` 解析 `.pos` 文件得到 `GnssSolution`，推入 `gnss_queue`：

```python
# src/stream/gnss_sol_sensor.py（简化示意）
class GnssSolSensor(Thread):
    """外部 GNSS 结果传感器线程。"""

    def __init__(self, config: dict, output_queue: Queue, control: ThreadControl):
        Thread.__init__(self, name="GnssSolSensor", daemon=True)
        self.gnss_cfg = config["gnss"]
        self.output_queue = output_queue
        self.control = control

    def run(self):
        try:
            formator = PosSolFormator(self.gnss_cfg)
            for line in open(self.gnss_cfg["external_sol_path"]):
                if not self.control.is_running():
                    break
                sol = formator.decode(line)
                if sol is not None:
                    self.output_queue.put(SensorData(tag="gnss_solution", gnss_solution=sol))
        finally:
            self.output_queue.put(None)  # EOF sentinel
```

### 9.3 前端策略类 — GnssPositioningStrategy（🚧 预留）

> **当前未实现**：以下设计为 IMU 不可用降级时的预留方案。
> 当前三种运行模式（`ins.enabled`：`off`/`on`/`tc`）中，`off` 模式下 `InternalGnssSensor` 直接产出 `GnssSolution` 推入队列，由 `SolutionLogger` 输出 `.pos` 文件；`on`/`tc` 模式下由 `LcStream`/`TcStream` 处理后经 `RSLTWriter` 输出 `.rslt`（100Hz）。

将 GNSS 处理器封装为前端里程计策略（仅 IMU 不可用降级时启用），与 `ImuMechStrategy` 实现相同接口，可热切换：

```python
class OdometryStrategy(ABC):
    """前端里程计算法策略基类（定义于 estimator.md）"""
    @abstractmethod
    def execute(self, measurement) -> 'InsState': ...


class GnssPositioningStrategy(OdometryStrategy):
    """GNSS 纯解算前端策略（IMU 不可用降级时启用）"""
    def __init__(self, gnss_provider: 'GnssSolutionProvider'):
        self._provider = gnss_provider

    def execute(self, measurement) -> 'InsState':
        solution = self._provider.get_solution(measurement.timestamp)
        return self._solution_to_state(solution)
```

### 9.4 数据流与队列流水线时序

三种运行模式由 `ins.enabled` 配置项决定（`coupling_mode` 字段已移除）：

```
模式 1：纯 GNSS (ins.enabled=off, 当前实现):
  InternalGnssSensor.run() 线程
    → RtklibEnv.setup() + init_nav()
    → rnx_decode().decode_obsfile() / decode_nav()
    → SppProcessor / RtkProcessor 逐历元 process_epoch()
    → sol_to_gnss_solution(sol)
    → output_queue.put(SensorData(tag="gnss_solution"))   (推入 gnss_queue)
    → SolutionLogger 从 gnss_queue 取数据
    → SolutionWriter.write(sol)                            (写 .pos 文件)
    → 文件结束推入 None sentinel

模式 2：松组合 LC (ins.enabled=lc, 当前实现):
  InternalGnssSensor.run() 线程
    → ... 同纯 GNSS 模式的解算流程 ...
    → output_queue.put(SensorData(tag="gnss_solution"))   (推入 gnss_queue)
  ImuSensor.run() 线程
    → ImuFormator 逐行解析 IMU CSV
    → output_queue.put(SensorData(tag="imu"))              (推入 imu_queue)
  LcStream 处理流
    → 时间对齐 + 单滤波 EKF (P)
    → RSLTWriter.write()                                   (写 .rslt 文件, 100Hz)

模式 3：紧组合 TC (ins.enabled=tc, 当前实现):
  InternalGnssSensor.run() 线程
    → ... 同纯 GNSS 模式的解算流程 ...
    → output_queue.put(SensorData(tag="gnss_solution"))   (推入 gnss_queue)
  ImuSensor.run() 线程
    → ImuFormator 逐行解析 IMU CSV
    → output_queue.put(SensorData(tag="imu"))              (推入 imu_queue)
  TcStream 处理流
    → 时间对齐 + 单滤波 EKF (P)
    → RSLTWriter.write()                                   (写 .rslt 文件, 100Hz)
```

**关键点**：
- GNSS 数据通过 `queue.put()` 流转，**无 `notify()` 调用**
- `LcIntegration` 从 `estimate_queue` 取数据，**无 `on_data()` 回调**
- 后端融合（EKF + NHC + ZUPT）由 `LcIntegration` 直接承担，**无 `LcFusionStrategy` 中间层**
- GNSS 量测作用于单滤波 P 矩阵

### 9.5 OOP 三大特性体现

| 特性 | 体现 |
|------|------|
| **封装** | GNSS 解算细节封装在 `GnssProcessor` 子类内部，外部仅通过队列接口访问；`InternalGnssSensor` 封装 RINEX 加载、简化、解算全流程 |
| **继承** | `GnssProcessor(ABC)` → `SppProcessor` / `RtkProcessor`；`InternalGnssSensor` / `GnssSolSensor` 继承 `Thread` |
| **多态** | `InternalGnssSensor` 根据 `positioning_mode` 动态创建 `SppProcessor` 或 `RtkProcessor`，调用统一的 `process_epoch()` 接口 |
