# 日志流和输出结果流方案

> 规划日志记录和输出结果的流式设计方案，与 StreamDesign.md 中的多线程架构衔接。
>
> **时间系统约定**：全框架内部统一使用 **Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）**。
> 输出文件（POS/CSV）的时间字段通过 `unix_to_gpst()` 转回 (week, sow) 写入，便于与 rtklib 输出格式对齐。
>
> **当前实现状态**：
> - ✅ 已实现：`src/log/writer_base.py::WriterBase(ABC)`、`src/log/solution_writer.py::SolutionWriter`（rtklib 风格 .pos，接受 `position_format`/`time_format` 参数）、`src/log/aligned_writer.py::AlignedWriter`（对齐块状 CSV）
> - ✅ 已实现：`src/log/rslt_writer.py::RSLTWriter`（INS 结果输出，接受 `position_format`/`time_format` 参数，100Hz `.rslt`）
> - ✅ 已实现：`src/log/trace_file_writer.py::TraceFileWriter`（生成 `.trace` 文件，含 GPS week+sow 时间戳，过滤无效调试行如 `pos=[0. 0. 0.]`、`x[clk]=N/A`、`clk_stored=[0. 0. 0.]`）
> - ✅ 已实现：`src/log/logger.py::Logger`（外部模式 / internal+on 模式：消费 imu_queue + gnss_queue，匹配后写 AlignedWriter；internal+on 模式下同时收集 IMU+GNSS 数据，流式结束后批量运行 LcRunner 输出松组合 .pos）
> - ✅ 已实现：`src/log/solution_logger.py::SolutionLogger`（内部模式：仅消费 gnss_queue，写 SolutionWriter）
> - ✅ 已实现：`src/log/aligner.py::Aligner`（IMU 积攒 + GNSS 收割的匹配器，时间戳基于 Unix）
> - ✅ 已实现：`src/core/ins/initializer.py::InsInitializer`（INS 初始化，三种模式 + 三阈值检验，详见 [初始化.md](file:///home/mxl/workplace/gipylib/skills/初始化.md)）
> - ✅ 已实现：`src/core/ins/lc_runner.py::LcRunner`（松组合批处理运行器，收集 IMU+GNSS 后批量执行 LC EKF，输出松组合 .pos，详见 [estimator.md](file:///home/mxl/workplace/gipylib/skills/estimator.md)）
> - 🚧 预留：RawDataWriter / Solution CSV/NMEA 输出 / INS 状态输出（当前未实现）
>
> **三种运行模式**（由 `ins.enabled` 配置项决定）：
> - `ins.enabled=off`（纯 GNSS）→ SolutionLogger + SolutionWriter → `.pos`
> - `ins.enabled=on`（松组合 LC）→ LcStream + RSLTWriter → `.rslt`（100Hz）
> - `ins.enabled=tc`（紧组合 TC）→ TcStream + RSLTWriter → `.rslt`（100Hz）
>
> **输出格式配置**：`output.position_format`（`llh`/`xyz`）、`output.time_format`（`gpst`/`datetime`）、`trace_level`（0-3）。
>
> **框架设计模式集成**：
> - **纯队列流水线**：`Logger` / `SolutionLogger` 作为估计线程或传感器线程的下游消费者，从对应队列取数据（`queue.get()`），**无观察者回调、无 notify()**，与传感器层统一为纯队列流水线
> - **依赖注入**：`Logger` / `SolutionLogger` 通过构造函数接收 `WriterBase` 实例，而非内部 new，便于替换输出格式
> - **OOP 三大特性**：封装（输出逻辑封装在 Writer 子类）、继承（`WriterBase(ABC)` → SolutionWriter/AlignedWriter）、多态（Logger 持有 WriterBase 抽象引用）

---

## 目录

- [1. 概述](#1-概述)
- [2. 日志类型与架构](#2-日志类型与架构)
- [3. Writer 类继承体系](#3-writer-类继承体系)
- [4. SolutionWriter — 解算结果输出](#4-solutionwriter--解算结果输出)
- [5. TraceFileWriter — 运行轨迹/调试输出](#5-tracefilewriter--运行轨迹调试输出)
- [6. RawDataWriter — 原始数据记录](#6-rawdatawriter--原始数据记录)
- [7. Logger 线程设计](#7-logger-线程设计)
- [8. 输出格式定义](#8-输出格式定义)
- [9. 与 StreamDesign.md 的衔接](#9-与-streamdesignmd-的衔接)
- [10. 设计模式集成](#10-设计模式集成)

---

## 1. 概述

### 1.1 模块文件

| 文件 | 职责 | 实现状态 |
|------|------|---------|
| `src/log/writer_base.py` | 输出器抽象基类 `WriterBase(ABC)`，定义 open/write/close 生命周期 | ✅ 已实现 |
| `src/log/solution_writer.py` | rtklib 风格 `.pos` 输出 `SolutionWriter(WriterBase)`，ECEF→LLH，sd ECEF→ENU，接受 `position_format`/`time_format` 参数 | ✅ 已实现 |
| `src/log/rslt_writer.py` | INS 结果输出 `RSLTWriter(WriterBase)`，100Hz `.rslt`，接受 `position_format`/`time_format` 参数 | ✅ 已实现 |
| `src/log/aligned_writer.py` | 对齐块状 CSV 输出 `AlignedWriter(WriterBase)`，G 行 + N 行 I | ✅ 已实现 |
| `src/log/trace_file_writer.py` | 轨迹/调试输出 `TraceFileWriter`，生成 `.trace` 文件，含 GPS week+sow 时间戳，过滤无效调试行 | ✅ 已实现 |
| `src/log/aligner.py` | IMU 积攒 + GNSS 收割的匹配器 `Aligner`（harvest_window=1.0s） | ✅ 已实现 |
| `src/log/logger.py` | 日志线程 `Logger`（external+on / internal+on 模式，消费 imu_queue + gnss_queue；internal+on 下同时收集数据供 LcRunner 批量运行） | ✅ 已实现 |
| `src/log/solution_logger.py` | 内部模式日志线程 `SolutionLogger`（仅消费 gnss_queue） | ✅ 已实现 |
| `src/core/ins/lc_runner.py` | 松组合批处理运行器 `LcRunner`（收集 IMU+GNSS 后批量执行 LC EKF，输出松组合 .pos） | ✅ 已实现 |
| `src/log/raw_data_writer.py` | 原始数据记录 `RawDataWriter(WriterBase)` | 🚧 预留 |

### 1.2 日志层次

```
┌─────────────────────────────────────────────────────────────────┐
│                     日志层次                                     │
│                                                                  │
│  Level 0: 无输出                                                 │
│  Level 1: 基本输出 — 解算结果（位置、速度、姿态、状态）           │
│  Level 2: 详细输出 — Level 1 + 协方差、卫星数、DOP、零偏         │
│  Level 3: 调试输出 — Level 2 + 观测残差、卡尔曼增益、F/H矩阵    │
│                                                                  │
│  运行日志: 使用 Python logging 模块，输出到终端和文件             │
└─────────────────────────────────────────────────────────────────┘
```

---

## 2. 日志类型与架构

### 2.1 三类日志

| 日志类型 | 数据来源 | 输出目标 | 优先级 | 是否必须 |
|----------|---------|---------|--------|---------|
| **解算日志 (Solution)** | `solution_queue` | 文件 + 终端 | 高 | 是 |
| **轨迹日志 (Trace)** | `trace_queue` | 文件 | 中 | 可选 |
| **原始数据日志 (Raw)** | `raw_log_queue` | 文件 | 低 | 可选 |

### 2.2 日志数据流

```
LcIntegration ──→ solution_queue ──→ Logger ──→ SolutionWriter ──→ solution.pos
(估计线程)            │                                │
                     │     trace_queue ───────────────┼──→ TraceWriter ──→ trace.txt
                     │                                │
Stream ─────→ raw_log_queue ─────────────────────────┴──→ RawDataWriter ──→ raw/
(Streamer)
```

> 说明：`LcIntegration` 作为估计线程，将单滤波解算结果 `Solution` 通过 `solution_queue.put()` 推入队列；`Logger` 通过 `solution_queue.get()` 消费，**全程无观察者回调**。`trace_queue` 同理由 `LcIntegration` 生产，用于记录单滤波预测/更新事件。

### 2.3 队列定义

| 队列 | 生产者 | 消费者 | 容量 | 数据类型 | 说明 |
|------|--------|--------|------|---------|------|
| `solution_queue` | `LcIntegration` 估计线程 | Logger 线程 | 200 | `Solution` | 单滤波融合解（反馈后输出） |
| `trace_queue` | `LcIntegration` 估计线程 | Logger 线程 | 500 | `TraceData` | 单滤波预测/更新事件 |
| `raw_log_queue` | 各 Streamer（ImuSensor/GnssRoverSensor 等） | Logger 线程 | 500 | `SensorData` | 原始观测数据，由传感器线程 `put()` |

> 三条队列均为 `queue.Queue`（线程安全），生产者通过 `put()` 推数据，Logger 通过 `get()` 消费，**无 notify() 回调、无观察者模式**。队列满时 `trace_queue` / `raw_log_queue` 静默丢弃，`solution_queue` 阻塞或丢弃（由配置决定）。

---

## 3. Writer 类继承体系

### 3.1 类图

```
                ┌──────────────────────┐
                │   WriterBase(ABC)    │
                │──────────────────────│
                │ + open()    [抽象]   │
                │ + write()   [抽象]   │
                │ + close()   [抽象]   │
                └──────────┬───────────┘
                           │
           ┌───────────────┼───────────────┐
           │               │               │
┌──────────▼─────────┐ ┌──▼──────────────┐ ┌▼──────────────────┐
│  SolutionWriter    │ │  TraceWriter    │ │  RawDataWriter    │
│────────────────────│ │─────────────────│ │───────────────────│
│ + open()           │ │ + open()        │ │ + open()          │
│ + write()          │ │ + write()       │ │ + write()         │
│ + close()          │ │ + close()       │ │ + close()         │
│────────────────────│ │─────────────────│ │───────────────────│
│ + write_header()   │ │ + write_state_  │ │                   │
│ + write_pos()      │ │   change()      │ │                   │
│ + write_csv()      │ │ + write_gnss_   │ │                   │
│ + write_nmea()     │ │   update()      │ │                   │
│                    │ │ + write_nhc_    │ │                   │
│                    │ │   update()      │ │                   │
│                    │ │ + write_zupt_   │ │                   │
│                    │ │   update()      │ │                   │
│                    │ │ + write_ekf_    │ │                   │
│                    │ │   predict()     │ │                   │
└────────────────────┘ └─────────────────┘ └───────────────────┘
```

### 3.2 WriterBase 抽象基类

```python
from abc import ABC, abstractmethod

class WriterBase(ABC):
    """输出器抽象基类

    所有输出器的统一接口，定义 open/write/close 生命周期。
    """

    @abstractmethod
    def open(self) -> None:
        """打开输出目标（文件、目录等）"""
        ...

    @abstractmethod
    def write(self, data) -> None:
        """写入一条数据"""
        ...

    @abstractmethod
    def close(self) -> None:
        """关闭输出目标，释放资源"""
        ...
```

---

## 4. SolutionWriter — 解算结果输出

### 4.1 类签名（实际实现）

```python
class SolutionWriter(WriterBase):
    """rtklib 风格 .pos 输出器（src/log/solution_writer.py）。

    输出格式（参考 rtklib-py postpos.savesol）:
        表头 + 每历元一行: week sow lat lon h Q ns sdn sde sdu sdne sdeu sdun age ratio
    ECEF → LLH（度）转换，sd ECEF → ENU。
    age/ratio 填 0（本项目 GnssSolution 无此字段）。
    timestamp 是 Unix 时间戳，输出时通过 unix_to_gpst() 转回 (week, sow)。

    输出格式可配置:
        position_format: "llh"（经纬度高，默认）或 "xyz"（ECEF 米）
        time_format: "gpst"（GPS 周内秒，默认）或 "datetime"（日历时间）
    """

    HEADER = (
        "%  GPST          latitude(deg) longitude(deg)  height(m)   Q  "
        "ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n"
    )

    def __init__(self, output_dir: str, filename: str = "solution.pos",
                 position_format: str = "llh", time_format: str = "gpst"): ...
    def open(self) -> None: ...
    def write(self, sol: GnssSolution) -> None: ...
    def close(self) -> None: ...
```

### 4.2 方法职责

| 方法 | 职责 |
|------|------|
| `open()` | 创建输出目录与文件，写入 `HEADER` |
| `write(sol)` | ECEF→LLH、sd ECEF→ENU（含非对角项），写入一行 .pos；week/sow 由 `unix_to_gpst(sol.timestamp)` 得到 |
| `close()` | 关闭文件句柄 |

### 4.3 ENU 协方差映射

SolutionWriter 优先使用 `GnssSolution.cov`（3×3 ECEF 完整协方差，含非对角项），与 rtklib-py `covenu` 一致；
若 `cov` 为 None，回退到 `diag(sd**2)` 仅对角线（兼容旧数据）。
ENU 协方差矩阵索引：`[0,0]=E, [1,1]=N, [2,2]=U`（与 rtklib-py `xyz2enu` 一致）。
输出顺序对齐 rtklib-py `postpos.savesol`：`sdn=N, sde=E, sdu=U, sdne=EN, sdeu=UE, sdun=NU`。

### 4.4 GnssSolution 数据结构（实际实现，src/core/data_types.py）

```python
@dataclass
class GnssSolution:
    """外部 GNSS 结果 / 内部 GNSS 解算结果（统一格式）"""
    timestamp: float          # Unix 时间戳（秒，与 rtklib-py gtime_t 一致）
    week: int                 # GPS 周号（由 timestamp 派生，便利字段）
    position: np.ndarray      # [3] ECEF (m)
    quality: int              # 1=SPP, 2=RTD, 5=LC（与 rtklib Sol.stat 一致）
    num_sv: int
    sd: np.ndarray            # [3] 位置标准差 (sdx, sdy, sdz)，ECEF
    cov: Optional[np.ndarray] = None  # [3,3] ECEF 协方差矩阵（可选，含非对角项）
```

> 内部模式由 `src/core/gnss/solution_converter.py::sol_to_gnss_solution` 把 rtklib-py `Sol` 对象转换为本项目 `GnssSolution`：
> `unix_ts = sol.t.time + sol.t.sec`，`week, _ = unix_to_gpst(unix_ts)`，`cov = sol.qr[0:3, 0:3]`。

### 4.5 单滤波 Solution（INS 启用后，预留）

INS 启用后（`internal` + `ins.enabled: "lc"`），输出 Solution 会扩展为含 IMU 状态的完整字段：

```python
@dataclass
class Solution:  # 预留，当前未实现
    """解算结果（单滤波反馈后输出）"""
    timestamp: float                          # Unix 时间戳 (s)
    position_enu: Optional[np.ndarray] = None # [3] ENU 位置 (m)
    velocity_enu: Optional[np.ndarray] = None # [3] ENU 速度 (m/s)
    attitude: Optional[np.ndarray] = None     # [3] 横滚/俯仰/航向 (rad)
    covariance_p: Optional[np.ndarray] = None # [15,15]~[24,24] 单滤波协方差 P (E 系)，维度 = StateIndex.dim
    status: str = "None"                      # None/SPP/RTD/RTK/LC/External
    num_satellites: int = 0
    pdop: float = 0.0
    gyro_bias: Optional[np.ndarray] = None    # [3] 陀螺零偏 (rad/s)
    accel_bias: Optional[np.ndarray] = None   # [3] 加计零偏 (m/s²)
    imu_angle: Optional[np.ndarray] = None    # [2] IMU安装角 (pitch, yaw) (rad)
    imu_leverarm: Optional[np.ndarray] = None # [3] IMU杆臂 (m)
    gnss_leverarm: Optional[np.ndarray] = None # [3] GNSS天线杆臂 (m, 可选)
```

> 协方差字段为 `covariance_p`（单滤波协方差 P，E 系，维度 = StateIndex.dim，15~24 维），对应单滤波架构的统一协方差矩阵。

---

## 5. TraceFileWriter — 运行轨迹/调试输出

> **实际实现**：`src/log/trace_file_writer.py::TraceFileWriter`，生成 `.trace` 文件，含 GPS week+sow 时间戳，过滤无效调试行（如 `pos=[0. 0. 0.]`、`x[clk]=N/A`、`clk_stored=[0. 0. 0.]`）。

### 5.1 类签名

```python
class TraceFileWriter:
    """运行轨迹/调试输出器（src/log/trace_file_writer.py），按 trace_level 控制输出详细程度

    生成 .trace 文件，时间戳为 GPS week+sow。
    过滤无效调试行：pos=[0. 0. 0.]、x[clk]=N/A、clk_stored=[0. 0. 0.] 等。
    """

    def __init__(self, output_dir: str, trace_level: int = 1) -> None: ...

    # ── WriterBase 接口实现 ──
    def open(self) -> None: ...
    def write(self, trace: TraceData) -> None: ...
    def close(self) -> None: ...

    # ── 分级输出方法 ──
    def write_state_change(self, trace: TraceData) -> None: ...
    def write_gnss_update(self, trace: TraceData) -> None: ...
    def write_nhc_update(self, trace: TraceData) -> None: ...
    def write_zupt_update(self, trace: TraceData) -> None: ...
    def write_ekf_predict(self, trace: TraceData) -> None: ...
```

### 5.2 方法职责与 trace_level 对应关系

| 方法 | 最低 trace_level | 输出内容 |
|------|-----------------|---------|
| `write_state_change()` | 1 | 状态变化：`from → to` |
| `write_gnss_update()` | 2 | GNSS 更新（作用于 P）：模式、卫星数、残差范数 |
| `write_nhc_update()` | 2 | NHC 更新（作用于 P）：横向/垂向速度约束残差 |
| `write_zupt_update()` | 2 | ZUPT 更新（3D，作用于 P，与 NHC 互斥）：零速约束残差 |
| `write_ekf_predict()` | 3 | 单滤波预测：dt、trace(P) |

`write(trace)` 方法根据 `trace.event` 和 `self.trace_level` 分发到对应方法，`trace_level == 0` 时直接返回。

### 5.3 TraceData 数据结构

```python
@dataclass
class TraceData:
    """轨迹/调试数据（单滤波事件）"""
    timestamp: float
    event: str    # 事件类型标识
    data: dict    # 事件数据字典

    # 事件类型:
    # "ekf_predict"    — 单滤波 EKF 预测
    # "gnss_update"    — GNSS 量测更新 (作用于 P)
    # "nhc_update"     — NHC 量测更新 (作用于 P)
    # "zupt_update"    — ZUPT 零速更新 (3D, 作用于 P, 与 NHC 互斥)
    # "feedback"       — 单滤波反馈
    # "time_align"     — 时间对齐 (增量切分, 4 种情况)
    # "init_state"     — 初始化状态变化
    # "outlier"        — 异常检测
```

---

## 6. RawDataWriter — 原始数据记录

### 6.1 类签名

```python
class RawDataWriter(WriterBase):
    """原始数据记录器，将传感器原始数据写入 CSV 文件"""

    def __init__(self, output_dir: str, enabled: bool = False) -> None: ...

    # ── WriterBase 接口实现 ──
    def open(self) -> None: ...    # 创建 raw/ 目录及各数据源 CSV 文件
    def write(self, sensor_data: SensorData) -> None: ...
    def close(self) -> None: ...   # 关闭所有文件句柄
```

### 6.2 输出文件

| 文件 | 内容 | CSV 列 |
|------|------|--------|
| `raw/imu_raw.csv` | IMU 原始数据 | timestamp, ax, ay, az, gx, gy, gz |
| `raw/rover_raw.csv` | 流动站原始数据 | timestamp, sat_id, pseudorange, phaserange, doppler, snr, freq |
| `raw/eph_raw.csv` | 星历原始数据 | timestamp, sat_id, toe, system |
| `raw/ref_raw.csv` | 基站原始数据 | timestamp, sat_id, pseudorange, phaserange, doppler, snr, freq |

`enabled=False` 时，`open()` / `write()` / `close()` 均为空操作。

---

## 7. Logger 线程设计

> **实际实现**：项目当前有两种日志线程，按运行模式自动选用：
> - **`Logger`（外部模式）**：消费 `imu_queue` + `gnss_queue`，通过 `Aligner` 匹配后写 `AlignedWriter`
> - **`SolutionLogger`（内部模式）**：仅消费 `gnss_queue`，直接写 `SolutionWriter`
>
> **🚧 预留设计**：INS 启用后（`internal` + `ins.enabled: "lc"`），将扩展为消费 `solution_queue` / `trace_queue` / `raw_log_queue` 三队列的完整 Logger（见 7.5 节）。

### 7.1 Logger（外部模式 / internal+on 模式，实际实现 src/log/logger.py）

```python
class Logger(Thread):
    """日志记录器线程。

    从 imu_queue、gnss_queue 消费数据，做 GNSS 触发匹配，
    委托 AlignedWriter 输出 CSV。可选地同时输出纯 GNSS .pos 文件。
    流式结束后可选地批量运行松组合 EKF (LcRunner) 输出 RTKLC.pos。
    依赖注入: 构造函数接收 AlignedWriter 与 Aligner 实例。
    """

    def __init__(self, imu_queue: Queue, gnss_queue: Queue,
                 writer: AlignedWriter, aligner: Aligner,
                 control: ThreadControl, gnss_writer=None,
                 lc_runner=None):
        Thread.__init__(self, name="Logger", daemon=True)
        self.imu_queue = imu_queue
        self.gnss_queue = gnss_queue
        self.writer = writer           # 依赖注入 AlignedWriter
        self.aligner = aligner         # 依赖注入 Aligner
        self.control = control
        self.gnss_writer = gnss_writer  # 可选 SolutionWriter（纯 GNSS .pos）
        self.lc_runner = lc_runner      # 可选 LcRunner（松组合批处理）
        self.imu_eof = False
        self._lc_imu_log = []           # LcRunner 收集的 IMU 数据
        self._lc_gnss_log = []          # LcRunner 收集的 GNSS 数据

    def run(self):
        self.writer.open()
        if self.gnss_writer is not None:
            self.gnss_writer.open()
        try:
            while self.control.is_running():
                # 1. 先把 imu_queue 中的数据搬到 aligner.imu_buffer
                self._drain_imu_queue()
                # 2. 取一个 GNSS 历元（阻塞，超时 0.1s）
                try:
                    gnss = self.gnss_queue.get(timeout=0.1)
                except Empty:
                    continue
                # 3. 区分 EOF sentinel 与正常数据
                if gnss is None:
                    self._drain_imu_queue()  # GNSS EOF，排空 IMU 后退出
                    break
                # 4. 解包 SensorData 并匹配写出
                if isinstance(gnss, SensorData):
                    gnss = gnss.gnss_solution
                # 同时输出纯 GNSS .pos 文件（如有配置）
                if self.gnss_writer is not None:
                    self.gnss_writer.write(gnss)
                # 收集 GNSS 数据供 LcRunner 批量运行
                if self.lc_runner is not None:
                    self._lc_gnss_log.append(gnss)
                # 等待 IMU 数据读到 >= GNSS 历元 + harvest_window
                self._wait_for_imu(gnss.timestamp + self.aligner.harvest_window)
                aligned = self.aligner.harvest(gnss)
                if aligned is not None:
                    self.writer.write(aligned)
        finally:
            self.writer.close()
            if self.gnss_writer is not None:
                self.gnss_writer.close()
            if self.lc_runner is not None:
                self._run_lc()

    def _run_lc(self):
        """流式结束后批量运行松组合 EKF。"""
        self.lc_runner.run(self._lc_imu_log, self._lc_gnss_log)

    def _wait_for_imu(self, gnss_timestamp: float) -> None:
        """阻塞直到 IMU 缓冲包含 >= gnss_timestamp 的数据，或 IMU EOF。
        同时收集 IMU 数据供 LcRunner 批量运行。"""

    def _drain_imu_queue(self):
        """非阻塞地把 imu_queue 中所有数据搬到 aligner.imu_buffer。
        同时收集 IMU 数据供 LcRunner 批量运行。"""
```

### 7.2 SolutionLogger（内部模式，实际实现 src/log/solution_logger.py）

```python
class SolutionLogger(Thread):
    """纯 GNSS 模式日志线程，仅消费 gnss_queue。

    把 GnssSolution 委托 SolutionWriter 输出。
    收到 None（EOF sentinel）后关闭 writer 并退出。
    依赖注入: 构造函数接收 WriterBase 实例（通常为 SolutionWriter）。
    """

    def __init__(self, gnss_queue: Queue, writer, control: ThreadControl):
        Thread.__init__(self, name="SolutionLogger", daemon=True)
        self.gnss_queue = gnss_queue
        self.writer = writer           # 依赖注入 SolutionWriter
        self.control = control

    def run(self):
        self.writer.open()
        try:
            while self.control.is_running():
                try:
                    data = self.gnss_queue.get(timeout=0.1)
                except Empty:
                    continue
                if data is None:
                    break  # EOF sentinel
                if isinstance(data, SensorData):
                    sol = data.gnss_solution
                    if sol is not None:
                        self.writer.write(sol)
        finally:
            self.writer.close()
```

### 7.3 Logger 的选用规则

三种运行模式由 `ins.enabled` 配置项决定（`coupling_mode` 字段已移除）：

| 运行模式 | ins.enabled | 选用处理流 | 选用 Logger | 消费队列 | Writer | 输出文件 |
|---------|-------------|-----------|------------|---------|--------|---------|
| 纯 GNSS | `off` | — | `SolutionLogger` | gnss_queue | SolutionWriter | `.pos` |
| 松组合 LC | `on` | `LcStream` | `RSLTWriter` | solution_queue | RSLTWriter | `.rslt`（100Hz） |
| 紧组合 TC | `tc` | `TcStream` | `RSLTWriter` | solution_queue | RSLTWriter | `.rslt`（100Hz） |

> **输出格式配置**：`output.position_format`（`llh`/`xyz`）、`output.time_format`（`gpst`/`datetime`）、`trace_level`（0-3）由 SolutionWriter / RSLTWriter 构造函数接收。
>
> **历史模式说明**：早期版本中 `ins.enabled=on` 对应"外部对齐模式"（`Logger` + `Aligner` + `AlignedWriter`，输出 `aligned.csv`）与"内部对齐 + 松组合批处理"（`Logger` + `gnss_writer` + `lc_runner`，输出 `.pos` + `aligned.csv` + `RTKLC.pos`）。当前已升级为流式 `LcStream` + `RSLTWriter`（100Hz `.rslt`）。

### 7.4 Aligner — IMU 积攒 + GNSS 收割匹配器（实际实现 src/log/aligner.py）

```python
class Aligner:
    """IMU 积攒 + GNSS 收割的匹配器（外部模式专用）。"""

    def __init__(self, imu_dt: float, harvest_window: float = 1.0):
        self.imu_dt = float(imu_dt)
        self.harvest_window = float(harvest_window)  # 收割窗口（秒）
        self.imu_buffer: deque = deque()

    def push_imu(self, imu: ImuMeasurement) -> None:
        """将一条 IMU 数据推入缓冲（timestamp 为 Unix 时间戳）。"""

    def harvest(self, gnss: GnssSolution) -> Optional[AlignedBlock]:
        """对一个 GNSS 历元做收割。

        丢弃 timestamp < t_gnss 的首部 IMU，收割
        t_gnss <= timestamp < t_gnss + harvest_window 的 IMU。
        Returns: AlignedBlock 或 None（收割列表为空）
        """
```

> 时间戳匹配基于 Unix 时间戳：`t_lo = gnss.timestamp`，`t_hi = gnss.timestamp + harvest_window`。
> `harvest_window` 默认 1.0 秒，确保每个 GNSS 历元收割到一个完整的 IMU 块。

### 7.5 完整 Logger 设计（🚧 预留：INS 启用后）

INS 启用后（`internal` + `ins.enabled: "lc"`），Logger 将扩展为消费三队列的完整设计：

```python
class Logger:  # 🚧 预留
    """完整日志记录器线程（INS 启用后）

    从 solution_queue、trace_queue、raw_log_queue 消费数据，
    委托给对应的 WriterBase 实例处理。
    """

    def __init__(self, solution_queue: Queue, trace_queue: Queue,
                 raw_log_queue: Queue, control, options) -> None: ...

    def run(self) -> None: ...
    def _drain_solution_queue(self) -> None: ...
    def _drain_trace_queue(self) -> None: ...
    def _drain_raw_log_queue(self) -> None: ...
    def _print_summary(self, sol: Solution) -> None: ...
```

| 属性 | 类型 | 说明 |
|------|------|------|
| `self.solution_writer` | `SolutionWriter` | 解算结果输出 |
| `self.trace_writer` | `TraceFileWriter` | 轨迹/调试输出 |
| `self.raw_data_writer` | `RawDataWriter` | 原始数据输出 |

主循环流程：
```
run() 主循环:
  1. 调用三个 Writer 的 open()
  2. while control.is_running():
       a. 优先 _drain_solution_queue() → solution_writer.write()
       b. 其次 _drain_trace_queue()    → trace_writer.write()
       c. 最后 _drain_raw_log_queue()  → raw_data_writer.write()
       d. 三队列均空时 sleep 避免空转
  3. 退出前刷新三个队列的剩余数据
  4. 调用三个 Writer 的 close()
```

### 7.6 LcIntegration 端 Trace 数据生成（🚧 预留）

`LcIntegration` 估计线程在单滤波 EKF 预测/量测更新/反馈后将 `TraceData` 通过 `trace_queue.put()` 放入队列，队列满时静默丢弃。Trace 事件记录单滤波的状态（P 矩阵迹、H 残差、时间对齐情况等）。

---

## 8. 输出格式定义

### 8.1 输出文件清单

| 文件 | 格式 | 内容 | 必须 |
|------|------|------|------|
| `solution.pos` | POS | 解算结果（位置、精度） | 是 |
| `solution.csv` | CSV | 解算结果（完整状态） | 可选 |
| `solution.nmea` | NMEA | NMEA 格式（兼容外部工具） | 可选 |
| `trace.txt` | 文本 | 运行轨迹/调试信息 | 可选 |
| `raw/imu_raw.csv` | CSV | IMU 原始数据 | 可选 |
| `raw/rover_raw.csv` | CSV | 流动站原始数据（内部模式） | 可选 |
| `raw/eph_raw.csv` | CSV | 星历原始数据（内部模式） | 可选 |
| `raw/ref_raw.csv` | CSV | 基站原始数据（内部模式） | 可选 |

### 8.1b 外部 GNSS 结果输入格式

外部模式下（`gnss_source: "external"`），`src/stream/gnss_sol_sensor.py::GnssSolSensor` 通过 `src/stream/formators.py::PosSolFormator` 读取外部 GNSS 定位结果文件：

| 格式 | 文件扩展名 | 解析 Formator | 实现状态 |
|------|-----------|--------------|---------|
| POS | `.pos` | `PosSolFormator` | ✅ 已实现（参考 rtklib 输出格式，含 ECEF + 精度） |
| NMEA | `.nmea` | `NmeaSolFormator` | 🚧 预留 |
| CSV | `.csv` | `CsvSolFormator` | 🚧 预留 |

> 注意：当前仅支持 POS 格式（参考 `src/utility/config_loader.py::SUPPORTED_EXTERNAL_FORMATS = {"pos"}`）。
> 输入 POS 文件时间字段（`yyyy/mm/dd hh:mm:ss.s`）通过 `ymdhms_to_gpst()` → `gpst_to_unix()` 转为 Unix 时间戳。
> 本程序内部模式输出的 `.pos` 文件可作为外部模式的输入，便于级联处理。

### 8.2 POS 格式（默认，参考 rtklib）

`SolutionWriter` 输出的 `.pos` 文件格式（参考 rtklib-py `postpos.savesol`）：

```
%  GPST          latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio
 2069   86400.000   30.123456789  120.123456789   50.1234   5  12   0.5000   0.5000   1.0000   0.1000   0.1000   0.2000   0.00    0.0
```

字段说明：

| 字段 | 格式 | 说明 |
|------|------|------|
| week | `%4d` | GPS 周号（由 `unix_to_gpst(sol.timestamp)` 得到） |
| sow | `%10.3f` | GPS 周内秒（同上） |
| lat | `%14.9f` | 纬度（度），ECEF→LLH 转换 |
| lon | `%14.9f` | 经度（度） |
| height | `%10.4f` | 高度（m） |
| Q | `%3d` | 解算质量: 1=SPP, 2=RTD, 4=浮点解, 5=LC（与 rtklib `Sol.stat` 一致） |
| ns | `%3d` | 使用卫星数 |
| sdn/sde/sdu | `%8.4f` | 北/东/天位置标准差（ENU 协方差对角线平方根） |
| sdne/sdeu/sdun | `%8.4f` | 位置标准差交叉项（ENU 协方差非对角项，带符号） |
| age | `%6.2f` | 差分龄期（本项目填 0.00） |
| ratio | `%6.1f` | 模糊度比率（本项目填 0.0） |

> 时间字段输出 `(week, sow)` 对齐 rtklib-py `savesol`，但内部存储为 Unix 时间戳。
> ENU 协方差顺序：`sdn=N, sde=E, sdu=U, sdne=EN, sdeu=UE, sdun=NU`（参考 `src/log/solution_writer.py`）。

### 8.3 CSV 格式

```
timestamp,pos_e,pos_n,pos_u,vel_e,vel_n,vel_u,roll,pitch,yaw,status,num_sat,pdop
```

### 8.4 NMEA 格式（可选）

```
$GPGGA,hhmmss.ss,llll.ll,a,yyyyy.yy,a,x,xx,x.x,x.x,M,x.x,M,x.x,xxxx*hh
$GPRMC,hhmmss.ss,A,llll.ll,a,yyyyy.yy,a,x.x,x.x,xxxxxx,x.x,a*hh
```

### 8.5 关键日志事件

| 事件 | 级别 | 模块 | 消息格式 |
|------|------|------|---------|
| 线程启动 | INFO | main | "线程 {name} 已启动" |
| 数据源开始 | INFO | stream | "开始读取: {file_path}" |
| 数据源结束 | INFO | stream | "数据源结束: {tag}" |
| 初始化状态变化 | INFO | scheduler | "状态变化: {from} → {to}" |
| SPP 解算成功 | DEBUG | gnss | "SPP: nsat={n}, PDOP={dop}" |
| RTD 解算成功 | DEBUG | gnss | "RTD: nsat={n}, baseline={bl}" |
| EKF 初始化 | INFO | estimator | "EKF 初始化完成" |
| 异常检测 | WARNING | estimator | "检测到异常: {type}" |
| 队列溢出 | WARNING | stream | "队列 {tag} 已满" |
| 全局退出 | INFO | main | "所有线程已退出" |

---

## 9. 与 StreamDesign.md 的衔接

### 9.1 StreamDesign.md 中已定义的日志相关内容

StreamDesign.md 第8节定义了 Logger 的基本框架：

```python
# StreamDesign.md 中的 Logger 定义
class Logger:
    def __init__(self, solution_queue, raw_log_queue, control, output_dir): ...
    def run(self): ...
    def _write_solution(self, sol: Solution): ...
```

### 9.2 本方案的扩展

| StreamDesign.md | 本方案 | 扩展内容 |
|----------------|--------|---------|
| 单一 `_write_solution()` | `SolutionWriter(WriterBase)` | 多格式输出（POS/CSV/NMEA），ABC 继承体系 |
| 无轨迹输出 | `TraceFileWriter` | 分级调试输出 |
| 简单原始数据记录 | `RawDataWriter(WriterBase)` | CSV 格式，按数据源分文件 |
| 无终端输出 | `Logger._print_summary()` | 定期终端摘要 |
| 无运行日志 | Python `logging` | 统一运行日志管理 |

### 9.3 队列衔接

StreamDesign.md 定义的队列（纯队列流水线，无观察者回调）：

| 队列 | 生产者 | 本方案使用方式 |
|------|--------|--------------|
| `solution_queue` | `LcIntegration` 估计线程 | Logger → SolutionWriter → 文件 |
| `raw_log_queue` | 各 Streamer | Logger → RawDataWriter → 文件 |

本方案新增的队列：

| 队列 | 生产者 | 说明 |
|------|--------|------|
| `trace_queue` | `LcIntegration` 估计线程 | Logger → TraceWriter → 文件（单滤波事件） |

### 9.4 配置衔接

StreamDesign.md 第11节定义了日志配置：

```yaml
logging:
  output_dir: "output"
  log_solution: true
  log_raw_data: false
  log_level: "INFO"
```

本方案扩展为：

```yaml
logging:
  output_dir: "output"
  solution_format: "pos"          # "pos" / "csv" / "nmea"
  trace_level: 1                  # 0=无, 1=基本, 2=详细, 3=调试
  log_raw_data: false
  log_level: "INFO"               # DEBUG/INFO/WARNING/ERROR
  terminal_summary_interval: 10   # 终端摘要间隔（每N条解算结果）
```

---

## 10. 设计模式集成

### 10.1 设计模式总览

本模块作为框架的输出层，集成依赖注入与 OOP 三大特性，与 GInsStream.md / StreamDesign.md 的**纯队列流水线**整体架构保持一致：

| 设计模式 | 实现 | 作用 |
|---------|------|------|
| **依赖注入** | `Logger` 构造函数接收 `WriterBase` 实例 | 解耦 Logger 与具体输出格式，便于替换/测试 |
| **OOP-封装** | 输出逻辑封装在 `WriterBase` 子类内部 | Logger 仅调用 `write()`，不关心格式细节 |
| **OOP-继承** | `WriterBase(ABC)` → SolutionWriter/TraceFileWriter/RawDataWriter | 统一 open/write/close 生命周期接口 |
| **OOP-多态** | Logger 持有 `WriterBase` 抽象引用 | 运行时调用具体子类的 write() 方法 |
| **纯队列流水线** | `LcIntegration`(生产者) → 队列 → Logger(消费者) | 全程 `queue.put()`/`queue.get()`，无观察者回调，解耦生产与消费 |

### 10.2 依赖注入 — Logger 构造函数

`Logger` / `SolutionLogger` 通过构造函数接收 `WriterBase` 实例（及 `Aligner`），而非内部 new，便于替换输出格式与测试：

```python
# ✅ 实际实现：Logger（src/log/logger.py，支持 external+on / internal+on 模式）
class Logger(Thread):
    """日志记录器线程（依赖注入）

    从 imu_queue、gnss_queue 消费数据，做 GNSS 触发匹配，
    委托注入的 AlignedWriter 与 Aligner 处理。
    internal+on 模式下额外注入 gnss_writer（纯 GNSS 输出）和 lc_runner（松组合批处理）。
    """
    def __init__(self, imu_queue: Queue, gnss_queue: Queue,
                 writer: AlignedWriter,   # 依赖注入 Writer
                 aligner: Aligner,        # 依赖注入 Aligner
                 control: ThreadControl,
                 gnss_writer=None,        # 可选依赖注入 SolutionWriter（纯 GNSS）
                 lc_runner=None):         # 可选依赖注入 LcRunner（松组合）
        # 持有 WriterBase 抽象引用（多态），不关心具体子类
        ...


# ✅ 实际实现：内部模式 SolutionLogger（src/log/solution_logger.py）
class SolutionLogger(Thread):
    """纯 GNSS 模式日志线程（依赖注入）

    从 gnss_queue 消费数据，委托注入的 WriterBase 实例处理。
    """
    def __init__(self, gnss_queue: Queue, writer,  # 依赖注入 Writer
                 control: ThreadControl):
        # 持有 WriterBase 抽象引用（多态），不关心具体子类
        ...


# 🚧 预留：INS 启用后的完整 Logger
class Logger:  # 预留
    """完整日志记录器线程（INS 启用后，依赖注入）

    从 solution_queue、trace_queue、raw_log_queue 消费数据，
    委托给注入的 WriterBase 实例处理。
    """
    def __init__(self, solution_queue: Queue, trace_queue: Queue,
                 raw_log_queue: Queue, control, options,
                 solution_writer: WriterBase,     # 依赖注入
                 trace_writer: WriterBase,        # 依赖注入
                 raw_data_writer: WriterBase):    # 依赖注入
        ...
```

### 10.3 依赖注入装配示例

装配过程集中在入口处（`src/main.py`），Logger/SolutionLogger 自身不 new 任何 Writer：

```python
# ✅ 实际装配：内部模式（src/main.py 节选）
def build_internal_mode(config, gnss_queue, control):
    """装配内部 GNSS 解算模式：SolutionLogger + SolutionWriter"""
    writer = SolutionWriter(
        output_dir=config["logging"]["output_dir"],
        filename="solution.pos",
    )
    return SolutionLogger(
        gnss_queue=gnss_queue,
        writer=writer,           # 依赖注入
        control=control,
    )


# ✅ 实际装配：外部模式（src/main.py 节选）
def build_external_mode(config, imu_queue, gnss_queue, control):
    """装配外部结果对齐模式：Logger + AlignedWriter + Aligner"""
    writer = AlignedWriter(
        output_dir=config["logging"]["output_dir"],
        filename="aligned.csv",
    )
    aligner = Aligner(
        imu_dt=1.0 / config["imu"]["rate"],
        harvest_window=config["align"]["harvest_window"],
    )
    return Logger(
        imu_queue=imu_queue,
        gnss_queue=gnss_queue,
        writer=writer,           # 依赖注入
        aligner=aligner,         # 依赖注入
        control=control,
    )


# ✅ 实际装配：内部对齐 + 松组合模式（src/main.py 路径 C 节选）
def build_internal_lc_mode(config, imu_queue, gnss_queue, control):
    """装配内部 GNSS + IMU 对齐 + 松组合 EKF 模式。
    输出三个文件：纯 GNSS .pos + 对齐 CSV + 松组合 .pos
    """
    # 1. 对齐块状 CSV 输出
    aligned_writer = AlignedWriter(
        output_dir=config["output"]["output_dir"],
        filename=config["output"].get("aligned_filename", "aligned.csv"),
    )
    # 2. 纯 GNSS 定位结果输出（文件名由 gnss_solution_filename 配置）
    gnss_writer = SolutionWriter(
        output_dir=config["output"]["output_dir"],
        filename=config["output"].get("gnss_solution_filename", "gnss_solution.pos"),
    )
    # 3. 松组合定位结果输出（文件名由 solution_filename 配置）
    lc_writer = SolutionWriter(
        output_dir=config["output"]["output_dir"],
        filename=config["output"].get("solution_filename", "RTKLC.pos"),
    )
    # 4. LcRunner（松组合批处理运行器）
    from src.core.ins.lc_runner import LcRunner
    lc_runner = LcRunner(config, lc_writer)
    aligner = Aligner(imu_dt=1.0 / config["ins"]["data_rate"])
    return Logger(
        imu_queue=imu_queue,
        gnss_queue=gnss_queue,
        writer=aligned_writer,    # 依赖注入 AlignedWriter
        aligner=aligner,          # 依赖注入 Aligner
        control=control,
        gnss_writer=gnss_writer,  # 依赖注入 SolutionWriter（纯 GNSS）
        lc_runner=lc_runner,      # 依赖注入 LcRunner（松组合）
    )


# 🚧 预留：INS 启用后的完整装配
def build_logger(config: dict, queues: dict, control) -> Logger:
    """装配完整日志记录器（依赖注入，INS 启用后）"""
    output_dir = config["logging"]["output_dir"]
    solution_writer = SolutionWriter(output_dir, format="pos")
    trace_writer = TraceFileWriter(output_dir, trace_level=1)
    raw_data_writer = RawDataWriter(output_dir, enabled=False)
    return Logger(
        solution_queue=queues["solution"],
        trace_queue=queues["trace"],
        raw_log_queue=queues["raw_log"],
        control=control, options=config,
        solution_writer=solution_writer,
        trace_writer=trace_writer,
        raw_data_writer=raw_data_writer,
    )
```

### 10.4 纯队列流水线衔接

本模块的 `Logger` / `SolutionLogger` 与传感器层**统一采用纯队列流水线**，全程通过 `queue.put()`/`queue.get()` 传递数据，**无观察者模式、无 notify() 回调**：

```
【当前实现：内部纯 GNSS 模式（internal + ins.enabled=off，路径 B）】
  InternalGnssSensor.run()
    → 逐历元 pntpos/relpos → GnssSolution
    → gnss_queue.put(SensorData(tag="gnss_solution"))   (含 None EOF sentinel)
        ↓
  SolutionLogger.run()                                  (queue.get() 消费)
    → SolutionWriter.write(GnssSolution)                (Unix → week/sow 输出 .pos)

【当前实现：外部对齐模式（external + ins.enabled=on，路径 A）】
  ImuSensor.run()
    → ImuFormator.decode(line) → ImuMeasurement         (gpst_to_unix 时间戳)
    → imu_queue.put(SensorData(tag="imu"))
        ↓                                                ↓
  GnssSolSensor.run()                                   Logger.run()
    → PosSolFormator.decode(line) → GnssSolution         → _drain_imu_queue() → Aligner.push_imu()
    → gnss_queue.put(SensorData(tag="gnss_solution"))    → gnss_queue.get() → Aligner.harvest(gnss)
        ↓                                                → AlignedWriter.write(AlignedBlock)
  (None EOF sentinel 触发 Logger 退出)

【当前实现：内部对齐 + 松组合模式（internal + ins.enabled=lc，路径 C）】
  ImuSensor.run()
    → ImuFormator.decode(line) → ImuMeasurement         (gpst_to_unix 时间戳)
    → imu_queue.put(SensorData(tag="imu"))
        ↓                                                ↓
  InternalGnssSensor.run()                              Logger.run()
    → 逐历元 pntpos/relpos → GnssSolution               → _drain_imu_queue() → Aligner.push_imu()
    → gnss_queue.put(SensorData(tag="gnss_solution"))    → gnss_queue.get()
        ↓                                                   ├─ gnss_writer.write(gnss)           → RTK.pos (纯 GNSS)
                                                            ├─ _lc_gnss_log.append(gnss)          (收集供 LcRunner)
                                                            ├─ _wait_for_imu() + _lc_imu_log.append (收集 IMU)
                                                            ├─ Aligner.harvest(gnss) → AlignedWriter.write → aligned_internal_rtk.csv
                                                            └─ (循环)
  (None EOF sentinel 触发 Logger 退出)
  finally:
    gnss_writer.close()
    Logger._run_lc():
      LcRunner.run(_lc_imu_log, _lc_gnss_log)
        → InsInitializer.initialize() → LcEstimator + LcIntegration
        → 整数秒输出 InsState+P → GnssSolution → lc_writer.write() → RTKLC.pos (松组合)

【🚧 预留：INS 启用后的完整流水线（internal + ins.enabled=lc，路径 D）】
  ImuSensor/GnssRoverSensor ──put()──→ imu_queue/sensor_queue
                                                ↓
                                     Scheduler（仅转发）          🚧 预留
                                                ↓
                                       estimate_queue
                                                ↓
  LcIntegration ──get()──→ estimate_queue
       │   时间对齐 + 单滤波 EKF (P)
       │   反馈 P
       ↓
  LcIntegration ──put()──→ solution_queue ──get()──→ Logger ──→ SolutionWriter
                         trace_queue                          ──→ TraceWriter
  Stream ──put()──→ raw_log_queue                          ──→ RawDataWriter
```

数据通路统一性：
- **传感器 → 输出（当前）**：`InternalGnssSensor` / `ImuSensor` / `GnssSolSensor` 通过 `output_queue.put()` 推入 `gnss_queue` / `imu_queue`，Logger/SolutionLogger 通过 `queue.get()` 消费，**无 Scheduler 中转**
- **传感器 → 估计 → 输出（预留）**：`LcIntegration` 通过 `solution_queue.put()` / `trace_queue.put()` 推入单滤波解算结果与事件
- **全程无回调**：Logger 通过 `queue.get()` 消费，缓冲削峰，异步落盘，避免 IO 阻塞融合

### 10.5 OOP 三大特性体现

| 特性 | 体现 |
|------|------|
| **封装** | 各 Writer 的格式化细节（POS/CSV/NMEA）封装在子类内部，Logger 仅调用 `write()` |
| **继承** | `WriterBase(ABC)` → `SolutionWriter` / `TraceFileWriter` / `RawDataWriter`，统一 open/write/close 接口 |
| **多态** | Logger 持有三个 `WriterBase` 抽象引用，运行时调用具体子类的 `write()` 方法 |
