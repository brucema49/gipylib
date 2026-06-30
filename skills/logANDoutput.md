# 日志流和输出结果流方案

> 规划日志记录和输出结果的流式设计方案，与 StreamDesign.md 中的多线程架构衔接。
>
> **时间系统约定**：全框架统一使用 GPS 秒（GPST，since 1980-01-06），输出文件时间戳（POS/NMEA/CSV）均使用 GPST。
>
> **框架设计模式集成**：
> - **纯队列流水线**：`Logger` 作为 `LcIntegration` 估计线程的下游消费者，从 `solution_queue` / `trace_queue` / `raw_log_queue` 取数据（`queue.get()`），**无观察者回调、无 notify()**，与传感器层统一为纯队列流水线
> - **依赖注入**：`Logger` 通过构造函数接收 `WriterBase` 实例（SolutionWriter/TraceWriter/RawDataWriter），而非内部 new，便于替换输出格式
> - **OOP 三大特性**：封装（输出逻辑封装在 Writer 子类）、继承（`WriterBase(ABC)` → SolutionWriter/TraceWriter/RawDataWriter）、多态（Logger 持有 WriterBase 抽象引用）

---

## 目录

- [1. 概述](#1-概述)
- [2. 日志类型与架构](#2-日志类型与架构)
- [3. Writer 类继承体系](#3-writer-类继承体系)
- [4. SolutionWriter — 解算结果输出](#4-solutionwriter--解算结果输出)
- [5. TraceWriter — 运行轨迹/调试输出](#5-tracewriter--运行轨迹调试输出)
- [6. RawDataWriter — 原始数据记录](#6-rawdatawriter--原始数据记录)
- [7. Logger 线程设计](#7-logger-线程设计)
- [8. 输出格式定义](#8-输出格式定义)
- [9. 与 StreamDesign.md 的衔接](#9-与-streamdesignmd-的衔接)
- [10. 设计模式集成](#10-设计模式集成)

---

## 1. 概述

### 1.1 模块文件

| 文件 | 职责 |
|------|------|
| `writer_base.py` | 输出器抽象基类 `WriterBase(ABC)` |
| `solution_writer.py` | 解算结果输出 `SolutionWriter(WriterBase)` |
| `trace_writer.py` | 运行轨迹/调试输出 `TraceWriter(WriterBase)` |
| `raw_data_writer.py` | 原始数据记录 `RawDataWriter(WriterBase)` |
| `logger.py` | 日志记录器主线程 `Logger` |

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

> 说明：`LcIntegration` 作为估计线程，将双滤波（P1 主滤波 + P2 NHC 子滤波）解算结果 `Solution` 通过 `solution_queue.put()` 推入队列；`Logger` 通过 `solution_queue.get()` 消费，**全程无观察者回调**。`trace_queue` 同理由 `LcIntegration` 生产，用于记录双滤波预测/更新事件。

### 2.3 队列定义

| 队列 | 生产者 | 消费者 | 容量 | 数据类型 | 说明 |
|------|--------|--------|------|---------|------|
| `solution_queue` | `LcIntegration` 估计线程 | Logger 线程 | 200 | `Solution` | 双滤波融合解（P1+P2 反馈后输出） |
| `trace_queue` | `LcIntegration` 估计线程 | Logger 线程 | 500 | `TraceData` | 双滤波预测/更新事件（P1/P2 独立） |
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

### 4.1 类签名

```python
class SolutionWriter(WriterBase):
    """解算结果输出器，支持 POS / CSV / NMEA 格式"""

    def __init__(self, output_dir: str, format: str = "pos",
                 trace_level: int = 1) -> None: ...

    # ── WriterBase 接口实现 ──
    def open(self) -> None: ...
    def write(self, sol: Solution) -> None: ...
    def close(self) -> None: ...

    # ── 格式化输出方法 ──
    def write_header(self) -> None: ...
    def write_pos(self, sol: Solution) -> None: ...
    def write_csv(self, sol: Solution) -> None: ...
    def write_nmea(self, sol: Solution) -> None: ...
```

### 4.2 方法职责

| 方法 | 职责 |
|------|------|
| `open()` | 根据格式创建输出文件，调用 `write_header()` |
| `write(sol)` | 根据 `self.format` 分发到 `write_pos()` / `write_csv()` / `write_nmea()` |
| `close()` | 关闭文件句柄 |
| `write_header()` | 写入文件头（POS 注释行 / CSV 列名 / NMEA 无头） |
| `write_pos(sol)` | POS 格式：时间、LLH、质量标记、卫星数、标准差 |
| `write_csv(sol)` | CSV 格式：时间戳、ENU 位置/速度、姿态角、状态、卫星数、PDOP |
| `write_nmea(sol)` | NMEA 格式：GGA + RMC 语句 |

### 4.3 Solution 数据结构

```python
@dataclass
class Solution:
    """解算结果（双滤波反馈后输出）"""
    timestamp: float                          # 时间戳 (s, GPST)
    position_enu: Optional[np.ndarray] = None # [3] ENU 位置 (m)
    velocity_enu: Optional[np.ndarray] = None # [3] ENU 速度 (m/s)
    attitude: Optional[np.ndarray] = None     # [3] 横滚/俯仰/航向 (rad)
    covariance_p1: Optional[np.ndarray] = None # [15,15] 或 [18,18] 主滤波协方差 P1 (E 系)
    covariance_p2: Optional[np.ndarray] = None # [5,5] NHC 子滤波协方差 P2 (v 系)
    status: str = "None"                      # None/SPP/RTD/RTK/LC/External
    num_satellites: int = 0
    pdop: float = 0.0
    gyro_bias: Optional[np.ndarray] = None    # [3] 陀螺零偏 (rad/s)
    accel_bias: Optional[np.ndarray] = None   # [3] 加计零偏 (m/s²)
    imu_angle: Optional[np.ndarray] = None    # [2] IMU安装角 (pitch, yaw) (rad)
    imu_leverarm: Optional[np.ndarray] = None # [3] IMU杆臂 (m)
    gnss_leverarm: Optional[np.ndarray] = None # [3] GNSS天线杆臂 (m, 可选)
```

> 协方差字段拆分为 `covariance_p1`（主滤波 P1，E 系，15/18 维）和 `covariance_p2`（NHC 子滤波 P2，v 系，5 维），对应双滤波架构的两套独立协方差矩阵。输出时 P2 为可选（仅启用 NHC 时存在）。

---

## 5. TraceWriter — 运行轨迹/调试输出

### 5.1 类签名

```python
class TraceWriter(WriterBase):
    """运行轨迹/调试输出器，按 trace_level 控制输出详细程度"""

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
| `write_gnss_update()` | 2 | GNSS 更新（作用于 P1）：模式、卫星数、残差范数 |
| `write_nhc_update()` | 2 | NHC 更新（H1 作用于 P1 + H2 作用于 P2）：横向/垂向速度约束残差 |
| `write_zupt_update()` | 2 | ZUPT 更新（3D，作用于 P1，与 NHC 互斥）：零速约束残差 |
| `write_ekf_predict()` | 3 | 双滤波预测：dt、trace(P1)、trace(P2) |

`write(trace)` 方法根据 `trace.event` 和 `self.trace_level` 分发到对应方法，`trace_level == 0` 时直接返回。

### 5.3 TraceData 数据结构

```python
@dataclass
class TraceData:
    """轨迹/调试数据（双滤波事件）"""
    timestamp: float
    event: str    # 事件类型标识
    data: dict    # 事件数据字典

    # 事件类型:
    # "ekf_predict"    — 双滤波 EKF 预测 (P1 + P2 独立)
    # "gnss_update"    — GNSS 量测更新 (仅作用于 P1)
    # "nhc_update"     — NHC 量测更新 (H1 作用于 P1 + H2 作用于 P2)
    # "zupt_update"    — ZUPT 零速更新 (3D, 仅作用于 P1, 与 NHC 互斥)
    # "feedback"       — 双滤波独立反馈 (P1 反馈 + P2 反馈)
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

### 7.1 类签名

```python
class Logger:
    """日志记录器线程

    从 solution_queue、trace_queue、raw_log_queue 消费数据，
    委托给对应的 WriterBase 实例处理。
    """

    def __init__(self, solution_queue: Queue, trace_queue: Queue,
                 raw_log_queue: Queue, control, options) -> None: ...

    # ── 线程入口 ──
    def run(self) -> None: ...

    # ── 队列消费 ──
    def _drain_solution_queue(self) -> None: ...
    def _drain_trace_queue(self) -> None: ...
    def _drain_raw_log_queue(self) -> None: ...

    # ── 终端输出 ──
    def _print_summary(self, sol: Solution) -> None: ...
```

### 7.2 Logger 持有的 Writer 实例

| 属性 | 类型 | 说明 |
|------|------|------|
| `self.solution_writer` | `SolutionWriter` | 解算结果输出 |
| `self.trace_writer` | `TraceWriter` | 轨迹/调试输出 |
| `self.raw_data_writer` | `RawDataWriter` | 原始数据输出 |

### 7.3 主循环流程

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

### 7.4 LcIntegration 端 Trace 数据生成

`LcIntegration` 估计线程在双滤波 EKF 预测/量测更新/反馈后将 `TraceData` 通过 `trace_queue.put()` 放入队列，队列满时静默丢弃。Trace 事件记录双滤波独立的状态（P1/P2 矩阵迹、H1/H2 残差、时间对齐情况等）。

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

外部模式下（`gnss_source: "external"`），GnssSolStreamer 读取外部 GNSS 定位结果文件：

| 格式 | 文件扩展名 | 解析 Formator | 说明 |
|------|-----------|--------------|------|
| POS | `.pos` | `PosSolFormator` | 参考 rtklib 输出格式，含 LLH + 精度 |
| NMEA | `.nmea` | `NmeaSolFormator` | 标准 NMEA-0183，$GPGGA/$GPRMC |
| CSV | `.csv` | `CsvSolFormator` | 自定义列格式，需配置列映射 |

> 注意：POS/NMEA 格式既是输出格式也是输入格式，外部模式下读取的文件格式与输出格式一致，
> 便于本程序输出的结果文件作为另一个实例的外部输入。

### 8.2 POS 格式（默认，参考 rtklib）

```
%  GPST   lat(deg)    lon(deg)     height(m)  Q  ns  sdn(m)  sde(m)  sdu(m)  sdne(m)  sden(m) sdun(m) age(s)  ratio
2023/01/15 08:00:00.0  30.12345678  120.12345678  50.123  5  12  0.5  0.5  1.0  0.1  0.1  0.2  0.0  0.0
```

字段说明：

| 字段 | 格式 | 说明 |
|------|------|------|
| GPST | yyyy/mm/dd hh:mm:ss.s | GPS 时间 |
| lat | deg | 纬度 |
| lon | deg | 经度 |
| height | m | 高度 |
| Q | int | 解算质量: 1=SPP, 2=RTD, 5=LC |
| ns | int | 使用卫星数 |
| sdn/sde/sdu | m | 北/东/天位置标准差 |
| sdne/sden/sdun | m | 位置标准差交叉项 |
| age | s | 差分龄期 |
| ratio | - | 模糊度比率（RTK用） |

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
| 无轨迹输出 | `TraceWriter(WriterBase)` | 分级调试输出 |
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
| `trace_queue` | `LcIntegration` 估计线程 | Logger → TraceWriter → 文件（双滤波事件） |

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
| **OOP-继承** | `WriterBase(ABC)` → SolutionWriter/TraceWriter/RawDataWriter | 统一 open/write/close 生命周期接口 |
| **OOP-多态** | Logger 持有 `WriterBase` 抽象引用 | 运行时调用具体子类的 write() 方法 |
| **纯队列流水线** | `LcIntegration`(生产者) → 队列 → Logger(消费者) | 全程 `queue.put()`/`queue.get()`，无观察者回调，解耦生产与消费 |

### 10.2 依赖注入 — Logger 构造函数

`Logger` 通过构造函数接收 `WriterBase` 实例，而非内部 new，便于替换输出格式与测试：

```python
class Logger:
    """日志记录器线程（依赖注入）

    从 solution_queue、trace_queue、raw_log_queue 消费数据，
    委托给注入的 WriterBase 实例处理。
    """

    def __init__(self, solution_queue: Queue, trace_queue: Queue,
                 raw_log_queue: Queue, control, options,
                 solution_writer: WriterBase,     # 依赖注入
                 trace_writer: WriterBase,        # 依赖注入
                 raw_data_writer: WriterBase):    # 依赖注入
        self._solution_queue = solution_queue
        self._trace_queue = trace_queue
        self._raw_log_queue = raw_log_queue
        self._control = control
        self._options = options
        # 持有 WriterBase 抽象引用（多态），不关心具体子类
        self._solution_writer = solution_writer
        self._trace_writer = trace_writer
        self._raw_data_writer = raw_data_writer
```

### 10.3 依赖注入装配示例

装配过程集中在入口处，Logger 自身不 new 任何 Writer：

```python
def build_logger(config: dict, queues: dict, control) -> Logger:
    """装配日志记录器（依赖注入）"""
    output_dir = config["logging"]["output_dir"]

    # 创建 Writer 实例
    solution_writer = SolutionWriter(
        output_dir, format=config["logging"]["solution_format"],
        trace_level=config["logging"]["trace_level"])
    trace_writer = TraceWriter(
        output_dir, trace_level=config["logging"]["trace_level"])
    raw_data_writer = RawDataWriter(
        output_dir, enabled=config["logging"]["log_raw_data"])

    # 依赖注入装配 Logger
    return Logger(
        solution_queue=queues["solution"],
        trace_queue=queues["trace"],
        raw_log_queue=queues["raw_log"],
        control=control,
        options=config,
        solution_writer=solution_writer,
        trace_writer=trace_writer,
        raw_data_writer=raw_data_writer,
    )
```

### 10.4 纯队列流水线衔接

本模块的 `Logger` 与传感器层、估计层**统一采用纯队列流水线**，全程通过 `queue.put()`/`queue.get()` 传递数据，**无观察者模式、无 notify() 回调**：

```
传感器层（纯队列流水线）:
  ImuSensor/GnssRoverSensor ──put()──→ imu_queue/sensor_queue
                                                ↓
                                     Scheduler（仅转发）
                                                ↓
                                       estimate_queue
                                                ↓
估计层（纯队列流水线）:                  ↓
  LcIntegration ──get()──→ estimate_queue
       │   时间对齐 + 双滤波 EKF (P1 主滤波 + P2 NHC 子滤波)
       │   独立反馈 P1 + P2
       ↓
  LcIntegration ──put()──→ solution_queue ──get()──→ Logger ──→ SolutionWriter
                         trace_queue                          ──→ TraceWriter
  Stream ──put()──→ raw_log_queue                          ──→ RawDataWriter
```

数据通路统一性：
- **传感器 → 估计**：`ImuSensor`/`GnssRoverSensor` 通过 `output_queue.put()` 推入 `imu_queue`/`sensor_queue`，Scheduler 仅转发到 `estimate_queue`
- **估计 → 输出**：`LcIntegration` 通过 `solution_queue.put()` / `trace_queue.put()` 推入双滤波解算结果与事件
- **原始数据 → 输出**：各 Streamer 通过 `raw_log_queue.put()` 推入原始观测
- **全程无回调**：Logger 通过 `queue.get()` 消费，缓冲削峰，异步落盘，避免 IO 阻塞融合

### 10.5 OOP 三大特性体现

| 特性 | 体现 |
|------|------|
| **封装** | 各 Writer 的格式化细节（POS/CSV/NMEA）封装在子类内部，Logger 仅调用 `write()` |
| **继承** | `WriterBase(ABC)` → `SolutionWriter` / `TraceWriter` / `RawDataWriter`，统一 open/write/close 接口 |
| **多态** | Logger 持有三个 `WriterBase` 抽象引用，运行时调用具体子类的 `write()` 方法 |
