# INS/GNSS 组合导航多线程框架设计文档

> 参考 GICI-LIB 的流式读取与多线程架构，设计一个简易但全面的 INS/GNSS 组合导航框架。
> 首选 Python 实现，接口设计保留 C++ 移植可能。
>
> 框架采用面向对象三大特性 + 设计模式组织：
> - **传感器抽象层 BaseSensor**：强制 `get_data()` 接口，统一所有传感器数据获取
> - **工厂模式 SensorFactory**：根据配置动态创建传感器实例，主程序不 new 具体类
> - **纯队列流水线**：Streamer → Queue → Scheduler → estimate_queue → Estimator → solution_queue → Logger，**不使用观察者模式**
> - **纯 threading**：所有数据流通过 `queue.Queue` + 独立线程驱动，**不使用 asyncio**，**不使用 shared_memory**
>
> **时间系统约定**：全框架内部统一使用 **Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）**。
> 输入端（`formators.py`）通过 `gpst_to_unix(week, sow)` 把 GPS 周+周内秒转为 Unix 时间戳；
> 输出端通过 `unix_to_gpst()` 转回 (week, sow) 写文件。
> 时间转换工具：`src/core/time_utils.py`（`gpst_to_unix` / `unix_to_gpst`，`GPST_EPOCH_UNIX = 315964800`）。
>
> **当前实现状态**：
> - ✅ 已实现：`src/stream/base.py::BaseSensor` + `StreamerBase`（流式读取基类，逐行读取 + EOF sentinel）
> - ✅ 已实现：`src/stream/factory.py::SensorFactory`（根据 gnss_source + ins.enabled 装配传感器）
> - ✅ 已实现：`src/stream/formators.py::ImuFormator`（GPST 格式）/ `EuRoCImuFormator`（EuRoC 格式）/ `PosSolFormator`（rtklib POS 解码，时间戳 Unix 化）
> - ✅ 已实现：`src/stream/imu_sensor.py::ImuSensor`（IMU 传感器线程，含 RFU→FRD 坐标系自动转换 `_convert_to_frd()`）
> - ✅ 已实现：`src/stream/gnss_sol_sensor.py::GnssSolSensor`（外部 GNSS 结果传感器线程）
> - ✅ 已实现：`src/stream/internal_gnss_sensor.py::InternalGnssSensor`（内部 GNSS 解算传感器线程，逐历元调用 pntpos/relpos，SPP 模式含多普勒测速）
> - ✅ 已实现：`src/core/ins/initializer.py::InsInitializer`（INS 初始化，三种模式 + 三阈值检验，详见 [初始化.md](file:///home/mxl/workplace/gipylib/skills/初始化.md)）
> - 🚧 预留：RoverSensor / EphSensor / RefSensor（独立星历/基站流，当前由 `InternalGnssSensor` 内部 RINEX 加载完成）
> - 🚧 预留：Scheduler / INS 机械编排核心 `InsCore` / 双滤波 EKF `LcIntegration`（当前三种模式均无 Scheduler，传感器直接推入 queue 由 Logger/SolutionLogger 消费）

---

## 目录

- [1. 设计目标与约束](#1-设计目标与约束)
- [2. 整体架构](#2-整体架构)
- [3. 核心数据类型](#3-核心数据类型)
- [4. 线程模型](#4-线程模型)
- [5. 流式读取层 (Stream)](#5-流式读取层-stream)
- [6. 数据集成层 (Integration)](#6-数据集成层-integration)
- [7. 估计融合层 (Estimate)](#7-估计融合层-estimate)
- [8. 日志层 (Log)](#8-日志层-log)
- [9. 时间对齐与初始化策略](#9-时间对齐与初始化策略)
- [10. 目录结构](#10-目录结构)
- [11. 配置文件格式](#11-配置文件格式)
- [12. C++ 移植考虑](#12-c-移植考虑)

---

## 1. 设计目标与约束

### 1.1 设计目标

| 目标 | 描述 |
|------|------|
| **流式读取** | 不全量加载文件，逐行/逐块读取，O(1) 内存占用 |
| **多线程并行** | 传感器读取、数据集成、估计解算、日志记录分线程执行 |
| **传感器抽象** | BaseSensor 强制 `get_data()` 接口，所有传感器统一抽象，针对接口编程 |
| **工厂装配** | SensorFactory 根据配置动态创建传感器实例，主程序持有 `list[BaseSensor]` |
| **纯队列流水线** | Streamer → Queue → Scheduler → estimate_queue → Estimator → solution_queue → Logger，全程无回调 |
| **纯 threading** | 所有数据流通过 `queue.Queue` + 狿立线程驱动，不使用 asyncio，不使用 shared_memory |
| **时间对齐下沉 Estimator** | Scheduler 仅转发到 estimate_queue，时间对齐由 Estimator 内部处理（参考 KF-GINS 增量切分） |
| **灵活初始化** | 支持 IMU 先到 / GNSS 先到两种场景的渐进式初始化 |
| **自定义解码** | 文本文件格式由用户定义，通过接口适配 |
| **可移植** | Python 首选，接口设计不依赖 Python 特有特性，保留 C++ 移植可能 |

### 1.2 约束

- 仅读取 4 类文件：IMU 文本、GNSS 观测值文本、GNSS 基站观测值文本、广播星历文本
- 无辅助参数文件（DCB/ATX/SP3/CLK 等）
- 文件为自定义文本格式，非 GICI 的二进制格式
- 仅设计框架，不实现具体 GNSS 算法

---

## 2. 整体架构

### 2.1 分层架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                          Main (主线程)                               │
│   解析配置 → 创建 Scheduler → 启动线程 → 主循环等待                  │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
       ┌───────────────────────┼───────────────────────┐
       ▼                       ▼                       ▼
┌─────────────┐        ┌─────────────┐         ┌─────────────┐
│  Stream 层  │        │ Integration │         │   Log 层    │
│ (数据读取)  │───────→│  (数据集成)  │────────→│ (日志记录)  │
│             │        │  仅转发      │         │             │
│ IMU Stream   │       │  ※不做时间对齐│        │ Solution Log│
│ Rover Stream │       │             │         │ Raw Data Log│
│ Eph Stream   │       │             │         │             │
│ Ref Stream   │       │             │         │             │
└─────────────┘        └──────┬──────┘         └─────────────┘
                              │
                              ▼
                       ┌─────────────┐
                       │ Estimate 层 │
                       │ (估计融合)   │
                       │             │
                       │ 时间对齐     │
                       │ SPP/INS初始化│
                       │ INS/GNSS融合│
                       └─────────────┘
```

### 2.2 数据流总览

```
文件系统                         线程                          线程
─────────                       ────                          ────

imu.txt ──→ IMUStreamer ──→ ImuFormator ──┐
                                            │
obs.txt ──→ RoverStreamer ──→ RoverFormator ┤
                                            ├──→ Scheduler ──→ Estimator
eph.txt ──→ EphStreamer ──→ EphFormator ────┤    (仅转发)     (时间对齐+解算)
                                            │        │
ref.txt ──→ RefStreamer ──→ RefFormator ────┘        │
                                                      ▼
                                                 Logger (日志)
```

---

## 3. 核心数据类型

### 3.1 IMU 数据

> **实际实现**（参考 `src/core/data_types.py`）：

```python
@dataclass
class ImuMeasurement:
    timestamp: float          # Unix 时间戳（秒，与 rtklib-py gtime_t 一致）
    week: int                 # GPS 周号（由 timestamp 派生，便利字段）
    accel: np.ndarray         # [3] m/s² 机体坐标系
    gyro: np.ndarray          # [3] rad/s 机体坐标系
```

> **早期设计版本**（含 `dt` 字段，预留 INS 启用后机械编排使用）：

```python
@dataclass
class ImuMeasurement_Design:
    timestamp: float                    # Unix 时间戳（秒）
    acceleration: np.ndarray            # [3] 加速度 (m/s^2), 机体坐标系
    angular_velocity: np.ndarray        # [3] 角速度 (rad/s), 机体坐标系
    dt: float = 0.0                     # 距上一时刻的时间间隔 (秒)
    # dt 字段说明:
    #   - 正常情况下 dt = timestamp - prev.timestamp
    #   - 增量切分后 dt 会被修改（参考 estimator.md 第9节）
    #   - 机械编排时通过 dt 计算增量: dtheta = omega * dt, dvel = f * dt
```

**IMU CSV 输入格式**（由 `src/stream/formators.py` 解码，支持两种格式，由配置项 `ins.imu_format` 选择）：

**GPST 格式**（`imu_format: "gpst"`，由 `ImuFormator` 解码）：
```
GPS week, GPS sow, gx, gy, gz, ax, ay, az
```
解码时通过 `gpst_to_unix(week, sow)` 转换为 Unix 时间戳。

**EuRoC 格式**（`imu_format: "euroc"`，由 `EuRoCImuFormator` 解码）：
```
timestamp_ns, wx, wy, wz, ax, ay, az
```
解码时 `timestamp = timestamp_ns / 1e9`（Unix 纳秒 → Unix 秒），GPS 周号由 `unix_to_gpst(timestamp)` 派生。原始坐标系默认为 RFU，由 `ImuSensor._convert_to_frd()` 转 FRD。

**格式选择**：`ImuSensor._create_formator(imu_format)` 工厂方法根据 `imu_format` 配置值创建对应解码器实例。

### 3.2 GNSS 观测值

```python
@dataclass
class GnssObservation:
    timestamp: float                    # 秒
    satellite_id: str                   # 如 "G01", "E12", "C03"
    pseudorange: float                  # 伪距 (m)
    phaserange: float                   # 载波相位 (cycles)
    doppler: float                      # 多普勒 (Hz)
    snr: float                          # 信噪比 (dB-Hz)
    frequency_label: str                # 频率标识, 如 "L1", "L2", "B1"

@dataclass
class GnssMeasurement:
    timestamp: float                    # 本历元时间戳
    observations: list[GnssObservation] # 本历元所有卫星观测值
```

### 3.3 广播星历

```python
@dataclass
class Ephemeris:
    satellite_id: str
    toe: float                          # 星历参考时间
    sqrt_a: float                       # 轨道半长轴平方根
    e: float                            # 偏心率
    i0: float                           # 轨道倾角
    omega0: float                       # 升交点赤经
    # ... 其他开普勒参数和改正项
    health: int                         # 健康状态

@dataclass
class EphemerisData:
    timestamp: float                    # 解码时间戳
    ephemerides: list[Ephemeris]        # 本批次所有星历
```

### 3.4 基站观测值

```python
# 与 GnssMeasurement 结构相同，仅角色不同
@dataclass
class ReferenceMeasurement(GnssMeasurement):
    station_id: str = ""                # 基站标识
```

### 3.5 统一数据容器

```python
@dataclass
class SensorData:
    """所有传感器数据的统一容器，一次只承载一种类型"""
    tag: str                            # 数据来源标识: "imu"/"rover"/"gnss_solution"/"ephemeris"/"reference"
    imu: Optional[ImuMeasurement] = None
    gnss: Optional[GnssMeasurement] = None           # 原始 GNSS 观测值（内部解算模式）
    gnss_solution: Optional[GnssSolution] = None     # GNSS 解算结果（外部结果模式或内部解算后）
    ephemeris: Optional[EphemerisData] = None
    reference: Optional[ReferenceMeasurement] = None
```

### 3.6 解算结果

```python
@dataclass
class Solution:
    timestamp: float
    position_enu: Optional[np.ndarray]  = None  # [3] ENU 位置 (m)
    velocity_enu: Optional[np.ndarray]  = None  # [3] ENU 速度 (m/s)
    attitude: Optional[np.ndarray]      = None  # [3] 横滚/俯仰/航向 (rad)
    covariance: Optional[np.ndarray]    = None  # [15,15] 协方差
    status: str = "None"                           # None/SPP/Float/Fixed/INS
    num_satellites: int = 0
```

---

## 4. 线程模型

### 4.1 线程规划

```
┌─────────────────────────────────────────────────────────────────────┐
│                         主线程 (Main Thread)                         │
│   解析配置 → 创建对象 → 启动工作线程 → 主循环等待退出信号            │
└─────────────────────────────────────────────────────────────────────┘

┌──────────────────┐  ┌──────────────────┐  ┌──────────────────┐
│  IMU 读取线程     │  │  流动站读取线程   │  │  星历读取线程     │
│  (IMUStreamer)   │  │  (RoverStreamer) │  │  (EphStreamer)   │
│                  │  │                  │  │                  │
│  逐行读取IMU文件  │  │  逐行读取观测文件 │  │  逐行读取星历文件 │
│  解码→推入队列    │  │  解码→推入队列    │  │  解码→推入队列    │
└────────┬─────────┘  └────────┬─────────┘  └────────┬─────────┘
         │                     │                     │
         ▼                     ▼                     ▼
   imu_queue            rover_queue            eph_queue

┌──────────────────┐
│  基站读取线程     │
│  (RefStreamer)   │
│                  │
│  逐行读取基站文件 │
│  解码→推入队列    │
└────────┬─────────┘
         │
         ▼
   ref_queue

         │                     │                     │
         ▼                     ▼                     ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    调度线程 (Scheduler Thread)                        │
│                                                                      │
│  从各队列取数据 → 初始化状态机 → 推入估计队列（仅转发，不做时间对齐）│
│                                                                      │
│  核心逻辑:                                                           │
│  ├── 未初始化: 根据配置判断模式, 差分模式先同步Rover与Ref时间          │
│  │   ├── 差分模式: Rover+Ref未同步 → 暂停超前线程,落后方追赶          │
│  │   ├── 同步后IMU早于GNSS → 持续读IMU, 暂停GNSS, 直到时间匹配      │
│  │   └── 同步后GNSS早于IMU → 纯GNSS解算, IMU不暂停持续读取检测       │
│  └── 已初始化: 直接转发到 estimate_queue（时间对齐由 Estimator 处理）│
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
                        estimate_queue

┌─────────────────────────────────────────────────────────────────────┐
│                    估计线程 (Estimator Thread)                        │
│                                                                      │
│  从估计队列取数据 → 时间对齐(增量切分) → 执行SPP/INS初始化/松组合解算  │
│  → 输出Solution                                                      │
│  ※ 时间对齐在此处理（参考 KF-GINS newImuProcess + imuInterpolate）   │
└──────────────────────────────┬──────────────────────────────────────┘
                               │
                               ▼
                        solution_queue

┌─────────────────────────────────────────────────────────────────────┐
│                    日志线程 (Logger Thread)                           │
│                                                                      │
│  从 solution_queue 取结果 → 写入文件/终端输出                        │
│  从 raw_log_queue 取原始数据 → 写入原始数据日志                      │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2 线程间通信

| 通信通道 | 类型 | 生产者 | 消费者 | 容量 |
|----------|------|--------|--------|------|
| `imu_queue` | `Queue[SensorData]` | IMU 读取线程 | 调度线程 | 2000 |
| `rover_queue` | `Queue[SensorData]` | 流动站读取线程 | 调度线程 | 100 |
| `eph_queue` | `Queue[SensorData]` | 星历读取线程 | 调度线程 | 50 |
| `ref_queue` | `Queue[SensorData]` | 基站读取线程 | 调度线程 | 100 |
| `estimate_queue` | `Queue[SensorData]` | 调度线程 | 估计线程 | 50 |
| `solution_queue` | `Queue[Solution]` | 估计线程 | 日志线程 | 200 |
| `raw_log_queue` | `Queue[tuple]` | 各读取线程 | 日志线程 | 500 |

**Python 实现**：使用 `queue.Queue`（线程安全，支持 `maxsize` 有界队列）。

**C++ 移植**：替换为 `std::queue` + `std::mutex` + `std::condition_variable` 封装的线程安全队列。

### 4.3 线程控制

```python
class ThreadControl:
    """全局线程控制，所有线程检查 running 标志决定是否退出"""
    running: bool = True

    def shutdown(self):
        self.running = False

    def is_running(self) -> bool:
        return self.running
```

---

## 5. 流式读取层 (Stream)

### 5.1 类层次

> **实际实现**（参考 `src/stream/base.py` / `imu_sensor.py` / `gnss_sol_sensor.py` / `internal_gnss_sensor.py`）：

```
═══════════════════════════════════════════════════════════
  传感器抽象层（统一 get_data 接口，无观察者模式）
═══════════════════════════════════════════════════════════
BaseSensor(ABC)                          # 传感器抽象基类，强制 get_data()（无 Subject 角色）
└── StreamerBase(BaseSensor, Thread)     # ✅ 文件流读取基类（逐行读取 + Formator 解码，仅 queue.put）
    ├── ImuSensor(StreamerBase)          # ✅ IMU 文本读取（继承 StreamerBase，绑定 ImuFormator）
    └── GnssSolSensor(StreamerBase)      # ✅ 外部 GNSS 结果读取（继承 StreamerBase，绑定 PosSolFormator）

InternalGnssSensor(Thread)               # ✅ 内部 GNSS 解算传感器（直接继承 Thread，不继承 BaseSensor）
                                         #   逐历元调用 SppProcessor/RtkProcessor，推入 gnss_queue

SensorFactory                            # ✅ 工厂模式：根据 gnss_source + ins.enabled 创建传感器列表

═══════════════════════════════════════════════════════════
  解码层（Formator，自定义格式适配）
═══════════════════════════════════════════════════════════
FormatorBase (抽象基类, 自定义解码接口)   # ✅
├── ImuFormator       — ✅ IMU CSV 解码（GPST 格式: week,sow → Unix 时间戳）
├── EuRoCImuFormator  — ✅ IMU CSV 解码（EuRoC 格式: timestamp_ns → Unix 时间戳, GPS 周号派生）
└── PosSolFormator    — ✅ rtklib POS 格式解码（ymdhms → GPST → Unix 时间戳）
```

**设计要点**：
- `StreamerBase` 继承 `BaseSensor` 与 `Thread`，实现 `get_data()` 返回解码后的数据
- **不使用 asyncio**：高频 IMU 仍走 `queue.Queue` + 独立线程，统一并发模型
- **不使用 shared_memory**：所有数据通过 `queue.put()` 传递，简化内存管理
- `SensorFactory.create_sensors(config, imu_queue, gnss_queue, control)` 返回 `list[BaseSensor]`
- **数据通路**：Streamer `run()` 线程 → `output_queue.put()` → Logger 拉取（当前无 Scheduler）
- **InternalGnssSensor 特殊**：不继承 `StreamerBase`，因为内部解算不是"逐行读取文件"，
  而是"加载 RINEX + 逐历元解算"，所以直接继承 `Thread`

### 5.2 StreamerBase 接口（实际实现）

> `StreamerBase` 继承 `BaseSensor` 与 `Thread`，实现 `get_data()` 接口。
> 内部封装逐行读取 + Formator 解码 + 队列推送的完整流程，文件读完后推入 `None` 作为 EOF sentinel。

```python
# src/stream/base.py
class BaseSensor(ABC):
    """传感器抽象基类，强制 get_data() 接口。"""

    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def get_data(self):
        """获取一条数据（非阻塞，无数据返回 None）。"""
        ...


class StreamerBase(BaseSensor, Thread):
    """流式读取器基类 — 逐行读取文本文件，O(1) 内存。

    继承 BaseSensor 与 Thread，内部封装：逐行读取 → Formator 解码 → 推入队列。
    文件读完后向队列推入 None 作为 EOF sentinel。
    """

    def __init__(self, file_path: str, formator, output_queue: Queue,
                 control: ThreadControl, tag: str):
        # 注意：必须先初始化 Thread，因为 Thread.name 是 property
        Thread.__init__(self, name=tag, daemon=True)
        BaseSensor.__init__(self, name=tag)
        self.file_path = file_path
        self.formator = formator
        self.output_queue = output_queue
        self.control = control
        self.tag = tag

    def run(self):
        """线程入口：逐行读取 → 解码 → 入队 → EOF sentinel。"""
        try:
            with open(self.file_path, "r", encoding="utf-8") as f:
                for line in f:
                    if not self.control.is_running():
                        break
                    data = self.formator.decode(line)
                    if data is not None:
                        self.output_queue.put(data)
        finally:
            self.output_queue.put(None)  # EOF sentinel

    def get_data(self):
        """非阻塞返回队列头部数据。"""
        try:
            return self.output_queue.get_nowait()
        except Empty:
            return None
```

**关键点**：
- 文件读完后在 `finally` 块中推入 `None` 作为 EOF sentinel，确保即使异常也会推入
- `get_data()` 从 `output_queue` 非阻塞取数据，无数据返回 `None`
- `Thread.__init__` 必须先于 `BaseSensor.__init__` 调用（因为 `Thread.name` 是 property）

### 5.3 ImuFormator / EuRoCImuFormator / PosSolFormator（实际实现）

> 三个 Formator 都在 `src/stream/formators.py` 中实现，时间戳在解码时统一转为 Unix 时间戳。
> `ImuSensor._create_formator(imu_format)` 工厂方法根据配置项 `ins.imu_format`（`gpst` / `euroc`）创建对应 IMU 解码器实例。

```python
# src/stream/formators.py
class ImuFormator(FormatorBase):
    """IMU 文本解码（GPST 格式）。
    输入列: GPS week, GPS sow, gx, gy, gz, ax, ay, az
    """
    def decode(self, line: str) -> Optional[SensorData]:
        parts = line.strip().split(",")
        week, sow = int(parts[0]), float(parts[1])
        imu = ImuMeasurement(
            timestamp=gpst_to_unix(week, sow),  # GPST → Unix 时间戳
            week=week,
            accel=np.array([ax, ay, az], dtype=np.float64),
            gyro=np.array([gx, gy, gz], dtype=np.float64),
        )
        return SensorData(tag="imu", imu=imu)


class EuRoCImuFormator(FormatorBase):
    """IMU 文本解码（EuRoC 格式）。
    输入列: timestamp [ns], w_x, w_y, w_z, a_x, a_y, a_z
    时间戳为 Unix 纳秒，除以 1e9 转换为 Unix 秒。
    GPS 周号由 Unix 时间戳派生（unix_to_gpst）。
    坐标系默认为 RFU (Right-Front-Up)，由 ImuSensor 负责转换为 FRD。
    """
    def decode(self, line: str) -> Optional[SensorData]:
        parts = line.strip().split(",")
        timestamp_ns = int(parts[0])
        timestamp = timestamp_ns / 1e9  # Unix 纳秒 → Unix 秒
        week, _ = unix_to_gpst(timestamp)  # GPS 周号派生
        imu = ImuMeasurement(
            timestamp=timestamp,
            week=week,
            accel=np.array([ax, ay, az], dtype=np.float64),
            gyro=np.array([wx, wy, wz], dtype=np.float64),
        )
        return SensorData(tag="imu", imu=imu)


class PosSolFormator(FormatorBase):
    """rtklib POS 格式解码。
    数据行: yyyy/mm/dd hh:mm:ss.s  x  y  z  Q  ns  sdx  sdy  sdz  ...
    """
    def decode(self, line: str) -> Optional[SensorData]:
        parts = line.split()
        y, mo, d = (int(x) for x in parts[0].split("/"))
        h, mi, s = parts[1].split(":")
        week, sow = ymdhms_to_gpst(y, mo, d, int(h), int(mi), float(s))
        sol = GnssSolution(
            timestamp=gpst_to_unix(week, sow),  # GPST → Unix 时间戳
            week=week,
            position=np.array([x, y_pos, z], dtype=np.float64),
            quality=q,
            num_sv=ns,
            sd=np.array([sdx, sdy, sdz], dtype=np.float64),
        )
        return SensorData(tag="gnss_solution", gnss_solution=sol)
```

### 5.4 FormatorBase 接口 — 自定义解码（实际实现）

```python
# src/stream/formators.py
class FormatorBase(ABC):
    """解码层抽象基类。"""

    @abstractmethod
    def decode(self, line: str) -> Optional[SensorData]:
        """解码一行文本。

        Returns:
            SensorData 或 None（注释/空行）
        """
        ...
```

> **与早期设计的差异**：
> - 实际 `FormatorBase.decode()` 返回 `SensorData`（已封装），而非原始数据对象
> - 移除了 `get_timestamp()` 抽象方法（时间戳在 `decode()` 内部通过 `gpst_to_unix()` 直接转换）
> - 移除了 `flush()` 方法（当前 IMU/POS 格式无需多行累积，每行独立解码）

**读取模式说明**（早期设计，当前未实现 `timed` 模式）：

| 模式 | `playback_mode` | 行为 | 适用场景 | 是否需要 `speed` |
|------|-----------------|------|----------|-----------------|
| **尽速读取** | `"asap"` (默认，当前唯一实现) | 读一行→解码→推队列，无 sleep，CPU 全速 | 后处理，多线程尽速读取 | **不需要** |
| **按数据时间驱动** | `"timed"` (🚧 预留) | 读一行→提取时间戳→sleep(Δt/speed)→推队列 | 模拟实时流，测试实时算法行为 | **需要**，默认 100 倍 |

### 5.5 流式 RINEX 解码框架（🚧 预留，当前由 InternalGnssSensor 内部完成）

> **当前实现**：本项目未独立实现流式 RINEX 解码器，而是由 `InternalGnssSensor` 内部调用
> rtklib-py 已吸收的 `src/core/gnss/rtklib/rinex.py` 的 `rnx_decode` / `decode_obsfile` / `decode_nav`
> 完成全量加载，再逐历元解算。以下为早期设计的流式 RINEX 解码方案，预留未来扩展。
>
> 参考项目：
> - `library/rtklib-py/src/rinex.py` — GNSS 解算库的一部分（已吸收到 `src/core/gnss/rtklib/rinex.py`）
> - `library/pyrinex-master/pyrinex/rinex3.py` — 纯 RINEX 解码库，全量读取，输出 xarray
>
> 本框架**仅参考解码逻辑**，改为**流式逐行解码**而非全量加载。

#### 5.4.1 参考项目解码分析

**rtklib-py 解码方式**：

| 方法 | 行为 | 特点 |
|------|------|------|
| `rnx_decode.decode_nav()` | 一次性读完所有星历到 `nav.eph[]` / `nav.geph[]` | 手动逐字段解析，`self.flt(line, col)` 按列提取 |
| `rnx_decode.decode_obsh()` | 读取 OBS 文件头 | 逐行解析信号表 |
| `rnx_decode.decode_obs()` | 一次性读完所有历元到 `self.obslist[]` | 历元头+卫星行循环，`Obs` 对象含 numpy 数组 |

**pyrinex 解码方式**：

| 方法 | 行为 | 特点 |
|------|------|------|
| `rinexnav3()` | 一次性读完所有星历 | **累积原始字符串 + `np.genfromtxt` 批量解析** |
| `_getObsTypes()` | 读取 OBS 文件头 | **正确处理续行** (nsig >= 14 时) |
| `_scan3()` | 一次性读完所有历元 | **累积原始字符串 + `np.genfromtxt` 批量解析** |
| `_newnav()` | 定义各系统字段名 | **完整的字段名映射表** (GPS/GAL/GLO/QZSS/BDS/SBAS) |

**两者对比**：

| 维度 | rtklib-py | pyrinex |
|------|-----------|---------|
| 数据解析 | 手动 `flt(line, col)` 逐字段提取 | 累积原始字符串 + `np.genfromtxt` 批量解析 |
| 字段定义 | 散落在 `decode_nav()` 各分支 | `_newnav()` 集中定义，按卫星系统返回字段名列表 |
| 头部续行 | 未处理 (nsig >= 14) | **正确处理** `_getObsTypes()` 中 `while n > 0` |
| 输出格式 | 自定义 `Obs`/`Eph`/`Geph` 类 | `xarray.Dataset` |
| gzip 支持 | 无 | `io.py` 中 `opener()` 透明支持 |
| RINEX 2 | 不支持 | `rinex2.py` 支持 |
| OBS 观测值解析 | 逐字段手动提取 | `(14,1,1)*Fmax` 定界符批量解析 |

**本框架的参考策略**：

| 参考来源 | 参考内容 | 不参考的内容 |
|----------|---------|-------------|
| **pyrinex** | 字段名定义 (`_newnav`)、头部续行处理、gzip 支持、RINEX 2 支持 | 全量读取、xarray 输出、`np.genfromtxt` 批量解析 |
| **rtklib-py** | 逐字段解析逻辑 (`flt`)、Obs/Eph 数据结构设计 | 全量读取、手动散落的字段定义 |

**RINEX 3.x OBS 文件结构**（关键：一个历元有多行）：

```
> 2023 01 15 08 00  0.0000000  0  9G12G15G18G21G24G27  ← 历元头: > + 时间 + 卫星数 + 卫星列表
 G12  24432510.280   128259527.3828 ...                 ← 卫星1观测值 (1行)
 G15  23456789.123   123456789.1234 ...                 ← 卫星2观测值 (1行)
 ...                                                     ← 每颗卫星一行
> 2023 01 15 08 00 30.0000000  0  8G12G15G18G21G24     ← 下一个历元
```

**RINEX 3.x NAV 文件结构**（每颗卫星的星历占 8 行）：

```
G01 2023 01 15 08 00 00  1.2345e-04 ...                ← 第1行: 卫星ID + 时钟参数
     7.6543e-01  1.2345e-02  ...                        ← 第2行: IODE, Crs, Δn, M0
     ...                                                 ← 第3-7行: 轨道参数
     0.0000e+00  0.0000e+00  ...                        ← 第8行: TOT, Fit Interval
```

#### 5.4.2 流式解码设计原则

| rtklib-py (全量) | 本框架 (流式) |
|-------------------|---------------|
| `decode_obs()` 一次读完所有历元 | `decode(line)` 逐行喂入，历元完成时返回 |
| `decode_nav()` 一次读完所有星历 | `decode(line)` 逐行喂入，一条星历完成时返回 |
| `Obs` 对象包含整个历元所有卫星 | `GnssMeasurement` 包含一个历元所有卫星 |
| `Eph`/`Geph` 对象包含一条星历 | `EphemerisData` 包含一条星历 |
| 返回 `obslist[]` 全量列表 | 返回单条数据，推入队列 |

#### 5.4.3 RINEX OBS 流式解码器

```python
class RinexObsFormator(FormatorBase):
    """RINEX 3.x 观测值流式解码器
    参考:
      - pyrinex: _getObsTypes() 头部续行处理, _scan3() 观测值解析
      - rtklib-py: rnx_decode.decode_obs() 历元累积逻辑, flt() 字段提取

    核心问题: 一个历元有多行(每卫星一行), 必须累积到历元完成才返回完整数据
    """

    def __init__(self, sig_tbl=None, skip_sig_tbl=None):
        # 信号表配置 (参考 rtklib-py rnx_decode.__init__)
        self.gnss_tbl = {'G': 0, 'E': 6, 'R': 2, 'J': 5, 'C': 3, 'S': 1}
        self.sig_tbl = sig_tbl or {}
        self.skip_sig_tbl = skip_sig_tbl or {}

        # 解码状态 (RINEX 头信息)
        self._header_parsed = False
        self._ver = -1.0
        self._nsig = {}       # sys → 每系统信号数
        self._sigid = {}      # sys → [sig_id per slot]
        self._typeid = {}     # sys → [type_id per slot] (0=C,1=L,2=S,3=D)
        self._nband = {}      # sys → 频带数
        self._fields = {}     # sys → [field_name per slot] (参考 pyrinex _getObsTypes)
        self._Fmax = 0        # 最大信号数 (参考 pyrinex)
        self._approx_pos = np.zeros(3)
        self._header = {}     # 头部信息字典 (参考 pyrinex)

        # 历元累积状态
        self._epoch_time = None       # 当前历元时间
        self._epoch_sats = []         # 当前历元卫星列表
        self._epoch_obs = []          # 当前历元观测值列表
        self._nsat_expected = 0       # 当前历元预期卫星数
        self._nsat_received = 0       # 当前历元已接收卫星数

    def decode(self, line: str) -> Optional[GnssMeasurement]:
        """逐行解码 RINEX OBS 文件

        返回值:
          - None: 头部/历元未完成/注释行
          - GnssMeasurement: 一个完整历元的数据
        """
        if not self._header_parsed:
            if self._parse_header_line(line):
                return None  # 继续读头
            return None

        # 历元头行: > year month day hour min sec flag nsat ...
        if line.startswith('>'):
            return self._start_new_epoch(line)

        # 卫星观测值行
        return self._parse_obs_line(line)

    def _parse_header_line(self, line: str) -> bool:
        """解析 RINEX 头部, 返回 True 表示头未结束
        参考 pyrinex _getObsTypes() — 正确处理续行"""
        if len(line) < 60:
            return not self._header_parsed

        label = line[60:].strip()
        content = line[:60]

        if label == 'END OF HEADER':
            self._header_parsed = True
            return False

        if label == 'RINEX VERSION / TYPE':
            self._ver = float(line[4:10])

        elif label == 'APPROX POSITION XYZ':
            self._approx_pos = np.array([
                float(line[0:14]), float(line[14:28]), float(line[28:42])
            ])

        elif 'SYS / # / OBS TYPES' in label:
            # 参考 pyrinex _getObsTypes(): 正确处理续行
            sys_char = content[0]
            if sys_char in self.gnss_tbl:
                sys = self.gnss_tbl[sys_char]
                N = int(content[3:6])  # 信号数

                # 第一行的信号列表
                fields_list = content[6:60].split()
                self._Fmax = max(N, self._Fmax)

                # 续行处理 (参考 pyrinex: while n > 0)
                n_remaining = N - 13
                # 注意: 续行在后续 decode() 调用中处理
                # 此处先记录续行状态
                if n_remaining > 0:
                    self._continuation_pending = {
                        'sys_char': sys_char, 'sys': sys,
                        'fields': fields_list, 'n_remaining': n_remaining
                    }
                else:
                    self._process_obs_types(sys, sys_char, fields_list, N)

        # 保存头部信息 (参考 pyrinex)
        if label.strip() not in self._header:
            self._header[label.strip()] = content
        else:
            self._header[label.strip()] += " " + content

        return True  # 头未结束

    def _process_obs_types(self, sys, sys_char, fields_list, N):
        """处理信号类型列表, 构建 typeid/sigid 映射
        参考 rtklib-py rnx_decode.decode_obsh() 信号分类逻辑"""
        self._nsig[sys] = N
        self._fields[sys_char] = fields_list

        typeid_list = []
        sigid_list = []
        for sig in fields_list:
            if sig[0] == 'C':
                typeid_list.append(0)  # Code (伪距)
            elif sig[0] == 'L':
                typeid_list.append(1)  # Phase (载波)
            elif sig[0] == 'S':
                typeid_list.append(2)  # Signal strength (信噪比)
            elif sig[0] == 'D':
                typeid_list.append(3)  # Doppler (多普勒)
            else:
                typeid_list.append(-1)
            # 信号频率标识 (如 L1C, C2W 等)
            sigid_list.append(sig[1:3] if len(sig) >= 3 else '')

        self._typeid[sys] = typeid_list
        self._sigid[sys] = sigid_list
        self._nband[sys] = len([t for t in typeid_list if t == 1])  # 载波频带数

    def _start_new_epoch(self, line: str) -> Optional[GnssMeasurement]:
        """处理历元头行, 如果有前一个历元则返回其完整数据
        参考 rtklib-py rnx_decode.decode_obs() + pyrinex _scan3()"""
        result = None

        # 如果有累积的历元数据, 先返回
        if self._epoch_time is not None and len(self._epoch_obs) > 0:
            result = GnssMeasurement(
                timestamp=self._epoch_time,
                observations=self._epoch_obs,
                approx_position=self._approx_pos.copy()
            )

        # 解析新历元头
        # > 2023 01 15 08 00  0.0000000  0  9G12G15G...
        year = int(line[2:6])
        month = int(line[7:9])
        day = int(line[10:12])
        hour = int(line[13:15])
        minute = int(line[16:18])
        sec = float(line[19:29])
        self._epoch_time = self._epoch2time([year, month, day, hour, minute, sec])
        self._nsat_expected = int(line[32:35])
        self._nsat_received = 0
        self._epoch_obs = []
        self._epoch_sats = []

        return result

    def _parse_obs_line(self, line: str) -> Optional[GnssMeasurement]:
        """解析单颗卫星观测值行
        参考:
          - rtklib-py: 逐字段提取 (16 字符/字段)
          - pyrinex: (14,1,1) 定界符 — 14 字符数据 + 1 字符 LLI + 1 字符 SSI
        """
        if len(line) < 4:
            return None

        sys_char = line[0]
        if sys_char not in self.gnss_tbl:
            return None

        sys = self.gnss_tbl[sys_char]
        prn = int(line[1:3])
        sat_id = f"{sys_char}{prn:02d}"

        # 解析观测值
        # 参考 pyrinex: 每个观测值占 14 字符, 后跟 1 字符 LLI + 1 字符 SSI
        # 参考 rtklib-py: 16 字符/字段, typeid 分类
        pseudoranges = []
        phaseranges = []
        dopplers = []
        snrs = []

        if sys in self._typeid:
            nsig = self._nsig[sys]
            for i in range(nsig):
                offset = 16 * i + 4  # rtklib-py 格式: 起始偏移 4, 每字段 16 字符
                if offset + 14 > len(line):
                    break

                obs_str = line[offset:offset + 14].strip()
                lli_str = line[offset + 14:offset + 15] if offset + 15 <= len(line) else ' '
                ssi_str = line[offset + 15:offset + 16] if offset + 16 <= len(line) else ' '

                try:
                    obs_val = float(obs_str) if obs_str else 0.0
                except ValueError:
                    obs_val = 0.0

                # 按 typeid 分类 (参考 rtklib-py)
                tid = self._typeid[sys][i] if i < len(self._typeid[sys]) else -1
                if tid == 0:    # C = Code (伪距)
                    pseudoranges.append(obs_val)
                elif tid == 1:  # L = Phase (载波)
                    phaseranges.append(obs_val)
                elif tid == 2:  # S = Signal strength (信噪比)
                    snrs.append(obs_val)
                elif tid == 3:  # D = Doppler (多普勒)
                    dopplers.append(obs_val)

        self._epoch_obs.append(GnssObservation(
            timestamp=self._epoch_time,
            satellite_id=sat_id,
            pseudorange=pseudoranges[0] if pseudoranges else 0.0,
            phaserange=phaseranges[0] if phaseranges else 0.0,
            doppler=dopplers[0] if dopplers else 0.0,
            snr=snrs[0] if snrs else 0.0,
            frequency_label=self._sigid.get(sys, [''])[0] if sys in self._sigid else ''
        ))
        self._nsat_received += 1

        # 如果已接收完所有卫星, 返回完整历元
        if self._nsat_received >= self._nsat_expected:
            result = GnssMeasurement(
                timestamp=self._epoch_time,
                observations=self._epoch_obs,
                approx_position=self._approx_pos.copy()
            )
            self._epoch_time = None
            self._epoch_obs = []
            return result

        return None  # 历元未完成

    def flush(self) -> Optional[GnssMeasurement]:
        """文件结束时, 返回最后一个未完成的历元"""
        if self._epoch_time is not None and len(self._epoch_obs) > 0:
            result = GnssMeasurement(
                timestamp=self._epoch_time,
                observations=self._epoch_obs,
                approx_position=self._approx_pos.copy()
            )
            self._epoch_time = None
            self._epoch_obs = []
            return result
        return None

    def get_timestamp(self, decoded: GnssMeasurement) -> float:
        return decoded.timestamp

    @staticmethod
    def _epoch2time(ep: list) -> float:
        """年月日时分秒 → Unix 时间戳（与 rtklib-py gtime_t 一致）

        实际实现使用 src/core/time_utils.py 中的 ymdhms_to_gpst + gpst_to_unix。
        """
        # 参考 rtklib-py epoch2time
        import datetime
        dt = datetime.datetime(ep[0], ep[1], ep[2], ep[3], ep[4], int(ep[5]))
        gps_epoch = datetime.datetime(1980, 1, 6)
        gpst_seconds = (dt - gps_epoch).total_seconds()
        # 转换为 Unix 时间戳：GPST_EPOCH_UNIX = 315964800
        return 315964800.0 + gpst_seconds
```

#### 5.4.4 RINEX NAV 流式解码器

```python
class RinexNavFormator(FormatorBase):
    """RINEX 3.x 广播星历流式解码器
    参考:
      - pyrinex: _newnav() 字段名定义, rinexnav3() 累积+解析
      - rtklib-py: rnx_decode.decode_nav() 逐字段解析

    核心问题: 每颗卫星的星历占 8 行(GPS/GAL/QZS) 或 4 行(GLO/SBAS),
             必须累积到行数足够才返回完整星历
    """

    # 各系统字段名定义 (参考 pyrinex _newnav)
    NAV_FIELDS = {
        'G': ['SVclockBias', 'SVclockDrift', 'SVclockDriftRate',   # 行1: 时钟
              'IODE', 'Crs', 'DeltaN', 'M0',                       # 行2
              'Cuc', 'Eccentricity', 'Cus', 'sqrtA',               # 行3
              'Toe', 'Cic', 'Omega0', 'Cis',                       # 行4
              'Io', 'Crc', 'omega', 'OmegaDot',                    # 行5
              'IDOT', 'CodesL2', 'GPSWeek', 'L2Pflag',             # 行6
              'SVacc', 'health', 'TGD', 'IODC',                    # 行7
              'TransTime', 'FitIntvl'],                             # 行8
        'E': ['SVclockBias', 'SVclockDrift', 'SVclockDriftRate',
              'IODnav', 'Crs', 'DeltaN', 'M0',
              'Cuc', 'Eccentricity', 'Cus', 'sqrtA',
              'Toe', 'Cic', 'Omega0', 'Cis',
              'Io', 'Crc', 'omega', 'OmegaDot',
              'IDOT', 'DataSrc', 'GALWeek',
              'SISA', 'health', 'BGDe5a', 'BGDe5b',
              'TransTime'],
        'J': ['SVclockBias', 'SVclockDrift', 'SVclockDriftRate',
              'IODE', 'Crs', 'DeltaN', 'M0',
              'Cuc', 'Eccentricity', 'Cus', 'sqrtA',
              'Toe', 'Cic', 'Omega0', 'Cis',
              'Io', 'Crc', 'omega', 'OmegaDot',
              'IDOT', 'CodesL2', 'GPSWeek', 'L2Pflag',
              'SVacc', 'health', 'TGD', 'IODC',
              'TransTime', 'FitIntvl'],
        'C': ['SVclockBias', 'SVclockDrift', 'SVclockDriftRate',
              'AODE', 'Crs', 'DeltaN', 'M0',
              'Cuc', 'Eccentricity', 'Cus', 'sqrtA',
              'Toe', 'Cic', 'Omega0', 'Cis',
              'Io', 'Crc', 'omega', 'OmegaDot',
              'IDOT', 'BDTWeek',
              'SVacc', 'SatH1', 'TGD1', 'TGD2',
              'TransTime', 'AODC'],
        'R': ['SVclockBias', 'SVrelFreqBias', 'MessageFrameTime',  # 行1: 时钟
              'X', 'dX', 'dX2', 'health',                          # 行2
              'Y', 'dY', 'dY2', 'FreqNum',                         # 行3
              'Z', 'dZ', 'dZ2', 'AgeOpInfo'],                       # 行4
        'S': ['SVclockBias', 'SVrelFreqBias', 'MessageFrameTime',
              'X', 'dX', 'dX2', 'health',
              'Y', 'dY', 'dY2', 'URA',
              'Z', 'dZ', 'dZ2', 'IODN'],
    }

    # 各系统星历行数
    NAV_LINES = {
        'G': 8, 'E': 8, 'J': 8, 'C': 8,  # GPS/GAL/QZS/BDS: 8 行
        'R': 4, 'S': 4,                    # GLONASS/SBAS: 4 行
    }

    def __init__(self):
        self.gnss_tbl = {'G': 0, 'E': 6, 'R': 2, 'J': 5, 'C': 3, 'S': 1}

        # 头部状态
        self._header_parsed = False
        self._ver = -1.0
        self._ion = np.zeros((2, 4))  # 电离层参数

        # 星历累积状态
        self._current_lines = []  # 当前星历的累积行
        self._current_sys = None
        self._current_sat = None
        self._current_sys_char = None

    def decode(self, line: str) -> Optional[EphemerisData]:
        """逐行解码 RINEX NAV 文件

        返回值:
          - None: 头部/星历未完成
          - EphemerisData: 一条完整星历
        """
        if not self._header_parsed:
            if self._parse_header_line(line):
                return None
            return None

        # 空行跳过
        if line.strip() == '':
            return None

        # 星历起始行: 卫星ID + 时钟参数
        if line[0] in self.gnss_tbl:
            # 如果有未完成的前一条星历, 丢弃 (行数不足)
            self._current_lines = [line]
            self._current_sys = self.gnss_tbl[line[0]]
            self._current_sys_char = line[0]
            prn = int(line[1:3])
            self._current_sat = f"{line[0]}{prn:02d}"
            return None

        # 星历续行
        if self._current_lines:
            self._current_lines.append(line)

            expected = self.NAV_LINES.get(self._current_sys_char, 8)

            if len(self._current_lines) >= expected:
                result = self._build_ephemeris()
                self._current_lines = []
                return result

        return None

    def _parse_header_line(self, line: str) -> bool:
        """解析 RINEX NAV 头部, 返回 True 表示头未结束
        参考 rtklib-py rnx_decode.decode_nav + pyrinex rinexnav3 头部解析"""
        if len(line) < 60:
            return not self._header_parsed

        label = line[60:].strip()

        if label == 'END OF HEADER':
            self._header_parsed = True
            return False

        if label == 'RINEX VERSION / TYPE':
            self._ver = float(line[4:10])

        elif label == 'IONOSPHERIC CORR':
            # 电离层参数 (参考 rtklib-py)
            if line[0:4] in ('GPSA', 'QZSA'):
                for k in range(4):
                    self._ion[0, k] = self._flt(line, k, 5)
            elif line[0:4] in ('GPSB', 'QZSB'):
                for k in range(4):
                    self._ion[1, k] = self._flt(line, k, 5)

        return True  # 头未结束

    def _build_ephemeris(self) -> EphemerisData:
        """从累积行构建星历对象
        参考 pyrinex: 字段名来自 NAV_FIELDS, 解析参考 rtklib-py"""
        lines = self._current_lines
        sys_char = self._current_sys_char
        fields = self.NAV_FIELDS[sys_char]

        # 累积原始字符串 (参考 pyrinex: raw += line[STARTCOL3:80])
        Lf = 19  # 每字段字符串长度 (参考 pyrinex)
        raw = lines[0][23:80]  # 第一行: 跳过卫星ID和时间
        for ln in lines[1:]:
            raw += ln[4:80] if len(ln) > 4 else ''
        raw = raw.replace('D', 'E').replace('d', 'e')

        # 使用字段名列表解析 (参考 pyrinex: np.genfromtxt)
        # 流式场景下用 _flt 逐字段提取 (参考 rtklib-py)
        values = []
        for i in range(len(fields)):
            s = Lf * i
            e = s + Lf
            if e > len(raw):
                values.append(0.0)
            else:
                try:
                    values.append(float(raw[s:e]))
                except ValueError:
                    values.append(0.0)

        # 构建字段字典 (参考 pyrinex: dsf[f] = d)
        field_dict = dict(zip(fields, values))

        # GLONASS/SBAS: km → m (参考 pyrinex)
        if sys_char in ('R', 'S'):
            for key in ('X', 'dX', 'dX2', 'Y', 'dY', 'dY2', 'Z', 'dZ', 'dZ2'):
                if key in field_dict:
                    field_dict[key] *= 1000

        # 解析时间 (行1 的 year/month/day/hour/minute/second)
        line0 = lines[0]
        year = int(line0[4:8])
        month = int(line0[9:11])
        day = int(line0[12:14])
        hour = int(line0[15:17])
        minute = int(line0[18:20])
        sec = int(line0[21:23])

        # 构建统一输出
        if sys_char in ('G', 'E', 'J', 'C'):
            return self._build_keplerian_eph(field_dict, sys_char)
        else:  # R, S
            return self._build_geph(field_dict, sys_char)

    def _build_keplerian_eph(self, f: dict, sys_char: str) -> EphemerisData:
        """构建开普勒轨道星历 (GPS/GAL/QZS/BDS)"""
        week_key = {'G': 'GPSWeek', 'E': 'GALWeek', 'J': 'GPSWeek', 'C': 'BDTWeek'}
        return EphemerisData(
            satellite_id=self._current_sat,
            system=self.gnss_tbl[sys_char],
            toe_seconds=f.get('Toe', 0),
            week=int(f.get(week_key.get(sys_char, 'GPSWeek'), 0)),
            clock_bias=[f.get('SVclockBias', 0), f.get('SVclockDrift', 0),
                        f.get('SVclockDriftRate', 0)],
            orbit_params={
                'iode': int(f.get('IODE', f.get('IODnav', f.get('AODE', 0)))),
                'iodc': int(f.get('IODC', f.get('AODC', 0))),
                'crs': f.get('Crs', 0), 'crc': f.get('Crc', 0),
                'cuc': f.get('Cuc', 0), 'cus': f.get('Cus', 0),
                'cic': f.get('Cic', 0), 'cis': f.get('Cis', 0),
                'deln': f.get('DeltaN', 0), 'M0': f.get('M0', 0),
                'e': f.get('Eccentricity', 0), 'sqrtA': f.get('sqrtA', 0),
                'i0': f.get('Io', 0), 'idot': f.get('IDOT', 0),
                'OMG0': f.get('Omega0', 0), 'OMGd': f.get('OmegaDot', 0),
                'omg': f.get('omega', 0),
                'toes': f.get('Toe', 0),
                'tgd': [f.get('TGD', f.get('BGDe5a', 0)),
                        f.get('TGD2', f.get('BGDe5b', 0))],
                'sva': f.get('SVacc', f.get('SISA', 0)),
                'svh': int(f.get('health', 0)),
            }
        )

    def _build_geph(self, f: dict, sys_char: str) -> EphemerisData:
        """构建 GLONASS/SBAS 星历 (参考 rtklib-py Geph 分支 + pyrinex 字段名)"""
        return EphemerisData(
            satellite_id=self._current_sat,
            system=self.gnss_tbl[sys_char],
            toe_seconds=0,  # GLONASS 使用 UTC 时间
            week=0,
            clock_bias=[f.get('SVclockBias', 0), f.get('SVrelFreqBias', 0), 0.0],
            orbit_params={
                'type': 'glonass' if sys_char == 'R' else 'sbas',
                'tod': f.get('MessageFrameTime', 0) % 86400,
                'pos': [f.get('X', 0), f.get('Y', 0), f.get('Z', 0)],
                'vel': [f.get('dX', 0), f.get('dY', 0), f.get('dZ', 0)],
                'acc': [f.get('dX2', 0), f.get('dY2', 0), f.get('dZ2', 0)],
                'svh': int(f.get('health', 0)),
                'frq': f.get('FreqNum', 0),
                'age': f.get('AgeOpInfo', 0),
            }
        )

    @staticmethod
    def _flt(line, col, start=4):
        """解析 RINEX 浮点数 (参考 rtklib-py rnx_decode.flt)"""
        s = 19 * col + start
        e = s + 19
        if e > len(line):
            return 0.0
        try:
            return float(line[s:e].replace("D", "E").replace("d", "e"))
        except ValueError:
            return 0.0

    def flush(self) -> Optional[EphemerisData]:
        """文件结束时, 丢弃未完成的星历 (星历不完整不可用)"""
        return None

    def get_timestamp(self, decoded: EphemerisData) -> float:
        return decoded.toe_seconds
```

#### 5.4.5 流式解码 vs 全量解码对比

```
rtklib-py 全量解码:
─────────────────────────────────────────────────
  fobs = open(obsfile)
  for line in fobs:           ← 一次性遍历
      ... 累积到 obslist[]    ← 全量存储
  fobs.close()
  # 后续: rov.obslist[index]  ← 随机访问

本框架流式解码:
─────────────────────────────────────────────────
  Streamer.run():
      while running:
          line = read_line()           ← 逐行读取
          data = formator.decode(line) ← 增量解码
          if data is not None:
              queue.put(data)          ← 推入队列, O(1) 内存
```

| 维度 | rtklib-py 全量 | 本框架流式 |
|------|---------------|-----------|
| 内存 | O(M×K), M=历元数, K=卫星数 | O(1), 仅当前历元 |
| 随机访问 | 支持 (`obslist[i]`) | 不支持, FIFO 队列 |
| Rover-Base 同步 | `next_obs()` 按 index 同步 | Scheduler 层时间对齐 |
| 适用场景 | 小数据集后处理 | 大数据集/实时/流式 |

#### 5.4.6 FormatorBase 接口扩展

为支持 RINEX 等需要累积多行的格式，`FormatorBase` 增加 `flush()` 方法：

```python
class FormatorBase(ABC):
    """自定义文件解码接口 — 用户继承此类实现自己的文本格式解析"""

    @abstractmethod
    def decode(self, line: str):
        """解码一行文本数据。返回解码后的数据对象，解析失败/未完成返回 None"""
        ...

    @abstractmethod
    def get_timestamp(self, decoded) -> float:
        """从解码结果中提取时间戳"""
        ...

    def flush(self):
        """文件读取结束时调用, 返回最后一个未完成的数据包 (如不完整历元)
        默认返回 None, 仅多行累积型 Formator 需要覆写"""
        return None
```

**StreamerBase.run() 中的调用**：

```python
def run(self):
    ...
    while self.control.is_running():
        ...
        line = self.read_line()
        if line is None:
            # 文件读完 → 刷新 Formator 缓冲区
            remaining = self.formator.flush()
            if remaining is not None:
                sensor_data = SensorData(tag=self.tag)
                self._fill_sensor_data(sensor_data, remaining)
                self.output_queue.put(sensor_data, timeout=1.0)
            # 推入 EOF 标记
            self.output_queue.put(EOF_SENSOR_DATA, timeout=1.0)
            break
        ...
```

#### 5.4.7 各 Formator 的累积行为

| Formator | 每条数据对应行数 | 累积方式 | flush() 行为 |
|----------|----------------|---------|-------------|
| `MyIMUFormator` | 1 行 | 无累积 | 返回 None |
| `RinexObsFormator` | N+1 行 (1行历元头 + N行卫星) | `_epoch_obs[]` 累积 | 返回最后一个不完整历元 |
| `RinexNavFormator` | 8 行 (GPS/GAL) / 4 行 (GLO) | `_current_lines[]` 累积 | 丢弃不完整星历 |
| `MyRoverFormator` | N 行 (自定义CSV, 同时间戳为一组) | `_current_epoch[]` 累积 | 返回最后一组 |

### 5.5 各 Streamer 的读取行为

| Streamer | 读取粒度 | 频率 | 暂停/恢复 | 文件结束行为 |
|----------|----------|------|-----------|-------------|
| `IMUStreamer` | 逐行 | 100-400Hz | 可被调度器暂停 | 推入 EOF 标记后退出 |
| `RoverStreamer` | 逐历元(多行) | 1Hz | 可被调度器暂停 | 推入 EOF 标记后退出 |
| `EphStreamer` | 逐行/逐块 | 不定期 | **永不暂停**, 直到读完所有数据 | 推入 EOF 标记后退出 |
| `RefStreamer` | 逐历元(多行) | 1Hz | 可被调度器暂停 | 推入 EOF 标记后退出 |

**Rover/Ref 的多行读取**：一个 GNSS 历元包含多行（每卫星一行），Formator 内部缓存直到一个完整历元解码完成才返回。`read_line()` 每次读一行 → `formator.decode(line)` 累积 → 完整历元时返回 `GnssMeasurement`，否则返回 `None`。已在 `MyRoverFormator` 示例中展示此模式，与 GICI 的增量解码思路一致。

### 5.6 EOF 标记与全局退出机制

当文件读取完毕时，向队列推入特殊标记通知下游：

```python
# 约定: SensorData 中 tag = "EOF" 表示该数据源已结束
EOF_SENSOR_DATA = SensorData(tag="EOF")
```

**全局退出机制**：

```python
class MainController:
    """主控制器，管理全局退出"""
    def __init__(self, streamers, scheduler, estimator, logger):
        self.streamers = streamers
        self.scheduler = scheduler
        self.estimator = estimator
        self.logger = logger
        self.eof_count = 0
        self.total_streams = len(streamers)

    def on_eof(self, tag: str):
        """某个 Streamer 完成文件读取"""
        logger.info(f"数据源结束: {tag}")
        self.eof_count += 1
        if self.eof_count >= self.total_streams:
            logger.info("所有数据源已读取完毕，等待处理完成...")
            self.shutdown_gracefully()

    def shutdown_gracefully(self):
        """优雅退出：等待队列处理完成"""
        # 等待估计队列处理完
        while not self.estimator.estimate_queue.empty():
            time.sleep(0.01)

        # 等待日志队列处理完
        while not self.logger.solution_queue.empty():
            time.sleep(0.01)

        # 通知所有线程退出
        self.control.shutdown()
```

### 5.7 SensorFactory — 工厂模式装配传感器

> 主程序不直接 new 传感器，而是通过 `SensorFactory` 根据配置创建 `list[BaseSensor]`。
> 这样切换数据源（文件→串口→网络）或调整 GNSS 模式（内部/外部）时，主程序无需修改。

```python
class SensorFactory:
    """传感器工厂：根据配置动态创建 BaseSensor 实例列表

    返回抽象类型 list[BaseSensor]，调用方遍历调用 get_data() 即可，
    不感知具体子类（IMUStreamer/GnssSolStreamer 等）。
    """

    @staticmethod
    def create_sensors(config: dict, formators: dict,
                       queues: dict, control: ThreadControl) -> list:
        sensors = []

        # IMU（始终创建，高频默认启用异步 IO）
        sensors.append(IMUStreamer(
            file_path=config["files"]["imu"],
            formator=formators["imu"],
            output_queue=queues["imu"],
            control=control, tag="imu",
            playback_mode=config.get("playback_mode", "asap"),
            speed=config.get("speed", 100.0),
        ))

        if config.get("gnss_source", "internal") == "internal":
            sensors.append(RoverStreamer(..., tag="rover"))
            sensors.append(EphStreamer(..., tag="eph"))
            if config.get("mode") == "differential":
                sensors.append(RefStreamer(..., tag="ref"))
        else:
            sensors.append(GnssSolStreamer(
                file_path=config["gnss_external"]["solution_file"],
                formator=formators["gnss_sol"],
                output_queue=queues["gnss_sol"],
                control=control, tag="gnss_sol",
            ))
        return sensors
```

### 5.8 数据通路 — 纯队列流水线（替代观察者模式）

> 本项目**不使用观察者模式**。传感器数据通过 `output_queue.put()` 传递给 Scheduler，
> Scheduler 转发到 `estimate_queue`，Estimator 消费后写入 `solution_queue`，Logger 输出。
> 全程无回调，无 `attach/detach/notify`，无 `on_data`。

```python
# 装配示例：传感器启动读取线程，数据写入各自队列
sensors = SensorFactory.create_sensors(config, formators, queues, control)
for s in sensors:
    s.start()          # 启动读取线程，run() 中调用 output_queue.put(sensor_data)

# Scheduler 从各 output_queue 拉取数据，仅转发到 estimate_queue
scheduler = Scheduler(queues, estimate_queue, control, config)
scheduler.start()

# Estimator 从 estimate_queue 拉取，内部完成时间对齐 + EKF 融合，结果写入 solution_queue
estimator = LcEstimator(estimate_queue, solution_queue, config)
estimator.start()

# Logger 从 solution_queue 拉取结果输出
logger = Logger(solution_queue, writers, config)
logger.start()
```

> **时间对齐下沉到 Estimator**：Scheduler 仅做转发，不做时间对齐。
> 时间对齐由 Estimator 内部处理（参考 KF-GINS `newImuProcess()` + `imuInterpolate()`），
> 维护 imupre/imucur 和 pending_gnss（deque 缓冲），处理 4 种时间对齐情况。

---

## 6. 数据集成层 (Integration)

### 6.1 初始化状态枚举

```python
from enum import Enum, auto

class InitState(Enum):
    """初始化状态机状态"""
    NOT_INITIALIZED = auto()      # 未初始化，判断模式
    ROVER_REF_SYNCING = auto()    # 差分模式：Rover-Ref 时间同步
    IMU_CATCHING_UP = auto()      # IMU 追赶 GNSS
    GNSS_ONLY = auto()            # 纯 GNSS 解算，等待 IMU
    INITIALIZING = auto()         # 执行初始化
    INITIALIZED = auto()          # 正常融合
```

### 6.2 星历缓冲区

```python
from collections import defaultdict
from typing import Dict, List

class EphemerisBuffer:
    """星历缓冲区，按卫星ID存储，支持时间查询"""

    def __init__(self):
        # sat_id -> list[Ephemeris]，按 toe 排序
        self._buffer: Dict[str, List[Ephemeris]] = defaultdict(list)
        self._max_per_sat = 10  # 每颗卫星最多保留 10 条星历

    def update(self, eph_data: EphemerisData):
        """更新星历缓冲区"""
        for eph in eph_data.ephemerides:
            sat_list = self._buffer[eph.satellite_id]
            sat_list.append(eph)
            # 按 toe 排序
            sat_list.sort(key=lambda x: x.toe)
            # 限制大小
            if len(sat_list) > self._max_per_sat:
                sat_list.pop(0)

    def get_near(self, timestamp: float, max_age: float = 7200.0) -> Optional[EphemerisData]:
        """获取最接近指定时间的星历"""
        result = []
        for sat_id, sat_list in self._buffer.items():
            if not sat_list:
                continue
            # 二分查找最接近的星历
            best = min(sat_list, key=lambda x: abs(x.toe - timestamp))
            if abs(best.toe - timestamp) <= max_age:
                result.append(best)

        if result:
            return EphemerisData(timestamp=timestamp, ephemerides=result)
        return None
```

### 6.3 Scheduler — 调度器

调度器是框架的核心，运行在独立线程中，负责：

1. 从各传感器队列取数据
2. 管理初始化状态机
3. 执行时间对齐
4. 将对齐后的数据推入估计队列

```python
class Scheduler:
    def __init__(self, imu_queue, rover_queue, eph_queue, ref_queue,
                 estimate_queue, streamers, control, options):
        self.imu_queue = imu_queue
        self.rover_queue = rover_queue
        self.eph_queue = eph_queue
        self.ref_queue = ref_queue
        self.estimate_queue = estimate_queue
        self.streamers = streamers          # 各 Streamer 引用, 用于暂停/恢复
        self.control = control
        self.options = options

        # 状态
        self.state = InitState.NOT_INITIALIZED
        self.latest_imu_time = -float('inf')
        self.latest_gnss_time = -float('inf')

        # 星历缓存 (累积式, 不按时间对齐)
        self.ephemeris_buffer = EphemerisBuffer()

        from collections import deque
        # IMU 缓冲区 (时间对齐用)
        self.imu_buffer = deque()                # deque[ImuMeasurement]

        # 流动站缓冲区 (时间对齐用)
        self.rover_buffer = deque()              # deque[GnssMeasurement]
        # 基站缓冲区 (时间对齐用)
        self.ref_buffer = deque()                # deque[ReferenceMeasurement]

        # Rover-Ref 同步阈值
        self.rover_ref_sync_threshold = options.scheduler.rover_ref_sync_threshold

        # 差分模式基站数据超时
        self._ref_sync_start_time = None
        self.ref_data_timeout = options.scheduler.get('ref_data_timeout', 30.0)  # 基站数据超时 (秒)

    def run(self):
        """调度线程主循环"""
        while self.control.is_running():
            # 1. 优先处理星历 (累积式, 不参与时间对齐, 永不暂停)
            self._drain_eph_queue()

            # 2. 根据初始化状态执行不同逻辑
            if self.state == InitState.NOT_INITIALIZED:
                self._handle_not_initialized()
            elif self.state == InitState.ROVER_REF_SYNCING:
                self._handle_rover_ref_syncing()
            elif self.state == InitState.GNSS_ONLY:
                self._handle_gnss_only()
            elif self.state == InitState.IMU_CATCHING_UP:
                self._handle_imu_catching_up()
            elif self.state == InitState.INITIALIZING:
                self._handle_initializing()
            elif self.state == InitState.INITIALIZED:
                self._handle_initialized()

            # 3. 避免空转: 所有队列都为空时 sleep
            if (self.imu_queue.empty() and self.rover_queue.empty() and
                self.ref_queue.empty() and self.eph_queue.empty()):
                time.sleep(0.001)  # 1ms

    def _drain_eph_queue(self):
        """清空星历队列, 累积到星历缓冲区"""
        while not self.eph_queue.empty():
            try:
                data = self.eph_queue.get_nowait()
                if data.ephemeris:
                    self.ephemeris_buffer.update(data.ephemeris)
            except queue.Empty:
                break

    def _drain_imu_queue(self):
        """清空 IMU 队列到 imu_buffer"""
        while not self.imu_queue.empty():
            try:
                data = self.imu_queue.get_nowait()
                if data.imu:
                    self.imu_buffer.append(data.imu)
                    self.latest_imu_time = max(self.latest_imu_time, data.imu.timestamp)
            except queue.Empty:
                break

    def _drain_rover_queue(self):
        """清空 Rover 队列到 rover_buffer"""
        while not self.rover_queue.empty():
            try:
                data = self.rover_queue.get_nowait()
                if data.gnss:
                    self.rover_buffer.append(data.gnss)
            except queue.Empty:
                break

    def _drain_ref_queue(self):
        """清空 Ref 队列到 ref_buffer"""
        while not self.ref_queue.empty():
            try:
                data = self.ref_queue.get_nowait()
                if data.reference:
                    self.ref_buffer.append(data.reference)
            except queue.Empty:
                break
```

### 6.4 初始化状态机

```
                    ┌──────────────────┐
                    │  NOT_INITIALIZED │
                    │  (未初始化)       │
                    └────────┬─────────┘
                             │
                  读取配置文件判断模式
                             │
              ┌──────────────┴──────────────┐
              │                             │
     mode="differential"              mode="spp"
     (差分模式)                       (单点模式)
              │                             │
              ▼                             │
    ┌─────────────────────┐                │
    │ ROVER_REF_SYNCING   │                │
    │ (Rover-Ref同步)      │                │
    │                     │                │
    │ Rover超前→暂停Rover │                │
    │ Ref超前→暂停Ref     │                │
    │ 落后者追赶直到      │                │
    │ 时间差<阈值         │                │
    │ 超时未收到Ref则报错 │                │
    └────────┬────────────┘                │
             │                             │
     Rover与Ref时间已同步                   │
             │                             │
             └──────────────┬──────────────┘
                            │
                  取GNSS时间与IMU时间比较
                            │
              ┌─────────────┼─────────────┐
              │                           │
     IMU时间 < GNSS时间          GNSS时间 <= IMU时间
              │                           │
              ▼                           ▼
    ┌──────────────────┐        ┌──────────────────┐
    │  IMU_CATCHING_UP │        │  GNSS_ONLY 模式   │
    │  暂停Rover/Ref   │        │  IMU不暂停读取    │
    │  持续读IMU       │        │  纯GNSS解算       │
    │  直到IMU时间>=   │        │  检测IMU到达后    │
    │  GNSS时间        │        │  进入初始化       │
    └────────┬─────────┘        └────────┬─────────┘
             │                           │
             └──────────────┬────────────┘
                            │
                   IMU和GNSS时间已匹配
                            │
                            ▼
                   ┌──────────────────┐
                   │  INITIALIZING    │
                   │  (执行初始化)     │
                   └────────┬─────────┘
                            │
                     初始化成功
                            │
                            ▼
                   ┌──────────────────┐
                   │  INITIALIZED     │
                   │  (正常融合)       │
                   └──────────────────┘
```

**关键设计**：
1. **差分/单点模式由配置文件决定**，不靠数据判断
2. **Rover-Ref 同步采用暂停/追赶策略**：超前方暂停、落后方追赶，不丢弃数据。在 `asap` 模式下追赶极快
3. 差分模式下长时间未收到基站数据将报错提示用户检查数据源

### 6.5 各状态的处理逻辑

#### NOT_INITIALIZED — 初始探测

```python
def _handle_not_initialized(self):
    """初始状态: 根据配置文件判断模式, 决定初始化路径"""

    if self.options.mode == "differential":
        # 差分模式: 先进入 Rover-Ref 同步 (不需要等 IMU)
        self._drain_rover_queue()
        self._drain_ref_queue()
        if len(self.rover_buffer) > 0 and len(self.ref_buffer) > 0:
            self.state = InitState.ROVER_REF_SYNCING
            self._ref_sync_start_time = time.time()  # 记录同步开始时间
    else:
        # 单点模式: 直接比较 IMU 和 GNSS 时间
        self._drain_imu_queue()
        self._drain_rover_queue()
        if len(self.imu_buffer) > 0 and len(self.rover_buffer) > 0:
            self._compare_imu_gnss_time()
```

#### ROVER_REF_SYNCING — Rover-Ref 时间同步

```python
def _handle_rover_ref_syncing(self):
    """差分模式: 先同步 Rover 和 Ref 的时间, 再与 IMU 比较
    核心策略: 暂停超前的线程, 让落后的线程追赶, 避免丢弃数据"""

    # 超时检查: 差分模式下长时间未收到基站数据
    if self._ref_sync_start_time is not None:
        elapsed = time.time() - self._ref_sync_start_time
        if elapsed > self.ref_data_timeout and len(self.ref_buffer) == 0:
            logger.error("差分模式但未收到基站数据，请检查基站数据源 (已等待 %.1f 秒)", elapsed)

    # 持续消费 Rover 和 Ref 队列
    self._drain_rover_queue()
    self._drain_ref_queue()

    if len(self.rover_buffer) == 0 or len(self.ref_buffer) == 0:
        # 数据不足: 恢复两个线程读取, 等待数据到达
        self._resume_rover_ref_readers()
        return

    # 检查 Rover 和 Ref 的时间差
    rover_time = self.rover_buffer[-1].timestamp  # 最新 Rover 时间
    ref_time = self.ref_buffer[-1].timestamp      # 最新 Ref 时间
    dt = rover_time - ref_time

    if abs(dt) <= self.rover_ref_sync_threshold:
        # Rover 和 Ref 时间已同步 → 恢复两个线程, 取较早者作为 GNSS 时间
        self._resume_rover_ref_readers()
        gnss_time = min(self.rover_buffer[0].timestamp, self.ref_buffer[0].timestamp)
        self.latest_gnss_time = gnss_time
        self._compare_imu_gnss_time()

    elif dt > self.rover_ref_sync_threshold:
        # Rover 超前于 Ref → 暂停 Rover, 让 Ref 追赶
        self.streamers['rover'].pause()
        self.streamers['ref'].resume()

    else:  # dt < -rover_ref_sync_threshold
        # Ref 超前于 Rover → 暂停 Ref, 让 Rover 追赶
        self.streamers['ref'].pause()
        self.streamers['rover'].resume()
```

**同步策略详解**：

```
情况1: Rover 超前 (rover_time > ref_time + 阈值)
──────────────────────────────────────────────────
时间轴: ──────────────────────────────────────→
Rover:  ... [t=10.0] [11.0] [12.0]  ← 超前, 暂停读取
Ref:    ... [t=10.0] [10.5]         ← 落后, 继续读取追赶

操作: 暂停 Rover, 恢复 Ref
结果: Ref 持续读取直到追上 Rover

情况2: Ref 超前 (ref_time > rover_time + 阈值)
──────────────────────────────────────────────────
时间轴: ──────────────────────────────────────→
Rover:  ... [t=10.0] [10.5]         ← 落后, 继续读取追赶
Ref:    ... [t=10.0] [11.0] [12.0]  ← 超前, 暂停读取

操作: 暂停 Ref, 恢复 Rover
结果: Rover 持续读取直到追上 Ref

情况3: 已同步 (|rover_time - ref_time| <= 阈值)
──────────────────────────────────────────────────
操作: 恢复两者, 进入 IMU-GNSS 时间比较
```

**为什么不丢弃数据**：旧方案在时间差过大时丢弃超前方的旧数据，但这会丢失有效观测值。暂停超前线程、让落后线程追赶的策略保留了所有数据，且在 `asap` 模式下追赶速度极快（CPU 全速读取），不会造成明显延迟。

#### _compare_imu_gnss_time — IMU 与 GNSS 时间比较

```python
def _compare_imu_gnss_time(self):
    """比较 IMU 时间和 GNSS 时间, 决定进入哪个状态"""
    if len(self.imu_buffer) == 0 or len(self.rover_buffer) == 0:
        return  # 数据不足, 继续等待

    imu_time = self.imu_buffer[-1].timestamp
    gnss_time = self.rover_buffer[0].timestamp  # 同步后的 GNSS 时间

    if imu_time < gnss_time:
        # IMU 时间早于 GNSS → 进入 IMU 追赶模式
        self.state = InitState.IMU_CATCHING_UP
        self._pause_rover_ref_readers()  # 暂停 Rover 和 Ref 读取
        self._resume_imu_readers()
    else:
        # GNSS 时间早于或等于 IMU → 进入纯 GNSS 模式
        # 注意: 不暂停 IMU 读取, 持续读取用于检测 IMU 到达
        self.state = InitState.GNSS_ONLY
        self._resume_imu_readers()
        self._resume_rover_ref_readers()
```

#### IMU_CATCHING_UP — IMU 追赶模式

```python
def _handle_imu_catching_up(self):
    """IMU 追赶模式: 持续读 IMU, 暂停 Rover/Ref, 直到时间匹配"""

    # 持续消费 IMU 队列
    self._drain_imu_queue()

    if len(self.imu_buffer) == 0 or len(self.rover_buffer) == 0:
        return

    imu_time = self.imu_buffer[-1].timestamp
    gnss_time = self.rover_buffer[0].timestamp  # 同步后的 GNSS 时间

    if imu_time >= gnss_time:
        # IMU 已追上 GNSS → 恢复 Rover/Ref 读取, 进入初始化
        self._resume_rover_ref_readers()
        self._resume_imu_readers()
        self.state = InitState.INITIALIZING
```

#### GNSS_ONLY — 纯 GNSS 解算模式

```python
def _handle_gnss_only(self):
    """纯 GNSS 模式: 仅用 GNSS 做 SPP 解算, 等待 IMU 到达"""

    # 处理 Rover 和 Ref 数据
    self._drain_rover_queue()
    self._drain_ref_queue()

    # 将 GNSS 数据推入估计队列 (标记为 SPP 模式)
    for rover_meas in self.rover_buffer:
        data = SensorData(tag="rover")
        data.gnss = rover_meas
        data.ephemeris = self.ephemeris_buffer.get_near(rover_meas.timestamp)
        data.reference = self._find_ref_near(rover_meas.timestamp)
        self.estimate_queue.put(data)
    self.rover_buffer.clear()

    # 检查 IMU 是否已到达 (IMU 从未暂停, 持续读取)
    self._drain_imu_queue()
    if len(self.imu_buffer) > 0:
        imu_time = self.imu_buffer[-1].timestamp
        gnss_time = self.latest_gnss_time
        if imu_time >= gnss_time:
            # IMU 已到达 → 进入初始化
            self.state = InitState.INITIALIZING
```

#### INITIALIZING — 初始化阶段

```python
def _handle_initializing(self):
    """初始化阶段: 收集 IMU 和 GNSS 数据, 执行初始化"""

    self._drain_imu_queue()
    self._drain_rover_queue()
    self._drain_ref_queue()

    # 更新 latest_imu_time
    if len(self.imu_buffer) > 0:
        self.latest_imu_time = self.imu_buffer[-1].timestamp

    # 检查是否有足够的 IMU 数据 (至少 init_time_window 秒)
    if len(self.imu_buffer) == 0 or len(self.rover_buffer) == 0:
        return

    imu_duration = self.imu_buffer[-1].timestamp - self.imu_buffer[0].timestamp
    if imu_duration < self.options.scheduler.init_time_window:
        return  # 继续收集 IMU 数据

    # 检查 GNSS 时间是否已覆盖
    gnss_time = self.rover_buffer[-1].timestamp
    if gnss_time > self.latest_imu_time:
        return  # 等待 IMU 追上

    # 数据充足, 推入估计队列进行初始化
    for imu_meas in self.imu_buffer:
        data = SensorData(tag="imu")
        data.imu = imu_meas
        self.estimate_queue.put(data)
    self.imu_buffer.clear()

    for rover_meas in self.rover_buffer:
        data = SensorData(tag="rover")
        data.gnss = rover_meas
        data.ephemeris = self.ephemeris_buffer.get_near(rover_meas.timestamp)
        data.reference = self._find_ref_near(rover_meas.timestamp)
        self.estimate_queue.put(data)
    self.rover_buffer.clear()

    # 初始化完成后进入正常融合模式
    self.state = InitState.INITIALIZED
```

#### INITIALIZED — 正常融合模式

```python
def _handle_initialized(self):
    """正常融合模式: 简化为直接转发

    时间同步逻辑由估计器内部处理（参考 estimator.md 第9节）
    Scheduler 只负责将数据按到达顺序推入估计队列
    """

    # 取出所有队列数据
    self._drain_imu_queue()
    self._drain_rover_queue()
    self._drain_ref_queue()

    # IMU 数据直接推入估计器（估计器内部维护 imupre/imucur）
    for imu_meas in self.imu_buffer:
        data = SensorData(tag="imu")
        data.imu = imu_meas
        self.estimate_queue.put(data)
    self.imu_buffer.clear()

    # GNSS 数据直接推入估计器（估计器内部处理时间对齐）
    # 估计器内部暂存为 pending_gnss (deque 缓冲，避免丢失多个 GNSS 历元)，在 IMU 处理时同步消费
    # 4 种时间对齐情况由估计器的 _is_to_update() 判断
    for rover_meas in self.rover_buffer:
        data = SensorData(tag="gnss_solution")
        data.gnss = rover_meas
        data.ephemeris = self.ephemeris_buffer.get_near(rover_meas.timestamp)
        data.reference = self._find_ref_near(rover_meas.timestamp)
        self.estimate_queue.put(data)
    self.rover_buffer.clear()
```

**设计变更说明**：

原方案中 Scheduler 负责时间对齐（检查 Rover 时间戳是否被 IMU 覆盖），新方案将时间同步逻辑下沉到估计器：

| 维度 | 原方案 | 新方案 |
|------|--------|--------|
| **时间对齐位置** | Scheduler 层 | 估计器内部 |
| **IMU 缓冲区** | Scheduler 维护 imu_buffer | 估计器维护 imupre/imucur |
| **GNSS 暂存** | Scheduler 检查时间覆盖后推入 | 估计器暂存为 pending_gnss |
| **IMU 插值** | Scheduler 层插值 | 估计器增量切分（参考 KF-GINS） |
| **Scheduler 职责** | 时间对齐 + 转发 | 仅转发 |

**优势**：
1. 估计器内部维护完整的 IMU 状态（imupre/imucur），可精确执行增量切分
2. 避免Scheduler 和估计器之间的状态同步问题
3. 与 KF-GINS 的单线程处理逻辑一致，更易于理解和维护

### 6.6 Streamer 暂停/恢复控制

调度器通过调用 Streamer 的 `pause()` / `resume()` 方法控制读取节奏：

```python
# Rover 和 Ref 同步暂停/恢复 (用于 IMU_CATCHING_UP 等状态)
def _pause_rover_ref_readers(self):
    self.streamers['rover'].pause()
    self.streamers['ref'].pause()

def _resume_rover_ref_readers(self):
    self.streamers['rover'].resume()
    self.streamers['ref'].resume()

# Rover 和 Ref 独立控制 (用于 ROVER_REF_SYNCING 暂停/追赶策略)
def _pause_rover_resume_ref(self):
    """Rover 超前: 暂停 Rover, 让 Ref 追赶"""
    self.streamers['rover'].pause()
    self.streamers['ref'].resume()

def _pause_ref_resume_rover(self):
    """Ref 超前: 暂停 Ref, 让 Rover 追赶"""
    self.streamers['ref'].pause()
    self.streamers['rover'].resume()

# IMU 控制
def _pause_imu_readers(self):
    self.streamers['imu'].pause()

def _resume_imu_readers(self):
    self.streamers['imu'].resume()
```

**注意**：
- 星历读取器 (`EphStreamer`) **永不暂停**，因为星历是累积式数据，需要持续加载直到文件读完
- Rover 和 Ref 读取器在 ROVER_REF_SYNCING 状态下**独立控制**：超前方暂停、落后方追赶
- 在其他状态（如 IMU_CATCHING_UP）下 Rover 和 Ref **同步暂停/恢复**，确保差分观测值的时间一致性

---

## 7. 估计融合层 (Estimate)

### 7.1 估计器接口

```python
class EstimatorBase(ABC):
    """估计器基类 — 定义统一接口"""

    @abstractmethod
    def add_measurement(self, data: SensorData) -> bool:
        """添加量测数据, 返回是否成功"""
        ...

    @abstractmethod
    def estimate(self) -> Optional[Solution]:
        """执行估计, 返回解算结果 (无解时返回 None)"""
        ...

    def set_initialization_result(self, result):
        """设置初始化结果 (由初始化器调用)"""
        ...
```

### 7.2 估计器层次

```
EstimatorBase
├── SPPEstimator          — 单点定位 (纯 GNSS, 初始化前使用)
├── GnssImuInitializer    — GNSS/IMU 联合初始化器
└── LcEstimator           — INS/GNSS 松组合估计器 (初始化后使用)
    └── 内部维护 imupre/imucur, pending_gnss
        时间同步与 IMU 插值由估计器内部处理（参考 estimator.md 第9节）
```

### 7.3 估计线程

```python
class EstimatorThread:
    def __init__(self, estimate_queue, solution_queue, control, options):
        self.estimate_queue = estimate_queue
        self.solution_queue = solution_queue
        self.control = control
        self.options = options

        # 根据阶段选择估计器
        self.spp_estimator = SPPEstimator(options)
        self.initializer = GnssImuInitializer(options)
        self.fusion_estimator = None  # 初始化后创建
        self.initialized = False

    def run(self):
        while self.control.is_running():
            try:
                data = self.estimate_queue.get(timeout=0.01)
            except Empty:
                continue

            if not self.initialized:
                # 初始化阶段
                if self.initializer.add_measurement(data):
                    result = self.initializer.estimate()
                    if result is not None:
                        # 初始化成功, 创建融合估计器
                        self.fusion_estimator = LcEstimator(self.options)
                        self.fusion_estimator.set_initialization_result(result)
                        self.initialized = True
                else:
                    # 纯 GNSS SPP 解算
                    if data.gnss is not None:
                        self.spp_estimator.add_measurement(data)
                        solution = self.spp_estimator.estimate()
                        if solution:
                            self.solution_queue.put(solution)
            else:
                # 正常融合阶段：根据数据类型分发到估计器
                # 时间同步与 IMU 插值由估计器内部处理（参考 estimator.md 第9节）
                solution = None
                if data.tag == "imu":
                    solution = self.fusion_estimator.add_imu(data.imu)
                elif data.tag == "gnss_solution":
                    solution = self.fusion_estimator.add_gnss(data.gnss_solution)

                if solution:
                    self.solution_queue.put(solution)
```

**估计线程的设计要点**：

1. **数据类型分发**：根据 `data.tag` 将数据分发到估计器的 `add_imu()` 或 `add_gnss()` 方法
2. **时间同步下沉**：时间同步逻辑由估计器内部处理，估计线程只负责转发
3. **IMU 数据触发处理**：IMU 数据到达时调用 `add_imu()`，触发 `_new_imu_process()` 完整处理流程
4. **GNSS 数据暂存**：GNSS 数据到达时调用 `add_gnss()`，仅暂存为 `pending_gnss`，不立即处理

---

## 8. 日志层 (Log)

### 8.1 日志类型

| 日志类型 | 内容 | 输出目标 |
|----------|------|----------|
| **解算日志** | Solution 结果 | 文件 + 终端 |
| **原始数据日志** | 传感器原始数据 | 文件 (可选) |
| **运行日志** | 状态转换、警告、错误 | 终端 + 文件 |

### 8.2 Logger 线程

```python
class Logger:
    def __init__(self, solution_queue, raw_log_queue, control, output_dir):
        self.solution_queue = solution_queue
        self.raw_log_queue = raw_log_queue
        self.control = control
        self.output_dir = output_dir
        self._solution_file = None

    def run(self):
        self._solution_file = open(f"{self.output_dir}/solution.txt", 'w')
        while self.control.is_running():
            # 处理解算结果
            try:
                solution = self.solution_queue.get(timeout=0.01)
                self._write_solution(solution)
            except Empty:
                pass

            # 处理原始数据日志 (可选)
            self._drain_raw_log()

        self._solution_file.close()

    def _write_solution(self, sol: Solution):
        line = (f"{sol.timestamp:.6f} "
                f"{' '.join(f'{v:.6f}' for v in sol.position_enu)} "
                f"{sol.status} {sol.num_satellites}\n")
        self._solution_file.write(line)
        self._solution_file.flush()
```

---

## 9. 时间对齐与初始化策略

### 9.1 时间对齐原则

| 传感器 | 对齐策略 | 原因 |
|--------|----------|------|
| **IMU** | 时间传播传感器，直接推入估计器 | IMU 是时间基准，数据必须按序即时处理 |
| **Rover** | 直接推入估计器，由估计器内部处理时间对齐 | 估计器维护 imupre/imucur，执行增量切分（参考 estimator.md 第9节） |
| **星历** | 累积式，不参与时间对齐 | 星历是辅助数据，不依赖时间戳严格对齐 |
| **Ref** | 先与 Rover 同步，再与 Rover 配对后推入 | RTK 差分需要 Rover+Ref 时间匹配 |

**时间同步逻辑下沉到估计器**：

原方案中 Scheduler 负责时间对齐（检查 Rover 时间戳是否被 IMU 覆盖），新方案将时间同步逻辑下沉到估计器内部：

- **估计器维护 imupre/imucur**：完整的 IMU 状态历史
- **GNSS 暂存为 pending_gnss**：在 IMU 处理时同步消费
- **4 种时间对齐情况**：参考 KF-GINS 的 `isToUpdate()` 判断
- **增量切分**：GNSS 时刻在两个 IMU 之间时，切分 IMU 增量而非弹出

详细实现参考 [estimator.md 第 9 节 时间同步与 IMU 插值](file:///e:/program_project/python/GInsStream/skills/estimator.md#9-时间同步与-imu-插值)。

### 9.2 初始化策略详解

#### 场景0: Rover 与 Ref 时间未同步 (差分模式前置步骤)

```
时间轴: ─────────────────────────────────────────→

Rover:  [t=10.0] [11.0] [12.0] ...
Ref:                      [t=12.05] [13.05] ...

阶段1 (NOT_INITIALIZED):
  读取配置: mode="differential"
  读取 Rover(t=10.0) 和 Ref(t=12.05)
  配置为差分模式 → 进入 ROVER_REF_SYNCING

阶段2 (ROVER_REF_SYNCING):
  rover_time(10.0) < ref_time(12.05) - 阈值(0.05)
  → Ref 超前, 暂停 Ref, Rover 继续读取追赶
  Rover 持续读取: [10.0] [11.0] [12.0]
  当 rover_time(12.0) 与 ref_time(12.05) 接近:
  |12.0 - 12.05| = 0.05 <= 阈值 → 同步成功!
  恢复两者读取
  GNSS时间 = min(12.0, 12.05) = 12.0

阶段3: 同步后比较 GNSS 时间与 IMU 时间 → 进入后续状态
```

#### 场景1: IMU 时间早于 GNSS 时间 (Rover-Ref 已同步后)

```
时间轴: ─────────────────────────────────────────→

IMU:   [t=0.0] [0.01] [0.02] ... [0.98] [0.99] [1.00] ...
Rover:                                         [t=1.0] ...
Ref:                                            [t=1.02] ...

阶段1 (NOT_INITIALIZED → ROVER_REF_SYNCING):
  Rover(t=1.0) 和 Ref(t=1.02): |1.0-1.02|=0.02 < 阈值(0.05) → 同步成功
  GNSS时间 = 1.0

阶段2 (IMU_CATCHING_UP):
  IMU时间(0.0) < GNSS时间(1.0)
  暂停 Rover/Ref 读取
  持续读取 IMU 数据
  IMU 缓冲区: [0.0, 0.01, 0.02, ..., 0.99, 1.00, ...]
  当 IMU 最新时间 >= GNSS 时间 → 追赶完成

阶段3 (INITIALIZING):
  恢复 Rover/Ref 读取
  使用 [0.0~1.0] 的 IMU 数据 + Rover(t=1.0) + Ref(t=1.02) 执行初始化
  初始化成功 → 进入 INITIALIZED
```

#### 场景2: GNSS 时间早于 IMU 时间 (Rover-Ref 已同步后)

```
时间轴: ─────────────────────────────────────────→

Rover:  [t=0.0] [1.0] [2.0] ... [9.0] [10.0] ...
Ref:    [t=0.02] [1.02] [2.02] ... [9.02] [10.02] ...
IMU:                                                   [t=10.0] ...

阶段1 (NOT_INITIALIZED → ROVER_REF_SYNCING):
  Rover(t=0.0) 和 Ref(t=0.02): |0.0-0.02|=0.02 < 阈值 → 同步成功
  GNSS时间 = 0.0

阶段2 (GNSS_ONLY):
  GNSS时间(0.0) < IMU时间(10.0)
  IMU 读取不暂停, 持续读取用于检测 IMU 到达
  持续读取 Rover/Ref 数据, 执行 SPP/RTD 解算
  输出: SPP 解算结果 (位置, 精度米级)

阶段3 (当 IMU 数据到达后):
  IMU 最新时间 >= GNSS 最新时间
  进入 INITIALIZING

阶段4 (INITIALIZING):
  使用 SPP/RTD 结果 + IMU 数据执行初始化
  初始化成功 → 进入 INITIALIZED
```

### 9.3 时间对齐缓冲区管理

```
                    ┌─────────────────────────────────┐
                    │         Scheduler 缓冲区         │
                    │                                  │
  IMU 数据流 ──→   │  imu_buffer: [t0, t1, ..., tn]  │
                    │  (deque, 按时间戳有序)           │
                    │                                  │
  Rover 数据流 ──→ │  rover_buffer: [t0, t1, ..., tm]│
                    │  (deque, 按时间戳有序)           │
                    │                                  │
  Ref 数据流 ──→   │  ref_buffer: [t0, t1, ..., tk]  │
                    │  (deque, 按时间戳有序)           │
                    │                                  │
                    │  ephemeris_buffer:               │
                    │  (dict[sat_id -> Ephemeris])     │
                    │  (覆盖更新, 不按时间排序)         │
                    └─────────────────────────────────┘

内存管理:
  - imu_buffer: 初始化后, 每次推入估计器时 pop_front, 保留窗口内数据
  - rover_buffer: 推入估计器后 pop_front
  - ref_buffer: 与 rover 配对后 pop_front
  - ephemeris_buffer: 固定大小, 新星历覆盖旧星历
```

### 9.4 基站与流动站配对

```python
def _find_ref_near(self, rover_time: float, max_age: float = 0.05
                   ) -> Optional[ReferenceMeasurement]:
    """找到与流动站时间戳最接近的基站观测值"""
    best_ref = None
    best_dt = float('inf')
    for ref in self.ref_buffer:
        dt = abs(ref.timestamp - rover_time)
        if dt < best_dt:
            best_dt = dt
            best_ref = ref
    if best_ref is not None and best_dt <= max_age:
        self.ref_buffer.remove(best_ref)
        return best_ref
    return None
```

---

## 10. 目录结构

> **说明**：以下为实际实现的 `src/stream/` 目录结构（与 GInsStream.md 第 4 节整体结构对应）。✅ 标记已实现，🚧 标记预留。

```
gipylib/
├── skills/
│   └── StreamDesign.md          # 本设计文档
│
├── data/
│   └── config.yaml              # 统一配置文件
│
├── src/
│   ├── main.py                  # ✅ 主入口（三种运行模式装配：路径 A/B/C）
│   │
│   ├── stream/                  # ✅ 传感器抽象层 + 流式读取层
│   │   ├── __init__.py
│   │   ├── base.py              # ✅ BaseSensor（抽象基类）+ StreamerBase（流式读取基类，继承 BaseSensor + Thread）
│   │   ├── factory.py           # ✅ SensorFactory 工厂模式动态创建传感器
│   │   ├── formators.py         # ✅ FormatorBase + ImuFormator (GPST) + EuRoCImuFormator (EuRoC) + PosSolFormator（解码器统一在此文件）
│   │   ├── imu_sensor.py        # ✅ ImuSensor IMU 传感器线程（继承 StreamerBase）
│   │   ├── gnss_sol_sensor.py   # ✅ GnssSolSensor 外部 GNSS 结果传感器线程（继承 StreamerBase）
│   │   ├── internal_gnss_sensor.py # ✅ InternalGnssSensor 内部 GNSS 解算传感器线程（直接 Thread 子类）
│   │   ├── rover_sensor.py      # 🚧 预留：GnssRoverSensor 流动站传感器（当前由 InternalGnssSensor 内部 RINEX 加载）
│   │   ├── eph_sensor.py        # 🚧 预留：EphSensor 星历传感器（同上）
│   │   └── ref_sensor.py        # 🚧 预留：GnssRefSensor 基准站传感器（同上）
│   │
│   ├── core/                    # 核心解算
│   │   ├── thread_control.py    # ✅ ThreadControl 线程控制
│   │   ├── time_utils.py        # ✅ 时间转换（gpst_to_unix / unix_to_gpst / ymdhms_to_gpst）
│   │   ├── data_types.py        # ✅ 核心数据类型（ImuMeasurement / GnssSolution / SensorData / AlignedBlock）
│   │   └── gnss/                # ✅ GNSS 解算模块（含 rtklib/ 吸收子包）
│   │
│   ├── log/                     # ✅ 日志层
│   │   ├── logger.py            # ✅ Logger（external+on / internal+on 模式，消费 imu_queue + gnss_queue）
│   │   ├── solution_logger.py   # ✅ SolutionLogger（internal+off 模式，仅消费 gnss_queue）
│   │   ├── writer_base.py       # ✅ WriterBase 输出器抽象基类
│   │   ├── solution_writer.py   # ✅ SolutionWriter rtklib 风格 .pos 输出
│   │   ├── aligned_writer.py    # ✅ AlignedWriter 对齐块状 CSV 输出
│   │   └── aligner.py           # ✅ Aligner IMU 积攒 + GNSS 收割的匹配器
│   │
│   └── utility/                 # ✅ 工具
│       ├── config_loader.py     # ✅ 配置解析（data/config.yaml）
│       └── rinex_simplifier.py  # ✅ RINEX 简化器
│
└── tests/                       # 测试
    └── test_gnss/               # ✅ GNSS 模块测试
```

**与早期设计的差异**：
- `formators/` 目录已合并为单文件 `formators.py`（含 `FormatorBase` + `ImuFormator` (GPST) + `EuRoCImuFormator` (EuRoC) + `PosSolFormator`）
- `base_sensor.py` + `streamer_base.py` 已合并为 `base.py`（`BaseSensor` + `StreamerBase`）
- `sensor_factory.py` 实际为 `factory.py`
- `imu_streamer.py` / `gnss_sol_streamer.py` 实际为 `imu_sensor.py` / `gnss_sol_sensor.py`
- `integration/` 目录当前未实现（无 Scheduler，传感器直接推入 queue 由 Logger 消费）
- `estimate/` 目录当前未实现（INS 启用后才会引入）

---

## 11. 配置文件格式

```yaml
# config.yaml — INS/GNSS 组合导航框架配置

# 运行模式
mode: "differential"              # "differential" (差分模式) 或 "spp" (单点模式)

# 读取模式
playback_mode: "asap"             # "asap" (尽速读取, 默认) 或 "timed" (按数据时间驱动)
speed: 100                        # 仅 timed 模式有效: 仿实时流加速倍率, 10表示10倍速读取

# 数据文件路径
files:
  imu: "data/imu.txt"
  gnss_rover: "data/gnss_rover.txt"    # 流动站观测值
  gnss_eph: "data/gnss_eph.txt"        # 广播星历
  gnss_ref: "data/gnss_ref.txt"        # 基准站观测值 (差分模式必填)

# 自定义 Formator 类 (Python 模块路径)
formators:
  imu: "formators.my_imu_formator.MyIMUFormator"
  rover: "formators.my_rover_formator.MyRoverFormator"
  eph: "formators.my_eph_formator.MyEphFormator"
  ref: "formators.my_ref_formator.MyRefFormator"

# 队列容量
queues:
  imu_maxsize: 2000
  rover_maxsize: 100
  eph_maxsize: 50
  ref_maxsize: 100
  estimate_maxsize: 50
  solution_maxsize: 200

# 调度选项
scheduler:
  rover_ref_sync_threshold: 0.05     # Rover-Ref 时间同步阈值 (秒)
  max_differential_age: 0.05          # 基站配对最大时间差 (秒)
  ref_data_timeout: 30.0              # 差分模式下基站数据超时 (秒), 超时后报错
  init_min_acceleration: 0.5          # 初始化最小加速度 (m/s^2)
  init_time_window: 2.0               # 初始化时间窗口 (秒)

# 估计选项
estimator:
  # SPP 选项
  spp_min_elevation: 15.0             # 最小仰角 (度)
  spp_min_snr: 30.0                   # 最小信噪比 (dB-Hz)

  # IMU 选项
  imu_sigma_g: 2.67e-4                # 陀螺白噪声 (rad/s/sqrt(Hz))
  imu_sigma_a: 0.0112                 # 加计白噪声 (m/s^2/sqrt(Hz))
  imu_sigma_bg: 1.0e-3                # 陀螺偏差噪声
  imu_sigma_ba: 1.0e-2                # 加计偏差噪声

# 日志选项
logging:
  output_dir: "output"
  log_solution: true
  log_raw_data: false
  log_level: "INFO"                   # DEBUG/INFO/WARNING/ERROR
```

---

## 12. C++ 移植考虑

### 12.1 接口设计原则

| 原则 | Python 实现 | C++ 移植方案 |
|------|-------------|-------------|
| 数据类型 | `@dataclass` + `numpy` | `struct` + `Eigen::Vector3d` |
| 抽象接口 | `ABC` + `@abstractmethod` | 纯虚函数 `= 0` |
| 线程安全队列 | `queue.Queue` | `std::queue` + `mutex` + `condition_variable` |
| 线程 | `threading.Thread` | `std::thread` |
| 配置解析 | `yaml-cpp` (Python: PyYAML) | `yaml-cpp` |
| 日志 | `logging` 模块 | `glog` 或 `spdlog` |
| 可选值 | `Optional[T]` | `std::optional<T>` (C++17) |

### 12.2 关键映射

```python
# Python                          // C++
Queue[maxsize]                  → ThreadSafeQueue<T>(size_t capacity)
SensorData                      → struct SensorData
FormatorBase.decode(line)       → virtual bool decode(const std::string& line) = 0
StreamerBase.run()              → virtual void run()  // 在 std::thread 中执行
Scheduler.run()                 → void run()           // 在 std::thread 中执行
ThreadControl.running           → std::atomic<bool> running_
np.ndarray                      → Eigen::VectorXd / Eigen::Matrix3d
```

### 12.3 不依赖 Python 特性的设计

- 不使用 Python 的 GIL 相关特性
- 不使用 `multiprocessing`（仅 `threading`）
- **不使用 `asyncio`**（避免与 threading 混用，所有数据流通过 `queue.Queue` + 独立线程驱动）
- **不使用 `multiprocessing.shared_memory`**（避免跨进程内存管理复杂度，所有数据通过 `queue.put()` 传递）
- 不使用 Python 特有的装饰器模式（`@abstractmethod` 对应 C++ 纯虚函数）
- 数据类型使用 `numpy.ndarray`（对应 C++ `Eigen`），不使用 Python list 做数值计算
- 文件 I/O 使用标准 `open/readline`（对应 C++ `std::ifstream::getline`）

---

## 附录: 线程生命周期

```
Main:
  ├── 创建 ThreadControl
  ├── 创建各 Queue
  ├── 创建 Formator 实例
  ├── 创建 Streamer 实例
  ├── 创建 Scheduler
  ├── 创建 EstimatorThread
  ├── 创建 Logger
  ├── 启动所有线程
  │   ├── IMUStreamer.thread.start()
  │   ├── RoverStreamer.thread.start()
  │   ├── EphStreamer.thread.start()     ← 永不暂停, 直到读完所有数据
  │   ├── RefStreamer.thread.start()
  │   ├── Scheduler.thread.start()
  │   ├── EstimatorThread.thread.start()
  │   └── Logger.thread.start()
  │
  ├── 主循环等待 (或处理 Ctrl+C 信号)
  │   while True:
  │       try: time.sleep(0.1)
  │       except KeyboardInterrupt: break
  │
  └── 关闭
      ├── ThreadControl.shutdown()     ← 通知所有线程退出
      ├── 等待各线程 join()
      └── 退出
```
