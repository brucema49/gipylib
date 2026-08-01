# 数据读取与日志记录模块设计文档

> 本文档描述 GInsStream 项目当前阶段（数据读取 + 日志记录 + 大致对齐）的模块化实现方案。
>
> **任务边界**：
> - 本次实现：读取 IMU 与外部 GNSS 结果文件，输出对齐后的 CSV
> - 本次不实现：GNSS 解算、INS 机械编排、EKF 融合等计算部分
>
> **设计依据**：
> - `skills/GInsStream.md`：整体代码框架、类继承体系
> - `skills/StreamDesign.md`：流式读取层与多线程架构
> - `skills/logANDoutput.md`：日志层与 Writer 体系
> - `skills/conf.md`：配置项定义
>
> **已确认的关键决策**（来自 brainstorming 阶段 AskUserQuestion）：
> 1. 时间对齐位置：暂不做精确插值，仅做 GNSS 触发的大致匹配
> 2. 数据源模式：仅 External 模式（IMU + 外部 GNSS 结果文件）
> 3. 匹配策略：GNSS 触发；IMU 频率由 config 指定（程序内检查必须为 100Hz，否则报错）；匹配窗口为 ±半个 IMU 历元间隔
> 4. 输出文件：仅 `aligned.csv`

---

## 目录

- [1. 架构总览](#1-架构总览)
- [2. 目录结构](#2-目录结构)
- [3. 类继承体系](#3-类继承体系)
- [4. 数据流](#4-数据流)
- [5. 关键接口](#5-关键接口)
- [6. 配置项映射](#6-配置项映射)
- [7. 输出文件格式](#7-输出文件格式)
- [8. 异常处理](#8-异常处理)
- [9. main.py 职责](#9-mainpy-职责)
- [10. 范围与非目标](#10-范围与非目标)

---

## 1. 架构总览

采用**两层数据通路**：Stream 层 + Log 层。

```
┌─────────── main.py ───────────┐
│ 读 config.yaml → 创建 sensor  │
│ list → 启动 Logger 线程 → 等 │
│ 待退出                        │
└────────────┬──────────────────┘
             ▼
┌─── Stream 层 (src/stream/) ───┐    ┌── Log 层 (src/log/) ──┐
│ ImuSensor (线程)              │    │ Logger (线程)          │
│   → imu_queue                 │───▶│  - Aligner 做 GNSS 触发│
│ GnssSolSensor (线程)         │    │    匹配（窗口 ±dt/2）  │
│   → gnss_queue                │───▶│  - AlignedWriter 写出  │
└───────────────────────────────┘    └────────────────────────┘
                                              ▼
                                       output/aligned.csv
```

**架构要点**：

- **纯队列流水线**：Streamer → `imu_queue`/`gnss_queue` → Logger → AlignedWriter，全程无观察者回调
- **纯 threading**：不使用 asyncio，不使用 shared_memory
- **GPST 时间系统**：全框架统一使用 GPS 秒（since 1980-01-06）
- **GNSS 触发的匹配**：每收到一个 GNSS 历元，从 imu_queue 缓冲中取窗口 `[t_gnss - dt_imu/2, t_gnss + dt_imu/2]` 内的 IMU 数据
- **main 精简**：仅做 config 加载 + 线程启动 + 等待，不包含业务逻辑

**选择两层架构的依据**：

1. 保留 queue 流水线（与 skills 文档一致），后续接入 Estimator 时只需在 Logger 之前插入 `estimate_queue`
2. GNSS 触发匹配需要同时持有 GNSS 与 IMU 数据，让 Logger 作为两条队列的汇合点最自然
3. main 职责清晰：只装配，不含业务逻辑
4. 后续计算部分接入时，Logger 内的匹配逻辑可平移到 Estimator，不影响 Stream 层

---

## 2. 目录结构

```
src/
├── main.py                    # 入口：读 config + 创建 sensor + 启动 Logger 线程
├── core/
│   ├── __init__.py
│   ├── data_types.py          # ImuMeasurement, GnssSolution, SensorData, AlignedRow
│   ├── thread_control.py      # ThreadControl
│   └── time_utils.py          # GPST 时间转换 (week/sow 与 yyyy/mm/dd 互转)
├── stream/
│   ├── __init__.py
│   ├── base.py                # BaseSensor(ABC) + StreamerBase(Thread)
│   ├── formators.py           # FormatorBase(ABC), ImuFormator, PosSolFormator
│   ├── imu_sensor.py          # ImuSensor
│   ├── gnss_sol_sensor.py     # GnssSolSensor
│   └── factory.py             # SensorFactory
├── log/
│   ├── __init__.py
│   ├── writer_base.py         # WriterBase(ABC)
│   ├── aligned_writer.py      # AlignedWriter
│   ├── aligner.py             # Aligner (GNSS 触发匹配核心算法)
│   └── logger.py              # Logger(Thread)
└── utility/
    └── config_loader.py       # YAML 读取 + IMU 频率检查
```

---

## 3. 类继承体系

```
BaseSensor(ABC)                            # 传感器抽象基类，强制 get_data()
└── StreamerBase(BaseSensor, threading.Thread)
    ├── ImuSensor                          # IMU 文本读取（GPST 格式）
    └── GnssSolSensor                      # 外部 GNSS 结果读取（rtklib POS 格式）

FormatorBase(ABC)                          # 解码层抽象基类
├── ImuFormator                           # IMU 文本解码（GPST 格式）
└── PosSolFormator                        # rtklib POS 格式解码

WriterBase(ABC)                            # 输出器抽象基类
└── AlignedWriter                         # 对齐数据 CSV 输出

SensorFactory                             # 工厂模式：根据配置创建 BaseSensor 实例列表
```

**设计模式应用**：

- **抽象基类（ABC）**：`BaseSensor`、`FormatorBase`、`WriterBase` 三个抽象层
- **工厂模式**：`SensorFactory.create_sensors()` 根据 config 动态创建传感器实例
- **依赖注入**：`Logger` 通过构造函数接收 `AlignedWriter` 实例
- **OOP 三大特性**：封装（输出逻辑封装在 Writer 子类）、继承（三层基类）、多态（Logger 持有 WriterBase 抽象引用）

---

## 4. 数据流

```
cpt_imu.csv ──→ ImuSensor ──→ ImuFormator ──→ imu_queue  ─┐
                                                              │
spp.pos    ──→ GnssSolSensor ──→ PosSolFormator ──→ gnss_queue ─┤
                                                              ▼
                                                          Logger 线程
                                                          ├─ 从 gnss_queue 取一历元
                                                          ├─ Aligner.match(gnss, imu_buffer):
                                                          │    窗口 [t - dt/2, t + dt/2]
                                                          ├─ 从 imu_queue 抽取窗口内 IMU
                                                          └─ AlignedWriter.write(aligned_row)
                                                              ▼
                                                       output/aligned.csv
```

**关键流程**：

1. `ImuSensor` 与 `GnssSolSensor` 在各自线程中逐行读取文件，通过 Formator 解码后 `put()` 到对应队列
2. `Logger` 主循环优先从 `gnss_queue` 取数据（GNSS 触发）
3. 收到一个 GNSS 历元后，从 `imu_queue`（先入 `Aligner.imu_buffer` 缓冲）中抽取时间窗口内的 IMU 数据
4. 调用 `AlignedWriter.write(aligned_row)` 输出一行到 `aligned.csv`
5. 收到 EOF sentinel 后，flush 缓冲，关闭文件

---

## 5. 关键接口

### 5.1 核心数据类型（core/data_types.py）

```python
from dataclasses import dataclass, field
from typing import Optional
import numpy as np

@dataclass
class ImuMeasurement:
    """IMU 单次测量"""
    timestamp: float          # GPST 秒
    week: int                 # GPS 周
    accel: np.ndarray         # [3] m/s² 机体坐标系
    gyro: np.ndarray          # [3] rad/s 机体坐标系

@dataclass
class GnssSolution:
    """外部 GNSS 结果（来自 spp.pos）"""
    timestamp: float          # GPST 秒
    week: int
    position: np.ndarray      # [3] ECEF (m)
    quality: int              # 1=SPP, 2=RTD, 5=LC
    num_sv: int
    sd: np.ndarray            # [3] 位置标准差 (sdn, sde, sdu)

@dataclass
class AlignedRow:
    """对齐后的一行输出数据"""
    week: int
    sow: float                # GNSS 历元时间
    imu_count: int            # 窗口内 IMU 数据条数
    imu_first_sow: float
    imu_last_sow: float
    gnss_pos: np.ndarray      # [3]
    gnss_q: int
    gnss_ns: int
    gnss_sd: np.ndarray       # [3]
    imu_avg_accel: np.ndarray # [3] 窗口均值
    imu_avg_gyro: np.ndarray  # [3] 窗口均值

@dataclass
class SensorData:
    """传感器数据统一容器（一次只承载一种类型）"""
    tag: str                                  # "imu" / "gnss_solution"
    imu: Optional[ImuMeasurement] = None
    gnss_solution: Optional[GnssSolution] = None
```

### 5.2 传感器层（stream/base.py）

```python
from abc import ABC, abstractmethod
from typing import Optional
from queue import Queue
from threading import Thread

class BaseSensor(ABC):
    """传感器抽象基类，强制 get_data() 接口"""
    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def get_data(self):
        """获取一条数据（非阻塞，无数据返回 None）"""
        ...

class StreamerBase(BaseSensor, Thread):
    """流式读取器基类 — 逐行读取文本文件，O(1) 内存

    继承 BaseSensor，实现 get_data() 统一接口。
    内部封装：逐行读取 → Formator 解码 → 推入队列。
    """
    def __init__(self, file_path: str, formator, output_queue: Queue,
                 control, tag: str):
        BaseSensor.__init__(self, name=tag)
        Thread.__init__(self, name=tag, daemon=True)
        self.file_path = file_path
        self.formator = formator
        self.output_queue = output_queue
        self.control = control
        self.tag = tag

    def run(self):
        """线程入口：逐行读取 → 解码 → 入队"""
        try:
            with open(self.file_path, 'r', encoding='utf-8') as f:
                for line in f:
                    if not self.control.is_running():
                        break
                    data = self.formator.decode(line)
                    if data is not None:
                        self.output_queue.put(data)
        finally:
            # EOF sentinel 通知下游
            self.output_queue.put(None)

    def get_data(self):
        """非阻塞返回队列头部数据（若需同步获取时调用）"""
        try:
            return self.output_queue.get_nowait()
        except Empty:
            return None
```

### 5.3 解码层（stream/formators.py）

```python
from abc import ABC, abstractmethod

class FormatorBase(ABC):
    """解码层抽象基类"""
    @abstractmethod
    def decode(self, line: str):
        """解码一行文本，返回 SensorData 或 None（注释/表头）"""
        ...

class ImuFormator(FormatorBase):
    """IMU 文本解码（GPST 格式）
    列：GPS week, GPS sow, gx, gy, gz, ax, ay, az
    """
    def decode(self, line: str):
        # 跳过注释/空行
        # 解析为 ImuMeasurement（封装于 SensorData）
        ...

class PosSolFormator(FormatorBase):
    """rtklib POS 格式解码
    跳过 % 开头注释行，解析数据行：
    yyyy/mm/dd hh:mm:ss.s  x  y  z  Q  ns  sdx  sdy  sdz  ...
    """
    def decode(self, line: str):
        # 跳过 % 注释行
        # 解析时间 → week/sow (经 time_utils 转换)
        # 返回 GnssSolution
        ...
```

### 5.4 对齐器（log/aligner.py）

```python
from collections import deque

class Aligner:
    """GNSS 触发的 IMU 匹配器

    每收到一个 GNSS 历元，从 IMU 缓冲中取时间窗口内数据：
    窗口 = [t_gnss - dt_imu/2, t_gnss + dt_imu/2]
    """
    def __init__(self, imu_dt: float):
        self.imu_dt = imu_dt
        self.half_window = imu_dt / 2.0
        self.imu_buffer = deque()  # 缓冲未匹配的 IMU 数据

    def push_imu(self, imu):
        """将一条 IMU 数据推入缓冲"""
        self.imu_buffer.append(imu)

    def match(self, gnss) -> Optional[AlignedRow]:
        """对一个 GNSS 历元做匹配，返回 AlignedRow 或 None（窗口内无 IMU）"""
        t_lo = gnss.timestamp - self.half_window
        t_hi = gnss.timestamp + self.half_window

        # 1. 弹出窗口左侧过期 IMU
        while self.imu_buffer and self.imu_buffer[0].timestamp < t_lo:
            self.imu_buffer.popleft()

        # 2. 收集窗口内 IMU
        windowed = [imu for imu in self.imu_buffer
                    if t_lo <= imu.timestamp <= t_hi]

        if not windowed:
            return None

        # 3. 构造 AlignedRow（IMU 均值等统计量）
        ...
```

### 5.5 日志器（log/logger.py）

```python
class Logger(Thread):
    """日志记录器线程

    从 imu_queue、gnss_queue 消费数据，做 GNSS 触发匹配，
    委托 AlignedWriter 输出。
    """
    def __init__(self, imu_queue, gnss_queue, writer, aligner, control):
        Thread.__init__(self, name="Logger", daemon=True)
        self.imu_queue = imu_queue
        self.gnss_queue = gnss_queue
        self.writer = writer
        self.aligner = aligner
        self.control = control
        self.imu_eof = False   # IMU 数据流是否已结束

    def run(self):
        self.writer.open()
        try:
            while self.control.is_running():
                # 1. 先把 imu_queue 中的数据搬到 aligner.imu_buffer
                #    （收到 IMU EOF sentinel 时标记 imu_eof）
                self._drain_imu_queue()
                # 2. 取一个 GNSS 历元（阻塞，超时 0.1s）
                try:
                    gnss = self.gnss_queue.get(timeout=0.1)
                except Empty:
                    continue  # 无数据，回到循环开头
                # 3. 区分 EOF sentinel 与正常数据
                if gnss is None:
                    # GNSS 文件已读完，等待剩余 IMU 也排空后退出
                    self._drain_imu_queue()
                    break
                # 4. 匹配并写出
                aligned = self.aligner.match(gnss)
                if aligned is not None:
                    self.writer.write(aligned)
        finally:
            self._flush_remaining()
            self.writer.close()

    def _drain_imu_queue(self):
        """非阻塞地把 imu_queue 中所有数据搬到 aligner.imu_buffer

        遇到 None sentinel 时不放入缓冲（标记 IMU EOF）。
        """
        while True:
            try:
                imu = self.imu_queue.get_nowait()
            except Empty:
                break
            if imu is None:
                self.imu_eof = True
                continue
            self.aligner.push_imu(imu)

# 注：EOF sentinel 统一使用 None
# - StreamerBase.run() 在文件读完时 output_queue.put(None)
# - Logger 收到 None 视为该队列对应数据源已结束
# - 超时无数据用 queue.Empty 异常区分，避免与 sentinel 歧义
```

### 5.6 输出器（log/aligned_writer.py）

```python
class AlignedWriter(WriterBase):
    """对齐数据 CSV 输出器"""
    def __init__(self, output_dir: str, filename: str = "aligned.csv"):
        self.output_dir = output_dir
        self.filename = filename
        self._file = None
        self._writer = None

    def open(self):
        # 创建 output_dir（若不存在）
        # 打开文件，写 CSV 表头
        ...

    def write(self, row: AlignedRow):
        # 写一行 CSV
        ...

    def close(self):
        # 关闭文件句柄
        ...
```

### 5.7 配置加载（utility/config_loader.py）

```python
import yaml

def load_config(path: str) -> dict:
    """加载 YAML 配置并做必要校验"""
    with open(path, 'r', encoding='utf-8') as f:
        cfg = yaml.safe_load(f)

    # IMU 频率检查：当前阶段必须为 100Hz
    data_rate = cfg['ins']['data_rate']
    if data_rate != 100:
        raise ValueError(
            f"IMU data_rate must be 100, got {data_rate}. "
            f"Current stage only supports 100Hz IMU."
        )

    # 数据源模式检查：当前阶段仅支持 external
    if cfg['gnss']['gnss_source'] != "external":
        raise ValueError(
            f"gnss_source must be 'external' in current stage, "
            f"got '{cfg['gnss']['gnss_source']}'"
        )

    return cfg
```

---

## 6. 配置项映射

从 `data/config.yaml` 读取以下字段：

| 配置项 | 用途 |
|--------|------|
| `ins.data_rate` | IMU 频率检查（必须=100，否则报错）+ 计算 `imu_dt = 1/data_rate`，进而算 `half_window = imu_dt/2` |
| `ins.imu_data_path` | IMU 文件路径（如 `data/cpt_imu.csv`） |
| `gnss.gnss_source` | 必须为 `"external"`，否则报错 |
| `gnss.external_sol_path` | 外部 GNSS 结果文件路径（如 `data/spp.pos`） |
| `gnss.external_sol_format` | 外部结果格式（必须为 `"pos"`，否则报错，当前阶段仅支持 POS） |
| `output.output_dir` | 输出目录（如 `output`） |

> 注意：`data_rate` 由配置文件指定（非硬编码），但当前阶段程序内部检查必须为 100Hz，否则报错。这样保留了未来扩展到其他频率的可能性。

> **配置文件预处理提示**：当前 `data/config.yaml` 中 `gnss.gnss_source` 默认为 `"internal"`，运行本程序前需要手动改为 `"external"`。若未改，`load_config` 会主动抛 `ValueError` 提示用户。

---

## 7. 输出文件格式

### 7.1 输出文件路径

`{output.output_dir}/aligned.csv`（默认 `output/aligned.csv`）

### 7.2 CSV 表头与字段

```csv
gps_week,gps_sow,imu_count,imu_first_sow,imu_last_sow,gnss_x,gnss_y,gnss_z,gnss_q,gnss_ns,gnss_sdx,gnss_sdy,gnss_sdz,imu_avg_ax,imu_avg_ay,imu_avg_az,imu_avg_gx,imu_avg_gy,imu_avg_gz
```

### 7.3 字段说明

| 字段 | 单位 | 说明 |
|------|------|------|
| `gps_week` | - | GPS 周号 |
| `gps_sow` | s | GPS 周内秒（GNSS 历元时间） |
| `imu_count` | - | 匹配窗口内 IMU 数据条数 |
| `imu_first_sow` | s | 窗口内首条 IMU 时间 |
| `imu_last_sow` | s | 窗口内末条 IMU 时间 |
| `gnss_x` / `gnss_y` / `gnss_z` | m | ECEF 位置 |
| `gnss_q` | - | 质量标志（1=SPP, 2=RTD, 5=LC） |
| `gnss_ns` | - | 使用卫星数 |
| `gnss_sdx` / `gnss_sdy` / `gnss_sdz` | m | 位置标准差 |
| `imu_avg_ax` / `imu_avg_ay` / `imu_avg_az` | m/s² | 窗口内加速度均值 |
| `imu_avg_gx` / `imu_avg_gy` / `imu_avg_gz` | rad/s | 窗口内角速度均值 |

### 7.4 示例行

```csv
2046,357254.000,1,357253.995,357254.005,-2408695.8323,4698107.8215,3566699.7973,5,6,5.0056,7.4084,8.7523,0.000458,0.106659,9.884033,-0.000640,0.000456,-0.000423
```

---

## 8. 异常处理

| 场景 | 处理方式 |
|------|---------|
| `data_rate != 100` | 启动时 `load_config` 抛 `ValueError` 退出 |
| `gnss_source != "external"` | 启动时 `load_config` 抛 `ValueError` 退出 |
| 文件不存在 | 启动时 Streamer 抛 `FileNotFoundError` 退出 |
| IMU 窗口内无数据 | 跳过该 GNSS 历元，不写 CSV 行（可选写 stderr warning） |
| IMU 时间戳超出 GNSS 时间范围 | 仅在窗口内匹配，区间外不报警 |
| YAML 解析错误 | 抛 `yaml.YAMLError` 退出 |
| 主线程收到 Ctrl+C | `ThreadControl.shutdown()` 通知所有线程退出 |

---

## 9. main.py 职责

`main.py` 仅作装配入口，不含业务逻辑：

```python
from queue import Queue
from src.core.thread_control import ThreadControl
from src.stream.factory import SensorFactory
from src.log.aligned_writer import AlignedWriter
from src.log.aligner import Aligner
from src.log.logger import Logger
from src.utility.config_loader import load_config

def main():
    # 1. 加载配置（含频率与模式校验）
    config = load_config("data/config.yaml")

    # 2. 创建共享对象
    control = ThreadControl()
    imu_queue = Queue(maxsize=2000)
    gnss_queue = Queue(maxsize=100)

    # 3. 工厂创建传感器
    sensors = SensorFactory.create_sensors(
        config, imu_queue, gnss_queue, control
    )

    # 4. 装配 Logger
    writer = AlignedWriter(output_dir=config['output']['output_dir'])
    aligner = Aligner(imu_dt=1.0 / config['ins']['data_rate'])
    logger = Logger(imu_queue, gnss_queue, writer, aligner, control)

    # 5. 启动所有线程
    for s in sensors:
        s.start()
    logger.start()

    # 6. 等待 Logger 结束（Logger 收到两个 EOF 后退出）
    logger.join()
    control.shutdown()
    for s in sensors:
        s.join()

if __name__ == "__main__":
    main()
```

**main 函数职责清单**：

1. 读取并校验配置
2. 创建共享队列与线程控制对象
3. 工厂创建传感器列表
4. 装配 Logger（含 writer 与 aligner 依赖注入）
5. 启动所有线程
6. 等待 Logger 线程结束 → 通知其他线程退出 → join

---

## 10. 范围与非目标

### 10.1 本次范围

- [x] 读取 `data/cpt_imu.csv`（GPST 格式 IMU）
- [x] 读取 `data/spp.pos`（rtklib POS 格式 GNSS 结果）
- [x] 多线程流水线（ImuSensor + GnssSolSensor + Logger）
- [x] GNSS 触发的 IMU 匹配（窗口 ±dt/2）
- [x] 输出 `output/aligned.csv`
- [x] 配置加载与频率/模式校验

### 10.2 本次非目标

- [ ] 精确插值时间对齐（参考 KF-GINS 增量切分）
- [ ] GNSS 内部解算（SPP/RTD/RTK）
- [ ] INS 机械编排
- [ ] EKF 松组合融合
- [ ] NHC 子滤波、ZUPT
- [ ] POS/NMEA 输出格式
- [ ] Trace 日志（仅保留接口预留）
- [ ] Rover/Ref/Eph 读取（仅 External 模式）

### 10.3 后续扩展预留

- 时间对齐下沉到 Estimator 时，Logger 中的 `Aligner` 可整体平移到 `Estimator` 内部，不影响 Stream 层
- 接入计算部分时，在 Logger 之前插入 `estimate_queue` + `Estimator` 线程即可
- `AlignedWriter` 类已抽象为 `WriterBase`，后续可扩展 `SolutionWriter` / `TraceWriter` / `RawDataWriter`

---

## 附录 A：与 skills 设计文档的对应关系

| 本设计文档章节 | 对应 skills 文档 |
|---------------|-----------------|
| 第 2 节 目录结构 | `GInsStream.md` 第 10 节、`StreamDesign.md` 第 10 节 |
| 第 3 节 类继承体系 | `GInsStream.md` 第 5 节、`StreamDesign.md` 第 5 节 |
| 第 5.2 节 StreamerBase | `StreamDesign.md` 第 5.2 节 |
| 第 5.3 节 FormatorBase | `StreamDesign.md` 第 5.3 节 |
| 第 5.5 节 Logger | `logANDoutput.md` 第 7 节 |
| 第 5.6 节 AlignedWriter | `logANDoutput.md` 第 3 节 WriterBase 体系 |
| 第 6 节 配置映射 | `conf.md` 全文 |
| 第 7 节 输出格式 | `logANDoutput.md` 第 8 节 |

## 附录 B：架构决策偏离说明

本设计相对于 skills 文档的偏离：

1. **时间对齐位置**：原设计在 Estimator 内部（参考 KF-GINS 增量切分）；本次因不实现计算部分，将"大致匹配"前移到 Logger（结合 Aligner 模块）。后续接入 Estimator 时可平移。
2. **简化匹配策略**：原设计为精确插值；本次为窗口内"大致匹配"（不插值），窗口 = ±半个 IMU 历元间隔。
3. **输出文件简化**：原设计包含 solution.pos/csv/nmea、trace.txt、raw/*.csv；本次仅输出 aligned.csv。
4. **数据源限制**：原设计支持 internal/external 双模式；本次仅 external 模式，启动时强制校验。
