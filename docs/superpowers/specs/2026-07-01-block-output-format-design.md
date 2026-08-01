# 块状输出格式设计

> 本设计基于第一阶段数据读取与日志记录实现（见 `2026-06-30-data-logging-design.md`），
> 将输出格式从"1 行压缩视图（GNSS + IMU 均值）"改为"GNSS 行 + N 行 IMU 行的块状交替格式"。

## 1. 背景与动机

### 1.1 当前实现

第一阶段已完成数据读取与日志记录模块，输出 `output/aligned.csv`，每行 19 列：

```
gps_week,gps_sow,imu_count,imu_first_sow,imu_last_sow,gnss_x,gnss_y,gnss_z,gnss_q,gnss_ns,gnss_sdx,gnss_sdy,gnss_sdz,imu_avg_ax,imu_avg_ay,imu_avg_az,imu_avg_gx,imu_avg_gy,imu_avg_gz
2046,357456.000000,1,357455.995755,357455.995755,-2408695.8323,4698107.8215,3566699.7973,5,6,5.0056,7.4084,8.7523,0.061340,0.080414,9.767456,0.000087,0.000228,-0.000065
```

匹配策略为 GNSS 触发窗口 `[t_gnss - dt_imu/2, t_gnss + dt_imu/2]`（±5ms）。

### 1.2 新需求

用户要求：
- 一个 GNSS 历元对应 nHz（100）个 IMU 历元
- GNSS 数据单独占一行，后续跟着约 100 行匹配好的 IMU 数据，交替往复
- 不要将 GNSS 与 IMU 数据混在一行

### 1.3 设计哲学

> 该程序本质是以 IMU 数据流式进行解算，GNSS 数据是不稳定的，应该以 IMU 数据为基线进行。

匹配策略从"GNSS 触发窗口 ±5ms"改为"IMU 流式积攒，GNSS 到来时收割 `timestamp ≤ t_gnss + 0.01s` 的 IMU"。

## 2. 设计决策

通过 5 项澄清问题收集的设计决策：

| 项 | 决策 | 说明 |
|---|---|---|
| 匹配策略 | IMU 流式积攒，GNSS 到来时收割 | 截止到 GNSS 时间戳后 10 毫秒内的 IMU 数据 |
| 设计哲学 | 以 IMU 为基线 | GNSS 不稳定，IMU 流式解算 |
| 行格式区分 | 类型标识列 G/I | 首列字符区分行类型 |
| 首部 IMU | 丢弃第一个 GNSS 时间戳之前的 IMU | 数据未对齐之前的部分不输出 |
| 字段内容 | 完整字段含 week | GNSS 行 11 列、IMU 行 9 列 |
| 输出文件 | 替换 aligned.csv | 不新增文件 |

## 3. 数据类型设计

### 3.1 新增 AlignedBlock

文件：`src/core/data_types.py`

```python
@dataclass
class AlignedBlock:
    """对齐后的块数据：1 个 GNSS + N 个 IMU。"""
    gnss: GnssSolution
    imu_list: List[ImuMeasurement]
```

### 3.2 弃用 AlignedRow

原 `AlignedRow`（19 列压缩行）删除，相关测试同步调整。

### 3.3 保留类型

`ImuMeasurement`、`GnssSolution`、`SensorData` 保持不变。

## 4. Aligner 设计

文件：`src/log/aligner.py`

### 4.1 核心变化

- `match()`（窗口 ±5ms）→ `harvest()`（收割 ≤ t_gnss + 0.01s）
- 新增 `align_started` 标志，实现首部 IMU 丢弃
- 新增 `harvest_margin` 参数（默认 0.01s）

### 4.2 完整实现

```python
"""IMU 积攒 + GNSS 收割的匹配器。

IMU 流式推入缓冲，GNSS 到来时收割 timestamp <= t_gnss + harvest_margin 的 IMU。
首个 GNSS 到来前，timestamp < t_gnss_first 的 IMU 被丢弃（首部不对齐数据不输出）。
"""
from collections import deque
from typing import Optional, List

from src.core.data_types import AlignedBlock, GnssSolution, ImuMeasurement


class Aligner:
    """IMU 积攒 + GNSS 收割的匹配器。"""

    def __init__(self, imu_dt: float, harvest_margin: float = 0.01):
        self.imu_dt = float(imu_dt)
        self.harvest_margin = float(harvest_margin)  # 默认 0.01s
        self.imu_buffer: deque = deque()
        self.align_started: bool = False  # 首个 GNSS 是否已到

    def push_imu(self, imu: ImuMeasurement) -> None:
        """将一条 IMU 数据推入缓冲。"""
        self.imu_buffer.append(imu)

    def harvest(self, gnss: GnssSolution) -> Optional[AlignedBlock]:
        """对一个 GNSS 历元做收割。

        首次调用时丢弃 timestamp < t_gnss 的首部 IMU。
        收割缓冲中 timestamp <= t_gnss + harvest_margin 的 IMU。

        Returns:
            AlignedBlock 或 None（收割列表为空）
        """
        t_cut = gnss.timestamp + self.harvest_margin
        # 首次收割：丢弃首部 IMU（timestamp < t_gnss）
        if not self.align_started:
            while self.imu_buffer and self.imu_buffer[0].timestamp < gnss.timestamp:
                self.imu_buffer.popleft()
            self.align_started = True
        # 收割 timestamp <= t_cut 的 IMU
        imu_list: List[ImuMeasurement] = []
        while self.imu_buffer and self.imu_buffer[0].timestamp <= t_cut:
            imu_list.append(self.imu_buffer.popleft())
        if not imu_list:
            return None
        return AlignedBlock(gnss=gnss, imu_list=imu_list)
```

### 4.3 收割时序示例

假设 GNSS 1Hz、IMU 100Hz、harvest_margin=0.01s：

```
GNSS t=357456.000 → 丢弃 t<357456.000 的 IMU，收割 [357456.000, 357456.010] 的 IMU（约 1-2 条）
GNSS t=357457.000 → 收割 (357456.010, 357457.010] 的 IMU（约 100 条）
GNSS t=357458.000 → 收割 (357457.010, 357458.010] 的 IMU（约 100 条）
...
```

**注**：第一个 GNSS 块的 IMU 数量较少（仅 `[t_gnss, t_gnss+0.01]` 范围），因为首部 IMU 已丢弃。后续块为完整 100 条。

## 5. Writer 设计

文件：`src/log/aligned_writer.py`

### 5.1 核心变化

- 单行 19 列 → 块格式（1 行 G + N 行 I）
- 不写表头（G 行 11 列、I 行 9 列，列数不同）
- 表头常量 `CSV_HEADER` 删除

### 5.2 完整实现

```python
"""对齐数据块状 CSV 输出器。"""
import csv
from pathlib import Path

from src.core.data_types import AlignedBlock
from src.log.writer_base import WriterBase


class AlignedWriter(WriterBase):
    """对齐数据块状 CSV 输出器。

    输出格式：
        G,week,sow,x,y,z,q,ns,sdx,sdy,sdz        (11 列)
        I,week,sow,gx,gy,gz,ax,ay,az              (9 列)
        I,week,sow,gx,gy,gz,ax,ay,az
        ...（N 行 I）
        G,week,sow,...
        ...
    """

    def __init__(self, output_dir: str, filename: str = "aligned.csv"):
        self.output_dir = Path(output_dir)
        self.filename = filename
        self._file = None
        self._writer = None

    def open(self) -> None:
        self.output_dir.mkdir(parents=True, exist_ok=True)
        path = self.output_dir / self.filename
        self._file = open(path, "w", encoding="utf-8", newline="")
        self._writer = csv.writer(self._file)
        # 不写表头：G 行 11 列、I 行 9 列，列数不同

    def write(self, block: AlignedBlock) -> None:
        if self._writer is None:
            raise RuntimeError("AlignedWriter not opened")
        g = block.gnss
        # G 行: G, week, sow, x, y, z, q, ns, sdx, sdy, sdz (11 列)
        self._writer.writerow([
            "G", g.week, f"{g.timestamp:.6f}",
            f"{g.position[0]:.4f}", f"{g.position[1]:.4f}", f"{g.position[2]:.4f}",
            g.quality, g.num_sv,
            f"{g.sd[0]:.4f}", f"{g.sd[1]:.4f}", f"{g.sd[2]:.4f}",
        ])
        # I 行: I, week, sow, gx, gy, gz, ax, ay, az (9 列)
        for imu in block.imu_list:
            self._writer.writerow([
                "I", imu.week, f"{imu.timestamp:.6f}",
                f"{imu.gyro[0]:.6f}", f"{imu.gyro[1]:.6f}", f"{imu.gyro[2]:.6f}",
                f"{imu.accel[0]:.6f}", f"{imu.accel[1]:.6f}", f"{imu.accel[2]:.6f}",
            ])

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
```

### 5.3 字段精度

| 字段 | 精度 | 说明 |
|---|---|---|
| GNSS sow | `%.6f` | 微秒级 |
| GNSS x/y/z | `%.4f` | 0.1mm |
| GNSS sdx/sdy/sdz | `%.4f` | 0.1mm |
| IMU sow | `%.6f` | 微秒级 |
| IMU gx/gy/gz | `%.6f` | rad/s |
| IMU ax/ay/az | `%.6f` | m/s² |

## 6. Logger 设计

文件：`src/log/logger.py`

### 6.1 核心变化

主循环结构保留（含竞态修复 `_wait_for_imu`），仅 `match` → `harvest`：

```python
# 原：aligned = self.aligner.match(gnss)
aligned = self.aligner.harvest(gnss)
if aligned is not None:
    self.writer.write(aligned)
```

### 6.2 保留不变

- `_drain_imu_queue()`：非阻塞搬 IMU 到缓冲
- `_wait_for_imu(gnss_timestamp)`：阻塞等待 IMU 缓冲追上 GNSS 时间戳（竞态修复）
- `imu_eof` 标志：IMU EOF sentinel 处理
- `SensorData` 解包逻辑

## 7. 输出格式示例

### 7.1 文件开头

```
G,2046,357456.000000,-2408695.8323,4698107.8215,3566699.7973,5,6,5.0056,7.4084,8.7523
I,2046,357456.005755,0.000087,0.000228,-0.000065,0.061340,0.080414,9.767456
I,2046,357456.015755,0.000091,0.000231,-0.000062,0.061352,0.080421,9.767448
...
```

### 7.2 块结构

每个 GNSS 历元对应一个块：
- 第 1 行：`G` 开头，11 列
- 后续 N 行：`I` 开头，9 列（N ≈ 100，第一个块可能较少）

### 7.3 文件结尾

GNSS 文件读完（EOF sentinel）后，Logger 排空 IMU 队列后退出。剩余 IMU 不输出（无对应 GNSS 块）。

## 8. 测试策略

### 8.1 单元测试

| 测试文件 | 改动 | 关键测试点 |
|---|---|---|
| `tests/test_data_types.py` | `AlignedRow` → `AlignedBlock` 构造测试 | 字段访问、默认值 |
| `tests/test_aligner.py` | `match` → `harvest` | 首部丢弃、收割边界 `<= t_cut`、空列表返回 None、连续收割不重叠、harvest_margin 自定义 |
| `tests/test_aligned_writer.py` | 块格式输出 | G 行 11 列、I 行 9 列、无表头、多块连续、文件打开/关闭 |
| `tests/test_logger.py` | 集成 harvest + 块写出 | 端到端线程、EOF 处理、竞态修复 |

### 8.2 端到端测试

`tests/test_e2e_real_data.py`：使用 `data/cpt_imu.csv` + `data/spp.pos` 验证：
- 每个块以 `G` 行开头
- 每个块后跟若干 `I` 行
- 首部无 IMU（第一个 GNSS 之前的 IMU 已丢弃）
- 总块数 = GNSS 历元数
- 文件无表头

## 9. 配置与 main.py

### 9.1 config.yaml

无改动。`ins.data_rate: 100` 仍用于 `config_loader` 校验。

### 9.2 main.py

`Aligner` 构造不变，`harvest_margin` 使用默认值 0.01s：

```python
aligner = Aligner(imu_dt=1.0 / config["ins"]["data_rate"])
# 等价于 Aligner(imu_dt=0.01, harvest_margin=0.01)
```

## 10. 改动文件清单

### 10.1 源代码（4 个文件）

| 文件 | 改动类型 | 说明 |
|---|---|---|
| `src/core/data_types.py` | 修改 | 新增 `AlignedBlock`，删除 `AlignedRow` |
| `src/log/aligner.py` | 修改 | `match` → `harvest`，新增 `align_started`、`harvest_margin` |
| `src/log/aligned_writer.py` | 修改 | 块格式输出，删除 `CSV_HEADER` |
| `src/log/logger.py` | 修改 | `match` → `harvest` |

### 10.2 测试（5 个文件）

| 文件 | 改动类型 |
|---|---|
| `tests/test_data_types.py` | 修改 |
| `tests/test_aligner.py` | 修改 |
| `tests/test_aligned_writer.py` | 修改 |
| `tests/test_logger.py` | 修改 |
| `tests/test_e2e_real_data.py` | 修改 |

### 10.3 不改动

- `src/stream/` 全部文件
- `src/core/time_utils.py`、`thread_control.py`
- `src/utility/config_loader.py`
- `src/main.py`
- `data/config.yaml`

## 11. 风险与权衡

### 11.1 已知行为变化

1. **第一个 GNSS 块的 IMU 数量较少**：首部 IMU 丢弃后，第一个块仅含 `[t_gnss_first, t_gnss_first + 0.01]` 范围的 IMU（约 1-2 条），非完整 100 条。
2. **无表头**：G 行与 I 行列数不同，无法用统一表头。如需表头可后续添加注释行。
3. **剩余 IMU 不输出**：GNSS EOF 后，缓冲中剩余的 IMU 不输出（无对应 GNSS 块）。

### 11.2 向后不兼容

`aligned.csv` 格式完全改变，旧格式（19 列压缩行）不再支持。如需保留旧格式，可参考方案 2（新增并行类），但本次不采用。
