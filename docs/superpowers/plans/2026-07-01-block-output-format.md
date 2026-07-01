# 块状输出格式改造 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 `output/aligned.csv` 从"1 行 19 列压缩视图"改为"G 行 + N 行 I 行块状交替格式"，匹配策略从"GNSS 触发窗口 ±5ms"改为"IMU 积攒 + GNSS 收割 ≤ t_gnss + 0.01s"。

**Architecture:** 原地改造方案。新增 `AlignedBlock` dataclass 替换 `AlignedRow`；`Aligner.match()` 改为 `Aligner.harvest()`，新增 `align_started` 标志实现首部 IMU 丢弃；`AlignedWriter.write()` 改为块格式（1 行 G + N 行 I，无表头）；`Logger` 主循环仅替换 `match` → `harvest`，保留竞态修复。采用"先并存后删除"策略确保每个任务结束后所有测试通过。

**Tech Stack:** Python 3、threading、queue.Queue、dataclasses、numpy、csv、pytest

**Spec:** `docs/superpowers/specs/2026-07-01-block-output-format-design.md`

---

## File Structure

| 文件 | 责任 | 改动 |
|---|---|---|
| `src/core/data_types.py` | 核心数据类型 | 新增 `AlignedBlock`，删除 `AlignedRow` |
| `src/log/aligner.py` | IMU 积攒 + GNSS 收割 | `match` → `harvest`，新增 `align_started`、`harvest_margin` |
| `src/log/aligned_writer.py` | 块状 CSV 输出 | 单行 19 列 → 块格式，删除 `CSV_HEADER` |
| `src/log/logger.py` | 日志线程主循环 | `match` → `harvest` |
| `tests/test_data_types.py` | 数据类型测试 | `AlignedRow` → `AlignedBlock` |
| `tests/test_aligner.py` | Aligner 测试 | `match` → `harvest` |
| `tests/test_aligned_writer.py` | Writer 测试 | 块格式断言 |
| `tests/test_logger.py` | Logger 测试 | 块格式断言 |
| `tests/test_e2e_real_data.py` | E2E 测试 | 块格式断言 |

---

## Task 1: 新增 AlignedBlock 数据类型

**Files:**
- Modify: `src/core/data_types.py`
- Test: `tests/test_data_types.py`

**策略：** 新增 `AlignedBlock`，暂保留 `AlignedRow`（避免破坏 aligner/aligned_writer 的 import）。

- [ ] **Step 1: 写失败测试**

在 `tests/test_data_types.py` 末尾追加（不修改现有 import 和测试）：

```python
def test_aligned_block_construction():
    from src.core.data_types import AlignedBlock, GnssSolution, ImuMeasurement
    gnss = GnssSolution(
        timestamp=357254.000,
        week=2046,
        position=np.array([-2408695.8323, 4698107.8215, 3566699.7973]),
        quality=5,
        num_sv=6,
        sd=np.array([5.0056, 7.4084, 8.7523]),
    )
    imu_list = [
        ImuMeasurement(
            timestamp=357254.005, week=2046,
            accel=np.array([0.0, 0.0, 9.8]),
            gyro=np.array([0.0, 0.0, 0.0]),
        ),
        ImuMeasurement(
            timestamp=357254.015, week=2046,
            accel=np.array([0.1, 0.0, 9.8]),
            gyro=np.array([0.0, 0.0, 0.0]),
        ),
    ]
    block = AlignedBlock(gnss=gnss, imu_list=imu_list)
    assert block.gnss.week == 2046
    assert len(block.imu_list) == 2
    assert block.imu_list[0].timestamp == 357254.005
    assert block.imu_list[1].accel[0] == 0.1
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python -m pytest tests/test_data_types.py::test_aligned_block_construction -v`
Expected: FAIL with `ImportError: cannot import name 'AlignedBlock'`

- [ ] **Step 3: 实现 AlignedBlock**

在 `src/core/data_types.py` 的 `AlignedRow` 类定义之后（`SensorData` 之前）新增：

```python
@dataclass
class AlignedBlock:
    """对齐后的块数据：1 个 GNSS + N 个 IMU。"""
    gnss: GnssSolution
    imu_list: List[ImuMeasurement]
```

同时在文件顶部 import 中添加 `List`：

```python
from typing import Optional, List
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python -m pytest tests/test_data_types.py -v`
Expected: PASS（4 个旧测试 + 1 个新测试全通过）

- [ ] **Step 5: 提交**

```bash
git add src/core/data_types.py tests/test_data_types.py
git commit -m "feat(data_types): add AlignedBlock dataclass for block output format"
```

---

## Task 2: Aligner 新增 harvest 方法

**Files:**
- Modify: `src/log/aligner.py`
- Test: `tests/test_aligner.py`

**策略：** 新增 `harvest()` 方法，暂保留 `match()`（旧测试仍用 match）。更新 import 加入 `AlignedBlock`。

- [ ] **Step 1: 写失败测试**

在 `tests/test_aligner.py` 末尾追加（保留现有 `_imu`/`_gnss` 辅助函数和旧测试）：

```python
def test_harvest_discards_head_imu_before_first_gnss():
    """首部 IMU（timestamp < t_gnss_first）应被丢弃。"""
    a = Aligner(imu_dt=0.01, harvest_margin=0.01)
    # 这些 IMU 时间戳 < 第一个 GNSS 时间戳，应被丢弃
    a.push_imu(_imu(2046, 357253.990))
    a.push_imu(_imu(2046, 357253.995))
    # 第一个 GNSS
    block = a.harvest(_gnss(2046, 357254.000))
    # 收割 [357254.000, 357254.010] 范围 IMU（首部已丢弃，缓冲可能空）
    # 没有有效 IMU，返回 None
    assert block is None
    assert a.align_started is True


def test_harvest_first_block_collects_imu_at_and_after_gnss():
    """首个 GNSS 收割 [t_gnss, t_gnss+margin] 的 IMU。"""
    a = Aligner(imu_dt=0.01, harvest_margin=0.01)
    # 首部 IMU（应丢弃）
    a.push_imu(_imu(2046, 357253.990))
    a.push_imu(_imu(2046, 357253.995))
    # 有效 IMU：[357254.000, 357254.010]
    a.push_imu(_imu(2046, 357254.000))
    a.push_imu(_imu(2046, 357254.005))
    a.push_imu(_imu(2046, 357254.010))
    # 后续 IMU（应留给下一块）
    a.push_imu(_imu(2046, 357254.015))

    block = a.harvest(_gnss(2046, 357254.000))
    assert block is not None
    assert len(block.imu_list) == 3
    assert block.imu_list[0].timestamp == 357254.000
    assert block.imu_list[-1].timestamp == 357254.010
    assert block.gnss.timestamp == 357254.000
    # 缓冲应剩 1 条
    assert len(a.imu_buffer) == 1


def test_harvest_consecutive_blocks_no_overlap():
    """连续两个 GNSS 块的 IMU 不重叠。"""
    a = Aligner(imu_dt=0.01, harvest_margin=0.01)
    # 假设 IMU 每 0.01s 一条，GNSS 1Hz
    # 第一个 GNSS t=100.000，收割 [100.000, 100.010]
    # 第二个 GNSS t=101.000，收割 (100.010, 101.010]
    for i in range(200):
        a.push_imu(_imu(2046, 100.000 + i * 0.01))

    block1 = a.harvest(_gnss(2046, 100.000))
    assert block1 is not None
    # 第一个块：首部丢弃后 [100.000, 100.010]，2 条
    assert len(block1.imu_list) == 2

    block2 = a.harvest(_gnss(2046, 101.000))
    assert block2 is not None
    # 第二个块：(100.010, 101.010]，约 100 条
    assert len(block2.imu_list) == 100
    # 无重叠
    assert block1.imu_list[-1].timestamp < block2.imu_list[0].timestamp


def test_harvest_returns_none_when_no_imu():
    """收割列表为空时返回 None。"""
    a = Aligner(imu_dt=0.01, harvest_margin=0.01)
    # 不推任何 IMU
    block = a.harvest(_gnss(2046, 357254.000))
    assert block is None


def test_harvest_custom_margin():
    """自定义 harvest_margin。"""
    a = Aligner(imu_dt=0.01, harvest_margin=0.02)
    a.push_imu(_imu(2046, 100.000))
    a.push_imu(_imu(2046, 100.010))
    a.push_imu(_imu(2046, 100.020))
    a.push_imu(_imu(2046, 100.030))  # 超出 margin

    block = a.harvest(_gnss(2046, 100.000))
    assert block is not None
    # 收割 [100.000, 100.020]，3 条
    assert len(block.imu_list) == 3
    assert block.imu_list[-1].timestamp == 100.020
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python -m pytest tests/test_aligner.py -v -k harvest`
Expected: FAIL with `AttributeError: 'Aligner' object has no attribute 'harvest'`

- [ ] **Step 3: 实现 harvest 方法**

在 `src/log/aligner.py` 中：

1. 更新文件顶部 docstring 和 import：

```python
"""IMU 积攒 + GNSS 收割的匹配器。

提供两种匹配策略：
- match(): GNSS 触发窗口 [t_gnss - dt_imu/2, t_gnss + dt_imu/2]（旧，将弃用）
- harvest(): IMU 积攒 + GNSS 收割 timestamp <= t_gnss + harvest_margin（新）
"""
from collections import deque
from typing import Optional, List

import numpy as np

from src.core.data_types import AlignedBlock, AlignedRow, GnssSolution, ImuMeasurement
```

2. 修改 `__init__` 新增 `harvest_margin` 和 `align_started`：

```python
class Aligner:
    """IMU 积攒 + GNSS 收割的匹配器。"""

    def __init__(self, imu_dt: float, harvest_margin: float = 0.01):
        self.imu_dt = float(imu_dt)
        self.harvest_margin = float(harvest_margin)
        self.half_window = self.imu_dt / 2.0
        self.imu_buffer: deque = deque()
        self.align_started: bool = False
```

3. 在 `match()` 方法之后新增 `harvest()` 方法：

```python
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

- [ ] **Step 4: 运行测试验证通过**

Run: `python -m pytest tests/test_aligner.py -v`
Expected: PASS（5 个旧 match 测试 + 5 个新 harvest 测试全通过）

- [ ] **Step 5: 提交**

```bash
git add src/log/aligner.py tests/test_aligner.py
git commit -m "feat(aligner): add harvest() method with head IMU discard and margin"
```

---

## Task 3: AlignedWriter 改为块格式输出

**Files:**
- Modify: `src/log/aligned_writer.py`
- Test: `tests/test_aligned_writer.py`

**策略：** 完全重写 `AlignedWriter` 为块格式，删除 `CSV_HEADER`。重写测试文件。此任务后 `AlignedRow` 在 Writer 中不再使用。

- [ ] **Step 1: 重写测试文件**

完全替换 `tests/test_aligned_writer.py` 内容为：

```python
import csv
import numpy as np
from src.log.aligned_writer import AlignedWriter
from src.core.data_types import AlignedBlock, GnssSolution, ImuMeasurement


def _gnss(week=2046, sow=357254.0):
    return GnssSolution(
        timestamp=sow, week=week,
        position=np.array([-2408695.8323, 4698107.8215, 3566699.7973]),
        quality=5, num_sv=6,
        sd=np.array([5.0056, 7.4084, 8.7523]),
    )


def _imu(week, sow, ax=0.0, ay=0.0, az=9.8, gx=0.0, gy=0.0, gz=0.0):
    return ImuMeasurement(
        timestamp=sow, week=week,
        accel=np.array([ax, ay, az]),
        gyro=np.array([gx, gy, gz]),
    )


def _make_block(week=2046, sow=357254.0, n_imu=2):
    return AlignedBlock(
        gnss=_gnss(week, sow),
        imu_list=[_imu(week, sow + i * 0.01) for i in range(n_imu)],
    )


def test_aligned_writer_creates_file_no_header(tmp_path):
    w = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    w.open()
    w.write(_make_block(n_imu=2))
    w.close()
    out = tmp_path / "aligned.csv"
    assert out.exists()
    lines = out.read_text(encoding="utf-8").splitlines()
    # 无表头：第一行直接是 G 行
    assert lines[0].startswith("G,")
    # 1 行 G + 2 行 I = 3 行
    assert len(lines) == 3


def test_aligned_writer_g_row_has_11_columns(tmp_path):
    w = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    w.open()
    w.write(_make_block(n_imu=1))
    w.close()
    lines = (tmp_path / "aligned.csv").read_text(encoding="utf-8").splitlines()
    g_fields = lines[0].split(",")
    # G, week, sow, x, y, z, q, ns, sdx, sdy, sdz = 11
    assert len(g_fields) == 11
    assert g_fields[0] == "G"
    assert g_fields[1] == "2046"


def test_aligned_writer_i_row_has_9_columns(tmp_path):
    w = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    w.open()
    w.write(_make_block(n_imu=1))
    w.close()
    lines = (tmp_path / "aligned.csv").read_text(encoding="utf-8").splitlines()
    i_fields = lines[1].split(",")
    # I, week, sow, gx, gy, gz, ax, ay, az = 9
    assert len(i_fields) == 9
    assert i_fields[0] == "I"


def test_aligned_writer_writes_multiple_blocks(tmp_path):
    w = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    w.open()
    w.write(_make_block(sow=357254.0, n_imu=3))
    w.write(_make_block(sow=357255.0, n_imu=2))
    w.close()
    lines = (tmp_path / "aligned.csv").read_text(encoding="utf-8").splitlines()
    # 块1: 1G + 3I = 4 行；块2: 1G + 2I = 3 行；共 7 行
    assert len(lines) == 7
    assert lines[0].startswith("G,")
    assert lines[1].startswith("I,")
    assert lines[4].startswith("G,")
    assert lines[5].startswith("I,")


def test_aligned_writer_correct_values(tmp_path):
    w = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    w.open()
    w.write(_make_block(sow=357254.0, n_imu=1))
    w.close()
    lines = (tmp_path / "aligned.csv").read_text(encoding="utf-8").splitlines()
    g_fields = lines[0].split(",")
    # 验证 GNSS 字段
    assert g_fields[1] == "2046"  # week
    assert abs(float(g_fields[2]) - 357254.0) < 1e-6  # sow
    assert abs(float(g_fields[3]) - -2408695.8323) < 1e-4  # x
    assert g_fields[7] == "5"  # quality
    assert g_fields[8] == "6"  # num_sv
    i_fields = lines[1].split(",")
    assert i_fields[1] == "2046"  # week
    assert abs(float(i_fields[2]) - 357254.0) < 1e-6  # sow


def test_aligned_writer_creates_output_dir_if_missing(tmp_path):
    sub = tmp_path / "new_subdir"
    w = AlignedWriter(output_dir=str(sub), filename="aligned.csv")
    w.open()
    w.write(_make_block())
    w.close()
    assert (sub / "aligned.csv").exists()


def test_aligned_writer_raises_if_not_opened(tmp_path):
    w = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    try:
        w.write(_make_block())
        assert False, "should raise RuntimeError"
    except RuntimeError:
        pass
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python -m pytest tests/test_aligned_writer.py -v`
Expected: FAIL（旧 AlignedWriter 仍用 AlignedRow，新测试用 AlignedBlock，类型不匹配）

- [ ] **Step 3: 重写 AlignedWriter**

完全替换 `src/log/aligned_writer.py` 内容为：

```python
"""对齐数据块状 CSV 输出器。"""
import csv
from pathlib import Path

from src.core.data_types import AlignedBlock
from src.log.writer_base import WriterBase


class AlignedWriter(WriterBase):
    """对齐数据块状 CSV 输出器。

    输出格式（无表头）：
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

- [ ] **Step 4: 运行测试验证通过**

Run: `python -m pytest tests/test_aligned_writer.py -v`
Expected: PASS（7 个新测试全通过）

同时验证未破坏其他测试：
Run: `python -m pytest tests/ -v --ignore=tests/test_logger.py --ignore=tests/test_e2e_real_data.py`
Expected: 除 test_logger（仍用旧 match）外全通过

- [ ] **Step 5: 提交**

```bash
git add src/log/aligned_writer.py tests/test_aligned_writer.py
git commit -m "feat(writer): rewrite AlignedWriter for block format (G row + N I rows)"
```

---

## Task 4: Logger 改用 harvest + 更新测试

**Files:**
- Modify: `src/log/logger.py`
- Test: `tests/test_logger.py`

**策略：** Logger 主循环 `match` → `harvest`，重写 test_logger.py 适配块格式。

- [ ] **Step 1: 重写测试文件**

完全替换 `tests/test_logger.py` 内容为：

```python
import csv
from queue import Queue
from src.core.thread_control import ThreadControl
from src.core.data_types import ImuMeasurement, GnssSolution
from src.log.aligner import Aligner
from src.log.aligned_writer import AlignedWriter
from src.log.logger import Logger
import numpy as np


def _imu(sow):
    return ImuMeasurement(
        timestamp=sow, week=2046,
        accel=np.array([0.0, 0.0, 9.8]),
        gyro=np.array([0.0, 0.0, 0.0]),
    )


def _gnss(sow):
    return GnssSolution(
        timestamp=sow, week=2046,
        position=np.zeros(3), quality=5, num_sv=6,
        sd=np.zeros(3),
    )


def test_logger_writes_block_format(tmp_path):
    imu_q = Queue()
    gnss_q = Queue()
    tc = ThreadControl()
    writer = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    aligner = Aligner(imu_dt=0.01, harvest_margin=0.01)
    logger = Logger(imu_q, gnss_q, writer, aligner, tc)

    # 推 3 条 IMU + 1 个 GNSS
    # GNSS t=357254.000，收割 [357254.000, 357254.010]
    imu_q.put(_imu(357253.990))  # 首部，应丢弃
    imu_q.put(_imu(357254.000))  # 收割
    imu_q.put(_imu(357254.010))  # 收割
    gnss_q.put(_gnss(357254.000))
    # EOF
    imu_q.put(None)
    gnss_q.put(None)

    logger.start()
    logger.join(timeout=5)
    assert not logger.is_alive()

    out = tmp_path / "aligned.csv"
    assert out.exists()
    lines = out.read_text(encoding="utf-8").splitlines()
    # 1 行 G + 2 行 I = 3 行（首部 1 条已丢弃）
    assert len(lines) == 3
    assert lines[0].startswith("G,")
    assert lines[1].startswith("I,")
    assert lines[2].startswith("I,")


def test_logger_skips_gnss_when_no_imu(tmp_path):
    imu_q = Queue()
    gnss_q = Queue()
    tc = ThreadControl()
    writer = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    aligner = Aligner(imu_dt=0.01, harvest_margin=0.01)
    logger = Logger(imu_q, gnss_q, writer, aligner, tc)

    # 不推 IMU，只推 GNSS
    gnss_q.put(_gnss(357254.000))
    imu_q.put(None)
    gnss_q.put(None)

    logger.start()
    logger.join(timeout=5)

    out = tmp_path / "aligned.csv"
    lines = out.read_text(encoding="utf-8").splitlines()
    # 无 IMU，收割列表为空，无输出
    assert len(lines) == 0
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python -m pytest tests/test_logger.py -v`
Expected: FAIL（Logger 仍调用 match，返回 AlignedRow，Writer 期望 AlignedBlock）

- [ ] **Step 3: 修改 Logger 用 harvest**

在 `src/log/logger.py` 中，将 `run()` 方法中的：

```python
                aligned = self.aligner.match(gnss)
                if aligned is not None:
                    self.writer.write(aligned)
```

改为：

```python
                aligned = self.aligner.harvest(gnss)
                if aligned is not None:
                    self.writer.write(aligned)
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python -m pytest tests/test_logger.py -v`
Expected: PASS（2 个测试全通过）

运行全部测试（除 e2e）：
Run: `python -m pytest tests/ -v --ignore=tests/test_e2e_real_data.py`
Expected: 除 test_aligner 旧 match 测试可能因 AlignedRow 仍存在而通过外，其余全通过

- [ ] **Step 5: 提交**

```bash
git add src/log/logger.py tests/test_logger.py
git commit -m "feat(logger): switch from match() to harvest() for block output"
```

---

## Task 5: 清理旧代码 + 更新 E2E 测试

**Files:**
- Modify: `src/log/aligner.py`（删除 `match` 方法、`half_window` 属性、`AlignedRow` import）
- Modify: `src/core/data_types.py`（删除 `AlignedRow`）
- Modify: `tests/test_aligner.py`（删除旧 match 测试）
- Modify: `tests/test_data_types.py`（删除旧 AlignedRow 测试）
- Modify: `tests/test_e2e_real_data.py`（块格式断言）

**策略：** 删除所有旧代码（`match`、`AlignedRow`、`half_window`、`CSV_HEADER` 已在 Task 3 删除），更新 E2E 测试适配块格式。

- [ ] **Step 1: 更新 E2E 测试**

完全替换 `tests/test_e2e_real_data.py` 内容为：

```python
import subprocess
import sys


def test_e2e_real_data_generates_block_format(project_root, tmp_path):
    """使用 data/cpt_imu.csv 与 data/spp.pos 跑完整流程，验证块格式输出。

    输出应为 G 行 + N 行 I 行交替，首部无 IMU（第一个 GNSS 之前的 IMU 已丢弃）。
    """
    imu_path = project_root / "data" / "cpt_imu.csv"
    gnss_path = project_root / "data" / "spp.pos"
    if not imu_path.exists() or not gnss_path.exists():
        import pytest
        pytest.skip("real data files not available")

    config = tmp_path / "config.yaml"
    out_dir = tmp_path / "output"
    config.write_text(
        "gnss:\n"
        "  gnss_source: 'external'\n"
        f"  external_sol_path: '{gnss_path.as_posix()}'\n"
        "  external_sol_format: 'pos'\n"
        "ins:\n"
        f"  imu_data_path: '{imu_path.as_posix()}'\n"
        "  data_rate: 100\n"
        "output:\n"
        f"  output_dir: '{out_dir.as_posix()}'\n",
        encoding="utf-8",
    )

    result = subprocess.run(
        [sys.executable, str(project_root / "src" / "main.py"), str(config)],
        cwd=str(project_root),
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert result.returncode == 0, f"stderr: {result.stderr[:2000]}"

    out_file = out_dir / "aligned.csv"
    assert out_file.exists()
    lines = out_file.read_text(encoding="utf-8").splitlines()
    # 无表头
    assert lines[0].startswith("G,"), f"first line should be G row: {lines[0][:50]}"

    # 统计 G 行数（= GNSS 历元数）和 I 行数
    g_count = sum(1 for ln in lines if ln.startswith("G,"))
    i_count = sum(1 for ln in lines if ln.startswith("I,"))
    # spp.pos 有约 2485 个 GNSS 历元
    assert g_count > 1000, f"only {g_count} G rows"
    # 每个 GNSS 块应跟若干 I 行（首块可能较少，后续约 100 条）
    assert i_count > g_count, f"i_count={i_count} should > g_count={g_count}"

    # 验证块结构：每个 G 行后跟若干 I 行，直到下一个 G 行
    in_block = False
    block_imu_count = 0
    max_block_imu = 0
    for ln in lines:
        if ln.startswith("G,"):
            if in_block:
                # 上一块结束
                pass
            in_block = True
            block_imu_count = 0
        elif ln.startswith("I,"):
            assert in_block, "I row without preceding G row"
            block_imu_count += 1
        if ln.startswith("G,") and block_imu_count == 0 and max_block_imu > 0:
            max_block_imu = max(max_block_imu, 0)
        max_block_imu = max(max_block_imu, block_imu_count)

    # 至少有一个块包含 IMU
    assert max_block_imu > 0, "no block has IMU"

    # 验证 G 行字段数
    g_fields = lines[0].split(",")
    assert len(g_fields) == 11, f"G row should have 11 fields, got {len(g_fields)}"

    # 验证 I 行字段数（找第一个 I 行）
    for ln in lines:
        if ln.startswith("I,"):
            i_fields = ln.split(",")
            assert len(i_fields) == 9, f"I row should have 9 fields, got {len(i_fields)}"
            break

    # 验证 IMU accel z 轴接近重力（~9.8 m/s²）
    for ln in lines:
        if ln.startswith("I,"):
            fields = ln.split(",")
            az = float(fields[8])  # ax, ay, az 中第 3 个
            assert 9.0 < az < 10.5, f"imu az={az} not near gravity"
            break
```

- [ ] **Step 2: 删除 aligner.py 中的 match 方法**

在 `src/log/aligner.py` 中：

1. 更新文件顶部 docstring 和 import（移除 AlignedRow、numpy）：

```python
"""IMU 积攒 + GNSS 收割的匹配器。

IMU 流式推入缓冲，GNSS 到来时收割 timestamp <= t_gnss + harvest_margin 的 IMU。
首个 GNSS 到来前，timestamp < t_gnss_first 的 IMU 被丢弃（首部不对齐数据不输出）。
"""
from collections import deque
from typing import Optional, List

from src.core.data_types import AlignedBlock, GnssSolution, ImuMeasurement
```

2. 删除 `__init__` 中的 `self.half_window` 行：

```python
class Aligner:
    """IMU 积攒 + GNSS 收割的匹配器。"""

    def __init__(self, imu_dt: float, harvest_margin: float = 0.01):
        self.imu_dt = float(imu_dt)
        self.harvest_margin = float(harvest_margin)
        self.imu_buffer: deque = deque()
        self.align_started: bool = False
```

3. 删除整个 `match()` 方法（从 `def match` 到其 `return AlignedRow(...)` 结束）

4. 保留 `push_imu` 和 `harvest` 方法

- [ ] **Step 3: 删除 data_types.py 中的 AlignedRow**

在 `src/core/data_types.py` 中：

1. 删除整个 `AlignedRow` dataclass 定义（从 `@dataclass` 到 `imu_avg_gyro: np.ndarray  # [3] 窗口均值`）

2. 确认 `AlignedBlock` 保留

- [ ] **Step 4: 删除旧测试**

在 `tests/test_aligner.py` 中：

1. 删除以下旧测试函数（保留 `_imu`、`_gnss` 辅助函数和所有 harvest 测试）：
   - `test_aligner_match_single_imu_at_center`
   - `test_aligner_match_multiple_imu_in_window`
   - `test_aligner_returns_none_when_window_empty`
   - `test_aligner_purges_old_imu`
   - `test_aligner_computes_avg`

在 `tests/test_data_types.py` 中：

1. 修改顶部 import，移除 `AlignedRow`：

```python
import numpy as np
from src.core.data_types import (
    ImuMeasurement, GnssSolution, SensorData
)
```

2. 删除 `test_aligned_row_construction` 测试函数

- [ ] **Step 5: 运行全部测试验证通过**

Run: `python -m pytest tests/ -v`
Expected: 所有测试 PASS（test_data_types 3 个 + test_aligner 5 个 + test_aligned_writer 7 个 + test_logger 2 个 + 其他不变 + test_e2e_real_data 1 个）

- [ ] **Step 6: 提交**

```bash
git add src/log/aligner.py src/core/data_types.py tests/test_aligner.py tests/test_data_types.py tests/test_e2e_real_data.py
git commit -m "refactor: remove legacy match()/AlignedRow, update e2e test for block format"
```

---

## Self-Review

### 1. Spec coverage

| Spec 章节/需求 | 对应 Task |
|---|---|
| §3 数据类型 AlignedBlock | Task 1 |
| §3.2 弃用 AlignedRow | Task 5 |
| §4 Aligner.harvest + align_started + harvest_margin | Task 2 |
| §4 首部 IMU 丢弃 | Task 2 (test_harvest_discards_head_imu_before_first_gnss) |
| §4 收割边界 <= t_cut | Task 2 (test_harvest_first_block_collects_imu_at_and_after_gnss) |
| §4 连续收割不重叠 | Task 2 (test_harvest_consecutive_blocks_no_overlap) |
| §4 自定义 margin | Task 2 (test_harvest_custom_margin) |
| §5 Writer 块格式（G 行 11 列 + I 行 9 列） | Task 3 |
| §5 不写表头 | Task 3 (test_aligned_writer_creates_file_no_header) |
| §6 Logger match → harvest | Task 4 |
| §6 保留竞态修复 _wait_for_imu | Task 4（不改动该方法） |
| §8 E2E 块格式验证 | Task 5 |
| §10.3 不改动 stream/core/utility/main/config | 全程不涉及 |

### 2. Placeholder scan

- 无 TBD/TODO ✅
- 所有代码块完整 ✅
- 所有 pytest 命令有 Expected ✅

### 3. Type consistency

- `AlignedBlock.gnss: GnssSolution` ✅（Task 1 定义，Task 3/4 使用）
- `AlignedBlock.imu_list: List[ImuMeasurement]` ✅
- `Aligner.harvest(gnss) -> Optional[AlignedBlock]` ✅（Task 2 定义，Task 4 调用）
- `AlignedWriter.write(block: AlignedBlock)` ✅（Task 3 定义，Task 4 Logger 调用）
- `harvest_margin: float = 0.01` ✅（Task 2 定义，Task 4/5 测试使用）
- `align_started: bool` ✅（Task 2 定义，Task 2 测试断言）
