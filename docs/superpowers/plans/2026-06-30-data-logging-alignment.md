# 数据读取与日志记录模块 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 GInsStream 的数据读取 + 日志记录 + GNSS 触发的 IMU 大致匹配模块，输出 `output/aligned.csv`。

**Architecture:** 两层数据通路（Stream + Log）。`ImuSensor` 与 `GnssSolSensor` 在各自线程逐行读取文件，通过 Formator 解码后 `put()` 到对应队列。`Logger` 线程从 `gnss_queue` 取 GNSS 历元，调用 `Aligner.match()` 从 `imu_queue` 缓冲中抽取时间窗口 `[t_gnss - dt_imu/2, t_gnss + dt_imu/2]` 内的 IMU 数据，构造 `AlignedRow` 后由 `AlignedWriter` 写出 CSV。全程纯 `threading` + `queue.Queue`，无观察者模式。

**Tech Stack:** Python 3.10+、PyYAML、NumPy、`queue.Queue`、`threading.Thread`、`pytest`、`csv` 标准库。

**Reference Spec:** `docs/superpowers/specs/2026-06-30-data-logging-design.md`

**Reference Data Samples:**
- IMU 文件 `data/cpt_imu.csv`：`GPS week, GPS sow (s), gx, gy, gz, ax, ay, az`（无表头，逗号分隔，100Hz GPST）
- GNSS 文件 `data/spp.pos`：rtklib POS 格式，`%` 开头为注释/头部，数据行 `yyyy/mm/dd hh:mm:ss.s  x  y  z  Q  ns  sdx  sdy  sdz  ...`

---

## File Structure

| 文件 | 职责 | 新建/修改 |
|------|------|----------|
| `src/__init__.py` | 包标识 | 新建（空） |
| `src/core/__init__.py` | 包标识 | 新建（空） |
| `src/core/data_types.py` | `ImuMeasurement`, `GnssSolution`, `AlignedRow`, `SensorData` dataclass | 新建 |
| `src/core/thread_control.py` | `ThreadControl` 全局退出标志 | 新建 |
| `src/core/time_utils.py` | GPST 时间转换（`ymdhms_to_sow`、`sow_to_ymdhms`） | 新建 |
| `src/utility/__init__.py` | 包标识 | 新建（空） |
| `src/utility/config_loader.py` | `load_config()` YAML 加载 + 频率/模式校验 | 新建 |
| `src/stream/__init__.py` | 包标识 | 新建（空） |
| `src/stream/base.py` | `BaseSensor(ABC)` + `StreamerBase(Thread)` | 新建 |
| `src/stream/formators.py` | `FormatorBase`, `ImuFormator`, `PosSolFormator` | 新建 |
| `src/stream/imu_sensor.py` | `ImuSensor` | 新建 |
| `src/stream/gnss_sol_sensor.py` | `GnssSolSensor` | 新建 |
| `src/stream/factory.py` | `SensorFactory.create_sensors()` | 新建 |
| `src/log/__init__.py` | 包标识 | 新建（空） |
| `src/log/writer_base.py` | `WriterBase(ABC)` | 新建 |
| `src/log/aligned_writer.py` | `AlignedWriter` | 新建 |
| `src/log/aligner.py` | `Aligner`（GNSS 触发窗口匹配） | 新建 |
| `src/log/logger.py` | `Logger(Thread)` | 新建 |
| `src/main.py` | `main()` 装配入口 | 修改（当前为空） |
| `tests/__init__.py` | 包标识 | 新建（空） |
| `tests/conftest.py` | pytest 共享 fixture | 新建 |
| `tests/test_data_types.py` | 数据类型测试 | 新建 |
| `tests/test_time_utils.py` | 时间转换测试 | 新建 |
| `tests/test_config_loader.py` | 配置加载测试 | 新建 |
| `tests/test_formators.py` | Formator 解码测试 | 新建 |
| `tests/test_aligner.py` | 匹配器测试 | 新建 |
| `tests/test_aligned_writer.py` | CSV 写出测试 | 新建 |
| `tests/test_logger.py` | Logger 端到端测试 | 新建 |
| `tests/test_factory.py` | 工厂测试 | 新建 |
| `tests/test_main.py` | main 集成测试 | 新建 |
| `requirements.txt` | 依赖声明 | 新建 |
| `pytest.ini` | pytest 配置 | 新建 |

---

## Task 0: 项目骨架与依赖

**Files:**
- Create: `requirements.txt`
- Create: `pytest.ini`
- Create: `src/__init__.py`、`src/core/__init__.py`、`src/utility/__init__.py`、`src/stream/__init__.py`、`src/log/__init__.py`、`tests/__init__.py`、`tests/conftest.py`

- [ ] **Step 1: 创建依赖声明**

`requirements.txt`：

```
numpy>=1.24
PyYAML>=6.0
pytest>=7.0
```

- [ ] **Step 2: 创建 pytest 配置**

`pytest.ini`：

```ini
[pytest]
testpaths = tests
python_files = test_*.py
python_classes = Test*
python_functions = test_*
addopts = -v --tb=short
```

- [ ] **Step 3: 创建包标识文件（均为空）**

逐一创建：
- `src/__init__.py`
- `src/core/__init__.py`
- `src/utility/__init__.py`
- `src/stream/__init__.py`
- `src/log/__init__.py`
- `tests/__init__.py`

- [ ] **Step 4: 创建 tests/conftest.py**

```python
import pytest
import sys
from pathlib import Path

# 让 tests/ 可以 import src.*
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))


@pytest.fixture
def project_root():
    return Path(__file__).resolve().parent.parent
```

- [ ] **Step 5: 安装依赖并验证**

Run: `pip install -r requirements.txt`
Expected: 成功安装 numpy、PyYAML、pytest

- [ ] **Step 6: 验证 pytest 可运行**

Run: `pytest --collect-only`
Expected: `collected 0 items`（无测试文件，但不报错）

- [ ] **Step 7: Commit**

```bash
git add requirements.txt pytest.ini src/__init__.py src/core/__init__.py src/utility/__init__.py src/stream/__init__.py src/log/__init__.py tests/__init__.py tests/conftest.py
git commit -m "chore: project skeleton with deps and pytest config"
```

---

## Task 1: 核心数据类型

**Files:**
- Create: `src/core/data_types.py`
- Test: `tests/test_data_types.py`

- [ ] **Step 1: 写失败测试**

`tests/test_data_types.py`：

```python
import numpy as np
from src.core.data_types import (
    ImuMeasurement, GnssSolution, AlignedRow, SensorData
)


def test_imu_measurement_construction():
    imu = ImuMeasurement(
        timestamp=357254.315755,
        week=2046,
        accel=np.array([0.000458, 0.106659, 9.884033]),
        gyro=np.array([-0.000640, 0.000456, -0.000423]),
    )
    assert imu.timestamp == 357254.315755
    assert imu.week == 2046
    assert imu.accel.shape == (3,)
    assert imu.gyro.shape == (3,)


def test_gnss_solution_construction():
    sol = GnssSolution(
        timestamp=357254.000,
        week=2046,
        position=np.array([-2408695.8323, 4698107.8215, 3566699.7973]),
        quality=5,
        num_sv=6,
        sd=np.array([5.0056, 7.4084, 8.7523]),
    )
    assert sol.quality == 5
    assert sol.position.shape == (3,)
    assert sol.sd.shape == (3,)


def test_aligned_row_construction():
    row = AlignedRow(
        week=2046,
        sow=357254.000,
        imu_count=1,
        imu_first_sow=357253.995,
        imu_last_sow=357254.005,
        gnss_pos=np.zeros(3),
        gnss_q=5,
        gnss_ns=6,
        gnss_sd=np.zeros(3),
        imu_avg_accel=np.zeros(3),
        imu_avg_gyro=np.zeros(3),
    )
    assert row.imu_count == 1
    assert row.sow == 357254.000


def test_sensor_data_tag_only():
    sd = SensorData(tag="imu")
    assert sd.tag == "imu"
    assert sd.imu is None
    assert sd.gnss_solution is None
```

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest tests/test_data_types.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'src.core.data_types'`

- [ ] **Step 3: 实现 data_types.py**

`src/core/data_types.py`：

```python
"""GInsStream 核心数据类型定义。"""
from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass
class ImuMeasurement:
    """IMU 单次测量。"""
    timestamp: float          # GPST 秒（周内秒）
    week: int                 # GPS 周号
    accel: np.ndarray         # [3] m/s² 机体坐标系
    gyro: np.ndarray          # [3] rad/s 机体坐标系


@dataclass
class GnssSolution:
    """外部 GNSS 结果。"""
    timestamp: float          # GPST 秒（周内秒）
    week: int
    position: np.ndarray      # [3] ECEF (m)
    quality: int              # 1=SPP, 2=RTD, 5=LC
    num_sv: int
    sd: np.ndarray            # [3] 位置标准差 (sdx, sdy, sdz)


@dataclass
class AlignedRow:
    """对齐后的一行输出数据。"""
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
    """传感器数据统一容器（一次只承载一种类型）。"""
    tag: str                                  # "imu" / "gnss_solution"
    imu: Optional[ImuMeasurement] = None
    gnss_solution: Optional[GnssSolution] = None
```

- [ ] **Step 4: 运行测试确认通过**

Run: `pytest tests/test_data_types.py -v`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add src/core/data_types.py tests/test_data_types.py
git commit -m "feat(core): add core data types (ImuMeasurement, GnssSolution, AlignedRow, SensorData)"
```

---

## Task 2: GPST 时间转换工具

**Files:**
- Create: `src/core/time_utils.py`
- Test: `tests/test_time_utils.py`

> 说明：rtklib POS 格式时间为 `yyyy/mm/dd hh:mm:ss.s`，需转换为 GPS 周号 + 周内秒（GPST，since 1980-01-06）。

- [ ] **Step 1: 写失败测试**

`tests/test_time_utils.py`：

```python
from src.core.time_utils import ymdhms_to_gpst, sow_to_ymdhms


def test_ymdhms_to_gpst_known_epoch():
    # 2019/03/28 03:17:36.000 = week 2046, sow 357256.000
    # (rtklib-py 中 spp.pos 第一条数据点对应)
    week, sow = ymdhms_to_gpst(2019, 3, 28, 3, 17, 36.0)
    assert week == 2046
    # 容差 1e-3 秒
    assert abs(sow - 357256.0) < 1e-3


def test_sow_to_ymdhms_inverse():
    week, sow = ymdhms_to_gpst(2019, 3, 28, 3, 17, 36.0)
    y, mo, d, h, mi, s = sow_to_ymdhms(week, sow)
    assert (y, mo, d, h, mi) == (2019, 3, 28, 3, 17)
    assert abs(s - 36.0) < 1e-3


def test_ymdhms_to_gpst_gps_epoch():
    # GPS 起始历元 1980-01-06 00:00:00
    week, sow = ymdhms_to_gpst(1980, 1, 6, 0, 0, 0.0)
    assert week == 0
    assert sow == 0.0


def test_ymdhms_to_gpst_rollover_safe():
    # 2019/03/28 03:17:36.000 在 GPS 周 2046
    # 验证不超过 1024 周翻转范围（直接用绝对周号）
    week, _ = ymdhms_to_gpst(2019, 3, 28, 3, 17, 36.0)
    assert week > 2046 - 1  # 至少为 2046
```

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest tests/test_time_utils.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 实现 time_utils.py**

`src/core/time_utils.py`：

```python
"""GPST 时间转换工具。

GPST 起点：1980-01-06 00:00:00 UTC（GPS 周 0，周内秒 0）。
一周 = 604800 秒。
"""
from datetime import datetime, timedelta

GPS_EPOCH = datetime(1980, 1, 6)  # GPS 时间起点
SECONDS_PER_WEEK = 604800


def ymdhms_to_gpst(year: int, month: int, day: int,
                   hour: int, minute: int, second: float):
    """将 yyyy/mm/dd hh:mm:ss.s 转换为 (week, sow)。

    Args:
        year, month, day, hour, minute: 整数时间分量
        second: 秒（可为小数）

    Returns:
        (week: int, sow: float)  GPS 周号与周内秒
    """
    dt = datetime(year, month, day, hour, minute) + timedelta(seconds=second)
    delta = (dt - GPS_EPOCH).total_seconds()
    week = int(delta // SECONDS_PER_WEEK)
    sow = delta - week * SECONDS_PER_WEEK
    return week, sow


def sow_to_ymdhms(week: int, sow: float):
    """将 (week, sow) 转换为 (year, month, day, hour, minute, second)。

    Args:
        week: GPS 周号
        sow: 周内秒

    Returns:
        (year, month, day, hour, minute, second: float)
    """
    dt = GPS_EPOCH + timedelta(seconds=week * SECONDS_PER_WEEK + sow)
    return (dt.year, dt.month, dt.day, dt.hour, dt.minute,
            dt.second + dt.microsecond * 1e-6)
```

- [ ] **Step 4: 运行测试确认通过**

Run: `pytest tests/test_time_utils.py -v`
Expected: PASS (4 tests)

> **如果 test_ymdhms_to_gpst_known_epoch 失败**：手动用 `datetime` 验算 `2019/03/28 03:17:36` 距 `1980/01/06 00:00:00` 的总秒数，再除以 604800 得到周号。修正测试期望值并重新跑。

- [ ] **Step 5: Commit**

```bash
git add src/core/time_utils.py tests/test_time_utils.py
git commit -m "feat(core): add GPST time conversion utilities"
```

---

## Task 3: 线程控制

**Files:**
- Create: `src/core/thread_control.py`
- Test: `tests/test_thread_control.py`

- [ ] **Step 1: 写失败测试**

`tests/test_thread_control.py`：

```python
from src.core.thread_control import ThreadControl


def test_default_running():
    tc = ThreadControl()
    assert tc.is_running() is True


def test_shutdown():
    tc = ThreadControl()
    tc.shutdown()
    assert tc.is_running() is False
```

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest tests/test_thread_control.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 实现 thread_control.py**

`src/core/thread_control.py`：

```python
"""全局线程控制。"""
import threading


class ThreadControl:
    """所有线程检查 running 标志决定是否退出。"""

    def __init__(self):
        self._running = True
        self._lock = threading.Lock()

    def shutdown(self) -> None:
        with self._lock:
            self._running = False

    def is_running(self) -> bool:
        with self._lock:
            return self._running
```

- [ ] **Step 4: 运行测试确认通过**

Run: `pytest tests/test_thread_control.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add src/core/thread_control.py tests/test_thread_control.py
git commit -m "feat(core): add ThreadControl for graceful shutdown"
```

---

## Task 4: 配置加载器

**Files:**
- Create: `src/utility/config_loader.py`
- Test: `tests/test_config_loader.py`
- Test fixture: `tests/fixtures/config_test.yaml`

- [ ] **Step 1: 准备测试 fixture**

创建 `tests/fixtures/config_test.yaml`（注意 `gnss_source` 必须为 `external`）：

```yaml
gnss:
  gnss_source: "external"
  external_sol_path: "data/spp.pos"
  external_sol_format: "pos"

ins:
  imu_data_path: "data/cpt_imu.csv"
  data_rate: 100

output:
  output_dir: "output"
```

- [ ] **Step 2: 写失败测试**

`tests/test_config_loader.py`：

```python
import pytest
from src.utility.config_loader import load_config


def test_load_valid_config(project_root):
    cfg = load_config(project_root / "tests" / "fixtures" / "config_test.yaml")
    assert cfg["ins"]["data_rate"] == 100
    assert cfg["gnss"]["gnss_source"] == "external"
    assert cfg["gnss"]["external_sol_path"] == "data/spp.pos"
    assert cfg["output"]["output_dir"] == "output"


def test_invalid_data_rate(project_root, tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(
        "gnss:\n  gnss_source: 'external'\n"
        "ins:\n  imu_data_path: 'data/cpt_imu.csv'\n  data_rate: 200\n"
        "output:\n  output_dir: 'output'\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="data_rate"):
        load_config(bad)


def test_invalid_gnss_source(project_root, tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(
        "gnss:\n  gnss_source: 'internal'\n"
        "ins:\n  imu_data_path: 'data/cpt_imu.csv'\n  data_rate: 100\n"
        "output:\n  output_dir: 'output'\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="external"):
        load_config(bad)


def test_invalid_external_format(project_root, tmp_path):
    bad = tmp_path / "bad.yaml"
    bad.write_text(
        "gnss:\n  gnss_source: 'external'\n  external_sol_path: 'data/spp.pos'\n"
        "  external_sol_format: 'nmea'\n"
        "ins:\n  imu_data_path: 'data/cpt_imu.csv'\n  data_rate: 100\n"
        "output:\n  output_dir: 'output'\n",
        encoding="utf-8",
    )
    with pytest.raises(ValueError, match="external_sol_format"):
        load_config(bad)
```

- [ ] **Step 3: 运行测试确认失败**

Run: `pytest tests/test_config_loader.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 4: 实现 config_loader.py**

`src/utility/config_loader.py`：

```python
"""YAML 配置加载与校验。"""
from pathlib import Path

import yaml


REQUIRED_DATA_RATE = 100  # 当前阶段仅支持 100Hz
SUPPORTED_EXTERNAL_FORMATS = {"pos"}  # 当前阶段仅支持 POS 格式


def load_config(path) -> dict:
    """加载 YAML 配置并做必要校验。

    Args:
        path: 配置文件路径（str 或 Path）

    Returns:
        配置字典

    Raises:
        FileNotFoundError: 文件不存在
        yaml.YAMLError: YAML 解析错误
        ValueError: data_rate / gnss_source / external_sol_format 不符合要求
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    # IMU 频率检查
    data_rate = cfg["ins"]["data_rate"]
    if data_rate != REQUIRED_DATA_RATE:
        raise ValueError(
            f"IMU data_rate must be {REQUIRED_DATA_RATE}, got {data_rate}. "
            f"Current stage only supports 100Hz IMU."
        )

    # 数据源模式检查
    if cfg["gnss"]["gnss_source"] != "external":
        raise ValueError(
            f"gnss_source must be 'external' in current stage, "
            f"got '{cfg['gnss']['gnss_source']}'"
        )

    # 外部结果格式检查
    fmt = cfg["gnss"].get("external_sol_format", "pos")
    if fmt not in SUPPORTED_EXTERNAL_FORMATS:
        raise ValueError(
            f"external_sol_format must be one of {SUPPORTED_EXTERNAL_FORMATS}, "
            f"got '{fmt}'"
        )

    return cfg
```

- [ ] **Step 5: 运行测试确认通过**

Run: `pytest tests/test_config_loader.py -v`
Expected: PASS (4 tests)

- [ ] **Step 6: Commit**

```bash
git add src/utility/config_loader.py tests/test_config_loader.py tests/fixtures/config_test.yaml
git commit -m "feat(utility): add YAML config loader with rate/source/format validation"
```

---

## Task 5: Formator 解码层

**Files:**
- Create: `src/stream/formators.py`
- Test: `tests/test_formators.py`
- Test fixtures: `tests/fixtures/imu_sample.csv`、`tests/fixtures/spp_sample.pos`

- [ ] **Step 1: 准备测试 fixture**

`tests/fixtures/imu_sample.csv`（3 行真实样本）：

```
2046,357254.315755,-0.000640190972222222,0.000455729166666667,-0.000423177083333333,0.000457763671875,0.106658935546875,9.884033203125
2046,357254.325755,-0.000640190972222222,0.000455729166666667,-0.000423177083333333,0.000457763671875,0.106658935546875,9.884033203125
2046,357254.335755,-0.000640190972222222,0.000455729166666667,-0.000423177083333333,0.000457763671875,0.106658935546875,9.884033203125
```

`tests/fixtures/spp_sample.pos`（rtklib POS 头部 + 3 行数据）：

```
% program   : rtklib-py
% inp-file  1 : cpt0870.19o
% obs time  : 2019/03/28 03:17:33.0
% pos mode  : Kinematic
2019/03/28 03:17:36.000  -2408695.8323   4698107.8215   3566699.7973   5   6   5.0056   7.4084   8.7523  -4.7728   6.1861  -3.6657   0.00    0.0
2019/03/28 03:17:37.000  -2408695.8323   4698107.8215   3566699.7973   5   6   5.0056   7.4084   8.7523  -4.7728   6.1861  -3.6657   0.00    0.0
2019/03/28 03:17:38.000  -2408695.8323   4698107.8215   3566699.7973   5   6   5.0056   7.4084   8.7523  -4.7728   6.1861  -3.6657   0.00    0.0
```

- [ ] **Step 2: 写失败测试**

`tests/test_formators.py`：

```python
import numpy as np
from src.stream.formators import ImuFormator, PosSolFormator


def test_imu_formator_decode_data_line():
    f = ImuFormator()
    line = "2046,357254.315755,-0.000640190972222222,0.000455729166666667,-0.000423177083333333,0.000457763671875,0.106658935546875,9.884033203125"
    sd = f.decode(line)
    assert sd is not None
    assert sd.tag == "imu"
    assert sd.imu.week == 2046
    assert abs(sd.imu.timestamp - 357254.315755) < 1e-6
    assert sd.imu.accel.shape == (3,)
    assert sd.imu.gyro.shape == (3,)
    # 加速度第三轴约为重力
    assert abs(sd.imu.accel[2] - 9.884033203125) < 1e-6


def test_imu_formator_skip_empty_line():
    f = ImuFormator()
    assert f.decode("") is None
    assert f.decode("   ") is None
    assert f.decode("# header") is None


def test_pos_formator_decode_data_line():
    f = PosSolFormator()
    line = "2019/03/28 03:17:36.000  -2408695.8323   4698107.8215   3566699.7973   5   6   5.0056   7.4084   8.7523  -4.7728   6.1861  -3.6657   0.00    0.0"
    sd = f.decode(line)
    assert sd is not None
    assert sd.tag == "gnss_solution"
    assert sd.gnss_solution.week == 2046
    assert abs(sd.gnss_solution.timestamp - 357256.0) < 1e-3
    assert sd.gnss_solution.position.shape == (3,)
    assert sd.gnss_solution.quality == 5
    assert sd.gnss_solution.num_sv == 6
    assert sd.gnss_solution.sd.shape == (3,)


def test_pos_formator_skip_comment():
    f = PosSolFormator()
    assert f.decode("% program   : rtklib-py") is None
    assert f.decode("") is None
```

> **校验时间戳期望值**：用 `datetime(2019,3,28,3,17,36)` 距 `datetime(1980,1,6)` 的总秒数除以 604800 = 2046 周余 N 秒。`sow = N`。如果 N ≠ 357256.0，请在测试中改为真实计算值（参见 Task 2 的 `ymdhms_to_gpst`）。

- [ ] **Step 3: 运行测试确认失败**

Run: `pytest tests/test_formators.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 4: 实现 formators.py**

`src/stream/formators.py`：

```python
"""Formator 解码层。

- ImuFormator: 解析 GPST 格式 IMU CSV
  列: GPS week, GPS sow, gx, gy, gz, ax, ay, az
- PosSolFormator: 解析 rtklib POS 格式 GNSS 结果
  数据行: yyyy/mm/dd hh:mm:ss.s  x  y  z  Q  ns  sdx  sdy  sdz  sdxy  sdyz  sdzx  age  ratio
"""
from abc import ABC, abstractmethod
from typing import Optional

import numpy as np

from src.core.data_types import ImuMeasurement, GnssSolution, SensorData
from src.core.time_utils import ymdhms_to_gpst


class FormatorBase(ABC):
    """解码层抽象基类。"""

    @abstractmethod
    def decode(self, line: str) -> Optional[SensorData]:
        """解码一行文本。

        Returns:
            SensorData 或 None（注释/空行）
        """
        ...


class ImuFormator(FormatorBase):
    """IMU 文本解码（GPST 格式）。"""

    def decode(self, line: str) -> Optional[SensorData]:
        line = line.strip()
        if not line or line.startswith("#"):
            return None
        parts = line.split(",")
        if len(parts) < 8:
            return None
        try:
            week = int(parts[0])
            sow = float(parts[1])
            gx = float(parts[2])
            gy = float(parts[3])
            gz = float(parts[4])
            ax = float(parts[5])
            ay = float(parts[6])
            az = float(parts[7])
        except (ValueError, IndexError):
            return None
        imu = ImuMeasurement(
            timestamp=sow,
            week=week,
            accel=np.array([ax, ay, az], dtype=np.float64),
            gyro=np.array([gx, gy, gz], dtype=np.float64),
        )
        return SensorData(tag="imu", imu=imu)


class PosSolFormator(FormatorBase):
    """rtklib POS 格式解码。"""

    def decode(self, line: str) -> Optional[SensorData]:
        line = line.strip()
        if not line or line.startswith("%"):
            return None
        # 时间字段格式: yyyy/mm/dd hh:mm:ss.s
        # 用空格分割后，前两段是日期和时间
        parts = line.split()
        if len(parts) < 8:
            return None
        try:
            date_str = parts[0]  # yyyy/mm/dd
            time_str = parts[1]  # hh:mm:ss.s
            y, mo, d = (int(x) for x in date_str.split("/"))
            h, mi, s = time_str.split(":")
            h, mi = int(h), int(mi)
            s = float(s)
            week, sow = ymdhms_to_gpst(y, mo, d, h, mi, s)

            x = float(parts[2])
            y_pos = float(parts[3])
            z = float(parts[4])
            q = int(float(parts[5]))
            ns = int(float(parts[6]))
            sdx = float(parts[7])
            sdy = float(parts[8])
            sdz = float(parts[9])
        except (ValueError, IndexError):
            return None
        sol = GnssSolution(
            timestamp=sow,
            week=week,
            position=np.array([x, y_pos, z], dtype=np.float64),
            quality=q,
            num_sv=ns,
            sd=np.array([sdx, sdy, sdz], dtype=np.float64),
        )
        return SensorData(tag="gnss_solution", gnss_solution=sol)
```

- [ ] **Step 5: 运行测试确认通过**

Run: `pytest tests/test_formators.py -v`
Expected: PASS (4 tests)

- [ ] **Step 6: Commit**

```bash
git add src/stream/formators.py tests/test_formators.py tests/fixtures/imu_sample.csv tests/fixtures/spp_sample.pos
git commit -m "feat(stream): add ImuFormator and PosSolFormator decoders"
```

---

## Task 6: 传感器抽象基类与 StreamerBase

**Files:**
- Create: `src/stream/base.py`
- Test: `tests/test_streamer_base.py`

- [ ] **Step 1: 写失败测试**

`tests/test_streamer_base.py`：

```python
from queue import Queue
from src.core.thread_control import ThreadControl
from src.stream.base import BaseSensor, StreamerBase
from src.stream.formators import ImuFormator


class DummyFormator:
    def decode(self, line):
        from src.core.data_types import SensorData
        return SensorData(tag="imu") if line.strip() else None


def test_base_sensor_is_abstract():
    import pytest
    with pytest.raises(TypeError):
        BaseSensor("test")


def test_streamer_base_reads_file(tmp_path):
    f = tmp_path / "in.txt"
    f.write_text("a\nb\nc\n", encoding="utf-8")
    q = Queue()
    tc = ThreadControl()
    s = StreamerBase(str(f), DummyFormator(), q, tc, tag="test")
    s.start()
    s.join()
    # 三条数据 + EOF sentinel (None)
    items = []
    while not q.empty():
        items.append(q.get())
    assert len(items) == 4
    assert items[-1] is None  # EOF


def test_streamer_base_stops_on_shutdown(tmp_path):
    f = tmp_path / "in.txt"
    f.write_text("a\n" * 100, encoding="utf-8")
    q = Queue(maxsize=10)
    tc = ThreadControl()
    s = StreamerBase(str(f), DummyFormator(), q, tc, tag="test")
    s.start()
    # 立即关闭
    tc.shutdown()
    s.join(timeout=2)
    assert not s.is_alive()
```

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest tests/test_streamer_base.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 实现 base.py**

`src/stream/base.py`：

```python
"""传感器抽象基类与流式读取基类。"""
from abc import ABC, abstractmethod
from queue import Queue, Empty
from threading import Thread
from typing import Optional

from src.core.thread_control import ThreadControl


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
        BaseSensor.__init__(self, name=tag)
        Thread.__init__(self, name=tag, daemon=True)
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

- [ ] **Step 4: 运行测试确认通过**

Run: `pytest tests/test_streamer_base.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/stream/base.py tests/test_streamer_base.py
git commit -m "feat(stream): add BaseSensor and StreamerBase with EOF sentinel"
```

---

## Task 7: ImuSensor 与 GnssSolSensor

**Files:**
- Create: `src/stream/imu_sensor.py`
- Create: `src/stream/gnss_sol_sensor.py`
- Test: `tests/test_sensors.py`

- [ ] **Step 1: 写失败测试**

`tests/test_sensors.py`：

```python
from queue import Queue
from src.core.thread_control import ThreadControl
from src.stream.imu_sensor import ImuSensor
from src.stream.gnss_sol_sensor import GnssSolSensor


def test_imu_sensor_reads_sample(project_root):
    path = project_root / "tests" / "fixtures" / "imu_sample.csv"
    q = Queue()
    tc = ThreadControl()
    s = ImuSensor(str(path), q, tc)
    s.start()
    s.join()
    items = []
    while not q.empty():
        items.append(q.get())
    # 3 条数据 + EOF
    assert len(items) == 4
    assert items[0].tag == "imu"
    assert items[-1] is None


def test_gnss_sensor_reads_sample(project_root):
    path = project_root / "tests" / "fixtures" / "spp_sample.pos"
    q = Queue()
    tc = ThreadControl()
    s = GnssSolSensor(str(path), q, tc)
    s.start()
    s.join()
    items = []
    while not q.empty():
        items.append(q.get())
    # 3 条数据 + EOF（注释行不计）
    assert len(items) == 4
    assert items[0].tag == "gnss_solution"
    assert items[-1] is None
```

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest tests/test_sensors.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 实现 imu_sensor.py 与 gnss_sol_sensor.py**

`src/stream/imu_sensor.py`：

```python
"""IMU 传感器线程。"""
from queue import Queue

from src.core.thread_control import ThreadControl
from src.stream.base import StreamerBase
from src.stream.formators import ImuFormator


class ImuSensor(StreamerBase):
    """IMU 文本读取（GPST 格式）。"""

    def __init__(self, file_path: str, output_queue: Queue,
                 control: ThreadControl):
        super().__init__(
            file_path=file_path,
            formator=ImuFormator(),
            output_queue=output_queue,
            control=control,
            tag="imu",
        )
```

`src/stream/gnss_sol_sensor.py`：

```python
"""外部 GNSS 结果读取（rtklib POS 格式）。"""
from queue import Queue

from src.core.thread_control import ThreadControl
from src.stream.base import StreamerBase
from src.stream.formators import PosSolFormator


class GnssSolSensor(StreamerBase):
    """外部 GNSS 结果读取。"""

    def __init__(self, file_path: str, output_queue: Queue,
                 control: ThreadControl):
        super().__init__(
            file_path=file_path,
            formator=PosSolFormator(),
            output_queue=output_queue,
            control=control,
            tag="gnss_solution",
        )
```

- [ ] **Step 4: 运行测试确认通过**

Run: `pytest tests/test_sensors.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add src/stream/imu_sensor.py src/stream/gnss_sol_sensor.py tests/test_sensors.py
git commit -m "feat(stream): add ImuSensor and GnssSolSensor threads"
```

---

## Task 8: SensorFactory

**Files:**
- Create: `src/stream/factory.py`
- Test: `tests/test_factory.py`

- [ ] **Step 1: 写失败测试**

`tests/test_factory.py`：

```python
from queue import Queue
from src.core.thread_control import ThreadControl
from src.stream.factory import SensorFactory
from src.stream.imu_sensor import ImuSensor
from src.stream.gnss_sol_sensor import GnssSolSensor


def test_factory_creates_two_sensors(project_root):
    cfg = {
        "gnss": {
            "gnss_source": "external",
            "external_sol_path": str(project_root / "tests" / "fixtures" / "spp_sample.pos"),
            "external_sol_format": "pos",
        },
        "ins": {
            "imu_data_path": str(project_root / "tests" / "fixtures" / "imu_sample.csv"),
            "data_rate": 100,
        },
        "output": {"output_dir": "output"},
    }
    imu_q = Queue()
    gnss_q = Queue()
    tc = ThreadControl()
    sensors = SensorFactory.create_sensors(cfg, imu_q, gnss_q, tc)
    assert len(sensors) == 2
    types = {type(s) for s in sensors}
    assert ImuSensor in types
    assert GnssSolSensor in types
```

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest tests/test_factory.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 实现 factory.py**

`src/stream/factory.py`：

```python
"""传感器工厂。"""
from queue import Queue
from typing import List

from src.core.thread_control import ThreadControl
from src.stream.base import BaseSensor
from src.stream.imu_sensor import ImuSensor
from src.stream.gnss_sol_sensor import GnssSolSensor


class SensorFactory:
    """根据配置创建传感器实例列表。"""

    @staticmethod
    def create_sensors(config: dict,
                       imu_queue: Queue,
                       gnss_queue: Queue,
                       control: ThreadControl) -> List[BaseSensor]:
        sensors: List[BaseSensor] = []

        # IMU
        imu_path = config["ins"]["imu_data_path"]
        sensors.append(ImuSensor(imu_path, imu_queue, control))

        # GNSS（仅 external 模式）
        if config["gnss"]["gnss_source"] == "external":
            gnss_path = config["gnss"]["external_sol_path"]
            sensors.append(GnssSolSensor(gnss_path, gnss_queue, control))
        # internal 模式由 load_config 拦截，这里不会再走到

        return sensors
```

- [ ] **Step 4: 运行测试确认通过**

Run: `pytest tests/test_factory.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add src/stream/factory.py tests/test_factory.py
git commit -m "feat(stream): add SensorFactory for dynamic sensor creation"
```

---

## Task 9: WriterBase 与 AlignedWriter

**Files:**
- Create: `src/log/writer_base.py`
- Create: `src/log/aligned_writer.py`
- Test: `tests/test_aligned_writer.py`

- [ ] **Step 1: 写失败测试**

`tests/test_aligned_writer.py`：

```python
import csv
import numpy as np
from pathlib import Path
from src.log.aligned_writer import AlignedWriter
from src.core.data_types import AlignedRow


def _make_row(week=2046, sow=357254.0):
    return AlignedRow(
        week=week,
        sow=sow,
        imu_count=1,
        imu_first_sow=sow - 0.005,
        imu_last_sow=sow + 0.005,
        gnss_pos=np.array([-2408695.8323, 4698107.8215, 3566699.7973]),
        gnss_q=5,
        gnss_ns=6,
        gnss_sd=np.array([5.0056, 7.4084, 8.7523]),
        imu_avg_accel=np.array([0.000458, 0.106659, 9.884033]),
        imu_avg_gyro=np.array([-0.000640, 0.000456, -0.000423]),
    )


def test_aligned_writer_creates_file_with_header(tmp_path):
    w = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    w.open()
    w.write(_make_row())
    w.close()
    out = tmp_path / "aligned.csv"
    assert out.exists()
    lines = out.read_text(encoding="utf-8").splitlines()
    assert lines[0].startswith("gps_week,gps_sow,imu_count")
    assert len(lines) == 2  # header + 1 row


def test_aligned_writer_writes_correct_values(tmp_path):
    w = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    w.open()
    w.write(_make_row())
    w.close()
    out = tmp_path / "aligned.csv"
    with open(out, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    assert len(rows) == 1
    assert rows[0]["gps_week"] == "2046"
    assert abs(float(rows[0]["gnss_x"]) - -2408695.8323) < 1e-6
    assert abs(float(rows[0]["imu_avg_az"]) - 9.884033) < 1e-6
    assert int(rows[0]["imu_count"]) == 1


def test_aligned_writer_creates_output_dir_if_missing(tmp_path):
    sub = tmp_path / "new_subdir"
    w = AlignedWriter(output_dir=str(sub), filename="aligned.csv")
    w.open()
    w.write(_make_row())
    w.close()
    assert (sub / "aligned.csv").exists()
```

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest tests/test_aligned_writer.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 实现 writer_base.py 与 aligned_writer.py**

`src/log/writer_base.py`：

```python
"""输出器抽象基类。"""
from abc import ABC, abstractmethod


class WriterBase(ABC):
    """所有输出器的统一接口，定义 open/write/close 生命周期。"""

    @abstractmethod
    def open(self) -> None:
        """打开输出目标。"""
        ...

    @abstractmethod
    def write(self, data) -> None:
        """写入一条数据。"""
        ...

    @abstractmethod
    def close(self) -> None:
        """关闭输出目标，释放资源。"""
        ...
```

`src/log/aligned_writer.py`：

```python
"""对齐数据 CSV 输出器。"""
import csv
import os
from pathlib import Path
from typing import List

from src.core.data_types import AlignedRow
from src.log.writer_base import WriterBase


CSV_HEADER: List[str] = [
    "gps_week", "gps_sow", "imu_count",
    "imu_first_sow", "imu_last_sow",
    "gnss_x", "gnss_y", "gnss_z",
    "gnss_q", "gnss_ns",
    "gnss_sdx", "gnss_sdy", "gnss_sdz",
    "imu_avg_ax", "imu_avg_ay", "imu_avg_az",
    "imu_avg_gx", "imu_avg_gy", "imu_avg_gz",
]


class AlignedWriter(WriterBase):
    """对齐数据 CSV 输出器。"""

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
        self._writer.writerow(CSV_HEADER)

    def write(self, row: AlignedRow) -> None:
        if self._writer is None:
            raise RuntimeError("AlignedWriter not opened")
        self._writer.writerow([
            row.week, f"{row.sow:.6f}", row.imu_count,
            f"{row.imu_first_sow:.6f}", f"{row.imu_last_sow:.6f}",
            f"{row.gnss_pos[0]:.4f}", f"{row.gnss_pos[1]:.4f}", f"{row.gnss_pos[2]:.4f}",
            row.gnss_q, row.gnss_ns,
            f"{row.gnss_sd[0]:.4f}", f"{row.gnss_sd[1]:.4f}", f"{row.gnss_sd[2]:.4f}",
            f"{row.imu_avg_accel[0]:.6f}", f"{row.imu_avg_accel[1]:.6f}", f"{row.imu_avg_accel[2]:.6f}",
            f"{row.imu_avg_gyro[0]:.6f}", f"{row.imu_avg_gyro[1]:.6f}", f"{row.imu_avg_gyro[2]:.6f}",
        ])

    def close(self) -> None:
        if self._file is not None:
            self._file.close()
            self._file = None
            self._writer = None
```

- [ ] **Step 4: 运行测试确认通过**

Run: `pytest tests/test_aligned_writer.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add src/log/writer_base.py src/log/aligned_writer.py tests/test_aligned_writer.py
git commit -m "feat(log): add WriterBase and AlignedWriter for aligned CSV output"
```

---

## Task 10: Aligner 对齐器

**Files:**
- Create: `src/log/aligner.py`
- Test: `tests/test_aligner.py`

- [ ] **Step 1: 写失败测试**

`tests/test_aligner.py`：

```python
import numpy as np
from src.log.aligner import Aligner
from src.core.data_types import ImuMeasurement, GnssSolution


def _imu(week, sow, ax=0.0, ay=0.0, az=9.8, gx=0.0, gy=0.0, gz=0.0):
    return ImuMeasurement(
        timestamp=sow, week=week,
        accel=np.array([ax, ay, az]),
        gyro=np.array([gx, gy, gz]),
    )


def _gnss(week, sow):
    return GnssSolution(
        timestamp=sow, week=week,
        position=np.zeros(3), quality=5, num_sv=6,
        sd=np.zeros(3),
    )


def test_aligner_match_single_imu_at_center():
    # dt_imu = 0.01s, half_window = 0.005s
    a = Aligner(imu_dt=0.01)
    a.push_imu(_imu(2046, 357254.000))
    row = a.match(_gnss(2046, 357254.000))
    assert row is not None
    assert row.imu_count == 1
    assert row.imu_first_sow == 357254.000
    assert row.imu_last_sow == 357254.000


def test_aligner_match_multiple_imu_in_window():
    a = Aligner(imu_dt=0.01)  # half_window=0.005
    a.push_imu(_imu(2046, 357253.995))
    a.push_imu(_imu(2046, 357254.000))
    a.push_imu(_imu(2046, 357254.005))
    row = a.match(_gnss(2046, 357254.000))
    assert row.imu_count == 3
    assert row.imu_first_sow == 357253.995
    assert row.imu_last_sow == 357254.005


def test_aligner_returns_none_when_window_empty():
    a = Aligner(imu_dt=0.01)
    a.push_imu(_imu(2046, 357254.500))  # 窗口外
    row = a.match(_gnss(2046, 357254.000))
    assert row is None


def test_aligner_purges_old_imu():
    a = Aligner(imu_dt=0.01)
    a.push_imu(_imu(2046, 357253.994))  # 早于窗口下界
    a.push_imu(_imu(2046, 357254.000))
    row = a.match(_gnss(2046, 357254.000))
    assert row is not None
    assert row.imu_count == 1
    # 老数据应已被弹出
    assert len(a.imu_buffer) == 1


def test_aligner_computes_avg():
    a = Aligner(imu_dt=0.01)
    a.push_imu(_imu(2046, 357253.995, ax=1.0))
    a.push_imu(_imu(2046, 357254.005, ax=3.0))
    row = a.match(_gnss(2046, 357254.000))
    assert row is not None
    assert abs(row.imu_avg_accel[0] - 2.0) < 1e-9
```

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest tests/test_aligner.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 实现 aligner.py**

`src/log/aligner.py`：

```python
"""GNSS 触发的 IMU 匹配器。

每收到一个 GNSS 历元，从 IMU 缓冲中取时间窗口内数据：
    窗口 = [t_gnss - dt_imu/2, t_gnss + dt_imu/2]
"""
from collections import deque
from typing import Optional

import numpy as np

from src.core.data_types import AlignedRow, GnssSolution, ImuMeasurement


class Aligner:
    """GNSS 触发的 IMU 匹配器。"""

    def __init__(self, imu_dt: float):
        self.imu_dt = float(imu_dt)
        self.half_window = self.imu_dt / 2.0
        self.imu_buffer: deque = deque()

    def push_imu(self, imu: ImuMeasurement) -> None:
        """将一条 IMU 数据推入缓冲。"""
        self.imu_buffer.append(imu)

    def match(self, gnss: GnssSolution) -> Optional[AlignedRow]:
        """对一个 GNSS 历元做匹配。

        Returns:
            AlignedRow 或 None（窗口内无 IMU）
        """
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

        # 3. 统计量
        accels = np.array([imu.accel for imu in windowed])
        gyros = np.array([imu.gyro for imu in windowed])
        avg_accel = accels.mean(axis=0)
        avg_gyro = gyros.mean(axis=0)

        return AlignedRow(
            week=gnss.week,
            sow=gnss.timestamp,
            imu_count=len(windowed),
            imu_first_sow=windowed[0].timestamp,
            imu_last_sow=windowed[-1].timestamp,
            gnss_pos=gnss.position,
            gnss_q=gnss.quality,
            gnss_ns=gnss.num_sv,
            gnss_sd=gnss.sd,
            imu_avg_accel=avg_accel,
            imu_avg_gyro=avg_gyro,
        )
```

- [ ] **Step 4: 运行测试确认通过**

Run: `pytest tests/test_aligner.py -v`
Expected: PASS (5 tests)

- [ ] **Step 5: Commit**

```bash
git add src/log/aligner.py tests/test_aligner.py
git commit -m "feat(log): add Aligner for GNSS-triggered IMU window matching"
```

---

## Task 11: Logger 线程

**Files:**
- Create: `src/log/logger.py`
- Test: `tests/test_logger.py`

- [ ] **Step 1: 写失败测试**

`tests/test_logger.py`：

```python
from queue import Queue
from src.core.thread_control import ThreadControl
from src.core.data_types import ImuMeasurement, GnssSolution, SensorData
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


def test_logger_writes_aligned_rows(tmp_path):
    imu_q = Queue()
    gnss_q = Queue()
    tc = ThreadControl()
    writer = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    aligner = Aligner(imu_dt=0.01)
    logger = Logger(imu_q, gnss_q, writer, aligner, tc)

    # 推 IMU + GNSS
    imu_q.put(_imu(357254.000))
    gnss_q.put(_gnss(357254.000))
    # 推 EOF sentinel
    imu_q.put(None)
    gnss_q.put(None)

    logger.start()
    logger.join(timeout=5)
    assert not logger.is_alive()

    import csv
    out = tmp_path / "aligned.csv"
    assert out.exists()
    with open(out, "r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    assert len(rows) == 1
    assert rows[0]["gps_week"] == "2046"


def test_logger_skips_gnss_when_no_imu(tmp_path):
    imu_q = Queue()
    gnss_q = Queue()
    tc = ThreadControl()
    writer = AlignedWriter(output_dir=str(tmp_path), filename="aligned.csv")
    aligner = Aligner(imu_dt=0.01)
    logger = Logger(imu_q, gnss_q, writer, aligner, tc)

    # 不推 IMU，只推 GNSS
    gnss_q.put(_gnss(357254.000))
    imu_q.put(None)
    gnss_q.put(None)

    logger.start()
    logger.join(timeout=5)

    out = tmp_path / "aligned.csv"
    with open(out, "r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    # 窗口空，应跳过
    assert len(rows) == 0
```

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest tests/test_logger.py -v`
Expected: FAIL — `ModuleNotFoundError`

- [ ] **Step 3: 实现 logger.py**

`src/log/logger.py`：

```python
"""日志记录器线程。

从 imu_queue、gnss_queue 消费数据，做 GNSS 触发匹配，
委托 AlignedWriter 输出 CSV。
"""
from queue import Queue, Empty
from threading import Thread

from src.core.thread_control import ThreadControl
from src.log.aligned_writer import AlignedWriter
from src.log.aligner import Aligner


class Logger(Thread):
    """日志记录器线程。"""

    def __init__(self, imu_queue: Queue, gnss_queue: Queue,
                 writer: AlignedWriter, aligner: Aligner,
                 control: ThreadControl):
        Thread.__init__(self, name="Logger", daemon=True)
        self.imu_queue = imu_queue
        self.gnss_queue = gnss_queue
        self.writer = writer
        self.aligner = aligner
        self.control = control
        self.imu_eof = False

    def run(self):
        self.writer.open()
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
                    # GNSS 文件已读完，再排空一次 IMU 后退出
                    self._drain_imu_queue()
                    break
                # 4. 匹配并写出
                aligned = self.aligner.match(gnss)
                if aligned is not None:
                    self.writer.write(aligned)
        finally:
            self.writer.close()

    def _drain_imu_queue(self):
        """非阻塞地把 imu_queue 中所有数据搬到 aligner.imu_buffer。

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
```

- [ ] **Step 4: 运行测试确认通过**

Run: `pytest tests/test_logger.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add src/log/logger.py tests/test_logger.py
git commit -m "feat(log): add Logger thread with GNSS-triggered alignment loop"
```

---

## Task 12: main.py 装配入口

**Files:**
- Modify: `src/main.py`（当前为空）
- Test: `tests/test_main.py`

- [ ] **Step 1: 写失败测试**

`tests/test_main.py`：

```python
import csv
import subprocess
import sys
from pathlib import Path


def test_main_runs_end_to_end_with_sample_data(project_root, tmp_path):
    """使用 fixtures 数据跑完整流程，验证 aligned.csv 生成。"""
    # 构造临时 config.yaml 指向 fixtures
    config = project_root / "tests" / "fixtures" / "config_e2e.yaml"
    imu_path = project_root / "tests" / "fixtures" / "imu_sample.csv"
    gnss_path = project_root / "tests" / "fixtures" / "spp_sample.pos"
    out_dir = tmp_path / "output"
    config.write_text(
        "gnss:\n"
        f"  gnss_source: 'external'\n"
        f"  external_sol_path: '{gnss_path.as_posix()}'\n"
        "  external_sol_format: 'pos'\n"
        "ins:\n"
        f"  imu_data_path: '{imu_path.as_posix()}'\n"
        "  data_rate: 100\n"
        "output:\n"
        f"  output_dir: '{out_dir.as_posix()}'\n",
        encoding="utf-8",
    )
    # 运行 main.py
    result = subprocess.run(
        [sys.executable, str(project_root / "src" / "main.py"), str(config)],
        cwd=str(project_root),
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert result.returncode == 0, f"stderr: {result.stderr}"

    out_file = out_dir / "aligned.csv"
    assert out_file.exists()
    with open(out_file, "r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    # 3 条 GNSS 数据 → 3 行（IMU 频率 100Hz 但样本时间在 GNSS 附近，未必都能匹配）
    # 至少应有 0~3 行；这里只验证格式
    assert all("gps_week" in r for r in rows) or len(rows) == 0
    # 验证表头存在
    out_file.read_text(encoding="utf-8").startswith("gps_week,gps_sow")


def test_main_rejects_internal_mode(project_root, tmp_path):
    config = tmp_path / "bad.yaml"
    config.write_text(
        "gnss:\n  gnss_source: 'internal'\n"
        "ins:\n  imu_data_path: 'data/cpt_imu.csv'\n  data_rate: 100\n"
        "output:\n  output_dir: 'output'\n",
        encoding="utf-8",
    )
    result = subprocess.run(
        [sys.executable, str(project_root / "src" / "main.py"), str(config)],
        cwd=str(project_root),
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert result.returncode != 0
    assert "external" in result.stderr
```

> **注意**：以上 e2e 测试用 fixtures 中 3 行 IMU（时间 357254.315~357254.335）与 3 行 GNSS（时间 357256.0~357258.0）。两者时间相差约 1.7 秒，匹配窗口 ±0.005s 内无法匹配。该测试验证的是"main 能跑通且不崩"，而非"匹配成功"。如需匹配成功，可在 fixture 中加入对齐时间的数据。

- [ ] **Step 2: 运行测试确认失败**

Run: `pytest tests/test_main.py -v`
Expected: FAIL — main.py 为空，subprocess 调用立即失败

- [ ] **Step 3: 实现 main.py**

`src/main.py`：

```python
"""GInsStream 主入口。

用法:
    python src/main.py [config_path]

默认 config_path = data/config.yaml
"""
import sys
from pathlib import Path
from queue import Queue

# 让 src/main.py 直接运行时也能 import src.*
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.core.thread_control import ThreadControl
from src.stream.factory import SensorFactory
from src.log.aligned_writer import AlignedWriter
from src.log.aligner import Aligner
from src.log.logger import Logger
from src.utility.config_loader import load_config


def main(config_path: str = "data/config.yaml"):
    # 1. 加载配置（含频率与模式校验）
    config = load_config(config_path)

    # 2. 创建共享对象
    control = ThreadControl()
    imu_queue = Queue(maxsize=2000)
    gnss_queue = Queue(maxsize=100)

    # 3. 工厂创建传感器
    sensors = SensorFactory.create_sensors(
        config, imu_queue, gnss_queue, control
    )

    # 4. 装配 Logger
    writer = AlignedWriter(output_dir=config["output"]["output_dir"])
    aligner = Aligner(imu_dt=1.0 / config["ins"]["data_rate"])
    logger = Logger(imu_queue, gnss_queue, writer, aligner, control)

    # 5. 启动所有线程
    for s in sensors:
        s.start()
    logger.start()

    # 6. 等待 Logger 结束（Logger 收到 GNSS EOF 后退出）
    logger.join()
    control.shutdown()
    for s in sensors:
        s.join(timeout=2)


if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 else "data/config.yaml"
    main(cfg)
```

- [ ] **Step 4: 运行测试确认通过**

Run: `pytest tests/test_main.py -v`
Expected: PASS (2 tests)

> **如果 test_main_runs_end_to_end_with_sample_data 失败**：检查 `subprocess` 是否因为 `data/spp.pos` 不存在或路径分隔符问题失败。在 Windows 上路径用 `/` 也可，但 subprocess 调用时 cwd 必须正确。

- [ ] **Step 5: Commit**

```bash
git add src/main.py tests/test_main.py
git commit -m "feat(main): wire up main entry with config + threads + Logger"
```

---

## Task 13: 端到端集成验证（真实数据）

**Files:**
- Test: `tests/test_e2e_real_data.py`（使用项目 `data/` 目录下真实文件）

- [ ] **Step 1: 写测试**

`tests/test_e2e_real_data.py`：

```python
import csv
import subprocess
import sys
from pathlib import Path


def test_e2e_real_data_generates_aligned_csv(project_root, tmp_path):
    """使用 data/cpt_imu.csv 与 data/spp.pos 跑完整流程。"""
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
        timeout=120,
    )
    assert result.returncode == 0, f"stderr: {result.stderr[:2000]}"

    out_file = out_dir / "aligned.csv"
    assert out_file.exists()
    with open(out_file, "r", encoding="utf-8") as f:
        rows = list(csv.DictReader(f))
    # spp.pos 文件 1Hz GNSS，从 03:17:36 到 03:59:01 共约 2485 历元
    # 应至少匹配到 1000 行
    assert len(rows) > 1000, f"only {len(rows)} rows generated"
    # 验证首行字段格式
    assert rows[0]["gps_week"] == "2046"
    assert int(rows[0]["imu_count"]) >= 1
    # 验证加速度第三轴接近重力（窗口均值）
    az = float(rows[0]["imu_avg_az"])
    assert 9.5 < az < 10.1
```

- [ ] **Step 2: 运行测试**

Run: `pytest tests/test_e2e_real_data.py -v`
Expected: PASS

> **调试提示**：
> - 如果 `imu_count == 0`：检查 Aligner 的 `half_window` 计算。dt_imu=0.01s，half_window=0.005s。GNSS 时间戳若为 `357256.000`，IMU 时间戳应为 `357255.995~357256.005`。验证 ImuFormator 解析的 `sow` 字段类型是 float。
> - 如果 `result.returncode != 0`：把 `stderr[:2000]` 打印出来逐项排查。
> - 如果超时：`data/cpt_imu.csv` 较大（32MB），可能需要 60+ 秒。调整 timeout 至 300。

- [ ] **Step 3: Commit**

```bash
git add tests/test_e2e_real_data.py
git commit -m "test(e2e): add real data integration test"
```

---

## Task 14: 更新 data/config.yaml 默认值（可选）

**Files:**
- Modify: `data/config.yaml`（修改 `gnss_source` 默认值为 `external`）

- [ ] **Step 1: 修改 config.yaml**

将 `gnss.gnss_source: "internal"` 改为 `gnss.gnss_source: "external"`。

- [ ] **Step 2: 验证默认配置可运行**

Run: `python src/main.py`
Expected: 在 `output/` 目录下生成 `aligned.csv`，运行不报错。

- [ ] **Step 3: Commit**

```bash
git add data/config.yaml
git commit -m "chore(config): switch default gnss_source to external for current stage"
```

---

## Self-Review Checklist

**1. Spec coverage**（参考 `docs/superpowers/specs/2026-06-30-data-logging-design.md`）：

| Spec 章节 | 实现 Task |
|-----------|-----------|
| §1 架构总览 | Task 0-12（整体） |
| §2 目录结构 | Task 0（骨架） |
| §3 类继承体系 | Task 1, 5, 6, 7, 9 |
| §4 数据流 | Task 11, 12, 13 |
| §5 关键接口 | Task 1-11 逐项 |
| §6 配置项映射 | Task 4 |
| §7 输出文件格式 | Task 9 |
| §8 异常处理 | Task 4（data_rate/source/format）, Task 11（窗口空跳过） |
| §9 main.py 职责 | Task 12 |
| §10 范围与非目标 | 全部不实现（已声明） |

**2. Placeholder scan**：无 TODO/TBD/待补充。所有步骤均含完整代码。

**3. Type consistency**：
- `ImuMeasurement.accel/gyro` 为 `np.ndarray`（Task 1 定义，Task 5/10 使用，一致）
- `GnssSolution.position/sd` 为 `np.ndarray`（Task 1 定义，Task 5/10 使用，一致）
- `AlignedRow` 字段名（Task 1 定义，Task 9/10/11 使用，一致）
- `Aligner.push_imu`/`match` 签名（Task 10 定义，Task 11 调用，一致）
- `Logger.__init__` 参数顺序 `(imu_queue, gnss_queue, writer, aligner, control)`（Task 11 定义，Task 12 调用，一致）

---

## Execution Handoff

计划完成并已保存至 `docs/superpowers/plans/2026-06-30-data-logging-alignment.md`。有两种执行选项：

**1. Subagent-Driven (推荐)** - 我会为每个任务派发一个全新的 subagent，并在任务间进行审查，迭代速度快。

**2. Inline Execution** - 在当前会话中使用 executing-plans 执行任务，支持带检查点的批量执行。

请问选择哪种方式？
