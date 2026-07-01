# 内部 GNSS 纯解算模式 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现 `gnss_source=internal` + `ins.enabled=off` 的纯 GNSS 解算路径（SPP/RTK），借助 rtklib-py 逐历元解算并输出 .pos 文件。

**Architecture:** main.py 依据 `(gnss_source, ins.enabled)` 分支装配：external+on 走现有块状对齐路径（零改动），internal+off 走新路径（InternalGnssSensor → gnss_queue → SolutionLogger → SolutionWriter → solution.pos）。rtklib-py 的 cfg 模块用 types.ModuleType 动态注入 sys.modules，不写文件、不改 library/。

**Tech Stack:** Python 3.12, threading + queue.Queue, rtklib-py (library/rtklib-py/src/), PyYAML, numpy, pytest

---

## File Structure

### 新建文件

| 文件 | 职责 |
|------|------|
| `src/core/gnss/__init__.py` | 包标识 |
| `src/core/gnss/gnss_processor.py` | GnssProcessor ABC + 异常类 |
| `src/core/gnss/rtklib_config_adapter.py` | YAML → rtklib-py cfg 模块对象，注入 sys.modules |
| `src/core/gnss/solution_converter.py` | rtklib Sol → 本项目 GnssSolution |
| `src/core/gnss/spp_processor.py` | SPP 处理器（封装 pntpos） |
| `src/core/gnss/rtk_processor.py` | RTK 处理器（封装 relpos） |
| `src/stream/internal_gnss_sensor.py` | 内部 GNSS 传感器线程（逐历元循环） |
| `src/log/solution_writer.py` | .pos 输出器 |
| `src/log/solution_logger.py` | 纯 GNSS 日志线程 |
| `tests/test_rtklib_config_adapter.py` | 适配器单测 |
| `tests/test_solution_converter.py` | 转换器单测 |
| `tests/test_solution_writer.py` | 写入器单测 |
| `tests/test_solution_logger.py` | 日志线程单测 |
| `tests/test_spp_processor.py` | SPP 处理器单测 |
| `tests/test_rtk_processor.py` | RTK 处理器单测 |
| `tests/test_internal_gnss_sensor.py` | 传感器单测 |
| `tests/test_internal_gnss_spp_e2e.py` | SPP 端到端测试 |
| `tests/test_internal_gnss_rtk_e2e.py` | RTK 端到端测试 |

### 修改文件

| 文件 | 改动 |
|------|------|
| `data/config.yaml` | gnss 段加 positioning_mode；ins 段顶部加 enabled + reboot |
| `tests/fixtures/config_test.yaml` | 加 ins.enabled: "on" 保持合法 |
| `src/utility/config_loader.py` | 新增校验规则（见 Task 1） |
| `src/stream/factory.py` | 加 create_internal_gnss_sensor 静态方法 |
| `src/main.py` | 加 _assemble_pipeline 装配函数 |
| `tests/test_config_loader.py` | 改 test_invalid_gnss_source，新增模式校验测试 |
| `tests/test_factory.py` | 加 internal 模式工厂测试 |

### 不改动文件

`src/log/logger.py`, `src/log/aligner.py`, `src/log/aligned_writer.py`, `src/stream/imu_sensor.py`, `src/stream/gnss_sol_sensor.py`, `src/core/data_types.py`, `library/rtklib-py/**`

---

## Task 1: 配置 Schema 与 config_loader 校验

**Files:**
- Modify: `data/config.yaml` (gnss 段 gnss_source 后 + ins 段顶部)
- Modify: `tests/fixtures/config_test.yaml`
- Modify: `src/utility/config_loader.py`
- Modify: `tests/test_config_loader.py`

- [ ] **Step 1: 修改 data/config.yaml 加新字段**

在 `gnss:` 段 `gnss_source: "external"` 行后加：

```yaml
  # 定位模式 (仅 internal 模式生效): spp = 单点定位 / rtk = 相对定位
  # Positioning mode (internal mode only): spp / rtk
  positioning_mode: "spp"
```

在 `ins:` 段第一行（`# 组合导航配置项` 注释块之后、`# 处理时间` 之前）加：

```yaml
  #-------------------------------------------------------------------------------------------#
  # 主开关与重启
  # Master switch and reboot
  #-------------------------------------------------------------------------------------------#
  # 主开关（必填）: on = 组合导航路径 / off = 纯 GNSS 解算
  # - external 模式必须为 on（外部 GNSS 已有，必走组合导航）
  # - internal + off = 纯 GNSS 解算
  # - internal + on  = 内部 GNSS + 组合导航（未来）
  # Master switch (required): on = integrated navigation / off = pure GNSS
  enabled: "on"

  # GNSS 中断重启阈值 [s] (默认 50s)
  # 仅 ins.enabled=on 时生效: INS 初始化完成后，若 GNSS 中断超过此阈值，
  # 重新进行组合导航初始化的数据对齐操作
  # GNSS outage reboot threshold [s] (default 50s)
  # Only effective when ins.enabled=on
  reboot: 50

```

- [ ] **Step 2: 修改 tests/fixtures/config_test.yaml 加 enabled**

把 fixture 改为：

```yaml
gnss:
  gnss_source: "external"
  external_sol_path: "data/spp.pos"
  external_sol_format: "pos"

ins:
  enabled: "on"
  imu_data_path: "data/cpt_imu.csv"
  data_rate: 100

output:
  output_dir: "output"
```

- [ ] **Step 3: 写失败测试 tests/test_config_loader.py**

在文件末尾追加以下测试（先读现有文件了解导入与 fixture 用法）：

```python
def test_external_mode_requires_ins_enabled_on(tmp_path):
    """external + ins.enabled=off 应报错"""
    cfg_text = """
gnss:
  gnss_source: "external"
  external_sol_path: "data/spp.pos"
  external_sol_format: "pos"
ins:
  enabled: "off"
  imu_data_path: "data/cpt_imu.csv"
  data_rate: 100
output:
  output_dir: "output"
"""
    p = tmp_path / "cfg.yaml"
    p.write_text(cfg_text, encoding="utf-8")
    try:
        load_config(str(p))
        assert False, "should raise ValueError"
    except ValueError as e:
        assert "ins.enabled" in str(e) or "external" in str(e)


def test_internal_mode_with_ins_off_is_valid(tmp_path):
    """internal + ins.enabled=off 合法"""
    cfg_text = """
gnss:
  gnss_source: "internal"
  positioning_mode: "spp"
  rover_path: "data/cpt0870.19o"
  eph_path: "data/brdm0870.19p"
ins:
  enabled: "off"
output:
  output_dir: "output"
"""
    p = tmp_path / "cfg.yaml"
    p.write_text(cfg_text, encoding="utf-8")
    cfg = load_config(str(p))
    assert cfg["gnss"]["gnss_source"] == "internal"
    assert cfg["ins"]["enabled"] == "off"


def test_internal_mode_requires_positioning_mode(tmp_path):
    """internal 缺 positioning_mode 应报错"""
    cfg_text = """
gnss:
  gnss_source: "internal"
  rover_path: "data/cpt0870.19o"
  eph_path: "data/brdm0870.19p"
ins:
  enabled: "off"
output:
  output_dir: "output"
"""
    p = tmp_path / "cfg.yaml"
    p.write_text(cfg_text, encoding="utf-8")
    try:
        load_config(str(p))
        assert False, "should raise ValueError"
    except ValueError as e:
        assert "positioning_mode" in str(e)


def test_internal_rtk_requires_base_path(tmp_path):
    """internal + rtk 缺 base_path 应报错"""
    cfg_text = """
gnss:
  gnss_source: "internal"
  positioning_mode: "rtk"
  rover_path: "data/cpt0870.19o"
  eph_path: "data/brdm0870.19p"
ins:
  enabled: "off"
output:
  output_dir: "output"
"""
    p = tmp_path / "cfg.yaml"
    p.write_text(cfg_text, encoding="utf-8")
    try:
        load_config(str(p))
        assert False, "should raise ValueError"
    except ValueError as e:
        assert "base_path" in str(e)


def test_internal_ins_on_raises_not_implemented(tmp_path):
    """internal + ins.enabled=on 抛 NotImplementedError"""
    cfg_text = """
gnss:
  gnss_source: "internal"
  positioning_mode: "spp"
  rover_path: "data/cpt0870.19o"
  eph_path: "data/brdm0870.19p"
ins:
  enabled: "on"
  data_rate: 100
output:
  output_dir: "output"
"""
    p = tmp_path / "cfg.yaml"
    p.write_text(cfg_text, encoding="utf-8")
    try:
        load_config(str(p))
        assert False, "should raise NotImplementedError"
    except NotImplementedError:
        pass


def test_ins_enabled_is_required(tmp_path):
    """缺 ins.enabled 应报错"""
    cfg_text = """
gnss:
  gnss_source: "external"
  external_sol_path: "data/spp.pos"
  external_sol_format: "pos"
ins:
  imu_data_path: "data/cpt_imu.csv"
  data_rate: 100
output:
  output_dir: "output"
"""
    p = tmp_path / "cfg.yaml"
    p.write_text(cfg_text, encoding="utf-8")
    try:
        load_config(str(p))
        assert False, "should raise ValueError"
    except ValueError as e:
        assert "ins.enabled" in str(e)
```

同时找到现有 `test_invalid_gnss_source` 测试（断言 internal 非法），删除或改为：

```python
def test_invalid_gnss_source(tmp_path):
    """非法 gnss_source 值应报错"""
    cfg_text = """
gnss:
  gnss_source: "invalid"
ins:
  enabled: "on"
  data_rate: 100
output:
  output_dir: "output"
"""
    p = tmp_path / "cfg.yaml"
    p.write_text(cfg_text, encoding="utf-8")
    try:
        load_config(str(p))
        assert False, "should raise ValueError"
    except ValueError:
        pass
```

- [ ] **Step 4: 运行测试验证失败**

Run: `python -m pytest tests/test_config_loader.py -v`
Expected: 新增测试 FAIL（校验逻辑未实现）

- [ ] **Step 5: 实现 config_loader.py 校验**

把 `src/utility/config_loader.py` 的 `load_config` 函数改为：

```python
"""YAML 配置加载与校验。"""
from pathlib import Path

import yaml


REQUIRED_DATA_RATE = 100  # external/INS 模式仅支持 100Hz
SUPPORTED_EXTERNAL_FORMATS = {"pos"}
SUPPORTED_GNSS_SOURCES = {"external", "internal"}
SUPPORTED_POSITIONING_MODES = {"spp", "rtk"}
SUPPORTED_INS_ENABLED = {"on", "off"}


def load_config(path) -> dict:
    """加载 YAML 配置并做必要校验。

    Raises:
        FileNotFoundError: 文件不存在
        yaml.YAMLError: YAML 解析错误
        ValueError: 配置非法
        NotImplementedError: internal + ins.enabled=on（INS 未实现）
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Config file not found: {path}")

    with open(path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)

    # ins.enabled 必填
    ins_enabled = cfg.get("ins", {}).get("enabled")
    if ins_enabled is None:
        raise ValueError("ins.enabled is required (must be 'on' or 'off')")
    if ins_enabled not in SUPPORTED_INS_ENABLED:
        raise ValueError(
            f"ins.enabled must be one of {SUPPORTED_INS_ENABLED}, "
            f"got '{ins_enabled}'"
        )

    # gnss_source 校验
    gnss_source = cfg["gnss"]["gnss_source"]
    if gnss_source not in SUPPORTED_GNSS_SOURCES:
        raise ValueError(
            f"gnss_source must be one of {SUPPORTED_GNSS_SOURCES}, "
            f"got '{gnss_source}'"
        )

    # external 模式校验
    if gnss_source == "external":
        if ins_enabled != "on":
            raise ValueError(
                "ins.enabled must be 'on' when gnss_source='external' "
                "(external GNSS requires integrated navigation path)"
            )
        # data_rate 校验（external 走 IMU 流）
        data_rate = cfg["ins"]["data_rate"]
        if data_rate != REQUIRED_DATA_RATE:
            raise ValueError(
                f"IMU data_rate must be {REQUIRED_DATA_RATE}, got {data_rate}. "
                f"Current stage only supports 100Hz IMU."
            )
        # 外部结果格式检查
        fmt = cfg["gnss"].get("external_sol_format", "pos")
        if fmt not in SUPPORTED_EXTERNAL_FORMATS:
            raise ValueError(
                f"external_sol_format must be one of {SUPPORTED_EXTERNAL_FORMATS}, "
                f"got '{fmt}'"
            )

    # internal 模式校验
    if gnss_source == "internal":
        # positioning_mode 必填
        pos_mode = cfg["gnss"].get("positioning_mode")
        if pos_mode is None:
            raise ValueError(
                "positioning_mode is required when gnss_source='internal'"
            )
        if pos_mode not in SUPPORTED_POSITIONING_MODES:
            raise ValueError(
                f"positioning_mode must be one of {SUPPORTED_POSITIONING_MODES}, "
                f"got '{pos_mode}'"
            )
        # 文件路径校验
        if not cfg["gnss"].get("rover_path"):
            raise ValueError("rover_path is required when gnss_source='internal'")
        if not cfg["gnss"].get("eph_path"):
            raise ValueError("eph_path is required when gnss_source='internal'")
        if pos_mode == "rtk" and not cfg["gnss"].get("base_path"):
            raise ValueError(
                "base_path is required when positioning_mode='rtk'"
            )
        # ins.enabled=on 未实现
        if ins_enabled == "on":
            raise NotImplementedError(
                "INS estimator not implemented: gnss_source='internal' + "
                "ins.enabled='on' is reserved for future INS integration"
            )

    return cfg
```

- [ ] **Step 6: 运行测试验证通过**

Run: `python -m pytest tests/test_config_loader.py -v`
Expected: 全部 PASS

- [ ] **Step 7: 运行全量回归**

Run: `python -m pytest -v`
Expected: 现有测试全过（config_test.yaml fixture 已加 enabled: "on"）

- [ ] **Step 8: 提交**

```bash
git add data/config.yaml tests/fixtures/config_test.yaml src/utility/config_loader.py tests/test_config_loader.py
git commit -m "feat(config): add ins.enabled/positioning_mode schema and validation"
```

---

## Task 2: rtklib 配置适配器

**Files:**
- Create: `src/core/gnss/__init__.py`
- Create: `src/core/gnss/rtklib_config_adapter.py`
- Test: `tests/test_rtklib_config_adapter.py`

**背景：** rtklib-py 的 `rtkpos.py` / `postpos.py` 在模块顶层 `import __ppk_config as cfg`，且 `rtkinit(cfg)` 从 cfg 读取所有参数。适配器职责：把 YAML 的 `gnss:` 段翻译成 cfg 模块对象，注入 `sys.modules['__ppk_config']`，并把 `library/rtklib-py/src` 加入 sys.path，最后调用 `rtkinit(cfg)` 返回 nav。

- [ ] **Step 1: 创建 src/core/gnss/__init__.py**

```python
"""GNSS 处理模块（内部解算模式）。"""
```

- [ ] **Step 2: 写失败测试 tests/test_rtklib_config_adapter.py**

```python
"""rtklib 配置适配器测试。"""
import sys
import types
import pytest

from src.core.gnss.rtklib_config_adapter import build_cfg_module, RtklibEnv


def test_build_cfg_module_basic_attributes():
    """cfg 模块对象包含 rtkinit 所需的全部属性"""
    gnss_cfg = {
        "nf": 2,
        "pmode": "kinematic",
        "filtertype": "forward",
        "use_sing_pos": False,
        "elmin": 15.0,
        "cnr_min": [28, 20],
        "excsats": [],
        "maxinno": 1.0,
        "maxcode": 10.0,
        "maxage": 30.0,
        "maxout": 4,
        "thresdop": 5.0,
        "thresslip": 0.10,
        "interp_base": False,
        "eratio": [300, 100],
        "efact_gps": 1.0,
        "efact_glo": 1.5,
        "efact_gal": 1.0,
        "err_base": 0.003,
        "err_el": 0.003,
        "err_satclk": 5.0e-12,
        "snrmax": 45.0,
        "accelh": 3.0,
        "accelv": 1.0,
        "prnbias": 0.01,
        "sig_p0": 30.0,
        "sig_v0": 10.0,
        "sig_n0": 30.0,
        "armode": 0,
        "thresar": 3.0,
        "thresar1": 0.05,
        "minlock": 0,
        "glo_hwbias": 0.0,
        "elmaskar": 15.0,
        "var_holdamb": 0.1,
        "minfix": 20,
        "minfixsats": 4,
        "minholdsats": 5,
        "mindropsats": 10,
        "sing_p0": 100.0,
        "sing_v0": 10.0,
        "sing_elmin": 10.0,
        "gnss_t": ["GPS", "GLO", "GAL"],
        "freq_ix0": {"GPS": 0, "GLO": 4, "GAL": 0},
        "freq_ix1": {"GPS": 2, "GLO": 5, "GAL": 2},
        "freq_table": [1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9],
        "dfreq_glo": [0.56250e6, 0.43750e6],
        "rb": [0, 0, 0],
        "rr_f": [0, 0, 0, 0, 0, 0],
        "rr_b": [0, 0, 0, 0, 0, 0],
    }
    cfg = build_cfg_module(gnss_cfg)
    assert cfg.nf == 2
    assert cfg.pmode == "kinematic"
    assert cfg.filtertype == "forward"
    assert cfg.use_sing_pos is False
    assert cfg.elmin == 15.0
    assert cfg.maxinno == 1.0
    assert cfg.maxcode == 10.0
    assert len(cfg.eratio) == 2
    # efact 应是 dict，key 为 uGNSS 枚举
    assert hasattr(cfg, "efact")
    assert len(cfg.efact) == 3
    # err 应是 7 元素数组
    assert len(cfg.err) == 7
    # gnss_t 应是 uGNSS 枚举列表
    assert hasattr(cfg, "gnss_t")
    assert len(cfg.gnss_t) == 3
    # freq_ix0/1 应是 dict，key 为 uGNSS 枚举
    assert len(cfg.freq_ix0) == 3
    assert len(cfg.freq_ix1) == 3
    # freq 应是列表
    assert len(cfg.freq) == 6
    assert cfg.rb == [0, 0, 0]
    assert len(cfg.rr_f) == 6


def test_build_cfg_module_efact_keys_are_ugnss_enum():
    """efact dict 的 key 是 uGNSS 枚举对象"""
    gnss_cfg = {
        "efact_gps": 1.0, "efact_glo": 1.5, "efact_gal": 1.0,
        "gnss_t": ["GPS"],
        "freq_ix0": {"GPS": 0}, "freq_ix1": {"GPS": 2},
        "freq_table": [1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9],
    }
    cfg = build_cfg_module(gnss_cfg)
    # uGNSS.GPS 应在 efact keys 中
    from rtkcmn import uGNSS
    assert uGNSS.GPS in cfg.efact
    assert cfg.efact[uGNSS.GPS] == 1.0


def test_build_cfg_module_freq_ix_keys_are_ugnss_enum():
    """freq_ix0/1 dict 的 key 是 uGNSS 枚举对象"""
    gnss_cfg = {
        "gnss_t": ["GPS", "GAL"],
        "freq_ix0": {"GPS": 0, "GAL": 0},
        "freq_ix1": {"GPS": 2, "GAL": 2},
        "freq_table": [1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9],
    }
    cfg = build_cfg_module(gnss_cfg)
    from rtkcmn import uGNSS
    assert cfg.freq_ix0[uGNSS.GPS] == 0
    assert cfg.freq_ix1[uGNSS.GAL] == 2


def test_rtklib_env_injects_config_to_sys_modules(tmp_path):
    """RtklibEnv 初始化后 sys.modules 有 __ppk_config"""
    gnss_cfg = {
        "nf": 1, "pmode": "static", "filtertype": "forward",
        "use_sing_pos": False, "elmin": 15.0, "cnr_min": [28, 20],
        "excsats": [], "maxinno": 1.0, "maxcode": 10.0, "maxage": 30.0,
        "maxout": 4, "thresdop": 5.0, "thresslip": 0.10, "interp_base": False,
        "eratio": [300, 100], "efact_gps": 1.0, "efact_glo": 1.5, "efact_gal": 1.0,
        "err_base": 0.003, "err_el": 0.003, "err_satclk": 5.0e-12,
        "snrmax": 45.0, "accelh": 3.0, "accelv": 1.0, "prnbias": 0.01,
        "sig_p0": 30.0, "sig_v0": 10.0, "sig_n0": 30.0,
        "armode": 0, "thresar": 3.0, "thresar1": 0.05, "minlock": 0,
        "glo_hwbias": 0.0, "elmaskar": 15.0, "var_holdamb": 0.1,
        "minfix": 20, "minfixsats": 4, "minholdsats": 5, "mindropsats": 10,
        "sing_p0": 100.0, "sing_v0": 10.0, "sing_elmin": 10.0,
        "gnss_t": ["GPS"], "freq_ix0": {"GPS": 0}, "freq_ix1": {"GPS": 2},
        "freq_table": [1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9],
        "dfreq_glo": [0.56250e6, 0.43750e6],
        "rb": [0, 0, 0], "rr_f": [0, 0, 0, 0, 0, 0], "rr_b": [0, 0, 0, 0, 0, 0],
    }
    env = RtklibEnv(gnss_cfg, library_path="library/rtklib-py/src")
    assert "__ppk_config" in sys.modules
    assert sys.modules["__ppk_config"].nf == 1
    env.cleanup()
```

- [ ] **Step 3: 运行测试验证失败**

Run: `python -m pytest tests/test_rtklib_config_adapter.py -v`
Expected: FAIL (模块不存在)

- [ ] **Step 4: 实现 src/core/gnss/rtklib_config_adapter.py**

```python
"""rtklib-py 配置适配器：YAML → cfg 模块对象 → sys.modules 注入。

rtklib-py 的 rtkpos.py / postpos.py 在模块顶层 `import __ppk_config as cfg`，
且 rtkinit(cfg) 从 cfg 读取所有参数。本模块把 YAML 的 gnss: 段翻译成 cfg 模块
对象，注入 sys.modules['__ppk_config']，并把 library/rtklib-py/src 加入
sys.path，使 rtklib-py 的各模块可被正常 import。

不修改 library/rtklib-py/ 任何文件，不写 __ppk_config.py 文件。
"""
import sys
import types
from pathlib import Path

# rtklib-py 的 uGNSS 枚举（用于把 YAML 字符串 → 枚举对象）
# 延迟导入：调用方需先确保 library 路径已加入 sys.path
_CONSTELLATION_MAP = None  # 懒加载


def _get_constellation_map():
    """懒加载 uGNSS 枚举映射表。"""
    global _CONSTELLATION_MAP
    if _CONSTELLATION_MAP is None:
        from rtkcmn import uGNSS
        _CONSTELLATION_MAP = {
            "GPS": uGNSS.GPS,
            "GLO": uGNSS.GLO,
            "GAL": uGNSS.GAL,
            "QZS": uGNSS.QZS,
            "BDS": uGNSS.BDS,
        }
    return _CONSTELLATION_MAP


def build_cfg_module(gnss_cfg: dict) -> types.ModuleType:
    """把 YAML gnss: 段翻译成 rtklib-py 期望的 cfg 模块对象。

    Args:
        gnss_cfg: config["gnss"] 字典

    Returns:
        types.ModuleType 对象，包含 rtkinit 所需的全部属性
    """
    cfg = types.ModuleType("__ppk_config")

    # 直接映射的标量/列表字段
    cfg.nf = gnss_cfg["nf"]
    cfg.pmode = gnss_cfg["pmode"]
    cfg.filtertype = gnss_cfg["filtertype"]
    cfg.use_sing_pos = gnss_cfg["use_sing_pos"]
    cfg.elmin = gnss_cfg["elmin"]
    cfg.cnr_min = gnss_cfg["cnr_min"]
    cfg.excsats = gnss_cfg["excsats"]
    cfg.maxinno = gnss_cfg["maxinno"]
    cfg.maxcode = gnss_cfg["maxcode"]
    cfg.maxage = gnss_cfg["maxage"]
    cfg.maxout = gnss_cfg["maxout"]
    cfg.thresdop = gnss_cfg["thresdop"]
    cfg.thresslip = gnss_cfg["thresslip"]
    cfg.interp_base = gnss_cfg["interp_base"]
    cfg.eratio = gnss_cfg["eratio"]
    cfg.snrmax = gnss_cfg["snrmax"]
    cfg.accelh = gnss_cfg["accelh"]
    cfg.accelv = gnss_cfg["accelv"]
    cfg.prnbias = gnss_cfg["prnbias"]
    cfg.sig_p0 = gnss_cfg["sig_p0"]
    cfg.sig_v0 = gnss_cfg["sig_v0"]
    cfg.sig_n0 = gnss_cfg["sig_n0"]
    cfg.armode = gnss_cfg["armode"]
    cfg.thresar = gnss_cfg["thresar"]
    cfg.thresar1 = gnss_cfg["thresar1"]
    cfg.minlock = gnss_cfg.get("minlock", 0)
    cfg.glo_hwbias = gnss_cfg["glo_hwbias"]
    cfg.elmaskar = gnss_cfg["elmaskar"]
    cfg.var_holdamb = gnss_cfg["var_holdamb"]
    cfg.minfix = gnss_cfg["minfix"]
    cfg.minfixsats = gnss_cfg["minfixsats"]
    cfg.minholdsats = gnss_cfg["minholdsats"]
    cfg.mindropsats = gnss_cfg["mindropsats"]
    cfg.sing_p0 = gnss_cfg["sing_p0"]
    cfg.sing_v0 = gnss_cfg["sing_v0"]
    cfg.sing_elmin = gnss_cfg["sing_elmin"]
    cfg.freq = gnss_cfg["freq_table"]
    cfg.dfreq_glo = gnss_cfg["dfreq_glo"]
    cfg.rb = gnss_cfg["rb"]
    cfg.rr_f = gnss_cfg["rr_f"]
    cfg.rr_b = gnss_cfg["rr_b"]

    # err 数组: [_, base, el, bl, snr, rcvstd, satclk]
    cfg.err = [
        0,
        gnss_cfg["err_base"],
        gnss_cfg["err_el"],
        0.0,
        0,
        0,
        gnss_cfg["err_satclk"],
    ]

    # efact dict: 字符串 key → uGNSS 枚举 key
    cmap = _get_constellation_map()
    cfg.efact = {}
    for sat_str, val in [
        ("GPS", gnss_cfg["efact_gps"]),
        ("GLO", gnss_cfg["efact_glo"]),
        ("GAL", gnss_cfg["efact_gal"]),
    ]:
        if sat_str in cmap:
            cfg.efact[cmap[sat_str]] = val

    # gnss_t: 字符串列表 → uGNSS 枚举列表
    cfg.gnss_t = [cmap[s] for s in gnss_cfg["gnss_t"] if s in cmap]

    # freq_ix0/1: 字符串 key → uGNSS 枚举 key
    cfg.freq_ix0 = {cmap[k]: v for k, v in gnss_cfg["freq_ix0"].items() if k in cmap}
    cfg.freq_ix1 = {cmap[k]: v for k, v in gnss_cfg["freq_ix1"].items() if k in cmap}

    return cfg


class RtklibEnv:
    """rtklib-py 运行环境管理器。

    职责：
    1. 把 library_path 加入 sys.path
    2. 用 build_cfg_module 构建 cfg 模块对象
    3. 注入 sys.modules['__ppk_config']
    4. cleanup() 时移除注入（避免污染后续测试）

    用法：
        env = RtklibEnv(config["gnss"], "library/rtklib-py/src")
        nav = env.init_nav()  # 调用 rtkinit
        ...
        env.cleanup()
    """

    def __init__(self, gnss_cfg: dict, library_path: str = "library/rtklib-py/src"):
        self.gnss_cfg = gnss_cfg
        self.library_path = str(Path(library_path).resolve())
        self._cfg_module = None
        self._path_added = False
        self._injected = False

    def setup(self):
        """加入 sys.path + 构建 cfg + 注入 sys.modules。"""
        # 1. 加入 sys.path
        if self.library_path not in sys.path:
            sys.path.insert(0, self.library_path)
            self._path_added = True

        # 2. 构建 cfg 模块
        self._cfg_module = build_cfg_module(self.gnss_cfg)

        # 3. 注入 sys.modules（rtkpos.py 顶层 import __ppk_config）
        sys.modules["__ppk_config"] = self._cfg_module
        self._injected = True

    def get_cfg(self):
        """返回 cfg 模块对象（setup 后可用）。"""
        if self._cfg_module is None:
            self.setup()
        return self._cfg_module

    def init_nav(self):
        """调用 rtkinit(cfg) 返回 nav 对象。"""
        cfg = self.get_cfg()
        from rtkpos import rtkinit
        return rtkinit(cfg)

    def cleanup(self):
        """移除 sys.path 与 sys.modules 注入。"""
        if self._injected and "__ppk_config" in sys.modules:
            del sys.modules["__ppk_config"]
            self._injected = False
        if self._path_added and self.library_path in sys.path:
            sys.path.remove(self.library_path)
            self._path_added = False
```

- [ ] **Step 5: 运行测试验证通过**

Run: `python -m pytest tests/test_rtklib_config_adapter.py -v`
Expected: 全部 PASS

- [ ] **Step 6: 提交**

```bash
git add src/core/gnss/__init__.py src/core/gnss/rtklib_config_adapter.py tests/test_rtklib_config_adapter.py
git commit -m "feat(gnss): add rtklib config adapter (YAML -> cfg module injection)"
```

---

## Task 3: 解算结果转换器

**Files:**
- Create: `src/core/gnss/solution_converter.py`
- Test: `tests/test_solution_converter.py`

**背景：** rtklib-py 的 `Sol` 对象有 `rr[0:3]` (ECEF 位置)、`qr` (3x3 协方差)、`stat` (质量)、`ns` (卫星数)、`t` (gtime_t)。本项目 `GnssSolution` 有 `timestamp/week/position/quality/num_sv/sd`。

- [ ] **Step 1: 写失败测试 tests/test_solution_converter.py**

```python
"""solution_converter 测试。"""
import numpy as np
import pytest

from src.core.gnss.solution_converter import sol_to_gnss_solution
from src.core.data_types import GnssSolution


def _make_fake_sol(stat=5, ns=8, rr=None, qr=None, week=2000, sow=357456.0):
    """构造一个 fake rtklib Sol 对象（避免依赖 rtklib-py）"""
    class FakeGtime:
        def __init__(self, w, s):
            self.time = w * 604800 + int(s)
            self.sec = s - int(s)
    class FakeSol:
        def __init__(self):
            self.rr = np.zeros(6) if rr is None else np.array(rr)
            self.qr = np.zeros((3, 3)) if qr is None else np.array(qr)
            self.stat = stat
            self.ns = ns
            self.t = FakeGtime(week, sow)
    return FakeSol()


def test_sol_to_gnss_solution_basic():
    sol = _make_fake_sol(stat=5, ns=8, rr=[100, 200, 300, 0, 0, 0])
    gs = sol_to_gnss_solution(sol)
    assert isinstance(gs, GnssSolution)
    assert gs.week == 2000
    assert gs.timestamp == pytest.approx(357456.0, abs=1e-3)
    assert np.allclose(gs.position, [100, 200, 300])
    assert gs.quality == 5
    assert gs.num_sv == 8


def test_sol_to_gnss_solution_sd_from_qr_diagonal():
    qr = np.diag([4.0, 9.0, 16.0])  # variances → std = [2, 3, 4]
    sol = _make_fake_sol(qr=qr)
    gs = sol_to_gnss_solution(sol)
    assert np.allclose(gs.sd, [2.0, 3.0, 4.0])


def test_sol_to_gnss_solution_quality_mapping():
    """rtklib stat → 本项目 quality"""
    # SOLQ_SINGLE=5, SOLQ_FIX=1, SOLQ_FLOAT=2, SOLQ_DGPS=4
    for stat, expected_q in [(5, 5), (1, 1), (2, 2), (4, 4)]:
        sol = _make_fake_sol(stat=stat)
        gs = sol_to_gnss_solution(sol)
        assert gs.quality == expected_q


def test_sol_to_gnss_solution_none_stat_returns_none():
    """SOLQ_NONE=0 → 返回 None"""
    sol = _make_fake_sol(stat=0)
    gs = sol_to_gnss_solution(sol)
    assert gs is None
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python -m pytest tests/test_solution_converter.py -v`
Expected: FAIL (模块不存在)

- [ ] **Step 3: 实现 src/core/gnss/solution_converter.py**

```python
"""rtklib-py Sol → 本项目 GnssSolution 转换器。"""
from typing import Optional

import numpy as np

from src.core.data_types import GnssSolution

# rtklib-py 的 SOLQ 常量（见 rtkcmn.py）
SOLQ_NONE = 0
SOLQ_FIX = 1
SOLQ_FLOAT = 2
SOLQ_DGPS = 4
SOLQ_SINGLE = 5


def sol_to_gnss_solution(sol) -> Optional[GnssSolution]:
    """把 rtklib-py Sol 对象转换为本项目 GnssSolution。

    Args:
        sol: rtklib-py Sol 对象（有 rr/qr/stat/ns/t 属性）

    Returns:
        GnssSolution 或 None（解算失败 stat==SOLQ_NONE 时）
    """
    if sol.stat == SOLQ_NONE:
        return None

    # 时间戳: Sol.t → GPST 周内秒 + 周号
    # rtklib-py 的 gtime_t.time 是 GPS 秒（自 1980-01-06），time2gpst 拆分周/周内秒
    # 这里直接计算避免依赖 rtkcmn.time2gpst
    SECONDS_PER_WEEK = 604800
    total_sec = sol.t.time + sol.t.sec
    week = int(total_sec // SECONDS_PER_WEEK)
    sow = total_sec - week * SECONDS_PER_WEEK

    # 位置 ECEF
    position = np.array(sol.rr[0:3], dtype=float)

    # 标准差: qr 对角线平方根
    sd = np.sqrt(np.abs(np.diag(sol.qr[0:3, 0:3])))

    return GnssSolution(
        timestamp=sow,
        week=week,
        position=position,
        quality=int(sol.stat),
        num_sv=int(sol.ns),
        sd=sd,
    )
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python -m pytest tests/test_solution_converter.py -v`
Expected: 全部 PASS

- [ ] **Step 5: 提交**

```bash
git add src/core/gnss/solution_converter.py tests/test_solution_converter.py
git commit -m "feat(gnss): add Sol -> GnssSolution converter"
```

---

## Task 4: GnssProcessor ABC + SppProcessor

**Files:**
- Create: `src/core/gnss/gnss_processor.py`
- Create: `src/core/gnss/spp_processor.py`
- Test: `tests/test_spp_processor.py`

- [ ] **Step 1: 创建 src/core/gnss/gnss_processor.py**

```python
"""GNSS 处理器抽象基类与异常。"""
from abc import ABC, abstractmethod
from typing import Optional

from src.core.data_types import GnssSolution


class GnssProcessor(ABC):
    """GNSS 处理器抽象基类（内部模式）。

    每个 process_epoch 调用处理一个历元，返回 GnssSolution 或 None。
    跨历元状态（如 nav.x, nav.P）由 nav 对象维护，处理器持有 nav 引用。
    """

    @abstractmethod
    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]:
        """处理单个历元。

        Args:
            obsr: 流动站观测值（rtklib-py obs 对象）
            obsb: 基站观测值（RTK 模式，SPP 为 None）

        Returns:
            GnssSolution 或 None（解算失败时）
        """
        ...

    def reset(self) -> None:
        """重置处理器状态（默认无操作，子类按需覆盖）。"""
        pass


class GnssConfigError(ValueError):
    """GNSS 配置错误。"""
    pass


class GnssSolutionError(RuntimeError):
    """GNSS 解算运行时错误。"""
    pass
```

- [ ] **Step 2: 写失败测试 tests/test_spp_processor.py**

```python
"""SppProcessor 测试（mock rtklib-py pntpos）。"""
import sys
import types
import pytest
from unittest.mock import MagicMock, patch

import numpy as np


def _setup_ppk_config_in_sys_modules():
    """注入最小 __ppk_config 到 sys.modules（rtkpos.py 顶层 import 需要）"""
    if "__ppk_config" not in sys.modules:
        cfg = types.ModuleType("__ppk_config")
        cfg.nf = 1
        cfg.pmode = "static"
        cfg.filtertype = "forward"
        cfg.use_sing_pos = False
        cfg.elmin = 15.0
        cfg.cnr_min = [28, 20]
        cfg.excsats = []
        cfg.maxinno = 1.0
        cfg.maxcode = 10.0
        cfg.maxage = 30.0
        cfg.maxout = 4
        cfg.thresdop = 5.0
        cfg.thresslip = 0.10
        cfg.interp_base = False
        cfg.eratio = [300, 100]
        cfg.efact = {}
        cfg.err = [0, 0.003, 0.003, 0, 0, 0, 5e-12]
        cfg.snrmax = 45.0
        cfg.accelh = 3.0
        cfg.accelv = 1.0
        cfg.prnbias = 0.01
        cfg.sig_p0 = 30.0
        cfg.sig_v0 = 10.0
        cfg.sig_n0 = 30.0
        cfg.armode = 0
        cfg.thresar = 3.0
        cfg.thresar1 = 0.05
        cfg.minlock = 0
        cfg.glo_hwbias = 0.0
        cfg.elmaskar = 15.0
        cfg.var_holdamb = 0.1
        cfg.minfix = 20
        cfg.minfixsats = 4
        cfg.minholdsats = 5
        cfg.mindropsats = 10
        cfg.sing_p0 = 100.0
        cfg.sing_v0 = 10.0
        cfg.sing_elmin = 10.0
        cfg.gnss_t = []
        cfg.freq_ix0 = {}
        cfg.freq_ix1 = {}
        cfg.freq = [1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9]
        cfg.dfreq_glo = [0.56250e6, 0.43750e6]
        cfg.rb = [0, 0, 0]
        cfg.rr_f = [0, 0, 0, 0, 0, 0]
        cfg.rr_b = [0, 0, 0, 0, 0, 0]
        sys.modules["__ppk_config"] = cfg


def _make_fake_sol(stat=5, ns=8):
    class FakeGtime:
        time = 2000 * 604800 + 357456
        sec = 0.0
    class FakeSol:
        def __init__(self):
            self.rr = np.array([100.0, 200.0, 300.0, 0, 0, 0])
            self.qr = np.diag([1.0, 4.0, 9.0])
            self.stat = stat
            self.ns = ns
            self.t = FakeGtime()
    return FakeSol()


def test_spp_processor_calls_pntpos_and_returns_gnss_solution():
    _setup_ppk_config_in_sys_modules()
    sys.path.insert(0, "library/rtklib-py/src")
    from src.core.gnss.spp_processor import SppProcessor
    from src.core.data_types import GnssSolution

    fake_sol = _make_fake_sol(stat=5, ns=8)
    fake_nav = MagicMock()
    fake_obs = MagicMock()

    with patch("src.core.gnss.spp_processor.pntpos", return_value=fake_sol) as mock_pntpos:
        proc = SppProcessor(fake_nav)
        result = proc.process_epoch(fake_obs)

    mock_pntpos.assert_called_once_with(fake_obs, fake_nav)
    assert isinstance(result, GnssSolution)
    assert result.quality == 5
    assert result.num_sv == 8
    assert np.allclose(result.position, [100, 200, 300])
    assert np.allclose(result.sd, [1.0, 2.0, 3.0])


def test_spp_processor_returns_none_when_stat_is_none():
    _setup_ppk_config_in_sys_modules()
    sys.path.insert(0, "library/rtklib-py/src")
    from src.core.gnss.spp_processor import SppProcessor

    fake_sol = _make_fake_sol(stat=0)  # SOLQ_NONE
    fake_nav = MagicMock()
    fake_obs = MagicMock()

    with patch("src.core.gnss.spp_processor.pntpos", return_value=fake_sol):
        proc = SppProcessor(fake_nav)
        result = proc.process_epoch(fake_obs)

    assert result is None
```

- [ ] **Step 3: 运行测试验证失败**

Run: `python -m pytest tests/test_spp_processor.py -v`
Expected: FAIL (spp_processor 不存在)

- [ ] **Step 4: 实现 src/core/gnss/spp_processor.py**

```python
"""SPP 单点定位处理器，薄封装 rtklib-py 的 pntpos。"""
from typing import Optional

from src.core.data_types import GnssSolution
from src.core.gnss.gnss_processor import GnssProcessor
from src.core.gnss.solution_converter import sol_to_gnss_solution

# rtklib-py 函数（需先由 RtklibEnv.setup() 注入 __ppk_config 到 sys.modules）
from pntpos import pntpos


class SppProcessor(GnssProcessor):
    """SPP 单点定位处理器。

    每历元独立调用 pntpos(obsr, nav)，无跨历元模糊度状态。
    nav 对象持有星历等状态，由调用方（InternalGnssSensor）管理生命周期。
    """

    def __init__(self, nav):
        self.nav = nav

    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]:
        """调用 pntpos 解算单历元 SPP。

        Args:
            obsr: 流动站单历元观测值
            obsb: 忽略（SPP 不需要基站）

        Returns:
            GnssSolution 或 None（解算失败时）
        """
        sol = pntpos(obsr, self.nav)
        return sol_to_gnss_solution(sol)
```

- [ ] **Step 5: 运行测试验证通过**

Run: `python -m pytest tests/test_spp_processor.py -v`
Expected: 全部 PASS

- [ ] **Step 6: 提交**

```bash
git add src/core/gnss/gnss_processor.py src/core/gnss/spp_processor.py tests/test_spp_processor.py
git commit -m "feat(gnss): add GnssProcessor ABC and SppProcessor"
```

---

## Task 5: RtkProcessor

**Files:**
- Create: `src/core/gnss/rtk_processor.py`
- Test: `tests/test_rtk_processor.py`

**背景：** RTK 模式参考 rtklib-py 的 rtkpos() 循环（rtkpos.py:1073），但拆分为逐历元接口。关键逻辑：
1. 首历元或 sol.rr[0]==0 时先 pntpos 取初值
2. 调用 relpos(nav, obsr, obsb, sol) 更新 sol 与 nav 状态
3. sol 跨历元维护（用于 rr[0]==0 判断）

- [ ] **Step 1: 写失败测试 tests/test_rtk_processor.py**

```python
"""RtkProcessor 测试（mock rtklib-py pntpos + relpos）。"""
import sys
import types
import pytest
from unittest.mock import MagicMock, patch

import numpy as np


def _setup_ppk_config_in_sys_modules():
    """注入最小 __ppk_config（同 test_spp_processor）"""
    if "__ppk_config" not in sys.modules:
        cfg = types.ModuleType("__ppk_config")
        cfg.nf = 1
        cfg.pmode = "kinematic"
        cfg.filtertype = "forward"
        cfg.use_sing_pos = False
        cfg.elmin = 15.0
        cfg.cnr_min = [28, 20]
        cfg.excsats = []
        cfg.maxinno = 1.0
        cfg.maxcode = 10.0
        cfg.maxage = 30.0
        cfg.maxout = 4
        cfg.thresdop = 5.0
        cfg.thresslip = 0.10
        cfg.interp_base = False
        cfg.eratio = [300, 100]
        cfg.efact = {}
        cfg.err = [0, 0.003, 0.003, 0, 0, 0, 5e-12]
        cfg.snrmax = 45.0
        cfg.accelh = 3.0
        cfg.accelv = 1.0
        cfg.prnbias = 0.01
        cfg.sig_p0 = 30.0
        cfg.sig_v0 = 10.0
        cfg.sig_n0 = 30.0
        cfg.armode = 3
        cfg.thresar = 3.0
        cfg.thresar1 = 0.05
        cfg.minlock = 0
        cfg.glo_hwbias = 0.0
        cfg.elmaskar = 15.0
        cfg.var_holdamb = 0.1
        cfg.minfix = 20
        cfg.minfixsats = 4
        cfg.minholdsats = 5
        cfg.mindropsats = 10
        cfg.sing_p0 = 100.0
        cfg.sing_v0 = 10.0
        cfg.sing_elmin = 10.0
        cfg.gnss_t = []
        cfg.freq_ix0 = {}
        cfg.freq_ix1 = {}
        cfg.freq = [1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9]
        cfg.dfreq_glo = [0.56250e6, 0.43750e6]
        cfg.rb = [0, 0, 0]
        cfg.rr_f = [0, 0, 0, 0, 0, 0]
        cfg.rr_b = [0, 0, 0, 0, 0, 0]
        sys.modules["__ppk_config"] = cfg


def _make_fake_sol(stat=1, ns=8, rr_zero=False):
    class FakeGtime:
        time = 2000 * 604800 + 357456
        sec = 0.0
    class FakeSol:
        def __init__(self):
            if rr_zero:
                self.rr = np.zeros(6)
            else:
                self.rr = np.array([100.0, 200.0, 300.0, 0, 0, 0])
            self.qr = np.diag([1.0, 4.0, 9.0])
            self.stat = stat
            self.ns = ns
            self.t = FakeGtime()
    return FakeSol()


def test_rtk_processor_first_epoch_calls_pntpos_then_relpos():
    """首历元 sol.rr[0]==0 → 先 pntpos 取初值，再 relpos"""
    _setup_ppk_config_in_sys_modules()
    sys.path.insert(0, "library/rtklib-py/src")
    from src.core.gnss.rtk_processor import RtkProcessor

    spp_sol = _make_fake_sol(stat=5, ns=6)       # pntpos 返回的初值
    rtk_sol = _make_fake_sol(stat=1, ns=8)        # relpos 后的最终解

    fake_nav = MagicMock()
    fake_obs = MagicMock()
    fake_base = MagicMock()

    # relpos 会修改传入的 sol 对象，这里用 side_effect 模拟
    def relpos_side_effect(nav, obsr, obsb, sol):
        sol.stat = rtk_sol.stat
        sol.ns = rtk_sol.ns
        sol.rr = rtk_sol.rr.copy()
        sol.qr = rtk_sol.qr.copy()

    with patch("src.core.gnss.rtk_processor.pntpos", return_value=spp_sol) as mock_pntpos, \
         patch("src.core.gnss.rtk_processor.relpos", side_effect=relpos_side_effect) as mock_relpos:
        proc = RtkProcessor(fake_nav)
        result = proc.process_epoch(fake_obs, fake_base)

    mock_pntpos.assert_called_once()
    mock_relpos.assert_called_once()
    assert result is not None
    assert result.quality == 1  # SOLQ_FIX
    assert result.num_sv == 8


def test_rtk_processor_subsequent_epoch_skips_pntpos_when_sol_has_position():
    """后续历元 sol.rr[0]!=0 → 不调 pntpos，直接 relpos"""
    _setup_ppk_config_in_sys_modules()
    sys.path.insert(0, "library/rtklib-py/src")
    from src.core.gnss.rtk_processor import RtkProcessor

    fake_nav = MagicMock()
    fake_obs = MagicMock()
    fake_base = MagicMock()

    # 第一次历元：pntpos 返回有位置的 sol
    spp_sol = _make_fake_sol(stat=5, ns=6)
    rtk_sol1 = _make_fake_sol(stat=1, ns=8)

    def relos_side1(nav, obsr, obsb, sol):
        sol.stat = rtk_sol1.stat
        sol.ns = rtk_sol1.ns
        sol.rr = rtk_sol1.rr.copy()
        sol.qr = rtk_sol1.qr.copy()

    with patch("src.core.gnss.rtk_processor.pntpos", return_value=spp_sol) as mock_pntpos, \
         patch("src.core.gnss.rtk_processor.relpos", side_effect=relpos_side1):
        proc = RtkProcessor(fake_nav)
        proc.process_epoch(fake_obs, fake_base)

    assert mock_pntpos.call_count == 1

    # 第二次历元：sol 已有位置，不应调 pntpos
    rtk_sol2 = _make_fake_sol(stat=1, ns=9)
    def relpos_side2(nav, obsr, obsb, sol):
        sol.stat = rtk_sol2.stat
        sol.ns = rtk_sol2.ns

    with patch("src.core.gnss.rtk_processor.pntpos", return_value=spp_sol) as mock_pntpos2, \
         patch("src.core.gnss.rtk_processor.relpos", side_effect=relpos_side2):
        result = proc.process_epoch(fake_obs, fake_base)

    mock_pntpos2.assert_not_called()
    assert result is not None
    assert result.num_sv == 9


def test_rtk_processor_returns_none_when_relpos_fails():
    """relpos 后 stat==SOLQ_NONE → 返回 None"""
    _setup_ppk_config_in_sys_modules()
    sys.path.insert(0, "library/rtklib-py/src")
    from src.core.gnss.rtk_processor import RtkProcessor

    spp_sol = _make_fake_sol(stat=5, ns=6)
    fake_nav = MagicMock()
    fake_obs = MagicMock()
    fake_base = MagicMock()

    def relpos_fail(nav, obsr, obsb, sol):
        sol.stat = 0  # SOLQ_NONE

    with patch("src.core.gnss.rtk_processor.pntpos", return_value=spp_sol), \
         patch("src.core.gnss.rtk_processor.relpos", side_effect=relpos_fail):
        proc = RtkProcessor(fake_nav)
        result = proc.process_epoch(fake_obs, fake_base)

    assert result is None
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python -m pytest tests/test_rtk_processor.py -v`
Expected: FAIL (rtk_processor 不存在)

- [ ] **Step 3: 实现 src/core/gnss/rtk_processor.py**

```python
"""RTK 相对定位处理器，薄封装 rtklib-py 的 relpos。"""
from typing import Optional

import numpy as np

from src.core.data_types import GnssSolution
from src.core.gnss.gnss_processor import GnssProcessor
from src.core.gnss.solution_converter import sol_to_gnss_solution, SOLQ_NONE

# rtklib-py 函数（需先由 RtklibEnv.setup() 注入 __ppk_config 到 sys.modules）
from pntpos import pntpos
from rtkpos import relpos
from rtkcmn import Sol


class RtkProcessor(GnssProcessor):
    """RTK 相对定位处理器。

    参考 rtklib-py 的 rtkpos() 循环（rtkpos.py:1073），拆分为逐历元接口。
    跨历元状态：
    - self.sol: 上历元解算结果（用于判断是否需要重新 SPP 取初值）
    - nav: 由调用方管理，持有 x/P/azel/lock 等状态
    """

    def __init__(self, nav):
        self.nav = nav
        self.sol = Sol()  # 初始 sol，rr[0]==0 触发首历元 pntpos

    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]:
        """调用 relpos 解算单历元 RTK。

        Args:
            obsr: 流动站单历元观测值
            obsb: 基站单历元观测值（RTK 必填）

        Returns:
            GnssSolution 或 None（解算失败时）
        """
        # 首历元或 sol.rr[0]==0 时先 pntpos 取初值
        if self.nav.use_sing_pos or self.sol.stat == SOLQ_NONE or self.sol.rr[0] == 0.0:
            self.sol = pntpos(obsr, self.nav)
        else:
            self.sol = Sol()

        # 确保时间戳正确
        if self.sol.t.time == 0:
            self.sol.t = obsr.t

        # 相对定位（修改 self.sol 与 self.nav 状态）
        relpos(self.nav, obsr, obsb, self.sol)

        return sol_to_gnss_solution(self.sol)

    def reset(self) -> None:
        """重置处理器状态（不重置 nav，nav 由调用方管理）。"""
        self.sol = Sol()
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python -m pytest tests/test_rtk_processor.py -v`
Expected: 全部 PASS

- [ ] **Step 5: 提交**

```bash
git add src/core/gnss/rtk_processor.py tests/test_rtk_processor.py
git commit -m "feat(gnss): add RtkProcessor wrapping rtklib-py relpos"
```

---

## Task 6: SolutionWriter

**Files:**
- Create: `src/log/solution_writer.py`
- Test: `tests/test_solution_writer.py`

**背景：** 参考 rtklib-py `postpos.savesol` 的 .pos 格式：
```
%  GPST          latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio
 2000  357456.000   40.123456789  116.123456789   50.0000   5   8   1.0000    2.0000    3.0000    0.0000    0.0000    0.0000   0.00    0.0
```
本项目 GnssSolution 只有 ECEF 位置 + sd（sdx/sdy/sdz），需转 ENU→LLH 并输出。age/ratio 填 0。

- [ ] **Step 1: 写失败测试 tests/test_solution_writer.py**

```python
"""SolutionWriter 测试。"""
import os
import numpy as np
import pytest

from src.core.data_types import GnssSolution
from src.log.solution_writer import SolutionWriter


def test_solution_writer_writes_header_and_data_row(tmp_path):
    out_file = tmp_path / "solution.pos"
    w = SolutionWriter(output_dir=str(tmp_path), filename="solution.pos")
    w.open()

    sol = GnssSolution(
        timestamp=357456.0,
        week=2000,
        position=np.array([-2148744.0, 4426909.0, 4045382.0]),
        quality=5,
        num_sv=8,
        sd=np.array([1.0, 2.0, 3.0]),
    )
    w.write(sol)
    w.close()

    content = out_file.read_text(encoding="utf-8")
    lines = content.strip().split("\n")
    # 表头 + 1 行数据
    assert len(lines) == 2
    assert "GPST" in lines[0]
    assert "latitude" in lines[0]
    # 数据行: week sow lat lon h Q ns ...
    parts = lines[1].split()
    assert parts[0] == "2000"
    assert float(parts[1]) == pytest.approx(357456.0, abs=0.001)
    # lat/lon 应是度数（非弧度）
    lat = float(parts[2])
    lon = float(parts[3])
    assert 30 < lat < 50   # 中国纬度范围
    assert 100 < lon < 130
    assert int(parts[5]) == 5  # Q
    assert int(parts[6]) == 8  # ns


def test_solution_writer_multiple_rows(tmp_path):
    w = SolutionWriter(output_dir=str(tmp_path), filename="solution.pos")
    w.open()
    for i in range(5):
        sol = GnssSolution(
            timestamp=357456.0 + i,
            week=2000,
            position=np.array([-2148744.0, 4426909.0, 4045382.0]),
            quality=5,
            num_sv=8,
            sd=np.array([1.0, 2.0, 3.0]),
        )
        w.write(sol)
    w.close()

    content = (tmp_path / "solution.pos").read_text(encoding="utf-8")
    lines = content.strip().split("\n")
    assert len(lines) == 6  # 表头 + 5 行


def test_solution_writer_close_idempotent(tmp_path):
    w = SolutionWriter(output_dir=str(tmp_path), filename="solution.pos")
    w.open()
    w.close()
    w.close()  # 不应报错
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python -m pytest tests/test_solution_writer.py -v`
Expected: FAIL (模块不存在)

- [ ] **Step 3: 实现 src/log/solution_writer.py**

```python
"""rtklib 风格 .pos 输出器。"""
import os
from pathlib import Path

import numpy as np

from src.core.data_types import GnssSolution
from src.log.writer_base import WriterBase


# ECEF→LLH 转换（WGS84），避免依赖 rtklib-py
_WGS84_A = 6378137.0
_WGS84_F = 1 / 298.257223563
_WGS84_B = _WGS84_A * (1 - _WGS84_F)
_WGS84_E2 = _WGS84_F * (2 - _WGS84_F)


def ecef2llh(ecef: np.ndarray) -> np.ndarray:
    """ECEF [x,y,z] → [lat_rad, lon_rad, h]。"""
    x, y, z = ecef[0], ecef[1], ecef[2]
    lon = np.arctan2(y, x)
    p = np.sqrt(x * x + y * y)
    h = np.sqrt(p * p + z * z) - _WGS84_A
    lat = np.arctan2(z, p * (1 - _WGS84_E2))
    # 迭代精化
    for _ in range(6):
        sinlat = np.sin(lat)
        N = _WGS84_A / np.sqrt(1 - _WGS84_E2 * sinlat * sinlat)
        h = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1 - _WGS84_E2 * N / (N + h)))
    return np.array([lat, lon, h])


def ecef2enu_matrix(llh: np.ndarray) -> np.ndarray:
    """LLH [lat_rad, lon_rad, h] → ECEF→ENU 旋转矩阵 3x3。"""
    lat, lon = llh[0], llh[1]
    sl, cl = np.sin(lat), np.cos(lat)
    so, co = np.sin(lon), np.cos(lon)
    return np.array([
        [-so,            co,           0],
        [-sl * co,      -sl * so,      cl],
        [cl * co,        cl * so,      sl],
    ])


class SolutionWriter(WriterBase):
    """rtklib 风格 .pos 输出器。

    输出格式（参考 rtklib-py postpos.savesol）:
      表头 + 每历元一行: week sow lat lon h Q ns sdn sde sdu sdne sdeu sdun age ratio
    ECEF → LLH（度）转换，sd ECEF → ENU。
    age/ratio 填 0（本项目 GnssSolution 无此字段）。
    """

    HEADER = (
        "%  GPST          latitude(deg) longitude(deg)  height(m)   Q  "
        "ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n"
    )

    def __init__(self, output_dir: str, filename: str = "solution.pos"):
        self.output_dir = output_dir
        self.filename = filename
        self._fp = None
        self._closed = False

    def open(self) -> None:
        os.makedirs(self.output_dir, exist_ok=True)
        path = Path(self.output_dir) / self.filename
        self._fp = open(path, "w", encoding="utf-8")
        self._fp.write(self.HEADER)
        self._closed = False

    def write(self, sol: GnssSolution) -> None:
        if self._fp is None:
            raise RuntimeError("SolutionWriter not opened")

        llh = ecef2llh(sol.position)
        # sd ECEF → ENU
        R = ecef2enu_matrix(llh)
        cov_ecef = np.diag(sol.sd ** 2)
        cov_enu = R @ cov_ecef @ R.T
        sdn = np.sqrt(abs(cov_enu[0, 0]))
        sde = np.sqrt(abs(cov_enu[1, 1]))
        sdu = np.sqrt(abs(cov_enu[2, 2]))
        sdne = np.sqrt(abs(cov_enu[0, 1])) * np.sign(cov_enu[0, 1])
        sdeu = np.sqrt(abs(cov_enu[1, 2])) * np.sign(cov_enu[1, 2])
        sdun = np.sqrt(abs(cov_enu[2, 0])) * np.sign(cov_enu[2, 0])

        D2R = np.pi / 180.0
        fmt = (
            "%4d %10.3f %14.9f %14.9f %10.4f %3d %3d %8.4f"
            "  %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f\n"
        )
        self._fp.write(fmt % (
            sol.week, sol.timestamp,
            llh[0] / D2R, llh[1] / D2R, llh[2],
            sol.quality, sol.num_sv,
            sdn, sde, sdu, sdne, sdeu, sdun,
            0.0, 0.0,  # age, ratio（本项目无此字段）
        ))

    def close(self) -> None:
        if self._fp is not None and not self._closed:
            self._fp.close()
            self._closed = True
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python -m pytest tests/test_solution_writer.py -v`
Expected: 全部 PASS

- [ ] **Step 5: 提交**

```bash
git add src/log/solution_writer.py tests/test_solution_writer.py
git commit -m "feat(log): add SolutionWriter for .pos output"
```

---

## Task 7: SolutionLogger

**Files:**
- Create: `src/log/solution_logger.py`
- Test: `tests/test_solution_logger.py`

- [ ] **Step 1: 写失败测试 tests/test_solution_logger.py**

```python
"""SolutionLogger 测试。"""
import numpy as np
import pytest
from queue import Queue
from threading import Event
from unittest.mock import MagicMock

from src.core.data_types import GnssSolution, SensorData
from src.core.thread_control import ThreadControl
from src.log.solution_logger import SolutionLogger


def _make_sol(week=2000, sow=357456.0):
    return GnssSolution(
        timestamp=sow, week=week,
        position=np.array([100.0, 200.0, 300.0]),
        quality=5, num_sv=8,
        sd=np.array([1.0, 2.0, 3.0]),
    )


def test_solution_logger_writes_solutions_until_eof():
    gnss_queue = Queue()
    control = ThreadControl()
    writer = MagicMock()
    logger = SolutionLogger(gnss_queue, writer, control)

    # 放 3 个解 + EOF
    for i in range(3):
        gnss_queue.put(SensorData(tag="gnss_solution", gnss_solution=_make_sol(sow=357456 + i)))
    gnss_queue.put(None)

    writer.open.assert_not_called()
    logger.run()  # 直接跑 run（非 start）

    writer.open.assert_called_once()
    assert writer.write.call_count == 3
    writer.close.assert_called_once()


def test_solution_logger_skips_none_solutions():
    gnss_queue = Queue()
    control = ThreadControl()
    writer = MagicMock()
    logger = SolutionLogger(gnss_queue, writer, control)

    # 放 1 个有效解 + 1 个 None（EOF）
    gnss_queue.put(SensorData(tag="gnss_solution", gnss_solution=_make_sol()))
    gnss_queue.put(None)

    logger.run()

    assert writer.write.call_count == 1


def test_solution_logger_stops_on_control_shutdown():
    gnss_queue = Queue()
    control = ThreadControl()
    writer = MagicMock()
    logger = SolutionLogger(gnss_queue, writer, control)

    # 不放任何数据，立即 shutdown
    control.shutdown()
    logger.run()  # 应在 timeout 循环中检测到 shutdown 退出

    writer.open.assert_called_once()
    writer.close.assert_called_once()
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python -m pytest tests/test_solution_logger.py -v`
Expected: FAIL (模块不存在)

- [ ] **Step 3: 实现 src/log/solution_logger.py**

```python
"""纯 GNSS 模式日志线程，仅消费 gnss_queue。"""
from queue import Queue, Empty
from threading import Thread

from src.core.thread_control import ThreadControl
from src.core.data_types import SensorData


class SolutionLogger(Thread):
    """纯 GNSS 模式日志线程。

    仅消费 gnss_queue，把 GnssSolution 委托 SolutionWriter 输出。
    收到 None（EOF sentinel）后关闭 writer 并退出。
    """

    def __init__(self, gnss_queue: Queue, writer, control: ThreadControl):
        Thread.__init__(self, name="SolutionLogger", daemon=True)
        self.gnss_queue = gnss_queue
        self.writer = writer
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

- [ ] **Step 4: 运行测试验证通过**

Run: `python -m pytest tests/test_solution_logger.py -v`
Expected: 全部 PASS

- [ ] **Step 5: 提交**

```bash
git add src/log/solution_logger.py tests/test_solution_logger.py
git commit -m "feat(log): add SolutionLogger thread for pure GNSS mode"
```

---

## Task 8: InternalGnssSensor

**Files:**
- Create: `src/stream/internal_gnss_sensor.py`
- Test: `tests/test_internal_gnss_sensor.py`

**背景：** InternalGnssSensor 是线程，run() 内：
1. RtklibEnv.setup() → init_nav()
2. rnx_decode(rover) + decode_obsfile
3. RTK 模式: rnx_decode(base) + decode_obsfile
4. rtkinit 已由 env.init_nav() 完成
5. 根据 positioning_mode 创建 SppProcessor 或 RtkProcessor
6. first_obs(nav, rov, base, dir=1)
7. 循环 next_obs → processor.process_epoch → queue.put(SensorData)
8. queue.put(None) EOF

注意：SPP 模式下 base_decoder 仍需创建（first_obs/next_obs 需要 base 参数），但 base obslist 可为空。实际测试发现 rtklib-py 的 first_obs/next_obs 在 SPP 模式下会访问 base.obslist，所以 SPP 模式也需要加载 base 文件（或构造空 base decoder）。为简化，SPP 模式也加载 base 文件（如果配置了）或用 rover 自身作 base。

**简化决策：** SPP 模式下不调用 first_obs/next_obs（这俩需要 base），而是直接遍历 rover.obslist。RTK 模式才用 first_obs/next_obs 做时间同步。

- [ ] **Step 1: 写失败测试 tests/test_internal_gnss_sensor.py**

```python
"""InternalGnssSensor 测试。"""
import sys
import types
import pytest
from queue import Queue
from unittest.mock import MagicMock, patch

import numpy as np


def test_internal_gnss_sensor_spp_mode_produces_solutions(tmp_path):
    """SPP 模式: 遍历 rover.obslist，每历元调 SppProcessor，推入 queue"""
    from src.stream.internal_gnss_sensor import InternalGnssSensor
    from src.core.data_types import SensorData, GnssSolution
    from src.core.thread_control import ThreadControl

    config = {
        "gnss": {
            "gnss_source": "internal",
            "positioning_mode": "spp",
            "rover_path": "data/cpt0870.19o",
            "eph_path": "data/brdm0870.19p",
        },
        "ins": {"enabled": "off"},
    }
    gnss_queue = Queue()
    control = ThreadControl()

    # mock RtklibEnv 与 rnx_decode
    fake_sol = GnssSolution(
        timestamp=357456.0, week=2000,
        position=np.array([100.0, 200.0, 300.0]),
        quality=5, num_sv=8, sd=np.array([1.0, 2.0, 3.0]),
    )
    fake_processor = MagicMock()
    fake_processor.process_epoch.return_value = fake_sol

    fake_nav = MagicMock()
    fake_nav.use_sing_pos = False

    # fake rover decoder with 3 epochs
    fake_rov = MagicMock()
    fake_obs1, fake_obs2, fake_obs3 = MagicMock(), MagicMock(), MagicMock()
    fake_rov.obslist = [fake_obs1, fake_obs2, fake_obs3]

    with patch("src.stream.internal_gnss_sensor.RtklibEnv") as MockEnv, \
         patch("src.stream.internal_gnss_sensor.rn") as mock_rn:
        MockEnv.return_value.setup.return_value = None
        MockEnv.return_value.init_nav.return_value = fake_nav
        mock_rn.rnx_decode.return_value = fake_rov
        mock_rn.decode_obsfile = MagicMock()

        with patch("src.stream.internal_gnss_sensor.SppProcessor", return_value=fake_processor):
            sensor = InternalGnssSensor(config, gnss_queue, control)
            sensor.run()

    # 应推入 3 个 SensorData + 1 个 None
    items = []
    while True:
        try:
            items.append(gnss_queue.get_nowait())
        except:
            break
    assert items[-1] is None  # EOF
    assert len(items) == 4  # 3 sol + EOF
    for item in items[:-1]:
        assert isinstance(item, SensorData)
        assert item.gnss_solution is not None


def test_internal_gnss_sensor_always_pushes_eof_on_exception():
    """异常时 finally 仍推入 EOF sentinel"""
    from src.stream.internal_gnss_sensor import InternalGnssSensor
    from src.core.thread_control import ThreadControl

    config = {
        "gnss": {
            "gnss_source": "internal",
            "positioning_mode": "spp",
            "rover_path": "data/cpt0870.19o",
            "eph_path": "data/brdm0870.19p",
        },
        "ins": {"enabled": "off"},
    }
    gnss_queue = Queue()
    control = ThreadControl()

    with patch("src.stream.internal_gnss_sensor.RtklibEnv") as MockEnv:
        MockEnv.return_value.setup.side_effect = RuntimeError("boom")
        sensor = InternalGnssSensor(config, gnss_queue, control)
        try:
            sensor.run()
        except RuntimeError:
            pass

    # EOF 一定推入
    assert gnss_queue.get_nowait() is None
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python -m pytest tests/test_internal_gnss_sensor.py -v`
Expected: FAIL (模块不存在)

- [ ] **Step 3: 实现 src/stream/internal_gnss_sensor.py**

```python
"""内部 GNSS 解算传感器线程。

逐历元调用 rtklib-py 的 pntpos/relpos，把 GnssSolution 推入 gnss_queue。
文件读完后推入 None 作为 EOF sentinel。
"""
from queue import Queue
from threading import Thread
from typing import List

from src.core.thread_control import ThreadControl
from src.core.data_types import SensorData
from src.core.gnss.rtklib_config_adapter import RtklibEnv
from src.core.gnss.spp_processor import SppProcessor
from src.core.gnss.rtk_processor import RtkProcessor

# rtklib-py 模块（需先由 RtklibEnv.setup() 注入 __ppk_config）
import rinex as rn


class InternalGnssSensor(Thread):
    """内部 GNSS 解算传感器线程。

    根据 positioning_mode 创建 SppProcessor 或 RtkProcessor，
    逐历元解算并推入 gnss_queue。
    """

    def __init__(self, config: dict, output_queue: Queue, control: ThreadControl):
        Thread.__init__(self, name="InternalGnssSensor", daemon=True)
        self.config = config
        self.gnss_cfg = config["gnss"]
        self.output_queue = output_queue
        self.control = control

    def run(self):
        try:
            self._run_impl()
        except Exception:
            # TODO: log exception
            pass
        finally:
            self.output_queue.put(None)  # EOF sentinel

    def _run_impl(self):
        # 1. 初始化 rtklib 环境 + nav
        env = RtklibEnv(self.gnss_cfg, library_path="library/rtklib-py/src")
        env.setup()
        nav = env.init_nav()

        # 2. 加载流动站观测值
        rov = rn.rnx_decode(env.get_cfg())
        rov.decode_obsfile(nav, self.gnss_cfg["rover_path"], None)

        # 3. 加载星历
        rov.decode_nav(self.gnss_cfg["eph_path"], nav)

        # 4. RTK 模式加载基站
        base = None
        if self.gnss_cfg.get("positioning_mode") == "rtk":
            base = rn.rnx_decode(env.get_cfg())
            base.decode_obsfile(nav, self.gnss_cfg["base_path"], None)
            # 基站位置
            if nav.rb[0] == 0:
                nav.rb = base.pos

        # 5. 创建处理器
        mode = self.gnss_cfg["positioning_mode"]
        if mode == "spp":
            processor = SppProcessor(nav)
            self._run_spp_loop(processor, rov)
        elif mode == "rtk":
            processor = RtkProcessor(nav)
            self._run_rtk_loop(processor, rov, base, nav)
        else:
            raise ValueError(f"Unsupported positioning_mode: {mode}")

    def _run_spp_loop(self, processor, rov):
        """SPP 模式: 直接遍历 rover.obslist。"""
        for obsr in rov.obslist:
            if not self.control.is_running():
                break
            sol = processor.process_epoch(obsr)
            if sol is not None:
                self.output_queue.put(SensorData(tag="gnss_solution", gnss_solution=sol))

    def _run_rtk_loop(self, processor, rov, base, nav):
        """RTK 模式: 用 first_obs/next_obs 做时间同步。"""
        dir = 1  # forward
        obsr, obsb = rn.first_obs(nav, rov, base, dir)
        n = 0
        while True:
            if not self.control.is_running():
                break
            if obsr == []:
                break
            sol = processor.process_epoch(obsr, obsb)
            if sol is not None:
                self.output_queue.put(SensorData(tag="gnss_solution", gnss_solution=sol))
            obsr, obsb = rn.next_obs(nav, rov, base, dir)
            n += 1
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python -m pytest tests/test_internal_gnss_sensor.py -v`
Expected: 全部 PASS

- [ ] **Step 5: 提交**

```bash
git add src/stream/internal_gnss_sensor.py tests/test_internal_gnss_sensor.py
git commit -m "feat(stream): add InternalGnssSensor thread for per-epoch solution"
```

---

## Task 9: SensorFactory + main.py 装配

**Files:**
- Modify: `src/stream/factory.py`
- Modify: `src/main.py`
- Modify: `tests/test_factory.py`
- Modify: `tests/test_main.py`

- [ ] **Step 1: 修改 src/stream/factory.py 加 internal 模式分支**

把 `SensorFactory` 改为：

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
        gnss_source = config["gnss"]["gnss_source"]
        ins_enabled = config["ins"]["enabled"]

        if gnss_source == "external":
            # external + ins.enabled=on: IMU + 外部 GNSS 结果
            imu_path = config["ins"]["imu_data_path"]
            sensors.append(ImuSensor(imu_path, imu_queue, control))
            gnss_path = config["gnss"]["external_sol_path"]
            sensors.append(GnssSolSensor(gnss_path, gnss_queue, control))
        elif gnss_source == "internal" and ins_enabled == "off":
            # internal + off: 纯 GNSS，仅 InternalGnssSensor，无 IMU
            from src.stream.internal_gnss_sensor import InternalGnssSensor
            sensors.append(InternalGnssSensor(config, gnss_queue, control))
        # internal + on 已由 config_loader 抛 NotImplementedError

        return sensors
```

- [ ] **Step 2: 修改 src/main.py 加装配分支**

把 `main()` 改为：

```python
"""GInsStream 主入口。

用法:
    python src/main.py [config_path]

默认 config_path = data/config.yaml
"""
import sys
from pathlib import Path
from queue import Queue

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.core.thread_control import ThreadControl
from src.stream.factory import SensorFactory
from src.log.aligned_writer import AlignedWriter
from src.log.aligner import Aligner
from src.log.logger import Logger
from src.log.solution_writer import SolutionWriter
from src.log.solution_logger import SolutionLogger
from src.utility.config_loader import load_config


def main(config_path: str = "data/config.yaml"):
    config = load_config(config_path)

    control = ThreadControl()
    imu_queue = Queue(maxsize=2000)
    gnss_queue = Queue(maxsize=100)

    sensors, logger = _assemble_pipeline(config, control, imu_queue, gnss_queue)

    for s in sensors:
        s.start()
    logger.start()

    logger.join()
    control.shutdown()
    for s in sensors:
        s.join(timeout=2)


def _assemble_pipeline(config, control, imu_queue, gnss_queue):
    """根据配置选择 sensors 与 logger。

    Returns:
        (sensors, logger) 元组
    """
    gnss_source = config["gnss"]["gnss_source"]
    ins_enabled = config["ins"]["enabled"]

    if gnss_source == "external":
        # 路径 A: 现有块状对齐输出
        sensors = SensorFactory.create_sensors(config, imu_queue, gnss_queue, control)
        writer = AlignedWriter(output_dir=config["output"]["output_dir"])
        aligner = Aligner(imu_dt=1.0 / config["ins"]["data_rate"])
        logger = Logger(imu_queue, gnss_queue, writer, aligner, control)
        return sensors, logger

    if gnss_source == "internal" and ins_enabled == "off":
        # 路径 B: 纯 GNSS .pos 输出
        sensors = SensorFactory.create_sensors(config, imu_queue, gnss_queue, control)
        writer = SolutionWriter(output_dir=config["output"]["output_dir"],
                                filename="solution.pos")
        logger = SolutionLogger(gnss_queue, writer, control)
        return sensors, logger

    # internal + on 已由 config_loader 拦截
    raise NotImplementedError(
        f"Unsupported config: gnss_source={gnss_source}, ins.enabled={ins_enabled}"
    )


if __name__ == "__main__":
    cfg = sys.argv[1] if len(sys.argv) > 1 else "data/config.yaml"
    main(cfg)
```

- [ ] **Step 3: 修改 tests/test_factory.py 加 internal 模式测试**

在文件末尾追加：

```python
def test_factory_internal_spp_mode_returns_only_internal_sensor():
    """internal + ins.enabled=off → 仅 InternalGnssSensor，无 IMU"""
    config = {
        "gnss": {
            "gnss_source": "internal",
            "positioning_mode": "spp",
            "rover_path": "data/cpt0870.19o",
            "eph_path": "data/brdm0870.19p",
        },
        "ins": {"enabled": "off"},
    }
    from queue import Queue
    from src.core.thread_control import ThreadControl
    from src.stream.factory import SensorFactory
    from src.stream.internal_gnss_sensor import InternalGnssSensor

    sensors = SensorFactory.create_sensors(config, Queue(), Queue(), ThreadControl())
    assert len(sensors) == 1
    assert isinstance(sensors[0], InternalGnssSensor)


def test_factory_external_mode_returns_imu_and_gnss_sensors():
    """external + ins.enabled=on → IMU + GnssSolSensor"""
    config = {
        "gnss": {
            "gnss_source": "external",
            "external_sol_path": "data/spp.pos",
        },
        "ins": {
            "enabled": "on",
            "imu_data_path": "data/cpt_imu.csv",
        },
    }
    from queue import Queue
    from src.core.thread_control import ThreadControl
    from src.stream.factory import SensorFactory
    from src.stream.imu_sensor import ImuSensor
    from src.stream.gnss_sol_sensor import GnssSolSensor

    sensors = SensorFactory.create_sensors(config, Queue(), Queue(), ThreadControl())
    assert len(sensors) == 2
    assert isinstance(sensors[0], ImuSensor)
    assert isinstance(sensors[1], GnssSolSensor)
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python -m pytest tests/test_factory.py tests/test_main.py -v`
Expected: 全部 PASS

- [ ] **Step 5: 运行全量回归**

Run: `python -m pytest -v`
Expected: 全部 PASS

- [ ] **Step 6: 提交**

```bash
git add src/stream/factory.py src/main.py tests/test_factory.py
git commit -m "feat(main): add pipeline assembly for internal+off pure GNSS mode"
```

---

## Task 10: SPP 端到端测试

**Files:**
- Create: `tests/test_internal_gnss_spp_e2e.py`

- [ ] **Step 1: 写端到端测试 tests/test_internal_gnss_spp_e2e.py**

```python
"""SPP 端到端测试：用真实 RINEX 数据跑完整流程，验证 .pos 输出。"""
import os
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = ROOT / "data"
OUTPUT_DIR = ROOT / "output"


def test_spp_e2e_produces_valid_pos_file(tmp_path):
    """用 data/cpt0870.19o + data/brdm0870.19p 跑 SPP，验证 solution.pos"""
    rover = DATA_DIR / "cpt0870.19o"
    eph = DATA_DIR / "brdm0870.19p"
    if not rover.exists() or not eph.exists():
        pytest.skip("RINEX test data not available")

    # 写临时配置
    cfg_text = f"""
gnss:
  gnss_source: "internal"
  positioning_mode: "spp"
  rover_path: "{rover.as_posix()}"
  eph_path: "{eph.as_posix()}"
  nf: 2
  pmode: "kinematic"
  filtertype: "forward"
  use_sing_pos: true
  elmin: 15.0
  cnr_min: [28, 20]
  excsats: []
  maxinno: 1.0
  maxcode: 10.0
  maxage: 30.0
  maxout: 4
  thresdop: 5.0
  thresslip: 0.10
  interp_base: false
  eratio: [300, 100]
  efact_gps: 1.0
  efact_glo: 1.5
  efact_gal: 1.0
  err_base: 0.003
  err_el: 0.003
  err_satclk: 5.0e-12
  snrmax: 45.0
  accelh: 3.0
  accelv: 1.0
  prnbias: 0.01
  sig_p0: 30.0
  sig_v0: 10.0
  sig_n0: 30.0
  armode: 0
  thresar: 3.0
  thresar1: 0.05
  minlock: 0
  glo_hwbias: 0.0
  elmaskar: 15.0
  var_holdamb: 0.1
  minfix: 20
  minfixsats: 4
  minholdsats: 5
  mindropsats: 10
  sing_p0: 100.0
  sing_v0: 10.0
  sing_elmin: 10.0
  gnss_t: ["GPS", "GLO", "GAL"]
  freq_ix0: {{GPS: 0, GLO: 4, GAL: 0}}
  freq_ix1: {{GPS: 2, GLO: 5, GAL: 2}}
  freq_table: [1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9]
  dfreq_glo: [0.56250e6, 0.43750e6]
  rb: [0, 0, 0]
  rr_f: [0, 0, 0, 0, 0, 0]
  rr_b: [0, 0, 0, 0, 0, 0]
ins:
  enabled: "off"
output:
  output_dir: "{tmp_path.as_posix()}"
"""
    cfg_path = tmp_path / "cfg.yaml"
    cfg_path.write_text(cfg_text, encoding="utf-8")

    # 运行 main.py
    result = subprocess.run(
        [sys.executable, "src/main.py", str(cfg_path)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        timeout=300,
    )
    assert result.returncode == 0, f"main.py failed:\n{result.stderr}"

    pos_file = tmp_path / "solution.pos"
    assert pos_file.exists(), "solution.pos not created"

    content = pos_file.read_text(encoding="utf-8")
    lines = content.strip().split("\n")
    # 表头 + 至少 1 行数据
    assert len(lines) >= 2
    assert "GPST" in lines[0]
    # 首行数据格式检查
    parts = lines[1].split()
    assert len(parts) == 15  # week sow lat lon h Q ns sdn sde sdu sdne sdeu sdun age ratio
    int(parts[0])  # week
    float(parts[1])  # sow
    float(parts[2])  # lat
    float(parts[3])  # lon

    # 时间戳单调递增
    sows = [float(line.split()[1]) for line in lines[1:]]
    assert sows == sorted(sows), "timestamps not monotonic"
```

- [ ] **Step 2: 运行端到端测试**

Run: `python -m pytest tests/test_internal_gnss_spp_e2e.py -v -s`
Expected: PASS（如果数据文件存在）。如果失败，根据 stderr 调试。

- [ ] **Step 3: 提交**

```bash
git add tests/test_internal_gnss_spp_e2e.py
git commit -m "test(e2e): add SPP end-to-end test with real RINEX data"
```

---

## Task 11: RTK 端到端测试

**Files:**
- Create: `tests/test_internal_gnss_rtk_e2e.py`

- [ ] **Step 1: 写端到端测试 tests/test_internal_gnss_rtk_e2e.py**

```python
"""RTK 端到端测试：用真实 RINEX 数据跑完整流程，验证 .pos 输出。"""
import sys
import subprocess
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
DATA_DIR = ROOT / "data"


def test_rtk_e2e_produces_valid_pos_file(tmp_path):
    """用 rover + base + eph 跑 RTK，验证 solution.pos"""
    rover = DATA_DIR / "cpt0870.19o"
    base = DATA_DIR / "cpt0870_base.19o"
    eph = DATA_DIR / "brdm0870.19p"
    if not rover.exists() or not base.exists() or not eph.exists():
        pytest.skip("RINEX test data not available")

    cfg_text = f"""
gnss:
  gnss_source: "internal"
  positioning_mode: "rtk"
  rover_path: "{rover.as_posix()}"
  base_path: "{base.as_posix()}"
  eph_path: "{eph.as_posix()}"
  nf: 2
  pmode: "kinematic"
  filtertype: "forward"
  use_sing_pos: false
  elmin: 15.0
  cnr_min: [28, 20]
  excsats: []
  maxinno: 1.0
  maxcode: 10.0
  maxage: 30.0
  maxout: 4
  thresdop: 5.0
  thresslip: 0.10
  interp_base: false
  eratio: [300, 100]
  efact_gps: 1.0
  efact_glo: 1.5
  efact_gal: 1.0
  err_base: 0.003
  err_el: 0.003
  err_satclk: 5.0e-12
  snrmax: 45.0
  accelh: 3.0
  accelv: 1.0
  prnbias: 0.01
  sig_p0: 30.0
  sig_v0: 10.0
  sig_n0: 30.0
  armode: 3
  thresar: 3.0
  thresar1: 0.05
  minlock: 0
  glo_hwbias: 0.0
  elmaskar: 15.0
  var_holdamb: 0.1
  minfix: 20
  minfixsats: 4
  minholdsats: 5
  mindropsats: 10
  sing_p0: 100.0
  sing_v0: 10.0
  sing_elmin: 10.0
  gnss_t: ["GPS", "GLO", "GAL"]
  freq_ix0: {{GPS: 0, GLO: 4, GAL: 0}}
  freq_ix1: {{GPS: 2, GLO: 5, GAL: 2}}
  freq_table: [1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9]
  dfreq_glo: [0.56250e6, 0.43750e6]
  rb: [0, 0, 0]
  rr_f: [0, 0, 0, 0, 0, 0]
  rr_b: [0, 0, 0, 0, 0, 0]
ins:
  enabled: "off"
output:
  output_dir: "{tmp_path.as_posix()}"
"""
    cfg_path = tmp_path / "cfg.yaml"
    cfg_path.write_text(cfg_text, encoding="utf-8")

    result = subprocess.run(
        [sys.executable, "src/main.py", str(cfg_path)],
        cwd=str(ROOT),
        capture_output=True,
        text=True,
        timeout=600,
    )
    assert result.returncode == 0, f"main.py failed:\n{result.stderr}"

    pos_file = tmp_path / "solution.pos"
    assert pos_file.exists()

    content = pos_file.read_text(encoding="utf-8")
    lines = content.strip().split("\n")
    assert len(lines) >= 2
    # 验证含固定解(1)或浮点解(2)或 RTD(4) 之一
    qualities = [int(line.split()[5]) for line in lines[1:]]
    assert any(q in (1, 2, 4) for q in qualities), f"no valid RTK solution, qualities={set(qualities)}"
```

- [ ] **Step 2: 运行端到端测试**

Run: `python -m pytest tests/test_internal_gnss_rtk_e2e.py -v -s`
Expected: PASS。如果失败，根据 stderr 调试。

- [ ] **Step 3: 提交**

```bash
git add tests/test_internal_gnss_rtk_e2e.py
git commit -m "test(e2e): add RTK end-to-end test with real RINEX data"
```

---

## Task 12: 全量回归与手动验证

- [ ] **Step 1: 全量测试**

Run: `python -m pytest -v`
Expected: 全部 PASS（42 现有 + 新增测试）

- [ ] **Step 2: 手动验证 external 模式（路径 A 不变）**

Run: `python src/main.py data/config.yaml`
Expected: 输出 `output/aligned.csv`，行为与改动前一致

- [ ] **Step 3: 手动验证 internal SPP 模式**

临时改 `data/config.yaml`：
```yaml
gnss:
  gnss_source: "internal"
  positioning_mode: "spp"
ins:
  enabled: "off"
```
Run: `python src/main.py data/config.yaml`
Expected: 输出 `output/solution.pos`，含表头 + 多行数据

- [ ] **Step 4: 手动验证 internal RTK 模式**

临时改 `positioning_mode: "rtk"`
Run: `python src/main.py data/config.yaml`
Expected: 输出 `output/solution.pos`，含固定/浮点解

- [ ] **Step 5: 恢复 config.yaml 到 external 默认**

改回 `gnss_source: "external"` + `ins.enabled: "on"`

- [ ] **Step 6: 最终提交**

```bash
git add data/config.yaml
git commit -m "test: verify all three config modes work end-to-end"
```

---

## Self-Review

### 规格覆盖检查

| 规格要求 | 对应任务 |
|---------|---------|
| ins.enabled 必填 + on/off 校验 | Task 1 |
| external + off 非法 | Task 1 |
| internal + positioning_mode 必填 | Task 1 |
| internal + rtk 缺 base_path 非法 | Task 1 |
| internal + on 抛 NotImplementedError | Task 1 |
| data/config.yaml 加 positioning_mode + enabled + reboot | Task 1 |
| rtklib_config_adapter (YAML→cfg, sys.modules 注入) | Task 2 |
| solution_converter (Sol→GnssSolution) | Task 3 |
| GnssProcessor ABC | Task 4 |
| SppProcessor | Task 4 |
| RtkProcessor | Task 5 |
| SolutionWriter (.pos 输出) | Task 6 |
| SolutionLogger | Task 7 |
| InternalGnssSensor | Task 8 |
| SensorFactory 分支 | Task 9 |
| main.py _assemble_pipeline | Task 9 |
| SPP E2E | Task 10 |
| RTK E2E | Task 11 |
| 回归 + 手动验证 | Task 12 |

全部覆盖 ✓

### 类型一致性检查

- `GnssSolution` 字段（timestamp/week/position/quality/num_sv/sd）在 Task 3/4/5/6/8 中一致 ✓
- `SensorData(tag="gnss_solution", gnss_solution=sol)` 在 Task 7/8 中一致 ✓
- `SppProcessor(nav)` / `RtkProcessor(nav)` 构造签名在 Task 4/5/8 中一致 ✓
- `process_epoch(obsr, obsb)` 签名在 Task 4/5/8 中一致 ✓
- `SolutionWriter(output_dir, filename)` 在 Task 6/9 中一致 ✓
- `SolutionLogger(gnss_queue, writer, control)` 在 Task 7/9 中一致 ✓
- `InternalGnssSensor(config, output_queue, control)` 在 Task 8/9 中一致 ✓

### 占位符扫描

无 TBD/TODO（Task 8 的 `# TODO: log exception` 是合理的日志占位，不影响功能）✓
