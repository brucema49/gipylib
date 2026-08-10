# 吸收 rtklib-py 到 src/ 子包设计

> 日期: 2026-07-03
> 主题: 将 library/rtklib-py 的 7 个核心算法模块吸收为 src/core/gnss/rtklib/ 子包，消除运行时对 library/ 的导入依赖；tools/ 脚本吸收为 src/tools/
> 范围: 导入路径重构 + __ppk_config 机制改造 + RtklibEnv 简化；不修改算法逻辑

---

## 1. 背景与目标

### 1.1 当前状态

gipylib 运行时通过 `RtklibEnv.setup()` 做两件事：
1. `sys.path.insert(0, "library/rtklib-py/src")` — 将 rtklib-py 源码目录加入搜索路径
2. `sys.modules["__ppk_config"] = build_cfg_module(...)` — 动态生成配置模块注入

gipylib 代码用延迟导入引用 rtklib-py：`from pntpos import pntpos`、`import rinex as rn` 等（共 10 处，分布在 5 个源文件 + 3 个测试文件 + 2 个工具脚本）。

rtklib-py 模块间用**顶层扁平导入**：`import rtkcmn as gn`、`from rtkcmn import rCST` 等（共约 35 行）。`__ppk_config` 被 rtkpos.py 和 postpos.py 在顶层导入。

### 1.2 问题

- `src/` 不自包含，运行时依赖 `library/rtklib-py/src/` 目录存在
- `sys.path` 注入是全局副作用，影响其他代码
- `sys.modules` 注入动态生成模块，难以调试与 IDE 解析
- `tools/` 脚本独立于 `src/`，不符合"所有源代码在 src/"的要求

### 1.3 目标

1. rtklib-py 7 个核心模块吸收为 `src/core/gnss/rtklib/` 子包，改用相对导入
2. `__ppk_config` 动态文件机制改为 `config.py` 模块级单例
3. `RtklibEnv` 移除 `sys.path`/`sys.modules` 操作，改用 `config.set_params()`
4. `tools/` 2 个脚本吸收为 `src/tools/`
5. `library/` 保留为只读参考，不再运行时引用
6. 现有 80+ 测试全过，SPP/RTK 比对仍 CONSISTENT

### 1.4 非目标

- 不修改 rtklib-py 的算法逻辑（只改导入语句）
- 不吸收 `run_ppk.py`、`config_f9p.py`、`config_phone.py`（参考入口与配置）
- 不删除 `library/` 目录（保留为参考）
- 不重构 gipylib 的业务代码结构

---

## 2. 目录结构

### 2.1 吸收后的 src/ 结构

```
src/
├── __init__.py
├── main.py
├── core/
│   ├── __init__.py
│   ├── data_types.py
│   ├── thread_control.py
│   ├── time_utils.py
│   └── gnss/
│       ├── __init__.py
│       ├── gnss_processor.py
│       ├── rtklib/                        ← 新增子包
│       │   ├── __init__.py                ← 导出公共 API
│       │   ├── config.py                  ← 替代 __ppk_config
│       │   ├── rtkcmn.py                  ← 从 library/ 吸收
│       │   ├── ephemeris.py
│       │   ├── mlambda.py
│       │   ├── rinex.py
│       │   ├── pntpos.py
│       │   ├── rtkpos.py
│       │   └── postpos.py
│       ├── rtklib_config_adapter.py       ← 简化
│       ├── rtk_processor.py               ← 改导入
│       ├── solution_converter.py
│       └── spp_processor.py               ← 改导入
├── log/
│   └── (不变)
├── stream/
│   ├── internal_gnss_sensor.py            ← 改导入
│   └── (其余不变)
├── tools/                                  ← 新增
│   ├── __init__.py
│   ├── verify_rtklib_py.py                ← 从 tools/ 吸收
│   └── compare_pos.py
└── utility/
    └── (不变)
```

### 2.2 保留不变

- `library/` 整体保留为只读参考（含 rtklib-py 原始代码）
- `tests/` 结构不变，仅改导入路径
- `data/`、`output/`、`docs/` 不变

---

## 3. 关键改动

### 3.1 rtklib-py 模块导入改写

7 个核心模块的顶层扁平导入改为相对导入。完整改写表：

**rtkcmn.py** — 无内部导入（仅 stdlib + numpy），无需改写

**ephemeris.py**:
```python
# 改前
from rtkcmn import uGNSS, rCST, timediff, timeadd, vnorm, time2epoch
from rtkcmn import sat2prn, trace
# 改后
from .rtkcmn import uGNSS, rCST, timediff, timeadd, vnorm, time2epoch
from .rtkcmn import sat2prn, trace
```

**mlambda.py** — 无内部导入（仅 numpy），无需改写

**rinex.py**:
```python
# 改前
from rtkcmn import uGNSS, rSIG, Eph, Geph, prn2sat, gpst2time, time2gpst, Obs, ...
import rtkcmn as gn
from ephemeris import satposs
# 改后
from .rtkcmn import uGNSS, rSIG, Eph, Geph, prn2sat, gpst2time, time2gpst, Obs, ...
from . import rtkcmn as gn
from .ephemeris import satposs
```

**pntpos.py**:
```python
# 改前
from rtkcmn import rCST, ecef2pos, geodist, satazel, ionmodel, tropmodel, ...
import rtkcmn as gn
from ephemeris import seleph, satposs
from rinex import rcvstds
# 改后
from .rtkcmn import rCST, ecef2pos, geodist, satazel, ionmodel, tropmodel, ...
from . import rtkcmn as gn
from .ephemeris import seleph, satposs
from .rinex import rcvstds
```

**rtkpos.py**:
```python
# 改前
import rtkcmn as gn
from rtkcmn import rCST, DTTOL, sat2prn, sat2freq, timediff, xyz2enu
import rinex as rn
from pntpos import pntpos
from ephemeris import satposs
from mlambda import mlambda
from rtkcmn import trace, tracemat, uGNSS
import __ppk_config as cfg
# 改后
from . import rtkcmn as gn
from .rtkcmn import rCST, DTTOL, sat2prn, sat2freq, timediff, xyz2enu
from . import rinex as rn
from .pntpos import pntpos
from .ephemeris import satposs
from .mlambda import mlambda
from .rtkcmn import trace, tracemat, uGNSS
from . import config as cfg
```

**postpos.py**:
```python
# 改前
from rtkpos import rtkpos, rtkinit
import __ppk_config as cfg
import rinex as rn
from pntpos import pntpos
import rtkcmn as gn
# 改后
from .rtkpos import rtkpos, rtkinit
from . import config as cfg
from . import rinex as rn
from .pntpos import pntpos
from . import rtkcmn as gn
```

### 3.2 `__ppk_config` → `config.py` 模块级单例

当前机制：`RtklibEnv` 调用 `build_cfg_module(gnss_cfg)` 动态生成一个模块对象，注入 `sys.modules["__ppk_config"]`。rtkpos.py 和 postpos.py 顶层 `import __ppk_config as cfg` 取到这个模块。

新机制：

```python
# src/core/gnss/rtklib/config.py
"""rtklib-py 配置单例。RtklibEnv.setup() 调用 set_params() 注入参数。

替代原 rtklib-py 的 __ppk_config 动态模块机制。
rtkpos.py / postpos.py 通过 `from . import config as cfg` 引用，
用法不变：cfg.pos_elmin, cfg.cnr_min 等。
"""
from .rtkcmn import uGNSS, rSIG  # 与原 __ppk_config.py 一致

_params = {}


class _CfgProxy:
    """属性代理，从 _params 字典取值。"""
    def __getattr__(self, name):
        try:
            return _params[name]
        except KeyError:
            raise AttributeError(
                f"rtklib config has no attribute '{name}'. "
                f"Did you call RtklibEnv.setup()?"
            )


cfg = _CfgProxy()


def set_params(params: dict) -> None:
    """RtklibEnv 调用此函数注入配置参数。

    params 字典的 key 应与原 __ppk_config.py 的模块级变量名一致，
    如 pos_elmin, cnr_min, eratio, armode 等。
    """
    _params.update(params)


def reset() -> None:
    """清空配置（测试隔离用）。"""
    _params.clear()
```

rtkpos.py 和 postpos.py 的 `import __ppk_config as cfg` 改为 `from . import config as cfg`。使用方式完全不变（`cfg.pos_elmin` 等属性访问）。

### 3.3 `RtklibEnv` 简化

**删除**：
- `self.library_path` 字段与构造参数
- `sys.path.insert(0, self.library_path)` 
- `sys.modules["__ppk_config"] = ...` 注入
- `build_cfg_module()` 函数（配置构建逻辑保留，改为返回 dict）
- `cleanup()` 中的 sys.path/sys.modules 清理

**新增/改写**：

```python
class RtklibEnv:
    def __init__(self, gnss_cfg: dict, library_path: str = None):
        # library_path 参数保留但忽略（向后兼容，打印 deprecation warning）
        self.gnss_cfg = gnss_cfg
        self._setup_done = False

    def setup(self):
        from .rtklib import config
        config.set_params(self._build_params())
        
        from .rtklib import rtkcmn as _gn
        if not hasattr(_gn, "trace_level"):
            _gn.tracelevel(0)
        self._setup_done = True

    def _build_params(self) -> dict:
        """从 gnss_cfg 构建 rtklib-py 配置参数字典。
        
        逻辑与原 build_cfg_module() 一致，但返回 dict 而非模块对象。
        """
        # ... (复用原 build_cfg_module 的参数翻译逻辑)
        return params

    def init_nav(self):
        from .rtklib.rtkpos import rtkinit
        from .rtklib import config
        return rtkinit(config.cfg)

    def cleanup(self):
        """重置配置（测试隔离用）。"""
        from .rtklib import config
        config.reset()
        self._setup_done = False
```

### 3.4 gipylib 代码导入改写

| 文件 | 原导入 | 改为 |
|------|--------|------|
| `spp_processor.py:19` | `from pntpos import pntpos` | `from ..rtklib.pntpos import pntpos` |
| `rtk_processor.py:24` | `from pntpos import pntpos` | `from ..rtklib.pntpos import pntpos` |
| `rtk_processor.py:25` | `from rtkpos import relpos, timediff` | `from ..rtklib.rtkpos import relpos, timediff` |
| `rtk_processor.py:26` | `from rtkcmn import Sol, gtime_t` | `from ..rtklib.rtkcmn import Sol, gtime_t` |
| `rtklib_config_adapter.py:21` | `from rtkcmn import uGNSS` | `from .rtklib.rtkcmn import uGNSS` |
| `rtklib_config_adapter.py:123` | `from rtkcmn import rSIG` | `from .rtklib.rtkcmn import rSIG` |
| `rtklib_config_adapter.py:166` | `import rtkcmn as _gn` | `from .rtklib import rtkcmn as _gn` |
| `rtklib_config_adapter.py:179` | `from rtkpos import rtkinit` | `from .rtklib.rtkpos import rtkinit` |
| `internal_gnss_sensor.py:67` | `import rinex as rn` | `from ..core.gnss.rtklib import rinex as rn` |
| `tools/verify_rtklib_py.py` (多处) | `import rinex as rn` 等 | `from src.core.gnss.rtklib import rinex as rn` 等 |

### 3.5 测试文件改写

3 个测试文件删除 `sys.path.insert(0, "library/rtklib-py/src")`，改导入：

| 文件 | 原导入 | 改为 |
|------|--------|------|
| `test_rtk_processor.py:82` | `sys.path.insert(...)` + `from rtkcmn import ...` | 删除 sys.path 行 + `from src.core.gnss.rtklib.rtkcmn import ...` |
| `test_rtklib_config_adapter.py:33` | 同上 | 同上 |
| `test_spp_processor.py:79` | 同上 | 同上 |
| `test_rtklib_config_adapter.py:63,70` | `from rtkcmn import uGNSS` | `from src.core.gnss.rtklib.rtkcmn import uGNSS` |

### 3.6 `__init__.py` 设计

```python
# src/core/gnss/rtklib/__init__.py
"""rtklib-py 核心算法子包。

从 library/rtklib-py/src/ 吸收，改为相对导入。
公共 API 通过此 __init__.py 导出，方便外部引用。
"""
from . import config  # 确保配置单例可用
from .rtkcmn import Sol, gtime_t, uGNSS, rSIG, rCST
from .rinex import rnx_decode
from .pntpos import pntpos
from .rtkpos import relpos, timediff, rtkinit, rtkpos
from .postpos import procpos, savesol
```

外部代码可用 `from ..rtklib import pntpos` 或 `from ..rtklib.pntpos import pntpos`（等价）。

---

## 4. 数据流

```
data/cfg_test_*.yaml
        │
        ▼
src/main.py
        │
        ▼
RtklibEnv(gnss_cfg).setup()
        │
        ├─ from .rtklib import config
        ├─ config.set_params(params)        ← 注入配置到 config._params
        ├─ from .rtklib import rtkcmn
        └─ rtkcmn.tracelevel(0)
                │
                ▼
        nav = env.init_nav()
                │
                ├─ from .rtklib.rtkpos import rtkinit
                ├─ from .rtklib import config
                └─ rtkinit(config.cfg)      ← cfg 是 _CfgProxy, 属性访问取 _params
                        │
                        ▼
        SppProcessor / RtkProcessor
                │
                ├─ from ..rtklib.pntpos import pntpos
                ├─ from ..rtklib.rtkpos import relpos
                └─ from ..rtklib.rtkcmn import Sol
                        │
                        ▼
                解算 → GnssSolution → SolutionWriter → .pos
```

**不再有**：
- `sys.path.insert(0, "library/rtklib-py/src")`
- `sys.modules["__ppk_config"] = ...`
- 对 `library/` 的任何运行时引用

---

## 5. 错误处理

| 场景 | 处理 |
|------|------|
| `config.set_params()` 未调用就访问 `cfg.pos_elmin` | `_CfgProxy.__getattr__` 抛 `AttributeError`，消息提示"Did you call RtklibEnv.setup()?" |
| rtklib-py 模块间相对导入失败 | 检查 `__init__.py` 存在；`from .rtkcmn import X` 需要 `src/core/gnss/rtklib/__init__.py` |
| 测试中 `from src.core.gnss.rtklib import ...` 失败 | `tests/conftest.py` 已 `sys.path.insert(0, 项目根)`，应能解析 `src.` 前缀 |
| `library/` 目录被误删 | 不影响运行（仅参考）；文档注明 library/ 为可选参考 |
| `RtklibEnv(library_path=...)` 旧调用 | 参数保留但忽略，打印 `DeprecationWarning` |
| 测试隔离：多个测试串扰 config | `RtklibEnv.cleanup()` 调 `config.reset()`；测试 fixture 用 `autouse` 确保清理 |

---

## 6. 测试与验证

### 6.1 新增测试

| 测试文件 | 测试内容 |
|---------|---------|
| `tests/test_rtklib_config.py` | `config.set_params()` + `cfg.attr` 代理 + `reset()` + 未设置时 `AttributeError` |
| `tests/test_rtklib_imports.py` | 7 个模块能被导入；相对导入正确；`__init__.py` 导出的符号可用 |

### 6.2 现有测试回归

80+ 现有测试全部应通过（仅导入路径变化，逻辑不变）。重点检查：
- `test_rtk_processor.py` — 导入 `relpos`, `Sol`, `gtime_t`
- `test_spp_processor.py` — 导入 `pntpos`
- `test_rtklib_config_adapter.py` — 导入 `uGNSS`，测试 `RtklibEnv.setup()`
- `test_internal_gnss_sensor.py` — 间接通过 `internal_gnss_sensor` 导入 `rinex`

### 6.3 端到端验证

```bash
# 1. SPP + RTK 功能验证
python src/main.py data/cfg_test_spp.yaml
python src/main.py data/cfg_test_rtk.yaml

# 2. verify 工具（现在从 src/ 导入）
python -m src.tools.verify_rtklib_py spp
python -m src.tools.verify_rtklib_py rtk

# 3. 比对（应与之前结果一致）
python -m src.tools.compare_pos output/test_spp.pos output/verify_spp.pos
python -m src.tools.compare_pos output/test_rtk.pos output/verify_rtk.pos
```

### 6.4 回归标准

- 80+ 现有测试全过
- 新增 2 个测试文件全过
- SPP 比对：CONSISTENT（平面 < 0.5m，高程 < 1m）
- RTK 比对：CONSISTENT（平面 < 0.5m，高程 < 1m）

---

## 7. 实施顺序

1. **创建子包骨架**：`src/core/gnss/rtklib/__init__.py` + `config.py`
2. **吸收 7 个核心模块**：从 `library/rtklib-py/src/` 复制，机械改写导入为相对导入
3. **改写 `RtklibEnv`**：移除 sys.path/sys.modules 操作，改用 `config.set_params()`；`build_cfg_module` 改为 `_build_params` 返回 dict
4. **改写 gipylib 5 个源文件导入**：spp_processor / rtk_processor / rtklib_config_adapter / internal_gnss_sensor
5. **创建 `src/tools/`**：移动 verify_rtklib_py.py 和 compare_pos.py，改写导入
6. **改写 3 个测试文件导入**：删除 sys.path.insert，改用 `from src.core.gnss.rtklib...`
7. **新增 2 个测试文件**：test_rtklib_config.py + test_rtklib_imports.py
8. **跑全量测试**：80+ 现有 + 2 新增，全部应过
9. **端到端验证**：SPP/RTK 跑通 + 比对 CONSISTENT
10. **清理**：旧 `tools/` 目录的脚本可保留作参考或删除；`library/` 保留

---

## 8. 设计决策记录

### 8.1 为何选子包 + 相对导入（而非扁平放 src/ 根）

- gipylib 已有清晰的包结构（core/gnss/log/stream/utility），rtklib-py 作为 GNSS 算法库自然属于 `core/gnss/` 下
- 相对导入使模块间依赖显式，IDE 能静态解析
- 扁平放 src/ 根会污染命名空间（7 个模块名与 gipylib 自有模块可能冲突）
- 子包隔离使"哪些是吸收的第三方代码"一目了然

### 8.2 为何用 config.py 单例而非保留动态模块生成

- 动态生成 `__ppk_config.py` 文件 + `sys.modules` 注入是全局副作用，难以调试
- 模块级单例（`_CfgProxy`）是 Pythonic 的配置注入方式
- `_CfgProxy.__getattr__` 提供清晰的错误提示（"Did you call RtklibEnv.setup()?"）
- `reset()` 方法便于测试隔离

### 8.3 为何保留 `library/` 而非删除

- 用户明确说"library 和 tools 里的代码仅供参考"
- 保留原始 rtklib-py 代码便于对照升级、查 diff
- 删除会丢失上游参考；保留不碍事（不再运行时引用）

### 8.4 为何 tools/ 脚本吸收为 `src/tools/` 而非删除

- `verify_rtklib_py.py` 和 `compare_pos.py` 是有价值的验证工具
- 放 `src/tools/` 符合"所有源代码在 src/"要求
- 用 `python -m src.tools.verify_rtklib_py` 调用，符合 Python 包规范

### 8.5 为何 `RtklibEnv` 保留 `library_path` 参数但忽略

- 向后兼容：现有代码 `RtklibEnv(gnss_cfg, library_path="...")` 不会报错
- 打印 `DeprecationWarning` 提示该参数已无用
- 未来版本可移除

---

## 9. 未来工作

1. 移除 `RtklibEnv.library_path` 参数（下个版本）
2. 删除旧 `tools/` 目录（确认无引用后）
3. 考虑将 `library/rtklib-py/` 作为 git submodule 或完全删除（如果不再需要参考）
4. rtklib-py 上游升级时，对照 `library/rtklib-py/` 原始代码 merge 改动到 `src/core/gnss/rtklib/`
