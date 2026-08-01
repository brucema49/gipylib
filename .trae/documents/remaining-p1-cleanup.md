# 剩余 P1/P2 残留清理计划

## 摘要

前序审核（filter-state-parameter-audit.md）已完成核心滤波估计器的单滤波迁移：state_index.py、lc_estimator.py、nhc.py、transfer_matrix.py、initializer.py、ins_update.py、data_types.py、config.yaml 及 4 个核心测试文件均已迁移到单 P 接口。

回归检查发现 **4 个外围文件**仍有 P1/P2 残留，其中 2 个测试文件会因 `initializer.initialize()` 返回值从 `(state, P1, P2)` 变为 `(state, P)` 而**直接崩溃**（ValueError: too many values to unpack）。本计划清理这些残留，使全项目命名统一为单滤波 `P`。

## 当前状态分析

### 会崩溃的文件（必须修复）

| 文件 | 问题 | 崩溃点 |
|------|------|--------|
| [diag_lc.py](file:///home/mxl/workplace/gipylib/tests/ins/diag_lc.py) | 使用旧 3 参数 LcEstimator + 3 返回值 initialize | 行 124, 162 |
| [test_mechanization.py](file:///home/mxl/workplace/gipylib/tests/ins/test_mechanization.py) | 使用旧 3 返回值 initialize + InsPropagate.P1 | 行 202 |

### 命名不一致的文件（一致性修复）

| 文件 | 问题 |
|------|------|
| [ins_propagate.py](file:///home/mxl/workplace/gipylib/src/core/ins/ins_propagate.py) | `P1` 属性/参数/字段命名，docstring "P1 主滤波" |
| [trace_writer.py](file:///home/mxl/workplace/gipylib/src/log/trace_writer.py) | CSV 列名 `P1_d0..P1_d14`，参数 `P1_diag` |

### 已确认无残留的区域

- `src/core/ins/` 核心：state_index.py、lc_estimator.py、nhc.py、transfer_matrix.py、initializer.py、ins_update.py、lc_integration.py ✅
- `src/core/data_types.py` ✅
- `data/config.yaml` ✅
- 4 个核心测试：test_state_index.py、test_transfer_matrix.py、test_lc_estimator.py、test_lc_integration.py、test_nhc.py、test_ins_update.py、test_lc_e2e.py ✅

## 修复方案

### 修改 1：ins_propagate.py — P1 → P 重命名

开环协方差传播器，与 LcEstimator 共享 TransferMatrix，应统一命名。

- 行 1 docstring：`"""INS 协方差传播 (P1 主滤波)。` → `"""INS 协方差传播。`
- 行 7：`只传播 P1` → `只传播 P`
- 行 23：`维护 15x15 P1 主滤波协方差矩阵。` → `维护协方差矩阵 P。`
- 行 27：`def __init__(self, P1: np.ndarray, config: dict):` → `def __init__(self, P: np.ndarray, config: dict):`
- 行 28：`self._P1 = P1.copy()` → `self._P = P.copy()`
- 行 31-32：log 中 `P1 shape` → `P shape`
- 行 36-37：`@property def P1(self) -> np.ndarray: return self._P1` → `@property def P(self) -> np.ndarray: return self._P`
- 行 40 docstring：`P1 = Φ·(P1 + 0.5Q)·Φ^T + 0.5Q` → `P = Φ·(P + 0.5Q)·Φ^T + 0.5Q`
- 行 62-63：`self._P1` → `self._P`（2 处）
- 行 66：`self._P1 = 0.5 * (self._P1 + self._P1.T)` → `self._P = 0.5 * (self._P + self._P.T)`

### 修改 2：trace_writer.py — P1_d → P_d 重命名

机械编排 trace CSV 输出器，与 InsPropagate 配套使用。

- 行 4 docstring：`P1_diag[15]` → `P_diag[15]`
- 行 23-25 HEADER：`"P1_d0", "P1_d1", ...` → `"P_d0", "P_d1", ...`（15 个列名）
- 行 41：`def write(self, state: InsState, P1_diag: np.ndarray)` → `def write(self, state: InsState, P_diag: np.ndarray)`
- 行 52：`float(P1_diag[i])` → `float(P_diag[i])`

### 修改 3：test_mechanization.py — 迁移单滤波接口 + P1 → P

开环机械编排测试，使用 InsPropagate（非 LcEstimator）。

**接口修复（防崩溃）：**
- 行 202-203：`init_state, init_P1, _ = initializer.initialize(...)` → `init_state, init_P = initializer.initialize(...)`

**变量/参数重命名：**
- 行 78：`def log_ins_state(logger, state, P1, P2):` → `def log_ins_state(logger, state, P):`（P2 参数从未使用，已传 None，直接删除）
- 行 89：`logger.info(f"P1 trace: {np.trace(P1):.6e}")` → `logger.info(f"P trace: {np.trace(P):.6e}")`
- 行 93：`def log_epoch_summary(logger, epoch, state, P1, tag=""):` → `def log_epoch_summary(logger, epoch, state, P, tag="")`
- 行 96：`p1_trace = float(np.trace(P1))` → `p_trace = float(np.trace(P))`
- 行 100：`P1_trace={p1_trace:.4e}` → `P_trace={p_trace:.4e}`
- 行 144：`init_P1 = None` → `init_P = None`
- 行 205：`log_ins_state(logger, init_state, init_P1, None)` → `log_ins_state(logger, init_state, init_P)`
- 行 224：`ins_propagate = InsPropagate(init_P1, cfg)` → `ins_propagate = InsPropagate(init_P, cfg)`
- 行 268, 269, 285, 290, 294：`ins_propagate.P1` → `ins_propagate.P`（5 处）

**verify_trace 函数中的 CSV 列名引用：**
- 行 355：注释 `# P1_d0..P1_d14` → `# P_d0..P_d14`（列索引 10-24 不变，只改注释）
- 行 356-364：`p1_trace_list`、`p1_init`、`p1_final` → `p_trace_list`、`p_init`、`p_final`（变量名，不影响逻辑）
- 行 360-364 log 中 `P1 trace` → `P trace`

### 修改 4：diag_lc.py — 迁移单滤波接口 + P1 → P

LC EKF 诊断脚本，使用 LcEstimator。

- 行 88-89：`init_P1 = None; init_P2 = None` → `init_P = None`
- 行 124-125：`init_state, init_P1, init_P2 = initializer.initialize(...)` → `init_state, init_P = initializer.initialize(...)`
- 行 162：`est = LcEstimator(init_state, init_P1, init_P2, cfg)` → `est = LcEstimator(init_state, init_P, cfg)`
- 行 191-193：print header 列名 `P1tr` → `Ptr`（cosmetic）
- 行 211：`p1_trace = float(np.trace(est.P1))` → `p_trace = float(np.trace(est.P))`
- 行 228：`{p1_trace:>12.4e}` → `{p_trace:>12.4e}`

## 假设与决策

1. **InsPropagate.P1 → P**：虽然 InsPropagate 是开环传播器（非滤波估计器），但项目已统一单滤波架构，保留 P1 命名会造成混淆。重命名为 P 与 LcEstimator 保持一致。

2. **TraceWriter P1_d → P_d**：CSV 列名变更会破坏旧 trace 文件兼容性，但 trace 文件是临时测试输出（output/trace_mech.csv），非持久数据，可接受。

3. **test_mechanization.py 的 log_ins_state 删除 P2 参数**：P2 参数从未被使用（调用处传 None），函数体内也未引用，直接删除而非保留为 `_`。

4. **不修改 skills 文档**：与前序计划一致，skills/*.md 为设计文档，后续统一清理。

## 验证步骤

### 步骤 1：机械编排测试
```bash
cd /home/mxl/workplace/gipylib
python3 tests/ins/test_mechanization.py
```
预期：生成 output/trace_mech.csv，无接口错误，P trace 单调增长。

### 步骤 2：单元测试回归
```bash
cd /home/mxl/workplace/gipylib
python3 -m pytest tests/ins/ -v
```
预期：全部通过，无 P1/P2/gyro_scale 相关错误。

### 步骤 3：残留搜索
```bash
grep -rn "\.P1\b\|init_P1\|init_P2\|est\.P1" --include="*.py" src/ tests/
```
预期：无匹配（ins_propagate.py 的 P1 已全部改为 P）。

## 执行顺序

1. 修改 ins_propagate.py（P1 → P，基础类先改）
2. 修改 trace_writer.py（P1_d → P_d，配套类）
3. 修改 test_mechanization.py（接口修复 + P1 → P）
4. 修改 diag_lc.py（接口修复 + P1 → P）
5. 运行机械编排测试（步骤 1）
6. 运行单元测试回归（步骤 2）
7. 残留搜索确认（步骤 3）
