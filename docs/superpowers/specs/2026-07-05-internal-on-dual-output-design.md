# internal+on 模式双定位文件 + trace 日志输出设计

> **日期**：2026-07-05
> **状态**：已批准（方案 A：最小扩展）
> **范围**：`gnss_source=internal` + `ins.enabled=on` 模式的输出文件规划

---

## 1. 背景与目标

### 1.1 当前状态

`internal+on` 模式（路径 C）当前仅输出 `aligned_internal_rtk.csv`（IMU+GNSS 对齐块状数据），用于数据对齐管线。该模式缺少：

1. **纯 GNSS 定位结果 .pos 文件**：用户需要在组合导航之外同时看到纯 GNSS 解算结果，便于对比验证
2. **组合导航定位结果 .pos 文件**：INS/GNSS 融合后的定位结果（待 `LcIntegration` 实现后填充）
3. **历元计算 trace 日志**：记录每历元的计算细节，包括 GNSS 解算和 INS 融合估计过程

### 1.2 已完成

- ✅ `gnss_solution.pos`：纯 GNSS 定位结果已实现并验证（与 `test_rtk.pos` 对比 CONSISTENT，2489/2489 历元匹配）
- ✅ `aligned_internal_rtk.csv`：对齐块状 CSV 已有

### 1.3 本设计范围

- 🚧 `ins_solution.pos`：组合导航定位结果（规划，待 `LcIntegration` 实现后开发）
- 🚧 `trace.log`：历元计算 trace 日志（规划，含 GNSS + INS 两部分）

---

## 2. 文件命名约定

`internal+on` 模式输出目录（`output/`）下的文件清单：

| 文件名 | 内容 | 格式 | 实现状态 |
|--------|------|------|---------|
| `gnss_solution.pos` | 纯 GNSS 定位结果（SPP/RTK） | rtklib .pos | ✅ 已实现 |
| `aligned_internal_rtk.csv` | IMU+GNSS 对齐块状数据 | CSV | ✅ 已实现 |
| `ins_solution.pos` | 组合导航定位结果（INS+GNSS 融合） | rtklib .pos | 🚧 规划 |
| `trace.log` | 历元计算 trace 日志（GNSS + INS） | 文本 | 🚧 规划 |

**命名规则**：
- `.pos` 文件以 `gnss_` / `ins_` 前缀区分来源
- `trace.log` 统一一个文件，内部用 `[GNSS]` / `[INS]` 标签区分
- 所有文件名可通过 config.yaml 自定义

---

## 3. ins_solution.pos 设计

### 3.1 格式

与 `gnss_solution.pos` 完全一致（rtklib .pos 格式），便于直接对比：

```
%  GPST          latitude(deg) longitude(deg)  height(m)   Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio
2046 357453.000   34.220376697  117.144021080    28.5971   2   4  22.1826    9.3695  17.0240  14.2044 -12.1777 -18.8464   0.00    0.0
...
```

### 3.2 数据流

```
LcIntegration.process_epoch()
    ↓
InsState (15-state ECEF: position/velocity/attitude/bias)
    ↓
ins_state_to_gnss_solution()  ← 新增转换函数
    ↓
GnssSolution (position/velocity/cov/quality)
    ↓
SolutionWriter.write()  ← 复用现有 writer
    ↓
ins_solution.pos
```

### 3.3 InsState → GnssSolution 转换函数

新增 `src/core/ins/state_converter.py::ins_state_to_gnss_solution(ins_state)`：

| InsState 字段 | GnssSolution 字段 | 转换说明 |
|---------------|-------------------|---------|
| `ins_state.timestamp` | `timestamp` | 直接映射 |
| `ins_state.position[0:3]` (ECEF) | `position` | 直接映射 |
| `ins_state.velocity[0:3]` (ECEF) | `velocity` | 直接映射 |
| `ins_state.P1[0:3, 0:3]` (ECEF 位置协方差) | `cov` | 提取 3×3 位置子块 |
| `ins_state.P1[3:6, 3:6]` (ECEF 速度协方差) | `vel_cov` | 提取 3×3 速度子块（可选） |
| 固定值 `5` (SOLQ_SINGLE) 或自定义 | `quality` | 组合导航解质量标记（需定义新的 quality 枚举） |
| GNSS 历元卫星数 | `num_sv` | 从 GNSS 解算结果透传 |

### 3.4 Logger 扩展

`Logger` 类已支持 `gnss_writer` 参数（本次已实现）。未来扩展 `ins_writer` 参数：

```python
class Logger(Thread):
    def __init__(self, ..., gnss_writer=None, ins_writer=None):
        ...
        self.ins_writer = ins_writer

    def run(self):
        ...
        # 组合导航完成后写入 ins_solution.pos
        if self.ins_writer is not None and ins_state is not None:
            gnss_sol = ins_state_to_gnss_solution(ins_state)
            self.ins_writer.write(gnss_sol)
```

### 3.5 实现依赖

- 🚧 `LcIntegration` 类（EKF 融合估计器，详见 [estimator.md](file:///home/mxl/workplace/gipylib/skills/estimator.md)）
- 🚧 `InsCore` 类（机械编排核心，详见 [机械编排.md](file:///home/mxl/workplace/gipylib/skills/机械编排.md)）

---

## 4. trace.log 设计

### 4.1 设计目标

trace.log 需要记录**两类**历元计算细节：

1. **GNSS 解算 trace**：卫星位置、观测残差、卡尔曼滤波状态、模糊度解算过程
2. **INS 融合估计 trace**：IMU 机械编排、EKF 时间更新/量测更新、初始化过程、状态反馈

### 4.2 统一 trace 文件方案

**单文件 + 标签区分**：所有 trace 写入同一个 `trace.log`，用前缀标签区分来源：

```
[GNSS] 2046 357453.000 estpos: n=8
[GNSS] 2046 357453.000 rescode: sat=G01 v=0.123 P=20456789.123 r=20456789.000
[GNSS] 2046 357453.000 estpos: solution converged, rr=...
[INS]  2046 357453.000 InsInitializer: static init, speed=0.155 < 0.5
[INS]  2046 357453.000 InsInitializer: attitude=[0.01, -0.02, 0.0]
[INS]  2046 357454.000 LcIntegration: time update, dt=0.01
[INS]  2046 357454.000 LcIntegration: meas update, pos_res=[0.1, -0.05, 0.2]
```

### 4.3 GNSS trace 实现

复用 rtklib-py 现有 trace 机制（`src/core/gnss/rtklib/rtkcmn.py`）：

| 函数 | 作用 |
|------|------|
| `traceopen(path)` | 打开 trace 文件 |
| `traceclose()` | 关闭 trace 文件 |
| `trace(level, fmt, ...)` | 写入 trace（level 1-5） |
| `tracelevel(level)` | 设置 trace 级别 |

**trace 级别**：
- 1 = error（仅错误）
- 2 = warning（警告 + 错误）
- 3 = info（基本信息，默认）
- 4 = debug（调试信息：残差、状态向量）
- 5 = verbose（详细：每颗卫星的观测值、协方差矩阵）

**改造点**：rtklib-py 的 trace 输出无 `[GNSS]` 前缀。需要在 `trace()` 函数中添加前缀，或在外层包装。

### 4.4 INS trace 实现

新增 `src/log/ins_trace.py` 模块，提供与 rtklib-py trace 类似的接口：

```python
# src/log/ins_trace.py
_fp = None
_level = 0

def ins_trace_open(path, level=3):
    global _fp, _level
    _fp = open(path, "a", encoding="utf-8")  # 追加模式，与 GNSS trace 共用同一文件
    _level = level

def ins_trace(level, fmt, *args):
    if _fp is None or level > _level:
        return
    msg = fmt % args if args else fmt
    _fp.write("[INS] %s\n" % msg)
    _fp.flush()

def ins_trace_close():
    if _fp is not None:
        _fp.close()
```

**INS trace 内容**（按模块）：

| 模块 | trace 内容 |
|------|-----------|
| `InsInitializer` | 初始化模式选择、阈值判断结果、姿态/速度/位置初值、协方差装配 |
| `ImuMechanizer` | 每 IMU 历元的姿态/速度/位置更新、Φ/Q 矩阵 |
| `LcIntegration` | 时间更新/量测更新、新息向量、卡尔曼增益、状态反馈 |
| `NhcFilter` | NHC 残差、自适应因子 |
| `Aligner` | reboot 检测、GNSS 中断、时间戳跳变 |

### 4.5 trace 文件生命周期

```
main.py 启动
  ↓
traceopen("output/trace.log", level=3)    ← GNSS trace 打开（写模式）
ins_trace_open("output/trace.log", level=3) ← INS trace 打开（追加模式）
  ↓
传感器线程启动 + Logger 线程启动
  ↓
[运行中] 每历元 GNSS + INS trace 写入同一文件
  ↓
Logger.join()  等待结束
  ↓
ins_trace_close()
traceclose()    ← GNSS trace 关闭
control.shutdown()
```

### 4.6 配置项

```yaml
output:
  trace_filename: "trace.log"     # trace 文件名
  trace_level: 3                   # trace 级别 1-5
```

**配置校验**（`config_loader.py`）：
- `trace_level` 必须为 1-5 的整数
- `trace_level=0` 或未配置时，不输出 trace 文件

---

## 5. config.yaml 新增字段

```yaml
output:
  output_dir: "output"                                       # 输出目录
  gnss_solution_filename: "gnss_solution.pos"                # 纯 GNSS 定位结果
  aligned_filename: "aligned_internal_rtk.csv"               # 对齐块状 CSV
  ins_solution_filename: "ins_solution.pos"                  # 组合导航定位结果（规划）
  trace_filename: "trace.log"                                # trace 日志（规划）
  trace_level: 3                                             # trace 级别 1-5（规划）
```

**各模式的输出文件矩阵**：

| 模式 | gnss_solution.pos | aligned.csv | ins_solution.pos | trace.log |
|------|:-:|:-:|:-:|:-:|
| internal + off（路径 B） | ✅ `solution.pos` | — | — | 🚧 |
| external + on（路径 A） | — | ✅ `aligned.csv` | 🚧 | 🚧 |
| internal + on（路径 C） | ✅ `gnss_solution.pos` | ✅ `aligned_internal_rtk.csv` | 🚧 | 🚧 |

---

## 6. 实现路线图

### 阶段 1：已完成 ✅

- [x] `gnss_solution.pos` 输出（Logger 增加 `gnss_writer` 参数）
- [x] main.py 路径 C 增加 SolutionWriter
- [x] 验证：与 `test_rtk.pos` 对比 CONSISTENT

### 阶段 2：trace.log 规划 🚧

- [ ] `src/log/ins_trace.py`：INS trace 模块
- [ ] rtklib-py trace 函数添加 `[GNSS]` 前缀（或外层包装）
- [ ] main.py 启动/关闭时调用 `traceopen` / `ins_trace_open`
- [ ] config_loader.py 增加 `trace_filename` / `trace_level` 校验
- [ ] 在 `InsInitializer` / `Aligner` 中埋点 ins_trace 调用

### 阶段 3：ins_solution.pos 规划 🚧

- [ ] `src/core/ins/state_converter.py`：InsState → GnssSolution 转换
- [ ] Logger 增加 `ins_writer` 参数
- [ ] main.py 路径 C 增加 ins SolutionWriter
- [ ] 依赖 `LcIntegration` 实现（详见 [estimator.md](file:///home/mxl/workplace/gipylib/skills/estimator.md)）

---

## 7. 已实现的代码变更

### 7.1 `src/log/logger.py`

`Logger` 类增加 `gnss_writer` 可选参数：
- `__init__` 接受 `gnss_writer=None`
- `run()` 中 `gnss_writer.open()` / `.write(gnss)` / `.close()`
- 每个 GNSS 历元在写入 AlignedWriter 之前，先写入 gnss_writer

### 7.2 `src/main.py`

路径 C（internal+on）增加 `gnss_solution.pos` 输出：
- 创建 `SolutionWriter(filename=gnss_solution_filename)`
- 传入 `Logger(..., gnss_writer=gnss_writer)`

### 7.3 验证结果

```
=== compare_pos: test=output/gnss_solution.pos ref=output/test_rtk.pos ===
test rows: 2489, ref rows: 2489
common epochs: 2489, only in test: 0, only in ref: 0
matched: 2489/2489
mismatched: 0/2489
=== RESULT: CONSISTENT ===
```

---

## 8. 不在本设计范围内

- `LcIntegration` EKF 融合估计器的实现（详见 [estimator.md](file:///home/mxl/workplace/gipylib/skills/estimator.md)）
- `InsCore` 机械编排核心的实现（详见 [机械编排.md](file:///home/mxl/workplace/gipylib/skills/机械编排.md)）
- NHC / ZUPT 子滤波器的实现
- 其他模式（internal+off / external+on）的 trace 输出（可后续扩展）
