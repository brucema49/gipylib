# 机械编排量测间漂移调查与修复实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 消除 gipylib 在相邻 GNSS 量测更新之间由错误速度状态引起的水平机械编排漂移，并用 ignav ECEF TC 路径和同一数据集验证结果。

**Architecture:** 先通过 Qins 分段诊断把量测反馈和 IMU 机械编排解耦，再修复已证实的最小原因。机械编排、TC feedback、协方差传播和输入时间/坐标适配分别保留现有模块边界；只有诊断证明后才触碰对应模块。

**Tech Stack:** Python 3, NumPy, pytest, YAML, ignav C++ reference output

---

日期：2026-08-11

## 1. 问题定义

目标是解决 `gipylib/data/output/rtkins-tc-nf1.rslt` 中每两个 GNSS 量测更新之间，纯机械编排状态相对真值产生明显水平漂移的问题。

现象不是输出位置在相邻 IMU 历元发生了无界的离散跳变，而是结果呈现周期性锯齿：

- `Qins=3` 表示 GNSS 量测更新完成后的状态；
- 随后的 `Qins=2` 是机械编排和协方差传播状态；
- 下一次 `Qins=3` 前，`Qins=2` 的东、北误差明显增大，然后被量测更新重新拉回。

因此，调查必须把“量测反馈造成的状态变化”和“反馈后 IMU 机械编排造成的状态变化”分开测量，不能只比较最终的 `Qins=3` RMS。

## 2. 已完成的基线验证

数据：

- gipylib：`data/output/rtkins-tc-nf1.rslt`
- 真值：`data/truth.csv`
- ignav：`../ignav-debug/a-cpt/output/rtktc.rslt`
- 评估窗口：GPS week `2046`，`359000` 到 `359100` 秒

基线结果：

- gipylib 评估窗口包含 `100` 个 `Qins=3` 点和 `9900` 个 `Qins=2` 点；
- 每个量测间隔约为 `1 s`；
- gipylib 的 `Qins=3` 位置误差约为 `0.3--0.6 m`，但 `Qins=2` 的水平误差周期性上升到约 `1--1.5 m`；
- 359000 秒附近，gipylib `Qins=3` ECEF 速度约为 `[2.21, 0.54, 0.89] m/s`，真值约为 `[3.16, 1.16, 0.68] m/s`；
- 同一时段 ignav 的机械编排速度接近真值，误差图没有同幅度的水平锯齿；
- gipylib 全文件相邻输出位置最大变化为 `16.5 m`，出现在初始化阶段 `Qins=0`，不是目标窗口内的 IMU 量测间机械编排跳变。

当前最可能的主因是：量测反馈完成后的名义速度或速度相关误差状态不正确，下一秒机械编排沿错误速度传播；这比单纯的地球自转补偿误差更符合现象。地球自转时间步实现中的明确缺陷仍需修复和回归验证，但其量级预计不足以单独解释约 1 m/s 的水平速度差。

## 3. 参考实现和配置边界

### 3.1 ignav 机械编排路径

使用以下代码作为 ECEF 参考：

- `ignav-debug/src/ins-gnss/ins-gnss-tc.cc:177`：TC 路径调用 `updateins()`；
- `ignav-debug/src/ins-gnss/ins.cc:580`：ECEF 机械编排入口；
- `ignav-debug/src/ins-gnss/ins.cc:819`：锥补和划桨补偿；
- `ignav-debug/src/ins-gnss/ins.cc:1239`：旋转和划桨补偿；
- `ignav-debug/src/ins-gnss/postpos.cc:1405`：IMU 坐标、增量/速率和单位适配。

`ins-gnss.cc:1412` 的通用路径可能根据 `soltype` 选择 NED 编排，但本问题应优先对照 TC 路径实际使用的 ECEF `updateins()`，避免将坐标系差异误判为算法缺陷。

### 3.2 输入一致性

对比配置时固定以下条件：

- GPS-only：ignav `pos1-navsys=1`，gipylib 暂时不能使用默认的 GPS/GLO/GAL 混合列表；
- 100 Hz；
- 速率式 IMU：`ins-imudecfmt=1`，gipylib `imu_format=euroc`；
- 弧度角速度；
- RFU 原始坐标统一转换到 FRD；
- 同一 rover/base/RINEX/IMU 文件；
- 零杆臂、关闭 NHC/ZUPT/ZARU，先排除附加约束影响。

输入层必须确认 gipylib 的 RFU→FRD 矩阵与 ignav `Crf={0,1,0,1,0,0,0,0,-1}` 一致，并确认 EuRoC 纳秒时间戳、GPS 时间和 RINEX 观测时间只发生一次 leap-second/时间偏移修正。

## 4. 调查顺序

### 阶段 A：建立可重复诊断输出

增加一个独立诊断入口或测试辅助函数，不改变正式结果文件。每个 GNSS 更新周期记录：

- 上一次 `Qins=3` 状态；
- 量测更新前状态；
- 量测更新后状态；
- 下一次 `Qins=3` 前最后一个 `Qins=2` 状态；
- 每个 IMU 步的真实 `dt`、校正后的 `dtheta/dvel`、`f_b`、`w_b_ib`；
- 位置、速度、姿态和误差协方差的增量。

诊断结果写入带日期或场景后缀的新文件，禁止覆盖当前 `data/problem/error_plot.png` 和已有结果。

### 阶段 B：先隔离机械编排本体

从同一个量测更新后的状态开始，只回放两个 GNSS 时刻之间的 IMU，不执行 EKF 量测更新，分别比较：

1. `InsUpdate.update()` 的位置、速度、姿态；
2. ignav `updateins()` 的位置、速度、姿态；
3. 真值 PVA；
4. 机械编排前后的 ENU 速度误差和位置误差。

判断规则：

- 若在第一个 IMU 步后就出现明显速度误差，检查 IMU 坐标、单位、姿态矩阵方向和零偏；
- 若速度正常但位置异常，检查 ECEF 位置积分、时间戳和位置更新顺序；
- 若纯回放稳定，而完整 TC 周期仍出现锯齿，问题属于量测更新反馈或协方差，而非机械编排公式。

### 阶段 C：隔离量测反馈和误差状态

对量测更新前后做差，重点检查：

- `src/core/tc/tc_measurement.py` 构造的 H、v、R 是否正确使用 ECEF 位置和钟差；
- `src/core/tc/tc_estimator.py:feedback` 的位置、速度、姿态和零偏反馈符号；
- `src/core/ins/transfer_matrix.py` 的误差模型符号是否与 `InsState` 的 ψ-error 约定一致；
- `P` 的位置-速度交叉协方差是否在传播和反馈后保持合理；
- `_clk_stored`、模糊度 stored 状态和反馈清零是否改变了 INS 速度误差的统计权重；
- GNSS 更新是否因为插值或过期观测被重复推进了一个时间片。

特别关注量测更新后速度与真值的差异：若差异在 feedback 后突然产生，则优先修复 H/反馈/P；若差异在 feedback 前已存在，则回到初始化和纯机械编排路径。

### 阶段 D：修复已确认的机械编排缺陷

首个确定缺陷位于 `src/core/ins/ins_update.py:146`：姿态更新重新从 `self.state.timestamp - self._prev_timestamp` 计算地球旋转时间，但这两个值在正常更新后相等，导致该项使用 `dt=0`。应让 `_attitude_update()` 使用 `update()` 已计算的当前 `dt`，并新增测试验证单步地球旋转补偿确实按时间变化。

该修复完成后重新运行阶段 B。只有当诊断证明仍存在机械编排公式差异，才继续调整旋转/划桨补偿、重力模型、位置梯形积分或误差传播矩阵，避免用协方差参数掩盖名义状态错误。

## 5. 测试和验证

### 单元测试

补充或扩展：

- `tests/ins/test_ins_update.py`：非零 `dt` 的地球旋转、连续两步时间推进、非正时间拒绝、RFU→FRD 后的静止比力方向；
- `tests/ins/test_mechanization.py`：固定初始状态回放一个量测周期，验证状态时间戳单调、位置/速度无 NaN；
- TC 测试：量测更新前后只反馈一次，插值 IMU 不重复计时，`Qins=2/3` 时刻顺序正确；
- 必要时在 `tests/ins` 新增 ignav 对照的短序列测试，使用已固定的输入向量，不依赖大型数据文件。

### 数据验证

每次候选修复后执行：

```bash
python3 -m pytest tests/ins tests/tc -q
python3 data/plot/error-rslt.py data/output/rtkins-tc-nf1.rslt llh
```

绘图或结果生成应使用临时输出路径或先复制配置，保留旧图作为基线。验证报告必须同时给出：

- `Qins=3` 量测点 RMS；
- 每个相邻量测周期内 `Qins=2` 的 E/N/U 峰值；
- 量测更新前后速度增量的最大值；
- 机械编排回放与 ignav 的位置、速度差；
- `dt` 的最小值、最大值、均值和异常计数。

验收以“量测间 `Qins=2` 漂移显著降低并接近 ignav 基线，同时量测更新不引入新的速度突变”为准；不能只以绿色 `Qins=3` RMS 变小作为通过标准。

## 6. 可执行任务清单

### Task 1: 为地球旋转时间步建立回归测试

**Files:**
- Modify: `src/core/ins/ins_update.py:94-150`
- Test: `tests/ins/test_ins_update.py`

- [x] **Step 1: 编写失败测试**

在 `TestRotationCompensation` 中增加单步姿态测试。测试以零陀螺增量和 `dt=0.01` s 为输入，期望更新后的 DCM 包含 `-EARTH_ROTATION_RATE * dt` 的 ECEF 地球旋转：

```python
def test_earth_rotation_uses_current_dt(self):
    state = make_init_state()
    updater = InsUpdate(state)
    dt = 0.01
    updater.update(make_imu(100.0 + dt, np.zeros(3), np.zeros(3)))
    expected = rodrigues(np.array([0.0, 0.0, -EARTH_ROTATION_RATE * dt]))
    np.testing.assert_allclose(updater.state.C_b_e, expected, atol=1e-12)
```

测试文件需补充：

```python
from src.core.ins.earth_param import EARTH_ROTATION_RATE
from src.core.ins.transfer_matrix import rodrigues
```

- [x] **Step 2: 运行失败测试**

运行：`python3 -m pytest tests/ins/test_ins_update.py::TestRotationCompensation::test_earth_rotation_uses_current_dt -q`

预期：当前实现失败，因为 `_attitude_update()` 从 `self.state.timestamp - self._prev_timestamp` 得到 0，而不是使用本次 `update()` 的 `dt`。

- [x] **Step 3: 实现最小修复**

将接口改为 `_attitude_update(self, dtheta_comp, dt)`，在 `update()` 中调用 `_attitude_update(dtheta_comp, dt)`，并删除 `_attitude_update()` 内部对两个历史时间戳的重新相减。地球旋转项只使用调用方已经检查过的正 `dt`。

- [x] **Step 4: 运行 INS 回归测试**

运行：`python3 -m pytest tests/ins/test_ins_update.py -q`

预期：该文件全部通过，且新增测试验证每个 IMU 步都使用当前时间间隔。

- [ ] **Step 5: 提交独立修复**

注：本次工作区包含用户尚未提交的多文件修复，按要求不在调查过程中擅自提交；代码和测试已在当前工作区验证。

```bash
git add src/core/ins/ins_update.py tests/ins/test_ins_update.py
git commit -m "fix: use current imu dt for earth rotation compensation"
```

### Task 2: 建立量测周期诊断器并验证反馈边界

**Files:**
- Create: `tools/diagnose_tc_mechanization_drift.py`
- Test: `tests/tc/test_tc_mechanization_drift.py`
- Reference: `data/output/rtkins-tc-nf1.rslt`, `data/truth.csv`, `../ignav-debug/a-cpt/output/rtktc.rslt`

- [x] **Step 1: 定义诊断数据结构和分段规则**

诊断器读取 `.rslt` 的 GPS week、SOW、LLH、速度和 Qins 列，按 `Qins=3` 分割周期；每一段记录上一个 `Qins=3`、中间所有 `Qins=2`、下一个 `Qins=3`。真值使用 `data/truth.csv` 的 ECEF PVA 线性内插到评价时间。

- [x] **Step 2: 编写诊断器测试**

使用内存中的三条样例记录验证：周期分段只在 `Qins=3` 处切换、乱序时间被拒绝、缺少中间 `Qins=2` 时报告异常而不是静默跳过。

- [x] **Step 3: 输出反馈前后和机械编排峰值**

每个周期输出：起止时间、`Qins=3` 位置/速度误差、`Qins=2` E/N/U 最大误差、量测反馈前后速度差，以及 IMU `dt` 统计。输出到标准输出或显式指定的新文件，不写入 `data/problem/error_plot.png`。

- [x] **Step 4: 运行目标窗口诊断**

运行：

```bash
python3 tools/diagnose_tc_mechanization_drift.py \
  --rslt data/output/rtkins-tc-nf1.rslt \
  --truth data/truth.csv \
  --start-week 2046 --start-sec 359000 \
  --end-week 2046 --end-sec 359100
```

预期：诊断再次显示 `Qins=2` 水平误差在每个量测周期内增长，并给出量测更新后速度与真值的差值。该输出作为后续分支判断的唯一输入。

### Task 3: 用短序列隔离纯机械编排和 TC feedback

**Files:**
- Modify only after evidence: `src/core/tc/tc_estimator.py`, `src/core/tc/tc_measurement.py`, `src/core/ins/transfer_matrix.py`
- Test: `tests/tc/test_tc_mechanization_drift.py`

- [x] **Step 1: 固定同一个量测更新后状态**

从诊断器记录的 `Qins=3` 状态复制位置、速度、姿态、零偏和协方差，使用同一段 IMU 回放一秒，并禁止 GNSS feedback。保存每个 IMU 步的 `dt`、`f_b`、`w_b_ib` 和 PVA。

- [x] **Step 2: 判定机械编排分支**

若第一步回放就偏离 ignav/真值，检查 `src/stream/imu_sensor.py` 的 RFU→FRD、`src/stream/formators.py` 的单位和时间、`src/core/ins/ins_update.py` 的姿态/速度更新；若回放稳定，则禁止修改机械编排公式，进入 feedback 分支。

- [x] **Step 3: 判定 feedback 分支**

比较量测更新前后 `state.vel_e`、`x[si.vel:si.vel+3]`、`P[pos,vel]` 和 `P[vel,vel]`。如果错误速度在 feedback 后产生，逐项核对 `tc_measurement.py` 的 H/v/R、`TcEstimator.feedback()` 的符号以及 stored clock/ambiguity 累积；每次只改一个因素并保留前后诊断输出。

- [ ] **Step 4: 添加最小回归断言**

回归测试必须断言：一次 GNSS 事件只执行一次 `time_update(interp)` 和一次 feedback；反馈后的状态时间戳等于 GNSS 时间；下一条 IMU 的 `dt` 等于其时间戳减去该 GNSS 时间；速度反馈不出现未由 H/v/R 解释的突变。

### Task 4: 统一参考配置并完成端到端验证

**Files:**
- Temporary only: a copied YAML configuration outside the repository
- Verify: `data/rtk-ins-tc.yaml`, `../ignav-debug/a-cpt/cpt-rtktc_gps.conf`

- [x] **Step 1: 生成临时对照配置**

只在临时副本中把 gipylib 星座列表改为 GPS-only，保持 IMU、数据率、RFU、速率式输入、零杆臂和约束开关与 ignav 对齐；不直接把实验性参数写回正式 YAML。

- [ ] **Step 2: 运行完整 INS/TC 测试**

```bash
python3 -m pytest tests/ins tests/tc -q
```

预期：退出码为 0，报告中无失败测试。

- [ ] **Step 3: 重新计算目标窗口指标**

使用新的结果文件和新的图文件执行 `error-rslt.py`，比较修复前后每个量测周期的 `Qins=2` 峰值、`Qins=3` RMS 和 feedback 速度增量；同时用 ignav `rtktc.rslt` 运行同一诊断器。

- [ ] **Step 4: 保存验证结论**

将命令、退出码、指标和未解决差异写入 issue 后续记录。只有 `Qins=2` 漂移显著降低且不引入新的 feedback 速度突变，才标记修复完成。

## 7. 当前验证基线

2026-08-11 在未修改源码的工作区执行 `python3 -m pytest tests/ins tests/tc -q`：共收集 `118` 项，`117` 项通过，`1` 项失败。

失败项为 `tests/tc/test_tc_estimator.py::test_feedback_preserves_gnss_params`。当前实现的契约是：时钟修正先累积到 `TcEstimator._clk_stored`，随后 `LcEstimator.feedback()` 清零全部 `x`；该测试仍断言时钟修正保留在 `x` 中。阶段 C 必须先决定并测试这一契约，再评估它是否影响量测间速度状态；本规划阶段不把该失败误报为机械编排公式已修复或已回归通过。

2026-08-13 的后续证据：

- `InsUpdate` 单步纯机械编排从真值 PVA 回放约 1 s 后，速度误差约 `0.234 m/s`、姿态误差约 `0.51/-0.46 deg`；因此不能把所有周期性误差归因于 GNSS feedback，纯机械编排仍需继续与 ignav 逐项对照。
- baseline Qins=3 的量测反馈速度修正方向与量测前速度误差余弦中位数约 `-0.999`，平均速度误差由 `1.362` 降至 `1.039 m/s`；反馈符号不是主要原因，误差在随后机械编排间隔重新生成。
- 过程噪声单变量实验：`pos_psd=0` 时 Qins=3 速度误差中位数约 `0.297 m/s`，但 Qins=2 三维峰值增至 `3.718 m`；`vel_psd=0` 时 Qins=2 三维峰值增至 `16.803 m`。两项不能单独作为最终修复，它们当前只是互相掩盖状态模型/可观测性问题。
- 发现并修正配置契约差异：本项目 `gnss.armode=0` 时仍无条件进入 TC `_handle_ambiguity()` 入口，而 ignav `pos2-armode=off` 不执行 LAMBDA/holdamb。已新增 AR-off/AR-on 回归测试；真实数据收益待端到端回放确认。

## 8. 实施边界和提交顺序

1. 先提交本规划和诊断工具/测试；
2. 再提交已由诊断证实的最小代码修复；
3. 每个修复单独运行单元测试和目标时间窗口回放；
4. 最后更新问题记录，附上基线、修复后指标和未解决风险。

本阶段不修改已有结果数据，不回滚用户当前对 `data/problem/error_plot.png` 的修改，也不把生成的大型数据文件纳入代码提交。
