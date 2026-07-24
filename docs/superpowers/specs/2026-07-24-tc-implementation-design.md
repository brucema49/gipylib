# 紧组合（TC）实现设计 — 适配 gipylib 项目

> 本文档是 `skills/紧组合.md` 的**适配版**：修正了 API 不匹配、裁剪了非必需范围、补充了分模块验证策略。
> 原始设计文档保留作为算法参考，本文档是实现权威。

## 1. 范围

### 1.1 实现（`src/core/tc/`）

| 文件 | 职责 |
|---|---|
| `__init__.py` | 模块导出 |
| `tc_state_index.py` | `TcStateIndex(StateIndex)` 扩展 clk_bias(3)/ambiguity(N) |
| `tc_estimator.py` | `TcEstimator(LcEstimator)` 扩展状态向量 + GNSS 参数反馈 |
| `tc_measurement.py` | `TcMeasurement` ABC + `SppTcMeas`/`RtkTcMeas`/`RtdTcMeas` |
| `tc_ambiguity.py` | `TcAmbiguity` 调用 `mlambda` 做固定 |
| `tc_integration.py` | `TcIntegration` GVINS 风格 IMU 消费 + TC 量测触发 + 降级 |
| `tc_degrade.py` | `TcDegradeManager` 简化版降级状态机 |
| `tc_stream.py` | `TcStream` 流式运行器（复用 `RSLTWriter`） |

### 1.2 新增流式传感器

- `src/stream/tc_gnss_sensor.py`：`TcGnssSensor` 输出原始 `(obsr, obsb, nav)`

### 1.3 复用不修改

- `src/core/ins/`：`LcEstimator`/`InsUpdate`/`InsInitializer`/`Constraints`/`StateIndex`/`TransferMatrix`/`StaticDetect`/`interpolator`
- `src/core/gnss/rtklib/`：`geodist`/`satazel`/`ecef2pos`/`ionmodel`/`tropmodel`/`tropmapf`/`satexclude`/`sat2prn`/`sat2freq`/`satposs`/`seleph`/`prange`/`varerr`/`ddres`/`mlambda`/`gettgd`/`first_obs`/`next_obs`/`rnx_decode`
- `src/log/rslt_writer.py`：`RSLTWriter`（LLH 输出，verify 脚本转 ECEF）
- `src/stream/imu_sensor.py`、`src/log/logger.py`、`src/utility/config_loader.py`

### 1.4 不做（YAGNI）

- ❌ `src/core/motion/` 约束层重构 — 紧组合直接调用现有 `Constraints`
- ❌ `src/core/robust/` 抗差估计 — 完全跳过
- ❌ `TcResiduals` 残差输出接口 — debug 用，非必需
- ❌ 松组合 `LcEstimator` 的残差接口反向移植

## 2. 关键 API 修正（相对 `skills/紧组合.md` 的差异）

### 2.1 卫星位置

```python
# 原设计（错）: rs, dts, svh = satposs(obs, nav, nav.ephover)
# 实际 (ephemeris.py:229):
rs, var, dts, svh = satposs(obs, nav)   # 返回 4 元组
# rs[i, :3] = 卫星位置, rs[i, 3:6] = 卫星速度
```

### 2.2 LAMBDA 模糊度固定

```python
# 原设计（错）: from rtkpos import resamb_LAMBDA; xa_fixed, ratio = resamb_LAMBDA(xa, Pa, n)
# 实际 (mlambda.py:145):
from src.core.gnss.rtklib.mlambda import mlambda
afix, s = mlambda(y_float, Q_float, m=2)
ratio = s[1] / s[0] if s[0] > 0 else 0.0
fixed = afix[:, 0]   # 最优整数解 (n,)
```

注：`rtkpos.resamb_lambda(nav, sats)` 操作全局 `nav.x`/`nav.P`，不适合 TC 独立状态管理，**不调用**。TC 直接用 `mlambda` 核心算法 + 自管 `ddidx` 逻辑。

### 2.3 伪距处理

```python
# SPP-INS: 复用 prange(nav, obs, i) 返回 L1 已减 TGD (pntpos.py:45)
# RTK/RTD-INS: 双频需直接读 obs.P[i,f] + 手动 TGD
for f in range(nav.nf):
    P_obs = obs.P[i, f]
    if sys != uGNSS.GLO:
        P_obs -= gettgd(obs.sat[i], eph, type=f)  # pntpos.py:36
```

### 2.4 varerr 分模式调用

```python
# SPP-INS (pntpos.py:25):
from src.core.gnss.rtklib.pntpos import varerr as spp_varerr
var = spp_varerr(nav, sys, el, nav.rcvstd[sat-1, 0])

# RTK/RTD-INS (rtkpos.py:250):
from src.core.gnss.rtklib.rtkpos import varerr as rtk_varerr
var = rtk_varerr(nav, sys, el, f, dt, rcvstd, snr_rover, snr_base)
```

## 3. 状态向量（TcStateIndex）

继承 `StateIndex`，INS 部分布局不变（pos/vel/att/gyro_bias/accel_bias + 可选块），扩展 GNSS 参数块：

```
δx = [δr^e, δv^e, δψ^e, δb_g, δb_a, 可选块, clk_bias(3), ambiguity(N)]
       0-2   3-5   6-8   9-11  12-14  ...     15+         ...
```

- **SPP-INS**：+ clk_bias(3) `[dtr, dtr_glo, dtr_gal]`
- **RTK-INS**：+ ambiguity(N)，N = 卫星对数 × 频率数（运行时动态）
- **RTD-INS**：无 GNSS 参数块（双差消除钟差，仅伪距无模糊度）

`TcStateIndex.from_config(config, mode)` 构建索引。`amb_idx(sat, freq, na)` 按 rtklib `IB(sat, freq, na)` 宏。

## 4. 量测方程

### 4.1 SPP-INS（伪距 + 可选多普勒）

- 残差：`v = P − (r + dtr − c·dts + dion + dtrp)`
- H 矩阵：`H[pos:pos+3] = LOS`，`H[clk_bias] = 1`（按卫星系统追加 GLO/GAL）
- 多普勒（可选）：`v_D = D·λ − (−LOS·(v_sat − v_rcv) + c·δḋ)`，`H[vel:vel+3] = −LOS`
- R：`varerr`（pntpos 版）对角阵

### 4.2 RTK-INS（双差载波 + 伪距）

- 双差残差（参考星 i，非参考星 j）：
  - 载波：`v_φ = DD(L·λ) − DD(r) − (λ_i·N_i − λ_j·N_j)`
  - 伪距：`v_P = DD(P) − DD(r)`
- H 矩阵：
  - `H[pos:pos+3] = LOS_i − LOS_j`
  - `H[att:att+3] = (LOS_i − LOS_j) @ skew(lever_e)`（杆臂耦合）
  - `H[amb_idx(i,f)] = λ_i`，`H[amb_idx(j,f)] = −λ_j`（仅载波）
- R：`varerr`（rtkpos 版）+ `ddcov` 双差协方差

### 4.3 RTD-INS（仅双差伪距）

- 同 RTK-INS 但只用伪距双差，**无模糊度参数**
- H 矩阵无 amb 列

### 4.4 符号约定（保留原设计 §6.4）

δx = est − true，H = LOS（非 −LOS）。原设计已自洽验证。

## 5. TcEstimator

继承 `LcEstimator`，复用 `time_update`/`joseph_update`/`feedback`：

- `__init__(state, P, config, mode)`：扩展 P/x 到 `tc_si.dim`，初始化 clk_bias P=100²、ambiguity 由 `TcAmbiguity` 管理
- `time_update(imu)` [override]：父类 + 钟差随机游走（P[clk,clk] += Q_clk·dt）
- `tc_meas_update(v, H, R, source)` [new]：调用 `joseph_update`
- `feedback()` [override]：父类 INS 反馈 + GNSS 参数反馈（钟差不反馈到 INS，模糊度由 `TcAmbiguity.apply_correction`）
- `switch_mode(new_mode, builder)` [new]：降级时状态向量重整（保留 INS+随机游走，重置 GNSS 块）
- `reboot(keep_random_walk=True)` [new]：重启保留 gyro_bias/accel_bias/lever_arm/imu_angle/imu_leverarm

杆臂四种配置（保留原设计 §6.3）：leverarm + estimate_leverarm 组合决定固定/在线估计/无杆臂。

## 6. 降级策略（简化版）

```
配置模式 (spp/rtd/rtk) → 量测构造 → 失败(卫星不足/残差过大/AR失败)
  → 降级一级 (rtk→rtd→spp→imu_only)
  → switch_mode: 切换量测构造器 + 状态向量重整
    - 保留: pos/vel/att/gyro_bias/accel_bias + 可选 lever_arm/imu_angle/imu_leverarm/time_sync
    - 重置: clk_bias(→0, P=100²), ambiguity(→清空, TcAmbiguity.reset())
  → GNSS 恢复时回配置模式 (direct 恢复, 不逐级)
  → 持续无 GNSS > reboot_threshold(30s) → reboot(keep_random_walk=True)
```

不做复杂状态机：连续失败 `fail_threshold` 次降级，GNSS 成功立即恢复。

## 7. 流水线集成（路径 D）

`src/main.py` 的 `_assemble_pipeline` 新增：

```python
if gnss_source == "internal" and ins_enabled == "tc":
    # 路径 D: 紧组合模式
    sensors = SensorFactory.create_sensors(config, imu_queue, obs_queue, control)
    # SensorFactory 需扩展: positioning_mode=="tc" 时创建 TcGnssSensor
    lc_filename = config["output"].get("tc_rslt_filename", "TC.rslt")
    lc_writer = RSLTWriter(output_dir=config["output"]["output_dir"],
                          filename=lc_filename)
    tc_mode = config["gnss"].get("tc_mode", "spp")
    from src.core.tc.tc_stream import TcStream
    tc_stream = TcStream(config, lc_writer, mode=tc_mode)
    logger = Logger(imu_queue, obs_queue, writer=None, aligner=None,
                    control=control, tc_stream=tc_stream)
    return sensors, logger
```

`Logger` 需扩展：识别 `SensorData.tag == "tc_obs"`，调用 `tc_stream.feed_obs(obsr, obsb, nav)`。

## 8. 初始化

`TcStream._try_init`：
1. 等待 IMU + 原始观测就绪（包夹条件）
2. 用首个历元 `pntpos.estpos` 做 SPP 粗定位获取初始 ECEF 位置
3. 调用 `InsInitializer.initialize(block, mode)` 获取 `InsState` + `init_P`（复用松组合初始化）
4. 创建 `TcEstimator(state, init_P, config, mode)` + `TcMeasurement` + `TcIntegration`
5. 回放缓冲

## 9. 约束处理

紧组合直接调用现有 `Constraints` 类（NHC/ZUPT/ZARU），结构与 `LcIntegration._apply_constraints` 一致：
- 静态（StaticDetect）→ ZUPT + ZARU（互斥 NHC）
- 运动 → NHC
- 约束作为伪量测调用 `tc_meas_update(v_c, H_c, R_c, source="nhc"/"zupt"/"zaru")` + `feedback()`

## 10. 分模块验证策略

按"先分模块验证再统一编程"，每模块独立 python 脚本：

| 阶段 | 脚本 | 验证内容 | 通过标准 |
|---|---|---|---|
| M1 | `tests/tc/test_tc_state_index.py` | 三模式状态维度、索引正确 | dim 匹配，amb_idx 正确 |
| M2 | `tests/tc/test_spp_tc_meas.py` | SppTcMeas 的 v/H/R 与 rtklib `pntpos.rescode` 残差对比 | v 数值一致（容差 1e-6） |
| M3 | `tests/tc/test_rtk_tc_meas.py` | RtkTcMeas/RtdTcMeas 双差 v/H 与 rtklib `rtkpos.ddres` 对比 | v 数值一致（容差 1e-6） |
| M4 | `tests/tc/test_tc_estimator.py` | TcEstimator 合成量测更新，状态收敛 | 已知解收敛，P 递减 |
| M5 | `tests/tc/test_tc_ambiguity.py` | TcAmbiguity 调 mlambda 与 rtklib `manage_amb_LAMBDA` ratio 对比 | ratio 一致 |
| M6 | 端到端 | 配置 spp/rtd/rtk 跑 `src/main.py`，输出 .rslt | SPP RMSE≤2m, RTD/RTK≤1m |

### 10.1 参考文件

- `data/rtktc.rslt`：SPP-INS 参考（4 颗卫星，Q=2 float，ECEF XYZ）→ **RMSE ≤ 2m**
- `data/rtktcgps.rslt`：RTK/RTD-INS 参考（6 颗卫星，Q=2 float，ECEF XYZ）→ **≤ 1m**
- 两者 ECEF 格式；TC 输出 LLH（复用 RSLTWriter），verify 脚本转 ECEF 对比（`verify_rslt.py` 已有逻辑）
- 硬约束：平面 ≤ 0.5m，高程 ≤ 1m（松组合标准）；TC 放宽至 SPP RMSE≤2m, RTD/RTK≤1m

### 10.2 verify 脚本

新建 `verify_tc_rslt.py`：对比 `data/output/TC.rslt`（LLH）与 `data/rtktc.rslt`/`data/rtktcgps.rslt`（ECEF），按 (week, sow) 匹配，LLH→ECEF 转换后求 ENU 误差。

## 11. 配置项新增

```yaml
gnss:
  positioning_mode: "tc"        # 新增 tc 模式
  tc_mode: "rtk"                # spp/rtd/rtk (初始模式, 降级链起点)
ins:
  enabled: "tc"                 # 新增 tc 开关 (区别于 "on" 松组合)
  tc_use_doppler: true          # SPP-INS 是否用多普勒
tc:
  degrade:
    min_sats: {rtk: 5, rtd: 4, spp: 4}
    fail_threshold: 3
    reboot_threshold: 30.0
    degrade_on_amb_fail: true
    recover_strategy: "direct"
output:
  tc_rslt_filename: "TC.rslt"
```

## 12. 与原设计文档的关系

- `skills/紧组合.md` 保留作为**算法参考**（H 矩阵推导、量测方程、参考代码映射）
- 本文档是**实现权威**：API 修正、范围裁剪、验证策略以本文档为准
- 原设计文档中涉及 `motion/`/`robust/`/`TcResiduals` 的章节本次不实现

## 13. 验收标准

1. M1-M5 单元测试全部通过
2. M6 端到端：三种模式均能输出 `data/output/TC.rslt`
3. SPP-INS：与 `data/rtktc.rslt` 对比，平面 RMSE ≤ 2m
4. RTD-INS / RTK-INS：与 `data/rtktcgps.rslt` 对比，平面误差 ≤ 1m
5. 降级链路：RTK 失败时能降级到 RTD/SPP，状态向量正确重整
