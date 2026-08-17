# 融合估计指导方案

> **单滤波**松组合 EKF：固定 15 维基础状态 + 可选参数块（GNSS 杆臂 3 / IMU 安装角 2 / IMU 杆臂 3 / 时间对齐 1），含 NHC 约束与 ZUPT 零速更新。
> 参数块/矩阵块管理采用 `StateIndex` dataclass；算法参考 ignav（H 矩阵公式、ψ-error 模型）；OOP 风格参考 GREAT-MSF-main。
>
> **设计原则**：不继承 ABC（YAGNI），`LcEstimator` / `LcIntegration` / `Constraints` 为普通类。
>
> **时间系统约定**：全框架内部统一使用 **Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）**。
>
> **当前实现状态**：
> - ✅ 已实现：GNSS 解算（`SppProcessor` / `RtkProcessor`）
> - ✅ 已实现：外部 GNSS 对齐输出（`Aligner` + `AlignedWriter`）
> - ✅ 已实现：内部 GNSS + IMU 数据对齐管线（路径 C）
> - ✅ 已实现：`InsInitializer`（三种初始化模式 + 三阈值检验）
> - ✅ 已实现：`InsUpdate`（E 系机械编排，ψ-error）
> - ✅ 已实现：`InsPropagate`（开环 P 协方差传播）+ `TransferMatrix`（F/Φ/Q，读取 `pos_psd` 等过程噪声 PSD）
> - ✅ 已实现：`LcEstimator`（单滤波 EKF，StateIndex 参数块，Joseph form）
> - ✅ 已实现：`LcIntegration`（最近邻时间对齐主循环，per-IMU 触发 + decimation）
> - ✅ 已实现：`Constraints`（NHC/ZUPT/ZARU 约束, 独立模块, 参考 ignav 分离架构）
> - ✅ 已实现：`LcRunner`（松组合批处理运行器，集成到 `main.py` 路径 C，由 `Logger` 在流式结束后调用，输出松组合 .pos）
> - ✅ 已验证：路径 C 三文件输出（RTK.pos + aligned CSV + RTKLC.pos），松组合结果与纯 GNSS 一致（planar <0.5m, elev <1m，2362 历元 100% 通过）
> - ✅ 已验证：E2E 测试通过（planar max=0.128m, elev max=0.372m, match rate 99.94%）
> - 🚧 预留：紧组合接口（`TcEstimator` / `TcIntegration`）

---

## 目录

- [1. 类设计](#1-类设计)
- [2. 状态向量与 StateIndex](#2-状态向量与-stateindex)
- [3. EKF 算法流程](#3-ekf-算法流程)
- [4. 松组合量测模型](#4-松组合量测模型)
- [5. 反馈机制](#5-反馈机制)
- [6. 时间对齐策略](#6-时间对齐策略)

---

## 1. 类设计

### 1.1 设计原则

本项目采用**普通类**（不继承 ABC），遵循 YAGNI 原则：

- **`LcEstimator`**：单滤波 EKF 估计器，持有 `InsUpdate` / `TransferMatrix` / `StateIndex`，维护协方差矩阵 P（维度 = si.dim），提供 `joseph_update()` 滤波步骤
- **`LcIntegration`**：松组合导航集成，持有 `LcEstimator` / `Constraints` / `StaticDetect`，维护 `imupre`/`imucur`/`pending_gnss`，执行最近邻时间对齐主循环
- **`Constraints`**：NHC/ZUPT/ZARU 约束更新器（独立模块, 参考 ignav ins-nhc/ins-zvu/ins-zaru 分离），持有 `Nhc` 量测构造器 + guards 配置，调用 `estimator.joseph_update()` 完成滤波

不使用 `InsKf` 基类、`OdometryStrategy` 策略层、`FusionStrategy` 层（设计阶段考虑过，实际未采用）。

### 1.2 类关系

```
LcIntegration (主循环: add_imu/add_gnss + 最近邻时间对齐)
├── LcEstimator (单滤波 EKF)
│   ├── InsUpdate (E 系机械编排)
│   ├── TransferMatrix (F/Φ/Q, 动态维度)
│   └── StateIndex (参数块索引管理)
├── Constraints (NHC/ZUPT/ZARU 约束, 独立模块)
│   └── Nhc (NHC 量测构造, 单 H 矩阵)
├── StaticDetect (静态检测: GLRT/MV/MAG/ARE/ALL)
└── pending_gnss: deque (GNSS 缓冲)
```

### 1.3 LcEstimator 接口

```python
class LcEstimator:
    def __init__(self, state: InsState, P: np.ndarray, config: dict):
        # P 维度 = si.dim (15~24)

    def time_update(self, imu: ImuMeasurement) -> None:
        # IMU 机械编排 + P 协方差传播 (GINav 中间值法)

    def meas_update_pos(self, gnss: GnssSolution) -> None:
        # GNSS 位置量测 (Joseph form)

    def meas_update_vel(self, gnss: GnssSolution) -> None:
        # GNSS 速度量测 (Joseph form)

    def joseph_update(self, Z, H, R) -> None:
        # Joseph form 量测更新 (对应 ignav filter, 供 Constraints 模块调用)

    def feedback(self) -> None:
        # 统一反馈校正 (ψ-error + 可选参数块)
```

> **NHC/ZUPT/ZARU 约束**已分离到 `src/core/ins/constraints.py` 的 `Constraints` 类，
> 通过 `estimator.joseph_update()` 调用滤波步骤，参考 ignav ins-nhc/ins-zvu/ins-zaru 分离架构。

---

## 2. 状态向量与 StateIndex

### 2.1 StateIndex 参数块管理

> `StateIndex` 结构管理参数块索引，未启用的可选块索引为 -1。

```python
@dataclass
class StateIndex:
    # 固定 15 维 (始终存在)
    pos: int = 0          # [0:3]   位置误差 δr^e
    vel: int = 3          # [3:6]   速度误差 δv^e
    att: int = 6          # [6:9]   姿态误差 δψ^e (ψ-error)
    gyro_bias: int = 9    # [9:12]  陀螺零偏 δb_g
    accel_bias: int = 12  # [12:15] 加计零偏 δb_a

    # 可选参数块 (-1 = 未启用)
    lever_arm: int = -1     # [15:18] GNSS 天线杆臂 (3 维, b 系)
    imu_angle: int = -1     # [18:20] IMU 安装角 [pitch, yaw] (2 维)
    imu_leverarm: int = -1  # [20:23] IMU 杆臂 b→v (3 维, NHC 用)
    time_sync: int = -1     # [23]    时间对齐误差 (1 维)

    dim: int = 15  # 总维数
```

### 2.2 配置开关

| 配置项 | 维度 | 说明 |
|--------|------|------|
| `estimate_leverarm` | +3 | GNSS 天线杆臂 |
| `estimate_mounting_angle` | +2 | IMU 安装角 [pitch, yaw] |
| `estimate_imu_leverarm` | +3 | IMU 杆臂（需 mounting_angle 启用） |
| `estimate_time_sync` | +1 | 时间对齐误差 |

默认配置（全 0）下 dim=15，全启用时 dim=24。

### 2.3 误差状态定义

```
固定 15 维 (E 系, ψ-error):
  δr^e    : ECEF 位置误差 (m)
  δv^e    : ECEF 速度误差 (m/s)
  δψ^e    : ECEF 姿态误差角 (rad), C_b^e_true = (I - [δψ^e×]) · C_b^e_est
  δb_g    : 陀螺零偏误差 (rad/s)
  δb_a    : 加计零偏误差 (m/s²)

可选:
  δl_gnss : GNSS 天线杆臂误差 (m, b 系)
  δθ_imu  : IMU 安装角误差 [δpitch, δyaw] (rad)
  δl_imu  : IMU 杆臂误差 (m, b 系)
  δt      : 时间对齐误差 (s)
```

**不包含比例因子误差**（已放弃）。

---

## 3. EKF 算法流程

### 3.1 预测步骤（time_update）

```
P = Φ·(P + 0.5Q)·Φ^T + 0.5Q  (GINav 中间值法)
```

时间更新先调用 `InsUpdate.update()`。只有 `last_update_accepted=True` 时才构造
`F/Phi/Q` 并传播 `P`；对非正 `dt`、小于 `1e-6 s` 或大于 `60 s` 的 IMU
间隔，名义状态和协方差都不得跨越式传播。正的异常间隔仅推进 INS 的时间
边界并清空其增量历史，使下一有效 IMU 从新的边界开始。

1. IMU 误差补偿：`dtheta_comp = dtheta - gyro_bias·dt`，`dvel_comp = dvel - accel_bias·dt`
2. INS 机械编排（E 系，参考 imu.md）
3. 构造 F 矩阵（ψ-error, E 系, 参考 ignav getF）：
   - `F_rv = I_3`
   - `F_vr = -2/(re·|pos|)·ge⊗pos`（重力梯度）
   - `F_vv = -2·[ω_ie^e×]`（Coriolis）
   - `F_vψ = -[f^e×]`（ψ-error 负号）
   - `F_vba = +C_b^e`
   - `F_ψψ = -[ω_ie^e×]`
   - `F_ψbg = +C_b^e`（ψ-error 正号）
   - 可选块 F 行/列 = 0（常数过程）
4. 离散化 Φ（自适应精度）：
   - dt ≤ 0.005s：一阶 `I + F·dt`
   - dt ≤ 0.01s：二阶 `I + F·dt + 0.5·(F·dt)²`
   - dt > 0.01s：矩阵指数 `_expm(F·dt)`（自实现 scaling-and-squaring）
5. Q 矩阵（GINav `G·Q_diag·G^T` 风格）
6. P 传播 + 对称化

### 3.2 量测更新（Joseph form）

```
S = H·P·H^T + R
K = P·H^T·S^{-1}
x = x + K·(Z - H·x)
P = (I-KH)·P·(I-KH)^T + K·R·K^T
P = 0.5·(P + P^T)
```

序贯更新：GNSS 位置 → GNSS 速度 → NHC/ZUPT，每个独立执行 Joseph form。

---

## 4. 松组合量测模型

### 4.1 量测类型

| 量测类型 | Z | H | R | 维度 |
|----------|---|---|---|------|
| GNSS 位置 | `pos_e - gnss.position` | `I(3)` + lever_arm/time_sync 项 | 固定 sigma per quality | 3 |
| GNSS 速度 | `vel_e - gnss.velocity` | `I(3)` + att/lever_arm/time_sync 项 | vel_sd 或默认 0.5 | 3 |
| NHC | `v^v[1:3]` (侧向+垂向) | 速度/姿态/陀螺零偏 + imu_angle/imu_leverarm | nhc_std² | 2 |
| ZUPT | `vel_e` (3D 速度归零) | `I(3)` 速度选择 | zupt_std² | 3 |

### 4.2 GNSS 位置量测 H 矩阵

```
H[:, pos]       = I_3
H[:, lever_arm] = -C_b_e           (若启用)
H[:, time_sync] = C_b_e·[w_b_ib×]·lever + v_e  (jacobian_p_dt, 若启用)
```

R 矩阵：固定 sigma per quality（FIX=0.02, FLOAT=0.05, DGPS=1.0, SPP=10.0）。
time_sync 未估计时，R 中加入时间偏差不确定性 `(speed·dt_offset)²/3`。

### 4.3 GNSS 速度量测 H 矩阵

```
H[:, vel]       = I_3
H[:, att]       = -[C_b_e·lever×]  (杆臂姿态贡献)
H[:, lever_arm] = [w_ie^e×]·C_b_e - C_b_e·[w_b_ib×]  (jacobian_v_dla)
H[:, time_sync] = C_b_e·[w_b_ib×]²·lever + a_e  (jacobian_v_dt)
```

### 4.4 NHC 量测（v 系, 单 H 矩阵）

```
v^v = R_b^v · C_e^b · v^e + R_b^v · [ω_eb^b×] · l_imu^b
Z = v^v[1:3]  (侧向 right + 垂向 down, 期望 0)

H[:, vel]        = (R_b^v · C_e^b)[1:3, :]
H[:, att]        = (R_b^v · C_e^b · [v^e×])[1:3, :]
H[:, gyro_bias]  = -(R_b^v · [l_imu×])[1:3, :]
H[:, imu_angle]  = [v^v×][1:3, 0:2]           (若启用)
H[:, imu_leverarm] = (R_b^v · [w_b_ib×])[1:3, :]  (若启用)
```

NHC 与 ZUPT 互斥：静止时（speed < static_speed_threshold）用 ZUPT，运动时用 NHC。

---

## 5. 反馈机制

```
位置:  r^e ← r^e - δr^e
速度:  v^e ← v^e - δv^e
姿态:  C_b^e ← (I - [δψ^e×]) · C_b^e  → SVD 正交化  (ψ-error 减号)
零偏:  b_g ← b_g + δb_g,  b_a ← b_a + δb_a  (加号)
杆臂:  l_gnss ← l_gnss + δl_gnss  (加号)
安装角: imu_angle ← imu_angle + δθ_imu  (加号)
IMU杆臂: l_imu ← l_imu - δl_imu  (减号)
时间:  time_sync ← time_sync + δt  (加号)
```

反馈后 `x` 清零。`Nhc.build_meas()` 在每次调用时自动从 `state` 刷新 `R_b^v`，无需额外调用。

---

## 6. 时间对齐策略

**最近邻匹配**（非 KF-GINS 增量切分，因参考项目均用增量式 IMU，不适用速率式 IMU）。

```
LcIntegration.add_imu(imu):
  1. 预推进: 若 pending GNSS 更近于当前 imucur, 用当前 state 做量测更新+反馈
  2. 推进: imupre←imucur, imucur←imu, time_update
  3. 推进后: 若 pending GNSS 更近于新 imucur, 做量测更新+反馈

LcIntegration.add_gnss(gnss):
  pending_gnss.append(gnss)  # 不直接触发更新
```

时间对齐误差（最大半个 IMU 采样周期 ≈ 5ms @100Hz）：
- `estimate_time_sync=1` 时作为状态 δt 在线估计
- 未启用时由 R 矩阵吸收（speed × dt_offset 项）

---

## 7. TC 估计器的实测约束与实现边界（2026-08-17）

### 7.1 TC 与 LC 必须使用不同的过程噪声策略

TC 量测直接作用于紧组合状态，RTK 双差相位/伪距或 SPP 伪距+Doppler 对状态提供的几何约束与 LC 不同。根据 `issue/8-11机械编排bug.md` 的同数据对照：

- TC 的 `pos_psd` 应配置为 `0.0`，否则位置协方差被人为持续放大，并经 `P[pos, vel]` 交叉项传递为约 `1 m/s` 的更新后速度常值偏差。
- TC 的 `vel_psd=0.5` 当前应保留；`pos_psd=0, vel_psd=0` 会使 P 过度收缩并可能触发恢复。
- LC 不得照搬 TC 的 `pos_psd=0`。高精度 LC 位置更新可能令 `P_pos` 坍缩，导致速度状态失去合理过程不确定性；LC 应保留配置文件中经过验证的非零值。

TC 时间更新仍使用：

```text
P = Phi @ (P + 0.5 * Q) @ Phi.T + 0.5 * Q
```

分块实现必须与完整矩阵传播数值等价，尤其不能丢失 `P[vel, accel_bias]`、`P[pos, vel]` 等交叉协方差。

### 7.2 RTK TC ambiguity 状态生命周期

`RtkTcMeas` 的 ambiguity 状态不是一次性初始化变量，必须跨 GNSS 历元维护：

- 复用 rtklib `udbias()` 的 LLI、Doppler 和 geometry-free 周跳检测；
- 对失锁卫星维护 outage 计数并按 `maxout` 重置；
- 每历元增加 `prnbias^2 * abs(nav.tt)` 的 ambiguity 方差；
- 按 phase-code coherency 重新初始化或修正 ambiguity；
- 保存 `nav.ph`、`nav.pt`、`nav.prev_lli`，并在历元结束清零 `nav.slip`；
- 处理 TC 的卫星主序布局与 rtklib `IB(sat, freq)` 布局之间的显式映射，禁止按数组形状直接赋值。

初始化值的单位必须是周：若 phase-code 差值为米，应除以对应波长后写入 ambiguity；量测模型中的相位项使用 `lambda * N`。

### 7.3 RTK 后验拒绝与恢复语义

TC 的 RTK 更新顺序应为：

```text
保存更新前 x/P
    -> Joseph update
    -> post-fit 4-sigma + chi-square 检查
    -> 通过：feedback
    -> 失败：恢复 x/P，丢弃本历元
```

已有 RTK 观测但 post-fit 不一致时，不应立即执行粗粒度 SPP 恢复；普通“没有有效量测”的失败才进入常规恢复计数。这样可避免一个错误的 RTK float 历元把 20--30 m 的 SPP 修正注入 INS。

### 7.4 TC 结果验收指标

不能只看 Qins=3 的位置 RMS。至少同时记录：

- Qins=3 位置、速度误差；
- 每个量测周期 Qins=2 的速度/位置峰值；
- 量测反馈速度增量；
- `k_ba_norm` 与 accel-bias feedback；
- phase/code 尝试数与接受数；
- `P[pos,pos]`、`P[vel,vel]` 及关键交叉协方差；
- 与 ignav `.stat` 的坐标系、时间和输出频率一致性。

当前最终验证表明：TC `pos_psd=0` 后 Q3 速度误差约 `0.0985 m/s`，接近 ignav INS `.stat` 的 `0.079 m/s`；Q2 速度误差中位约 `0.260 m/s`、峰值约 `0.552 m/s`。ignav 缺少可直接对应的 100 Hz 量测间速度金标准，因此不能声称两者逐历元完全一致。
