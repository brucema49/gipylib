# GNSS/INS 松组合 EKF 融合滤波器 设计

> 基于 ABC 类继承设计实现**真正双滤波**松组合 EKF：主滤波 P1（E 系 15 维）+ NHC 子滤波 P2（v 系 5 维），含 NHC 约束与 ZUPT 零速更新，闭环反馈。
> 参考 `tools/gnss_ins_lc_nhc`（双滤波架构 + NHC/ZUPT + 反馈）、`tools/ignav`（H 矩阵 + 序贯 Joseph form）、`tools/GREAT-MSF-main`（OOP 类形状）。
> 机械编排已完成（`ins_update.py`，对齐 ignav）；F/Φ/Q 矩阵已完成（`transfer_matrix.py`，ψ-error）。
>
> **验证目标**：实现后输出整数秒 ECEF 解，与 `data/rtktcgps.rslt`（ignav 在同数据上的 LC 参考解）整数秒解对比，平面误差 ≤0.5m、高程 ≤1m。

---

## 1. 范围

**本次实现（全量 LC）**：
- P1 主滤波：time_update / meas_update（GNSS 位置+速度）/ feedback
- P2 NHC 子滤波：time_update / meas_update（NHC 的 H2 部分）/ feedback
- ZUPT 量测（仅 P1，与 NHC 互斥）
- `LcEstimator` 类（滤波数学层）
- `LcIntegration` 类（时间对齐 + 主循环 + 反馈调度）
- `Nhc` 辅助模块（H1/H2 拆分 + R_b^v 维护）
- 离线 e2e 测试 runner + 验证脚本

**预留（不在本次）**：
- 紧组合（TcEstimator / TcIntegration）
- δt 时间对齐误差在线估计（P1 扩维至 16 维）
- GNSS 杆臂在线估计（estimate_gnss_leverarm，P1 扩维至 18 维）

## 2. 架构与类形状

```
src/core/ins/
├── ins_update.py          # 已有: InsUpdate (E系机械编排)
├── ins_propagate.py       # 已有: InsPropagate (P1 开环传播) → 重构进 LcEstimator
├── transfer_matrix.py     # 已有: TransferMatrix (F/Φ/Q, ψ-error) → 复用
├── initializer.py         # 已有: InsInitializer
├── lc_estimator.py        # 新增: LcEstimator (P1+P2, time_update/meas_update/feedback)
├── lc_integration.py      # 新增: LcIntegration (时间对齐+主循环+反馈调度)
└── nhc.py                 # 新增: NHC 量测构造 (H1/H2 拆分, R_b^v 维护)
```

### 2.1 LcEstimator

持有 `InsState`、`InsUpdate`、P1(15×15)/P2(5×5)、x1/x2 误差状态。**不继承 ABC**（YAGNI — 仅一个估计器，紧组合预留但不实现），但保留接口形状便于未来抽 `InsKf`。

```python
class LcEstimator:
    def __init__(self, state: InsState, P1: np.ndarray, P2: np.ndarray, config: dict):
        self.ins_update = InsUpdate(state)
        self.tm = TransferMatrix(config)
        self.P1 = P1.copy()
        self.P2 = P2.copy()
        self.x1 = np.zeros(15)
        self.x2 = np.zeros(5)
        self._nhc = Nhc(config)
        # NHC/ZUPT 阈值
        self.static_speed_threshold = config["ins"].get("static_speed_threshold", 0.5)
        self.angular_velocity_threshold = config["ins"].get("angular_velocity_threshold", 30*math.pi/180)

    def time_update(self, imu: ImuMeasurement) -> None:
        """IMU 机械编排 + P1/P2 协方差传播。"""
        # 1. InsUpdate.update(imu) → 机械编排, 更新 state
        # 2. P1: F1/Φ1/Q1 → P1 = Φ1·(P1+0.5Q1)·Φ1^T + 0.5Q1
        # 3. P2: Φ2=I → P2 = P2 + Q2·dt

    def meas_update_pos(self, gnss: GnssSolution) -> None:
        """GNSS 位置量测更新 (仅 P1, 3 维)。"""
        # Z, H1_pos, R_pos → Joseph form

    def meas_update_vel(self, gnss: GnssSolution) -> None:
        """GNSS 速度量测更新 (仅 P1, 3 维, 可选)。"""

    def meas_update_nhc(self, imu: ImuMeasurement) -> None:
        """NHC 量测更新 (P1 的 H1 部分 + P2 的 H2 部分, 2 维)。"""
        # Z_nhc, H1, H2, R_nhc → 分别对 P1/P2 做 Joseph form

    def meas_update_zupt(self) -> None:
        """ZUPT 量测更新 (仅 P1, 3 维速度约束)。"""

    def feedback(self) -> None:
        """双滤波独立反馈校正。P1: pos/vel/att(ψ+)/bias; P2: 安装角(四元数左乘)+杆臂。"""
        # 反馈后 x1 ← 0, x2 ← 0
```

### 2.2 LcIntegration

持有 `LcEstimator` + GNSS 解算结果引用。维护 `imupre`/`imucur`/`pending_gnss`。**最近邻时间对齐**（替换 KF-GINS 增量切分）。

```python
class LcIntegration:
    def __init__(self, estimator: LcEstimator, config: dict):
        self.est = estimator
        self.imupre = None
        self.imucur = None
        self.pending_gnss = collections.deque()
        self.time_align_threshold = 1e-3  # 1ms

    def add_imu(self, imu: ImuMeasurement) -> Optional[Solution]:
        """添加 IMU: imupre←imucur, imucur←imu; time_update; 检查 pending_gnss 触发量测更新。"""

    def add_gnss(self, gnss: GnssSolution) -> None:
        """添加 GNSS: append 到 pending_gnss。"""

    def _try_gnss_update(self) -> None:
        """最近邻判断: 若 imucur 距 gnss 最近, 触发 meas_update + feedback。"""
```

### 2.3 Nhc

```python
class Nhc:
    """NHC 量测构造 (v 系下, H1/H2 拆分)。"""
    def __init__(self, config: dict):
        self.imu_angle = ...  # [pitch, yaw] from InsState
        self.imu_leverarm = ...
        self.R_b_v = ...  # 由 imu_angle 构造

    def build_meas(self, state: InsState, imu: ImuMeasurement):
        """返回 (Z_nhc[2], H1[2,15], H2[2,5], R_nhc[2,2])。"""

    def feedback(self, x2: np.ndarray, state: InsState) -> None:
        """P2 反馈: 四元数左乘更新安装角, 更新 R_b_v; 杆臂修正。"""
```

## 3. 状态向量与矩阵

### 3.1 P1 主滤波（15 维，复用现有 TransferMatrix）

```
x1 = [δr^e(3), δv^e(3), δψ^e(3), δb_g(3), δb_a(3)]^T
F1: 已实现 (transfer_matrix.py, ψ-error: F_vψ=-skew(f_e), F_ψbg=+C_b^e)
Φ1: 自适应精度 (已实现: dt≤0.005 一阶, dt≤0.01 二阶, dt>0.01 _expm)
Q1: G·Q_diag·G^T (已实现, G[6:9,6:9]=+C_b^e ψ-error 正号)
P1 传播: Φ1·(P1+0.5Q1)·Φ1^T + 0.5Q1 (GINav 中间值法, 从 InsPropagate 迁移)
```

### 3.2 P2 NHC 子滤波（5 维，新增）

```
x2 = [δθ_imu(2), δl_imu(3)]^T  (pitch, yaw 安装角 + IMU 杆臂)
F2 = zeros(5,5)  (常数过程, 对齐 gnss_ins_lc_nhc)
Φ2 = I_5
Q2 = diag([σ_angle², σ_angle², σ_lever², σ_lever², σ_lever²]) · dt
     σ_angle = 1e-3 rad/sqrt(s), σ_lever = 1e-4 m/sqrt(s) (小量随机游走)
P2 传播: P2 = P2 + Q2·dt  (Φ2=I 时简化)
```

## 4. 时间对齐（最近邻）

替换 estimator.md §9.4–9.7 的 KF-GINS 增量切分。`LcIntegration` 逻辑：

```
新 IMU 到达:
  imupre ← imucur
  imucur ← 新 IMU
  → est.time_update(imucur)  (机械编排 + P1/P2 传播)
  → _try_gnss_update():
      while pending_gnss 非空:
        gnss = pending_gnss[0]
        d_cur = |imucur.t - gnss.t|
        d_pre = |imupre.t - gnss.t|
        if d_cur <= d_pre:  # imucur 更近
          # 先 time_update 已完成, 现在做量测更新
          est.meas_update_pos(gnss)
          est.meas_update_vel(gnss)  (可选)
          # NHC 或 ZUPT (互斥)
          if speed < static_speed_threshold: est.meas_update_zupt()
          elif |ω| < angular_velocity_threshold: est.meas_update_nhc(imucur)
          est.feedback()
          pending_gnss.popleft()
        else:
          break  # 等下一个 IMU (imupre 更近, 但 imupre 已被消费)

新 GNSS 到达:
  pending_gnss.append(gnss)
```

**特点**：
- 最近邻不修改原始 IMU 历元
- 时间对齐误差（≤5ms@100Hz）由 P1 速度协方差自然吸收
- δt 在线估计仍预留（不在本次实现）

## 5. 量测模型

### 5.1 GNSS 位置量测（仅 P1，3 维）

参考 ignav ins-gnss.cc:676-773 + gnss_ins_lc_nhc gpsprocess.cc:22-45。

```
Z_pos = r_ins^e - r_gnss^e + C_b^e · l_gnss^b
        (l_gnss=0 默认, estimate_gnss_leverarm=false 本次不实现)

H1_pos[3,15]: H[:, 0:3] = I_3   (位置对位置误差)

R_pos[3,3] = diag(σ²) 自适应:
  SPP:       σ = 10m
  RTD:       σ = 1m
  RTK-fix:   σ = 0.02m
  RTK-float: σ = 0.5m
  External:  使用配置默认或外部协方差
```

### 5.2 GNSS 速度量测（仅 P1，3 维，可选）

```
Z_vel = v_ins^e - v_gnss^e

H1_vel[3,15]:
  H[:, 3:6] = I_3
  H[:, 6:9] = -[C_b^e · l_gnss^b ×]   (l_gnss=0 时退化为 0)

R_vel[3,3] = diag(0.5²)  默认, 或使用 GNSS 速度协方差
```

### 5.3 NHC 量测（P1+P2 拆分，2 维）

参考 gnss_ins_lc_nhc navstate.cc:343-353。v 系下侧向/垂向速度为 0：

```
v^v = R_b^v · C_e^b · v^e + R_b^v · [ω_eb^b ×] · l_imu^b
Z_nhc = [v_right_v, v_down_v] = [0, 0]

H1[2,15] (作用 P1, ψ-error 正号):
  H1[:, VEL(3:6)]   = R_b^v · C_e^b              (= R_b^v · C_b^e^T)
  H1[:, ATT(6:9)]   = R_b^v · C_e^b · [v^e ×]    (ψ-error: 正号)
  H1[:, GYRO(9:12)] = R_b^v · [l_imu^b ×]

H2[2,5] (作用 P2):
  H2[:, ANGLE(0:2)] = [v^v ×][:, 1:3]            (skew(v^v) 第 1,2 列, 对应 pitch/yaw)
  H2[:, LEVER(2:5)] = R_b^v · [ω_eb^b ×]

R_nhc[2,2] = diag(0.1², 0.1²)
```

### 5.4 ZUPT 量测（仅 P1，3 维，与 NHC 互斥）

```
Z_zupt = v_ins^e = 0  (3D 速度约束)

H1_zupt[3,15]: H[:, 3:6] = I_3

R_zupt[3,3] = diag(0.05²)  (小噪声, 强约束)
```

### 5.5 NHC/ZUPT 切换

```
speed = |v^e|
if speed < static_speed_threshold (0.5 m/s):
    → ZUPT (仅 P1)
elif |ω_eb^b| < angular_velocity_threshold (30°/s):
    → NHC (P1 的 H1 + P2 的 H2)
else:
    → 急转弯, 两者都不用 (纯 INS 推算)
```

## 6. 量测更新与反馈

### 6.1 序贯 Joseph form

参考 ignav ins-gnss.cc:987-1062。每个量测独立执行：

```
for each measurement (pos, vel, nhc_h1, nhc_h2, zupt):
  S = H · P · H^T + R
  K = P · H^T · S^{-1}
  x = x + K · (Z - H · x)
  I_KH = I - K · H
  P = I_KH · P · I_KH^T + K · R · K^T   (Joseph form, 数值稳定)
  P = 0.5 · (P + P^T)                    (对称化)
```

P1 和 P2 各自独立执行 Joseph form（NHC 时 H1 作用 P1、H2 作用 P2）。

### 6.2 反馈

参考 estimator.md §6 + gnss_ins_lc_nhc navfilter.cc:236-270。

**P1 反馈（ψ-error）**：
```
r^e ← r^e - δr^e
v^e ← v^e - δv^e
C_b^e ← (I + [δψ^e×]) · C_b^e  → 正交化  (ψ-error: 加号)
b_g ← b_g - δb_g
b_a ← b_a - δb_a
反馈后 x1 ← 0
```

**P2 反馈**：
```
安装角: 由 [δpitch, δyaw] 构造小角度四元数 δq
         q_imu ← δq ⊗ q_imu  (四元数左乘)
         更新 R_b^v = R_b^v(新 q_imu)
IMU 杆臂: l_imu^b ← l_imu^b - δl_imu
反馈后 x2 ← 0
```

## 7. estimator.md 清理项

1. **§9.4–9.7**：删除 KF-GINS 增量切分代码（`_imu_interpolate`、4 种情况），替换为最近邻逻辑（与§9.1 一致）
2. **§5.6 vs §9.12**：§5.6 改为"不估计 δt，时间对齐误差由 P1 速度协方差吸收"；§9.12 保留为明确的"未来扩展（预留）"
3. **§11 行 18 状态**：从"🚧 预留...（下一阶段：INS 机械编排）"改为"✅ 机械编排已完成；🚧 下一阶段：EKF 融合（LcEstimator/LcIntegration）"
4. **§5.4 H2_angle 记号**：`[v^v ×]_{:,2:3}` 改为 `[v^v ×][:, 1:3]`（Python 0-indexed，对应 pitch/yaw 列），并补注释

## 8. 测试与验证计划

### 8.1 单元测试

**`tests/ins/test_lc_estimator.py`**：
- `test_h1_pos`: H1_pos 构造正确性（H[:, 0:3] = I_3）
- `test_h1_vel`: H1_vel 构造（l_gnss=0 时 H[:, 6:9] = 0）
- `test_h1_nhc_h2_nhc`: NHC 的 H1/H2 拆分（数值对比手算）
- `test_h1_zupt`: H1_zupt 构造
- `test_joseph_form`: K 增益 + Joseph form 协方差更新（对称+正定）
- `test_p1_feedback_psi_error`: P1 反馈符号（ψ-error 加号，验证 C_b^e 正交）
- `test_p2_feedback_quaternion`: P2 反馈（四元数左乘 + R_b^v 更新）
- `test_nhc_zupt_switch`: NHC/ZUPT 切换逻辑

**`tests/ins/test_lc_integration.py`**：
- `test_time_align_nearest`: 最近邻选择（imucur vs imupre）
- `test_pending_gnss_deque`: 多 GNSS 历元缓冲
- `test_full_cycle`: IMU→time_update→GNSS→meas_update→feedback 完整周期

### 8.2 集成测试（e2e）

**`tests/ins/test_lc_e2e.py`**：
- 喂 `data/cpt_imu.csv` + `data/cpt0870.19o`+base（内部 RTK 解算，复用 `InternalGnssSensor`）
- `InsInitializer` 速度矢量初始化 → `LcIntegration.add_imu/add_gnss` 主循环
- 输出整数秒 ECEF (x,y,z) + 速度到 `data/lc_output.csv`

### 8.3 验证脚本

**`tests/ins/verify_lc.py`**：
- 提取 `data/rtktcgps.rslt` 整数秒解到 `data/rtktcgps_1hz.csv`
- 读 `data/lc_output.csv` 与 `data/rtktcgps_1hz.csv`
- 按时间戳最近邻对齐（容差 1ms，超容差跳过该历元），计算 ECEF 三轴误差 → 转 ENU 平面/高程误差
- 输出 max/mean/std 误差 + 匹配历元数
- pass 判定：匹配历元数 ≥ 总历元 95%；平面 max ≤0.5m，高程 max ≤1m

## 9. 实现顺序（TDD）

1. 清理 estimator.md（§7 四项）
2. 新建 `nhc.py` + 单元测试（H1/H2/R_b^v 构造）
3. 新建 `lc_estimator.py` + 单元测试（time_update 迁移、meas_update_pos/vel、Joseph form、feedback P1+P2）
4. 新建 `lc_integration.py` + 单元测试（最近邻时间对齐、完整周期）
5. e2e 测试 runner + 验证脚本
6. 跑 e2e，对比 rtktcgps.rslt，迭代调试至满足 ≤0.5m/1m
7. 更新 project_memory.md
