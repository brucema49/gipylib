# GNSS/INS 松组合 EKF 融合滤波器 实现计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 实现全量松组合 EKF（P1 主滤波 + P2 NHC 子滤波 + ZUPT + 闭环反馈），输出整数秒 ECEF 解与 `data/rtktcgps.rslt` 对比，平面误差 ≤0.5m、高程 ≤1m。

**Architecture:** `LcEstimator`（滤波数学层，P1+P2）+ `LcIntegration`（最近邻时间对齐 + 主循环）+ `Nhc`（H1/H2 拆分 + R_b^v 维护）。复用现有 `InsUpdate`/`TransferMatrix`/`InsInitializer`。ψ-error 模型已对齐 ignav。

**Tech Stack:** Python 3, numpy, pytest, 现有 `src/core/ins/` 模块。

## Global Constraints

- 系统默认 Python 2.7，必须用 `python3` 执行
- 所有源码放 `src/` 目录；`library/`/`tools/` 仅供参考不导入
- 时间戳统一 Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）
- ψ-error 模型：F_vψ=-skew(f_e), F_ψbg=+C_b^e, 反馈 `C_b^e ← (I+[δψ^e×])·C_b^e`（加号）
- IMU 速率式（gyro rad/s, accel m/s²），机械编排时 ×dt 转增量
- 三阈值：static_speed_threshold(0.5 m/s), dynamic_speed_threshold(4.0 m/s), angular_velocity_threshold(30°/s)
- 验证硬约束：平面位置误差 ≤0.5m（lat/lon 差 <5e-6 度），高程 ≤1m

---

## File Structure

```
src/core/ins/
├── ins_update.py          # 已有: InsUpdate (E系机械编排) — 复用
├── ins_propagate.py       # 已有: InsPropagate (P1 开环传播) — 保留供开环测试
├── transfer_matrix.py     # 已有: TransferMatrix (F/Φ/Q) — 复用
├── initializer.py         # 已有: InsInitializer — 复用
├── attitude.py            # 已有: euler2dcm/dcm2euler/dcm2quat/quat2dcm — 复用
├── earth_param.py         # 已有: ecef2llh/cal_Ce2n/gravity_ecef/georadi — 复用
├── nhc.py                 # 新增: NHC 量测构造 (R_b^v, H1, H2, Z_nhc, P2 反馈)
├── lc_estimator.py        # 新增: LcEstimator (P1+P2, time_update/meas_update/feedback)
└── lc_integration.py      # 新增: LcIntegration (最近邻时间对齐 + 主循环)

tests/ins/
├── test_nhc.py            # 新增: NHC 单元测试
├── test_lc_estimator.py   # 新增: LcEstimator 单元测试
├── test_lc_integration.py # 新增: LcIntegration 单元测试
├── test_lc_e2e.py         # 新增: e2e 测试 runner
└── verify_lc.py           # 新增: 验证脚本 (对比 rtktcgps.rslt)

skills/estimator.md        # 修改: 清理 4 处矛盾
```

---

### Task 1: 清理 estimator.md 的 4 处矛盾

**Files:**
- Modify: `skills/estimator.md`

**Interfaces:** 无（文档清理）

- [ ] **Step 1: 修复 §9.4–9.7 增量切分与§9.1 最近邻冲突**

在 `skills/estimator.md` 中，将 §9.4（`_is_to_update` 4 种情况判断）、§9.5（`_imu_interpolate` 增量切分）、§9.6（`_new_imu_process` 4 种情况处理）、§9.7（四种情况流程图）整段替换为最近邻匹配说明。保留 §9.1–§9.3 与 §9.8–§9.12。

替换内容（替换 §9.4–9.7 整段）：

```markdown
### 9.4 最近邻时间对齐（替换 KF-GINS 增量切分）

> **策略演变**：原参考 KF-GINS 的 `isToUpdate()` + `imuInterpolate()` 实现增量切分，
> 但 KF-GINS/gnss_ins_lc_nhc/GINav 三个参考项目均使用增量式 IMU（dtheta/dvel），
> 其增量切分方法不适用于本项目的速率式 IMU（gyro/accel）。
> 本项目统一采用 **GNSS 时间最近邻匹配** 策略。

```python
def _try_gnss_update(self) -> None:
    """最近邻判断: 若 imucur 距 gnss 最近, 触发 meas_update + feedback。

    当 GNSS 时间戳落在 [imupre.t, imucur.t] 之间时,
    选时间戳最近者作为 GNSS 时刻的代表性测量。
    最近邻不修改原始 IMU 历元, 时间对齐误差由 P1 速度协方差吸收。
    """
    while self.pending_gnss:
        gnss = self.pending_gnss[0]
        d_cur = abs(self.imucur.timestamp - gnss.timestamp)
        d_pre = abs(self.imupre.timestamp - gnss.timestamp)
        if d_cur <= d_pre:
            # imucur 更近: time_update 已完成, 现在做量测更新
            self.estimator.meas_update_pos(gnss)
            if gnss.velocity is not None:
                self.estimator.meas_update_vel(gnss)
            self._apply_nhc_or_zupt()
            self.estimator.feedback()
            self.pending_gnss.popleft()
        else:
            break  # imupre 更近, 但 imupre 已被消费, 等下一个 IMU
```

### 9.5 主处理流程

```python
def add_imu(self, imu: ImuMeasurement) -> Optional[Solution]:
    """添加 IMU: imupre←imucur, imucur←imu; time_update; 检查 pending_gnss。"""
    if self.imucur is not None:
        self.imupre = self.imucur
    self.imucur = imu
    self.estimator.time_update(imu)
    self._try_gnss_update()
    return self._get_solution()

def add_gnss(self, gnss: GnssSolution) -> None:
    """添加 GNSS: append 到 pending_gnss deque。"""
    self.pending_gnss.append(gnss)

def _apply_nhc_or_zupt(self) -> None:
    """NHC/ZUPT 互斥选择 (三阈值系统)。"""
    speed = float(np.linalg.norm(self.estimator.ins_update.state.vel_e))
    w_b_ib = self.estimator.ins_update.w_b_ib
    omega_norm = float(np.linalg.norm(w_b_ib))
    if speed < self.static_speed_threshold:
        self.estimator.meas_update_zupt()
    elif omega_norm < self.angular_velocity_threshold:
        self.estimator.meas_update_nhc(self.imucur)
    # else: 急转弯, 两者都不用
```

### 9.6 时间对齐流程图

```
GNSS(t1) 到达, IMU 缓冲区有 t0, t2 (t0 < t2)
─────────────────────────────────────────────────────────────
情况 A: |t1 - t2| ≤ |t1 - t0| (t2 即 imucur 更近)
  ┌─────────────────────────────────────┐
  │ 1. time_update: t0 → t2 (完整 dt)  │
  │ 2. GNSS 量测更新 (使用 t2 时刻状态) │
  │ 3. NHC/ZUPT (互斥)                  │
  │ 4. feedback                          │
  └─────────────────────────────────────┘

情况 B: |t1 - t0| < |t1 - t2| (t0 即 imupre 更近)
  ┌─────────────────────────────────────┐
  │ 1. time_update: t0 → t2 (完整 dt)  │
  │ 2. 不触发量测更新 (imupre 已消费)   │
  │ 3. 等下一个 IMU 到达后重新判断      │
  └─────────────────────────────────────┘
  ※ 若 GNSS 始终靠近已消费的 imupre, 会在下一个 IMU 周期
    作为 pending_gnss 被消费 (此时新 imucur 更近)
```
```

- [ ] **Step 2: 修复 §5.6 与 §9.12 的 δt 估计矛盾**

在 `skills/estimator.md` §5.6 末尾追加：

```markdown
注: 本项目不估计时间同步参数 δt。最近邻匹配的时间对齐误差
（最大半个 IMU 采样周期 ≈ 5ms @100Hz）由 P1 速度协方差自然吸收。
δt 在线估计作为未来扩展预留（详见 §9.12）, 不在当前实现范围。
```

- [ ] **Step 3: 修复 §11 行 18 实现状态过期**

将 `skills/estimator.md` 第 18 行：
```
> - 🚧 预留：`InsKf` / `LcEstimator` / `LcIntegration` / NHC / ZUPT / 紧组合接口（下一阶段：INS 机械编排）
```
替换为：
```
> - ✅ 已实现：`InsUpdate`（E 系机械编排，对齐 ignav ψ-error）
> - ✅ 已实现：`InsPropagate`（P1 开环协方差传播）+ `TransferMatrix`（F/Φ/Q，ψ-error）
> - 🚧 下一阶段：EKF 融合（`LcEstimator` / `LcIntegration` / `Nhc` / NHC / ZUPT / 闭环反馈）
> - 🚧 预留：紧组合接口（`TcEstimator` / `TcIntegration`）
```

- [ ] **Step 4: 修复 §5.4 H2_angle 记号模糊**

在 `skills/estimator.md` §5.4 中，将：
```
H2_angle = [v^v ×]_{:,2:3}              (速度对安装角误差 δθ_imu，仅 2 列)
```
替换为：
```
H2_angle = [v^v ×][:, 1:3]              (skew(v^v) 第 1,2 列, 对应 pitch/yaw; Python 0-indexed)
```

- [ ] **Step 5: 提交**

```bash
git add skills/estimator.md
git commit -m "docs(estimator): 清理 4 处矛盾 - 最近邻/δt/状态/H2记号"
```

---

### Task 2: 创建 NHC 模块 + 单元测试

**Files:**
- Create: `src/core/ins/nhc.py`
- Test: `tests/ins/test_nhc.py`

**Interfaces:**
- Consumes: `InsState` (from `src.core.data_types`), `ImuMeasurement`, `skew` (from `transfer_matrix`), `cal_Ce2n`/`ecef2llh` (from `earth_param`), `euler2dcm`/`quat2dcm`/`dcm2quat` (from `attitude`)
- Produces: `Nhc` class with `build_meas(state, imu) -> (Z, H1, H2, R)` and `feedback(x2, state) -> InsState`

- [ ] **Step 1: 写失败测试 `tests/ins/test_nhc.py`**

```python
#!/usr/bin/env python3
"""NHC 量测构造单元测试。"""
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import pytest

from src.core.data_types import ImuMeasurement, InsState
from src.core.ins.nhc import Nhc
from src.core.ins.attitude import euler2dcm, dcm2quat
from src.core.ins.earth_param import ecef2llh, llh2ecef, cal_Cn2e


def make_state(pos_e=None, vel_e=None, att_rpy=None, imu_angle=None,
               imu_leverarm=None):
    """构造测试用 InsState。"""
    if pos_e is None:
        pos_e = np.array([0.0, 0.0, 0.0])
    if vel_e is None:
        vel_e = np.array([10.0, 0.0, 0.0])
    if att_rpy is None:
        att_rpy = np.array([0.0, 0.0, 0.0])
    if imu_angle is None:
        imu_angle = np.array([0.0, 0.0])
    if imu_leverarm is None:
        imu_leverarm = np.array([0.0, 0.0, 0.0])
    lat, lon, _ = ecef2llh(pos_e) if np.linalg.norm(pos_e) > 1e6 else (0.0, 0.0, 0.0)
    C_b_n = euler2dcm(att_rpy)
    if np.linalg.norm(pos_e) > 1e6:
        C_b_e = cal_Cn2e(lat, lon) @ C_b_n
    else:
        C_b_e = C_b_n
    return InsState(
        timestamp=0.0, pos_e=pos_e, vel_e=vel_e, C_b_e=C_b_e,
        q_b_e=dcm2quat(C_b_e), att_rpy=att_rpy,
        gyro_bias=np.zeros(3), accel_bias=np.zeros(3),
        gyro_scale=np.zeros(3), accel_scale=np.zeros(3),
        imu_angle=imu_angle, imu_leverarm=imu_leverarm,
        leverarm=np.zeros(3),
    )


def test_r_b_v_identity_when_angle_zero():
    """安装角为 0 时 R_b^v = I。"""
    nhc = Nhc({})
    nhc.update_from_state(make_state(imu_angle=np.array([0.0, 0.0])))
    np.testing.assert_allclose(nhc.R_b_v, np.eye(3), atol=1e-12)


def test_r_b_v_pitch():
    """安装角 pitch=10° 时 R_b^v 正确。"""
    pitch = math.radians(10.0)
    nhc = Nhc({})
    nhc.update_from_state(make_state(imu_angle=np.array([pitch, 0.0])))
    # R_b^v = R_v^b^T, R_v^b 由 pitch/yaw 构造
    expected = np.array([
        [math.cos(pitch), 0, math.sin(pitch)],
        [0, 1, 0],
        [-math.sin(pitch), 0, math.cos(pitch)],
    ])
    np.testing.assert_allclose(nhc.R_b_v, expected, atol=1e-10)


def test_h1_h2_shapes():
    """H1 是 2x15, H2 是 2x5, Z 是 2, R 是 2x2。"""
    pos_e = llh2ecef(math.radians(30.0), math.radians(120.0), 50.0)
    state = make_state(pos_e=pos_e, vel_e=np.array([10.0, 0.0, 0.0]))
    imu = ImuMeasurement(timestamp=1.0, week=2046,
                         accel=np.array([0.0, 0.0, 9.8]),
                         gyro=np.array([0.0, 0.0, 0.0]))
    nhc = Nhc({})
    nhc.update_from_state(state)
    Z, H1, H2, R = nhc.build_meas(state, imu)
    assert Z.shape == (2,)
    assert H1.shape == (2, 15)
    assert H2.shape == (2, 5)
    assert R.shape == (2, 2)


def test_z_nhc_zero_when_forward_only():
    """纯前向运动 (v^v = [10, 0, 0]) 时 Z_nhc = [0, 0]。"""
    pos_e = llh2ecef(math.radians(30.0), math.radians(120.0), 50.0)
    state = make_state(pos_e=pos_e, vel_e=np.array([10.0, 0.0, 0.0]),
                       att_rpy=np.array([0.0, 0.0, 0.0]),
                       imu_angle=np.array([0.0, 0.0]))
    imu = ImuMeasurement(timestamp=1.0, week=2046,
                         accel=np.array([0.0, 0.0, 9.8]),
                         gyro=np.array([0.0, 0.0, 0.0]))
    nhc = Nhc({})
    nhc.update_from_state(state)
    Z, _, _, _ = nhc.build_meas(state, imu)
    np.testing.assert_allclose(Z, np.zeros(2), atol=1e-9)


def test_feedback_updates_imu_angle():
    """P2 反馈: 安装角四元数左乘更新。"""
    state = make_state(imu_angle=np.array([0.0, 0.0]))
    nhc = Nhc({})
    nhc.update_from_state(state)
    x2 = np.array([0.01, 0.02, 0.0, 0.0, 0.0])  # δpitch=0.01, δyaw=0.02
    new_state = nhc.feedback(x2, state)
    # 安装角应增加 (反馈: q_imu ← δq ⊗ q_imu, 然后提取 pitch/yaw)
    assert abs(new_state.imu_angle[0] - 0.01) < 1e-9
    assert abs(new_state.imu_angle[1] - 0.02) < 1e-9


def test_feedback_updates_leverarm():
    """P2 反馈: 杆臂修正。"""
    state = make_state(imu_leverarm=np.array([0.1, 0.2, 0.3]))
    nhc = Nhc({})
    nhc.update_from_state(state)
    x2 = np.array([0.0, 0.0, 0.01, 0.02, 0.03])
    new_state = nhc.feedback(x2, state)
    np.testing.assert_allclose(new_state.imu_leverarm,
                               np.array([0.09, 0.18, 0.27]), atol=1e-12)
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python3 -m pytest tests/ins/test_nhc.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'src.core.ins.nhc'`

- [ ] **Step 3: 实现 `src/core/ins/nhc.py`**

```python
"""NHC 量测构造 (v 系下, H1/H2 拆分)。

参考 gnss_ins_lc_nhc navstate.cc:343-353。
v 系 = 车体坐标系 (前-右-下), b 系 = IMU 坐标系 (FRD)。
R_b^v 由 IMU 安装角 [pitch, yaw] 构造 (roll 假设 0)。

NHC 约束: 车体侧向/垂向速度为零
  v^v = R_b^v · C_e^b · v^e + R_b^v · [ω_eb^b ×] · l_imu^b
  Z_nhc = [v_right_v, v_down_v] = [0, 0]

H 矩阵拆分 (双滤波):
  H1 (作用 P1, 15 维): 速度/姿态/陀螺零偏贡献
  H2 (作用 P2, 5 维): 安装角/杆臂贡献
"""
import math

import numpy as np

from src.core.data_types import ImuMeasurement, InsState
from src.core.ins.attitude import dcm2quat, quat2dcm
from src.core.ins.transfer_matrix import skew


def _rpy_to_dcm_v_from_b(pitch: float, yaw: float) -> np.ndarray:
    """IMU 安装角 [pitch, yaw] → 旋转矩阵 R_v^b (b→v 的转置即 R_b^v)。

    参考 gnss_ins_lc_nhc: 安装角只有 pitch/yaw (roll=0)。
    R_v^b = R_y(pitch) · R_z(yaw)  (先 yaw 后 pitch, ZYX 顺序, roll=0)

    Returns:
        R_v^b: 3x3, v 系向量 → b 系向量
    """
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    # R_z(yaw) @ R_y(pitch) @ R_x(0) = ZYX with roll=0
    return np.array([
        [cy * cp, -sy, cy * sp],
        [sy * cp,  cy, sy * sp],
        [-sp,      0,  cp     ],
    ], dtype=np.float64)


class Nhc:
    """NHC 量测构造 (v 系下, H1/H2 拆分)。

    维护 R_b^v (b→v 旋转矩阵), 由 IMU 安装角 [pitch, yaw] 构造。
    每次 P2 反馈后调用 update_from_state 刷新 R_b^v。
    """

    def __init__(self, config: dict):
        ins_cfg = config.get("ins", {}) if config else {}
        self.nhc_std = ins_cfg.get("nhc_std", 0.1)  # m/s
        # 初始 R_b^v = I (安装角为 0)
        self.R_b_v = np.eye(3, dtype=np.float64)
        self.imu_angle = np.zeros(2, dtype=np.float64)
        self.imu_leverarm = np.zeros(3, dtype=np.float64)

    def update_from_state(self, state: InsState) -> None:
        """从 InsState 刷新 R_b^v / imu_angle / imu_leverarm。"""
        self.imu_angle = state.imu_angle.copy()
        self.imu_leverarm = state.imu_leverarm.copy()
        # R_v^b 由安装角构造, R_b^v = R_v^b^T
        R_v_b = _rpy_to_dcm_v_from_b(self.imu_angle[0], self.imu_angle[1])
        self.R_b_v = R_v_b.T

    def build_meas(self, state: InsState,
                   imu: ImuMeasurement) -> tuple:
        """构造 NHC 量测。

        Returns:
            (Z[2], H1[2,15], H2[2,5], R[2,2])
        """
        self.update_from_state(state)

        C_b_e = state.C_b_e
        C_e_b = C_b_e.T
        v_e = state.vel_e
        w_b_ib = imu.gyro
        l_imu_b = self.imu_leverarm

        # v^v = R_b^v · C_e^b · v^e + R_b^v · [ω_eb^b ×] · l_imu^b
        # 取侧向(right, index 1)和垂向(down, index 2)分量
        v_v = self.R_b_v @ C_e_b @ v_e + self.R_b_v @ skew(w_b_ib) @ l_imu_b
        # Z = -v_v[1:3] (使 H·x = Z 时残差为 0)
        # 量测方程: Z = -v_v[1:3] (INS 预测的侧向/垂向速度, 期望为 0)
        Z = -v_v[1:3].copy()

        # H1 [2, 15]: 速度/姿态/陀螺零偏贡献 (ψ-error 正号)
        # H1[:, VEL(3:6)]   = (R_b^v · C_e^b)[1:3, :]      速度对 δv^e
        # H1[:, ATT(6:9)]   = (R_b^v · C_e^b · [v^e ×])[1:3, :]  速度对 δψ^e (ψ-error +)
        # H1[:, GYRO(9:12)] = (R_b^v · [l_imu^b ×])[1:3, :]  速度对 δb_g
        RbCe = self.R_b_v @ C_e_b
        H1 = np.zeros((2, 15), dtype=np.float64)
        H1[:, 3:6] = RbCe[1:3, :]
        H1[:, 6:9] = (RbCe @ skew(v_e))[1:3, :]
        H1[:, 9:12] = (self.R_b_v @ skew(l_imu_b))[1:3, :]

        # H2 [2, 5]: 安装角/杆臂贡献
        # H2[:, ANGLE(0:2)] = [v^v ×][:, 1:3]  (skew(v^v) 第 1,2 列, 对应 pitch/yaw)
        # H2[:, LEVER(2:5)] = (R_b^v · [ω_eb^b ×])[1:3, :]  速度对 δl_imu
        H2 = np.zeros((2, 5), dtype=np.float64)
        v_v_skew = skew(v_v)
        H2[:, 0:2] = v_v_skew[1:3, 1:3]
        H2[:, 2:5] = (self.R_b_v @ skew(w_b_ib))[1:3, :]

        # R [2, 2]
        R = np.diag([self.nhc_std ** 2, self.nhc_std ** 2]).astype(np.float64)

        return Z, H1, H2, R

    def feedback(self, x2: np.ndarray, state: InsState) -> InsState:
        """P2 反馈: 安装角四元数左乘 + 杆臂修正。

        Args:
            x2: [δpitch, δyaw, δlx, δly, δlz]
            state: 当前 InsState

        Returns:
            更新后的 InsState (imu_angle, imu_leverarm 修改)
        """
        delta_pitch = float(x2[0])
        delta_yaw = float(x2[1])
        delta_lever = x2[2:5].copy()

        # 安装角反馈: 直接累加 (小角度近似, 参考 gnss_ins_lc_nhc)
        new_imu_angle = np.array([
            self.imu_angle[0] + delta_pitch,
            self.imu_angle[1] + delta_yaw,
        ], dtype=np.float64)

        # 杆臂反馈: l_imu ← l_imu - δl_imu
        new_imu_leverarm = self.imu_leverarm - delta_lever

        # 返回新状态 (浅拷贝其他字段)
        import dataclasses
        new_state = dataclasses.replace(state)
        new_state.imu_angle = new_imu_angle
        new_state.imu_leverarm = new_imu_leverarm
        return new_state
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python3 -m pytest tests/ins/test_nhc.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: 提交**

```bash
git add src/core/ins/nhc.py tests/ins/test_nhc.py
git commit -m "feat(ins): NHC 量测构造模块 (H1/H2 拆分 + R_b^v 维护)"
```

---

### Task 3: 创建 LcEstimator 核心 + P1 量测更新/反馈

**Files:**
- Create: `src/core/ins/lc_estimator.py`
- Test: `tests/ins/test_lc_estimator.py`

**Interfaces:**
- Consumes: `InsUpdate` (from `ins_update`), `TransferMatrix` (from `transfer_matrix`), `Nhc` (from `nhc`), `InsState`/`ImuMeasurement`/`GnssSolution` (from `data_types`), `skew` (from `transfer_matrix`), `dcm2quat`/`quat2dcm` (from `attitude`), `ecef2llh`/`cal_Ce2n` (from `earth_param`)
- Produces: `LcEstimator` class with `time_update(imu)`, `meas_update_pos(gnss)`, `meas_update_vel(gnss)`, `meas_update_zupt()`, `meas_update_nhc(imu)`, `feedback()`, properties `state`/`P1`/`P2`/`x1`/`x2`

- [ ] **Step 1: 写失败测试 `tests/ins/test_lc_estimator.py`**

```python
#!/usr/bin/env python3
"""LcEstimator 单元测试。"""
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import pytest

from src.core.data_types import GnssSolution, ImuMeasurement, InsState
from src.core.ins.lc_estimator import LcEstimator
from src.core.ins.attitude import euler2dcm, dcm2quat
from src.core.ins.earth_param import cal_Cn2e, ecef2llh, llh2ecef


def make_state(pos_e=None, vel_e=None, att_rpy=None):
    if pos_e is None:
        pos_e = llh2ecef(math.radians(30.0), math.radians(120.0), 50.0)
    if vel_e is None:
        vel_e = np.array([10.0, 0.0, 0.0])
    if att_rpy is None:
        att_rpy = np.array([0.0, 0.0, 0.0])
    lat, lon, _ = ecef2llh(pos_e)
    C_b_n = euler2dcm(att_rpy)
    C_b_e = cal_Cn2e(lat, lon) @ C_b_n
    return InsState(
        timestamp=1.0, pos_e=pos_e, vel_e=vel_e, C_b_e=C_b_e,
        q_b_e=dcm2quat(C_b_e), att_rpy=att_rpy,
        gyro_bias=np.zeros(3), accel_bias=np.zeros(3),
        gyro_scale=np.zeros(3), accel_scale=np.zeros(3),
        imu_angle=np.zeros(2), imu_leverarm=np.zeros(3),
        leverarm=np.zeros(3),
    )


def make_config():
    return {
        "ins": {
            "corr_time_of_gyro_bias": 0.01,
            "corr_time_of_acce_bias": 0.01,
            "gyro_psd": 3.38802348178723e-09,
            "accel_psd": 2.60420170553977e-06,
            "gyro_bias_psd": 2.61160339323310e-14,
            "acce_bias_psd": 1.66067346797506e-09,
            "static_speed_threshold": 0.5,
            "angular_velocity_threshold": 30.0 * math.pi / 180.0,
            "nhc_std": 0.1,
            "zupt_std": 0.05,
        }
    }


def test_init_shapes():
    """LcEstimator 初始化: P1 15x15, P2 5x5, x1 15, x2 5。"""
    state = make_state()
    P1 = np.eye(15) * 0.01
    P2 = np.eye(5) * 0.01
    est = LcEstimator(state, P1, P2, make_config())
    assert est.P1.shape == (15, 15)
    assert est.P2.shape == (5, 5)
    assert est.x1.shape == (15,)
    assert est.x2.shape == (5,)
    np.testing.assert_allclose(est.x1, np.zeros(15))
    np.testing.assert_allclose(est.x2, np.zeros(5))


def test_time_update_propagates_P1_P2():
    """time_update: P1 trace 增长, P2 trace 增长。"""
    state = make_state()
    P1 = np.eye(15) * 0.01
    P2 = np.eye(5) * 0.01
    est = LcEstimator(state, P1, P2, make_config())
    imu = ImuMeasurement(timestamp=1.01, week=2046,
                         accel=np.array([0.0, 0.0, 9.8]),
                         gyro=np.array([0.0, 0.0, 0.01]))
    trace1_before = np.trace(est.P1)
    trace2_before = np.trace(est.P2)
    est.time_update(imu)
    assert np.trace(est.P1) > trace1_before
    assert np.trace(est.P2) > trace2_before


def test_meas_update_pos_reduces_P1_trace():
    """GNSS 位置量测更新: P1 trace 减小。"""
    state = make_state()
    P1 = np.eye(15) * 10.0
    P2 = np.eye(5) * 0.01
    est = LcEstimator(state, P1, P2, make_config())
    # GNSS 位置 = INS 位置 (无误差) → 量测残差为 0, P1 减小
    gnss = GnssSolution(
        timestamp=1.0, week=2046, position=state.pos_e.copy(),
        quality=1, num_sv=8, sd=np.array([0.02, 0.02, 0.02]),
    )
    trace_before = np.trace(est.P1)
    est.meas_update_pos(gnss)
    assert np.trace(est.P1) < trace_before


def test_meas_update_pos_joseph_symmetric():
    """Joseph form 后 P1 对称。"""
    state = make_state()
    P1 = np.eye(15) * 10.0
    P2 = np.eye(5) * 0.01
    est = LcEstimator(state, P1, P2, make_config())
    gnss = GnssSolution(
        timestamp=1.0, week=2046,
        position=state.pos_e + np.array([1.0, 0.5, -0.3]),
        quality=1, num_sv=8, sd=np.array([0.02, 0.02, 0.02]),
    )
    est.meas_update_pos(gnss)
    np.testing.assert_allclose(est.P1, est.P1.T, atol=1e-10)


def test_meas_update_zupt_reduces_vel_variance():
    """ZUPT 量测更新: 速度方差 (P1[3:6, 3:6]) 减小。"""
    state = make_state(vel_e=np.array([0.0, 0.0, 0.0]))
    P1 = np.eye(15) * 1.0
    P2 = np.eye(5) * 0.01
    est = LcEstimator(state, P1, P2, make_config())
    vel_var_before = np.trace(est.P1[3:6, 3:6])
    est.meas_update_zupt()
    vel_var_after = np.trace(est.P1[3:6, 3:6])
    assert vel_var_after < vel_var_before


def test_feedback_p1_psi_error_corrects_state():
    """P1 反馈: 位置/速度修正, 姿态 ψ-error 加号更新, C_b^e 正交。"""
    state = make_state()
    P1 = np.eye(15) * 0.01
    P2 = np.eye(5) * 0.01
    est = LcEstimator(state, P1, P2, make_config())
    # 注入误差: δr=1m, δv=0.1m/s, δψ=0.01rad
    est.x1[0:3] = np.array([1.0, 0.0, 0.0])
    est.x1[3:6] = np.array([0.1, 0.0, 0.0])
    est.x1[6:9] = np.array([0.0, 0.01, 0.0])
    pos_before = est.state.pos_e.copy()
    est.feedback()
    # 位置应减小 (r ← r - δr)
    assert est.state.pos_e[0] < pos_before[0]
    # x1 清零
    np.testing.assert_allclose(est.x1, np.zeros(15))
    # C_b^e 正交 (C @ C^T ≈ I)
    np.testing.assert_allclose(est.state.C_b_e @ est.state.C_b_e.T,
                               np.eye(3), atol=1e-9)


def test_feedback_p2_updates_imu_angle():
    """P2 反馈: 安装角/杆臂修正, x2 清零。"""
    state = make_state()
    P1 = np.eye(15) * 0.01
    P2 = np.eye(5) * 0.01
    est = LcEstimator(state, P1, P2, make_config())
    est.x2[0:2] = np.array([0.01, 0.02])
    est.x2[2:5] = np.array([0.001, 0.002, 0.003])
    est.feedback()
    assert abs(est.state.imu_angle[0] - 0.01) < 1e-9
    assert abs(est.state.imu_angle[1] - 0.02) < 1e-9
    np.testing.assert_allclose(est.x2, np.zeros(5))
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python3 -m pytest tests/ins/test_lc_estimator.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'src.core.ins.lc_estimator'`

- [ ] **Step 3: 实现 `src/core/ins/lc_estimator.py`**

```python
"""松组合 EKF 估计器 (P1 主滤波 + P2 NHC 子滤波)。

参考:
- gnss_ins_lc_nhc navfilter.cc (TimeUpdate/MeasureUpdate/ReviseState)
- ignav ins-gnss.cc (H 矩阵 + 序贯 Joseph form)
- GINav ins_time_updata.m (中间值法 P 传播)

P1: 15 维 E 系 [δr^e, δv^e, δψ^e, δb_g, δb_a] (ψ-error)
P2: 5 维 v 系 [δθ_imu(2), δl_imu(3)] (NHC 子滤波)
"""
import logging
import math

import numpy as np

from src.core.data_types import GnssSolution, ImuMeasurement, InsState
from src.core.ins.attitude import dcm2euler, dcm2quat
from src.core.ins.earth_param import cal_Ce2n, ecef2llh
from src.core.ins.ins_update import InsUpdate
from src.core.ins.nhc import Nhc
from src.core.ins.transfer_matrix import TransferMatrix, skew

logger = logging.getLogger(__name__)


class LcEstimator:
    """松组合 EKF 估计器 (双滤波 P1 + P2)。

    P1 主滤波: 15 维 E 系, 复用 TransferMatrix (F/Φ/Q, ψ-error)
    P2 NHC 子滤波: 5 维 v 系, F2=0 (常数过程)
    """

    def __init__(self, state: InsState, P1: np.ndarray, P2: np.ndarray,
                 config: dict):
        self.ins_update = InsUpdate(state)
        self.tm = TransferMatrix(config)
        self._nhc = Nhc(config)
        self.P1 = P1.copy().astype(np.float64)
        self.P2 = P2.copy().astype(np.float64)
        self.x1 = np.zeros(15, dtype=np.float64)
        self.x2 = np.zeros(5, dtype=np.float64)

        ins_cfg = config.get("ins", {})
        self.static_speed_threshold = ins_cfg.get("static_speed_threshold", 0.5)
        self.angular_velocity_threshold = ins_cfg.get(
            "angular_velocity_threshold", 30.0 * math.pi / 180.0)
        self.zupt_std = ins_cfg.get("zupt_std", 0.05)
        # GNSS 量测噪声默认 (按 quality 自适应)
        self._gnss_pos_std = {
            1: 10.0,   # SPP
            2: 1.0,    # RTD
            4: 1.0,    # DGPS
            5: 0.02,   # RTK fix (LC mode in rtklib)
            0: 0.5,    # RTK float / unknown
        }
        self._gnss_vel_std = ins_cfg.get("gnss_vel_std", 0.5)

    @property
    def state(self) -> InsState:
        return self.ins_update.state

    @property
    def nhc(self) -> Nhc:
        return self._nhc

    # ===== 时间更新 =====

    def time_update(self, imu: ImuMeasurement) -> None:
        """IMU 机械编排 + P1/P2 协方差传播。

        P1: Φ1·(P1+0.5Q1)·Φ1^T + 0.5Q1 (GINav 中间值法)
        P2: P2 + Q2·dt (Φ2=I, F2=0)
        """
        # 1. 机械编排 (更新 state, f_b, w_b_ib)
        self.ins_update.update(imu)

        # 2. P1 传播 (复用 InsPropagate 逻辑)
        dt = imu.timestamp - self.ins_update._prev_timestamp
        if dt <= 0.0:
            return
        C_b_e = self.ins_update.state.C_b_e
        f_b = self.ins_update.f_b
        w_b_ib = self.ins_update.w_b_ib
        pos_e = self.ins_update.state.pos_e
        F = self.tm.build_F(C_b_e, f_b, w_b_ib, pos_e)
        Phi = self.tm.build_Phi(F, dt)
        Q1 = self.tm.build_Q(dt, C_b_e)
        P0 = self.P1 + 0.5 * Q1
        self.P1 = Phi @ P0 @ Phi.T + 0.5 * Q1
        self.P1 = 0.5 * (self.P1 + self.P1.T)

        # 3. P2 传播 (F2=0, Φ2=I)
        Q2 = self._build_Q2(dt)
        self.P2 = self.P2 + Q2
        self.P2 = 0.5 * (self.P2 + self.P2.T)

    def _build_Q2(self, dt: float) -> np.ndarray:
        """P2 过程噪声 (5x5, 小量随机游走)。"""
        # σ_angle = 1e-3 rad/sqrt(s), σ_lever = 1e-4 m/sqrt(s)
        sigma_angle = 1e-3
        sigma_lever = 1e-4
        q = np.array([sigma_angle ** 2, sigma_angle ** 2,
                      sigma_lever ** 2, sigma_lever ** 2, sigma_lever ** 2])
        return np.diag(q * dt).astype(np.float64)

    # ===== 量测更新 =====

    def meas_update_pos(self, gnss: GnssSolution) -> None:
        """GNSS 位置量测更新 (仅 P1, 3 维, Joseph form)。"""
        state = self.ins_update.state
        # Z = r_ins^e - r_gnss^e (+ 杆臂, 本次 leverarm=0)
        Z = state.pos_e - gnss.position
        # H1[:, 0:3] = I_3
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 0:3] = np.eye(3)
        # R 自适应
        sigma = self._gnss_pos_std.get(gnss.quality, 0.5)
        # 若 gnss.sd 有效则用 sd
        if gnss.sd is not None and np.all(gnss.sd > 0):
            R = np.diag(gnss.sd ** 2).astype(np.float64)
        else:
            R = np.diag([sigma ** 2] * 3).astype(np.float64)
        self._joseph_update_P1(Z, H, R)

    def meas_update_vel(self, gnss: GnssSolution) -> None:
        """GNSS 速度量测更新 (仅 P1, 3 维, Joseph form)。"""
        if gnss.velocity is None:
            return
        state = self.ins_update.state
        Z = state.vel_e - gnss.velocity
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 3:6] = np.eye(3)
        # 杆臂旋转补偿项 H[:, 6:9] = -[C_b^e · l_gnss^b ×] (l_gnss=0 时为 0)
        if np.linalg.norm(state.leverarm) > 1e-9:
            H[:, 6:9] = -skew(state.C_b_e @ state.leverarm)
        if gnss.vel_sd is not None and np.all(gnss.vel_sd > 0):
            R = np.diag(gnss.vel_sd ** 2).astype(np.float64)
        else:
            R = np.diag([self._gnss_vel_std ** 2] * 3).astype(np.float64)
        self._joseph_update_P1(Z, H, R)

    def meas_update_zupt(self) -> None:
        """ZUPT 量测更新 (仅 P1, 3 维速度约束)。"""
        state = self.ins_update.state
        Z = state.vel_e.copy()  # 期望为 0
        H = np.zeros((3, 15), dtype=np.float64)
        H[:, 3:6] = np.eye(3)
        R = np.diag([self.zupt_std ** 2] * 3).astype(np.float64)
        self._joseph_update_P1(Z, H, R)

    def meas_update_nhc(self, imu: ImuMeasurement) -> None:
        """NHC 量测更新 (P1 的 H1 部分 + P2 的 H2 部分, 2 维)。"""
        Z, H1, H2, R_nhc = self._nhc.build_meas(self.ins_update.state, imu)
        # P1 更新 (H1)
        self._joseph_update_P1(Z, H1, R_nhc)
        # P2 更新 (H2)
        self._joseph_update_P2(Z, H2, R_nhc)

    def _joseph_update_P1(self, Z: np.ndarray, H: np.ndarray,
                          R: np.ndarray) -> None:
        """P1 Joseph form 量测更新。"""
        S = H @ self.P1 @ H.T + R
        K = self.P1 @ H.T @ np.linalg.inv(S)
        innov = Z - H @ self.x1
        self.x1 = self.x1 + K @ innov
        I_KH = np.eye(15) - K @ H
        self.P1 = I_KH @ self.P1 @ I_KH.T + K @ R @ K.T
        self.P1 = 0.5 * (self.P1 + self.P1.T)

    def _joseph_update_P2(self, Z: np.ndarray, H: np.ndarray,
                          R: np.ndarray) -> None:
        """P2 Joseph form 量测更新。"""
        S = H @ self.P2 @ H.T + R
        K = self.P2 @ H.T @ np.linalg.inv(S)
        innov = Z - H @ self.x2
        self.x2 = self.x2 + K @ innov
        I_KH = np.eye(5) - K @ H
        self.P2 = I_KH @ self.P2 @ I_KH.T + K @ R @ K.T
        self.P2 = 0.5 * (self.P2 + self.P2.T)

    # ===== 反馈 =====

    def feedback(self) -> None:
        """双滤波独立反馈校正。P1: pos/vel/att(ψ+)/bias; P2: 安装角+杆臂。"""
        self._feedback_P1()
        self._feedback_P2()
        self.x1[:] = 0.0
        self.x2[:] = 0.0

    def _feedback_P1(self) -> None:
        """P1 反馈 (ψ-error: 姿态加号)。"""
        import dataclasses
        state = self.ins_update.state
        delta_pos = self.x1[0:3]
        delta_vel = self.x1[3:6]
        delta_psi = self.x1[6:9]
        delta_bg = self.x1[9:12]
        delta_ba = self.x1[12:15]

        new_pos = state.pos_e - delta_pos
        new_vel = state.vel_e - delta_vel
        # ψ-error: C_b^e ← (I + [δψ×]) · C_b^e
        C_b_e_new = (np.eye(3) + skew(delta_psi)) @ state.C_b_e
        # 正交化 (Gram-Schmidt: SVD 投影到最近正交矩阵)
        U, _, Vt = np.linalg.svd(C_b_e_new)
        C_b_e_new = U @ Vt
        # 提取欧拉角 (需 C_b^n)
        lat, lon, _ = ecef2llh(new_pos)
        C_e_n = cal_Ce2n(lat, lon)
        C_b_n_new = C_e_n @ C_b_e_new
        from src.core.ins.attitude import dcm2euler
        att_rpy = dcm2euler(C_b_n_new)

        new_state = dataclasses.replace(state)
        new_state.pos_e = new_pos
        new_state.vel_e = new_vel
        new_state.C_b_e = C_b_e_new
        new_state.q_b_e = dcm2quat(C_b_e_new)
        new_state.att_rpy = att_rpy
        new_state.gyro_bias = state.gyro_bias - delta_bg
        new_state.accel_bias = state.accel_bias - delta_ba
        self.ins_update.state = new_state
        # InsUpdate 内部 _prev_timestamp 不变, _prev_dtheta/dvel 不变
        # (反馈不影响增量历史, 仅影响下一历元的零偏/姿态)

    def _feedback_P2(self) -> None:
        """P2 反馈: 安装角 + 杆臂。"""
        new_state = self._nhc.feedback(self.x2, self.ins_update.state)
        self.ins_update.state = new_state
        self._nhc.update_from_state(new_state)
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python3 -m pytest tests/ins/test_lc_estimator.py -v`
Expected: PASS (7 tests)

- [ ] **Step 5: 提交**

```bash
git add src/core/ins/lc_estimator.py tests/ins/test_lc_estimator.py
git commit -m "feat(ins): LcEstimator 双滤波 EKF (P1+P2, Joseph form, ψ-error 反馈)"
```

---

### Task 4: 创建 LcIntegration + 单元测试

**Files:**
- Create: `src/core/ins/lc_integration.py`
- Test: `tests/ins/test_lc_integration.py`

**Interfaces:**
- Consumes: `LcEstimator` (from `lc_estimator`), `ImuMeasurement`/`GnssSolution` (from `data_types`)
- Produces: `LcIntegration` class with `add_imu(imu)`, `add_gnss(gnss)`, properties `estimator`/`initialized`

- [ ] **Step 1: 写失败测试 `tests/ins/test_lc_integration.py`**

```python
#!/usr/bin/env python3
"""LcIntegration 单元测试。"""
import math
import sys
from collections import deque
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import pytest

from src.core.data_types import GnssSolution, ImuMeasurement, InsState
from src.core.ins.lc_estimator import LcEstimator
from src.core.ins.lc_integration import LcIntegration
from src.core.ins.attitude import euler2dcm, dcm2quat
from src.core.ins.earth_param import cal_Cn2e, ecef2llh, llh2ecef


def make_state(pos_e=None, vel_e=None, att_rpy=None, timestamp=1.0):
    if pos_e is None:
        pos_e = llh2ecef(math.radians(30.0), math.radians(120.0), 50.0)
    if vel_e is None:
        vel_e = np.array([10.0, 0.0, 0.0])
    if att_rpy is None:
        att_rpy = np.array([0.0, 0.0, 0.0])
    lat, lon, _ = ecef2llh(pos_e)
    C_b_n = euler2dcm(att_rpy)
    C_b_e = cal_Cn2e(lat, lon) @ C_b_n
    return InsState(
        timestamp=timestamp, pos_e=pos_e, vel_e=vel_e, C_b_e=C_b_e,
        q_b_e=dcm2quat(C_b_e), att_rpy=att_rpy,
        gyro_bias=np.zeros(3), accel_bias=np.zeros(3),
        gyro_scale=np.zeros(3), accel_scale=np.zeros(3),
        imu_angle=np.zeros(2), imu_leverarm=np.zeros(3),
        leverarm=np.zeros(3),
    )


def make_config():
    return {
        "ins": {
            "corr_time_of_gyro_bias": 0.01,
            "corr_time_of_acce_bias": 0.01,
            "gyro_psd": 3.38802348178723e-09,
            "accel_psd": 2.60420170553977e-06,
            "gyro_bias_psd": 2.61160339323310e-14,
            "acce_bias_psd": 1.66067346797506e-09,
            "static_speed_threshold": 0.5,
            "angular_velocity_threshold": 30.0 * math.pi / 180.0,
            "nhc_std": 0.1,
            "zupt_std": 0.05,
        }
    }


def test_init_empty_buffers():
    """LcIntegration 初始化: imupre/imucur=None, pending_gnss 空。"""
    state = make_state()
    est = LcEstimator(state, np.eye(15) * 0.01, np.eye(5) * 0.01, make_config())
    integ = LcIntegration(est, make_config())
    assert integ.imupre is None
    assert integ.imucur is None
    assert len(integ.pending_gnss) == 0


def test_add_imu_first_sets_imucur_only():
    """第一个 IMU: imucur 设置, imupre 仍 None, 不触发 time_update。"""
    state = make_state(timestamp=1.0)
    est = LcEstimator(state, np.eye(15) * 0.01, np.eye(5) * 0.01, make_config())
    integ = LcIntegration(est, make_config())
    imu = ImuMeasurement(timestamp=1.01, week=2046,
                         accel=np.array([0.0, 0.0, 9.8]),
                         gyro=np.array([0.0, 0.0, 0.0]))
    integ.add_imu(imu)
    assert integ.imucur is imu
    assert integ.imupre is None


def test_add_imu_second_triggers_time_update():
    """第二个 IMU: imupre←imucur, imucur←新, 触发 time_update。"""
    state = make_state(timestamp=1.0)
    est = LcEstimator(state, np.eye(15) * 0.01, np.eye(5) * 0.01, make_config())
    integ = LcIntegration(est, make_config())
    imu1 = ImuMeasurement(timestamp=1.01, week=2046,
                          accel=np.array([0.0, 0.0, 9.8]),
                          gyro=np.array([0.0, 0.0, 0.0]))
    imu2 = ImuMeasurement(timestamp=1.02, week=2046,
                          accel=np.array([0.0, 0.0, 9.8]),
                          gyro=np.array([0.0, 0.0, 0.0]))
    integ.add_imu(imu1)
    trace_before = np.trace(est.P1)
    integ.add_imu(imu2)
    assert integ.imupre is imu1
    assert integ.imucur is imu2
    assert np.trace(est.P1) > trace_before  # P1 传播


def test_add_gnss_buffers_in_deque():
    """add_gnss: append 到 pending_gnss。"""
    state = make_state()
    est = LcEstimator(state, np.eye(15) * 0.01, np.eye(5) * 0.01, make_config())
    integ = LcIntegration(est, make_config())
    gnss = GnssSolution(timestamp=1.5, week=2046,
                        position=state.pos_e.copy(),
                        quality=1, num_sv=8, sd=np.array([0.02, 0.02, 0.02]))
    integ.add_gnss(gnss)
    assert len(integ.pending_gnss) == 1


def test_nearest_neighbor_imucur_triggers_update():
    """GNSS 靠近 imucur: 触发量测更新 + feedback, pending_gnss 弹出。"""
    state = make_state(timestamp=1.0)
    est = LcEstimator(state, np.eye(15) * 10.0, np.eye(5) * 0.01, make_config())
    integ = LcIntegration(est, make_config())
    imu1 = ImuMeasurement(timestamp=1.00, week=2046,
                          accel=np.array([0.0, 0.0, 9.8]),
                          gyro=np.array([0.0, 0.0, 0.0]))
    imu2 = ImuMeasurement(timestamp=1.01, week=2046,
                          accel=np.array([0.0, 0.0, 9.8]),
                          gyro=np.array([0.0, 0.0, 0.0]))
    integ.add_imu(imu1)
    # GNSS 靠近 imu2 (t=1.009)
    gnss = GnssSolution(timestamp=1.009, week=2046,
                        position=state.pos_e.copy(),
                        quality=1, num_sv=8, sd=np.array([0.02, 0.02, 0.02]))
    integ.add_gnss(gnss)
    trace_before = np.trace(est.P1)
    integ.add_imu(imu2)  # 触发 time_update + gnss update
    assert len(integ.pending_gnss) == 0  # 已弹出
    assert np.trace(est.P1) < trace_before  # 量测更新减小 P1


def test_nearest_neighbor_imupre_waits():
    """GNSS 靠近 imupre (已消费): 不触发, 留在 deque。"""
    state = make_state(timestamp=1.0)
    est = LcEstimator(state, np.eye(15) * 10.0, np.eye(5) * 0.01, make_config())
    integ = LcIntegration(est, make_config())
    imu1 = ImuMeasurement(timestamp=1.00, week=2046,
                          accel=np.array([0.0, 0.0, 9.8]),
                          gyro=np.array([0.0, 0.0, 0.0]))
    imu2 = ImuMeasurement(timestamp=1.10, week=2046,
                          accel=np.array([0.0, 0.0, 9.8]),
                          gyro=np.array([0.0, 0.0, 0.0]))
    integ.add_imu(imu1)
    # GNSS 靠近 imu1 (t=1.001, imupre)
    gnss = GnssSolution(timestamp=1.001, week=2046,
                        position=state.pos_e.copy(),
                        quality=1, num_sv=8, sd=np.array([0.02, 0.02, 0.02]))
    integ.add_gnss(gnss)
    integ.add_imu(imu2)  # time_update, 但 GNSS 靠近 imupre, 不触发
    # GNSS 仍在 deque (等下一个 IMU, 那时新 imucur 可能更近)
    assert len(integ.pending_gnss) == 1
```

- [ ] **Step 2: 运行测试验证失败**

Run: `python3 -m pytest tests/ins/test_lc_integration.py -v`
Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: 实现 `src/core/ins/lc_integration.py`**

```python
"""松组合导航集成 (最近邻时间对齐 + 主循环)。

参考:
- KF-GINS newImuProcess (imupre/imucur/pending_gnss 结构)
- 项目约定: 最近邻匹配 (替换 KF-GINS 增量切分, 适配速率式 IMU)

主循环:
  add_imu: imupre←imucur, imucur←imu; time_update; _try_gnss_update
  add_gnss: append 到 pending_gnss
  _try_gnss_update: 最近邻判断, imucur 更近则 meas_update + feedback
"""
import collections
import logging
import math
from typing import Optional

import numpy as np

from src.core.data_types import GnssSolution, ImuMeasurement

logger = logging.getLogger(__name__)


class LcIntegration:
    """松组合导航集成 (最近邻时间对齐 + 双滤波 EKF 主循环)。"""

    def __init__(self, estimator, config: dict):
        self.est = estimator
        ins_cfg = config.get("ins", {})
        self.static_speed_threshold = ins_cfg.get("static_speed_threshold", 0.5)
        self.angular_velocity_threshold = ins_cfg.get(
            "angular_velocity_threshold", 30.0 * math.pi / 180.0)
        self.time_align_threshold = ins_cfg.get("time_align_threshold", 1e-3)

        self.imupre: Optional[ImuMeasurement] = None
        self.imucur: Optional[ImuMeasurement] = None
        self.pending_gnss: collections.deque = collections.deque()

    def add_imu(self, imu: ImuMeasurement) -> None:
        """添加 IMU: 更新 imupre/imucur, time_update, 尝试 GNSS 量测更新。"""
        if self.imucur is not None:
            self.imupre = self.imucur
        self.imucur = imu
        # 第一个 IMU 不触发 time_update (无 imupre)
        if self.imupre is None:
            return
        self.est.time_update(imu)
        self._try_gnss_update()

    def add_gnss(self, gnss: GnssSolution) -> None:
        """添加 GNSS: append 到 pending_gnss deque。"""
        self.pending_gnss.append(gnss)

    def _try_gnss_update(self) -> None:
        """最近邻判断: 若 imucur 距 gnss 最近, 触发量测更新 + feedback。"""
        while self.pending_gnss:
            gnss = self.pending_gnss[0]
            d_cur = abs(self.imucur.timestamp - gnss.timestamp)
            d_pre = abs(self.imupre.timestamp - gnss.timestamp)
            if d_cur <= d_pre:
                # imucur 更近: 触发量测更新
                self.est.meas_update_pos(gnss)
                if gnss.velocity is not None:
                    self.est.meas_update_vel(gnss)
                self._apply_nhc_or_zupt()
                self.est.feedback()
                self.pending_gnss.popleft()
            else:
                # imupre 更近, 但 imupre 已被消费, 等下一个 IMU
                # 若 GNSS 时间戳远小于 imupre (过时数据), 丢弃
                if gnss.timestamp < self.imupre.timestamp - 1.0:
                    logger.warning(
                        f"丢弃过时 GNSS 历元 t={gnss.timestamp:.6f} "
                        f"(imupre={self.imupre.timestamp:.6f})"
                    )
                    self.pending_gnss.popleft()
                    continue
                break

    def _apply_nhc_or_zupt(self) -> None:
        """NHC/ZUPT 互斥选择 (三阈值系统)。"""
        speed = float(np.linalg.norm(self.est.state.vel_e))
        w_b_ib = self.est.ins_update.w_b_ib
        omega_norm = float(np.linalg.norm(w_b_ib))
        if speed < self.static_speed_threshold:
            self.est.meas_update_zupt()
        elif omega_norm < self.angular_velocity_threshold:
            self.est.meas_update_nhc(self.imucur)
        # else: 急转弯, 两者都不用
```

- [ ] **Step 4: 运行测试验证通过**

Run: `python3 -m pytest tests/ins/test_lc_integration.py -v`
Expected: PASS (6 tests)

- [ ] **Step 5: 提交**

```bash
git add src/core/ins/lc_integration.py tests/ins/test_lc_integration.py
git commit -m "feat(ins): LcIntegration 最近邻时间对齐 + 主循环"
```

---

### Task 5: 创建 e2e 测试 runner + 验证脚本

**Files:**
- Create: `tests/ins/test_lc_e2e.py`
- Create: `tests/ins/verify_lc.py`

**Interfaces:**
- Consumes: `LcIntegration`, `LcEstimator`, `InsInitializer`, `InternalGnssSensor`, `ImuSensor`
- Produces: `data/lc_output.csv` (整数秒 ECEF 解), `data/rtktcgps_1hz.csv` (参考解), 验证报告

- [ ] **Step 1: 实现 `tests/ins/test_lc_e2e.py`**

```python
#!/usr/bin/env python3
"""LC EKF 端到端测试: 喂 cpt_imu.csv + RTK, 输出整数秒 ECEF 解。

流程:
1. InternalGnssSensor (RTK) + ImuSensor 读取数据
2. InsInitializer 速度矢量初始化
3. LcIntegration 主循环: add_imu/add_gnss, 双滤波 EKF
4. 输出整数秒 ECEF (x,y,z,vx,vy,vz) 到 data/lc_output.csv

用法: python3 tests/ins/test_lc_e2e.py
"""
import csv
import logging
import math
import sys
from collections import deque
from pathlib import Path
from queue import Queue

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import yaml

from src.core.thread_control import ThreadControl
from src.core.data_types import AlignedBlock
from src.core.ins.initializer import InsInitializer, InitMode
from src.core.ins.lc_estimator import LcEstimator
from src.core.ins.lc_integration import LcIntegration
from src.stream.imu_sensor import ImuSensor
from src.stream.internal_gnss_sensor import InternalGnssSensor


def setup_logging():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(message)s",
        datefmt="%H:%M:%S",
    )
    return logging.getLogger("test_lc_e2e")


def load_config():
    cfg_path = PROJECT_ROOT / "data" / "cfg_test_rtk.yaml"
    with open(cfg_path, "r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    cfg["ins"]["enabled"] = "on"
    cfg["ins"]["imu_data_path"] = "data/cpt_imu.csv"
    cfg["ins"]["imu_coordinate_system"] = "RFU"
    cfg["ins"]["data_rate"] = 100
    cfg["ins"]["static_speed_threshold"] = 0.5
    cfg["ins"]["dynamic_speed_threshold"] = 4.0
    cfg["ins"]["angular_velocity_threshold"] = 30.0 * math.pi / 180.0
    cfg["ins"]["gnss_buffer_size"] = 3
    cfg["ins"]["static_duration"] = 10.0
    cfg["ins"]["initial_pos_std_si"] = [30.0, 30.0, 30.0]
    cfg["ins"]["initial_vel_std_si"] = [10.0, 10.0, 10.0]
    cfg["ins"]["initial_att_std_si"] = [0.00524, 0.00524, 0.00524]
    cfg["ins"]["gyro_bias_std_si"] = [2.424e-5, 2.424e-5, 2.424e-5]
    cfg["ins"]["acce_bias_std_si"] = [0.0489, 0.0489, 0.0489]
    cfg["ins"]["nhc_std"] = 0.1
    cfg["ins"]["zupt_std"] = 0.05
    return cfg


def main():
    logger = setup_logging()
    logger.info("启动 LC EKF 端到端测试")
    cfg = load_config()
    data_rate = cfg["ins"]["data_rate"]

    control = ThreadControl()
    imu_queue = Queue()
    gnss_queue = Queue()

    imu_sensor = ImuSensor(
        cfg["ins"]["imu_data_path"], imu_queue, control,
        cfg["ins"].get("imu_coordinate_system", "FRD"),
    )
    gnss_sensor = InternalGnssSensor(cfg, gnss_queue, control)

    initializer = InsInitializer(cfg)

    imu_sensor.start()
    gnss_sensor.start()

    output_path = PROJECT_ROOT / "data" / "lc_output.csv"

    try:
        # 阶段 1: 初始化 (速度矢量)
        initialized = False
        imu_buffer = deque(maxlen=data_rate * 3600)
        init_state = None
        init_P1 = None
        init_P2 = None
        gnss_count = 0

        logger.info("阶段 1: 等待 GNSS 速度超阈值初始化...")
        while control.is_running() and not initialized:
            while True:
                try:
                    data = imu_queue.get_nowait()
                except Exception:
                    break
                if data is None:
                    break
                if data.imu is not None:
                    imu_buffer.append(data.imu)
            try:
                data = gnss_queue.get(timeout=0.1)
            except Exception:
                continue
            if data is None:
                break
            if data.gnss_solution is None:
                continue
            gnss_count += 1
            gnss = data.gnss_solution
            if gnss.velocity is None:
                continue
            speed = np.linalg.norm(gnss.velocity)
            if speed < cfg["ins"]["dynamic_speed_threshold"]:
                continue
            t_gnss = gnss.timestamp
            imu_list = [imu for imu in imu_buffer
                        if t_gnss - 2.0 <= imu.timestamp <= t_gnss + 1.0]
            if len(imu_list) < 2:
                continue
            has_before = any(imu.timestamp <= t_gnss for imu in imu_list)
            has_after = any(imu.timestamp >= t_gnss for imu in imu_list)
            if not (has_before and has_after):
                continue
            gyro_norms = [float(np.linalg.norm(imu.gyro)) for imu in imu_list]
            if float(np.mean(gyro_norms)) >= cfg["ins"]["angular_velocity_threshold"]:
                continue
            block = AlignedBlock(gnss=gnss, imu_list=imu_list)
            try:
                init_state, init_P1, init_P2 = initializer.initialize(
                    block, InitMode.VELOCITY_VECTOR)
                logger.info(f"初始化成功: t={init_state.timestamp:.3f}, "
                            f"speed={speed:.3f} m/s")
                initialized = True
            except ValueError as e:
                continue

        if not initialized:
            logger.error("初始化失败")
            control.shutdown()
            imu_sensor.join(timeout=2.0)
            gnss_sensor.join(timeout=5.0)
            return

        # 阶段 2: LC EKF 主循环
        logger.info("阶段 2: LC EKF 主循环")
        est = LcEstimator(init_state, init_P1, init_P2, cfg)
        integ = LcIntegration(est, cfg)

        # 排空 IMU 队列
        while True:
            try:
                data = imu_queue.get_nowait()
            except Exception:
                break
            if data is None:
                break
            if data.imu is not None:
                imu_buffer.append(data.imu)

        # 收集所有 IMU + GNSS 数据, 按时间排序
        all_imus = [imu for imu in imu_buffer if imu.timestamp > init_state.timestamp]
        # 继续从队列读 GNSS
        all_gnss = []
        while True:
            try:
                data = gnss_queue.get(timeout=0.5)
            except Exception:
                break
            if data is None:
                break
            if data.gnss_solution is not None:
                all_gnss.append(data.gnss_solution)

        logger.info(f"收集: IMU={len(all_imus)}, GNSS={len(all_gnss)}")

        # 按时间排序合并 (IMU 和 GNSS 交错)
        events = [(imu.timestamp, "imu", imu) for imu in all_imus]
        events += [(gnss.timestamp, "gnss", gnss) for gnss in all_gnss
                   if gnss.timestamp > init_state.timestamp]
        events.sort(key=lambda e: e[0])

        # 输出 CSV
        with open(output_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["week", "sec", "x", "y", "z", "vx", "vy", "vz"])
            epoch = 0
            last_output_sec = -1
            for t, tag, data in events:
                if tag == "imu":
                    integ.add_imu(data)
                else:
                    integ.add_gnss(data)
                # 输出整数秒解
                state = est.state
                # GPST week + sec (sec = timestamp - week_start_unix)
                from src.core.time_utils import unix_to_gpst
                week, sec = unix_to_gpst(state.timestamp)
                if abs(sec - round(sec)) < 0.005:  # 整数秒 ±5ms
                    int_sec = round(sec)
                    if int_sec != last_output_sec:
                        writer.writerow([
                            week, f"{int_sec}.000",
                            f"{state.pos_e[0]:.4f}", f"{state.pos_e[1]:.4f}",
                            f"{state.pos_e[2]:.4f}",
                            f"{state.vel_e[0]:.6f}", f"{state.vel_e[1]:.6f}",
                            f"{state.vel_e[2]:.6f}",
                        ])
                        last_output_sec = int_sec
                epoch += 1
                if epoch % 10000 == 0:
                    logger.info(f"epoch={epoch}, t={t:.3f}, "
                                f"speed={np.linalg.norm(state.vel_e):.3f}")

        logger.info(f"输出: {output_path}")
        control.shutdown()
        imu_sensor.join(timeout=2.0)
        gnss_sensor.join(timeout=5.0)

    except Exception as e:
        logger.exception(f"测试异常: {e}")
        control.shutdown()
        imu_sensor.join(timeout=2.0)
        gnss_sensor.join(timeout=5.0)
        raise


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: 实现 `tests/ins/verify_lc.py`**

```python
#!/usr/bin/env python3
"""验证脚本: 对比 lc_output.csv 与 rtktcgps.rslt 整数秒解。

提取 rtktcgps.rslt 整数秒解到 rtktcgps_1hz.csv,
按时间戳最近邻对齐 lc_output.csv, 计算 ENU 平面/高程误差。
pass 判定: 平面 max ≤0.5m, 高程 max ≤1m。

用法: python3 tests/ins/verify_lc.py
"""
import csv
import math
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np

from src.core.ins.earth_param import cal_Ce2n, ecef2llh


def parse_rtktcgps(path: Path):
    """解析 rtktcgps.rslt, 提取整数秒解。

    格式: week sec x y z Q Qins ns sdx sdy sdz sdxy sdyz sdzx age ratio
          vx vy vz sdvx sdvy sdvz sdvxy sdvyz sdvzx
    """
    ref = {}
    with open(path, "r") as f:
        for line in f:
            if line.startswith("%") or not line.strip():
                continue
            parts = line.split()
            if len(parts) < 22:
                continue
            week = int(parts[0])
            sec = float(parts[1])
            x, y, z = float(parts[2]), float(parts[3]), float(parts[4])
            qins = int(parts[6])
            # 只取 Qins=3 (ins-gnss LC) 的整数秒
            if qins != 3:
                continue
            if abs(sec - round(sec)) > 0.005:
                continue
            int_sec = round(sec)
            key = (week, int_sec)
            ref[key] = np.array([x, y, z])
    return ref


def parse_lc_output(path: Path):
    """解析 lc_output.csv。"""
    out = {}
    with open(path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            week = int(row["week"])
            sec = float(row["sec"])
            x, y, z = float(row["x"]), float(row["y"]), float(row["z"])
            int_sec = round(sec)
            key = (week, int_sec)
            out[key] = np.array([x, y, z])
    return out


def main():
    rtk_path = PROJECT_ROOT / "data" / "rtktcgps.rslt"
    lc_path = PROJECT_ROOT / "data" / "lc_output.csv"

    if not lc_path.exists():
        print(f"FAIL: {lc_path} 不存在, 先运行 test_lc_e2e.py")
        sys.exit(1)

    ref = parse_rtktcgps(rtk_path)
    out = parse_lc_output(lc_path)
    print(f"参考解 (rtktcgps Qins=3 整数秒): {len(ref)} 历元")
    print(f"LC 输出 (整数秒): {len(out)} 历元")

    # 对齐
    common = set(ref.keys()) & set(out.keys())
    print(f"匹配历元: {len(common)}")
    if not common:
        print("FAIL: 无匹配历元")
        sys.exit(1)

    # 取第一个匹配历元的位置作为 ENU 原点
    keys = sorted(common)
    ref0 = ref[keys[0]]
    lat0, lon0, _ = ecef2llh(ref0)
    C_e_n = cal_Ce2n(lat0, lon0)

    e_errors = []
    n_errors = []
    u_errors = []
    for k in keys:
        d_ecef = out[k] - ref[k]
        d_enu = C_e_n @ d_ecef
        e_errors.append(d_enu[1])  # East
        n_errors.append(d_enu[0])  # North
        u_errors.append(-d_enu[2])  # Up (NED 的 Down 取反)

    e_errors = np.array(e_errors)
    n_errors = np.array(n_errors)
    u_errors = np.array(u_errors)
    horiz = np.sqrt(e_errors ** 2 + n_errors ** 2)

    print(f"\n=== 误差统计 (m) ===")
    print(f"东向 E: max={np.max(np.abs(e_errors)):.4f}, "
          f"mean={np.mean(np.abs(e_errors)):.4f}, std={np.std(e_errors):.4f}")
    print(f"北向 N: max={np.max(np.abs(n_errors)):.4f}, "
          f"mean={np.mean(np.abs(n_errors)):.4f}, std={np.std(n_errors):.4f}")
    print(f"高程 U: max={np.max(np.abs(u_errors)):.4f}, "
          f"mean={np.mean(np.abs(u_errors)):.4f}, std={np.std(u_errors):.4f}")
    print(f"平面  : max={np.max(horiz):.4f}, "
          f"mean={np.mean(horiz):.4f}, std={np.std(horiz):.4f}")

    match_ratio = len(common) / max(len(ref), 1)
    horiz_pass = np.max(horiz) <= 0.5
    vert_pass = np.max(np.abs(u_errors)) <= 1.0
    match_pass = match_ratio >= 0.95

    print(f"\n=== 判定 ===")
    print(f"匹配率: {match_ratio*100:.1f}% (≥95% {'PASS' if match_pass else 'FAIL'})")
    print(f"平面 max: {np.max(horiz):.4f}m (≤0.5m {'PASS' if horiz_pass else 'FAIL'})")
    print(f"高程 max: {np.max(np.abs(u_errors)):.4f}m (≤1.0m {'PASS' if vert_pass else 'FAIL'})")

    if horiz_pass and vert_pass and match_pass:
        print("\n>>> 整体验证: PASS <<<")
        sys.exit(0)
    else:
        print("\n>>> 整体验证: FAIL <<<")
        sys.exit(1)


if __name__ == "__main__":
    main()
```

- [ ] **Step 3: 检查 time_utils.py 是否有 unix_to_gpst**

Run: `python3 -c "from src.core.time_utils import unix_to_gpst; print(unix_to_gpst(1e9))"`
Expected: 输出 `(week, sec)` 元组。若失败, 实现 `unix_to_gpst`。

- [ ] **Step 4: 提交**

```bash
git add tests/ins/test_lc_e2e.py tests/ins/verify_lc.py
git commit -m "test(ins): LC EKF e2e runner + 验证脚本 (对比 rtktcgps.rslt)"
```

---

### Task 6: 运行全部单元测试 + e2e + 验证

**Files:** 无（运行验证）

- [ ] **Step 1: 运行全部单元测试**

Run: `python3 -m pytest tests/ins/ -v --tb=short`
Expected: 全部 PASS（test_nhc 6 + test_lc_estimator 7 + test_lc_integration 6 = 19 tests）

- [ ] **Step 2: 运行 e2e 测试**

Run: `python3 tests/ins/test_lc_e2e.py`
Expected: 输出 `data/lc_output.csv`，无异常，整数秒解数量 > 0

- [ ] **Step 3: 运行验证脚本**

Run: `python3 tests/ins/verify_lc.py`
Expected: 平面 max ≤0.5m, 高程 max ≤1m, 匹配率 ≥95%

- [ ] **Step 4: 若失败, 调试**

常见问题排查：
- **平面误差大**: 检查 H1_pos 的 R 矩阵是否过大（GNSS sd 过大导致不修正）；检查 feedback 符号（ψ-error 加号）
- **姿态跳变**: 检查 SVD 正交化是否正确；检查 dcm2euler 输入是否为 C_b^n（非 C_b^e）
- **P1 爆炸**: 检查 Q 矩阵量级；检查 Joseph form 实现
- **匹配率低**: 检查整数秒提取逻辑（rtktcgps Qins=3 筛选）

- [ ] **Step 5: 提交最终结果**

```bash
git add -A
git commit -m "test(ins): LC EKF 验证通过 (平面≤0.5m, 高程≤1m)"
```

---

### Task 7: 更新 project_memory.md

**Files:**
- Modify: `/home/mxl/.trae-cn/memory/projects/-home-mxl-workplace-gipylib/project_memory.md`

- [ ] **Step 1: 追加 LC EKF 实现记录**

在 `## Engineering Conventions` 末尾追加：

```markdown
- LcEstimator 实现双滤波 EKF: P1(15维 E系 ψ-error) + P2(5维 v系 NHC), Joseph form 量测更新, 全闭环反馈
- LcIntegration 使用最近邻时间对齐 (替换 KF-GINS 增量切分), imupre/imucur/pending_gnss deque 结构
- NHC H 矩阵拆分: H1[2,15] 作用 P1 (速度/姿态/陀螺零偏), H2[2,5] 作用 P2 (安装角/杆臂)
- NHC/ZUPT 互斥: speed<0.5→ZUPT(仅P1), speed≥0.5且|ω|<30°/s→NHC(P1+P2), 急转弯两者都不用
- P2 反馈: 安装角直接累加 (小角度近似), 杆臂 l_imu ← l_imu - δl_imu
- 验证: 对比 data/rtktcgps.rslt (ignav LC 参考解) 整数秒 ECEF, 平面≤0.5m 高程≤1m
```

- [ ] **Step 2: 提交**

```bash
git add /home/mxl/.trae-cn/memory/projects/-home-mxl-workplace-gipylib/project_memory.md
git commit -m "docs(memory): 追加 LC EKF 实现记录"
```

---

## Self-Review

**1. Spec coverage:**
- §1 范围（全量 LC）→ Tasks 2-5 ✅
- §2 类形状（LcEstimator/LcIntegration/Nhc）→ Tasks 2-4 ✅
- §3 状态向量与矩阵（P1 复用, P2 新增）→ Task 3 ✅
- §4 时间对齐（最近邻）→ Task 4 ✅
- §5 量测模型（GNSS pos/vel, NHC H1/H2, ZUPT）→ Tasks 2-3 ✅
- §6 序贯 Joseph form + 双滤波反馈 → Task 3 ✅
- §7 estimator.md 清理 → Task 1 ✅
- §8 测试与验证 → Tasks 2-6 ✅

**2. Placeholder scan:** 无 TBD/TODO，所有代码块完整。

**3. Type consistency:** `LcEstimator.time_update(imu)` / `meas_update_pos(gnss)` / `meas_update_nhc(imu)` / `feedback()` 在 Task 3 定义, Task 4 调用, 签名一致。`Nhc.build_meas(state, imu)` / `feedback(x2, state)` 在 Task 2 定义, Task 3 调用, 签名一致。

## Execution Handoff

Plan complete and saved to `docs/superpowers/plans/2026-07-08-gnss-ins-lc-ekf.md`. 用户已授权自主执行, 将使用 inline execution (executing-plans 模式) 按 Task 1→7 顺序执行。
