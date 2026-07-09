# 滤波估计器状态参数重构计划

## Summary

将当前双滤波架构 (P1 15维 + P2 5维) 重构为**单滤波架构**，采用 `StateIndex` 参数块管理（参考 gnss_ins_lc_nhc）和矩阵块构造方式（参考 GREAT-MSF）。状态向量由固定 15 维基础参数 + 可选参数块组成，算法参考 ignav。

**目标状态结构**：
- 固定 15 维：pos(3) + vel(3) + att(3) + gyro_bias(3) + accel_bias(3)
- 可选 3 维：GNSS 天线杆臂 (b系)
- 可选 2 维：IMU 安装角误差 (pitch, yaw)
- 可选 3 维：IMU 杆臂 (b→v, NHC 用) — 与安装角绑定
- 可选 1 维：时间对齐误差 (IMU-GNSS 时间同步)
- **放弃**：比例因子误差、非正交角误差等

总维数：15 (最小) ~ 24 (全部启用)

## Current State Analysis

### 当前架构问题

| 问题 | 现状 | 影响 |
|------|------|------|
| 双滤波 P1/P2 分离 | P1=15维硬编码, P2=5维独立 | 无法统一管理参数块, H 矩阵需拆分 H1/H2 |
| 无 StateIndex | 状态索引硬编码 (0:3, 3:6, ...) | 添加可选参数需修改多处硬编码索引 |
| GNSS 杆臂未估计 | `leverarm` 仅在 H_vel 中硬编码, 不作为状态 | 无法在线标定天线杆臂 |
| 时间对齐缺失 | 完全未实现 | IMU-GNSS 时间偏差无法补偿 |
| 安装角在 P2 | NHC H2 仅 5维, 与 P1 分离 | 不符合 gnss_ins_lc_nhc 单滤波模式 |

### 关键文件

- [lc_estimator.py](file:///home/mxl/workplace/gipylib/src/core/ins/lc_estimator.py) — 双滤波估计器, P1=15硬编码, P2=5独立
- [transfer_matrix.py](file:///home/mxl/workplace/gipylib/src/core/ins/transfer_matrix.py) — F/Φ/Q 构造, 固定 15×15
- [nhc.py](file:///home/mxl/workplace/gipylib/src/core/ins/nhc.py) — NHC 量测, 返回 H1[2,15]+H2[2,5] 拆分
- [data_types.py](file:///home/mxl/workplace/gipylib/src/core/data_types.py) — InsState 无 time_sync 字段

### 参考实现分析

**gnss_ins_lc_nhc** (单滤波 + StateIndex):
```cpp
struct StateIndex {
    int pos_index_, vel_index_, att_index_;
    int gyro_bias_index_, acce_bias_index_;
    int imu_angle_index_, imu_leverarm_index_;
    int total_state;
};
void SetStateIndex(StateIndex& si) {
    si.pos_index_ = 0; si.vel_index_ = 3; ...;
    if (evaluate_imu_angle) { si.imu_angle_index_ = basic+3; ...; basic += 5; }
    si.total_state = basic + 3;
}
```

**GREAT-MSF** (单滤波 + t_gallpar + block操作):
```cpp
Ft.block(9, 9, 3, 3) = sins._tauG;  // eb-eb
Hk.block(0, 0, 3, 3) = askew(Ceb * lever);  // pos-att
param_of_sins.getParam(_name, par_type::ODO_k, "");  // 检查参数是否存在
```

**ignav** (xn/xi 函数 + H矩阵公式):
```c
// H 矩阵 (build_HVR):
H[idt, pos] = Cbe @ skew(omgb) @ lever + ve    // jacobian_p_dt
H[idt, vel] = Cbe @ skew(omgb)² @ lever + ae   // jacobian_v_dt
H[ila, pos] = -Cbe                               // line 713
H[ila, vel] = skew(w_ie_e) @ Cbe - Cbe @ skew(omgb)  // jacobian_v_dla
// F/Q: lever_arm=常数(F=0,Q=0), time_sync=随机游走(F=0,Q=psd.dt*dt)
```

## Proposed Changes

### 1. 新建 `state_index.py` — StateIndex 参数块管理

**文件**: `src/core/ins/state_index.py` (新建)

**设计**: 镜像 gnss_ins_lc_nhc 的 StateIndex 结构, Python dataclass。

```python
@dataclass
class StateIndex:
    """状态参数块索引管理 (参考 gnss_ins_lc_nhc StateIndex)。"""
    # 固定 15 维 (始终存在)
    pos: int = 0          # 0-2
    vel: int = 3          # 3-5
    att: int = 6          # 6-8
    gyro_bias: int = 9    # 9-11
    accel_bias: int = 12  # 12-14
    # 可选参数块 (-1 表示未启用)
    lever_arm: int = -1     # 3 维, GNSS 天线杆臂
    imu_angle: int = -1     # 2 维, IMU 安装角
    imu_leverarm: int = -1  # 3 维, IMU 杆臂 (b→v)
    time_sync: int = -1     # 1 维, 时间对齐误差
    # 总维数
    dim: int = 15

    @classmethod
    def from_config(cls, config: dict) -> "StateIndex":
        """根据配置构建状态索引 (参考 SetStateIndex)。"""
        si = cls()
        ins_cfg = config.get("ins", {})
        idx = 15  # 基础 15 维后
        if ins_cfg.get("estimate_leverarm", 0):
            si.lever_arm = idx; idx += 3
        if ins_cfg.get("estimate_mounting_angle", 0):
            si.imu_angle = idx; idx += 2
            if ins_cfg.get("estimate_imu_leverarm", 0):
                si.imu_leverarm = idx; idx += 3
        if ins_cfg.get("estimate_time_sync", 0):
            si.time_sync = idx; idx += 1
        si.dim = idx
        return si

    def has_lever_arm(self) -> bool: return self.lever_arm >= 0
    def has_imu_angle(self) -> bool: return self.imu_angle >= 0
    def has_imu_leverarm(self) -> bool: return self.imu_leverarm >= 0
    def has_time_sync(self) -> bool: return self.time_sync >= 0
```

### 2. 修改 `transfer_matrix.py` — 支持 StateIndex 动态维度

**文件**: [transfer_matrix.py](file:///home/mxl/workplace/gipylib/src/core/ins/transfer_matrix.py)

**改动**:
- `build_F` 接受 `StateIndex`, 返回 `si.dim × si.dim` 矩阵
- 基础 15×15 块复用现有逻辑, 可选块追加
- `build_Phi` / `build_Q` 同步扩展

**F 矩阵可选块** (参考 ignav):
- `lever_arm`: F=0 (常数, 无状态转移)
- `imu_angle`: F=0 (常数)
- `imu_leverarm`: F=0 (常数)
- `time_sync`: F=0 (随机游走, F=0, Q≠0)

**Q 矩阵可选块**:
- `lever_arm`: Q=0 (常数) 或极小随机游走
- `imu_angle`: Q = sigma_angle² · dt (从当前 _build_Q2 迁移)
- `imu_leverarm`: Q = sigma_lever² · dt (从当前 _build_Q2 迁移)
- `time_sync`: Q = time_sync_psd · dt (新增配置项)

```python
class TransferMatrix:
    def __init__(self, config: dict, state_index: StateIndex):
        self.si = state_index
        # ... 现有参数 ...
        # 新增可选参数 PSD
        ins_cfg = config.get("ins", {})
        self.lever_arm_psd = ins_cfg.get("lever_arm_psd", 0.0)  # 常数=0
        self.imu_angle_psd = ins_cfg.get("imu_angle_psd", 1e-6)
        self.imu_leverarm_psd = ins_cfg.get("imu_leverarm_psd", 1e-8)
        self.time_sync_psd = ins_cfg.get("time_sync_psd", 1e-4)

    def build_F(self, C_b_e, f_b, w_b_ib, pos_e) -> np.ndarray:
        n = self.si.dim
        F = np.zeros((n, n), dtype=np.float64)
        # 基础 15×15 (复用现有逻辑)
        F[0:15, 0:15] = self._build_F_base(C_b_e, f_b, w_b_ib, pos_e)
        # 可选块: F=0 (无需额外赋值)
        return F

    def build_Q(self, dt, C_b_e) -> np.ndarray:
        n = self.si.dim
        Q = np.zeros((n, n), dtype=np.float64)
        # 基础 15×15 (复用现有逻辑)
        Q[0:15, 0:15] = self._build_Q_base(dt, C_b_e)
        # 可选块
        si = self.si
        if si.has_imu_angle():
            Q[si.imu_angle:si.imu_angle+2, si.imu_angle:si.imu_angle+2] = \
                np.diag([self.imu_angle_psd * dt] * 2)
        if si.has_imu_leverarm():
            Q[si.imu_leverarm:si.imu_leverarm+3, si.imu_leverarm:si.imu_leverarm+3] = \
                np.diag([self.imu_leverarm_psd * dt] * 3)
        if si.has_time_sync():
            Q[si.time_sync, si.time_sync] = self.time_sync_psd * dt
        # lever_arm: Q=0 (常数)
        return Q
```

### 3. 修改 `lc_estimator.py` — 单滤波架构

**文件**: [lc_estimator.py](file:///home/mxl/workplace/gipylib/src/core/ins/lc_estimator.py)

**核心改动**:
1. 移除 P2, 统一为 `P` (N×N) 和 `x` (N维)
2. 使用 `StateIndex` 管理状态索引
3. H 矩阵按 StateIndex 放置块 (参考 GREAT-MSF block操作)
4. NHC 不再拆分 H1/H2, 返回完整 H (N列)
5. feedback 统一处理所有参数块

**H 矩阵构造** (参考 ignav build_HVR):

```python
def meas_update_pos(self, gnss: GnssSolution) -> None:
    state = self.ins_update.state
    si = self.si
    # Z = r_ins - r_gnss (已扣除杆臂的 INS 位置)
    Z = state.pos_e - gnss.position
    H = np.zeros((3, si.dim), dtype=np.float64)
    H[:, si.pos:si.pos+3] = np.eye(3)
    # 可选: GNSS 杆臂
    if si.has_lever_arm():
        H[:, si.lever_arm:si.lever_arm+3] = -state.C_b_e  # ignav: H[ila,pos]=-Cbe
    # 可选: 时间对齐
    if si.has_time_sync():
        # ignav jacobian_p_dt: dt1 = Cbe @ skew(omgb) @ lever + ve
        lever = state.leverarm
        dt1 = state.C_b_e @ skew(self.ins_update.w_b_ib) @ lever + state.vel_e
        H[:, si.time_sync] = dt1
    R = self._build_pos_R(gnss)
    self._joseph_update(Z, H, R)

def meas_update_vel(self, gnss: GnssSolution) -> None:
    state = self.ins_update.state
    si = self.si
    Z = state.vel_e - gnss.velocity
    H = np.zeros((3, si.dim), dtype=np.float64)
    H[:, si.vel:si.vel+3] = np.eye(3)
    # 杆臂对速度的姿态贡献 (现有逻辑)
    if np.linalg.norm(state.leverarm) > 1e-9:
        H[:, si.att:si.att+3] = -skew(state.C_b_e @ state.leverarm)
    # 可选: GNSS 杆臂
    if si.has_lever_arm():
        # ignav jacobian_v_dla: dla = skew(w_ie_e) @ Cbe - Cbe @ skew(omgb)
        dla = skew(self.tm.w_ie_e) @ state.C_b_e - state.C_b_e @ skew(self.ins_update.w_b_ib)
        H[:, si.lever_arm:si.lever_arm+3] = dla
    # 可选: 时间对齐
    if si.has_time_sync():
        # ignav jacobian_v_dt: dt2 = Cbe @ skew(omgb)² @ lever + ae
        lever = state.leverarm
        w_skew = skew(self.ins_update.w_b_ib)
        dt2 = state.C_b_e @ w_skew @ w_skew @ lever + self.ins_update.a_e
        H[:, si.time_sync] = dt2
    R = self._build_vel_R(gnss)
    self._joseph_update(Z, H, R)
```

**NHC 量测更新** (单 H 矩阵):
```python
def meas_update_nhc(self, imu: ImuMeasurement) -> None:
    Z, H, R = self._nhc.build_meas(self.ins_update.state, imu, self.si)
    self._joseph_update(Z, H, R)
```

**统一 Joseph 更新**:
```python
def _joseph_update(self, Z, H, R) -> None:
    S = H @ self.P @ H.T + R
    K = self.P @ H.T @ np.linalg.inv(S)
    innov = Z - H @ self.x
    self.x = self.x + K @ innov
    I_KH = np.eye(self.si.dim) - K @ H
    self.P = I_KH @ self.P @ I_KH.T + K @ R @ K.T
    self.P = 0.5 * (self.P + self.P.T)
```

**统一反馈**:
```python
def feedback(self) -> None:
    state = self.ins_update.state
    si = self.si
    # 基础 15 维 (ψ-error, 现有逻辑)
    delta_pos = self.x[si.pos:si.pos+3]
    delta_vel = self.x[si.vel:si.vel+3]
    delta_psi = self.x[si.att:si.att+3]
    delta_bg = self.x[si.gyro_bias:si.gyro_bias+3]
    delta_ba = self.x[si.accel_bias:si.accel_bias+3]
    # ... 现有 P1 反馈逻辑 ...
    # 可选: GNSS 杆臂
    if si.has_lever_arm():
        new_state.leverarm = state.leverarm + self.x[si.lever_arm:si.lever_arm+3]
    # 可选: 安装角 + IMU 杆臂
    if si.has_imu_angle():
        new_state.imu_angle = state.imu_angle + self.x[si.imu_angle:si.imu_angle+2]
    if si.has_imu_leverarm():
        new_state.imu_leverarm = state.imu_leverarm - self.x[si.imu_leverarm:si.imu_leverarm+3]
    # 可选: 时间对齐
    if si.has_time_sync():
        new_state.time_sync = state.time_sync + self.x[si.time_sync]
    self.x[:] = 0.0
```

### 4. 修改 `nhc.py` — 返回单 H 矩阵

**文件**: [nhc.py](file:///home/mxl/workplace/gipylib/src/core/ins/nhc.py)

**改动**: `build_meas` 接受 `StateIndex`, 返回 `Z[2], H[2, N], R[2,2]` (不再拆分 H1/H2)

```python
def build_meas(self, state: InsState, imu: ImuMeasurement,
               si: StateIndex) -> tuple:
    """构造 NHC 量测 (单 H 矩阵, N = si.dim)。"""
    # ... 现有 v_v 计算 ...
    H = np.zeros((2, si.dim), dtype=np.float64)
    # H1 部分 (基础 15 维)
    H[:, si.vel:si.vel+3] = RbCe[1:3, :]
    H[:, si.att:si.att+3] = (RbCe @ skew(v_e))[1:3, :]
    H[:, si.gyro_bias:si.gyro_bias+3] = -(self.R_b_v @ skew(l_imu_b))[1:3, :]
    # H2 部分 (可选, 按 StateIndex 放置)
    if si.has_imu_angle():
        H[:, si.imu_angle:si.imu_angle+2] = skew(v_v)[1:3, 0:2]
    if si.has_imu_leverarm():
        H[:, si.imu_leverarm:si.imu_leverarm+3] = (self.R_b_v @ skew(w_b_ib))[1:3, :]
    return Z, H, R
```

移除 `feedback` 方法 (反馈逻辑统一到 LcEstimator)。

### 5. 修改 `data_types.py` — 新增 time_sync 字段

**文件**: [data_types.py](file:///home/mxl/workplace/gipylib/src/core/data_types.py)

```python
@dataclass
class InsState:
    # ... 现有字段 ...
    time_sync: float = 0.0  # IMU-GNSS 时间对齐误差 (s)
```

### 6. 修改 `config.yaml` — 新增配置项

**文件**: [config.yaml](file:///home/mxl/workplace/gipylib/data/config.yaml)

```yaml
ins:
  #--- 可选状态参数开关 (参考 gnss_ins_lc_nhc evaluate_*) ---#
  estimate_leverarm: 0              # 1=估计 GNSS 天线杆臂 (3维)
  estimate_mounting_angle: 0        # 1=估计 IMU 安装角 (2维, 启用 P2→合并到主滤波)
  estimate_imu_leverarm: 0          # 1=估计 IMU 杆臂 b→v (3维, 需 estimate_mounting_angle=1)
  estimate_time_sync: 0             # 1=估计时间对齐误差 (1维)
  #--- 可选参数过程噪声 PSD ---#
  lever_arm_psd: 0.0                # GNSS 杆臂随机游走 PSD (m²/s, 0=常数)
  imu_angle_psd: 1.0e-6             # 安装角随机游走 PSD (rad²/s)
  imu_leverarm_psd: 1.0e-8          # IMU 杆臂随机游走 PSD (m²/s)
  time_sync_psd: 1.0e-4             # 时间对齐随机游走 PSD (s²/s)
  #--- 可选参数初始不确定度 ---#
  lever_arm_std: [0.1, 0.1, 0.1]    # GNSS 杆臂初始标准差 [m]
  time_sync_std: 0.01               # 时间对齐初始标准差 [s]
```

### 7. 修改测试文件

**测试策略**: 保持现有 E2E 测试 (无可选参数时行为不变), 新增单元测试覆盖可选参数块。

- [test_lc_estimator.py](file:///home/mxl/workplace/gipylib/tests/ins/test_lc_estimator.py) — 更新为单滤波接口
- [test_nhc.py](file:///home/mxl/workplace/gipylib/tests/ins/test_nhc.py) — 更新 build_meas 签名
- [test_transfer_matrix.py](file:///home/mxl/workplace/gipylib/tests/ins/test_transfer_matrix.py) — 新增动态维度测试
- [test_lc_e2e.py](file:///home/mxl/workplace/gipylib/tests/ins/test_lc_e2e.py) — 默认配置 (无可选参数) 验证不回归
- 新建 `test_state_index.py` — StateIndex 构建与索引正确性
- 新建 `test_optional_params.py` — 启用各可选参数块的 H 矩阵正确性

## Assumptions & Decisions

1. **单滤波架构**: 合并 P1+P2 为单滤波 (参考 gnss_ins_lc_nhc/GREAT-MSF 均为单滤波)
2. **IMU 杆臂保留为可选**: 当前 P2 估计 imu_leverarm, 重构后保留为可选参数块 (与 imu_angle 绑定, 需先启用 imu_angle)
3. **状态顺序**: pos(0-2), vel(3-5), att(6-8), gyro_bias(9-11), accel_bias(12-14), [lever_arm], [imu_angle], [imu_leverarm], [time_sync]
4. **杆臂符号**: GNSS 杆臂反馈 `leverarm += delta` (δlever = lever_true - lever_est, 与 ignav 一致)
5. **时间对齐符号**: `time_sync += delta` (δt = t_true - t_est)
6. **默认配置**: 所有可选参数默认关闭, 确保现有 E2E 测试不回归
7. **F 矩阵可选块**: lever_arm/imu_angle/imu_leverarm/time_sync 均为 F=0 (常数或随机游走)
8. **杆臂 Q**: lever_arm 默认 Q=0 (常数标定), 可通过 lever_arm_psd 配置为随机游走
9. **a_e 字段**: InsUpdate 需暴露 `a_e` (ECEF 加速度) 供 time_sync H 矩阵使用, 从机械编排中获取
10. **向后兼容**: 无可选参数时 (si.dim=15), 行为与当前 P1 完全一致

## Verification Steps

### 阶段 1: 单元测试
1. `test_state_index.py`: 验证 StateIndex.from_config 正确构建各组合 (15/18/20/23/24 维)
2. `test_transfer_matrix.py`: 验证 F/Φ/Q 维度正确, 可选块为零或正确 PSD
3. `test_nhc.py`: 验证 build_meas 返回 H 维度 = si.dim, H2 块放置正确
4. `test_lc_estimator.py`: 验证单滤波 Joseph 更新, feedback 覆盖所有启用块

### 阶段 2: 回归测试 (默认配置, 无可选参数)
5. `test_lc_e2e.py`: 运行现有 E2E 测试, 验证 planar max ≤0.5m, elev max ≤1.0m, match rate ≥95%
6. 对比 lc_gnss_ref.csv, 确认输出与重构前一致

### 阶段 3: 可选参数验证
7. 启用 `estimate_leverarm=1` (杆臂初值=配置值), 验证杆臂收敛且位置精度不退化
8. 启用 `estimate_mounting_angle=1`, 验证安装角估计与 NHC H2 一致
9. 启用 `estimate_time_sync=1` (time_sync_std=0.001), 验证时间偏差估计稳定
10. 全部启用 (24维), 验证滤波器稳定且精度达标

### 验证标准 (与项目约束一致)
- 平面位置误差 ≤ 0.5m (lat/lon 差 < 5e-6 度)
- 高程误差 ≤ 1.0m
- 匹配率 ≥ 95%
- 53+ 单元测试全部通过
