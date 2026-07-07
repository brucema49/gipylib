# INS 机械编排与协方差传播设计

> 日期: 2026-07-07
> 状态: 已批准（用户授权自主执行）
> 参考: tools/gnss_ins_lc_nhc（E 系方程）、tools/GINav（算法结构与协方差传播风格）

## 1. 目标

实现 INS 机械编排正向递推（姿态/速度/位置）与协方差传播（P1 主滤波），开环模式验证几十秒输出。

## 2. 架构

### 2.1 模块布局

```
src/core/ins/
├── ins_core.py         (新增) InsCore：E 系机械编排正向递推
├── transfer_matrix.py  (新增) TransferMatrix：F/Φ/Q 构造
└── ins_kf.py           (新增) InsKf：协方差传播（P1）

src/log/
└── trace_writer.py     (新增) TraceWriter：机械编排 trace 输出

tests/ins/
└── test_mechanization.py (新增) 开环递推 50s 验证
```

### 2.2 数据流

```
InsInitializer → (InsState, P1)
       ↓
  InsCore.update(imu)         ← 每历元正向递推
       ↓
  InsKf.propagate(imu, ins_core)  ← 每历元协方差传播
       ↓
  TraceWriter.write(state, P1_diag)  ← 历元级 trace
       ↓
  trace_mech.csv
```

### 2.3 设计原则

- InsCore 与 InsKf 解耦：正向递推不持有协方差，协方差传播不修改 InsState
- TransferMatrix 独立可测：F/Φ/Q 构造逻辑复杂，单独单元测试
- 开环模式：InsKf 只传播 P1，不反馈修正 InsState
- IMU 补偿内联：零偏/标度补偿在 InsCore.update 内部完成

## 3. 组件接口

### 3.1 InsCore

```python
class InsCore:
    def __init__(self, state: InsState): ...
    def update(self, imu: ImuMeasurement) -> InsState:
        """一步递推：IMU 补偿 → 姿态 → 速度 → 位置"""
    @property
    def f_b(self) -> np.ndarray: ...  # 当前历元比力（b 系），供 InsKf 使用
    @property
    def w_b_ib(self) -> np.ndarray: ...  # 当前历元角速度（b 系），供 InsKf 使用
```

### 3.2 TransferMatrix

```python
class TransferMatrix:
    def __init__(self, config: dict): ...
    def build_F(self, state: InsState, f_b: np.ndarray, w_b_ib: np.ndarray) -> np.ndarray:
        """15x15 连续时间 F 矩阵 [pos, vel, att, gyro_bias, accel_bias]"""
    def build_Phi(self, F: np.ndarray, dt: float) -> np.ndarray:
        """Phi = I + F*dt + 0.5*(F*dt)^2"""
    def build_Q(self, dt: float, C_b_e: np.ndarray) -> np.ndarray:
        """15x15 离散 Q 矩阵 (G*Q_diag*G' 风格)"""
```

### 3.3 InsKf

```python
class InsKf:
    def __init__(self, P1: np.ndarray, config: dict): ...
    def propagate(self, imu: ImuMeasurement, ins_core: InsCore) -> None:
        """P1 = Φ·(P1+0.5Q)·Φ^T + 0.5Q (GINav 中间值法)"""
    @property
    def P1(self) -> np.ndarray: ...
```

### 3.4 TraceWriter

```python
class TraceWriter:
    def __init__(self, path: str): ...
    def open(self) -> None: ...
    def write(self, state: InsState, P1_diag: np.ndarray) -> None: ...
    def close(self) -> None: ...
```

字段: timestamp, pos_ecef[3], vel_ecef[3], att_rpy[3], P1_diag[15]

## 4. E 系机械编排方程

参考 gnss_ins_lc_nhc navmech.cc，速率式 IMU 转增量（×dt）后套用。

### 4.1 IMU 补偿

```
dtheta = gyro * dt;  dvel = accel * dt
dtheta_comp = (dtheta - gyro_bias * dt) * (1 - gyro_scale)
dvel_comp   = (dvel   - accel_bias * dt) * (1 - accel_scale)
```

### 4.2 姿态更新（含锥补）

```
phi_b = dtheta_comp + skew(dtheta_prev) * dtheta_comp / 12.0   # 锥补
C_bb = rodrigues(phi_b)
zeta = [0, 0, omega_ie] * dt
C_ee = rodrigues(-zeta)   # 地球自转补偿
C_b_e_new = C_ee @ C_b_e_prev @ C_bb
```

### 4.3 速度更新（含旋转/划桨补偿）

```
v_rot  = 0.5 * cross(dtheta_comp, dvel_comp)
v_scul = (cross(dtheta_prev, dvel_comp) + cross(dvel_prev, dtheta_comp)) / 12.0
g_e = gravity_ecef(pos_e_prev)
delta_v_cor = (g_e - 2*cross(omega_ie_e, vel_e_prev)) * dt
C_ee_v = I - skew(omega_ie_e * 0.5 * dt)
delta_v = C_ee_v @ C_b_e_prev @ (dvel_comp + v_rot + v_scul)
vel_e_new = vel_e_prev + delta_v_cor + delta_v
```

### 4.4 位置更新（梯形）

```
pos_e_new = pos_e_prev + 0.5 * (vel_e_prev + vel_e_new) * dt
```

## 5. F 矩阵（15 维，保留 Coriolis 项）

状态顺序: [δr^e(3), δv^e(3), δφ^e(3), δb_g(3), δb_a(3)]

```
F_rr = 0                    F_rv = I                  F_rφ = 0
F_vr = 0                    F_vv = -2*[ω_ie^e×]       F_vφ = [C_b_e·f_b×]
F_φr = 0                    F_φv = 0                  F_φφ = -[ω_ie^e×]
F_vbg = 0                   F_vba = C_b_e
F_φbg = -C_b_e              F_φba = 0
F_bgbg = -I / tau_gyro      F_baba = -I / tau_acce
```

离散化: Φ = I + F·dt + 0.5·(F·dt)²

## 6. Q 矩阵（GINav G·Q_diag·G^T 中间值法）

```
G (15x15):
  G[6:9, 6:9]  = -C_b_e   # 陀螺噪声 → 姿态
  G[3:6, 3:6]  =  C_b_e   # 加计噪声 → 速度
  G[9:12, 9:12] = I        # 陀螺零偏驱动噪声
  G[12:15, 12:15] = I      # 加计零偏驱动噪声

Q_diag (15x15 对角):
  Q[0:3]   = 0
  Q[3:6]   = accel_psd * dt
  Q[6:9]   = gyro_psd * dt
  Q[9:12]  = gyro_bias_psd * dt
  Q[12:15] = acce_bias_psd * dt

Q0 = G @ Q_diag @ G.T
P0 = P1 + 0.5 * Q0
P1_new = Phi @ P0 @ Phi.T + 0.5 * Q0
```

## 7. 测试方案

- 数据: data/cpt_imu.csv (IMU) + data/spp.pos (GNSS, 仅用于初始化)
- 初始化: 速度矢量模式（VELOCITY_VECTOR）
- 递推时长: 50s（reboot 阈值），约 5000 历元（100Hz）
- 模式: 开环（无 GNSS 量测更新，无状态反馈）
- 输出: trace_mech.csv，每历元一行
- 验收:
  - trace 文件存在且行数 ≈ 5000
  - 位置/速度/姿态无 NaN
  - P1 对角线单调增长（无 GNSS 修正，协方差发散属正常）
  - 终端打印前 5 历元和最后 5 历元的摘要

## 8. 错误处理

- IMU 时间戳非单调递增: 跳过该历元，warning
- dt <= 0: 跳过
- dt 异常大（> 1s）: 警告但继续
- C_b_e 正交化: 每历元结束后归一化四元数
