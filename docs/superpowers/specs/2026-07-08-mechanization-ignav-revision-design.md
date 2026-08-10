# INS 机械编排对齐 ignav 修订设计

> 日期: 2026-07-08
> 状态: 已批准（用户授权自主执行）
> 前置: 2026-07-07-mechanization-design.md（已实现的机械编排基础）
> 参考: tools/ignav（速率式 IMU 机械编排、ψ-error 误差模型）

## 1. 目标与背景

### 1.1 目标

将当前 INS 机械编排（基于 gnss_ins_lc_nhc 的 φ-error 模型）对齐到 tools/ignav 的实现（速率式 IMU、ψ-error 误差模型），修正精度差异和存储 bug。

### 1.2 背景

当前项目机械编排基于 gnss_ins_lc_nhc（φ-error），但 gnss_ins_lc_nhc 使用增量式 IMU。tools/ignav 使用速率式 IMU（与本项目一致），是更合适的参考项目。

对比 ignav 与当前实现，识别出以下差异：

| 差异项 | 当前实现 | ignav | 影响 |
|---|---|---|---|
| 旋转补偿 | 一阶 0.5·cross(dθ,dv) | 精确 Rodrigues a1/a2 + 二阶项 | 高速旋转精度 |
| F_vr 重力梯度 | 0（缺失） | -2/(re·\|pos\|)·ge⊗pos | 协方差传播精度 |
| 误差模型 | φ-error (F_vφ=+skew, F_φbg=-C_b_e) | ψ-error (F_vφ=-skew, F_φbg=+C_b_e) | 约定差异（数学等价） |
| Φ 矩阵精度 | 二阶 Taylor | 自适应（1st/2nd/expm） | 大 dt 精度 |
| _prev 存储 | 未补偿值（bug） | 已补偿值 | 锥补/划桨正确性 |

### 1.3 设计决策（用户确认）

1. **修改范围**: 两处关键差异 + 全面对齐 ignav
2. **误差模型**: 切换为 ψ-error（对齐 ignav）
3. **Φ 矩阵精度**: 自适应选择（≥200Hz 一阶, 100-200Hz 二阶, <100Hz 矩阵指数）
4. **增量式接口**: 保留现有速率→增量预处理结构，不过度抽象

## 2. F 矩阵修改 (transfer_matrix.py)

### 2.1 误差模型切换 φ-error → ψ-error

| F 矩阵元素 | 当前 (φ-error) | 修改后 (ψ-error) |
|---|---|---|
| F_vφ (3:6, 6:9) | `+skew(f_e)` | `-skew(f_e)` |
| F_φbg (6:9, 9:12) | `-C_b_e` | `+C_b_e` |
| F_vv (3:6, 3:6) | `-2*skew(w_ie)` | 不变 |
| F_φφ (6:9, 6:9) | `-skew(w_ie)` | 不变 |
| F_vba (3:6, 12:15) | `+C_b_e` | 不变 |
| F_rv (0:3, 3:6) | `I` | 不变 |

### 2.2 添加重力梯度 F_vr

```python
# F_vr = -2/(re·|pos|) · ge ⊗ pos  (3×3 外积, 参考 ignav getF line 490)
ge = gravity_ecef(pos_e)
lat, _, _ = ecef2llh(pos_e)
re = georadi(lat)
F[3:6, 0:3] = -2.0 / (re * np.linalg.norm(pos_e)) * np.outer(ge, pos_e)
```

### 2.3 build_F 签名变更

当前: `build_F(C_b_e, f_b, w_b_ib)`  
修改后: `build_F(C_b_e, f_b, w_b_ib, pos_e)`

`ins_propagate.py` 调用处同步传入 `state.pos_e`。

### 2.4 Φ 矩阵自适应精度

```python
def build_Phi(self, F, dt):
    Fdt = F * dt
    if dt <= 0.005:
        # ≥200Hz (含200Hz): 一阶
        return np.eye(15) + Fdt
    elif dt <= 0.01:
        # 100-200Hz (含100Hz): 二阶
        return np.eye(15) + Fdt + 0.5 * (Fdt @ Fdt)
    else:
        # <100Hz: 矩阵指数 (scaling-and-squaring, 不引入 scipy)
        return _expm(Fdt)
```

矩阵指数自实现（scaling-and-squaring + Taylor 级数，约 15 行），避免引入 scipy 依赖。

### 2.5 修改后 F 矩阵全貌

```
状态顺序: [δr^e(3), δv^e(3), δψ^e(3), δb_g(3), δb_a(3)]

F_rr = 0                                 F_rv = I               F_rψ = 0
F_vr = -2/(re·|pos|)·ge⊗pos              F_vv = -2*[ω_ie^e×]    F_vψ = -[C_b_e·f_b×]
F_ψr = 0                                 F_ψv = 0               F_ψψ = -[ω_ie^e×]
F_vbg = 0                                F_vba = C_b_e
F_ψbg = +C_b_e                           F_ψba = 0
F_bgbg = -I / tau_gyro                   F_baba = -I / tau_acce
```

### 2.6 Q 矩阵 G 映射符号同步

Q = G·Q_diag·G^T 中，G 的符号对 Q 数值无影响（正负抵消），但为保持 ψ-error 约定一致性，同步翻转陀螺噪声→姿态映射符号:

| G 矩阵元素 | 当前 (φ-error) | 修改后 (ψ-error) |
|---|---|---|
| G[6:9, 6:9] (陀螺噪声→姿态) | `-C_b_e` | `+C_b_e` |
| G[3:6, 3:6] (加计噪声→速度) | `+C_b_e` | 不变 |
| G[9:12, 9:12] | `I` | 不变 |
| G[12:15, 12:15] | `I` | 不变 |

## 3. 机械编排修改 (ins_update.py)

### 3.1 旋转补偿升级：一阶 → 精确 Rodrigues

当前（一阶）:
```python
v_rot = 0.5 * np.cross(dtheta_comp, dvel_comp)
```

修改为 ignav 精确 Rodrigues（参考 ignav rotscull_corr, ins.cc line 1239-1278）:
```python
dak = dtheta_comp
dvk = dvel_comp
dak_norm = np.linalg.norm(dak)
if dak_norm < 1e-12:
    v_rot = np.zeros(3, dtype=np.float64)
else:
    dak_sq = dak_norm ** 2
    a1 = (1.0 - math.cos(dak_norm)) / dak_sq
    a2 = (1.0 - math.sin(dak_norm) / dak_norm) / dak_sq
    v_rot = a1 * np.cross(dak, dvk) + a2 * np.cross(dak, np.cross(dak, dvk))
```

划桨补偿保持不变（与 ignav 一致）:
```python
v_scul = (np.cross(prev_dtheta, dvel_comp) + np.cross(prev_dvel, dtheta_comp)) / 12.0
```

### 3.2 修复 _prev_dtheta/_prev_dvel 存储 bug

**Bug**: 当前存储未补偿值:
```python
self._prev_dtheta = dtheta.copy()      # 未补偿
self._prev_dvel = dvel.copy()          # 未补偿
```

锥补/划桨补偿需要已补偿值（ignav 存储 omgbp/fbp 即零偏校正后的值）。

修复:
```python
self._prev_dtheta = dtheta_comp.copy()  # 已补偿
self._prev_dvel = dvel_comp.copy()      # 已补偿
```

### 3.3 不变项

- 位置更新: 梯形积分（与 ignav 的 v·dt+a/2·dt² 数学等价）
- 锥补: 1/12·cross(prev_θ, curr_θ)（与 ignav 一致）
- 姿态更新: Rodrigues + C_ee 地球自转补偿（与 ignav 一致）
- 速度 Coriolis: (g_e - 2·cross(ω_ie, vel))·dt（与 ignav 一致）
- IMU 速率→增量预处理: 保持现有结构（隐式保留增量式接口）

## 4. 辅助文件修改

### 4.1 earth_param.py 新增 georadi

```python
def georadi(lat: float) -> float:
    """地心半径 (参考 ignav georadi, ins-gnss.cc line 214-218)。"""
    e = 0.0818191908425  # WGS84 偏心率
    s = math.sin(lat)
    c = math.cos(lat)
    re = 6378137.0  # WGS84 长半轴
    return re / math.sqrt(1.0 - e**2 * s**2) * \
           math.sqrt(c**2 + (1.0 - e**2)**2 * s**2)
```

### 4.2 transfer_matrix.py 新增 _expm

```python
def _expm(A: np.ndarray, order: int = 10) -> np.ndarray:
    """矩阵指数 (scaling-and-squaring + Taylor 级数, 不依赖 scipy)。"""
    n = A.shape[0]
    norm = float(np.linalg.norm(A, np.inf))
    s = int(np.ceil(np.log2(norm))) if norm > 1.0 else 0
    A_scaled = A / (2.0 ** s)
    result = np.eye(n, dtype=np.float64)
    term = np.eye(n, dtype=np.float64)
    for k in range(1, order + 1):
        term = term @ A_scaled / k
        result += term
    for _ in range(s):
        result = result @ result
    return result
```

### 4.3 ins_propagate.py 调用同步

`build_F` 调用处增加 `pos_e` 参数:
```python
F = self.transfer_matrix.build_F(
    C_b_e=ins_update.state.C_b_e,
    f_b=ins_update.f_b,
    w_b_ib=ins_update.w_b_ib,
    pos_e=ins_update.state.pos_e,  # 新增
)
```

## 5. 文档更新

### 5.1 skills/estimator.md

- F 矩阵描述: φ-error → ψ-error（F_vφ/F_φbg 符号翻转说明）
- 添加 F_vr 重力梯度项描述
- Φ 矩阵: 二阶 Taylor → 自适应（≥200Hz 一阶, 100-200Hz 二阶, <100Hz expm）
- 旋转补偿: 一阶 → 精确 Rodrigues（a1/a2 系数 + 二阶项）
- _prev 值存储: 未补偿 → 已补偿（bug 修复说明）

### 5.2 skills/imu.md

- 机械编排旋转补偿描述同步更新

### 5.3 项目记忆 (project_memory.md)

- 更新 F 矩阵约定: φ-error → ψ-error（对齐 ignav）
- 新增 F_vr 重力梯度项
- 新增 Φ 矩阵自适应精度
- 新增旋转补偿精确 Rodrigues
- 新增 _prev 值存储已补偿值

## 6. 测试策略

### 6.1 单元测试更新

| 测试文件 | 更新内容 |
|---|---|
| tests/ins/test_transfer_matrix.py | F_vφ/F_φbg 符号期望值翻转; 新增 F_vr 重力梯度测试; Φ 自适应三档测试 |
| tests/ins/test_ins_update.py | 旋转补偿期望值更新为精确 Rodrigues; _prev 值存储验证 |
| tests/ins/test_earth_param.py | 新增 georadi 函数测试（赤道/极点/中纬度） |

### 6.2 回归测试

- 机械编排开环测试（5001 历元）: 验证修改后正常运行
- 36 个单元测试全部回归
- internal+on 端到端测试: aligned_internal_rtk.csv 输出正常

## 7. 影响范围与风险

### 7.1 影响文件

| 文件 | 修改类型 |
|---|---|
| src/core/ins/transfer_matrix.py | F 矩阵符号 + F_vr + Φ 自适应 + _expm + Q G 映射符号 |
| src/core/ins/ins_update.py | 旋转补偿 + _prev 存储 bug 修复 |
| src/core/ins/earth_param.py | 新增 georadi |
| src/core/ins/ins_propagate.py | build_F 调用同步 |
| skills/estimator.md | 文档更新 |
| skills/imu.md | 文档更新 |
| tests/ins/test_transfer_matrix.py | 测试更新 |
| tests/ins/test_ins_update.py | 测试更新 |
| tests/ins/test_earth_param.py | 新增 georadi 测试 |

### 7.2 风险评估

- **ψ-error 切换**: 项目当前无 KF 测量更新，切换仅影响 F 矩阵和未来 H 矩阵约定，风险低
- **旋转补偿升级**: 精确 Rodrigues 在小角度时退化为近似一阶，数值稳定
- **_prev bug 修复**: 修正后锥补/划桨使用正确的已补偿值，可能改变开环测试数值
- **Φ 自适应**: 100Hz 下用二阶（与当前一致），不影响主路径；expm 路径仅在 dt>0.01s 时触发

### 7.3 后续注意事项

- 未来实现 KF 测量更新时，H 矩阵姿态部分需用 ψ-error 约定
- 未来实现闭环校正时，状态反馈需用 ψ-error 符号约定
- 保留增量式 IMU 接口: 未来增量式 IMU 只需跳过 `dtheta = gyro*dt` 转换
