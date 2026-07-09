# 滤波估计器状态参数审核与修复计划

## 摘要

用户要求审核滤波估计器状态参数：固定 15 维基础状态 + 可选 3 维杆臂 + 可选 2 维安装角 + 可选 1 维时间对齐，放弃比例因子等其他误差，算法参考 ignav，参数块/矩阵块形式参考 gnss_ins_lc_nhc 和 GREAT-MSF。

经审核，核心模块（state_index.py、lc_estimator.py、nhc.py、transfer_matrix.py）已正确实现单滤波 + StateIndex 架构，H 矩阵公式对齐 ignav。但存在 **6 类遗留问题**需修复：比例因子残留、initializer 双滤波接口、两个测试文件未迁移、文档注释过时。

## 当前状态分析

### ✅ 已符合要求（无需修改）

| 文件 | 状态 |
|------|------|
| [state_index.py](file:///home/mxl/workplace/gipylib/src/core/ins/state_index.py) | 15 固定 + 可选 lever_arm(3)/imu_angle(2)/imu_leverarm(3)/time_sync(1)，无比例因子 ✅ |
| [lc_estimator.py](file:///home/mxl/workplace/gipylib/src/core/ins/lc_estimator.py) | 单滤波，StateIndex 管理，H 矩阵对齐 ignav（jacobian_p_dt/jacobian_v_dt/jacobian_v_dla）✅ |
| [nhc.py](file:///home/mxl/workplace/gipylib/src/core/ins/nhc.py) | 单 H 矩阵，StateIndex 参数块，可选块正确接入 ✅ |
| [transfer_matrix.py](file:///home/mxl/workplace/gipylib/src/core/ins/transfer_matrix.py) | F/Φ/Q 动态维度，可选块 F=0，Q 随机游走 PSD ✅ |
| [ins_update.py](file:///home/mxl/workplace/gipylib/src/core/ins/ins_update.py) `a_e` 属性 | 已添加，供 time_sync H 矩阵使用 ✅ |
| config.yaml 可选开关 | estimate_leverarm/mounting_angle/imu_leverarm/time_sync + PSD ✅ |

### ❌ 需修复的问题

#### 问题 A：比例因子残留（用户要求"放弃比例因子误差"）

- **A1** [data_types.py:42-43](file:///home/mxl/workplace/gipylib/src/core/data_types.py#L42-L43)：`InsState` 仍有 `gyro_scale`、`accel_scale` 字段
- **A2** [ins_update.py:87-88,120-121](file:///home/mxl/workplace/gipylib/src/core/ins/ins_update.py#L87-L88)：IMU 补偿使用 `(1.0 - scale)` 乘法，新状态传递 scale
- **A3** [initializer.py:360-369,392-393](file:///home/mxl/workplace/gipylib/src/core/ins/initializer.py#L360-L369)：读取 `initial_gyro_scale`/`initial_acce_scale`，设置到状态
- **A4** [config.yaml:146-152,159-164](file:///home/mxl/workplace/gipylib/data/config.yaml#L146-L164)：比例因子配置项（initial_gyro_scale、initial_acce_scale、evaluate_imu_scale、gyro_scale_std、acce_scale_std、corr_time_of_gyro_scale、corr_time_of_acce_scale）
- **A5** 测试文件 `make_state` 设置 `gyro_scale`/`accel_scale`：test_nhc.py、test_lc_estimator.py、test_lc_integration.py、test_ins_update.py

#### 问题 B：initializer.py 仍返回 (P1, P2) 双滤波接口

- **B1** [initializer.py:66](file:///home/mxl/workplace/gipylib/src/core/ins/initializer.py#L66)：返回类型 `Tuple[InsState, np.ndarray, np.ndarray]`
- **B2** [initializer.py:74](file:///home/mxl/workplace/gipylib/src/core/ins/initializer.py#L74)：docstring "Returns: (InsState, P1, P2)"
- **B3** [initializer.py:116](file:///home/mxl/workplace/gipylib/src/core/ins/initializer.py#L116)：`P1, P2 = self._set_initial_variance(mode)`
- **B4** [initializer.py:118](file:///home/mxl/workplace/gipylib/src/core/ins/initializer.py#L118)：`return state, P1, P2`
- **B5** [initializer.py:399-446](file:///home/mxl/workplace/gipylib/src/core/ins/initializer.py#L399-L446)：`_set_initial_variance` 返回 (P1=15×15, P2=5×5)，应改为返回单个 P（维度 = si.dim，可选块在 StateIndex 位置填充）

#### 问题 C：test_lc_e2e.py 使用旧双滤波接口

- **C1** [test_lc_e2e.py:113-114](file:///home/mxl/workplace/gipylib/tests/ins/test_lc_e2e.py#L113-L114)：`init_P1 = None` / `init_P2 = None`
- **C2** [test_lc_e2e.py:156](file:///home/mxl/workplace/gipylib/tests/ins/test_lc_e2e.py#L156)：`init_state, init_P1, init_P2 = initializer.initialize(...)`
- **C3** [test_lc_e2e.py:203](file:///home/mxl/workplace/gipylib/tests/ins/test_lc_e2e.py#L203)：`est = LcEstimator(init_state, init_P1, init_P2, cfg)`

#### 问题 D：test_lc_integration.py 使用旧双滤波接口

- **D1** [test_lc_integration.py:33](file:///home/mxl/workplace/gipylib/tests/ins/test_lc_integration.py#L33)：make_state 设置 gyro_scale/accel_scale
- **D2** 多处（行 59,69,82,101,113,140,162）：`LcEstimator(state, np.eye(15)*0.01, np.eye(5)*0.01, make_config())` → 移除第三个参数
- **D3** 行 91,95,126,129,153,156：`est.P1` → `est.P`

#### 问题 E：lc_integration.py 文档注释过时

- **E1** [lc_integration.py:28](file:///home/mxl/workplace/gipylib/src/core/ins/lc_integration.py#L28)：docstring "双滤波 EKF" → "单滤波 EKF"
- **E2** [lc_integration.py:58](file:///home/mxl/workplace/gipylib/src/core/ins/lc_integration.py#L58)：注释 "P1/P2 协方差传播" → "P 协方差传播"

#### 问题 F：skills 文档引用比例因子（低优先级，文档非代码）

- skills/imu.md、skills/config.md、skills/conf.md、skills/初始化.md、skills/机械编排.md 引用比例因子
- 本次不修改 skills 文档（用户关注滤波估计器代码，文档可后续清理）

## 修复方案

### 修改 1：移除比例因子（问题 A）

**data_types.py**（A1）：
- 删除 `gyro_scale` 和 `accel_scale` 两个字段（行 42-43）
- `InsState` 变为：timestamp, pos_e, vel_e, C_b_e, q_b_e, att_rpy, gyro_bias, accel_bias, imu_angle, imu_leverarm, leverarm, time_sync

**ins_update.py**（A2）：
- 行 87：`dtheta_comp = (dtheta - self.state.gyro_bias * dt) * (1.0 - self.state.gyro_scale)` → `dtheta_comp = dtheta - self.state.gyro_bias * dt`
- 行 88：`dvel_comp = (dvel - self.state.accel_bias * dt) * (1.0 - self.state.accel_scale)` → `dvel_comp = dvel - self.state.accel_bias * dt`
- 行 120-121：删除 `gyro_scale=...`、`accel_scale=...` 两行

**initializer.py**（A3）：
- 行 316-317 docstring：删除比例因子说明
- 行 338-340：删除 `constant_ppm` 常量（仅比例因子用）
- 行 359-369：删除 `gyro_scale_ppm`/`accel_scale_ppm`/`gyro_scale`/`accel_scale` 读取与转换
- 行 392-393：删除 `gyro_scale=gyro_scale, accel_scale=accel_scale`

**config.yaml**（A4）：
- 行 146-147：删除比例因子单位说明注释
- 行 150-151：删除 `initial_gyro_scale`/`initial_acce_scale`
- 行 152：删除 `evaluate_imu_scale`
- 行 159-160：删除 `gyro_scale_std`/`acce_scale_std`
- 行 163-164：删除 `corr_time_of_gyro_scale`/`corr_time_of_acce_scale`

**测试文件 make_state**（A5）：
- test_nhc.py:38、test_lc_estimator.py:33、test_lc_integration.py:33、test_ins_update.py:22-23：删除 `gyro_scale=np.zeros(3), accel_scale=np.zeros(3)` 两行

### 修改 2：initializer.py 返回单 P（问题 B）

**initialize() 方法**（B1-B4）：
- 行 66 返回类型：`-> Tuple[InsState, np.ndarray]`
- 行 74 docstring：`Returns: (InsState, P): 初始状态 + 单滤波协方差`
- 行 116：`P = self._set_initial_variance(mode)`
- 行 118：`return state, P`

**_set_initial_variance() 方法**（B5）：
- 返回类型：`-> np.ndarray`
- 引入 `StateIndex.from_config(self.config)` 获取 si
- 构建 `P = np.zeros((si.dim, si.dim))`
- 基础 15 维：复用现有 pos/vel/att/gyro_bias/accel_bias std 逻辑，填入 `P[0:15, 0:15]`
- 可选块按 StateIndex 位置填充：
  - `si.has_lever_arm()`：`P[si.lever_arm:si.lever_arm+3, si.lever_arm:si.lever_arm+3] = diag(lever_arm_std²)`
  - `si.has_imu_angle()`：`P[si.imu_angle:si.imu_angle+2, ...] = diag(radians(imu_angle_std)²)`
  - `si.has_imu_leverarm()`：`P[si.imu_leverarm:si.imu_leverarm+3, ...] = diag(imu_leverarm_std²)`
  - `si.has_time_sync()`：`P[si.time_sync, si.time_sync] = time_sync_std²`
- 删除原 P2 构造逻辑（imu_angle_std/imu_leverarm_std 独立 5×5）
- config.yaml 已有 lever_arm_std（行 226）、time_sync_std（行 227）；imu_angle_std（行 212）、imu_leverarm_std（行 213）已有

### 修改 3：test_lc_e2e.py 迁移单滤波接口（问题 C）

- 行 113-114：`init_P1 = None` / `init_P2 = None` → `init_P = None`
- 行 156：`init_state, init_P1, init_P2 = initializer.initialize(block, InitMode.VELOCITY_VECTOR)` → `init_state, init_P = initializer.initialize(block, InitMode.VELOCITY_VECTOR)`
- 行 158-159 log 中 `init_state` 不变
- 行 203：`est = LcEstimator(init_state, init_P1, init_P2, cfg)` → `est = LcEstimator(init_state, init_P, cfg)`

### 修改 4：test_lc_integration.py 迁移单滤波接口（问题 D）

- 行 33：make_state 删除 `gyro_scale=np.zeros(3), accel_scale=np.zeros(3)`
- 行 59：`LcEstimator(state, np.eye(15) * 0.01, np.eye(5) * 0.01, make_config())` → `LcEstimator(state, np.eye(15) * 0.01, make_config())`
- 行 69、82、101：同上修改
- 行 91、95：`est.P1` → `est.P`
- 行 113、140、162：`LcEstimator(state, np.eye(15) * 10.0, np.eye(5) * 0.01, make_config())` → `LcEstimator(state, np.eye(15) * 10.0, make_config())`
- 行 126、129、153、156：`est.P1` → `est.P`

### 修改 5：lc_integration.py 文档注释（问题 E）

- 行 28 docstring：`松组合导航集成 (最近邻时间对齐 + 双滤波 EKF 主循环)` → `松组合导航集成 (最近邻时间对齐 + 单滤波 EKF 主循环)`
- 行 58 注释：`# 2. 推进 (机械编排 + P1/P2 协方差传播)` → `# 2. 推进 (机械编排 + P 协方差传播)`

## 假设与决策

1. **比例因子完全移除**：用户明确"放弃比例因子误差"，从 InsState、ins_update、initializer、config、测试中全部删除（非置 0 保留）。ins_update 的 IMU 补偿公式从 `(dtheta - bias*dt)*(1-scale)` 简化为 `dtheta - bias*dt`。

2. **initializer 返回单 P 维度 = si.dim**：默认配置（无可选参数）时 P 为 15×15，与原 P1 一致，E2E 测试不受影响。启用可选参数时 P 自动扩展。

3. **skills 文档本次不修改**：用户关注滤波估计器代码，skills/*.md 为设计文档，比例因子描述可在后续文档清理时统一处理，避免本次改动范围过大。

4. **不修改 lc_estimator.py**：已正确实现单滤波 + StateIndex，H 矩阵对齐 ignav，无需改动。

5. **不修改 nhc.py / transfer_matrix.py / state_index.py**：已正确实现，无需改动。

## 验证步骤

### 步骤 1：单元测试
```bash
cd /home/mxl/workplace/gipylib
python3 -m pytest tests/ins/test_state_index.py tests/ins/test_transfer_matrix.py tests/ins/test_nhc.py tests/ins/test_lc_estimator.py tests/ins/test_lc_integration.py tests/ins/test_ins_update.py -v
```
预期：全部通过，无 gyro_scale/accel_scale 相关错误。

### 步骤 2：E2E 测试
```bash
cd /home/mxl/workplace/gipylib
python3 tests/ins/test_lc_e2e.py
```
预期：生成 data/lc_output.csv，无接口错误。

### 步骤 3：精度验证
对比 data/lc_output.csv 与参考解 data/rtktcgps.rslt：
- 平面位置误差 max ≤ 0.5m（lat/lon 差 < 5e-6 度）
- 高程误差 max ≤ 1.0m
- 匹配率 ≥ 95%

使用现有 verify_lc.py（若存在）或手动对比。

### 步骤 4：回归确认
- 默认配置（estimate_leverarm=0 等）下，P 为 15×15，E2E 精度与之前一致（planar max=0.128m, elev max=0.372m, match rate 99.94%）
- 确认无残留 `gyro_scale`/`accel_scale`/`P1`/`P2`/`init_P1`/`init_P2` 引用

## 执行顺序

1. 修改 data_types.py（移除 InsState 比例因子字段）— 影响最广，先改
2. 修改 ins_update.py（移除比例因子使用）
3. 修改 initializer.py（移除比例因子 + 返回单 P）
4. 修改 config.yaml（移除比例因子配置）
5. 修改 lc_integration.py（文档注释）
6. 修改 4 个测试文件的 make_state（移除比例因子）
7. 修改 test_lc_integration.py（单滤波接口）
8. 修改 test_lc_e2e.py（单滤波接口）
9. 运行单元测试（步骤 1）
10. 运行 E2E 测试 + 精度验证（步骤 2-3）
11. 回归确认（步骤 4）
