# 松组合 (LC) 优化设计

## 背景

issue/7-30松组合.md 要求:
- SPP-INS LC RMSE < 2m (当前 10.55m)
- RTK-INS LC RMSE < 1m (当前 5.70m)
- 预初始化阶段只输出纯 GNSS (Qins=0), IMU 不解算
- 缓存 5 个动态 GNSS 历元, 首尾位置差分得 yaw
- RTK 位置方差需提高
- 杆臂参数必须考虑 (已启用 estimate_leverarm=1)

## 根因

1. **静态回退初始化**: `LcStream._try_init` / `LcRunner._initialize` 在首历元(车静止)
   立即触发静态初始化, yaw=0 (真值≈290°), EKF 无法快速收敛。
2. **预初始化无输出**: 预初始化阶段缓冲所有数据, 不输出 Qins=0 纯 GNSS 结果。
3. **位置差分缓冲区过小**: `gnss_buffer_size=3`, issue 要求 5 历元。
4. **RTK 位置方差过小**: FIX sigma=0.15, RTK 平面精度 0.3m, 需增大。

## 设计

### 1. 预初始化阶段: 纯 GNSS 输出 (Qins=0)

**位置**: `src/core/ins/lc_stream.py`

- `feed_gnss`: 预初始化时调用 `writer.write_gnss_only()` 输出 1Hz 纯 GNSS (Qins=0)
- `feed_imu`: 预初始化时直接丢弃 IMU (不解算, 不缓冲)
- 移除 `_init_imu` 缓冲, 仅保留 `_init_gnss` 缓冲供动态初始化

**参考**: issue 第 2 行 "一条一条的弹出GNSS观测进行纯GNSS解算, imu数据直接弹出, 不用进行解算"

### 2. 移除静态回退, 仅动态初始化

**位置**: `src/core/ins/lc_stream.py` `_try_init`, `src/core/ins/lc_runner.py` `_initialize`

- 移除 `InitMode.STATIC` 回退分支
- 仅 `POSITION_DIFF` 模式 (RTK) 或 `VELOCITY_VECTOR` 模式 (SPP) 初始化
- 初始化条件: 连续 5 个 GNSS 历元平面差分速度均 > 阈值

### 3. 位置差分缓冲区: 3 → 5 历元

**位置**: `data/spp-ins-lc.yaml`, `data/config.yaml`

- `gnss_buffer_size: 3 → 5` (5 历元 span=4s, 首尾差分)
- `InsInitializer._align_motion_displacement` 已支持缓冲区逻辑, 无需改代码

### 4. 提高 RTK 位置方差

**位置**: `src/core/ins/lc_estimator.py` `_gnss_pos_std`

- FIX: 0.15 → 0.3 (RTK 平面精度 0.3m, 增大给 EKF 惯性平滑 false fix)
- FLOAT: 0.05 → 0.1
- DGPS: 1.0 (不变)
- SPP: 10.0 (不变)

### 5. 杆臂-姿态耦合 (已实现)

`lc_estimator.py` H 矩阵已含:
- 位置: `H[lever_arm] = -C_b_e`
- 速度: `H[att] = -skew(C_b_e @ leverarm)` (杆臂非零时)

config `estimate_leverarm=1` 已启用, 无需修改。

## 验证

- `python3 src/main.py data/spp-ins-lc.yaml && python3 _eval_spplc.py` → RMSE < 2m
- `python3 src/main.py data/config.yaml && python3 _eval_rtklc.py` → RMSE < 1m
- 检查 rslt 中 Qins=0 (预初始化), Qins=3 (量测更新) 分布
