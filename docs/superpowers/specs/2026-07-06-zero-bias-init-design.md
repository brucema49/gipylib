# IMU 零偏参数初始化设计

> 日期: 2026-07-06
> 状态: 已批准
> 参考: tools/gnss_ins_lc_nhc `StartAligning` (navinitialized.cc 行 67-230)

## 背景

分析 `tools/gnss_ins_lc_nhc` 发现该项目从 config 读取非零的 IMU 零偏和比例因子进行初始化。本项目已有零偏读取代码框架，但存在两个差距：

1. **单位转换常量错误**: `constant_mGal` 用了标准定义 `1e-5`，而 gnss_ins_lc_nhc 用 `1e-6 × g0`
2. **比例因子未读取**: `gyro_scale` / `accel_scale` 硬编码为 0，未从 config 读取
3. **无静态标定接口**: 无法在未来扩展静态标定功能

## 设计

### 1. 单位转换常量修正

与 gnss_ins_lc_nhc `constant.hpp` (行 20-36) 完全对齐:

| 常量 | 值 | 用途 |
|------|-----|------|
| `constant_g0` | 9.7803267715 | 重力加速度 |
| `dh2rs` | π/180/3600 ≈ 4.848e-6 | deg/hour → rad/s |
| `constant_mgal` | 1e-6 × g0 ≈ 9.78e-6 | mGal → m/s² |
| `constant_ppm` | 1e-6 | ppm → dimensionless |

### 2. `_assemble_state` 改动

- 零偏: 优先使用静态标定结果 (`_calibrated_gyro_bias`), 否则从 config 读取 `initial_gyro_bias` / `initial_acce_bias`
- 比例因子: 从 config 读取 `initial_gyro_scale` / `initial_acce_scale` (当前值为 0)

### 3. 静态标定接口 (预留)

新增 `_calibrate_bias_from_static_imu(imu_list, gnss)` 方法:
- 当前返回 `None` (不标定)
- 未来可实现: 静止时陀螺平均 = 地球自转分量 + 零偏, 据此估计零偏
- 参考: gnss_ins_lc_nhc `AcceLeveling` 计算了平均陀螺但未用于零偏估计

### 4. 集成点

- `_align_static`: 调用 `_calibrate_bias_from_static_imu`, 存储结果到 `_calibrated_gyro_bias` / `_calibrated_accel_bias`
- `_assemble_state`: 静态模式优先使用标定结果
- `reset_gnss_buffer`: 清空标定结果

### 5. 配置

`data/config.yaml` 保持 `initial_gyro_bias` / `initial_acce_bias` / `initial_gyro_scale` / `initial_acce_scale` 值为 `[0, 0, 0]` (传感器不同, 依赖 EKF 在线估计)。

## 验证

- 运行 `test_static_init.py` / `test_velocity_init.py` / `test_position_diff_init.py`
- 验证比例因子读取不影响现有行为 (config 值为 0)
- 验证静态标定接口返回 None 时不影响现有流程
