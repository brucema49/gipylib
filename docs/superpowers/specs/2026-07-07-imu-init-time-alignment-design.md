# IMU 初始化时间对齐策略修订设计

> 日期: 2026-07-07
> 状态: 已批准（用户授权自主执行）
> 关联: 2026-07-07-mechanization-design.md（机械编排设计，需保证兼容）

## 1. 背景与动机

当前 `src/core/ins/interpolator.py` 实现了速率式 IMU 线性插值，文档（`skills/初始化.md` 第 4 节）声称参考 `tools/gnss_ins_lc_nhc` 的 `SortData()` 方法。经调查发现此参考依据**存在根本性错误**：

### 1.1 gnss_ins_lc_nhc 使用增量式 IMU

`ImuData` 类的字段 `gyro_`/`acce_` **实为角度增量(dtheta)/速度增量(dvel)**，非角速度/加速度：

- `navmech.cc` 第 52 行：`wibb_ = gyro_ / dt`（除以 dt 才得到角速度）
- `navmech.cc` 第 53 行：`fibb_ = acce_ / dt`（除以 dt 才得到加速度）
- `navdataque.cc` `SortData()` 第 80-83 行做的是**增量切分**：
  ```cpp
  middle_imu.acce_ = imu.acce_ * dt;   // 按比例分割增量
  imu.acce_ -= middle_imu.acce_;        // 剩余保留在 imu 中
  ```
- 增量切分法**不适用于速率式数据**（速率式应做线性插值而非分割）

### 1.2 GINav 也使用增量式 IMU 且初始化无时间插值

- `ins_mech.m` 使用 `imu.dw`/`imu.dv`（增量式）
- `ins_init.m` 直接从 `avp0` 初始化，**无 IMU 插值**
- `gnss_ins_lc.m` 的 LC 量测更新直接用当前 INS 状态，**无 IMU/GNSS 时间对齐**

### 1.3 结论

两个参考项目均不提供适用于速率式 IMU 的初始化时间对齐方案。按照用户指令，改为**GNSS 时间最近邻匹配 IMU 数据**，时间对齐误差后续放入卡尔曼滤波估计。

## 2. 改动范围

| 文件 | 改动类型 | 说明 |
|------|---------|------|
| `src/core/ins/interpolator.py` | 修改 | `imu_interpolate` 改为最近邻匹配；`is_to_update` 简化 |
| `src/core/ins/initializer.py` | 修改 | 方法名 `_interpolate_imu` → `_align_imu_to_gnss` |
| `skills/初始化.md` 第 4 节 | 重写 | 说明参考项目均为增量式不适用，改为最近邻策略 |
| `skills/imu.md` 第 5 节 | 更新 | 同步插值策略描述 |
| `skills/estimator.md` | 备注 | 时间对齐误差作为 KF 状态参数（预留） |

## 3. 核心算法

### 3.1 最近邻匹配（替代线性插值）

```python
def imu_interpolate(imu_pre, imu_cur, t_gnss):
    """最近邻匹配：返回时间戳最接近 t_gnss 的 IMU 历元（不插值）。

    时间对齐误差（最大半个 IMU 采样周期 ≈ 5ms @100Hz）后续由 KF 估计。
    """
    dt_pre = abs(imu_pre.timestamp - t_gnss)
    dt_cur = abs(imu_cur.timestamp - t_gnss)
    nearest = imu_pre if dt_pre <= dt_cur else imu_cur
    return ImuMeasurement(
        timestamp=t_gnss,       # 用 GNSS 时间戳标记
        week=nearest.week,
        accel=nearest.accel.copy(),
        gyro=nearest.gyro.copy(),
    )
```

### 3.2 is_to_update 简化

```python
def is_to_update(t0, t2, t_gnss, threshold=1e-3):
    """判断 GNSS 时间戳与 IMU 时间区间的关系（最近邻版）。

    Returns:
        0: t_gnss 不在 [t0, t2] 之间
        1: t_gnss 靠近 t0（最近邻为 imu_pre）
        2: t_gnss 靠近 t2（最近邻为 imu_cur）
        3: t0 < t_gnss < t2（在区间内，选最近邻）
    """
    if t0 <= t_gnss <= t2:
        dt0 = abs(t0 - t_gnss)
        dt2 = abs(t2 - t_gnss)
        return 1 if dt0 <= dt2 else 2
    return 0
```

### 3.3 find_bracket_imus 不变

仍用于在 `imu_list` 中定位包夹 `t_gnss` 的 IMU 历元对，返回 `(imu_pre, imu_cur, case)`。

## 4. 不变的部分

- `find_bracket_imus` 接口和逻辑不变
- 初始化三种模式（静态/速度矢量/位置差分）逻辑不变
- 三阈值检验逻辑不变
- 数据对齐管线（路径 C）不变
- 机械编排设计（2026-07-07-mechanization-design.md）不依赖插值模块，兼容

## 5. 时间对齐误差与 KF（预留）

时间对齐误差 `δt = t_imu_nearest - t_gnss` 最大为半个 IMU 采样周期（@100Hz → 5ms）。此误差的影响：
- 位置误差：`δp = v * δt`（@30m/s → 0.15m）
- 速度误差：`δv = a * δt`（@1g → 4.9mm/s）

后续在 EKF 状态向量中增加时间偏差参数 `δt`，通过 GNSS 量测更新在线估计。**当前不实现，仅在文档中预留接口说明。**

## 6. 测试验证

1. 运行 `tests/ins/test_static_init.py` — 静态初始化测试
2. 运行 `tests/ins/test_velocity_init.py` — 速度矢量初始化测试
3. 运行 `tests/ins/test_position_diff_init.py` — 位置差分初始化测试
4. 运行 `tests/test_formators.py` — 数据流测试
5. 运行 internal+on 模式 — 对齐管线端到端验证

## 7. 风险评估

- **低风险**：最近邻匹配比线性插值更简单，不会引入新 bug
- **时间精度损失**：最大 5ms（@100Hz），对初始化精度影响可忽略
- **后续 KF 扩展**：预留接口，不影响当前实现
