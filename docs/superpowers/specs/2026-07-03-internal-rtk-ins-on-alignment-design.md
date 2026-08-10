# Internal RTK + INS-on 数据对齐管线设计

## 1. 背景与目标

### 1.1 当前状态

项目支持两种已实现的运行模式：

- **external + ins.enabled=on**：ImuSensor + GnssSolSensor（读 .pos 文件）→ Logger + Aligner → AlignedWriter → `output/aligned.csv` ✅
- **internal + ins.enabled=off**：InternalGnssSensor（SPP/RTK 实时解算）→ SolutionLogger → SolutionWriter → `output/*.pos` ✅
- **internal + ins.enabled=on**：被 `config_loader.py` 第 88-92 行拦截抛 `NotImplementedError` 🚧

### 1.2 目标

启用 `internal + ins.enabled=on` 模式（RTK 实时解算 + IMU 流式读取），输出 aligned.csv 格式文件，验证实时 GNSS 解算结果与 IMU 数据的时间匹配。

**明确不在范围内**：INS 估计器（15 维 EKF 双滤波）、NHC/ZUPT、机械编排等。本设计仅实现数据对齐管线，为未来 INS 估计器预留数据通路。

### 1.3 数据范围

- IMU 数据（`data/cpt_imu.csv`）：week 2046, sow 357254.3 - 359817.3，256298 样本，100Hz
- GNSS 观测（`data/cpt0870.19o` + `data/cpt0870_base.19o`）：week 2046, sow 357453 - 359941，2489 历元，1Hz
- IMU 比 GNSS 早约 200s 开始（Aligner 会丢弃 GNSS 历元前的 IMU）
- IMU 比 GNSS 早约 124s 结束（GNSS 末段无 IMU 匹配，AlignedBlock 为空时跳过）

## 2. 设计方案

### 2.1 核心思路：复用 external 模式的对齐管线

external 模式的 Logger 已实现完整的对齐逻辑：
- `_drain_imu_queue()`：非阻塞搬运 IMU 到 aligner 缓冲
- `_wait_for_imu(gnss_timestamp + harvest_window)`：阻塞等待 IMU 数据到达
- `aligner.harvest(gnss)`：收割 [t_gnss, t_gnss+1.0s) 的 IMU 数据

将传感器源从 GnssSolSensor（读文件）替换为 InternalGnssSensor（实时解算），其余管线完全复用。

### 2.2 数据流

```
ImuSensor (100Hz, data/cpt_imu.csv) → imu_queue ─┐
                                                  ├→ Logger → Aligner → AlignedWriter → aligned_internal_rtk.csv
InternalGnssSensor (RTK, 1Hz) → gnss_queue ──────┘
```

Logger 线程从两个队列消费数据：
1. 优先排空 imu_queue 到 aligner.imu_buffer
2. 阻塞取一个 GNSS 历元（timeout=0.1s 适应 RTK 解算慢）
3. `_wait_for_imu` 等待 IMU 数据到达 gnss.timestamp + harvest_window
4. `aligner.harvest(gnss)` 收割对齐块
5. AlignedWriter 写入 CSV

### 2.3 并发正确性分析

| 场景 | 处理机制 |
|------|----------|
| IMU 流式快（100Hz）vs RTK 解算慢（~10-50ms/历元） | Logger 的 `gnss_queue.get(timeout=0.1)` 阻塞等待 GNSS，IMU 在 imu_queue 积压（maxsize=2000），不会丢数据 |
| IMU 早于 GNSS 启动（~200s） | Aligner.harvest 丢弃 `timestamp < t_gnss` 的首部 IMU |
| IMU 早于 GNSS 结束（~124s） | GNSS 历元到来时 imu_buffer 为空，harvest 返回 None，Logger 跳过该 GNSS 历元 |
| GNSS 历元间 IMU 积压 | 每次 harvest 后 imu_buffer 保留未收割的 IMU，下一历元继续收割 |

## 3. 改动清单

### 3.1 `src/utility/config_loader.py`

移除 `internal + ins.enabled=on` 的 `NotImplementedError` 拦截，改为校验 `ins.imu_data_path` 存在：

```python
# 修改前（第 88-92 行）：
if ins_enabled == "on":
    raise NotImplementedError(
        "INS estimator not implemented: gnss_source='internal' + "
        "ins.enabled='on' is reserved for future INS integration"
    )

# 修改后：
if ins_enabled == "on":
    if not cfg["ins"].get("imu_data_path"):
        raise ValueError(
            "ins.imu_data_path is required when gnss_source='internal' "
            "and ins.enabled='on'"
        )
```

### 3.2 `src/stream/factory.py`

新增 `internal + ins.enabled=on` 分支：

```python
elif gnss_source == "internal" and ins_enabled == "on":
    # internal + on: IMU 流式 + 内部 GNSS 实时解算 → 对齐输出
    imu_path = config["ins"]["imu_data_path"]
    sensors.append(ImuSensor(imu_path, imu_queue, control))
    from src.stream.internal_gnss_sensor import InternalGnssSensor
    sensors.append(InternalGnssSensor(config, gnss_queue, control))
```

### 3.3 `src/main.py`

新增 `internal + on` 路径，复用 external 模式的 Logger + Aligner + AlignedWriter：

```python
if gnss_source == "internal" and ins_enabled == "on":
    # 路径 C: 内部 GNSS 实时解算 + IMU 对齐输出
    sensors = SensorFactory.create_sensors(config, imu_queue, gnss_queue, control)
    filename = config["output"].get("aligned_filename", "aligned.csv")
    writer = AlignedWriter(
        output_dir=config["output"]["output_dir"],
        filename=filename,
    )
    aligner = Aligner(imu_dt=1.0 / config["ins"]["data_rate"])
    logger = Logger(imu_queue, gnss_queue, writer, aligner, control)
    return sensors, logger
```

### 3.4 `data/cfg_test_internal_rtk_aligned.yaml`（新建）

基于 `cfg_test_rtk.yaml` 的 GNSS 配置 + ins 配置：

```yaml
gnss:
  gnss_source: "internal"
  positioning_mode: "rtk"
  rover_path: "data/cpt0870.19o"
  base_path: "data/cpt0870_base.19o"
  eph_path: "data/brdm0870.19p"
  # ...（RTK 参数同 cfg_test_rtk.yaml）
ins:
  enabled: "on"
  imu_data_path: "data/cpt_imu.csv"
  data_rate: 100
output:
  output_dir: "output"
  aligned_filename: "aligned_internal_rtk.csv"
```

## 4. 验证方案

### 4.1 格式验证

运行 `python src/main.py data/cfg_test_internal_rtk_aligned.yaml`，检查输出：

1. **行结构**：G 行 11 列 + 紧跟 N 行 I 行（9 列），块状结构
2. **时间匹配**：每个 G 行后续 I 行的 sow 在 [t_gnss, t_gnss+1.0s) 范围内
3. **时间顺序**：G 行 sow 单调递增；每块内 I 行 sow 单调递增

### 4.2 位置一致性验证

对比 `output/aligned_internal_rtk.csv` 的 G 行位置与 `output/test_rtk.pos` 的位置：
- 平面误差 < 0.5m（lat/lon 差 < 5e-6 度）
- 高程误差 < 1m

### 4.3 对齐完整性验证

- 统计 G 行数量（应接近 2489，末段无 IMU 的历元会被跳过）
- 统计每块 I 行数量（应接近 100，对应 100Hz × 1.0s 窗口）

## 5. 不在范围内

- INS 估计器（15 维 EKF 双滤波）
- NHC/ZUPT 策略
- IMU 机械编排
- 初始对准
- 状态反馈机制

这些将在后续 INS 启用任务中实现，本设计仅打通数据对齐管线。
