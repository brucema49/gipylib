# GVINS 风格 IMU 数据消费流程 设计

> 参考 `tools/GVINS/estimator/src/estimator_node.cpp` 的 `process()` 函数（lines 321-378）IMU 消费模式，
> 重构 `LcIntegration.add_imu`：每取到一个 IMU 数据，先判断 GNSS 队头时间戳与 IMU 时间戳的关系，
> 若 `imu.t < gnss.t` 进行机械编排；若 `imu.t >= gnss.t` 线性插值到 GNSS 时间戳并触发融合。
> IMU 队列大小 200 条，GNSS 队列大小 3 个历元。
>
> **验证目标**：运行 `data/config.yaml`，生成 `data/output/RTK.pos` + `data/output/RTKLC.pos` + `data/output/aligned_internal_rtk.csv`，
> RTKLC.pos 整数秒 ECEF 解与 RTK.pos（纯 GNSS）整数秒解对比，平面误差 ≤0.5m、高程 ≤1m（沿用项目硬约束）。

---

## 1. 范围

**本次实现**：
- `src/main.py`：`imu_queue` 容量 2000→200，`gnss_queue` 容量 100→3
- `src/core/ins/interpolator.py`：新增 `imu_interpolate_linear()` GVINS 风格线性插值函数
- `src/core/ins/lc_integration.py`：重构 `add_imu()` 用 GVINS 风格插值替代最近邻匹配
- `src/log/logger.py`：简化 LC 喂入逻辑（移除 `_lc_imu_buffer` 中转缓冲，直接按时间顺序喂入）

**预留（不在本次）**：
- 紧组合扩展（事件循环骨架不变，仅替换 GNSS 来源）
- 多 GNSS 批量入队（当前仍按 1 GNSS/历元 节奏喂入）

**不动**：
- `LcStream`（`feed_imu`/`feed_gnss` 接口不变，内部调 `_integ.add_imu`/`add_gnss`）
- `LcEstimator`（`time_update`/`meas_update_pos`/`meas_update_vel`/`feedback` 不变）
- `InsUpdate`（机械编排核心不变，`_prev_timestamp` 自动跟踪）
- `Initializer`（初始化阶段也使用 `imu_interpolate_linear`，与主循环一致）

## 2. GVINS 参考模式

### 2.1 GVINS `process()` 核心循环（estimator_node.cpp lines 338-378）

```cpp
for (auto &imu_data : imu_msg) {
    double t = imu_data->header.stamp.toSec();
    double img_t = img_msg->header.stamp.toSec() + estimator_ptr->td;  // GNSS 时间
    if (t <= img_t) {
        // IMU 在 GNSS 之前 → 直接机械编排
        double dt = t - current_time;
        current_time = t;
        estimator_ptr->processIMU(dt, accel, gyro);
    } else {
        // IMU 在 GNSS 之后 → 线性插值到 GNSS 时间
        double dt_1 = img_t - current_time;   // [last_imu, gnss_t]
        double dt_2 = t - img_t;               // [gnss_t, curr_imu]
        current_time = img_t;
        double w1 = dt_2 / (dt_1 + dt_2);      // 前一历元权重
        double w2 = dt_1 / (dt_1 + dt_2);      // 当前历元权重
        // 在 gnss_t 处线性插值
        dx = w1 * dx + w2 * imu_data->linear_acceleration.x;
        dy = w1 * dy + w2 * imu_data->linear_acceleration.y;
        dz = w1 * dz + w2 * imu_data->linear_acceleration.z;
        rx = w1 * rx + w2 * imu_data->angular_velocity.x;
        ry = w1 * ry + w2 * imu_data->angular_velocity.y;
        rz = w1 * rz + w2 * imu_data->angular_velocity.z;
        // 用 dt_1 推进到 gnss_t
        estimator_ptr->processIMU(dt_1, interpolated_accel, interpolated_gyro);
    }
}
// IMU 循环结束后处理 GNSS 量测
if (GNSS_ENABLE && !gnss_msg.empty())
    estimator_ptr->processGNSS(gnss_msg);
```

### 2.2 关键不变量

1. **时间单调**：`current_time` 单调递增，`dt >= 0` 始终成立
2. **插值只在跨越 GNSS 时刻时发生**：仅当 `imu.t >= gnss.t` 且前一 IMU 在 `gnss.t` 之前
3. **插值后状态对齐 GNSS 时间**：`current_time = gnss_t`，下一 IMU 的 `dt = next_t - gnss_t`
4. **当前 IMU 不丢弃**：GVINS 中 `imu_buf.front()` 在 `getMeasurements` 中不被 pop（line 169），下一轮仍可用；本项目中等效为：插值后继续用当前 IMU 做下一段机械编排（`dt_2` 段）

### 2.3 本项目适配差异

| 方面 | GVINS | 本项目 |
|------|-------|--------|
| IMU 数据形式 | 速率式 (accel m/s², gyro rad/s) | 速率式（同） |
| 调用粒度 | 批量 imu_msg 列表 | 单条 `feed_imu(imu)` |
| 推进接口 | `processIMU(dt, accel, gyro)` | `time_update(imu)` 内部算 dt |
| GNSS 触发 | 循环结束后 `processGNSS` | 插值后立即 `meas_update_*` + `feedback` |
| 跨 GNSS 的 IMU 残段 | 留给下一轮处理 | 本轮立即处理 `dt_2` 段 |
| 队列容量 | imu_buf=2000, gnss=100 | imu_queue=200, gnss_queue=3 |

**残段处理选择**：GVINS 把 `[gnss_t, imu.t]` 段留给下一轮（因 `imu_buf.front()` 不 pop）。本项目 `feed_imu` 是单条推送，无法"留给下一轮"，因此本轮立即处理：
- 先 `time_update(interp@gnss_t)` 推进 `dt_1 = gnss_t - prev_t`
- 触发 GNSS 量测更新 + 反馈
- 再 `time_update(imu)` 推进 `dt_2 = imu.t - gnss_t`

这等价于 GVINS 的两轮处理（本轮处理 `dt_1`，下一轮处理 `dt_2`），但合并为单次调用，避免 IMU 数据重入。

## 3. 架构与模块

```
src/
├── main.py                              # 修改: Queue maxsize 2000→200, 100→3
├── core/ins/
│   ├── interpolator.py                  # 修改: 新增 imu_interpolate_linear()
│   └── lc_integration.py                # 重构: add_imu 用 GVINS 风格
└── log/
    └── logger.py                        # 简化: 移除 _lc_imu_buffer, 直接按时间顺序喂入
```

### 3.1 `interpolator.py` — 新增线性插值

保留现有 `imu_interpolate`（最近邻，初始化用），新增 `imu_interpolate_linear`：

```python
def imu_interpolate_linear(imu_pre: ImuMeasurement,
                           imu_cur: ImuMeasurement,
                           t_target: float) -> Optional[ImuMeasurement]:
    """GVINS 风格线性插值：在 t_target 处线性插值 IMU 数据。

    参考 estimator_node.cpp process() lines 361-374:
        w1 = dt_2 / (dt_1 + dt_2)  # imu_pre 权重
        w2 = dt_1 / (dt_1 + dt_2)  # imu_cur 权重
        accel = w1 * imu_pre.accel + w2 * imu_cur.accel
        gyro  = w1 * imu_pre.gyro  + w2 * imu_cur.gyro

    Args:
        imu_pre: 前一 IMU (timestamp <= t_target)
        imu_cur: 当前 IMU (timestamp >= t_target)
        t_target: 插值目标时刻（通常为 GNSS 时间戳）

    Returns:
        t_target 时刻的 ImuMeasurement（线性插值数据），或 None（区间不包含 t_target）
    """
    t0, t2 = imu_pre.timestamp, imu_cur.timestamp
    if not (t0 <= t_target <= t2) or t2 <= t0:
        return None
    dt_1 = t_target - t0
    dt_2 = t2 - t_target
    w1 = dt_2 / (dt_1 + dt_2)
    w2 = dt_1 / (dt_1 + dt_2)
    return ImuMeasurement(
        timestamp=t_target,
        week=imu_pre.week,
        accel=w1 * imu_pre.accel + w2 * imu_cur.accel,
        gyro =w1 * imu_pre.gyro  + w2 * imu_cur.gyro,
    )
```

### 3.2 `lc_integration.py` — `add_imu` 重构

**替换**：原 `_process_pending` 最近邻匹配 → GVINS 风格插值触发

**新 `add_imu` 流程**：

```python
def add_imu(self, imu: ImuMeasurement) -> None:
    """GVINS 风格 IMU 消费：每条 IMU 检查 GNSS 队头时间戳。

    - imu.t < gnss.t：直接机械编排（无 GNSS 触发）
    - imu.t >= gnss.t 且 imucur.t <= gnss.t：
        1) 线性插值到 gnss.t
        2) time_update(interp) 推进 dt_1 = gnss.t - imucur.t
        3) meas_update_pos/vel(gnss) + feedback  触发融合
        4) 弹出 gnss，循环处理后续 GNSS（多个 GNSS 落在同一 IMU 区间时）
        5) 继续用当前 imu 推进 dt_2 = imu.t - gnss.t
    """
    if self.imucur is None:
        self.imucur = imu
        self._static_detect.push(imu)
        return

    cur = self.imucur  # 当前已推进到的 IMU（最初为 self.imucur）

    # 1. 处理所有落入 [cur.t, imu.t] 区间的 GNSS
    while self.pending_gnss:
        gnss = self.pending_gnss[0]
        t_gnss = gnss.timestamp

        if t_gnss < cur.timestamp:
            # GNSS 已过期（比 cur 还早）：直接在 cur 时刻做量测更新
            # （理论上不应发生，因 Logger 按时间顺序喂入；防御性处理）
            self._apply_gnss_update(gnss)
            self.pending_gnss.popleft()
            continue

        if t_gnss > imu.timestamp:
            # GNSS 在当前 IMU 之后：留给后续 IMU 处理
            break

        # cur.t <= t_gnss <= imu.t：GVINS 风格插值触发
        if t_gnss == cur.timestamp:
            # GNSS 恰好对齐 cur：无需插值，直接触发
            interp = cur
        else:
            # 线性插值到 t_gnss
            interp = imu_interpolate_linear(cur, imu, t_gnss)
            if interp is None:
                break
            # 推进 dt_1 = t_gnss - cur.t
            self.imupre = cur
            self.imucur = interp
            self.est.time_update(interp)
            self._static_detect.push(interp)
            self._apply_constraints(interp)

        # 触发 GNSS 量测更新 + 反馈
        self._apply_gnss_update(gnss)
        self.pending_gnss.popleft()
        cur = interp  # 后续 GNSS 从 interp 时刻继续

    # 2. 推进当前 IMU（dt = imu.t - cur.t）
    self.imupre = cur
    self.imucur = imu
    self.est.time_update(imu)
    self._static_detect.push(imu)
    self._apply_constraints(imu)
```

**关键不变量**：
- `imucur.timestamp` 始终等于 `est.state.timestamp`（机械编排自动同步）
- GNSS 触发后状态停留在 `gnss.t`，当前 IMU 的 `dt_2 = imu.t - gnss.t` 由 `time_update(imu)` 推进
- 多个 GNSS 落在同一 IMU 区间时按时间顺序逐个处理（while 循环）
- `_apply_constraints` 在每次 mechanization 后调用（保持原 NHC/ZUPT/ZARU decimation 语义）

**移除**：`_process_pending`、`pre_advance`/`post_advance` 双段逻辑（GVINS 风格统一为单段触发）

### 3.3 `logger.py` — 简化 LC 喂入

**移除**：`_lc_imu_buffer: deque`（中转缓冲）

**原逻辑**（复杂）：
1. drain IMU → `_lc_imu_buffer`
2. 取 GNSS → 写 RTK.pos
3. 等待 IMU 覆盖 `gnss.t + harvest_window` → 追加到 `_lc_imu_buffer`
4. 从 `_lc_imu_buffer` 按 `gnss.t` 分前后两段喂入 `LcStream`

**新逻辑**（简化）：
1. drain IMU → 直接 `lc_stream.feed_imu(imu)`（按时间顺序）
2. 取 GNSS → 写 RTK.pos
3. 等待 IMU 覆盖 `gnss.t + harvest_window` → 期间取到的 IMU 直接 `feed_imu`
4. `lc_stream.feed_gnss(gnss)`（入 pending_gnss 队列）
5. drain 剩余 IMU → 直接 `feed_imu`（这些 IMU 触发 GNSS 处理）

**关键点**：
- IMU 按到达顺序直接喂入 `LcStream`，无需中转缓冲
- GNSS 必须在 IMU 跨越 `gnss.t` 之前入队（由 `_wait_for_imu` 保证）
- `LcIntegration.add_imu` 内部检查 `pending_gnss` 队头，自动触发插值+融合

```python
# Logger.run() 核心循环（简化版）
while self.control.is_running():
    # 1. drain IMU → 直接喂入 LcStream
    new_imus = self._drain_imu_queue()
    if self.lc_stream is not None:
        for imu in new_imus:
            self.lc_stream.feed_imu(imu)

    # 2. 取一个 GNSS（阻塞）
    try:
        gnss = self.gnss_queue.get(timeout=0.1)
    except Empty:
        continue
    if gnss is None:  # EOF
        # 喂入剩余 IMU
        remaining = self._drain_imu_queue()
        if self.lc_stream is not None:
            for imu in remaining:
                self.lc_stream.feed_imu(imu)
        break
    if isinstance(gnss, SensorData):
        gnss = gnss.gnss_solution

    # 3. 写 RTK.pos
    if self.gnss_writer is not None:
        self.gnss_writer.write(gnss)

    # 4. 等待 IMU 覆盖 harvest 窗口（期间喂入 LcStream）
    wait_imus = self._wait_for_imu(gnss.timestamp + self.aligner.harvest_window)
    if self.lc_stream is not None:
        for imu in wait_imus:
            self.lc_stream.feed_imu(imu)

    # 5. GNSS 入队 LcStream（pending_gnss）
    if self.lc_stream is not None:
        self.lc_stream.feed_gnss(gnss)

    # 6. harvest 对齐块 → 写 CSV
    aligned = self.aligner.harvest(gnss)
    if aligned is not None:
        self.writer.write(aligned)
```

### 3.4 `main.py` — 队列容量

```python
# 原
imu_queue = Queue(maxsize=2000)
gnss_queue = Queue(maxsize=100)
# 新
imu_queue = Queue(maxsize=200)   # 200 条 IMU = 2s @100Hz
gnss_queue = Queue(maxsize=3)    # 3 个 GNSS 历元 = 3s @1Hz
```

**风险**：队列变小后，若 Logger 处理速度跟不上传感器生产速度，传感器线程会阻塞在 `queue.put()`。但本项目为离线处理（从文件读），传感器生产速度 ≫ Logger 消费速度时阻塞是正常的反压机制，不影响正确性。

## 4. 数据流（internal+on 模式）

```
┌─────────────┐   imu_queue(200)   ┌──────────┐
│ ImuSensor   ├───────────────────▶│          │
└─────────────┘                    │  Logger  │  feed_imu(imu)   ┌──────────┐
                                   │  Thread  ├─────────────────▶│          │
┌─────────────┐  gnss_queue(3)     │          │  feed_gnss(gnss) │ LcStream │
│ GnssSensor  ├───────────────────▶│          ├─────────────────▶│   ↓      │
└─────────────┘                    │          │                  │ LcIntegration
                                   │          │                  │   ↓      │
                                   │          │  harvest aligned │  add_imu │
                                   │  Aligner  │◄─────────────────│  (GVINS) │
                                   │          │                  │   ↓      │
                                   │          │                  │ LcEstimator
                                   │          │                  │   ↓      │
                                   │          │  write RTKLC.pos │          │
                                   │          │◄─────────────────│          │
                                   └────┬─────┘                  └──────────┘
                                        │ write RTK.pos
                                        │ write aligned.csv
                                        ▼
                                   ┌──────────┐
                                   │  output/ │
                                   └──────────┘
```

**时序**（典型一秒）：
1. ImuSensor 生产 100 条 IMU → imu_queue（部分阻塞，等 Logger 消费）
2. GnssSensor 生产 1 条 GNSS → gnss_queue
3. Logger drain IMU → `feed_imu`（推进前 99 条，t < gnss.t）
4. Logger 取 GNSS → 写 RTK.pos
5. Logger 等待 IMU 覆盖 `gnss.t + harvest_window`（取第 100 条 + 下一秒前几条）
6. Logger `feed_gnss(gnss)` → pending_gnss
7. Logger drain 下一批 IMU → `feed_imu`（第一条 t >= gnss.t 触发插值+融合）
8. Logger harvest aligner → 写 aligned.csv

## 5. 边界条件与错误处理

| 情况 | 处理 |
|------|------|
| IMU 队列耗尽（EOF） | Logger 喂入剩余 IMU，`LcStream.finalize()` 刷出 pending 输出 |
| GNSS 队列耗尽（EOF） | Logger 喂入剩余 IMU（无新 GNSS 触发），finalize |
| `imu.t == gnss.t` | 直接用当前 IMU 触发（无需插值），`interp = cur` |
| `gnss.t < cur.t`（过期 GNSS） | 防御性直接量测更新（不插值），记 warning |
| `gnss.t > imu.t` | 留给后续 IMU（break while 循环） |
| 多 GNSS 落在同一 IMU 区间 | while 循环逐个插值触发 |
| `dt <= 0`（时间戳非单调） | `time_update` 内部已有 warning + skip |
| pending_gnss 为空 | 直接机械编排（无融合触发） |

## 6. 验证计划

### 6.1 运行验证

```bash
cd /home/mxl/workplace/gipylib
python3 src/main.py data/config.yaml
```

**预期输出**：
- `data/output/RTK.pos`（纯 GNSS 解算）
- `data/output/RTKLC.pos`（松组合 EKF）
- `data/output/aligned_internal_rtk.csv`（对齐块状 CSV）

### 6.2 正确性验证

1. **文件完整性**：三个输出文件均生成，行数 > 0
2. **时间对齐**：RTKLC.pos 与 RTK.pos 整数秒时间戳一致
3. **精度对比**（项目硬约束）：
   - 平面位置误差 ≤ 0.5m（lat/lon 差 < 5e-6 度）
   - 高程误差 ≤ 1m
4. **运行无异常**：无 Python traceback，无非预期 warning
5. **运行时长**：记录终端输出的 `运行时长: Xs`

### 6.3 回归验证

对比本次 RTKLC.pos 与上一次运行（最近邻策略）的输出：
- 整数秒 ECEF 解差异应在合理范围（GVINS 插值 vs 最近邻，差异 < 0.1m 量级）
- 若差异过大，检查插值实现是否正确

## 7. 测试文档

详见 `docs/superpowers/plans/2026-07-18-gvins-style-imu-consumption-test.md`。

## 8. 风险与缓解

| 风险 | 缓解 |
|------|------|
| 队列变小导致传感器线程频繁阻塞 | 离线处理可接受；若需在线，恢复大队列 |
| GVINS 插值引入数值误差 | 100Hz IMU 下插值误差 < 5ms，由 KF time_sync 状态吸收 |
| 多 GNSS 同区间处理顺序错误 | while 循环按 `pending_gnss` FIFO 顺序处理，保证时间单调 |
| `feed_gnss` 时机晚于 IMU 跨越 gnss.t | Logger `_wait_for_imu` 保证 IMU 覆盖窗口后才 `feed_gnss` |
| 初始化阶段 IMU 对齐 | 与主循环一致使用 `imu_interpolate_linear`，统一时间对齐策略 |
