# IMU-GNSS Interpolation Fusion Timing Design

## Goal

修复 IMU 速率数据跨越 GNSS 历元时的融合输出时序：LC 和 TC 均在线性插值出的 GNSS 时刻完成传播与量测更新，并将 Qins=3 状态写在该 GNSS 时刻，而不是写在后续约 0.006 秒的原始 IMU 历元。

## Current Finding

`src/core/ins/lc_integration.py` 和 `src/core/tc/tc_integration.py` 已经使用 `imu_interpolate_linear()` 生成 GNSS 时刻的 IMU。当前流程在插值时刻调用量测更新，但融合后没有立即写出；随后继续处理当前原始 IMU，并把 `last_qins=3` 带到了该原始 IMU 的输出。因此输入 GNSS 为整数秒时，结果文件中的 Qins=3 常见为 `整数秒+0.006`。

## Scope

### In scope

- LC 流式融合路径的 GNSS 插值、传播、更新和输出时序。
- TC 流式融合路径的 GNSS 插值、传播、更新和输出时序。
- Qins 标记与输出时间戳的一致性。
- 插值边界、相同时间戳和多个待处理 GNSS 的自动化测试。
- 使用 `data/rtk-ins紧组合.yaml` 的完整回放，以及 `data/plot/error-rslt(1).py` 的前后精度对比。

### Out of scope

- 不改变 `imu_interpolate_linear()` 的反距离线性插值公式。
- 不改变 GNSS 量测模型、EKF 矩阵、feedback、初始化算法或噪声参数。
- 不重构输入队列为全量事件排序器。
- 不强制修改外部结果文件格式；沿用现有 `RSLTWriter`。

## Data Flow

对于当前已处理到 `t_prev`、新到达 IMU 为 `t_cur` 且队首 GNSS 满足 `t_prev < t_gnss <= t_cur`：

1. 使用 `t_prev` 与 `t_cur` 的 IMU 线性插值，构造 `imu_gnss.timestamp == t_gnss`。
2. 调用 `time_update(imu_gnss)`，将名义状态和协方差传播到 `t_gnss`。
3. 执行 LC/TC GNSS 量测更新、反馈和约束相关的既有逻辑。
4. 量测更新成功或按既有逻辑完成后，立即写出融合后状态，强制使用 `qins=3` 且状态时间戳为 `t_gnss`。
5. 将内部处理游标设为 `imu_gnss`，继续用当前原始 IMU 从 `t_gnss` 传播到 `t_cur`。
6. 输出 `t_cur` 的机械编排状态，标记为 `qins=2`，除非现有约束逻辑在该历元产生新的量测更新。

特殊情况：

- `t_gnss == t_prev`：不重复传播，直接更新并输出一次 `t_gnss/Qins=3`。
- `t_gnss > t_cur`：GNSS 保留在 pending 队列，不提前更新。
- `t_gnss < t_prev`：保留现有防御性过期更新路径，但更新后立即以 GNSS 时间输出 Qins=3。
- 同一个 IMU 区间包含多个 GNSS：按队列顺序逐个插值、传播、更新、输出，每个 GNSS 只输出一个融合状态。
- 插值失败或时间不单调：保留当前安全行为，不消费 pending GNSS，并记录可诊断日志。

## Module Changes

### `src/core/ins/lc_integration.py`

- 在 `_apply_gnss_update()` 完成后由调用方立即写出融合状态。
- 将“插值时刻临时输出”与“融合后输出”分开，避免融合前状态继承 Qins=3。
- 保持约束更新、位置差分速度和 pending 队列现有行为。

### `src/core/tc/tc_integration.py`

- 在 `_trigger_meas()` 完成后由调用方立即写出 `t_gnss/Qins=3`。
- 删除或调整当前插值分支中融合前的错误状态输出。
- 保持 TC 量测构造、降级、模糊度和 feedback 行为不变。

### `src/log/rslt_writer.py`

- 原则上不改格式和时间转换；只验证其使用传入 `InsState.timestamp`。
- 若需要支持显式输出时间，使用现有 state timestamp，不在 writer 内重新推断 GNSS 时间。

## Tests

新增或扩展单元测试，使用 fake estimator/writer 或现有最小状态构造，验证：

1. LC：GNSS 在两条 IMU 之间时，融合输出记录的时间等于 GNSS 时间，Qins=3 不附着于下一条 IMU。
2. TC：同样验证原始观测触发路径，融合输出时间等于 `obsr.t`，Qins=3 不附着于下一条 IMU。
3. 两条路径均验证后续原始 IMU 仍继续传播，时间戳和 Qins=2 正常。
4. GNSS 恰好与当前 IMU 对齐时不重复传播、不重复输出。
5. 多个 pending GNSS 跨同一 IMU 区间时，每个 GNSS 各有一个按时间递增的 Qins=3 输出。
6. 现有插值数学测试继续通过，插值值和边界行为不变。

## Full-Run Acceptance

1. 运行 `python src/main.py data/rtk-ins紧组合.yaml`，确认生成对应 `rslt_filename` 文件且进程成功结束。
2. 对新结果运行 `python 'data/plot/error-rslt(1).py'`，记录 Qins=3 样本数、E/N/U/水平/三维 RMSE、最大值和 95% 分位。
3. 与修复前同配置结果比较：Qins=3 时间应回到 GNSS 历元（允许浮点格式误差，绝对误差不超过 `1e-3 s`）；精度指标不得恶化超过 1%。
4. 若完整回放因环境或已有基线问题失败，必须保留失败命令、日志和已完成的单元测试结果，不宣称全量验收通过。

## Risks and Rollback

- 主要风险是改变输出行数量或 Qins=3 统计样本选择。通过单元测试和回放前后样本数对比控制。
- 不改变滤波状态更新公式；若精度或时序回归失败，可只回退集成器中新增的“融合后立即写出”逻辑，保留插值函数本身。
