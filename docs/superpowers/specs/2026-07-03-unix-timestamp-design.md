# 统一 Unix 时间戳系统设计

## 背景

rtklib-py 的 `gtime_t.time` 字段存储 Unix 秒（从 1970-01-01 00:00:00 UTC 起算），
`gtime_t.sec` 存储不足秒的小数部分。`epoch2time` 通过 `(year-1970)*365 + ...` 计算天数，
证明 `time` 字段就是 Unix 秒。

本项目当前 `GnssSolution.timestamp` 和 `ImuMeasurement.timestamp` 存储的是 GPS 周内秒（sow），
与 rtklib-py 内部时间表示不一致。需要统一为 Unix 时间戳。

## 方案

### 核心原则

`timestamp` 字段统一为 Unix 时间戳（float 秒，= `gtime_t.time + gtime_t.sec`）。
`week` 字段保留作为派生便利字段（由 timestamp 推导）。

### 改动清单

1. **`src/core/time_utils.py`** — 新增转换函数：
   - `GPST_EPOCH_UNIX = 315964800`（1980-01-06 UTC 的 Unix 时间戳）
   - `gpst_to_unix(week, sow) -> float`：GPS 周+周内秒 → Unix 时间戳
   - `unix_to_gpst(unix_ts) -> (week, sow)`：Unix 时间戳 → (GPS 周, 周内秒)

2. **`src/core/data_types.py`** — 更新注释：
   - `ImuMeasurement.timestamp`：`# Unix 时间戳（秒）`
   - `GnssSolution.timestamp`：`# Unix 时间戳（秒）`

3. **`src/core/gnss/solution_converter.py`** — 直接使用 Unix 时间戳：
   ```python
   unix_ts = sol.t.time + sol.t.sec
   week, _ = unix_to_gpst(unix_ts)
   return GnssSolution(timestamp=unix_ts, week=week, ...)
   ```

4. **`src/log/solution_writer.py`** — 输出时转换回 (week, sow)：
   ```python
   week, sow = unix_to_gpst(sol.timestamp)
   # 写入 week, sow
   ```

5. **`src/log/aligned_writer.py`** — 输出时转换回 (week, sow)：
   ```python
   week, sow = unix_to_gpst(g.timestamp)
   # 写入 week, sow
   ```

6. **`src/stream/formators.py`** — 输入时转换为 Unix：
   - `ImuFormator`: `timestamp = gpst_to_unix(week, sow)`
   - `PosSolFormator`: `timestamp = gpst_to_unix(week, sow)`

### 测试更新

所有测试 fixture 中的 `timestamp=sow` 改为 `timestamp=gpst_to_unix(week, sow)`，
验证点改为检查 Unix 时间戳或输出文件中的 (week, sow)。

## 验证

运行 `verify_rtklib_py spp/rtk` 生成 reference，运行 gipylib 出 test 输出，
用 `compare_pos` 逐历元比对，确保 SPP/RTK 结果与 rtklib-py 一致
（平面 < 0.5m，高程 < 1m）。
