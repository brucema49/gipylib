# 组合导航 RSLT 输出格式设计

> 参考 `tools/ignav/ins-gnss/solution.cc` 的 `outins` 函数 (lines 1490-1582) 输出策略，
> 重构组合导航输出：组合导航模式 (`internal+on`) 下输出文件改为 `.rslt` 后缀，
> 输出频率提升至 IMU 频率 (100Hz)，字段追加 ECEF 速度、姿态 (roll/pitch/yaw) 及对应协方差。
> 纯 GNSS 模式 (`internal+off`) 保留现有 `.pos` 格式不变。
>
> **验证目标**：运行 `data/config.yaml`，生成 `data/output/RTKLC.rslt`，
> 与参考文件 `data/rtktcgps.rslt` 重叠历元对比，平面位置误差 ≤ 0.5m、高程 ≤ 1m。

---

## 1. 范围

**本次实现**：
- `src/log/rslt_writer.py`：新增 `RSLTWriter` 类（ignav outins XYZ + 速度 + 姿态）
- `src/core/ins/lc_integration.py`：新增 `last_qins` 属性跟踪更新类型
- `src/core/ins/lc_stream.py`：重构输出逻辑，移除 1Hz 整数秒缓冲，改为 per-IMU 直接写
- `src/main.py`：`internal+on` 模式下 `lc_writer` 改用 `RSLTWriter`
- `data/config.yaml`：新增 `rslt_filename` 配置项

**不动**：
- `src/log/solution_writer.py`：纯 GNSS 路径仍用 `.pos` 格式
- `src/log/aligned_writer.py`：对齐块 CSV 输出不变
- `src/log/logger.py`：事件循环骨架不变（`_lc_imu_buffer` 仍按 GNSS 节奏喂入 LcStream）
- `src/core/ins/lc_estimator.py`：`time_update` / `meas_update_pos` / `meas_update_vel` / `feedback` 不变
- `src/core/ins/ins_update.py`：机械编排不变（`att_rpy` 在 update 末尾已同步）

**预留（不在本次）**：
- 紧组合扩展
- ignav outins 的 vb/ab/bg/imuraw 等扩展字段

## 2. 参考文件分析

### 2.1 `data/rtktcgps.rslt` 格式（236,031 行 @ 100Hz）

```
% (x/y/z-ecef=WGS84,Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,ns=# of satellites),Qins=1:ins mechanization,2:ins mechanization and propagate states and covariance,3:ins-gnss loosely-coupled updates
%  GPST              x-ecef(m)      y-ecef(m)      z-ecef(m)   Q Qins  ns     sdx(m)     sdy(m)     sdz(m)    sdxy(m)    sdyz(m)    sdzx(m) age(s)  ratio    vx(m/s)    vy(m/s)    vz(m/s)       sdvx       sdvy       sdvz      sdvxy      sdvyz      sdvzx
2046 357456.996  -2408694.5292   4698106.3328   3566698.2024   2    0   6     1.4498     2.1271     2.2151    -1.3894     1.4960    -1.0131   0.00    0.0   -0.05412    0.75136    0.78820    9.49217    9.49338    9.49361   -0.13587    0.14542   -0.09823
```

**字段**（24 列）：
- `week` `sow`：GPST 时间
- `x` `y` `z`：ECEF 位置 (m)
- `Q`：GNSS quality (1=FIX, 2=FLOAT, ...)
- `Qins`：INS quality (0=未运行 / 1=mech / 2=mech+propagate / 3=LC update)
- `ns`：卫星数
- `sdx` `sdy` `sdz`：ECEF 位置 std (m)
- `sdxy` `sdyz` `sdzx`：ECEF 位置交叉协方差（**带符号**）
- `age` `ratio`：RTK age / ratio
- `vx` `vy` `vz`：ECEF 速度 (m/s)
- `sdvx` `sdvy` `sdvz`：ECEF 速度 std (m/s)
- `sdvxy` `sdvyz` `sdvzx`：ECEF 速度交叉协方差（**带符号**）

**观察**：
- 参考文件所有行 `Qins=0`，但速度 std 从 9.49 → 0.005 收敛，说明 EKF 实际运行，Qins 未跟踪
- 100Hz 输出（行间 0.01s），由 INS 机械编排推进位置/速度
- 无姿态列 —— 本次设计新增

### 2.2 ignav `outins` 函数（solution.cc L1490-1582）

XYZ 模式（`SOLF_XYZ`）字段顺序：
```
GPST  x y z  Q ista ns  sdx sdy sdz sdxy sdyz sdzx  age ratio  [vx vy vz sdvx sdvy sdvz sdvxy sdvyz sdvzx]  [roll pitch yaw sdroll sdpitch sdyaw]
```
- 速度字段在 `opt->outvel` 时输出
- 姿态字段在 `opt->outatt` 时输出，单位 deg（`sol->att[i] * R2D`）
- 姿态 std 同样 deg（`SQRT(sol->qa[i]) * R2D`）

### 2.3 本项目适配

| 方面 | ignav | 本项目 |
|------|-------|--------|
| 位置 | ECEF (m) | ECEF (`state.pos_e`) |
| 速度 | ECEF (m/s) | ECEF (`state.vel_e`) |
| 姿态 | rad → deg | `state.att_rpy` (rad) → deg |
| 位置协方差 | `Pp[9]` (3x3 ECEF) | `P[si.pos:pos+3, si.pos:pos+3]` |
| 速度协方差 | `Pv[9]` | `P[si.vel:vel+3, si.vel:vel+3]` |
| 姿态协方差 | `sol->qa[9]` | `P[si.att:att+3, si.att:att+3]` |
| ista | base sat idx | 填 0（无） |
| age/ratio | sol->age/ratio | 填 0（LC 不跟踪） |
| Qins | sol->qins | `LcIntegration.last_qins` |

## 3. 架构与模块

```
src/
├── main.py                              # 修改: lc_writer 改用 RSLTWriter
├── log/
│   └── rslt_writer.py                   # 新增: RSLTWriter 类
├── core/ins/
│   ├── lc_integration.py                # 修改: 新增 last_qins 属性
│   └── lc_stream.py                     # 重构: 移除 1Hz 缓冲, per-IMU 写出
└── data/config.yaml                     # 新增: rslt_filename 配置项
```

### 3.1 `rslt_writer.py` — 新增 RSLTWriter

```python
class RSLTWriter(WriterBase):
    """ignav outins 风格 .rslt 输出器 (XYZ ECEF + 速度 + 姿态)。

    输出格式 (参考 tools/ignav/ins-gnss/solution.cc outins L1531-1549):
      表头 + 每历元一行: week sow x y z Q Qins ns
                        sdx sdy sdz sdxy sdyz sdzx age ratio
                        vx vy vz sdvx sdvy sdvz sdvxy sdvyz sdvzx
                        roll pitch yaw sdroll sdpitch sdyaw
    位置/速度用 ECEF, 姿态用 deg, 协方差带符号 (sqrt(|c|)*sign(c))。
    """

    HEADER = (
        "% (x/y/z-ecef=WGS84,Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,"
        "ns=# of satellites),"
        "Qins=1:ins mechanization,"
        "2:ins mechanization and propagate states and covariance,"
        "3:ins-gnss loosely-coupled updates\n"
        "%  GPST              x-ecef(m)      y-ecef(m)      z-ecef(m)   "
        "Q Qins  ns     sdx(m)     sdy(m)     sdz(m)    sdxy(m)    sdyz(m)    "
        "sdzx(m) age(s)  ratio    vx(m/s)    vy(m/s)    vz(m/s)       "
        "sdvx       sdvy       sdvz      sdvxy      sdvyz      sdvzx   "
        "roll(deg)  pitch(deg)    yaw(deg)  sdroll(d) sdpitch(d)  sdyaw(d)\n"
    )

    def __init__(self, output_dir: str, filename: str = "RTKLC.rslt"):
        self.output_dir = output_dir
        self.filename = filename
        self._fp = None
        self._closed = False

    def open(self) -> None: ...

    def write(self, state: InsState, P: np.ndarray, si: StateIndex,
              q: int, qins: int, num_sv: int) -> None:
        """写一行: 位置/速度/姿态 + 协方差。

        Args:
            state: InsState (含 pos_e, vel_e, att_rpy)
            P: 完整状态协方差矩阵 (si.dim x si.dim)
            si: StateIndex (取 pos/vel/att 子块)
            q: 最近 GNSS quality (1/2/4/5)
            qins: 2=time_update, 3=meas_update
            num_sv: 最近 GNSS num_sv
        """
        pos_idx, vel_idx, att_idx = si.pos, si.vel, si.att
        Pp = P[pos_idx:pos_idx+3, pos_idx:pos_idx+3]
        Pv = P[vel_idx:vel_idx+3, vel_idx:vel_idx+3]
        Pa = P[att_idx:att_idx+3, att_idx:att_idx+3]

        week, sow = unix_to_gpst(state.timestamp)
        att_deg = state.att_rpy * (180.0 / np.pi)  # rad → deg
        # 协方差带符号 (与参考文件 sdxy 等负值一致)
        sdx, sdy, sdz = sqrt_diag(Pp)
        sdxy = signed_sqrt(Pp[0, 1]); sdyz = signed_sqrt(Pp[1, 2]); sdzx = signed_sqrt(Pp[0, 2])
        sdvx, sdvy, sdvz = sqrt_diag(Pv)
        sdvxy = signed_sqrt(Pv[0, 1]); sdvyz = signed_sqrt(Pv[1, 2]); sdvzx = signed_sqrt(Pv[0, 2])
        sdroll, sdpitch, sdyaw = sqrt_diag(Pa) * (180.0 / np.pi)

        fmt = (
            "%4d %10.3f %14.4f %14.4f %14.4f %3d %3d %3d"
            " %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f %6.2f %6.1f"
            " %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f"
            " %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f\n"
        )
        self._fp.write(fmt % (
            week, sow,
            state.pos_e[0], state.pos_e[1], state.pos_e[2],
            q, qins, num_sv,
            sdx, sdy, sdz, sdxy, sdyz, sdzx, 0.0, 0.0,
            state.vel_e[0], state.vel_e[1], state.vel_e[2],
            sdvx, sdvy, sdvz, sdvxy, sdvyz, sdvzx,
            att_deg[0], att_deg[1], att_deg[2],
            sdroll, sdpitch, sdyaw,
        ))

    def close(self) -> None: ...
```

**辅助函数**：
- `sqrt_diag(M)`：返回 `[sqrt(|M[i,i]|)]` 三维数组
- `signed_sqrt(c)`：`sqrt(|c|) * sign(c)`，与参考文件负值一致

### 3.2 `lc_integration.py` — 新增 `last_qins` 跟踪

新增实例属性 `last_qins: int`，初值 2（mech+propagate）。

**更新规则**（Qins 反映**本历元是否发生量测更新**，与 ignav 一致）：
- `add_imu` 开始时：`last_qins = 2`（默认：仅机械编排 + 协方差传播）
- `_apply_gnss_update` 内 `feedback()` 后：`last_qins = 3`（量测更新完成）
- `_apply_constraints` 内若 `applied=True`：`last_qins = 3`（约束量测更新）
- `add_imu` 末尾的 `time_update(imu)` **不重置** `last_qins`（保留本历元的量测更新标记）

**关键不变量**：`add_imu` 返回后 `last_qins` 反映本历元是否发生任何量测更新。若本 IMU 区间内有 GNSS 触发或约束触发，输出 Qins=3；否则 Qins=2。

```python
def add_imu(self, imu: ImuMeasurement) -> None:
    self.last_qins = 2  # 默认: 仅机械编排
    # ... GVINS 风格处理, 中间 _apply_gnss_update / _apply_constraints 可置 last_qins=3 ...
    # 末尾 time_update(imu) 不重置 last_qins

def _apply_gnss_update(self, gnss):
    self.est.meas_update_pos(gnss)
    if gnss.velocity is not None:
        self.est.meas_update_vel(gnss)
    self.est.feedback()
    self.last_qins = 3

def _apply_constraints(self, imu):
    # ... 若 applied:
    self.est.feedback()
    self.last_qins = 3
```

### 3.3 `lc_stream.py` — per-IMU 输出

**移除**：
- `_OUTPUT_SEC_TOLERANCE` 常量
- `_pending: dict` 缓冲
- `_buffer_output()` 方法
- `_flush_completed()` 方法
- `_state_to_sol()` 静态方法
- `finalize()` 中的 pending 刷出逻辑

**新增**：
- `_write_state(qins: int)` 方法：取 `self._est.state` + `P` + `si` + `qins` 调 `writer.write()`

**修改**：
- `feed_imu`：`add_imu` 后立即 `_write_state(integ.last_qins)`
- `feed_gnss`：仅 `add_gnss` 入队，不写输出
- `_replay_buffer`：每次 `add_imu` 后 `_write_state`
- `finalize`：仅 close，无 pending 刷出

```python
def feed_imu(self, imu: ImuMeasurement) -> None:
    if not self._initialized:
        self._init_imu.append(imu)
        if self._init_gnss and imu.timestamp >= self._init_gnss[-1].timestamp:
            self._try_init()
        return
    self._integ.add_imu(imu)
    self._write_state(self._integ.last_qins)

def _write_state(self, qins: int) -> None:
    """写当前状态到 .rslt 文件。"""
    self.writer.write(
        state=self._est.state,
        P=self._est.P,
        si=self._si,
        q=self._last_q,
        qins=qins,
        num_sv=self._last_gnss_ns,
    )
```

**Q (GNSS quality) 跟踪**：新增 `_last_q: int`，初值 5（SPP）。`feed_gnss` 时更新为 `gnss.quality`。
- 初始化前 GNSS 不入队，初始化成功后 `_last_q` 从首个 GNSS 取
- 实际上初始化时已有 GNSS，`_last_q` 在 `_try_init` 末尾设置为该 GNSS 的 quality

### 3.4 `main.py` — 修改 lc_writer

```python
# 原
lc_filename = config["output"].get("solution_filename", "RTKLC.pos")
lc_writer = SolutionWriter(output_dir=..., filename=lc_filename)

# 新
lc_filename = config["output"].get("rslt_filename", "RTKLC.rslt")
lc_writer = RSLTWriter(output_dir=..., filename=lc_filename)
```

`internal+off` 路径不变（仍用 `SolutionWriter`）。

### 3.5 `config.yaml` — 新增配置

```yaml
output:
  # 新增
  rslt_filename: "RTKLC.rslt"    # 组合导航输出文件名 (internal+on)
```

`solution_filename` 配置项保留（兼容性，但 internal+on 模式下不再使用）。

## 4. 数据流

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
                                   │          │  write RTKLC.rslt│          │
                                   │          │◄─────────────────│ (per-IMU)│
                                   └────┬─────┘                  └──────────┘
                                        │ write RTK.pos (1Hz, .pos 格式不变)
                                        │ write aligned.csv (块状, 不变)
                                        ▼
                                   ┌──────────┐
                                   │  output/ │
                                   └──────────┘
```

**时序**（典型一秒，100Hz IMU）：
1. ImuSensor 生产 100 条 IMU → imu_queue
2. GnssSensor 生产 1 条 GNSS → gnss_queue
3. Logger drain IMU → `_lc_imu_buffer`
4. Logger 取 GNSS → 写 RTK.pos（1Hz，.pos 格式）
5. Logger `_wait_for_imu` 覆盖 harvest 窗口
6. Logger 按时间交错喂入 LcStream：
   - GNSS 前的 IMU（99 条）→ `feed_imu` → 每条写一行 RTKLC.rslt（Qins=2）
   - GNSS → `feed_gnss` → 入 pending_gnss（不写输出）
   - GNSS 后的 IMU（1 条）→ `feed_imu` → 触发 GVINS 插值 + meas_update → 写一行 RTKLC.rslt（Qins=3）
7. Logger harvest aligner → 写 aligned.csv

**输出频率**：100Hz（每条 IMU 一行），匹配参考文件。

## 5. 边界条件与错误处理

| 情况 | 处理 |
|------|------|
| 初始化前（`_initialized=False`） | 不写输出（无 InsState） |
| 初始化首个 IMU（`imucur=None`） | `add_imu` 直接 return，`last_qins=2`，写一行 |
| IMU EOF | Logger 喂入剩余 IMU，`LcStream.finalize()` 仅 close |
| GNSS EOF | 后续 IMU 仅做机械编排，Qins=2，Q 保持最后值 |
| `att_rpy` 在初始化前未定义 | 初始化成功后才有 InsState，无需处理 |
| 协方差矩阵非正定 | `sqrt(|c|)` 取绝对值，符号单独保留 |
| `time_update` 内部 dt<=0 skip | `last_qins` 仍设为 2（视为 mech only） |

## 6. 验证计划

### 6.1 运行

```bash
cd /home/mxl/workplace/gipylib
python3 src/main.py data/config.yaml
```

**预期输出**：
- `data/output/RTK.pos`（1Hz 纯 GNSS，未变）
- `data/output/RTKLC.rslt`（100Hz 组合导航，新格式）
- `data/output/aligned_internal_rtk.csv`（未变）

### 6.2 正确性验证

1. **文件完整性**：三个输出文件均生成，RTKLC.rslt 行数 > 0
2. **格式正确性**：
   - 表头两行（注释 + 字段定义）
   - 每行 30 列（24 基础 + 6 姿态）
   - 时间戳单调递增，间距 0.01s
3. **与参考文件对比**：
   - 加载 `data/rtktcgps.rslt` 与 `data/output/RTKLC.rslt`
   - 按 GPST (week, sow) 匹配重叠历元（容差 0.005s）
   - 平面位置误差：`sqrt(dx² + dy²) ≤ 0.5m`
   - 高程误差：`|dz| ≤ 1m`
   - 速度合理性：`|v_ours - v_ref| < 0.5 m/s`（无硬约束，参考用）
4. **运行无异常**：无 Python traceback

### 6.3 回归验证

- `RTK.pos` 与上次运行对比：行数、内容一致（纯 GNSS 路径未改）
- `aligned_internal_rtk.csv` 与上次运行对比：行数、内容一致（对齐路径未改）

## 7. 风险与缓解

| 风险 | 缓解 |
|------|------|
| 参考文件 Qins=0 而我方 Qins=2/3 | 不影响位置对比，仅 Qins 列语义不同 |
| 时间范围不重叠 | 验证时报告重叠历元数，若 < 100 视为异常并排查 |
| 100Hz 写入 IO 压力 | 离线处理可接受；~23.6 万行文本 < 50MB |
| 姿态 att_rpy 未在 mechanization 后更新 | 已验证 `InsUpdate.update` L109 同步更新 `att_rpy` |
| GVINS 插值触发的 GNSS update 后 time_update(imu) 覆盖 Qins=3 | `last_qins` 在 `_apply_gnss_update` 设为 3，`add_imu` 末尾不重置（仅在 add_imu 开头初始化为 2） |
| `_lc_imu_buffer` 按时间交错喂入破坏 per-IMU 写出节奏 | LcStream 内部按 feed_imu 顺序处理，per-IMU 写出与 Logger 喂入节奏解耦 |
