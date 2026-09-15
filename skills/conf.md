# 配置文件说明

> **当前状态索引（2026-09-15）**：配置说明需同时区分相位 `maxinno` 与伪距 `maxcode`；**INS 参数已拆 `ins`/`ins_tc`/`ins_lc` 三段，`pos_psd`/`vel_psd` 为 LC-only 机制禁止用于 TC**。当前 RTK 浮点基线为 `prnbias=0.03`，输出支持 `stat_level`/`trace_enabled`，Data19 可使用 `ins.imu_time_offset_s`。详见 [项目当前状态](项目当前状态.md) 第 4.4 节。

> 定义 GInsStream 统一定位解算配置文件的格式、字段与默认值。
> 配置文件采用 YAML 格式，存放于 `data/config.yaml`（参考配置：`data/spp-ins-lc.yaml`、`data/rtdtc.yaml`、`phone/rtdtc.yaml`、`data/ignav-rtktc.conf`）。
>
> **配置来源约定**：
> - **GNSS 部分配置项**：参考 rtklib-py 的 `config_phone.py` / `config_f9p.py` / `__ppk_config.py`
>   （rtklib-py 已吸收到 `src/core/gnss/rtklib/`，配置通过 `src/core/gnss/rtklib_config_adapter.py` 注入）
> - **组合导航部分配置项**：参考 ignav 的 `configure.ini`（ignav 已吸收到 `src/core/ins/`）
> - **文件格式**：仿照 `tools/KF-GINS/config/kf-gins.yaml` 的中英双语注释 YAML 风格
>
> **时间系统约定**：全框架内部统一使用 **Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）**。
> 输入端（`src/stream/formators.py`）将 GPS 周+周内秒通过 `gpst_to_unix()` 转为 Unix 时间戳；
> 输出端（`src/log/solution_writer.py` / `aligned_writer.py`）通过 `unix_to_gpst()` 转回 (week, sow) 写文件。
> 时间转换工具位于 `src/core/time_utils.py`，常量 `GPST_EPOCH_UNIX = 315964800`。
>
> **当前项目状态**（`ins.enabled` 控制组合模式：`off`=纯 GNSS / `lc`=松组合 / `tc`=紧组合；已废弃的 `coupling_mode` 字段已移除）：
> - ✅ 已实现：纯 GNSS 模式（`ins.enabled: "off"`，`gnss_source` 必须为 `internal`）→ 输出 `.pos` 文件（`SolutionLogger` + `SolutionWriter`）
> - ✅ 已实现：松组合模式（`ins.enabled: "lc"`）→ 输出 `.pos` + `aligned.csv` + `.rslt`（100Hz，ECEF 位置/速度 + 姿态）
> - ✅ 已实现：紧组合模式（`ins.enabled: "tc"`）→ 输出 `.rslt`（100Hz，`TcStream` + `RSLTWriter`）
> - ✅ 已实现：INS 初始化模块 `src/core/ins/initializer.py::InsInitializer`（三种模式：静态 / 速度矢量 / 位置差分，三阈值检验），详见 [初始化.md](file:///home/mxl/workplace/gipylib/skills/初始化.md)
> - ✅ 已实现：`src/core/ins/` 下 `interpolator.py` / `earth_param.py` / `attitude.py`（初始化支撑模块）
> - ✅ 已实现：`ImuSensor` RFU→FRD 坐标系自动转换（`_convert_to_frd()`）
> - ✅ 已实现：SPP 多普勒测速（`pntpos.py::estvel` / `resdop`，速度填入 `sol.rr[3:6]`）
> - ✅ 已实现：`TraceFileWriter`（`src/log/trace_file_writer.py`）重定向 rtklib-py trace 输出，将 Unix 时间戳转换为 GPS 周+周内秒，过滤无效调试行（`pos=[0. 0. 0.]`、`x[clk]=N/A`、`clk_stored=[0. 0. 0.]`）
> - ✅ 已实现：`SolutionWriter` / `RSLTWriter` 支持 `position_format` 与 `time_format` 输出格式参数
>
> **三种运行模式对比**：
> | 模式 | gnss_source | ins.enabled | GNSS 来源 | IMU | 输出 | Logger/Writer |
> |------|-------------|-------------|-----------|-----|------|---------------|
> | 纯 GNSS | internal | off | 实时解算 | 无 | `.pos` | SolutionLogger + SolutionWriter |
> | 松组合 | internal/external | on | 实时解算/外部文件 | 流式 | `.pos` + `aligned.csv` + `.rslt` | SolutionWriter + AlignedWriter + RSLTWriter |
> | 紧组合 | internal | tc | 实时解算 | 流式 | `.rslt` | TcStream + RSLTWriter |
>
> **单滤波架构对应**（INS 启用后的设计，参考 [estimator.md](file:///home/mxl/workplace/gipylib/skills/estimator.md)）：
> - 单一 P 矩阵（E 系，StateIndex 参数块，15~24 维 = 固定 15 维 + 可选 GNSS 杆臂 3 / IMU 安装角 2 / IMU 杆臂 3 / 时间对齐 1）
> - NHC/ZUPT 互斥：静止时仅 ZUPT（3D，作用于 P），运动时仅 NHC（2D，作用于 P）

---

## 目录

- [配置文件说明](#配置文件说明)
  - [目录](#目录)
  - [1. 概述](#1-概述)
  - [2. 配置文件格式](#2-配置文件格式)
  - [3. GNSS 配置项（参考 rtklib-py）](#3-gnss-配置项参考-rtklib-py)
    - [3.1 数据源与文件路径](#31-数据源与文件路径)
    - [3.2 定位模式](#32-定位模式)
    - [3.3 观测值筛选](#33-观测值筛选)
    - [3.4 周跳与粗差检测](#34-周跳与粗差检测)
    - [3.5 卡尔曼滤波统计](#35-卡尔曼滤波统计)
    - [3.6 模糊度解算](#36-模糊度解算)
    - [3.7 单点定位参数](#37-单点定位参数)
    - [3.8 星座与信号配置](#38-星座与信号配置)
    - [3.9 基站与初始位置](#39-基站与初始位置)
  - [4. 组合导航配置项（参考 ignav）](#4-组合导航配置项参考-ignav)
    - [4.1 处理时间](#41-处理时间)
    - [4.2 使能开关](#42-使能开关)
    - [4.3 数据路径与采样率](#43-数据路径与采样率)
    - [4.4 初始对准](#44-初始对准)
    - [4.5 初始状态](#45-初始状态)
    - [4.6 IMU 噪声参数](#46-imu-噪声参数)
    - [4.7 NHC 配置](#47-nhc-配置)
    - [4.8 ZUPT 配置](#48-zupt-配置)
    - [4.9 杆臂与安装角](#49-杆臂与安装角)
    - [4.10 初始协方差](#410-初始协方差)
    - [4.11 GNSS 中断模拟](#411-gnss-中断模拟)
  - [5. 输出配置](#5-输出配置)
  - [6. 状态向量对应（单滤波, StateIndex）](#6-状态向量对应单滤波-stateindex)
    - [6.1 固定 15 维（E 系, ψ-error）](#61-固定-15-维e-系-ψ-error)
    - [6.2 可选参数块](#62-可选参数块)
    - [6.3 反馈机制](#63-反馈机制)
  - [7. 数据文件说明](#7-数据文件说明)
    - [7.1 IMU 数据文件](#71-imu-数据文件)
    - [7.2 GNSS 观测值文件](#72-gnss-观测值文件)
    - [7.3 GNSS 定位结果文件](#73-gnss-定位结果文件)
    - [7.4 输出文件](#74-输出文件)

---

## 1. 概述

GInsStream 采用**单一 YAML 配置文件**驱动整个定位解算流程，涵盖：

1. **GNSS 解算配置**（SPP/RTK/RTD，参考 rtklib-py）
2. **组合导航配置**（INS 机械编排 + 单滤波 EKF + NHC/ZUPT，参考 ignav）
3. **输出配置**（POS/CSV/NMEA 格式、日志级别）

配置文件支持两种 GNSS 数据源模式：
- **内部解算模式**（`gnss_source: "internal"`）：从原始观测值解算 SPP/RTK/RTD
- **外部结果模式**（`gnss_source: "external"`）：直接读取外部 GNSS 定位结果文件（POS/NMEA/CSV）

---

## 2. 配置文件格式

- 文件格式：YAML 1.1
- 文件位置：`data/config.yaml`（参考配置：`data/spp-ins-lc.yaml`、`data/rtdtc.yaml`、`phone/rtdtc.yaml`、`data/ignav-rtktc.conf`）
- 注释风格：中英双语注释（参考 kf-gins.yaml）
- 数组：使用 YAML 内联数组语法 `[a, b, c]` 或块状语法
- 布尔值：`true` / `false`
- 字符串：可加引号或不加引号（含特殊字符时建议加引号）
- 时间单位：除特别说明外，**内部时间戳使用 Unix 秒（与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）**；
  YAML 配置文件中涉及时间字段的语义见各小节说明（如 `start_time` 为周内秒）

---

## 3. GNSS 配置项（参考 rtklib-py）

> 以下配置项映射自 rtklib-py 的 `config_phone.py` / `config_f9p.py` / `__ppk_config.py`。
> rtklib-py 已吸收到 `src/core/gnss/rtklib/`，配置通过 `src/core/gnss/rtklib_config_adapter.py::build_params()`
> 翻译为 params dict，由 `config.set_params()` 注入 `_CfgProxy` 单例（参考 `src/core/gnss/rtklib/config.py`）。
> 命名保持与 rtklib-py 一致，便于算法移植与对照。

### 3.1 数据源与文件路径

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `gnss_source` | str | `"internal"` | GNSS 数据源模式：`internal` 内部解算 / `external` 外部结果文件 |
| `rover_path` | str | `""` | 流动站观测值文件路径（internal 模式） |
| `base_path` | str | `""` | 基站观测值文件路径（internal 模式 RTK） |
| `eph_path` | str | `""` | 星历文件路径（internal 模式） |
| `external_sol_path` | str | `""` | 外部 GNSS 结果文件路径（external 模式） |
| `external_sol_format` | str | `"pos"` | 外部结果格式：`pos` / `nmea` / `csv` |
| `default_pos_std` | float | `1.0` | 外部结果默认位置标准差 (m) |
| `default_vel_std` | float | `0.5` | 外部结果默认速度标准差 (m/s) |
| `pos_covariance_available` | bool | `false` | 外部结果是否含协方差 |

### 3.2 定位模式

| 字段 | 类型 | 默认值 | 说明 | rtklib-py 对应 |
|------|------|--------|------|---------------|
| `nf` | int | `2` | 频率数 (1 或 2) | `nf` |
| `pmode` | str | `"kinematic"` | 定位模式：`static` / `kinematic` | `pmode` |
| `filtertype` | str | `"forward"` | 滤波类型：`forward` / `backward` / `combined` / `combined_noreset` | `filtertype` |
| `use_sing_pos` | bool | `false` | 每历元是否重新单精解初始位置 | `use_sing_pos` |

### 3.3 观测值筛选

| 字段 | 类型 | 默认值 | 单位 | 说明 | rtklib-py 对应 |
|------|------|--------|------|------|---------------|
| `elmin` | float | `15.0` | deg | 最小仰角（浮点解） | `elmin` |
| `cnr_min` | [float, float] | `[28, 20]` | dB-Hz | 最小信噪比 [freq1, freq2] | `cnr_min` |
| `remove_sat` | list | `[]` | — | 剔除卫星列表（如 `["C01"]`）；内部转换为卫星号后交给 `satexclude`。遗留别名 `excsats` 仍被接受并与之合并 | `excsats` |

### 3.4 周跳与粗差检测

| 字段 | 类型 | 默认值 | 单位 | 说明 | rtklib-py 对应 |
|------|------|--------|------|------|---------------|
| `maxinno` | float | `5.0`（RTK 建议） | m | 载波相位双差创新/粗差阈值；不等于伪距门限 | `maxinno` |
| `maxcode` | float | `30.0`（RTKLIB 参考） | m | 伪距创新/粗差阈值，与 `maxinno` 独立 | `maxcode` |
| `maxage` | float | `30.0` | s | 最大差分龄期 | `maxage` |
| `maxout` | int | `4` | epoch | 最大差分中断历元数 | `maxout` |
| `thresdop` | float | `5.0` | — | 多普勒法周跳检测阈值 | `thresdop` |
| `thresslip` | float | `0.10` | — | 几何无关组合（LG）周跳检测阈值 | `thresslip` |
| `interp_base` | bool | `false` | — | 是否插值基站观测值 | `interp_base` |

### 3.5 卡尔曼滤波统计

| 字段 | 类型 | 默认值 | 单位 | 说明 | rtklib-py 对应 |
|------|------|--------|------|------|---------------|
| `eratio` | [float, float] | `[300, 100]` | — | 伪距/载波噪声比 [L1, L5/L2] | `eratio` |
| `efact_gps` | float | `1.0` | — | GPS 星座相对权重 | `efact[uGNSS.GPS]` |
| `efact_glo` | float | `1.5` | — | GLONASS 星座相对权重 | `efact[uGNSS.GLO]` |
| `efact_gal` | float | `1.0` | — | Galileo 星座相对权重 | `efact[uGNSS.GAL]` |
| `err_base` | float | `0.003` | m | 基线误差 sigma | `err[1]` |
| `err_el` | float | `0.003` | m | 仰角误差 sigma | `err[2]` |
| `err_satclk` | float | `5e-12` | s | 卫星钟误差 sigma | `err[6]` |
| `snrmax` | float | `45.0` | dB-Hz | 方差计算最大信噪比 | `snrmax` |
| `accelh` | float | `3.0` | m/s² | 水平加速度噪声 sigma | `accelh` |
| `accelv` | float | `1.0` | m/s² | 垂直加速度噪声 sigma | `accelv` |
| `prnbias` | float | `0.01`（通用适配默认；RTK 验证基线 `0.03`） | cycles | 载波相位偏差 sigma；旧值 `0.5` 会使浮点模糊度协方差过大 | `prnbias` |
| `sig_p0` | float | `30.0` | m | 初始位置 sigma | `sig_p0` |
| `sig_v0` | float | `10.0` | m/s | 初始速度/加速度 sigma | `sig_v0` |
| `sig_n0` | float | `30.0` | m | 初始模糊度 sigma | `sig_n0` |

### 3.6 模糊度解算

| 字段 | 类型 | 默认值 | 说明 | rtklib-py 对应 |
|------|------|--------|------|---------------|
| `armode` | int | `0` | 模糊度解算模式：0=off / 1=continuous / 3=fix-and-hold | `armode` |
| `thresar` | float | `3.0` | AR ratio 检验阈值 | `thresar` |
| `thresar1` | float | `0.05` | AR 位置变化阈值（同时用于卡尔曼加速度更新） | `thresar1` |
| `minlock` | int | `0` | 参与 AR 的最小连续锁定历元数 | `minlock` |
| `glo_hwbias` | float | `0.0` | GLONASS 硬件偏差 (m) | `glo_hwbias` |
| `elmaskar` | float | `15.0` | deg | AR 仰角掩模 | `elmaskar` |
| `var_holdamb` | float | `0.1` | m² | 保持模糊度方差 | `var_holdamb` |
| `minfix` | int | `20` | 触发 hold 的最小固定样本数 | `minfix` |
| `minfixsats` | int | `4` | 固定时最小卫星对数 | `minfixsats` |
| `minholdsats` | int | `5` | hold 时最小卫星对数 | `minholdsats` |
| `mindropsats` | int | `10` | 丢弃卫星的最小卫星对数 | `mindropsats` |

### 3.7 单点定位参数

| 字段 | 类型 | 默认值 | 单位 | 说明 | rtklib-py 对应 |
|------|------|--------|------|------|---------------|
| `sing_p0` | float | `100.0` | m | 单点定位初始位置 sigma | `sing_p0` |
| `sing_v0` | float | `10.0` | m/s | 单点定位初始速度 sigma | `sing_v0` |
| `sing_elmin` | float | `10.0` | deg | 单点定位最小仰角 | `sing_elmin` |

### 3.8 星座与信号配置

信号映射默认由 `src/utility/rinex_improve.py` + `src/utility/gnutlib/`
（LibGnut 移植）按文件头自动规划；**真实数据与 LibGnut 库级默认冲突时，用
`raw_band_priority` 手动固定频点**（数据自检无法替代人工判断，见 3.8.2）：

| 字段 | 类型 | 默认值 | 说明 | rtklib-py 对应 |
|------|------|--------|------|---------------|
| `gnss_t` | list | `["GPS", "GLO", "GAL"]` | 启用星座列表；北斗显式写 `BDS` | `gnss_t` |
| `nf` | int | `1` | 频点数上限（每系统槽位数），与自动波段方案共同决定槽位 | `nf` |
| `raw_band_priority` | dict | 无 | **手动频点映射（推荐显式保留）**：raw RINEX band 优先序，长度 = `nf`；声明后跳过自动规划 | — |
| `raw_signal_priority` | dict | 无 | 同频跟踪属性（code/phase/doppler/snr）优先串覆盖（GREAT RAW_MIX 风格） | — |
| `freq_table` / `freq_ix0` / `freq_ix1` / `dfreq_glo` | — | 自动派生 | 缺失时由 `gnutlib.gsys` 的 LibGnut 频率表（10.23 MHz 乘数、GLONASS FDMA 间隔）派生；与手写等价（按 Hz 值查表） | `freq` / `freq_ix` / `dfreq_glo` |

**支持的信号类型**（sig_tbl 映射）：

| 信号码 | rtklib-py 枚举 |
|--------|---------------|
| `1C` / `1X` / `1W` | L1C / L1X / L1W |
| `2W` / `2L` / `2C` / `2X` | L2W / L2L / L2C / L2X |
| `5Q` / `5X` | L5Q / L5X |
| `7Q` / `7X` | L7Q / L7X |
| `1I` | BDS B1I（`1561.098 MHz`） |
| `7I` | BDS B2I/B2b（`1207.14 MHz`） |

BDS 广播星历从 BDT 转 GPST 时执行 `week+1356` 和 `toc/toe/tot+14 s`；BDS GEO
PRN `1..5`、`59+` 使用专用坐标旋转。GPS+BDS SPP 与 SPP-TC 使用 BDS 独立 ISB。

#### 3.8.1 自动信号方案（rinex_improve + gnutlib）

处理逻辑以 LibGnut（`GREAT-MSF/src/LibGnut`）为第一权威，输出保持 pyrinrx
（rtklib-py）可直接解码的 RINEX 与频点槽位语义：

```
RINEX 文件头 SYS / # / OBS TYPES
  → gnutlib.parse_obs_header + fix_band          # BDS C1x→C2x(≤3.03)、C3x→C6x、C7D/P/Z→C9(P/Z)(≥3.04)
  → plan_stream_signals(rover, base, gnss_t, nf)  # 流动站 ∩ 基站共同波段
       · 按 LibGnut GNSS_BAND_PRIORITY（gutils/gnss.h）排序后截断到 nf
       · band_consistency 码-相自检：用错波长时 Δ(C-λL) 显著变大 ⇒ 剔除(fail-open)
  → auto_freq_plan                                # 波段 → gnutlib 频率值 → freq_table/freq_ix/dfreq_glo
  → needs_improvement / improve_rinex             # 频带归一化 + 同频跟踪码选择 + C-L-D-S 交织重写
       · 跟踪码按 LibGnut range/phase_order_attr_raw（gobsgnss.h，末位优先）
       · code_availability 剔除稀疏跟踪码（如手机基站 GPS L1M 仅 25% 有值）
  → rnx_decode.decode_obs                         # 自动方案作为 raw_band_to_slot
  → zdres / selsat / DD 行（RtkTcMeas / RtdTcMeas）
```

三套编号仍然存在（band / freq_ix / 槽位），但**全部在 `rinex_improve` 内部
闭合**：用户只声明星座与频点数上限。

| 编号域 | 例子 | 定义处 | 用途 |
|---|---|---|---|
| **raw RINEX band digit** | 观测码第二字符：`C2I`→2、`C5Q`→5、`C6I`→6 | RINEX 3 观测码（经 `fix_band` 归一化） | 物理频点标识；自动方案的值域 |
| **solver freq_ix** | `freq_table[6]` | `auto_freq_plan` 派生 | 解算侧频率查找；**与 band 数字无对应关系** |
| **decoder slot** | 槽位 0/1 | `raw_band_priority_to_slot_mapping` | 按方案顺序分配，流动/基站一致 |

`LEGACY_FREQ_BAND_METADATA`（`freq_ix → raw band`）与
`raw_band_priority` 仍被支持，用于复现历史配置；两者同时存在时以显式
`raw_band_priority` 为准（`gnss_band_mapping.resolve_raw_band_priority`）。

**强制校验**：未知系统或 band 不在 `RAW_BAND_DOMAIN`、band 重复、
`raw_band_priority` 长度 ≠ `nf`、`freq_ix` 无 metadata 可解析、increment
输入禁止二次转换，均 `ValueError`。

#### 3.8.2 何时必须显式 `raw_band_priority`

自动方案按 LibGnut 库级优先级选择，**物理上一致但质量不必最优**：库默认
（BDS B1I 优先）与真实数据集的最优频点可能不同，数据自检（码-相一致性、
观测码可用性）只能排除物理错误，不能替代人工选优。实测（campus01，
truth-left 10 Hz，`remove_sat: ["C01"]`）：自动 B1I 3D RMS 1.123 m、
手动 B3I 1.272 m、手动 B3I 且不剔除 C01 1.187 m、旧链路（B3I）1.145 m
——同一数据集差异可达 0.15 m 量级，故**保留手动频点映射**：

| 场景 | 处理 |
|---|---|
| 数据集标定的共同载波与 LibGnut 默认不同（如 campus01 BDS 用 B3I） | `raw_band_priority: {GPS: [1], GAL: [1], BDS: [6], GLO: [1]}` |
| 需要固定单频点以做对照实验（如手机 GPS L1 + BDS B1I） | `raw_band_priority: {GPS: [1], BDS: [2]}` |
| 复现历史配置 | 保留原 `raw_band_priority`（legacy `freq_ix*` 亦可，二者取显式 `raw_band_priority`） |

**书写规则**：键可用 `GPS/GAL/BDS/GLO` 或 RINEX 单字符 `G/E/C/R`；值是有序
band 数字表（第一元素 = 槽位 0）；长度 = `nf`；值域 `G/E/J/C: {1,2,5,6,7,8}`、
`R: {1,2,3,4,6}`。

**验证方法**（自动方案审计）——

```bash
python3 -c "
from src.utility.config_loader import load_config
from src.utility.rinex_improve import resolve_stream_plan, improve_rinex
cfg = load_config('你的配置.yaml')['gnss']
resolved, plan = resolve_stream_plan(cfg, cfg['rover_path'], cfg.get('base_path'))
print('自动方案:', plan, '频率映射:', resolved['freq_ix0'], resolved['freq_ix1'])
improve_rinex(cfg['rover_path'], '/tmp/simplified.obs', band_plan=plan,
              gnss_t=cfg['gnss_t'])
"
grep 'SYS / # / OBS TYPES' /tmp/simplified.obs   # 每个目标系统都应在场
```

逐卫星残差核查：RTD 用 `phone/plot/rtd_residual_stats.py`（逐系统×频点×卫星
的码 DD prefit 残差统计）；TC 开启 `tc.measurement_trace` 后用
`phone/plot/residual_dump.py`。

### 3.9 基站与初始位置

| 字段 | 类型 | 默认值 | 单位 | 说明 | rtklib-py 对应 |
|------|------|--------|------|------|---------------|
| `rb_format` | str | `"xyz"` | — | 基站坐标格式：`xyz`=ECEF 直角坐标 (m) / `llh`=经纬度高 [lat_deg, lon_deg, h_m]。`llh` 时由 `config_loader._normalize_rb()` 内部转为 xyz | — |
| `rb` | [float, float, float] | `[0, 0, 0]` | m | 基站位置（`rb_format=xyz`: ECEF [x,y,z]；`rb_format=llh`: [lat, lon, h]；全 0 表示用 RINEX 头） | `rb` |
| `rr_f` | list | `[0,0,0,0,0,0]` | m, m/s | 流动站初始位置速度（正向，0 表示自动单精解） | `rr_f` |
| `rr_b` | list | `[0,0,0,0,0,0]` | m, m/s | 流动站初始位置速度（反向，0 表示自动单精解） | `rr_b` |

---

## 4. 组合导航配置项（参考 ignav）

> 以下配置项参考 ignav 的 `configure.ini`（ignav 已吸收到 `src/core/ins/`），命名与原项目一致。
> 坐标系约定：机械编排在 E 系（ECEF）下进行，IMU 体坐标系为 FRD，车体坐标系为 FRD。

### 4.1 处理时间

| 字段 | 类型 | 默认值 | 单位 | 说明 | 参考来源 |
|------|------|--------|------|------|---------------------|
| `process_date` | [int, int, int] | `[2024, 12, 20]` | — | 处理日期 [year, month, day] | `process_date` |
| `start_time` | float | `0` | s | 起始时间（当日秒或 GPST 周） | `start_time` |
| `end_time` | float | `-1` | s | 结束时间（-1 表示处理至文件结束） | `end_time` |

### 4.2 使能开关

| 字段 | 类型 | 默认值 | 说明 | 参考来源 |
|------|------|--------|------|---------------------|
| `gnss_enable` | int | `1` | GNSS 量测使能（0=关闭） | `gnss_enable` |
| `imu_enable` | int | `1` | IMU 机械编排放能 | `imu_enable` |
| `nhc_enable` | int | `2` | NHC 策略（0-9，见 4.7） | `nhc_enable` |
| `zupt_enable` | int | `0` | ZUPT 策略（0=关闭，1=启用 3D 零速更新；与 NHC 互斥） | GInsStream 扩展 |

### 4.3 数据路径与采样率

| 字段 | 类型 | 默认值 | 单位 | 说明 | 参考来源 |
|------|------|--------|------|------|---------------------|
| `imu_data_path` | str | `""` | — | IMU 数据文件路径 | `imu_data_path` |
| `gnss_data_path` | str | `""` | — | GNSS 数据文件路径（外部模式结果文件） | `gnss_data_path` |
| `result_output_path` | str | `"output/"` | — | 结果输出目录 | `result_output_path` |
| `data_rate` | int | `100` | Hz | IMU 采样率 | `data_rate` |
| `result_output_rate` | int | `1` | Hz | 结果输出率 | `result_output_rate` |
| `imudatalen` | int | `7` | — | IMU 文件列数（只用前 7 列） | 参考 kf-gins |
| `imu_format` | str | `"gpst"` | — | IMU 数据文件格式。`gpst`=GPS 周+周内秒（`week,sow,gx,gy,gz,ax,ay,az`），由 `ImuFormator` 解码；`euroc`=Unix 纳秒时间戳（`timestamp_ns,wx,wy,wz,ax,ay,az`），由 `EuRoCImuFormator` 解码。`ImuSensor._create_formator()` 根据此参数工厂创建对应解码器（详见 [imu.md](file:///home/mxl/workplace/gipylib/skills/imu.md) / [StreamDesign.md](file:///home/mxl/workplace/gipylib/skills/StreamDesign.md)） | GInsStream 扩展 |

### 4.4 初始对准

> 详见 [初始化.md 第 5 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#5-初始化模式分类) 和 [第 5.4 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#54-动态初始化默认模式选择基于-gnss-解算模式)。
> **当前实现状态**：`InsInitializer` 已实现三种模式（静态 / 速度矢量 / 位置差分），详见 [初始化.md](file:///home/mxl/workplace/gipylib/skills/初始化.md)。

| 字段 | 类型 | 默认值 | 单位 | 说明 | 参考来源 |
|------|------|--------|------|------|---------------------|
| `alignnment_velocity_threshold` | float | `4.0` | m/s | **已废弃**（保留兼容），由 `static_speed_threshold` / `dynamic_speed_threshold` 替代 | `alignnment_velocity_threshold` |
| `motion_threshold` | float | `0.5` | m/s | **已废弃**（保留兼容），由三阈值替代 | — |
| `static_speed_threshold` | float | `0.5` | m/s | **静态检测速度阈值**：GNSS 速度范数 < 此值判定为静态，进入静对准（[初始化.md 5.4.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#543-三组独立阈值static_speed--dynamic_speed--angular_velocity)） | — |
| `dynamic_speed_threshold` | float | `4.0` | m/s | **动态速度阈值**：GNSS 速度范数 > 此值才可进入动对准（速度矢量法与位置差分法均适用） | — |
| `angular_velocity_threshold_deg` | float | `30.0` | deg/s | **动态角速度阈值**：陀螺角速度范数 < 此值才可进入动对准（≈0.5236 rad/s），避免转弯时初始化 | — |
| `imu_coordinate_system` | str | `"FRD"` | — | IMU 原始坐标系：`FRD`=前右下（默认） / `RFU`=右前上（`ImuSensor` 读取时自动转换为 FRD） | — |
| `alignnment_dynamic_method` | str | `"auto"` | — | 动对准方法选择（[初始化.md 5.4.1](file:///home/mxl/workplace/gipylib/skills/初始化.md#541-默认规则)）。`auto`=按 GNSS 模式自动选择；`velocity_vector`=强制速度矢量；`position_diff`=强制位置差分 | — |
| `gnss_velocity_fallback` | str | `"position_diff"` | — | GNSS 无速度时回退策略（[初始化.md 5.4.2](file:///home/mxl/workplace/gipylib/skills/初始化.md#542-gnss-不提供速度时的统一回退策略)）。动态模式下 GNSS 不提供速度时统一使用位置差分法 | — |
| `gnss_buffer_size` | int | `3` | 个 | 位置差分初始化的 GNSS 历元缓冲区大小（[初始化.md 9.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#93-处理流程伪代码)）。运动阈值达到时确保缓冲区存满 N 个历元，但只用最新两个历元计算差分速度 | — |
| `alignnment_attitude_mode` | int | `1` | — | 0=自动对准 / 1=使用给定姿态 `initial_att` | `alignnment_attitude_mode` |
| `alignnment_posvelatt_mode` | int | `0` | — | 1=使用给定位置速度姿态对准（最高优先级） | `alignnment_posvelatt_mode` |
| `high_precision_ins_mode` | bool | `false` | — | 高精度 INS 初始化模式（[初始化调整.md](file:///home/mxl/workplace/gipylib/skills/初始化调整.md)）。`false`=低精度模式（本项目当前实现：静态位置 GNSS 历史平均 + 姿态置 0 + 速度置 0）；`true`=高精度模式（预留，AcceLeveling + 解析寻北，后续实现） | GInsStream 扩展 |
| `static_duration` | double | `10.0` | s | 静态初始化 GNSS 位置平均窗口（[初始化.md 7.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#73-静态位置平均)）。GNSS 历史少于 `static_duration` 秒时用全部历元求平均；多于时取最新 `static_duration` 秒内的历元求平均 | GInsStream 扩展 |

**三阈值选择建议**（[初始化.md 5.4.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#543-三组独立阈值static_speed--dynamic_speed--angular_velocity)）：

| 场景 | `static_speed_threshold` | `dynamic_speed_threshold` | `angular_velocity_threshold_deg` | 原因 |
|------|--------------------------|---------------------------|----------------------------------|------|
| 默认（车辆） | 0.5 m/s | 4.0 m/s | 30.0 deg/s | 区分静止与低速行驶；角速度约束避免转弯时初始化 |
| 行人/低速车辆 | 0.2 m/s | 1.0 m/s | 20.0 deg/s | 避免低速被误判为静止 |
| 高速车辆 | 1.0 m/s | 5.0 m/s | 30.0 deg/s | 避免停车等红灯时误判为运动 |
| 严格静态启动 | 0.1 m/s | — | — | 仅在确实静止时进入静对准 |

对准模式优先级（从高到低）：
1. `alignnment_posvelatt_mode=1` → 使用 `initial_pos` / `initial_vel` / `initial_att`
2. `alignnment_attitude_mode=1` → 位置速度取首个 GNSS 历元，姿态取 `initial_att`
3. 自动对准：根据三阈值决策（[初始化.md 5.4.4](file:///home/mxl/workplace/gipylib/skills/初始化.md#544-决策树)）
   - 静态：GNSS 速度 < `static_speed_threshold` → 静对准（AcceLeveling）
   - 动态：GNSS 速度 > `dynamic_speed_threshold` 且陀螺角速度范数 < `angular_velocity_threshold_deg` → 动对准（速度矢量 / 位置差分）
   - 否则：延迟初始化，等待满足阈值条件的历元

### 4.5 初始状态

| 字段 | 类型 | 默认值 | 单位 | 说明 | 参考来源 |
|------|------|--------|------|------|---------------------|
| `initial_pos` | [float, float, float] | `[0, 0, 0]` | deg, deg, m | 初始位置 [lat, lon, alt] | `initial_pos` |
| `initial_vel` | [float, float, float] | `[0, 0, 0]` | m/s | 初始速度 [N, E, D] | `initial_vel` |
| `initial_att` | [float, float, float] | `[0, 0, 0]` | deg | 初始姿态 [roll, pitch, yaw]（ZYX 旋转） | `initial_att` |
| `initial_gyro_bias` | [float, float, float] | `[0, 0, 0]` | deg/h | 初始陀螺零偏 | `initial_gyro_bias` |
| `initial_acce_bias` | [float, float, float] | `[0, 0, 0]` | mGal | 初始加计零偏 | `initial_acce_bias` |

### 4.6 IMU 噪声参数

| 字段 | 类型 | 默认值 | 单位 | 说明 | 参考来源 |
|------|------|--------|------|------|---------------------|
| `velocity_random_walk` | [float, float, float] | `[0.1, 0.1, 0.1]` | m/s/√hr | 速度随机游走 (VRW) | `velocity_random_walk` |
| `attitude_random_walk` | [float, float, float] | `[0.1, 0.1, 0.1]` | deg/√hr | 姿态随机游走 (ARW) | `attitude_random_walk` |
| `gyro_bias_std` | [float, float, float] | `[25, 25, 25]` | deg/h | 陀螺零偏标准差 | `gyro_bias_std` |
| `acce_bias_std` | [float, float, float] | `[200, 200, 200]` | mGal | 加计零偏标准差 | `acce_bias_std` |
| `corr_time_of_gyro_bias` | float | `0.01` | h | 陀螺零偏相关时间 | `corr_time_of_gyro_bias` |
| `corr_time_of_acce_bias` | float | `0.01` | h | 加计零偏相关时间 | `corr_time_of_acce_bias` |
| `position_random_walk` | [float, float, float] | `[0, 0, 0]` | — | 位置随机游走 | `position_random_walk` |

### 4.7 NHC 配置

NHC（非完整性约束）利用车辆运动学假设（车轮不侧滑、不腾空）作为虚拟速度观测，在 GNSS 中断期间抑制惯性推算误差发散。

**`nhc_enable` 策略表**（参考 ignav）：

| `nhc_enable` | 观测维度 | 坐标系 | 观测量 | 作用目标 | 备注 |
|:---:|:---:|:---:|---|---|---|
| `0` | — | — | 不启用 NHC | — | — |
| `1` | 1D | v 系 | 侧向速度 | P（H_nhc） | 需开启 `evaluate_imu_angle` |
| `2` | 2D | v 系 | 侧向 + 垂向速度 | P（H_nhc） | 默认策略 |
| `3` | 3D | v 系 | 前向 + 侧向 + 垂向 | P（H_nhc） | 高速时放宽前向 R |
| `4` | 3D | n 系 | 姿态（roll/pitch/yaw） | P | LSTM/GT 姿态虚拟观测 |
| `5` | 6D | v+n | 姿态 3D + 速度 3D | P | 联合约束 |
| `6` | 2D | v 系 | 侧向 + 垂向 | P | Ga-St VBAKF 自适应 |
| `7` | 1D | v 系 | 侧向 | P | Ga-St VBAKF 自适应 |
| `8` | 3D | n 系（ENU） | 东+北+天速度 | P | LSTM/GT ENU 速度 |
| `9` | 2D | v 系 | 前向 + 侧向 | P | 速度大时放宽 R |

**NHC 通用参数**：

| 字段 | 类型 | 默认值 | 单位 | 说明 | 参考来源 |
|------|------|--------|------|------|---------------------|
| `nhc_gt` | int | `0` | — | 1=使用真值参考速度/姿态（仿真用） | `nhc_gt` |
| `nhc_lstm` | int | `0` | — | 1=使用 LSTM 预测速度/姿态（部署用，与 `nhc_gt` 互斥） | `nhc_lstm` |
| `nhc_start` | float | `0` | s | NHC 生效起始时刻（GPST 周内秒） | `nhc_start` |
| `nhc_end` | float | `604800` | s | NHC 生效结束时刻（GPST 周内秒） | `nhc_end` |
| `nhc_R_lateral` | float | `0.01` | m²/s² | 侧向速度观测噪声 R 初值 | `nhc_R_lateral` |
| `nhc_R_vertical` | float | `0.01` | m²/s² | 垂向速度观测噪声 R 初值（策略 6） | `nhc_R_vertical` |

**VBAKF 自适应参数**（策略 6/7）：

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `nhc_turn_alpha` | float | `0.9` | 偏航角变化率低通滤波系数 |
| `nhc_turn_beta` | float | `0.5` | 转弯强度平滑系数 |
| `nhc_turn_threshold` | float | `0.1` | 转弯判断阈值 |
| `nhc_turn_vel_threshold` | float | `2.0` | m/s | 触发转弯检测的最低前向速度 |
| `nhc_turn_gamma` | float | `10.0` | 转弯强度与 R 放大的映射系数 |
| `nhc_turn_smooth` | int | `5` | epoch | 转弯强度平滑窗口 |
| `nhc_vb_iter` | int | `3` | VBAKF 迭代次数 |
| `nhc_vb_zeta` | float | `1.0` | 遗忘因子 |
| `nhc_vb_nu` | float | `3.0` | 逆 Wishart 先验自由度 |

### 4.8 ZUPT 配置

> ZUPT（零速更新）与 NHC 互斥：静止时仅 ZUPT（3D，作用于 P），运动时仅 NHC。
> 框架通过速度阈值自动切换（参考 NHC 速度检测逻辑）。

| 字段 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `zupt_enable` | int | `0` | — | 0=关闭 / 1=启用零速更新 |
| `zupt_vel_threshold` | float | `0.5` | m/s | 零速检测速度阈值 |
| `zupt_R` | [float, float, float] | `[0.01, 0.01, 0.01]` | (m/s)² | ZUPT 三维速度观测噪声 R |
| `zupt_min_static_epoch` | int | `5` | epoch | 判定静止的最小连续历元数 |

### 4.9 杆臂与安装角

| 字段 | 类型 | 默认值 | 单位 | 说明 | 参考来源 |
|------|------|--------|------|------|---------------------|
| `leverarm` | [float, float, float] | `[0, 0, 0]` | m | GNSS 天线杆臂（b 系前右下） | `leverarm` |
| `antlever` | [float, float, float] | `[0, 0, 0]` | m | 天线杆臂（IMU 系前右下，参考 kf-gins） | `antlever` |
| `initial_imu_angle` | [float, float] | `[0, 0]` | deg | 初始 IMU 安装角 [pitch, yaw]（roll 假设为 0） | `initial_imu_angle` |
| `initial_imu_leverarm` | [float, float, float] | `[0, 0, 0]` | m | 初始 IMU 杆臂（b→v） | `initial_imu_leverarm` |
| `evaluate_imu_angle` | int | `0` | — | 1=估计 IMU 安装角和杆臂（StateIndex 参数块 5 维） | `evaluate_imu_angle` |
| `imu_angle_std` | [float, float] | `[10, 10]` | deg | IMU 安装角初始标准差 [pitch, yaw] | `imu_angle_std` |
| `imu_leverarm_std` | [float, float, float] | `[1, 1, 1]` | m | IMU 杆臂初始标准差 | `imu_leverarm_std` |

### 4.10 初始协方差

> 单滤波协方差 P_0，维度 = StateIndex.dim（默认 15，可选扩展）。

| 字段 | 类型 | 默认值 | 单位 | 说明 | 参考来源 |
|------|------|--------|------|------|---------------------|
| `use_define_variance_pos_vel` | int | `1` | — | 1=使用自定义位置速度方差 | `use_define_variance_pos_vel` |
| `use_define_variance_att` | int | `1` | — | 1=使用自定义姿态方差 | `use_define_variance_att` |
| `initial_pos_std` | [float, float, float] | `[0.5, 0.5, 0.5]` | m | 初始位置标准差 [N, E, D] | `initial_pos_std` |
| `initial_vel_std` | [float, float, float] | `[0.5, 0.5, 0.5]` | m/s | 初始速度标准差 [N, E, D] | `initial_vel_std` |
| `initial_att_std` | [float, float, float] | `[0.2, 0.2, 0.5]` | deg | 初始姿态标准差 [roll, pitch, yaw] | `initial_att_std` |

> 可选块（imu_angle, imu_leverarm, lever_arm, time_sync）的初始方差由对应 std 配置项自动填充。

### 4.11 GNSS 中断模拟

| 字段 | 类型 | 默认值 | 说明 | 参考来源 |
|------|------|--------|------|---------------------|
| `gnsslog_enable` | int | `0` | GNSS 日志使能 | `gnsslog_enable` |
| `gnss_break` | list | `[0, 30, 60, 0]` | 中断参数 [start, break_time, interval_time, break_number] | `gnss_break` |
| `gnss_break_array` | list | `[]` | 中断时间段数组（成对） | `gnss_break_array` |
| `gnss_break_type` | int | `0` | 0=用 `gnss_break` / 1=用 `gnss_break_array` | `gnss_break_type` |

---

## 5. 输出配置

| 字段 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `output_dir` | str | `"output"` | 输出目录 |
| `gnss_filename` | str | `"RTK.pos"` | 纯 GNSS 解算结果文件名（`.pos`，`ins.enabled=off/lc` 输出） |
| `aligned_filename` | str | `"aligned.csv"` | 对齐块状 CSV 文件名（`ins.enabled=lc` 输出） |
| `rslt_filename` | str | `"RTKLC.rslt"` | 组合导航结果文件名（`.rslt`，`ins.enabled=lc/tc` 输出，100Hz，ECEF 位置/速度 + 姿态） |
| `position_format` | str | `"llh"` | 位置输出格式：`llh`=经纬度高 (lat/lon/h) / `xyz`=ECEF 直角坐标 |
| `time_format` | str | `"gpst"` | 时间输出格式：`gpst`=GPS 周+周内秒 / `datetime`=YYYY/MM/DD HH:MM:SS.sss |
| `trace_level` | int | `0` | trace 文件级别：0=off / 1=info / 2=detail / 3=debug。生成与主输出同名的 `.trace` 文件，存放于 `output_dir` |
| `trace_enabled` | bool | `true` | trace 总开关；关闭时即使 `trace_level>0` 也不生成 `.trace` |
| `stat_level` | int | `0` | stat 级别：0=off / 1=基础状态 / 2=TC 更新摘要 / 3=逐卫星与矩阵明细（当前 S4 预留） |
| `stat_rate` | str | `"update"` | stat 采样速率：`update` / `second` / `imu` |
| `stat_filename` | str | `""` | stat 文件名；为空时使用主输出 stem + `.stat` |
| `log_raw_data` | bool | `false` | 是否记录原始数据到 raw/ |
| `log_level` | str | `"INFO"` | 运行日志级别：DEBUG/INFO/WARNING/ERROR |
| `terminal_summary_interval` | int | `10` | 终端摘要间隔（每 N 条解算结果） |
| `filter_debug_log_enable` | int | `0` | 1=输出滤波调试日志（F/H/P 矩阵等） |

---

## 6. 状态向量对应（单滤波, StateIndex）

### 6.1 固定 15 维（E 系, ψ-error）

| 索引 | 状态量 | 维度 | 对应配置项 |
|------|--------|------|-----------|
| 0–2 | 位置误差 δr^e | 3 | `initial_pos` / `initial_pos_std` |
| 3–5 | 速度误差 δv^e | 3 | `initial_vel` / `initial_vel_std` |
| 6–8 | 姿态误差 δψ^e | 3 | `initial_att` / `initial_att_std` |
| 9–11 | 陀螺零偏 δb_g | 3 | `initial_gyro_bias` / `gyro_bias_std` |
| 12–14 | 加计零偏 δb_a | 3 | `initial_acce_bias` / `acce_bias_std` |

### 6.2 可选参数块

| 索引 | 状态量 | 维度 | 启用条件 | 对应配置项 |
|------|--------|------|---------|-----------|
| 15–17 | GNSS 杆臂 δl_gnss | 3 | `estimate_leverarm=1` | `leverarm` / `lever_arm_std` |
| 18–19 | IMU 安装角 δθ_imu | 2 | `estimate_mounting_angle=1` | `imu_angle` / `imu_angle_std` |
| 20–22 | IMU 杆臂 δl_imu | 3 | `estimate_imu_leverarm=1`（需 mounting_angle） | `imu_leverarm` / `imu_leverarm_std` |
| 23 | 时间对齐 δt | 1 | `estimate_time_sync=1` | `time_sync_std` |

> 不包含比例因子误差（已放弃）。默认全 0 时 dim=15，全启用时 dim=24。

### 6.3 反馈机制

统一反馈（ψ-error）：位置/速度/姿态减号，零偏/杆臂/安装角/时间加号。详见 [estimator.md 第 5 节](file:///home/mxl/workplace/gipylib/skills/estimator.md#5-反馈机制)。

---

## 7. 数据文件说明

### 7.1 IMU 数据文件

| 文件 | 格式 | 说明 |
|------|------|------|
| `data/cpt_euroc.csv` | EuRoC 格式 | EuRoC 格式的 IMU 文件 |
| `data/cpt_imu.csv` | ADIS 格式 | ADIS 格式的 IMU 文件，逗号和空格都可以做分隔符 |

> IMU 文件解码由 `src/stream/formators.py` 实现，支持两种格式（由配置项 `ins.imu_format` 选择）：
> - **GPST 格式**（`imu_format: "gpst"`，由 `ImuFormator` 解码，对应 `data/cpt_imu.csv`）：列格式 `week,sow,gx,gy,gz,ax,ay,az`，解码时通过 `gpst_to_unix(week, sow)` 转为 Unix 时间戳。
> - **EuRoC 格式**（`imu_format: "euroc"`，由 `EuRoCImuFormator` 解码，对应 `data/cpt_euroc.csv`）：列格式 `timestamp_ns,wx,wy,wz,ax,ay,az`，解码时 `timestamp = timestamp_ns / 1e9`（Unix 纳秒 → Unix 秒），GPS 周号由 `unix_to_gpst` 派生；坐标系默认 RFU，由 `ImuSensor._convert_to_frd()` 转 FRD。
>
> `ImuSensor._create_formator(imu_format)` 工厂方法根据 `imu_format` 配置值创建对应解码器实例。

### 7.2 GNSS 观测值文件

| 文件 | 格式 | 说明 |
|------|------|------|
| `data/cpt0870.19o` | RINEX | 流动站观测值文件 |
| `data/cpt0870_base.19o` | RINEX | 基站观测值文件 |
| `data/brdm0870.19p` | RINEX | 星历文件（广播星历） |

> RINEX 解码由 `src/core/gnss/rtklib/rinex.py::rnx_decode` 实现（吸收自 rtklib-py）。
> 必要时由 `src/utility/rinex_improve.py` 按自动信号方案改写频点与跟踪码
> （LibGnut 移植见 `src/utility/gnutlib/`；旧 `rinex_simplifier.py` 保留兼容 API）。

### 7.3 GNSS 定位结果文件

| 文件 | 格式 | 说明 |
|------|------|------|
| `data/spp.pos` | POS | GNSS 定位结果文件（外部模式输入） |

> 外部 POS 文件解码由 `src/stream/formators.py::PosSolFormator` 实现，
> 时间字段（`yyyy/mm/dd hh:mm:ss.s`）通过 `ymdhms_to_gpst()` → `gpst_to_unix()` 转为 Unix 时间戳。
> 其他格式的定位结果文件（NMEA/CSV）当前未实现，仅支持 POS 格式（参考 `src/utility/config_loader.py::SUPPORTED_EXTERNAL_FORMATS`）。

### 7.4 输出文件

| 文件 | 格式 | 内容 | 触发模式 |
|------|------|------|---------|
| `output/RTK.pos` | POS | 纯 GNSS 解算结果（SPP/RTK），由 `src/log/solution_writer.py::SolutionWriter` 输出 | `ins.enabled=off/lc` |
| `output/aligned.csv` | CSV | 对齐块状输出（G + N 行 I），由 `src/log/aligned_writer.py::AlignedWriter` 输出 | `ins.enabled=lc` |
| `output/RTKLC.rslt` | RSLT | 组合导航结果（100Hz，ECEF 位置/速度 + 姿态），松组合由 `RSLTWriter` 输出 / 紧组合由 `TcStream` + `RSLTWriter` 输出 | `ins.enabled=lc/tc` |
| `output/<name>.trace` | 文本 | rtklib-py trace 输出，由 `src/log/trace_file_writer.py::TraceFileWriter` 重定向，Unix 时间戳转 GPS 周+周内秒，过滤无效调试行 | `trace_level>0` |
| `output/raw/imu_raw.csv` | CSV | IMU 原始数据 | `log_raw_data=true`（可选） |
| `output/raw/rover_raw.csv` | CSV | 流动站原始数据（内部模式） | `log_raw_data=true`（可选） |

> 输出文件按 `ins.enabled` 启用：
> - `off`（纯 GNSS）：`gnss_filename`（`.pos`，`SolutionLogger` + `SolutionWriter`）
> - `on`（松组合）：`gnss_filename`（`.pos`）+ `aligned_filename`（`.csv`，`AlignedWriter`）+ `rslt_filename`（`.rslt`，100Hz，`RSLTWriter`）
> - `tc`（紧组合）：`rslt_filename`（`.rslt`，100Hz，`TcStream` + `RSLTWriter`）
>
> `SolutionWriter` 与 `RSLTWriter` 接受 `position_format` 与 `time_format` 参数控制输出格式。
> `trace_level>0` 时生成 `.trace` 文件（与主输出同名，存放于 `output_dir`），由 `TraceFileWriter` 重定向 rtklib-py trace 输出。
> `config_loader.py` 校验 `ins.enabled=off` 时 `gnss_source` 必须为 `internal`；`ins.enabled=lc/tc` 时 `imu_data_path` 必填。
