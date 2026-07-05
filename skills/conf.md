# 配置文件说明

> 定义 GInsStream 统一定位解算配置文件的格式、字段与默认值。
> 配置文件采用 YAML 格式，存放于 `data/config.yaml`（测试样例：`data/cfg_test_spp.yaml`、`data/cfg_test_rtk.yaml`）。
>
> **配置来源约定**：
> - **GNSS 部分配置项**：参考 rtklib-py 的 `config_phone.py` / `config_f9p.py` / `__ppk_config.py`
>   （rtklib-py 已吸收到 `src/core/gnss/rtklib/`，配置通过 `src/core/gnss/rtklib_config_adapter.py` 注入）
> - **组合导航部分配置项**：参考 `tools/gnss_ins_lc_nhc` 项目的 `config/configure.ini`
> - **文件格式**：仿照 `tools/KF-GINS/config/kf-gins.yaml` 的中英双语注释 YAML 风格
>
> **时间系统约定**：全框架内部统一使用 **Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）**。
> 输入端（`src/stream/formators.py`）将 GPS 周+周内秒通过 `gpst_to_unix()` 转为 Unix 时间戳；
> 输出端（`src/log/solution_writer.py` / `aligned_writer.py`）通过 `unix_to_gpst()` 转回 (week, sow) 写文件。
> 时间转换工具位于 `src/core/time_utils.py`，常量 `GPST_EPOCH_UNIX = 315964800`。
>
> **当前项目状态**：
> - ✅ 已实现：内部 GNSS 模式（SPP/RTK，`gnss_source: "internal"` + `ins.enabled: "off"`）→ 输出 `.pos` 文件
> - ✅ 已实现：外部 GNSS 模式（`gnss_source: "external"` + `ins.enabled: "on"`）→ 输出对齐块状 CSV
> - ✅ 已实现：内部 GNSS + IMU 对齐模式（`gnss_source: "internal"` + `ins.enabled: "on"`）→ 实时 RTK/SPP 解算 + IMU 流式读取 → Aligner 匹配 → 输出对齐块状 CSV（数据对齐管线，不含 INS EKF）
> - ✅ 已实现：INS 初始化模块 `src/core/ins/initializer.py::InsInitializer`（三种模式：静态 / 速度矢量 / 位置差分，三阈值检验），详见 [初始化.md](file:///home/mxl/workplace/gipylib/skills/初始化.md)
> - ✅ 已实现：`src/core/ins/` 下 `interpolator.py` / `earth_param.py` / `attitude.py`（初始化支撑模块）
> - ✅ 已实现：`ImuSensor` RFU→FRD 坐标系自动转换（`_convert_to_frd()`）
> - ✅ 已实现：SPP 多普勒测速（`pntpos.py::estvel` / `resdop`，速度填入 `sol.rr[3:6]`）
> - 🚧 预留：INS 机械编排核心 `InsCore` / 双滤波 EKF `LcIntegration` / NHC / ZUPT / 紧组合接口（下一阶段实现）
>
> **三种运行模式对比**：
> | 模式 | gnss_source | ins.enabled | GNSS 来源 | IMU | 输出 | Logger |
> |------|-------------|-------------|-----------|-----|------|--------|
> | 纯 GNSS | internal | off | 实时解算 | 无 | `.pos` | SolutionLogger |
> | 外部对齐 | external | on | 外部文件 | 流式 | `aligned.csv` | Logger |
> | 内部对齐 | internal | on | 实时解算 | 流式 | `aligned.csv` | Logger |
>
> **双滤波架构对应**（INS 启用后的设计，当前未实现）：
> - 主滤波 P1（E 系，15 维 = 位置3+速度3+姿态3+陀螺零偏3+加计零偏3；可选 +3 维 GNSS 杆臂 = 18 维）
> - NHC 子滤波 P2（v 系，5 维 = IMU 安装角 2 + IMU 杆臂 3）
> - NHC/ZUPT 互斥：静止时仅 ZUPT（3D，作用于 P1），运动时仅 NHC（H1 作用于 P1 + H2 作用于 P2）

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
  - [4. 组合导航配置项（参考 gnss\_ins\_lc\_nhc）](#4-组合导航配置项参考-gnss_ins_lc_nhc)
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
  - [6. 双滤波状态向量对应](#6-双滤波状态向量对应)
    - [6.1 主滤波 P1（E 系，误差状态）](#61-主滤波-p1e-系误差状态)
    - [6.2 NHC 子滤波 P2（v 系，误差状态）](#62-nhc-子滤波-p2v-系误差状态)
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
2. **组合导航配置**（INS 机械编排 + 双滤波 EKF + NHC/ZUPT，参考 gnss_ins_lc_nhc）
3. **输出配置**（POS/CSV/NMEA 格式、日志级别）

配置文件支持两种 GNSS 数据源模式：
- **内部解算模式**（`gnss_source: "internal"`）：从原始观测值解算 SPP/RTK/RTD
- **外部结果模式**（`gnss_source: "external"`）：直接读取外部 GNSS 定位结果文件（POS/NMEA/CSV）

---

## 2. 配置文件格式

- 文件格式：YAML 1.1
- 文件位置：`data/config.yaml`（测试样例：`data/cfg_test_spp.yaml`、`data/cfg_test_rtk.yaml`）
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
| `excsats` | list | `[]` | — | 排除卫星列表（如 `["G01", "G23"]`） | `excsats` |

### 3.4 周跳与粗差检测

| 字段 | 类型 | 默认值 | 单位 | 说明 | rtklib-py 对应 |
|------|------|--------|------|------|---------------|
| `maxinno` | float | `1.0` | m | 载波相位周跳/粗差阈值 | `maxinno` |
| `maxcode` | float | `10.0` | m | 伪距粗差阈值 | `maxcode` |
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
| `prnbias` | float | `0.01` | cycles | 载波相位偏差 sigma | `prnbias` |
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

| 字段 | 类型 | 默认值 | 说明 | rtklib-py 对应 |
|------|------|--------|------|---------------|
| `gnss_t` | list | `["GPS", "GLO", "GAL"]` | 启用星座列表 | `gnss_t` |
| `freq_ix0` | dict | `{GPS: 0, GLO: 4, GAL: 0}` | 第一频率索引（L1） | `freq_ix0` |
| `freq_ix1` | dict | `{GPS: 2, GLO: 5, GAL: 2}` | 第二频率索引（L5/E5b） | `freq_ix1` |
| `freq_table` | list | `[1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9]` | 支持频率表 (Hz) | `freq` |
| `dfreq_glo` | [float, float] | `[0.56250e6, 0.43750e6]` | Hz | GLONASS 频率间隔 [L1, L2] | `dfreq_glo` |

**支持的信号类型**（sig_tbl 映射）：

| 信号码 | rtklib-py 枚举 |
|--------|---------------|
| `1C` / `1X` / `1W` | L1C / L1X / L1W |
| `2W` / `2L` / `2C` / `2X` | L2W / L2L / L2C / L2X |
| `5Q` / `5X` | L5Q / L5X |
| `7Q` / `7X` | L7Q / L7X |

### 3.9 基站与初始位置

| 字段 | 类型 | 默认值 | 单位 | 说明 | rtklib-py 对应 |
|------|------|--------|------|------|---------------|
| `rb` | [float, float, float] | `[0, 0, 0]` | m | 基站 ECEF 位置（0,0,0 表示用 RINEX 头） | `rb` |
| `rr_f` | list | `[0,0,0,0,0,0]` | m, m/s | 流动站初始位置速度（正向，0 表示自动单精解） | `rr_f` |
| `rr_b` | list | `[0,0,0,0,0,0]` | m, m/s | 流动站初始位置速度（反向，0 表示自动单精解） | `rr_b` |

---

## 4. 组合导航配置项（参考 gnss_ins_lc_nhc）

> 以下配置项映射自 `tools/gnss_ins_lc_nhc/config/configure.ini`，命名与原项目一致。
> 坐标系约定：机械编排在 E 系（ECEF）下进行，IMU 体坐标系为 FRD，车体坐标系为 FRD。

### 4.1 处理时间

| 字段 | 类型 | 默认值 | 单位 | 说明 | gnss_ins_lc_nhc 对应 |
|------|------|--------|------|------|---------------------|
| `process_date` | [int, int, int] | `[2024, 12, 20]` | — | 处理日期 [year, month, day] | `process_date` |
| `start_time` | float | `0` | s | 起始时间（当日秒或 GPST 周） | `start_time` |
| `end_time` | float | `-1` | s | 结束时间（-1 表示处理至文件结束） | `end_time` |

### 4.2 使能开关

| 字段 | 类型 | 默认值 | 说明 | gnss_ins_lc_nhc 对应 |
|------|------|--------|------|---------------------|
| `gnss_enable` | int | `1` | GNSS 量测使能（0=关闭） | `gnss_enable` |
| `imu_enable` | int | `1` | IMU 机械编排放能 | `imu_enable` |
| `nhc_enable` | int | `2` | NHC 策略（0-9，见 4.7） | `nhc_enable` |
| `zupt_enable` | int | `0` | ZUPT 策略（0=关闭，1=启用 3D 零速更新；与 NHC 互斥） | GInsStream 扩展 |

### 4.3 数据路径与采样率

| 字段 | 类型 | 默认值 | 单位 | 说明 | gnss_ins_lc_nhc 对应 |
|------|------|--------|------|------|---------------------|
| `imu_data_path` | str | `""` | — | IMU 数据文件路径 | `imu_data_path` |
| `gnss_data_path` | str | `""` | — | GNSS 数据文件路径（外部模式结果文件） | `gnss_data_path` |
| `result_output_path` | str | `"output/"` | — | 结果输出目录 | `result_output_path` |
| `data_rate` | int | `100` | Hz | IMU 采样率 | `data_rate` |
| `result_output_rate` | int | `1` | Hz | 结果输出率 | `result_output_rate` |
| `imudatalen` | int | `7` | — | IMU 文件列数（只用前 7 列） | 参考 kf-gins |

### 4.4 初始对准

> 详见 [初始化.md 第 5 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#5-初始化模式分类) 和 [第 5.4 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#54-动态初始化默认模式选择基于-gnss-解算模式)。
> **当前实现状态**：`InsInitializer` 已实现三种模式（静态 / 速度矢量 / 位置差分），详见 [初始化.md](file:///home/mxl/workplace/gipylib/skills/初始化.md)。

| 字段 | 类型 | 默认值 | 单位 | 说明 | gnss_ins_lc_nhc 对应 |
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

| 字段 | 类型 | 默认值 | 单位 | 说明 | gnss_ins_lc_nhc 对应 |
|------|------|--------|------|------|---------------------|
| `initial_pos` | [float, float, float] | `[0, 0, 0]` | deg, deg, m | 初始位置 [lat, lon, alt] | `initial_pos` |
| `initial_vel` | [float, float, float] | `[0, 0, 0]` | m/s | 初始速度 [N, E, D] | `initial_vel` |
| `initial_att` | [float, float, float] | `[0, 0, 0]` | deg | 初始姿态 [roll, pitch, yaw]（ZYX 旋转） | `initial_att` |
| `initial_gyro_bias` | [float, float, float] | `[0, 0, 0]` | deg/h | 初始陀螺零偏 | `initial_gyro_bias` |
| `initial_acce_bias` | [float, float, float] | `[0, 0, 0]` | mGal | 初始加计零偏 | `initial_acce_bias` |
| `initial_gyro_scale` | [float, float, float] | `[0, 0, 0]` | ppm | 初始陀螺比例因子 | `initial_gyro_scale` |
| `initial_acce_scale` | [float, float, float] | `[0, 0, 0]` | ppm | 初始加计比例因子 | `initial_acce_scale` |
| `evaluate_imu_scale` | int | `0` | — | 1=估计 IMU 比例因子（P1 扩展 6 维） | `evaluate_imu_scale` |

### 4.6 IMU 噪声参数

| 字段 | 类型 | 默认值 | 单位 | 说明 | gnss_ins_lc_nhc 对应 |
|------|------|--------|------|------|---------------------|
| `velocity_random_walk` | [float, float, float] | `[0.1, 0.1, 0.1]` | m/s/√hr | 速度随机游走 (VRW) | `velocity_random_walk` |
| `attitude_random_walk` | [float, float, float] | `[0.1, 0.1, 0.1]` | deg/√hr | 姿态随机游走 (ARW) | `attitude_random_walk` |
| `gyro_bias_std` | [float, float, float] | `[25, 25, 25]` | deg/h | 陀螺零偏标准差 | `gyro_bias_std` |
| `acce_bias_std` | [float, float, float] | `[200, 200, 200]` | mGal | 加计零偏标准差 | `acce_bias_std` |
| `gyro_scale_std` | [float, float, float] | `[500, 500, 500]` | ppm | 陀螺比例因子标准差 | `gyro_scale_std` |
| `acce_scale_std` | [float, float, float] | `[500, 500, 500]` | ppm | 加计比例因子标准差 | `acce_scale_std` |
| `corr_time_of_gyro_bias` | float | `0.01` | h | 陀螺零偏相关时间 | `corr_time_of_gyro_bias` |
| `corr_time_of_acce_bias` | float | `0.01` | h | 加计零偏相关时间 | `corr_time_of_acce_bias` |
| `corr_time_of_gyro_scale` | float | `0.01` | h | 陀螺比例因子相关时间 | `corr_time_of_gyro_scale` |
| `corr_time_of_acce_scale` | float | `0.01` | h | 加计比例因子相关时间 | `corr_time_of_acce_scale` |
| `position_random_walk` | [float, float, float] | `[0, 0, 0]` | — | 位置随机游走 | `position_random_walk` |

### 4.7 NHC 配置

NHC（非完整性约束）利用车辆运动学假设（车轮不侧滑、不腾空）作为虚拟速度观测，在 GNSS 中断期间抑制惯性推算误差发散。

**`nhc_enable` 策略表**（参考 gnss_ins_lc_nhc）：

| `nhc_enable` | 观测维度 | 坐标系 | 观测量 | 作用目标 | 备注 |
|:---:|:---:|:---:|---|---|---|
| `0` | — | — | 不启用 NHC | — | — |
| `1` | 1D | v 系 | 侧向速度 | P1（H1）+ P2（H2） | 需开启 `evaluate_imu_angle` |
| `2` | 2D | v 系 | 侧向 + 垂向速度 | P1（H1）+ P2（H2） | 默认策略 |
| `3` | 3D | v 系 | 前向 + 侧向 + 垂向 | P1（H1）+ P2（H2） | 高速时放宽前向 R |
| `4` | 3D | n 系 | 姿态（roll/pitch/yaw） | P1 | LSTM/GT 姿态虚拟观测 |
| `5` | 6D | v+n | 姿态 3D + 速度 3D | P1 + P2 | 联合约束 |
| `6` | 2D | v 系 | 侧向 + 垂向 | P1 + P2 | Ga-St VBAKF 自适应 |
| `7` | 1D | v 系 | 侧向 | P1 + P2 | Ga-St VBAKF 自适应 |
| `8` | 3D | n 系（ENU） | 东+北+天速度 | P1 | LSTM/GT ENU 速度 |
| `9` | 2D | v 系 | 前向 + 侧向 | P1 + P2 | 速度大时放宽 R |

**NHC 通用参数**：

| 字段 | 类型 | 默认值 | 单位 | 说明 | gnss_ins_lc_nhc 对应 |
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

> ZUPT（零速更新）与 NHC 互斥：静止时仅 ZUPT（3D，作用于 P1），运动时仅 NHC。
> 框架通过速度阈值自动切换（参考 NHC 速度检测逻辑）。

| 字段 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `zupt_enable` | int | `0` | — | 0=关闭 / 1=启用零速更新 |
| `zupt_vel_threshold` | float | `0.5` | m/s | 零速检测速度阈值 |
| `zupt_R` | [float, float, float] | `[0.01, 0.01, 0.01]` | (m/s)² | ZUPT 三维速度观测噪声 R |
| `zupt_min_static_epoch` | int | `5` | epoch | 判定静止的最小连续历元数 |

### 4.9 杆臂与安装角

| 字段 | 类型 | 默认值 | 单位 | 说明 | gnss_ins_lc_nhc 对应 |
|------|------|--------|------|------|---------------------|
| `leverarm` | [float, float, float] | `[0, 0, 0]` | m | GNSS 天线杆臂（b 系前右下） | `leverarm` |
| `antlever` | [float, float, float] | `[0, 0, 0]` | m | 天线杆臂（IMU 系前右下，参考 kf-gins） | `antlever` |
| `initial_imu_angle` | [float, float] | `[0, 0]` | deg | 初始 IMU 安装角 [pitch, yaw]（roll 假设为 0） | `initial_imu_angle` |
| `initial_imu_leverarm` | [float, float, float] | `[0, 0, 0]` | m | 初始 IMU 杆臂（b→v） | `initial_imu_leverarm` |
| `evaluate_imu_angle` | int | `0` | — | 1=估计 IMU 安装角和杆臂（P2 子滤波 5 维） | `evaluate_imu_angle` |
| `imu_angle_std` | [float, float] | `[10, 10]` | deg | IMU 安装角初始标准差 [pitch, yaw] | `imu_angle_std` |
| `imu_leverarm_std` | [float, float, float] | `[1, 1, 1]` | m | IMU 杆臂初始标准差 | `imu_leverarm_std` |

### 4.10 初始协方差

> 双滤波协方差：P1_0（主滤波）与 P2_0（NHC 子滤波）分别设置。

| 字段 | 类型 | 默认值 | 单位 | 说明 | 作用目标 | gnss_ins_lc_nhc 对应 |
|------|------|--------|------|------|---------|---------------------|
| `use_define_variance_pos_vel` | int | `1` | — | 1=使用自定义位置速度方差 | P1 | `use_define_variance_pos_vel` |
| `use_define_variance_att` | int | `1` | — | 1=使用自定义姿态方差 | P1 | `use_define_variance_att` |
| `initial_pos_std` | [float, float, float] | `[0.5, 0.5, 0.5]` | m | 初始位置标准差 [N, E, D] | P1 | `initial_pos_std` |
| `initial_vel_std` | [float, float, float] | `[0.5, 0.5, 0.5]` | m/s | 初始速度标准差 [N, E, D] | P1 | `initial_vel_std` |
| `initial_att_std` | [float, float, float] | `[0.2, 0.2, 0.5]` | deg | 初始姿态标准差 [roll, pitch, yaw] | P1 | `initial_att_std` |

> P2_0（NHC 子滤波）由 `imu_angle_std` 和 `imu_leverarm_std` 自动构造（见 4.9）。

### 4.11 GNSS 中断模拟

| 字段 | 类型 | 默认值 | 说明 | gnss_ins_lc_nhc 对应 |
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
| `solution_filename` | str | `"solution.pos"` | 内部模式输出 `.pos` 文件名（如 `test_spp.pos`、`test_rtk.pos`） |
| `solution_format` | str | `"pos"` | 解算结果格式：`pos` / `csv` / `nmea` |
| `trace_level` | int | `1` | 轨迹输出级别：0=无 / 1=基本 / 2=详细 / 3=调试 |
| `log_raw_data` | bool | `false` | 是否记录原始数据到 raw/ |
| `log_level` | str | `"INFO"` | 运行日志级别：DEBUG/INFO/WARNING/ERROR |
| `terminal_summary_interval` | int | `10` | 终端摘要间隔（每 N 条解算结果） |
| `filter_debug_log_enable` | int | `0` | 1=输出滤波调试日志（F/H/P 矩阵等） |

---

## 6. 双滤波状态向量对应

### 6.1 主滤波 P1（E 系，误差状态）

| 索引 | 状态量 | 维度 | 是否必选 | 对应配置项 |
|------|--------|------|---------|-----------|
| 0–2 | 位置误差 δr_e | 3 | 必选 | `initial_pos` / `initial_pos_std` |
| 3–5 | 速度误差 δv_e | 3 | 必选 | `initial_vel` / `initial_vel_std` |
| 6–8 | 姿态误差 δθ | 3 | 必选 | `initial_att` / `initial_att_std` |
| 9–11 | 陀螺零偏 ε_b | 3 | 必选 | `initial_gyro_bias` / `gyro_bias_std` |
| 12–14 | 加计零偏 ∇_b | 3 | 必选 | `initial_acce_bias` / `acce_bias_std` |
| 15–17 | 陀螺比例因子 | 3 | 可选（`evaluate_imu_scale=1`） | `initial_gyro_scale` / `gyro_scale_std` |
| 18–20 | 加计比例因子 | 3 | 可选（`evaluate_imu_scale=1`） | `initial_acce_scale` / `acce_scale_std` |
| 21–23 | GNSS 杆臂 | 3 | 可选 | `leverarm` |

### 6.2 NHC 子滤波 P2（v 系，误差状态）

| 索引 | 状态量 | 维度 | 是否必选 | 对应配置项 |
|------|--------|------|---------|-----------|
| 0–1 | IMU 安装角误差 δθ_imu [pitch, yaw] | 2 | 必选 | `initial_imu_angle` / `imu_angle_std` |
| 2–4 | IMU 杆臂误差 δl_imu | 3 | 必选 | `initial_imu_leverarm` / `imu_leverarm_std` |

> P2 仅在 `evaluate_imu_angle=1` 且 `nhc_enable≠0` 时启用。

### 6.3 反馈机制

- **P1 反馈**：GNSS 量测更新、ZUPT 更新、NHC H1 部分更新后反馈至 INS 状态（位置、速度、姿态、零偏、比例因子、杆臂）
- **P2 反馈**：NHC H2 部分更新后独立反馈至 IMU 安装角和杆臂

---

## 7. 数据文件说明

### 7.1 IMU 数据文件

| 文件 | 格式 | 说明 |
|------|------|------|
| `data/cpt_euroc.csv` | EuRoC 格式 | EuRoC 格式的 IMU 文件 |
| `data/cpt_imu.csv` | ADIS 格式 | ADIS 格式的 IMU 文件，逗号和空格都可以做分隔符 |

> IMU 文件解码由 `src/stream/formators.py::ImuFormator` 实现，列格式：`week,sow,gx,gy,gz,ax,ay,az`。
> 解码时通过 `gpst_to_unix(week, sow)` 转为 Unix 时间戳。

### 7.2 GNSS 观测值文件

| 文件 | 格式 | 说明 |
|------|------|------|
| `data/cpt0870.19o` | RINEX | 流动站观测值文件 |
| `data/cpt0870_base.19o` | RINEX | 基站观测值文件 |
| `data/brdm0870.19p` | RINEX | 星历文件（广播星历） |

> RINEX 解码由 `src/core/gnss/rtklib/rinex.py::rnx_decode` 实现（吸收自 rtklib-py）。
> 必要时由 `src/utility/rinex_simplifier.py` 简化为 2 频点。

### 7.3 GNSS 定位结果文件

| 文件 | 格式 | 说明 |
|------|------|------|
| `data/spp.pos` | POS | GNSS 定位结果文件（外部模式输入） |

> 外部 POS 文件解码由 `src/stream/formators.py::PosSolFormator` 实现，
> 时间字段（`yyyy/mm/dd hh:mm:ss.s`）通过 `ymdhms_to_gpst()` → `gpst_to_unix()` 转为 Unix 时间戳。
> 其他格式的定位结果文件（NMEA/CSV）当前未实现，仅支持 POS 格式（参考 `src/utility/config_loader.py::SUPPORTED_EXTERNAL_FORMATS`）。

### 7.4 输出文件

| 文件 | 格式 | 内容 | 必须 |
|------|------|------|------|
| `output/test_spp.pos` / `output/test_rtk.pos` | POS | 内部纯 GNSS 模式解算结果（SPP/RTK），由 `src/log/solution_writer.py::SolutionWriter` 输出 | 是（internal+off 模式） |
| `output/aligned.csv` | CSV | 外部对齐模式块状输出（G + N 行 I），由 `src/log/aligned_writer.py::AlignedWriter` 输出 | 是（external 模式） |
| `output/aligned_internal_rtk.csv` | CSV | 内部对齐模式块状输出（G + N 行 I），实时 RTK 解算 + IMU 对齐 | 是（internal+on 模式） |
| `output/solution.csv` | CSV | 解算结果（完整状态） | 可选（未实现） |
| `output/solution.nmea` | NMEA | NMEA 格式 | 可选（未实现） |
| `output/trace.txt` | 文本 | 运行轨迹/调试信息 | 可选（未实现） |
| `output/raw/imu_raw.csv` | CSV | IMU 原始数据 | 可选（未实现） |
| `output/raw/rover_raw.csv` | CSV | 流动站原始数据（内部模式） | 可选（未实现） |

> 内部纯 GNSS 模式（`internal` + `ins.enabled: "off"`）主入口：`src/main.py` → 路径 B → `SolutionLogger` + `SolutionWriter`，
> 输出 `.pos` 文件，文件名由 `output.solution_filename` 指定（如 `test_spp.pos`、`test_rtk.pos`）。
> 外部对齐模式（`external` + `ins.enabled: "on"`）主入口：`src/main.py` → 路径 A → `Logger` + `AlignedWriter` + `Aligner`，
> 输出对齐块状 CSV，文件名默认 `aligned.csv`。
> 内部对齐模式（`internal` + `ins.enabled: "on"`）主入口：`src/main.py` → 路径 C → `Logger` + `AlignedWriter` + `Aligner`，
> 输出对齐块状 CSV，文件名由 `output.aligned_filename` 指定（如 `aligned_internal_rtk.csv`）。
> `config_loader.py` 校验 `internal + on` 时要求 `ins.imu_data_path` 必须存在（不再抛 `NotImplementedError`）。
