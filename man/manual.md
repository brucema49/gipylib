# 配置文件使用手册

本手册详细讲解项目 YAML 配置文件中各参数的含义、**单位**、默认值、取值范围与**适用模式（松组合 LC / 紧组合 TC / 纯 GNSS）**。`data/` 配置用于 CPT 等参考数据；`data-great/rtktc-rate.yaml` 与 `data-great/rtktc-increment.yaml` 是与 GREAT-MSF campus01 单频 strict-float 对照的紧组合配置。不同数据集的噪声和星座设置不能直接混用。

> **当前状态（2026-09-15）**：本手册以当前源码为准。要点：
> - RTK 浮点基线使用 `prnbias=0.03`；相位 `maxinno`（5 m）与伪距 `maxcode`（30 m）分开设置。
> - **INS 参数分为三段**：`ins`（松紧组合共用，即 usually）、`ins_tc`（仅紧组合）、`ins_lc`（仅松组合）。
> - **`pos_psd` / `vel_psd` 是松组合专属项，不能用于紧组合**（见 [3.0](#30-ins-参数的三段结构) 与 [3.8](#38-过程噪声-psd)）。
> - 与 GREAT 对照时的零偏初值/过程模型结论见 `issue/9-14机械编排发散.md`。
> - 实验结果必须同时记录数据集、模式、配置快照、评估窗口和更新/传播点规则。

配置文件采用 YAML 格式，包含段：`gnss`（GNSS 解算）、`ins`（组合导航**共用**参数）、`ins_tc`（紧组合专属 INS 参数）、`ins_lc`（松组合专属 INS 参数）、`tc`（紧组合滤波配置）、`output`（输出配置）。

运行命令：`python3 src/main.py <config_path>`（默认 `data/config.yaml`）。

---

## 目录

- [1. 运行模式](#1-运行模式)
- [2. GNSS 配置 (gnss)](#2-gnss-配置-gnss)
  - [2.1 数据源与文件路径](#21-数据源与文件路径)
  - [2.2 定位模式](#22-定位模式)
  - [2.3 观测值筛选](#23-观测值筛选)
  - [2.4 周跳与粗差检测](#24-周跳与粗差检测)
  - [2.5 卡尔曼滤波统计](#25-卡尔曼滤波统计)
  - [2.6 模糊度解算](#26-模糊度解算)
  - [2.7 单点定位参数](#27-单点定位参数)
  - [2.8 星座与信号配置](#28-星座与信号配置)
  - [2.9 基站与初始位置](#29-基站与初始位置)
- [3. INS 配置 (ins / ins_tc / ins_lc)](#3-ins-配置-ins--ins_tc--ins_lc)
  - [3.0 INS 参数的三段结构](#30-ins-参数的三段结构)
  - [3.1 主开关](#31-主开关)
  - [3.2 数据路径与采样率](#32-数据路径与采样率)
  - [3.3 使能开关（NHC/ZUPT/ZARU）](#33-使能开关nhczuptzaru)
  - [3.4 初始对准](#34-初始对准)
  - [3.5 IMU 误差标定](#35-imu-误差标定)
  - [3.6 杆臂与安装角](#36-杆臂与安装角)
  - [3.7 初始不确定度](#37-初始不确定度)
  - [3.8 过程噪声 PSD](#38-过程噪声-psd)
  - [3.9 NHC 配置](#39-nhc-配置)
  - [3.10 ZUPT 配置](#310-zupt-配置)
  - [3.11 ZARU 配置](#311-zaru-配置)
  - [3.12 静态检测配置](#312-静态检测配置)
  - [3.13 可选状态参数开关](#313-可选状态参数开关)
  - [3.14 单位与 PSD 换算总表](#314-单位与-psd-换算总表)
- [4. 紧组合配置 (tc)](#4-紧组合配置-tc)
- [5. 输出配置 (output)](#5-输出配置-output)
- [6. 配置校验规则](#6-配置校验规则)
- [7. 典型配置示例](#7-典型配置示例)

---

## 1. 运行模式

运行模式由 `ins.enabled` 字段控制（不再使用 `coupling_mode`）：

| `ins.enabled` | 模式 | 说明 | 输出文件 |
|---------------|------|------|----------|
| `off` | 纯 GNSS | 仅 GNSS 解算（SPP/RTK/RTD），不使用 IMU | `.pos`（1Hz） |
| `lc` | 松组合（LC） | GNSS 定位结果 + IMU 机械编排 + EKF 融合 | `.pos` + `.csv` + `.rslt`（100Hz） |
| `tc` | 紧组合（TC） | GNSS 原始观测值 + IMU 机械编排 + EKF 融合 | `.rslt`（100Hz） |

**约束**：`ins.enabled=off` 时，`gnss.gnss_source` 必须为 `internal`（纯 GNSS 模式不支持外部结果输入）。

---

## 2. GNSS 配置 (gnss)

GNSS 配置部分参考 rtklib-py，已吸收到 `src/core/gnss/rtklib/`。

### 2.1 数据源与文件路径

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `gnss_source` | str | `"internal"` | GNSS 数据源：`internal`=内部解算；`external`=读取外部结果文件 |
| `positioning_mode` | str | `"rtk"` | 定位模式（仅 `internal` 生效）：`spp`/`rtd`/`rtk` |
| `rover_path` | str | `"data/cpt0870.19o"` | 流动站观测文件路径（`internal` 必填） |
| `base_path` | str | `"data/cpt0870_base.19o"` | 基站观测文件路径（`internal` + `rtk`/`rtd` 必填） |
| `eph_path` | str | `"data/brdm0870.19p"` | 星历文件路径（`internal` 必填） |
| `external_sol_path` | str | `""` | 外部 GNSS 结果文件路径（`external` 必填） |
| `external_sol_format` | str | `"pos"` | 外部结果格式：`pos`/`nmea`/`csv`（当前仅支持 `pos`） |
| `default_pos_std` | float | `1.0` | 外部结果默认位置标准差 [m] |
| `default_vel_std` | float | `0.5` | 外部结果默认速度标准差 [m/s] |
| `pos_covariance_available` | bool | `false` | 外部结果是否含协方差信息 |

### 2.2 定位模式

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `nf` | int | `1` | 频率数：1=单频（L1）；2=双频（L1+L2/L5） |
| `pmode` | str | `"kinematic"` | rtklib-py 定位模式：`static`/`kinematic` |
| `filtertype` | str | `"forward"` | 滤波类型：`forward`/`backward`/`combined`/`combined_noreset` |
| `use_sing_pos` | bool | `false` | 每历元是否重新单点定位初始化位置 |

### 2.3 观测值筛选

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `elmin` | float | `10.0` | deg | 最小仰角，低于此仰角的卫星被剔除 |
| `cnr_min` | array[2] | `[28, 20]` | dB-Hz | 最小信噪比阈值 [freq1, freq2] |
| `excsats` | array | `[]` | — | 排除卫星列表，如 `["G01", "G23"]` |

### 2.4 周跳与粗差检测

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `maxinno` | float | `5.0` | m | 载波相位周跳/粗差阈值；代码中作为相位双差门限 |
| `maxcode` | float | `30.0` | m | 伪距双差粗差阈值；手机 RTD-TC 已验证配置使用 `60.0` 以容纳较大的初始状态误差 |
| `maxage` | float | `30.0` | s | 最大差分龄期 |
| `maxout` | int | `4` | — | 最大差分中断历元数 |
| `thresdop` | float | `5.0` | — | 多普勒法周跳检测阈值 |
| `thresslip` | float | `0.10` | — | 几何无关组合（LG）周跳检测阈值 |
| `interp_base` | bool | `false` | — | 是否插值基站观测值 |

### 2.5 卡尔曼滤波统计

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `eratio` | array[2] | `[300, 100]` | — | 伪距/载波噪声比 [L1, L2/L5] |
| `efact_gps` | float | `1.0` | — | GPS 星座相对权重 |
| `efact_glo` | float | `1.5` | — | GLONASS 星座相对权重 |
| `efact_gal` | float | `1.0` | — | Galileo 星座相对权重 |
| `err_base` | float | `0.003` | m | 基线误差 sigma |
| `err_el` | float | `0.003` | m | 仰角误差 sigma |
| `err_satclk` | float | `5.0e-12` | s | 卫星钟误差 sigma |
| `snrmax` | float | `45.0` | dB-Hz | 方差计算最大信噪比 |
| `accelh` | float | `20.0` | m/s² | 水平加速度噪声 sigma（增大以跟踪车辆动态） |
| `accelv` | float | `10.0` | m/s² | 垂直加速度噪声 sigma |
| `pos_psd` | float | `0.0` | m²/s | **RTKLIB 滤波器**的位置随机游走 PSD（经 `rtklib_config_adapter` 传给 `params["pos_psd"]`）。⚠️ 与 `ins_lc.pos_psd` 是**两个不同参数**：前者属于 `gnss` 段的 RTKLIB GNSS 滤波，后者属于 INS 的 EKF `Q`，且后者为松组合专属 |
| `prnbias` | float | `0.5` | cycles | 载波相位偏差 sigma（增大防止 P_amb 坍缩） |
| `sig_p0` | float | `30.0` | m | 初始位置 sigma |
| `sig_v0` | float | `10.0` | m/s | 初始速度/加速度 sigma |
| `sig_n0` | float | `30.0` | m | 初始模糊度 sigma |

### 2.6 模糊度解算

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `armode` | int | `0` | 模糊度解算模式：0=off / 1=continuous / 3=fix-and-hold |
| `thresar` | float | `3.0` | AR ratio 检验阈值 |
| `thresar1` | float | `0.5` | AR 位置方差阈值 [m²]（posvar 超此值时跳过 AR） |
| `minlock` | int | `0` | 参与 AR 的最小连续锁定历元数 |
| `glo_hwbias` | float | `0.0` | GLONASS 硬件偏差 [m] |
| `elmaskar` | float | `15.0` | AR 仰角掩模 [deg] |
| `var_holdamb` | float | `0.1` | 保持模糊度方差 [m²] |
| `minfix` | int | `20` | 触发 hold 的最小固定样本数 |
| `minfixsats` | int | `4` | 固定时最小卫星对数 |
| `minholdsats` | int | `5` | hold 时最小卫星对数 |
| `mindropsats` | int | `10` | 丢弃卫星的最小卫星对数 |

### 2.7 单点定位参数

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `sing_p0` | float | `30.0` | m | SPP 初始位置 sigma |
| `sing_v0` | float | `10.0` | m/s | SPP 初始速度 sigma |
| `sing_elmin` | float | `10.0` | deg | SPP 最小仰角 |

### 2.8 星座与信号配置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `gnss_t` | array | `["GPS", "GLO", "GAL"]` | 启用星座列表。手机当前正式配置为 `["GPS", "GAL"]`，BDS 因 TC 双差残差系统性偏差暂时停用 |
| `freq_ix0` | dict | `{GPS: 0, GLO: 4, GAL: 0, BDS: 0}` | 第一频率索引（L1/E1/B1I） |
| `freq_ix1` | dict | `{GPS: 1, GLO: 5, GAL: 2, BDS: 3}` | 第二频率索引（L2/E5a/B2b） |
| `freq_table` | array | `[1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9]` | 频率表 [Hz] |
| `dfreq_glo` | array[2] | `[0.56250e6, 0.43750e6]` | GLONASS 频率间隔 [L1, L2] [Hz] |

> **注意**：BDS GEO 轨道、BDT→GPST 时标和 BDS 独立钟差处理已在代码中修复；手机配置仍暂不启用 BDS，恢复前需完成逐卫星残差验证。

### 2.9 基站与初始位置

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `rb_format` | str | `"xyz"` | 基站坐标格式：`xyz`=ECEF 直角坐标 [m]；`llh`=经纬度高 [lat_deg, lon_deg, h_m] |
| `rb` | array[3] | `[-2408697.451, 4698107.975, 3566695.2035]` | 基站位置（`xyz`: ECEF [m]；`llh`: [deg, deg, m]；全 0: 用 RINEX 头） |
| `rr_f` | array[6] | `[0,0,0,0,0,0]` | 流动站初始位置速度（正向）[m, m/s]（全 0: 自动 SPP） |
| `rr_b` | array[6] | `[0,0,0,0,0,0]` | 流动站初始位置速度（反向）[m, m/s]（全 0: 自动 SPP） |

**`rb_format` 说明**：
- `xyz`：`rb` 直接为 ECEF 直角坐标 [x, y, z]（单位：米）
- `llh`：`rb` 为 [纬度, 经度, 高度]（纬度/经度单位：度，高度单位：米）。配置加载时自动转换为 ECEF xyz，内部统一使用 xyz。

```yaml
# xyz 格式示例
rb_format: "xyz"
rb: [-2408697.451, 4698107.975, 3566695.2035]

# llh 格式示例
rb_format: "llh"
rb: [34.052235, -118.243683, 100.0]
```

---

## 3. INS 配置 (ins / ins_tc / ins_lc)

### 3.0 INS 参数的三段结构

INS 参数按**是否需要区分松紧组合**拆成三段。加载器（`src/utility/config_loader.py`
的 `_merge_mode_specific_ins()`）在读取 `ins.enabled` 之后、任何消费方读值之前，
把对应段**合并进 `cfg["ins"]`**，因此业务代码只看到合并后的 `ins` 段。

| 段名 | 含义 | 生效条件 |
|---|---|---|
| `ins` | **共用参数**（usually）：松紧组合语义完全相同 | 始终生效 |
| `ins_tc` | **紧组合专属**参数，或需要覆盖共用值的 TC 版本 | `ins.enabled == "tc"` |
| `ins_lc` | **松组合专属**参数，或需要覆盖共用值的 LC 版本 | `ins.enabled == "lc"` |

- `ins.enabled == "off"`（纯 GNSS）时 `ins_tc` / `ins_lc` **都不生效**。
- 段内不允许写 `enabled`；模式只能由 `ins.enabled` 决定。
- 未激活的段允许保留（同一文件可同时描述两种模式，便于对照），但**不会**被合并。

**划分依据**：是否存在**独立的 GNSS PVT（位置/速度）观测行**。
松组合把 GNSS 解算出的位置/速度当作观测，因此需要（也必须）配置观测噪声、位置差分
速度，以及为稳定 `P` 而额外注入的 `pos_psd` / `vel_psd`；紧组合只有双差伪距/相位
观测，**位置与速度的不确定度必须完全由 `Q`（IMU 噪声）与 `H/R` 传播得到**，额外注入
`pos_psd` / `vel_psd` 会破坏与 GREAT 的等价性
（见 `issue/9-14机械编排发散.md` 的 P5）。

#### 3.0.1 仅松组合：`ins_lc` 专属键

这些键**只能**出现在 `ins_lc` 段；出现在 `ins` 段且 `ins.enabled == "tc"` 时，
加载器直接报错。

| 键 | 用途 | 为什么 TC 不能用 |
|---|---|---|
| `pos_psd` | 额外位置随机游走注入 `P_pos` | TC 的位置不确定度来自速度耦合与观测，注入等价于人为放大 `P` |
| `vel_psd` | 额外速度随机游走直接注入 `P_vel` | 同上；历史上把 `vel_psd` 当"压跳变"手段曾使窗口 RMS 膨胀到 572 m |
| `gnss_vel_std` | GNSS 速度观测 σ | TC 无独立速度观测行 |
| `gnss_sd_scale` / `gnss_sd_axis_scale` | GNSS 位置 σ 缩放 / 分轴缩放 | TC 无独立位置观测行 |
| `gnss_time_sync_noise_s` | GNSS 时间同步噪声 | 仅 LC 观测模型使用 |
| `use_reported_gnss_sd` | 是否采用 GNSS 上报 σ | 同上 |
| `vertical_sigma_factor` | 高程 σ 放大因子 | 同上 |
| `pos_diff_vel_std` | 位置差分速度 σ 下限 | 同上 |
| `position_diff_velocity_update` | 是否用位置差分速度做更新 | 同上 |
| `innov_reject_threshold` / `innov_reject_warmup` | 位置创新拒绝 | 同上（TC 使用后验残差 + 卫星级 DD 重建） |

> **`nhc_warmup` 不是松组合专属**：它同时被 `lc_integration.py:108` 与
> `tc_integration.py:143` 消费，属于**共用参数**，应写在共用 `ins` 段。
> 写在 `ins_lc` / `ins_tc` 段也合法，但只在该模式生效——曾因此让
> `data/spp-ins-tc.yaml` 的 `nhc_warmup: 10` 在 TC 下静默失效（退回默认 30）。

#### 3.0.2 废弃死键（出现即报错）

以下键在仓库内**没有任何消费点**（`DEPRECATED_INS_KEYS`），写进任何 ins 段都会
在加载时报错。它们历史上属于 LC 语义，如需恢复对应功能必须同时修改代码与清单：

| 键 | 原意 | 状态 |
|---|---|---|
| `rtk_float_pos_std` | RTK FLOAT 位置 σ 下限 | 死键，2026-09-15 从全部配置移除 |
| `pos_diff_vel_max_std` | 直接 GNSS 速度 σ 超阈时改用位置差分速度 | 死键，同上 |

#### 3.0.3 仅紧组合：`ins_tc` 专属键

| 键 | 用途 | 为什么 LC 不能用 |
|---|---|---|
| `tc_use_doppler` | 是否把多普勒作为速度观测加入 DD 系统 | LC 直接使用 GNSS 解算速度，没有 DD 观测行 |

#### 3.0.4 写法示例

```yaml
ins:                       # 共用 (usually)
  enabled: "tc"
  imu_data_path: "...
  gyro_psd: 7.11620139001727e-09
  ...

ins_tc:                    # 仅 ins.enabled=tc 生效
  tc_use_doppler: false

ins_lc:                    # 仅 ins.enabled=lc 生效 (TC 模式下休眠)
  pos_psd: 0.0
  vel_psd: 0.0
```

### 3.1 主开关

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled` | str | `"lc"` | 主开关：`lc`=松组合 / `off`=纯 GNSS / `tc`=紧组合 |

> **重要**：不再使用 `coupling_mode` 字段。`ins.enabled` 是唯一的模式控制参数。
> `enabled` 只能写在 `ins` 段；写在 `ins_tc` / `ins_lc` 段会报错。

### 3.2 数据路径与采样率

**适用模式**：松紧组合共用 —— 写入 `ins` 段。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `imu_data_path` | str | `"data/cpt_euroc.csv"` | IMU 数据文件路径 |
| `data_rate` | int | `100` | IMU 采样率 [Hz]（`external` 模式仅支持 100） |
| `imu_format` | str | `"euroc"` | IMU 数据格式：`gpst`=GPS 周+周内秒；`euroc`=Unix 纳秒时间戳 |
| `imu_coordinate_system` | str | `"RFU"` | IMU 原始坐标系：`FRD`=前右下（默认）；`RFU`=右前上（读取时自动转换） |
| `imu_time_offset_s` | float | `0.0` | s | 在 IMU 喂入 TC 边界平移时间戳。仅在数据集确认存在固定 IMU-GNSS 时标偏移时设置；不能直接套用其它数据集的偏移值 |

**IMU 数据格式说明**：
- `gpst` 格式：CSV 文件，列为 `week, sow, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z`
- `euroc` 格式：CSV 文件，列为 `timestamp_ns, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z`（Unix 纳秒时间戳）

**坐标系说明**：
- 项目内部统一使用 FRD（前-右-下）坐标系
- 若 IMU 原始数据为 RFU（右-前-上），设置 `imu_coordinate_system: "RFU"`，读取时自动转换（FRD = [RFU_y, RFU_x, -RFU_z]）

### 3.3 使能开关（NHC/ZUPT/ZARU）

**适用模式**：松紧组合共用 —— 写入 `ins` 段（NHC/ZUPT/ZARU 在 LC 与 TC 两条路径都会读取）。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `nhc_enable` | int | `0` | NHC（非完整性约束）开关：0=关 / 1=开 |
| `zupt_enable` | int | `0` | ZUPT（零速更新）开关：0=关 / 1=3D 零速更新 |
| `zaru_enable` | int | `0` | ZARU（零角速率更新）开关：0=关 / 1=零角速率更新 |

**约束互斥逻辑**：
- 静态（GNSS 速度 < `static_speed_threshold`）：ZUPT + ZARU（互斥于 NHC）
- 运动：NHC（互斥于 ZUPT/ZARU）

### 3.4 初始对准

**适用模式**：松紧组合共用 —— 写入 `ins` 段。动对准使用的 GNSS 信息在两种模式下都是同一套 GNSS 结果/状态。

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `static_speed_threshold` | float | `0.5` | m/s | 静态检测阈值（GNSS 速度 < 此值判定为静态） |
| `dynamic_speed_threshold` | float | `3.0` | m/s | 动态速度阈值（平面速度 > 此值才可动对准）；手机配置使用 `4.0` |
| `angular_velocity_threshold_deg` | float | `30.0` | deg/s | 动态角速度阈值（陀螺范数 < 此值才可动对准） |
| `alignnment_dynamic_method` | str | `"position_diff"` | — | 动对准方法：`auto`/`velocity_vector`/`position_diff` |
| `gnss_buffer_size` | int | `5` | — | 位置差分 GNSS 历元缓冲区大小；手机配置为 `3`，CPT 配置为 `5` |
| `high_precision_ins_mode` | bool | `false` | — | 高精度 INS 初始化模式：`false`=低精度（当前实现）；`true`=高精度（预留） |
| `static_duration` | float | `10.0` | s | 静态初始化 GNSS 位置平均窗口（少于 10s 用全部，多于 10s 取最新 10s） |

**初始化模式说明**：
- `auto`：RTK/RTD 模式默认使用 `position_diff`（位置差分），SPP 模式默认使用 `velocity_vector`（多普勒测速）
- `velocity_vector`：使用 GNSS 速度方向计算 yaw（yaw = atan2(v_E, v_N)），需 GNSS 提供速度值
- `position_diff`：缓存 5 个 GNSS 历元，用首尾位置差分计算速度矢量，再计算 yaw

**动态速度阈值（按模式）**：
- SPP 模式：平面速度 > 3 m/s
- RTK/RTD 模式：平面速度 > 2 m/s

### 3.5 IMU 误差标定

**适用模式**：松紧组合共用 —— 写入 `ins` 段。

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `initial_gyro_bias` | array[3] | `[0, 0, 0]` | deg/h | 初始陀螺零偏 [x, y, z]（×dh2rs → rad/s） |
| `initial_acce_bias` | array[3] | `[0, 0, 0]` | mGal | 初始加计零偏 [x, y, z]（×1e-6×g0 → m/s²，g0=9.7803267715） |

> 当前默认置 0，依赖 EKF 在线估计零偏。

### 3.6 杆臂与安装角

**适用模式**：松紧组合共用 —— 写入 `ins` 段。

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `leverarm` | array[3] | `[0.0, 0.0, 0.0]` | m | GNSS 天线杆臂（b 系 FRD） |
| `initial_imu_angle` | array[2] | `[0, 0]` | deg | 初始 IMU 安装角 [pitch, yaw]（roll=0） |
| `initial_imu_leverarm` | array[3] | `[0.0, 0.0, 0.0]` | m | 初始 IMU 杆臂（b→v） |

### 3.7 初始不确定度

**适用模式**：松紧组合共用 —— 写入 `ins` 段。注意 `acce_bias_std_si` 的单位陷阱（见 [3.14.5](#3145-初始标准差对照易错点)）。

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `initial_pos_std_si` | array[3] | `[30.0, 30.0, 30.0]` | m | 位置初始标准差 |
| `initial_vel_std_si` | array[3] | `[10.0, 10.0, 10.0]` | m/s | 速度初始标准差 |
| `initial_att_std_si` | array[3] | `[0.00524, 0.00524, 0.00524]` | rad | 姿态初始标准差 |
| `gyro_bias_std_si` | array[3] | `[2.424e-5, ...]` | rad/s | 陀螺零偏初始标准差 |
| `acce_bias_std_si` | array[3] | `[0.0489, 0.0489, 0.0489]` | m/s² | 加计零偏初始标准差 |
| `lever_arm_std` | array[3] | `[0.1, 0.1, 0.1]` | m | GNSS 杆臂初始标准差 |
| `imu_angle_std` | array[2] | `[10.0, 10.0]` | deg | IMU 安装角初始标准差 [pitch, yaw] |
| `imu_leverarm_std` | array[3] | `[1.0, 1.0, 1.0]` | m | IMU 杆臂初始标准差 |
| `time_sync_std` | float | `0.01` | s | 时间对齐初始标准差 |

### 3.8 过程噪声 PSD

离散 `Q` 由 `Q = G·diag(PSD·dt)·Gᵀ` 构造（`src/core/ins/transfer_matrix.py`），
其中姿态块与速度块经 `C_b_e` 旋转到 E 系。因此**配置里的 PSD 是连续时间功率谱密度**，
其单位取决于它驱动的状态块。

| 参数 | 类型 | 默认值 | 单位 | 驱动的状态块 | 模式 |
|------|------|--------|------|--------------|------|
| `gyro_psd` | float | `3.388e-09` | rad²/s = (rad/s)²·s | 姿态误差 `δψᵉ`（陀螺白噪声 / ARW 的平方） | 共用 |
| `accel_psd` | float | `2.604e-06` | m²/s³ = (m/s²)²·s | 速度 `δvᵉ`（加速度计白噪声 / VRW 的平方） | 共用 |
| `gyro_bias_psd` | float | `2.612e-14` | rad²/s³ = (rad/s²)²·s | 陀螺零偏 `δb_g` 随机游走 | 共用 |
| `acce_bias_psd` | float | `1.661e-09` | m²/s⁵ = (m/s³)²·s | 加计零偏 `δb_a` 随机游走 | 共用 |
| `gyro_bias_corr_time_s` | float | `36.0`（=0.01 h） | s | 零偏一阶 Gauss-Markov 相关时间；**`.inf` = 纯随机游走**（无 `-I/τ` 阻尼） | 共用 |
| `acce_bias_corr_time_s` | float | `36.0`（=0.01 h） | s | 同上（加计） | 共用 |
| `pos_psd` | float | LC `5.0`；TC **不允许** | m²/s | 额外位置随机游走（**松组合专属**，见 3.0.1） | **LC** |
| `vel_psd` | float | LC `0.5`；TC **不允许** | m²/s² | 额外速度随机游走（**松组合专属**，见 3.0.1） | **LC** |
| `vel_var_floor` | float | `0.0` | m²/s² | 速度协方差下限；非零时防止 `P_vel` 过小导致机动段卡死 | 共用 |
| `pos_diff_vel_std` | float | `0.15` | m/s | 位置差分速度 σ 下限（**松组合专属**） | **LC** |
| `innov_reject_threshold` | float | `0.0` | m | 位置创新拒绝阈值（0=禁用，**松组合专属**） | **LC** |
| `innov_reject_warmup` | int | `100` | — | 创新拒绝预热历元数（**松组合专属**） | **LC** |

> `rtk_float_pos_std` / `pos_diff_vel_max_std` 是无消费点的死键，已废弃，
> 出现在任何 ins 段都会报错（见 3.0.2）。

> ⚠️ **`pos_psd` / `vel_psd` 不能用于紧组合。**
> 这两个键被登记为松组合专属（`config_loader.INS_LC_ONLY_KEYS`）。把它们写在 `ins`
> 段且 `ins.enabled == "tc"` 时，加载器会直接抛
> `ValueError: ins.enabled='tc': 以下参数只适用于松组合 ...`。
> 原因：它们把额外的位置/速度随机游走直接注入 `P`，会人为放大协方差、掩盖真实的
> 状态可估计性问题。历史教训：把 `vel_psd` 当作"压跳变"手段曾使窗口 RMS 膨胀到
> `572 / 376 / 684 m`（详见 `issue/9-14机械编排发散.md` 的 P5）。
> 若 TC 出现跳变，**先查零偏可估计性**（初值方差、过程模型），而不是放大 `Q`。

> **调参提示**：LC 中 `pos_psd=0` 时 `P_pos` 可能在量测更新后趋近 0，`K→0`，滤波器
> 锁死无法跟踪 GNSS；但 TC **不能**照搬该经验值。RTK-TC MECH-16 使用 `pos_psd=0`、
> `vel_psd=0` 和 `vel_var_floor=3e-4`。LC 与手机 RTD-TC 的过程噪声必须按各自数据集验证。

> **零偏过程模型**：GREAT-MSF 的 `t_gsins::_tauG/_tauA` 恒为零向量
> （`gins.cpp:32,46`），`set_Ft()` 直接把它写成 `eb-eb` / `db-db` 对角块
> （`gins.cpp:416-418`），即 **GREAT 的零偏是纯随机游走**。
> 与 GREAT 对照时应显式声明 `gyro_bias_corr_time_s: .inf` 与
> `acce_bias_corr_time_s: .inf`；缺省的 `36 s` 会引入 `-I/τ` 阻尼，使 `P_bg/P_ba`
> 收敛到 `Q·τ/2` 的稳态后不再增长，等价于强迫零偏"回到零"，滤波器会拒绝继续修正它。

### 3.9 NHC 配置

**适用模式**：松紧组合共用 —— 写入 `ins` 段（NHC 观测行在 LC 与 TC 两条路径都会构造）。

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `nhc_std` | float | `0.05` | m/s | NHC 速度观测标准差 |
| `nhc_max_vel` | float | `0.5` | m/s | NHC 单维速度 guard（侧向速度超此值跳过该维） |
| `nhc_max_gyro` | float | `30.0` | deg/s | NHC 角速率 guard（剧烈转弯跳过整个 NHC） |
| `nhc_decimation` | int | `5` | epoch | NHC 抽样间隔（5=100Hz 下 20Hz） |
| `nhc_warmup` | int | LC `1` / TC `30` | epoch | NHC 预热历元数（**共用参数**，两种模式的缺省值不同；见 3.0.1 末尾说明） |

### 3.10 ZUPT 配置

**适用模式**：松紧组合共用 —— 写入 `ins` 段。

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `zupt_std` | float | `0.05` | m/s | ZUPT 速度观测标准差 |
| `zupt_max_vel` | float | `0.1` | m/s | ZUPT 速度 guard |
| `zupt_max_gyro` | float | `10.0` | deg/s | ZUPT 角速率 guard |
| `zupt_min_count` | int | `15` | epoch | ZUPT 最小间隔 |

### 3.11 ZARU 配置

**适用模式**：松紧组合共用 —— 写入 `ins` 段。

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `zaru_std` | float | `0.01745` | rad/s | ZARU 角速率观测标准差（1°/s） |
| `zaru_max_vel` | float | `0.1` | m/s | ZARU 速度 guard |
| `zaru_max_gyro` | float | `5.0` | deg/s | ZARU 角速率 guard |
| `zaru_min_count` | int | `100` | epoch | ZARU 最小间隔 |

### 3.12 静态检测配置

**适用模式**：松紧组合共用 —— 写入 `ins` 段。

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `static_detect_method` | str | `"GLRT"` | 检测方法：`GLRT`/`MV`/`MAG`/`ARE`/`ALL` |
| `static_window_size` | int | `20` | 滑动窗口大小 [epoch]（100Hz 下 0.2s） |
| `static_sig_gyro` | float | `0.1` | 陀螺噪声 σ [rad/s] |
| `static_sig_accl` | float | `0.1` | 加计噪声 σ [m/s²] |
| `static_gamma_glrt` | float | `100.0` | GLRT 阈值 |
| `static_gamma_mv` | float | `50.0` | MV 阈值 |
| `static_gamma_mag` | float | `50.0` | MAG 阈值 |
| `static_gamma_are` | float | `50.0` | ARE 阈值 |

### 3.13 可选状态参数开关

**适用模式**：松紧组合共用 —— 写入 `ins` 段。

状态向量固定 15 维：`[δr^e(3), δv^e(3), δψ^e(3), δb_g(3), δb_a(3)]`。以下开关启用可选参数块：

| 参数 | 类型 | 默认值 | 维数 | 说明 |
|------|------|--------|------|------|
| `estimate_leverarm` | int | `1` | 3 | 1=估计 GNSS 天线杆臂 |
| `estimate_mounting_angle` | int | `0` | 2 | 1=估计 IMU 安装角 [pitch, yaw] |
| `estimate_imu_leverarm` | int | `0` | 3 | 1=估计 IMU 杆臂 b→v（需 `estimate_mounting_angle=1`） |
| `estimate_time_sync` | int | `0` | 1 | 1=估计时间对齐误差 |

> 启用杆臂在线估计后，`.rslt` 文件每行追加 6 列（lever_x, lever_y, lever_z, sdlx, sdly, sdlz）。杆臂参数 1Hz 更新频率，Qins=3 时更新。

### 3.14 单位与 PSD 换算总表

#### 3.14.1 基本单位约定

项目内部**一律使用 SI**：位置 `m`、速度 `m/s`、加速度 `m/s²`、角度 `rad`、
角速度 `rad/s`、时间 `s`。配置中以 `_si` 结尾的键表示"已经是 SI，不再做单位换算"。

| 量 | 常用工程单位 | → SI |
|---|---|---|
| 角度 | `deg` | × `π/180` = × `1.745329251994e-2` |
| 角速度 | `deg/s` | × `π/180` |
| 角速度（慢漂） | `deg/h` | × `π/180/3600` = × `4.848136811095e-6` |
| 加速度 | `mg`（毫重力） | × `g0×1e-3` = × `9.7803267714e-3` |
| 加速度 | `mGal` | × `1e-5`（`1 Gal = 1e-2 m/s²`） |
| 重力常数 | `g0` | `9.7803267714 m/s²`（GREAT `t_gglv::g0`） |

#### 3.14.2 随机游走系数的两种写法

IMU 噪声既可用 **PSD**（功率谱密度，`Q` 直接消费）也可用 **随机游走系数**
（ARW/VRW，器件手册常见）表示。二者关系：

```
PSD = (随机游走系数)²
```

随机游走系数的单位是「量纲 / √时间」，平方后即为「量纲² / 时间」，正是 PSD 的单位。

| 块 | 配置项 | PSD 单位 | 对应随机游走系数 | 手册常见写法 |
|---|---|---|---|---|
| 姿态 | `gyro_psd` | `rad²/s` | `rad/√s` | ARW：`deg/√h` |
| 速度 | `accel_psd` | `m²/s³` | `m/s²/√Hz` = `m/s^{3/2}` | VRW：`m/s/√h` 或 `mg/√Hz` |
| 陀螺零偏 | `gyro_bias_psd` | `rad²/s³` | `rad/s/√s` | 零偏游走：`deg/h/√h` |
| 加计零偏 | `acce_bias_psd` | `m²/s⁵` | `m/s²/√s` | 零偏游走：`mg/√h` |
| 位置 | `pos_psd` | `m²/s` | `m/√s` | `m/√h` |

#### 3.14.3 PSD 换算系数（工程单位 → SI）

记 `deg = π/180`、`hur = 3600`、`g0 = 9.7803267714`。

| 从 | 到 PSD (SI) | 换算式 |
|---|---|---|
| ARW `A` [deg/√h] | `gyro_psd` [rad²/s] | `(A · deg / √hur)²` = `(A · 2.908882e-4)²` |
| VRW `V` [m/s/√h] | `accel_psd` [m²/s³] | `(V / √hur)²` = `(V / 60)²` |
| VRW `V` [mg/√Hz] | `accel_psd` [m²/s³] | `(V · g0 · 1e-3)²` |
| 零偏游走 `B` [deg/h/√h] | `gyro_bias_psd` [rad²/s³] | `(B · deg / hur / √hur)²` = `(B · 8.080230e-8)²` |
| 零偏游走 `D` [mg/√h] | `acce_bias_psd` [m²/s⁵] | `(D · g0 · 1e-3 / √hur)²` = `(D · 1.630054e-4)²` |
| 位置游走 `P` [m/√h] | `pos_psd` [m²/s] | `(P / √hur)²` = `(P / 60)²` |

#### 3.14.4 GREAT-MSF ADIS16470 对照表（campus01 正式配置使用的值）

GREAT 的模型表（`gset/gsetins.cpp:642-650`）以「数值 ÷ 单位常量」保存，回填 `Qt`
时（`gins/gins.cpp:578`）再乘以单位常量后取平方，因此 **`Qt` 就是括号里字面量的平方**。

| GREAT 模型表写法 | 物理值 | `Qt`（SI PSD） | gipylib 配置项 |
|---|---|---|---|
| `AttProcNoisePSD = 17.4 * mpsh` | `0.29 deg/√h` | `7.1162e-9` | `gyro_psd: 7.11620139001727e-09` |
| `VelProcNoisePSD = 0.687225e-3 / mgpsHz` | `0.0703 mg/√Hz` | `4.7228e-7` | `accel_psd: 4.72278200625e-07` |
| `PosProcNoisePSD = 1e-4 / mpsh` | `6.0e-3 m/√h` | `1.0e-8` | （位置 PSD 属松组合专属项，TC 不注入） |
| `GyroBiasProcNoisePSD = 8.56014618e-2 / mpsh` | `5.136 deg/h/√h` | `1.7223e-13` | `gyro_bias_psd: 1.72231306427737e-13` |
| `AcceBiasProcNoisePSD = 3.1640625e-4 / mgpsh` | `1.941 mg/√h` | `1.0011e-7` | `acce_bias_psd: 1.0010896e-07` |

#### 3.14.5 初始标准差对照（易错点）

| 量 | GREAT 模型表 | 进入 `Pk` 的值 | gipylib 配置项 | SI 值 |
|---|---|---|---|---|
| 位置 | `3 m` | `3` | `initial_pos_std_si` | `[3.0, 3.0, 3.0]` |
| 速度 | `0.5 m/s` | `0.5` | `initial_vel_std_si` | `[0.5, 0.5, 0.5]` |
| 姿态 | `0.5, 0.5, 5 deg` | 同上转 rad | `initial_att_std_si` | `[8.7266e-3, 8.7266e-3, 8.7266e-2]` |
| 陀螺零偏 | `120 deg/h` | `120·dph` | `gyro_bias_std_si` | `[5.81776417331e-4, ...]` |
| **加计零偏** | **`5e-2 / mg`** | **`0.05`（m/s²）= 5.11 mg** | `acce_bias_std_si` | **`[0.05, 0.05, 0.05]`** |

> ⚠️ **加计零偏是最容易写错的一项**。`gsetins.cpp:646` 写的是
> `Vector3d(5e-2,...) / t_gglv::mg`，而 `gins.cpp:552` 回填时又 `db_tmp * t_gglv::mg`，
> 除完再乘回，**进入 `Pk` 的就是字面量 `0.05`（m/s²）**，即 `5.11 mg`。
> 曾误按 “0.05 mg” 填成 `4.89016338575e-4`，比 GREAT 小 **102 倍**，直接导致零偏被
> 滤波器锁死、机械编排每秒漂移 `0.5 m`（详见 `issue/9-14机械编排发散.md` 的 P1）。
> 另注意：GREAT XML `<Estimator>` 里的 `InitialSTD` / `ProcNoiseSD` 在
> `IMUErrorModel Type != "Customize"` 时**不生效**（`gsetign.cpp:311...409`），
> 真正生效的是 `IMUErrorModels()` 模型表。

---

## 4. 紧组合配置 (tc)

仅 `ins.enabled=tc` 时生效。

```yaml
tc:
  degrade:
    fail_threshold: 3        # 连续量测失败次数触发降级
    reboot_threshold: 30.0   # 降级后持续失败时长触发 reboot [s]
```

**降级策略**：rtk → rtd → spp → imu_only。连续量测失败超 `fail_threshold` 次触发降级；降级后持续失败超 `reboot_threshold` 秒触发 reboot（重新初始化）。

**紧组合模式说明**：
- SPP-INS：伪距 + 多普勒观测
- RTK-INS：双差载波 + 伪距观测
- RTD-INS：双差伪距观测
- RTD/RTK 量测使用 rover/base 共视卫星形成双差；单颗 GPS 观测不能单独形成双差
- 量测构造后的有效残差数 `n_meas < 4` 时跳过 EKF 更新；卫星原始数量不等于有效量测数量
- 量测失败时尝试 SPP 位置 fallback，但 SPP 仍要求至少 4 颗有效 GPS 卫星
- 启动阶段按 `positioning_mode` 使用 GNSS 结果初始化；初始化前输出 `Qins=0`

### RTK-TC 当前验证基线

`data/rtk-ins紧组合.yaml` 的 MECH-16/FINAL4 证据为：目标窗口 GPS Week 2046、SOW `359000--359100` 水平 RMSE `0.0394 m`，全段 `0.2595 m`，传播速度 RMSE 约 `0.099 m/s`，量测间速度增长约 `8.0 cm/s`。这是正式非 AR 基线；`armode=3` 的约 `0.019 m` 结果属于独立 AR 分支，不能与基线收益混报。

TC 模糊度管理已完成 LAMBDA 异常降级、SD→DD 变换、正定保护、过程噪声和健康门控修复。若参考 ignav 的 R/Q 或观测策略使目标窗口、最差机动窗口或全段恶化，必须回退并保留对照证据。

### 手机 RTD-TC 当前状态

`phone/rtdtc.yaml` 当前采用 `positioning_mode: rtd`、`ins.enabled: tc`、GPS+Galileo（BDS 暂停）。手机 MEMS IMU 噪声显著高于 CPT 传感器，因此使用较大的 `gyro_psd`、`accel_psd` 与零偏随机游走；这些数值来自手机数据实测调谐，不能直接替换为 CPT 配置中的小噪声。

> ⚠️ 该配置历史上还使用过 `vel_psd`。`vel_psd` 现已归为**松组合专属**
> （[3.0.1](#301-仅松组合ins_lc-专属键)），`ins.enabled=tc` 时写在 `ins` 段会直接报错。
> 若确需保留该运行点，必须把 `ins.enabled` 改为 `lc`，或改为调整 `accel_psd` /
> 零偏过程模型等**共用**参数，并重新出具对照证据。

手机 RTD-TC 结果可用以下脚本评估：

```bash
python phone/eval_enu.py phone/output/RTDTC.rslt phone/mate40ref.kf
python phone/error_rslt.py phone/output/RTDTC.rslt
python phone/tra-mech.py
```

图像和运行日志写入 `phone/plot/`。`error_rslt.py` 的 ENU 误差图按 `Qins=2`（红色，机械编排）和 `Qins=3`（绿色，量测更新）标记；`tra-mech.py` 默认绘制 GPS week 2382、SOW 115665--115755（08:07:45--08:09:15）的平面轨迹。

---

## 5. 输出配置 (output)

```yaml
output:
  output_dir: "data/output"
  gnss_filename: "RTK.pos"
  aligned_filename: "aligned.csv"
  rslt_filename: "RTKLC.rslt"
  position_format: "llh"
  time_format: "gpst"
  trace_enabled: true
  trace_level: 0
  stat_level: 0
  stat_rate: "update"
  stat_filename: ""
```

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `output_dir` | str | `"data/output"` | 输出目录 |
| `gnss_filename` | str | `"RTK.pos"` | 纯 GNSS 解算结果文件名（`.pos`） |
| `aligned_filename` | str | `"aligned.csv"` | 对齐块状 CSV 文件名（`ins.enabled=lc` 输出） |
| `rslt_filename` | str | `"RTKLC.rslt"` | 组合导航结果文件名（`.rslt`，`ins.enabled=lc/tc` 输出，100Hz） |
| `position_format` | str | `"llh"` | 位置输出格式：`llh`=经纬度高 / `xyz`=ECEF 直角坐标 |
| `time_format` | str | `"gpst"` | 时间输出格式：`gpst`=GPS 周+周内秒 / `datetime`=YYYY/MM/DD HH:MM:SS.sss |
| `trace_level` | int | `0` | trace 文件等级：0=off / 1=info / 2=detail / 3=debug |
| `trace_enabled` | bool | `true` | trace 总开关；为 `false` 时不生成 trace |
| `stat_level` | int | `0` | 状态诊断等级：0=off；1=基础状态；2=增加 TC 更新摘要；3=逐卫星/逐模糊度/矩阵明细（S4 当前预留） |
| `stat_rate` | str | `"update"` | 状态输出节奏：`update`=GNSS/约束更新点；`second`=整数秒；`imu`=每个 IMU 点 |
| `stat_filename` | str | `""` | 状态诊断文件名；留空时由输出模块按模式自动命名 |

**输出文件按模式启用**：
- `ins.enabled=off` → `.pos`（纯 GNSS，1Hz）
- `ins.enabled=lc` → `.pos` + `.csv` + `.rslt`（纯 GNSS 副输出 + 对齐 CSV + 松组合 100Hz）
- `ins.enabled=tc` → `.rslt`（紧组合 100Hz）

**位置格式说明**：
- `llh`：输出经纬度（度）+ 高度（米），格式 `lat lon h`
- `xyz`：输出 ECEF 直角坐标（米），格式 `x y z`

**时间格式说明**：
- `gpst`：GPS 周 + 周内秒，格式 `week sow`（如 `2055 357453.000`）
- `datetime`：年月日时分秒，格式 `YYYY/MM/DD HH:MM:SS.sss`（如 `2019/03/27 00:17:33.000`）

**trace 文件**：
- `trace_level > 0` 时生成 `.trace` 文件，与主输出文件同名（如 `RTKLC.trace`）
- trace 文件中的时间戳自动转换为 GPS 周+周内秒格式
- 自动过滤无效 DEBUG 输出（如 `pos=[0. 0. 0.]`、`x[clk]=N/A`、`clk_stored=[0. 0. 0.]`）
- `trace_level=3`（debug）可查看滤波器内部状态，用于调试

**`.rslt` 文件格式**（100Hz，ignav outins 风格）：

| 列号 | 字段 | 说明 |
|------|------|------|
| 0-1 | week, sow | GPS 周 + 周内秒 |
| 2-4 | lat/lon/h 或 x/y/z | 位置（按 `position_format`） |
| 5 | Q | GNSS 质量标志（1=FIX, 2=FLOAT, 4=RTD/DGPS, 5=SPP） |
| 6 | Qins | INS 状态标志（0=未初始化, 2=机械编排, 3=量测更新） |
| 7 | ns | 卫星数 |
| 8-13 | sdn, sde, sdu, sdne, sdeu, sdun | 位置协方差 |
| 14-15 | age, ratio | 差分龄期 / AR ratio |
| 16-18 | vx, vy, vz | ECEF 速度 |
| 19-24 | sdvx, sdvy, sdvz, sdvxy, sdvyz, sdvzx | 速度协方差 |
| 25-27 | roll, pitch, yaw | 姿态角（度） |
| 28-30 | sdroll, sdpitch, sdyaw | 姿态协方差 |
| 31-36 | lever_x, lever_y, lever_z, sdlx, sdly, sdlz | 杆臂参数（仅 `estimate_leverarm=1`） |

当 `output.stat_level >= 2` 时，状态文件还会在 TC 更新点写入 `GIPY_EPOCH`、`GIPY_COV`、`GIPY_INNOV`、`GIPY_GAIN` 等诊断摘要；更新点由 `update_flag` 标识，`Qins` 仅是结果行上的状态属性。

---

## 6. 配置校验规则

配置加载时（`src/utility/config_loader.py`）执行以下校验：

1. **`coupling_mode` 已废弃**：若配置文件含 `coupling_mode` 字段将报错，请删除并使用 `ins.enabled`
2. **`ins.enabled` 必填**：取值必须为 `lc`/`off`/`tc`
3. **INS 三段合并与跨模式校验**（见 [3.0](#30-ins-参数的三段结构)）：
   - 按 `ins.enabled` 把 `ins_tc`（仅 `tc`）或 `ins_lc`（仅 `lc`）合并进 `ins`；`off` 时都不合并
   - `ins_tc` / `ins_lc` 必须是 mapping，且**不得包含 `enabled`**
   - `ins_tc` 中不得出现松组合专属键；`ins_lc` 中不得出现紧组合专属键
   - **共用 `ins` 段不得出现当前模式禁止的键**：例如 `ins.enabled=tc` 时写 `pos_psd` / `vel_psd` 直接报错；`ins.enabled=lc` 时写 `tc_use_doppler` 直接报错
   - **废弃死键检查**：`rtk_float_pos_std` / `pos_diff_vel_max_std` 出现在任何 ins 段都报错（见 3.0.2）
4. **`gnss_source` 校验**：必须为 `internal` 或 `external`
5. **纯 GNSS 模式约束**：`ins.enabled=off` 时 `gnss_source` 必须为 `internal`
6. **基站坐标格式校验**：`rb_format` 必须为 `xyz` 或 `llh`；`llh` 时自动转 ECEF
7. **输出格式校验**：`position_format` ∈ {llh, xyz}；`time_format` ∈ {gpst, datetime}；`trace_level` ∈ {0, 1, 2, 3}；`stat_level` ∈ {0, 1, 2, 3}；`stat_rate` ∈ {update, second, imu}
8. **零偏相关时间**：`gyro_bias_corr_time_s` / `acce_bias_corr_time_s` 必须是正数或 `+inf`；`0`、负数、`nan` 报错
9. **`external` 模式约束**：`ins.enabled` 必须为 `lc`，`data_rate` 必须为 100
10. **`internal` 模式约束**：`positioning_mode` 必填（spp/rtd/rtk）；`rover_path`、`eph_path` 必填；`rtk`/`rtd` 模式 `base_path` 必填；`ins.enabled=lc/tc` 时 `imu_data_path` 必填

---

## 7. 典型配置示例

### 7.1 纯 GNSS RTK 解算

```yaml
gnss:
  gnss_source: "internal"
  positioning_mode: "rtk"
  rover_path: "data/cpt0870.19o"
  base_path: "data/cpt0870_base.19o"
  eph_path: "data/brdm0870.19p"
  rb_format: "xyz"
  rb: [-2408697.451, 4698107.975, 3566695.2035]

ins:
  enabled: "off"

output:
  output_dir: "data/output"
  gnss_filename: "RTK.pos"
  position_format: "llh"
  time_format: "gpst"
  trace_level: 0
```

### 7.2 松组合导航（LC）

```yaml
gnss:
  gnss_source: "internal"
  positioning_mode: "rtk"
  rover_path: "data/cpt0870.19o"
  base_path: "data/cpt0870_base.19o"
  eph_path: "data/brdm0870.19p"
  rb_format: "xyz"
  rb: [-2408697.451, 4698107.975, 3566695.2035]

ins:                 # 共用 (usually)
  enabled: "lc"
  imu_data_path: "data/cpt_euroc.csv"
  data_rate: 100
  imu_format: "euroc"
  imu_coordinate_system: "RFU"
  estimate_leverarm: 1
  gyro_psd: 3.388e-09
  accel_psd: 2.604e-06
  gyro_bias_psd: 2.612e-14
  acce_bias_psd: 1.661e-09

ins_lc:              # 松组合专属: 独立 GNSS PVT 观测与 P 稳定项
  pos_psd: 5.0
  vel_psd: 0.5
  pos_diff_vel_std: 0.15
  innov_reject_threshold: 0.0
  innov_reject_warmup: 100

output:
  output_dir: "data/output"
  gnss_filename: "RTK.pos"
  rslt_filename: "RTKLC.rslt"
  position_format: "llh"
  time_format: "gpst"
  trace_level: 0
```

### 7.3 紧组合导航（TC）

```yaml
gnss:
  gnss_source: "internal"
  positioning_mode: "rtk"
  rover_path: "data/cpt0870.19o"
  base_path: "data/cpt0870_base.19o"
  eph_path: "data/brdm0870.19p"
  rb_format: "xyz"
  rb: [-2408697.451, 4698107.975, 3566695.2035]

ins:                 # 共用 (usually)
  enabled: "tc"
  imu_data_path: "data/cpt_euroc.csv"
  data_rate: 100
  imu_format: "euroc"
  imu_coordinate_system: "RFU"
  gyro_psd: 3.388e-09
  accel_psd: 2.604e-06
  gyro_bias_psd: 2.612e-14
  acce_bias_psd: 1.661e-09
  # 注意: pos_psd / vel_psd 不能出现在 TC 的 ins 段 (松组合专属)

ins_tc:              # 紧组合专属
  tc_use_doppler: false

tc:
  degrade:
    fail_threshold: 3
    reboot_threshold: 30.0

output:
  output_dir: "data/output"
  rslt_filename: "RTKTC.rslt"
  position_format: "llh"
  time_format: "gpst"
  trace_level: 3
```

#### 7.3.1 与 GREAT-MSF campus01 对照的正式紧组合配置

`data-great/rtktc-rate.yaml`（原始 100 Hz 速率直入）与
`data-great/rtktc-increment.yaml`（同一原始流转增量）是本仓库当前的正式对照配置，
两者输出必须逐行一致。关键取值：

```yaml
ins:
  enabled: "tc"
  initial_pos_std_si: [3.0, 3.0, 3.0]                 # m
  initial_vel_std_si: [0.5, 0.5, 0.5]                 # m/s
  initial_att_std_si: [8.7266e-3, 8.7266e-3, 8.7266e-2]  # rad = 0.5/0.5/5 deg
  gyro_bias_std_si: [5.81776417331e-4, ...]           # rad/s = 120 deg/h
  acce_bias_std_si: [0.05, 0.05, 0.05]                # m/s² = 5.11 mg  ← 不是 0.05 mg
  gyro_psd: 7.11620139001727e-09                      # rad²/s
  accel_psd: 4.72278200625e-07                        # m²/s³
  gyro_bias_psd: 1.72231306427737e-13                 # rad²/s³
  acce_bias_psd: 1.0010896e-07                        # m²/s⁵
  gyro_bias_corr_time_s: .inf                         # GREAT: 零偏随机游走
  acce_bias_corr_time_s: .inf

ins_tc:
  tc_use_doppler: false
```

窗口 `181000--181100`、10 Hz 真值最近邻（无插值/重采样）的当前结果为
gipylib `E/N/U = 0.4500 / 0.2155 / 0.2267 m`，GREAT `0.8627 / 0.7334 / 0.4594 m`。

### 7.4 基站坐标使用 llh 格式

```yaml
gnss:
  rb_format: "llh"
  rb: [34.052235, -118.243683, 100.0]   # [lat_deg, lon_deg, h_m]
```

加载时自动转换为 ECEF xyz，内部统一使用 xyz。

### 7.5 手机 RTD-TC 配置

手机配置可直接参考 `phone/rtdtc.yaml`：

```yaml
gnss:
  gnss_source: "internal"
  positioning_mode: "rtd"
  nf: 1
  gnss_t: ["GPS", "GAL"]       # BDS 当前暂停，待手机 C2I 残差专项验证
  maxinno: 5.0                  # 相位门限（RTD 当前不使用载波相位）
  maxcode: 60.0                 # 手机初始状态误差较大时使用

ins:
  enabled: "tc"
  imu_data_path: "phone/phone_imu_euroc.csv"
  imu_format: "euroc"
  imu_coordinate_system: "RFU"
  data_rate: 100
  imu_time_offset_s: 0.0
```

手机 IMU 的过程噪声应按实测调谐。当前正式配置使用 `gyro_psd=1e-5`、`accel_psd=1e-2`、`gyro_bias_psd=1e-10`、`acce_bias_psd=1e-4`；不要直接复制 CPT 级小噪声。

> ⚠️ 2026-09-15 schema 迁移说明：`phone/rtdtc.yaml` 历史上把 `pos_psd=0.0`、
> `vel_psd=1.0`、`innov_reject_*` 写在共用 `ins` 段并在 TC 中生效。按新 schema
> 这些是**松组合专属**键（[3.0.1](#301-仅松组合ins_lc-专属键)），已移入休眠的
> `ins_lc` 段 —— TC 运行时**不再注入** `vel_psd=1.0`，`P_vel` 演化完全由
> `Q` 与 `H/R` 传播决定。迁移后的手机 RTD-TC 结果需要重新出具对照证据；
> 若水平 RMSE 相对 `7.72 m` 基线恶化，应先查零偏可估计性（初始方差、随机游走），
> 而不是恢复 `vel_psd`。

手机 RTD-TC 的已验证结果约为水平 RMSE `7.72 m`，较改前 `14.45 m` 改善，但仍明显差于纯 RTD 约 `3.58 m`。当前暂停 BDS，恢复前必须完成逐卫星残差、频率映射和 RTKLIB 对照；不能把 `maxcode=60` 推广为通用默认值。

### 7.6 输出 ECEF 坐标 + 日期时间格式

```yaml
output:
  position_format: "xyz"
  time_format: "datetime"
  trace_level: 2
```

输出文件中位置列为 ECEF [x, y, z]（米），时间列为 `YYYY/MM/DD HH:MM:SS.sss` 格式，并生成 `.trace` 文件（detail 级别）。
