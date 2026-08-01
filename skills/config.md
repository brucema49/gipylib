# 配置参数详细说明

> 本文档详细解释 `data/config.yaml` 中所有配置参数的含义、单位、默认值、取值范围与参考来源。
> 配置文件采用**行内注释**风格（注释在参数右侧），本文档提供完整说明。
> 与 [初始化.md](file:///home/mxl/workplace/gipylib/skills/初始化.md) 保持一致，初始化相关参数交叉引用初始化.md 对应章节。
>
> **配置文件布局**：
> - `data/config.yaml`：完整配置模板（含所有参数）
> - `data/spp-ins-lc.yaml`：SPP + 松组合参考配置（internal + ins.lc）
> - `data/rtdtc.yaml` / `phone/rtdtc.yaml`：RTD + 紧组合参考配置（internal + ins.tc）
> - `data/ignav-rtktc.conf`：ignav 紧组合参考配置
>
> **时间系统约定**：全框架内部统一使用 Unix 时间戳（`gtime_t.time + gtime_t.sec`），转换工具 `src/core/time_utils.py`（`gpst_to_unix` / `unix_to_gpst`，`GPST_EPOCH_UNIX = 315964800`）。
>
> **坐标系约定**：
> - 机械编排与初始化主计算在 E 系（ECEF）
> - 体坐标系 b 系：FRD（前-右-下）
> - 车体坐标系 v 系：FRD，与 b 系通过安装角旋转矩阵 `R_b^v` 关联
> - 导航系 n 系：ENU（仅输出与中间量转换用）
> - 姿态角顺序：航向(yaw) → 俯仰(pitch) → 横滚(roll)，ZYX 旋转

---

## 目录

- [1. GNSS 配置 (gnss)](#1-gnss-配置-gnss)
  - [1.1 数据源与文件路径](#11-数据源与文件路径)
  - [1.2 定位模式](#12-定位模式)
  - [1.3 观测值筛选](#13-观测值筛选)
  - [1.4 周跳与粗差检测](#14-周跳与粗差检测)
  - [1.5 卡尔曼滤波统计](#15-卡尔曼滤波统计)
  - [1.6 模糊度解算](#16-模糊度解算)
  - [1.7 单点定位](#17-单点定位)
  - [1.8 星座与信号](#18-星座与信号)
  - [1.9 基站与初始位置](#19-基站与初始位置)
- [2. INS 配置 (ins)](#2-ins-配置-ins)
  - [2.1 主开关与 Reboot](#21-主开关与-reboot)
  - [2.2 处理时间](#22-处理时间)
  - [2.3 使能开关](#23-使能开关)
  - [2.4 数据路径与采样率](#24-数据路径与采样率)
  - [2.5 初始对准](#25-初始对准)
  - [2.6 初始状态](#26-初始状态)
  - [2.7 IMU 噪声参数](#27-imu-噪声参数)
  - [2.8 NHC 配置](#28-nhc-配置)
  - [2.9 ZUPT 配置](#29-zupt-配置)
  - [2.9b ZARU 配置](#29b-zaru-配置)
  - [2.9c 静态检测配置](#29c-静态检测配置)
  - [2.10 杆臂与安装角](#210-杆臂与安装角)
  - [2.10b 可选状态参数开关](#210b-可选状态参数开关单滤波-stateindex-管理)
  - [2.11 初始协方差](#211-初始协方差)
  - [2.12 GNSS 中断模拟](#212-gnss-中断模拟)
- [3. 输出配置 (output)](#3-输出配置-output)

---

## 1. GNSS 配置 (gnss)

GNSS 配置部分参考 rtklib-py 的 `config_phone.py` / `config_f9p.py`，已吸收到 `src/core/gnss/rtklib/`。

### 1.1 数据源与文件路径

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `gnss_source` | str | `"external"` | — | `internal` / `external` | GNSS 数据源模式。`internal`=内部 SPP/RTK 解算；`external`=读取外部结果文件 |
| `positioning_mode` | str | `"rtk"` | — | `spp` / `rtk` | 定位模式（仅 `internal` 模式生效）。`spp`=单点定位；`rtk`=相对定位 |
| `rover_path` | str | `"data/cpt0870.19o"` | — | 有效文件路径 | 流动站观测值文件路径（`internal` 模式必填） |
| `base_path` | str | `"data/cpt0870_base.19o"` | — | 有效文件路径 | 基站观测值文件路径（`internal` + `rtk` 模式必填） |
| `eph_path` | str | `"data/brdm0870.19p"` | — | 有效文件路径 | 星历文件路径（`internal` 模式必填） |
| `external_sol_path` | str | `"data/spp.pos"` | — | 有效文件路径 | 外部 GNSS 结果文件路径（`external` 模式必填） |
| `external_sol_format` | str | `"pos"` | — | `pos` / `nmea` / `csv` | 外部结果文件格式。当前 `external` 模式仅支持 `pos` |
| `default_pos_std` | double | `1.0` | m | >0 | 外部结果默认位置标准差（当 `pos_covariance_available=false` 时使用） |
| `default_vel_std` | double | `0.5` | m/s | >0 | 外部结果默认速度标准差 |
| `pos_covariance_available` | bool | `false` | — | true/false | 外部结果是否包含协方差信息 |

**校验规则**（`src/utility/config_loader.py`）：
- `gnss_source` 必须为 `internal` 或 `external`
- `ins.enabled=off` 时 `gnss_source` 必须为 `internal`（纯 GNSS 模式不支持外部结果输入）
- `external` 模式下 `ins.enabled` 必须为 `on`，且 `data_rate` 必须为 100
- `internal` 模式下 `positioning_mode` 必填，`rover_path` 和 `eph_path` 必填
- `internal` + `rtk`/`rtd` 模式下 `base_path` 必填
- `internal` + `ins.enabled=on/tc` 模式下 `imu_data_path` 必填

### 1.2 定位模式

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `nf` | int | `2` | — | 1 / 2 | 频率数。1=单频（L1）；2=双频（L1+L5/L2） |
| `pmode` | str | `"kinematic"` | — | `static` / `kinematic` | rtklib-py 定位模式。`static`=静态；`kinematic`=动态 |
| `filtertype` | str | `"forward"` | — | `forward` / `backward` / `combined` / `combined_noreset` | 滤波类型。`forward`=前向；`backward`=后向；`combined`=组合；`combined_noreset`=组合不重置 |
| `use_sing_pos` | bool | `false` | — | true/false | 每历元是否重新单精解初始位置 |

### 1.3 观测值筛选

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `elmin` | double | `15.0` | deg | [0, 90] | 最小仰角，低于此仰角的卫星被剔除 |
| `cnr_min` | array[2] | `[28, 20]` | dB-Hz | >0 | 最小信噪比阈值 [freq1, freq2]，低于此值的观测值被剔除 |
| `excsats` | array | `[]` | — | 卫星 ID 列表 | 排除卫星列表，如 `["G01", "G23"]` |

### 1.4 周跳与粗差检测

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `maxinno` | double | `1.0` | m | >0 | 载波相位周跳/粗差检测阈值（创新值），超限观测值被剔除 |
| `maxcode` | double | `10.0` | m | >0 | 伪距粗差检测阈值，超限观测值被剔除 |
| `maxage` | double | `30.0` | s | >0 | 最大差分龄期，超限后差分修正失效 |
| `maxout` | int | `4` | epoch | >0 | 最大差分中断历元数，超限后差分重置 |
| `thresdop` | double | `5.0` | — | >0 | 多普勒法周跳检测阈值 |
| `thresslip` | double | `0.10` | — | >0 | 几何无关组合（LG）周跳检测阈值 |
| `interp_base` | bool | `false` | — | true/false | 是否插值基站观测值到流动站历元 |

### 1.5 卡尔曼滤波统计

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `eratio` | array[2] | `[300, 100]` | — | >0 | 伪距/载波噪声比 [L1, L5/L2]，用于构造观测协方差 |
| `efact_gps` | double | `1.0` | — | >0 | GPS 星座相对权重 |
| `efact_glo` | double | `1.5` | — | >0 | GLONASS 星座相对权重 |
| `efact_gal` | double | `1.0` | — | >0 | Galileo 星座相对权重 |
| `err_base` | double | `0.003` | m | >0 | 基线误差 sigma |
| `err_el` | double | `0.003` | m | >0 | 仰角相关误差 sigma |
| `err_satclk` | double | `5.0e-12` | s | >0 | 卫星钟误差 sigma |
| `snrmax` | double | `45.0` | dB-Hz | >0 | 方差计算最大信噪比，超过此值按此值计算 |
| `accelh` | double | `3.0` | m/s² | >0 | 水平加速度噪声 sigma（系统噪声） |
| `accelv` | double | `1.0` | m/s² | >0 | 垂直加速度噪声 sigma（系统噪声） |
| `prnbias` | double | `0.01` | cycles | >0 | 载波相位偏差 sigma |
| `sig_p0` | double | `30.0` | m | >0 | 初始位置 sigma |
| `sig_v0` | double | `10.0` | m/s | >0 | 初始速度/加速度 sigma |
| `sig_n0` | double | `30.0` | m | >0 | 初始模糊度 sigma |

### 1.6 模糊度解算

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `armode` | int | `0` | — | 0 / 1 / 3 | 模糊度解算模式。0=off；1=continuous；3=fix-and-hold |
| `thresar` | double | `3.0` | — | >0 | AR ratio 检验阈值，比值大于此值才固定 |
| `thresar1` | double | `0.05` | m | >0 | AR 位置变化阈值（兼用于卡尔曼加速度更新） |
| `minlock` | int | `0` | epoch | ≥0 | 参与模糊度解算的最小连续锁定历元数 |
| `glo_hwbias` | double | `0.0` | m | — | GLONASS 硬件偏差 |
| `elmaskar` | double | `15.0` | deg | [0, 90] | AR 仰角掩模，低于此仰角的卫星不参与模糊度解算 |
| `var_holdamb` | double | `0.1` | m² | >0 | 保持模糊度方差 |
| `minfix` | int | `20` | — | >0 | 触发 hold 的最小固定样本数 |
| `minfixsats` | int | `4` | — | >0 | 固定时最小卫星对数 |
| `minholdsats` | int | `5` | — | >0 | hold 时最小卫星对数 |
| `mindropsats` | int | `10` | — | >0 | 丢弃卫星的最小卫星对数 |

### 1.7 单点定位

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `sing_p0` | double | `100.0` | m | >0 | SPP 初始位置 sigma |
| `sing_v0` | double | `10.0` | m/s | >0 | SPP 初始速度 sigma |
| `sing_elmin` | double | `10.0` | deg | [0, 90] | SPP 最小仰角 |

### 1.8 星座与信号

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `gnss_t` | array | `["GPS", "GLO", "GAL"]` | — | 星座列表 | 启用星座列表 |
| `freq_ix0` | dict | `{GPS: 0, GLO: 4, GAL: 0}` | — | 频率索引 | 第一频率索引（L1） |
| `freq_ix1` | dict | `{GPS: 2, GLO: 5, GAL: 2}` | — | 频率索引 | 第二频率索引（L5/E5b） |
| `freq_table` | array[6] | `[1.57542e9, ...]` | Hz | >0 | 支持频率表，索引对应 `freq_ix0`/`freq_ix1` 中的值 |
| `dfreq_glo` | array[2] | `[0.56250e6, 0.43750e6]` | Hz | >0 | GLONASS 频率间隔 [L1, L2]，用于 FDMA 频率计算 |

### 1.9 基站与初始位置

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `rb_format` | str | `"xyz"` | — | `xyz` / `llh` | 基站坐标格式。`xyz`=ECEF 直角坐标 [x,y,z] (m)；`llh`=经纬度高 [lat_deg, lon_deg, h_m]，由 `config_loader._normalize_rb()` 转为 xyz |
| `rb` | array[3] | `[0, 0, 0]` | m | — | 基站位置（`rb_format=xyz`: ECEF [x,y,z]；`rb_format=llh`: [lat,lon,h]）。全 0 表示使用 RINEX 头中的基站位置 |
| `rr_f` | array[6] | `[0, 0, 0, 0, 0, 0]` | m, m/s | — | 流动站初始位置速度（正向）[x, y, z, vx, vy, vz]。全 0 表示自动单精解 |
| `rr_b` | array[6] | `[0, 0, 0, 0, 0, 0]` | m, m/s | — | 流动站初始位置速度（反向）[x, y, z, vx, vy, vz]。全 0 表示自动单精解 |

---

## 2. INS 配置 (ins)

INS 配置部分参考 ignav 的 `configure.ini` 和 `tools/KF-GINS/config/kf-gins.yaml`。

### 2.1 主开关与 Reboot

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `enabled` | str | `"lc"` | — | `off` / `lc` / `tc` | 主开关（必填）。`off`=纯 GNSS 解算；`lc`=松组合；`tc`=紧组合。已废弃的 `coupling_mode` 字段已移除，改用 `ins.enabled` 控制组合模式 |
| `reboot` | double | `50` | s | >0 | GNSS 中断重启阈值，等价于 `gnss_outage_threshold`（[初始化.md 13.6 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#136-reboot-识别与重新初始化)）。INS 初始化完成后，若 GNSS 中断超过此阈值，重新进行组合导航初始化 |
| `imu_outage_threshold` | double | `1.0` | s | >0 | IMU 中断 reboot 阈值。IMU 数据流中断超过此值时触发 reboot |
| `timestamp_jump_threshold` | double | `10.0` | s | >0 | 时间戳跳变 reboot 阈值。IMU/GNSS 时间戳跳变超过此值时触发 reboot |
| `cov_divergence_threshold` | double | `100.0` | m² | >0 | 协方差发散 reboot 阈值。协方差 P 对角元素最大值超过此值时触发 reboot |
| `enable_gnss_mode_degrade_reboot` | bool | `false` | — | true/false | 是否启用 GNSS 模式降级 reboot（如 RTK→SPP） |
| `gnss_mode_degrade_threshold` | double | `60.0` | s | >0 | GNSS 模式降级持续时长阈值 |
| `reboot_use_prior_state` | bool | `false` | — | true/false | reboot 后重新初始化时是否使用前一次状态作为先验 |

**校验规则**：
- `enabled` 必填，必须为 `off` / `on` / `tc`（`coupling_mode` 字段已废弃，存在则报错）
- `enabled=off` 时 `gnss_source` 必须为 `internal`（纯 GNSS 模式不支持外部结果输入）
- `external` 模式下 `enabled` 必须为 `on`
- `internal` + `enabled=on/tc` 模式下 `imu_data_path` 必填

**Reboot 机制**（详见 [初始化.md 第 13.6 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#136-reboot-识别与重新初始化)）：
- `Aligner` 负责检测数据流层面的 reboot 信号（GNSS 中断、IMU 中断、时间戳跳变）
- `InsInitializer` / `LcIntegration` 负责检测状态层面的 reboot 信号（协方差发散、模式降级）
- 两者通过 `RebootEvent` 事件统一调度

### 2.2 处理时间

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `process_date` | array[3] | `[2024, 12, 20]` | — | 有效日期 | 处理日期 [year, month, day] |
| `start_time` | double | `0` | s | ≥0 | 起始时间（当日秒或 GPST 周内秒） |
| `end_time` | double | `-1` | s | -1 或 >start_time | 结束时间。-1 表示处理至文件结束 |

### 2.3 使能开关

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `gnss_enable` | int | `1` | — | 0 / 1 | GNSS 量测使能。0=关闭 GNSS 量测更新 |
| `imu_enable` | int | `1` | — | 0 / 1 | IMU 机械编排放能。0=关闭机械编排 |
| `nhc_enable` | int | `2` | — | 0-9 | NHC 策略（见下方策略表）。静态时与 ZUPT/ZARU 互斥（静态→ZUPT+ZARU，运动→NHC） |
| `zupt_enable` | int | `0` | — | 0 / 1 | ZUPT 策略。0=关闭；1=3D 零速更新。静态时与 NHC 互斥 |
| `zaru_enable` | int | `0` | — | 0 / 1 | ZARU 策略。0=关闭；1=零角速率更新（估计陀螺零偏）。静态时与 NHC 互斥 |

**NHC 策略表**：

| 值 | 维度 | 坐标系 | 说明 |
|----|------|--------|------|
| 0 | — | — | 不启用 NHC |
| 1 | 1D | v 系 | 侧向速度约束 |
| 2 | 2D | v 系 | 侧向 + 垂向速度约束（默认） |
| 3 | 3D | v 系 | 前向 + 侧向 + 垂向（高速时放宽前向 R） |
| 4 | 3D | n 系 | 姿态约束（roll/pitch/yaw, LSTM/GT） |
| 5 | 6D | v+n | 姿态 3D + 速度 3D |
| 6 | 2D | v 系 | 侧向 + 垂向（Ga-St VBAKF 自适应） |
| 7 | 1D | v 系 | 侧向（Ga-St VBAKF 自适应） |
| 8 | 3D | n 系(ENU) | 东 + 北 + 天速度 |
| 9 | 2D | v 系 | 前向 + 侧向（速度大时放宽 R） |

> **约束互斥逻辑**（参考 ignav，详见 [NHC_ZUPT.md](file:///home/mxl/workplace/gipylib/skills/NHC_ZUPT.md)）：
> - 静态检测（GLRT/MV/MAG/ARE/ALL）判定为静止 → ZUPT + ZARU（若 enable=1）
> - 静态检测判定为运动 → NHC（若 enable≠0）
> - 三者通过 per-IMU 触发 + decimation 互斥应用

### 2.4 数据路径与采样率

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `imu_data_path` | str | `"data/cpt_imu.csv"` | — | 有效文件路径 | IMU 数据文件路径（`ins.enabled=on` 时必填） |
| `gnss_data_path` | str | `"data/spp.pos"` | — | 有效文件路径 | GNSS 数据文件路径（`external` 模式结果文件） |
| `result_output_path` | str | `"output/"` | — | 有效目录 | 结果输出目录 |
| `data_rate` | double | `100` | Hz | >0 | IMU 采样率。`external` 模式仅支持 100 Hz |
| `result_output_rate` | double | `1` | Hz | >0 | 结果输出率 |
| `imudatalen` | int | `7` | — | ≥7 | IMU 文件列数（只用前 7 列） |
| `imu_format` | str | `"gpst"` | — | `gpst` / `euroc` | IMU 数据文件格式。`gpst`=GPS 周+周内秒（`week,sow,gx,gy,gz,ax,ay,az`），由 `ImuFormator` 解码；`euroc`=Unix 纳秒时间戳（`timestamp_ns,wx,wy,wz,ax,ay,az`），由 `EuRoCImuFormator` 解码。`ImuSensor._create_formator()` 根据此参数工厂创建对应解码器（详见 [imu.md](file:///home/mxl/workplace/gipylib/skills/imu.md) / [StreamDesign.md](file:///home/mxl/workplace/gipylib/skills/StreamDesign.md)） |

### 2.5 初始对准

> 详见 [初始化.md 第 5 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#5-初始化模式分类) 和 [第 5.4 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#54-动态初始化默认模式选择基于-gnss-解算模式)。

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `alignnment_posvelatt_mode` | int | `0` | — | 0 / 1 | 1=配置完整 PVA（最高优先级，[初始化.md 第 6 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#6-配置设定初始化setting)）；0=否 |
| `alignnment_attitude_mode` | int | `1` | — | 0 / 1 | 1=配置姿态（位置/速度来自 GNSS）；0=自动对准 |
| `motion_threshold` | double | `0.5` | m/s | >0 | **已废弃**（保留兼容），由 `static_speed_threshold` / `dynamic_speed_threshold` 替代（[初始化.md 5.4.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#543-三组独立阈值static_speed--dynamic_speed--angular_velocity)） |
| `static_speed_threshold` | double | `0.5` | m/s | >0 | **静态检测速度阈值**：GNSS 速度范数 < 此值判定为静态，进入静对准（[初始化.md 5.4.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#543-三组独立阈值static_speed--dynamic_speed--angular_velocity)） |
| `dynamic_speed_threshold` | double | `4.0` | m/s | >0 | **动态速度阈值**：GNSS 速度范数 > 此值才可进入动对准（适用于速度矢量法与位置差分法） |
| `angular_velocity_threshold_deg` | double | `30.0` | deg/s | >0 | **动态角速度阈值**：陀螺角速度范数 < 此值才可进入动对准（≈0.5236 rad/s），避免转弯时初始化 |
| `imu_coordinate_system` | str | `"FRD"` | — | `FRD` / `RFU` | IMU 原始坐标系：`FRD`=前右下（默认） / `RFU`=右前上（`ImuSensor` 读取时自动转换为 FRD） |
| `alignnment_dynamic_method` | str | `"auto"` | — | `auto` / `velocity_vector` / `position_diff` | 动对准方法选择（[初始化.md 5.4.1](file:///home/mxl/workplace/gipylib/skills/初始化.md#541-默认规则)）。`auto`=按 GNSS 模式自动选择；`velocity_vector`=强制速度矢量；`position_diff`=强制位置差分 |
| `gnss_velocity_fallback` | str | `"position_diff"` | — | `doppler` / `position_diff` / `auto` | GNSS 无速度时回退策略（[初始化.md 5.4.2](file:///home/mxl/workplace/gipylib/skills/初始化.md#542-gnss-不提供速度时的统一回退策略)）。**注意**：动态模式下 GNSS 不提供速度时统一使用位置差分法 |
| `gnss_buffer_size` | int | `3` | 个 | ≥2 | 位置差分初始化的 GNSS 历元缓冲区大小（[初始化.md 9.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#93-处理流程伪代码)）。运动阈值达到时确保缓冲区存满 N 个历元，但**只用最新两个历元**计算差分速度。第 3 个历元提供历史冗余（fallback） |
| `high_precision_ins_mode` | bool | `false` | — | true / false | 高精度 INS 初始化模式（[初始化调整.md](file:///home/mxl/workplace/gipylib/skills/初始化调整.md)）。`false`=低精度模式（本项目当前实现：静态位置 GNSS 历史平均 + 姿态置 0 + 速度置 0）；`true`=高精度模式（预留，AcceLeveling + 解析寻北，后续实现） |
| `static_duration` | double | `10.0` | s | >0 | 静态初始化 GNSS 位置平均窗口（[初始化.md 7.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#73-静态位置平均)）。GNSS 历史少于 `static_duration` 秒时用全部历元求平均；多于时取最新 `static_duration` 秒内的历元求平均 |

**`alignnment_dynamic_method="auto"` 时的默认规则**：

| GNSS 解算模式 | 默认动对准方法 | 速度来源 |
|--------------|--------------|---------|
| SPP | 速度矢量初始化（[初始化.md 第 8 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#8-动态初始化---速度矢量初始化)） | 多普勒测速 `sol.vel` |
| RTK Fixed / RTK Float | 位置差分初始化（[初始化.md 第 9 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#9-动态初始化---位移矢量初始化)） | 相邻历元位置差分 |
| RTD | 位置差分初始化 | 相邻历元位置差分 |
| External | 按 `gnss_velocity_fallback` 配置 | 配置决定 |

**三阈值选择建议**（[初始化.md 5.4.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#543-三组独立阈值static_speed--dynamic_speed--angular_velocity)）：

| 场景 | `static_speed_threshold` | `dynamic_speed_threshold` | `angular_velocity_threshold_deg` | 原因 |
|------|--------------------------|---------------------------|----------------------------------|------|
| 默认（车辆） | 0.5 m/s | 4.0 m/s | 30.0 deg/s | 区分静止与低速行驶；角速度约束避免转弯时初始化 |
| 行人/低速车辆 | 0.2 m/s | 1.0 m/s | 20.0 deg/s | 避免低速被误判为静止 |
| 高速车辆 | 1.0 m/s | 5.0 m/s | 30.0 deg/s | 避免停车等红灯时误判为运动 |
| 严格静态启动 | 0.1 m/s | — | — | 仅在确实静止时进入静对准 |

### 2.6 初始状态

> 详见 [初始化.md 第 11 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#11-初始状态装配)。

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `initial_pos` | array[3] | `[0, 0, 0]` | deg, deg, m | — | 初始位置 [lat, lon, alt] |
| `initial_vel` | array[3] | `[0.0, 0.0, 0.0]` | m/s | — | 初始速度 [N, E, D] |
| `initial_att` | array[3] | `[0.0, 0.0, 0.0]` | deg | — | 初始姿态 [roll, pitch, yaw]（ZYX 旋转） |
| `initial_gyro_bias` | array[3] | `[0, 0, 0]` | deg/h | — | 初始陀螺零偏 [x, y, z] |
| `initial_acce_bias` | array[3] | `[0, 0, 0]` | mGal | — | 初始加计零偏 [x, y, z] |

**单位转换**（参考 ignav `StartAligning` 与 `constant.hpp`，[初始化.md 11.2 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#112-单位转换参考-kf-gins-loadconfig)）：
- 纬度/经度：度 → 弧度（`rad = deg * D2R`）
- 姿态角：度 → 弧度
- 陀螺零偏：deg/h → rad/s（`rad/s = deg/h * dh2rs`，`dh2rs = π / 180.0 / 3600.0`）
- 加计零偏：mGal → m/s²（`m/s² = mGal * 1e-6 * g0`，`g0 = 9.7803267715`，即 `constant_mGal ≈ 9.78e-6`；**注意：非标准 `1e-5`**）

### 2.7 IMU 噪声参数

> 包含传感器级噪声参数（VRW/ARW/bias_std/corr_time）和过程噪声 PSD 参数（用于 Q 矩阵构造，由 `TransferMatrix` 读取）。

#### 2.7.1 传感器级噪声参数

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `velocity_random_walk` | array[3] | `[0.1, 0.1, 0.1]` | m/s/√hr | >0 | 速度随机游走（VRW）[x, y, z] |
| `attitude_random_walk` | array[3] | `[0.1, 0.1, 0.1]` | deg/√hr | >0 | 姿态随机游走（ARW）[x, y, z] |
| `gyro_bias_std` | array[3] | `[25.0, 25.0, 25.0]` | deg/h | >0 | 陀螺零偏标准差 [x, y, z] |
| `acce_bias_std` | array[3] | `[200.0, 200.0, 200.0]` | mGal | >0 | 加计零偏标准差 [x, y, z] |
| `corr_time_of_gyro_bias` | double | `0.01` | h | >0 | 陀螺零偏相关时间（一阶高斯-马尔科夫过程） |
| `corr_time_of_acce_bias` | double | `0.01` | h | >0 | 加计零偏相关时间 |

#### 2.7.2 过程噪声 PSD 参数（Q 矩阵构造）

> 由 `src/core/ins/transfer_matrix.py::TransferMatrix` 读取，用于构造过程噪声矩阵 Q。
> **调谐级 PSD** 在传感器噪声基础上增大以计入车辆动力学，使 P_pos 在 1s 内增长到与 R 可比（K≈0.5）。

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `gyro_psd` | double | `1.0e-5` | rad²/s | >0 | 陀螺过程噪声 PSD（调谐值）。传感器级为 3.4e-9（CPT 战术级，仅含传感器噪声） |
| `accel_psd` | double | `1.0e-2` | m²s⁻³ | >0 | 加计过程噪声 PSD（调谐值）。传感器级为 2.6e-6 |
| `gyro_bias_psd` | double | `1.0e-10` | rad²s⁻³ | >0 | 陀螺零偏随机游走 PSD（调谐值） |
| `acce_bias_psd` | double | `1.0e-4` | m²s⁻⁵ | >0 | 加计零偏随机游走 PSD（调谐值） |
| `pos_psd` | double | `5.0e-3` | m²/s | ≥0 | **位置随机游走 PSD（LC EKF 必需项）**。使 P_pos 1s 内增长 ~0.005，配合 R=0.0025(sigma=0.05) 使 K≈0.8。无 pos_psd 时 P_pos 在量测更新后趋近 0，K→0，滤波器锁死无法跟踪 GNSS |

> **注意**：`pos_psd` 是 LC EKF 必需项。`TransferMatrix` 通过 `ins_cfg.get("pos_psd", 0.0)` 读取（标量），Q 矩阵使用 `pos_psd * dt`。旧版 `position_random_walk`（数组）已废弃，不再使用。

#### 2.7.3 初始不确定度（SI 单位，从 cpt-rtktc_gps.conf 提取）

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `initial_pos_std_si` | array[3] | `[30.0, 30.0, 30.0]` | m | >0 | 初始位置不确定度（1σ, per axis, ECEF） |
| `initial_vel_std_si` | array[3] | `[10.0, 10.0, 10.0]` | m/s | >0 | 初始速度不确定度（1σ, per axis, ECEF） |
| `initial_att_std_si` | array[3] | `[0.00524, 0.00524, 0.00524]` | rad | >0 | 初始姿态不确定度（1σ, per axis） |
| `gyro_bias_std_si` | array[3] | `[2.424e-5, ...]` | rad/s | >0 | 陀螺零偏不确定度（from ins-uncbg） |
| `acce_bias_std_si` | array[3] | `[0.0489, ...]` | m/s² | >0 | 加计零偏不确定度（from ins-uncba） |

### 2.8 NHC 配置

> NHC（Non-Holonomic Constraint，非完整性约束）假设车辆在 v 系（车体系）下侧向和垂向速度为 0。
> 参考 ignav `ins-nhc.cc`，包含 guards（速度/角速率阈值）和 decimation（抽样间隔）。

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `nhc_std` | double | `0.5` | m/s | >0 | NHC 速度观测标准差。std=0.5 + decimation=10 防止 100Hz 过约束高程退化 |
| `nhc_max_vel` | double | `0.5` | m/s | >0 | NHC 单维速度 guard（ignav MAXVEL，超阈剔除该维） |
| `nhc_max_gyro` | double | `30.0` | deg/s | >0 | NHC 角速率 guard（ignav，剧烈转弯跳过整个 NHC） |
| `nhc_decimation` | int | `10` | epoch | ≥1 | NHC 抽样间隔（1=每IMU历元，10=100Hz下10Hz） |
| `nhc_gt` | int | `0` | — | 0 / 1 | 1=使用真值参考速度/姿态（仿真用），与 `nhc_lstm` 互斥 |
| `nhc_lstm` | int | `0` | — | 0 / 1 | 1=使用 LSTM 预测速度/姿态（部署用），与 `nhc_gt` 互斥 |
| `nhc_start` | double | `0` | s | ≥0 | NHC 生效起始时刻（GPST 周内秒） |
| `nhc_end` | double | `604800` | s | >nhc_start | NHC 生效结束时刻（GPST 周内秒） |
| `nhc_R_lateral` | double | `0.01` | m²/s² | >0 | 侧向速度观测噪声 R 初值 |
| `nhc_R_vertical` | double | `0.01` | m²/s² | >0 | 垂向速度观测噪声 R 初值（策略 6） |
| `nhc_turn_alpha` | double | `0.9` | — | (0, 1) | 偏航角变化率低通滤波系数（策略 6/7） |
| `nhc_turn_beta` | double | `0.5` | — | (0, 1) | 转弯强度平滑系数 |
| `nhc_turn_threshold` | double | `0.1` | — | >0 | 转弯判断阈值 |
| `nhc_turn_vel_threshold` | double | `2.0` | m/s | >0 | 触发转弯检测的最低前向速度 |
| `nhc_turn_gamma` | double | `10.0` | — | >0 | 转弯强度与 R 放大的映射系数 |
| `nhc_turn_smooth` | int | `5` | epoch | >0 | 转弯强度平滑窗口 |
| `nhc_vb_iter` | int | `3` | — | >0 | VBAKF 迭代次数 |
| `nhc_vb_zeta` | double | `1.0` | — | >0 | VBAKF 遗忘因子 |
| `nhc_vb_nu` | double | `3.0` | — | >0 | 逆 Wishart 先验自由度 |

> **NHC 调谐经验**：std=0.5 + decimation=10（10Hz）防止 100Hz 过约束导致高程退化。详见 [NHC_ZUPT.md](file:///home/mxl/workplace/gipylib/skills/NHC_ZUPT.md)。

### 2.9 ZUPT 配置

> ZUPT（Zero-velocity Update，零速更新）在车辆静止时约束三维速度为 0。与 NHC 互斥。
> 参考 ignav `ins-zvu.cc`，包含 guards 和 ignav 常量映射。

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `zupt_std` | double | `0.05` | m/s | >0 | ZUPT 速度观测标准差（ignav VARVEL=SQR(0.05)） |
| `zupt_max_vel` | double | `0.1` | m/s | >0 | ZUPT 速度 guard（ignav MAXVEL，超阈跳过） |
| `zupt_max_gyro` | double | `10.0` | deg/s | >0 | ZUPT 角速率 guard（ignav MAXGYRO） |
| `zupt_min_count` | int | `15` | epoch | >0 | ZUPT 最小间隔（ignav MINZC=15） |
| `zupt_vel_threshold` | double | `0.5` | m/s | >0 | 零速检测速度阈值（兼容旧版，推荐用 static_detect） |
| `zupt_R` | array[3] | `[0.01, 0.01, 0.01]` | (m/s)² | >0 | ZUPT 三维速度观测噪声 R [x, y, z]（兼容旧版） |
| `zupt_min_static_epoch` | int | `5` | epoch | >0 | 判定静止的最小连续历元数（兼容旧版） |

### 2.9b ZARU 配置

> ZARU（Zero Angular Rate Update，零角速率更新）在车辆静止时约束三维角速度为 0，估计陀螺零偏。
> 参考 ignav `ins-zaru.cc`。静态时与 NHC 互斥。

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `zaru_std` | double | `0.01745` | rad/s | >0 | ZARU 角速率观测标准差（ignav VARARE=SQR(1°/s)） |
| `zaru_max_vel` | double | `0.1` | m/s | >0 | ZARU 速度 guard（ignav MAXVEL） |
| `zaru_max_gyro` | double | `5.0` | deg/s | >0 | ZARU 角速率 guard（ignav MAXGYRO，比 ZUPT 更严） |
| `zaru_min_count` | int | `100` | epoch | >0 | ZARU 最小间隔（ignav MINZAC=100） |

### 2.9c 静态检测配置

> 参考 ignav `ins-static-detect.cc`，用于 NHC/ZUPT/ZARU 互斥选择（静态→ZUPT/ZARU，运动→NHC）。

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `static_detect_method` | str | `"GLRT"` | — | GLRT/MV/MAG/ARE/ALL | 检测方法 |
| `static_window_size` | int | `20` | epoch | >0 | 滑动窗口大小（100Hz下0.2s） |
| `static_sig_gyro` | double | `0.1` | rad/s | >0 | 陀螺噪声 σ |
| `static_sig_accl` | double | `0.1` | m/s² | >0 | 加计噪声 σ |
| `static_gamma_glrt` | double | `100.0` | — | >0 | GLRT 阈值 |
| `static_gamma_mv` | double | `50.0` | — | >0 | MV 阈值 |
| `static_gamma_mag` | double | `50.0` | — | >0 | MAG 阈值 |
| `static_gamma_are` | double | `50.0` | — | >0 | ARE 阈值 |

### 2.10 杆臂与安装角

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `leverarm` | array[3] | `[0.0, 0.0, 0.0]` | m | — | GNSS 天线杆臂（b 系 FRD） |
| `antlever` | array[3] | `[0.0, 0.0, 0.0]` | m | — | 天线杆臂（IMU 系 FRD，参考 KF-GINS） |
| `initial_imu_angle` | array[2] | `[0, 0]` | deg | — | 初始 IMU 安装角 [pitch, yaw]（roll 假设为 0） |
| `initial_imu_leverarm` | array[3] | `[0.0, 0.0, 0.0]` | m | — | 初始 IMU 杆臂（b→v） |
| `imu_angle_std` | array[2] | `[10.0, 10.0]` | deg | >0 | IMU 安装角初始标准差 [pitch, yaw] |
| `imu_leverarm_std` | array[3] | `[1.0, 1.0, 1.0]` | m | >0 | IMU 杆臂初始标准差 [x, y, z] |

### 2.10b 可选状态参数开关（单滤波 StateIndex 管理）

> 参考 ignav `evaluate_*`，由 `StateIndex` dataclass 管理参数块索引（-1=未启用）。

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `estimate_leverarm` | int | `0` | — | 0 / 1 | 1=估计 GNSS 天线杆臂（3维，算法参考 ignav） |
| `estimate_mounting_angle` | int | `0` | — | 0 / 1 | 1=估计 IMU 安装角（2维，替代旧 `evaluate_imu_angle`） |
| `estimate_imu_leverarm` | int | `0` | — | 0 / 1 | 1=估计 IMU 杆臂 b→v（3维，需 `estimate_mounting_angle=1`） |
| `estimate_time_sync` | int | `1` | — | 0 / 1 | 1=估计时间对齐误差（1维，算法参考 ignav） |
| `lever_arm_psd` | double | `0.0` | m²/s | ≥0 | GNSS 杆臂随机游走 PSD（0=常数） |
| `imu_angle_psd` | double | `1.0e-6` | rad²/s | ≥0 | 安装角随机游走 PSD |
| `imu_leverarm_psd` | double | `1.0e-8` | m/s | ≥0 | IMU 杆臂随机游走 PSD |
| `time_sync_psd` | double | `1.0e-4` | s²/s | ≥0 | 时间对齐随机游走 PSD |
| `lever_arm_std` | array[3] | `[0.1, 0.1, 0.1]` | m | >0 | GNSS 杆臂初始标准差 |
| `time_sync_std` | double | `0.01` | s | >0 | 时间对齐初始标准差 |

### 2.11 初始协方差

> 详见 [初始化.md 第 12 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#12-初始协方差矩阵设置)。

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `use_define_variance_pos_vel` | int | `1` | — | 0 / 1 | 1=使用自定义位置/速度方差；0=使用 GNSS 解算协方差 |
| `use_define_variance_att` | int | `1` | — | 0 / 1 | 1=使用自定义姿态方差；0=按对准模式默认 |
| `initial_pos_std` | array[3] | `[0.5, 0.5, 0.5]` | m | >0 | 初始位置标准差 [N, E, D] |
| `initial_vel_std` | array[3] | `[0.5, 0.5, 0.5]` | m/s | >0 | 初始速度标准差 [N, E, D] |
| `initial_att_std` | array[3] | `[0.2, 0.2, 0.5]` | deg | >0 | 初始姿态标准差 [roll, pitch, yaw] |

**注**：P 中 IMU 安装角/杆臂参数块的初始协方差由 `imu_angle_std` 和 `imu_leverarm_std` 自动构造，无需单独配置。

**姿态协方差默认值**（`use_define_variance_att=0` 时，参考 [初始化.md 12.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#123-不同对准模式下的姿态协方差默认值)）：

| 对准模式 | roll std | pitch std | yaw std | 原因 |
|---------|----------|-----------|---------|------|
| SETTING | 配置值 | 配置值 | 配置值 | 使用配置值 |
| IN_MOTION | 5° | 5° | 5° | 动对准精度有限，三轴均较松 |
| STATIONARY_AND_MOTION | 2° | 2° | 5° | roll/pitch 由加速度计对准（精度较好），yaw 由 GNSS 速度方向确定（精度较差） |

### 2.12 GNSS 中断模拟

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `gnsslog_enable` | int | `0` | — | 0 / 1 | GNSS 日志使能 |
| `gnss_break` | array[4] | `[0, 30, 60, 0]` | s, s, s, 次 | — | 中断参数 [start, break_time, interval_time, break_number] |
| `gnss_break_array` | array | `[]` | s | — | 中断时间段数组（成对，与 `gnss_break_type=1` 配合使用） |
| `gnss_break_type` | int | `0` | — | 0 / 1 | 0=使用 `gnss_break`；1=使用 `gnss_break_array` |

---

## 3. 输出配置 (output)

| 参数 | 类型 | 默认值 | 单位 | 取值范围 | 说明 |
|------|------|--------|------|---------|------|
| `output_dir` | str | `"output"` | — | 有效目录 | 输出目录 |
| `gnss_filename` | str | `"RTK.pos"` | — | 有效文件名 | 纯 GNSS 解算结果文件名（`.pos`，`ins.enabled=off/on` 输出，由 `SolutionWriter` 输出） |
| `aligned_filename` | str | `"aligned.csv"` | — | 有效文件名 | 对齐块状 CSV 文件名（`ins.enabled=on` 输出，由 `AlignedWriter` 输出） |
| `rslt_filename` | str | `"RTKLC.rslt"` | — | 有效文件名 | 组合导航结果文件名（`.rslt`，`ins.enabled=on/tc` 输出，100Hz，ECEF 位置/速度 + 姿态，由 `RSLTWriter` 输出） |
| `position_format` | str | `"llh"` | — | `llh` / `xyz` | 位置输出格式。`llh`=经纬度高 (lat/lon/h)；`xyz`=ECEF 直角坐标 |
| `time_format` | str | `"gpst"` | — | `gpst` / `datetime` | 时间输出格式。`gpst`=GPS 周+周内秒；`datetime`=YYYY/MM/DD HH:MM:SS.sss |
| `trace_level` | int | `0` | — | 0 / 1 / 2 / 3 | trace 文件级别。0=off；1=info；2=detail；3=debug。生成与主输出同名的 `.trace` 文件，存放于 `output_dir`（由 `TraceFileWriter` 重定向 rtklib-py trace 输出，Unix 时间戳转 GPS 周+周内秒，过滤无效调试行） |
| `log_raw_data` | bool | `false` | — | true/false | 是否记录原始数据到 `raw/` |
| `log_level` | str | `"INFO"` | — | `DEBUG` / `INFO` / `WARNING` / `ERROR` | 运行日志级别 |
| `terminal_summary_interval` | int | `10` | — | >0 | 终端摘要间隔（每 N 条解算结果输出一次摘要） |
| `filter_debug_log_enable` | int | `0` | — | 0 / 1 | 1=输出滤波调试日志（F/H/P 矩阵等） |

> **三种运行模式的输出文件**：
>
> | 模式 | gnss_source | ins.enabled | 输出文件 |
> |------|-------------|-------------|---------|
> | 纯 GNSS | `internal` | `off` | `gnss_filename`（`.pos`，`SolutionLogger` + `SolutionWriter`） |
> | 松组合 | `internal`/`external` | `on` | `gnss_filename`（`.pos`）+ `aligned_filename`（`.csv`，`AlignedWriter`）+ `rslt_filename`（`.rslt`，100Hz，`RSLTWriter`） |
> | 紧组合 | `internal` | `tc` | `rslt_filename`（`.rslt`，100Hz，`TcStream` + `RSLTWriter`） |

---

## 附录：配置文件对比

| 配置文件 | gnss_source | positioning_mode | ins.enabled | 用途 |
|---------|-------------|------------------|-------------|------|
| `data/config.yaml` | internal | rtk | lc | 完整配置模板（含所有参数） |
| `data/spp-ins-lc.yaml` | internal | spp | lc | SPP + 松组合参考配置 |
| `data/rtdtc.yaml` | internal | rtd | tc | RTD + 紧组合参考配置 |
| `phone/rtdtc.yaml` | internal | rtd | tc | 手机端 RTD + 紧组合参考配置 |
| `data/ignav-rtktc.conf` | internal | rtk | tc | ignav 紧组合参考配置 |

## 附录：与初始化.md 的参数对应关系

| 初始化.md 参数 | config.yaml 参数 | 说明 |
|---------------|-----------------|------|
| `alignnment_posvelatt_mode` | `ins.alignnment_posvelatt_mode` | 直接对应 |
| `alignnment_attitude_mode` | `ins.alignnment_attitude_mode` | 直接对应 |
| `motion_threshold` | `ins.motion_threshold` | **已废弃**（保留兼容），由三阈值替代 |
| `static_speed_threshold` | `ins.static_speed_threshold` | 直接对应（新增，[初始化.md 5.4.3](file:///home/mxl/workplace/gipylib/skills/初始化.md#543-三组独立阈值static_speed--dynamic_speed--angular_velocity)） |
| `dynamic_speed_threshold` | `ins.dynamic_speed_threshold` | 直接对应（新增，动对准速度下限） |
| `angular_velocity_threshold_deg` | `ins.angular_velocity_threshold_deg` | 直接对应（新增，动对准角速度上限） |
| `imu_coordinate_system` | `ins.imu_coordinate_system` | 直接对应（新增，`FRD`/`RFU`，`ImuSensor` 读取时自动转换） |
| `alignnment_dynamic_method` | `ins.alignnment_dynamic_method` | 直接对应（新增） |
| `gnss_velocity_fallback` | `ins.gnss_velocity_fallback` | 直接对应（新增） |
| `gnss_buffer_size` | `ins.gnss_buffer_size` | 直接对应（新增） |
| `gnss_outage_threshold` | `ins.reboot` | 等价（保留原参数名 `reboot`） |
| `imu_outage_threshold` | `ins.imu_outage_threshold` | 直接对应（新增） |
| `timestamp_jump_threshold` | `ins.timestamp_jump_threshold` | 直接对应（新增） |
| `cov_divergence_threshold` | `ins.cov_divergence_threshold` | 直接对应（新增） |
| `enable_gnss_mode_degrade_reboot` | `ins.enable_gnss_mode_degrade_reboot` | 直接对应（新增） |
| `gnss_mode_degrade_threshold` | `ins.gnss_mode_degrade_threshold` | 直接对应（新增） |
| `reboot_use_prior_state` | `ins.reboot_use_prior_state` | 直接对应（新增） |
| `initial_pos` / `initial_vel` / `initial_att` | `ins.initial_pos` / `initial_vel` / `initial_att` | 直接对应 |
| `initial_gyro_bias` 等 IMU 误差 | `ins.initial_gyro_bias` 等 | 直接对应 |
| `gyro_bias_std` 等 IMU 噪声 | `ins.gyro_bias_std` 等 | 直接对应 |
| `leverarm` / `antlever` | `ins.leverarm` / `ins.antlever` | 直接对应 |
| `initial_imu_angle` / `initial_imu_leverarm` | `ins.initial_imu_angle` / `ins.initial_imu_leverarm` | 直接对应 |
| `imu_angle_std` / `imu_leverarm_std` | `ins.imu_angle_std` / `ins.imu_leverarm_std` | 直接对应 |
| `evaluate_imu_angle` | `ins.estimate_mounting_angle` | 重命名（单滤波 StateIndex 参数块管理） |
| `estimate_leverarm` / `estimate_mounting_angle` / `estimate_imu_leverarm` / `estimate_time_sync` | `ins.estimate_*` | 直接对应（单滤波可选参数块开关） |
| `pos_psd` | `ins.pos_psd` | 直接对应（LC EKF 必需项，位置随机游走 PSD） |
| `use_define_variance_pos_vel` / `use_define_variance_att` | `ins.use_define_variance_pos_vel` / `ins.use_define_variance_att` | 直接对应 |
| `initial_pos_std` / `initial_vel_std` / `initial_att_std` | `ins.initial_pos_std` / `initial_vel_std` / `initial_att_std` | 直接对应 |
| `data_rate` | `ins.data_rate` | 直接对应 |

## 附录：配置加载校验规则

参考 `src/utility/config_loader.py::load_config`：

1. **`coupling_mode` 已废弃**：存在该字段则报错，改用 `ins.enabled` 控制组合模式
2. **`ins.enabled` 必填**：必须为 `off` / `on` / `tc`
3. **`gnss_source` 校验**：必须为 `internal` 或 `external`
4. **`ins.enabled=off` 校验**：`gnss_source` 必须为 `internal`（纯 GNSS 模式不支持外部结果输入）
5. **基站坐标归一化**：`rb_format=llh` 时由 `_normalize_rb()` 转为 ECEF xyz
6. **输出格式校验**：`position_format` ∈ {llh, xyz}；`time_format` ∈ {gpst, datetime}；`trace_level` ∈ {0,1,2,3}
7. **`external` 模式校验**：
   - `ins.enabled` 必须为 `on`
   - `data_rate` 必须为 100
   - `external_sol_format` 必须为 `pos`（当前仅支持）
8. **`internal` 模式校验**：
   - `positioning_mode` 必填，必须为 `spp` / `rtd` / `rtk`
   - `rover_path` 必填
   - `eph_path` 必填
   - `positioning_mode=rtk/rtd` 时 `base_path` 必填
   - `ins.enabled=on/tc` 时 `imu_data_path` 必填
