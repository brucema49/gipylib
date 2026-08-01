# 配置文件使用手册

本手册详细讲解 `data/config.yaml` 配置文件中所有参数的含义、单位、默认值与取值范围。

配置文件采用 YAML 格式，包含三大段：`gnss`（GNSS 解算）、`ins`（组合导航）、`output`（输出配置）。另有 `tc` 段在紧组合模式下生效。

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
- [3. INS 配置 (ins)](#3-ins-配置-ins)
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
| `nf` | int | `2` | 频率数：1=单频（L1）；2=双频（L1+L2/L5） |
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
| `maxinno` | float | `1.0` | m | 载波相位周跳/粗差阈值 |
| `maxcode` | float | `30.0` | m | 伪距粗差阈值 |
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
| `pos_psd` | float | `0.0` | m²/s | 位置随机游走 PSD（TC 模式设 0，LC 模式建议 5.0） |
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
| `gnss_t` | array | `["GPS", "GLO", "GAL"]` | 启用星座列表 |
| `freq_ix0` | dict | `{GPS: 0, GLO: 4, GAL: 0, BDS: 0}` | 第一频率索引（L1/E1/B1I） |
| `freq_ix1` | dict | `{GPS: 1, GLO: 5, GAL: 2, BDS: 3}` | 第二频率索引（L2/E5a/B2b） |
| `freq_table` | array | `[1.57542e9, 1.22760e9, 1.17645e9, 1.20714e9, 1.60200e9, 1.24600e9]` | 频率表 [Hz] |
| `dfreq_glo` | array[2] | `[0.56250e6, 0.43750e6]` | GLONASS 频率间隔 [L1, L2] [Hz] |

> **注意**：北斗卫星观测无需特殊处理，rtklib-py 中 BDS 星历解码器受限。

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

## 3. INS 配置 (ins)

### 3.1 主开关

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `enabled` | str | `"lc"` | 主开关：`lc`=松组合 / `off`=纯 GNSS / `tc`=紧组合 |

> **重要**：不再使用 `coupling_mode` 字段。`ins.enabled` 是唯一的模式控制参数。

### 3.2 数据路径与采样率

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `imu_data_path` | str | `"data/cpt_euroc.csv"` | IMU 数据文件路径 |
| `data_rate` | int | `100` | IMU 采样率 [Hz]（`external` 模式仅支持 100） |
| `imu_format` | str | `"euroc"` | IMU 数据格式：`gpst`=GPS 周+周内秒；`euroc`=Unix 纳秒时间戳 |
| `imu_coordinate_system` | str | `"RFU"` | IMU 原始坐标系：`FRD`=前右下（默认）；`RFU`=右前上（读取时自动转换） |

**IMU 数据格式说明**：
- `gpst` 格式：CSV 文件，列为 `week, sow, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z`
- `euroc` 格式：CSV 文件，列为 `timestamp_ns, gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z`（Unix 纳秒时间戳）

**坐标系说明**：
- 项目内部统一使用 FRD（前-右-下）坐标系
- 若 IMU 原始数据为 RFU（右-前-上），设置 `imu_coordinate_system: "RFU"`，读取时自动转换（FRD = [RFU_y, RFU_x, -RFU_z]）

### 3.3 使能开关（NHC/ZUPT/ZARU）

| 参数 | 类型 | 默认值 | 说明 |
|------|------|--------|------|
| `nhc_enable` | int | `0` | NHC（非完整性约束）开关：0=关 / 1=开 |
| `zupt_enable` | int | `0` | ZUPT（零速更新）开关：0=关 / 1=3D 零速更新 |
| `zaru_enable` | int | `0` | ZARU（零角速率更新）开关：0=关 / 1=零角速率更新 |

**约束互斥逻辑**：
- 静态（GNSS 速度 < `static_speed_threshold`）：ZUPT + ZARU（互斥于 NHC）
- 运动：NHC（互斥于 ZUPT/ZARU）

### 3.4 初始对准

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `static_speed_threshold` | float | `0.5` | m/s | 静态检测阈值（GNSS 速度 < 此值判定为静态） |
| `dynamic_speed_threshold` | float | `3.0` | m/s | 动态速度阈值（平面速度 > 此值才可动对准） |
| `angular_velocity_threshold_deg` | float | `30.0` | deg/s | 动态角速度阈值（陀螺范数 < 此值才可动对准） |
| `alignnment_dynamic_method` | str | `"position_diff"` | — | 动对准方法：`auto`/`velocity_vector`/`position_diff` |
| `gnss_buffer_size` | int | `5` | — | 位置差分 GNSS 历元缓冲区大小（5 历元 span=4s，首尾差分计算速度） |
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

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `initial_gyro_bias` | array[3] | `[0, 0, 0]` | deg/h | 初始陀螺零偏 [x, y, z]（×dh2rs → rad/s） |
| `initial_acce_bias` | array[3] | `[0, 0, 0]` | mGal | 初始加计零偏 [x, y, z]（×1e-6×g0 → m/s²，g0=9.7803267715） |

> 当前默认置 0，依赖 EKF 在线估计零偏。

### 3.6 杆臂与安装角

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `leverarm` | array[3] | `[0.0, 0.0, 0.0]` | m | GNSS 天线杆臂（b 系 FRD） |
| `initial_imu_angle` | array[2] | `[0, 0]` | deg | 初始 IMU 安装角 [pitch, yaw]（roll=0） |
| `initial_imu_leverarm` | array[3] | `[0.0, 0.0, 0.0]` | m | 初始 IMU 杆臂（b→v） |

### 3.7 初始不确定度

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

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `gyro_psd` | float | `3.388e-09` | rad²/s | 陀螺过程噪声 PSD |
| `accel_psd` | float | `2.604e-06` | m²s⁻³ | 加计过程噪声 PSD |
| `gyro_bias_psd` | float | `2.612e-14` | rad²s⁻³ | 陀螺零偏随机游走 PSD |
| `acce_bias_psd` | float | `1.661e-09` | m²s⁻⁵ | 加计零偏随机游走 PSD |
| `pos_psd` | float | `5.0` | m²/s | 位置随机游走 PSD（LC 模式必需，防止 P_pos 趋零导致滤波锁死） |
| `pos_diff_vel_std` | float | `0.15` | m/s | 位置差分速度 sigma 下限 |
| `vel_psd` | float | `0.5` | m²/s² | 速度随机游走 PSD |
| `innov_reject_threshold` | float | `0.0` | m | 位置创新拒绝阈值（0=禁用） |
| `innov_reject_warmup` | int | `100` | — | 创新拒绝预热历元数（前 N 个 GNSS 历元不拒绝） |

> **调参提示**：`pos_psd` 是 LC EKF 必需项。无 `pos_psd` 时 P_pos 在量测更新后趋近 0，K→0，滤波器锁死无法跟踪 GNSS。`pos_psd=5.0` 使 P_pos 在 1s 内增长约 5，配合 R=0.0225（sigma=0.15）使 K≈0.98。

### 3.9 NHC 配置

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `nhc_std` | float | `0.05` | m/s | NHC 速度观测标准差 |
| `nhc_max_vel` | float | `0.5` | m/s | NHC 单维速度 guard（侧向速度超此值跳过该维） |
| `nhc_max_gyro` | float | `30.0` | deg/s | NHC 角速率 guard（剧烈转弯跳过整个 NHC） |
| `nhc_decimation` | int | `5` | epoch | NHC 抽样间隔（5=100Hz 下 20Hz） |

### 3.10 ZUPT 配置

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `zupt_std` | float | `0.05` | m/s | ZUPT 速度观测标准差 |
| `zupt_max_vel` | float | `0.1` | m/s | ZUPT 速度 guard |
| `zupt_max_gyro` | float | `10.0` | deg/s | ZUPT 角速率 guard |
| `zupt_min_count` | int | `15` | epoch | ZUPT 最小间隔 |

### 3.11 ZARU 配置

| 参数 | 类型 | 默认值 | 单位 | 说明 |
|------|------|--------|------|------|
| `zaru_std` | float | `0.01745` | rad/s | ZARU 角速率观测标准差（1°/s） |
| `zaru_max_vel` | float | `0.1` | m/s | ZARU 速度 guard |
| `zaru_max_gyro` | float | `5.0` | deg/s | ZARU 角速率 guard |
| `zaru_min_count` | int | `100` | epoch | ZARU 最小间隔 |

### 3.12 静态检测配置

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

状态向量固定 15 维：`[δr^e(3), δv^e(3), δψ^e(3), δb_g(3), δb_a(3)]`。以下开关启用可选参数块：

| 参数 | 类型 | 默认值 | 维数 | 说明 |
|------|------|--------|------|------|
| `estimate_leverarm` | int | `1` | 3 | 1=估计 GNSS 天线杆臂 |
| `estimate_mounting_angle` | int | `0` | 2 | 1=估计 IMU 安装角 [pitch, yaw] |
| `estimate_imu_leverarm` | int | `0` | 3 | 1=估计 IMU 杆臂 b→v（需 `estimate_mounting_angle=1`） |
| `estimate_time_sync` | int | `0` | 1 | 1=估计时间对齐误差 |

> 启用杆臂在线估计后，`.rslt` 文件每行追加 6 列（lever_x, lever_y, lever_z, sdlx, sdly, sdlz）。杆臂参数 1Hz 更新频率，Qins=3 时更新。

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
- 量测更新在 `n_meas < 4` 时跳过（防止发散）
- 启动阶段使用 RTK 计算结果初始化（不使用 SPP）

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
  trace_level: 0
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
| 5 | Q | GNSS 质量标志（1=FIX, 2=FLOAT, 5=SPP） |
| 6 | Qins | INS 状态标志（0=未初始化, 2=机械编排, 3=量测更新） |
| 7 | ns | 卫星数 |
| 8-13 | sdn, sde, sdu, sdne, sdeu, sdun | 位置协方差 |
| 14-15 | age, ratio | 差分龄期 / AR ratio |
| 16-18 | vx, vy, vz | ECEF 速度 |
| 19-24 | sdvx, sdvy, sdvz, sdvxy, sdvyz, sdvzx | 速度协方差 |
| 25-27 | roll, pitch, yaw | 姿态角（度） |
| 28-30 | sdroll, sdpitch, sdyaw | 姿态协方差 |
| 31-36 | lever_x, lever_y, lever_z, sdlx, sdly, sdlz | 杆臂参数（仅 `estimate_leverarm=1`） |

---

## 6. 配置校验规则

配置加载时（`src/utility/config_loader.py`）执行以下校验：

1. **`coupling_mode` 已废弃**：若配置文件含 `coupling_mode` 字段将报错，请删除并使用 `ins.enabled`
2. **`ins.enabled` 必填**：取值必须为 `lc`/`off`/`tc`
3. **`gnss_source` 校验**：必须为 `internal` 或 `external`
4. **纯 GNSS 模式约束**：`ins.enabled=off` 时 `gnss_source` 必须为 `internal`
5. **基站坐标格式校验**：`rb_format` 必须为 `xyz` 或 `llh`；`llh` 时自动转 ECEF
6. **输出格式校验**：`position_format` ∈ {llh, xyz}；`time_format` ∈ {gpst, datetime}；`trace_level` ∈ {0, 1, 2, 3}
7. **`external` 模式约束**：`ins.enabled` 必须为 `lc`，`data_rate` 必须为 100
8. **`internal` 模式约束**：`positioning_mode` 必填（spp/rtd/rtk）；`rover_path`、`eph_path` 必填；`rtk`/`rtd` 模式 `base_path` 必填；`ins.enabled=lc/tc` 时 `imu_data_path` 必填

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

ins:
  enabled: "lc"
  imu_data_path: "data/cpt_euroc.csv"
  data_rate: 100
  imu_format: "euroc"
  imu_coordinate_system: "RFU"
  estimate_leverarm: 1

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

ins:
  enabled: "tc"
  imu_data_path: "data/cpt_euroc.csv"
  data_rate: 100
  imu_format: "euroc"
  imu_coordinate_system: "RFU"

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

### 7.4 基站坐标使用 llh 格式

```yaml
gnss:
  rb_format: "llh"
  rb: [34.052235, -118.243683, 100.0]   # [lat_deg, lon_deg, h_m]
```

加载时自动转换为 ECEF xyz，内部统一使用 xyz。

### 7.5 输出 ECEF 坐标 + 日期时间格式

```yaml
output:
  position_format: "xyz"
  time_format: "datetime"
  trace_level: 2
```

输出文件中位置列为 ECEF [x, y, z]（米），时间列为 `YYYY/MM/DD HH:MM:SS.sss` 格式，并生成 `.trace` 文件（detail 级别）。
