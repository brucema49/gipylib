# GInsStream 整体代码框架

> GNSS/INS 松/紧组合流式导航项目，基于纯 threading + 队列流水线流式读取架构，实现 SPP+RTK/NHC/ZUPT 松组合融合导航。
> 矩阵运算使用 numpy，框架参考 gnss_ins_lc_nhc、GINav、GREAT-MSF，采用面向对象三大特性（封装/继承/多态）+ ABC 类继承体系设计，保留紧组合扩展接口。
>
> **时间系统约定**：全框架内部统一使用 **Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）**。
> rtklib-py 的 `gtime_t.time` 即 Unix 整数秒，`gtime_t.sec` 为不足秒的小数部分。
> 输入端（`formators.py`）通过 `gpst_to_unix(week, sow)` 把 GPS 周+周内秒转为 Unix 时间戳；
> 输出端（`solution_writer.py` / `aligned_writer.py`）通过 `unix_to_gpst()` 转回 (week, sow) 写文件。
> 时间转换工具：`src/core/time_utils.py`（`gpst_to_unix` / `unix_to_gpst`，`GPST_EPOCH_UNIX = 315964800`）。
>
> **当前实现状态**：
> - ✅ 已实现：rtklib-py 已吸收到 `src/core/gnss/rtklib/`（7 个核心模块），通过 `_CfgProxy` 单例管理配置
> - ✅ 已实现：`SppProcessor` / `RtkProcessor` 薄封装 rtklib-py 的 `pntpos` / `relpos`（SPP 含多普勒测速 `estvel` / `resdop`）
> - ✅ 已实现：`InternalGnssSensor`（内部模式传感器线程）/ `GnssSolSensor`（外部模式传感器线程）
> - ✅ 已实现：`ImuSensor` + `ImuFormator`（IMU CSV 流式读取，含 RFU→FRD 坐标系自动转换）+ `Aligner`（IMU 积攒 + GNSS 收割匹配）
> - ✅ 已实现：`SolutionWriter` / `AlignedWriter` / `Logger` / `SolutionLogger`
> - ✅ 已实现：三种运行模式——internal+off（纯 GNSS，输出 .pos）/ external+on（外部对齐 CSV）/ internal+on（内部对齐 CSV，实时解算+IMU 对齐）
> - ✅ 已实现：`src/core/ins/initializer.py::InsInitializer`（INS 初始化，三种模式：静态 / 速度矢量 / 位置差分，三阈值检验，详见 [初始化.md](file:///home/mxl/workplace/gipylib/skills/初始化.md)）
> - ✅ 已实现：`src/core/ins/` 下 `interpolator.py` / `earth_param.py` / `attitude.py`（初始化支撑模块）
> - 🚧 预留：INS 机械编排核心 `InsCore` / 双滤波 EKF `LcIntegration` / NHC / ZUPT / 紧组合接口（下一阶段实现）

### 设计模式总览

本项目综合运用多种设计模式组织框架，使各模块职责清晰、可扩展、可替换：

| 设计模式 | 应用场景 | 解决的问题 |
|---------|---------|-----------|
| **工厂模式** | `SensorFactory` 动态创建传感器实例 | 根据配置文件动态装配 IMU/Rover/Ref/Eph/GnssSol 传感器，避免硬编码 |
| **策略模式** | 前端里程计算法切换 | 前端：IMU 机械编排为主，IMU 不可用时可降级为 GNSS 纯解算（SPP/RTK/RTD） |
| **模板方法** | `BaseSensor.get_data()` / `InsKf` 框架流程 | 基类定义算法骨架，子类填充具体步骤 |
| **依赖注入** | 框架通过构造函数接收具体传感器/策略实例 | 框架不 new 具体类，由外部装配后注入，便于测试与替换 |

> **数据通路**：本项目采用**纯队列流水线**，不使用观察者模式。
> 数据流向：Streamer → Queue → Scheduler → EstimateQueue → EstimatorThread → SolutionQueue → Logger，全程无回调。详见第 5.1e 节"数据通路"。

### 性能优化策略

| 优化 | 手段 | 收益 |
|------|------|------|
| **纯 threading** | 所有数据流通过 `queue.Queue` + 独立线程驱动 | 模型简单可靠，调试方便，避免 asyncio/threading 混用复杂度 |
| **OOP 三大特性** | 封装状态、继承复用、多态替换 | 框架稳定可扩展，新传感器/算法只需继承基类即可接入 |

> **并发模型约束**：不使用 `asyncio`（避免与 threading 混用），不使用 `multiprocessing.shared_memory`（避免跨进程内存管理复杂度）。所有数据通路通过 `queue.Queue` 在线程间传递。

---

## 目录

- [1. 项目概述](#1-项目概述)
- [2. 技术规格](#2-技术规格)
- [3. 整体架构](#3-整体架构)
- [4. 目录结构](#4-目录结构)
- [5. 类继承体系与设计模式](#5-类继承体系与设计模式)
- [6. 核心数据类型](#6-核心数据类型)
- [7. 模块职责与接口](#7-模块职责与接口)
- [8. 数据流与处理流程](#8-数据流与处理流程)
- [9. 各子目录详细规划](#9-各子目录详细规划)
- [10. 配置文件](#10-配置文件)
- [11. 扩展接口（紧组合预留）](#11-扩展接口紧组合预留)
- [12. 参考代码映射](#12-参考代码映射)

---

## 1. 项目概述

GInsStream 是一个 GNSS/INS 组合导航项目，核心特点是**流式读取**——不全量加载文件，逐行/逐块读取，O(1) 内存占用。采用 ABC 类继承体系 + 设计模式（工厂/策略/依赖注入）设计，参考 GREAT-MSF 的 t_gbasemodel / t_gcombmodel / t_gsins / t_gsinskf / t_gintegration 模式。当前阶段实现松组合功能，包含：

- **GNSS 定位**：SPP 单点定位 + RTK（含 RTD 退化模式）
- **外部 GNSS 结果流**：支持直接读取外部 GNSS 定位结果文件，跳过内部 GNSS 解算
- **INS 导航**：IMU 机械编排（支持增量式和速率式两种 IMU 数据格式）
- **松组合融合**：真正双滤波 EKF（主滤波 15 维 / 18 维含 GNSS 杆臂 + NHC 子滤波 5 维），含 NHC 约束与 ZUPT 零速更新
- **时间对齐**：IMU/GNSS 数据插值（参考 KF-GINS `imuInterpolate` 增量切分方案，**时间对齐是本项目重中之重**）
- **流式架构**：纯 threading + 队列流水线，时间对齐，反压机制

### 1.1 RTK 与 RTD 的关系

RTD 是 RTK 的退化模式（RTK 不进行模糊度固定，仅使用码双差）。二者统一在 RtkProcessor 中：

- **RTK 载波差分模式**：使用载波相位双差观测值 + 码双差，进行模糊度固定
- **RTD 码差分模式**：仅使用码双差观测值，不进行模糊度解算（RTK 的退化）

通过配置 `rtk_mode: "carrier"` / `rtk_mode: "code"` 切换。

### 1.2 GNSS 数据源模式

本项目支持两种 GNSS 数据源模式，通过配置文件 `gnss_source` 切换：

| 模式 | 配置值 | 输入数据流 | 内部GNSS解算 | 适用场景 |
|------|--------|-----------|-------------|---------|
| **内部解算模式** | `"internal"` | IMU + Rover + Eph + Ref | 需要（SPP/RTK） | 标准组合导航 |
| **外部结果模式** | `"external"` | IMU + GNSS定位结果文件 | 不需要 | 使用外部实时GNSS解算流 |

- **内部解算模式**（默认）：程序内部进行 GNSS 解算（SPP/RTK），需要 Rover/Ref/Eph 数据流
- **外部结果模式**：直接读取外部 GNSS 定位结果文件（如 POS/NMEA 格式），无需 Rover/Ref/Eph 数据流，仅需要 IMU 数据流和 GNSS 结果流。适用于已有外部 GNSS 解算结果的场景

---

## 2. 技术规格

| 项目 | 规格 |
|------|------|
| **EKF 状态向量** | **真正双滤波**：主滤波 15 维（E 系，可选 +GNSS 杆臂 3 维 = 18 维）；NHC 子滤波 5 维（v 系，IMU 安装角 2 + IMU 杆臂 3）。两套独立 P/F/H/Q/R 矩阵和反馈机制 |
| **GNSS 模式** | SPP（初始化/单点）+ RTK（载波差分/码差分退化）+ 外部结果流 |
| **NHC 约束** | v 系（车体坐标系）下侧向+垂向速度=0（2 维观测），需安装角旋转矩阵 R_b^v，由 NHC 子滤波估计 |
| **ZUPT 约束** | 零速更新：检测车辆静止时将 3 维速度误差作为量测约束（与 NHC 互斥：静止用 ZUPT，运动用 NHC） |
| **机械编排** | E 系（ECEF）下进行，参考 gnss_ins_lc_nhc |
| **IMU 数据格式** | 增量式（Δθ/Δv）和速率式（ω/f）均支持 |
| **时间对齐** | 增量切分（参考 KF-GINS `imuInterpolate`），4 种时间对齐情况处理；**时间对齐是重中之重**；无时间同步状态参数 |
| **时间系统** | 全框架内部统一使用 Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）；输入端 `gpst_to_unix(week, sow)`，输出端 `unix_to_gpst()` |
| **矩阵运算** | numpy |
| **坐标系** | e 系：ECEF；b 系：IMU 本体 FRD；v 系：车体 FRD；n 系：导航系 ENU（仅输出用） |
| **参考框架** | gnss_ins_lc_nhc（C++松组合+NHC+安装角）、GINav（MATLAB组合导航）、GREAT-MSF（C++类继承体系）、KF-GINS（增量切分时间对齐） |
| **设计模式** | 工厂模式（SensorFactory 创建传感器）、策略模式（前端里程计算法切换）、依赖注入（构造函数接收具体实例） |
| **并发模型** | 纯 threading + `queue.Queue`（不使用 asyncio，不使用 shared_memory） |
| **流式架构** | 参考 StreamDesign.md |

### 2.1 双滤波架构

> 参考 gnss_ins_lc_nhc 的滤波架构设计。本项目采用**真正双滤波架构**，主滤波与 NHC 子滤波完全独立，各自维护状态向量、协方差矩阵、F/H/Q/R 矩阵和反馈机制。

#### 2.1.1 主滤波（E 系，P1 矩阵）

**状态向量 x1**（15 维固定 + 3 维可选 GNSS 杆臂 = 15 或 18 维）：

```
索引  状态变量           符号           维度  坐标系
──────────────────────────────────────────────────
0-2   位置误差           δr^e          3    E 系
3-5   速度误差           δv^e          3    E 系
6-8   姿态误差角         δφ^e          3    E 系
9-11  陀螺零偏           δb_g          3    b 系
12-14 加计零偏           δb_a          3    b 系
15-17 GNSS杆臂误差       δl_gnss       3    b 系（可选）
```

- **F1 矩阵**：15×15 或 18×18，由 INS 机械编排误差传播构造
- **H1 矩阵**：GNSS 位置/速度量测、ZUPT 量测（速度约束）
- **P1 矩阵**：15×15 或 18×18，独立维护
- **反馈**：修正 δr^e、δv^e、δφ^e、δb_g、δb_a、δl_gnss

#### 2.1.2 NHC 子滤波（v 系，P2 矩阵）

**状态向量 x2**（固定 5 维）：

```
索引  状态变量           符号           维度  坐标系
──────────────────────────────────────────────────
0-1   IMU安装角误差      δθ_imu        2    v 系 (pitch, yaw)
2-4   IMU杆臂误差        δl_imu        3    b 系
```

- **F2 矩阵**：5×5，安装角/杆臂视为常量过程，F2 ≈ I（或带小量随机游走）
- **H2 矩阵**：NHC 量测（v 系下侧向/垂向速度对 δθ_imu、δl_imu 的偏导）
- **P2 矩阵**：5×5，独立维护，与 P1 无耦合
- **反馈**：修正 R_b^v 安装角矩阵和 IMU 杆臂 l_imu

#### 2.1.3 双滤波协同关系

```
主滤波 P1 (15×15 或 18×18)         NHC 子滤波 P2 (5×5)
   │                                  │
   │  ←── 共享 INS 状态 ──←           │
   │     (v^e, C_b^e, ω_ib^b)        │
   │                                  │
   ▼                                  ▼
GNSS 位置/速度量测更新            NHC 量测更新
ZUPT 量测更新                     (v 系下侧向/垂向速度=0)
   │                                  │
   ▼                                  ▼
P1 反馈：修正 δr^e, δv^e,          P2 反馈：修正 δθ_imu, δl_imu
δφ^e, δb_g, δb_a, δl_gnss        (更新 R_b^v, l_imu)
```

**关键说明**：
- NHC 子滤波使用主滤波的 v^e、C_b^e、ω_ib^b 作为量测构造的输入，但不修改主滤波状态
- NHC 子滤波的反馈结果（R_b^v、l_imu）反馈给 NHC 量测构造模块，下次 NHC 量测使用更新后的参数
- 两套滤波器各自独立 time_update / meas_update / feedback，互不干扰

#### 2.1.4 配置开关

- `estimate_imu_angle: true` → 启用 NHC 子滤波的安装角估计（2维：pitch, yaw）
- `estimate_imu_leverarm: true` → 启用 NHC 子滤波的 IMU 杆臂估计（3维）
- `estimate_gnss_leverarm: false` → 主滤波估计 GNSS 杆臂（3维，默认关闭）
- 当 `estimate_imu_angle=false` 且 `estimate_imu_leverarm=false` 时，NHC 子滤波不运行，NHC 直接使用固定安装角和杆臂
- 安装角只有 2 维（pitch, yaw），roll 假设为 0（参考 gnss_ins_lc_nhc）
- **不包含时间同步参数**：本项目通过 KF-GINS 增量切分方案精确对齐 IMU/GNSS 时间

#### 2.1.5 安装角与 NHC 的关系

```
b 系（IMU 本体坐标系）与 v 系（车体坐标系）之间通过安装角旋转矩阵 R_b^v 关联：

  v^v = R_b^v * v^b

NHC 约束在 v 系（车体坐标系）下进行：
  v_right_v = 0  →  侧向速度约束
  v_down_v  = 0  →  垂向速度约束

安装角误差 δθ_imu 导致 R_b^v 不准确，影响 NHC 约束精度。
NHC 子滤波估计安装角后可修正 R_b^v，提高 NHC 约束效果。
```

### 2.2 NHC 观测模型（v 系下，双滤波分离）

NHC 假设车辆在车体坐标系(v系)侧向和垂向速度为零：

```
v^v = R_b^v * C_e^b * v^e + R_b^v * [ω_eb^b ×] * l_imu^b

NHC 约束（v 系下）：
  v_right_v  = 0  →  侧向速度约束
  v_down_v   = 0  →  垂向速度约束

观测方程（参考 gnss_ins_lc_nhc odo.md）：
  Z_nhc = [v_right_v; v_down_v] = [0; 0]
```

**双滤波下的 H 矩阵分离**（参考 gnss_ins_lc_nhc navstate.cc:343-353）：

NHC 量测同时依赖主滤波状态（δv^e、δφ^e、δb_g）和 NHC 子滤波状态（δθ_imu、δl_imu）。
为保持两套 P 矩阵独立，将 H 矩阵拆分为两部分：

```
主滤波贡献 H1（作用到 P1，仅更新 P1 中速度/姿态/陀螺零偏部分）：
  H1_vel   = R_b^v * C_e^b^T              (速度对速度误差 δv^e)
  H1_att   = -R_b^v * C_e^b^T * [v^e ×]  (速度对姿态误差 δφ^e)
  H1_gyro  = R_b^v * [l_imu^b ×]          (速度对陀螺零偏 δb_g)

NHC 子滤波贡献 H2（作用到 P2，仅更新 P2 中安装角/杆臂部分）：
  H2_angle = [v^v ×]_{:,2:3}              (速度对安装角误差 δθ_imu，仅2列)
  H2_lever = R_b^v * [ω_eb^b ×]           (速度对IMU杆臂误差 δl_imu)
```

**实施方式**：
- 主滤波量测更新：`K1 = P1 * H1^T * (H1*P1*H1^T + R)^-1`，只更新主滤波状态
- NHC 子滤波量测更新：`K2 = P2 * H2^T * (H2*P2*H2^T + R_nhc)^-1`，只更新子滤波状态
- R 矩阵需根据 H1、H2 分别构造对应的量测噪声

> **NHC/ZUPT 互斥**：NHC 与 ZUPT 不可同时启用。静止检测触发 ZUPT（3D 速度约束，作用于主滤波 P1），此时 NHC 不参与；运动状态下使用 NHC（2D 侧向/垂向速度约束，作用于 P1+P2）。

---

## 3. 整体架构

```
模式A: 内部解算模式 (gnss_source: "internal")

┌──────────────────────────────────────────────────────────────────────────┐
│                           Main (主线程)                                   │
│   解析配置 → 创建 Scheduler → 启动线程 → 主循环等待                        │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
       ┌───────────────────────┼───────────────────────┐
       ▼                       ▼                       ▼
┌─────────────┐        ┌─────────────┐         ┌─────────────┐
│  Stream 层  │        │ Integration │         │   Log 层    │
│ (数据读取)  │ queue  │  (数据集成)  │ est_q   │ (日志/输出) │
│             │.put()─→│  Scheduler   │.put()─→│             │
│ IMU Stream  │        │  (仅转发)    │        │ Solution    │
│ Rover Stream│        │  ※不做时间对齐│        │ Trace       │
│ Eph Stream  │        │             │         │ Raw Data    │
│ Ref Stream  │        │             │         │             │
└─────────────┘        └──────┬──────┘         └─────────────┘
                              │
                              ▼
                       ┌─────────────┐
                       │  Core 层    │
                       │ (核心解算)   │
                       │             │
                       │ gnss/       │
                       │  ├ SPP      │
                       │  └ RTK(RTD) │
                       │ imu/        │
                       │  ├ 机械编排  │
                       │  ├ 初始化   │
                       │  └ 插值器   │
                       │ estimator/  │
                       │  ├ EKF(双滤波)│
                       │  │  ├ P1 主滤波│
                       │  │  └ P2 NHC子滤波│
                       │  ├ NHC(H1/H2)│
                       │  └ ZUPT(仅P1)│
                       │  + 时间对齐  │
                       │  (imupre/imucur)│
                       │  (pending_gnss deque)│
                       └─────────────┘


模式B: 外部结果模式 (gnss_source: "external")

┌──────────────────────────────────────────────────────────────────────────┐
│                           Main (主线程)                                   │
│   解析配置 → 创建 Scheduler → 启动线程 → 主循环等待                        │
└──────────────────────────────┬───────────────────────────────────────────┘
                               │
       ┌───────────────────────┼───────────────────────┐
       ▼                       ▼                       ▼
┌─────────────┐        ┌─────────────┐         ┌─────────────┐
│  Stream 层  │        │ Integration │         │   Log 层    │
│ (数据读取)  │ queue  │  (数据集成)  │ est_q   │ (日志/输出) │
│             │.put()─→│  Scheduler   │.put()─→│             │
│ IMU Stream  │        │  (仅转发)    │        │ Solution    │
│             │        │  ※不做时间对齐│        │ Trace       │
│ GNSS Sol    │        │  ※无GNSS解算 │        │ Raw Data    │
│  Stream     │        │  直接使用    │         │             │
│ (定位结果)  │        │  外部结果    │         │             │
└─────────────┘        └──────┬──────┘         └─────────────┘
                              │
                              ▼
                       ┌─────────────┐
                       │  Core 层    │
                       │ (核心解算)   │
                       │             │
                       │ imu/        │
                       │  ├ 机械编排  │
                       │  ├ 初始化   │
                       │  └ 插值器   │
                       │ estimator/  │
                       │  ├ EKF(双滤波)│
                       │  │  ├ P1 主滤波│
                       │  │  └ P2 NHC子滤波│
                       │  ├ NHC(H1/H2)│
                       │  └ ZUPT(仅P1)│
                       │  + 时间对齐  │
                       │  (imupre/imucur)│
                       │  (pending_gnss deque)│
                       └─────────────┘
                       ※ gnss/ 模块不参与
```

**与 StreamDesign.md 的关系**：StreamDesign.md 已规划了 Stream 层和 Integration 层的详细设计，本文档聚焦 Core 层（gnss/imu/estimator）和 Log 层的详细规划，以及 ABC 类继承体系设计。

**关键架构变化（相比初版）**：
1. **纯队列流水线**：Streamer → Queue → Scheduler → estimate_queue → Estimator → solution_queue → Logger，无观察者回调
2. **时间对齐下沉到 Estimator**：Scheduler 仅转发，时间对齐由 Estimator 内部处理（参考 KF-GINS 增量切分）
3. **双滤波架构**：主滤波 P1（15/18 维）+ NHC 子滤波 P2（5 维），独立 P/F/H/Q/R 矩阵和反馈
4. **纯 threading**：不使用 asyncio，不使用 shared_memory

---

## 4. 目录结构

> **说明**：以下为实际实现的目录结构。✅ 标记已实现，🚧 标记预留（当前未实现）。

```
gipylib/
├── skills/                        # 指导文档
│   ├── GInsStream.md              # 本文件：整体代码框架
│   ├── StreamDesign.md            # 流式读取框架设计
│   ├── gnss.md                    # GNSS 算法架构设计
│   ├── imu.md                     # IMU 机械编排方案
│   ├── estimator.md               # 融合估计指导方案
│   ├── logANDoutput.md            # 日志流和输出结果流方案
│   └── conf.md                    # 配置文件说明
│
├── library/                       # 参考代码库（不修改，不导入）
│   ├── rtklib-py/                 # GNSS 解算参考（已吸收到 src/core/gnss/rtklib/）
│   └── pyrinex/                   # RINEX 解码参考
│
├── tools/                         # 组合导航参考代码（不修改，不导入）
│   ├── GINav/                     # MATLAB 组合导航
│   ├── gnss_ins_lc_nhc/          # C++ 松组合+NHC（主要参考）
│   └── GREAT-MSF-main/           # C++ 类继承体系参考（ABC模式）
│
├── data/                          # 测试数据 + 配置
│   ├── config.yaml                # 统一配置文件
│   ├── cpt0870.19o                # 流动站 RINEX 观测文件
│   ├── cpt0870_base.19o           # 基站 RINEX 观测文件
│   ├── brdm0870.19p               # 星历文件
│   ├── cpt_imu.csv                # IMU 数据文件
│   └── spp.pos                    # 外部 GNSS 结果文件（示例）
│
├── output/                        # 输出目录
│   ├── test_spp.pos               # SPP 解算结果（internal+spp+off 模式）
│   ├── test_rtk.pos               # RTK 解算结果（internal+rtk+off 模式）
│   ├── aligned.csv                # 对齐块状输出（external+on 模式）
│   └── aligned_internal_rtk.csv   # 对齐块状输出（internal+on 模式，实时 RTK+IMU）
│
├── src/                           # 具体代码实现
│   ├── __init__.py
│   ├── main.py                    # ✅ 主入口（三种运行模式装配：路径 A/B/C）
│   │
│   ├── stream/                    # ✅ 传感器抽象层 + 流式读取
│   │   ├── __init__.py
│   │   ├── base.py                # ✅ BaseSensor 传感器抽象基类（强制 get_data）
│   │   ├── factory.py             # ✅ SensorFactory 工厂模式动态创建传感器
│   │   ├── formators.py           # ✅ ImuFormator / PosSolFormator 解码器
│   │   ├── imu_sensor.py          # ✅ ImuSensor IMU 传感器线程
│   │   ├── gnss_sol_sensor.py     # ✅ GnssSolSensor 外部 GNSS 结果传感器线程
│   │   └── internal_gnss_sensor.py # ✅ InternalGnssSensor 内部 GNSS 解算传感器线程
│   │
│   ├── core/                      # 核心解算相关代码
│   │   ├── __init__.py
│   │   ├── thread_control.py      # ✅ ThreadControl 线程控制
│   │   ├── time_utils.py          # ✅ 时间转换（gpst_to_unix / unix_to_gpst）
│   │   ├── data_types.py          # ✅ 核心数据类型（ImuMeasurement / GnssSolution / SensorData / AlignedBlock）
│   │   │
│   │   ├── gnss/                  # ✅ GNSS 解算模块
│   │   │   ├── __init__.py
│   │   │   ├── gnss_processor.py  # ✅ GnssProcessor(ABC) 抽象基类
│   │   │   ├── spp_processor.py   # ✅ SppProcessor 薄封装 rtklib-py pntpos
│   │   │   ├── rtk_processor.py   # ✅ RtkProcessor 薄封装 rtklib-py relpos
│   │   │   ├── solution_converter.py # ✅ sol_to_gnss_solution（rtklib-py Sol → GnssSolution）
│   │   │   ├── rtklib_config_adapter.py # ✅ RtklibEnv + build_params（YAML → rtklib-py 配置注入）
│   │   │   └── rtklib/            # ✅ rtklib-py 已吸收的子包（相对导入 + _CfgProxy 单例）
│   │   │       ├── __init__.py
│   │   │       ├── config.py      # ✅ _CfgProxy 单例配置管理
│   │   │       ├── ephemeris.py   # ✅ 星历计算
│   │   │       ├── mlambda.py     # ✅ MLAMBDA 模糊度解算
│   │   │       ├── pntpos.py      # ✅ SPP 单点定位
│   │   │       ├── postpos.py     # ✅ 后处理驱动
│   │   │       ├── rinex.py       # ✅ RINEX 解码
│   │   │       ├── rtkcmn.py      # ✅ 通用工具（坐标变换、时间等）
│   │   │       └── rtkpos.py      # ✅ RTK 相对定位
│   │   │
│   │   ├── ins/                   # 🚧 IMU/INS 模块（预留，当前未实现）
│   │   │   ├── ins_core.py        # 🚧 InsCore INS核心（参考 t_gsins）
│   │   │   ├── ins_kf.py          # 🚧 InsKf 卡尔曼滤波基类（参考 t_gsinskf）
│   │   │   ├── imu_preprocess.py  # 🚧 ImuPreprocessor 预处理基类
│   │   │   ├── interpolator.py    # 🚧 Interpolator 插值基类（参考 t_ginterp）
│   │   │   ├── ins_init.py        # 🚧 INS 初始化（粗对准+精对准）
│   │   │   ├── earth_param.py     # 🚧 地球参数（重力、自转角速度等）
│   │   │   └── attitude.py        # 🚧 姿态表示与转换（四元数/欧拉角/DCM）
│   │   │
│   │   └── estimator/             # 🚧 融合估计模块（预留，当前未实现）
│   │       ├── integration.py     # 🚧 Integration 组合导航集成基类
│   │       ├── ekf.py             # 🚧 EKF 滤波器核心（双滤波）
│   │       ├── lc_estimator.py    # 🚧 LcEstimator 松组合估计器
│   │       ├── lc_measurement.py  # 🚧 松组合量测更新
│   │       ├── lc_feedback.py     # 🚧 松组合反馈
│   │       ├── nhc.py             # 🚧 NHC 约束（H1/H2 拆分）
│   │       ├── zupt.py            # 🚧 ZUPT 零速更新（仅 P1）
│   │       ├── state_vector.py    # 🚧 双滤波状态索引
│   │       └── tc_interface.py    # 🚧 紧组合预留接口
│   │
│   ├── log/                       # ✅ 日志输出流
│   │   ├── __init__.py
│   │   ├── logger.py              # ✅ Logger（external+on 模式，消费 imu_queue + gnss_queue）
│   │   ├── solution_logger.py     # ✅ SolutionLogger（internal+off 模式，消费 gnss_queue）
│   │   ├── writer_base.py         # ✅ WriterBase 输出器抽象基类
│   │   ├── solution_writer.py     # ✅ SolutionWriter rtklib 风格 .pos 输出
│   │   ├── aligned_writer.py      # ✅ AlignedWriter 对齐块状 CSV 输出
│   │   ├── aligner.py             # ✅ Aligner IMU 积攒 + GNSS 收割的匹配器
│   │   ├── trace_writer.py        # 🚧 TraceWriter 运行轨迹/调试输出（预留）
│   │   └── raw_data_writer.py     # 🚧 RawDataWriter 原始数据记录（预留）
│   │
│   ├── tools/                     # ✅ 辅助工具
│   │   ├── __init__.py
│   │   ├── verify_rtklib_py.py    # ✅ 直接运行 rtklib-py 出参考结果
│   │   └── compare_pos.py         # ✅ 逐历元 .pos 文件比对
│   │
│   └── utility/                   # ✅ 工具
│       ├── __init__.py
│       ├── config_loader.py       # ✅ 配置解析（data/config.yaml）
│       └── rinex_simplifier.py    # ✅ RINEX 简化器
│
└── tests/                         # 测试
    ├── test_gnss/                 # ✅ GNSS 模块测试
    ├── test_imu/                  # 🚧 IMU 模块测试（预留）
    └── test_estimator/            # 🚧 Estimator 模块测试（预留）
```

---

## 5. 类继承体系与设计模式

> 参考 GREAT-MSF 的 t_gbasemodel / t_gcombmodel / t_gsins / t_gsinskf / t_gintegration 模式，
> 使用 Python ABC（抽象基类）定义类继承层次，明确抽象方法与子类职责。
> 在 ABC 继承体系之上，叠加工厂/策略/依赖注入等设计模式，体现面向对象三大特性：
> - **封装**：传感器内部状态与解码逻辑封装在 BaseSensor 子类中，外部仅通过 `get_data()` 获取数据
> - **继承**：BaseSensor → 各具体传感器；InsKf → LcEstimator/TcEstimator；OdometryStrategy → 各前端策略
> - **多态**：框架持有 BaseSensor / OdometryStrategy 抽象引用，运行时调用具体子类实现

### 5.0 设计模式在框架中的落地

#### 5.0.1 传感器抽象层 BaseSensor（封装 + 多态的根基）

所有传感器（IMU/Rover/Ref/Eph/GnssSol）的统一抽象基类。强制子类实现 `get_data()` 接口，
框架通过 `BaseSensor` 抽象引用操作具体传感器，实现"针对接口编程，不针对实现编程"。

#### 5.0.2 工厂模式 SensorFactory（动态创建传感器实例）

根据配置文件动态创建传感器实例，避免在主程序中硬编码 `new IMUStreamer(...)`。
工厂返回 `BaseSensor` 抽象类型，主程序不感知具体子类。

#### 5.0.3 策略模式（仅前端里程计算法切换）

- **前端策略 OdometryStrategy**：仅 `ImuMechStrategy` 一种实现，IMU 不可用时可降级为 GNSS 纯解算
- **不再设置后端策略 FusionStrategy**：松组合 EKF + NHC + ZUPT 直接由 `LcEstimator` 实现，紧组合由 `TcEstimator` 实现，无需策略抽象
- 框架持有前端策略抽象引用，构造函数注入具体策略

#### 5.0.4 依赖注入（构造函数接收具体实例）

框架核心类（Integration/Estimator）通过构造函数接收具体传感器实例与策略实例，
而非内部 new。便于单元测试注入 mock，也便于运行时替换实现。

#### 5.0.5 数据通路（纯队列流水线，无观察者模式）

本项目**不使用观察者模式**。数据通过 `queue.Queue` 在线程间传递：

```
Streamer 线程          Scheduler 线程         Estimator 线程         Logger 线程
  │                       │                       │                       │
  │── queue.put() ──→     │                       │                       │
  │                       │── estimate_queue.put() ──→                   │
  │                       │                       │── solution_queue.put() ──→
  │                       │                       │                       │
```

- 各 Streamer 在独立线程中读取数据，调用 `queue.put()` 写入对应数据队列
- Scheduler 线程从各数据队列拉取数据，仅做**转发**到 `estimate_queue`（不做时间对齐）
- Estimator 线程从 `estimate_queue` 拉取数据，内部完成**时间对齐 + EKF 融合**，结果写入 `solution_queue`
- Logger 线程从 `solution_queue` 拉取结果输出

**时间对齐下沉到 Estimator**：参考 KF-GINS 的 `newImuProcess()` + `imuInterpolate()`，由 Estimator 内部维护 imupre/imucur 和 pending_gnss（deque 缓冲），处理 4 种时间对齐情况。

### 5.1 类继承总览

```
═══════════════════════════════════════════════════════════════
  传感器抽象层（工厂模式）
═══════════════════════════════════════════════════════════════
BaseSensor(ABC)                   # 传感器抽象基类，强制 get_data() 接口（无观察者）
├── ImuSensor(BaseSensor)         # IMU 传感器（机械编排前端数据源）
├── GnssRoverSensor(BaseSensor)   # 流动站 GNSS 观测传感器（内部模式）
├── GnssRefSensor(BaseSensor)     # 基准站 GNSS 观测传感器（内部模式）
├── EphSensor(BaseSensor)         # 星历传感器
└── GnssSolSensor(BaseSensor)     # 外部 GNSS 定位结果传感器（外部模式）

SensorFactory                     # 工厂：根据配置创建 BaseSensor 实例

═══════════════════════════════════════════════════════════════
  策略层（仅前端里程计算法切换，无后端策略）
═══════════════════════════════════════════════════════════════
OdometryStrategy(ABC)             # 前端里程计算法策略基类
└── ImuMechStrategy               #   IMU 机械编排前端策略（唯一实现）

═══════════════════════════════════════════════════════════════
  GNSS 算法层（ABC 继承）
═══════════════════════════════════════════════════════════════
BaseModel(ABC)                    # 参考 t_gbasemodel, 抽象方法: cmb_equ()
└── CombDD(BaseModel)             # 参考 t_gcombDD, 双差组合(用于RTK)，单一分支

GnssProcessor(ABC)               # GNSS处理器基类（内部模式）
├── SppProcessor(GnssProcessor)  # SPP处理器
└── RtkProcessor(GnssProcessor)  # RTK处理器(RTD是其退化模式)

GnssSolutionProvider(ABC)        # GNSS结果提供者基类
├── GnssInternalProvider         # 内部解算提供者（组合GnssProcessor）
└── GnssExternalProvider         # 外部结果文件提供者（读取GNSS定位结果文件）

═══════════════════════════════════════════════════════════════
  INS / 估计层（ABC 继承 + 模板方法，双滤波架构）
═══════════════════════════════════════════════════════════════
InsCore                          # 参考 t_gsins, INS核心(姿态/速度/位置更新)
InsKf(ABC)                       # 参考 t_gsinskf, INS卡尔曼滤波基类
├── LcEstimator(InsKf)           # 松组合估计器（持有 P1 主滤波 + P2 NHC 子滤波）
└── TcEstimator(InsKf)           # 紧组合估计器(预留)

Integration(ABC)                 # 参考 t_gintegration, 组合导航集成基类
├── LcIntegration(Integration)   # 松组合集成
└── TcIntegration(Integration)   # 紧组合集成(预留)

Interpolator(ABC)                # 参考 t_ginterp, 插值基类
├── LinearInterpolator           # 线性插值
└── PolyInterpolator             # 多项式插值

ImuPreprocessor(ABC)             # IMU预处理基类
├── RateImuPreprocessor          # 速率式IMU预处理
└── DeltaImuPreprocessor         # 增量式IMU预处理

═══════════════════════════════════════════════════════════════
  输出层（ABC 继承）
═══════════════════════════════════════════════════════════════
WriterBase(ABC)                  # 输出器基类
├── SolutionWriter(WriterBase)   # 解算结果输出
├── TraceWriter(WriterBase)      # 轨迹调试输出
└── RawDataWriter(WriterBase)    # 原始数据输出
```

### 5.1b BaseSensor — 传感器抽象层（强制 get_data 接口）

> 传感器抽象层是框架"针对接口编程"的根基。所有传感器（文件流/串口/网络）
> 统一实现 `get_data()`，上层通过 `BaseSensor` 抽象引用操作，不感知具体数据源。
> BaseSensor **不承担观察者模式 Subject 角色**，数据通过 `queue.put()` 传递。

```python
from abc import ABC, abstractmethod
from typing import Optional

class BaseSensor(ABC):
    """传感器抽象基类

    强制子类实现 get_data()，统一所有传感器数据获取接口。
    具体子类负责：文件/串口/网络读取 → Formator 解码 → 标准化数据。

    设计意图：
      - 封装：传感器内部读取与解码细节对外不可见
      - 多态：框架持有 BaseSensor 列表，循环调用 get_data() 而不关心具体类型
      - 可扩展：新增传感器只需继承 BaseSensor 并实现 get_data()

    注意：不包含观察者模式 Subject 接口（attach/detach/notify）。
    数据通过 queue.Queue 在线程间传递，详见 5.1e 数据通路。
    """

    def __init__(self, name: str):
        self._name = name
        self._paused = False

    @abstractmethod
    def get_data(self) -> Optional[object]:
        """获取一条数据（非阻塞，无数据返回 None）

        所有传感器必须实现此接口。返回的数据类型由具体传感器决定：
          - ImuSensor      → ImuMeasurement
          - GnssRoverSensor → GnssMeasurement
          - EphSensor      → EphemerisData
          - GnssSolSensor  → GnssSolution

        框架主循环通过此接口拉取数据，实现"针对接口编程"。
        """
        ...

    @abstractmethod
    def open(self) -> bool:
        """打开数据源（文件/串口/网络连接）"""
        ...

    @abstractmethod
    def close(self) -> None:
        """关闭数据源，释放资源"""
        ...

    # ── 流控接口（调度器使用）──
    def pause(self) -> None:
        self._paused = True

    def resume(self) -> None:
        self._paused = False

    @property
    def name(self) -> str:
        return self._name


class ImuSensor(BaseSensor):
    """IMU 传感器：读取 IMU 文件/串口 → Formator 解码 → ImuMeasurement
    """
    ...


class GnssSolSensor(BaseSensor):
    """外部 GNSS 定位结果传感器（外部模式专用）

    读取 POS/NMEA/CSV 结果文件，get_data() 返回 GnssSolution。
    """
    ...
```

### 5.1c SensorFactory — 工厂模式动态创建传感器

> 工厂根据配置文件动态装配传感器，主程序不 new 具体类，只持有 `list[BaseSensor]`。
> 这样切换数据源（文件→串口→网络）或新增传感器类型时，主程序无需修改。

```python
class SensorFactory:
    """传感器工厂

    根据配置动态创建 BaseSensor 实例。返回抽象类型，调用方不感知具体子类。
    支持的传感器类型由配置 files/formators 决定。
    """

    @staticmethod
    def create_sensors(config: dict, formators: dict) -> list:
        """根据配置创建所有传感器实例列表

        参数:
            config: 配置字典（files, playback_mode, gnss_source 等）
            formators: 各传感器的 Formator 实例字典

        返回:
            list[BaseSensor]，主程序遍历调用 get_data() 即可
        """
        sensors = []

        # IMU 传感器（始终创建）
        sensors.append(ImuSensor(
            name="imu",
            formator=formators["imu"],
            file_path=config["files"]["imu"],
            playback_mode=config.get("playback_mode", "asap"),
            speed=config.get("speed", 100.0),
        ))

        # 根据 GNSS 数据源模式创建不同传感器
        if config.get("gnss_source", "internal") == "internal":
            # 内部解算模式：需要 Rover/Ref/Eph
            sensors.append(GnssRoverSensor(name="rover", ...))
            sensors.append(EphSensor(name="eph", ...))
            if config.get("mode") == "differential":
                sensors.append(GnssRefSensor(name="ref", ...))
        else:
            # 外部结果模式：只需 GnssSol 传感器
            sensors.append(GnssSolSensor(
                name="gnss_sol",
                file_path=config["gnss_external"]["solution_file"],
                format=config["gnss_external"]["solution_format"],
            ))

        return sensors
```

### 5.1d 策略模式 — 仅前端里程计算法切换

> 策略模式将"算法族"封装为独立类，使它们可互换。
> 本项目简化策略模式：**仅保留前端策略 OdometryStrategy**，后端融合策略已删除（松组合 EKF + NHC + ZUPT 直接由 `LcEstimator` 实现）。

```python
class OdometryStrategy(ABC):
    """前端里程计算法策略基类

    定义"产生里程计结果"的统一接口。当前唯一实现：IMU 机械编排。
    IMU 不可用时可降级为 GNSS 纯解算（不通过策略实现，直接由 Scheduler 切换数据源）。
    """

    @abstractmethod
    def compute(self, sensor_data: object) -> Optional[object]:
        """根据传感器数据计算里程计结果"""
        ...

    @abstractmethod
    def reset(self) -> None:
        """重置前端状态"""
        ...


class ImuMechStrategy(OdometryStrategy):
    """IMU 机械编排前端策略

    使用 InsCore 进行 E 系机械编排，输出 INS 导航状态。
    适用于 IMU 数据驱动的递推场景。"""
    ...


# ── 框架通过构造函数注入前端策略（依赖注入）──
class Integration:
    """组合导航集成（持有前端策略引用，不 new 具体策略）

    通过构造函数接收前端策略，实现依赖注入。
    后端融合由 LcEstimator 直接实现，无需策略抽象。
    """
    def __init__(self,
                 frontend: OdometryStrategy,   # 注入前端策略
                 sensors: list,                # 注入传感器列表（BaseSensor）
                 config: dict):
        self._frontend = frontend
        self._sensors = sensors
```

### 5.1e 数据通路 — 纯队列流水线（替代观察者模式）

> 本项目**不使用观察者模式**。多传感器数据同步与融合触发通过**纯队列流水线**实现：
> 数据由 Streamer 写入队列，Scheduler 转发，Estimator 消费，全程无回调。

#### 5.1e.1 队列定义

| 队列名 | 生产者 | 消费者 | 数据类型 | 说明 |
|--------|--------|--------|---------|------|
| `imu_queue` | ImuSensor | Scheduler | ImuMeasurement | IMU 高频数据流 |
| `rover_queue` | GnssRoverSensor | Scheduler | GnssMeasurement | 流动站观测（内部模式） |
| `ref_queue` | GnssRefSensor | Scheduler | GnssMeasurement | 基准站观测（内部模式） |
| `eph_queue` | EphSensor | Scheduler | EphemerisData | 星历数据（内部模式） |
| `gnss_sol_queue` | GnssSolSensor | Scheduler | GnssSolution | 外部 GNSS 结果（外部模式） |
| `estimate_queue` | Scheduler | Estimator | SensorData（统一封装） | Estimator 输入队列，**时间对齐由 Estimator 内部处理** |
| `solution_queue` | Estimator | Logger | Solution | 融合结果输出队列 |

#### 5.1e.2 数据流路径

```
模式A: 内部解算模式

  ImuSensor  ──→ imu_queue  ──┐
  RoverSensor ──→ rover_queue ──┤
  EphSensor  ──→ eph_queue  ──┼──→ Scheduler ──→ estimate_queue ──→ Estimator ──→ solution_queue ──→ Logger
  RefSensor  ──→ ref_queue  ──┘    (仅转发)        (时间对齐+EKF)     (输出)


模式B: 外部结果模式

  ImuSensor  ──→ imu_queue  ──┐
  GnssSolSensor ──→ gnss_sol_queue ──┴──→ Scheduler ──→ estimate_queue ──→ Estimator ──→ solution_queue ──→ Logger
                                          (仅转发)        (时间对齐+EKF)     (输出)
```

#### 5.1e.3 Scheduler 职责（仅转发，不做时间对齐）

- Scheduler 从各数据队列拉取数据，**仅做转发**到 `estimate_queue`
- **不做时间对齐**：时间对齐下沉到 Estimator 内部处理（参考 KF-GINS `newImuProcess()` + `imuInterpolate()`）
- Scheduler 维护 INITIALIZED 状态机，但状态切换通过 `estimate_queue` 传递给 Estimator

#### 5.1e.4 Estimator 职责（时间对齐 + EKF 融合）

- Estimator 从 `estimate_queue` 拉取数据
- 维护 `imupre` / `imucur`（前一历元 / 当前历元 IMU）
- 维护 `pending_gnss`（**deque 缓冲**，避免丢失多个 GNSS 历元）
- 处理 4 种时间对齐情况（参考 estimator.md 第 9 节）
- 完成双滤波 EKF 融合，结果写入 `solution_queue`

#### 5.1e.5 Logger 职责（消费 solution_queue）

- Logger 线程从 `solution_queue` 拉取融合结果
- 调用 `SolutionWriter` / `TraceWriter` / `RawDataWriter` 输出
- 详见 logANDoutput.md

### 5.2 BaseModel — 观测模型基类

> 本项目简化 CombModel 层次：删除 CombIF（无电离层组合）和 CombAll（全频组合），
> 仅保留 CombDD（双差组合，用于 RTK）单一分支。BaseModel → CombDD 直接继承。

```python
from abc import ABC, abstractmethod

class BaseModel(ABC):
    """观测模型抽象基类，参考 GREAT-MSF t_gbasemodel"""

    @abstractmethod
    def cmb_equ(self, epoch: float, params: dict, obs_data: dict) -> tuple:
        """组合观测方程

        Args:
            epoch: 当前历元时间戳
            params: 当前参数状态
            obs_data: 观测数据

        Returns:
            (B, P, l): 系数矩阵、权重矩阵、残差向量
        """
        ...


class CombDD(BaseModel):
    """双差组合模型，参考 GREAT-MSF t_gcombDD，用于RTK

    本项目单一分支：删除 CombIF（无电离层组合）和 CombAll（全频组合），
    RTK 双差观测方程直接由 CombDD 实现。
    """

    def __init__(self, config: dict):
        self._frequency = config.get("frequency", 2)
        self._sig_code = {}    # 各系统码观测噪声
        self._sig_phase = {}   # 各系统相位观测噪声
        self._base_data = None
        self._site = ""
        self._site_base = ""

    def set_base_data(self, base_data: list):
        """设置基准站观测数据"""
        ...

    def set_site(self, site: str, site_base: str):
        """设置流动站/基准站标识"""
        ...

    def cmb_equ(self, epoch: float, params: dict, obs_data: dict) -> tuple:
        ...
```

### 5.3 InsCore / InsKf — INS 核心与卡尔曼滤波

```python
class InsCore:
    """INS 核心类，参考 GREAT-MSF t_gsins

    负责姿态、速度、位置的单步更新（E 系下机械编排），
    不涉及卡尔曼滤波，仅纯 INS 递推。
    """

    def __init__(self, qbe0=None, ve0=None, re0=None, t0=0.0):
        self.att = None       # [3] 姿态 (pitch, roll, yaw) rad（n系下，输出用）
        self.ve = None        # [3] ECEF 速度 m/s
        self.re = None        # [3] ECEF 位置 m
        self.qbe = None       # 四元数 q_be（b系到e系）
        self.Cbe = None       # [3,3] DCM C_b^e（b系到e系）
        self.eb = None        # [3] 陀螺零偏
        self.db = None        # [3] 加计零偏

    def align_coarse(self, wmm: np.ndarray, vmm: np.ndarray) -> np.ndarray:
        """粗对准，参考 t_gsins::align_coarse"""
        ...

    def update(self, wm: list, vm: list, dt: float) -> None:
        """INS 机械编排更新，参考 t_gsins::Update"""
        ...


class InsKf(ABC):
    """INS 卡尔曼滤波基类，参考 GREAT-MSF t_gsinskf

    管理 EKF 状态向量、协方差、F/H 矩阵，
    提供时间更新、量测更新、反馈的抽象框架。
    """

    def __init__(self, config: dict):
        self.sins = InsCore()        # INS 核心实例
        self.Ft = None               # 状态转移矩阵
        self.Pk = None               # 状态协方差矩阵
        self.Hk = None               # 量测矩阵
        self.Rk = None               # 量测噪声矩阵
        self.Xk = None               # 状态向量
        self.Qt = None               # 过程噪声向量
        self.lever = None            # 杆臂

    @abstractmethod
    def set_Ft(self) -> None:
        """构造状态转移矩阵 Ft，参考 t_gsinskf::set_Ft"""
        ...

    @abstractmethod
    def set_Hk(self) -> None:
        """构造量测矩阵 Hk，参考 t_gsinskf::set_Hk"""
        ...

    def time_update(self, kfts: float, inflation: float = 1.0) -> None:
        """EKF 时间更新，参考 t_gsinskf::time_update"""
        ...

    @abstractmethod
    def meas_update(self) -> int:
        """EKF 量测更新，参考 t_gsinskf::_meas_update"""
        ...

    @abstractmethod
    def feedback(self) -> None:
        """状态反馈，参考 t_gsinskf::feedback"""
        ...


class LcEstimator(InsKf):
    """松组合估计器"""

    def set_Ft(self) -> None:
        ...

    def set_Hk(self) -> None:
        ...

    def meas_update(self) -> int:
        ...

    def feedback(self) -> None:
        ...


class TcEstimator(InsKf):
    """紧组合估计器（预留）"""

    def set_Ft(self) -> None:
        ...

    def set_Hk(self) -> None:
        ...

    def meas_update(self) -> int:
        ...

    def feedback(self) -> None:
        ...
```

### 5.4 GnssProcessor — GNSS 处理器

```python
class GnssProcessor(ABC):
    """GNSS 处理器抽象基类（内部模式）"""

    @abstractmethod
    def process_epoch(self, obs: dict, eph: dict, **kwargs) -> 'GnssSolution':
        """处理单个历元

        Args:
            obs: 观测数据
            eph: 星历数据
            **kwargs: 额外参数（如基准站数据等）

        Returns:
            GnssSolution 解算结果
        """
        ...


class SppProcessor(GnssProcessor):
    """SPP 单点定位处理器"""

    def process_epoch(self, obs: dict, eph: dict, **kwargs) -> 'GnssSolution':
        ...


class RtkProcessor(GnssProcessor):
    """RTK 处理器，RTD 是其退化模式

    通过 rtk_mode 配置切换：
      - "carrier": 载波差分（完整RTK，含模糊度固定）
      - "code":    码差分（RTD退化模式，仅码双差）
    """

    def __init__(self, config: dict):
        self._rtk_mode = config.get("rtk_mode", "code")  # "carrier" or "code"
        self._comb_model = None  # CombDD 实例
        self._max_baseline = config.get("max_baseline", 20.0)

    def process_epoch(self, obs: dict, eph: dict, **kwargs) -> 'GnssSolution':
        ...

    @property
    def is_carrier_mode(self) -> bool:
        """是否为载波差分模式"""
        return self._rtk_mode == "carrier"

    @property
    def is_code_mode(self) -> bool:
        """是否为码差分模式（RTD退化）"""
        return self._rtk_mode == "code"
```

### 5.4b GnssSolutionProvider — GNSS 结果提供者

```python
class GnssSolutionProvider(ABC):
    """GNSS 结果提供者抽象基类

    统一内部解算和外部结果文件两种 GNSS 数据源，
    对 Integration 层提供统一的 GnssSolution 接口。
    """

    @abstractmethod
    def get_solution(self, timestamp: float) -> Optional['GnssSolution']:
        """获取指定时刻的 GNSS 解算结果

        Args:
            timestamp: 目标时间戳 (s)

        Returns:
            GnssSolution 或 None（无可用结果时）
        """
        ...

    @abstractmethod
    def is_available(self) -> bool:
        """当前 GNSS 结果是否可用"""
        ...


class GnssInternalProvider(GnssSolutionProvider):
    """内部解算提供者

    组合 GnssProcessor（SppProcessor/RtkProcessor），
    从原始观测值解算得到 GnssSolution。
    """

    def __init__(self, gnss_processor: GnssProcessor, config: dict):
        self._processor = gnss_processor
        self._mode = config.get("mode", "differential")

    def get_solution(self, timestamp: float) -> Optional['GnssSolution']:
        """调用 GnssProcessor 解算"""
        ...

    def is_available(self) -> bool:
        ...


class GnssExternalProvider(GnssSolutionProvider):
    """外部结果文件提供者

    从 GNSS 定位结果文件（POS/NMEA/CSV）流式读取，
    直接返回 GnssSolution，无需内部 GNSS 解算。
    """

    def __init__(self, config: dict):
        self._solution_format = config.get("gnss_solution_format", "pos")
        self._solution_buffer = []  # 缓冲最近结果用于插值
        self._interpolator = None   # 可选：对结果插值到IMU时刻

    def get_solution(self, timestamp: float) -> Optional['GnssSolution']:
        """从外部结果文件读取或插值"""
        ...

    def is_available(self) -> bool:
        ...

    def feed_solution(self, sol: 'GnssSolution') -> None:
        """从 GnssSolStreamer 接收新的定位结果"""
        ...
```

### 5.5 Integration — 组合导航集成

```python
class Integration(ABC):
    """组合导航集成基类，参考 GREAT-MSF t_gintegration

    统一管理 INS 递推、GNSS 更新、反馈的完整流程。
    t_gintegration 继承自 t_gsinskf + t_gpvtflt，
    本项目采用组合模式替代多继承。
    """

    def __init__(self, config: dict):
        self._ins_kf = None          # InsKf 子类实例
        self._gnss_provider = None   # GnssSolutionProvider 子类实例（统一内部/外部）
        self._interpolator = None    # Interpolator 实例
        self._imu_preprocessor = None  # ImuPreprocessor 实例

    @abstractmethod
    def process_batch(self, beg: float, end: float) -> int:
        """批量处理，参考 t_gintegration::processBatchFB"""
        ...

    @abstractmethod
    def process_epoch(self, epoch: float) -> int:
        """单历元处理，参考 t_gintegration::_processEpoch"""
        ...

    @abstractmethod
    def gnss_update(self) -> int:
        """GNSS 量测更新，参考 t_gintegration::_GNSS_Update"""
        ...


class LcIntegration(Integration):
    """松组合集成"""

    def process_batch(self, beg: float, end: float) -> int:
        ...

    def process_epoch(self, epoch: float) -> int:
        ...

    def gnss_update(self) -> int:
        ...


class TcIntegration(Integration):
    """紧组合集成（预留）"""

    def process_batch(self, beg: float, end: float) -> int:
        ...

    def process_epoch(self, epoch: float) -> int:
        ...

    def gnss_update(self) -> int:
        ...
```

### 5.6 Interpolator — 插值器

```python
class Interpolator(ABC):
    """插值基类，参考 GREAT-MSF t_ginterp

    用于 IMU/GNSS 数据时间对齐，将不同采样率的数据
    插值到统一时间基准上。
    """

    @abstractmethod
    def interpolate(self, data: dict, target_time: float) -> float:
        """插值计算

        Args:
            data: 已知数据点 {timestamp: value}
            target_time: 目标时间戳

        Returns:
            插值结果
        """
        ...


class LinearInterpolator(Interpolator):
    """线性插值，参考 t_ginterp::linear"""

    def interpolate(self, data: dict, target_time: float) -> float:
        ...


class PolyInterpolator(Interpolator):
    """多项式插值，参考 t_ginterp::spline"""

    def __init__(self, order: int = 3):
        self._order = order

    def interpolate(self, data: dict, target_time: float) -> float:
        ...
```

### 5.7 ImuPreprocessor — IMU 预处理

```python
class ImuPreprocessor(ABC):
    """IMU 预处理抽象基类"""

    @abstractmethod
    def preprocess(self, raw_data: dict) -> dict:
        """预处理单条 IMU 数据

        Args:
            raw_data: 原始 IMU 数据

        Returns:
            标准化后的 IMU 数据
        """
        ...


class RateImuPreprocessor(ImuPreprocessor):
    """速率式 IMU 预处理（ω/f 输入）"""

    def preprocess(self, raw_data: dict) -> dict:
        ...


class DeltaImuPreprocessor(ImuPreprocessor):
    """增量式 IMU 预处理（Δθ/Δv 输入）"""

    def preprocess(self, raw_data: dict) -> dict:
        ...
```

### 5.8 WriterBase — 输出器

```python
class WriterBase(ABC):
    """输出器抽象基类"""

    @abstractmethod
    def write(self, data: dict) -> None:
        """写入数据"""
        ...

    @abstractmethod
    def write_header(self) -> None:
        """写入文件头"""
        ...


class SolutionWriter(WriterBase):
    """解算结果输出"""

    def write(self, data: dict) -> None:
        ...

    def write_header(self) -> None:
        ...


class TraceWriter(WriterBase):
    """运行轨迹/调试输出"""

    def write(self, data: dict) -> None:
        ...

    def write_header(self) -> None:
        ...


class RawDataWriter(WriterBase):
    """原始数据记录"""

    def write(self, data: dict) -> None:
        ...

    def write_header(self) -> None:
        ...
```

---

## 6. 核心数据类型

> 基础数据类型（ImuMeasurement, GnssMeasurement 等）已在 StreamDesign.md 第3节定义。
> 此处补充 Core 层和 Estimator 层需要的数据类型。

### 6.1 INS 状态（双滤波分离）

> 双滤波架构下，状态拆分为：主滤波状态 `InsState`（含 P1）+ NHC 子滤波状态 `NhcSubState`（含 P2）。

```python
@dataclass
class InsState:
    """主滤波 INS 导航状态（E 系下，参考 gnss_ins_lc_nhc NavInfo）

    对应主滤波 P1 矩阵，状态向量 x1 = [δr^e, δv^e, δφ^e, δb_g, δb_a, (δl_gnss)]
    """
    timestamp: float                     # 当前时间戳 (s, Unix 时间戳，与 rtklib-py gtime_t 一致)
    position: np.ndarray                 # [3] 位置 (ECEF, m)
    velocity: np.ndarray                 # [3] 速度 (ECEF, m/s)
    attitude: np.ndarray                 # [4] 姿态四元数 q_be (b系到e系)
    rotation: np.ndarray                 # [3,3] 旋转矩阵 C_b^e (b系到e系)
    gyro_bias: np.ndarray                # [3] 陀螺零偏 (rad/s)
    accel_bias: np.ndarray               # [3] 加计零偏 (m/s²)
    # IMU 原始观测（共享给 NHC 子滤波构造量测）
    w_ib_b: Optional[np.ndarray]         # [3] IMU角速度 (b系, rad/s)
    f_ib_b: Optional[np.ndarray]         # [3] IMU比力 (b系, m/s²)
    # GNSS 杆臂（可选，主滤波估计）
    gnss_leverarm: Optional[np.ndarray]  # [3] GNSS天线杆臂 (b系) (m)
    # 主滤波协方差矩阵 P1
    covariance_p1: np.ndarray            # [N1,N1] 主滤波协方差 (N1=15 或 18)


@dataclass
class NhcSubState:
    """NHC 子滤波状态（v 系，独立 P2 矩阵）

    对应 NHC 子滤波 P2 矩阵，状态向量 x2 = [δθ_imu(2), δl_imu(3)]
    与主滤波 P1 矩阵完全独立，互不耦合。
    """
    timestamp: float                     # 当前时间戳 (s, Unix 时间戳，与主滤波同步)
    imu_angle: np.ndarray                # [2] IMU安装角 (pitch, yaw) (rad)
    imu_leverarm: np.ndarray             # [3] IMU杆臂 (b系) (m)
    imu_angle_rotation: np.ndarray       # [3,3] R_b^v 安装角旋转矩阵
    imu_angle_quat: Optional[np.ndarray] # [4] 安装角四元数（可选）
    # NHC 子滤波协方差矩阵 P2
    covariance_p2: np.ndarray            # [5,5] NHC 子滤波协方差
    # 是否启用（由配置决定）
    enabled: bool                        # estimate_imu_angle 或 estimate_imu_leverarm 为 True 时启用
```

### 6.2 GNSS 解算结果

> **实际实现**（参考 `src/core/data_types.py`）：

```python
@dataclass
class GnssSolution:
    """外部 GNSS 结果。"""
    timestamp: float          # Unix 时间戳（秒，与 rtklib-py gtime_t 一致）
    week: int                 # GPS 周号（由 timestamp 派生，便利字段）
    position: np.ndarray      # [3] ECEF (m)
    quality: int              # 1=SPP, 2=RTD, 5=LC（与 rtklib SOLQ_* 一致）
    num_sv: int               # 使用卫星数
    sd: np.ndarray            # [3] 位置标准差 (sdx, sdy, sdz)
    cov: Optional[np.ndarray] = None  # [3,3] ECEF 协方差矩阵（可选，含非对角项）
```

> **早期设计版本**（含速度/PDOP/status 等字段，当前未实现，预留 INS 启用后扩展）：

```python
@dataclass
class GnssSolution_Ext:
    """GNSS 解算结果（松组合量测输入，预留扩展）"""
    timestamp: float                     # Unix 时间戳 (s)
    position: np.ndarray                 # [3] 位置 (ECEF m)
    velocity: Optional[np.ndarray]       # [3] 速度 (ECEF m/s, 可选)
    pos_covariance: Optional[np.ndarray] # [3,3] 位置协方差
    vel_covariance: Optional[np.ndarray] # [3,3] 速度协方差
    mode: str                            # "SPP" / "RTK" / "RTD"
    num_satellites: int                  # 使用卫星数
    pdop: float                          # PDOP
    status: str                          # "OK" / "Degraded" / "Invalid"
```

### 6.3 EKF 量测数据

```python
@dataclass
class EkfMeasurement:
    """EKF 量测数据包"""
    timestamp: float
    gnss_position: Optional[GnssSolution] = None   # GNSS 位置量测
    gnss_velocity: Optional[np.ndarray] = None      # GNSS 速度量测 (多普勒)
    nhc_available: bool = False                     # NHC 是否可用
    # 紧组合预留
    gnss_obs: Optional[GnssMeasurement] = None      # 原始GNSS观测值（紧组合用）
    ephemeris: Optional[EphemerisData] = None       # 星历数据（紧组合用）
```

---

## 7. 模块职责与接口

### 7.1 stream/ — 流式读取层

> 详细设计见 StreamDesign.md，此处仅列出与 Core 层的接口。

**输出接口**：各 Streamer 通过 `Queue[SensorData]` 输出数据到 Integration 层。

### 7.2 integration/ — 数据集成层

> 详细设计见 StreamDesign.md，此处仅列出与 Core 层的接口。

**输出接口**：Scheduler 通过 `estimate_queue` 输出 `SensorData` 到 Estimator。

### 7.3 core/gnss/ — GNSS 解算模块

> **实际实现**（rtklib-py 已吸收，不重新实现算法）：

| 文件 | 职责 | 实现状态 |
|------|------|---------|
| `gnss_processor.py` | `GnssProcessor(ABC)` 抽象基类，定义 `process_epoch(obsr, obsb)` 接口 | ✅ 已实现 |
| `spp_processor.py` | `SppProcessor` 薄封装 rtklib-py `pntpos(obsr, nav)` | ✅ 已实现 |
| `rtk_processor.py` | `RtkProcessor` 薄封装 rtklib-py `relpos(nav, obsr, obsb, sol)`，含跨历元状态管理 | ✅ 已实现 |
| `solution_converter.py` | `sol_to_gnss_solution(sol)` 把 rtklib-py `Sol` 转为本项目 `GnssSolution` | ✅ 已实现 |
| `rtklib_config_adapter.py` | `RtklibEnv` + `build_params()` 把 YAML 配置注入 rtklib-py `_CfgProxy` 单例 | ✅ 已实现 |
| `rtklib/` 子包 | rtklib-py 已吸收的 7 个核心模块（`config`/`ephemeris`/`mlambda`/`pntpos`/`postpos`/`rinex`/`rtkcmn`/`rtkpos`） | ✅ 已实现 |

> **早期设计版本**（含 `base_model.py`/`comb_model.py`/`satpos.py` 等，当前未实现，预留紧组合扩展）：
>
> | 文件 | 职责 | 状态 |
> |------|------|------|
> | `base_model.py` | 观测模型抽象基类 `BaseModel.cmb_equ()` | 🚧 预留 |
> | `comb_model.py` | 组合观测模型 `CombDD` | 🚧 预留 |
> | `satpos.py` | 卫星位置/速度计算 | 🚧 预留（当前由 rtklib-py 内部完成） |
> | `ephemeris.py` | 星历管理 | 🚧 预留（当前由 rtklib-py 内部完成） |
> | `troposphere.py` | 对流层改正 | 🚧 预留（当前由 rtklib-py 内部完成） |
> | `ionosphere.py` | 电离层改正 | 🚧 预留（当前由 rtklib-py 内部完成） |
> | `sat_az_el.py` | 卫星方位角/仰角 | 🚧 预留（当前由 rtklib-py 内部完成） |
> | `coord_transform.py` | 坐标变换工具 | 🚧 预留（当前直接调用 rtklib-py `rtkcmn.py` 函数） |

**详细指导**：见 [gnss.md](gnss.md)

### 7.4 core/imu/ — IMU/INS 模块

| 文件 | 职责 | 主要类/接口 |
|------|------|------------|
| `ins_core.py` | INS 核心（机械编排） | `InsCore.update()`, `InsCore.align_coarse()` |
| `ins_kf.py` | INS 卡尔曼滤波基类 | `InsKf.time_update()`, `InsKf.meas_update()`, `InsKf.feedback()` |
| `imu_preprocess.py` | IMU 数据预处理 | `RateImuPreprocessor`, `DeltaImuPreprocessor` |
| `interpolator.py` | 数据插值（时间对齐） | `LinearInterpolator`, `PolyInterpolator` |
| `ins_init.py` | INS 初始化 | `coarse_align()`, `fine_align()` |
| `earth_param.py` | 地球参数 | `gravity()`, `earth_rate()`, `rn()`, `rm()` |
| `attitude.py` | 姿态表示与转换 | `quat2dcm()`, `dcm2quat()`, `euler2dcm()`, `dcm2euler()` |

**详细指导**：见 [imu.md](imu.md)

### 7.5 core/estimator/ — 融合估计模块

| 文件 | 职责 | 主要类/接口 |
|------|------|------------|
| `integration.py` | 组合导航集成基类 | `Integration.process_epoch()`, `Integration.gnss_update()` |
| `ekf.py` | EKF 滤波器核心（双滤波） | `predict(x, P, F, Q)`, `update(x, P, Z, H, R)`（P1/P2 独立） |
| `lc_estimator.py` | 松组合估计器（双滤波） | `LcEstimator` (InsKf子类，持有 P1/P2 双矩阵) |
| `lc_measurement.py` | 松组合量测更新 | `gnss_pos_update()`, `gnss_vel_update()`, `nhc_update()`（NHC 拆分 H1/H2） |
| `lc_feedback.py` | 松组合反馈（独立） | `feedback_main(state, dx1)`, `feedback_nhc(nhc_state, dx2)` |
| `nhc.py` | NHC 约束（H1/H2 拆分） | `nhc_measurement(state) -> (Z, H1, H2, R)` |
| `zupt.py` | ZUPT 零速更新（仅 P1） | `detect_zero_velocity(imu) -> bool`, `zupt_measurement(state) -> (Z, H, R)` |
| `state_vector.py` | 双滤波状态索引 | `MainStateIndex`, `NhcSubStateIndex` 枚举类 |
| `tc_interface.py` | 紧组合预留接口 | `TcMeasurement` 数据类 |

**详细指导**：见 [estimator.md](estimator.md)

### 7.6 log/ — 日志输出流

> **实际实现**：

| 文件 | 职责 | 实现状态 |
|------|------|---------|
| `logger.py` | `Logger` 线程（external+on 模式，消费 imu_queue + gnss_queue，调用 Aligner 匹配后写 AlignedWriter） | ✅ 已实现 |
| `solution_logger.py` | `SolutionLogger` 线程（internal+off 模式，消费 gnss_queue，写 SolutionWriter） | ✅ 已实现 |
| `writer_base.py` | `WriterBase` 输出器抽象基类 | ✅ 已实现 |
| `solution_writer.py` | `SolutionWriter` rtklib 风格 .pos 输出（Unix → week/sow 转换） | ✅ 已实现 |
| `aligned_writer.py` | `AlignedWriter` 对齐块状 CSV 输出（IMU + GNSS 对齐后写文件） | ✅ 已实现 |
| `aligner.py` | `Aligner` IMU 积攒 + GNSS 收割的匹配器（基于 Unix 时间戳） | ✅ 已实现 |
| `trace_writer.py` | `TraceWriter` 运行轨迹/调试输出 | 🚧 预留 |
| `raw_data_writer.py` | `RawDataWriter` 原始数据记录 | 🚧 预留 |

**详细指导**：见 [logANDoutput.md](logANDoutput.md)

---

## 8. 数据流与处理流程

### 8.1 完整数据流

**模式A：内部解算模式 (gnss_source: "internal")**

```
文件系统              Stream层              Integration层          Core层               Log层
─────────            ────────             ──────────────         ──────              ──────

imu.txt ──→ IMUStreamer ──→ imu_queue ──┐
                                          │
obs.txt ──→ RoverStreamer ──→ rover_queue┤
                                          ├──→ Scheduler ──→ Integration ──→ Logger
eph.txt ──→ EphStreamer ──→ eph_queue ───┤    (时间对齐)     (EKF融合)       (输出)
                                          │       │              │
ref.txt ──→ RefStreamer ──→ ref_queue ───┘       │              │
                                                  │              │
                                                  ▼              ▼
                                           ┌──────────┐   ┌──────────────┐
                                           │ gnss/    │   │ estimator/   │
                                           │ SPP/RTK  │   │ EKF+LC+NHC   │
                                           └──────────┘   └──────────────┘
                                                  ▲
                                           ┌──────────┐
                                           │ imu/     │
                                           │ 插值对齐  │
                                           └──────────┘
```

**模式B：外部结果模式 (gnss_source: "external")**

```
文件系统              Stream层              Integration层          Core层               Log层
─────────            ────────             ──────────────         ──────              ──────

imu.txt ──→ IMUStreamer ──→ imu_queue ──┐
                                          ├──→ Scheduler ──→ Integration ──→ Logger
gnss.pos ──→ GnssSolStreamer → gnss_sol_q┘    (时间对齐)     (EKF融合)       (输出)
                                                  │              │
                                          ※ 无GNSS解算          │
                                          ※ 直接使用            ▼
                                          外部结果       ┌──────────────┐
                                                         │ estimator/   │
                                                         │ EKF+LC+NHC   │
                                                         └──────────────┘
                                                                ▲
                                                         ┌──────────┐
                                                         │ imu/     │
                                                         │ 插值对齐  │
                                                         └──────────┘
```

### 8.2 松组合处理流程（双滤波，参考 gnss_ins_lc_nhc + KF-GINS）

```
┌─────────────────────────────────────────────────────────────────────┐
│                     松组合主循环（双滤波架构）                          │
│                                                                      │
│  1. 从 estimate_queue 拉取数据（IMU 或 GNSS）                         │
│     │                                                                │
│     ▼                                                                │
│  2. 时间对齐判断（参考 KF-GINS newImuProcess + imuInterpolate）       │
│     │  4 种时间对齐情况：                                              │
│     │    case 0: 无 GNSS，仅 IMU 预测                                 │
│     │    case 1: GNSS 历元 = IMU 历元，直接融合                       │
│     │    case 2: GNSS 历元落在两个 IMU 之间，增量切分                  │
│     │    case 3: 多个 GNSS 历元待处理（pending_gnss deque 消费）      │
│     ▼                                                                │
│  3. IMU 预处理（ImuPreprocessor）                                     │
│     │  增量式/速率式 → 标准化格式                                     │
│     ▼                                                                │
│  4. INS 机械编排（InsCore.update，E 系）                             │
│     │  姿态→速度→位置更新（参考 gnss_ins_lc_nhc navmech.cc）          │
│     ▼                                                                │
│  5. 主滤波 EKF 时间更新（P1 矩阵）                                   │
│     │  x1_pred = x1 + F1 * dt * dx1                                  │
│     │  P1_pred = F1 * P1 * F1^T + Q1                                 │
│     ▼                                                                │
│  5b. NHC 子滤波 EKF 时间更新（P2 矩阵，独立）                         │
│     │  x2_pred = x2（F2 ≈ I，常量过程）                               │
│     │  P2_pred = F2 * P2 * F2^T + Q2                                 │
│     ▼                                                                │
│  6. 检查是否有 GNSS 量测？                                            │
│     │                                                                │
│     ├── 有 GNSS → 6a. 获取 GNSS 结果                                 │
│     │              ├── 内部模式: GnssInternalProvider → SPP/RTK       │
│     │              └── 外部模式: GnssExternalProvider → 直接读取        │
│     │              6b. 主滤波 GNSS 位置/速度量测更新（更新 P1）        │
│     │                                                                │
│     ▼                                                                │
│  7. 检查 NHC/ZUPT 互斥条件？                                          │
│     │                                                                │
│     ├── 运动状态 → 7a. NHC 量测更新                                   │
│     │              ├── 主滤波贡献 H1 → 更新 P1（速度/姿态/陀螺零偏）  │
│     │              └── 子滤波贡献 H2 → 更新 P2（安装角/杆臂）         │
│     │                                                                │
│     ├── 静止状态 → 7b. ZUPT 量测更新（仅主滤波 P1）                   │
│     │              3 维速度=0 约束，H 仅作用于 P1                     │
│     │                                                                │
│     ▼                                                                │
│  8. 双滤波状态反馈（独立反馈）                                        │
│     │  主滤波反馈：修正 δr^e, δv^e, δφ^e, δb_g, δb_a, δl_gnss       │
│     │  子滤波反馈：修正 δθ_imu, δl_imu（更新 R_b^v, l_imu）           │
│     │  协方差矩阵分别反馈 P1, P2                                      │
│     ▼                                                                │
│  9. 输出融合结果 → solution_queue                                    │
│     │                                                                │
│     ▼                                                                │
│  返回 1                                                              │
└─────────────────────────────────────────────────────────────────────┘
```

> **时间对齐是重中之重**：本项目支持内部解算 GNSS 和外部 GNSS 结果两种模式，
> 两种模式的 GNSS 时间戳来源不同，但时间对齐逻辑完全一致（参考 estimator.md 第 9 节）。

### 8.3 初始化流程

```
┌─────────────────────────────────────────────────────────────────┐
│                     初始化流程                                    │
│                                                                  │
│  阶段1: GNSS 先行                                                │
│  ├── SPP 解算获取初始位置（精度 ~10m）                           │
│  └── RTK 码差分模式获取差分位置（精度 ~1m，差分模式可用时）       │
│                                                                  │
│  阶段2: INS 粗对准                                               │
│  ├── 静态粗对准：利用加速度计和陀螺仪感知重力和地球自转           │
│  │   └── 水平角由重力确定，方位角由 GNSS 速度确定                │
│  ├── 动态粗对准：利用 GNSS 速度辅助                              │
│  │   └── 航向角由 GNSS 速度方向确定                              │
│  └── 输出：初始姿态 C_b^e                                       │
│                                                                  │
│  阶段3: EKF 精对准                                               │
│  ├── 初始化状态向量 x = 0（误差状态）                            │
│  ├── 初始化协方差矩阵 P                                         │
│  │   ├── 位置：SPP/RTK 精度                                      │
│  │   ├── 速度：GNSS 速度精度                                     │
│  │   ├── 姿态：粗对准精度（水平 1°, 方位 5°~10°）               │
│  │   ├── 零偏：IMU 规格书标称值                                  │
│  │   ├── 安装角：初始估计精度                                    │
│  │   ├── IMU杆臂：初始估计精度                                   │
│  │   └── GNSS杆臂：初始估计精度（可选）                          │
│  └── 运行若干历元 EKF 更新完成精对准                             │
│                                                                  │
│  阶段4: 进入正常松组合融合                                       │
└─────────────────────────────────────────────────────────────────┘
```

---

## 9. 各子目录详细规划

### 9.1 src/core/gnss/

**职责**：从 rtklib-py 二次修改，适配流式架构，实现 SPP 和 RTK（含 RTD 退化模式）。采用 BaseModel → CombDD 单一分支抽象体系。

**关键改造点**：
1. rtklib-py 是全量读取 → 改为逐历元处理
2. 数据结构从 rtklib-py 的 Obs/Eph/Nav → 适配本项目的 GnssMeasurement/EphemerisData
3. 新增 RTK 双差定位（参考 GREAT-MSF t_gcombDD + GINav relpos），RTD 为其码差分退化模式
4. 坐标变换函数独立提取
5. 引入 BaseModel → CombDD 单一分支（删除 CombIF/CombAll），仅支持双差组合观测模型

**详细指导**：见 [gnss.md](gnss.md)

### 9.2 src/core/ins/

**职责**：实现 INS 初始化与机械编排，支持增量式和速率式 IMU 数据，提供数据插值功能。

**关键实现**（已实现 + 预留）：
1. ✅ `InsInitializer`：INS 初始化，三种模式（静态 / 速度矢量 / 位置差分）+ 三阈值检验（详见 [初始化.md](file:///home/mxl/workplace/gipylib/skills/初始化.md)）
2. ✅ `interpolator.py`：IMU/GNSS 时间对齐插值（`is_to_update` / `imu_interpolate` / `find_bracket_imus`，参考 KF-GINS `imuInterpolate`）
3. ✅ `earth_param.py`：WGS84 地球参数、`ecef2llh` / `llh2ecef` / `cal_Ce2n` / `gravity_ecef`
4. ✅ `attitude.py`：姿态表示与转换（`euler2dcm` / `dcm2quat` / `att_caln2e` 等）
5. 🚧 `InsCore`：INS 核心，参考 GREAT-MSF t_gsins（姿态/速度/位置更新）— 待实现
6. 🚧 `ImuPreprocessor`：IMU 预处理（增量式↔速率式转换）— 待实现
7. 🚧 `ImuMechanizer`：机械编排主入口（驱动姿态/速度/位置递推 + Φ/Q 构造）— 待实现

**详细指导**：见 [imu.md](imu.md)

### 9.3 src/core/estimator/

**职责**：实现真正双滤波松组合 EKF（主滤波 P1 15/18 维 + NHC 子滤波 P2 5 维），含 NHC 约束（H1/H2 拆分），保留紧组合接口。采用 Integration 抽象体系。**时间同步与 IMU 插值逻辑由估计器内部处理**（参考 KF-GINS 增量切分方案，**时间对齐是重中之重**）。

**关键实现**：
1. Integration：组合导航集成基类，参考 GREAT-MSF t_gintegration
2. LcEstimator：松组合估计器（InsKf 子类，持有 P1/P2 双矩阵），参考 t_gsinskf + gnss_ins_lc_nhc navfilter.cc
3. 主滤波 F1/Q1 + NHC 子滤波 F2/Q2（参考 navmech.cc 中的 F 矩阵构造）
4. 松组合量测更新：GNSS 位置/速度（更新 P1）+ NHC（H1 更新 P1，H2 更新 P2）+ ZUPT（仅更新 P1）
5. 双滤波反馈机制：主滤波反馈（修正 δr^e/δv^e/δφ^e/δb_g/δb_a/δl_gnss）+ 子滤波反馈（修正 δθ_imu/δl_imu）
6. 紧组合预留接口：TcEstimator、TcIntegration
7. **时间同步与 IMU 插值**：估计器内部维护 imupre/imucur，GNSS 数据暂存为 pending_gnss（**deque 缓冲，避免丢失多个 GNSS 历元**），4 种时间对齐情况处理（参考 KF-GINS newImuProcess + imuInterpolate）

**详细指导**：见 [estimator.md](estimator.md)（特别是第 9 节 时间同步与 IMU 插值）

### 9.4 src/log/

**职责**：日志流和输出结果流。采用 WriterBase 抽象体系。

**关键实现**：
1. WriterBase：输出器抽象基类
2. 解算结果输出（多种格式）
3. 运行轨迹/调试输出
4. 原始数据记录（可选）

**详细指导**：见 [logANDoutput.md](logANDoutput.md)

---

## 10. 配置文件

```yaml
# config/default.yaml

# 运行模式
gnss_source: "internal"          # "internal" (内部GNSS解算) / "external" (外部GNSS结果文件)

# 时间系统（全框架内部统一使用 Unix 时间戳）
time_system: "unix"              # 内部统一 Unix 时间戳（与 rtklib-py gtime_t.time + gtime_t.sec 一致）
                                 # 输入端 formators.py 通过 gpst_to_unix(week, sow) 转换
                                 # 输出端 solution_writer.py / aligned_writer.py 通过 unix_to_gpst() 转回 (week, sow)

# 前端策略（仅前端策略切换，无后端策略）
strategy:
  frontend: "imu_mech"           # 前端里程计算法: "imu_mech" (IMU机械编排，唯一实现)

# GNSS 选项（内部模式生效）
gnss:
  elevation_mask: 15.0            # 最小仰角 (度)
  snr_mask: 30.0                  # 最小信噪比 (dB-Hz)
  spp_max_iter: 10                # SPP 最大迭代次数
  rtk_mode: "code"                # "code" (RTD码差分) / "carrier" (RTK载波差分)
  rtk_max_baseline: 20.0          # RTK 最大基线长度 (km)
  iono_correction: "broadcast"    # "broadcast" / "none"
  trop_correction: "saastamoinen" # "saastamoinen" / "none"

# 外部 GNSS 结果选项（外部模式生效）
gnss_external:
  solution_file: ""               # GNSS 定位结果文件路径
  solution_format: "pos"          # "pos" / "csv" / "nmea"
  pos_covariance_available: false  # 外部结果是否包含协方差信息
  default_pos_std: 1.0            # 外部结果默认位置标准差 (m)
  default_vel_std: 0.5            # 外部结果默认速度标准差 (m/s)

# IMU 选项
imu:
  data_format: "rate"             # "rate" (速率式) / "delta" (增量式)
  output_rate: 200                # IMU 输出频率 (Hz)
  sigma_g: 2.67e-4                # 陀螺白噪声 (rad/s/√Hz)
  sigma_a: 0.0112                 # 加计白噪声 (m/s²/√Hz)
  sigma_bg: 1.0e-3                # 陀螺零偏不稳定性 (rad/s)
  sigma_ba: 1.0e-2                # 加计零偏不稳定性 (m/s²)
  sigma_bg_p: 1.0e-5              # 陀螺零偏随机游走 (rad/s/√s)
  sigma_ba_p: 1.0e-4              # 加计零偏随机游走 (m/s²/√s)

# 插值选项（时间对齐，参考 KF-GINS imuInterpolate）
interpolation:
  method: "linear"                # "linear" / "poly"
  poly_order: 3                   # 多项式插值阶数（method="poly"时生效）
  # 增量切分方案：参考 KF-GINS，dt 会被原地修改，不弹出数据

# 估计器选项（双滤波架构）
estimator:
  # 双滤波状态维度（独立维护，无需手动设置总维度）
  # 主滤波 P1: 15维 (固定) + 3维 (可选 GNSS 杆臂) = 15 或 18 维
  # NHC 子滤波 P2: 5维 (安装角2 + IMU杆臂3)，独立矩阵
  estimate_imu_angle: true         # 启用 NHC 子滤波的安装角估计（2维：pitch, yaw）
  estimate_imu_leverarm: true      # 启用 NHC 子滤波的 IMU 杆臂估计（3维）
  estimate_gnss_leverarm: false    # 主滤波估计 GNSS 杆臂（3维，默认关闭）
  # 主滤波 P1 初始协方差
  init_std_pos: [10.0, 10.0, 10.0]         # 位置初始标准差 (m)
  init_std_vel: [1.0, 1.0, 1.0]            # 速度初始标准差 (m/s)
  init_std_att: [5.0, 5.0, 30.0]           # 姿态初始标准差 (度)
  init_std_gyro_bias: [0.1, 0.1, 0.1]      # 陀螺零偏初始标准差 (rad/s)
  init_std_accel_bias: [0.1, 0.1, 0.1]     # 加计零偏初始标准差 (m/s²)
  init_std_gnss_leverarm: [0.5, 0.5, 0.5]  # GNSS杆臂初始标准差 (m)
  # NHC 子滤波 P2 初始协方差
  init_std_imu_angle: [5.0, 5.0]           # IMU安装角初始标准差 (度, pitch/yaw)
  init_std_imu_leverarm: [0.5, 0.5, 0.5]   # IMU杆臂初始标准差 (m)
  # NHC（作用于 P1 + P2）
  nhc_enabled: true
  nhc_std: [0.1, 0.1]             # NHC 观测噪声标准差 (m/s)
  # ZUPT 零速更新（仅作用于 P1，与 NHC 互斥）
  zupt_enabled: true              # 是否启用 ZUPT
  zupt_std: [0.05, 0.05, 0.05]    # ZUPT 观测噪声标准差 (m/s)
  zupt_acc_threshold: 0.5         # 零速检测加速度阈值 (m/s²)
  zupt_gyro_threshold: 0.05       # 零速检测角速度阈值 (rad/s)
  zupt_min_static_window: 1.0     # 零速检测最小静止窗口 (s)
  # IMU安装角初始值（参考 gnss_ins_lc_nhc 的 initial_imu_angle）
  initial_imu_angle: [0.0, 0.0]   # IMU安装角初始值 (度, pitch/yaw)
  # IMU杆臂初始值（参考 gnss_ins_lc_nhc 的 initial_imu_leverarm）
  initial_imu_leverarm: [0.0, 0.0, 0.0]  # IMU杆臂初始值 (m, b系)
  # GNSS杆臂初始值
  initial_gnss_leverarm: [0.0, 0.0, 0.0]  # GNSS天线杆臂初始值 (m, b系)

# 日志选项
logging:
  output_dir: "output"
  solution_format: "pos"          # "pos" / "csv" / "nmea"
  trace_level: 1                  # 0=无, 1=基本, 2=详细, 3=调试
  log_raw_data: false
```

---

## 11. 扩展接口（紧组合预留）

### 11.1 紧组合量测数据

```python
@dataclass
class TcMeasurement:
    """紧组合量测数据（预留）"""
    timestamp: float
    gnss_obs: GnssMeasurement          # 原始 GNSS 观测值
    ephemeris: EphemerisData           # 星历数据
    reference: Optional[ReferenceMeasurement] = None  # 基站观测值（差分）
```

### 11.2 紧组合扩展点

| 模块 | 松组合实现 | 紧组合扩展 |
|------|-----------|-----------|
| `estimator/lc_estimator.py` | LcEstimator (InsKf子类，双滤波 P1+P2) | TcEstimator (InsKf子类) |
| `estimator/integration.py` | LcIntegration | TcIntegration |
| `core/gnss/comb_model.py` | CombDD 双差组合（单一分支） | 伪距/载波相位残差组合 |
| `core/gnss/spp.py` | SppProcessor 独立解算 | 改为伪距残差计算 |
| `core/gnss/rtk.py` | RtkProcessor | 紧组合模糊度估计 |
| `estimator/state_vector.py` | 双滤波独立索引 MainStateIndex / NhcSubStateIndex | 增加模糊度状态索引 |
| `integration/scheduler.py` | 仅转发到 estimate_queue（不做时间对齐） | 原始观测值对齐 |

---

## 12. 参考代码映射

### 12.1 GREAT-MSF → 本项目

| GREAT-MSF | 本项目 | 说明 |
|-----------|--------|------|
| `LibGREAT/gmodels/gbasemodel.h` | `core/gnss/base_model.py` | 观测模型抽象基类，抽象方法 `cmb_equ()` |
| `LibGREAT/gmodels/gcombmodel.h` + `gcombDD.h` | `core/gnss/comb_model.py` | 组合观测模型（仅保留 CombDD 单一分支，删除 CombIF/CombAll） |
| `LibGREAT/gins/gins.h` t_gsins | `core/imu/ins_core.py` InsCore | INS 核心（姿态/速度/位置更新、粗对准） |
| `LibGREAT/gins/gins.h` t_gsinskf | `core/imu/ins_kf.py` InsKf | INS 卡尔曼滤波基类（Ft/Hk/时间更新/量测更新/反馈） |
| `LibGREAT/gmsf/gintegration.h` | `core/estimator/integration.py` Integration | 组合导航集成基类（process_epoch/gnss_update） |
| `LibGREAT/gproc/gpvtflt.h` t_gpvtflt | `core/gnss/rtk.py` RtkProcessor | RTK 处理器（含 RTD 退化模式） |
| `LibGREAT/gproc/gspp.h` | `core/gnss/spp.py` SppProcessor | SPP 处理器 |
| `LibGnut/gmodels/ginterp.h` t_ginterp | `core/imu/interpolator.py` Interpolator | 插值基类（线性/多项式），用于 IMU/GNSS 时间对齐 |
| `LibGREAT/gins/gbase.h` t_gbase | `core/imu/attitude.py` | 姿态表示与转换（四元数/DCM/欧拉角） |
| `LibGREAT/gins/gearth.h` | `core/imu/earth_param.py` | 地球参数 |
| KF-GINS `GIEngine::newImuProcess` + `imuInterpolate` | `core/estimator/lc_estimator.py` 时间对齐逻辑 | 增量切分时间对齐方案，4 种时间对齐情况处理 |

### 12.2 gnss_ins_lc_nhc → 本项目

| gnss_ins_lc_nhc | 本项目 | 说明 |
|-----------------|--------|------|
| `src/filter/navfilter.cc` | `core/estimator/ekf.py` + `InsKf.time_update()` | EKF 预测+更新 |
| `src/imu/navmech.cc` | `core/imu/ins_core.py` + `InsKf.set_Ft()` | 机械编排+F矩阵 |
| `src/imu/navinitalized.cc` | `core/imu/ins_init.py` | INS 初始化 |
| `src/process/navstate.cc` | `core/estimator/lc_measurement.py` + `nhc.py` | 量测更新+NHC |
| `src/data/navgnss.cc` | `core/gnss/spp.py` + `core/gnss/rtk.py` | GNSS 数据处理 |
| `src/data/navimu.cc` | `core/imu/imu_preprocess.py` | IMU 数据预处理 |

### 12.3 rtklib-py → 本项目（已吸收）

> **重要变更**：rtklib-py 原为外部 `library/rtklib-py/`，已吸收为 `src/core/gnss/rtklib/` 子包。
> 通过 `config.py` 的 `_CfgProxy` 单例管理配置（由 `RtklibEnv.setup()` 调用 `config.set_params()` 注入），
> 不再修改 `sys.path` 或 `sys.modules`。子包内模块使用相对导入（如 `from .rtkcmn import ...`）。
> `library/rtklib-py/` 仅保留为参考代码，不再导入。

| rtklib-py 原位置 | 本项目吸收位置 | 调用方式 | 说明 |
|-----------|--------|------|------|
| `library/rtklib-py/src/pntpos.py` | `src/core/gnss/rtklib/pntpos.py` | `SppProcessor` 薄封装 `pntpos(obsr, nav)` | SPP 算法 |
| `library/rtklib-py/src/rtkpos.py` | `src/core/gnss/rtklib/rtkpos.py` | `RtkProcessor` 薄封装 `relpos(nav, obsr, obsb, sol)` | RTK 相对定位 |
| `library/rtklib-py/src/ephemeris.py` | `src/core/gnss/rtklib/ephemeris.py` | 直接调用 | 星历计算 |
| `library/rtklib-py/src/rtkcmn.py` | `src/core/gnss/rtklib/rtkcmn.py` | 直接调用 | 通用工具（坐标变换、时间、Sol 等） |
| `library/rtklib-py/src/mlambda.py` | `src/core/gnss/rtklib/mlambda.py` | `relpos` 内部调用 | MLAMBDA 模糊度解算 |
| `library/rtklib-py/src/rinex.py` | `src/core/gnss/rtklib/rinex.py` | `InternalGnssSensor` 调用 `rnx_decode` / `decode_obsfile` / `decode_nav` / `first_obs` / `next_obs` | RINEX 解码 |
| `library/rtklib-py/src/postpos.py` | `src/core/gnss/rtklib/postpos.py` | 后处理驱动（参考） | 批处理驱动 |
| `library/rtklib-py/src/config.py` | `src/core/gnss/rtklib/config.py` | `RtklibEnv.setup()` → `config.set_params()` | `_CfgProxy` 单例配置管理 |

**配置注入路径**：
`data/config.yaml` (gnss 段) → `RtklibEnv(gnss_cfg)` → `build_params()` → `config.set_params()` → `_CfgProxy` 单例 → `pntpos`/`relpos` 通过 `from .config import cfg` 读取

**Sol → GnssSolution 转换**（`src/core/gnss/solution_converter.py::sol_to_gnss_solution`）：
- `Sol.t.time + Sol.t.sec` → `GnssSolution.timestamp`（Unix 时间戳）
- `unix_to_gpst(timestamp)[0]` → `GnssSolution.week`（GPS 周号，派生字段）
- `Sol.rr[0:3]` → `GnssSolution.position`（ECEF 位置）
- `Sol.stat` → `GnssSolution.quality`（1=SPP, 2=RTD, 5=LC）
- `Sol.ns` → `GnssSolution.num_sv`（使用卫星数，由处理器回填）
- `sqrt(diag(Sol.qr[0:3,0:3]))` → `GnssSolution.sd`（ECEF 位置标准差）
- `Sol.qr[0:3, 0:3]` → `GnssSolution.cov`（ECEF 协方差矩阵，含非对角项）

### 12.4 GINav → 本项目

| GINav | 本项目 | 说明 |
|-------|--------|------|
| `src/gnss_ins_lc/gnss_ins_lc.m` | `core/estimator/integration.py` Integration | 松组合主循环 |
| `src/gnss_ins_lc/lc_filter.m` | `core/imu/ins_kf.py` InsKf.set_Ft() | 松组合滤波 |
| `src/gnss_ins_lc/lc_feedback.m` | `core/estimator/lc_feedback.py` | 反馈机制 |
| `src/gnss_ins_lc/udsol_lc.m` | `core/estimator/lc_measurement.py` | 状态更新 |
| `src/ins/ins_mech.m` | `core/imu/ins_core.py` InsCore | INS 机械编排 |
| `src/ins/ins_init.m` | `core/imu/ins_init.py` | INS 初始化 |
| `src/ins/ins_align.m` | `core/imu/ins_init.py` | INS 对准 |
| `src/gnss/spp/sppos.m` | `core/gnss/spp.py` | SPP 算法 |
| `src/gnss/relpos/relpos.m` | `core/gnss/rtk.py` RtkProcessor | RTK 相对定位 |
