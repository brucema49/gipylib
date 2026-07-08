# 融合估计指导方案

> 基于 ABC 类继承设计实现**真正双滤波**松组合 EKF：主滤波 15 维（E 系，可选 +3 维 GNSS 杆臂 = 18 维）+ NHC 子滤波 5 维（v 系，IMU 安装角 2 + IMU 杆臂 3），含 NHC 约束与 ZUPT 零速更新，保留紧组合扩展接口。
> 参考 GREAT-MSF 的 `t_gsinskf`（INS 卡尔曼滤波基类）和 `t_gintegration`（组合导航类）的继承模式。
> 参考 gnss_ins_lc_nhc 的双滤波架构：E 系主滤波 + v 系 NHC 子滤波。
>
> **时间系统约定**：全框架内部统一使用 **Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）**。
> 时间对齐、状态传播、量测更新等所有时间相关计算均基于 Unix 时间戳。
> 时间转换工具：`src/core/time_utils.py`（`gpst_to_unix` / `unix_to_gpst`，`GPST_EPOCH_UNIX = 315964800`）。
>
> **当前实现状态**：
> - ✅ 已实现：GNSS 解算部分（`SppProcessor` / `RtkProcessor`），可独立运行输出 `.pos` 文件
> - ✅ 已实现：外部 GNSS 结果对齐输出（`Aligner` + `AlignedWriter`，IMU 积攒 + GNSS 收割的匹配器）
> - ✅ 已实现：内部 GNSS + IMU 数据对齐管线（`internal + ins.enabled=on`，路径 C），实时 RTK/SPP 解算 + IMU 流式读取 → Aligner 匹配 → `aligned_internal_rtk.csv` 输出
> - ✅ 已实现：`src/core/ins/initializer.py::InsInitializer`（INS 初始化，三种模式 + 三阈值检验，详见 [初始化.md](file:///home/mxl/workplace/gipylib/skills/初始化.md)）
> - ✅ 已实现：`src/core/ins/` 下 `interpolator.py` / `earth_param.py` / `attitude.py`（初始化支撑模块，EKF 也可复用）
> - ✅ 已实现：SPP 多普勒测速（`pntpos.py::estvel` / `resdop`，速度填入 `sol.rr[3:6]`，用于 INS 动态初始化）
> - 🚧 预留：`InsKf` / `LcEstimator` / `LcIntegration` / NHC / ZUPT / 紧组合接口（下一阶段：INS 机械编排）
>
> **INS 机械编排的下一步**：
> 路径 C（数据对齐管线）已打通 `ImuSensor` → `imu_queue` 和 `InternalGnssSensor` → `gnss_queue` 的数据通路，
> `Logger` + `Aligner` 已实现 IMU 积攒 + GNSS 收割的时间匹配。
> 下一步是实现 `LcIntegration` 估计线程，从 `estimate_queue` 消费 `AlignedBlock`，
> 调用 `ImuMechStrategy.execute()` 做 E 系机械编排，再由 `InsKf.time_update()` / `meas_update()` 做双滤波 EKF。
> 路径 C 的 `AlignedBlock` 结构（`gnss: GnssSolution + imu_list: List[ImuMeasurement]`）可直接作为 `LcIntegration.process_epoch()` 的输入。
>
> **框架设计模式集成**（INS 启用后的设计）：
> - **策略模式（仅前端）**：前端里程计算法封装为 `ImuMechStrategy(OdometryStrategy)`，IMU 不可用时可降级为 GNSS 纯解算；后端融合算法直接由 `LcIntegration` 承担，**不再使用 FusionStrategy 层**
> - **纯队列流水线**：`LcIntegration` 作为估计线程主体，从 `estimate_queue` 取数据，**不继承 FusionObserver**，**无 `on_data()` 回调**，时间对齐与融合触发在主循环中执行
> - **依赖注入**：框架核心类通过构造函数接收具体传感器实例与 `OdometryStrategy` 实例，而非内部 new，便于测试与替换
> - **OOP 三大特性**：封装（双滤波状态分别封装在 P1/P2 矩阵中）、继承（InsKf → LcEstimator/TcEstimator）、多态（前端持有 `OdometryStrategy` 抽象引用）

---

## 目录

- [1. 概述](#1-概述)
- [2. 类继承体系](#2-类继承体系)
- [3. 状态向量定义](#3-状态向量定义)
- [4. EKF 算法流程](#4-ekf-算法流程)
- [5. 松组合量测模型](#5-松组合量测模型)
- [6. 反馈机制](#6-反馈机制)
- [7. 紧组合预留接口](#7-紧组合预留接口)
- [8. 估计线程主循环](#8-估计线程主循环)
- [9. 时间同步与 IMU 插值](#9-时间同步与-imu-插值)
- [10. 策略模式与观察者模式集成](#10-策略模式与观察者模式集成)

---

## 1. 概述

### 1.1 设计原则

本项目融合估计模块采用 **ABC（抽象基类）继承体系**，参考 GREAT-MSF 的类设计模式：

- **`t_gsinskf`** → 本项目 `InsKf(ABC)`：INS 卡尔曼滤波基类，定义时间更新、量测更新、反馈等抽象接口
- **`t_gintegration`**（多继承 `t_gsinskf` + `t_gpvtflt`）→ 本项目 `LcIntegration(Integration)`：组合导航类，融合 INS 滤波与 GNSS 处理能力，**作为估计线程主体从 `estimate_queue` 取数据**（不继承 FusionObserver）

核心设计思想：
1. **抽象基类定义接口**，子类实现具体算法
2. **松组合 / 紧组合共享 InsKf 基类**，仅在量测模型层面分化
3. **Integration 层组合 InsKf + GnssProcessor**，类似 `t_gintegration` 多继承 `t_gsinskf` + `t_gpvtflt`
4. **RTD 不是独立模块**，RTK 处理器同时覆盖 SPP / RTD / RTK 模式
5. **真正双滤波架构**：主滤波 P1（15/18 维 E 系）+ NHC 子滤波 P2（5 维 v 系），两套独立 P/F/H/Q/R 矩阵和反馈机制
6. **纯队列流水线驱动融合触发**：`LcIntegration` 从 `estimate_queue` 取数据，在主循环中执行时间对齐与融合，**无 `on_data()` 回调**
7. **依赖注入**：框架通过构造函数接收 `OdometryStrategy` 实例与 `list[BaseSensor]` 实例

### 1.2 松组合 vs 紧组合

| 维度 | 松组合 | 紧组合 |
|------|--------|--------|
| **GNSS 输入** | GNSS 解算结果（位置/速度） | 原始观测值（伪距/载波） |
| **量测模型** | 位置/速度差 | 伪距/载波残差 |
| **状态向量** | 真正双滤波：主滤波 P1（15/18 维）+ NHC 子滤波 P2（5 维） | 15+可选+模糊度 |
| **GNSS 可用性** | 需至少 4 颗卫星解算 | 1 颗卫星即可约束 |
| **实现复杂度** | 低 | 高 |
| **精度** | 受限于 GNSS 解算精度 | 更高（直接利用原始观测值） |

---

## 2. 类继承体系

### 2.1 继承关系图

```
═══════════════════════════════════════════════════════════════
  策略层（策略模式：仅前端里程计算法；后端融合由 LcIntegration 直接承担）
═══════════════════════════════════════════════════════════════
OdometryStrategy(ABC)             # 前端里程计算法策略基类
├── ImuMechStrategy               #   IMU 机械编排前端策略（详见 imu.md）
└── GnssPositioningStrategy       #   GNSS 纯解算前端策略（IMU 不可用降级，详见 gnss.md）
    ├── SppStrategy               #     SPP 单点定位
    └── RtkStrategy               #     RTK/RTD 差分定位

═══════════════════════════════════════════════════════════════
  INS 卡尔曼滤波层（ABC 继承 + 模板方法 + 双滤波分离）
═══════════════════════════════════════════════════════════════
                        ┌─────────────────────┐
                        │    InsKf(ABC)        │
                        │  参考 t_gsinskf      │
                        │                      │
                        │  抽象方法:            │
                        │   time_update()      │  (含 P1/P2 两套独立调用)
                        │   meas_update()      │  (H1 作用于 P1, H2 作用于 P2)
                        │   set_Ft()           │  (F1/F2 两套)
                        │   set_Hk()           │  (H1/H2 两套)
                        │   feedback()         │  (P1/P2 独立反馈)
                        │                      │
                        │  具体方法:            │
                        │   add_imu()          │
                        │   align_coarse()     │
                        │   align_pva()        │
                        │   align_vva()        │
                        └─────────┬───────────┘
                                  │
                    ┌─────────────┼─────────────┐
                    │                           │
                    ▼                           ▼
        ┌───────────────────┐       ┌───────────────────┐
        │  LcEstimator      │       │  TcEstimator      │
        │  (松组合估计器)    │       │  (紧组合估计器)    │
        │                   │       │  预留              │
        │  双滤波实现:      │       │                    │
        │   P1 (15/18 维)   │       │  重写:             │
        │   P2 (5 维 NHC)   │       │   time_update()   │
        │   time_update()   │       │   meas_update()   │
        │   meas_update()   │       │   set_Ft()        │
        │   set_Ft()        │       │   set_Hk()        │
        │   set_Hk()        │       │   feedback()      │
        │   feedback()      │       │                    │
        └───────────────────┘       └───────────────────┘

═══════════════════════════════════════════════════════════════
  组合导航集成层（ABC 继承 + 纯队列流水线 + 依赖注入）
═══════════════════════════════════════════════════════════════
                        ┌─────────────────────────────────┐
                        │  Integration(ABC)               │
                        │  参考 t_gintegration             │
                        │                                  │
                        │  抽象方法:                        │
                        │   process_epoch()                │  从 estimate_queue 取数据
                        │   _gnss_update()                 │
                        │   _get_meas()                    │
                        │   _time_align()                  │  时间对齐(增量切分)
                        │                                  │
                        │  持有策略引用（依赖注入）:         │
                        │   _frontend: OdometryStrategy    │  仅前端策略
                        │   _sensors: list[BaseSensor]     │
                        └─────────┬───────────────────────┘
                                  │
                    ┌─────────────┼─────────────┐
                    │                           │
                    ▼                           ▼
        ┌───────────────────────────┐       ┌───────────────────┐
        │  LcIntegration            │       │  TcIntegration    │
        │  (松组合导航)              │       │  (紧组合导航)      │
        │                           │       │  预留              │
        │  继承:                     │       │                    │
        │   Integration              │       │                    │
        │                           │       │                    │
        │  组合:                     │       │                    │
        │   InsKf 滤波能力(双滤波)   │       │                    │
        │     ├ P1 主滤波            │       │                    │
        │     └ P2 NHC 子滤波       │       │                    │
        │   GnssSolutionProvider     │       │                    │
        │   前端策略                  │       │                    │
        │                           │       │                    │
        │  主循环 process_epoch():   │       │                    │
        │   从 estimate_queue 取数据 │       │                    │
        │   IMU到达→前端编排+P1+P2预测│      │                    │
        │   GNSS到达→暂存pending_gnss│       │                    │
        │     (deque 缓冲)           │       │                    │
        │   时间对齐→双滤波量测更新   │       │                    │
        │   独立反馈 P1 + P2         │       │                    │
        └───────────────────────────┘       └───────────────────┘

═══════════════════════════════════════════════════════════════
  数据通路（纯队列流水线，不使用观察者模式）
═══════════════════════════════════════════════════════════════
estimate_queue (Queue[SensorData]) → LcIntegration.process_epoch()
                                    → 时间对齐 + 双滤波 EKF + 输出 Solution
                                    → solution_queue (Queue[Solution])
                                    → Logger

※ 传感器层仅作为 BaseSensor（无 Subject 角色），数据通过 queue.put() 流入
   estimate_queue，LcIntegration 通过 queue.get() 取数据，无回调。
```

### 2.2 InsKf(ABC) — INS 卡尔曼滤波基类

> 参考 GREAT-MSF `t_gsinskf`，定义 INS 卡尔曼滤波的标准接口。

```python
from abc import ABC, abstractmethod
import numpy as np

class InsKf(ABC):
    """INS 卡尔曼滤波基类

    参考 GREAT-MSF t_gsinskf，定义松组合/紧组合共享的
    INS 卡尔曼滤波接口。子类需实现具体的 F/H 构造和反馈逻辑。
    """

    # ===== 抽象方法（子类必须实现）=====

    @abstractmethod
    def time_update(self, dt: float, inflation: float = 1.0) -> None:
        """卡尔曼时间更新（预测）

        参考 t_gsinskf::time_update(kfts, inflation)
        - 构造状态转移矩阵 Φ = I + F*dt
        - 协方差预测: P = Φ*P*Φ^T + Q*dts
        - 误差状态预测值始终为 0

        ※ 真正双滤波：分别对 P1（15/18 维）和 P2（5 维）独立执行 time_update
        """

    @abstractmethod
    def meas_update(self) -> int:
        """卡尔曼量测更新

        参考 t_gsinskf::_meas_update()
        - 根据 Hk/Zk/Rk 执行卡尔曼更新
        - 支持序贯更新（GNSS 位置 + 速度 + NHC/ZUPT）
        - NHC 量测的 H 矩阵拆分为 H1（作用 P1）和 H2（作用 P2），分别执行 meas_update
        - ZUPT 仅作用于 P1（3D 速度约束）
        - 返回: 1 正常, -1 异常
        """

    @abstractmethod
    def set_Ft(self) -> None:
        """构造状态转移矩阵 F

        参考 t_gsinskf::set_Ft()
        - 主滤波 F1: 15×15 基础矩阵 + 可选 GNSS 杆臂扩展 = 15/18 维
        - NHC 子滤波 F2: 5×5，安装角/IMU 杆臂视为常量过程，F2 ≈ I（或带小量随机游走）
        - 紧组合: 扩展维度（含模糊度状态）
        """

    @abstractmethod
    def set_Hk(self) -> None:
        """构造量测矩阵 H

        参考 t_gsinskf::set_Hk() 和 gnss_ins_lc_nhc navstate.cc:343-353
        - 主滤波 H1: GNSS 位置/速度选择矩阵 + ZUPT 速度约束 + NHC 速度/姿态/陀螺零偏贡献
        - NHC 子滤波 H2: NHC 量测对安装角 δθ_imu / IMU 杆臂 δl_imu 的偏导
        - H1 作用于 P1（仅更新 P1 中速度/姿态/陀螺零偏部分）
        - H2 作用于 P2（仅更新 P2 中安装角/杆臂部分）
        - 紧组合: 伪距/载波残差观测矩阵
        """

    @abstractmethod
    def feedback(self) -> None:
        """反馈校正（全闭环，P1/P2 独立反馈）

        参考 t_gsinskf::feedback() 和 gnss_ins_lc_nhc navfilter.cc
        - P1 反馈: 位置/速度/姿态/陀螺零偏/加计零偏/GNSS 杆臂修正
        - P2 反馈: 安装角（四元数乘法，左乘小角度旋转）+ IMU 杆臂修正
        - 协方差矩阵对称性/正定性保证（P1、P2 分别处理）
        """

    # ===== 具体方法（基类实现）=====

    def add_imu(self, imu_data) -> None:
        """添加 IMU 数据

        参考 t_gsinskf::Add_IMU()
        - IMU 数据缓冲
        - 触发 INS 机械编排
        """

    def align_coarse(self, wm: np.ndarray, vm: np.ndarray) -> np.ndarray:
        """粗对准

        参考 t_gsinskf::align_coarse()
        - 解析法粗对准，需静态数据
        - 返回初始姿态
        """

    def align_pva(self, pos: np.ndarray) -> bool:
        """位置辅助对准

        参考 t_gsinskf::align_pva()
        - 利用已知位置辅助 INS 对准
        """

    def align_vva(self, vel: np.ndarray) -> bool:
        """速度辅助对准

        参考 t_gsinskf::align_vva()
        - 利用已知速度辅助 INS 对准
        """

    # ===== 滤波矩阵（参考 t_gsinskf 成员；真正双滤波，P1/P2 两套独立矩阵）=====
    # Ft1:  np.ndarray  — 主滤波状态转移矩阵 F1 (15×15 或 18×18)
    # Ft2:  np.ndarray  — NHC 子滤波状态转移矩阵 F2 (5×5)
    # Pk1:  np.ndarray  — 主滤波协方差矩阵 P1 (15×15 或 18×18)
    # Pk2:  np.ndarray  — NHC 子滤波协方差矩阵 P2 (5×5)
    # Hk1:  np.ndarray  — 主滤波量测矩阵 H1（GNSS/ZUPT/NHC 对 P1 的贡献）
    # Hk2:  np.ndarray  — NHC 子滤波量测矩阵 H2（NHC 对 P2 的贡献）
    # Rk:   np.ndarray  — 量测噪声矩阵（按量测类型构造）
    # Phik1: np.ndarray — 主滤波离散化状态转移矩阵
    # Phik2: np.ndarray — NHC 子滤波离散化状态转移矩阵
    # Xk1:  np.ndarray  — 主滤波状态向量 x1
    # Xk2:  np.ndarray  — NHC 子滤波状态向量 x2
    # Zk:   np.ndarray  — 量测向量
    # Qt1:  np.ndarray  — 主滤波过程噪声
    # Qt2:  np.ndarray  — NHC 子滤波过程噪声
```

### 2.3 LcEstimator(InsKf) — 松组合估计器

```python
class LcEstimator(InsKf):
    """松组合估计器（真正双滤波实现）

    实现 InsKf 的所有抽象方法，维护主滤波 P1 (E 系, 15/18 维)
    和 NHC 子滤波 P2 (v 系, 5 维) 两套独立矩阵和反馈逻辑。
    """

    def time_update(self, dt: float, inflation: float = 1.0) -> None:
        """松组合时间更新（P1 和 P2 独立执行）

        P1:
        - 调用 set_Ft() 构造 F1 (15×15 基础 + 可选 GNSS 杆臂 3 维)
        - Φ1 = I + F1*dt
        - P1 = Φ1*P1*Φ1^T + Q1*dt

        P2:
        - 调用 set_Ft() 构造 F2 (5×5，F2 ≈ I)
        - Φ2 = I + F2*dt
        - P2 = Φ2*P2*Φ2^T + Q2*dt
        """

    def meas_update(self) -> int:
        """松组合量测更新（序贯，P1/P2 协同但分别更新）

        执行顺序:
        1. GNSS 位置更新 (3 维，仅作用 P1)
        2. GNSS 速度更新 (3 维，仅作用 P1，可选)
        3. NHC 约束更新 (2 维):
           - H1 部分: K1 = P1*H1^T*(H1*P1*H1^T+R)^-1, 仅更新 P1 中速度/姿态/陀螺零偏
           - H2 部分: K2 = P2*H2^T*(H2*P2*H2^T+R_nhc)^-1, 仅更新 P2 中安装角/杆臂
        4. ZUPT 更新 (3 维速度约束，仅作用 P1；与 NHC 互斥，静止时启用)
        """

    def set_Ft(self) -> None:
        """构造双滤波状态转移矩阵 F1 (15/18 维) 和 F2 (5 维)

        F1: 参考 gnss_ins_lc_nhc navstate.cc，E 系误差传播
        F2: 安装角/杆臂视为常量过程，F2 ≈ I（或带小量随机游走）
        详见第 4 节 EKF 算法流程
        """

    def set_Hk(self) -> None:
        """构造双滤波量测矩阵 H1（作用于 P1）和 H2（作用于 P2）

        根据可用量测类型动态构造:
        - GNSS 位置 (仅 H1): H1[:, 0:3] = I, H1[:, GNSS_LEVER] = C_b^e
        - GNSS 速度 (仅 H1): H1[:, 3:6] = I, H1[:, 6:9] = 姿态项, H1[:, GNSS_LEVER] = 杆臂旋转项
        - NHC (H1+H2 拆分，参考 gnss_ins_lc_nhc navstate.cc:343-353):
          H1_vel  = R_b^v * C_e^b^T              (速度对 δv^e)
          H1_att  = +R_b^v * C_e^b^T * [v^e ×]  (速度对 δψ^e, ψ-error 正号)
          H1_gyro = R_b^v * [l_imu^b ×]          (速度对 δb_g)
          H2_angle = [v^v ×]_{:,2:3}              (速度对 δθ_imu，仅 2 列)
          H2_lever = R_b^v * [ω_eb^b ×]           (速度对 δl_imu)
        - ZUPT (仅 H1): H1[:, 3:6] = I（3D 速度约束，与 NHC 互斥）
        """

    def feedback(self) -> None:
        """双滤波独立反馈校正

        P1 反馈 (参考 gnss_ins_lc_nhc navfilter.cc:262-268):
        - 位置: r^e ← r^e - δr^e
        - 速度: v^e ← v^e - δv^e
        - 姿态: C_b^e ← (I + [δψ^e×]) * C_b^e  (ψ-error: 加号)
        - 零偏: b_g ← b_g - δb_g, b_a ← b_a - δb_a
        - GNSS 杆臂: l_gnss ← l_gnss - δl_gnss（可选）

        P2 反馈:
        - 安装角: q_imu ← δq_imu * q_imu（四元数左乘小角度旋转，仅 pitch/yaw）
        - IMU 杆臂: l_imu ← l_imu - δl_imu
        - 反馈结果更新 R_b^v 和 l_imu，供下次 NHC 量测构造使用
        """
```

### 2.4 TcEstimator(InsKf) — 紧组合估计器（预留）

```python
class TcEstimator(InsKf):
    """紧组合估计器（预留）

    状态向量扩展至 15+可选+模糊度维度，
    量测模型使用伪距/载波残差。
    """

    def time_update(self, dt: float, inflation: float = 1.0) -> None: ...
    def meas_update(self) -> int: ...
    def set_Ft(self) -> None: ...
    def set_Hk(self) -> None: ...
    def feedback(self) -> None: ...
```

### 2.5 Integration(ABC) — 组合导航基类

> 参考 GREAT-MSF `t_gintegration`，定义组合导航的标准接口。**纯队列流水线驱动**，无观察者模式。

```python
class Integration(ABC):
    """组合导航基类（纯队列流水线驱动）

    参考 GREAT-MSF t_gintegration，定义松组合/紧组合导航的
    历元处理接口。t_gintegration 多继承 t_gsinskf + t_gpvtflt，
    本项目中 Integration 持有 InsKf 和 GnssSolutionProvider 实例。
    GnssSolutionProvider 统一内部解算和外部结果文件两种 GNSS 数据源。

    本类**不继承 FusionObserver**，无 on_data() 回调；
    数据通过 estimate_queue 传入，由主循环 process_epoch() 处理。
    """

    @abstractmethod
    def process_epoch(self, epoch_data) -> int:
        """处理单个历元（从 estimate_queue 取数据后调用）

        参考 t_gintegration::_processEpoch()
        - 时间对齐（增量切分，参考 KF-GINS newImuProcess）
        - INS 机械编排 + 双滤波 EKF 预测（P1/P2 独立）
        - GNSS 量测更新（仅 P1）+ NHC 量测更新（H1 作用 P1 + H2 作用 P2）
        - 双滤波独立反馈
        - 输出结果到 solution_queue
        """

    @abstractmethod
    def _gnss_update(self) -> int:
        """GNSS 观测更新

        参考 t_gintegration::_GNSS_Update()
        - 松组合: 获取 GNSS 位置/速度解算结果（仅作用于主滤波 P1）
        - 紧组合: 构建伪距/载波残差观测方程
        """

    @abstractmethod
    def _get_meas(self) -> int:
        """获取量测数据

        参考 t_gintegration::_getMeas()
        - 判断量测类型（位置/速度/NHC/ZUPT/伪距/载波）
        - 量测可用性检验
        """

    @abstractmethod
    def _time_align(self) -> int:
        """时间对齐（增量切分）

        参考 KF-GINS newImuProcess() + imuInterpolate()
        - 维护 imupre/imucur 两个 IMU 历元
        - 维护 pending_gnss (deque 缓冲)
        - 处理 4 种时间对齐情况（_is_to_update 判断 0/1/2/3）
        - 详见第 9 节
        """
```

### 2.6 LcIntegration(Integration) — 松组合导航

```python
class LcIntegration(Integration):
    """松组合导航（真正双滤波 + 纯队列流水线）

    参考 t_gintegration（多继承 t_gsinskf + t_gpvtflt）的设计，
    本类组合 InsKf 滤波能力（双滤波 P1/P2）与 GnssSolutionProvider
    GNSS 结果提供能力。

    设计模式集成:
    - 单继承: Integration（融合框架）
    - 依赖注入: 构造函数接收 OdometryStrategy / BaseSensor 实例
    - **纯队列流水线**: 从 estimate_queue 取数据，无观察者模式，无 on_data() 回调
    - 后端融合算法（EKF + NHC + ZUPT）由本类直接承担，**不使用 FusionStrategy 层**

    GnssSolutionProvider 统一两种 GNSS 数据源:
    - GnssInternalProvider: 内部解算（SPP/RTD/RTK）
    - GnssExternalProvider: 外部结果文件（POS/NMEA/CSV）

    内部模式下 GnssProcessor 统一处理 SPP/RTD/RTK:
    - 无基站数据 → SPP 单点定位
    - 有基站数据 + 码差分 → RTD 模式
    - 有基站数据 + 载波差分 → RTK 模式
    RTD 不是独立模块，而是 RtkProcessor 的一种工作模式。

    外部模式下直接使用外部 GNSS 定位结果，无需 Rover/Ref/Eph 数据流。
    """

    def __init__(self, inskf: InsKf, gnss_provider: 'GnssSolutionProvider',
                 odometry_strategy: 'OdometryStrategy',
                 sensors: list['BaseSensor']):
        # ── 依赖注入：核心组件由外部传入，不在内部 new ──
        self._inskf = inskf                  # INS 卡尔曼滤波器（双滤波）
        self._gnss_provider = gnss_provider  # GNSS 结果提供者（统一内部/外部）
        self._odometry_strategy = odometry_strategy  # 前端里程计策略（IMU机械编排/GNSS纯解算）
        self._sensors = sensors                       # 传感器实例列表

        # ── 时间对齐状态（参考 KF-GINS）──
        self._imupre = None        # 上一 IMU 历元
        self._imucur = None        # 当前 IMU 历元
        self._pending_gnss = collections.deque()  # GNSS 缓冲（deque，避免丢失多个 GNSS 历元）

        # ── 无观察者注册：传感器仅持有 queue.Queue，数据通过 queue.put() 流入 estimate_queue ──

    def process_epoch(self, epoch_data) -> int:
        """松组合历元处理（主循环从 estimate_queue 取数据后调用）

        流程:
        1. 数据到达 → 时间对齐（增量切分）
           - IMU 数据 → 更新 imupre/imucur，按需切分到 GNSS 历元时刻
           - GNSS 数据 → 入 pending_gnss deque 缓冲
        2. IMU 时间对齐到 GNSS 历元时刻 → 前端策略机械编排 + 双滤波 EKF 预测（P1/P2 独立）
        3. GNSS 结果 → GnssSolutionProvider.get_solution()
           ├── 内部模式: GnssInternalProvider → GnssProcessor 解算（SPP/RTD/RTK）
           └── 外部模式: GnssExternalProvider → 直接读取外部结果
        4. GNSS 结果 → EKF 量测更新（仅作用 P1）
        5. NHC 约束 → 双滤波量测更新（H1 作用 P1 + H2 作用 P2）；或 ZUPT（仅 P1，与 NHC 互斥）
        6. 双滤波独立反馈校正（P1 + P2）
        7. 输出结果到 solution_queue
        """

    def _gnss_update(self) -> int:
        """松组合 GNSS 更新（仅作用于主滤波 P1）

        从 GnssSolutionProvider 获取 GNSS 结果:
        - GnssSolution.mode == "SPP" → 使用 SPP 位置/速度
        - GnssSolution.mode == "RTD" → 使用 RTD 位置/速度
        - GnssSolution.mode == "RTK" → 使用 RTK 固定解/浮点解
        - GnssSolution.mode == "External" → 使用外部结果

        根据解算模式自适应调整量测噪声 R:
        - SPP: 位置噪声 ~10m
        - RTD: 位置噪声 ~1m
        - RTK 固定解: 位置噪声 ~0.02m
        - RTK 浮点解: 位置噪声 ~0.5m
        - External: 使用配置的默认噪声或外部结果自带的协方差
        """

    def _get_meas(self) -> int:
        """获取松组合量测

        判断当前历元可用的量测类型:
        - GNSS 位置量测是否可用（仅 P1）
        - GNSS 速度量测是否可用（仅 P1）
        - NHC 约束是否可用（H1 作用 P1 + H2 作用 P2；运动时启用，与 ZUPT 互斥）
        - ZUPT 是否可用（仅 P1，静止时启用，与 NHC 互斥）
        """

    def _time_align(self) -> int:
        """时间对齐（增量切分，参考 KF-GINS newImuProcess + imuInterpolate）

        - 维护 imupre/imucur 两个 IMU 历元
        - 维护 pending_gnss (deque 缓冲)
        - 处理 4 种时间对齐情况（_is_to_update 判断 0/1/2/3）
        - 详见第 9 节
        """
```

### 2.7 TcIntegration(Integration) — 紧组合导航（预留）

```python
class TcIntegration(Integration):
    """紧组合导航（预留）

    参考 t_gintegration 的 TCI 模式，
    直接使用 GNSS 原始观测值进行融合。
    """

    def process_epoch(self, epoch_data) -> int: ...
    def _gnss_update(self) -> int: ...
    def _get_meas(self) -> int: ...
```

---

## 3. 状态向量定义

### 3.1 状态索引

> **真正双滤波架构**：主滤波与 NHC 子滤波**完全独立**，各自维护状态向量和索引。
> 参考 gnss_ins_lc_nhc 的双滤波设计：主滤波在 E 系，NHC 子滤波在 v 系。

```python
from enum import IntEnum

class MainStateIndex(IntEnum):
    """主滤波状态索引（E 系，P1 矩阵）

    参考 gnss_ins_lc_nhc 主滤波设计。
    状态向量 x1（固定 15 维 + 可选 3 维 GNSS 杆臂 = 15 或 18 维）:

      x1 = [δr^e, δv^e, δψ^e, δb_g, δb_a, (δl_gnss)]^T

    主滤波独立维护 P1/F1/H1/Q1/R1 矩阵和反馈机制。
    """
    # 位置误差 (3, E 系)
    POS_X = 0       # ECEF X 位置误差 (m)
    POS_Y = 1       # ECEF Y 位置误差 (m)
    POS_Z = 2       # ECEF Z 位置误差 (m)

    # 速度误差 (3, E 系)
    VEL_X = 3       # ECEF X 速度误差 (m/s)
    VEL_Y = 4       # ECEF Y 速度误差 (m/s)
    VEL_Z = 5       # ECEF Z 速度误差 (m/s)

    # 姿态误差角 (3, E 系)
    ATT_X = 6       # ECEF X 姿态误差 (rad)
    ATT_Y = 7       # ECEF Y 姿态误差 (rad)
    ATT_Z = 8       # ECEF Z 姿态误差 (rad)

    # 陀螺零偏 (3)
    GYRO_BX = 9     # X轴陀螺零偏 (rad/s)
    GYRO_BY = 10    # Y轴陀螺零偏 (rad/s)
    GYRO_BZ = 11    # Z轴陀螺零偏 (rad/s)

    # 加计零偏 (3)
    ACCEL_BX = 12   # X轴加计零偏 (m/s²)
    ACCEL_BY = 13   # Y轴加计零偏 (m/s²)
    ACCEL_BZ = 14   # Z轴加计零偏 (m/s²)

    # ===== 主滤波可选扩展状态 =====
    # GNSS杆臂 (3维, estimate_gnss_leverarm=true 时, b 系，E系下估计)
    GNSS_LEVER_X = 15     # GNSS天线杆臂 X (m, b系)
    GNSS_LEVER_Y = 16     # GNSS天线杆臂 Y (m, b系)
    GNSS_LEVER_Z = 17     # GNSS天线杆臂 Z (m, b系)

# 主滤波状态维度
# P1_DIM = 15 (默认) 或 18 (estimate_gnss_leverarm=true)

# 主滤波常用切片
POS_SLICE = slice(0, 3)
VEL_SLICE = slice(3, 6)
ATT_SLICE = slice(6, 9)
GYRO_B_SLICE = slice(9, 12)
ACCEL_B_SLICE = slice(12, 15)
GNSS_LEVER_SLICE = slice(15, 18)   # estimate_gnss_leverarm=true


class NhcSubStateIndex(IntEnum):
    """NHC 子滤波状态索引（v 系，P2 矩阵）

    参考 gnss_ins_lc_nhc 子滤波设计。
    状态向量 x2（固定 5 维）:

      x2 = [δθ_imu(2), δl_imu(3)]^T

    子滤波独立维护 P2/F2/H2/Q2/R2 矩阵和反馈机制。
    安装角只有 2 维（pitch, yaw），roll 假设为 0（参考 gnss_ins_lc_nhc）。
    """
    # IMU安装角 (2维, estimate_imu_angle=true 时, v 系)
    IMU_ANGLE_PITCH = 0  # IMU安装角 pitch (rad)
    IMU_ANGLE_YAW   = 1  # IMU安装角 yaw (rad)
    # 注意: 安装角只有2维(pitch, yaw)，roll假设为0
    # 参考 gnss_ins_lc_nhc: imu_angle_ 为 Vector2d

    # IMU杆臂 (3维, estimate_imu_leverarm=true 时, b 系)
    IMU_LEVER_X = 2      # IMU杆臂 X (m, b系)
    IMU_LEVER_Y = 3      # IMU杆臂 Y (m, b系)
    IMU_LEVER_Z = 4      # IMU杆臂 Z (m, b系)

# NHC 子滤波状态维度
# P2_DIM = 5 (默认启用 estimate_imu_angle 和 estimate_imu_leverarm)
# 当 estimate_imu_angle=false 且 estimate_imu_leverarm=false 时，NHC 子滤波不运行

# NHC 子滤波切片
IMU_ANGLE_SLICE = slice(0, 2)
IMU_LEVER_SLICE = slice(2, 5)

# 配置开关:
# - estimate_imu_angle: true → 启用 NHC 子滤波的安装角估计（2维 pitch, yaw）
# - estimate_imu_leverarm: true → 启用 NHC 子滤波的 IMU 杆臂估计（3维）
# - estimate_gnss_leverarm: false → 主滤波估计 GNSS 杆臂（3维，默认关闭）
# - 当 estimate_imu_angle=false 且 estimate_imu_leverarm=false 时，NHC 子滤波不运行，
#   NHC 直接使用固定安装角和杆臂
# - 不包含时间同步参数：通过 KF-GINS 增量切分方案精确对齐 IMU/GNSS 时间
```

### 3.2 误差状态定义

```
主滤波（P1，E 系）:
  δr^e    : E 系下位置误差 (m)
  δv^e    : E 系下速度误差 (m/s)
  δψ^e    : E 系下姿态误差角 (rad)，ψ 系 (对齐 ignav)
  δb_g    : 陀螺零偏误差 (rad/s)
  δb_a    : 加计零偏误差 (m/s²)
  δl_gnss : GNSS天线杆臂误差 [δlx, δly, δlz] (m, b系)（可选）

NHC 子滤波（P2，v 系）:
  δθ_imu  : IMU安装角误差 [δpitch, δyaw] (rad)
  δl_imu  : IMU杆臂误差 [δlx, δly, δlz] (m, b系)
```

**姿态误差定义（E 系 ψ 系, 对齐 ignav）**：

```
C_b^e_true = (I + [δψ^e×]) * C_b^e_est
```

**双滤波架构说明（参考 gnss_ins_lc_nhc, F 矩阵对齐 ignav ψ-error）**：

```
主滤波（P1，E 系，15/18 维）:
  - 固定 15 状态: δr^e, δv^e, δψ^e, δb_g, δb_a
  - 可选扩展: GNSS 杆臂 δl_gnss(3)（b系表示，E系下估计，estimate_gnss_leverarm=true）
  - 用于: INS 机械编排预测、GNSS 位置/速度量测更新、ZUPT 量测更新
  - F1/H1/Q1/R1 矩阵在 E 系下构造（参考 gnss_ins_lc_nhc navmech.cc / navstate.cc）
  - 独立维护 P1 协方差矩阵，独立反馈

NHC 子滤波（P2，v 系，5 维）:
  - 状态: δθ_imu(2) + δl_imu(3)
  - F2 ≈ I（安装角/杆臂视为常量过程，或带小量随机游走）
  - H2 在 v 系下构造（参考 gnss_ins_lc_nhc navstate.cc:343-353）
  - 独立维护 P2 协方差矩阵，独立反馈
  - 共享主滤波的 INS 状态（v^e, C_b^e, ω_ib^b）用于 NHC 量测构造
  - 不修改主滤波状态，反馈结果（R_b^v, l_imu）仅用于下次 NHC 量测构造
```

---

## 4. EKF 算法流程

> **真正双滤波**：主滤波 P1 和 NHC 子滤波 P2 各自独立执行 time_update / meas_update / feedback。
> 主滤波在 E 系（15/18 维），NHC 子滤波在 v 系（5 维），F/H/P/Q/R 两套独立。

### 4.1 预测步骤（time_update）

```
输入: 当前 INS 状态, IMU 原始观测 (ω_ib_b, f_b), 协方差 P1/P2, 时间步长 dt

1. IMU 误差补偿
   ω_ib_b_corr = ω_ib_b - b_g
   f_b_corr    = f_b - b_a

2. INS 机械编排
   参考 imu.md，更新位置/速度/姿态

3. 主滤波构造状态转移矩阵 F1 (set_Ft)
   F1 为 15×15 或 18×18 矩阵（E 系，可选 GNSS 杆臂扩展），E 系下分块结构:

   F1 = [F_rr  F_rv  0     0     0     0    ]   位置误差方程(E系)
       [F_vr  F_vv  F_vψ  F_vb  F_va  0    ]   速度误差方程(E系, ψ-error)
       [0     0     F_ψψ  F_ψb  0     0    ]   姿态误差方程(E系, ψ-error)
       [0     0     0     F_bb  0     0    ]   陀螺零偏方程
       [0     0     0     0     F_aa  0    ]   加计零偏方程
       [0     0     0     0     0     0    ]   GNSS杆臂方程（常数，可选）

   GNSS杆臂假设为常数，F1 中对应行为 0。

   E 系下 F1 矩阵关键子块 (ψ-error 模型, 对齐 ignav getF):
   - F_rv: 位置对速度的偏导 (E系下 = I_3)
   - F_vr: 速度对位置的偏导 (重力梯度, -2/(re·|pos|)·ge⊗pos, 参考 ignav getF)
   - F_vv: 速度对速度的偏导 (-[2ω_ie^e ×])，E系下地球自转为常数
   - F_vψ: 速度对姿态的偏导 (-[f^e ×])，ψ-error 负号 (对齐 ignav)
   - F_vba: 速度对加计零偏的偏导 (+C_b^e)
   - F_ψψ: 姿态对姿态的偏导 (-[ω_ie^e ×])，E系下无ω_en^n项
   - F_ψbg: 姿态对陀螺零偏的偏导 (+C_b^e)，ψ-error 正号 (对齐 ignav)
   - F_bgbg: 陀螺零偏一阶马尔可夫 (-I/τ_g)
   - F_baba: 加计零偏一阶马尔可夫 (-I/τ_a)

   ψ-error vs φ-error 差异 (本项目从 φ-error 切换到 ψ-error):
   - F_vψ = -[f^e×]  (φ-error 为 +[f^e×])
   - F_ψbg = +C_b^e  (φ-error 为 -C_b^e)
   - 姿态反馈符号相反: C_b^e ← (I + [δψ^e×])·C_b^e  (φ-error 为减号)

   E 系 vs n 系 F1 矩阵差异:
   - E 系无需计算导航系旋转角速度 ω_en^n（不存在）
   - E 系地球自转 ω_ie^e = [0,0,ω_e] 为常数
   - E 系重力 g^e 为位置函数（非常数）
   - E 系 F_rv = I_3（位置对速度的偏导为单位阵）

4. NHC 子滤波构造状态转移矩阵 F2 (set_Ft)
   F2 为 5×5 矩阵（v 系），安装角/IMU杆臂视为常量过程:
   F2 ≈ I (或带小量随机游走)
   F2 = diag([0, 0, 0, 0, 0])  或  diag([1/τ_angle, 1/τ_angle, 1/τ_lever, 1/τ_lever, 1/τ_lever])

5. 离散化（双滤波分别处理, 自适应精度, 对齐 ignav precPhi）
   Φ1 自适应精度 (基于 dt):
     - dt ≤ 0.005s  (≥200Hz): 一阶 Φ = I + F·dt
     - dt ≤ 0.01s   (100-200Hz): 二阶 Φ = I + F·dt + 0.5·(F·dt)²
     - dt > 0.01s   (<100Hz): 矩阵指数 Φ = expm(F·dt) (自实现 _expm, 不依赖 scipy)
   Φ2 = I + F2 * dt     (5×5, NHC 子滤波用一阶)
   Q_d1 = Q1 * dt
   Q_d2 = Q2 * dt

6. 协方差预测（双滤波独立, GINav 中间值法）
   P1 = Φ1·(P1 + 0.5·Q_d1)·Φ1^T + 0.5·Q_d1
   P2 = Φ2·(P2 + 0.5·Q_d2)·Φ2^T + 0.5·Q_d2
   P1 = 0.5 * (P1 + P1^T)  (确保对称性)
   P2 = 0.5 * (P2 + P2^T)

7. 误差状态预测值始终为 0
   x1_pred = 0
   x2_pred = 0
```

### 4.2 更新步骤（meas_update）

> NHC 量测的 H 矩阵拆分为 H1（作用于 P1）和 H2（作用于 P2），分别对 P1、P2 执行 meas_update；
> GNSS 位置/速度、ZUPT 仅作用于 P1。

```
输入: 预测协方差 P1/P2, 量测向量 Z, 量测矩阵 H1/H2, 量测噪声 R

主滤波 P1 更新（GNSS 位置/速度、NHC 的 H1 部分、ZUPT）:
1. 新息协方差  S1 = H1 * P1 * H1^T + R
2. 卡尔曼增益  K1 = P1 * H1^T * S1^{-1}
3. 状态更新    x1 = x1 + K1 * (Z - H1 * x1)
4. 协方差更新   I_K1H1 = I - K1 * H1
               P1 = I_K1H1 * P1 * I_K1H1^T + K1 * R * K1^T
               P1 = 0.5 * (P1 + P1^T)

NHC 子滤波 P2 更新（NHC 的 H2 部分）:
1. 新息协方差  S2 = H2 * P2 * H2^T + R_nhc
2. 卡尔曼增益  K2 = P2 * H2^T * S2^{-1}
3. 状态更新    x2 = x2 + K2 * (Z - H2 * x2)
4. 协方差更新   I_K2H2 = I - K2 * H2
               P2 = I_K2H2 * P2 * I_K2H2^T + K2 * R_nhc * K2^T
               P2 = 0.5 * (P2 + P2^T)

序贯更新（多个量测依次执行，P1 和 P2 各自序贯）
```

---

## 5. 松组合量测模型

### 5.1 量测更新类型

| 量测类型 | 观测向量 Z | 观测矩阵 H | 观测噪声 R | 维度 | 作用目标 |
|----------|-----------|-----------|-----------|------|---------|
| **GNSS 位置** | INS位置 - GNSS位置 + 杆臂投影 | H1（位置选择 + 杆臂项） | GNSS位置协方差 | 3 | 仅 P1 |
| **GNSS 速度** | INS速度 - GNSS速度 + 杆臂旋转补偿 | H1（速度选择 + 姿态/杆臂项） | GNSS速度协方差 | 3 | 仅 P1 |
| **NHC** | 车体坐标系(v系)侧向/垂向速度 | H1（速度/姿态/陀螺零偏贡献）+ H2（安装角/杆臂贡献） | NHC噪声 | 2 | P1（H1）+ P2（H2） |
| **ZUPT** | 3D 速度约束（v=0） | H1（速度选择） | ZUPT噪声 | 3 | 仅 P1（与 NHC 互斥，静止时启用） |

### 5.2 GNSS 位置量测模型（E 系下）

```
量测方程（E 系下，参考 gnss_ins_lc_nhc navstate.cc）:
  Z_pos = r_ins^e - r_gnss^e + C_b^e * l_gnss^b

其中:
  r_ins^e  : INS 推算位置 (ECEF)
  r_gnss^e : GNSS 解算位置 (ECEF)
  C_b^e * l_gnss^b : GNSS 天线杆臂在 E 系投影

观测矩阵 H_pos [3, N]:
  H[:, 0:3]          = I_3          (位置对位置误差，E系)
  H[:, GNSS_LEVER]   = C_b^e        (位置对GNSS杆臂误差，estimate_gnss_leverarm=true)

观测噪声 R_pos [3, 3]:
  根据解算模式自适应:
  - SPP:   σ = 10m  → R = diag(σ²)
  - RTD:   σ = 1m   → R = diag(σ²)
  - RTK固定: σ = 0.02m → R = diag(σ²)
  - RTK浮点: σ = 0.5m  → R = diag(σ²)
  E 系下可直接使用 GNSS 解算提供的 ECEF 协方差，无需坐标转换
```

### 5.3 GNSS 速度量测模型（E 系下）

```
量测方程（E 系下）:
  Z_vel = v_ins^e - v_gnss^e + [ω_eb^e ×] * C_b^e * l_gnss^b

其中:
  v_ins^e  : INS 推算速度 (ECEF)
  v_gnss^e : GNSS 速度 (ECEF)
  [ω_eb^e ×] * C_b^e * l_gnss^b : 杆臂旋转补偿

观测矩阵 H_vel [3, N]:
  H[:, 3:6]          = I_3                    (速度对速度误差，E系)
  H[:, 6:9]          = -[C_b^e * l_gnss^b ×]  (速度对姿态误差)
  H[:, GNSS_LEVER]   = [ω_eb^e ×] * C_b^e     (速度对GNSS杆臂误差，estimate_gnss_leverarm=true)

观测噪声 R_vel [3, 3]:
  默认 σ = 0.5 m/s，或使用 GNSS 解算提供的速度协方差
```

### 5.4 NHC 约束量测模型（v 系下，双滤波 H 矩阵拆分）

> **真正双滤波下的 H 矩阵分离**（参考 gnss_ins_lc_nhc navstate.cc:343-353）：
> NHC 量测同时依赖主滤波状态（δv^e、δψ^e、δb_g）和 NHC 子滤波状态（δθ_imu、δl_imu）。
> 为保持两套 P 矩阵独立，将 H 矩阵拆分为 H1（作用于 P1）和 H2（作用于 P2）两部分。

```
原理（参考 gnss_ins_lc_nhc odo.md 和 navstate.cc:343-353）:
  地面车辆车体坐标系(v系)侧向和垂向速度为零:
  v^v = R_b^v * C_e^b * v^e + R_b^v * [ω_eb^b ×] * l_imu^b
  约束: v_right_v = 0, v_down_v = 0

  其中:
  - R_b^v 为安装角旋转矩阵（IMU b系 → 车体 v系），由 NHC 子滤波估计后更新
  - C_e^b 为姿态旋转矩阵（ECEF → b系），由主滤波维护
  - ω_eb^b 为 b系下 IMU 角速度
  - l_imu^b 为 IMU 杆臂（b系），由 NHC 子滤波估计后更新

量测方程:
  Z_nhc = [v_right_v; v_down_v] = [0; 0]

H 矩阵拆分（参考 gnss_ins_lc_nhc navstate.cc:343-353）:

主滤波贡献 H1（作用于 P1，仅更新 P1 中速度/姿态/陀螺零偏部分）:
  H1_vel   = R_b^v * C_e^b^T              (速度对速度误差 δv^e)
  H1_att   = +R_b^v * C_e^b^T * [v^e ×]  (速度对姿态误差 δψ^e, ψ-error 正号)
  H1_gyro  = R_b^v * [l_imu^b ×]          (速度对陀螺零偏 δb_g)
  对应 MainStateIndex.VEL_X..VEL_Z / ATT_X..ATT_Z / GYRO_BX..GYRO_BZ

NHC 子滤波贡献 H2（作用于 P2，仅更新 P2 中安装角/杆臂部分）:
  H2_angle = [v^v ×]_{:,2:3}              (速度对安装角误差 δθ_imu，仅 2 列)
            参考 gnss_ins_lc_nhc: H_angle = [Vvv×]_{:,2:3}
  H2_lever = R_b^v * [ω_eb^b ×]           (速度对 IMU 杆臂误差 δl_imu)
            参考 gnss_ins_lc_nhc: H_lever = Rbv*[webb×]
  对应 NhcSubStateIndex.IMU_ANGLE_PITCH..IMU_ANGLE_YAW / IMU_LEVER_X..IMU_LEVER_Z

实施方式:
  - 主滤波量测更新: K1 = P1 * H1^T * (H1*P1*H1^T + R_nhc)^-1, 仅更新主滤波状态 x1
  - NHC 子滤波量测更新: K2 = P2 * H2^T * (H2*P2*H2^T + R_nhc)^-1, 仅更新子滤波状态 x2
  - R 矩阵需根据 H1、H2 分别构造对应的量测噪声（同一 NHC 量测可使用相同 R_nhc）

观测噪声 R_nhc [2, 2]:
  σ = [0.1, 0.1] m/s

NHC 子滤波特点:
  - 安装角和 IMU 杆臂只在 NHC 子滤波中估计，不在主滤波的状态中
  - H1/H2 在 v 系下构造，需要安装角旋转矩阵 R_b^v
  - 主滤波的 δv^e/δψ^e/δb_g 通过 C_e^b 转换到 v 系参与 NHC 更新
  - NHC 子滤波的反馈结果（R_b^v, l_imu）仅用于下次 NHC 量测构造，不修改主滤波状态

可用性判断:
  1. 配置中启用 NHC（estimate_imu_angle=true 或 estimate_imu_leverarm=true）
  2. 水平速度 > 阈值（避免零速时噪声放大）
  3. 非剧烈转弯状态
  4. 与 ZUPT 互斥：静止时使用 ZUPT（仅 P1），运动时使用 NHC（P1+P2）
```

### 5.5 RTK 解算结果（含 RTD 模式）在松组合中的使用

```
GnssProcessor 统一处理 SPP/RTD/RTK:

┌─────────────────────────────────────────────────────────┐
│                   GnssProcessor                         │
│                                                         │
│  输入: GnssMeasurement + ReferenceMeasurement(可选)     │
│                                                         │
│  ┌───────────────────────────────────────────────────┐  │
│  │ 无基站数据 → SPP 模式                              │  │
│  │   pntpos() 单点定位                                │  │
│  │   位置精度 ~10m                                    │  │
│  ├───────────────────────────────────────────────────┤  │
│  │ 有基站数据 → RTK 处理器                            │  │
│  │   ├─ 码差分 (RTD 模式)                            │  │
│  │   │   伪距双差定位                                 │  │
│  │   │   位置精度 ~1m                                 │  │
│  │   └─ 载波差分 (RTK 模式)                          │  │
│  │       双差 + 模糊度固定                            │  │
│  │       固定解精度 ~0.02m, 浮点解精度 ~0.5m         │  │
│  └───────────────────────────────────────────────────┘  │
│                                                         │
│  输出: GnssSolution (mode, position, velocity, cov)     │
└─────────────────────────────────────────────────────────┘

松组合根据 GnssSolution.mode 自适应:
- mode == "SPP"  → 使用 SPP 结果，R 按粗精度设置
- mode == "RTD"  → 使用 RTD 结果，R 按中等精度设置
- mode == "RTK"  → 使用 RTK 结果，R 根据 fixed/float 设置
```

### 5.6 时间对齐策略

```
本项目不估计时间同步参数，而是通过增量切分精确对齐 IMU/GNSS 时间
（参考 KF-GINS newImuProcess + imuInterpolate）:

※ 所有 timestamp 均为 Unix 时间戳（float 秒，与 rtklib-py gtime_t 一致）

1. Estimator 内部维护 imupre/imucur 两个 IMU 历元
2. GNSS 历元到达时，进入 pending_gnss (deque 缓冲，避免丢失多个 GNSS 历元)
3. 当 imupre.timestamp ≤ gnss.timestamp ≤ imucur.timestamp 时，按 4 种时间对齐情况处理:
   - 情况 0: imupre.timestamp == imucur.timestamp == gnss.timestamp  → 直接 GNSS 量测更新
   - 情况 1: imupre.timestamp < imucur.timestamp == gnss.timestamp   → 直接 GNSS 量测更新（imucur 即对齐）
   - 情况 2: imupre.timestamp == gnss.timestamp < imucur.timestamp   → 直接 GNSS 量测更新（imupre 即对齐）
   - 情况 3: imupre.timestamp < gnss.timestamp < imucur.timestamp    → 增量切分，修改 dt 后重新机械编排
4. 时间对齐由 Estimator 内部处理，Scheduler 仅转发
5. 详见第 9 节
```

---

## 6. 反馈机制

> **真正双滤波独立反馈**：主滤波 P1 和 NHC 子滤波 P2 各自独立执行反馈。

### 6.1 反馈原理

EKF 更新后得到误差状态 δx1（主滤波）和 δx2（NHC 子滤波），各自独立反馈修正（全闭环校正）：

```
主滤波 P1 反馈:
位置反馈:   r^e ← r^e - δr^e
速度反馈:   v^e ← v^e - δv^e
姿态反馈:   C_b^e ← (I + [δψ^e×]) * C_b^e  → 正交化  (ψ-error: 加号, 与 φ-error 减号相反)
陀螺零偏:   b_g ← b_g - δb_g
加计零偏:   b_a ← b_a - δb_a
GNSS杆臂反馈: l_gnss ← l_gnss - δl_gnss（可选，estimate_gnss_leverarm=true）

NHC 子滤波 P2 反馈:
安装角反馈: q_imu ← δq_imu * q_imu（四元数左乘小角度旋转，仅 pitch/yaw）
            参考 gnss_ins_lc_nhc: 安装角通过四元数乘法修正
            反馈后更新 R_b^v 矩阵
IMU杆臂反馈: l_imu ← l_imu - δl_imu
            反馈后更新 l_imu^b，供下次 NHC 量测构造使用

注意:
  - P2 反馈结果仅影响 NHC 量测构造（R_b^v, l_imu），不修改主滤波 P1 的状态
  - P1 反馈结果影响 INS 机械编排和下次预测
```

### 6.2 协方差反馈保证

```
反馈后协方差矩阵处理（P1、P2 各自处理）:
1. 确保对称性: P1 = 0.5 * (P1 + P1^T),  P2 = 0.5 * (P2 + P2^T)
2. 确保正定性: 特征值裁剪 (最小特征值 ≥ 1e-12)
3. Joseph form 更新已在 EKF update 中完成
```

---

## 7. 紧组合预留接口

### 7.1 紧组合与松组合的差异

| 维度 | 松组合 (LcEstimator) | 紧组合 (TcEstimator) |
|------|---------------------|---------------------|
| **量测输入** | GnssSolution (位置/速度) | GnssMeasurement (原始观测值) |
| **量测模型** | 位置/速度差 | 伪距/载波残差 |
| **状态维度** | 15+可选(默认20,全配置23) | 15+可选+模糊度 |
| **F 矩阵** | N×N (N=15/17/18/20/23) | 扩展维度 |
| **H 矩阵** | 位置/速度选择矩阵 | 伪距/载波观测矩阵 |
| **反馈** | 仅 INS 状态 | INS 状态 + GNSS 参数（模糊度等） |

### 7.2 紧组合预留接口描述

```
TcEstimator 需额外实现的接口:

1. set_Ft() 扩展
   - 状态向量扩展至 15+可选+模糊度
   - F 矩阵增加模糊度状态行/列
   - 模糊度状态转移: 一阶高斯-马尔可夫或常数模型

2. set_Hk() 扩展
   - 伪距残差观测方程:
     Z_pr = ρ_ins - ρ_gnss + 对流层/电离层改正
     H_pr: 位置偏导 + 钟差偏导 + 模糊度偏导
   - 载波相位残差观测方程:
     Z_cp = Φ_ins - Φ_gnss + 改正
     H_cp: 类似伪距，但模糊度系数为波长

3. meas_update() 扩展
   - 参考 t_gintegration::_GNSS_Update() 的 TCI 分支
   - 需构建双差观测方程
   - 模糊度固定（LAMBDA 方法）

4. feedback() 扩展
   - 参考 t_gintegration::_gnss_feedback()
   - 除 INS 状态反馈外，还需反馈 GNSS 参数
   - 模糊度固定后需更新协方差矩阵

5. _merge_pose() / _merge_init()
   - 参考 t_gintegration::_merge_pose()
   - TC 模式下 INS 与 GNSS 滤波器的状态融合
   - 初始协方差矩阵合并
```

---

## 8. 估计线程主循环

### 8.1 完整处理流程

```
LcIntegration.process_epoch(epoch_data)   ← 从 estimate_queue 取数据
│
├── 0. 时间对齐（增量切分，参考 KF-GINS newImuProcess）
│   ├── IMU 数据 → 更新 imupre/imucur
│   ├── GNSS 数据 → 入 pending_gnss (deque 缓冲)
│   └── _is_to_update() 判断 4 种情况（0/1/2/3）
│
├── 1. IMU 数据处理（双滤波独立 time_update）
│   ├── add_imu(imu_data)           # IMU 数据缓冲，更新 imupre/imucur
│   ├── INS 机械编排               # 位置/速度/姿态更新（E 系，参考 imu.md）
│   ├── time_update P1(dt)          # 主滤波预测 (set_Ft1 + Φ1 + P1 传播)
│   └── time_update P2(dt)          # NHC 子滤波预测 (F2≈I + Φ2 + P2 传播)
│
├── 2. GNSS 数据处理（仅作用于主滤波 P1）
│   ├── GnssSolutionProvider.get_solution(timestamp)
│   │   ├── 内部模式: GnssInternalProvider → GnssProcessor 解算
│   │   │   ├── 无基站 → SPP 解算
│   │   │   └── 有基站 → RTK 处理器 (RTD/RTK 模式)
│   │   └── 外部模式: GnssExternalProvider → 直接读取外部结果
│   │
│   ├── _gnss_update()              # 获取 GNSS 结果
│   │   └── GnssSolution (mode, pos, vel, cov)
│   │
│   ├── set_Hk() H1                 # 构造主滤波量测矩阵 H1
│   │   ├── GNSS 位置量测 H1/Z/R (仅 P1)
│   │   └── GNSS 速度量测 H1/Z/R (可选，仅 P1)
│   │
│   └── meas_update P1              # 主滤波序贯更新
│       ├── 位置更新 (仅 P1)
│       └── 速度更新 (仅 P1，可选)
│
├── 3. NHC / ZUPT 约束（互斥）
│   ├── 运动状态: NHC 量测（H1 作用 P1 + H2 作用 P2）
│   │   ├── set_Hk() H1             # NHC 对 P1 的贡献（速度/姿态/陀螺零偏）
│   │   ├── set_Hk() H2             # NHC 对 P2 的贡献（安装角/IMU杆臂）
│   │   ├── meas_update P1          # K1 = P1*H1^T*(H1*P1*H1^T+R)^-1，更新 x1, P1
│   │   └── meas_update P2          # K2 = P2*H2^T*(H2*P2*H2^T+R_nhc)^-1，更新 x2, P2
│   │
│   └── 静止状态: ZUPT 量测（仅作用 P1）
│       └── meas_update P1          # 3D 速度约束，K1 = P1*H1^T*...
│
├── 4. 双滤波独立反馈校正
│   ├── feedback P1()               # 主滤波反馈
│   │   ├── 位置/速度/姿态修正
│   │   ├── 零偏修正
│   │   └── GNSS杆臂修正（可选）
│   │
│   └── feedback P2()               # NHC 子滤波反馈
│       ├── 安装角修正（四元数左乘，仅 pitch/yaw）→ 更新 R_b^v
│       └── IMU杆臂修正 → 更新 l_imu^b
│
└── 5. 输出结果
    └── Solution → solution_queue
```

### 8.2 初始化阶段

```
初始化条件: 首次获得 GNSS 解算结果 + 足够 IMU 数据

初始化流程:
1. GNSS 解算获取初始位置/速度
2. 粗对准: align_coarse(wm, vm)
   - 需静态数据，解析法确定初始姿态
3. 或位置辅助对准: align_pva(pos)
   - 利用已知位置辅助对准
4. 或速度辅助对准: align_vva(vel)
   - 利用已知速度辅助对准
5. 初始化双滤波协方差矩阵 P1_0 / P2_0
   - 主滤波 P1_0:
     - 位置/速度: 由 GNSS 解算精度确定
     - 姿态: 由对准精度确定
     - 零偏: 由 IMU 规格确定
     - GNSS杆臂: 由先验精度确定（可选）
   - NHC 子滤波 P2_0:
     - 安装角: 由先验精度确定（pitch/yaw）
     - IMU杆臂: 由先验精度确定
```

### 8.3 异常处理

```
1. GNSS 解算失败
   - 跳过本历元量测更新，纯 INS 推算
   - 协方差持续增大，直至 GNSS 恢复

2. 新息检验失败 (outlier_detect)
   - 参考 t_gsinskf::outlier_detect()
   - 归一化后验残差超限 → 膨胀量测噪声或跳过

3. 协方差矩阵异常
   - 对称性修复: P = 0.5*(P + P^T)
   - 正定性修复: 特征值裁剪

4. IMU 数据中断
   - 标记 INS 状态不可用
   - 等待 IMU 数据恢复后重新初始化
```

---

## 9. 时间同步与 IMU 对齐

> **时间系统**：本节所有时间戳（`imu.timestamp`、`gnss.timestamp`、`t0`/`t1`/`t2`/`t_gnss`）均为 **Unix 时间戳（float 秒，与 rtklib-py `gtime_t.time + gtime_t.sec` 一致）**。
> 时间差 `dt` 通过 Unix 时间戳相减直接得到，无需 GPST/Unix 转换。
> 时间转换工具：`src/core/time_utils.py`（`gpst_to_unix` / `unix_to_gpst`，`GPST_EPOCH_UNIX = 315964800`）。
>
> **IMU 数据结构说明**：本节代码使用**早期设计版本** `ImuMeasurement_Design`（含 `dt`、`angular_velocity`、`acceleration` 字段，详见 StreamDesign.md 第 3.1 节），
> 该版本将在 INS 启用后实现。当前实际 `ImuMeasurement`（`src/core/data_types.py`）字段为 `timestamp` / `week` / `accel` / `gyro`，无 `dt` 字段（`dt` 由相邻历元 timestamp 差计算）。
>
> **时间对齐策略**（2026-07-07 修订）：原计划参考 KF-GINS 的 `imuInterpolate()`（增量切分）实现机械编排阶段的时间同步，但经调查发现 KF-GINS / gnss_ins_lc_nhc / GINav 三个参考项目均使用**增量式 IMU**（dtheta/dvel），其增量切分方法不适用于本项目的**速率式 IMU**（gyro/accel）。因此本项目统一采用 **GNSS 时间最近邻匹配** 策略（详见 [imu.md 第 5 节](file:///home/mxl/workplace/gipylib/skills/imu.md#5-imugnss-时间对齐最近邻匹配) 和 [初始化.md 第 4 节](file:///home/mxl/workplace/gipylib/skills/初始化.md#4-imu-时间对齐到-gnss-时间戳最近邻匹配)）。时间对齐误差（最大半个 IMU 采样周期 ≈ 5ms @100Hz）将由 EKF 作为状态参数 `δt` 在线估计（详见第 9.12 节）。

### 9.1 设计原则

> **策略演变**：原参考 KF-GINS 的 `GIEngine::newImuProcess()` 和 `imuInterpolate()` 实现增量切分，现已改为最近邻匹配。

**核心问题**：当 GNSS 量测时刻 `t_gnss`（Unix 时间戳）落在两个 IMU 时刻 `t0` 和 `t2` 之间时，选择哪个 IMU 历元作为 `t_gnss` 时刻的代表性测量？

**本项目方案**（最近邻匹配）：
1. **估计器内部维护 `imupre`/`imucur`**（参考 KF-GINS 的成员变量设计）
2. **GNSS 数据暂存为 `pending_gnss`**，在 IMU 处理时同步消费
3. **2 种时间对齐情况处理**（在区间内选最近邻 / 不在区间内）
4. **最近邻匹配不修改原始 IMU 历元**，仅生成标记为 `t_gnss` 时刻的新 `ImuMeasurement`
5. **时间对齐误差由 KF 在线估计**（`δt` 作为状态参数，详见第 9.12 节）
6. **Scheduler 简化为直接转发**，时间同步逻辑下沉到估计器

### 9.2 估计器内部状态

```python
class LcEstimator(Integration):
    """松组合估计器"""

    def __init__(self, options):
        # IMU 状态（参考 KF-GINS 的 imupre_/imucur_）
        self.imupre: Optional[ImuMeasurement] = None  # 上一时刻 IMU
        self.imucur: Optional[ImuMeasurement] = None  # 当前时刻 IMU

        # INS 状态（参考 KF-GINS 的 pvapre_/pvacur_）
        self.pvapre: Optional[InsState] = None
        self.pvacur: Optional[InsState] = None

        # 待处理的 GNSS 量测（时间戳对齐用，deque 缓冲避免丢失多个 GNSS 历元）
        self.pending_gnss: collections.deque = collections.deque()

        # 时间对齐阈值（参考 KF-GINS TIME_ALIGN_ERR）
        self.time_align_threshold = 1e-3  # 1ms
```

### 9.3 数据接口

```python
def add_imu(self, imu: ImuMeasurement) -> Optional[Solution]:
    """添加 IMU 数据并处理

    参考 KF-GINS main() 循环: addImuData + newImuProcess
    """
    if self.imucur is not None:
        self.imupre = self.imucur
    self.imucur = imu
    return self._new_imu_process()

def add_gnss(self, gnss: GnssSolution) -> Optional[Solution]:
    """添加 GNSS 数据，暂存待处理

    参考 KF-GINS addGnssData: 仅暂存，不立即处理
    GNSS 数据在 IMU 处理时同步消费
    使用 deque 缓冲，可暂存多个 GNSS 历元（避免丢失）
    """
    self.pending_gnss.append(gnss)
    return None
```

### 9.4 时间对齐判断

参考 KF-GINS 的 `isToUpdate()` 函数，判断 GNSS 更新时机：

```python
def _is_to_update(self, t0: float, t2: float, t_gnss: float) -> int:
    """判断 GNSS 更新时机（参考 KF-GINS isToUpdate）

    参数（均为 Unix 时间戳，float 秒）:
        t0: imupre 时间戳
        t2: imucur 时间戳
        t_gnss: GNSS 量测时间戳

    返回值:
        0: GNSS 时间不在 [t0, t2] 之间，只做 INS 传播
        1: GNSS 时间靠近 t0，先 GNSS 更新再 INS 传播
        2: GNSS 时间靠近 t2，先 INS 传播再 GNSS 更新
        3: GNSS 时间在 (t0, t2) 之间但不靠近任一，需增量切分
    """
    if abs(t0 - t_gnss) < self.time_align_threshold:
        return 1  # 靠近 t0
    elif abs(t2 - t_gnss) <= self.time_align_threshold:
        return 2  # 靠近 t2
    elif t0 < t_gnss < t2:
        return 3  # 在中间
    else:
        return 0  # 不在区间内
```

### 9.5 IMU 增量切分

参考 KF-GINS 的 `imuInterpolate()`，本项目使用角速度/加速度形式（非增量形式），切分时只需修改 `dt`：

```python
def _imu_interpolate(self, imu_pre: ImuMeasurement,
                     imu_cur: ImuMeasurement,
                     timestamp: float) -> ImuMeasurement:
    """IMU 增量切分（参考 KF-GINS imuInterpolate）

    关键：imu_cur 被原地修改，保留剩余增量！

    与 KF-GINS 的差异:
      - KF-GINS 使用增量形式 (dtheta/dvel)，按比例切分增量
      - 本项目使用角速度/加速度形式，切分时只修改 dt
      - 机械编排时通过 dt 计算增量: dtheta = omega * dt

    ※ 本代码使用早期设计版本 ImuMeasurement_Design（含 dt/angular_velocity/acceleration 字段）。
      实际实现时（INS 启用后），imu_cur.angular_velocity → imu_cur.gyro,
      imu_cur.acceleration → imu_cur.accel；dt 由相邻 timestamp 差计算或新增字段。
    ※ timestamp 参数为 Unix 时间戳（float 秒）。
    """
    dt_total = imu_cur.timestamp - imu_pre.timestamp
    lamda = (timestamp - imu_pre.timestamp) / dt_total

    # 创建中间时刻 IMU（前半段）
    midimu = ImuMeasurement(
        timestamp=timestamp,
        angular_velocity=imu_cur.angular_velocity,  # 角速度不变
        acceleration=imu_cur.acceleration,          # 加速度不变
        dt=timestamp - imu_pre.timestamp            # 前半段 dt
    )

    # 关键：imu_cur 原地保留剩余增量（只修改 dt）
    imu_cur.dt = imu_cur.timestamp - timestamp

    return midimu
```

### 9.6 主处理流程

参考 KF-GINS 的 `newImuProcess()`，处理 4 种时间对齐情况：

```python
def _new_imu_process(self) -> Optional[Solution]:
    """处理新的 IMU 数据（参考 KF-GINS newImuProcess）"""
    if self.imupre is None or self.imucur is None:
        return None

    # 当前 IMU 时间作为系统时间
    timestamp = self.imucur.timestamp

    # 判断是否需要 GNSS 更新（从 deque 取最早一个 GNSS 历元）
    if self.pending_gnss:
        gnss = self.pending_gnss[0]   # 窥视队首，不弹出
        updatetime = gnss.timestamp
        res = self._is_to_update(self.imupre.timestamp,
                                 self.imucur.timestamp,
                                 updatetime)
    else:
        res = 0

    if res == 0:
        # 只传播导航状态
        self._ins_propagation(self.imupre, self.imucur)
    elif res == 1:
        # GNSS 靠近 imupre：先 GNSS 更新，再传播
        gnss = self.pending_gnss.popleft()
        self._gnss_update(gnss)
        self._state_feedback()
        self.pvapre = self.pvacur
        self._ins_propagation(self.imupre, self.imucur)
    elif res == 2:
        # GNSS 靠近 imucur：先传播，再 GNSS 更新
        self._ins_propagation(self.imupre, self.imucur)
        gnss = self.pending_gnss.popleft()
        self._gnss_update(gnss)
        self._state_feedback()
    else:  # res == 3
        # GNSS 在两个 IMU 之间：增量切分
        gnss = self.pending_gnss.popleft()
        midimu = self._imu_interpolate(self.imupre, self.imucur, updatetime)

        # 前半段传播
        self._ins_propagation(self.imupre, midimu)

        # GNSS 更新
        self._gnss_update(gnss)
        self._state_feedback()

        # 后半段传播（imucur 已被切分，dt 变小）
        self.pvapre = self.pvacur
        self._ins_propagation(midimu, self.imucur)

    # deque 中剩余 GNSS 历元保留，等下次 IMU 到达时继续处理
    self.pvapre = self.pvacur
    return self._get_solution()
```

### 9.7 四种情况处理流程图

```
GNSS(t1) 到达，IMU 缓冲区有 t0, t2
─────────────────────────────────────────────────────────────

情况1 (res=1): |t1 - t0| < 阈值 (GNSS 靠近 t0)
  ┌─────────────────────────────────────┐
  │ 1. GNSS 更新 (使用 t0 时刻状态)     │
  │ 2. 状态反馈                          │
  │ 3. INS 传播: t0 → t2 (完整增量)     │
  └─────────────────────────────────────┘

情况2 (res=2): |t1 - t2| < 阈值 (GNSS 靠近 t2)
  ┌─────────────────────────────────────┐
  │ 1. INS 传播: t0 → t2 (完整增量)     │
  │ 2. GNSS 更新 (使用 t2 时刻状态)     │
  │ 3. 状态反馈                          │
  └─────────────────────────────────────┘

情况3 (res=3): t0 < t1 < t2 且不靠近任一 (GNSS 在中间)
  ┌─────────────────────────────────────┐
  │ 1. 增量切分:                         │
  │    midimu = t0→t1 增量               │
  │    imucur 保留 t1→t2 剩余增量        │
  │                                      │
  │ 2. INS 传播: t0 → t1 (前半段)        │
  │ 3. GNSS 更新 (t1 时刻状态)           │
  │ 4. 状态反馈                          │
  │ 5. INS 传播: t1 → t2 (后半段)        │
  │    (imucur 已切分，dt 变小)          │
  └─────────────────────────────────────┘

情况0 (res=0): GNSS 时间不在 [t0, t2] 之间
  ┌─────────────────────────────────────┐
  │ 只做 INS 传播: t0 → t2 (完整增量)   │
  └─────────────────────────────────────┘

下一次 IMU 数据 t3 到达:
  ┌─────────────────────────────────────┐
  │ imupre = imucur (t2, 剩余增量)      │
  │ imucur = 新数据 t3                   │
  │ 正常处理 t2 → t3                     │
  └─────────────────────────────────────┘
```

### 9.8 与 E 系机械编排的适配

本项目的机械编排在 E 系下进行（参考 imu.md 6.5 节），`_ins_propagation` 的实现：

```python
def _ins_propagation(self, imu_pre: ImuMeasurement,
                     imu_cur: ImuMeasurement):
    """E 系机械编排（参考 imu.md 6.5 节）

    使用切分后的 dt 进行机械编排

    ※ 本代码使用早期设计版本 ImuMeasurement_Design（含 dt/angular_velocity/acceleration 字段）。
      实际实现时（INS 启用后），imu_cur.angular_velocity → imu_cur.gyro,
      imu_cur.acceleration → imu_cur.accel；dt 由相邻 timestamp 差计算或新增字段。
    ※ imu_pre.timestamp / imu_cur.timestamp 为 Unix 时间戳（float 秒）。
    """
    dt = imu_cur.dt  # 使用切分后的 dt

    # 1. 姿态更新（E 系）
    omega_ib_b = imu_cur.angular_velocity
    self.pvacur.rotation = self.ins_core.attitude_update(
        self.pvapre.rotation, omega_ib_b, dt)

    # 2. 速度更新（E 系）
    f_b = imu_cur.acceleration
    self.pvacur.velocity = self.ins_core.velocity_update(
        self.pvapre.velocity, f_b, self.pvacur.rotation,
        self.pvapre.position, dt)

    # 3. 位置更新（E 系，ECEF 递推）
    self.pvacur.position = self.ins_core.position_update(
        self.pvapre.position, self.pvacur.velocity, dt)

    # 4. EKF 预测（协方差传播）
    self.ins_kf.set_Ft()
    self.ins_kf.predict(dt)
```

### 9.9 与 KF-GINS 的对比

| 维度 | KF-GINS | 本项目 |
|------|---------|--------|
| **IMU 数据形式** | 增量 (dtheta/dvel) | 角速度/加速度 |
| **切分方式** | 按比例切分增量 | 只修改 dt |
| **坐标系** | n 系 | E 系 |
| **架构** | 单线程顺序处理 | 多线程流式 |
| **GNSS 暂存** | gnssdata_ 成员变量 | pending_gnss (deque 缓冲) |
| **时间对齐阈值** | TIME_ALIGN_ERR | time_align_threshold |
| **4 种情况处理** | isToUpdate() | _is_to_update() |

### 9.10 Scheduler 层的简化

由于时间同步逻辑下沉到估计器，Scheduler 的 `_handle_initialized` 简化为直接转发：

```python
def _handle_initialized(self):
    """正常融合模式：简化为直接转发

    时间同步逻辑由估计器内部处理（参考 9.6 节）
    """
    self._drain_imu_queue()
    self._drain_rover_queue()

    # IMU 数据直接推入估计器（估计器内部维护 imupre/imucur）
    for imu_meas in self.imu_buffer:
        data = SensorData(tag="imu")
        data.imu = imu_meas
        self.estimate_queue.put(data)
    self.imu_buffer.clear()

    # GNSS 数据直接推入估计器（估计器内部处理时间对齐）
    for gnss_meas in self.rover_buffer:
        data = SensorData(tag="gnss_solution")
        data.gnss_solution = gnss_meas
        self.estimate_queue.put(data)
    self.rover_buffer.clear()
```

### 9.11 估计线程的改造

```python
class EstimatorThread:
    """估计线程：根据数据类型分发到估计器"""

    def run(self):
        while self.control.is_running():
            try:
                data = self.estimate_queue.get(timeout=0.01)
            except Empty:
                continue

            if data.tag == "imu":
                solution = self.fusion_estimator.add_imu(data.imu)
            elif data.tag == "gnss_solution":
                solution = self.fusion_estimator.add_gnss(data.gnss_solution)
            else:
                continue

            if solution:
                self.solution_queue.put(solution)
```

### 9.12 时间对齐误差的 KF 在线估计（预留接口）

> **状态**：预留接口，当前未实现。最近邻匹配策略已上线，时间对齐误差暂时容忍（最大 5ms @100Hz），后续通过 EKF 状态扩维在线估计。

**背景**：最近邻匹配策略的时间对齐误差为半个 IMU 采样周期（100Hz 下约 5ms）。该误差会耦合到位置/速度量测中，影响高精度场景下的融合精度。通过将时间误差 `δt` 作为 EKF 状态参数在线估计，可补偿该误差。

**状态扩维方案**：

| 维度 | 当前状态（15 维） | 扩维后（16 维） |
|------|------------------|-----------------|
| 状态向量 | `[δr^e(3), δv^e(3), δψ^e(3), δb_g(3), δb_a(3)]` | `[δr^e(3), δv^e(3), δψ^e(3), δb_g(3), δb_a(3), δt(1)]` |
| F 矩阵 | 15×15 | 16×16（新增 `δt` 行/列，`F_δt,δt = -1/τ_δt`） |
| Q 矩阵 | 15×15 | 16×16（新增 `Q_δt = σ_δt² · dt`） |
| 量测模型 | `H = [I_3, 0, ..., 0]`（位置量测） | `H = [I_3, 0, ..., 0, ∂r/∂δt]`（新增时间误差耦合项） |

**观测模型耦合**：
- 时间误差 `δt` 通过 IMU 采样时刻与 GNSS 时刻的偏差耦合到位置/速度量测
- `∂r/∂δt ≈ v^e`（位置误差 ≈ 速度 × 时间误差）
- `∂v/∂δt ≈ a^e`（速度误差 ≈ 加速度 × 时间误差）

**实现计划**：
1. 扩展 `InsState` 增加 `time_bias` 字段
2. 扩展 `TransferMatrix.build_F` / `build_Q` 增加 `δt` 维度
3. 扩展量测雅可比 `H` 增加时间误差耦合项
4. 初始协方差 `P0_δt` 设为 (半个采样周期)² ≈ (5ms)² = 2.5e-5 s²

---

## 10. 队列流水线集成

### 10.1 设计模式总览

本模块作为框架的融合核心，采用**纯队列流水线 + 仅前端策略模式**，与 GInsStream.md / StreamDesign.md 的整体架构保持一致：

| 设计模式 | 角色 | 本模块实现 | 作用 |
|---------|------|-----------|------|
| **策略模式（仅前端）** | 前端里程计 | `ImuMechStrategy(OdometryStrategy)` | 封装 IMU 机械编排，IMU 不可用时可降级为 GNSS 纯解算前端 |
| **纯队列流水线** | 融合触发 | `LcIntegration` 从 `estimate_queue` 取数据 | 无观察者回调，主循环 `process_epoch()` 驱动融合 |
| **依赖注入** | 解耦 | 构造函数接收 `OdometryStrategy` 与传感器实例 | 便于测试与替换，不在内部 new |

> **不使用 FusionStrategy 层**：后端融合算法（EKF + NHC + ZUPT）直接由 `LcIntegration` 承担。
> **不使用观察者模式**：传感器不持有观察者列表，无 `attach/detach/notify`，无 `on_data()` 回调。

### 10.2 策略基类定义（仅前端）

```python
from abc import ABC, abstractmethod

class OdometryStrategy(ABC):
    """前端里程计算法策略基类

    封装前端里程计的解算逻辑，与融合层解耦。
    子类:
    - ImuMechStrategy: IMU 机械编排前端（详见 imu.md）
    - GnssPositioningStrategy: GNSS 纯解算前端（IMU 不可用降级，详见 gnss.md）
      ├── SppStrategy
      └── RtkStrategy
    """

    @abstractmethod
    def execute(self, measurement) -> 'InsState':
        """执行前端里程计解算，返回当前 INS 状态"""
        ...

# 注：不再定义 FusionStrategy 基类
# 后端融合（EKF + NHC + ZUPT）由 LcIntegration 直接承担，方法包括:
#   - _ins_propagation()      时间更新（双滤波 P1/P2 独立）
#   - _gnss_update()           GNSS 量测更新（仅 P1）
#   - _nhc_update()            NHC 量测更新（H1 作用 P1 + H2 作用 P2）
#   - _zupt_update()           ZUPT 量测更新（仅 P1，与 NHC 互斥）
#   - _state_feedback()        双滤波独立反馈
```

### 10.3 队列流水线集成（替代观察者模式）

```python
# 估计线程主循环（替代原 on_data() 回调）
class EstimatorThread:
    """估计线程：从 estimate_queue 取数据，调用 LcIntegration.process_epoch()

    无观察者回调，无 on_data()；数据通过 queue.Queue 在线程间传递。
    """

    def __init__(self, integration: 'LcIntegration',
                 estimate_queue: 'Queue[SensorData]',
                 solution_queue: 'Queue[Solution]',
                 control: 'ThreadControl'):
        self.integration = integration
        self.estimate_queue = estimate_queue
        self.solution_queue = solution_queue
        self.control = control

    def run(self):
        """主循环：从 estimate_queue 取数据 → process_epoch → 推入 solution_queue"""
        while self.control.is_running():
            try:
                data = self.estimate_queue.get(timeout=0.01)
            except Empty:
                continue

            # 调用 LcIntegration 的处理方法（内部完成时间对齐 + 双滤波 EKF）
            solution = self.integration.process_epoch(data)

            if solution is not None:
                self.solution_queue.put(solution)
```

### 10.4 数据通路（纯队列流水线）

```
┌──────────────┐    queue.put()    ┌──────────────┐    queue.put()    ┌──────────────┐
│  Stream 层   │ ───────────────→ │ Integration │ ───────────────→ │  Estimate   │
│ (各 Streamer)│   imu_queue等    │  Scheduler   │  estimate_queue  │   Thread    │
└──────────────┘                  │  (仅转发)    │                  │             │
                                  └──────────────┘                  └──────┬──────┘
                                                                         │
                                                                    queue.put()
                                                                    solution_queue
                                                                         │
                                                                         ▼
                                                                  ┌──────────────┐
                                                                  │  Log 层     │
                                                                  │  (Logger)   │
                                                                  └──────────────┘
```

**关键点**：
- 传感器（`BaseSensor` 子类）仅持有 `output_queue`，通过 `queue.put()` 推数据，**无 Subject 角色、无 attach/notify**
- `Scheduler` 仅从各 sensor_queue 取数据并转发到 `estimate_queue`，**不做时间对齐**
- `LcIntegration.process_epoch()` 从 `estimate_queue` 取数据，**内部完成时间对齐（增量切分）+ 双滤波 EKF + 输出 Solution**
- 输出 Solution 推入 `solution_queue`，由 Logger 消费

### 10.5 依赖注入装配示例

框架通过构造函数接收具体实例，装配过程集中在入口处（参考 GInsStream.md）：

```python
def build_lc_pipeline(config: dict) -> 'LcIntegration':
    """装配松组合流水线（依赖注入）

    所有具体实例在此创建并注入，LcIntegration 自身不 new 任何依赖。
    不创建 FusionStrategy，后端融合由 LcIntegration 直接承担。
    """
    # 1. 创建传感器实例（工厂模式，详见 StreamDesign.md）
    imu_sensor = SensorFactory.create("imu", config["imu"])
    gnss_sensor = SensorFactory.create("gnss", config["gnss"])

    # 2. 创建 INS 滤波器与 GNSS 结果提供者
    inskf = LcEstimator(config["ekf"])   # 双滤波实现
    gnss_provider = GnssSolutionProviderFactory.create(config["gnss_source"])

    # 3. 创建前端策略实例（仅前端，不再创建 FusionStrategy）
    odometry_strategy = ImuMechStrategy(inskf.ins_core, inskf.preprocessor)

    # 4. 依赖注入装配 LcIntegration（不注册观察者）
    integration = LcIntegration(
        inskf=inskf,
        gnss_provider=gnss_provider,
        odometry_strategy=odometry_strategy,
        sensors=[imu_sensor, gnss_sensor],
    )
    return integration
```

### 10.6 前端策略热切换（仅前端）

运行时可替换前端策略实例，无需修改 LcIntegration 内部：

```python
# 场景：IMU 故障，切换为 GNSS 纯解算前端
integration._odometry_strategy = SppStrategy(gnss_processor)

# 后端融合算法（EKF + NHC + ZUPT）由 LcIntegration 直接承担，无 FusionStrategy 切换
# 如需紧组合，使用 TcIntegration（预留）
```

### 10.7 OOP 三大特性体现

| 特性 | 体现 |
|------|------|
| **封装** | 双滤波状态（x1, x2, P1, P2, F1, F2, H1, H2）封装在 `InsKf` 子类内部，外部仅通过 `process_epoch()` 访问 |
| **继承** | `InsKf(ABC)` → `LcEstimator` / `TcEstimator`；`Integration(ABC)` → `LcIntegration` / `TcIntegration` |
| **多态** | 框架持有 `OdometryStrategy` 抽象引用，运行时调用具体子类方法（ImuMechStrategy / SppStrategy / RtkStrategy） |
