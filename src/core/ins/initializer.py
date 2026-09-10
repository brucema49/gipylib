"""INS 初始化器。

参考 KF-GINS initialize。
支持三种初始化模式: 静态、速度矢量、位置差分。

本项目实现低精度 INS 初始化（初始化调整.md）：
- 静态: GNSS 历史平均位置, vel=0, att=[0,0,0], 不做 AcceLeveling
- 动态: roll=0, pitch=0, yaw=atan2(v_E, v_N)
- IMU 误差: 只初始化陀螺/加计零偏, 从 config 读取
"""
import math
from collections import deque
from enum import Enum
from typing import Optional, Tuple, List

import numpy as np

from src.core.data_types import AlignedBlock, GnssSolution, ImuMeasurement, InsState
from src.core.ins.attitude import att_caln2e, dcm2quat, euler2dcm
from src.core.ins.earth_param import cal_Ce2n, cal_Cn2e, ecef2llh, llh2ecef
from src.core.ins.interpolator import find_bracket_imus, imu_interpolate_linear
from src.core.ins.state_index import StateIndex


class InitMode(Enum):
    """初始化模式。"""
    STATIC = "static"
    VELOCITY_VECTOR = "velocity_vector"
    POSITION_DIFF = "position_diff"
    FIXED = "fixed"


class InsInitializer:
    """INS 初始化器。

    根据 GNSS 数据和 IMU 数据执行初始对准, 装配初始状态和协方差。
    不执行机械编排, 仅完成初始化阶段。
    """

    def __init__(self, config: dict):
        self.config = config
        ins_cfg = config.get("ins", {})
        # 三组独立阈值:
        #   静态检测: GNSS 速度范数 < static_speed_threshold (0.5 m/s)
        #   动态速度: 速度范数 > dynamic_speed_threshold (4.0 m/s)
        #   动态角速度: 陀螺范数 < angular_velocity_threshold (30 deg/s ≈ 0.5236 rad/s)
        self.static_speed_threshold = ins_cfg.get("static_speed_threshold", 0.5)
        self.dynamic_speed_threshold = ins_cfg.get("dynamic_speed_threshold", 4.0)
        self.angular_velocity_threshold = ins_cfg.get(
            "angular_velocity_threshold", 30.0 * math.pi / 180.0
        )
        self.gnss_buffer_size = ins_cfg.get("gnss_buffer_size", 3)
        self.gnss_buffer: deque = deque(maxlen=self.gnss_buffer_size)
        self.static_duration = ins_cfg.get("static_duration", 10.0)
        self.imu_rate = ins_cfg.get("data_rate", 100)
        # 高精度模式开关（初始化调整.md）：本项目当前只实现低精度模式
        self.high_precision_ins_mode = ins_cfg.get("high_precision_ins_mode", False)
        # GNSS 位置缓冲区（用于低精度静态初始化的位置平均，static_duration 秒窗口）
        self.gnss_position_buffer: deque = deque(maxlen=int(self.static_duration) + 1)
        # 静态模式 GNSS 平均位置 (_align_static 计算, _assemble_state 使用)
        self._static_pos_mean: Optional[np.ndarray] = None
        # 静态标定零偏 (预留接口, _calibrate_bias_from_static_imu 计算)
        # 当前始终为 None, 未来可实现静态零偏标定
        self._calibrated_gyro_bias: Optional[np.ndarray] = None
        self._calibrated_accel_bias: Optional[np.ndarray] = None

    def initialize(self, aligned_block: AlignedBlock,
                   mode: InitMode) -> Tuple[InsState, np.ndarray]:
        """初始化主入口。

        Args:
            aligned_block: 对齐后的 GNSS + IMU 数据块
            mode: 初始化模式

        Returns:
            (InsState, P): 初始状态 + 单滤波协方差 (维度 = si.dim)
        """
        gnss = aligned_block.gnss
        imu_list = aligned_block.imu_list

        # 1. IMU 时间对齐到 GNSS 时间戳 (GVINS 风格线性插值, 验证包夹条件)
        # 动态初始化仅需最新 IMU (不要求包夹), 静态初始化需要包夹做加速度计调平
        if mode == InitMode.STATIC:
            interp_imu = self._align_imu_to_gnss(imu_list, gnss.timestamp)
            if interp_imu is None:
                raise ValueError(
                    "IMU 数据不满足包夹条件 (GNSS 时间戳前后需各有 IMU 历元)"
                )

        # 2. 动态模式下检查陀螺角速度范数 (< 30 deg/s)
        if mode in (InitMode.VELOCITY_VECTOR, InitMode.POSITION_DIFF):
            gyro_norm = self._compute_gyro_norm(imu_list, gnss.timestamp)
            if gyro_norm >= self.angular_velocity_threshold:
                raise ValueError(
                    f"角速度范数 {gyro_norm:.4f} rad/s "
                    f"(={math.degrees(gyro_norm):.2f} deg/s) "
                    f"超过阈值 {self.angular_velocity_threshold:.4f} rad/s "
                    f"(={math.degrees(self.angular_velocity_threshold):.2f} deg/s), "
                    f"动态初始化要求角速度较小"
                )

        # 3. 按模式执行对准, 返回 (att_rpy, vel_e_for_state)
        if mode == InitMode.STATIC:
            result = self._align_static(aligned_block.imu_list, gnss)
        elif mode == InitMode.VELOCITY_VECTOR:
            result = self._align_motion_velocity(gnss)
        elif mode == InitMode.POSITION_DIFF:
            result = self._align_motion_displacement(gnss)
        elif mode == InitMode.FIXED:
            result = self._align_fixed()
        else:
            raise ValueError(f"Unsupported init mode: {mode}")

        if result is None:
            raise ValueError("对准失败 (缓冲区未满或未达运动阈值)")
        att_rpy, vel_e = result

        # 4. 装配初始状态 (静态模式使用 GNSS 历史平均位置)
        state = self._assemble_state(gnss, att_rpy, vel_e, mode)

        # 5. 装配初始协方差
        P = self._set_initial_variance(mode)

        return state, P

    def _align_fixed(self) -> Tuple[np.ndarray, np.ndarray]:
        """Use externally supplied PVA fields, matching KF-GINS."""
        ins_cfg = self.config.get("ins", {})
        att_deg = np.asarray(ins_cfg["initial_attitude_deg"], dtype=float)
        vel_n = np.asarray(ins_cfg.get("initial_velocity_ned", [0.0, 0.0, 0.0]), dtype=float)
        if att_deg.shape != (3,) or vel_n.shape != (3,):
            raise ValueError("fixed initialization PVA fields must have three elements")
        pos_llh = np.asarray(ins_cfg["initial_position_llh"], dtype=float)
        if pos_llh.shape != (3,):
            raise ValueError("initial_position_llh must have three elements")
        lat, lon = math.radians(pos_llh[0]), math.radians(pos_llh[1])
        return np.radians(att_deg), cal_Cn2e(lat, lon) @ vel_n

    def _align_imu_to_gnss(self, imu_list: List[ImuMeasurement],
                           t_gnss: float) -> Optional[ImuMeasurement]:
        """IMU 数据时间对齐到 GNSS 时间戳 (GVINS 风格线性插值, 与主循环一致)。

        从 imu_list 中找包夹 t_gnss 的两个历元，线性插值到 t_gnss。
        与 LcIntegration.add_imu 使用同一套插值策略 (imu_interpolate_linear),
        保证初始化与机械编排的 IMU 数据输入口径一致。
        """
        if not imu_list:
            return None
        bracket = find_bracket_imus(imu_list, t_gnss)
        if bracket is None:
            return None
        imu_pre, imu_cur, _ = bracket
        return imu_interpolate_linear(imu_pre, imu_cur, t_gnss)

    def _compute_gyro_norm(self, imu_list: List[ImuMeasurement],
                           t_gnss: float, window: float = 1.0) -> float:
        """计算 GNSS 时间戳附近窗口内 IMU 陀螺角速度的平均范数。

        动态初始化要求角速度较小 (< 30 deg/s), 确保车辆未在急转弯。

        Args:
            imu_list: IMU 数据列表
            t_gnss: GNSS 时间戳
            window: 时间窗口 [s] (前后各 window 秒)

        Returns:
            平均角速度范数 [rad/s], 无数据时返回 inf
        """
        t_start = t_gnss - window
        t_end = t_gnss + window
        window_imus = [imu for imu in imu_list
                       if t_start <= imu.timestamp <= t_end]
        if not window_imus:
            window_imus = list(imu_list)
        if not window_imus:
            return float('inf')
        gyro_norms = [float(np.linalg.norm(imu.gyro)) for imu in window_imus]
        return float(np.mean(gyro_norms))

    def _align_static(self, imu_list: List[ImuMeasurement],
                      gnss: GnssSolution) -> Tuple[np.ndarray, np.ndarray]:
        """静态对准（低精度模式 + 加速度计调平）。

        低精度模式：用加速度计平均值估计 roll/pitch (AcceLeveling), yaw=0。
        - 位置: GNSS 历史平均（static_duration 秒窗口，少于 10s 用全部，多于 10s 取最新 10s）
        - 速度: 置 0
        - 姿态: [roll, pitch, yaw=0] (roll/pitch 由加速度计调平)

        静态检测阈值: GNSS 速度范数 < static_speed_threshold (0.5 m/s)

        Returns:
            (att_rpy, vel_e): 姿态 [roll, pitch, yaw] 和 ECEF 速度 (静态为 0)
        """
        # 静态检测: GNSS 速度范数必须低于静态阈值
        if gnss.velocity is not None:
            speed = float(np.linalg.norm(gnss.velocity))
            if speed >= self.static_speed_threshold:
                raise ValueError(
                    f"GNSS 速度 {speed:.3f} m/s 超过静态阈值 "
                    f"{self.static_speed_threshold} m/s, 不满足静态条件"
                )

        # GNSS 位置缓冲区：追加当前位置，按 static_duration 秒窗口过滤
        self.gnss_position_buffer.append((gnss.timestamp, gnss.position.copy()))
        t_now = gnss.timestamp
        t_start = t_now - self.static_duration
        window_positions = [pos for (t, pos) in self.gnss_position_buffer
                            if t >= t_start]
        if not window_positions:
            window_positions = [gnss.position.copy()]  # 少于 10s 用全部

        # 计算平均位置
        pos_mean = np.mean(window_positions, axis=0)
        # 保存平均位置供 _assemble_state 使用 (静态模式)
        self._static_pos_mean = pos_mean

        # 尝试静态标定零偏 (预留接口, 当前返回 None)
        # 未来可实现: 静止时陀螺平均 = 地球自转分量 + 零偏, 据此估计零偏
        calibrated = self._calibrate_bias_from_static_imu(imu_list, gnss)
        if calibrated is not None:
            self._calibrated_gyro_bias, self._calibrated_accel_bias = calibrated
        else:
            self._calibrated_gyro_bias = None
            self._calibrated_accel_bias = None

        # 加速度计调平 (AcceLeveling): 用静态加速度计平均值估计 roll/pitch
        # 静止时 f_body = [g sin θ, -g sin φ cos θ, -g cos φ cos θ] (FRD)
        # 故 pitch = atan2(f_x, sqrt(f_y² + f_z²)), roll = atan2(-f_y, -f_z)
        # yaw 无法从加速度计估计, 设为 0
        if imu_list:
            acc_mean = np.mean([imu.accel for imu in imu_list], axis=0)
            fx, fy, fz = float(acc_mean[0]), float(acc_mean[1]), float(acc_mean[2])
            pitch = math.atan2(fx, math.sqrt(fy * fy + fz * fz))
            roll = math.atan2(-fy, -fz)
            yaw = 0.0
            att_rpy = np.array([roll, pitch, yaw], dtype=np.float64)
        else:
            att_rpy = np.zeros(3, dtype=np.float64)  # roll=0, pitch=0, yaw=0
        vel_e = np.zeros(3, dtype=np.float64)  # 静态速度为 0
        return att_rpy, vel_e

    def _calibrate_bias_from_static_imu(
        self, imu_list: List[ImuMeasurement], gnss: GnssSolution
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """静态标定 IMU 零偏（预留接口，当前不实现）。

        原理: 车辆静止时, 陀螺测量值 = 地球自转分量 + 零偏 + 噪声。
        通过对静态 IMU 数据求平均可估计零偏。加速度计同理。

        本接口为后续优化预留。

        Args:
            imu_list: 静态期间的 IMU 数据列表
            gnss: 当前 GNSS 解 (用于计算位置处的地球自转分量)

        Returns:
            (gyro_bias_rad_s, accel_bias_m_s2) 或 None (不标定)
        """
        return None

    def _align_motion_velocity(self, gnss: GnssSolution
                               ) -> Tuple[np.ndarray, np.ndarray]:
        """速度矢量对准: GNSS 速度方向计算 yaw（初始化调整.md）。

        本项目简化版:
        - yaw = atan2(v_E, v_N) (东向速度与北向速度)
        - pitch = 0 (初始化调整.md: 其他姿态角设为 0)
        - roll = 0

        速度阈值检查使用平面速度范数 sqrt(v_E^2 + v_N^2) (不含垂向),
        与 _align_motion_displacement 保持一致。

        Returns:
            (att_rpy, vel_e): 姿态和 ECEF 速度
        """
        if gnss.velocity is None:
            raise ValueError("GNSS 无速度数据, 无法执行速度矢量对准")

        vel_e = gnss.velocity.copy()
        # ECEF 速度 → NED 速度 (cal_Ce2n 返回 NED 系: N, E, D)
        lat, lon, _ = ecef2llh(gnss.position)
        C_e_n = cal_Ce2n(lat, lon)
        vel_n = C_e_n @ vel_e  # vel_n[0]=North, vel_n[1]=East, vel_n[2]=Down

        # 平面速度范数 (EN), 不含垂向分量
        planar_speed = math.sqrt(vel_n[0] ** 2 + vel_n[1] ** 2)
        if planar_speed < self.dynamic_speed_threshold:
            raise ValueError(
                f"GNSS 平面速度 {planar_speed:.3f} m/s 低于动态速度阈值 "
                f"{self.dynamic_speed_threshold} m/s"
            )

        # 初始化调整.md: yaw = atan2(v_E, v_N), 其他姿态角设为 0
        roll = 0.0
        pitch = 0.0
        yaw = math.atan2(vel_n[1], vel_n[0])  # atan2(East, North) — 航向角

        att_rpy = np.array([roll, pitch, yaw], dtype=np.float64)
        return att_rpy, vel_e

    def _align_motion_displacement(self, gnss: GnssSolution
                                   ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """位置差分对准: gnss_buffer_size 历元缓冲, 所有相邻历元平面速度(EN)均超阈值才初始化。

        参考 issue/7-30松组合.md。
        速度阈值检查使用平面速度范数 sqrt(v_E^2 + v_N^2) (不含垂向)。
        要求缓冲区内所有相邻历元差分得到的平面速度均 >= 阈值, 保证连续运动。
        速度计算使用首尾历元差分 (span 最大, 噪声最小), issue 要求 "首尾位置差分"。

        Returns:
            (att_rpy, vel_e) 或 None (缓冲区未满或未达运动阈值)
        """
        self.gnss_buffer.append(gnss)

        if len(self.gnss_buffer) < self.gnss_buffer_size:
            return None  # 缓冲区未满, 延迟初始化

        # 检查缓冲区内所有相邻历元的平面差分速度均 >= 阈值
        for i in range(1, len(self.gnss_buffer)):
            gnss_prev = self.gnss_buffer[i - 1]
            gnss_curr = self.gnss_buffer[i]
            dt = gnss_curr.timestamp - gnss_prev.timestamp
            if dt <= 0:
                return None

            vel_e = (gnss_curr.position - gnss_prev.position) / dt
            # ECEF → ENU, 取平面速度 (EN)
            lat, lon, _ = ecef2llh(gnss_curr.position)
            C_e2n = cal_Ce2n(lat, lon)
            vel_n = C_e2n @ vel_e
            planar_speed = math.sqrt(vel_n[0] ** 2 + vel_n[1] ** 2)  # EN
            if planar_speed < self.dynamic_speed_threshold:
                return None  # 某一对相邻历元未达阈值, 继续等待

        # 用首尾历元差分计算速度 (span 最大, 噪声最小, issue 要求 "首尾位置差分")
        gnss_first = self.gnss_buffer[0]
        gnss_last = self.gnss_buffer[-1]
        dt = gnss_last.timestamp - gnss_first.timestamp
        vel_e = (gnss_last.position - gnss_first.position) / dt

        # 用差分速度走速度矢量对准
        gnss_with_vel = GnssSolution(
            timestamp=gnss_last.timestamp,
            week=gnss_last.week,
            position=gnss_last.position.copy(),
            quality=gnss_last.quality,
            num_sv=gnss_last.num_sv,
            sd=gnss_last.sd.copy(),
            cov=gnss_last.cov,
            velocity=vel_e,
            vel_sd=None,
        )
        return self._align_motion_velocity(gnss_with_vel)

    def _assemble_state(self, gnss: GnssSolution, att_rpy: np.ndarray,
                        vel_e: np.ndarray, mode: InitMode) -> InsState:
        """装配初始状态 (初始化.md 第 11 节, 初始化调整.md)。

        - 静态模式: 位置使用 _align_static 计算的 GNSS 历史平均位置
        - 动态模式: 位置使用 GNSS 当前历元位置
        - IMU 零偏: 优先使用静态标定结果, 否则从 config 读取
                    initial_gyro_bias (deg/h → rad/s)
                    initial_acce_bias (mGal → m/s²)
        """
        ins_cfg = self.config.get("ins", {})

        # 静态模式使用 GNSS 历史平均位置 (低精度模式, 初始化调整.md)
        if mode == InitMode.FIXED:
            pos_llh = np.asarray(ins_cfg["initial_position_llh"], dtype=float)
            pos_e = llh2ecef(math.radians(pos_llh[0]), math.radians(pos_llh[1]), pos_llh[2])
        elif mode == InitMode.STATIC and self._static_pos_mean is not None:
            pos_e = self._static_pos_mean.copy()
        else:
            pos_e = gnss.position.copy()
        if vel_e is None:
            vel_e = np.zeros(3, dtype=np.float64)

        # 姿态矩阵: C_b^e = C_n^e × C_b^n
        lat, lon, _ = ecef2llh(pos_e)
        C_b_n = euler2dcm(att_rpy)
        C_b_e = att_caln2e(lat, lon, C_b_n)
        q_b_e = dcm2quat(C_b_e)

        # 单位转换常量
        constant_g0 = 9.7803267715
        dh2rs = math.pi / 180.0 / 3600.0       # deg/hour → rad/s
        constant_mgal = 1e-6 * constant_g0      # mGal → m/s²

        # IMU 零偏: 优先使用静态标定结果, 否则从 config 读取
        if (mode == InitMode.STATIC
                and self._calibrated_gyro_bias is not None):
            gyro_bias = self._calibrated_gyro_bias.copy()
            accel_bias = self._calibrated_accel_bias.copy()
        else:
            gyro_bias_deg_h = np.array(
                ins_cfg.get("initial_gyro_bias", [0.0, 0.0, 0.0]),
                dtype=np.float64,
            )
            accel_bias_mgal = np.array(
                ins_cfg.get("initial_acce_bias", [0.0, 0.0, 0.0]),
                dtype=np.float64,
            )
            gyro_bias = gyro_bias_deg_h * dh2rs
            accel_bias = accel_bias_mgal * constant_mgal

        imu_angle = np.array(ins_cfg.get("initial_imu_angle", [0, 0]),
                             dtype=np.float64)
        if imu_angle.size >= 2:
            imu_angle = np.array([math.radians(imu_angle[0]),
                                  math.radians(imu_angle[1])])
        else:
            imu_angle = np.zeros(2, dtype=np.float64)
        imu_leverarm = np.array(ins_cfg.get("initial_imu_leverarm", [0, 0, 0]),
                                dtype=np.float64)
        leverarm = np.array(ins_cfg.get("leverarm", [0, 0, 0]),
                            dtype=np.float64)
        gyro_scale = np.array(ins_cfg.get("initial_gyro_scale_ppm", [0, 0, 0]),
                              dtype=np.float64) * 1.0e-6
        accel_scale = np.array(ins_cfg.get("initial_acce_scale_ppm", [0, 0, 0]),
                               dtype=np.float64) * 1.0e-6

        # The INS position is the IMU reference point.  Awesome's truth and
        # the external RTK solution refer to different points, so move the
        # initial state from the GNSS antenna to the IMU using the fixed
        # body-frame lever arm.  Estimated lever arms retain the legacy
        # state convention and are handled by their optional state block.
        if (mode != InitMode.FIXED and np.linalg.norm(leverarm) > 1e-12
                and not ins_cfg.get("estimate_leverarm", 0)):
            pos_e = pos_e - C_b_e @ leverarm

        return InsState(
            timestamp=gnss.timestamp,
            pos_e=pos_e,
            vel_e=vel_e,
            C_b_e=C_b_e,
            q_b_e=q_b_e,
            att_rpy=att_rpy,
            gyro_bias=gyro_bias,
            accel_bias=accel_bias,
            imu_angle=imu_angle,
            imu_leverarm=imu_leverarm,
            leverarm=leverarm,
            gyro_scale=gyro_scale,
            accel_scale=accel_scale,
        )

    def _set_initial_variance(self, mode: InitMode) -> np.ndarray:
        """装配初始协方差 P (单滤波, 维度 = si.dim)。

        基础 15 维 [pos(3), vel(3), att(3), gyro_bias(3), accel_bias(3)]
        可选块按 StateIndex 位置填充: lever_arm(3), imu_angle(2),
        imu_leverarm(3), time_sync(1)

        Note: yaw 不确定度按对准模式调整: roll/pitch 可从加速度计调平获得
        (精度 ~0.3°), 但 yaw 在静态对准时不可观测 (设 pi), 在动态对准时
        取决于 GNSS 速度/位置精度 (velocity_vector ~0.3 rad, position_diff ~1.0 rad)。
        若使用过小的 yaw std, 滤波器无法纠正初始 yaw 误差, 导致位置发散振荡。
        """
        ins_cfg = self.config.get("ins", {})
        si = StateIndex.from_config(self.config)

        P = np.zeros((si.dim, si.dim), dtype=np.float64)

        # 基础 15 维初始不确定度 (1σ, SI 单位)
        pos_std = np.array(ins_cfg.get("initial_pos_std_si",
                                       [30.0, 30.0, 30.0]), dtype=np.float64)
        vel_std = np.array(ins_cfg.get("initial_vel_std_si",
                                       [10.0, 10.0, 10.0]), dtype=np.float64)
        att_std = np.array(ins_cfg.get("initial_att_std_si",
                                       [0.00524, 0.00524, 0.00524]),
                           dtype=np.float64)
        # yaw 不确定度按对准模式调整: roll/pitch 由加速度计调平 (config 值合理),
        # yaw 在静态对准不可观测 (pi rad), 动态对准取决于 GNSS 精度
        if mode == InitMode.STATIC:
            att_std[2] = math.pi          # yaw 完全未知
        elif mode == InitMode.VELOCITY_VECTOR:
            att_std[2] = 0.3              # ~17 deg (GNSS 速度精度依赖)
        elif mode == InitMode.POSITION_DIFF:
            att_std[2] = 1.0              # ~57 deg (位置差分精度差)
        gyro_bias_std = np.array(ins_cfg.get("gyro_bias_std_si",
                                             [2.424e-5, 2.424e-5, 2.424e-5]),
                                 dtype=np.float64)
        acce_bias_std = np.array(ins_cfg.get("acce_bias_std_si",
                                             [0.0489, 0.0489, 0.0489]),
                                 dtype=np.float64)

        P[0:3, 0:3] = np.diag(pos_std ** 2)
        P[3:6, 3:6] = np.diag(vel_std ** 2)
        P[6:9, 6:9] = np.diag(att_std ** 2)
        P[9:12, 9:12] = np.diag(gyro_bias_std ** 2)
        P[12:15, 12:15] = np.diag(acce_bias_std ** 2)

        if si.has_imu_scale():
            gyro_scale_std = np.array(
                ins_cfg.get("gyro_scale_std_ppm", [300.0, 300.0, 300.0]),
                dtype=np.float64) * 1.0e-6
            accel_scale_std = np.array(
                ins_cfg.get("acce_scale_std_ppm", [300.0, 300.0, 300.0]),
                dtype=np.float64) * 1.0e-6
            P[si.gyro_scale:si.gyro_scale + 3,
              si.gyro_scale:si.gyro_scale + 3] = np.diag(gyro_scale_std ** 2)
            P[si.accel_scale:si.accel_scale + 3,
              si.accel_scale:si.accel_scale + 3] = np.diag(accel_scale_std ** 2)

        # 可选块: GNSS 杆臂 (随机常数, 初始不确定度 lever_arm_std)
        if si.has_lever_arm():
            lever_arm_std = np.array(
                ins_cfg.get("lever_arm_std", [0.1, 0.1, 0.1]),
                dtype=np.float64)
            i = si.lever_arm
            P[i:i+3, i:i+3] = np.diag(lever_arm_std ** 2)

        # 可选块: IMU 安装角 (imu_angle_std, deg → rad)
        if si.has_imu_angle():
            imu_angle_std = np.array(
                ins_cfg.get("imu_angle_std", [10.0, 10.0]),
                dtype=np.float64)
            imu_angle_std = np.radians(imu_angle_std)
            i = si.imu_angle
            P[i:i+2, i:i+2] = np.diag(imu_angle_std ** 2)

        # 可选块: IMU 杆臂 (imu_leverarm_std)
        if si.has_imu_leverarm():
            imu_leverarm_std = np.array(
                ins_cfg.get("imu_leverarm_std", [1.0, 1.0, 1.0]),
                dtype=np.float64)
            i = si.imu_leverarm
            P[i:i+3, i:i+3] = np.diag(imu_leverarm_std ** 2)

        # 可选块: 时间对齐 (time_sync_std)
        if si.has_time_sync():
            time_sync_std = float(ins_cfg.get("time_sync_std", 0.01))
            i = si.time_sync
            P[i, i] = time_sync_std ** 2

        return P

    def reset_gnss_buffer(self):
        """清空 GNSS 历元缓冲区 (reboot 时调用)。"""
        self.gnss_buffer.clear()
        self.gnss_position_buffer.clear()
        self._static_pos_mean = None
        self._calibrated_gyro_bias = None
        self._calibrated_accel_bias = None
