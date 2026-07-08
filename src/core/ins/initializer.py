"""INS 初始化器。

参考 KF-GINS initialize + gnss_ins_lc_nhc StartAligning/AcceLeveling/MotionAligned。
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
from src.core.ins.earth_param import cal_Ce2n, ecef2llh
from src.core.ins.interpolator import find_bracket_imus, imu_interpolate


class InitMode(Enum):
    """初始化模式。"""
    STATIC = "static"
    VELOCITY_VECTOR = "velocity_vector"
    POSITION_DIFF = "position_diff"


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
                   mode: InitMode) -> Tuple[InsState, np.ndarray, np.ndarray]:
        """初始化主入口。

        Args:
            aligned_block: 对齐后的 GNSS + IMU 数据块
            mode: 初始化模式

        Returns:
            (InsState, P1, P2): 初始状态 + 主滤波协方差 + NHC 子滤波协方差
        """
        gnss = aligned_block.gnss
        imu_list = aligned_block.imu_list

        # 1. IMU 时间对齐到 GNSS 时间戳 (最近邻匹配, 验证包夹条件)
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
        else:
            raise ValueError(f"Unsupported init mode: {mode}")

        if result is None:
            raise ValueError("对准失败 (缓冲区未满或未达运动阈值)")
        att_rpy, vel_e = result

        # 4. 装配初始状态 (静态模式使用 GNSS 历史平均位置)
        state = self._assemble_state(gnss, att_rpy, vel_e, mode)

        # 5. 装配初始协方差
        P1, P2 = self._set_initial_variance(mode)

        return state, P1, P2

    def _align_imu_to_gnss(self, imu_list: List[ImuMeasurement],
                           t_gnss: float) -> Optional[ImuMeasurement]:
        """IMU 数据时间对齐到 GNSS 时间戳 (最近邻匹配, 初始化.md 第 4 节)。

        从 imu_list 中找包夹 t_gnss 的两个历元，选时间戳最近者返回。
        时间对齐误差 (最大半个 IMU 采样周期) 后续由 KF 估计。
        """
        if not imu_list:
            return None
        bracket = find_bracket_imus(imu_list, t_gnss)
        if bracket is None:
            return None
        imu_pre, imu_cur, _ = bracket
        return imu_interpolate(imu_pre, imu_cur, t_gnss)

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
        """静态对准（低精度模式，初始化调整.md）。

        低精度模式：不做 AcceLeveling，姿态全设为 0。
        - 位置: GNSS 历史平均（static_duration 秒窗口，少于 10s 用全部，多于 10s 取最新 10s）
        - 速度: 置 0
        - 姿态: [0, 0, 0]

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

        # 低精度模式: 姿态全设为 0, 不做 AcceLeveling
        att_rpy = np.zeros(3, dtype=np.float64)  # roll=0, pitch=0, yaw=0
        vel_e = np.zeros(3, dtype=np.float64)  # 静态速度为 0
        return att_rpy, vel_e

    def _calibrate_bias_from_static_imu(
        self, imu_list: List[ImuMeasurement], gnss: GnssSolution
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """静态标定 IMU 零偏（预留接口，当前不实现）。

        原理: 车辆静止时, 陀螺测量值 = 地球自转分量 + 零偏 + 噪声。
        通过对静态 IMU 数据求平均可估计零偏。加速度计同理。

        参考: gnss_ins_lc_nhc AcceLeveling (navinitialized.cc 行 47-59)
        计算了平均陀螺但未用于零偏估计。本接口为后续优化预留。

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

        参考 gnss_ins_lc_nhc MotionAligned, 本项目简化版:
        - yaw = atan2(v_E, v_N) (东向速度与北向速度)
        - pitch = 0 (初始化调整.md: 其他姿态角设为 0)
        - roll = 0

        Returns:
            (att_rpy, vel_e): 姿态和 ECEF 速度
        """
        if gnss.velocity is None:
            raise ValueError("GNSS 无速度数据, 无法执行速度矢量对准")

        vel_e = gnss.velocity.copy()
        speed = np.linalg.norm(vel_e)
        if speed < self.dynamic_speed_threshold:
            raise ValueError(
                f"GNSS 速度 {speed:.3f} m/s 低于动态速度阈值 "
                f"{self.dynamic_speed_threshold} m/s"
            )

        # ECEF 速度 → NED 速度 (cal_Ce2n 返回 NED 系: N, E, D)
        lat, lon, _ = ecef2llh(gnss.position)
        C_e_n = cal_Ce2n(lat, lon)
        vel_n = C_e_n @ vel_e  # vel_n[0]=North, vel_n[1]=East, vel_n[2]=Down

        # 初始化调整.md: yaw = atan2(v_E, v_N), 其他姿态角设为 0
        roll = 0.0
        pitch = 0.0
        yaw = math.atan2(vel_n[1], vel_n[0])  # atan2(East, North) — 航向角

        att_rpy = np.array([roll, pitch, yaw], dtype=np.float64)
        return att_rpy, vel_e

    def _align_motion_displacement(self, gnss: GnssSolution
                                   ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """位置差分对准: 3 历元缓冲, 最新两个差分计算速度。

        参考 gnss_ins_lc_nhc InterpolateGnssVel + MotionAligned。

        Returns:
            (att_rpy, vel_e) 或 None (缓冲区未满或未达运动阈值)
        """
        self.gnss_buffer.append(gnss)

        if len(self.gnss_buffer) < self.gnss_buffer_size:
            return None  # 缓冲区未满, 延迟初始化

        # 用最新两个历元计算速度
        gnss_prev = self.gnss_buffer[-2]
        gnss_curr = self.gnss_buffer[-1]
        dt = gnss_curr.timestamp - gnss_prev.timestamp
        if dt <= 0:
            return None

        vel_e = (gnss_curr.position - gnss_prev.position) / dt
        speed = np.linalg.norm(vel_e)
        if speed < self.dynamic_speed_threshold:
            return None  # 未达到动态速度阈值

        # 用差分速度走速度矢量对准
        gnss_with_vel = GnssSolution(
            timestamp=gnss_curr.timestamp,
            week=gnss_curr.week,
            position=gnss_curr.position.copy(),
            quality=gnss_curr.quality,
            num_sv=gnss_curr.num_sv,
            sd=gnss_curr.sd.copy(),
            cov=gnss_curr.cov,
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
        - IMU 比例因子: 从 config 读取 initial_gyro_scale / initial_acce_scale
                        (ppm → dimensionless), 参考 gnss_ins_lc_nhc
                        StartAligning 行 81-100
        """
        ins_cfg = self.config.get("ins", {})

        # 静态模式使用 GNSS 历史平均位置 (低精度模式, 初始化调整.md)
        if mode == InitMode.STATIC and self._static_pos_mean is not None:
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

        # 单位转换常量 (与 gnss_ins_lc_nhc constant.hpp 行 20-36 一致)
        constant_g0 = 9.7803267715
        dh2rs = math.pi / 180.0 / 3600.0       # deg/hour → rad/s
        constant_mgal = 1e-6 * constant_g0      # mGal → m/s² (gnss_ins_lc_nhc 定义)
        constant_ppm = 1e-6                      # ppm → dimensionless

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

        # 比例因子: 从 config 读取 (gnss_ins_lc_nhc StartAligning 行 90-100)
        gyro_scale_ppm = np.array(
            ins_cfg.get("initial_gyro_scale", [0.0, 0.0, 0.0]),
            dtype=np.float64,
        )
        accel_scale_ppm = np.array(
            ins_cfg.get("initial_acce_scale", [0.0, 0.0, 0.0]),
            dtype=np.float64,
        )
        gyro_scale = gyro_scale_ppm * constant_ppm
        accel_scale = accel_scale_ppm * constant_ppm

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

        return InsState(
            timestamp=gnss.timestamp,
            pos_e=pos_e,
            vel_e=vel_e,
            C_b_e=C_b_e,
            q_b_e=q_b_e,
            att_rpy=att_rpy,
            gyro_bias=gyro_bias,
            accel_bias=accel_bias,
            gyro_scale=gyro_scale,
            accel_scale=accel_scale,
            imu_angle=imu_angle,
            imu_leverarm=imu_leverarm,
            leverarm=leverarm,
        )

    def _set_initial_variance(self, mode: InitMode
                              ) -> Tuple[np.ndarray, np.ndarray]:
        """装配初始协方差 P1/P2 (初始化.md 第 12 节)。

        P1: 主滤波 15x15 [pos(3), vel(3), att(3), gyro_bias(3), accel_bias(3)]
        P2: NHC 子滤波 5x5 [imu_angle(2), imu_leverarm(3)]

        Note: mode 参数保留用于未来按模式调整协方差。
        """
        ins_cfg = self.config.get("ins", {})

        # 初始不确定度 (1σ, SI 单位)
        pos_std = np.array(ins_cfg.get("initial_pos_std_si",
                                       [30.0, 30.0, 30.0]), dtype=np.float64)
        vel_std = np.array(ins_cfg.get("initial_vel_std_si",
                                       [10.0, 10.0, 10.0]), dtype=np.float64)
        att_std = np.array(ins_cfg.get("initial_att_std_si",
                                       [0.00524, 0.00524, 0.00524]),
                           dtype=np.float64)
        gyro_bias_std = np.array(ins_cfg.get("gyro_bias_std_si",
                                             [2.424e-5, 2.424e-5, 2.424e-5]),
                                 dtype=np.float64)
        acce_bias_std = np.array(ins_cfg.get("acce_bias_std_si",
                                             [0.0489, 0.0489, 0.0489]),
                                 dtype=np.float64)

        # P1: 15x15 对角矩阵
        P1 = np.diag(np.concatenate([
            pos_std ** 2,
            vel_std ** 2,
            att_std ** 2,
            gyro_bias_std ** 2,
            acce_bias_std ** 2,
        ]))

        # P2: 5x5 (imu_angle[2] + imu_leverarm[3])
        imu_angle_std = np.array(ins_cfg.get("imu_angle_std", [10.0, 10.0]),
                                 dtype=np.float64)
        imu_angle_std = np.radians(imu_angle_std)
        imu_leverarm_std = np.array(ins_cfg.get("imu_leverarm_std",
                                                [1.0, 1.0, 1.0]),
                                    dtype=np.float64)
        P2 = np.diag(np.concatenate([
            imu_angle_std ** 2,
            imu_leverarm_std ** 2,
        ]))

        return P1, P2

    def reset_gnss_buffer(self):
        """清空 GNSS 历元缓冲区 (reboot 时调用)。"""
        self.gnss_buffer.clear()
        self.gnss_position_buffer.clear()
        self._static_pos_mean = None
        self._calibrated_gyro_bias = None
        self._calibrated_accel_bias = None
