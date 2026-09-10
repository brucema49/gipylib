"""松组合导航流式运行器。

替代 LcRunner 的批处理架构。增量喂入 IMU+GNSS，流式输出 RTKLC.rslt。

流程:
1. 初始化前：每个 GNSS 到来时输出纯 GNSS (Qins=0, 1Hz), IMU 仅保留最新一条
   (不解算, 不缓冲)。每个 GNSS 到来时尝试动态初始化 (position_diff / velocity_vector)。
2. 初始化成功：切换增量模式, 后续 IMU 喂入 LcIntegration
3. 增量模式：feed_imu 做时间更新 + per-IMU 写出 (.rslt 一行)
   feed_gnss 入 pending 队列（由后续 IMU 跨越 gnss.t 时 GVINS 风格触发）
4. 输出：per-IMU (100Hz)，每条 IMU 后立即写一行（Qins 跟踪本历元量测更新类型）

参考 issue/7-30松组合.md:
  - "还未初始化时, 一条一条的弹出GNSS观测进行纯GNSS解算, imu数据直接弹出, 不用进行解算"
  - "缓存5个动态的GNSS历元结果, 通过首尾位置差分得到大致的平面速度"
  - 不使用静态回退初始化 (yaw=0 会导致偏航角发散)
"""
import json
import logging
import math
from pathlib import Path
from typing import List, Optional

import numpy as np

from src.core.data_types import AlignedBlock, GnssSolution, ImuMeasurement, InsState
from src.core.ins.initializer import InsInitializer, InitMode
from src.core.ins.lc_estimator import LcEstimator
from src.core.ins.lc_integration import LcIntegration
from src.core.ins.state_index import StateIndex
from src.core.time_utils import unix_to_gpst

logger = logging.getLogger(__name__)


class LcStream:
    """流式松组合导航：增量喂入 IMU+GNSS，per-IMU 流式输出 RTKLC.rslt。

    预初始化阶段输出纯 GNSS (Qins=0, 1Hz), IMU 不解算。
    动态初始化 (position_diff / velocity_vector) 成功后切换增量模式。
    """

    def __init__(self, config: dict, writer, stat_writer=None):
        self.config = config
        self.writer = writer
        self.stat_writer = stat_writer
        self.initializer = InsInitializer(config)
        self._init_mode = self._select_init_mode(config)

        # 预初始化阶段: 仅缓冲 GNSS (供位置差分), 仅保留最新 IMU (供陀螺检测)
        self._init_gnss: List[GnssSolution] = []
        self._latest_imu: Optional[ImuMeasurement] = None

        # 初始化后状态
        self._initialized = False
        self._est: Optional[LcEstimator] = None
        self._integ: Optional[LcIntegration] = None
        self._si: Optional[StateIndex] = None
        self._last_gnss_ns = 0
        self._last_q = 5  # 最近 GNSS quality (初值 5=SPP, feed_gnss 时更新)
        self._output_count = 0

        # KF-GINS 对齐诊断输出。默认关闭；配置路径后在指定 SOW 窗口写一条
        # JSONL 记录，包含 F/Phi/Q/H/R/K 以及 GNSS 反馈增量。
        ins_cfg = config.get("ins", {})
        self._matrix_diag_path = ins_cfg.get("lc_matrix_diagnostics_path")
        self._matrix_diag_start_sow = float(
            ins_cfg.get("diagnostics_start_sow", -math.inf))
        self._matrix_diag_end_sow = float(
            ins_cfg.get("diagnostics_end_sow", math.inf))
        self._matrix_diag_file = None

    def open(self) -> None:
        self.writer.open()
        if self.stat_writer is not None:
            self.stat_writer.open()
        if self._matrix_diag_path:
            path = Path(self._matrix_diag_path)
            path.parent.mkdir(parents=True, exist_ok=True)
            self._matrix_diag_file = path.open("w", encoding="utf-8")

    def close(self) -> None:
        if self._matrix_diag_file is not None:
            self._matrix_diag_file.close()
            self._matrix_diag_file = None
        if self.stat_writer is not None:
            self.stat_writer.close()
        self.writer.close()

    @staticmethod
    def _select_init_mode(config: dict) -> InitMode:
        ins_cfg = config.get("ins", {})
        if ins_cfg.get("initialization_mode") == "fixed":
            return InitMode.FIXED
        method = ins_cfg.get("alignnment_dynamic_method", "auto")
        if method == "velocity_vector":
            return InitMode.VELOCITY_VECTOR
        if method == "position_diff":
            return InitMode.POSITION_DIFF
        pos_mode = config.get("gnss", {}).get("positioning_mode", "rtk")
        if pos_mode == "spp":
            return InitMode.VELOCITY_VECTOR
        return InitMode.POSITION_DIFF

    # ===== 增量喂入 =====

    def feed_imu(self, imu: ImuMeasurement) -> None:
        """喂入 IMU。预初始化仅保留最新一条 (不解算), 初始化后做 time_update + 写出。"""
        if not self._initialized:
            # 预初始化: IMU 直接弹出, 仅保留最新一条供初始化陀螺检测
            self._latest_imu = imu
            return
        self._integ.add_imu(imu)
        self._write_state(self._integ.last_qins)

    def feed_gnss(self, gnss: GnssSolution) -> None:
        """喂入 GNSS。预初始化输出纯 GNSS (Qins=0) + 尝试动态初始化, 初始化后入 pending 队列。"""
        if not self._initialized:
            # 预初始化: 输出纯 GNSS (Qins=0, 使用 positioning_mode 规定的模式)
            self._write_gnss_only(gnss)
            self._init_gnss.append(gnss)
            # 每个新 GNSS 到来时尝试动态初始化
            self._try_init()
            return
        self._integ.add_gnss(gnss)
        self._last_gnss_ns = gnss.num_sv
        self._last_q = gnss.quality

    # ===== 初始化 =====

    def _try_init(self) -> None:
        """用当前 GNSS 缓冲 + 最新 IMU 尝试动态初始化。

        仅动态模式 (position_diff / velocity_vector), 不使用静态回退。
        position_diff: 缓冲区满 (gnss_buffer_size 历元) 且所有相邻差分速度均超阈值。
        velocity_vector: 当前 GNSS 速度超阈值。
        """
        if not self._init_gnss:
            return

        gnss = self._init_gnss[-1]
        t_gnss = gnss.timestamp

        # 构造 imu_list (仅最新 IMU, 供陀螺角速度检测)
        imu_list = [self._latest_imu] if self._latest_imu is not None else []

        # 陀螺角速度检测 (动态初始化要求角速度较小)
        cfg = self.config["ins"]
        angular_thr = cfg.get("angular_velocity_threshold_deg", 30.0)
        if angular_thr > math.pi:
            angular_thr = math.radians(angular_thr)
        if imu_list:
            gyro_norm = float(np.linalg.norm(imu_list[0].gyro))
            if gyro_norm >= angular_thr:
                return

        block = AlignedBlock(gnss=gnss, imu_list=imu_list)
        dynamic_thr = cfg.get("dynamic_speed_threshold", 4.0)

        # 尝试动态初始化 (position_diff / velocity_vector)
        init_state = None
        init_P = None
        try:
            init_state, init_P = self.initializer.initialize(
                block, self._init_mode)
            logger.info(
                f"LcStream 动态初始化成功: t={init_state.timestamp:.3f}, "
                f"mode={self._init_mode.value}")
        except ValueError:
            pass

        if init_state is None:
            return

        # 初始化成功, 创建估计器 + 集成器
        self._est = LcEstimator(init_state, init_P, self.config)
        self._integ = LcIntegration(
            self._est, self.config, output_callback=self._write_integration_output
        )
        self._si = StateIndex.from_config(self.config)
        self._initialized = True
        # 初始化用 GNSS 的 quality / num_sv 作为后续 per-IMU 输出的最近 GNSS 元数据
        self._last_q = gnss.quality
        self._last_gnss_ns = gnss.num_sv

        # KF-GINS applies the first GNSS position update at the first IMU
        # boundary after its supplied fixed PVA.  Preserve that timing for
        # fixed initialization instead of silently dropping the seed epoch.
        if self._init_mode == InitMode.FIXED:
            self._integ.add_gnss(gnss)

        # 清空初始化缓冲
        self._init_gnss.clear()
        self._latest_imu = None

    def _check_init_speed(self, gnss: GnssSolution,
                          threshold: float) -> Optional[float]:
        """检查 GNSS 速度是否满足初始化条件。

        position_diff 模式: 不检查速度 (由 initializer 内部缓冲区处理)
        velocity_vector 模式: 检查 GNSS 速度 > 阈值
        """
        if self._init_mode == InitMode.POSITION_DIFF:
            return 0.0

        if gnss.velocity is None:
            return None

        speed = float(np.linalg.norm(gnss.velocity))
        if speed < threshold:
            return None
        return speed

    # ===== per-IMU 输出 =====

    def _write_integration_output(self, state, P, qins: int) -> None:
        """Write a state emitted at an interpolated GNSS boundary."""
        self.writer.write(
            state=state,
            P=P,
            si=self._si,
            q=self._last_q,
            qins=qins,
            num_sv=self._last_gnss_ns,
        )
        if self.stat_writer is not None:
            self.stat_writer.write(
                state=state, P=P, si=self._si, q=self._last_q,
                qins=qins, num_sv=self._last_gnss_ns,
            )
        self._write_matrix_diagnostic(state, qins)
        self._output_count += 1

    @staticmethod
    def _diag_value(value):
        """Convert numpy/scalar values into JSON-safe diagnostic values."""
        if isinstance(value, np.ndarray):
            return value.tolist()
        if isinstance(value, (np.floating, np.integer)):
            return value.item()
        if isinstance(value, dict):
            return {key: LcStream._diag_value(val) for key, val in value.items()}
        if isinstance(value, (list, tuple)):
            return [LcStream._diag_value(val) for val in value]
        return value

    def _write_matrix_diagnostic(self, state, qins: int) -> None:
        """Write matrix/feedback diagnostics at the configured GNSS epoch."""
        if self._matrix_diag_file is None or qins != 3 or self._est is None:
            return
        week, sow = unix_to_gpst(float(state.timestamp))
        if not (self._matrix_diag_start_sow <= sow < self._matrix_diag_end_sow):
            return
        diag = {
            "week": int(week),
            "sow": float(sow),
            "timestamp": float(state.timestamp),
            "state": {
                "pos_e": state.pos_e.copy(),
                "vel_e": state.vel_e.copy(),
                "gyro_bias": state.gyro_bias.copy(),
                "accel_bias": state.accel_bias.copy(),
                "gyro_scale": state.gyro_scale.copy(),
                "accel_scale": state.accel_scale.copy(),
            },
            "P": self._est.P.copy(),
            "time_update": self._est.last_time_update_diag,
            # 一个 GNSS 历元通常包含位置、速度两次更新；完整保留最近更新序列。
            "measurement_updates": self._est.meas_update_diags[-4:],
            "last_measurement_update": self._est.last_meas_update_diag,
            "feedback_x": self._est.last_feedback_x.copy(),
            "delta_bg": self._est.last_feedback_x[
                self._est.si.gyro_bias:self._est.si.gyro_bias + 3].copy(),
            "delta_ba": self._est.last_feedback_x[
                self._est.si.accel_bias:self._est.si.accel_bias + 3].copy(),
            "delta_scale": {
                "gyro": (self._est.last_feedback_x[
                    self._est.si.gyro_scale:self._est.si.gyro_scale + 3].copy()
                    if self._est.si.has_imu_scale() else np.zeros(3)),
                "accel": (self._est.last_feedback_x[
                    self._est.si.accel_scale:self._est.si.accel_scale + 3].copy()
                    if self._est.si.has_imu_scale() else np.zeros(3)),
            },
        }
        self._matrix_diag_file.write(
            json.dumps(self._diag_value(diag), separators=(",", ":")) + "\n")
        self._matrix_diag_file.flush()

    def _write_state(self, qins: int) -> None:
        """写当前状态到 .rslt 文件 (per-IMU 100Hz)。"""
        self.writer.write(
            state=self._est.state,
            P=self._est.P,
            si=self._si,
            q=self._last_q,
            qins=qins,
            num_sv=self._last_gnss_ns,
        )
        if self.stat_writer is not None:
            self.stat_writer.write(
                state=self._est.state, P=self._est.P, si=self._si,
                q=self._last_q, qins=qins, num_sv=self._last_gnss_ns,
            )
        self._output_count += 1

    def _write_gnss_only(self, gnss: GnssSolution) -> None:
        """预初始化阶段输出纯 GNSS 解 (Qins=0, 速度=0, 姿态=0)。"""
        pos_sd = gnss.sd if gnss.sd is not None else None
        self.writer.write_gnss_only(
            timestamp=gnss.timestamp,
            pos_e=gnss.position,
            q=gnss.quality,
            num_sv=gnss.num_sv,
            pos_sd=pos_sd,
        )
        if self.stat_writer is not None:
            self.stat_writer.write_gnss_only(
                timestamp=gnss.timestamp,
                rr=gnss.position,
                quality=gnss.quality,
                ns=gnss.num_sv,
                pos_sd=pos_sd if gnss.sd is not None else np.full(3, 10.0),
            )
        self._output_count += 1

    def finalize(self) -> int:
        """流式结束，返回总输出数。"""
        if not self._initialized:
            logger.warning("LcStream: 未初始化，仅输出纯 GNSS 结果")
        logger.info(f"LcStream 输出: {self._output_count} 历元")
        return self._output_count
