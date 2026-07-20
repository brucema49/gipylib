"""松组合导航流式运行器。

替代 LcRunner 的批处理架构。增量喂入 IMU+GNSS，流式输出 RTKLC.rslt。

流程:
1. 初始化前：小缓冲累积 IMU+GNSS，每个新 GNSS 到来时尝试初始化
2. 初始化成功：回放缓冲中 init 时间戳之后的事件，然后切换增量模式
3. 增量模式：feed_imu 做时间更新 + per-IMU 写出 (.rslt 一行)
   feed_gnss 入 pending 队列（由后续 IMU 跨越 gnss.t 时 GVINS 风格触发）
4. 输出：per-IMU (100Hz)，每条 IMU 后立即写一行（Qins 跟踪本历元量测更新类型）

TC 扩展点：当前 GNSS 由上游 RTKLIB 预解算后入队。未来 TC 时，
把"从队列取预解算 GNSS"替换为"用 INS 先验调用 GNSS 解算"，
事件循环骨架不变。
"""
import logging
import math
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

    初始化前用小缓冲累积数据（到找到 init 历元为止），初始化后增量处理。
    输出 per-IMU (100Hz)，每条 IMU 后立即写一行 .rslt。
    """

    def __init__(self, config: dict, writer):
        self.config = config
        self.writer = writer
        self.initializer = InsInitializer(config)
        self._init_mode = self._select_init_mode(config)

        # 初始化前缓冲（小，到找到 init 历元为止）
        self._init_imu: List[ImuMeasurement] = []
        self._init_gnss: List[GnssSolution] = []

        # 初始化后状态
        self._initialized = False
        self._est: Optional[LcEstimator] = None
        self._integ: Optional[LcIntegration] = None
        self._si: Optional[StateIndex] = None
        self._last_gnss_ns = 0
        self._last_q = 5  # 最近 GNSS quality (初值 5=SPP, feed_gnss 时更新)
        self._output_count = 0

    def open(self) -> None:
        self.writer.open()

    def close(self) -> None:
        self.writer.close()

    @staticmethod
    def _select_init_mode(config: dict) -> InitMode:
        ins_cfg = config.get("ins", {})
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
        """喂入 IMU。初始化前缓冲，初始化后做 time_update + per-IMU 写出。"""
        if not self._initialized:
            self._init_imu.append(imu)
            # 当有 GNSS 且当前 IMU 时间戳 >= GNSS 时间戳时，尝试初始化
            # （此时 before + after IMU 都已就绪）
            if self._init_gnss and imu.timestamp >= self._init_gnss[-1].timestamp:
                self._try_init()
            return
        self._integ.add_imu(imu)
        self._write_state(self._integ.last_qins)

    def feed_gnss(self, gnss: GnssSolution) -> None:
        """喂入 GNSS。初始化前仅缓冲，初始化后入 pending 队列。

        初始化由 feed_imu 在 after-IMU 到达时触发（确保 before+after 都就绪）。
        GNSS 量测更新由后续 IMU 跨越 gnss.t 时 GVINS 风格触发 (LcIntegration.add_imu)。
        """
        if not self._initialized:
            self._init_gnss.append(gnss)
            return
        self._integ.add_gnss(gnss)
        self._last_gnss_ns = gnss.num_sv
        self._last_q = gnss.quality

    # ===== 初始化 =====

    def _try_init(self) -> None:
        """用当前缓冲数据尝试初始化。成功则回放缓冲数据并切换增量模式。"""
        if not self._init_gnss:
            return

        gnss = self._init_gnss[-1]
        t_gnss = gnss.timestamp
        imu_block = [imu for imu in self._init_imu
                     if t_gnss - 2.0 <= imu.timestamp <= t_gnss + 1.0]
        if len(imu_block) < 2:
            return

        has_before = any(imu.timestamp <= t_gnss for imu in imu_block)
        has_after = any(imu.timestamp >= t_gnss for imu in imu_block)
        if not (has_before and has_after):
            return

        cfg = self.config["ins"]
        angular_thr = cfg.get("angular_velocity_threshold_deg", 30.0)
        if angular_thr > math.pi:
            angular_thr = math.radians(angular_thr)
        gyro_norms = [float(np.linalg.norm(imu.gyro)) for imu in imu_block]
        if float(np.mean(gyro_norms)) >= angular_thr:
            return

        dynamic_thr = cfg.get("dynamic_speed_threshold", 4.0)
        static_thr = cfg.get("static_speed_threshold", 0.5)
        block = AlignedBlock(gnss=gnss, imu_list=imu_block)

        # 1) 尝试配置的动态模式
        init_state = None
        init_P = None
        speed = self._check_init_speed(gnss, dynamic_thr)
        if speed is not None:
            try:
                init_state, init_P = self.initializer.initialize(
                    block, self._init_mode)
                logger.info(
                    f"LcStream 初始化成功: t={init_state.timestamp:.3f}, "
                    f"speed={speed:.3f} m/s, mode={self._init_mode.value}")
            except ValueError:
                pass

        # 2) 回退：静态模式
        if init_state is None:
            gnss_speed = (float(np.linalg.norm(gnss.velocity))
                          if gnss.velocity is not None else 0.0)
            if gnss_speed < static_thr:
                try:
                    init_state, init_P = self.initializer.initialize(
                        block, InitMode.STATIC)
                    logger.info(
                        f"LcStream 初始化成功 (静态回退): "
                        f"t={init_state.timestamp:.3f}, "
                        f"gnss_speed={gnss_speed:.3f} m/s")
                except ValueError:
                    return
            else:
                return

        if init_state is None:
            return

        # 初始化成功，创建估计器 + 集成器
        self._est = LcEstimator(init_state, init_P, self.config)
        self._integ = LcIntegration(self._est, self.config)
        self._si = StateIndex.from_config(self.config)
        self._initialized = True
        # 初始化用 GNSS 的 quality / num_sv 作为后续 per-IMU 输出的最近 GNSS 元数据
        self._last_q = gnss.quality
        self._last_gnss_ns = gnss.num_sv

        # 回放缓冲中 init 时间戳之后的事件
        init_gnss_idx = len(self._init_gnss) - 1
        self._replay_buffer(init_state.timestamp, init_gnss_idx)

        # 清空初始化缓冲
        self._init_imu.clear()
        self._init_gnss.clear()

    def _replay_buffer(self, init_ts: float, init_gnss_idx: int) -> None:
        """回放初始化缓冲中 init_ts 之后的事件。"""
        events = []
        for imu in self._init_imu:
            if imu.timestamp > init_ts:
                events.append((imu.timestamp, "imu", imu))
        for j in range(init_gnss_idx + 1, len(self._init_gnss)):
            gnss = self._init_gnss[j]
            if gnss.timestamp > init_ts:
                events.append((gnss.timestamp, "gnss", gnss))
        # 同时间戳时 GNSS 先于 IMU
        events.sort(key=lambda e: (e[0], 0 if e[1] == "gnss" else 1))

        for _, tag, data in events:
            if tag == "imu":
                self._integ.add_imu(data)
                self._write_state(self._integ.last_qins)
            else:
                self._integ.add_gnss(data)
                self._last_gnss_ns = data.num_sv
                self._last_q = data.quality

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
        self._output_count += 1

    def finalize(self) -> int:
        """流式结束，返回总输出数。"""
        if not self._initialized:
            logger.warning("LcStream: 未初始化，无 LC 输出")
            return 0
        logger.info(f"LcStream 输出: {self._output_count} 历元")
        return self._output_count
