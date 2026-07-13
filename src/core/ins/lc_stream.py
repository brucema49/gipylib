"""松组合导航流式运行器。

替代 LcRunner 的批处理架构。增量喂入 IMU+GNSS，流式输出 RTKLC.pos。

流程:
1. 初始化前：小缓冲累积 IMU+GNSS，每个新 GNSS 到来时尝试初始化
2. 初始化成功：回放缓冲中 init 时间戳之后的事件，然后切换增量模式
3. 增量模式：feed_imu 做时间更新，feed_gnss 入 pending 队列（由后续 IMU 驱动处理）
4. 输出：1 秒滑动窗口，到 N+1 秒首事件时刷 N 秒（保留 post-GNSS 状态语义）

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
from src.core.time_utils import unix_to_gpst, gpst_to_unix

logger = logging.getLogger(__name__)

_OUTPUT_SEC_TOLERANCE = 0.006  # IMU 时间戳偏移容忍量 (s)


class LcStream:
    """流式松组合导航：增量喂入 IMU+GNSS，流式输出 RTKLC.pos。

    初始化前用小缓冲累积数据（到找到 init 历元为止），初始化后增量处理。
    输出用 1 秒滑动窗口（到 N+1 秒首事件时刷 N 秒），保留 post-GNSS 状态语义。
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
        self._output_count = 0

        # 输出缓冲：(week, int_sec) -> GnssSolution（每秒保留最后一次状态）
        self._pending: dict = {}

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
        """喂入 IMU。初始化前缓冲，初始化后做时间更新 + 刷出已完成秒。"""
        if not self._initialized:
            self._init_imu.append(imu)
            # 当有 GNSS 且当前 IMU 时间戳 >= GNSS 时间戳时，尝试初始化
            # （此时 before + after IMU 都已就绪）
            if self._init_gnss and imu.timestamp >= self._init_gnss[-1].timestamp:
                self._try_init()
            return
        self._integ.add_imu(imu)
        self._buffer_output()
        self._flush_completed(imu.timestamp)

    def feed_gnss(self, gnss: GnssSolution) -> None:
        """喂入 GNSS。初始化前仅缓冲，初始化后入 pending 队列。

        初始化由 feed_imu 在 after-IMU 到达时触发（确保 before+after 都就绪）。
        """
        if not self._initialized:
            self._init_gnss.append(gnss)
            return
        self._integ.add_gnss(gnss)
        self._last_gnss_ns = gnss.num_sv
        self._buffer_output()
        self._flush_completed(gnss.timestamp)

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
            else:
                self._integ.add_gnss(data)
                self._last_gnss_ns = data.num_sv
            self._buffer_output()
            self._flush_completed(data.timestamp)

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

    # ===== 输出缓冲 =====

    def _buffer_output(self) -> None:
        """将当前状态缓冲到对应整数秒（保留最后一次状态）。"""
        state = self._est.state
        week, sec = unix_to_gpst(state.timestamp)
        if abs(sec - round(sec)) < _OUTPUT_SEC_TOLERANCE:
            int_sec = round(sec)
            key = (week, int_sec)
            self._pending[key] = self._state_to_sol(
                state, self._est.P, self._si, self._last_gnss_ns)

    def _flush_completed(self, current_ts: float) -> None:
        """刷出已过输出窗口的整数秒（写入文件）。"""
        done_keys = []
        for (week, int_sec) in self._pending:
            key_ts = gpst_to_unix(week, int_sec)
            if key_ts + _OUTPUT_SEC_TOLERANCE < current_ts:
                done_keys.append((week, int_sec))
        for k in sorted(done_keys):
            self.writer.write(self._pending.pop(k))
            self._output_count += 1

    def finalize(self) -> int:
        """流式结束，刷出剩余 pending，返回总输出数。"""
        if not self._initialized:
            logger.warning("LcStream: 未初始化，无 LC 输出")
            return 0
        for k in sorted(self._pending.keys()):
            self.writer.write(self._pending[k])
            self._output_count += 1
        self._pending.clear()
        logger.info(f"LcStream 输出: {self._output_count} 历元")
        return self._output_count

    @staticmethod
    def _state_to_sol(state: InsState, P: np.ndarray, si: StateIndex,
                      num_sv: int) -> GnssSolution:
        """InsState + P → GnssSolution (供 SolutionWriter 输出)。"""
        pos_idx = si.pos
        pos_cov = P[pos_idx:pos_idx+3, pos_idx:pos_idx+3]
        sd = np.sqrt(np.abs(np.diag(pos_cov)))
        week, _ = unix_to_gpst(state.timestamp)
        return GnssSolution(
            timestamp=state.timestamp,
            week=week,
            position=state.pos_e.copy(),
            quality=1,  # LC filtered (FIX-like)
            num_sv=num_sv,
            sd=sd,
            cov=pos_cov,
        )
