"""松组合导航批处理运行器。

收集 IMU + GNSS 数据后批量执行 LC EKF, 输出松组合定位结果到 SolutionWriter。
复用 test_lc_e2e.py 验证过的初始化 + LcIntegration 主循环逻辑。

流程:
1. 根据 config 选择初始化模式 (auto/velocity_vector/position_diff)
2. 等待 GNSS 速度超阈值, 构建 AlignedBlock 初始化 INS
3. 按时间戳交错处理 IMU + GNSS, 运行 LcIntegration
4. 整数秒输出 LC 状态 (InsState+P → GnssSolution → SolutionWriter)
"""
import logging
import math
from typing import List, Optional, Tuple

import numpy as np

from src.core.data_types import AlignedBlock, GnssSolution, ImuMeasurement, InsState
from src.core.ins.initializer import InsInitializer, InitMode
from src.core.ins.lc_estimator import LcEstimator
from src.core.ins.lc_integration import LcIntegration
from src.core.ins.state_index import StateIndex
from src.core.time_utils import unix_to_gpst

logger = logging.getLogger(__name__)

_OUTPUT_SEC_TOLERANCE = 0.006  # IMU 时间戳偏移容忍量 (s)


class LcRunner:
    """松组合导航批处理运行器。

    收集 IMU + GNSS 列表, 批量运行 LC EKF, 输出到 SolutionWriter。
    """

    def __init__(self, config: dict, lc_writer):
        self.config = config
        self.writer = lc_writer
        self.initializer = InsInitializer(config)
        self._init_mode = self._select_init_mode(config)

    @staticmethod
    def _select_init_mode(config: dict) -> InitMode:
        """根据 config 选择初始化模式。

        auto: RTK → position_diff, SPP → velocity_vector (项目约定)
        velocity_vector / position_diff: 直接使用
        """
        ins_cfg = config.get("ins", {})
        method = ins_cfg.get("alignnment_dynamic_method", "auto")
        if method == "velocity_vector":
            return InitMode.VELOCITY_VECTOR
        if method == "position_diff":
            return InitMode.POSITION_DIFF
        # auto: 按 positioning_mode 选择
        pos_mode = config.get("gnss", {}).get("positioning_mode", "rtk")
        if pos_mode == "spp":
            return InitMode.VELOCITY_VECTOR
        return InitMode.POSITION_DIFF

    def run(self, imu_list: List[ImuMeasurement],
            gnss_list: List[GnssSolution]) -> int:
        """批量运行 LC EKF, 输出松组合结果。

        Args:
            imu_list: 全部 IMU 测量 (按时间排序)
            gnss_list: 全部 GNSS 解 (按时间排序)

        Returns:
            输出的历元数
        """
        if not imu_list or not gnss_list:
            logger.warning("LcRunner: IMU 或 GNSS 数据为空, 跳过 LC")
            return 0

        self.writer.open()
        try:
            init_state, init_P, init_idx = self._initialize(
                imu_list, gnss_list)
            if init_state is None:
                logger.error("LcRunner: 初始化失败, 无法运行 LC EKF")
                return 0

            count = self._run_lc(imu_list, gnss_list,
                                 init_state, init_P, init_idx)
            return count
        finally:
            self.writer.close()

    def _initialize(self, imu_list: List[ImuMeasurement],
                    gnss_list: List[GnssSolution]
                    ) -> Tuple[Optional[InsState], Optional[np.ndarray], int]:
        """初始化 INS: 先尝试配置模式 (position_diff), 失败则回退静态模式。

        静态模式在静止段即可初始化 (位置平均 + 零姿态),
        position_diff 需要运动速度超阈值。

        Returns:
            (init_state, init_P, gnss_index) 或 (None, None, -1)
        """
        cfg = self.config["ins"]
        dynamic_thr = cfg.get("dynamic_speed_threshold", 4.0)
        angular_thr = cfg.get("angular_velocity_threshold_deg", 30.0)
        if angular_thr > math.pi:
            angular_thr = math.radians(angular_thr)
        static_thr = cfg.get("static_speed_threshold", 0.5)

        for i, gnss in enumerate(gnss_list):
            t_gnss = gnss.timestamp
            imu_block = [imu for imu in imu_list
                         if t_gnss - 2.0 <= imu.timestamp <= t_gnss + 1.0]
            if len(imu_block) < 2:
                continue

            has_before = any(imu.timestamp <= t_gnss for imu in imu_block)
            has_after = any(imu.timestamp >= t_gnss for imu in imu_block)
            if not (has_before and has_after):
                continue

            gyro_norms = [float(np.linalg.norm(imu.gyro)) for imu in imu_block]
            if float(np.mean(gyro_norms)) >= angular_thr:
                continue

            block = AlignedBlock(gnss=gnss, imu_list=imu_block)

            # 1) 尝试配置的动态模式 (position_diff / velocity_vector)
            speed = self._check_init_speed(gnss, dynamic_thr)
            if speed is not None:
                try:
                    state, P = self.initializer.initialize(
                        block, self._init_mode)
                    logger.info(
                        f"LcRunner 初始化成功: t={state.timestamp:.3f}, "
                        f"speed={speed:.3f} m/s, mode={self._init_mode.value}, "
                        f"gnss_epoch={i}")
                    return state, P, i
                except ValueError:
                    pass

            # 2) 回退: 静态模式 (静止段可用, 位置平均 + 零姿态)
            gnss_speed = (float(np.linalg.norm(gnss.velocity))
                          if gnss.velocity is not None else 0.0)
            if gnss_speed < static_thr:
                try:
                    state, P = self.initializer.initialize(
                        block, InitMode.STATIC)
                    logger.info(
                        f"LcRunner 初始化成功 (静态回退): "
                        f"t={state.timestamp:.3f}, "
                        f"gnss_speed={gnss_speed:.3f} m/s, gnss_epoch={i}")
                    return state, P, i
                except ValueError:
                    continue

        return None, None, -1

    def _check_init_speed(self, gnss: GnssSolution,
                          threshold: float) -> Optional[float]:
        """检查 GNSS 速度是否满足初始化条件。

        position_diff 模式: 不检查速度 (由 initializer 内部缓冲区处理)
        velocity_vector 模式: 检查 GNSS 速度 > 阈值
        """
        if self._init_mode == InitMode.POSITION_DIFF:
            # position_diff 内部维护缓冲区, 速度由差分计算
            # 只要 GNSS 不是完全静止就尝试 (initializer 内部判断)
            return 0.0  # 占位, 实际速度由 initializer 计算

        if gnss.velocity is None:
            return None

        speed = float(np.linalg.norm(gnss.velocity))
        if speed < threshold:
            return None
        return speed

    def _run_lc(self, imu_list: List[ImuMeasurement],
                gnss_list: List[GnssSolution],
                init_state: InsState, init_P: np.ndarray,
                init_gnss_idx: int) -> int:
        """运行 LC EKF 主循环, 输出整数秒状态。

        Returns:
            输出历元数
        """
        est = LcEstimator(init_state, init_P, self.config)
        integ = LcIntegration(est, self.config)
        si = StateIndex.from_config(self.config)

        # 构建事件列表 (IMU + GNSS, 按时间戳排序)
        # 同时间戳时 GNSS 先于 IMU: GNSS 入队 pending 后, 紧随的 IMU post-advance
        # 立即处理该 GNSS 更新, 整数秒输出反映 post-GNSS 状态
        events = [(imu.timestamp, "imu", imu) for imu in imu_list
                  if imu.timestamp > init_state.timestamp]
        for j in range(init_gnss_idx + 1, len(gnss_list)):
            gnss = gnss_list[j]
            if gnss.timestamp > init_state.timestamp:
                events.append((gnss.timestamp, "gnss", gnss))
        events.sort(key=lambda e: (e[0], 0 if e[1] == "gnss" else 1))

        output_count = 0
        output_buffer: dict = {}
        last_gnss_ns = 0

        for _, tag, data in events:
            if tag == "imu":
                integ.add_imu(data)
            else:
                integ.add_gnss(data)
                last_gnss_ns = data.num_sv

            state = est.state
            week, sec = unix_to_gpst(state.timestamp)
            if abs(sec - round(sec)) < _OUTPUT_SEC_TOLERANCE:
                int_sec = round(sec)
                key = (week, int_sec)
                output_buffer[key] = self._state_to_sol(
                    state, est.P, si, last_gnss_ns)

        for key in sorted(output_buffer.keys()):
            self.writer.write(output_buffer[key])
            output_count += 1

        logger.info(f"LcRunner 输出: {output_count} 历元 "
                    f"(事件总数={len(events)})")
        return output_count

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
