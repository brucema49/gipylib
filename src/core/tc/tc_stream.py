"""紧组合导航流式运行器。

类似 LcStream，但使用 TcIntegration 替代 LcIntegration。
消费 IMU + 原始 GNSS 观测 (obsr, obsb, nav)，流式输出 .rslt。

流程:
1. 初始化前：小缓冲累积 IMU+GNSS 原始观测，每个新 GNSS 到来时尝试初始化
2. 初始化成功：回放缓冲数据，然后切换增量模式
3. 增量模式：feed_imu 做 time_update + per-IMU 写出 (.rslt 一行)
   feed_gnss_raw 入 pending 队列（由后续 IMU 跨越 gnss.t 时 GVINS 风格触发）
4. 输出：per-IMU (100Hz)，每条 IMU 后立即写一行
"""
import logging
from typing import List, Optional

from src.core.data_types import ImuMeasurement
from src.core.tc.tc_integration import TcIntegration

logger = logging.getLogger(__name__)


class TcStream:
    """流式紧组合导航：增量喂入 IMU+原始GNSS观测，per-IMU 流式输出 .rslt。

    与 LcStream 的区别：
    - 使用 TcIntegration (原始观测融合) 替代 LcIntegration (GnssSolution 融合)
    - feed_gnss 接收原始观测 (obsr, obsb, nav) 而非 GnssSolution
    - 初始化由 TcIntegration._try_init 内部完成 (SPP 粗定位)
    """

    def __init__(self, config: dict, writer):
        self.config = config
        self.writer = writer
        self._tc_mode = config.get("gnss", {}).get("positioning_mode", "spp")
        self._integ = TcIntegration(config, mode=self._tc_mode)

        # 初始化前缓冲
        self._init_imu: List[ImuMeasurement] = []
        self._init_obs: list = []   # [(obsr, obsb, nav, t_gnss), ...]
        self._initialized = False
        self._output_count = 0

    @property
    def initialized(self) -> bool:
        return self._initialized

    def open(self) -> None:
        self.writer.open()

    def close(self) -> None:
        self.writer.close()

    # ===== 增量喂入 =====

    def feed_imu(self, imu: ImuMeasurement) -> None:
        """喂入 IMU。初始化前缓冲，初始化后委托 TcIntegration。"""
        if not self._initialized:
            self._init_imu.append(imu)
            # 当有 GNSS obs 且当前 IMU 时间戳 >= GNSS 时间戳时, 尝试初始化
            if self._init_obs and imu.timestamp >= self._init_obs[-1][3]:
                self._try_init()
            return
        self._integ.add_imu(imu)
        self._write_state(self._integ.last_qins)

    def feed_gnss_raw(self, obsr, obsb, nav) -> None:
        """喂入原始 GNSS 观测。初始化前缓冲，初始化后委托 TcIntegration。"""
        if not self._initialized:
            t_gnss = float(obsr.t.time + obsr.t.sec)
            self._init_obs.append((obsr, obsb, nav, t_gnss))
            # 若 IMU 已覆盖第一个 GNSS obs 时间, 尝试初始化
            if self._init_imu and self._init_obs:
                first_t = self._init_obs[0][3]
                if self._init_imu[-1].timestamp >= first_t:
                    self._try_init()
            return
        self._integ.add_gnss(obsr, obsb, nav)

    # ===== 初始化 =====

    def _try_init(self) -> None:
        """委托 TcIntegration._try_init 完成初始化。"""
        # 把缓冲数据填入 TcIntegration 的内部缓冲
        self._integ._init_imu = list(self._init_imu)
        self._integ._init_obs = list(self._init_obs)
        self._integ._try_init()

        if self._integ.initialized:
            self._initialized = True
            logger.info(
                f"TcStream 初始化成功: mode={self._tc_mode}, "
                f"pos={self._integ.state.pos_e}")
            # 清空初始化缓冲 (TcIntegration._replay_buffer 已回放)
            self._init_imu.clear()
            self._init_obs.clear()

    # ===== 输出 =====

    def _write_state(self, qins: int) -> None:
        """per-IMU 写出 .rslt 一行。"""
        state = self._integ.state
        P = self._integ.P
        si = self._integ.si
        if state is None or P is None or si is None:
            return
        num_sv = self._integ._last_ns if hasattr(self._integ, '_last_ns') else 0
        q = self._integ._last_q if hasattr(self._integ, '_last_q') else 5
        self.writer.write(state, P, si, q, qins, num_sv)
        self._output_count += 1
