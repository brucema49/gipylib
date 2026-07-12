"""松组合导航集成 (最近邻时间对齐 + 主循环)。

参考:
- KF-GINS newImuProcess (imupre/imucur/pending_gnss 结构)
- ignav postpos.cc (NHC/ZUPT/ZARU per-IMU 触发 + decimation)
- 项目约定: 最近邻匹配 (替换 KF-GINS 增量切分, 适配速率式 IMU)

主循环 (每个新 IMU 到来时):
  1. 预推进检查: 若 pending GNSS 更近于当前 imucur (即旧 imucur),
     用当前 state (imucur 时刻) 做量测更新 + 反馈, 然后出队。
  2. 推进: imupre←imucur, imucur←imu, time_update。
  3. 约束更新: NHC/ZUPT/ZARU (per-IMU, decimation 控制, 互斥)。
  4. 推进后检查: 若 pending GNSS 更近于新 imucur, 做量测更新 + 反馈。

GNSS 由 add_gnss 入队, 不直接触发更新 (等下一个 IMU 决定最近邻)。
NHC/ZUPT/ZARU 由 IMU 触发 (decimation), 不依赖 GNSS。
"""
import collections
import logging
from typing import Optional

from src.core.data_types import GnssSolution, ImuMeasurement
from src.core.ins.constraints import Constraints
from src.core.ins.static_detect import StaticDetect

logger = logging.getLogger(__name__)


class _DecimationCounter:
    """Decimation 计数器 (参考 ignav nc++>nhz 逻辑)。

    should_trigger() 在计数达到 min_count 时返回 True 并复位,
    无论后续 guard 是否通过 (与 ignav 一致)。
    """

    def __init__(self, min_count: int):
        self.min_count = max(0, int(min_count))
        self.count = 0

    def should_trigger(self) -> bool:
        if self.count >= self.min_count:
            self.count = 0
            return True
        self.count += 1
        return False


class LcIntegration:
    """松组合导航集成 (最近邻时间对齐 + 单滤波 EKF 主循环)。

    NHC/ZUPT/ZARU 作为可选约束, per-IMU 触发 (decimation 控制):
      - 静态 (StaticDetect): ZUPT + ZARU (互斥于 NHC)
      - 运动: NHC (需非剧烈转弯)
    """

    def __init__(self, estimator, config: dict):
        self.est = estimator
        ins_cfg = config.get("ins", {})
        # 使能开关
        self.nhc_enable = int(ins_cfg.get("nhc_enable", 0))
        self.zupt_enable = int(ins_cfg.get("zupt_enable", 0))
        self.zaru_enable = int(ins_cfg.get("zaru_enable", 0))
        # Decimation
        self._nhc_counter = _DecimationCounter(
            int(ins_cfg.get("nhc_decimation", 1)))
        self._zupt_counter = _DecimationCounter(
            int(ins_cfg.get("zupt_min_count", 15)))
        self._zaru_counter = _DecimationCounter(
            int(ins_cfg.get("zaru_min_count", 100)))
        # 静态检测
        self._static_detect = StaticDetect(config)
        # 约束更新模块 (NHC/ZUPT/ZARU, 独立于 LcEstimator)
        self._constraints = Constraints(config)
        # 超过此秒数认为 GNSS 已过期, 丢弃 (避免用错位状态做更新)
        self._stale_threshold = 0.5

        self.imupre: Optional[ImuMeasurement] = None
        self.imucur: Optional[ImuMeasurement] = None
        self.pending_gnss: collections.deque = collections.deque()

    def add_imu(self, imu: ImuMeasurement) -> None:
        """添加 IMU: 预推进 GNSS → 推进 → 约束 → 推进后 GNSS。"""
        if self.imucur is None:
            self.imucur = imu
            self._static_detect.push(imu)
            return

        cur_t = self.imucur.timestamp
        new_t = imu.timestamp

        # 1. 预推进: 处理更近于当前 imucur 的 pending GNSS
        self._process_pending(cur_t, new_t, pre_advance=True)

        # 2. 推进 (机械编排 + P 协方差传播)
        self.imupre = self.imucur
        self.imucur = imu
        self.est.time_update(imu)
        self._static_detect.push(imu)

        # 3. 约束更新 (NHC/ZUPT/ZARU, per-IMU, decimation)
        self._apply_constraints(imu)

        # 4. 推进后: 处理更近于新 imucur 的 pending GNSS
        self._process_pending(self.imucur.timestamp, self.imupre.timestamp,
                              pre_advance=False)

    def add_gnss(self, gnss: GnssSolution) -> None:
        """添加 GNSS: append 到 pending_gnss deque。"""
        self.pending_gnss.append(gnss)

    def _process_pending(self, t_ref: float, t_other: float,
                         pre_advance: bool) -> None:
        """处理 pending GNSS 队列。

        Args:
            t_ref: 参考时刻 (pre_advance: 当前 imucur; post_advance: 新 imucur)
            t_other: 对比时刻 (pre_advance: 新 imu; post_advance: 新 imupre)
            pre_advance: True 表示推进前 (用当前 state), False 表示推进后
        """
        while self.pending_gnss:
            gnss = self.pending_gnss[0]
            d_ref = abs(t_ref - gnss.timestamp)
            d_other = abs(t_other - gnss.timestamp)

            if pre_advance:
                # 推进前: 若 imucur 更近 (或并列), 用当前 state 处理
                should_process = d_ref <= d_other
            else:
                # 推进后: 若新 imucur 严格更近, 用新 state 处理
                # (并列情况已在推进前处理, 这里不重复)
                should_process = d_ref < d_other

            if should_process:
                # 丢弃严重过期的 GNSS (距参考时刻 > 阈值)
                if d_ref > self._stale_threshold and gnss.timestamp < t_ref:
                    logger.warning(
                        f"丢弃过期 GNSS t={gnss.timestamp:.6f} "
                        f"(t_ref={t_ref:.6f}, d={d_ref:.4f}s)"
                    )
                    self.pending_gnss.popleft()
                    continue
                self._apply_gnss_update(gnss)
                self.pending_gnss.popleft()
            else:
                # GNSS 更近于另一侧, 等待
                break

    def _apply_gnss_update(self, gnss: GnssSolution) -> None:
        """应用 GNSS 位置/速度量测更新 + 反馈。"""
        self.est.meas_update_pos(gnss)
        if gnss.velocity is not None:
            self.est.meas_update_vel(gnss)
        self.est.feedback()

    def _apply_constraints(self, imu: ImuMeasurement) -> None:
        """NHC/ZUPT/ZARU 约束更新 (per-IMU, decimation, 互斥)。

        互斥逻辑 (参考 ignav postpos.cc):
          静态 → ZUPT + ZARU (若启用)
          运动 → NHC (若启用, 需非剧烈转弯)
        """
        if not (self.nhc_enable or self.zupt_enable or self.zaru_enable):
            return

        state = self.est.state
        is_static = self._static_detect.detect(state.pos_e)

        applied = False
        if is_static:
            if self.zupt_enable and self._zupt_counter.should_trigger():
                if self._constraints.zupt(self.est):
                    applied = True
            if self.zaru_enable and self._zaru_counter.should_trigger():
                if self._constraints.zaru(self.est, imu):
                    applied = True
        elif self.nhc_enable and self._nhc_counter.should_trigger():
            if self._constraints.nhc(self.est, imu):
                applied = True

        if applied:
            self.est.feedback()
