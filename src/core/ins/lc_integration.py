"""松组合导航集成 (GVINS 风格 IMU 消费 + 主循环)。

参考:
- tools/GVINS/estimator/src/estimator_node.cpp process() (IMU 跨 GNSS 时刻线性插值)
- KF-GINS newImuProcess (imupre/imucur/pending_gnss 结构)
- ignav postpos.cc (NHC/ZUPT/ZARU per-IMU 触发 + decimation)

主循环 (每个新 IMU 到来时, GVINS 风格):
  1. 检查 pending_gnss 队头:
     - 若 gnss.t < cur.t: 防御性直接量测更新 (过期 GNSS)
     - 若 cur.t <= gnss.t <= imu.t: 线性插值到 gnss.t, time_update(interp),
       触发 GNSS 量测更新 + 反馈, 弹出 gnss, cur←interp, 循环处理后续 GNSS
     - 若 gnss.t > imu.t: 留给后续 IMU, 跳出循环
  2. 推进当前 IMU: imupre←cur, imucur←imu, time_update(imu)
  3. 约束更新: NHC/ZUPT/ZARU (per-IMU, decimation 控制, 互斥)

GNSS 由 add_gnss 入队, 不直接触发更新 (等 IMU 跨越 gnss.t 时触发)。
NHC/ZUPT/ZARU 由 IMU 触发 (decimation), 不依赖 GNSS。
"""
import collections
import logging
from typing import Optional

from src.core.data_types import GnssSolution, ImuMeasurement
from src.core.ins.constraints import Constraints
from src.core.ins.interpolator import imu_interpolate_linear
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
    """松组合导航集成 (GVINS 风格 IMU 消费 + 单滤波 EKF 主循环)。

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

        self.imupre: Optional[ImuMeasurement] = None
        self.imucur: Optional[ImuMeasurement] = None
        self.pending_gnss: collections.deque = collections.deque()

    def add_imu(self, imu: ImuMeasurement) -> None:
        """GVINS 风格 IMU 消费: 每条 IMU 检查 GNSS 队头时间戳。

        - imu.t < gnss.t: 直接机械编排 (无 GNSS 触发)
        - imu.t >= gnss.t 且 cur.t <= gnss.t:
          1) 线性插值到 gnss.t
          2) time_update(interp) 推进 dt_1 = gnss.t - cur.t
          3) meas_update_pos/vel(gnss) + feedback 触发融合
          4) 弹出 gnss, 循环处理后续 GNSS (多 GNSS 同区间)
          5) cur ← interp, 继续用当前 imu 推进 dt_2 = imu.t - gnss.t

        参考: tools/GVINS/estimator/src/estimator_node.cpp process() lines 338-378
        """
        if self.imucur is None:
            self.imucur = imu
            self._static_detect.push(imu)
            return

        cur = self.imucur  # 当前已推进到的 IMU

        # 1. 处理所有落入 [cur.t, imu.t] 区间的 GNSS (GVINS 风格插值触发)
        while self.pending_gnss:
            gnss = self.pending_gnss[0]
            t_gnss = gnss.timestamp

            if t_gnss < cur.timestamp:
                # GNSS 已过期 (比 cur 还早): 防御性直接量测更新
                # (Logger 按时间顺序喂入, 理论上不应发生)
                logger.debug(
                    f"过期 GNSS t={t_gnss:.6f} < cur.t={cur.timestamp:.6f}, "
                    f"直接量测更新"
                )
                self._apply_gnss_update(gnss)
                self.pending_gnss.popleft()
                continue

            if t_gnss > imu.timestamp:
                # GNSS 在当前 IMU 之后: 留给后续 IMU 处理
                break

            # cur.t <= t_gnss <= imu.t: GVINS 风格插值触发
            if t_gnss == cur.timestamp:
                # GNSS 恰好对齐 cur: 无需插值, 直接触发
                interp = cur
            else:
                # 线性插值到 t_gnss
                interp = imu_interpolate_linear(cur, imu, t_gnss)
                if interp is None:
                    break
                # 推进 dt_1 = t_gnss - cur.t
                self.imupre = cur
                self.imucur = interp
                self.est.time_update(interp)
                self._static_detect.push(interp)
                self._apply_constraints(interp)

            # 触发 GNSS 量测更新 + 反馈
            self._apply_gnss_update(gnss)
            self.pending_gnss.popleft()
            cur = interp  # 后续 GNSS 从 interp 时刻继续

        # 2. 推进当前 IMU (dt = imu.t - cur.t)
        self.imupre = cur
        self.imucur = imu
        self.est.time_update(imu)
        self._static_detect.push(imu)
        self._apply_constraints(imu)

    def add_gnss(self, gnss: GnssSolution) -> None:
        """添加 GNSS: append 到 pending_gnss deque。"""
        self.pending_gnss.append(gnss)

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
