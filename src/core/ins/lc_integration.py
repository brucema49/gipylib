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
import dataclasses
import logging
import math
from typing import Optional

import numpy as np

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
        # 位置差分速度: 当 GNSS 无速度输出 (如 RTK) 时, 用相邻位置差分计算速度
        # vel = (pos_cur - pos_prev) / dt, vel_sd = sqrt(2) * pos_sigma / dt
        self._prev_gnss_pos: Optional[np.ndarray] = None
        self._prev_gnss_ts: Optional[float] = None
        self._pos_diff_vel_std = float(ins_cfg.get("pos_diff_vel_std", 0.5))
        # Qins 跟踪 (与 ignav outins 一致):
        #   2 = mech + propagate (time_update only)
        #   3 = LC update (GNSS meas_update 或约束触发)
        # add_imu 开始时置 2, _apply_gnss_update / _apply_constraints 触发后置 3,
        # 末尾 time_update(imu) 不重置 (保留本历元量测更新标记)
        self.last_qins: int = 2
        # NHC warmup: 动态初始化后需等待首次 GNSS 量测更新修正 yaw, 再启用 NHC
        # (与 TcIntegration._nhc_warmup 一致, 默认 1 = 至少 1 次 GNSS 更新后启用)
        self._meas_count: int = 0
        self._nhc_warmup = int(ins_cfg.get("nhc_warmup", 1))

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
            self.last_qins = 2
            return

        self.last_qins = 2  # 默认: 仅机械编排 + 协方差传播

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
        """应用 GNSS 位置/速度量测更新 + 反馈。

        当 GNSS 无速度输出 (如 RTK) 时, 用位置差分计算速度:
          vel = (pos_cur - pos_prev) / dt
        位置差分速度噪声: sqrt(2) * pos_sigma / dt, 不小于 pos_diff_vel_std。
        """
        self.est.meas_update_pos(gnss)

        # 速度量测更新: 优先使用 GNSS 速度, 但 RTK 的 vel_sd 可能极大 (20+ m/s)
        # 导致 K≈0。当 vel_sd 过大时, 用位置差分速度替代 (sigma 更可靠)。
        vel_for_update = gnss.velocity
        vel_sd_for_update = gnss.vel_sd
        use_pos_diff = False
        if vel_for_update is not None and vel_sd_for_update is not None:
            max_sd = float(np.max(vel_sd_for_update))
            if max_sd > 2.0:  # RTK vel_sd 不可靠, 用位置差分替代
                use_pos_diff = True

        if (vel_for_update is None or use_pos_diff) and self._prev_gnss_pos is not None:
            dt = gnss.timestamp - self._prev_gnss_ts
            if dt > 0.5:  # 仅在合理时间间隔内计算 (避免 GNSS 中断后差分)
                vel_diff = (gnss.position - self._prev_gnss_pos) / dt
                vel_for_update = vel_diff
                # 位置差分速度 sigma (per-axis): sqrt(2) * pos_sd[i] / dt
                if gnss.sd is not None:
                    pos_sd = gnss.sd
                else:
                    pos_sd = np.full(3, 1.0)
                vel_sd_arr = np.maximum(
                    math.sqrt(2) * pos_sd / dt,
                    self._pos_diff_vel_std)
                vel_sd_for_update = vel_sd_arr

        if vel_for_update is not None:
            # 构造带速度的 GnssSolution 副本 (避免修改原对象)
            gnss_with_vel = dataclasses.replace(gnss)
            gnss_with_vel.velocity = vel_for_update
            gnss_with_vel.vel_sd = vel_sd_for_update
            self.est.meas_update_vel(gnss_with_vel)

        self.est.feedback()
        self.last_qins = 3  # LC 量测更新完成
        self._meas_count += 1  # NHC warmup: 计数 GNSS 量测更新

        # 缓存当前 GNSS 位置供下次位置差分
        self._prev_gnss_pos = gnss.position.copy()
        self._prev_gnss_ts = gnss.timestamp

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
        elif (self.nhc_enable and self._nhc_counter.should_trigger()
              and self._meas_count >= self._nhc_warmup):
            if self._constraints.nhc(self.est, imu):
                applied = True

        if applied:
            self.est.feedback()
            self.last_qins = 3  # 约束量测更新完成
