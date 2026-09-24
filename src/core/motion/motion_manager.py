"""运动约束管理器（松紧组合共用）。

把原先在 ``LcIntegration._apply_constraints`` 与 ``TcIntegration._apply_constraints``
里**逐字重复两份**的调度逻辑抽到一处：使能开关、decimation、静态检测、互斥选择、
NHC warmup、残差记录、抗差接入。

约束算法本身仍在 ``src/core/ins/constraints.py``（``Constraints``），
零速检测仍在 ``src/core/ins/static_detect.py``（``StaticDetect``），本模块只负责编排。

用法::

    self.motion = MotionConstraintManager(config, nhc_warmup=30)
    ...
    self.motion.push_imu(imu)
    if self.motion.apply_constraints(imu, self.est, meas_count=self._meas_count):
        self.est.feedback()
        self.last_qins = 3
"""
from __future__ import annotations

import logging

import numpy as np

from src.core.data_types import ImuMeasurement
from src.core.ins.constraints import Constraints
from src.core.ins.static_detect import StaticDetect
from src.core.motion.decimation_counter import DecimationCounter
from src.core.motion.motion_residuals import (
    ConstraintResidualLog,
    ConstraintResidualSnapshot,
    standardized_residual,
)
from src.core.robust.robust_estimator import build_robust_estimator

logger = logging.getLogger(__name__)


class MotionConstraintManager:
    """运动约束管理器（松紧组合共用）。

    互斥逻辑 (参考 ignav postpos.cc, skills/NHC_ZUPT.md §3):
        静态 → ZUPT (+ ZARU)
        运动 → NHC

    Args:
        config: 完整配置字典（读 ``ins`` 段与 ``robust`` 段）
        nhc_warmup: 至少经过多少次 GNSS 量测更新后才施加 NHC
            （LC 默认 1，TC 默认 30；等待航向收敛）
    """

    def __init__(self, config: dict, *, nhc_warmup: int = 1):
        ins_cfg = (config or {}).get("ins", {}) or {}
        self.nhc_enable = int(ins_cfg.get("nhc_enable", 0))
        self.zupt_enable = int(ins_cfg.get("zupt_enable", 0))
        self.zaru_enable = int(ins_cfg.get("zaru_enable", 0))
        self.nhc_warmup = int(nhc_warmup)

        self._nhc_counter = DecimationCounter(
            int(ins_cfg.get("nhc_decimation", 1)))
        self._zupt_counter = DecimationCounter(
            int(ins_cfg.get("zupt_min_count", 15)))
        self._zaru_counter = DecimationCounter(
            int(ins_cfg.get("zaru_min_count", 100)))

        # 算法层（本模块只编排，不含公式）
        self.detector = StaticDetect(config)
        self.constraints = Constraints(config)

        # 残差输出（默认关闭）+ 抗差估计（默认关闭）
        self.residuals = ConstraintResidualLog(config)
        self.robust = build_robust_estimator(config)

        self.applied_counts = {"nhc": 0, "zupt": 0, "zaru": 0}
        self.static_count = 0
        self.detect_count = 0

    # ---- 状态查询 ----

    @property
    def enabled(self) -> bool:
        return bool(self.nhc_enable or self.zupt_enable or self.zaru_enable)

    def open(self) -> None:
        """打开可选的残差转储文件。"""
        self.residuals.open()

    def close(self) -> None:
        self.residuals.close()

    # ---- IMU 侧 ----

    def push_imu(self, imu: ImuMeasurement) -> None:
        """把 IMU 样本喂给零速检测滑动窗口。"""
        self.detector.push(imu)

    def apply_constraints(self, imu: ImuMeasurement, estimator,
                          *, meas_count: int = 0) -> bool:
        """对当前 IMU 历元应用约束，返回是否执行了更新。

        返回 True 时调用方负责 ``estimator.feedback()``（约束层不做反馈，
        与 skills/NHC_ZUPT.md §7.3 一致）。
        """
        if not self.enabled:
            return False

        state = estimator.state
        # 静止判据 = IMU 窗口检验 AND 速度判据 (NHC_ZUPT.md §2.4)
        is_static = bool(self.detector.detect(state.pos_e, state.vel_e))
        self.detect_count += 1
        self.static_count += 1 if is_static else 0

        timestamp = float(getattr(imu, "timestamp", 0.0) or 0.0)
        applied = False
        if is_static:
            if self.zupt_enable and self._zupt_counter.should_trigger():
                applied |= self._inject(
                    "zupt", estimator, timestamp,
                    lambda est: self.constraints.build_zupt(est))
            if self.zaru_enable and self._zaru_counter.should_trigger():
                applied |= self._inject(
                    "zaru", estimator, timestamp,
                    lambda est: self.constraints.build_zaru(est, imu))
        elif (self.nhc_enable and self._nhc_counter.should_trigger()
              and meas_count >= self.nhc_warmup):
            applied |= self._inject(
                "nhc", estimator, timestamp,
                lambda est: self.constraints.build_nhc(est, imu))
        return applied

    # ---- 内部: 单次伪量测注入 ----

    def _inject(self, source: str, estimator, timestamp: float, build) -> bool:
        """构造 → (抗差) → joseph_update → 残差记录。"""
        built = build(estimator)
        if built is None:
            return False
        Z, H, R = built
        Z = np.asarray(Z, dtype=np.float64)
        if Z.size == 0:
            return False

        x_prior = estimator.x.copy()
        P_prior = estimator.P.copy()
        innov = Z - H @ x_prior

        R_used = R
        std_res = None
        if self.robust is not None and self.robust.applies_to(source):
            R_used = self.robust.adjust_R(innov, H, R, P_prior, source)
            std_res = standardized_residual(innov, H, R, P_prior)
        elif self.residuals.enabled:
            std_res = standardized_residual(innov, H, R, P_prior)

        estimator.joseph_update(Z, H, R_used, update_kind=source,
                                update_timestamp=timestamp)

        self.applied_counts[source] = self.applied_counts.get(source, 0) + 1
        if self.residuals.enabled:
            dx = estimator.x - x_prior
            self.residuals.record(ConstraintResidualSnapshot(
                timestamp=timestamp,
                source=source,
                v_prior=innov,
                H=np.asarray(H).copy(),
                R=np.asarray(R).copy(),
                P_prior=P_prior,
                x_prior=x_prior,
                v_posterior=innov - H @ dx,
                P_posterior=estimator.P.copy(),
                x_posterior=estimator.x.copy(),
                R_used=None if R_used is R else np.asarray(R_used).copy(),
                std_res=std_res,
                info={"n_obs": int(Z.size)},
            ))
        return True
