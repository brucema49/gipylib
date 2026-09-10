"""紧组合导航集成 (GVINS 风格 IMU 消费 + TC 量测触发)。

参考:
- src/core/ins/lc_integration.py (GVINS 风格主循环骨架)
- tools/GVINS/estimator/src/estimator_node.cpp process() (IMU 跨 GNSS 时刻线性插值)
- ignav postpos.cc (NHC/ZUPT/ZARU per-IMU 触发 + decimation)

与 LcIntegration 的关键区别:
  - GNSS 输入是原始观测 (obsr, obsb, nav), 非 GnssSolution
  - 初始化: 先用 SPP 粗定位, 再调 InsInitializer
  - 量测更新: 调 TcMeasurement.build → estimator.tc_meas_update
  - 降级: TcDegradeManager 管理 rtk→rtd→spp→imu_only
  - 模糊度: RTK 模式调 TcAmbiguity.try_fix

主循环 (每个新 IMU 到来时, GVINS 风格):
  1. 检查 pending_obs 队头:
     - 若 obs.t < cur.t: 防御性直接量测更新 (过期 obs)
     - 若 cur.t <= obs.t <= imu.t: 线性插值到 obs.t, time_update(interp),
       触发 TC 量测更新 + 反馈, 弹出 obs, cur←interp, 循环处理后续 obs
     - 若 obs.t > imu.t: 留给后续 IMU, 跳出循环
  2. 推进当前 IMU: imupre←cur, imucur←imu, time_update(imu)
  3. 约束更新: NHC/ZUPT/ZARU (per-IMU, decimation 控制, 互斥)
"""
import collections
import csv
import logging
import math
from copy import copy
from pathlib import Path
from typing import Optional, List

import numpy as np

from src.core.data_types import (
    AlignedBlock,
    GnssSolution,
    ImuMeasurement,
    IncrementImuData,
)
from src.core.ins.attitude import euler2dcm, att_caln2e, dcm2quat
from src.core.ins.constraints import Constraints
from src.core.ins.earth_param import cal_Ce2n, ecef2llh
from src.core.ins.initializer import InsInitializer, InitMode
from src.core.ins.interpolator import (
    imu_interpolate_linear,
    split_increment_at_gnss,
)
from src.core.ins.lc_integration import _DecimationCounter
from src.core.ins.static_detect import StaticDetect
from src.core.time_utils import unix_to_gpst
from src.core.tc.tc_ambiguity import TcAmbiguity
from src.core.tc.tc_degrade import TcDegradeManager
from src.core.tc.tc_estimator import TcEstimator
from src.core.tc.tc_measurement import SppTcMeas, RtkTcMeas, RtdTcMeas
from src.log.tc_matrix_diagnostics import TcMatrixDiagnosticWriter

logger = logging.getLogger(__name__)

_MEAS_BUILDERS = {"spp": SppTcMeas, "rtk": RtkTcMeas, "rtd": RtdTcMeas}

# Unix↔GPST conversion of a GNSS epoch and the SOW stored on an increment can
# differ by a few ULP at campus01 magnitudes (~1.8e5 s).  A GNSS epoch that is
# mathematically on an increment endpoint must still be consumed once by that
# increment, so snap such near-boundary SOW values.  The tolerance stays far
# below the 1 ms endpoint rule used by ``split_increment_at_gnss``.
_SOW_ENDPOINT_TOLERANCE_S = 1.0e-6


class TcIntegration:
    """紧组合导航集成 (GVINS 风格 IMU 消费 + TC 量测触发)。

    初始化前用小缓冲累积 IMU+GNSS 原始观测，初始化后增量处理。
    NHC/ZUPT/ZARU 作为可选约束, per-IMU 触发 (decimation 控制):
      - 静态 (StaticDetect): ZUPT + ZARU (互斥于 NHC)
      - 运动: NHC (需非剧烈转弯)
    """

    def __init__(self, config: dict, mode: str = "spp", output_callback=None):
        self._cfg = config
        self._mode = mode
        self._output_callback = output_callback
        self._initializer = InsInitializer(config)
        ins_cfg = config.get("ins", {})
        # 初始化模式选择 (与 LcStream 一致)
        self._init_mode = self._select_init_mode(config)

        # 约束 + 静态检测
        self.nhc_enable = int(ins_cfg.get("nhc_enable", 0))
        self.zupt_enable = int(ins_cfg.get("zupt_enable", 0))
        self.zaru_enable = int(ins_cfg.get("zaru_enable", 0))
        self._nhc_counter = _DecimationCounter(
            int(ins_cfg.get("nhc_decimation", 1)))
        self._nhc_warmup = int(ins_cfg.get("nhc_warmup", 30))
        self._zupt_counter = _DecimationCounter(
            int(ins_cfg.get("zupt_min_count", 15)))
        self._zaru_counter = _DecimationCounter(
            int(ins_cfg.get("zaru_min_count", 100)))
        self._static_detect = StaticDetect(config)
        self._constraints = Constraints(config)

        # 降级管理
        tc_cfg = config.get("tc", {}).get("degrade", {})
        self._degrade = TcDegradeManager(
            initial_mode=mode,
            fail_threshold=tc_cfg.get("fail_threshold", 3),
            reboot_threshold=tc_cfg.get("reboot_threshold", 30.0))

        # 模糊度 (RTK 才用)
        self._ambiguity = TcAmbiguity(
            thresar=float(config.get("gnss", {}).get("thresar", 3.0)))

        # 估计器 + 量测构造器 (初始化后创建)
        self._est: Optional[TcEstimator] = None
        self._meas_builder = None
        # Input payload form and the requested propagation form are separate:
        # native increments stay increments, while rate input may explicitly
        # opt into the same increment-boundary path.  ``rate_to_increment`` is
        # the canonical switch; ``imu_data_process_form`` is the derived field
        # filled by the config loader and kept for direct-dict callers.
        process_form = ins_cfg.get("imu_data_process_form")
        if process_form is None and "rate_to_increment" in ins_cfg:
            process_form = ("increment" if ins_cfg["rate_to_increment"]
                            else "rate")
        self._imu_data_process_form = str(process_form or "rate").lower()

        # IMU 状态 (GVINS 风格)
        self.imupre: Optional[ImuMeasurement] = None
        self.imucur: Optional[ImuMeasurement] = None
        # GNSS 原始观测队列: (obsr, obsb, nav, t_gnss)
        self.pending_obs: collections.deque = collections.deque()

        # 初始化前缓冲
        self._init_imu: List[ImuMeasurement] = []
        self._init_obs: list = []   # [(obsr, obsb, nav, t_gnss), ...]
        # 5秒 GNSS 位置缓存 (用于动态初始化: 首尾位置差分计算 yaw)
        self._gnss_pos_cache: list = []  # [(t, pos_ecef), ...]
        self._initialized = False
        self._last_gnss_t: float = 0.0
        self._last_q: int = 5       # 最近 GNSS quality (初值 5=SPP)
        self._last_ns: int = 0      # 最近 num_sv
        self._writer = None
        self._output_count = 0
        self.last_qins: int = 2
        # 一次性 yaw 航向对齐状态 (ins.yaw_align_on_move=1 时启用)
        self._yaw_aligned = False
        self._yaw_hist = collections.deque()
        # 最近一次量测更新的轻量信息快照 (stat_writer GIPY_* 使用)
        self.last_update_info = None
        # false fix 检测: 跟踪上次量测更新后的位置, 检测异常跳变
        self._last_meas_pos: Optional[np.ndarray] = None
        self._amb_fixed: bool = False
        # 量测更新计数 (用于收敛期保护: 前 N 个历元禁用 NIS/false_fix/跳变检验)
        self._meas_count: int = 0
        self._convergence_warmup: int = 10  # 收敛预热历元数 (仅跳变检验)
        # 量测连续失败计数 (用于发散恢复: 连续失败超阈值时尝试 SPP 重初始化)
        self._consecutive_failures: int = 0
        self._recovery_threshold: int = 10  # 连续失败 10 个历元后尝试恢复
        # 上一 GNSS 历元时刻 (gtime_t), 供 udbias 计算历元间隔 (随机游走/失锁计时)
        self._prev_obs_t = None
        # Optional, explicit-only measurement boundary diagnostics.  Keeping
        # this disabled by default prevents normal runs from creating files.
        self._diagnostics_path = str(
            config.get("tc", {}).get("diagnostics_path", ""))
        self._matrix_diagnostics = None
        matrix_path = str(config.get("tc", {}).get(
            "matrix_diagnostics_path", ""))
        if matrix_path:
            try:
                self._matrix_diagnostics = TcMatrixDiagnosticWriter(
                    matrix_path, mode=mode)
                self._matrix_diagnostics.open()
            except OSError as exc:
                logger.warning("TC matrix diagnostics disabled: %s", exc)
                self._matrix_diagnostics = None

    @property
    def initialized(self) -> bool:
        return self._initialized

    @property
    def state(self):
        return self._est.state if self._est is not None else None

    @property
    def P(self):
        return self._est.P if self._est is not None else None

    @property
    def si(self):
        return self._est.si if self._est is not None else None

    def set_writer(self, writer) -> None:
        self._writer = writer

    def close(self) -> None:
        """Close optional diagnostic output without affecting the filter."""
        if self._matrix_diagnostics is not None:
            try:
                self._matrix_diagnostics.close()
            except OSError as exc:
                logger.warning("TC matrix diagnostics close failed: %s", exc)
            finally:
                self._matrix_diagnostics = None

    def _disable_matrix_diagnostics(self, exc: Exception) -> None:
        logger.warning("TC matrix diagnostics disabled after write failure: %s", exc)
        self.close()

    def _record_latest_propagation(self) -> None:
        """Write the latest estimator propagation, if diagnostics are enabled."""
        if self._matrix_diagnostics is None or self._est is None:
            return
        snapshot = getattr(self._est, "last_propagation_snapshot", None)
        if snapshot is None:
            return
        try:
            self._matrix_diagnostics.write_imu_prop(
                timestamp=snapshot["timestamp"],
                dt=snapshot["dt"],
                p_before=snapshot["p_before"],
                phi=snapshot["phi"],
                q=snapshot["q"],
                p_after=snapshot["p_after"],
            )
        except (OSError, TypeError, ValueError) as exc:
            self._disable_matrix_diagnostics(exc)

    def _record_matrix_update_diagnostic(self, timestamp: float, pre_p,
                                         post_p, innovation, s_matrix,
                                         k_gain, feedback_x, accepted: bool,
                                         info: dict | None = None) -> None:
        """Write one GNSS update snapshot, including rejected updates."""
        if self._matrix_diagnostics is None:
            return
        snapshot = getattr(self._est, "last_propagation_snapshot", None)
        if snapshot is None:
            phi = np.eye(15, dtype=np.float64)
            q = np.zeros((15, 15), dtype=np.float64)
            dt = -1.0
        else:
            phi = snapshot["phi"]
            q = snapshot["q"]
            dt = snapshot["dt"]
        diag_info = dict(info or {})
        diag_info["dt"] = float(dt)
        try:
            self._matrix_diagnostics.write_update(
                timestamp=timestamp,
                p_before=np.asarray(pre_p)[:15, :15],
                p_after=np.asarray(post_p)[:15, :15],
                phi=phi,
                q=q,
                innovation=innovation,
                S_diag=np.diag(s_matrix),
                K=k_gain,
                feedback_x=feedback_x,
                accepted=accepted,
                info=diag_info,
            )
        except (OSError, TypeError, ValueError) as exc:
            self._disable_matrix_diagnostics(exc)

    def _emit_output(self, qins: int) -> None:
        """Emit a state at an exact fusion boundary when a stream is attached."""
        if self._output_callback is not None:
            self._output_callback(self._est.state, self._est.P, qins)
        elif self._writer is not None:
            self._write_state(qins)

    def _emit_propagation(self) -> None:
        """Write exactly one row per successful mechanization commit.

        Called only after a real ``InsUpdate`` commit (and after any GNSS
        measurement that refines the same endpoint), never for a pure
        measurement or bookkeeping step.  Duplicate or synthetic rows would
        break the 100 Hz spacing contract of ``nav-100hz.csv``.
        """
        if self._est is None:
            return
        ins_update = getattr(self._est, "ins_update", None)
        if ins_update is not None and not getattr(
                ins_update, "last_update_accepted", True):
            return
        self._emit_output(self.last_qins)
        self.last_qins = 2

    @staticmethod
    def _select_init_mode(config: dict) -> InitMode:
        """TC 模式初始化模式选择: 动态位置差分。

        静态初始化 yaw=0 在本数据集 (EuRoC) 上错误: truth yaw≈290° at t=0,
        290° 初始误差违反 EKF 小角度假设, yaw 永远无法收敛。
        改用 POSITION_DIFF: 等车辆运动 (速度>阈值) 后用速度方向计算 yaw,
        与 ignav 参考一致 (t+730s 初始化, yaw 误差仅 ~5°)。
        无人机起飞/巡航阶段速度方向≈yaw, 仅纯侧飞时偏差大 (占比低)。
        """
        return InitMode.POSITION_DIFF

    @staticmethod
    def _restore_nav(nav, saved_x, saved_P, saved_fix=None, saved_lock=None):
        """恢复 nav 状态 (SPP/RTK 副作用清除)。"""
        nav.x[:] = saved_x
        nav.P[:] = saved_P
        if saved_fix is not None:
            nav.fix[:] = saved_fix
        if saved_lock is not None:
            nav.lock[:] = saved_lock

    # ===== 增量喂入 =====

    def add_imu(self, imu: ImuMeasurement) -> None:
        """GVINS 风格 IMU 消费: 每条 IMU 检查 GNSS 队头时间戳。"""
        if not self._initialized:
            self._init_imu.append(imu)
            # 限制缓冲区大小: 只保留最近 5 秒的 IMU 数据 (避免 O(N²) 遍历)
            # 初始化只需 GNSS 历元前后 2s 的 IMU 数据
            if len(self._init_imu) > 1000:  # 100Hz × 10s = 1000
                self._init_imu = self._init_imu[-500:]
            # 当有 GNSS obs 且当前 IMU 时间戳 >= GNSS 时间戳时, 尝试初始化
            if self._init_obs and imu.timestamp >= self._init_obs[-1][3]:
                self._try_init()
            return

        # Dispatch before the legacy rate path.  Native increments must never
        # enter rate interpolation, and rate-to-increment is an explicit
        # conversion performed once at the propagation boundary.
        if imu.is_increment():
            had_previous = self.imucur is not None
            self._add_increment_imu(imu)
            if had_previous and self._est is not None:
                self._check_velocity_divergence()
            return
        if self._imu_data_process_form == "increment":
            had_previous = self.imucur is not None
            self._add_rate_as_increment_imu(imu)
            if had_previous and self._est is not None:
                self._check_velocity_divergence()
            return

        if self.imucur is None:
            self.imucur = imu
            self._static_detect.push(imu)
            self.last_qins = 2
            return

        self.last_qins = 2  # 默认: 仅机械编排 + 协差variance propagation

        cur = self.imucur

        # 1. 处理所有落入 [cur.t, imu.t] 区间的 GNSS obs (GVINS 风格插值触发)
        while self.pending_obs:
            obsr, obsb, nav, t_gnss = self.pending_obs[0]

            if t_gnss < cur.timestamp:
                # GNSS 已过期 (比 cur 还早): 防御性直接量测更新
                # (该端点已写过行, 不产生新的机械编排行)
                logger.debug(
                    f"过期 GNSS obs t={t_gnss:.6f} < cur.t={cur.timestamp:.6f}, "
                    f"直接量测更新")
                self._trigger_meas(cur, obsr, obsb, nav, t_gnss)
                self.last_qins = 2
                self.pending_obs.popleft()
                continue

            if t_gnss > imu.timestamp:
                # GNSS 在当前 IMU 之后: 留给后续 IMU 处理
                break

            # cur.t <= t_gnss <= imu.t: GVINS 风格插值触发
            mechanized = False
            if t_gnss == cur.timestamp:
                # 该端点已在上一轮机械编排写出, 这里只做量测修正
                interp = cur
            else:
                interp = imu_interpolate_linear(cur, imu, t_gnss)
                if interp is None:
                    break
                self.imupre = cur
                self.imucur = interp
                self._est.time_update(interp)
                self._record_latest_propagation()
                self._static_detect.push(interp)
                self._apply_constraints(interp)
                mechanized = True

            # 触发 TC 量测更新 + 反馈
            self._trigger_meas(interp, obsr, obsb, nav, t_gnss)
            if mechanized:
                self._emit_propagation()
            self.last_qins = 2
            self.pending_obs.popleft()
            cur = interp

        # 2. 推进当前 IMU (dt = imu.t - cur.t)
        self.imupre = cur
        self.imucur = imu
        self._est.time_update(imu)
        self._record_latest_propagation()
        self._static_detect.push(imu)
        self._apply_constraints(imu)

        self._check_velocity_divergence()

        self._emit_propagation()

    def _check_velocity_divergence(self) -> None:
        """Reset an obviously divergent TC velocity after propagation."""
        vel = self._est.state.vel_e
        speed = float(np.linalg.norm(vel))
        if speed <= 50.0:
            return
        logger.warning(
            f"TC vel_diverge (t={self._est.state.timestamp:.3f}): "
            f"speed={speed:.2f} m/s > 50, reset vel & inflate P")
        self._est.state.vel_e = np.zeros(3, dtype=np.float64)
        si = self._est.si
        for k in range(3):
            self._est.P[si.vel + k, si.vel + k] = 100.0 ** 2
        # 清零速度与其他状态的交叉协方差
        for k in range(3):
            for j in range(si.dim):
                if j < si.vel or j >= si.vel + 3:
                    self._est.P[si.vel + k, j] = 0.0
                    self._est.P[j, si.vel + k] = 0.0
        self._est.x[si.vel:si.vel + 3] = 0.0

    def _propagate_increment_segment(self, previous: ImuMeasurement,
                                     segment: ImuMeasurement) -> None:
        """Consume one already-bounded increment and apply TC side effects."""
        self.imupre = previous
        self.imucur = segment
        self._est.time_update(segment)
        self._record_latest_propagation()
        self._static_detect.push(segment)
        self._apply_constraints(segment)

    @staticmethod
    def _raw_gnss_sow(obsr, timestamp: float) -> float:
        """Return GNSS SOW without requiring a new GREAT input format."""
        source_sow = getattr(obsr, "source_sow", None)
        if source_sow is not None:
            return float(source_sow)
        _week, sow = unix_to_gpst(timestamp)
        return float(sow)

    def _add_rate_as_increment_imu(self, imu: ImuMeasurement) -> None:
        """Convert each rate interval once, then use native split handling."""
        if not imu.is_rate():
            raise TypeError("rate-to-increment processing requires rate IMU input")
        if self.imucur is None:
            self.imucur = imu
            self._static_detect.push(imu)
            self.last_qins = 2
            self.last_split_action = "none"
            self.last_split_ratio = None
            return

        previous = self.imucur
        dt = imu.timestamp - previous.timestamp
        if dt <= 0.0:
            raise ValueError("rate IMU timestamps must be strictly increasing")
        # ``self.imucur`` alternates between the raw rate sample and the
        # synthesized increment boundary from the previous conversion, so both
        # forms must be accepted here.  The rate payload itself is never
        # mutated; the boundary is rebuilt with the current interval.
        if previous.is_increment():
            previous_sow = previous.increment_view().sow
            previous_boundary = previous
        elif previous.is_rate():
            _week, previous_sow = unix_to_gpst(previous.timestamp)
            previous_boundary = ImuMeasurement(
                timestamp=previous.timestamp,
                week=previous.week,
                payload=IncrementImuData(
                    dtheta=np.zeros(3), dvel=np.zeros(3), dt=dt,
                    sow=previous_sow,
                ),
            )
        else:
            raise TypeError("rate-to-increment processing cannot mix payload forms")
        rate = imu.rate_view()
        current = ImuMeasurement(
            timestamp=imu.timestamp,
            week=imu.week,
            payload=IncrementImuData(
                dtheta=rate.gyro * dt,
                dvel=rate.accel * dt,
                dt=dt,
                sow=previous_sow + dt,
            ),
        )
        self.imucur = previous_boundary
        self._add_increment_imu(current)

    def _add_increment_imu(self, imu: ImuMeasurement) -> None:
        """Consume a native increment using KF-GINS head/update/tail order."""
        if not imu.is_increment():
            raise TypeError("native increment processing requires increment IMU input")
        if self.imucur is None:
            self.imucur = imu
            self._static_detect.push(imu)
            self.last_qins = 2
            self.last_split_action = "none"
            self.last_split_ratio = None
            return
        if not self.imucur.is_increment():
            raise TypeError("rate and increment IMU samples cannot be mixed")

        self.last_qins = 2
        self.last_split_action = "none"
        self.last_split_ratio = None
        cur = self.imucur
        segment_current = imu
        current_propagated = False

        while self.pending_obs:
            obsr, obsb, nav, t_gnss = self.pending_obs[0]
            gnss_sow = self._raw_gnss_sow(obsr, t_gnss)
            prev_sow = cur.increment_view().sow
            current_sow = segment_current.increment_view().sow
            # Snap a floating-point GNSS epoch onto the increment endpoint it
            # actually belongs to, so a strict ``>`` cannot defer an endpoint
            # measurement by one increment.
            if abs(gnss_sow - current_sow) <= _SOW_ENDPOINT_TOLERANCE_S:
                gnss_sow = current_sow
            elif abs(gnss_sow - prev_sow) <= _SOW_ENDPOINT_TOLERANCE_S:
                gnss_sow = prev_sow

            if gnss_sow < prev_sow:
                # 该端点已写过行, 只做量测修正
                logger.debug(
                    "过期增量 GNSS sow=%.9f < cur.sow=%.9f, 直接量测更新",
                    gnss_sow, prev_sow,
                )
                self._trigger_meas(cur, obsr, obsb, nav, t_gnss)
                self.last_qins = 2
                self.pending_obs.popleft()
                continue
            if gnss_sow > current_sow:
                break

            action, head, tail = split_increment_at_gnss(
                cur, segment_current, gnss_sow
            )
            self.last_split_action = action
            if action == "previous":
                # 量测落在 cur 端点上, 不产生新的机械编排行
                self._trigger_meas(cur, obsr, obsb, nav, t_gnss)
                self.last_qins = 2
                self.pending_obs.popleft()
                continue

            if action == "current":
                self._propagate_increment_segment(cur, segment_current)
                self._trigger_meas(segment_current, obsr, obsb, nav, t_gnss)
                self._emit_propagation()
                self.pending_obs.popleft()
                cur = segment_current
                current_propagated = True
                continue

            if action == "split":
                self.last_split_ratio = float(
                    head.increment_view().dt
                    / segment_current.increment_view().dt
                )
                self._propagate_increment_segment(cur, head)
                self._trigger_meas(head, obsr, obsb, nav, t_gnss)
                self._emit_propagation()
                self.pending_obs.popleft()
                cur = head
                segment_current = tail
                continue

            # ``outside`` is handled by endpoint checks above.
            break

        if not current_propagated and segment_current.timestamp > cur.timestamp:
            self._propagate_increment_segment(cur, segment_current)
            self._emit_propagation()

    def add_gnss(self, obsr, obsb, nav) -> None:
        """添加 GNSS 原始观测: append 到 pending_obs deque。"""
        t_gnss = float(obsr.t.time + obsr.t.sec)
        if not self._initialized:
            self._init_obs.append((obsr, obsb, nav, t_gnss))
            # 限制 GNSS 缓冲区: 只保留最近 5 个历元 (初始化器内部 gnss_buffer=3)
            # 避免长时间未初始化时内存持续增长
            if len(self._init_obs) > 5:
                self._init_obs = self._init_obs[-5:]
            # 未初始化时输出纯 GNSS 解 (Qins=0, 1Hz GNSS频率, 姿态=0)
            self._write_gnss_only(obsr, obsb, nav, t_gnss)
            # 若 IMU 已覆盖第一个 GNSS obs 时间, 尝试初始化
            if self._init_imu and self._init_obs:
                first_t = self._init_obs[0][3]
                if self._init_imu[-1].timestamp >= first_t:
                    self._try_init()
            return
        self.pending_obs.append((obsr, obsb, nav, t_gnss))
        self._last_gnss_t = t_gnss

    # ===== 初始化 =====

    def _try_init(self) -> None:
        """用当前缓冲数据尝试初始化。成功则回放缓冲数据并切换增量模式。

        每次 _try_init 只处理最新 GNSS 历元, InsInitializer 内部 gnss_buffer
        通过 _align_motion_displacement 自然累积 (每次 append 一个历元)。
        当缓冲满 gnss_buffer_size 个历元且平面速度(EN)均达阈值时, 初始化成功。
        动态速度阈值: SPP > 3m/s, RTK/RTD > 2m/s。

        注: 不做早期返回检查, 否则 gnss_buffer 从第 gnss_buffer_size 个历元
        才开始填充, 延迟初始化 2 个历元。
        """
        if not self._init_obs:
            return

        # 只处理最新 GNSS 历元 (避免遍历所有历元导致 nav.x 被污染)
        obsr, obsb, nav, t_gnss = self._init_obs[-1]
        # 确保 IMU 数据已就绪 (动态初始化只需最新 IMU 数据, 不需要 5 秒缓冲)
        if not self._init_imu:
            return

        # 保存 nav 状态: SPP+相对定位会修改 nav 内部状态，完成后恢复。
        saved_x = nav.x.copy()
        saved_P = nav.P.copy()
        saved_fix = nav.fix.copy() if hasattr(nav, 'fix') else None
        saved_lock = nav.lock.copy() if hasattr(nav, 'lock') else None

        # SPP 粗定位 (GPS-only 避免 BDS/GAL 时间系统偏差导致发散)
        try:
            from src.core.gnss.rtklib.ephemeris import satposs
            from src.core.gnss.rtklib.pntpos import estpos
            from src.core.tc.tc_stream import _filter_gps_svh
            rs, var, dts, svh = satposs(obsr, nav)
            svh_gps = _filter_gps_svh(obsr, svh)
            if np.any(nav.rb):
                nav.x[0:3] = nav.rb
            sol, x_spp = estpos(obsr, nav, rs[:, :3], dts, svh_gps)
            if not sol.stat:
                self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)
                return
        except Exception as e:
            logger.debug(f"TC init SPP 异常: {e}")
            self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)
            return

        # RTK/RTD 模式: relpos 双差解算
        quality = 5
        rr = x_spp[:3].copy()
        ns = int(sol.ns)
        pos_sd = np.array([10.0, 10.0, 10.0])
        rtk_rr = None
        if self._mode in ("rtk", "rtd") and obsb is not None:
            try:
                from src.core.gnss.rtklib.rtkpos import relpos
                from src.core.gnss.rtklib.rtkcmn import Sol, SOLQ_NONE
                nav.x[0:6] = sol.rr[0:6]
                nav.x[6:9] = 1e-6
                rtk_sol = Sol()
                rtk_sol.t = obsr.t
                relpos(nav, obsr, obsb, rtk_sol)
                if rtk_sol.stat != SOLQ_NONE:
                    rtk_rr = rtk_sol.rr[:3].copy()
                    # 仅 RTK 保留 SPP 一致性检验以拦截载波 false fix。RTD
                    # 是码双差定位，必须使用其相对位置初始化而非被 SPP 阈值否决。
                    pos_diff = float(np.linalg.norm(rtk_rr - x_spp[:3]))
                    if self._mode == "rtk" and pos_diff > 50.0:
                        logger.warning(
                            f"TC init reject: RTK-SPP pos diff {pos_diff:.2f}m > 50m "
                            f"(t={t_gnss:.1f}, q={rtk_sol.stat}), skip init")
                        self._restore_nav(nav, saved_x, saved_P,
                                          saved_fix, saved_lock)
                        return
                    rr = rtk_rr
                    quality = 4 if self._mode == "rtd" else rtk_sol.stat
                    ns = rtk_sol.ns if rtk_sol.ns > 0 else (
                        nav.ns if nav.ns > 0 else ns)
                    if quality == 1:
                        pos_sd = np.array([0.1, 0.1, 0.1])
                    elif quality == 2:
                        pos_sd = np.array([0.3, 0.3, 0.3])
                    elif quality == 4:
                        pos_sd = np.array([1.0, 1.0, 1.0])
            except Exception as e:
                logger.debug(f"TC init RTK 异常: {e}")

        # 恢复 nav 状态 (SPP+RTK 已获取所需结果, 无需保留 nav 副作用)
        self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)

        # 缓存 GNSS 位置 (5 秒窗口, 用于动态初始化: 首尾位置差分计算 yaw)
        self._gnss_pos_cache.append((t_gnss, rr.copy()))
        # 限制缓存大小: 按时间保留最近 6s (span=5s 初始化窗口)
        # 注: 按历元数修剪假设 1Hz GNSS, 5Hz 数据 (如 Data19 ROVE.20O)
        # 6 历元仅覆盖 1.0s, 导致 span<5.0 恒不满足, TC 永远无法初始化
        if len(self._gnss_pos_cache) > 1:
            t_cut = self._gnss_pos_cache[-1][0] - 6.0
            if self._gnss_pos_cache[0][0] < t_cut:
                self._gnss_pos_cache = [e for e in self._gnss_pos_cache
                                        if e[0] >= t_cut]

        if len(self._gnss_pos_cache) < 2:
            return

        t_first, pos_first = self._gnss_pos_cache[0]
        t_last, pos_last = self._gnss_pos_cache[-1]
        span = t_last - t_first
        if span < 5.0:
            return  # 不足 5 秒, 继续累积

        # 动态速度阈值: 基于运动检测所用位置源
        if quality == 1:  # FIX
            init_speed_thr = 2.0
        else:  # FLOAT / DGPS / SPP
            init_speed_thr = 3.0

        # 首尾位置差分计算速度矢量 (5 秒窗口)
        vel_e = (pos_last - pos_first) / span

        # 平面速度 (EN) 检查
        lat, lon, _ = ecef2llh(pos_last)
        C_e_n = cal_Ce2n(lat, lon)
        vel_n = C_e_n @ vel_e
        planar_speed = math.sqrt(vel_n[0] ** 2 + vel_n[1] ** 2)
        if planar_speed < init_speed_thr:
            logger.debug(f"TC init speed={planar_speed:.2f} < {init_speed_thr} "
                         f"t={t_gnss:.1f} span={span:.1f}s")
            return

        # yaw = atan2(v_E, v_N), pitch=0, roll=0
        yaw = math.atan2(vel_n[1], vel_n[0])
        att_rpy = np.array([0.0, 0.0, yaw], dtype=np.float64)

        # 初始化位置: RTK/RTD 使用 rover/base 相对解，SPP 使用单点解。
        if self._mode in ("rtk", "rtd") and obsb is not None and quality != 5:
            pos_for_state = rr.copy()
        else:
            pos_for_state = pos_last.copy()

        gnss_sol = GnssSolution(
            timestamp=t_gnss, week=0, position=pos_for_state,
            quality=quality, num_sv=ns, sd=pos_sd,
            velocity=vel_e, vel_sd=None,
        )

        # 装配初始状态 (使用 InsInitializer 的 _assemble_state / _set_initial_variance)
        init_state = self._initializer._assemble_state(
            gnss_sol, att_rpy, vel_e, InitMode.POSITION_DIFF)
        init_P = self._initializer._set_initial_variance(InitMode.POSITION_DIFF)

        # 创建估计器 + 量测构造器
        self._est = TcEstimator(init_state, init_P, self._cfg, self._mode)
        builder_cls = _MEAS_BUILDERS.get(self._mode, SppTcMeas)
        self._meas_builder = builder_cls(self._cfg)
        # 用 SPP 钟差初始化 direct clk estimate (加速收敛, 否则需 150s+ 收敛)
        si = self._est.si
        if si.clk_bias >= 0 and len(x_spp) >= 6:
            self._est._clk_stored[0] = float(x_spp[3])   # GPS
            self._est._clk_stored[1] = float(x_spp[4])   # GLO
            self._est._clk_stored[2] = float(x_spp[5])   # GAL
            if len(x_spp) >= 7:
                self._est._clk_stored[3] = float(x_spp[6])   # BDS
            for k in range(4):
                self._est.P[si.clk_bias + k, si.clk_bias + k] = 10.0 ** 2
        self._initialized = True
        self._last_q = quality
        self._last_ns = ns
        self._last_gnss_t = init_state.timestamp

        logger.info(
            f"TcIntegration 初始化成功: t={init_state.timestamp:.3f}, "
            f"mode={self._mode}, span={span:.1f}s, "
            f"planar_speed={planar_speed:.2f}m/s, yaw={math.degrees(yaw):.1f}°")

        # 回放缓冲中 init 时间戳之后的事件
        self._replay_buffer(init_state.timestamp)

        # 清空初始化缓冲
        self._init_imu.clear()
        self._init_obs.clear()
        self._gnss_pos_cache.clear()

    def _replay_buffer(self, init_ts: float) -> None:
        """回放初始化缓冲中 init_ts 之后的事件。"""
        events = []
        for imu in self._init_imu:
            if imu.timestamp > init_ts:
                events.append((imu.timestamp, "imu", imu))
        for j in range(len(self._init_obs)):
            obsr, obsb, nav, t_gnss = self._init_obs[j]
            if t_gnss > init_ts:
                events.append((t_gnss, "gnss", (obsr, obsb, nav)))
        # 同时间戳时 GNSS 先于 IMU
        events.sort(key=lambda e: (e[0], 0 if e[1] == "gnss" else 1))

        for _, tag, data in events:
            if tag == "imu":
                self.add_imu(data)
            else:
                obsr, obsb, nav = data
                self.add_gnss(obsr, obsb, nav)

    # ===== TC 量测触发 =====

    def _trigger_meas(self, interp_imu: ImuMeasurement,
                      obsr, obsb, nav, t_gnss: float) -> None:
        """GVINS 风格: 构造 TC 量测 + 触发更新。

        Args:
            interp_imu: 插值到 t_gnss 的 IMU (或 cur if t_gnss==cur.t)
            obsr/obsb/nav: GNSS 原始观测
            t_gnss: GNSS 时间戳
        """
        si = self._est.si
        # 重置钟差为白噪声模型 (必须在 effective_x/build 之前)
        # 这样 innovation 用 clk=0 计算, KF 每历元独立估计钟差,
        # 避免 SPP 初始化的 (pos,clk) 自洽性导致位置误差被钟差吸收
        self._est.reset_clk_variance()
        # 构造 effective_x: amb/clk 部分用 effective (stored + error)
        # 此时 clk_stored=0, x[clk]=0, 故 effective clk=0
        x = self._est.effective_x()
        mode = self._degrade.current_mode
        previous_obs_t = self._prev_obs_t

        # 按当前模式构造量测
        try:
            if mode == "spp":
                v, H, R, info = self._meas_builder.build(
                    self._est.state, obsr, nav, si, x=x)
            elif mode == "rtk":
                v, H, R, info = self._meas_builder.build(
                    self._est.state, obsr, nav, si, x=x, obsb=obsb,
                    P=self._est.P, amb_init_target=self._est,
                    previous_obs_t=previous_obs_t)
            else:   # rtd
                v, H, R, info = self._meas_builder.build(
                    self._est.state, obsr, nav, si, x=x, obsb=obsb)
        except Exception as e:
            logger.warning(f"TC meas build 异常 (mode={mode}): {e}")
            self._degrade.on_fail(self._est, "build_error")
            return

        # udbias 已在 build 内执行 (rtk 模式), 更新上一历元时刻供下一历元
        self._prev_obs_t = copy(obsr.t)

        if len(v) == 0:
            # 量测构建返回空 (卫星被 outlier 拒绝/共视卫星不足):
            # 用 SPP 3D 位置做 fallback 位置更新, 防止 INS 自由漂移
            logger.debug(f"TC no_meas (mode={mode}, t={t_gnss:.3f}): "
                         f"obsr sats={len(obsr.sat)}, obsb sats={len(obsb.sat) if obsb is not None else 0}")
            if self._spp_fallback_update(obsr, obsb, nav, t_gnss):
                return
            self._on_meas_failure(obsr, obsb, nav, t_gnss)
            return

        # 更新 num_sv / quality
        n_meas = info.get("n", len(v))
        self._last_ns = n_meas
        # Q 值: SPP=5, RTD=4, RTK FIX=1, RTK FLOAT=2
        # armode=0 (无模糊度解算) 时 _amb_fixed 始终为 False → Q=2 (FLOAT)
        if mode == "spp":
            self._last_q = 5
        elif mode == "rtk":
            self._last_q = 1 if self._amb_fixed else 2
        else:  # rtd
            self._last_q = 4

        # 量测数不足时用 SPP 位置 fallback (防止 INS 自由漂移)
        # 1-3 个伪距无法约束 15+ 维状态, 但 SPP 最小二乘能解算 3D 位置
        min_meas = 4
        if n_meas < min_meas:
            logger.debug(f"TC skip_meas (mode={mode}, t={t_gnss:.3f}): "
                         f"n_meas={n_meas} < {min_meas}, try SPP fallback, "
                         f"obsr={len(obsr.sat)}, obsb={len(obsb.sat) if obsb is not None else 0}")
            if self._spp_fallback_update(obsr, obsb, nav, t_gnss):
                return
            self._on_meas_failure(obsr, obsb, nav, t_gnss, recover=False)
            return

        # 收敛期保护: 前 _convergence_warmup 个历元禁用跳变检验
        # 初始化后模糊度未收敛, 位置会自然调整, 跳变检验会误拒
        in_warmup = self._meas_count < self._convergence_warmup

        # 注: 移除 NIS 检验和 false_fix 检测 (TC vs SPP 一致性)
        # 这些机制过度拒绝有效量测导致滤波器发散 (96/173286 历元通过)
        # ignav 仅用 chi-square 检验残差 (valsol), 无 NIS/SPP 一致性检验
        # false fix 通过正确协方差矩阵和模糊度管理预防, 而非事后拒绝

        # RTK 模糊度管理
        if self._ambiguity_enabled(mode, si.has_ambiguity()):
            self._handle_ambiguity(info, obsr, nav)

        # 记录量测更新前位置 (用于跳变检测)
        pre_update_pos = self._est.state.pos_e.copy()

        # Capture the exact feedback boundary.  The nominal state changes in
        # feedback(), so all pre-update values must be copied before this call.
        pre_velocity = self._est.state.vel_e.copy()
        pre_p = self._est.P.copy()
        pre_x = self._est.x.copy()
        # 量测更新前的 S / K 结构 (诊断): S = H·P·Hᵀ + R, K = P·Hᵀ·S⁻¹
        s_matrix = H @ pre_p @ H.T + R
        s_diag_median = float(np.median(np.diag(s_matrix)))
        s_inv = np.linalg.inv(s_matrix)
        K_gain = pre_p @ H.T @ s_inv
        k_pos_norm = float(np.linalg.norm(K_gain[si.pos:si.pos + 3], axis=1).max())
        k_vel_norm = float(np.linalg.norm(K_gain[si.vel:si.vel + 3], axis=1).max())
        k_att_norm = float(np.linalg.norm(K_gain[si.att:si.att + 3], axis=1).max())
        k_ba_norm = float(np.linalg.norm(K_gain[si.accel_bias:si.accel_bias + 3], axis=1).max())
        k_bg_norm = float(np.linalg.norm(K_gain[si.gyro_bias:si.gyro_bias + 3], axis=1).max())
        pre_pos_var_median = float(np.median(np.diag(pre_p[si.pos:si.pos + 3, si.pos:si.pos + 3])))
        if si.has_ambiguity():
            amb = slice(si.amb_start, si.amb_start + si.n_amb)
            pre_amb_var_median = float(np.median(np.diag(pre_p[amb, amb])))
            pre_pos_amb_cov_norm = float(np.linalg.norm(pre_p[si.pos:si.pos + 3, amb]))
            h_amb_max = float(np.abs(H[:, amb]).max()) if H.shape[1] > si.amb_start else 0.0
            k_amb_norm = float(np.linalg.norm(K_gain[amb, :], axis=1).max())
            post_amb_var_median = -1.0  # 由 post_p 在诊断里计算
        else:
            pre_amb_var_median = -1.0
            pre_pos_amb_cov_norm = -1.0
            h_amb_max = 0.0
            k_amb_norm = -1.0
            post_amb_var_median = -1.0
        feedback_x = self._est.tc_meas_update(v, H, R, source=mode)
        if feedback_x is None:
            reject_info = dict(info)
            reject_info["postfit_norm"] = float(
                np.linalg.norm(np.asarray(v) - H @ pre_x))
            self._record_matrix_update_diagnostic(
                timestamp=t_gnss,
                pre_p=pre_p,
                post_p=pre_p,
                innovation=np.asarray(v) - H @ pre_x,
                s_matrix=s_matrix,
                k_gain=K_gain,
                feedback_x=np.zeros_like(self._est.x),
                accepted=False,
                info=reject_info,
            )
            logger.debug(
                "TC postfit_reject (mode=%s, t=%.3f): discard measurement epoch",
                mode, t_gnss)
            # ignav valpos() keeps the previous INS state when an RTK float
            # solution fails validation.  A coarse SPP feedback here would
            # overwrite it with a 20-30 m position correction and defeat the
            # post-fit gate.
            self._on_meas_failure(obsr, obsb, nav, t_gnss)
            return
        update_info = dict(info)
        postfit = np.asarray(v) - H @ feedback_x
        update_info["postfit_norm"] = float(np.linalg.norm(postfit))
        update_info["postfit_chi2"] = float(np.sum(
            postfit * postfit / np.diag(R)))
        self._record_matrix_update_diagnostic(
            timestamp=t_gnss,
            pre_p=pre_p,
            post_p=self._est.P,
            innovation=np.asarray(v) - H @ pre_x,
            s_matrix=s_matrix,
            k_gain=K_gain,
            feedback_x=feedback_x,
            accepted=True,
            info=update_info,
        )
        if si.has_ambiguity():
            post_amb_var_median = float(np.median(np.diag(self._est.P[amb, amb])))
        self._record_measurement_diagnostic(
            timestamp=t_gnss,
            mode=mode,
            n_meas=n_meas,
            innovation=v,
            pre_velocity=pre_velocity,
            velocity_correction=feedback_x[si.vel:si.vel + 3],
            attitude_correction=feedback_x[si.att:si.att + 3],
            gyro_bias_correction=feedback_x[si.gyro_bias:si.gyro_bias + 3],
            accel_bias_correction=feedback_x[si.accel_bias:si.accel_bias + 3],
            pre_p=pre_p,
            post_p=self._est.P,
            s_diag_median=s_diag_median,
            k_pos_norm=k_pos_norm,
            k_vel_norm=k_vel_norm,
            k_att_norm=k_att_norm,
            k_ba_norm=k_ba_norm,
            k_bg_norm=k_bg_norm,
            pre_pos_var_median=pre_pos_var_median,
            pre_amb_var_median=pre_amb_var_median,
            pre_pos_amb_cov_norm=pre_pos_amb_cov_norm,
            post_amb_var_median=post_amb_var_median,
            h_amb_max=h_amb_max,
            k_amb_norm=k_amb_norm,
            n_phase_att=int(info.get("n_phase_att", 0)),
            n_phase_acc=int(info.get("n_phase_acc", 0)),
            n_code_att=int(info.get("n_code_att", 0)),
            n_code_acc=int(info.get("n_code_acc", 0)),
        )
        # 轻量更新信息快照 (供 stat_writer 的 GIPY_INNOV/GIPY_GAIN 使用)
        self.last_update_info = {
            "mode": mode,
            "innovation_norm": float(np.linalg.norm(v)),
            "n_meas": int(n_meas),
            "n_phase_acc": int(info.get("n_phase_acc", 0)),
            "n_code_acc": int(info.get("n_code_acc", 0)),
            "k_pos_norm": float(k_pos_norm),
            "k_vel_norm": float(k_vel_norm),
            "k_att_norm": float(k_att_norm),
            "k_bg_norm": float(k_bg_norm),
            "k_ba_norm": float(k_ba_norm),
            "ref_sats": info.get("ref_sats", []),
        }
        self._degrade.on_success(self._est)
        self.last_qins = 3  # TC 量测更新完成
        self._meas_count += 1
        self._consecutive_failures = 0  # 量测成功, 重置失败计数

        # 位置跳变检测: 若单次量测更新导致位置跳变 > 50m, 视为 false fix, 回滚
        # 收敛期跳过此检验 (模糊度收敛过程中位置会自然调整)
        # 阈值 50m: 仅拦截极端 false fix, 允许 RTK FLOAT 的正常调整
        if not in_warmup:
            post_update_pos = self._est.state.pos_e
            pos_jump = float(np.linalg.norm(post_update_pos - pre_update_pos))
            if pos_jump > 50.0:
                logger.warning(
                    f"TC pos_jump_reject (mode={mode}, t={t_gnss:.3f}): "
                    f"jump={pos_jump:.2f}m > 30m, false fix suspected, "
                    f"reset ambiguity & rollback")
                # 回滚位置 (恢复更新前状态)
                self._est.state.pos_e = pre_update_pos
                # 重置模糊度 (清空 stored, 放大 P 对角线, 清零交叉项)
                if si.has_ambiguity():
                    self._est._N_stored[:] = 0.0
                    amb_slice = slice(si.amb_start, si.amb_start + si.n_amb)
                    self._est.P[:, amb_slice] = 0.0
                    self._est.P[amb_slice, :] = 0.0
                    for k in range(si.n_amb):
                        self._est.P[si.amb_start + k, si.amb_start + k] = 100.0 ** 2
                self._amb_fixed = False
                self._ambiguity.reset()
                # 不降级: 回滚位置 + 重置模糊度即可, 降级到 imu_only 更危险
                return

        self._last_meas_pos = self._est.state.pos_e.copy()
        self._maybe_align_yaw(t_gnss)

    def _maybe_align_yaw(self, t_gnss: float) -> None:
        """一次性 yaw 航向对齐 (等价 ignav ant2inins/vel2head 语义)。

        背景: RTD-TC 码差量测对姿态零可观测 (零杆臂时 H[att]=0), 静态初始化
        yaw 完全未知 (P_yaw=π²); 手机 MEMS 陀螺漂移使 yaw 在传播中发散到
        数十度, 重力泄漏产生 m/s² 级虚假加速度。车辆起步后, 用自身更新点
        位置在时间窗内的位移方向作为航向观测 (公共水平偏置在差分中抵消),
        一次性把 yaw 拉齐并把 P_yaw 收紧。

        触发条件(全部满足, 且仅执行一次):
          - 配置 ins.yaw_align_on_move=1
          - 尚未对齐
          - 已积累 >= yaw_align_window 秒的对齐缓存
          - 缓存首尾水平位移 >= yaw_align_min_disp 米 (保证航向可观测)
        """
        ins_cfg = self._cfg.get("ins", {})
        if not int(ins_cfg.get("yaw_align_on_move", 0)) or self._yaw_aligned:
            return
        pos = self._est.state.pos_e.copy()
        self._yaw_hist.append((t_gnss, pos))
        win = float(ins_cfg.get("yaw_align_window", 10.0))
        min_disp = float(ins_cfg.get("yaw_align_min_disp", 15.0))
        t0, p0 = self._yaw_hist[0]
        if t_gnss - t0 < win:
            return
        d = pos - p0
        disp_h = float(np.hypot(d[0], d[1]))
        if disp_h < min_disp:
            self._yaw_hist.popleft()
            return
        # ECEF 位移 -> ENU 航向
        lat, lon, _ = ecef2llh(pos)
        sl, cl, so, co = np.sin(lat), np.cos(lat), np.sin(lon), np.cos(lon)
        R = np.array([[-so, co, 0.0],
                      [-sl * co, -sl * so, cl],
                      [cl * co, cl * so, sl]])
        enu = R @ d
        heading_ned = math.atan2(enu[0], enu[1])  # atan2(East, North)

        state = self._est.state
        roll, pitch, _ = state.att_rpy
        cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll),
                                  math.cos(pitch), math.sin(pitch),
                                  math.cos(heading_ned), math.sin(heading_ned))
        C_b_n = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ])
        C_e_n = cal_Ce2n(lat, lon)
        new_Cbe = C_e_n.T @ C_b_n
        from src.core.ins.attitude import dcm2quat, quat2dcm
        state.C_b_e = quat2dcm(dcm2quat(new_Cbe))
        state.q_b_e = dcm2quat(state.C_b_e)
        state.att_rpy = np.array([roll, pitch, heading_ned])
        si = self._est.si
        yaw_var = float(ins_cfg.get("yaw_align_std", 5.0)) ** 2
        for k in range(3):
            self._est.P[si.att + k, si.att + k] = max(
                yaw_var if k == 2 else self._est.P[si.att + k, si.att + k], 0.0)
            self._est.x[si.att + k] = 0.0
        self._yaw_aligned = True
        logger.warning(
            f"TC yaw aligned from displacement: heading={math.degrees(heading_ned):.1f} deg, "
            f"disp={disp_h:.1f}m over {t_gnss - t0:.1f}s")

    def _ambiguity_enabled(self, mode: str, has_ambiguity: bool) -> bool:
        """判断当前 TC 配置是否允许执行模糊度固定/保持。

        ignav 的 ``pos2-armode=off`` 不仅禁止输出整数解，也禁止
        LAMBDA 和 holdamb 约束；TC 必须保持相同的配置语义。
        """
        armode = int(self._cfg.get("gnss", {}).get("armode", 0))
        return mode == "rtk" and armode > 0 and has_ambiguity

    def _record_measurement_diagnostic(self, timestamp: float, mode: str,
                                       n_meas: int, innovation: np.ndarray,
                                       pre_velocity: np.ndarray,
                                       velocity_correction: np.ndarray,
                                       attitude_correction: np.ndarray,
                                       gyro_bias_correction: np.ndarray,
                                       accel_bias_correction: np.ndarray,
                                       pre_p: np.ndarray,
                                       post_p: np.ndarray,
                                       s_diag_median: float = -1.0,
                                       k_pos_norm: float = -1.0,
                                       k_vel_norm: float = -1.0,
                                       k_att_norm: float = -1.0,
                                       k_ba_norm: float = -1.0,
                                       k_bg_norm: float = -1.0,
                                       pre_pos_var_median: float = -1.0,
                                       pre_amb_var_median: float = -1.0,
                                       pre_pos_amb_cov_norm: float = -1.0,
                                       post_amb_var_median: float = -1.0,
                                       h_amb_max: float = -1.0,
                                       k_amb_norm: float = -1.0,
                                       n_phase_att: int = 0,
                                       n_phase_acc: int = 0,
                                       n_code_att: int = 0,
                                       n_code_acc: int = 0) -> None:
        """Append one explicit measurement-boundary diagnostic CSV record."""
        if not self._diagnostics_path:
            return

        path = Path(self._diagnostics_path)
        path.parent.mkdir(parents=True, exist_ok=True)
        fieldnames = [
            "timestamp", "mode", "n_meas", "innovation_norm",
            "pre_velocity_norm", "velocity_feedback_x",
            "velocity_feedback_y", "velocity_feedback_z",
            "velocity_feedback_norm", "pre_vel_var_x", "pre_vel_var_y",
            "pre_vel_var_z", "post_vel_var_x", "post_vel_var_y",
            "post_vel_var_z", "pre_pos_vel_cov_norm",
            "post_pos_vel_cov_norm", "pre_pos_att_cov_norm",
            "post_pos_att_cov_norm", "pre_vel_att_cov_norm",
            "post_vel_att_cov_norm", "pre_pos_gbias_cov_norm",
            "post_pos_gbias_cov_norm", "pre_vel_gbias_cov_norm",
            "post_vel_gbias_cov_norm", "attitude_feedback_norm",
            "gyro_bias_feedback_x", "gyro_bias_feedback_y",
            "gyro_bias_feedback_z", "gyro_bias_feedback_norm",
            "accel_bias_feedback_norm",
            "accel_bias_feedback_x", "accel_bias_feedback_y", "accel_bias_feedback_z",
            "accel_bias_state_x", "accel_bias_state_y", "accel_bias_state_z",
            "s_diag_median", "k_pos_norm", "k_vel_norm", "k_att_norm",
            "k_ba_norm", "k_bg_norm", "pre_pos_var_median",
            "pre_amb_var_median", "pre_pos_amb_cov_norm",
            "post_amb_var_median", "h_amb_max", "k_amb_norm",
            "n_phase_att", "n_phase_acc", "n_code_att", "n_code_acc",
        ]
        pre_vel_var = np.diag(pre_p[3:6, 3:6])
        post_vel_var = np.diag(post_p[3:6, 3:6])
        row = {
            "timestamp": float(timestamp),
            "mode": mode,
            "n_meas": int(n_meas),
            "innovation_norm": float(np.linalg.norm(innovation)),
            "pre_velocity_norm": float(np.linalg.norm(pre_velocity)),
            "velocity_feedback_x": float(velocity_correction[0]),
            "velocity_feedback_y": float(velocity_correction[1]),
            "velocity_feedback_z": float(velocity_correction[2]),
            "velocity_feedback_norm": float(np.linalg.norm(velocity_correction)),
            "pre_vel_var_x": float(pre_vel_var[0]),
            "pre_vel_var_y": float(pre_vel_var[1]),
            "pre_vel_var_z": float(pre_vel_var[2]),
            "post_vel_var_x": float(post_vel_var[0]),
            "post_vel_var_y": float(post_vel_var[1]),
            "post_vel_var_z": float(post_vel_var[2]),
            "pre_pos_vel_cov_norm": float(np.linalg.norm(pre_p[0:3, 3:6])),
            "post_pos_vel_cov_norm": float(np.linalg.norm(post_p[0:3, 3:6])),
            "pre_pos_att_cov_norm": float(np.linalg.norm(pre_p[0:3, 6:9])),
            "post_pos_att_cov_norm": float(np.linalg.norm(post_p[0:3, 6:9])),
            "pre_vel_att_cov_norm": float(np.linalg.norm(pre_p[3:6, 6:9])),
            "post_vel_att_cov_norm": float(np.linalg.norm(post_p[3:6, 6:9])),
            "pre_pos_gbias_cov_norm": float(np.linalg.norm(pre_p[0:3, 9:12])),
            "post_pos_gbias_cov_norm": float(np.linalg.norm(post_p[0:3, 9:12])),
            "pre_vel_gbias_cov_norm": float(np.linalg.norm(pre_p[3:6, 9:12])),
            "post_vel_gbias_cov_norm": float(np.linalg.norm(post_p[3:6, 9:12])),
            "attitude_feedback_norm": float(np.linalg.norm(attitude_correction)),
            "gyro_bias_feedback_x": float(gyro_bias_correction[0]),
            "gyro_bias_feedback_y": float(gyro_bias_correction[1]),
            "gyro_bias_feedback_z": float(gyro_bias_correction[2]),
            "gyro_bias_feedback_norm": float(np.linalg.norm(gyro_bias_correction)),
            "accel_bias_feedback_norm": float(np.linalg.norm(accel_bias_correction)),
            "accel_bias_feedback_x": float(accel_bias_correction[0]),
            "accel_bias_feedback_y": float(accel_bias_correction[1]),
            "accel_bias_feedback_z": float(accel_bias_correction[2]),
            "accel_bias_state_x": float(self._est.state.accel_bias[0]) if self._est is not None else float("nan"),
            "accel_bias_state_y": float(self._est.state.accel_bias[1]) if self._est is not None else float("nan"),
            "accel_bias_state_z": float(self._est.state.accel_bias[2]) if self._est is not None else float("nan"),
            "s_diag_median": float(s_diag_median),
            "k_pos_norm": float(k_pos_norm),
            "k_vel_norm": float(k_vel_norm),
            "k_att_norm": float(k_att_norm),
            "k_ba_norm": float(k_ba_norm),
            "k_bg_norm": float(k_bg_norm),
            "pre_pos_var_median": float(pre_pos_var_median),
            "pre_amb_var_median": float(pre_amb_var_median),
            "pre_pos_amb_cov_norm": float(pre_pos_amb_cov_norm),
            "post_amb_var_median": float(post_amb_var_median),
            "h_amb_max": float(h_amb_max),
            "k_amb_norm": float(k_amb_norm),
            "n_phase_att": int(n_phase_att),
            "n_phase_acc": int(n_phase_acc),
            "n_code_att": int(n_code_att),
            "n_code_acc": int(n_code_acc),
        }
        write_header = not path.exists() or path.stat().st_size == 0
        with path.open("a", encoding="utf-8", newline="") as fp:
            writer = csv.DictWriter(fp, fieldnames=fieldnames)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

    def _spp_fallback_update(self, obsr, obsb, nav, t_gnss: float) -> bool:
        """SPP 3D 位置 fallback: 当 TC 量测失败时用 SPP 位置约束 INS。

        场景: 卫星数不足 (<4 颗 GPS) 或残差超阈值被 outlier 拒绝时,
        TC 量测被跳过。INS 自由积分会导致垂直通道快速发散 (Schuler 不稳定)。
        SPP 最小二乘能正确处理 (pos, clk) 相关性, 即便几何差, 3D 位置
        精度 (~20m) 仍远优于纯 INS 漂移 (可达 50m+/30s)。

        策略: 用 SPP 位置做 LC 风格位置量测更新 (仅 pos 3 维, H=I),
        sigma 自适应: ns>=6 用 20m, ns<6 用 30m (几何差时增大 R 降低 K)。
        不更新速度/姿态/钟差/模糊度 (H 对应列为 0)。
        """
        from src.core.gnss.rtklib.ephemeris import satposs
        from src.core.gnss.rtklib.pntpos import estpos
        from src.core.tc.tc_stream import _filter_gps_svh

        saved_x = nav.x.copy()
        saved_P = nav.P.copy()
        saved_fix = nav.fix.copy() if hasattr(nav, 'fix') else None
        saved_lock = nav.lock.copy() if hasattr(nav, 'lock') else None
        try:
            rs, var, dts, svh = satposs(obsr, nav)
            svh_gps = _filter_gps_svh(obsr, svh)
            if np.any(nav.rb):
                nav.x[0:3] = nav.rb
            sol, x_spp = estpos(obsr, nav, rs[:, :3], dts, svh_gps)
            if not sol.stat:
                self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)
                return False
        except Exception as e:
            self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)
            logger.debug(f"SPP fallback fail (t={t_gnss:.1f}): {e}")
            return False
        self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)

        spp_pos = x_spp[:3].copy()
        est = self._est
        si = est.si

        # 位置 innovation: Z = predicted - observed = state.pos - spp_pos
        Z = est.state.pos_e - spp_pos
        H = np.zeros((3, si.dim), dtype=np.float64)
        H[:, si.pos:si.pos + 3] = np.eye(3)
        # SPP sigma 自适应: 卫星少时增大 (几何差)
        n_gps = int(sol.ns) if sol.ns > 0 else 4
        sigma_spp = 20.0 if n_gps >= 6 else 30.0
        R = np.diag([sigma_spp ** 2] * 3).astype(np.float64)

        est.joseph_update(Z, H, R)
        est.feedback()

        self._last_q = 5        # SPP
        self._last_ns = n_gps
        self.last_qins = 3      # 量测更新完成
        self._meas_count += 1
        self._consecutive_failures = 0
        logger.debug(
            f"SPP fallback (t={t_gnss:.3f}): ns={n_gps}, "
            f"pos_innov={float(np.linalg.norm(Z)):.2f}m, sigma={sigma_spp}")
        return True

    def _on_meas_failure(self, obsr, obsb, nav, t_gnss: float,
                         recover: bool = True) -> None:
        """量测失败处理: 累计失败次数, 超阈值时尝试 SPP 恢复。

        当连续失败超 _recovery_threshold 个历元且 GNSS 观测可用时,
        尝试用 SPP 重新初始化位置, 使系统能从发散中恢复。
        """
        self._consecutive_failures += 1
        if not recover:
            return
        if self._consecutive_failures < self._recovery_threshold:
            return
        # 避免频繁尝试: 每次失败后才尝试, 成功后计数清零
        if self._consecutive_failures % self._recovery_threshold != 0:
            return
        self._try_recovery(obsr, obsb, nav, t_gnss)

    def _try_recovery(self, obsr, obsb, nav, t_gnss: float) -> bool:
        """SPP 恢复: 用 SPP 位置重置 INS 状态, 重置降级管理器。

        场景: 量测持续失败 (位置发散/卫星数不足), 降级到 imu_only 后
        位置漂移过远, 量测构建器无法构造有效量测 (zdres 残差过大被 outlier 拒绝)。
        恢复策略: 用 SPP 重新定位, 重置 INS 位置/速度/P, 重置降级管理器到初始模式。

        Returns:
            True 恢复成功, False 失败
        """
        from src.core.gnss.rtklib.ephemeris import satposs
        from src.core.gnss.rtklib.pntpos import estpos
        from src.core.tc.tc_stream import _filter_gps_svh

        saved_x = nav.x.copy()
        saved_P = nav.P.copy()
        saved_fix = nav.fix.copy() if hasattr(nav, 'fix') else None
        saved_lock = nav.lock.copy() if hasattr(nav, 'lock') else None
        try:
            rs, var, dts, svh = satposs(obsr, nav)
            svh_gps = _filter_gps_svh(obsr, svh)
            if np.any(nav.rb):
                nav.x[0:3] = nav.rb
            sol, x_spp = estpos(obsr, nav, rs[:, :3], dts, svh_gps)
            if not sol.stat:
                self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)
                logger.warning(
                    f"TC recovery_fail (t={t_gnss:.1f}): SPP stat={sol.stat}")
                return False
        except Exception as e:
            self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)
            logger.warning(f"TC recovery_excp (t={t_gnss:.1f}): {e}")
            return False
        self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)

        # SPP 成功: 用 SPP 位置重置 INS 状态
        spp_pos = x_spp[:3].copy()
        est = self._est
        si = est.si

        # 重置 INS 物理状态 (保留姿态: 发散期间姿态可能仍可信)
        est.state.pos_e = spp_pos.copy()
        est.state.vel_e = np.zeros(3, dtype=np.float64)

        # 重置 EKF 误差状态和协方差
        est.x[:] = 0.0
        # 位置: SPP 精度 ~10m
        for k in range(3):
            est.P[si.pos + k, si.pos + k] = 10.0 ** 2
        # 速度: 未知, 大方差
        for k in range(3):
            est.P[si.vel + k, si.vel + k] = 10.0 ** 2
        # 姿态: 保留原方差 (姿态可能仍可信)
        # 零偏/杆臂等: 保留原方差
        # 钟差: 重置
        if si.clk_bias >= 0:
            for k in range(4):
                est.P[si.clk_bias + k, si.clk_bias + k] = 100.0 ** 2
            est._clk_stored[:] = 0.0
            if len(x_spp) >= 6:
                est._clk_stored[0] = float(x_spp[3])
                est._clk_stored[1] = float(x_spp[4])
                est._clk_stored[2] = float(x_spp[5])
                if len(x_spp) >= 7:
                    est._clk_stored[3] = float(x_spp[6])
                for k in range(4):
                    est.P[si.clk_bias + k, si.clk_bias + k] = 10.0 ** 2
        # 模糊度: 重置
        if si.has_ambiguity():
            est._N_stored[:] = 0.0
            amb_slice = slice(si.amb_start, si.amb_start + si.n_amb)
            est.P[:, amb_slice] = 0.0
            est.P[amb_slice, :] = 0.0
            for k in range(si.n_amb):
                est.P[si.amb_start + k, si.amb_start + k] = 100.0 ** 2

        # 清零交叉协方差 (位置/速度/钟差/模糊度与其他状态的交叉项)
        reset_idx = list(range(si.pos, si.vel + 3))
        if si.clk_bias >= 0:
            reset_idx.extend(range(si.clk_bias, si.clk_bias + 4))
        if si.has_ambiguity():
            reset_idx.extend(range(si.amb_start, si.amb_start + si.n_amb))
        keep_idx = [i for i in range(si.dim) if i not in set(reset_idx)]
        for i in reset_idx:
            for j in keep_idx:
                est.P[i, j] = 0.0
                est.P[j, i] = 0.0

        # 重置降级管理器到初始模式
        self._degrade.current_mode = self._degrade.initial_mode
        self._degrade._fail_count = 0
        self._degrade._rebooted = False

        # 重置量测构造器到初始模式
        builder_cls = _MEAS_BUILDERS.get(self._mode, SppTcMeas)
        self._meas_builder = builder_cls(self._cfg)

        # 重置模糊度管理器
        self._ambiguity.reset()
        self._amb_fixed = False

        # 重置失败计数和收敛期保护 (恢复后需重新收敛)
        self._consecutive_failures = 0
        self._meas_count = 0

        logger.warning(
            f"TC recovery_ok (t={t_gnss:.1f}): SPP pos={spp_pos}, "
            f"reset to mode={self._degrade.initial_mode}")
        return True

    def _handle_ambiguity(self, info: dict, obsr, nav) -> None:
        """RTK 模糊度固定 (LAMBDA, 仅有效星) + ignav 风格 holdamb。

        1. 有效星筛选: 仅取本历元相位 DD 实际使用的模糊度槽位
           (对齐 rtklib ddidx; 全量 MAXSAT*nf 槽位含大量未跟踪星,
           协方差奇异导致 LD 失败/ratio 失真)
        2. try_fix: LAMBDA 整数搜索, 成功则对应槽位 N_stored=fixed, ε_N=0
           (部分固定: 未参与槽位保持浮点)
        3. holdamb: 对实际使用的模糊度添加约束量测 (v=0, R=VAR_HOLDAMB=0.001),
           通过 joseph_update 降低 P[amb] 同时保留交叉协方差。
           参考 ignav rtkpos.cc holdamb(): filter(x,P,H,v,R) 而非直接置 P。
        """
        si = self._est.si
        if not si.has_ambiguity():
            return
        # 仅取当前历元相位双差实际使用的模糊度槽位参与 AR。
        # 槽位按 MAXSAT*nf 全量分配, 未跟踪卫星的行列恒为 0/陈旧值,
        # 全维 LAMBDA 会因协方差奇异而失败或 ratio 失真 (实测非正定占
        # ~30% 历元、ratio 恒不通过); 对应 rtklib ddidx 只选有效共视星。
        pairs = info.get("pairs", [])
        phase_pairs_all = [p for p in pairs if p[3] == 0]
        if len(phase_pairs_all) < 2:
            self._amb_fixed = False
            return

        # ---- SD→DD 变换后 LAMBDA (基于 nav 侧一致协方差) ----
        # SD 模糊度存在公共基准方向 (全体 ±1 周期): 该方向从未被 DD 观测
        # 约束, 其方差保持初始化量级; 直接对 SD 子块做 LAMBDA 会沿该方向
        # 自由漂移 —— 候选解全体 +1 而残差几乎不变, ratio 恒 ≈1。
        # 对齐 rtklib resamb_LAMBDA: y = D·x, Qb = D·Q·Dᵀ; 固定后按
        # restamb 语义写回 (基准星保持浮点, 其余槽位 = 基准浮点值 − DD 整数)。
        from src.core.gnss.rtklib.rtkcmn import uGNSS

        def _nav_ib(sat: int, freq: int) -> int:
            return nav.na + uGNSS.MAXSAT * freq + sat - 1

        P_nav_diag = np.diag(nav.P)
        healthy = []
        seen_sat = set()
        for p in phase_pairs_all:
            frq = p[2]
            ok_var = all(
                1e-6 < P_nav_diag[_nav_ib(s, frq)] < 5.0e3 for s in (p[0], p[1])
            )
            if ok_var:
                healthy.append(p)
                seen_sat.update((p[0], p[1]))
        if len(healthy) < 2 or len(seen_sat) < 3:
            self._amb_fixed = False
            return

        # 基准星 = 出现最多的 DD 首元
        cnt = {}
        for p in healthy:
            cnt[p[0]] = cnt.get(p[0], 0) + 1
        ref_sat = max(cnt, key=cnt.get)
        ref_frq = next(p[2] for p in healthy if p[0] == ref_sat)

        dds = []  # (j_sat, j_frq, sign): y = N_ref − sign*N_j
        for p in healthy:
            if p[0] == ref_sat and p[2] == ref_frq:
                dds.append((p[1], p[2], 1.0))
            elif p[1] == ref_sat and p[2] == ref_frq:
                dds.append((p[0], p[2], -1.0))
        nb = len(dds)
        if nb <= 0:
            self._amb_fixed = False
            return

        r_i = _nav_ib(ref_sat, ref_frq)
        nb = len(dds)
        y_dd = np.zeros(nb)
        Q_dd = np.zeros((nb, nb))
        for k, (ks, kf, kg) in enumerate(dds):
            ki = _nav_ib(ks, kf)
            y_dd[k] = nav.x[r_i] - kg * nav.x[ki]
            # Q[k,m] = Var(N_r − N_k 与 N_r − N_m 的协方差)
            #        = Q[r,r] − Q[r,m'] − Q[k',r] + Q[k',m']
            for m, (ms, mf, mg) in enumerate(dds):
                mi = _nav_ib(ms, mf)
                Q_dd[k, m] = (nav.P[r_i, r_i] - nav.P[r_i, mi]
                              - nav.P[ki, r_i] + nav.P[ki, mi])

        posvar = float(np.mean(np.diag(
            self._est.P[si.pos:si.pos + 3, si.pos:si.pos + 3])))
        fixed_dd, ratio, ok = self._ambiguity.try_fix(y_dd, Q_dd, posvar)
        if not ok:
            self._amb_fixed = False
            return

        # restamb 写回: 基准星保持浮点; 其余 = 基准浮点值 − DD 整数 (TC+nav 双写)
        n_ref = float(nav.x[r_i])
        fixed_slots = []
        for k, (js, jf, sg) in enumerate(dds):
            new_n = n_ref - sg * fixed_dd[k]
            tc_idx = si.amb_idx(js, jf)
            compact = tc_idx - si.amb_start
            self._est._N_stored[compact] = new_n
            self._est.x[tc_idx] = 0.0
            nav.x[_nav_ib(js, jf)] = new_n
            fixed_slots.append(tc_idx)

        # ignav 风格 holdamb: 仅对本次实际固定的槽位添加约束量测
        # v[k]=0 (已写回整数), joseph_update(v=0,H,R) 不改变状态但收缩 P,
        # 保留交叉协方差; 未固定槽位不受影响
        VAR_HOLDAMB = 0.001  # cycle², 与 ignav rtkpos.cc 一致
        n_const = len(fixed_slots)
        v_const = np.zeros(n_const)
        H_const = np.zeros((n_const, si.dim))
        R_const = np.eye(n_const) * VAR_HOLDAMB
        for k, col in enumerate(fixed_slots):
            H_const[k, col] = 1.0
        self._est.joseph_update(v_const, H_const, R_const)
        self._est.feedback()

        logger.info(
            f"TC amb fixed: ratio={ratio:.2f}, nb={nb}, "
            f"n_const={n_const}, holdamb R={VAR_HOLDAMB}")
        self._amb_fixed = True

    # ===== 约束 =====

    def _apply_constraints(self, imu: ImuMeasurement) -> None:
        """NHC/ZUPT/ZARU 约束更新 (per-IMU, decimation, 互斥)。"""
        if not (self.nhc_enable or self.zupt_enable or self.zaru_enable):
            return

        state = self._est.state
        is_static = self._static_detect.detect(state.pos_e)

        applied = False
        if is_static:
            if self.zupt_enable and self._zupt_counter.should_trigger():
                if self._constraints.zupt(self._est):
                    applied = True
            if self.zaru_enable and self._zaru_counter.should_trigger():
                if self._constraints.zaru(self._est, imu):
                    applied = True
        elif (self.nhc_enable and self._nhc_counter.should_trigger()
              and self._meas_count >= self._nhc_warmup):
            if self._constraints.nhc(self._est, imu):
                applied = True

        if applied:
            self._est.feedback()
            self.last_qins = 3  # 约束量测更新完成

    # ===== 输出 =====

    def _write_gnss_only(self, obsr, obsb, nav, t_gnss: float) -> None:
        """未初始化时输出纯 GNSS 解 (Qins=0, 速度=0, 姿态=0)。

        根据 self._mode 选择解算方式:
          - spp: SPP 单点定位 (GPS-only 避免 BDS/GAL 时间偏差发散)
          - rtk/rtd: relpos 双差解算
        输出频率为 GNSS 频率 (1Hz)。
        """
        if self._writer is None:
            return

        # SPP 粗定位 (GPS-only, 保存/恢复 nav 状态)
        from src.core.tc.tc_stream import _filter_gps_svh
        saved_x = nav.x.copy()
        saved_P = nav.P.copy()
        saved_fix = nav.fix.copy() if hasattr(nav, 'fix') else None
        saved_lock = nav.lock.copy() if hasattr(nav, 'lock') else None
        try:
            from src.core.gnss.rtklib.ephemeris import satposs
            from src.core.gnss.rtklib.pntpos import estpos
            rs, var, dts, svh = satposs(obsr, nav)
            svh_gps = _filter_gps_svh(obsr, svh)
            if np.any(nav.rb):
                nav.x[0:3] = nav.rb
            sol, x_spp = estpos(obsr, nav, rs[:, :3], dts, svh_gps)
            if not sol.stat:
                self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)
                return
        except Exception as e:
            logger.debug(f"GNSS-only SPP 异常: {e}")
            self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)
            return

        quality = 5       # 默认 SPP
        rr = x_spp[:3].copy()
        ns = int(sol.ns)
        pos_sd = np.array([10.0, 10.0, 10.0])

        # RTK/RTD 模式: 用 relpos 双差解算
        # 注意: relpos 会修改 nav 内部状态, 保存/恢复避免污染
        if self._mode in ("rtk", "rtd") and obsb is not None:
            try:
                from src.core.gnss.rtklib.rtkpos import relpos
                from src.core.gnss.rtklib.rtkcmn import Sol, SOLQ_NONE
                nav.x[0:6] = sol.rr[0:6]
                nav.x[6:9] = 1e-6
                rtk_sol = Sol()
                rtk_sol.t = obsr.t
                relpos(nav, obsr, obsb, rtk_sol)
                if rtk_sol.stat != SOLQ_NONE:
                    rr = rtk_sol.rr[:3].copy()
                    quality = rtk_sol.stat
                    ns = rtk_sol.ns if rtk_sol.ns > 0 else (
                        nav.ns if nav.ns > 0 else ns)
                    if quality == 1:
                        pos_sd = np.array([0.1, 0.1, 0.1])
                    elif quality == 2:
                        pos_sd = np.array([0.3, 0.3, 0.3])
                    elif quality == 4:
                        pos_sd = np.array([1.0, 1.0, 1.0])
            except Exception as e:
                logger.debug(f"GNSS-only RTK 异常: {e}")
            finally:
                self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)
        else:
            self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)

        self._writer.write_gnss_only(t_gnss, rr, quality, ns, pos_sd)
        self._output_count += 1

    def _write_state(self, qins: int) -> None:
        """写当前状态到 .rslt 文件 (per-IMU 100Hz)。"""
        if self._writer is None:
            return
        self._writer.write(
            state=self._est.state,
            P=self._est.P,
            si=self._est.si,
            q=self._last_q,
            qins=qins,
            num_sv=self._last_ns,
        )
        self._output_count += 1

    def finalize(self) -> int:
        """流式结束, 返回总输出数。"""
        if not self._initialized:
            logger.warning("TcIntegration: 未初始化, 无 TC 输出")
            return 0
        logger.info(f"TcIntegration 输出: {self._output_count} 历元")
        return self._output_count
