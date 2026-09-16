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
from copy import copy, deepcopy
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
from src.core.tc.tc_ambiguity import (
    GREAT_MAXDEV_CYCLES,
    GREAT_MAXSIG_CYCLES,
    TcAmbiguity,
)
from src.core.tc.tc_degrade import TcDegradeManager
from src.core.tc.tc_estimator import TcEstimator
from src.core.tc.tc_measurement import SppTcMeas, RtkTcMeas, RtdTcMeas
from src.log.tc_init_propagation_trace import (
    build_initialization_input_record,
    build_trace_record,
)
from src.log.tc_matrix_diagnostics import TcMatrixDiagnosticWriter

logger = logging.getLogger(__name__)

_MEAS_BUILDERS = {"spp": SppTcMeas, "rtk": RtkTcMeas, "rtd": RtdTcMeas}

# Unix↔GPST conversion of a GNSS epoch and the SOW stored on an increment can
# differ by a few ULP at campus01 magnitudes (~1.8e5 s).  A GNSS epoch that is
# mathematically on an increment endpoint must still be consumed once by that
# increment, so snap such near-boundary SOW values.  The tolerance stays far
# below the 1 ms endpoint rule used by ``split_increment_at_gnss``.
_SOW_ENDPOINT_TOLERANCE_S = 1.0e-6
_INIT_CLONE_FIRST_SOW = 180634.0
_INIT_CLONE_LAST_SOW = 180635.0
_INIT_CLONE_SOW_STEP = 0.01
_INIT_CLONE_SOW_EPSILON = 1.0e-9
_INIT_HEADING_STABILITY_RAD = math.radians(10.0)


def _headings_are_stable(headings, *, max_span_rad=_INIT_HEADING_STABILITY_RAD,
                         minimum_count=3):
    """Check a short sequence of wrapped headings for stable motion.

    GREAT's POS alignment accepts a position-vector heading only after a
    subsequent vector agrees within 10 degrees.  The TC initializer keeps a
    five-second GNSS window, so use the last three endpoint vectors as the
    equivalent streaming evidence: all wrapped differences must fit inside
    that same 10-degree GREAT acceptance span.  This is an observability gate
    only; it does not alter any EKF noise, weighting, or residual threshold.
    """
    values = np.asarray(list(headings), dtype=float).reshape(-1)
    if values.size < int(minimum_count) or not np.all(np.isfinite(values)):
        return False
    reference = float(values[0])
    wrapped = np.arctan2(np.sin(values - reference),
                         np.cos(values - reference))
    return float(np.max(wrapped) - np.min(wrapped)) <= float(max_span_rad)


class TcIntegration:
    """紧组合导航集成 (GVINS 风格 IMU 消费 + TC 量测触发)。

    初始化前用小缓冲累积 IMU+GNSS 原始观测，初始化后增量处理。
    NHC/ZUPT/ZARU 作为可选约束, per-IMU 触发 (decimation 控制):
      - 静态 (StaticDetect): ZUPT + ZARU (互斥于 NHC)
      - 运动: NHC (需非剧烈转弯)
    """

    def __init__(self, config: dict, mode: str = "spp", output_callback=None,
                 measurement_trace_sink=None,
                 init_propagation_trace_sink=None,
                 initialization_input_trace_sink=None):
        self._cfg = config
        self._mode = mode
        self._output_callback = output_callback
        self._measurement_trace_sink = measurement_trace_sink
        self._init_propagation_trace_sink = init_propagation_trace_sink
        self._initialization_input_trace_sink = initialization_input_trace_sink
        # The interval is deliberately bounded by the first post-init GNSS
        # update.  This documents that these samples are not a GNSS-free run:
        # initialization itself already consumed GNSS observations.
        self._init_propagation_trace_active = False
        self._init_propagation_trace_first_gnss_sow = None
        self._init_propagation_trace_pending = (
            [] if init_propagation_trace_sink is not None else None
        )
        # Optional diagnostic-only fork.  It is created only with an explicit
        # trace sink so the default path has no clone or file side effects.
        self._init_propagation_clone = None
        self._init_propagation_clone_active = False
        self._init_propagation_clone_prev_sow = None
        self._init_propagation_clone_curr_sow = None
        self._init_propagation_clone_sample_count = 0
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

        # 模糊度 (RTK 才用)。参数对齐 GREAT gsetamb 与 rtklib manage_amb_LAMBDA:
        #   thresar   <- gnss.thresar      (GREAT <ratio> 3.0)
        #   thresar_var <- gnss.thresar1   (rtklib 位置方差门限)
        #   min_amb   <- gnss.minfixsats-1 (GREAT full_fix_num=3 / rtklib nb>=3)
        #   min_common_epochs <- gnss.minlock (GREAT min_common_time=30 s → 历元)
        #   part_fix  <- gnss.ar_part_fix  (GREAT part_fix, 默认关)
        gnss_cfg = config.get("gnss", {}) or {}
        self._ambiguity = TcAmbiguity(
            thresar=float(gnss_cfg.get("thresar", 3.0)),
            thresar_var=float(gnss_cfg.get("thresar1", 0.5)),
            hold_count=int(gnss_cfg.get("minfix", 10)),
            min_amb=int(gnss_cfg.get("minfixsats", 4)) - 1,
            min_common_epochs=int(gnss_cfg.get("minlock", 0)),
            part_fix=bool(gnss_cfg.get("ar_part_fix", False)),
            part_min_amb=int(gnss_cfg.get("part_fix_num", 2)),
            # GREAT widelane/narrowlane_decision 门限: 整型性判决
            # (max|浮点−整数| <= maxdev 周 且 max sigma <= maxsig 周)。
            # 默认取 GREAT 值; 协方差偏保守时可临时放宽 ar_maxsig 做归因。
            maxdev=float(gnss_cfg.get("ar_maxdev", GREAT_MAXDEV_CYCLES)),
            maxsig=float(gnss_cfg.get("ar_maxsig", GREAT_MAXSIG_CYCLES)),
        )

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
        # Source payload form for the latest diagnostic record.  This keeps a
        # rate-to-increment conversion from looking like an input increment in
        # provenance telemetry; it never participates in estimator routing.
        self._trace_last_input_form: str | None = None
        # GNSS 原始观测队列: (obsr, obsb, nav, t_gnss)
        self.pending_obs: collections.deque = collections.deque()

        # 初始化前缓冲
        self._init_imu: List[ImuMeasurement] = []
        self._init_obs: list = []   # [(obsr, obsb, nav, t_gnss), ...]
        # 5秒 GNSS 位置缓存 (用于动态初始化: 首尾位置差分计算 yaw)
        self._gnss_pos_cache: list = []  # [(t, pos_ecef, source_sow), ...]
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
        tc_cfg = config.get("tc", {}) if isinstance(config, dict) else {}
        # GREAT does not overwrite the INS trajectory with an unrelated SPP
        # position after a rejected DD epoch.  Keep the legacy fallbacks
        # available to other applications, but allow the formal GREAT
        # comparison profiles to disable both reset mechanisms explicitly.
        self._spp_fallback_enabled = bool(tc_cfg.get("spp_fallback_enable", True))
        self._auto_recovery_enabled = bool(tc_cfg.get("auto_recovery", True))
        self._velocity_guard_enabled = bool(tc_cfg.get("velocity_guard_enable", True))
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

    def _build_measurement_builder(self):
        """Create the selected builder with the optional DD trace sink.

        Both DD modes (rtk/rtd) accept the observational measurement trace;
        SPP has no DD rows and keeps the plain constructor.
        """
        builder_cls = _MEAS_BUILDERS.get(self._mode, SppTcMeas)
        if (self._mode in ("rtk", "rtd")
                and self._measurement_trace_sink is not None):
            return builder_cls(self._cfg, trace_sink=self._measurement_trace_sink)
        return builder_cls(self._cfg)

    def close(self) -> None:
        """Close optional diagnostic output without affecting the filter."""
        self._flush_pending_init_propagation_trace(
            first_gnss_observed=self._init_propagation_trace_first_gnss_sow
            is not None
        )
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
        if (self._est is None or
                (self._matrix_diagnostics is None and
                 self._init_propagation_trace_sink is None)):
            return
        snapshot = getattr(self._est, "last_propagation_snapshot", None)
        if snapshot is None:
            return
        self._emit_init_propagation_trace(snapshot)
        if self._matrix_diagnostics is None:
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

    def _trace_propagation_sow(self, timestamp: float) -> float | None:
        """Return only an explicitly carried source increment SOW."""
        current = self.imucur
        if current is not None:
            try:
                if (current.is_increment() and
                        abs(float(current.timestamp) - float(timestamp)) <= 1e-9):
                    return float(current.increment_view().sow)
            except (AttributeError, TypeError, ValueError):
                pass
        return None

    def _trace_imu_sow(self, imu) -> float | None:
        if imu is None:
            return None
        try:
            if imu.is_increment():
                return float(imu.increment_view().sow)
        except (AttributeError, TypeError, ValueError):
            return None
        return None

    def _emit_init_propagation_trace(self, snapshot: dict) -> None:
        """Emit one committed pre-first-GNSS propagation, when explicitly enabled."""
        if (self._init_propagation_trace_sink is None or
                not self._init_propagation_trace_active):
            return
        try:
            state = self._est.state
            current = self.imucur
            source_form = self._trace_last_input_form
            configured_source_form = str(
                self._cfg.get("ins", {}).get("imu_data_form", "")) or None
            if source_form is None:
                source_form = configured_source_form
            if source_form is None:
                source_form = (
                    "increment" if current is not None and
                    current.is_increment() else "rate"
                )
            source_form = str(source_form).strip().lower()
            propagation_form = (
                "increment" if current is not None and current.is_increment()
                else "rate"
            )
            source_sow_available = source_form != "rate"
            propagation_sow = (
                self._trace_propagation_sow(snapshot["timestamp"])
                if source_sow_available else None
            )
            propagation_start_sow = (
                self._trace_imu_sow(self.imupre)
                if source_sow_available else None
            )
            propagation_end_sow = (
                self._trace_imu_sow(current)
                if source_sow_available else None
            )
            ins_update = getattr(self._est, "ins_update", None)
            publication_allowed = (
                ins_update is None or
                getattr(ins_update, "last_update_accepted", True)
            )
            record = build_trace_record(
                "imu_propagation_pre_gnss", state,
                sow=propagation_sow,
                dt=snapshot["dt"], gnss_free=False,
                isolation="pre_first_gnss_update;not_gnss_free",
                first_gnss_sow=self._init_propagation_trace_first_gnss_sow,
                input_imu_form=source_form,
                propagation_form=propagation_form,
                imu_prev_sow=propagation_start_sow,
                imu_curr_sow=propagation_end_sow,
                imu_sample_count=1,
                gnss_measurement_inserted=0,
                main_run_gnss_update_seen=0,
                state_source=(
                    "main_run_gnss_assisted;pre_first_gnss_update;"
                    "not_gnss_free"
                ),
                state_epoch_sow=propagation_sow,
                published_event_epoch_sow=(
                    propagation_sow if publication_allowed else None),
                propagation_interval_start_sow=propagation_start_sow,
                propagation_interval_end_sow=propagation_end_sow,
                propagation_interval_dt_s=(
                    snapshot["dt"] if source_sow_available else None),
                position_frame_id="ECEF",
                velocity_frame_id="ECEF",
                attitude_frame_id="ECEF",
                velocity_provenance="ins_mechanization",
            )
            if (self._init_propagation_trace_first_gnss_sow is None and
                    self._init_propagation_trace_pending is not None):
                # The producer can deliver IMU ahead of the GNSS queue.  Hold
                # only this bounded diagnostic slice until the first GNSS
                # timestamp is known, so every row carries an honest bound.
                self._init_propagation_trace_pending.append(record)
            else:
                self._deliver_init_propagation_trace(record)
        except Exception as exc:
            # Diagnostics are strictly observational.  A malformed optional
            # snapshot must not change propagation or update ordering.
            logger.debug("TC init propagation trace unavailable: %s", exc)

    def _emit_initialization_complete_trace(self, state, *, sow=None) -> None:
        """Emit the real initialization handoff, explicitly GNSS-assisted."""
        if self._init_propagation_trace_sink is None:
            return
        try:
            record = build_trace_record(
                "initialization_complete", state, sow=sow, gnss_free=False,
                isolation="gnss_assisted_initialization;not_gnss_free",
                first_gnss_sow=self._init_propagation_trace_first_gnss_sow,
                state_source=(
                    "main_run_gnss_assisted_initialization;not_gnss_free"
                ),
            )
            self._deliver_init_propagation_trace(record)
        except Exception as exc:
            logger.debug("TC initialization trace unavailable: %s", exc)

    def _emit_initialization_input_trace(self, state, *, source_sow=None,
                                         raw_position=None,
                                         rtk_rr_ecef=None,
                                         rtk_antenna_ecef=None,
                                         raw_velocity=None,
                                         raw_attitude=None,
                                         derived_velocity=None,
                                         derived_attitude=None,
                                         velocity_diff_start_position_ecef=None,
                                         velocity_diff_end_position_ecef=None,
                                         velocity_diff_start_sow=None,
                                         velocity_diff_end_sow=None,
                                         initialization_gnss_week=None,
                                         initialization_gnss_sow=None,
                                         raw_imu_prev=None,
                                         raw_imu_curr=None,
                                         body_frame=None, body_order=None,
                                         nav_frame=None, nav_order=None,
                                         state_time_unix_s=None,
                                         state_time_source=None,
                                         imu_update_applied=None,
                                         imu_update_time_unix_s=None,
                                         imu_consumed=None,
                                         imu_consumed_time_unix_s=None,
                                         gnss_measurement_epoch_sow=None,
                                         gnss_source_epoch_sow=None,
                                         spp_epoch_sow=None,
                                         relpos_epoch_sow=None,
                                         cache_sample_start_sow=None,
                                         cache_sample_end_sow=None,
                                         state_epoch_sow=None,
                                         published_event_epoch_sow=None,
                                         propagation_interval_start_sow=None,
                                         propagation_interval_end_sow=None,
                                         propagation_interval_dt_s=None,
                                         position_frame_id=None,
                                         velocity_frame_id=None,
                                         attitude_frame_id=None) -> None:
        """Emit the raw-to-canonical initialization handoff, if requested."""
        sink = self._initialization_input_trace_sink
        if sink is None:
            return
        try:
            ins_cfg = self._cfg.get("ins", {})
            self._deliver_initialization_input_trace(
                build_initialization_input_record(
                    timestamp=getattr(state, "timestamp", None),
                    source_sow=source_sow,
                    initialization_gnss_week=initialization_gnss_week,
                    initialization_gnss_sow=initialization_gnss_sow,
                    raw_position=raw_position,
                    rtk_rr_ecef=rtk_rr_ecef,
                    rtk_antenna_ecef=rtk_antenna_ecef,
                    raw_velocity=raw_velocity,
                    raw_attitude=raw_attitude,
                    derived_velocity=derived_velocity,
                    derived_attitude=derived_attitude,
                    velocity_diff_start_position_ecef=(
                        velocity_diff_start_position_ecef),
                    velocity_diff_end_position_ecef=(
                        velocity_diff_end_position_ecef),
                    raw_lever=ins_cfg.get("leverarm"),
                    raw_imu_prev=raw_imu_prev,
                    raw_imu_curr=raw_imu_curr,
                    state=state,
                    position_frame="ECEF",
                    position_order="x,y,z",
                    position_units="m",
                    velocity_frame="ECEF",
                    velocity_order="x,y,z",
                    velocity_units="m/s",
                    attitude_frame="NED",
                    attitude_order="roll,pitch,yaw",
                    attitude_units="rad",
                    body_frame=body_frame,
                    body_order=body_order,
                    nav_frame=nav_frame,
                    nav_order=nav_order,
                    lever_frame="FRD",
                    lever_order="front,right,down",
                    lever_units="m",
                    lever_applied=True,
                    state_source="tc_integration_initialization",
                    position_source="tc_initialization_position_ecef",
                    velocity_source="tc_initialization_position_difference_ecef",
                    attitude_source="tc_initialization_ned_rpy_zyx",
                    lever_source="ins.leverarm_config",
                    derived_velocity_source=(
                        "tc_initialization_position_difference_ecef"
                    ),
                    derived_velocity_provenance=(
                        "tc_initialization;gnss_position_difference;"
                        "source_sow_interval"
                    ),
                    derived_velocity_frame="ECEF",
                    derived_velocity_order="x,y,z",
                    derived_velocity_units="m/s",
                    derived_attitude_source="tc_initialization_ned_rpy_zyx",
                    derived_attitude_provenance=(
                        "tc_initialization;derived_from_velocity_difference;"
                        "ned_rpy_zyx"
                    ),
                    derived_attitude_frame="NED",
                    derived_attitude_order="roll,pitch,yaw",
                    derived_attitude_units="rad",
                    position_provenance="tc_initialization;rover_solution_ecef",
                    velocity_provenance=(
                        "tc_initialization;gnss_position_difference;"
                        "source_sow_interval"
                    ),
                    attitude_provenance=(
                        "tc_initialization;derived_from_velocity_difference;"
                        "ned_rpy_zyx"
                    ),
                    lever_provenance="config;fixed_frd;imu_to_gnss",
                    velocity_diff_start_sow=velocity_diff_start_sow,
                    velocity_diff_end_sow=velocity_diff_end_sow,
                    velocity_diff_source="gnss_position_difference",
                    state_time_unix_s=state_time_unix_s,
                    state_time_source=state_time_source,
                    imu_update_applied=imu_update_applied,
                    imu_update_time_unix_s=imu_update_time_unix_s,
                    imu_consumed=imu_consumed,
                    imu_consumed_time_unix_s=imu_consumed_time_unix_s,
                    gnss_measurement_epoch_sow=gnss_measurement_epoch_sow,
                    gnss_source_epoch_sow=gnss_source_epoch_sow,
                    spp_epoch_sow=spp_epoch_sow,
                    relpos_epoch_sow=relpos_epoch_sow,
                    cache_sample_start_sow=cache_sample_start_sow,
                    cache_sample_end_sow=cache_sample_end_sow,
                    state_epoch_sow=state_epoch_sow,
                    published_event_epoch_sow=published_event_epoch_sow,
                    propagation_interval_start_sow=(
                        propagation_interval_start_sow),
                    propagation_interval_end_sow=(
                        propagation_interval_end_sow),
                    propagation_interval_dt_s=propagation_interval_dt_s,
                    position_frame_id=position_frame_id,
                    velocity_frame_id=velocity_frame_id,
                    attitude_frame_id=attitude_frame_id,
                )
            )
        except Exception as exc:
            # A diagnostic serialization failure must not alter initialization.
            logger.debug("TC initialization input trace unavailable: %s", exc)

    def _deliver_initialization_input_trace(self, record: dict) -> None:
        sink = self._initialization_input_trace_sink
        if sink is None:
            return
        writer = getattr(sink, "write", None)
        appender = getattr(sink, "append", None)
        if writer is not None:
            writer(record)
        elif appender is not None:
            appender(record)
        elif callable(sink):
            sink(record)
        else:
            raise TypeError(
                "initialization input trace sink must be callable, writable, "
                "or appendable"
            )

    def _start_init_propagation_clone(self, *, sow=None) -> None:
        """Start the opt-in GNSS-free mechanization diagnostic fork.

        The fork is a deep copy of the already initialized ``InsUpdate``.
        It receives only original IMU samples and is intentionally independent
        of the TC estimator, GNSS queue, constraints, feedback, and output.
        """
        if (self._init_propagation_trace_sink is None or
                not self._initialized or self._est is None or
                self._init_propagation_clone is not None):
            return
        source = getattr(self._est, "ins_update", None)
        if source is None:
            return
        try:
            clone = deepcopy(source)
            if sow is not None:
                try:
                    sow = float(sow)
                    if not np.isfinite(sow) or not 0.0 <= sow < 604800.0:
                        sow = None
                except (TypeError, ValueError):
                    sow = None
            self._init_propagation_clone = clone
            # A clone without a source SOW cannot be window-bounded.  Emit
            # its initialization metadata as unavailable, then leave the
            # optional fork inert instead of inventing a Unix-derived window.
            self._init_propagation_clone_active = sow is not None
            self._init_propagation_clone_prev_sow = sow
            self._init_propagation_clone_curr_sow = sow
            self._init_propagation_clone_sample_count = 0
            self._deliver_init_propagation_trace(build_trace_record(
                "initialization_complete", clone.state, sow=sow,
                gnss_free=True,
                isolation="gnss_free_clone;no_gnss_measurement",
                first_gnss_sow=None,
                imu_sample_count=0,
                gnss_measurement_inserted=0,
                main_run_gnss_update_seen=0,
                state_source="diagnostic_copy_no_gnss",
            ))
        except Exception as exc:
            # Optional diagnostics must never prevent the main run.
            logger.debug("TC GNSS-free init clone unavailable: %s", exc)
            self._init_propagation_clone = None
            self._init_propagation_clone_active = False

    @staticmethod
    def _init_clone_imu_sow(imu) -> float | None:
        if imu.is_increment():
            return float(imu.increment_view().sow)
        # Rate samples do not carry source SOW.  Their Unix timestamp is not
        # an acceptable substitute for the source/window trace contract.
        return None

    def _record_init_propagation_clone(self, imu) -> None:
        """Feed exactly one original IMU sample to the GNSS-free clone."""
        clone = self._init_propagation_clone
        if (clone is None or not self._init_propagation_clone_active):
            return
        try:
            previous_sow = self._init_propagation_clone_curr_sow
            # This is the only call made on the diagnostic copy.  In
            # particular, never route it through TcEstimator.time_update().
            clone.update(imu)
            if not clone.last_update_accepted:
                return
            current_sow = self._init_clone_imu_sow(imu)
            if current_sow is None:
                # The clone may still mechanize this sample, but its source
                # window is unbounded without an increment SOW.  Do not
                # fabricate one from the Unix timestamp.
                self._init_propagation_clone_active = False
                return
            self._init_propagation_clone_prev_sow = previous_sow
            self._init_propagation_clone_curr_sow = current_sow
            self._init_propagation_clone_sample_count = 1
            if current_sow + _INIT_CLONE_SOW_EPSILON < _INIT_CLONE_FIRST_SOW:
                return
            if current_sow - _INIT_CLONE_SOW_EPSILON > _INIT_CLONE_LAST_SOW:
                self._init_propagation_clone_active = False
                return
            source_form = "increment" if imu.is_increment() else "rate"
            self._deliver_init_propagation_trace(build_trace_record(
                "imu_propagation_pre_gnss", clone.state, sow=current_sow,
                dt=clone.last_dt, gnss_free=True,
                isolation="gnss_free_clone;no_gnss_measurement",
                first_gnss_sow=None,
                input_imu_form=source_form,
                propagation_form=source_form,
                imu_prev_sow=previous_sow,
                imu_curr_sow=current_sow,
                imu_sample_count=1,
                gnss_measurement_inserted=0,
                main_run_gnss_update_seen=0,
                state_source="diagnostic_copy_no_gnss",
            ))
            if current_sow + _INIT_CLONE_SOW_EPSILON >= _INIT_CLONE_LAST_SOW:
                self._init_propagation_clone_active = False
        except Exception as exc:
            logger.debug("TC GNSS-free init clone step unavailable: %s", exc)

    def _deliver_init_propagation_trace(self, record: dict) -> None:
        sink = self._init_propagation_trace_sink
        if sink is None:
            return
        writer = getattr(sink, "write", None)
        appender = getattr(sink, "append", None)
        if writer is not None:
            writer(record)
        elif appender is not None:
            appender(record)
        elif callable(sink):
            sink(record)
        else:
            raise TypeError(
                "TC init propagation trace sink must be callable, writable, "
                "or appendable"
            )

    def _flush_pending_init_propagation_trace(self, *,
                                              first_gnss_observed: bool) -> None:
        pending = self._init_propagation_trace_pending
        if not pending:
            return
        if first_gnss_observed:
            for record in pending:
                record["first_gnss_sow"] = float(
                    self._init_propagation_trace_first_gnss_sow)
                record["isolation"] = "pre_first_gnss_update;not_gnss_free"
                record["gnss_isolation"] = record["isolation"]
                self._deliver_init_propagation_trace(record)
        else:
            for record in pending:
                record["isolation"] = (
                    "pre_first_gnss_update;first_gnss_not_observed;"
                    "not_gnss_free"
                )
                record["gnss_isolation"] = record["isolation"]
                self._deliver_init_propagation_trace(record)
        pending.clear()

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

    def _emit_measurement_trace_stage(self, stage: str, *, update=None,
                                       state=None, P=None, x=None) -> None:
        """Append an opt-in nominal-state stage without touching the filter."""
        builder = self._meas_builder
        if builder is None or not hasattr(builder, "emit_trace_stage"):
            return
        try:
            if state is None:
                state = self._est.state if self._est is not None else None
            if P is None:
                P = self._est.P if self._est is not None else None
            if x is None and self._est is not None:
                x = self._est.effective_x()
            builder.emit_trace_stage(
                stage, state=state,
                si=self._est.si if self._est is not None else None,
                x=x, P=P, update=update,
            )
        except Exception as exc:
            # Stage trace is observational.  A malformed optional snapshot
            # must never alter update acceptance or stream ordering.
            logger.debug("TC measurement stage trace unavailable: %s", exc)

    def _measurement_trace_builder(self):
        """Return the enabled RTK measurement trace builder, if any."""
        builder = self._meas_builder
        if (builder is None
                or getattr(builder, "_measurement_trace", None) is None
                or not hasattr(builder, "emit_trace_stage")):
            return None
        return builder

    def _emit_final_measurement_trace(self, builder, *, status, accepted,
                                       state=None, P=None, x=None,
                                       reason=None, pos_jump_m=None,
                                       limit_m=None):
        """Emit exactly one final record after the TC gate/rollback settles."""
        if builder is None:
            return
        pending = None
        take_pending = getattr(self._est, "take_tc_postfit_trace", None)
        if take_pending is not None:
            try:
                pending = take_pending(builder)
            except Exception:
                pending = None
        update = {
            "final": True,
            "status": str(status),
            "accepted": bool(accepted),
            "attempted": True,
        }
        if reason is not None:
            update["reason"] = str(reason)
        if pos_jump_m is not None:
            update["pos_jump_m"] = float(pos_jump_m)
        if limit_m is not None:
            update["limit_m"] = float(limit_m)
        if accepted and pending is not None:
            postfit = pending.get("postfit")
            postfit_array = None
            if postfit is not None:
                try:
                    candidate = np.asarray(postfit, dtype=float).reshape(-1)
                    if np.all(np.isfinite(candidate)):
                        postfit_array = candidate
                except Exception:
                    postfit_array = None
            if postfit_array is not None:
                update.update({
                    "postfit": postfit_array.copy(),
                    "postfit_definition": "v_minus_H_x_post",
                    "postfit_scope": (
                        "measurement_update_post_joseph_pre_feedback"
                    ),
                    "postfit_available": True,
                    "postfit_variance": None,
                    "postfit_variance_available": False,
                })
            excluded = pending.get("excluded_rows", ())
            if excluded:
                update["excluded_rows"] = [int(index) for index in excluded]
        try:
            if state is None:
                state = self._est.state if self._est is not None else None
            if P is None:
                P = self._est.P if self._est is not None else None
            if x is None and self._est is not None:
                x = self._est.effective_x()
            builder.emit_trace_stage(
                "post_measurement", state=state,
                si=self._est.si if self._est is not None else None,
                x=x, P=P, update=update,
            )
        except Exception as exc:
            # Final trace delivery is observational and must not alter the
            # already settled update/rollback result.
            logger.debug("TC final measurement trace unavailable: %s", exc)

    def _emit_output(self, qins: int) -> None:
        """Emit a state at an exact fusion boundary when a stream is attached."""
        if self._output_callback is not None:
            self._output_callback(self._est.state, self._est.P, qins)
        elif self._writer is not None:
            self._write_state(qins)

    @staticmethod
    def _build_rtk_retry_groups(info: dict, n_rows: int):
        """Map DD rows to satellite groups for same-epoch retry.

        GREAT removes the target satellite owning the worst normalized row and
        reruns the outer update.  ``RtkTcMeas`` emits one ``pairs`` entry per
        accepted row, so this mapping is lossless and remains diagnostic-only
        until ``TcEstimator`` explicitly needs a retry.
        """
        groups = {}
        row_indices = list(info.get("row_indices", ()))
        for pair_order, pair in enumerate(info.get("pairs", ())):
            row_index = (row_indices[pair_order]
                         if pair_order < len(row_indices) else pair_order)
            if row_index >= int(n_rows) or len(pair) < 2:
                continue
            # Insert target first: a failed DD row is normally attributed to
            # its non-reference satellite, matching GREAT's _remove_sat.
            for sat in (pair[1], pair[0]):
                try:
                    key = int(sat)
                except (TypeError, ValueError):
                    continue
                groups.setdefault(key, []).append(int(row_index))
        sat_ids = []
        row_groups = []
        for sat, rows in groups.items():
            if rows:
                sat_ids.append(int(sat))
                row_groups.append(rows)
        return row_groups, sat_ids

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
        # Record the untouched input form for diagnostics only.  In particular
        # a rate sample converted below must retain unavailable source SOW.
        if self._init_propagation_trace_sink is not None:
            try:
                self._trace_last_input_form = (
                    "increment" if imu.is_increment() else
                    "rate" if imu.is_rate() else None
                )
            except (AttributeError, TypeError, ValueError):
                self._trace_last_input_form = None
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

        # Run the diagnostic copy before the normal IMU route.  The original
        # payload is passed unchanged; the main estimator remains untouched by
        # this GNSS-free fork.
        self._record_init_propagation_clone(imu)

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
        if imu.timestamp <= cur.timestamp:
            # When GNSS lands exactly on the current raw-rate endpoint, the
            # interpolation branch above has already committed this physical
            # interval.  Do not send the same endpoint through InsUpdate a
            # second time (which would create a dt=0 propagation attempt).
            self._check_velocity_divergence()
            return
        self._est.time_update(imu)
        self._record_latest_propagation()
        self._static_detect.push(imu)
        self._apply_constraints(imu)

        self._check_velocity_divergence()

        self._emit_propagation()

    def _check_velocity_divergence(self) -> None:
        """Reset an obviously divergent TC velocity after propagation."""
        if not self._velocity_guard_enabled:
            return
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

    @staticmethod
    def _explicit_gnss_week(obsr) -> int | None:
        """Return the source GNSS week, or null when the source lacks it."""
        source_time = getattr(obsr, "t", None)
        if source_time is None:
            return None
        try:
            from src.core.gnss.rtklib.rtkcmn import time2gpst

            week, _sow = time2gpst(source_time)
            return int(week)
        except (AttributeError, TypeError, ValueError, OverflowError, ImportError):
            return None

    def _initialization_imu_bracket(self, target_timestamp: float):
        """Find buffered raw IMU rows bracketing the init source epoch.

        This helper is used only by the opt-in initialization trace.  It does
        not interpolate, convert, or consume samples; unavailable sides stay
        ``None`` so the trace cannot claim a synthetic input row.
        """
        valid = []
        for imu in self._init_imu:
            try:
                if np.isfinite(float(imu.timestamp)):
                    valid.append(imu)
            except (AttributeError, TypeError, ValueError):
                continue
        if not valid:
            return None, None
        previous = [imu for imu in valid if imu.timestamp <= target_timestamp]
        current = [imu for imu in valid if imu.timestamp >= target_timestamp]
        prev = max(previous, key=lambda imu: imu.timestamp) if previous else None
        curr = min(current, key=lambda imu: imu.timestamp) if current else None
        return prev, curr

    @staticmethod
    def _explicit_gnss_sow(obsr) -> float | None:
        """Return SOW from the GNSS source epoch, never Unix-time fallback.

        ``Obs`` carries the parser's RTKLIB ``gtime_t`` in ``t``.  Converting
        that source epoch with ``time2gpst`` preserves the real source SOW;
        unlike ``unix_to_gpst(timestamp)``, it does not manufacture a trace
        interval from the integration timestamp.  A source-provided SOW is
        preferred when an upstream adapter exposes one explicitly.
        """
        source_sow = getattr(obsr, "source_sow", None)
        if source_sow is not None:
            try:
                value = float(source_sow)
            except (TypeError, ValueError):
                value = None
            if value is not None and np.isfinite(value) and 0.0 <= value < 604800.0:
                return value

        source_time = getattr(obsr, "t", None)
        if source_time is None:
            return None
        try:
            from src.core.gnss.rtklib.rtkcmn import time2gpst

            _week, value = time2gpst(source_time)
            value = float(value)
        except (AttributeError, TypeError, ValueError, OverflowError, ImportError):
            return None
        return value if np.isfinite(value) and 0.0 <= value < 604800.0 else None

    @staticmethod
    def _explicit_solution_sow(solution) -> float | None:
        """Return a positioning solution's source GPST SOW, if available.

        SPP and relative solutions carry their own ``gtime_t`` in ``t``.
        This helper intentionally has no Unix timestamp fallback: a solution
        epoch that cannot be read remains unavailable in the diagnostic row.
        """
        source_sow = getattr(solution, "source_sow", None)
        if source_sow is not None:
            try:
                value = float(source_sow)
            except (TypeError, ValueError):
                value = None
            if value is not None and np.isfinite(value) and 0.0 <= value < 604800.0:
                return value

        source_time = getattr(solution, "t", None)
        if source_time is None:
            return None
        try:
            from src.core.gnss.rtklib.rtkcmn import time2gpst

            _week, value = time2gpst(source_time)
            value = float(value)
        except (AttributeError, TypeError, ValueError, OverflowError, ImportError):
            return None
        return value if np.isfinite(value) and 0.0 <= value < 604800.0 else None

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
        if (self._init_propagation_trace_active and
                self._init_propagation_trace_first_gnss_sow is None):
            self._init_propagation_trace_first_gnss_sow = (
                self._explicit_gnss_sow(obsr)
            )
            if self._init_propagation_trace_first_gnss_sow is not None:
                self._flush_pending_init_propagation_trace(
                    first_gnss_observed=True
                )

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
        # The following values are source-time provenance only.  They are
        # populated from the GNSS/solution objects below and are never used by
        # initialization itself.
        provenance_enabled = self._initialization_input_trace_sink is not None
        gnss_measurement_epoch_sow = (
            self._explicit_gnss_sow(obsr) if provenance_enabled else None
        )
        gnss_source_epoch_sow = gnss_measurement_epoch_sow
        spp_epoch_sow = None
        relpos_epoch_sow = None

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
            if provenance_enabled:
                spp_epoch_sow = self._explicit_solution_sow(sol)
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
                if provenance_enabled:
                    relpos_epoch_sow = self._explicit_solution_sow(rtk_sol)
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
        # Keep source SOW as diagnostic metadata only.  The timestamp remains
        # the existing initialization/math input; no source SOW is inferred
        # from Unix time for the trace contract.
        self._gnss_pos_cache.append(
            (t_gnss, rr.copy(), self._explicit_gnss_sow(obsr))
        )
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

        first_entry = self._gnss_pos_cache[0]
        last_entry = self._gnss_pos_cache[-1]
        t_first, pos_first = first_entry[:2]
        t_last, pos_last = last_entry[:2]
        # The INS state is stamped at the latest GNSS epoch.  Its velocity is
        # therefore the local derivative at that epoch (the latest adjacent
        # RTK positions), rather than the average over the whole alignment
        # baseline.  GREAT POS alignment follows the same endpoint derivative
        # contract; the full cache span remains the motion/observability gate.
        velocity_entry = self._gnss_pos_cache[-2]
        t_velocity, pos_velocity = velocity_entry[:2]
        velocity_diff_start_sow = (
            velocity_entry[2] if len(velocity_entry) > 2 else None
        )
        velocity_diff_end_sow = (
            last_entry[2] if len(last_entry) > 2 else None
        )
        cache_sample_start_sow = (
            first_entry[2] if len(first_entry) > 2 else None
        )
        cache_sample_end_sow = (
            last_entry[2] if len(last_entry) > 2 else None
        )
        span = t_last - t_first
        if span < 5.0:
            return  # 不足 5 秒, 继续累积

        velocity_span = t_last - t_velocity
        if velocity_span <= 0.0:
            return

        # 动态速度阈值: 基于运动检测所用位置源
        if quality == 1:  # FIX
            init_speed_thr = 2.0
        else:  # FLOAT / DGPS / SPP
            init_speed_thr = 3.0

        # Match GREAT POS alignment's observability contract.  A long
        # position span alone is not sufficient while the platform is
        # turning: the resulting chord heading can be stale by the time the
        # INS handoff occurs.  Require the last three endpoint displacement
        # headings to agree within GREAT's 10-degree acceptance span.  The
        # check is streaming and uses only the already buffered GNSS rows.
        # The production GNSS stream normally has >=4 buffered epochs once
        # the five-second span is available.  Keep the historical two-row
        # RTD/SPP unit-test path valid for synthetic callers that provide only
        # the endpoint pair; with fewer than three endpoint headings there is
        # no stability evidence to evaluate.
        if len(self._gnss_pos_cache) >= 4:
            heading_samples = []
            for previous, current in zip(self._gnss_pos_cache[-4:-1],
                                         self._gnss_pos_cache[-3:]):
                dt_heading = float(current[0] - previous[0])
                if dt_heading <= 0.0:
                    return
                delta_e = (current[1] - previous[1]) / dt_heading
                lat_h, lon_h, _ = ecef2llh(current[1])
                velocity_n_h = cal_Ce2n(lat_h, lon_h) @ delta_e
                horizontal_norm = math.hypot(
                    float(velocity_n_h[0]), float(velocity_n_h[1]))
                if horizontal_norm <= 1.0e-9:
                    return
                heading_samples.append(math.atan2(
                    float(velocity_n_h[1]), float(velocity_n_h[0])))
            if not _headings_are_stable(heading_samples):
                return

        # 相邻末端位置差分计算状态时刻的速度矢量。
        vel_e = (pos_last - pos_velocity) / velocity_span

        # Keep the exact buffered source rows used to bracket the initialization
        # epoch in the opt-in trace.  This is observational metadata only; the
        # initializer has never consumed these rows as a mechanization update.
        if self._initialization_input_trace_sink is not None:
            raw_imu_prev, raw_imu_curr = self._initialization_imu_bracket(t_gnss)
        else:
            raw_imu_prev, raw_imu_curr = None, None

        lat, lon, _ = ecef2llh(pos_last)
        vel_n = cal_Ce2n(lat, lon) @ vel_e
        planar_speed = math.hypot(vel_n[0], vel_n[1])
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
        init_P = self._initializer._set_initial_variance(
            InitMode.POSITION_DIFF, init_state.pos_e
        )

        # 创建估计器 + 量测构造器
        self._est = TcEstimator(init_state, init_P, self._cfg, self._mode)
        self._meas_builder = self._build_measurement_builder()
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

        if self._initialization_input_trace_sink is not None:
            self._emit_initialization_input_trace(
                init_state,
                source_sow=self._explicit_gnss_sow(obsr),
                raw_position=pos_for_state,
                rtk_rr_ecef=rtk_rr,
                # RTKLIB's ``Sol.rr`` is the rover antenna ECEF solution.
                # Keep both semantic labels explicit for cross-implementation
                # audits; no lever-arm conversion is inferred here.
                rtk_antenna_ecef=rtk_rr,
                derived_velocity=vel_e,
                derived_attitude=att_rpy,
                velocity_diff_start_position_ecef=pos_first,
                velocity_diff_end_position_ecef=pos_last,
                velocity_diff_start_sow=velocity_diff_start_sow,
                velocity_diff_end_sow=velocity_diff_end_sow,
                gnss_measurement_epoch_sow=gnss_measurement_epoch_sow,
                gnss_source_epoch_sow=gnss_source_epoch_sow,
                spp_epoch_sow=spp_epoch_sow,
                relpos_epoch_sow=relpos_epoch_sow,
                cache_sample_start_sow=cache_sample_start_sow,
                cache_sample_end_sow=cache_sample_end_sow,
                state_epoch_sow=gnss_measurement_epoch_sow,
                # No navigation output has been published at the handoff
                # row; the first real output is annotated by _emit_output's
                # propagation record instead of being fabricated here.
                published_event_epoch_sow=None,
                position_frame_id="ECEF",
                velocity_frame_id="ECEF",
                attitude_frame_id="NED",
                initialization_gnss_week=self._explicit_gnss_week(obsr),
                initialization_gnss_sow=self._explicit_gnss_sow(obsr),
                raw_imu_prev=raw_imu_prev,
                raw_imu_curr=raw_imu_curr,
                body_frame="FRD",
                body_order="front,right,down",
                nav_frame="NED",
                nav_order="north,east,down",
                state_time_unix_s=init_state.timestamp,
                state_time_source="initialization_state.timestamp",
                # Initialization assembles the state from GNSS/PVA and has
                # not applied or consumed an estimator IMU update at this row.
                imu_update_applied=False,
                imu_update_time_unix_s=None,
                imu_consumed=False,
                imu_consumed_time_unix_s=None,
            )

        # Initialization is GNSS-assisted in this pipeline.  The optional
        # propagation audit starts a deep-copied, GNSS-free mechanization fork
        # at this exact handoff; the main estimator remains on its normal path.
        if self._init_propagation_trace_sink is not None:
            self._start_init_propagation_clone(
                sow=self._explicit_gnss_sow(obsr))

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
            # Both payload forms need the sample at the handoff as the
            # previous boundary/seed.  A native increment at ``init_ts`` is
            # not integrated again: ``_add_increment_imu`` only stores the
            # first sample and consumes the next interval.  A rate sample at
            # the same timestamp likewise seeds rate-to-increment conversion.
            # Dropping either seed would make the first propagated interval
            # span two samples (or drop the first native interval entirely).
            if imu.timestamp > init_ts:
                events.append((imu.timestamp, "imu", imu))
                continue
            if imu.timestamp != init_ts:
                continue
            is_rate = getattr(imu, "is_rate", None)
            is_increment = getattr(imu, "is_increment", None)
            if ((callable(is_rate) and is_rate()) or
                    (callable(is_increment) and is_increment())):
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
        # A propagation immediately before this call is still part of the
        # requested pre-GNSS window; subsequent propagation is not.  Flipping
        # this flag here preserves the existing measurement ordering.
        self._init_propagation_trace_active = False
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
        trace_builder = self._measurement_trace_builder()
        defer_trace = trace_builder is not None
        if defer_trace:
            self._meas_builder._defer_trace_emit = True

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
        finally:
            if defer_trace:
                self._meas_builder._defer_trace_emit = False

        # udbias 已在 build 内执行 (rtk 模式), 更新上一历元时刻供下一历元
        self._prev_obs_t = copy(obsr.t)

        if len(v) == 0:
            # 量测构建返回空 (卫星被 outlier 拒绝/共视卫星不足):
            # 用 SPP 3D 位置做 fallback 位置更新, 防止 INS 自由漂移
            logger.debug(f"TC no_meas (mode={mode}, t={t_gnss:.3f}): "
                         f"obsr sats={len(obsr.sat)}, obsb sats={len(obsb.sat) if obsb is not None else 0}")
            if (self._spp_fallback_enabled
                    and self._spp_fallback_update(obsr, obsb, nav, t_gnss)):
                self._emit_final_measurement_trace(
                    trace_builder, status="rejected_gate", accepted=False,
                )
                return
            self._on_meas_failure(obsr, obsb, nav, t_gnss)
            self._emit_final_measurement_trace(
                trace_builder, status="rejected_gate", accepted=False,
            )
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
            if (self._spp_fallback_enabled
                    and self._spp_fallback_update(obsr, obsb, nav, t_gnss)):
                self._emit_final_measurement_trace(
                    trace_builder, status="rejected_gate", accepted=False,
                )
                return
            self._on_meas_failure(obsr, obsb, nav, t_gnss, recover=False)
            self._emit_final_measurement_trace(
                trace_builder, status="rejected_gate", accepted=False,
            )
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
        # The measurement builder is the owner of the current DD trace
        # identity.  Pass it only when its opt-in sink is enabled; the
        # estimator then freezes and emits post-fit immediately after Joseph
        # and before feedback.  The optional kwarg is omitted on the normal
        # path to preserve compatibility with estimator test doubles.
        retry_enabled = bool(self._cfg.get("tc", {}).get(
            "satellite_retry_enable", True))
        retry_groups, retry_satellites = (
            self._build_rtk_retry_groups(info, len(v))
            if mode == "rtk" and retry_enabled else ([], [])
        )
        retry_builder = getattr(self._meas_builder, "_last_retry_builder", None)
        retry_kw = ({
            "retry_groups": retry_groups,
            "retry_satellites": retry_satellites,
            "retry_builder": retry_builder,
        } if retry_groups and retry_builder is not None else {})
        if trace_builder is None:
            feedback_x = self._est.tc_meas_update(
                v, H, R, source=mode, **retry_kw)
        else:
            feedback_x = self._est.tc_meas_update(
                v, H, R, source=mode, trace=trace_builder, **retry_kw)
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
            self._emit_final_measurement_trace(
                trace_builder, status="rejected_gate", accepted=False,
                state=self._est.state, P=self._est.P, x=pre_x,
                reason="postfit_gate",
            )
            return
        update_info = dict(info)
        used_v = getattr(self._est, "last_tc_update_v", None)
        used_H = getattr(self._est, "last_tc_update_H", None)
        used_R = getattr(self._est, "last_tc_update_R", None)
        rebuilt_info = getattr(self._est, "last_tc_update_info", None)
        if used_v is None or used_H is None or used_R is None:
            used_v, used_H, used_R = v, H, R
        if rebuilt_info:
            # Preserve the original trace metadata while replacing numerical
            # counts/pairs with the final rebuilt DD model used by the EKF.
            for key, value in rebuilt_info.items():
                if key != "retry_builder":
                    update_info[key] = value
        postfit = getattr(self._est, "last_tc_postfit", None)
        if postfit is None:
            postfit = np.asarray(used_v) - used_H @ feedback_x
        else:
            postfit = np.asarray(postfit, dtype=float)
        active_rows = tuple(getattr(
            self._est, "last_tc_active_row_indices", range(len(used_v))))
        active_index = np.asarray(active_rows, dtype=int)
        if active_index.size and postfit.size == len(used_v):
            postfit_for_stats = postfit[active_index]
            r_for_stats = used_R[np.ix_(active_index, active_index)]
        else:
            postfit_for_stats = postfit
            r_for_stats = used_R
        update_info["postfit_norm"] = float(np.linalg.norm(postfit_for_stats))
        update_info["postfit_chi2"] = float(np.sum(
            postfit_for_stats * postfit_for_stats /
            np.maximum(np.diag(r_for_stats), np.finfo(float).tiny)))
        update_info["excluded_rows"] = [
            int(index) for index in getattr(
                self._est, "last_tc_outlier_indices", ())]
        # A true GREAT-style retry changes H/R and therefore the innovation
        # covariance and gain.  Recompute these diagnostics from the final
        # rebuilt model instead of leaving the pre-retry matrices in the CSV.
        if (used_H.shape != H.shape or used_R.shape != R.shape
                or not np.array_equal(used_H, H)):
            s_matrix = used_H @ pre_p @ used_H.T + used_R
            s_diag_median = float(np.median(np.diag(s_matrix)))
            K_gain = pre_p @ used_H.T @ np.linalg.inv(s_matrix)
            k_pos_norm = float(np.linalg.norm(
                K_gain[si.pos:si.pos + 3], axis=1).max())
            k_vel_norm = float(np.linalg.norm(
                K_gain[si.vel:si.vel + 3], axis=1).max())
            k_att_norm = float(np.linalg.norm(
                K_gain[si.att:si.att + 3], axis=1).max())
            k_ba_norm = float(np.linalg.norm(
                K_gain[si.accel_bias:si.accel_bias + 3], axis=1).max())
            k_bg_norm = float(np.linalg.norm(
                K_gain[si.gyro_bias:si.gyro_bias + 3], axis=1).max())
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
            n_meas=int(update_info.get("n", len(used_v))),
            innovation=used_v,
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
            "innovation_norm": float(np.linalg.norm(used_v)),
            "n_meas": int(update_info.get("n", len(used_v))),
            "n_phase_acc": int(update_info.get("n_phase_acc", 0)),
            "n_code_acc": int(update_info.get("n_code_acc", 0)),
            "k_pos_norm": float(k_pos_norm),
            "k_vel_norm": float(k_vel_norm),
            "k_att_norm": float(k_att_norm),
            "k_bg_norm": float(k_bg_norm),
            "k_ba_norm": float(k_ba_norm),
            "ref_sats": update_info.get("ref_sats", []),
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
                self._emit_final_measurement_trace(
                    trace_builder, status="rejected_rollback", accepted=False,
                    state=self._est.state, P=self._est.P, x=feedback_x,
                    reason="pos_jump", pos_jump_m=pos_jump, limit_m=50.0,
                )
                return

        self._last_meas_pos = self._est.state.pos_e.copy()
        self._maybe_align_yaw(t_gnss)
        self._emit_final_measurement_trace(
            trace_builder, status="accepted", accepted=True,
            state=self._est.state, P=self._est.P,
        )

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
            "gyro_bias_state_x", "gyro_bias_state_y", "gyro_bias_state_z",
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
            "gyro_bias_state_x": float(self._est.state.gyro_bias[0]) if self._est is not None else float("nan"),
            "gyro_bias_state_y": float(self._est.state.gyro_bias[1]) if self._est is not None else float("nan"),
            "gyro_bias_state_z": float(self._est.state.gyro_bias[2]) if self._est is not None else float("nan"),
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
        if not self._auto_recovery_enabled:
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
        self._meas_builder = self._build_measurement_builder()

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

        # DD 定义统一为 y = N_ref − N_j (与下方 Q_dd 的协方差定义一致)。
        # 参考星出现在两位时方向不同, 但 DD 值本身不随方向变号——原先按
        # sign 变号会与 Q_dd 不一致并产生错误整数 (实测 +2.2 m 高程常偏)。
        dds = []  # (j_sat, j_frq, orientation): +1=参考星在首位, -1=在第二位
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
        for k, (ks, kf, _orient) in enumerate(dds):
            ki = _nav_ib(ks, kf)
            y_dd[k] = nav.x[r_i] - nav.x[ki]
            # Q[k,m] = Var(N_r − N_k 与 N_r − N_m 的协方差)
            #        = Q[r,r] − Q[r,m'] − Q[k',r] + Q[k',m']
            for m, (ms, mf, mg) in enumerate(dds):
                mi = _nav_ib(ms, mf)
                Q_dd[k, m] = (nav.P[r_i, r_i] - nav.P[r_i, mi]
                              - nav.P[ki, r_i] + nav.P[ki, mi])

        # 卫星连续参与历元数 (GREAT _lock_epo_num): 本历元未出现的卫星清零
        self._ambiguity.update_lock(seen_sat)

        posvar = float(np.mean(np.diag(
            self._est.P[si.pos:si.pos + 3, si.pos:si.pos + 3])))
        sat_pairs = [(ref_sat, js) for (js, _jf, _sg) in dds]
        fixed_dd, ratio, ok = self._ambiguity.try_fix(
            y_dd, Q_dd, posvar, sat_pairs=sat_pairs)
        if not ok:
            self._amb_fixed = False
            logger.debug(
                f"TC amb rejected: nb={nb}, ratio={ratio:.2f}, maxdev="
                f"{self._ambiguity.last_maxdev}, maxsig="
                f"{self._ambiguity.last_maxsig}, posvar={posvar:.3f}")
            return

        # restamb 写回: 基准星保持浮点; 其余 = 基准浮点值 − DD 整数 (TC+nav 双写)
        # 部分固定时未通过的分量为 NaN, 保持浮点不写回
        n_ref = float(nav.x[r_i])
        fixed_slots = []
        for k, (js, jf, _orient) in enumerate(dds):
            if not np.isfinite(fixed_dd[k]):
                continue
            new_n = n_ref - fixed_dd[k]
            tc_idx = si.amb_idx(js, jf)
            compact = tc_idx - si.amb_start
            self._est._N_stored[compact] = new_n
            self._est.x[tc_idx] = 0.0
            nav.x[_nav_ib(js, jf)] = new_n
            fixed_slots.append(tc_idx)
        if not fixed_slots:
            self._amb_fixed = False
            return

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
        logger.debug(
            f"TC amb diag: ratio={ratio:.2f}, nb={nb}, partial="
            f"{self._ambiguity.last_partial}, maxdev="
            f"{self._ambiguity.last_maxdev}, maxsig="
            f"{self._ambiguity.last_maxsig}")
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
