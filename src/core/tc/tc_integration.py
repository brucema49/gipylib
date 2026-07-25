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
import logging
import math
from typing import Optional, List

import numpy as np

from src.core.data_types import AlignedBlock, GnssSolution, ImuMeasurement
from src.core.ins.constraints import Constraints
from src.core.ins.initializer import InsInitializer, InitMode
from src.core.ins.interpolator import imu_interpolate_linear
from src.core.ins.lc_integration import _DecimationCounter
from src.core.ins.static_detect import StaticDetect
from src.core.tc.tc_ambiguity import TcAmbiguity
from src.core.tc.tc_degrade import TcDegradeManager
from src.core.tc.tc_estimator import TcEstimator
from src.core.tc.tc_measurement import SppTcMeas, RtkTcMeas, RtdTcMeas

logger = logging.getLogger(__name__)

_MEAS_BUILDERS = {"spp": SppTcMeas, "rtk": RtkTcMeas, "rtd": RtdTcMeas}


class TcIntegration:
    """紧组合导航集成 (GVINS 风格 IMU 消费 + TC 量测触发)。

    初始化前用小缓冲累积 IMU+GNSS 原始观测，初始化后增量处理。
    NHC/ZUPT/ZARU 作为可选约束, per-IMU 触发 (decimation 控制):
      - 静态 (StaticDetect): ZUPT + ZARU (互斥于 NHC)
      - 运动: NHC (需非剧烈转弯)
    """

    def __init__(self, config: dict, mode: str = "spp"):
        self._cfg = config
        self._mode = mode
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

        # IMU 状态 (GVINS 风格)
        self.imupre: Optional[ImuMeasurement] = None
        self.imucur: Optional[ImuMeasurement] = None
        # GNSS 原始观测队列: (obsr, obsb, nav, t_gnss)
        self.pending_obs: collections.deque = collections.deque()

        # 初始化前缓冲
        self._init_imu: List[ImuMeasurement] = []
        self._init_obs: list = []   # [(obsr, obsb, nav, t_gnss), ...]
        self._initialized = False
        self._last_gnss_t: float = 0.0
        self._last_q: int = 5       # 最近 GNSS quality (初值 5=SPP)
        self._last_ns: int = 0      # 最近 num_sv
        self._writer = None
        self._output_count = 0
        self.last_qins: int = 2

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

    @staticmethod
    def _select_init_mode(config: dict) -> InitMode:
        ins_cfg = config.get("ins", {})
        method = ins_cfg.get("alignnment_dynamic_method", "auto")
        if method == "velocity_vector":
            return InitMode.VELOCITY_VECTOR
        if method == "position_diff":
            return InitMode.POSITION_DIFF
        pos_mode = config.get("gnss", {}).get("positioning_mode", "spp")
        if pos_mode == "spp":
            return InitMode.VELOCITY_VECTOR
        return InitMode.POSITION_DIFF

    # ===== 增量喂入 =====

    def add_imu(self, imu: ImuMeasurement) -> None:
        """GVINS 风格 IMU 消费: 每条 IMU 检查 GNSS 队头时间戳。"""
        if not self._initialized:
            self._init_imu.append(imu)
            # 当有 GNSS obs 且当前 IMU 时间戳 >= GNSS 时间戳时, 尝试初始化
            if self._init_obs and imu.timestamp >= self._init_obs[-1][3]:
                self._try_init()
            return

        if self.imucur is None:
            self.imucur = imu
            self._static_detect.push(imu)
            self.last_qins = 2
            self._write_state(self.last_qins)
            return

        self.last_qins = 2  # 默认: 仅机械编排 + 协方差传播

        cur = self.imucur

        # 1. 处理所有落入 [cur.t, imu.t] 区间的 GNSS obs (GVINS 风格插值触发)
        while self.pending_obs:
            obsr, obsb, nav, t_gnss = self.pending_obs[0]

            if t_gnss < cur.timestamp:
                # GNSS 已过期 (比 cur 还早): 防御性直接量测更新
                logger.debug(
                    f"过期 GNSS obs t={t_gnss:.6f} < cur.t={cur.timestamp:.6f}, "
                    f"直接量测更新")
                self._trigger_meas(cur, obsr, obsb, nav, t_gnss)
                self.pending_obs.popleft()
                continue

            if t_gnss > imu.timestamp:
                # GNSS 在当前 IMU 之后: 留给后续 IMU 处理
                break

            # cur.t <= t_gnss <= imu.t: GVINS 风格插值触发
            if t_gnss == cur.timestamp:
                interp = cur
            else:
                interp = imu_interpolate_linear(cur, imu, t_gnss)
                if interp is None:
                    break
                self.imupre = cur
                self.imucur = interp
                self._est.time_update(interp)
                self._static_detect.push(interp)
                self._apply_constraints(interp)
                self._write_state(self.last_qins)

            # 触发 TC 量测更新 + 反馈
            self._trigger_meas(interp, obsr, obsb, nav, t_gnss)
            self.pending_obs.popleft()
            cur = interp

        # 2. 推进当前 IMU (dt = imu.t - cur.t)
        self.imupre = cur
        self.imucur = imu
        self._est.time_update(imu)
        self._static_detect.push(imu)
        self._apply_constraints(imu)
        self._write_state(self.last_qins)

    def add_gnss(self, obsr, obsb, nav) -> None:
        """添加 GNSS 原始观测: append 到 pending_obs deque。"""
        t_gnss = float(obsr.t.time + obsr.t.sec)
        if not self._initialized:
            self._init_obs.append((obsr, obsb, nav, t_gnss))
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

        使用第一个 GNSS obs 做初始化 (而非最后一个), 后续 GNSS obs 供量测更新。
        """
        if not self._init_obs:
            return

        obsr, obsb, nav, t_gnss = self._init_obs[0]
        imu_block = [imu for imu in self._init_imu
                     if t_gnss - 2.0 <= imu.timestamp <= t_gnss + 1.0]
        if len(imu_block) < 2:
            return

        has_before = any(imu.timestamp <= t_gnss for imu in imu_block)
        has_after = any(imu.timestamp >= t_gnss for imu in imu_block)
        if not (has_before and has_after):
            return

        # 角速度检查
        cfg = self._cfg["ins"]
        angular_thr = cfg.get("angular_velocity_threshold_deg", 30.0)
        if angular_thr > math.pi:
            angular_thr = math.radians(angular_thr)
        gyro_norms = [float(np.linalg.norm(imu.gyro)) for imu in imu_block]
        if float(np.mean(gyro_norms)) >= angular_thr:
            return

        # SPP 粗定位 (用 obsr + nav)
        try:
            from src.core.gnss.rtklib.ephemeris import satposs
            from src.core.gnss.rtklib.pntpos import estpos
            rs, var, dts, svh = satposs(obsr, nav)
            # 用基站位作初始猜测 (比 [0,0,0] 收敛快且准)
            if np.any(nav.rb):
                nav.x[0:3] = nav.rb
            sol, x_spp = estpos(obsr, nav, rs[:, :3], dts, svh)
            if not sol.stat:
                logger.warning("TC init: SPP 失败, 等待下一历元")
                return
        except Exception as e:
            logger.warning(f"TC init SPP 异常: {e}")
            return

        # 构造 GnssSolution 供 InsInitializer 使用
        rr = x_spp[:3].copy()
        gnss_sol = GnssSolution(
            timestamp=t_gnss,
            week=0,
            position=rr,
            quality=5,       # SPP
            num_sv=int(sol.ns),
            sd=np.array([sol.rr[3], sol.rr[4], sol.rr[5]]
                        if hasattr(sol, 'rr') and len(sol.rr) >= 6
                        else [10.0, 10.0, 10.0]),
            velocity=None,
            vel_sd=None,
        )

        dynamic_thr = cfg.get("dynamic_speed_threshold", 4.0)
        static_thr = cfg.get("static_speed_threshold", 0.5)
        block = AlignedBlock(gnss=gnss_sol, imu_list=imu_block)

        # 1) 尝试配置的动态模式 (TC 无 GNSS 速度, 用位置差分)
        init_state = None
        init_P = None
        if self._init_mode == InitMode.POSITION_DIFF:
            try:
                init_state, init_P = self._initializer.initialize(
                    block, InitMode.POSITION_DIFF)
            except ValueError:
                pass

        # 2) 回退: 静态模式
        if init_state is None:
            try:
                init_state, init_P = self._initializer.initialize(
                    block, InitMode.STATIC)
            except ValueError:
                return

        if init_state is None:
            return

        # 创建估计器 + 量测构造器
        self._est = TcEstimator(init_state, init_P, self._cfg, self._mode)
        builder_cls = _MEAS_BUILDERS.get(self._mode, SppTcMeas)
        self._meas_builder = builder_cls(self._cfg)
        # 用 SPP 钟差初始化 direct clk estimate (加速收敛, 否则需 150s+ 收敛)
        # x_spp[3]=GPS c*dt(m), x_spp[4]=GLO-GPS bias(m), x_spp[5]=GAL-GPS bias(m)
        # 修复后: _clk_stored 是 direct estimate, self.x[clk_bias] 是 ε_clk (init 0)
        si = self._est.si
        if si.clk_bias >= 0 and len(x_spp) >= 6:
            self._est._clk_stored[0] = float(x_spp[3])   # GPS
            self._est._clk_stored[1] = float(x_spp[4])   # GLO
            self._est._clk_stored[2] = float(x_spp[5])   # GAL
            # SPP 钟差不确定度 ~10m, 远小于默认 100m
            for k in range(3):
                self._est.P[si.clk_bias + k, si.clk_bias + k] = 10.0 ** 2
        self._initialized = True
        self._last_q = 5
        self._last_ns = int(sol.ns)
        self._last_gnss_t = t_gnss

        logger.info(
            f"TcIntegration 初始化成功: t={init_state.timestamp:.3f}, "
            f"mode={self._mode}, pos={init_state.pos_e}")

        # 回放缓冲中 init 时间戳之后的事件
        self._replay_buffer(init_state.timestamp)

        # 清空初始化缓冲
        self._init_imu.clear()
        self._init_obs.clear()

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
        # 构造 effective_x: amb/clk 部分用 effective (stored + error)
        # 这样 build 用 current direct estimate (N_stored+ε_N) 构造 v,
        # 而 joseph_update 用 self.x (error state) 计算 H @ self.x (estimated error).
        x = self._est.effective_x()
        mode = self._degrade.current_mode

        # 按当前模式构造量测
        try:
            if mode == "spp":
                v, H, R, info = self._meas_builder.build(
                    self._est.state, obsr, nav, si, x=x)
            else:   # rtk / rtd
                v, H, R, info = self._meas_builder.build(
                    self._est.state, obsr, nav, si, x=x, obsb=obsb)
        except Exception as e:
            logger.warning(f"TC meas build 异常 (mode={mode}): {e}")
            self._degrade.on_fail(self._est, "build_error")
            return

        if len(v) == 0:
            logger.debug(f"TC no_meas (mode={mode}, t={t_gnss:.3f}): "
                         f"obsr sats={len(obsr.sat)}, obsb sats={len(obsb.sat) if obsb is not None else 0}")
            self._degrade.on_fail(self._est, "no_meas")
            return

        # 更新 num_sv / quality
        n_meas = info.get("n", len(v))
        self._last_ns = n_meas
        self._last_q = 5 if mode == "spp" else (1 if mode == "rtk" else 4)

        # RTK 模糊度管理
        if mode == "rtk" and si.has_ambiguity():
            self._handle_ambiguity(info, obsr, nav)

        # 量测更新 + 反馈
        self._est.tc_meas_update(v, H, R, source=mode)
        self._degrade.on_success(self._est)
        self.last_qins = 3  # TC 量测更新完成
        # DEBUG: 监控状态演化 (前 5 历元)
        if self._output_count < 5:
            logger.warning(
                f"DEBUG t={t_gnss:.3f} mode={mode}: "
                f"pos={self._est.state.pos_e}, "
                f"x[pos]={self._est.x[si.pos:si.pos+3]}, "
                f"x[clk]={self._est.x[si.clk_bias:si.clk_bias+3] if si.clk_bias>=0 else 'N/A'}, "
                f"clk_stored={self._est._clk_stored}, "
                f"N_stored_norm={float(np.linalg.norm(self._est._N_stored)):.3f}")

    def _handle_ambiguity(self, info: dict, obsr, nav) -> None:
        """RTK 模糊度固定 (LAMBDA)。

        从 estimator 状态提取 effective N (N_stored + ε_N), 调 TcAmbiguity.try_fix,
        成功则将整数解赋给 N_stored 并清零 ε_N (像 INS error state feedback)。
        """
        si = self._est.si
        if not si.has_ambiguity():
            return
        amb_slice = slice(si.amb_start, si.amb_start + si.n_amb)
        # effective N = N_stored + ε_N (current best direct estimate)
        N_effective = self._est._N_stored + self._est.x[amb_slice]
        P_amb = self._est.P[amb_slice, amb_slice]
        fixed, ratio, ok = self._ambiguity.try_fix(N_effective, P_amb)
        if ok:
            # 应用整数解: N_stored = fixed, ε_N = 0
            self._est._N_stored = fixed.copy()
            self._est.x[amb_slice] = 0.0
            logger.debug(f"TC amb fixed: ratio={ratio:.2f}, n={len(fixed)}")

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
        elif self.nhc_enable and self._nhc_counter.should_trigger():
            if self._constraints.nhc(self._est, imu):
                applied = True

        if applied:
            self._est.feedback()
            self.last_qins = 3  # 约束量测更新完成

    # ===== 输出 =====

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
