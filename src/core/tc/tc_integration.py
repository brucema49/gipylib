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
        # false fix 检测: 跟踪上次量测更新后的位置, 检测异常跳变
        self._last_meas_pos: Optional[np.ndarray] = None
        self._amb_fixed: bool = False
        # 量测更新计数 (用于收敛期保护: 前 N 个历元禁用 NIS/false_fix/跳变检验)
        self._meas_count: int = 0
        self._convergence_warmup: int = 10  # 收敛预热历元数 (仅跳变检验)
        # 量测连续失败计数 (用于发散恢复: 连续失败超阈值时尝试 SPP 重初始化)
        self._consecutive_failures: int = 0
        self._recovery_threshold: int = 10  # 连续失败 10 个历元后尝试恢复

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

        if self.imucur is None:
            self.imucur = imu
            self._static_detect.push(imu)
            self.last_qins = 2
            self._write_state(self.last_qins)
            return

        self.last_qins = 2  # 默认: 仅机械编排 + 协差variance propagation

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

        # 速度发散检测: 速度 > 50 m/s (180 km/h) 说明滤波器已发散
        # (地面车辆最大速度 ~30 m/s, 50 m/s 是安全阈值)
        # 重置速度为 0, 放大速度协方差, 等待下次 GNSS 量测修正位置
        vel = self._est.state.vel_e
        speed = float(np.linalg.norm(vel))
        if speed > 50.0:
            logger.warning(
                f"TC vel_diverge (t={imu.timestamp:.3f}): "
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

        self._write_state(self.last_qins)

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
        imu_block = [imu for imu in self._init_imu
                     if t_gnss - 2.0 <= imu.timestamp <= t_gnss + 1.0]
        if len(imu_block) < 2:
            return
        has_before = any(imu.timestamp <= t_gnss for imu in imu_block)
        has_after = any(imu.timestamp >= t_gnss for imu in imu_block)
        if not (has_before and has_after):
            return

        # 保存 nav 状态: SPP+RTK 会修改 nav 内部状态, 初始化失败时需恢复
        # 防止污染后续 _write_gnss_only 的 SPP 初始猜测
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
                    # RTK-SPP 位置一致性检验: 差异 > 50m 说明 RTK false fix, 拒绝初始化
                    # (SPP 精度 10m 级, 正常 RTK FIX 与 SPP 差异 < 30m;
                    #  false fix 可偏差 280m+, 用此检查过滤不可靠的 RTK 解)
                    pos_diff = float(np.linalg.norm(rtk_rr - x_spp[:3]))
                    if pos_diff > 50.0:
                        logger.warning(
                            f"TC init reject: RTK-SPP pos diff {pos_diff:.2f}m > 50m "
                            f"(t={t_gnss:.1f}, q={rtk_sol.stat}), skip init")
                        self._restore_nav(nav, saved_x, saved_P,
                                          saved_fix, saved_lock)
                        return
                    rr = rtk_rr
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
                logger.debug(f"TC init RTK 异常: {e}")

        # 恢复 nav 状态 (SPP+RTK 已获取所需结果, 无需保留 nav 副作用)
        self._restore_nav(nav, saved_x, saved_P, saved_fix, saved_lock)

        # 动态速度阈值: 基于运动检测所用位置源
        # RTK FIX (quality==1): 用 2.0 m/s (精确稳定, 用户要求 RTK/RTD > 2m/s)
        # RTK FLOAT / SPP: 用 3.0 m/s (SPP 位置噪声 3-5m, 但 5.0 导致初始化延迟到 t+730s;
        #   降至 3.0 提前初始化, SPP 噪声通过 gnss_buffer 多历元平滑过滤)
        if quality == 1:  # FIX
            init_speed_thr = 2.0
        else:  # FLOAT / DGPS / SPP: 用 SPP 位置做运动检测
            init_speed_thr = 3.0
        original_thr = self._initializer.dynamic_speed_threshold
        self._initializer.dynamic_speed_threshold = init_speed_thr

        # 位置差分运动检测: RTK FLOAT 启动阶段噪声大 (滤波器未收敛, 位置跳变 2-3m),
        # 会误触发动对准阈值. 使用 SPP 位置做运动检测 (更稳定, 静止时平面速度 <0.5m/s).
        # 但初始化位置始终用 RTK 解 (用户要求: 不使用 SPP 初始化紧组合参数)
        if quality == 1:  # FIX
            pos_for_init = rr.copy()
        else:  # FLOAT / DGPS: 用 SPP 位置做运动检测, 但初始化位置用 RTK
            pos_for_init = x_spp[:3].copy()

        gnss_sol = GnssSolution(
            timestamp=t_gnss, week=0, position=pos_for_init,
            quality=quality, num_sv=ns, sd=pos_sd,
            velocity=None, vel_sd=None,
        )
        block = AlignedBlock(gnss=gnss_sol, imu_list=imu_block)

        # 尝试初始化 (static 模式: yaw=0, 与 truth 一致; position_diff: 速度方向→yaw)
        init_state = None
        init_P = None
        try:
            init_state, init_P = self._initializer.initialize(
                block, self._init_mode)
        except ValueError as e:
            logger.debug(f"TC init fail t={t_gnss:.1f}: {e}")
        except Exception as e:
            logger.debug(f"TC init excp t={t_gnss:.1f}: {e}")

        # 恢复原始阈值
        self._initializer.dynamic_speed_threshold = original_thr

        if init_state is None:
            buf_len = len(self._initializer.gnss_buffer)
            logger.debug(f"TC init None t={t_gnss:.1f} buf={buf_len}/{self._initializer.gnss_buffer_size} "
                         f"q={quality}")
            return

        # 初始化位置始终用 RTK 解 (用户要求: 不使用 SPP 初始化紧组合参数)
        # RTK FLOAT 虽然有噪声, 但精度远优于 SPP (SPP 高程误差可达 20m+)
        if self._mode in ("rtk", "rtd") and obsb is not None and quality != 5:
            init_state.pos_e = rr.copy()

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
        self._last_q = quality
        self._last_ns = ns
        self._last_gnss_t = init_state.timestamp

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
        # 重置钟差为白噪声模型 (必须在 effective_x/build 之前)
        # 这样 innovation 用 clk=0 计算, KF 每历元独立估计钟差,
        # 避免 SPP 初始化的 (pos,clk) 自洽性导致位置误差被钟差吸收
        self._est.reset_clk_variance()
        # 构造 effective_x: amb/clk 部分用 effective (stored + error)
        # 此时 clk_stored=0, x[clk]=0, 故 effective clk=0
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
            self._on_meas_failure(obsr, obsb, nav, t_gnss)
            return

        # 收敛期保护: 前 _convergence_warmup 个历元禁用跳变检验
        # 初始化后模糊度未收敛, 位置会自然调整, 跳变检验会误拒
        in_warmup = self._meas_count < self._convergence_warmup

        # 注: 移除 NIS 检验和 false_fix 检测 (TC vs SPP 一致性)
        # 这些机制过度拒绝有效量测导致滤波器发散 (96/173286 历元通过)
        # ignav 仅用 chi-square 检验残差 (valsol), 无 NIS/SPP 一致性检验
        # false fix 通过正确协方差矩阵和模糊度管理预防, 而非事后拒绝

        # RTK 模糊度管理
        if mode == "rtk" and si.has_ambiguity():
            self._handle_ambiguity(info, obsr, nav)

        # 记录量测更新前位置 (用于跳变检测)
        pre_update_pos = self._est.state.pos_e.copy()

        # 量测更新 + 反馈 (钟差已在 _trigger_meas 开头重置)
        self._est.tc_meas_update(v, H, R, source=mode)
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

    def _on_meas_failure(self, obsr, obsb, nav, t_gnss: float) -> None:
        """量测失败处理: 累计失败次数, 超阈值时尝试 SPP 恢复。

        当连续失败超 _recovery_threshold 个历元且 GNSS 观测可用时,
        尝试用 SPP 重新初始化位置, 使系统能从发散中恢复。
        """
        self._consecutive_failures += 1
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
            for k in range(3):
                est.P[si.clk_bias + k, si.clk_bias + k] = 100.0 ** 2
            est._clk_stored[:] = 0.0
            if len(x_spp) >= 6:
                est._clk_stored[0] = float(x_spp[3])
                est._clk_stored[1] = float(x_spp[4])
                est._clk_stored[2] = float(x_spp[5])
                for k in range(3):
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
            reset_idx.extend(range(si.clk_bias, si.clk_bias + 3))
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
        """RTK 模糊度固定 (LAMBDA) + ignav 风格 holdamb。

        1. try_fix: LAMBDA 整数搜索, 成功则 N_stored=fixed, ε_N=0
        2. holdamb: 对实际使用的模糊度添加约束量测 (v=0, R=VAR_HOLDAMB=0.001),
           通过 joseph_update 降低 P[amb] 同时保留交叉协方差。
           参考 ignav rtkpos.cc holdamb(): filter(x,P,H,v,R) 而非直接置 P。

        与旧实现的区别:
        - 旧: P[:,amb]=0, P[amb,:]=0, P[amb,amb]=0.001 (清零交叉项, 过度自信)
        - 新: 仅约束 info["pairs"] 中的模糊度, 保留交叉协方差, 新卫星 P 不受影响
        """
        si = self._est.si
        if not si.has_ambiguity():
            return
        amb_slice = slice(si.amb_start, si.amb_start + si.n_amb)
        # effective N = N_stored + ε_N (current best direct estimate)
        N_effective = self._est._N_stored + self._est.x[amb_slice]
        P_amb = self._est.P[amb_slice, amb_slice]
        # 位置方差 (rtklib thresar1 逻辑): P[pos,pos] 对角均值
        posvar = float(np.mean(np.diag(
            self._est.P[si.pos:si.pos + 3, si.pos:si.pos + 3])))
        fixed, ratio, ok = self._ambiguity.try_fix(N_effective, P_amb, posvar)
        if not ok:
            self._amb_fixed = False
            return

        # 应用整数解: N_stored = fixed, ε_N = 0
        self._est._N_stored = fixed.copy()
        self._est.x[amb_slice] = 0.0

        # ignav 风格 holdamb: 对实际使用的模糊度添加约束量测
        # v[k] = fixed[i] - N_effective[i] = 0 (已设 N_stored=fixed, ε_N=0)
        # H[k, i] = 1, R[k,k] = VAR_HOLDAMB = 0.001
        # joseph_update(v=0, H, R) 不改变状态 (K·v=0), 但通过 Joseph 形式降低 P[amb]
        # 关键: 保留交叉协方差, 未使用的模糊度槽位 P 不受影响 (新卫星可正常初始化)
        VAR_HOLDAMB = 0.001  # cycle², 与 ignav rtkpos.cc 一致
        pairs = info.get("pairs", [])
        # 提取 phase 量测 (code=0) 涉及的 (sat, freq) 唯一对
        used_amb_idx = set()
        for sat1, sat2, frq, code in pairs:
            if code != 0:
                continue  # 仅 phase 涉及模糊度
            ii = si.amb_idx(sat1, frq)
            jj = si.amb_idx(sat2, frq)
            if ii >= 0:
                used_amb_idx.add(ii)
            if jj >= 0:
                used_amb_idx.add(jj)
        if used_amb_idx:
            used_idx = sorted(used_amb_idx)
            n_const = len(used_idx)
            v_const = np.zeros(n_const)  # v=0 (N_effective=fixed)
            H_const = np.zeros((n_const, si.dim))
            R_const = np.eye(n_const) * VAR_HOLDAMB
            for k, idx in enumerate(used_idx):
                H_const[k, idx] = 1.0
            # joseph_update 降低 P[used_amb] 同时保留交叉协方差
            self._est.joseph_update(v_const, H_const, R_const)
            # 反馈 (清零 ε_N, 累积到 stored)
            self._est.feedback()

        logger.info(
            f"TC amb fixed: ratio={ratio:.2f}, n={len(fixed)}, "
            f"n_const={len(used_amb_idx)}, holdamb R={VAR_HOLDAMB}")
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
