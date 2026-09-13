"""紧组合量测构造 (SPP/RTK/RTD)。

架构:
  TcMeasurement ABC: 统一接口 build(state, obs, nav, si) -> (v, H, R, info)
  SppTcMeas:  伪距 (可选多普勒) 单点, 参考 GINav rescode_sppins
  RtkTcMeas:  双差载波+伪距, 参考 GINav ddres_rtkins / GREAT-MSF gsppflt
  RtdTcMeas:  仅双差伪距 (无模糊度)

符号约定 (与 GINav/GREAT-MSF 一致, standard innovation):
  v = y - h(x_est)   (实测减预测 = innovation)
  H = ∂h/∂x           (位置列 = +LOS = ∂pred/∂δr, INS 误差状态)
                      (clk/amb 列 = +1/+λ = ∂pred/∂direct_state, 直接状态)
  joseph_update: x += K·innov
    - INS 误差状态: feedback state -= x (减误差)
    - GNSS 直接状态: effective_x = stored + x, feedback stored += x (加修正)
"""
import math
from copy import copy, deepcopy
from collections.abc import Mapping
import numpy as np

from src.core.data_types import InsState
from src.core.ins.transfer_matrix import skew
from src.core.tc.tc_state_index import TcStateIndex
from src.core.gnss.rtklib.rtkcmn import (geodist, satazel, ecef2pos,
                                          satexclude, ionmodel, tropmodel,
                                          tropmapf, sat2freq, uGNSS,
                                          sat2prn, sat2id, timediff)
from src.core.gnss.rtklib.ephemeris import satposs
from src.core.gnss.rtklib.pntpos import varerr as spp_varerr, prange, gettgd, REL_HUMI
from src.core.gnss.rtklib.rtkpos import (
    zdres, selsat, ddcov, IB, udbias, varerr as rtk_varerr,
)
from src.core.gnss.rtklib import rCST
from src.core.time_utils import unix_to_gpst
from src.log.tc_measurement_trace import trace_from


# Same alpha=0.001 critical values used by ignav's rtkcmn::chisqr.
_CHI_SQR_001 = (
    10.8, 13.8, 16.3, 18.5, 20.5, 22.5, 24.3, 26.1, 27.9, 29.6,
    31.3, 32.9, 34.5, 36.1, 37.7, 39.3, 40.8, 42.3, 43.8, 45.3,
    46.8, 48.3, 49.7, 51.2, 52.6, 54.1, 55.5, 56.9, 58.3, 59.7,
    61.1, 62.5, 63.9, 65.2, 66.6, 68.0, 69.3, 70.7, 72.1, 73.4,
    74.7, 76.0, 77.3, 78.6, 80.0, 81.3, 82.6, 84.0, 85.4, 86.7,
    88.0, 89.3, 90.6, 91.9, 93.3, 94.7, 96.0, 97.4, 98.7, 100.0,
    101.0, 102.0, 103.0, 104.0, 105.0, 107.0, 108.0, 109.0, 110.0, 112.0,
    113.0, 114.0, 115.0, 116.0, 118.0, 119.0, 120.0, 122.0, 123.0, 125.0,
    126.0, 127.0, 128.0, 129.0, 131.0, 132.0, 133.0, 134.0, 135.0, 137.0,
    138.0, 139.0, 140.0, 142.0, 143.0, 144.0, 145.0, 147.0, 148.0, 149.0,
)


def validate_tc_postfit(residual: np.ndarray, covariance: np.ndarray,
                        n_parameters: int, sigma_limit: float = 4.0) -> bool:
    """Validate TC post-fit residuals using ignav's ``valpos`` rules.

    The TC path bypasses rtklib-py's ``relpos`` wrapper, so it must perform
    this check after the EKF correction and before feeding the correction back
    into the nominal INS state.  As in ignav, the diagonal of ``R`` is used
    for the per-residual and chi-square checks.
    """
    residual = np.asarray(residual, dtype=np.float64).reshape(-1)
    covariance = np.asarray(covariance, dtype=np.float64)
    if residual.size == 0:
        return False
    variances = np.diag(covariance)
    if variances.size != residual.size or np.any(variances <= 0.0):
        return False
    if np.any(residual * residual > (sigma_limit ** 2) * variances):
        return False
    dof = residual.size - int(n_parameters)
    if dof <= 0:
        return True
    critical = _CHI_SQR_001[min(dof, len(_CHI_SQR_001)) - 1]
    statistic = float(np.sum(residual * residual / variances))
    return statistic <= critical


def sync_tc_ambiguities_with_rtklib(nav, obsb, obsr, iu, ir, estimator,
                                    previous_obs_t=None):
    """Run rtklib's ambiguity temporal update (``udbias``) on the TC block.

    rtklib stores ambiguities as ``IB(sat, freq)`` (frequency-major) while
    ``TcStateIndex`` stores them satellite-major. Both cover ``MAXSAT * nf``
    slots, so the exchange is a permutation over *all* slots (not just the
    current common-view sats) — this lets ``udbias``'s outage counter and
    phase-code coherency offset operate on a complete state vector.

    Returns the rover observation time (a ``gtime_t`` copy) to pass back as
    ``previous_obs_t`` on the next call.
    """
    si = estimator.si
    if not si.has_ambiguity():
        return copy(obsr.t)

    amb_pairs = [
        (si.amb_idx(s, f), IB(s, f, nav.na))
        for f in range(nav.nf)
        for s in range(1, uGNSS.MAXSAT + 1)
    ]
    # 1. TC -> rtklib: effective ambiguity (stored + error) and covariance.
    #    仅拷贝已初始化的健康槽位: 排除 init≈1e4 的僵尸槽位。否则其 10000
    #    会经本步写入 nav.P, 再被步骤3当作 "nav 值" 回读到 TC 对角,
    #    清零逻辑被自身回写值 defeat, 僵尸永生 (issue/8-22 第12节)。
    effective = estimator.effective_x()
    ptc_diag = np.diag(estimator.P)
    healthy = {tc_i for tc_i, _ in amb_pairs if 1e-9 < ptc_diag[tc_i] < 5.0e3}
    for tc_idx, rtk_idx in amb_pairs:
        if tc_idx in healthy:
            nav.x[rtk_idx] = effective[tc_idx]
    for tc_i, rtk_i in amb_pairs:
        if tc_i not in healthy:
            continue
        for tc_j, rtk_j in amb_pairs:
            if tc_j in healthy:
                nav.P[rtk_i, rtk_j] = estimator.P[tc_i, tc_j]

    # 2. Temporal update: cycle-slip detection + re-init + random walk.
    if previous_obs_t is not None:
        nav.tt = timediff(obsr.t, previous_obs_t)
    else:
        nav.tt = 0.0
    udbias(nav, obsb, obsr, iu, ir)

    # 3. rtklib -> TC.  ``udbias`` may have reset the state to 0 (cycle slip /
    #    outage), in which case the TC slot is cleared and re-seeded from
    #    ``nav.P``; otherwise only the stored value and diagonal are updated.
    sig_n0_sq = float(getattr(estimator, "_tc_config", {}).get(
        "gnss", {}).get("sig_n0", 30.0)) ** 2
    for tc_idx, rtk_idx in amb_pairs:
        compact = tc_idx - si.amb_start
        if nav.x[rtk_idx] == 0.0:
            estimator._N_stored[compact] = 0.0
            estimator.x[tc_idx] = 0.0
            estimator.P[tc_idx, :] = 0.0
            estimator.P[:, tc_idx] = 0.0
            # 重播种为初始方差 (待用状态), 而非回读被步骤1污染的 nav.P
            estimator.P[tc_idx, tc_idx] = sig_n0_sq
        else:
            estimator._N_stored[compact] = nav.x[rtk_idx] - estimator.x[tc_idx]
            estimator.P[tc_idx, tc_idx] = nav.P[rtk_idx, rtk_idx]

    return copy(obsr.t)


def save_tc_phase_state(nav, obsb, obsr, iu, ir):
    """Record phase / LLI state for the next epoch's cycle-slip detection.

    Mirrors the tail of ``relpos()`` (rtkpos.py lines 1064-1079): ``nav.ph``
    / ``nav.pt`` hold the current epoch's phase and time per receiver, and
    ``nav.prev_lli`` holds the current LLI flags, so ``detslp_dop`` /
    ``detslp_ll`` can compare consecutive epochs. ``nav.slip`` is reset here
    after ``udbias``/``_build_dd`` have consumed it.
    """
    sats = obsr.sat[iu]
    for i, sat in enumerate(sats):
        for f in range(nav.nf):
            if obsb.L[ir[i], f] != 0:
                nav.pt[0, sat - 1, f] = obsb.t
                nav.ph[0, sat - 1, f] = obsb.L[ir[i], f]
            if obsr.L[iu[i], f] != 0:
                nav.pt[1, sat - 1, f] = obsr.t
                nav.ph[1, sat - 1, f] = obsr.L[iu[i], f]
    nav.slip[:, :] = 0
    for f in range(nav.nf):
        ix0 = np.where((obsb.L[:, f] != 0) | (obsb.lli[:, f] != 0))[0]
        ix1 = np.where((obsr.L[:, f] != 0) | (obsr.lli[:, f] != 0))[0]
        nav.prev_lli[obsb.sat[ix0] - 1, f, 0] = obsb.lli[ix0, f]
        nav.prev_lli[obsr.sat[ix1] - 1, f, 1] = obsr.lli[ix1, f]


class TcMeasurement:
    """紧组合量测构造基类。"""

    def build(self, state, obsr, nav, si, x=None, obsb=None):
        """构造量测残差 v, 雅可比 H, 协方差 R。

        Args:
            state: InsState (用 pos_e/vel_e/C_b_e)
            obsr: rover 观测
            nav: 导航数据
            si: TcStateIndex
            x: EKF 状态向量 (含 clk_bias/ambiguity), 可选
            obsb: base 观测 (RTK/RTD 用), 可选

        Returns:
            (v[m], H[m, dim], R[m, m], info: dict)
        """
        raise NotImplementedError


class SppTcMeas(TcMeasurement):
    """SPP-INS 伪距量测 (参考 GINav rescode_sppins, innovation 形式)。

    v[i] = P[i] - h(est) = P - (rho + dtr - c*dts + dion + dtrp)
    H[pos:pos+3, i] = +LOS[i]     (= ∂pred/∂δr = -∂pred/∂pos, 与 GINav 一致)
    H[clk_bias, i] = 1.0          (= ∂pred/∂clk_base, GPS 公共钟差, 所有卫星)
    H[clk_bias + sys_off, i] = 1.0 (= ∂pred/∂inter_sys_bias, 仅非 GPS 卫星)

    钟差模型 (与 GINav rescode_sppins.m:47-62 / rtklib-py pntpos.py:113-126 一致):
      GPS:  dtr = x[clk_bias+0]
      BDS:  dtr = x[clk_bias+0] + x[clk_bias+3]
      GAL:  dtr = x[clk_bias+0] + x[clk_bias+2]
    """

    def __init__(self, config: dict):
        ins = config.get("ins", {}) if config else {}
        gnss = config.get("gnss", {}) if config else {}
        self.use_doppler = ins.get("tc_use_doppler", False)
        self.elmin = math.radians(gnss.get("elmin", 15.0))
        # SPP 伪距量测噪声 sigma (m):
        # spp_varerr 返回 ~0.004m (仅用于最小二乘加权), 不适合 EKF 量测噪声.
        # 实际 SPP 伪距噪声含电离层/对流层残差 + 多径, sigma ≈ 3-5m.
        self._spp_sigma = float(gnss.get("tc_spp_sigma", 3.0))
        # Doppler 量测噪声 sigma [m/s] (含钟漂残差 + 热噪声 + 多径)
        # TC 模式无独立钟漂状态, 钟漂 (<0.3 m/s) 吸收到 R 中
        self._doppler_sigma = float(gnss.get("tc_doppler_sigma", 0.3))
        # 粗差拒绝阈值 (防止异常值通过交叉协方差 corrupt yaw)
        self._max_code = float(gnss.get("maxcode", 30.0))
        self._max_doppler = 10.0  # m/s

    def build(self, state, obsr, nav, si, x=None, obsb=None):
        """构造 SPP 伪距量测 (innovation: P - h(est))。

        h(est) = r + dtr_est - c*dts + dion + dtrp
        其中 dtr_est = effective_x[clk_bias + sys_off] (stored + ε_clk)。

        注: 仅使用 GPS 卫星, 与 GPS-only SPP 初始化 (tc_integration._try_init 中
        _filter_gps_svh) 一致。rtklib-py 不能正确处理本数据集 BDS/GAL 观测
        (BDS 周数 +1356 修复后残差仍 ~km 量级), inter-system bias 状态虽建模
        但初始化未估计, 残差会拉偏位置导致发散。
        """
        rr = state.pos_e
        vr = state.vel_e
        pos = ecef2pos(rr)
        rs, var, dts, svh = satposs(obsr, nav)
        n = len(obsr.sat)
        v_list, H_rows, R_diag = [], [], []
        used_sats = []
        # 按 sat 编号升序遍历, 与 rtklib rescode 一致
        for i in np.argsort(obsr.sat):
            i = int(i)
            sat = obsr.sat[i]
            if svh[i] != 0:
                continue
            r, e = geodist(rs[i, :3], rr)
            if r <= 0.0:
                continue
            az, el = satazel(pos, e)
            if el < self.elmin:
                continue
            if satexclude(sat, var[i], svh[i], nav):
                continue
            P = prange(nav, obsr, i)
            if P == 0.0:
                continue
            # 电离层/对流层 (与 pntpos.rescode 一致, 确保 TC 跟踪 SPP 参考解)
            dion = ionmodel(obsr.t, pos, az, el, nav.ion)
            freq = sat2freq(sat, 0, nav)
            dion *= (nav.freq[0] / freq) ** 2
            # 对流层: 用天顶延迟 + NMF 映射 (单次仰角依赖, 比pntpos更准确)
            # pntpos.py 有 bug: tropmodel已含1/sin(el), 又乘NMF映射 (双重仰角依赖)
            trop_hs, trop_wet, _ = tropmodel(obsr.t, pos, np.deg2rad(90.0), REL_HUMI)
            mapfh, mapfw = tropmapf(obsr.t, pos, el)
            dtrp = mapfh * trop_hs + mapfw * trop_wet
            # 钟差估计值 (从 effective_x 取 = stored + ε_clk)
            # 模型 (与 GINav rescode_sppins.m:47-62 / rtklib-py pntpos.py:113-126 一致):
            #   GPS:  dtr = x[clk_bias+0]                    (common receiver clock)
            #   BDS:  dtr = x[clk_bias+0] + x[clk_bias+3]     (clock + inter-system bias)
            #   GAL:  dtr = x[clk_bias+0] + x[clk_bias+2]     (clock + inter-system bias)
            sys_off = self._sys_clk_offset(sat)
            dtr_est = 0.0
            if x is not None and si.clk_bias >= 0:
                dtr_est = float(x[si.clk_bias])          # GPS clock base (always)
                if sys_off != 0:
                    dtr_est += float(x[si.clk_bias + sys_off])  # inter-system bias
            # 残差 (innovation: P - h(est), 与 GINav/GREAT-MSF 一致)
            rho = r + dtrp + dion
            v_i = P - (rho + dtr_est - rCST.CLIGHT * dts[i])
            # 自适应 R: 残差超阈值时膨胀 R (而非硬拒绝), 防止高度误差增大时
            # 高仰角卫星被全部拒掉导致垂直几何恶化 (恶性循环)
            sys_gnss = _sys_gnss(sat)
            efact = nav.efact[sys_gnss]   # GPS=1.0, GLO=1.5, GAL=1.0
            sig = self._spp_sigma * efact
            if abs(v_i) > self._max_code:
                # 残差超阈: 膨胀 R (Huber 风格), 保留量测但降低权重
                # 避免硬拒绝导致卫星数减少 + 几何恶化
                sig = sig * (abs(v_i) / self._max_code)
            R_i = sig ** 2
            # H 行: H[pos] = +LOS = ∂pred/∂δr = -∂pred/∂pos (INS 误差状态)
            H_row = np.zeros(si.dim)
            H_row[si.pos:si.pos + 3] = e
            H_row[si.clk_bias] = 1.0                      # GPS clock base (always)
            if sys_off != 0:
                H_row[si.clk_bias + sys_off] = 1.0        # inter-system bias (non-GPS)
            # 杆臂姿态耦合: ∂ρ/∂att = -e · skew(C_b_e · lever_b)
            # 即使 lever_b=[0,0,0], 在线估计后激活, 使 yaw 通过杆臂可观测
            lever_b = state.leverarm
            lever_e = state.C_b_e @ lever_b
            H_row[si.att:si.att + 3] = -e @ skew(lever_e)
            # 杆臂 Jacobian: ∂ρ/∂lever = e · C_b_e
            if si.has_lever_arm():
                H_row[si.lever_arm:si.lever_arm + 3] = e @ state.C_b_e
            v_list.append(v_i)
            H_rows.append(H_row)
            R_diag.append(R_i)
            used_sats.append(sat)

            # Doppler 量测 (可选): 使速度可观, 通过 F[vel,att] 耦合使 yaw 可观
            if self.use_doppler and obsr.D[i, 0] != 0.0:
                freq_d = sat2freq(sat, 0, nav)
                if freq_d > 0:
                    lam = rCST.CLIGHT / freq_d
                    # 几何距离变化率: rate = dot(vs - vr, e) + Sagnac
                    vs = rs[i, 3:6] - vr  # 卫星相对接收机速度
                    rate = float(np.dot(vs, e))
                    # Sagnac 修正 (与 ignav doppHVR 一致)
                    rate += rCST.OMGE / rCST.CLIGHT * (
                        vs[1] * rr[0] + rs[i, 0] * vr[0]
                        - vs[0] * rr[1] - rs[i, 1] * vr[1])
                    # innovation: v = -lam*D - rate (钟漂吸收到 R)
                    v_dop = -lam * obsr.D[i, 0] - rate
                    # 粗差拒绝: Doppler 残差超阈跳过
                    if abs(v_dop) > self._max_doppler:
                        continue
                    # H 行: H[vel] = +e (∂pred/∂ε_vel, INS 误差状态, 与 H[pos]=+e 一致)
                    H_dop = np.zeros(si.dim)
                    H_dop[si.vel:si.vel + 3] = e
                    # R: 仰角相关 (ignav STDOPP/sin(el)), 含钟漂残差
                    sin_el = max(math.sin(el), 0.1)
                    sig_dop = self._doppler_sigma / math.sqrt(sin_el)
                    v_list.append(v_dop)
                    H_rows.append(H_dop)
                    R_diag.append(sig_dop ** 2)
                    used_sats.append(sat)
        if not v_list:
            return np.array([]), np.zeros((0, si.dim)), np.zeros((0, 0)), {}
        v = np.array(v_list)
        H = np.array(H_rows)              # H[m, dim]
        R = np.diag(R_diag)
        return v, H, R, {"sats": used_sats, "n": len(v)}

    @staticmethod
    def _sys_clk_offset(sat):
        """GPS=0, GLO=1, GAL=2, BDS=3 (对应 clk_bias 四维块)。

        BDS 独立钟差状态: BDT 与 GPST 存在系统偏差, 与 SPP (pntpos NX=7)
        的星间钟偏处理一致。
        """
        sys, _ = sat2prn(sat)
        if sys == uGNSS.GPS:
            return 0
        if sys == uGNSS.GAL:
            return 2
        if sys == uGNSS.BDS:
            return 3
        return 1


def _sys_gnss(sat):
    """卫星编号 → uGNSS 系统常量 (供 varerr)。"""
    sys, _ = sat2prn(sat)
    return sys


_TRACE_SYSTEM_NAMES = {
    uGNSS.GPS: "G",
    uGNSS.GLO: "R",
    uGNSS.GAL: "E",
    uGNSS.BDS: "C",
    uGNSS.QZS: "J",
    uGNSS.SBS: "S",
}


def _trace_system_name(system):
    """Return the compact constellation name used by observation traces."""
    return _TRACE_SYSTEM_NAMES.get(system, str(system))


def _trace_observation_time(obs):
    """Read an observation timestamp without requiring a concrete Obs type."""
    timestamp = getattr(obs, "timestamp", None)
    if timestamp is not None:
        return float(timestamp)
    t = getattr(obs, "t", None)
    if t is None:
        return 0.0
    return float(getattr(t, "time", 0.0)) + float(getattr(t, "sec", 0.0))


def _trace_mapping_metadata(*observations):
    """Return optional mapping identity exposed by an observation payload."""
    values = {"mapping_hash": None, "mapping_owner": None}
    for observation in observations:
        if observation is None:
            continue
        sources = [observation, getattr(observation, "mapping_trace", None)]
        metadata = getattr(observation, "metadata", None)
        if isinstance(metadata, Mapping):
            sources.append(metadata)
        for source in sources:
            if source is None:
                continue
            for field in values:
                if values[field] is not None:
                    continue
                if isinstance(source, Mapping):
                    value = source.get(field)
                else:
                    value = getattr(source, field, None)
                if value is not None:
                    values[field] = str(value)
        if values["mapping_hash"] is not None and values["mapping_owner"] is not None:
            break
    return values


class _DdBase(TcMeasurement):
    """RTK/RTD 双差量测共用骨架。

    复用 rtklib-py 的 zdres() 计算零差残差, selsat() 选择共视卫星,
    本类负责:
      - 在 INS 位置上重算 rover zdres
      - 把 rtklib 的 H[states, meas] 转置为 H[meas, states] 并重映射到 si 索引
      - 维护 ambiguity 槽位 (RTK 才有)
    """

    def __init__(self, config: dict, *, trace_sink=None, trace_callback=None):
        config = config or {}
        gnss = config.get("gnss", {}) if config else {}
        self.elmin = math.radians(gnss.get("elmin", 15.0))
        self.use_phase = True     # RTK=True, RTD=False
        self.use_code = True
        # 用于决定 ref sat 的 sig_n0 (与 rtklib ddres 一致)
        self._sig_n0 = float(config.get("gnss", {}).get("sig_n0", 30.0))
        self._measurement_trace = trace_from(
            sink=trace_sink, callback=trace_callback)
        # This identity is local to one builder/trace stream.  It prevents a
        # block ordinal from being mistaken for a globally reusable ID after
        # a recovery rebuilds the measurement builder.
        type(self)._trace_instance_sequence = getattr(
            type(self), "_trace_instance_sequence", 0) + 1
        self._trace_instance_id = (
            f"{type(self).__name__}-{type(self)._trace_instance_sequence}")
        self._trace_epoch = 0
        self._active_trace_context = None
        # The last complete pre-measurement record is retained only while an
        # explicit trace sink is enabled.  TcIntegration uses it to append a
        # post-measurement snapshot with the same event identity; it is never
        # consulted by the measurement builder.
        self._last_trace_record = None

    @staticmethod
    def _trace_state_snapshot(state, si=None, x=None, stage=None, P=None):
        """Return a JSON-safe nominal-state snapshot for stage auditing.

        Values are copied from the already available state/direct-state
        objects.  Missing optional values are explicitly ``None`` rather than
        inferred from another convention.  This helper is reached only when
        tracing is enabled.
        """
        def vector(value, size):
            try:
                result = np.asarray(value, dtype=float).reshape(-1)
                if result.size != size or not np.all(np.isfinite(result)):
                    return None
                return [float(item) for item in result]
            except Exception:
                return None

        def matrix(value):
            try:
                result = np.asarray(value, dtype=float)
                if result.shape != (3, 3) or not np.all(np.isfinite(result)):
                    return None
                return [[float(item) for item in row] for row in result]
            except Exception:
                return None

        timestamp = None
        sow = None
        if state is not None:
            try:
                timestamp = float(state.timestamp)
                if not np.isfinite(timestamp):
                    timestamp = None
                if timestamp is not None:
                    _week, sow = unix_to_gpst(timestamp)
                    sow = float(sow)
            except Exception:
                timestamp = None

        pos = vector(getattr(state, "pos_e", None), 3)
        vel = vector(getattr(state, "vel_e", None), 3)
        C_b_e = matrix(getattr(state, "C_b_e", None))
        q_b_e = vector(getattr(state, "q_b_e", None), 4)
        att_rpy = vector(getattr(state, "att_rpy", None), 3)
        lever_body = vector(getattr(state, "leverarm", None), 3)
        antenna = None
        if pos is not None and C_b_e is not None and lever_body is not None:
            try:
                antenna = [
                    float(pos[i] + sum(C_b_e[i][j] * lever_body[j]
                                      for j in range(3)))
                    for i in range(3)
                ]
            except Exception:
                antenna = None

        clock = {
            "available": False,
            "units": "m",
            "definition": "effective direct clock state = stored + error",
            "values": None,
        }
        if si is not None and x is not None:
            try:
                clk0 = int(getattr(si, "clk_bias", -1))
                values = np.asarray(x, dtype=float).reshape(-1)
                if clk0 >= 0 and values.size >= clk0 + 4:
                    clock["available"] = bool(np.all(np.isfinite(values[clk0:clk0 + 4])))
                    if clock["available"]:
                        clock["values"] = [float(item) for item in values[clk0:clk0 + 4]]
            except Exception:
                pass

        ambiguity = {
            "available": False,
            "units": "cycles",
            "definition": "effective ambiguity state = stored + error; satellite-major slots",
            "count": 0,
            "finite_count": 0,
            "nonzero_count": 0,
            "min": None,
            "max": None,
            "mean": None,
        }
        if si is not None and x is not None:
            try:
                if bool(si.has_ambiguity()):
                    start = int(si.amb_start)
                    count = int(si.n_amb)
                    values = np.asarray(x, dtype=float).reshape(-1)[start:start + count]
                    finite = values[np.isfinite(values)]
                    ambiguity["available"] = True
                    ambiguity["count"] = count
                    ambiguity["finite_count"] = int(finite.size)
                    ambiguity["nonzero_count"] = int(np.count_nonzero(finite))
                    if finite.size:
                        ambiguity["min"] = float(np.min(finite))
                        ambiguity["max"] = float(np.max(finite))
                        ambiguity["mean"] = float(np.mean(finite))
            except Exception:
                pass

        snapshot = {
            "state_stage": None if stage is None else str(stage),
            "state_timestamp": timestamp,
            "state_sow": sow,
            "ins_pos_ecef": pos,
            "ins_pos_ecef_x": None if pos is None else pos[0],
            "ins_pos_ecef_y": None if pos is None else pos[1],
            "ins_pos_ecef_z": None if pos is None else pos[2],
            "ins_vel_ecef": vel,
            "ins_vel_ecef_x": None if vel is None else vel[0],
            "ins_vel_ecef_y": None if vel is None else vel[1],
            "ins_vel_ecef_z": None if vel is None else vel[2],
            "antenna_pos_ecef": antenna,
            "antenna_pos_ecef_x": None if antenna is None else antenna[0],
            "antenna_pos_ecef_y": None if antenna is None else antenna[1],
            "antenna_pos_ecef_z": None if antenna is None else antenna[2],
            "antenna_definition": "pos_ecef + C_body_to_ecef @ lever_body",
            "attitude_repr": "C_body_to_ecef",
            "attitude_C_body_to_ecef": C_b_e,
            "attitude_q0": None if q_b_e is None else q_b_e[0],
            "attitude_q1": None if q_b_e is None else q_b_e[1],
            "attitude_q2": None if q_b_e is None else q_b_e[2],
            "attitude_q3": None if q_b_e is None else q_b_e[3],
            "attitude_rpy_rad": att_rpy,
            "attitude_rpy_units": "rad; C_body_to_ecef is authoritative",
            "lever_body": lever_body,
            "lever_frame": "body",
            "lever_units": "m",
            "lever_ecef": None if antenna is None or pos is None else [
                float(antenna[i] - pos[i]) for i in range(3)
            ],
            "clock_summary": clock,
            "ambiguity_summary": ambiguity,
        }
        if P is not None:
            try:
                covariance = np.asarray(P, dtype=float)
                diag = np.diag(covariance)
                finite = diag[np.isfinite(diag)]
                snapshot["covariance_summary"] = {
                    "available": bool(finite.size == diag.size),
                    "dimension": int(covariance.shape[0])
                    if covariance.ndim == 2 and covariance.shape[0] == covariance.shape[1]
                    else None,
                    "diag_min": float(np.min(finite)) if finite.size else None,
                    "diag_max": float(np.max(finite)) if finite.size else None,
                    "trace": float(np.trace(covariance))
                    if covariance.ndim == 2 and covariance.shape[0] == covariance.shape[1]
                    else None,
                }
            except Exception:
                snapshot["covariance_summary"] = {
                    "available": False, "dimension": None,
                    "diag_min": None, "diag_max": None, "trace": None,
                }
        return snapshot

    def _trace_begin(self, obsr, obsb, nav, state=None, si=None, x=None):
        """Create a trace context only when an explicit sink is enabled."""
        if self._measurement_trace is None:
            return None
        metadata = _trace_mapping_metadata(obsr, obsb, nav)
        systems = [_trace_system_name(system) for system in nav.gnss_t]
        context = {
            "epoch": self._trace_epoch,
            "time": _trace_observation_time(obsr),
            "_obsr": obsr,
            "_obsb": obsb,
            "trace_instance_id": self._trace_instance_id,
            "trace_event_id": f"{self._trace_instance_id}:{self._trace_epoch}",
            "stage_schema": "NOMINAL_STATE_STAGE_AUDIT_V1",
            "stage": "pre_measurement",
            "state_stage": "pre_measurement",
            "_pre_state": state,
            "_pre_si": si,
            "_pre_x": x,
            "mapping_hash": metadata["mapping_hash"],
            "mapping_owner": metadata["mapping_owner"],
            "constellation": systems,
            "candidate_sats": {system: [] for system in systems},
            "candidate_prns": {system: [] for system in systems},
            "candidate_sats_semantics": "common_view_dd_input",
            "common_view_sats": {system: [] for system in systems},
            "common_view_prns": {system: [] for system in systems},
            "common_view_candidates": {system: {} for system in systems},
            "exclusion_reasons": [],
            "_selected_ref_sats": {system: [] for system in systems},
            "dd_keys": [],
            "dd_rows": [],
            "_dd_attempt_counter": 0,
            "_block_counter": 0,
            "_covariance_blocks": [],
            "n_code": 0,
            "n_phase": 0,
            "_code_attempted": 0,
            "_phase_attempted": 0,
            "_code_rejected": 0,
            "_phase_rejected": 0,
            "float_state_counter": 0,
            "float_update_count": 0,
            "ar_call_counter": 0,
            "ambiguity_resolution_calls": 0,
            "solution_mode": "float",
            "float_state_counter_semantics": "deprecated alias of float_update_count",
            "ar_call_counter_semantics": "deprecated alias of ambiguity_resolution_calls",
        }
        self._trace_epoch += 1
        return context

    def _trace_candidates(self, context, sat, nav):
        """Snapshot the common-view candidates before slot filtering."""
        if context is None:
            return
        for system in nav.gnss_t:
            name = _trace_system_name(system)
            idx = _DdBase._sys_idx(sat, system)
            context["candidate_sats"][name] = [int(sat[i]) for i in idx]
            context["candidate_prns"][name] = [sat2id(int(sat[i])) for i in idx]
            context["common_view_sats"][name] = [int(sat[i]) for i in idx]
            context["common_view_prns"][name] = [sat2id(int(sat[i])) for i in idx]
            context["common_view_candidates"][name] = {
                str(slot): [
                    {
                        "sat": int(sat[i]),
                        "prn": sat2id(int(sat[i])),
                        "raw_signal": {
                            kind: self._trace_raw_signal_fields(
                                context.get("_obsr"), context.get("_obsb"),
                                int(sat[i]), slot, kind)
                            for kind in ("phase", "code")
                        },
                    }
                    for i in idx
                ]
                for slot in range(nav.nf)
            }

    @staticmethod
    def _trace_exclusion(context, sat, system, index, slot, kind, reason):
        if context is None:
            return
        context["exclusion_reasons"].append({
            "sat": int(sat[index]),
            "constellation": _trace_system_name(system),
            "slot": int(slot),
            "kind": kind,
            "reason": reason,
        })

    @staticmethod
    def _trace_h_nonzero(row):
        """Return a JSON-safe sparse H row without rounding its values."""
        return {
            str(index): float(value)
            for index, value in enumerate(np.asarray(row, dtype=float))
            if value != 0.0
        }

    @staticmethod
    def _trace_set_geometry(context, obsr, obsb, rs, rsb,
                            rover_ecef, base_ecef):
        """Attach the pure geometric ranges used by the audit trace.

        ``zdres`` also applies satellite clock, troposphere and antenna
        corrections.  The fields below intentionally identify only the pure
        ECEF range term, so a consumer cannot mistake it for the complete
        computed observation model.
        """
        if context is None:
            return
        try:
            rover_positions = {
                int(sat): np.asarray(rs[index, :3], dtype=float).copy()
                for index, sat in enumerate(obsr.sat)
            }
            base_positions = {
                int(sat): np.asarray(rsb[index, :3], dtype=float).copy()
                for index, sat in enumerate(obsb.sat)
            }
            context["_geometry"] = {
                "rover_ecef": np.asarray(rover_ecef, dtype=float).copy(),
                "base_ecef": np.asarray(base_ecef, dtype=float).copy(),
                "rover_sat_ecef": rover_positions,
                "base_sat_ecef": base_positions,
            }
        except Exception:
            # A diagnostic-only decomposition may be unavailable for a custom
            # test payload; never make the solver depend on it.
            context["_geometry"] = None

    @staticmethod
    def _trace_geometry_terms(context, ref_sat, target_sat,
                              los_ref=None, los_target=None):
        geometry = context.get("_geometry") if context is not None else None
        result = {
            "geometry_dd": None,
            "sat_ref_ecef": None,
            "sat_target_ecef": None,
            "los_ref": None,
            "los_target": None,
            "los_dd": None,
        }
        for key, value in (("los_ref", los_ref), ("los_target", los_target)):
            try:
                item = np.asarray(value, dtype=float).reshape(-1)
                if item.size == 3 and np.all(np.isfinite(item)):
                    result[key] = [float(entry) for entry in item]
            except Exception:
                pass
        if result["los_ref"] is not None and result["los_target"] is not None:
            result["los_dd"] = [
                float(a - b)
                for a, b in zip(result["los_ref"], result["los_target"])
            ]
        if not isinstance(geometry, Mapping):
            return result
        try:
            rover = np.asarray(geometry["rover_ecef"], dtype=float)
            base = np.asarray(geometry["base_ecef"], dtype=float)
            rr = np.asarray(geometry["rover_sat_ecef"][int(ref_sat)], dtype=float)
            rt = np.asarray(geometry["rover_sat_ecef"][int(target_sat)], dtype=float)
            br = np.asarray(geometry["base_sat_ecef"][int(ref_sat)], dtype=float)
            bt = np.asarray(geometry["base_sat_ecef"][int(target_sat)], dtype=float)
            if any(item.shape != (3,) or not np.all(np.isfinite(item))
                   for item in (rover, base, rr, rt, br, bt)):
                return result
            result["sat_ref_ecef"] = [float(item) for item in rr]
            result["sat_target_ecef"] = [float(item) for item in rt]
            result["geometry_dd"] = float(
                np.linalg.norm(rr - rover) - np.linalg.norm(br - base)
                - np.linalg.norm(rt - rover) + np.linalg.norm(bt - base)
            )
        except Exception:
            pass
        return result

    @staticmethod
    def _trace_raw_signal_fields(obsr, obsb, sat, slot, kind=None):
        """Read only explicit upstream raw-signal metadata, never infer it.

        The current Obs object stores decoded slot arrays but not the source
        RINEX signal identity per satellite.  A future reader may attach a
        ``raw_signal_by_slot`` mapping; accepting it here keeps the diagnostic
        schema forward-compatible while preventing legacy ``freq_ix`` values
        from being mislabeled as raw bands.
        """
        def side(observation):
            empty = {"raw_band": None, "track": None}
            try:
                table = getattr(observation, "raw_signal_by_slot", None)
                if not isinstance(table, Mapping):
                    return empty
                keys = []
                if kind is not None:
                    keys.extend(((int(sat), int(slot), str(kind)),
                                 f"{int(sat)}:{int(slot)}:{kind}"))
                keys.extend(((int(sat), int(slot)), f"{int(sat)}:{int(slot)}"))
                entry = None
                for key in keys:
                    entry = table.get(key)
                    if entry is not None:
                        break
                if not isinstance(entry, Mapping):
                    return empty
                raw_band = entry.get("raw_band")
                track = entry.get("track")
                # An optional mapping is untrusted observational input.  If
                # either conversion fails, discard the whole side rather
                # than allowing diagnostics to affect the solver.
                if raw_band is not None:
                    raw_band = int(raw_band)
                if track is not None:
                    track = str(track)
                return {"raw_band": raw_band, "track": track}
            except Exception:
                return empty

        rover = side(obsr)
        base = side(obsb)
        complete = all(
            side_value[field] is not None
            for side_value in (rover, base)
            for field in ("raw_band", "track")
        )
        comparable = complete and rover == base
        if comparable:
            status = "comparable"
        elif not any(value is not None
                     for item in (rover, base)
                     for value in item.values()):
            status = "unavailable"
        elif not complete:
            status = "incomplete"
        else:
            status = "mismatch"
        return {
            "rover": rover,
            "base": base,
            "completeness": {
                "rover": all(value is not None for value in rover.values()),
                "base": all(value is not None for value in base.values()),
            },
            "status": status,
            "comparable": bool(comparable),
        }

    def _trace_dd_row(self, context, *, obsr, obsb, system, ref_sat,
                      target_sat, slot, kind, observed_minus_geometry,
                      predicted_state_term, innovation, h_row, status,
                      row_index=None, covariance_block=None,
                      covariance_row=None, ref_variance=None,
                      target_variance=None, los_ref=None, los_target=None):
        """Append one attempted DD row to the observational trace."""
        if context is None:
            return
        raw_signal = self._trace_raw_signal_fields(
            obsr, obsb, target_sat, slot, kind)
        ref_raw_signal = self._trace_raw_signal_fields(
            obsr, obsb, ref_sat, slot, kind)
        raw_band = (raw_signal["rover"]["raw_band"]
                    if raw_signal["comparable"] and ref_raw_signal["comparable"]
                    else None)
        track = (raw_signal["rover"]["track"]
                 if raw_signal["comparable"] and ref_raw_signal["comparable"]
                 else None)
        both_comparable = bool(
            raw_signal["comparable"] and ref_raw_signal["comparable"])
        geometry_terms = self._trace_geometry_terms(
            context, ref_sat, target_sat, los_ref=los_ref,
            los_target=los_target)
        if both_comparable:
            row_raw_status = "comparable"
        elif (raw_signal["status"] == "unavailable"
              and ref_raw_signal["status"] == "unavailable"):
            row_raw_status = "unavailable"
        elif (raw_signal["status"] == "mismatch"
              or ref_raw_signal["status"] == "mismatch"):
            row_raw_status = "mismatch"
        else:
            row_raw_status = "incomplete"
        row = {
            "attempt_index": int(context["_dd_attempt_counter"]),
            "row_index": None if row_index is None else int(row_index),
            "status": str(status),
            "system": _trace_system_name(system),
            "state_stage": "pre_measurement",
            "ref_sat": int(ref_sat),
            "target_sat": int(target_sat),
            "ref_prn": sat2id(int(ref_sat)),
            "target_prn": sat2id(int(target_sat)),
            "slot": int(slot),
            "kind": str(kind),
            "raw_band": raw_band,
            "track": track,
            "raw_signal": {
                "rover": raw_signal["rover"],
                "base": raw_signal["base"],
                "completeness": raw_signal["completeness"],
                "status": raw_signal["status"],
            },
            "target_raw_signal": {
                "rover": raw_signal["rover"],
                "base": raw_signal["base"],
                "completeness": raw_signal["completeness"],
                "status": raw_signal["status"],
            },
            "target_raw_signal_comparable": raw_signal["comparable"],
            "ref_raw_signal": {
                "rover": ref_raw_signal["rover"],
                "base": ref_raw_signal["base"],
                "completeness": ref_raw_signal["completeness"],
                "status": ref_raw_signal["status"],
            },
            "ref_raw_signal_comparable": ref_raw_signal["comparable"],
            "raw_signal_comparable": both_comparable,
            "raw_signal_status": row_raw_status,
            "dd_observation_minus_geometry": float(observed_minus_geometry),
            "observation_minus_computed_model": float(observed_minus_geometry),
            "observation_minus_geometry_units": "m",
            "prefit_innovation": float(innovation),
            "prefit_definition": "l = DD(observation - computed_model) [m]",
            "postfit_definition": "postfit_residual = l - A*dx [m]",
            "postfit_residual": None,
            "geometry_dd": geometry_terms["geometry_dd"],
            "geometry_units": "m",
            "geometry_definition": (
                "rho_rover(ref)-rho_base(ref)-rho_rover(target)+"
                "rho_base(target); pure ECEF range term, ref-target"
            ),
            "sat_ref_ecef": geometry_terms["sat_ref_ecef"],
            "sat_target_ecef": geometry_terms["sat_target_ecef"],
            "los_ref": geometry_terms["los_ref"],
            "los_target": geometry_terms["los_target"],
            "los_dd": geometry_terms["los_dd"],
            "linearized_prediction_correction": float(predicted_state_term),
            "prediction_definition": (
                "phase: lambda_ref*N_ref-lambda_target*N_target plus "
                "optional GLO HW term; code: 0 [m]"
            ),
            "predicted_state_term": float(predicted_state_term),
            "innovation": float(innovation),
            "residual": float(innovation),
            "h_nonzero": self._trace_h_nonzero(h_row),
            "design_row": self._trace_h_nonzero(h_row),
            "design_row_definition": "A = d(computed_model)/d(state_error)",
            "design_row_units": "mixed SI derivatives per state component",
            "h_norm": float(np.linalg.norm(h_row)),
            "covariance_block": (
                None if covariance_block is None else int(covariance_block)),
            "covariance_block_id": (
                None if covariance_block is None else
                f"{context['trace_instance_id']}:{context['epoch']}:{covariance_block}"),
            "covariance_row": (
                None if covariance_row is None else int(covariance_row)),
            "r_ref": None if ref_variance is None else float(ref_variance),
            "r_target": None if target_variance is None else float(target_variance),
            "r_diag": (
                None if ref_variance is None or target_variance is None
                else float(ref_variance + target_variance)),
        }
        context["dd_rows"].append(row)
        context["_dd_attempt_counter"] += 1

    def _trace_finish(self, context, v, H, nav, si, amb_init_target=None,
                      R=None, P=None):
        """Complete and emit one immutable epoch-level summary."""
        if context is None:
            return
        selected = context.pop("_selected_ref_sats")
        references = {}
        for system, refs in selected.items():
            if refs:
                # The normal nf=2 path selects one highest-elevation reference
                # for every slot.  Preserve a scalar for that common case and
                # retain per-slot values if data quality requires divergence.
                references[system] = refs[0] if all(ref == refs[0] for ref in refs) else refs
        context["reference_sats"] = references
        context["selected_ref_sats"] = selected
        context["refs"] = references
        context["counts"] = {
            "code": {
                "attempted": int(context.pop("_code_attempted", 0)),
                "accepted": int(context["n_code"]),
                "rejected": int(context.pop("_code_rejected", 0)),
            },
            "phase": {
                "attempted": int(context.pop("_phase_attempted", 0)),
                "accepted": int(context["n_phase"]),
                "rejected": int(context.pop("_phase_rejected", 0)),
            },
        }
        context["n_total"] = int(len(v))
        matrix_shape = list(np.asarray(R).shape) if R is not None else [
            int(len(v)), int(len(v))]
        covariance_rows = [
            {
                "row_index": row["row_index"],
                "block": row["covariance_block"],
                "block_id": row["covariance_block_id"],
                "block_row": row["covariance_row"],
                "reference_variance": row["r_ref"],
                "target_variance": row["r_target"],
                "diagonal": row["r_diag"],
            }
            for row in context["dd_rows"]
            if row["status"] == "accepted"
        ]
        context["covariance"] = {
            "representation": "dd_shared_reference",
            "matrix_shape": matrix_shape,
            "row_count": int(len(v)),
            "rows": covariance_rows,
            "blocks": [],
        }
        for block in context.pop("_covariance_blocks", []):
            block = dict(block)
            indices = list(block.pop("row_indices", []))
            block["row_indices"] = indices
            block["packed_lower"] = []
            if indices and R is not None:
                matrix = np.asarray(R, dtype=float)
                submatrix = matrix[np.ix_(indices, indices)]
                block["packed_lower"] = [
                    float(submatrix[i, j])
                    for i in range(len(indices))
                    for j in range(i + 1)
                ]
            context["covariance"]["blocks"].append(block)
        context.pop("_dd_attempt_counter", None)
        context.pop("_block_counter", None)
        context.pop("_obsr", None)
        context.pop("_obsb", None)
        context["residual"] = {
            "shape": list(np.asarray(v).shape),
            "rank": 0 if len(v) == 0 else 1,
            "norm": float(np.linalg.norm(v)) if len(v) else 0.0,
        }
        jacobian = {"shape": list(np.asarray(H).shape)}
        if not H.size:
            jacobian["rank"] = 0
        else:
            try:
                if not np.all(np.isfinite(H)):
                    raise ValueError("non_finite")
                jacobian["rank"] = int(np.linalg.matrix_rank(H))
            except Exception as exc:
                jacobian["rank"] = "unavailable"
                jacobian["rank_error"] = str(exc)
        context["jacobian"] = jacobian
        # Keep these aliases convenient for line-oriented consumers.
        context["residual_shape"] = context["residual"]["shape"]
        context["jacobian_shape"] = context["jacobian"]["shape"]
        context["jacobian_rank"] = context["jacobian"]["rank"]
        pre_state = context.pop("_pre_state", None)
        pre_si = context.pop("_pre_si", si)
        pre_x = context.pop("_pre_x", None)
        context.pop("_geometry", None)
        snapshot = self._trace_state_snapshot(
            pre_state, pre_si, pre_x, stage="pre_measurement", P=P)
        context.update(snapshot)
        context["state_stage"] = "pre_measurement"
        context["stage"] = "pre_measurement"
        context["nominal_state"] = deepcopy(snapshot)
        context["update"] = {
            "attempted": True,
            "accepted": None,
            "status": "pending",
            "definition": "pre_measurement snapshot precedes EKF/Joseph update",
        }
        if amb_init_target is not None:
            context["float_update_count"] = int(
                getattr(amb_init_target, "_meas_count", 0))
            context["float_state_counter"] = context["float_update_count"]
        # This builder never invokes ambiguity resolution.  ``nav.nb_ar`` is
        # a number of ambiguity states, not a call counter, and must not be
        # reported as one.
        context["ambiguity_resolution_calls"] = 0
        context["ar_call_counter"] = context["ambiguity_resolution_calls"]
        # Materialize a distinct pre record before emitting.  The post-stage
        # record is derived from this immutable diagnostic snapshot later.
        self._last_trace_record = deepcopy(context)
        self._measurement_trace.emit(self._last_trace_record)

    def emit_trace_stage(self, stage, *, state, si, x=None, P=None,
                         update=None):
        """Emit a post-measurement stage for the last RTK event.

        This method is observational and intentionally no-ops without an
        enabled trace or a completed pre record.  ``update`` is a diagnostic
        payload supplied by the integration boundary; it is never fed back
        into the estimator.
        """
        if self._measurement_trace is None or self._last_trace_record is None:
            return
        stage = str(stage)
        if stage not in {"pre_measurement", "post_measurement"}:
            raise ValueError("trace stage must be pre_measurement or post_measurement")
        record = deepcopy(self._last_trace_record)
        record["stage"] = stage
        record["state_stage"] = stage
        snapshot = self._trace_state_snapshot(state, si, x, stage=stage, P=P)
        record.update(snapshot)
        record["nominal_state"] = deepcopy(snapshot)
        update_record = dict(update or {})
        feedback_x = update_record.get("feedback_x")
        try:
            feedback_values = np.asarray(feedback_x, dtype=float).reshape(-1)
        except Exception:
            feedback_values = np.zeros(0, dtype=float)
        postfit = update_record.get("postfit")
        try:
            postfit_values = np.asarray(postfit, dtype=float).reshape(-1)
        except Exception:
            postfit_values = np.zeros(0, dtype=float)
        for row in record.get("dd_rows", []):
            row["state_stage"] = stage
            index = row.get("row_index")
            if index is None:
                continue
            if index < postfit_values.size and np.isfinite(postfit_values[index]):
                row["postfit_residual"] = float(postfit_values[index])
            elif feedback_values.size:
                try:
                    predicted = sum(
                        float(feedback_values[int(key)]) * float(value)
                        for key, value in row.get("design_row", {}).items()
                    )
                    row["postfit_residual"] = float(
                        row["prefit_innovation"] - predicted)
                except Exception:
                    row["postfit_residual"] = None
        update_record["stage"] = stage
        update_record.setdefault("attempted", True)
        update_record.setdefault("accepted", None)
        update_record.setdefault("status", "completed")
        update_record.setdefault(
            "postfit_definition", "postfit_residual = l - A*dx [m]")
        update_record.pop("feedback_x", None)
        update_record.pop("postfit", None)
        record["update"] = update_record
        self._measurement_trace.emit(record)

    # ---- 子类重写 ----
    def _amb_idx(self, sat, freq, si):
        """子类提供 ambiguity 索引 (RTK) 或返回 -1 (RTD)。"""
        return -1

    def _has_amb(self, si):
        return False

    # ---- 共用工具 ----
    @staticmethod
    def _max_el_ref_sat(el, idx, P_diag=None, sig_n0_sq=None):
        """选最高仰角且非刚 reset 的参考卫星 (与 rtklib ddres 一致)。

        ddres 中: 从高到低遍历, 第一个 P[ii,ii] <= sig_n0² 的就选; 若全 reset, 用最高。
        """
        i_el = idx[np.argsort(el[idx])]
        # i_el 是按 el 升序, 倒序遍历选最高
        for i in i_el[::-1]:
            if P_diag is None or sig_n0_sq is None:
                return i
            if P_diag[i] <= sig_n0_sq:
                return i
        return i_el[0]  # 全 reset 时用最高

    def _build_dd(self, nav, x, P, yr, er, yu, eu, sat, el, dt, obsr, si, state,
                  trace_context=None):
        """构造双差 v / H / R (H 为 [m, si.dim])。

        与 rtklib ddres 数学等价, 但:
          - H 转置为 [meas, states]
          - 位置列重映射到 si.pos
          - ambiguity 列重映射到 si.amb_idx(sat, freq) (RTK) 或忽略 (RTD)
        """
        if trace_context is None:
            trace_context = self._active_trace_context
        _c = rCST.CLIGHT
        nf = nav.nf
        ns = len(el)
        v_list, H_rows = [], []
        Ri_list, Rj_list = [], []
        nb_per_block = []   # ddcov 用
        used_pairs = []     # (i, j, freq, code) 供 info
        n_phase_att = n_phase_acc = n_phase_rej = 0
        n_code_att = n_code_acc = n_code_rej = 0
        P_diag = np.diag(P) if P is not None else None
        sig_n0_sq = self._sig_n0 ** 2
        self._trace_candidates(trace_context, sat, nav)

        # 用于 amb 索引: 当 si.amb_idx 返回 -1 (RTD) 时跳过 ambiguity 列
        for sys in nav.gnss_t:
            frequencies = range(nf * 2) if self.use_phase else range(nf, nf * 2)
            for f in frequencies:
                frq = f % nf
                code = 1 if f >= nf else 0
                kind = "code" if code else "phase"
                # 该 sys 内的 sat 索引
                all_idx = self._sys_idx(sat, sys)
                idx = all_idx
                # 同时要求 yr/yu 非零 (有 base+rover 残差)
                nozero = np.where((yr[:, f] != 0) & (yu[:, f] != 0))[0]
                idx = np.intersect1d(idx, nozero)
                if trace_context is not None:
                    for missing in np.setdiff1d(all_idx, idx):
                        self._trace_exclusion(
                            trace_context, sat, sys, missing, frq, kind,
                            "missing_observation")
                if len(idx) == 0:
                    continue
                # 选参考卫星 (最高仰角, 非刚 reset)
                # 注: RTK 才用 P 判 reset, RTD 无 ambiguity 直接选最高
                if self._has_amb(si) and P_diag is not None:
                    ref_i = self._pick_ref_with_P(sat, idx, el, si, frq,
                                                   P_diag, sig_n0_sq, nav)
                else:
                    i_el = idx[np.argsort(el[idx])]
                    ref_i = i_el[-1]
                if trace_context is not None:
                    trace_context["_selected_ref_sats"][
                        _trace_system_name(sys)].append(int(sat[ref_i]))
                freqi = sat2freq(sat[ref_i], frq, nav)
                lami = _c / freqi
                block_count = 0
                block_start = len(v_list)
                attempted_block_start = (
                    trace_context["_dd_attempt_counter"]
                    if trace_context is not None else None)
                if trace_context is not None:
                    covariance_block = trace_context["_block_counter"]
                    trace_context["_block_counter"] += 1
                else:
                    covariance_block = len(nb_per_block)
                for j in idx:
                    if j == ref_i:
                        continue
                    if code:
                        n_code_att += 1
                    else:
                        n_phase_att += 1
                    if trace_context is not None:
                        trace_context[
                            "_code_attempted" if code else "_phase_attempted"] += 1
                    # 双差残差 (innovation: v = y - h(x_est) = observed - predicted)
                    # 与 GINav ddres_rtkins / GREAT-MSF gsppflt 一致
                    # rtklib zdres 返回 y = P - rho (observed - predicted),
                    # 故 DD_y = (yu_i - yr_i) - (yu_j - yr_j) 已是 innovation
                    dd_observation_minus_geometry = (
                        (yu[ref_i, f] - yr[ref_i, f])
                        - (yu[j, f] - yr[j, f]))
                    v_nv = dd_observation_minus_geometry
                    # H 行: d(rho_i - rho_j)/d(rr) = -e_i + e_j (几何观测方程)
                    # INS 误差状态 ε (pos_true = pos_nominal - ε):
                    #   dh/d(ε) = -dh/d(pos) = e_i - e_j
                    # 与 GINav ddres_rtkins H(1:3) = LOS_i - LOS_j 一致
                    H_row = np.zeros(si.dim)
                    H_row[si.pos:si.pos + 3] = eu[ref_i, :] - eu[j, :]
                    # 杆臂姿态耦合 (双差): ∂(DD_ρ)/∂att = -(eu_ref - eu_j) · skew(C_b_e · lever_b)
                    lever_b = state.leverarm
                    lever_e = state.C_b_e @ lever_b
                    los_dd = eu[ref_i, :] - eu[j, :]
                    H_row[si.att:si.att + 3] = -los_dd @ skew(lever_e)
                    # 杆臂 Jacobian: ∂(DD_ρ)/∂lever = (eu_ref - eu_j) · C_b_e
                    if si.has_lever_arm():
                        H_row[si.lever_arm:si.lever_arm + 3] = -los_dd @ state.C_b_e
                    if (not code) and self.use_phase:
                        # phase: pred = DD_rho + λ*(N_i - N_j)
                        # innovation: v = obs - pred = DD_y - λ*(N_i - N_j)
                        # 即 v_nv -= λ_i*N_i - λ_j*N_j (减去模糊度预测项)
                        # H 行: dh/d(N_i) = λ_i, dh/d(N_j) = -λ_j (直接状态)
                        freqj = sat2freq(sat[j], frq, nav)
                        lamj = _c / freqj
                        ii_amb = self._amb_idx(sat[ref_i], frq, si)
                        jj_amb = self._amb_idx(sat[j], frq, si)
                        if ii_amb >= 0 and jj_amb >= 0:
                            v_nv -= lami * x[ii_amb] - lamj * x[jj_amb]
                            H_row[ii_amb] = lami
                            H_row[jj_amb] = -lamj
                        else:
                            # 无 amb 索引 (RTD 不应走到这里, 因为 use_phase=False)
                            pass
                    # GLO hw bias (predicted += df*hwbias, innovation: v -= df*hwbias)
                    # 与 GINav ddres_rtkins v -= df*x(il+ff) 一致
                    if sys == uGNSS.GLO and nav.glo_hwbias != 0:
                        freqj = sat2freq(sat[j], frq, nav)
                        df = (freqi - freqj) / nav.dfreq_glo[frq]
                        v_nv -= df * nav.glo_hwbias
                    # outlier test (与 rtklib 一致, 超阈值的跳过)
                    thresadj = 10 if (not code and self._has_amb(si) and P_diag is not None
                                      and (P_diag[self._amb_idx(sat[ref_i], frq, si)] >= sig_n0_sq
                                           or P_diag[self._amb_idx(sat[j], frq, si)] >= sig_n0_sq)) else 1
                    if abs(v_nv) > nav.maxinno[code] * thresadj:
                        # 维护 vsat/rejc, 供 udbias 的失锁计数 (outc) 使用 (与 ddres 一致)
                        nav.vsat[sat[j] - 1, frq] = 0
                        nav.rejc[sat[j] - 1, frq] += 1
                        if code:
                            n_code_rej += 1
                            if trace_context is not None:
                                trace_context["_code_rejected"] += 1
                        else:
                            n_phase_rej += 1
                            if trace_context is not None:
                                trace_context["_phase_rejected"] += 1
                        self._trace_exclusion(
                            trace_context, sat, sys, j, frq, kind, "outlier")
                        self._trace_dd_row(
                            trace_context,
                            obsr=(trace_context.get("_obsr")
                                  if trace_context is not None else None),
                            obsb=(trace_context.get("_obsb")
                                  if trace_context is not None else None),
                            system=sys, ref_sat=sat[ref_i], target_sat=sat[j],
                            slot=frq, kind=kind,
                            observed_minus_geometry=dd_observation_minus_geometry,
                            predicted_state_term=(
                                dd_observation_minus_geometry - v_nv),
                            innovation=v_nv, h_row=H_row, status="rejected",
                            los_ref=eu[ref_i], los_target=eu[j])
                        continue
                    # 单差方差
                    si_idx = sat[ref_i] - 1
                    sj_idx = sat[j] - 1
                    Ri = rtk_varerr(nav, sys, el[ref_i], f, dt,
                                    nav.rcvstd[si_idx, f],
                                    nav.SNR_rover[si_idx, frq],
                                    nav.SNR_base[si_idx, frq])
                    Rj = rtk_varerr(nav, sys, el[j], f, dt,
                                    nav.rcvstd[sj_idx, f],
                                    nav.SNR_rover[sj_idx, frq],
                                    nav.SNR_base[sj_idx, frq])
                    if (not code) and self.use_phase:
                        # 通过 outlier 检验的相位量测标记 vsat=1 (与 ddres 一致)
                        nav.vsat[si_idx, frq] = 1
                        nav.vsat[sj_idx, frq] = 1
                    if code:
                        n_code_acc += 1
                    else:
                        n_phase_acc += 1
                    v_list.append(v_nv)
                    H_rows.append(H_row)
                    Ri_list.append(Ri)
                    Rj_list.append(Rj)
                    used_pairs.append((int(sat[ref_i]), int(sat[j]), frq, code))
                    if trace_context is not None:
                        trace_context["dd_keys"].append({
                            "ref": int(sat[ref_i]),
                            "target": int(sat[j]),
                            "slot": int(frq),
                            "kind": kind,
                        })
                        trace_context["n_code" if code else "n_phase"] += 1
                        self._trace_dd_row(
                            trace_context,
                            obsr=(trace_context.get("_obsr")
                                  if trace_context is not None else None),
                            obsb=(trace_context.get("_obsb")
                                  if trace_context is not None else None),
                            system=sys, ref_sat=sat[ref_i], target_sat=sat[j],
                            slot=frq, kind=kind,
                            observed_minus_geometry=dd_observation_minus_geometry,
                            predicted_state_term=(
                                dd_observation_minus_geometry - v_nv),
                            innovation=v_nv, h_row=H_row, status="accepted",
                            row_index=len(v_list) - 1,
                            covariance_block=covariance_block,
                            covariance_row=block_count,
                            ref_variance=Ri, target_variance=Rj,
                            los_ref=eu[ref_i], los_target=eu[j])
                    block_count += 1
                if block_count > 0:
                    nb_per_block.append(block_count)
                if trace_context is not None and attempted_block_start is not None \
                        and trace_context["_dd_attempt_counter"] > attempted_block_start:
                    attempt_indices = list(range(
                        attempted_block_start,
                        trace_context["_dd_attempt_counter"]))
                    row_indices = list(range(
                        block_start, block_start + block_count))
                    if block_count == len(attempt_indices):
                        block_status = "accepted"
                    elif block_count:
                        block_status = "partial_rejection"
                    else:
                        block_status = "all_rejected"
                    block_id = (
                        f"{trace_context['trace_instance_id']}:{trace_context['epoch']}"
                        f":{covariance_block}")
                    trace_context["_covariance_blocks"].append({
                        "block": int(covariance_block),
                        "block_id": block_id,
                        "system": _trace_system_name(sys),
                        "kind": kind,
                        "slot": int(frq),
                        "ref_sat": int(sat[ref_i]),
                        "ref_prn": sat2id(int(sat[ref_i])),
                        "row_start": int(block_start),
                        "row_count": int(block_count),
                        "row_indices": row_indices,
                        "attempt_indices": attempt_indices,
                        "attempt_count": len(attempt_indices),
                        "status": block_status,
                        "off_diagonal": "reference_variance",
                    })
        info = {"pairs": used_pairs, "n": len(v_list),
                "ref_sats": sorted({p[0] for p in used_pairs}),
                "n_phase_att": n_phase_att, "n_phase_acc": n_phase_acc,
                "n_phase_rej": n_phase_rej,
                "n_code_att": n_code_att, "n_code_acc": n_code_acc,
                "n_code_rej": n_code_rej}
        if not v_list:
            return (np.array([]), np.zeros((0, si.dim)),
                    np.zeros((0, 0)), info)
        v = np.array(v_list)
        H = np.array(H_rows)
        # ddcov 构造 R
        R = ddcov(np.array(nb_per_block), len(nb_per_block),
                  np.array(Ri_list), np.array(Rj_list), len(v_list))
        info["n"] = len(v)
        return v, H, R, info

    @staticmethod
    def _sys_idx(sat_list, sys_ref):
        """与 rtklib sysidx 等价: 返回 sys 内 sat 的下标列表。"""
        idx = []
        for k, s in enumerate(sat_list):
            sys, _ = sat2prn(s)
            if sys == sys_ref:
                idx.append(k)
        return np.array(idx, dtype=int)

    def _pick_ref_with_P(self, sat, idx, el, si, frq, P_diag, sig_n0_sq, nav):
        """与 rtklib ddres 选 ref sat 一致: 高仰角优先 + 非刚 reset。"""
        i_el = idx[np.argsort(el[idx])]
        for i in i_el[::-1]:
            ii_amb = self._amb_idx(sat[i], frq, si)
            if ii_amb < 0 or P_diag[ii_amb] <= sig_n0_sq:
                return i
        return i_el[0]


class RtkTcMeas(_DdBase):
    """RTK-INS 双差载波+伪距量测 (参考 rtkpos.ddres)。

    v[k] = (yu[i,f]-yr[i,f]) - (yu[j,f]-yr[j,f]) - λ_i*N_i + λ_j*N_j  (phase)
    v[k] = (yu[i,f]-yr[i,f]) - (yu[j,f]-yr[j,f])                       (code)
    """

    def __init__(self, config: dict, *, trace_sink=None, trace_callback=None):
        super().__init__(config, trace_sink=trace_sink,
                         trace_callback=trace_callback)
        self.use_phase = True
        self.use_code = True

    def _amb_idx(self, sat, freq, si):
        if not si.has_ambiguity():
            return -1
        return si.amb_idx(sat, freq)

    def _has_amb(self, si):
        return si.has_ambiguity()

    def build(self, state, obsr, nav, si, x=None, obsb=None, P=None,
             amb_init_target=None, previous_obs_t=None):
        """构造 RTK 双差量测。

        amb_init_target: 可选估计器引用, 通过 rtklib ``udbias`` 做模糊度
        时域更新 (周跳探测/失锁重置/相位-伪距重初始化/随机游走)。
        previous_obs_t: 上一 GNSS 历元时刻 (``gtime_t``), 供 ``udbias``
        计算历元间隔 (随机游走步长 + 失锁计时)。
        """
        trace_context = self._trace_begin(
            obsr, obsb, nav, state=state, si=si, x=x)
        if obsb is None:
            v = np.array([])
            H = np.zeros((0, si.dim))
            self._trace_finish(
                trace_context, v, H, nav, si, R=np.zeros((0, 0)), P=P)
            return v, H, np.zeros((0, 0)), {}
        # 1. 卫星位置 / 钟差
        rs, var, dts, svh = satposs(obsr, nav)
        rsb, varb, dtsb, svhb = satposs(obsb, nav)
        # 2. base zdres (使用 nav.rb)
        nav.vsat[:, :] = 0   # 与 relpos 一致, 清零
        yr, er, azelr = zdres(nav, obsb, rsb, dtsb, svhb, varb, nav.rb, 0)
        # 3. 共视卫星
        ns, iu, ir = selsat(nav, obsr, obsb, azelr[:, 1])
        # 模糊度时域更新 (与 ignav udbias 等价): 周跳探测/失锁重置/
        # 相位-伪距重初始化/随机游走。每个历元都执行 (即使 ns<=0),
        # 以维护 outc/slip/随机游走状态, 与 rtklib relpos 在 selsat 前
        # 调 udstate 一致。
        if amb_init_target is not None:
            sync_tc_ambiguities_with_rtklib(
                nav, obsb, obsr, iu, ir, amb_init_target, previous_obs_t)
        if ns <= 0:
            # ``udbias`` has consumed this epoch's slip mask even without a
            # common satellite.  Save LLI history and clear that mask before
            # the next epoch can regain common observations.
            save_tc_phase_state(nav, obsb, obsr, iu, ir)
            v = np.array([])
            H = np.zeros((0, si.dim))
            self._trace_finish(
                trace_context, v, H, nav, si, R=np.zeros((0, 0)), P=P)
            return v, H, np.zeros((0, 0)), {}
        # 4. rover zdres (使用 INS 天线位置)
        rr = state.pos_e + state.C_b_e @ state.leverarm
        self._trace_set_geometry(
            trace_context, obsr, obsb, rs, rsb, rr, nav.rb)
        yu, eu, azel = zdres(nav, obsr, rs, dts, svh, var, rr, 1)
        # decode stdevs
        from src.core.gnss.rtklib import rinex as rn
        rn.rcvstds(nav, obsr)
        # 5. SNR 保存 (与 relpos 一致)
        for f in range(nav.nf):
            nav.SNR_rover[obsr.sat[iu] - 1, f] = obsr.S[iu, f]
            nav.SNR_base[obsb.sat[ir] - 1, f] = obsb.S[ir, f]
        # 6. 截取共视卫星
        yr, er = yr[ir, :], er[ir, :]
        yu, eu = yu[iu, :], eu[iu, :]
        sats = obsr.sat[iu]
        els = azel[iu, 1]
        nav.azel[sats - 1] = azel[iu]
        # 7. dt
        dt = timediff(obsr.t, obsb.t)
        nav.dt = dt
        # 8. EKF 状态向量 x (含 amb)
        if x is None:
            x = np.zeros(si.dim)
        # 8.5 udbias 已在上方 (selsat 后) 执行, 这里重算 effective_x
        #     (stored 可能已被 udbias 周期滑/重初始化更新)
        if amb_init_target is not None:
            x = amb_init_target.effective_x()
        if trace_context is not None:
            trace_context["_pre_x"] = np.asarray(x, dtype=float).copy()
        # P 用于 ref sat 选择 (选择非刚 reset 的卫星作参考)
        # 9. 构造双差
        self._active_trace_context = trace_context
        try:
            # Keep the historical call shape intact for diagnostic/test
            # subclasses that replace _build_dd; the real implementation
            # reads the active context above.
            v, H, R, info = self._build_dd(
                nav, x, P, yr, er, yu, eu, sats, els, dt, obsr, si, state)
        finally:
            self._active_trace_context = None
        # 10. 失锁计数重置 + 相位/LLI 状态保存 (与 relpos 尾部一致)
        for f in range(nav.nf):
            ix = np.where(nav.vsat[:, f] > 0)[0]
            nav.outc[ix, f] = 0
        save_tc_phase_state(nav, obsb, obsr, iu, ir)
        self._trace_finish(trace_context, v, H, nav, si,
                           amb_init_target=amb_init_target, R=R, P=P)
        return v, H, R, info


class RtdTcMeas(_DdBase):
    """RTD-INS 双差伪距量测 (无模糊度, 仅 code)。

    v[k] = (yu[i,f]-yr[i,f]) - (yu[j,f]-yr[j,f])   (仅 P, 不含 phase)
    """

    def __init__(self, config: dict):
        super().__init__(config)
        self.use_phase = False
        self.use_code = True
        ins = config.get("ins", {}) if config else {}
        self.use_doppler = bool(ins.get("tc_use_doppler", False))
        self._doppler_sigma = float(config.get("gnss", {}).get("tc_doppler_sigma", 0.3))

    def _amb_idx(self, sat, freq, si):
        return -1

    def _has_amb(self, si):
        return False

    def _build_doppler_dd(self, nav, si, sat, el, eu, doppler_r,
                          doppler_b, velocity_e=None):
        sat = np.asarray(sat)
        dr = np.asarray(doppler_r, dtype=float).reshape(-1)
        db = np.asarray(doppler_b, dtype=float).reshape(-1)
        idx = np.flatnonzero((dr != 0.0) & (db != 0.0))
        if len(dr) != len(sat) or len(db) != len(sat) or len(idx) < 2:
            return np.array([]), np.zeros((0, si.dim)), np.zeros((0, 0)), {"n_doppler_acc": 0}
        ref_i = idx[np.argmax(np.asarray(el)[idx])]
        vals, rows, ri, rj = [], [], [], []
        for j in idx:
            if j == ref_i:
                continue
            row = np.zeros(si.dim)
            row[si.vel:si.vel + 3] = np.asarray(eu)[ref_i] - np.asarray(eu)[j]
            observed = (dr[ref_i] - db[ref_i]) - (dr[j] - db[j])
            # Range-rate prediction is -H_vel*v_nominal for this error-state
            # convention, so innovation is observed + H_vel*v_nominal.
            predicted_error = (float(row[si.vel:si.vel + 3] @ velocity_e)
                               if velocity_e is not None else 0.0)
            vals.append(observed + predicted_error)
            rows.append(row)
            ri.append(self._doppler_sigma ** 2)
            rj.append(self._doppler_sigma ** 2)
        R = ddcov(np.array([len(vals)]), 1, np.asarray(ri), np.asarray(rj), len(vals))
        return np.asarray(vals), np.asarray(rows), R, {"n_doppler_acc": len(vals), "doppler_ref_sat": int(sat[ref_i])}

    def build(self, state, obsr, nav, si, x=None, obsb=None):
        """构造 RTD 双差伪距量测。"""
        if obsb is None:
            return np.array([]), np.zeros((0, si.dim)), np.zeros((0, 0)), {}
        rs, var, dts, svh = satposs(obsr, nav)
        rsb, varb, dtsb, svhb = satposs(obsb, nav)
        nav.vsat[:, :] = 0
        yr, er, azelr = zdres(nav, obsb, rsb, dtsb, svhb, varb, nav.rb, 0)
        ns, iu, ir = selsat(nav, obsr, obsb, azelr[:, 1])
        if ns <= 0:
            return np.array([]), np.zeros((0, si.dim)), np.zeros((0, 0)), {}
        rr = state.pos_e
        yu, eu, azel = zdres(nav, obsr, rs, dts, svh, var, rr, 1)
        from src.core.gnss.rtklib import rinex as rn
        rn.rcvstds(nav, obsr)
        for f in range(nav.nf):
            nav.SNR_rover[obsr.sat[iu] - 1, f] = obsr.S[iu, f]
            nav.SNR_base[obsb.sat[ir] - 1, f] = obsb.S[ir, f]
        yr, er = yr[ir, :], er[ir, :]
        yu, eu = yu[iu, :], eu[iu, :]
        sats = obsr.sat[iu]
        els = azel[iu, 1]
        nav.azel[sats - 1] = azel[iu]
        dt = timediff(obsr.t, obsb.t)
        nav.dt = dt
        if x is None:
            x = np.zeros(si.dim)
        P = None
        # RTD 的 _build_dd 直接只遍历 code 频率，并保留 code DD 的完整协方差。
        v, H, R, info = self._build_dd(
            nav, x, P, yr, er, yu, eu, sats, els, dt, obsr, si, state)
        if self.use_doppler and obsb is not None:
            rmap = {int(s): i for i, s in enumerate(obsr.sat)}
            bmap = {int(s): i for i, s in enumerate(obsb.sat)}
            dr, db = [], []
            for s in sats:
                ir0, ib0 = rmap.get(int(s), -1), bmap.get(int(s), -1)
                if ir0 < 0 or ib0 < 0:
                    dr.append(0.0); db.append(0.0); continue
                lam = rCST.CLIGHT / sat2freq(int(s), 0, nav)
                dr.append(-lam * float(obsr.D[ir0, 0]))
                db.append(-lam * float(obsb.D[ib0, 0]))
            vd, Hd, Rd, idop = self._build_doppler_dd(nav, si, sats, els, eu, dr, db, state.vel_e)
            if len(vd):
                n0 = len(v)
                v = np.concatenate((v, vd))
                H = np.vstack((H, Hd))
                R = np.block([[R, np.zeros((n0, len(vd)))],
                              [np.zeros((len(vd), n0)), Rd]])
                info.update(idop)
        return v, H, R, info
