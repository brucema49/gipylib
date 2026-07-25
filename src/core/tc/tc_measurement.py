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
import numpy as np

from src.core.data_types import InsState
from src.core.tc.tc_state_index import TcStateIndex
from src.core.gnss.rtklib.rtkcmn import (geodist, satazel, ecef2pos,
                                          satexclude, ionmodel, tropmodel,
                                          tropmapf, sat2freq, uGNSS,
                                          sat2prn, timediff)
from src.core.gnss.rtklib.ephemeris import satposs
from src.core.gnss.rtklib.pntpos import varerr as spp_varerr, prange, gettgd, REL_HUMI
from src.core.gnss.rtklib.rtkpos import zdres, selsat, ddcov, varerr as rtk_varerr
from src.core.gnss.rtklib import rCST


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
    H[clk_bias + sys_off, i] = 1.0 (= ∂pred/∂clk, 直接状态)
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

    def build(self, state, obsr, nav, si, x=None, obsb=None):
        """构造 SPP 伪距量测 (innovation: P - h(est))。

        h(est) = r + dtr_est - c*dts + dion + dtrp
        其中 dtr_est = effective_x[clk_bias + sys_off] (stored + ε_clk)。
        """
        rr = state.pos_e
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
            sys_off = self._sys_clk_offset(sat)
            dtr_est = 0.0
            if x is not None and si.clk_bias >= 0:
                dtr_est = float(x[si.clk_bias + sys_off])
            # 残差 (innovation: P - h(est), 与 GINav/GREAT-MSF 一致)
            rho = r + dtrp + dion
            v_i = P - (rho + dtr_est - rCST.CLIGHT * dts[i])
            # H 行: H[pos] = +LOS = ∂pred/∂δr = -∂pred/∂pos (INS 误差状态)
            H_row = np.zeros(si.dim)
            H_row[si.pos:si.pos + 3] = e
            H_row[si.clk_bias + sys_off] = 1.0
            # R: 使用实际伪距噪声 sigma (含电离层/对流层残差 + 多径)
            # spp_varerr 仅返回 ~0.004m (最小二乘加权用), 不适合 EKF 量测噪声
            sys_gnss = _sys_gnss(sat)
            efact = nav.efact[sys_gnss]   # GPS=1.0, GLO=1.5, GAL=1.0
            sig = self._spp_sigma * efact
            R_i = sig ** 2
            v_list.append(v_i)
            H_rows.append(H_row)
            R_diag.append(R_i)
            used_sats.append(sat)
        if not v_list:
            return np.array([]), np.zeros((0, si.dim)), np.zeros((0, 0)), {}
        v = np.array(v_list)
        H = np.array(H_rows)              # H[m, dim]
        R = np.diag(R_diag)
        return v, H, R, {"sats": used_sats, "n": len(v)}

    @staticmethod
    def _sys_clk_offset(sat):
        """GPS=0, GLO=1, GAL=2 (对应 clk_bias 三维块)。"""
        if 1 <= sat <= 32:
            return 0
        if 101 <= sat <= 132:
            return 1
        return 2


def _sys_gnss(sat):
    """卫星编号 → uGNSS 系统常量 (供 varerr)。"""
    if 1 <= sat <= 32:
        return uGNSS.GPS
    if 101 <= sat <= 132:
        return uGNSS.GLO
    return uGNSS.GAL


class _DdBase(TcMeasurement):
    """RTK/RTD 双差量测共用骨架。

    复用 rtklib-py 的 zdres() 计算零差残差, selsat() 选择共视卫星,
    本类负责:
      - 在 INS 位置上重算 rover zdres
      - 把 rtklib 的 H[states, meas] 转置为 H[meas, states] 并重映射到 si 索引
      - 维护 ambiguity 槽位 (RTK 才有)
    """

    def __init__(self, config: dict):
        gnss = config.get("gnss", {}) if config else {}
        self.elmin = math.radians(gnss.get("elmin", 15.0))
        self.use_phase = True     # RTK=True, RTD=False
        self.use_code = True
        # 用于决定 ref sat 的 sig_n0 (与 rtklib ddres 一致)
        self._sig_n0 = float(config.get("gnss", {}).get("sig_n0", 30.0))

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

    def _build_dd(self, nav, x, P, yr, er, yu, eu, sat, el, dt, obsr, si):
        """构造双差 v / H / R (H 为 [m, si.dim])。

        与 rtklib ddres 数学等价, 但:
          - H 转置为 [meas, states]
          - 位置列重映射到 si.pos
          - ambiguity 列重映射到 si.amb_idx(sat, freq) (RTK) 或忽略 (RTD)
        """
        _c = rCST.CLIGHT
        nf = nav.nf
        ns = len(el)
        # 单差最大对数 (phase+code 各一组)
        max_nv = ns * nf * 2
        v_list, H_rows = [], []
        Ri_list, Rj_list = [], []
        nb_per_block = []   # ddcov 用
        used_pairs = []     # (i, j, freq, code) 供 info
        P_diag = np.diag(P) if P is not None else None
        sig_n0_sq = self._sig_n0 ** 2

        # 用于 amb 索引: 当 si.amb_idx 返回 -1 (RTD) 时跳过 ambiguity 列
        for sys in nav.gnss_t:
            for f in range(nf * 2):
                frq = f % nf
                code = 1 if f >= nf else 0
                # 该 sys 内的 sat 索引
                idx = self._sys_idx(sat, sys)
                # 同时要求 yr/yu 非零 (有 base+rover 残差)
                nozero = np.where((yr[:, f] != 0) & (yu[:, f] != 0))[0]
                idx = np.intersect1d(idx, nozero)
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
                freqi = sat2freq(sat[ref_i], frq, nav)
                lami = _c / freqi
                block_count = 0
                for j in idx:
                    if j == ref_i:
                        continue
                    # 双差残差 (innovation: v = y - h(x_est) = observed - predicted)
                    # 与 GINav ddres_rtkins / GREAT-MSF gsppflt 一致
                    # rtklib zdres 返回 y = P - rho (observed - predicted),
                    # 故 DD_y = (yu_i - yr_i) - (yu_j - yr_j) 已是 innovation
                    v_nv = (yu[ref_i, f] - yr[ref_i, f]) - (yu[j, f] - yr[j, f])
                    # H 行: d(rho_i - rho_j)/d(rr) = -e_i + e_j (几何观测方程)
                    # INS 误差状态 ε (pos_true = pos_nominal - ε):
                    #   dh/d(ε) = -dh/d(pos) = e_i - e_j
                    # 与 GINav ddres_rtkins H(1:3) = LOS_i - LOS_j 一致
                    H_row = np.zeros(si.dim)
                    H_row[si.pos:si.pos + 3] = eu[ref_i, :] - eu[j, :]
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
                    v_list.append(v_nv)
                    H_rows.append(H_row)
                    Ri_list.append(Ri)
                    Rj_list.append(Rj)
                    used_pairs.append((int(sat[ref_i]), int(sat[j]), frq, code))
                    block_count += 1
                if block_count > 0:
                    nb_per_block.append(block_count)
        if not v_list:
            return (np.array([]), np.zeros((0, si.dim)),
                    np.zeros((0, 0)), {})
        v = np.array(v_list)
        H = np.array(H_rows)
        # ddcov 构造 R
        R = ddcov(np.array(nb_per_block), len(nb_per_block),
                  np.array(Ri_list), np.array(Rj_list), len(v_list))
        info = {"pairs": used_pairs, "n": len(v),
                "ref_sats": sorted({p[0] for p in used_pairs})}
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

    def __init__(self, config: dict):
        super().__init__(config)
        self.use_phase = True
        self.use_code = True

    def _amb_idx(self, sat, freq, si):
        if not si.has_ambiguity():
            return -1
        return si.amb_idx(sat, freq)

    def _has_amb(self, si):
        return si.has_ambiguity()

    def build(self, state, obsr, nav, si, x=None, obsb=None):
        """构造 RTK 双差量测。"""
        if obsb is None:
            return np.array([]), np.zeros((0, si.dim)), np.zeros((0, 0)), {}
        # 1. 卫星位置 / 钟差
        rs, var, dts, svh = satposs(obsr, nav)
        rsb, varb, dtsb, svhb = satposs(obsb, nav)
        # 2. base zdres (使用 nav.rb)
        nav.vsat[:, :] = 0   # 与 relpos 一致, 清零
        yr, er, azelr = zdres(nav, obsb, rsb, dtsb, svhb, varb, nav.rb, 0)
        # 3. 共视卫星
        ns, iu, ir = selsat(nav, obsr, obsb, azelr[:, 1])
        if ns <= 0:
            return np.array([]), np.zeros((0, si.dim)), np.zeros((0, 0)), {}
        # 4. rover zdres (使用 INS 位置)
        rr = state.pos_e
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
        # P 用于 ref sat 选择 (可选)
        P = None   # 由 estimator 提供; verification 时用 None
        # 9. 构造双差
        return self._build_dd(nav, x, P, yr, er, yu, eu, sats, els, dt, obsr, si)


class RtdTcMeas(_DdBase):
    """RTD-INS 双差伪距量测 (无模糊度, 仅 code)。

    v[k] = (yu[i,f]-yr[i,f]) - (yu[j,f]-yr[j,f])   (仅 P, 不含 phase)
    """

    def __init__(self, config: dict):
        super().__init__(config)
        self.use_phase = False
        self.use_code = True

    def _amb_idx(self, sat, freq, si):
        return -1

    def _has_amb(self, si):
        return False

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
        # RTD 只取 code: 在 _build_dd 中通过 use_phase=False 控制
        # 但 _build_dd 仍会遍历 phase/code, 这里在构造后过滤掉 phase 行
        v_all, H_all, R_all, info = self._build_dd(
            nav, x, P, yr, er, yu, eu, sats, els, dt, obsr, si)
        if len(v_all) == 0:
            return v_all, H_all, R_all, info
        # info['pairs'] 中的 code 字段: 0=phase, 1=code
        pairs = info["pairs"]
        code_mask = np.array([p[3] == 1 for p in pairs])
        v = v_all[code_mask]
        H = H_all[code_mask]
        # R 需要重新构造 (因为删除了 phase 行, ddcov 的分块结构变了)
        # 简单处理: 对角取 R_all 对角元素
        if R_all.shape[0] == len(v_all):
            R = np.diag(np.diag(R_all)[code_mask])
        else:
            R = R_all[code_mask][:, code_mask]
        info["n"] = len(v)
        info["pairs"] = [p for p in pairs if p[3] == 1]
        return v, H, R, info
