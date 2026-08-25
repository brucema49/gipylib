"""ignav 兼容 stat 文件输出器 (GIPY_STAT_V1)。

参考 ignav rtkpos.cc outsolstat() 的基础记录名与字段顺序:
  $POSI     week tow ista x y z                     (ECEF m)
  $VELACCI  week tow ista vE vN vU aE aN aU          (ENU m/s, m/s²)
  $ATT      week tow ista roll pitch yaw             (FRD deg)
  $GBIAS    week tow ista bgx bgy bgz                (body rad/s)
  $ABIAS    week tow ista bax bay baz                (body m/s²)
  $INSTA    week tow ista

与 ignav 的兼容差异 (在文件头声明, 不复制其缺陷):
  - $ABIAS 三轴均为真实加计零偏 (ignav 历史实现把 y/z 打印成 bg[1]/bg[2]);
  - 每条记录独占一行且以换行结束 (ignav 部分 $INSTA 缺换行导致粘连);
  - $VELACCI 的加速度为 $VELACCI 速度的有限差分 (gipylib InsState 不存加速度)。

GIPY_* 扩展记录 (stat_level>=2, TC 更新点):
  GIPY_EPOCH,week,sow,update_flag,qins,mode,nsv,n_phase,n_code,ref_sat
  GIPY_COV, week,sow,Pp*3,Pv*3,Pa*3,Pgb*3,Pba*3,min_eig,cond,sym_err
  GIPY_INNOV,week,sow,innov_norm,n_meas,n_phase_acc,n_code_acc
  GIPY_GAIN, week,sow,k_pos,k_vel,k_att,k_gbias,k_abias

ista 语义 (文件头声明): 0=GNSS-only/未初始化, 2=机械编排+协方差传播,
3=量测更新 (与 .rslt 的 Qins 一致, 仅作输出属性, 更新识别用 update_flag)。
"""
import os
from pathlib import Path

import numpy as np

from src.core.time_utils import unix_to_gpst
from src.log.solution_writer import ecef2llh, ecef2enu_matrix


def _fmt(v, prec=".4f"):
    """有限值格式化; nan/inf 显式输出。"""
    if v is None or not np.isfinite(v):
        return "nan"
    return format(float(v), prec)


class StatWriter:
    """ignav 风格 stat 文件输出器。

    用法 (TC):
        sw = StatWriter(output_dir, stat_level=1, stat_rate="update",
                        filename="", ref_filename="RTKINS.rslt", mode="tc")
        sw.open()
        sw.write(state, P, si, q, qins, num_sv, update_info=None)
        sw.write_gnss_only(t, rr, quality, ns, pos_sd)   # 初始化前
        sw.close()

    用法 (off 纯 GNSS):
        sw.write_gnss(gnss_solution)
    """

    SCHEMA = "GIPY_STAT_V1"

    def __init__(self, output_dir: str, stat_level: int = 1,
                 stat_rate: str = "update", filename: str = "",
                 ref_filename: str = "RTKINS.rslt", mode: str = "tc"):
        self.output_dir = output_dir
        self.stat_level = int(stat_level)
        self.stat_rate = str(stat_rate)
        stem = Path(ref_filename).stem
        self.filename = filename if filename else stem + ".stat"
        self.mode = mode
        self._fp = None
        self._last_t = None
        self._last_vel_enu = None
        self._last_second = None
        self._init_wait_logged = False
        self._update_info = None

    # -------------------------------------------------- 生命周期
    @property
    def enabled(self) -> bool:
        return self.stat_level > 0

    def open(self) -> None:
        if not self.enabled:
            return
        os.makedirs(self.output_dir, exist_ok=True)
        path = Path(self.output_dir) / self.filename
        self._fp = open(path, "w", encoding="utf-8")
        self._fp.write(
            f"# {self.SCHEMA}\n"
            "# time_system=GPST time_precision=0.001\n"
            f"# mode={self.mode} state_rate={self.stat_rate}\n"
            "# position_frame=ECEF velocity_frame=ENU order=E,N,U "
            "attitude_frame=FRD order=roll,pitch,yaw\n"
            "# units: pos=m vel=m/s acc=m/s^2 att=deg gbias=rad/s abias=m/s^2 "
            "cov=m^2,m^2/s^2,rad^2,(rad/s)^2,(m/s^2)^2\n"
            "# qins: 0=GNSS-only/uninitialized 2=mechanization+P propagation "
            "3=measurement update\n"
            "# compatibility: ignav-basic-records=yes "
            "gipy-extensions=GIPY_*; $ABIAS 三轴均为真实加计零偏; "
            "每条记录独占一行\n"
        )
        self._closed = False

    def close(self) -> None:
        if self._fp is not None:
            self._fp.close()
            self._fp = None

    def flush(self) -> None:
        if self._fp is not None:
            self._fp.flush()

    # -------------------------------------------------- 速率过滤
    def _pass_rate(self, timestamp: float, qins: int) -> bool:
        if self.stat_rate == "imu":
            return True
        if self.stat_rate == "update":
            return qins != 2  # 更新点 + 初始化前的 GNSS 历元
        if self.stat_rate == "second":
            sec = int(timestamp)
            if self._last_second == sec:
                return False
            self._last_second = sec
            return True
        return True

    @staticmethod
    def _time(timestamp: float):
        week, sow = unix_to_gpst(timestamp)
        return week, sow

    # -------------------------------------------------- TC/LC 状态
    def write(self, state, P, si, q: int, qins: int, num_sv: int,
              update_info: dict | None = None) -> None:
        if self._fp is None or self.stat_level <= 0:
            return
        if not self._pass_rate(state.timestamp, qins):
            return
        week, sow = self._time(state.timestamp)
        ista = qins

        # 加速度: $VELACCI 速度的有限差分 (ENU)
        R = ecef2enu_matrix(ecef2llh(state.pos_e))
        vel_enu = state.vel_e @ R.T
        acc_enu = np.full(3, np.nan)
        if self._last_vel_enu is not None and self._last_t is not None:
            dt = state.timestamp - self._last_t
            if dt > 1e-3:
                acc_enu = (vel_enu - self._last_vel_enu) / dt
        self._last_vel_enu = vel_enu.copy()
        self._last_t = state.timestamp

        att_deg = state.att_rpy / np.pi * 180.0

        f = self._fp
        f.write("$POSI,%d,%.3f,%d,%s,%s,%s\n"
                % (week, sow, ista, *(_fmt(v, ".4f") for v in state.pos_e)))
        f.write("$VELACCI,%d,%.3f,%d,%s,%s,%s,%s,%s,%s\n"
                % (week, sow, ista,
                   *(_fmt(v, ".4f") for v in vel_enu),
                   *(_fmt(a, ".4f") for a in acc_enu)))
        f.write("$ATT,%d,%.3f,%d,%s,%s,%s\n"
                % (week, sow, ista, *(_fmt(a, ".4f") for a in att_deg)))
        f.write("$GBIAS,%d,%.3f,%d,%s,%s,%s\n"
                % (week, sow, ista, *(_fmt(b, ".7f") for b in state.gyro_bias)))
        f.write("$ABIAS,%d,%.3f,%d,%s,%s,%s\n"
                % (week, sow, ista, *(_fmt(b, ".7f") for b in state.accel_bias)))
        f.write("$INSTA,%d,%.3f,%d\n" % (week, sow, ista))

        if self.stat_level >= 2 and qins == 3:
            self._write_gipy_update(week, sow, P, si, num_sv, update_info)
        if self.stat_rate == "update":
            self.flush()

    # -------------------------------------------------- TC 扩展记录
    def _write_gipy_update(self, week, sow, P, si, num_sv,
                           update_info: dict | None) -> None:
        f = self._fp
        info = update_info or {}

        def blk(idx):
            return ",".join(_fmt(P[i, i], ".6g") for i in idx)

        f.write("GIPY_EPOCH,%d,%.3f,1,3,%s,%d,%s,%s,%s\n"
                % (week, sow, info.get("mode", self.mode), num_sv,
                   _fmt(info.get("n_phase_acc"), ".0f"),
                   _fmt(info.get("n_code_acc"), ".0f"),
                   ",".join(str(s) for s in info.get("ref_sats", []))
                   or "-"))
        f.write("GIPY_COV,%d,%.3f,%s,%s,%s,%s,%s,%s,%s,%s\n"
                % (week, sow,
                   blk(range(si.pos, si.pos + 3)),
                   blk(range(si.vel, si.vel + 3)),
                   blk(range(si.att, si.att + 3)),
                   blk(range(si.gyro_bias, si.gyro_bias + 3)),
                   blk(range(si.accel_bias, si.accel_bias + 3)),
                   _fmt(self._min_eig(P), ".6g"),
                   _fmt(self._cond(P), ".6g"),
                   _fmt(self._sym_err(P), ".3g")))
        if info:
            f.write("GIPY_INNOV,%d,%.3f,%s,%d,%s,%s\n"
                    % (week, sow,
                       _fmt(info.get("innovation_norm"), ".4f"),
                       int(info.get("n_meas", 0)),
                       _fmt(info.get("n_phase_acc"), ".0f"),
                       _fmt(info.get("n_code_acc"), ".0f")))
            f.write("GIPY_GAIN,%d,%.3f,%s,%s,%s,%s,%s\n"
                    % (week, sow,
                       _fmt(info.get("k_pos_norm"), ".6g"),
                       _fmt(info.get("k_vel_norm"), ".6g"),
                       _fmt(info.get("k_att_norm"), ".6g"),
                       _fmt(info.get("k_bg_norm"), ".6g"),
                       _fmt(info.get("k_ba_norm"), ".6g")))

    @staticmethod
    def _min_eig(P) -> float:
        try:
            n = P.shape[0]
            if n > 400:  # 大矩阵采样 INS 块, 避免逐更新点全特征值开销
                return float(np.linalg.eigvalsh(P[:60, :60]).min())
            return float(np.linalg.eigvalsh(P).min())
        except np.linalg.LinAlgError:
            return float("nan")

    @staticmethod
    def _cond(P) -> float:
        try:
            return float(np.linalg.cond(P[:60, :60] if P.shape[0] > 400 else P))
        except np.linalg.LinAlgError:
            return float("nan")

    @staticmethod
    def _sym_err(P) -> float:
        n = min(60, P.shape[0])
        Q = P[:n, :n]
        return float(np.max(np.abs(Q - Q.T)))

    # -------------------------------------------------- 初始化前 / 纯 GNSS
    def write_gnss_only(self, t_gnss=None, rr=None, quality: int = 0,
                        ns: int = 0, pos_sd=None, timestamp=None,
                        pos_e=None) -> None:
        """初始化前/纯 GNSS 历元 → $POS + $INSTA(0)。

        兼容两种调用风格:
          TC:  write_gnss_only(t_gnss, rr, quality, ns, pos_sd)
          LC:  write_gnss_only(timestamp=..., pos_e=..., q=..., num_sv=..., pos_sd=...)
        """
        if self._fp is None or self.stat_level <= 0:
            return
        t = t_gnss if t_gnss is not None else timestamp
        pos = rr if rr is not None else pos_e
        if t is None or pos is None:
            return
        if pos_sd is None:
            pos_sd = np.full(3, np.nan)
        if not self._pass_rate(t, 0):
            return
        week, sow = self._time(t)
        f = self._fp
        f.write("$POS,%d,%.3f,%d,%s,%s,%s,%s,%s,%s\n"
                % (week, sow, quality,
                   *(_fmt(v, ".4f") for v in np.asarray(pos)),
                   *(_fmt(s, ".4f") for s in np.asarray(pos_sd))))
        f.write("$INSTA,%d,%.3f,0\n" % (week, sow))
        if not self._init_wait_logged:
            f.write("GIPY_EVT,%d,%.3f,INIT_WAIT\n" % (week, sow))
            self._init_wait_logged = True
        if self.stat_rate == "update":
            self.flush()

    def write_gnss(self, sol) -> None:
        """off 模式: 纯 GNSS GnssSolution → $POS (+$VELACC 有速度时)。"""
        if self._fp is None or self.stat_level <= 0:
            return
        if not self._pass_rate(sol.timestamp, 0):
            return
        week, sow = self._time(sol.timestamp)
        f = self._fp
        f.write("$POS,%d,%.3f,%d,%s,%s,%s,%s,%s,%s\n"
                % (week, sow, sol.quality,
                   *(_fmt(v, ".4f") for v in sol.position),
                   *(_fmt(s, ".4f") for s in np.asarray(sol.sd))))
        if sol.velocity is not None:
            R = ecef2enu_matrix(ecef2llh(sol.position))
            venu = np.asarray(sol.velocity) @ R.T
            f.write("$VELACC,%d,%.3f,%d,%s,%s,%s,nan,nan,nan\n"
                    % (week, sow, sol.quality,
                       *(_fmt(v, ".4f") for v in venu)))
        f.write("$INSTA,%d,%.3f,0\n" % (week, sow))
        if self.stat_rate == "update":
            self.flush()
