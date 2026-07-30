"""ignav outins 风格 .rslt 输出器 (LLH 位置 + ECEF 速度 + FRD 姿态)。

参考 tools/ignav/ins-gnss/solution.cc outins (L1490-1582)。
组合导航模式 (internal+on) 专用，纯 GNSS 模式仍用 SolutionWriter。

公共参数 (位置/sd/Q/ns/age/ratio) 与 SolutionWriter (RTK.pos) 完全一致:
  - 位置: LLH (lat deg, lon deg, h m)
  - sd: ENU 顺序 (sdn sde sdu sdne sdeu sdun)
INS 多出来的参数 (Qins/速度/姿态) 保持 ignav outins 风格:
  - 速度: ECEF (vx vy vz) + ECEF sd
  - 姿态: FRD deg (roll pitch yaw) + deg sd
"""
import os
from pathlib import Path

import numpy as np

from src.core.data_types import InsState
from src.core.ins.state_index import StateIndex
from src.core.time_utils import unix_to_gpst
from src.log.solution_writer import ecef2llh, ecef2enu_matrix
from src.log.writer_base import WriterBase


def _sqrt_diag(M: np.ndarray) -> np.ndarray:
    """取 3x3 矩阵对角线 sqrt(|·|)，返回 [3] 数组。"""
    return np.sqrt(np.abs(np.diag(M)))


def _signed_sqrt(c: float) -> float:
    """带符号 sqrt: sqrt(|c|) * sign(c)。"""
    if c == 0.0:
        return 0.0
    return float(np.sqrt(abs(c)) * np.sign(c))


class RSLTWriter(WriterBase):
    """ignav outins 风格 .rslt 输出器 (LLH 位置 + ECEF 速度 + FRD 姿态)。

    输出格式:
      表头两行 (注释 + 字段定义) + 每历元一行:
        week sow lat lon h Q Qins ns
        sdn sde sdu sdne sdeu sdun age ratio       (与 RTK.pos 公共参数一致, ENU 顺序)
        vx vy vz sdvx sdvy sdvz sdvxy sdvyz sdvzx  (ECEF, ignav outins 固定格式)
        roll pitch yaw sdroll sdpitch sdyaw        (FRD deg, ignav outins 固定格式)

    位置 LLH, 位置 sd ENU; 速度 ECEF, 速度 sd ECEF; 姿态 deg, 姿态 sd deg。
    age/ratio 填 0 (LC 不跟踪)。
    """

    HEADER = (
        "% (lat/lon/h=WGS84,Q=1:fix,2:float,3:sbas,4:dgps,5:single,6:ppp,"
        "ns=# of satellites),"
        "Qins=1:ins mechanization,"
        "2:ins mechanization and propagate states and covariance,"
        "3:ins-gnss loosely-coupled updates\n"
        "%  GPST          latitude(deg) longitude(deg)  height(m)   "
        "Q Qins  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) "
        "age(s)  ratio    vx(m/s)    vy(m/s)    vz(m/s)       "
        "sdvx       sdvy       sdvz      sdvxy      sdvyz      sdvzx   "
        "roll(deg)  pitch(deg)    yaw(deg)  sdroll(d) sdpitch(d)  sdyaw(d)"
        "  lever_x(m)  lever_y(m)  lever_z(m)  sdlx(m)  sdly(m)  sdlz(m)\n"
    )

    def __init__(self, output_dir: str, filename: str = "RTKLC.rslt"):
        self.output_dir = output_dir
        self.filename = filename
        self._fp = None
        self._closed = False

    def open(self) -> None:
        os.makedirs(self.output_dir, exist_ok=True)
        path = Path(self.output_dir) / self.filename
        self._fp = open(path, "w", encoding="utf-8")
        self._fp.write(self.HEADER)
        self._closed = False

    def write(self, state: InsState, P: np.ndarray, si: StateIndex,
              q: int, qins: int, num_sv: int) -> None:
        """写一行: 位置(LLH)/速度(ECEF)/姿态(FRD) + 协方差。

        Args:
            state: InsState (含 pos_e, vel_e, att_rpy)
            P: 完整状态协方差矩阵 (si.dim x si.dim)
            si: StateIndex (取 pos/vel/att 子块)
            q: 最近 GNSS quality (1/2/4/5)
            qins: 2=time_update, 3=meas_update
            num_sv: 最近 GNSS num_sv
        """
        if self._fp is None:
            raise RuntimeError("RSLTWriter not opened")

        pos_idx, vel_idx, att_idx = si.pos, si.vel, si.att
        Pp = P[pos_idx:pos_idx + 3, pos_idx:pos_idx + 3]
        Pv = P[vel_idx:vel_idx + 3, vel_idx:vel_idx + 3]
        Pa = P[att_idx:att_idx + 3, att_idx:att_idx + 3]

        week, sow = unix_to_gpst(state.timestamp)
        D2R = np.pi / 180.0
        att_deg = state.att_rpy / D2R  # rad → deg

        # 位置: ECEF → LLH (与 SolutionWriter 一致)
        llh = ecef2llh(state.pos_e)
        lat_deg = llh[0] / D2R
        lon_deg = llh[1] / D2R
        h = llh[2]

        # 位置 sd: ECEF Pp → ENU (与 SolutionWriter 一致, 顺序 sdn sde sdu sdne sdeu sdun)
        R = ecef2enu_matrix(llh)
        cov_enu = R @ Pp @ R.T
        sdn = float(np.sqrt(abs(cov_enu[1, 1])))   # N
        sde = float(np.sqrt(abs(cov_enu[0, 0])))   # E
        sdu = float(np.sqrt(abs(cov_enu[2, 2])))   # U
        sdne = _signed_sqrt(float(cov_enu[0, 1]))  # NE = EN
        sdeu = _signed_sqrt(float(cov_enu[2, 0]))  # UE = EU
        sdun = _signed_sqrt(float(cov_enu[1, 2]))  # NU = UN

        # 速度 sd: ECEF (ignav outins 固定格式)
        sdvx, sdvy, sdvz = _sqrt_diag(Pv)
        sdvxy = _signed_sqrt(float(Pv[0, 1]))
        sdvyz = _signed_sqrt(float(Pv[1, 2]))
        sdvzx = _signed_sqrt(float(Pv[0, 2]))

        # 姿态 sd: deg (ignav outins 固定格式)
        sdroll, sdpitch, sdyaw = _sqrt_diag(Pa) / D2R  # rad → deg

        # 杆臂参数 (b 系 FRD, m): 在线估计时输出 state.leverarm + P 对角 sqrt
        # 杆臂更新频率 1Hz (Qins=3 量测更新), 1s 内 100Hz 输出值相同 (状态不变)
        lever = state.leverarm
        if si.has_lever_arm():
            i = si.lever_arm
            sdl = _sqrt_diag(P[i:i + 3, i:i + 3])
        else:
            sdl = np.zeros(3)

        fmt = (
            "%4d %10.3f %14.9f %14.9f %10.4f %3d %3d %3d"
            " %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %6.2f %6.1f"
            " %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f"
            " %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f"
            " %10.5f %10.5f %10.5f %9.5f %9.5f %9.5f\n"
        )
        self._fp.write(fmt % (
            week, sow,
            lat_deg, lon_deg, h,
            q, qins, num_sv,
            sdn, sde, sdu, sdne, sdeu, sdun, 0.0, 0.0,
            state.vel_e[0], state.vel_e[1], state.vel_e[2],
            sdvx, sdvy, sdvz, sdvxy, sdvyz, sdvzx,
            att_deg[0], att_deg[1], att_deg[2],
            sdroll, sdpitch, sdyaw,
            lever[0], lever[1], lever[2],
            sdl[0], sdl[1], sdl[2],
        ))

    def write_gnss_only(self, timestamp: float, pos_e: np.ndarray,
                        q: int, num_sv: int, pos_sd: np.ndarray = None) -> None:
        """未初始化时输出纯 GNSS 解 (Qins=0, 速度=0, 姿态=0)。

        Args:
            timestamp: Unix 时间戳 (s)
            pos_e: ECEF 位置 [3] (m)
            q: GNSS quality (1/2/4/5)
            num_sv: 卫星数
            pos_sd: 位置标准差 [3] ECEF (m), 默认 10m
        """
        if self._fp is None:
            raise RuntimeError("RSLTWriter not opened")
        if pos_sd is None:
            pos_sd = np.array([10.0, 10.0, 10.0])

        week, sow = unix_to_gpst(timestamp)
        D2R = np.pi / 180.0

        llh = ecef2llh(pos_e)
        lat_deg = llh[0] / D2R
        lon_deg = llh[1] / D2R
        h = llh[2]

        # 位置 sd: ECEF → ENU
        R = ecef2enu_matrix(llh)
        Pp = np.diag(pos_sd ** 2)
        cov_enu = R @ Pp @ R.T
        sdn = float(np.sqrt(abs(cov_enu[1, 1])))
        sde = float(np.sqrt(abs(cov_enu[0, 0])))
        sdu = float(np.sqrt(abs(cov_enu[2, 2])))
        sdne = _signed_sqrt(float(cov_enu[0, 1]))
        sdeu = _signed_sqrt(float(cov_enu[2, 0]))
        sdun = _signed_sqrt(float(cov_enu[1, 2]))

        fmt = (
            "%4d %10.3f %14.9f %14.9f %10.4f %3d %3d %3d"
            " %9.4f %9.4f %9.4f %9.4f %9.4f %9.4f %6.2f %6.1f"
            " %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f"
            " %10.4f %10.4f %10.4f %10.4f %10.4f %10.4f"
            " %10.5f %10.5f %10.5f %9.5f %9.5f %9.5f\n"
        )
        # Qins=0, 速度=0, 姿态=0, sd=0, 杆臂=0 (未初始化)
        self._fp.write(fmt % (
            week, sow,
            lat_deg, lon_deg, h,
            q, 0, num_sv,   # Qins=0 (未初始化)
            sdn, sde, sdu, sdne, sdeu, sdun, 0.0, 0.0,
            0.0, 0.0, 0.0,   # 速度=0
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0,  # 速度 sd=0
            0.0, 0.0, 0.0,   # 姿态=0
            0.0, 0.0, 0.0,   # 姿态 sd=0
            0.0, 0.0, 0.0,   # 杆臂=0
            0.0, 0.0, 0.0,   # 杆臂 sd=0
        ))

    def close(self) -> None:
        if self._fp is not None and not self._closed:
            self._fp.close()
            self._closed = True
