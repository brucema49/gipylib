"""rtklib 风格 .pos 输出器。"""
import os
from pathlib import Path

import numpy as np

from src.core.data_types import GnssSolution
from src.core.time_utils import unix_to_gpst, sow_to_ymdhms
from src.log.writer_base import WriterBase


_WGS84_A = 6378137.0
_WGS84_F = 1 / 298.257223563
_WGS84_B = _WGS84_A * (1 - _WGS84_F)
_WGS84_E2 = _WGS84_F * (2 - _WGS84_F)


def ecef2llh(ecef: np.ndarray) -> np.ndarray:
    """ECEF [x,y,z] → [lat_rad, lon_rad, h]。"""
    x, y, z = ecef[0], ecef[1], ecef[2]
    lon = np.arctan2(y, x)
    p = np.sqrt(x * x + y * y)
    h = np.sqrt(p * p + z * z) - _WGS84_A
    lat = np.arctan2(z, p * (1 - _WGS84_E2))
    for _ in range(6):
        sinlat = np.sin(lat)
        N = _WGS84_A / np.sqrt(1 - _WGS84_E2 * sinlat * sinlat)
        h = p / np.cos(lat) - N
        lat = np.arctan2(z, p * (1 - _WGS84_E2 * N / (N + h)))
    return np.array([lat, lon, h])


def ecef2enu_matrix(llh: np.ndarray) -> np.ndarray:
    """LLH [lat_rad, lon_rad, h] → ECEF→ENU 旋转矩阵 3x3。"""
    lat, lon = llh[0], llh[1]
    sl, cl = np.sin(lat), np.cos(lat)
    so, co = np.sin(lon), np.cos(lon)
    return np.array([
        [-so,            co,           0],
        [-sl * co,      -sl * so,      cl],
        [cl * co,        cl * so,      sl],
    ])


class SolutionWriter(WriterBase):
    """rtklib 风格 .pos 输出器。

    输出格式可配置:
      - position_format: llh (经纬度+高, 默认) / xyz (ECEF 直角坐标)
      - time_format: gpst (GPS周+周内秒, 默认) / datetime (YYYY/MM/DD HH:MM:SS.sss)

    表头 + 每历元一行:
      gpst+llh:   week sow lat lon h Q ns sdn sde sdu sdne sdeu sdun age ratio
      gpst+xyz:   week sow x y z Q ns sdn sde sdu sdne sdeu sdun age ratio
      datetime+llh: YYYY/MM/DD HH:MM:SS.sss lat lon h Q ns sdn sde sdu sdne sdeu sdun age ratio
      datetime+xyz: YYYY/MM/DD HH:MM:SS.sss x y z Q ns sdn sde sdu sdne sdeu sdun age ratio

    ECEF → LLH（度）转换，sd ECEF → ENU。
    age/ratio 填 0（本项目 GnssSolution 无此字段）。
    """

    def __init__(self, output_dir: str, filename: str = "solution.pos",
                 position_format: str = "llh", time_format: str = "gpst"):
        self.output_dir = output_dir
        self.filename = filename
        self.position_format = position_format
        self.time_format = time_format
        self._fp = None
        self._closed = False

    def _build_header(self) -> str:
        if self.time_format == "datetime":
            time_col = "  GPST(datetime)             "
        else:
            time_col = "  GPST          "
        if self.position_format == "xyz":
            pos_cols = "   x(m)        y(m)        z(m)   "
        else:
            pos_cols = " latitude(deg) longitude(deg)  height(m)   "
        return (
            "%" + time_col + pos_cols +
            "Q  ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n"
        )

    def open(self) -> None:
        os.makedirs(self.output_dir, exist_ok=True)
        path = Path(self.output_dir) / self.filename
        self._fp = open(path, "w", encoding="utf-8")
        self._fp.write(self._build_header())
        self._closed = False

    def _format_time(self, timestamp: float):
        week, sow = unix_to_gpst(timestamp)
        if self.time_format == "datetime":
            y, mo, d, h, mi, s = sow_to_ymdhms(week, sow)
            return "%04d/%02d/%02d %02d:%02d:%06.3f" % (y, mo, d, h, mi, s)
        return "%4d %10.3f" % (week, sow)

    def write(self, sol: GnssSolution) -> None:
        if self._fp is None:
            raise RuntimeError("SolutionWriter not opened")

        llh = ecef2llh(sol.position)
        R = ecef2enu_matrix(llh)
        # 优先使用完整 3x3 ECEF 协方差（含非对角项），与 rtklib-py covenu 一致
        # 回退到 diag(sd**2) 仅对角线（兼容旧 GnssSolution 无 cov 字段的情况）
        if sol.cov is not None:
            cov_ecef = sol.cov
        else:
            cov_ecef = np.diag(sol.sd ** 2)
        cov_enu = R @ cov_ecef @ R.T
        # ENU 协方差矩阵: [0,0]=E, [1,1]=N, [2,2]=U (与 rtklib-py xyz2enu 一致)
        # 输出顺序对齐 rtklib-py postpos.savesol: sdn=N, sde=E, sdu=U, sdne=EN, sdeu=UE, sdun=NU
        sdn = np.sqrt(abs(cov_enu[1, 1]))
        sde = np.sqrt(abs(cov_enu[0, 0]))
        sdu = np.sqrt(abs(cov_enu[2, 2]))
        sdne = np.sqrt(abs(cov_enu[0, 1])) * np.sign(cov_enu[0, 1])
        sdeu = np.sqrt(abs(cov_enu[2, 0])) * np.sign(cov_enu[2, 0])
        sdun = np.sqrt(abs(cov_enu[1, 2])) * np.sign(cov_enu[1, 2])

        D2R = np.pi / 180.0
        time_str = self._format_time(sol.timestamp)

        if self.position_format == "xyz":
            pos_fmt = "%14.4f %14.4f %14.4f"
            pos_vals = (sol.position[0], sol.position[1], sol.position[2])
        else:
            pos_fmt = "%14.9f %14.9f %10.4f"
            pos_vals = (llh[0] / D2R, llh[1] / D2R, llh[2])

        line = "%s " + pos_fmt + " %3d %3d %8.4f  %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f\n"
        self._fp.write(line % (
            time_str,
            *pos_vals,
            sol.quality, sol.num_sv,
            sdn, sde, sdu, sdne, sdeu, sdun,
            0.0, 0.0,
        ))

    def close(self) -> None:
        if self._fp is not None and not self._closed:
            self._fp.close()
            self._closed = True
