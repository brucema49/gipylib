"""rtklib 风格 .pos 输出器。"""
import os
from pathlib import Path

import numpy as np

from src.core.data_types import GnssSolution
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

    输出格式（参考 rtklib-py postpos.savesol）:
      表头 + 每历元一行: week sow lat lon h Q ns sdn sde sdu sdne sdeu sdun age ratio
    ECEF → LLH（度）转换，sd ECEF → ENU。
    age/ratio 填 0（本项目 GnssSolution 无此字段）。
    """

    HEADER = (
        "%  GPST          latitude(deg) longitude(deg)  height(m)   Q  "
        "ns   sdn(m)   sde(m)   sdu(m)  sdne(m)  sdeu(m)  sdun(m) age(s)  ratio\n"
    )

    def __init__(self, output_dir: str, filename: str = "solution.pos"):
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

    def write(self, sol: GnssSolution) -> None:
        if self._fp is None:
            raise RuntimeError("SolutionWriter not opened")

        llh = ecef2llh(sol.position)
        R = ecef2enu_matrix(llh)
        cov_ecef = np.diag(sol.sd ** 2)
        cov_enu = R @ cov_ecef @ R.T
        sdn = np.sqrt(abs(cov_enu[0, 0]))
        sde = np.sqrt(abs(cov_enu[1, 1]))
        sdu = np.sqrt(abs(cov_enu[2, 2]))
        sdne = np.sqrt(abs(cov_enu[0, 1])) * np.sign(cov_enu[0, 1])
        sdeu = np.sqrt(abs(cov_enu[1, 2])) * np.sign(cov_enu[1, 2])
        sdun = np.sqrt(abs(cov_enu[2, 0])) * np.sign(cov_enu[2, 0])

        D2R = np.pi / 180.0
        fmt = (
            "%4d %10.3f %14.9f %14.9f %10.4f %3d %3d %8.4f"
            "  %8.4f %8.4f %8.4f %8.4f %8.4f %6.2f %6.1f\n"
        )
        self._fp.write(fmt % (
            sol.week, sol.timestamp,
            llh[0] / D2R, llh[1] / D2R, llh[2],
            sol.quality, sol.num_sv,
            sdn, sde, sdu, sdne, sdeu, sdun,
            0.0, 0.0,
        ))

    def close(self) -> None:
        if self._fp is not None and not self._closed:
            self._fp.close()
            self._closed = True
