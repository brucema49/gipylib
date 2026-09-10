"""Sensors and parsers for the Awesome GINS dataset."""
from queue import Queue

import numpy as np

from src.core.data_types import GnssSolution, ImuMeasurement, SensorData
from src.core.thread_control import ThreadControl
from src.core.time_utils import gpst_to_unix
from src.stream.base import StreamerBase


_WGS84_A = 6378137.0
_WGS84_F = 1.0 / 298.257223563
_WGS84_E2 = _WGS84_F * (2.0 - _WGS84_F)


def _llh_to_ecef(lat_deg: float, lon_deg: float, height_m: float) -> np.ndarray:
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    sin_lat = np.sin(lat)
    cos_lat = np.cos(lat)
    n = _WGS84_A / np.sqrt(1.0 - _WGS84_E2 * sin_lat * sin_lat)
    return np.array([
        (n + height_m) * cos_lat * np.cos(lon),
        (n + height_m) * cos_lat * np.sin(lon),
        (n * (1.0 - _WGS84_E2) + height_m) * sin_lat,
    ], dtype=np.float64)


def _enu_to_ecef(lat_deg: float, lon_deg: float) -> np.ndarray:
    lat = np.radians(lat_deg)
    lon = np.radians(lon_deg)
    sin_lat, cos_lat = np.sin(lat), np.cos(lat)
    sin_lon, cos_lon = np.sin(lon), np.cos(lon)
    # Columns are the ECEF unit vectors for E, N and U.
    return np.array([
        [-sin_lon, -sin_lat * cos_lon, cos_lat * cos_lon],
        [cos_lon, -sin_lat * sin_lon, cos_lat * sin_lon],
        [0.0, cos_lat, sin_lat],
    ], dtype=np.float64)


class AwesomeImuFormator:
    """Decode ``sow dtheta(3) dvel(3)`` into rate-form IMU samples."""

    def __init__(self, week: int = 0, start_sow: float | None = None,
                 end_sow: float | None = None):
        self.week = int(week)
        self.start_sow = start_sow
        self.end_sow = end_sow
        self._previous_sow = None

    def decode(self, line: str):
        line = line.strip()
        if not line or line.startswith("#"):
            return None
        parts = line.split()
        if len(parts) < 7:
            return None
        try:
            values = [float(value) for value in parts[:7]]
        except ValueError:
            return None

        sow = values[0]
        dtheta = np.asarray(values[1:4], dtype=np.float64)
        dvel = np.asarray(values[4:7], dtype=np.float64)
        if self._previous_sow is None:
            self._previous_sow = sow
            return None
        previous_sow = self._previous_sow
        dt = sow - previous_sow
        self._previous_sow = sow
        if not np.isfinite(dt) or dt <= 0.0:
            raise ValueError(f"Awesome IMU timestamps must increase, dt={dt}")
        if ((self.start_sow is not None and sow < self.start_sow)
                or (self.end_sow is not None and sow > self.end_sow)):
            return None
        # The rest of the pipeline advances on Unix-float timestamps.  At a
        # GPS week near 1.5e9 Unix seconds, subtracting two floats introduces
        # microsecond-scale quantization.  Use that same timestamp interval
        # for rate conversion so rate * pipeline_dt reconstructs the original
        # increment exactly, matching KF-GINS' increment-domain propagation.
        timestamp = gpst_to_unix(self.week, sow)
        previous_timestamp = gpst_to_unix(self.week, previous_sow)
        timestamp_dt = timestamp - previous_timestamp
        if not np.isfinite(timestamp_dt) or timestamp_dt <= 0.0:
            raise ValueError(f"Awesome IMU Unix timestamps must increase, dt={timestamp_dt}")
        imu = ImuMeasurement(
            timestamp=timestamp,
            week=self.week,
            accel=dvel / timestamp_dt,
            gyro=dtheta / timestamp_dt,
        )
        return SensorData(tag="imu", imu=imu)


class AwesomeGnssFormator:
    """Decode Awesome's SOW/LLH/ENU-standard-deviation RTK rows."""

    def __init__(self, week: int = 0, start_sow: float | None = None,
                 end_sow: float | None = None):
        self.week = int(week)
        self.start_sow = start_sow
        self.end_sow = end_sow

    def decode(self, line: str):
        line = line.strip()
        if not line or line.startswith("#"):
            return None
        parts = line.split()
        if len(parts) < 7:
            return None
        try:
            sow, lat, lon, height = (float(value) for value in parts[:4])
            sd_n, sd_e, sd_u = (float(value) for value in parts[4:7])
        except ValueError:
            return None
        if ((self.start_sow is not None and sow < self.start_sow)
                or (self.end_sow is not None and sow > self.end_sow)):
            return None
        if not np.all(np.isfinite([sow, lat, lon, height, sd_n, sd_e, sd_u])):
            return None
        if min(sd_n, sd_e, sd_u) < 0.0:
            raise ValueError("Awesome GNSS standard deviations must be non-negative")
        position = _llh_to_ecef(lat, lon, height)
        ecef_from_enu = _enu_to_ecef(lat, lon)
        cov_enu = np.diag([sd_e * sd_e, sd_n * sd_n, sd_u * sd_u])
        cov = ecef_from_enu @ cov_enu @ ecef_from_enu.T
        cov = 0.5 * (cov + cov.T)
        gnss = GnssSolution(
            timestamp=gpst_to_unix(self.week, sow),
            week=self.week,
            position=position,
            quality=1,
            num_sv=0,
            sd=np.sqrt(np.maximum(np.diag(cov), 0.0)),
            cov=cov,
        )
        return SensorData(tag="gnss_solution", gnss_solution=gnss)


class AwesomeImuSensor(StreamerBase):
    """Stream the Awesome incremental IMU file."""

    def __init__(self, file_path: str, output_queue: Queue,
                 control: ThreadControl, week: int = 0,
                 start_sow: float | None = None, end_sow: float | None = None):
        super().__init__(file_path, AwesomeImuFormator(week, start_sow, end_sow), output_queue,
                         control, "awesome_imu")


class AwesomeGnssSensor(StreamerBase):
    """Stream the Awesome external RTK solution file."""

    def __init__(self, file_path: str, output_queue: Queue,
                 control: ThreadControl, week: int = 0,
                 start_sow: float | None = None, end_sow: float | None = None):
        super().__init__(file_path, AwesomeGnssFormator(week, start_sow, end_sow), output_queue,
                         control, "awesome_gnss")
