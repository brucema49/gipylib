"""Opt-in initialization/propagation trace for TC cross-implementation audits.

The trace is intentionally a separate sink from the per-GNSS measurement
trace.  It records the state already committed by the estimator and never
participates in initialization, mechanization, or measurement acceptance.
"""

from __future__ import annotations

import csv
import json
import logging
from pathlib import Path

import numpy as np

from src.core.ins.attitude import dcm2quat, euler2dcm
from src.core.ins.earth_param import cal_Ce2n, cal_Cn2e, ecef2llh, llh2ecef
from src.core.time_utils import unix_to_gpst


logger = logging.getLogger(__name__)


SCHEMA = "GREAT_GIPY_INIT_PROPAGATION_TRACE_V1"
INITIALIZATION_INPUT_SCHEMA = "GREAT_GIPY_INITIALIZATION_INPUT_TRACE_V1"
DEFAULT_FILENAMES = {
    "jsonl": "tc-init-propagation-trace.jsonl",
    "csv": "tc-init-propagation-trace.csv",
}
INITIALIZATION_INPUT_DEFAULT_FILENAMES = {
    "jsonl": "tc-initialization-input-trace.jsonl",
    "csv": "tc-initialization-input-trace.csv",
}


def _vector(value, size):
    try:
        result = np.asarray(value, dtype=float).reshape(-1)
        if result.size != size or not np.all(np.isfinite(result)):
            return None
        return [float(item) for item in result]
    except Exception:
        return None


def _matrix(value):
    try:
        result = np.asarray(value, dtype=float)
        if result.shape != (3, 3) or not np.all(np.isfinite(result)):
            return None
        return [[float(item) for item in row] for row in result]
    except Exception:
        return None


def build_trace_record(event: str, state, *, sow: float | None = None,
                       dt: float | None = None,
                       gnss_free: bool = False,
                       isolation: str | None = None,
                       first_gnss_sow: float | None = None,
                       input_imu_form: str | None = None,
                       propagation_form: str | None = None,
                       imu_prev_sow: float | None = None,
                       imu_curr_sow: float | None = None,
                       imu_sample_count: int = 0,
                       gnss_measurement_inserted: int = 0,
                       main_run_gnss_update_seen: int = 0,
                       state_source: str | None = None) -> dict:
    """Build one schema-complete state record from an existing state object.

    ``sow`` is accepted separately so callers that retain source GPST can
    preserve it exactly instead of reconstructing it from a large Unix
    timestamp.  Missing optional attitude/lever fields remain explicit nulls;
    no alternate convention is inferred.
    """
    timestamp = None
    state_sow = None
    week = None
    if state is not None:
        try:
            timestamp = float(state.timestamp)
            week, state_sow = unix_to_gpst(timestamp)
            week = int(week)
            state_sow = float(state_sow)
        except Exception:
            timestamp = None
            state_sow = None

    pos = _vector(getattr(state, "pos_e", None), 3)
    vel = _vector(getattr(state, "vel_e", None), 3)
    C_b_e = _matrix(getattr(state, "C_b_e", None))
    quaternion = _vector(getattr(state, "q_b_e", None), 4)
    lever = _vector(
        getattr(state, "leverarm",
                getattr(state, "lever_body", getattr(state, "lever", None))),
        3,
    )
    antenna = None
    if pos is not None and C_b_e is not None and lever is not None:
        antenna = [
            float(pos[i] + sum(C_b_e[i][j] * lever[j] for j in range(3)))
            for i in range(3)
        ]

    exact_sow = None if sow is None else float(sow)
    record = {
        "schema": SCHEMA,
        "event_seq": None,
        "event": str(event),
        "stage": str(event),
        "timestamp": timestamp,
        "gps_week": week,
        "week": week,
        "sow": exact_sow if exact_sow is not None else state_sow,
        "source_sow": exact_sow if exact_sow is not None else state_sow,
        "exact_sow": exact_sow if exact_sow is not None else state_sow,
        "state_sow": state_sow,
        "timestamp_unix_s": timestamp,
        "gnss_free": bool(gnss_free),
        "isolation": isolation,
        "gnss_isolation": isolation,
        "first_gnss_sow": (None if first_gnss_sow is None
                            else float(first_gnss_sow)),
        "dt": None if dt is None else float(dt),
        "input_imu_form": input_imu_form,
        "propagation_form": propagation_form,
        "imu_prev_sow": (None if imu_prev_sow is None
                          else float(imu_prev_sow)),
        "imu_curr_sow": (None if imu_curr_sow is None
                          else float(imu_curr_sow)),
        "imu_sample_count": int(imu_sample_count),
        "gnss_measurement_inserted": int(gnss_measurement_inserted),
        "main_run_gnss_update_seen": int(main_run_gnss_update_seen),
        "state_source": state_source,
        "pos_ecef": pos,
        "pos_ecef_x": None if pos is None else pos[0],
        "pos_ecef_y": None if pos is None else pos[1],
        "pos_ecef_z": None if pos is None else pos[2],
        "vel_ecef": vel,
        "vel_ecef_x": None if vel is None else vel[0],
        "vel_ecef_y": None if vel is None else vel[1],
        "vel_ecef_z": None if vel is None else vel[2],
        "antenna_ecef": antenna,
        "antenna_ecef_x": None if antenna is None else antenna[0],
        "antenna_ecef_y": None if antenna is None else antenna[1],
        "antenna_ecef_z": None if antenna is None else antenna[2],
        "antenna_pos_ecef": antenna,
        "C_b_e": C_b_e,
        "quaternion": quaternion,
        "attitude_repr": "C_body_to_ecef",
        "attitude_C_body_to_ecef": C_b_e,
        "lever": lever,
        "leverarm": lever,
        "lever_frame": "body_imu_to_gnss_m",
        "lever_units": "m",
        # Flat aliases use the names in the existing GREAT stage trace.  They
        # are redundant by design and make a row directly joinable by SOW.
        "ins_pos_ecef_x": None if pos is None else pos[0],
        "ins_pos_ecef_y": None if pos is None else pos[1],
        "ins_pos_ecef_z": None if pos is None else pos[2],
        "ins_vel_ecef_x": None if vel is None else vel[0],
        "ins_vel_ecef_y": None if vel is None else vel[1],
        "ins_vel_ecef_z": None if vel is None else vel[2],
        "antenna_pos_ecef_x": None if antenna is None else antenna[0],
        "antenna_pos_ecef_y": None if antenna is None else antenna[1],
        "antenna_pos_ecef_z": None if antenna is None else antenna[2],
        "attitude_q0": None if quaternion is None else quaternion[0],
        "attitude_q1": None if quaternion is None else quaternion[1],
        "attitude_q2": None if quaternion is None else quaternion[2],
        "attitude_q3": None if quaternion is None else quaternion[3],
        "lever_x": None if lever is None else lever[0],
        "lever_y": None if lever is None else lever[1],
        "lever_z": None if lever is None else lever[2],
    }
    if C_b_e is not None:
        for row in range(3):
            for col in range(3):
                record[f"C_b_e_{row}{col}"] = C_b_e[row][col]
    else:
        for row in range(3):
            for col in range(3):
                record[f"C_b_e_{row}{col}"] = None
    if quaternion is None:
        for index in range(4):
            record[f"quaternion_q{index}"] = None
    else:
        for index, value in enumerate(quaternion):
            record[f"quaternion_q{index}"] = value
    return record


def _initialization_input_vector(value, size=3):
    """Return a finite JSON vector, or an honest null for unavailable input."""
    try:
        result = np.asarray(value, dtype=np.float64).reshape(-1)
        if result.size != size or not np.all(np.isfinite(result)):
            return None
        return [float(item) for item in result]
    except Exception:
        return None


def _initialization_input_matrix(value):
    """Return a finite 3x3 JSON matrix, or null when it is unavailable."""
    try:
        result = np.asarray(value, dtype=np.float64)
        if result.shape != (3, 3) or not np.all(np.isfinite(result)):
            return None
        return [[float(item) for item in row] for row in result]
    except Exception:
        return None


def _enu_to_ecef(lat: float, lon: float) -> np.ndarray:
    """Build the canonical ENU -> ECEF rotation from the existing NED basis."""
    C_e_n = cal_Ce2n(lat, lon)
    # ENU rows are E, N, Up; NED rows are N, E, Down.
    C_e_enu = np.array([C_e_n[1], C_e_n[0], -C_e_n[2]], dtype=np.float64)
    return C_e_enu.T


def _navigation_to_ecef(frame: str, lat: float, lon: float) -> np.ndarray | None:
    frame = str(frame or "").strip().upper()
    if frame == "NED":
        return cal_Cn2e(lat, lon)
    if frame == "ENU":
        return _enu_to_ecef(lat, lon)
    return None


def build_initialization_input_record(*, timestamp: float | None = None,
                                      source_sow: float | None = None,
                                      raw_position=None,
                                      raw_velocity=None,
                                      raw_attitude=None,
                                      raw_lever=None,
                                      state=None,
                                      position_frame: str = "ECEF",
                                      position_order: str | None = None,
                                      position_units: str = "m",
                                      velocity_frame: str = "ECEF",
                                      velocity_order: str | None = None,
                                      velocity_units: str = "m/s",
                                      attitude_frame: str = "NED",
                                      attitude_order: str = "roll,pitch,yaw",
                                      attitude_units: str = "rad",
                                      lever_frame: str = "FRD",
                                      lever_order: str = "front,right,down",
                                      lever_units: str = "m",
                                      lever_applied: bool = True,
                                      state_source: str | None = None,
                                      position_source: str | None = None,
                                      velocity_source: str | None = None,
                                      attitude_source: str | None = None) -> dict:
    """Build the common, observational initialization-input record.

    Raw values retain the convention in which they entered initialization;
    canonical values are independently derived as ECEF position/velocity and
    body-to-ECEF ``C_b_e``.  A supplied ``state`` is recorded as the actual
    converted state, while ``canonical_*`` fields preserve the transparent
    conversion result for comparison.  Unsupported or missing fields remain
    null and retain their definitions instead of being guessed.
    """
    raw_pos = _initialization_input_vector(raw_position)
    raw_vel = _initialization_input_vector(raw_velocity)
    raw_att = _initialization_input_vector(raw_attitude)
    raw_lev = _initialization_input_vector(raw_lever)
    position_frame = str(position_frame or "").strip().upper() or None
    velocity_frame = str(velocity_frame or "").strip().upper() or None
    attitude_frame = str(attitude_frame or "").strip().upper() or None
    lever_frame = str(lever_frame or "").strip().upper() or None

    pos_ecef = None
    if raw_pos is not None and position_frame == "ECEF":
        pos_ecef = raw_pos
    elif raw_pos is not None and position_frame == "LLH":
        try:
            pos_ecef = llh2ecef(
                np.radians(raw_pos[0]), np.radians(raw_pos[1]), raw_pos[2]
            ).tolist()
        except Exception:
            pos_ecef = None

    lat = lon = None
    if pos_ecef is not None:
        try:
            lat, lon, _height = ecef2llh(np.asarray(pos_ecef, dtype=np.float64))
        except Exception:
            lat = lon = None

    vel_ecef = None
    if raw_vel is not None and velocity_frame == "ECEF":
        vel_ecef = raw_vel
    elif raw_vel is not None and lat is not None and velocity_frame in {"NED", "ENU"}:
        try:
            vel_ecef = (_navigation_to_ecef(velocity_frame, lat, lon)
                        @ np.asarray(raw_vel, dtype=np.float64)).tolist()
        except Exception:
            vel_ecef = None

    C_b_e = None
    if raw_att is not None and lat is not None and attitude_frame in {"NED", "ENU"}:
        try:
            attitude = np.asarray(raw_att, dtype=np.float64)
            if str(attitude_units or "").strip().lower() in {"deg", "degree", "degrees"}:
                attitude = np.radians(attitude)
            C_b_n = euler2dcm(attitude)
            C_n_e = _navigation_to_ecef(attitude_frame, lat, lon)
            C_b_e = (C_n_e @ C_b_n).tolist()
        except Exception:
            C_b_e = None

    lever_frd = None
    if raw_lev is not None and lever_frame == "FRD":
        lever_frd = raw_lev
    elif raw_lev is not None and lever_frame == "RFU":
        # Keep the existing reader convention explicit; this is diagnostic
        # conversion only and does not alter the sensor or estimator path.
        lever_frd = [raw_lev[1], raw_lev[0], -raw_lev[2]]
    lever_ecef = None
    if lever_frd is not None and C_b_e is not None:
        lever_ecef = (np.asarray(C_b_e) @ np.asarray(lever_frd)).tolist()
    canonical_pos = None
    if pos_ecef is not None:
        canonical_pos = list(pos_ecef)
        if lever_applied and lever_ecef is not None:
            canonical_pos = (np.asarray(canonical_pos) - np.asarray(lever_ecef)).tolist()
    canonical_q = None if C_b_e is None else dcm2quat(np.asarray(C_b_e)).tolist()

    actual_pos = _initialization_input_vector(getattr(state, "pos_e", None)) if state is not None else canonical_pos
    actual_vel = _initialization_input_vector(getattr(state, "vel_e", None)) if state is not None else vel_ecef
    actual_C = _initialization_input_matrix(getattr(state, "C_b_e", None)) if state is not None else _initialization_input_matrix(C_b_e)
    actual_q = _initialization_input_vector(getattr(state, "q_b_e", None), 4) if state is not None else _initialization_input_vector(canonical_q, 4)
    if source_sow is None and timestamp is not None:
        try:
            _week, source_sow = unix_to_gpst(float(timestamp))
        except Exception:
            source_sow = None
    exact_sow = None if source_sow is None else float(source_sow)
    week = None
    state_sow = None
    if timestamp is not None:
        try:
            week, state_sow = unix_to_gpst(float(timestamp))
            week = int(week)
            state_sow = float(state_sow)
        except Exception:
            pass

    if position_order is None:
        position_order = "latitude_deg,longitude_deg,height_m" if position_frame == "LLH" else "x,y,z"
    if velocity_order is None:
        velocity_order = {
            "NED": "north,east,down",
            "ENU": "east,north,up",
        }.get(velocity_frame, "x,y,z")
    position_definition = {
        "ECEF": "raw [x,y,z] in Earth-fixed Cartesian coordinates",
        "LLH": "raw [latitude_deg,longitude_deg,height_m] converted with WGS84",
    }.get(position_frame, "raw position frame unavailable or unsupported")
    velocity_definition = {
        "ECEF": "raw [x,y,z] velocity in ECEF",
        "NED": "raw [north,east,down] velocity; converted with C_n^e",
        "ENU": "raw [east,north,up] velocity; converted with C_enu^e",
    }.get(velocity_frame, "raw velocity frame unavailable or unsupported")
    attitude_definition = (
        "raw roll,pitch,yaw in navigation frame; ZYX yaw-pitch-roll; "
        "C_b^e = C_nav^e @ C_b^nav"
    )
    lever_definition = (
        "raw IMU-to-GNSS lever in FRD [front,right,down] meters; "
        "lever_ecef = C_b^e @ lever_frd"
    )
    record = {
        "schema": INITIALIZATION_INPUT_SCHEMA,
        "event_seq": None,
        "event": "initialization_input",
        "stage": "initialization_input",
        "timestamp": None if timestamp is None else float(timestamp),
        "gps_week": week,
        "week": week,
        "sow": exact_sow if exact_sow is not None else state_sow,
        "source_sow": exact_sow,
        "exact_sow": exact_sow,
        "state_sow": state_sow,
        "timestamp_unix_s": None if timestamp is None else float(timestamp),
        "state_source": state_source,
        "position_source": position_source,
        "velocity_source": velocity_source,
        "attitude_source": attitude_source,
        "raw_position": raw_pos,
        "raw_position_frame": position_frame,
        "raw_position_order": position_order,
        "raw_position_units": position_units,
        "raw_position_definition": position_definition,
        "position_definition": position_definition,
        "raw_velocity": raw_vel,
        "raw_velocity_frame": velocity_frame,
        "raw_velocity_order": velocity_order,
        "raw_velocity_units": velocity_units,
        "raw_velocity_definition": velocity_definition,
        "raw_attitude": raw_att,
        "raw_attitude_frame": attitude_frame,
        "raw_attitude_order": attitude_order,
        "raw_attitude_units": attitude_units,
        "raw_attitude_definition": attitude_definition,
        "attitude_definition": attitude_definition,
        "raw_lever": raw_lev,
        "raw_lever_frame": lever_frame,
        "raw_lever_order": lever_order,
        "raw_lever_units": lever_units,
        "raw_lever_definition": lever_definition,
        "lever_applied": bool(lever_applied),
        "lever_frd": lever_frd,
        "lever_ecef": lever_ecef,
        "canonical_position_ecef": canonical_pos,
        "canonical_pos_ecef": canonical_pos,
        "canonical_velocity_ecef": vel_ecef,
        "canonical_vel_ecef": vel_ecef,
        "canonical_C_b_e": _initialization_input_matrix(C_b_e),
        "canonical_quaternion_b_e": canonical_q,
        "canonical_quaternion": canonical_q,
        "converted_position_ecef": actual_pos,
        "converted_pos_ecef": actual_pos,
        "converted_velocity_ecef": actual_vel,
        "converted_vel_ecef": actual_vel,
        "converted_C_b_e": actual_C,
        "converted_quaternion_b_e": actual_q,
        "converted_quaternion": actual_q,
        "converted_position_definition": (
            "ECEF IMU reference position after optional antenna lever subtraction"
        ),
        "converted_velocity_definition": "ECEF velocity in [x,y,z] m/s",
        "converted_attitude_definition": "C_b^e body/FRD to ECEF",
        "converted_quaternion_definition": "[w,x,y,z] equivalent to converted C_b_e",
    }
    # Flat aliases make the row joinable with the existing GREAT stage trace.
    for prefix, value in (("raw_position", raw_pos),
                          ("raw_velocity", raw_vel),
                          ("raw_attitude", raw_att),
                          ("raw_lever", raw_lev),
                          ("converted_position_ecef", actual_pos),
                          ("converted_velocity_ecef", actual_vel),
                          ("lever_ecef", lever_ecef)):
        if value is not None:
            for index, item in enumerate(value):
                record[f"{prefix}_{index}"] = item
        else:
            size = 4 if "quaternion" in prefix else 3
            for index in range(size):
                record[f"{prefix}_{index}"] = None
    for prefix, value in (("raw_position", raw_pos),
                          ("raw_velocity", raw_vel),
                          ("raw_attitude", raw_att),
                          ("raw_lever", raw_lev)):
        record[prefix.removeprefix("raw_")] = value
    for prefix, value in (("canonical_pos_ecef", canonical_pos),
                          ("canonical_vel_ecef", vel_ecef)):
        if value is not None:
            for index, item in enumerate(value):
                record[f"{prefix}_{index}"] = item
        else:
            for index in range(3):
                record[f"{prefix}_{index}"] = None
    for prefix, value in (("canonical_pos_ecef", canonical_pos),
                          ("canonical_vel_ecef", vel_ecef),
                          ("converted_pos_ecef", actual_pos),
                          ("converted_vel_ecef", actual_vel)):
        axis_names = ("x", "y", "z")
        for axis, item in zip(axis_names, value or (None, None, None)):
            record[f"{prefix}_{axis}"] = item
    for row in range(3):
        for col in range(3):
            record[f"converted_C_b_e_{row}{col}"] = (
                None if actual_C is None else actual_C[row][col]
            )
            record[f"canonical_C_b_e_{row}{col}"] = (
                None if C_b_e is None else C_b_e[row][col]
            )
    for index in range(4):
        record[f"converted_quaternion_q{index}"] = (
            None if actual_q is None else actual_q[index]
        )
        record[f"canonical_quaternion_q{index}"] = (
            None if canonical_q is None else canonical_q[index]
        )
    return record


class TcInitPropagationTraceWriter:
    """Persist initialization/propagation records only when explicitly enabled."""

    SCHEMA = SCHEMA
    DEFAULT_FILENAMES = DEFAULT_FILENAMES
    CONFIG_KEY = "init_propagation_trace"

    def __init__(self, config: dict | str | Path | None = None, *,
                 output_dir=None, filename=None, format=None, enabled=None):
        if isinstance(config, dict):
            output = config.get("output", {})
            tc = config.get("tc", {})
            trace_cfg = tc.get(self.CONFIG_KEY, {})
        else:
            output = {}
            trace_cfg = {}
            if config is not None:
                output_dir = config
        if not isinstance(output, dict):
            raise ValueError("output must be a mapping")
        if trace_cfg is None:
            trace_cfg = {}
        if not isinstance(trace_cfg, dict):
            raise ValueError(f"tc.{self.CONFIG_KEY} must be a mapping")
        if output_dir is None:
            output_dir = output.get("output_dir", "output")
        if enabled is None:
            enabled = trace_cfg.get("enabled", False)
        if isinstance(enabled, str):
            value = enabled.strip().lower()
            if value in {"true", "1", "yes", "on"}:
                enabled = True
            elif value in {"false", "0", "no", "off"}:
                enabled = False
            else:
                raise ValueError(
                    f"tc.{self.CONFIG_KEY}.enabled must be boolean")
        self.enabled = bool(enabled)
        if format is None:
            format = trace_cfg.get("format", "jsonl")
        self.format = str(format).strip().lower()
        if self.format not in self.DEFAULT_FILENAMES:
            raise ValueError(
                f"tc.{self.CONFIG_KEY}.format must be 'jsonl' or 'csv'")
        if filename is None:
            filename = trace_cfg.get("filename",
                                     self.DEFAULT_FILENAMES[self.format])
        self.filename = str(filename)
        self._validate_basename(self.filename)
        self.output_dir = Path(str(output_dir))
        self.path = self.output_dir / self.filename
        self._fp = None
        self._csv_writer = None
        self._csv_fields = None
        self._event_seq = 0
        self._failed = False

    @classmethod
    def _validate_basename(cls, filename: str) -> None:
        if (not filename or filename in (".", "..") or
                Path(filename).name != filename or "/" in filename or
                "\\" in filename or Path(filename).is_absolute()):
            raise ValueError(
                f"tc.{cls.CONFIG_KEY}.filename must be a basename")

    def open(self) -> None:
        if not self.enabled or self._failed:
            return
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self._fp = self.path.open("w", encoding="utf-8", newline="")
            self._event_seq = 0
            self._csv_writer = None
            self._csv_fields = None
        except Exception as exc:
            self._disable(exc, "open")

    def close(self) -> None:
        fp = self._fp
        if fp is None:
            return
        try:
            fp.close()
        except Exception as exc:
            self._disable(exc, "close")
        else:
            self._fp = None
            self._csv_writer = None
            self._csv_fields = None

    def write_initialization_complete(self, state, *, sow=None,
                                      isolation=(
                                          "gnss_assisted_initialization;"
                                          "not_gnss_free"),
                                      first_gnss_sow=None) -> None:
        self.write(build_trace_record(
            "initialization_complete", state, sow=sow,
            gnss_free=False, isolation=isolation,
            first_gnss_sow=first_gnss_sow,
            state_source="main_run_gnss_assisted_initialization;not_gnss_free",
        ))

    def write_imu_propagation_pre_gnss(self, state, *, sow=None, dt=None,
                                       isolation=(
                                           "pre_first_gnss_update;"
                                           "not_gnss_free"),
                                       first_gnss_sow=None,
                                       input_imu_form=None,
                                       propagation_form=None,
                                       imu_prev_sow=None,
                                       imu_curr_sow=None,
                                       imu_sample_count=1,
                                       gnss_measurement_inserted=0,
                                       main_run_gnss_update_seen=0,
                                       state_source=(
                                           "main_run_gnss_assisted;"
                                           "pre_first_gnss_update;not_gnss_free")) -> None:
        self.write(build_trace_record(
            "imu_propagation_pre_gnss", state, sow=sow, dt=dt,
            gnss_free=False, isolation=isolation,
            first_gnss_sow=first_gnss_sow,
            input_imu_form=input_imu_form,
            propagation_form=propagation_form,
            imu_prev_sow=imu_prev_sow, imu_curr_sow=imu_curr_sow,
            imu_sample_count=imu_sample_count,
            gnss_measurement_inserted=gnss_measurement_inserted,
            main_run_gnss_update_seen=main_run_gnss_update_seen,
            state_source=state_source,
        ))

    def write_initialization_input(self, **kwargs) -> None:
        """Write one common initialization-input record when enabled."""
        self.write(build_initialization_input_record(**kwargs))

    def write(self, record: dict) -> None:
        if not self.enabled or self._fp is None or self._failed:
            return
        try:
            if not isinstance(record, dict):
                raise TypeError("trace records must be mappings")
            event = {str(key): self._normalize(value)
                     for key, value in record.items()}
            event.setdefault("schema", self.SCHEMA)
            event["event_seq"] = self._event_seq
            self._event_seq += 1
            if self.format == "jsonl":
                self._fp.write(self._canonical_json(event))
                self._fp.write("\n")
            else:
                self._write_csv(event)
            self._fp.flush()
        except Exception as exc:
            self._disable(exc, "write")

    @classmethod
    def _normalize(cls, value):
        if isinstance(value, dict):
            return {str(key): cls._normalize(item)
                    for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._normalize(item) for item in value]
        if hasattr(value, "tolist"):
            return cls._normalize(value.tolist())
        if value is None or isinstance(value, (str, int, float, bool)):
            return value
        return str(value)

    @staticmethod
    def _canonical_json(value) -> str:
        return json.dumps(value, ensure_ascii=False, sort_keys=True,
                          separators=(",", ":"), allow_nan=False)

    def _write_csv(self, event: dict) -> None:
        fields = tuple(event.keys())
        if self._csv_writer is None:
            self._csv_fields = fields
            self._csv_writer = csv.DictWriter(
                self._fp, fieldnames=fields, extrasaction="ignore")
            self._csv_writer.writeheader()
        elif tuple(self._csv_fields) != fields:
            raise ValueError("TC init propagation trace CSV schema changed")
        row = {}
        for field in self._csv_fields:
            value = event.get(field)
            row[field] = (self._canonical_json(value)
                          if isinstance(value, (dict, list)) else
                          "" if value is None else str(value))
        self._csv_writer.writerow(row)

    def _disable(self, exc: Exception, operation: str) -> None:
        self._failed = True
        fp = self._fp
        self._fp = None
        self._csv_writer = None
        self._csv_fields = None
        if fp is not None:
            try:
                fp.close()
            except Exception:
                pass
        logger.warning("TC init propagation trace %s failed; sink disabled: %s",
                       operation, exc)


class TcInitializationInputTraceWriter(TcInitPropagationTraceWriter):
    """Persist only the common initialization-input audit event."""

    SCHEMA = INITIALIZATION_INPUT_SCHEMA
    CONFIG_KEY = "initialization_input_trace"
    DEFAULT_FILENAMES = INITIALIZATION_INPUT_DEFAULT_FILENAMES
