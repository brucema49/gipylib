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

from src.core.time_utils import unix_to_gpst


logger = logging.getLogger(__name__)


SCHEMA = "GREAT_GIPY_INIT_PROPAGATION_TRACE_V1"
DEFAULT_FILENAMES = {
    "jsonl": "tc-init-propagation-trace.jsonl",
    "csv": "tc-init-propagation-trace.csv",
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


class TcInitPropagationTraceWriter:
    """Persist initialization/propagation records only when explicitly enabled."""

    SCHEMA = SCHEMA
    DEFAULT_FILENAMES = DEFAULT_FILENAMES

    def __init__(self, config: dict | str | Path | None = None, *,
                 output_dir=None, filename=None, format=None, enabled=None):
        if isinstance(config, dict):
            output = config.get("output", {})
            tc = config.get("tc", {})
            trace_cfg = tc.get("init_propagation_trace", {})
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
            raise ValueError("tc.init_propagation_trace must be a mapping")
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
                    "tc.init_propagation_trace.enabled must be boolean")
        self.enabled = bool(enabled)
        if format is None:
            format = trace_cfg.get("format", "jsonl")
        self.format = str(format).strip().lower()
        if self.format not in self.DEFAULT_FILENAMES:
            raise ValueError(
                "tc.init_propagation_trace.format must be 'jsonl' or 'csv'")
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

    @staticmethod
    def _validate_basename(filename: str) -> None:
        if (not filename or filename in (".", "..") or
                Path(filename).name != filename or "/" in filename or
                "\\" in filename or Path(filename).is_absolute()):
            raise ValueError(
                "tc.init_propagation_trace.filename must be a basename")

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
