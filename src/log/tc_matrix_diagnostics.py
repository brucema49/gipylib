"""Opt-in JSONL snapshots for the TC bias-comparison experiments."""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from src.core.time_utils import unix_to_gpst


class TcMatrixDiagnosticWriter:
    """Write bounded propagation and GNSS-update snapshots.

    The writer is deliberately independent of the filter.  It validates and
    copies the matrices at the boundary, so later mutations of ``P`` cannot
    alter an already emitted record.
    """

    SCHEMA = "GIPY_TC_MATRIX_DIAG_V1"
    MATRIX_ORDER = "pos,vel,att,gyro_bias,accel_bias"
    MATRIX_SHAPE = (15, 15)

    def __init__(self, path, mode: str = "tc"):
        self.path = Path(path) if path else None
        self.mode = str(mode)
        self._fp = None
        self._imu_count = 0

    def open(self) -> None:
        if self.path is None:
            return
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self._fp = self.path.open("w", encoding="utf-8")
        self._write({
            "schema": self.SCHEMA,
            "mode": self.mode,
            "time_system": "GPST+unix_timestamp",
            "units": {
                "timestamp": "unix_s",
                "dt": "s",
                "P": "SI covariance",
                "Phi": "dimensionless",
                "Q": "SI covariance",
            },
            "matrix_order": self.MATRIX_ORDER,
            "matrix_shape": list(self.MATRIX_SHAPE),
        })

    def close(self) -> None:
        if self._fp is not None:
            self._fp.close()
            self._fp = None

    def write_imu_prop(self, timestamp, dt, p_before, phi, q, p_after) -> None:
        if self._fp is None or self._imu_count >= 10:
            return
        self._imu_count += 1
        self._write(self._matrix_record(
            "imu_prop", timestamp, dt, p_before, phi, q, p_after))

    def write_update(self, timestamp, p_before, p_after, phi, q,
                     innovation, S_diag, K, feedback_x, accepted,
                     info: dict | None = None) -> None:
        if self._fp is None:
            return
        info = info or {}
        record = self._matrix_record(
            "tc_update", timestamp, info.get("dt", -1.0),
            p_before, phi, q, p_after)
        record.update({
            "accepted": bool(accepted),
            "innovation": self._array(innovation),
            "innovation_norm": float(np.linalg.norm(innovation)),
            "S_diag": self._array(S_diag),
            "K_block_norms": self._block_norms(K),
            "feedback_x": self._array(feedback_x),
            "postfit_norm": self._scalar(info.get("postfit_norm", np.nan)),
            "postfit_chi2": self._scalar(info.get("postfit_chi2", np.nan)),
            "n_phase_att": int(info.get("n_phase_att", 0)),
            "n_phase_acc": int(info.get("n_phase_acc", 0)),
            "n_phase_rej": int(info.get("n_phase_rej", 0)),
            "n_code_att": int(info.get("n_code_att", 0)),
            "n_code_acc": int(info.get("n_code_acc", 0)),
            "n_code_rej": int(info.get("n_code_rej", 0)),
            "ref_sats": [int(value) for value in info.get("ref_sats", [])],
            "pairs": [list(pair) for pair in info.get("pairs", [])],
        })
        self._write(record)

    def _matrix_record(self, event, timestamp, dt,
                       p_before, phi, q, p_after) -> dict:
        week, sow = unix_to_gpst(float(timestamp))
        return {
            "event": str(event),
            "mode": self.mode,
            "timestamp": float(timestamp),
            "week": int(week),
            "sow": float(sow),
            "dt": float(dt),
            "matrix_order": self.MATRIX_ORDER,
            "p_before": self._matrix(p_before),
            "p_after": self._matrix(p_after),
            "phi": self._matrix(phi),
            "q": self._matrix(q),
        }

    @classmethod
    def _matrix(cls, value) -> list[list[float | str]]:
        array = np.asarray(value, dtype=float)
        if array.shape != cls.MATRIX_SHAPE:
            raise ValueError(
                f"TC matrix must have shape {cls.MATRIX_SHAPE}, got {array.shape}")
        return [cls._array(row) for row in array]

    @staticmethod
    def _array(value) -> list[float | str]:
        array = np.asarray(value, dtype=float).reshape(-1)
        return [TcMatrixDiagnosticWriter._scalar(item) for item in array]

    @staticmethod
    def _scalar(value) -> float | str:
        value = float(value)
        if np.isfinite(value):
            return value
        if np.isnan(value):
            return "nan"
        return "inf" if value > 0.0 else "-inf"

    @staticmethod
    def _block_norms(K) -> dict[str, float | str]:
        array = np.asarray(K, dtype=float)
        blocks = {
            "pos": slice(0, 3),
            "vel": slice(3, 6),
            "att": slice(6, 9),
            "gyro_bias": slice(9, 12),
            "accel_bias": slice(12, 15),
        }
        return {
            name: TcMatrixDiagnosticWriter._scalar(
                np.linalg.norm(array[selection]))
            for name, selection in blocks.items()
        }

    def _write(self, record: dict) -> None:
        if self._fp is None:
            return
        self._fp.write(json.dumps(record, ensure_ascii=False,
                                   allow_nan=False, separators=(",", ":")))
        self._fp.write("\n")
