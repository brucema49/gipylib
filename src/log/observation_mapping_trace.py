"""Opt-in audit records for the RINEX observation mapping boundary.

The reader is deliberately kept independent from any solver state.  A trace
sink is supplied by a caller when an audit is wanted; with no sink/callback,
record construction is skipped at the call site and decoding stays on its
existing fast path.
"""

from __future__ import annotations

import hashlib
import json
from collections.abc import Mapping
from typing import Any, Callable


TRACE_FIELDS = (
    "epoch", "time", "sensor_id", "instance_id", "mapping_hash",
    "constellation", "input_obs_code", "raw_band", "mapping_owner",
    "simplified_obs_code", "decoder_obs_code", "slot", "disposition",
    "error_class", "error_code",
)
_DISPOSITIONS = frozenset(("decoded", "skip", "reject"))
_SYSTEM_ALIASES = {
    "G": "G", "GPS": "G",
    "R": "R", "GLO": "R", "GLONASS": "R",
    "E": "E", "GAL": "E", "GALILEO": "E",
    "C": "C", "BDS": "C", "BEIDOU": "C",
    "J": "J", "QZS": "J", "QZSS": "J",
    "S": "S", "SBS": "S", "SBAS": "S",
}


def mapping_hash(raw_band_priority: Mapping[str, Any]) -> str:
    """Return a stable digest of an instance's ordered raw-band mapping."""
    canonical = {}
    for system, bands in raw_band_priority.items():
        system_name = str(system).strip().upper()
        system_name = _SYSTEM_ALIASES.get(system_name, system_name)
        if isinstance(bands, Mapping):
            # Accept an already-derived ``raw_band -> slot`` view as a
            # convenience for reader callers.  Sorting by slot preserves the
            # instance's configured order in the digest.
            ordered = [band for band, _ in sorted(
                bands.items(), key=lambda item: item[1])]
        else:
            ordered = list(bands)
        canonical[system_name] = [int(band) for band in ordered]
    payload = json.dumps(
        canonical, sort_keys=True, separators=(",", ":"), ensure_ascii=True)
    return hashlib.sha256(payload.encode("ascii")).hexdigest()


class ObservationMappingError(SystemExit, ValueError):
    """Controlled reader-domain error retaining legacy ``SystemExit`` API."""

    def __init__(self, message: str, *, error_code: str,
                 error_class: str = "observation_mapping"):
        super().__init__(message)
        self.error_class = str(error_class)
        self.error_code = str(error_code)


class ObservationMappingTrace:
    """Emit one normalized mapping record to an explicit sink or callback.

    ``sink`` may implement ``write(record)`` or ``append(record)``.  A plain
    callback is also accepted.  Diagnostic failures are intentionally ignored
    so enabling an audit cannot perturb positioning or reader behavior.
    """

    def __init__(self, *, raw_band_priority: Mapping[str, Any],
                 sensor_id: str = "", instance_id: str = "",
                 mapping_owner: str | None = None,
                 sink: Any = None, callback: Callable[[dict], Any] | None = None):
        self.sensor_id = str(sensor_id)
        self.instance_id = str(instance_id)
        self.mapping_owner = (
            self.instance_id if mapping_owner is None else str(mapping_owner))
        self.mapping_hash = mapping_hash(raw_band_priority)
        self._sink = sink
        self._callback = callback

    @property
    def enabled(self) -> bool:
        return self._sink is not None or self._callback is not None

    def emit(self, *, epoch: int, time: float,
             constellation: str, input_obs_code: str | None,
             raw_band: int | None, simplified_obs_code: str | None = None,
             decoder_obs_code: str | None = None, slot: int | None = None,
             disposition: str, error_class: str | None = None,
             error_code: str | None = None) -> None:
        """Emit a schema-complete record when this trace is enabled."""
        if not self.enabled:
            return
        if disposition not in _DISPOSITIONS:
            raise ValueError(f"unknown observation trace disposition: {disposition}")
        record = {
            "epoch": int(epoch),
            "time": float(time),
            "sensor_id": self.sensor_id,
            "instance_id": self.instance_id,
            "mapping_hash": self.mapping_hash,
            "constellation": str(constellation),
            # ``system`` is the compact reader-facing alias used by existing
            # RINEX diagnostics; retain ``constellation`` as the canonical
            # schema name for downstream data-great records.
            "system": str(constellation),
            "input_obs_code": input_obs_code,
            "raw_band": None if raw_band is None else int(raw_band),
            "mapping_owner": self.mapping_owner,
            "simplified_obs_code": simplified_obs_code,
            "decoder_obs_code": decoder_obs_code,
            "slot": None if slot is None else int(slot),
            "disposition": disposition,
            "error_class": error_class,
            "error_code": error_code,
        }
        try:
            if self._sink is not None:
                writer = getattr(self._sink, "write", None)
                appender = getattr(self._sink, "append", None)
                if writer is not None:
                    writer(record)
                elif appender is not None:
                    appender(record)
                elif callable(self._sink):
                    self._sink(record)
                else:
                    raise TypeError("observation mapping sink must be callable, writable, or appendable")
            if self._callback is not None:
                self._callback(record)
        except Exception:
            # Trace is observational only.  A broken optional sink must not
            # change decoded observations or positioning outcomes.
            return


def trace_from(raw_band_priority: Mapping[str, Any], *, sink=None,
               callback=None, sensor_id: str = "", instance_id: str = "",
               mapping_owner: str | None = None) -> ObservationMappingTrace | None:
    """Build a trace object only when an explicit sink/callback is supplied."""
    if sink is None and callback is None:
        return None
    return ObservationMappingTrace(
        raw_band_priority=raw_band_priority, sensor_id=sensor_id,
        instance_id=instance_id, mapping_owner=mapping_owner,
        sink=sink, callback=callback)
