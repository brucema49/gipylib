"""Opt-in, observational trace sink for TC measurement epochs.

The TC measurement builder owns the observation graph, while this adapter
owns delivery only.  No sink/callback means no records are allocated and no
trace work is performed.  Delivery failures are swallowed so diagnostics
cannot change a positioning result.
"""

from __future__ import annotations

from collections.abc import Callable
import csv
import json
import logging
from pathlib import Path
from typing import Any

from src.core.time_utils import unix_to_gpst


logger = logging.getLogger(__name__)


class TcMeasurementTrace:
    """Emit one complete TC measurement-epoch record to an explicit sink."""

    def __init__(self, *, sink: Any = None,
                 callback: Callable[[dict], Any] | None = None):
        self._sink = sink
        self._callback = callback

    @property
    def enabled(self) -> bool:
        return self._sink is not None or self._callback is not None

    def emit(self, record: dict) -> None:
        """Deliver ``record`` when enabled; never affect the filter on error."""
        if not self.enabled:
            return
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
                    raise TypeError(
                        "TC measurement trace sink must be callable, "
                        "writable, or appendable"
                    )
            if self._callback is not None:
                self._callback(record)
        except Exception:
            # The trace is deliberately observational.  A broken optional
            # sink/callback must not perturb measurement construction.
            return


def trace_from(*, sink: Any = None,
               callback: Callable[[dict], Any] | None = None
               ) -> TcMeasurementTrace | None:
    """Build a trace adapter only when an explicit sink/callback is supplied."""
    if sink is None and callback is None:
        return None
    return TcMeasurementTrace(sink=sink, callback=callback)


class TcMeasurementTraceWriter:
    """Persist TC measurement graph events as JSONL or CSV.

    This writer is deliberately opt-in.  It accepts the complete application
    configuration so the output location remains owned by ``output.output_dir``
    and a trace filename cannot escape that directory.  A writer failure only
    disables this optional sink; it must never interrupt positioning.
    """

    SCHEMA = "GIPY_TC_MEASUREMENT_TRACE_V1"
    DEFAULT_FILENAMES = {
        "jsonl": "tc-measurement-trace.jsonl",
        "csv": "tc-measurement-trace.csv",
    }

    def __init__(self, config: dict | str | Path | None = None, *,
                 output_dir=None, filename=None, format=None, enabled=None):
        # The application uses the full config, while small diagnostic callers
        # can provide the four writer options directly.  Keeping both forms
        # avoids making the sink depend on the rest of the pipeline.
        if isinstance(config, dict):
            output = config.get("output", {})
            tc = config.get("tc", {})
            trace_cfg = tc.get("measurement_trace", {})
        else:
            output = {}
            trace_cfg = {}
            if config is not None:
                output_dir = config
        if not isinstance(output, dict):
            raise ValueError("output must be a mapping")
        if output_dir is None:
            output_dir = output.get("output_dir", "output")
        if not isinstance(trace_cfg, dict) and trace_cfg is not None:
            raise ValueError("tc.measurement_trace must be a mapping")
        if trace_cfg is None:
            trace_cfg = {}

        if enabled is None:
            enabled = trace_cfg.get("enabled", False)
        if isinstance(enabled, str):
            enabled_text = enabled.strip().lower()
            if enabled_text in {"true", "1", "yes", "on"}:
                enabled = True
            elif enabled_text in {"false", "0", "no", "off"}:
                enabled = False
            else:
                raise ValueError("tc.measurement_trace.enabled must be boolean")
        self.enabled = bool(enabled)
        if format is None:
            format = trace_cfg.get("format", "jsonl")
        self.format = str(format).strip().lower()
        if self.format not in self.DEFAULT_FILENAMES:
            raise ValueError(
                "tc.measurement_trace.format must be 'jsonl' or 'csv'"
            )
        if filename is None:
            filename = trace_cfg.get(
                "filename", self.DEFAULT_FILENAMES[self.format])
        self.filename = str(filename)
        self._validate_basename(self.filename)
        self.output_dir = Path(str(output_dir))
        self.path = self.output_dir / self.filename
        self._fp = None
        self._csv_writer = None
        self._csv_fields = None
        self._failed = False

    @staticmethod
    def _validate_basename(filename: str) -> None:
        """Reject absolute and directory-containing trace filenames."""
        if not filename or filename in (".", ".."):
            raise ValueError("tc.measurement_trace.filename must be a basename")
        # ``Path`` on POSIX does not treat a Windows drive/backslash as a
        # separator.  Reject both separator conventions explicitly so config
        # validation is platform-independent.
        if Path(filename).name != filename or "/" in filename or "\\" in filename:
            raise ValueError("tc.measurement_trace.filename must be a basename")
        if Path(filename).is_absolute():
            raise ValueError("tc.measurement_trace.filename must be a basename")

    def open(self) -> None:
        if not self.enabled or self._failed:
            return
        try:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            self._fp = self.path.open("w", encoding="utf-8", newline="")
            if self.format == "csv":
                self._csv_writer = None
                self._csv_fields = None
        except Exception as exc:
            self._disable(exc, "open")

    def write(self, record: dict) -> None:
        """Write one trace event, swallowing sink failures."""
        if not self.enabled or self._fp is None or self._failed:
            return
        try:
            event = self._record(record)
            if self.format == "jsonl":
                self._fp.write(self._canonical_json(event))
                self._fp.write("\n")
            else:
                self._write_csv(event)
            self._fp.flush()
        except Exception as exc:
            self._disable(exc, "write")

    def close(self) -> None:
        fp = self._fp
        self._fp = None
        self._csv_writer = None
        self._csv_fields = None
        if fp is None:
            return
        try:
            fp.close()
        except Exception as exc:
            self._disable(exc, "close")

    @classmethod
    def _record(cls, record: dict) -> dict:
        if not isinstance(record, dict):
            raise TypeError("TC measurement trace records must be mappings")
        normalized = {
            str(key): cls._normalize(value) for key, value in record.items()
        }
        normalized.setdefault("schema", cls.SCHEMA)
        timestamp = normalized.get("time", normalized.get("timestamp"))
        if timestamp is not None:
            week, sow = unix_to_gpst(float(timestamp))
            normalized["week"] = int(normalized.get("week", week))
            normalized["sow"] = float(normalized.get("sow", sow))
        else:
            normalized.setdefault("week", None)
            normalized.setdefault("sow", None)
        return normalized

    @classmethod
    def _normalize(cls, value):
        """Convert common scientific Python values to JSON-native values."""
        if isinstance(value, dict):
            return {str(key): cls._normalize(item)
                    for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [cls._normalize(item) for item in value]
        # Keep this module free of a hard numpy dependency while accepting
        # numpy scalar/array values when numpy is already in use by TC.
        if hasattr(value, "tolist"):
            return cls._normalize(value.tolist())
        if value is None or isinstance(value, (str, int, float, bool)):
            return value
        return str(value)

    @classmethod
    def _canonical_json(cls, value) -> str:
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
            raise ValueError("TC measurement trace CSV schema changed")
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
        logger.warning("TC measurement trace %s failed; sink disabled: %s",
                       operation, exc)
