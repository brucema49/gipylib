"""Validate input files before launching workers or rewriting RINEX data."""
import math
from pathlib import Path
from datetime import datetime


class InputTextReader:
    """Track the physical line, including continuation records read with readline."""

    def __init__(self, path):
        self.path = path
        self.line_number = 0
        self.stream = open(path, "r", encoding="utf-8")

    def readline(self):
        try:
            line = self.stream.readline()
        except UnicodeDecodeError as exc:
            raise ValueError(f"{self.path}: line {self.line_number + 1}: invalid UTF-8 input: {exc}") from exc
        if line:
            self.line_number += 1
        return line

    def __iter__(self):
        return self

    def __next__(self):
        line = self.readline()
        if not line:
            raise StopIteration
        return line

    @property
    def closed(self):
        return self.stream.closed

    def close(self):
        self.stream.close()

    def __enter__(self):
        return self

    def __exit__(self, kind, error, traceback):
        self.close()
        if (kind in (ValueError, IndexError, UnicodeDecodeError)
                and not str(error).startswith(f"{self.path}:")):
            raise ValueError(f"{self.path}: line {self.line_number}: {error}") from error


def validate_rinex_header(path, kind):
    """Return the supported RINEX 3 version; fail with the original filename."""
    with InputTextReader(path) as stream:
        first = stream.readline()
        if first[60:80].strip() != "RINEX VERSION / TYPE":
            raise ValueError(f"{path}: line 1: missing RINEX VERSION / TYPE header")
        try:
            version = float(first[:9])
        except ValueError as exc:
            raise ValueError(f"{path}: line 1: invalid RINEX version {first[:9]!r}") from exc
        if not math.isfinite(version) or not 3.02 <= version < 4.0:
            raise ValueError(
                f"{path}: unsupported RINEX version {version:.2f}; "
                "this reader supports RINEX 3.02 <= version < 4.00. "
                "Convert the input to a supported RINEX 3 format."
            )
        if first[20:21] != kind:
            raise ValueError(f"{path}: line 1: expected RINEX file type {kind}, got {first[20:21]!r}")
        for line in stream:
            if line[60:73] == "END OF HEADER":
                return version
    raise ValueError(f"{path}: incomplete RINEX header: missing END OF HEADER")


def validate_epoch(year, month, day, hour, minute, second):
    """Validate calendar fields before RTKLIB can normalize/default them."""
    datetime(year, month, day, hour, minute)
    if not math.isfinite(second) or not 0 <= second < 61:
        raise ValueError(f"invalid epoch seconds {second!r}")


def validate_gnss_files(config):
    """Check original RINEX headers before signal planning and rewriting."""
    gnss = config["gnss"]
    if gnss["gnss_source"] == "internal":
        validate_rinex_header(gnss["rover_path"], "O")
        validate_rinex_header(gnss["eph_path"], "N")
        if gnss["positioning_mode"] in ("rtk", "rtd"):
            bases = gnss["base_path"]
            for path in bases if isinstance(bases, (list, tuple)) else [bases]:
                validate_rinex_header(path, "O")


def validate_input_files(config):
    """Validate whole text inputs, including tails outside the GNSS overlap."""
    validate_gnss_files(config)
    gnss = config["gnss"]
    ins = config["ins"]
    paths = []
    if gnss["gnss_source"] != "internal":
        paths.append(gnss["external_sol_path"])
    if ins["enabled"] != "off":
        paths.append(ins["imu_data_path"])
    for path in paths:
        with open(path, "rb") as stream:
            if not stream.read(1):
                raise ValueError(f"{Path(path)}: empty input file")

    # Use fresh instances of the actual parsers/axis converters. Stateful
    # incremental parsers used here are separate from the solving pipeline.
    from queue import Queue
    from src.core.thread_control import ThreadControl
    from src.stream.factory import SensorFactory
    from src.stream.base import StreamerBase
    sensors = SensorFactory.create_sensors(config, Queue(), Queue(), ThreadControl())
    for sensor in sensors:
        if not isinstance(sensor, StreamerBase):
            continue
        records = 0
        with InputTextReader(sensor.file_path) as stream:
            for line in stream:
                data = sensor.formator.decode(line)
                if data is not None:
                    sensor._process_data(data)
                    records += 1
        if records == 0:
            raise ValueError(f"{sensor.file_path}: no usable data records in the configured time range")
