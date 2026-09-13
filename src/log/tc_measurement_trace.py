"""Opt-in, observational trace sink for TC measurement epochs.

The TC measurement builder owns the observation graph, while this adapter
owns delivery only.  No sink/callback means no records are allocated and no
trace work is performed.  Delivery failures are swallowed so diagnostics
cannot change a positioning result.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any


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
