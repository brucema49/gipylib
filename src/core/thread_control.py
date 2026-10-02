"""全局线程控制。"""
import threading
import logging
from queue import Full


class ThreadControl:
    """所有线程检查 running 标志决定是否退出。"""

    def __init__(self):
        self._running = True
        self._lock = threading.Lock()
        self._error = None

    def fail(self, source: str, error: Exception) -> None:
        """Publish the first worker failure and cancel the whole pipeline."""
        with self._lock:
            first = self._error is None
            if first:
                self._error = RuntimeError(f"{source}: {error}")
                self._error.__cause__ = error
            self._running = False
        if first:
            logging.getLogger(__name__).error("%s: %s", source, error)

    def raise_if_failed(self) -> None:
        with self._lock:
            error = self._error
        if error is not None:
            raise error

    def put(self, queue, item) -> bool:
        """Allow producers blocked on full queues to exit on cancellation."""
        while self.is_running():
            try:
                queue.put(item, timeout=0.1)
                return True
            except Full:
                continue
        return False

    def shutdown(self) -> None:
        with self._lock:
            self._running = False

    def is_running(self) -> bool:
        with self._lock:
            return self._running
