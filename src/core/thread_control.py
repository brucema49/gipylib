"""全局线程控制。"""
import threading


class ThreadControl:
    """所有线程检查 running 标志决定是否退出。"""

    def __init__(self):
        self._running = True
        self._lock = threading.Lock()

    def shutdown(self) -> None:
        with self._lock:
            self._running = False

    def is_running(self) -> bool:
        with self._lock:
            return self._running
