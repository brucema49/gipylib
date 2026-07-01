"""纯 GNSS 模式日志线程，仅消费 gnss_queue。"""
from queue import Queue, Empty
from threading import Thread

from src.core.thread_control import ThreadControl
from src.core.data_types import SensorData


class SolutionLogger(Thread):
    """纯 GNSS 模式日志线程。

    仅消费 gnss_queue，把 GnssSolution 委托 SolutionWriter 输出。
    收到 None（EOF sentinel）后关闭 writer 并退出。
    """

    def __init__(self, gnss_queue: Queue, writer, control: ThreadControl):
        Thread.__init__(self, name="SolutionLogger", daemon=True)
        self.gnss_queue = gnss_queue
        self.writer = writer
        self.control = control

    def run(self):
        self.writer.open()
        try:
            while self.control.is_running():
                try:
                    data = self.gnss_queue.get(timeout=0.1)
                except Empty:
                    continue
                if data is None:
                    break  # EOF sentinel
                if isinstance(data, SensorData):
                    sol = data.gnss_solution
                    if sol is not None:
                        self.writer.write(sol)
        finally:
            self.writer.close()
