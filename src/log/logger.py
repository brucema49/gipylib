"""日志记录器线程。

从 imu_queue、gnss_queue 消费数据，做 GNSS 触发匹配，
委托 AlignedWriter 输出 CSV。
"""
from queue import Queue, Empty
from threading import Thread

from src.core.thread_control import ThreadControl
from src.core.data_types import SensorData
from src.log.aligned_writer import AlignedWriter
from src.log.aligner import Aligner


class Logger(Thread):
    """日志记录器线程。"""

    def __init__(self, imu_queue: Queue, gnss_queue: Queue,
                 writer: AlignedWriter, aligner: Aligner,
                 control: ThreadControl):
        Thread.__init__(self, name="Logger", daemon=True)
        self.imu_queue = imu_queue
        self.gnss_queue = gnss_queue
        self.writer = writer
        self.aligner = aligner
        self.control = control
        self.imu_eof = False

    def run(self):
        self.writer.open()
        try:
            while self.control.is_running():
                # 1. 先把 imu_queue 中的数据搬到 aligner.imu_buffer
                self._drain_imu_queue()
                # 2. 取一个 GNSS 历元（阻塞，超时 0.1s）
                try:
                    gnss = self.gnss_queue.get(timeout=0.1)
                except Empty:
                    continue
                # 3. 区分 EOF sentinel 与正常数据
                if gnss is None:
                    # GNSS 文件已读完，再排空一次 IMU 后退出
                    self._drain_imu_queue()
                    break
                # 4. 解包 SensorData 并匹配写出
                if isinstance(gnss, SensorData):
                    gnss = gnss.gnss_solution
                # 等待 IMU 数据读到 >= GNSS 历元时间，避免竞态
                self._wait_for_imu(gnss.timestamp)
                aligned = self.aligner.match(gnss)
                if aligned is not None:
                    self.writer.write(aligned)
        finally:
            self.writer.close()

    def _wait_for_imu(self, gnss_timestamp: float) -> None:
        """阻塞直到 IMU 缓冲包含 >= gnss_timestamp 的数据，或 IMU EOF。

        防止 Logger 处理 GNSS 历元过快，导致对应 IMU 数据尚未被传感器读到。
        """
        while self.control.is_running() and not self.imu_eof:
            buf = self.aligner.imu_buffer
            if buf and buf[-1].timestamp >= gnss_timestamp:
                return
            try:
                imu = self.imu_queue.get(timeout=0.1)
            except Empty:
                continue
            if imu is None:
                self.imu_eof = True
                return
            if isinstance(imu, SensorData):
                imu = imu.imu
            self.aligner.push_imu(imu)

    def _drain_imu_queue(self):
        """非阻塞地把 imu_queue 中所有数据搬到 aligner.imu_buffer。

        遇到 None sentinel 时不放入缓冲（标记 IMU EOF）。
        """
        while True:
            try:
                imu = self.imu_queue.get_nowait()
            except Empty:
                break
            if imu is None:
                self.imu_eof = True
                continue
            # 解包 SensorData 取出 ImuMeasurement
            if isinstance(imu, SensorData):
                imu = imu.imu
            self.aligner.push_imu(imu)
