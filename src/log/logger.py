"""日志记录器线程（统一流式事件循环）。

从 imu_queue、gnss_queue 消费数据，做 GNSS 触发匹配，
委托 AlignedWriter 输出 CSV。可选地同时输出纯 GNSS .pos 文件。
可选地增量运行松组合 EKF (LcStream) 流式输出 RTKLC.pos。

事件循环（每个 GNSS 历元）：
  1. 搬运 IMU → aligner 缓冲 + LC 缓冲
  2. 取 GNSS 历元 → 立即写 RTK.pos
  3. 等待 IMU 覆盖 harvest 窗口
  4. 从 LC 缓冲按时间交错喂入 LcStream（只喂 harvest 窗口内的 IMU）
     GNSS 前的 IMU → GNSS → GNSS 后的 IMU（窗口内），超出的留给下一历元
     GNSS 后的 IMU (t >= gnss.t) 触发 LcIntegration 的 GVINS 风格插值+融合
  5. harvest 对齐块 → 写 aligned.csv

GVINS 风格 IMU 消费 (lc_integration.add_imu)：
  IMU 按时间顺序喂入 LcStream。当某条 IMU.t >= gnss.t 时，
  LcIntegration 自动线性插值到 gnss.t 并触发 GNSS 量测更新。
  关键不变量：GNSS 必须在 t >= gnss.t 的 IMU 之前入队（由 _lc_imu_buffer 分前后段保证）。
"""
import logging
from collections import deque
from queue import Queue, Empty
from threading import Thread

from src.core.thread_control import ThreadControl
from src.core.data_types import SensorData, ImuMeasurement
from src.log.aligned_writer import AlignedWriter
from src.log.aligner import Aligner

logger = logging.getLogger(__name__)

# Unix timestamps around the campus01 epoch are represented with a spacing of
# roughly 2.4e-7 s.  Treating an IMU sample that is within a few ulps of a
# GNSS epoch as "before" the epoch would make the published GNSS-time row
# contain the pre-update state.  This tolerance only orders equal-time
# events; it does not resample or alter either sensor timestamp.
_GNSS_IMU_TIME_TOLERANCE_S = 1.0e-6


class Logger(Thread):
    """松组合日志记录器线程（统一流式事件循环）。

    Args:
        imu_queue: IMU 数据队列
        gnss_queue: GNSS 数据队列
        writer: AlignedWriter，输出对齐块状 CSV
        aligner: Aligner，IMU 积攒 + GNSS 收割匹配器
        control: ThreadControl 线程控制
        gnss_writer: 可选的 SolutionWriter，同时输出纯 GNSS .pos
        lc_stream: 可选的 LcStream，增量运行松组合 EKF 流式输出 RTKLC.pos
    """

    def __init__(self, imu_queue: Queue, gnss_queue: Queue,
                 writer: AlignedWriter, aligner: Aligner,
                 control: ThreadControl, gnss_writer=None,
                 lc_stream=None):
        Thread.__init__(self, name="Logger", daemon=True)
        self.imu_queue = imu_queue
        self.gnss_queue = gnss_queue
        self.writer = writer
        self.aligner = aligner
        self.control = control
        self.gnss_writer = gnss_writer
        self.lc_stream = lc_stream
        self.imu_eof = False
        # LC 专用 IMU 缓冲：暂存 IMU，按 GNSS 节奏喂入 LcStream
        # 离线模式下 IMU 传感器可能远超 GNSS 解算速度，必须节流到 harvest 窗口
        # 同时保证 GNSS 在 t>=gnss.t 的 IMU 之前入队（GVINS 插值触发前提）
        self._lc_imu_buffer: deque = deque()

    def run(self):
        self.writer.open()
        if self.gnss_writer is not None:
            self.gnss_writer.open()
        if self.lc_stream is not None:
            self.lc_stream.open()
        try:
            while self.control.is_running():
                # 1. 搬运可用 IMU 到 aligner + LC 缓冲
                new_imus = self._drain_imu_queue()
                if self.lc_stream is not None:
                    for imu in new_imus:
                        self._lc_imu_buffer.append(imu)

                # 2. 取一个 GNSS 历元（阻塞，超时 0.1s）
                try:
                    gnss = self.gnss_queue.get(timeout=0.1)
                except Empty:
                    # 无 GNSS，IMU 已入缓冲，等 GNSS 到来后再按时间顺序喂入
                    continue

                # 3. EOF sentinel
                if gnss is None:
                    remaining = self._drain_imu_queue()
                    if self.lc_stream is not None:
                        for imu in remaining:
                            self._lc_imu_buffer.append(imu)
                        # 喂入所有剩余 IMU
                        while self._lc_imu_buffer:
                            self.lc_stream.feed_imu(
                                self._lc_imu_buffer.popleft())
                    break

                # 4. 解包 SensorData
                if isinstance(gnss, SensorData):
                    gnss = gnss.gnss_solution

                # 5. 立即写纯 GNSS .pos
                if self.gnss_writer is not None:
                    self.gnss_writer.write(gnss)

                # 6. 等待 IMU 覆盖 harvest 窗口
                wait_imus = self._wait_for_imu(
                    gnss.timestamp + self.aligner.harvest_window)
                if self.lc_stream is not None:
                    for imu in wait_imus:
                        self._lc_imu_buffer.append(imu)

                # 7. 从 LC 缓冲按时间交错喂入 LcStream
                #    只喂 gnss.timestamp + harvest_window 以内的 IMU
                #    超出的留在缓冲给下一个 GNSS 历元
                #    同时间戳 GNSS 先于 IMU（与批处理排序一致）
                #    GNSS 后的 IMU (t >= gnss.t) 触发 GVINS 插值+融合
                if self.lc_stream is not None:
                    feed_cutoff = gnss.timestamp + self.aligner.harvest_window
                    before = []
                    after = []
                    while self._lc_imu_buffer:
                        imu = self._lc_imu_buffer[0]
                        if imu.timestamp > feed_cutoff:
                            break
                        if imu.timestamp < gnss.timestamp:
                            before.append(self._lc_imu_buffer.popleft())
                        else:
                            after.append(self._lc_imu_buffer.popleft())
                    for imu in before:
                        self.lc_stream.feed_imu(imu)
                    self.lc_stream.feed_gnss(gnss)
                    for imu in after:
                        self.lc_stream.feed_imu(imu)

                # 8. harvest 对齐块 → 写 CSV
                aligned = self.aligner.harvest(gnss)
                if aligned is not None:
                    self.writer.write(aligned)
        finally:
            self.writer.close()
            if self.gnss_writer is not None:
                self.gnss_writer.close()
            if self.lc_stream is not None:
                self.lc_stream.finalize()
                self.lc_stream.close()

    def _wait_for_imu(self, gnss_timestamp: float):
        """阻塞直到 IMU 缓冲包含 >= gnss_timestamp 的数据，或 IMU EOF。

        返回等待期间新到达的 IMU 列表（同时推入 aligner 缓冲）。
        """
        new_imus = []
        while self.control.is_running() and not self.imu_eof:
            buf = self.aligner.imu_buffer
            if buf and buf[-1].timestamp >= gnss_timestamp:
                return new_imus
            try:
                imu = self.imu_queue.get(timeout=0.1)
            except Empty:
                continue
            if imu is None:
                self.imu_eof = True
                return new_imus
            if isinstance(imu, SensorData):
                imu = imu.imu
            self.aligner.push_imu(imu)
            new_imus.append(imu)
        return new_imus

    def _drain_imu_queue(self):
        """非阻塞搬运 imu_queue 到 aligner 缓冲。

        返回新搬运的 IMU 列表。遇到 None sentinel 标记 IMU EOF。
        """
        new_imus = []
        while True:
            try:
                imu = self.imu_queue.get_nowait()
            except Empty:
                break
            if imu is None:
                self.imu_eof = True
                continue
            if isinstance(imu, SensorData):
                imu = imu.imu
            self.aligner.push_imu(imu)
            new_imus.append(imu)
        return new_imus


class TcLogger(Thread):
    """紧组合日志记录器线程（GVINS 风格 IMU/GNSS 时序交错喂入 TcStream）。

    与 Logger (LC) 的区别：
      - gnss_queue 承载原始 GNSS 观测 (SensorData.gnss_raw=(obsr,obsb,nav))
        而非预解算 GnssSolution
      - 不需要 Aligner/AlignedWriter (TcStream per-IMU 直接写 .rslt)
      - 不需要 gnss_writer (TC 不输出纯 GNSS .pos)
      - 使用 tc_stream (TcStream) 替代 lc_stream (LcStream)

    事件循环（每个 GNSS 历元）：
      1. 搬运 IMU → tc_imu_buffer
      2. 取一个 GNSS 原始观测（阻塞，超时 0.1s）
      3. EOF sentinel：喂入所有剩余 IMU
      4. 等待 IMU 覆盖 harvest 窗口（默认 1.0s）
      5. 从 tc_imu_buffer 按时间交错喂入 TcStream
         GNSS 前的 IMU → GNSS 原始观测 → GNSS 后的 IMU（窗口内）
         超出的留给下一历元
         GNSS 后的 IMU (t >= gnss.t) 触发 TcIntegration 的 GVINS 风格插值+融合

    GVINS 风格 IMU 消费 (tc_integration.add_imu)：
      IMU 按时间顺序喂入。当某条 IMU.t >= gnss.t 时，
      TcIntegration 自动线性插值到 gnss.t 并触发 GNSS 量测更新。
      关键不变量：GNSS 必须在 t >= gnss.t 的 IMU 之前入队（由 _tc_imu_buffer 分前后段保证）。
    """

    def __init__(self, imu_queue: Queue, gnss_queue: Queue,
                 tc_stream, control: ThreadControl,
                 harvest_window: float = 1.0):
        Thread.__init__(self, name="TcLogger", daemon=True)
        self.imu_queue = imu_queue
        self.gnss_queue = gnss_queue
        self.tc_stream = tc_stream
        self.control = control
        self.harvest_window = float(harvest_window)
        self.imu_eof = False
        # TC 专用 IMU 缓冲：暂存 IMU，按 GNSS 节奏喂入 TcStream
        # 离线模式下 IMU 传感器可能远超 GNSS 解算速度，必须节流到 harvest 窗口
        # 同时保证 GNSS 在 t>=gnss.t 的 IMU 之前入队（GVINS 插值触发前提）
        self._tc_imu_buffer: deque = deque()

    def _partition_imu_window(self, t_gnss: float, feed_cutoff: float | None = None):
        """Split buffered IMU samples around one GNSS epoch.

        Samples within the timestamp representation tolerance are placed in
        the ``after`` side so the GNSS event is queued before the endpoint
        IMU.  The remaining samples stay in the buffer for the next epoch.
        """
        if feed_cutoff is None:
            feed_cutoff = t_gnss + self.harvest_window
        before = []
        after = []
        while self._tc_imu_buffer:
            imu = self._tc_imu_buffer[0]
            if imu.timestamp > feed_cutoff:
                break
            if imu.timestamp < t_gnss - _GNSS_IMU_TIME_TOLERANCE_S:
                before.append(self._tc_imu_buffer.popleft())
            else:
                after.append(self._tc_imu_buffer.popleft())
        return before, after

    def run(self):
        self.tc_stream.open()
        prefetched_gnss = None
        have_prefetched_gnss = False
        try:
            while self.control.is_running():
                # 1. 搬运可用 IMU 到 TC 缓冲
                new_imus = self._drain_imu_queue()
                for imu in new_imus:
                    self._tc_imu_buffer.append(imu)

                # 2. 取一个 GNSS 原始观测历元（阻塞，超时 0.1s）
                if have_prefetched_gnss:
                    gnss = prefetched_gnss
                    prefetched_gnss = None
                    have_prefetched_gnss = False
                else:
                    try:
                        gnss = self.gnss_queue.get(timeout=0.1)
                    except Empty:
                        # 无 GNSS，IMU 已入缓冲，等 GNSS 到来后再按时间顺序喂入
                        continue

                # 3. EOF sentinel
                if gnss is None:
                    remaining = self._drain_imu_queue()
                    for imu in remaining:
                        self._tc_imu_buffer.append(imu)
                    # 喂入所有剩余 IMU
                    while self._tc_imu_buffer:
                        self.tc_stream.feed_imu(
                            self._tc_imu_buffer.popleft())
                    break

                # 4. 解包 SensorData → (obsr, obsb, nav)
                if isinstance(gnss, SensorData):
                    obsr, obsb, nav = gnss.gnss_raw
                else:
                    obsr, obsb, nav = gnss
                t_gnss = float(obsr.t.time + obsr.t.sec)

                # Keep one GNSS epoch in hand when the producer has already
                # queued it.  A full one-second harvest window otherwise lets
                # the current iteration consume the next epoch's endpoint
                # IMU before that GNSS update is visible to the stream.
                try:
                    prefetched_gnss = self.gnss_queue.get_nowait()
                    have_prefetched_gnss = True
                except Empty:
                    prefetched_gnss = None
                    have_prefetched_gnss = False

                feed_cutoff = t_gnss + self.harvest_window
                if have_prefetched_gnss and prefetched_gnss is not None:
                    if isinstance(prefetched_gnss, SensorData):
                        next_obsr = prefetched_gnss.gnss_raw[0]
                    else:
                        next_obsr = prefetched_gnss[0]
                    next_t = float(next_obsr.t.time + next_obsr.t.sec)
                    if next_t > t_gnss:
                        feed_cutoff = min(
                            feed_cutoff,
                            next_t - _GNSS_IMU_TIME_TOLERANCE_S,
                        )

                # 5. 等待 IMU 覆盖 harvest 窗口
                wait_imus = self._wait_for_imu(feed_cutoff)
                for imu in wait_imus:
                    self._tc_imu_buffer.append(imu)

                # 6. 从 TC 缓冲按时间交错喂入 TcStream
                #    只喂 t_gnss + harvest_window 以内的 IMU
                #    超出的留在缓冲给下一个 GNSS 历元
                #    同时间戳 GNSS 先于 IMU（与批处理排序一致）
                #    GNSS 后的 IMU (t >= gnss.t) 触发 GVINS 插值+融合
                before, after = self._partition_imu_window(
                    t_gnss, feed_cutoff=feed_cutoff)
                for imu in before:
                    self.tc_stream.feed_imu(imu)
                self.tc_stream.feed_gnss_raw(obsr, obsb, nav)
                for imu in after:
                    self.tc_stream.feed_imu(imu)
        finally:
            self.tc_stream.finalize()
            self.tc_stream.close()

    def _wait_for_imu(self, gnss_timestamp: float):
        """阻塞直到 IMU 缓冲包含 >= gnss_timestamp 的数据，或 IMU EOF。

        返回等待期间新到达的 IMU 列表。
        """
        new_imus = []
        while self.control.is_running() and not self.imu_eof:
            if self._tc_imu_buffer and \
                    self._tc_imu_buffer[-1].timestamp >= gnss_timestamp:
                return new_imus
            try:
                imu = self.imu_queue.get(timeout=0.1)
            except Empty:
                continue
            if imu is None:
                self.imu_eof = True
                return new_imus
            if isinstance(imu, SensorData):
                imu = imu.imu
            new_imus.append(imu)
        return new_imus

    def _drain_imu_queue(self):
        """非阻塞搬运 imu_queue 到 TC 缓冲。

        返回新搬运的 IMU 列表。遇到 None sentinel 标记 IMU EOF。
        """
        new_imus = []
        while True:
            try:
                imu = self.imu_queue.get_nowait()
            except Empty:
                break
            if imu is None:
                self.imu_eof = True
                continue
            if isinstance(imu, SensorData):
                imu = imu.imu
            new_imus.append(imu)
        return new_imus
