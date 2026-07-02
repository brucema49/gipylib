"""IMU 传感器线程。"""
from queue import Queue

from src.core.thread_control import ThreadControl
from src.stream.base import StreamerBase
from src.stream.formators import ImuFormator


class ImuSensor(StreamerBase):
    """IMU 文本读取（GPST 格式）。"""

    def __init__(self, file_path: str, output_queue: Queue,
                 control: ThreadControl):
        super().__init__(
            file_path=file_path,
            formator=ImuFormator(),
            output_queue=output_queue,
            control=control,
            tag="imu",
        )
