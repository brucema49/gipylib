"""外部 GNSS 结果读取（rtklib POS 格式）。"""
from queue import Queue

from src.core.thread_control import ThreadControl
from src.stream.base import StreamerBase
from src.stream.formators import PosSolFormator


class GnssSolSensor(StreamerBase):
    """外部 GNSS 结果读取。"""

    def __init__(self, file_path: str, output_queue: Queue,
                 control: ThreadControl):
        super().__init__(
            file_path=file_path,
            formator=PosSolFormator(),
            output_queue=output_queue,
            control=control,
            tag="gnss_solution",
        )
