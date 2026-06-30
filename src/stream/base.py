"""传感器抽象基类与流式读取基类。"""
from abc import ABC, abstractmethod
from queue import Queue, Empty
from threading import Thread
from typing import Optional

from src.core.thread_control import ThreadControl


class BaseSensor(ABC):
    """传感器抽象基类，强制 get_data() 接口。"""

    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def get_data(self):
        """获取一条数据（非阻塞，无数据返回 None）。"""
        ...


class StreamerBase(BaseSensor, Thread):
    """流式读取器基类 — 逐行读取文本文件，O(1) 内存。

    继承 BaseSensor 与 Thread，内部封装：逐行读取 → Formator 解码 → 推入队列。
    文件读完后向队列推入 None 作为 EOF sentinel。
    """

    def __init__(self, file_path: str, formator, output_queue: Queue,
                 control: ThreadControl, tag: str):
        # 注意：必须先初始化 Thread，因为 Thread.name 是 property，
        # 依赖 _initialized 标志；若先调 BaseSensor.__init__ 设 self.name 会报错。
        Thread.__init__(self, name=tag, daemon=True)
        BaseSensor.__init__(self, name=tag)
        self.file_path = file_path
        self.formator = formator
        self.output_queue = output_queue
        self.control = control
        self.tag = tag

    def run(self):
        """线程入口：逐行读取 → 解码 → 入队 → EOF sentinel。"""
        try:
            with open(self.file_path, "r", encoding="utf-8") as f:
                for line in f:
                    if not self.control.is_running():
                        break
                    data = self.formator.decode(line)
                    if data is not None:
                        self.output_queue.put(data)
        finally:
            self.output_queue.put(None)  # EOF sentinel

    def get_data(self):
        """非阻塞返回队列头部数据。"""
        try:
            return self.output_queue.get_nowait()
        except Empty:
            return None
