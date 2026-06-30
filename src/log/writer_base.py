"""输出器抽象基类。"""
from abc import ABC, abstractmethod


class WriterBase(ABC):
    """所有输出器的统一接口，定义 open/write/close 生命周期。"""

    @abstractmethod
    def open(self) -> None:
        """打开输出目标。"""
        ...

    @abstractmethod
    def write(self, data) -> None:
        """写入一条数据。"""
        ...

    @abstractmethod
    def close(self) -> None:
        """关闭输出目标，释放资源。"""
        ...
