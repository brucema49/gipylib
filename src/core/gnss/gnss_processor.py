"""GNSS 处理器抽象基类与异常。"""
from abc import ABC, abstractmethod
from typing import Optional

from src.core.data_types import GnssSolution


class GnssProcessor(ABC):
    """GNSS 处理器抽象基类（内部模式）。

    每个 process_epoch 调用处理一个历元，返回 GnssSolution 或 None。
    跨历元状态（如 nav.x, nav.P）由 nav 对象维护，处理器持有 nav 引用。
    """

    @abstractmethod
    def process_epoch(self, obsr, obsb=None) -> Optional[GnssSolution]:
        """处理单个历元。

        Args:
            obsr: 流动站观测值（rtklib-py obs 对象）
            obsb: 基站观测值（RTK 模式，SPP 为 None）

        Returns:
            GnssSolution 或 None（解算失败时）
        """
        ...

    def reset(self) -> None:
        """重置处理器状态（默认无操作，子类按需覆盖）。"""
        pass


class GnssConfigError(ValueError):
    """GNSS 配置错误。"""
    pass


class GnssSolutionError(RuntimeError):
    """GNSS 解算运行时错误。"""
    pass
