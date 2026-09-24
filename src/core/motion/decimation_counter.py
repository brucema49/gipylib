"""Decimation 计数器。

参考 ignav ``postpos.cc`` 的 ``nc++ > nhz ? nc=0,true:false``：
计数达到 ``min_count`` 时触发并复位，**无论后续 guard 是否通过**
（guard 失败也要消耗掉这次触发机会，否则会退化成"每历元都试"）。
"""


class DecimationCounter:
    """Decimation 计数器。

    ``min_count = 0`` 时每个历元都触发（等价于 ignav 的 ``nhz = 0``）。
    """

    def __init__(self, min_count: int):
        self.min_count = max(0, int(min_count))
        self.count = 0

    def should_trigger(self) -> bool:
        if self.count >= self.min_count:
            self.count = 0
            return True
        self.count += 1
        return False

    def reset(self) -> None:
        self.count = 0
