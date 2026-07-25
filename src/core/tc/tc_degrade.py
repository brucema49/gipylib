"""紧组合降级管理器 (简化版)。

降级链: rtk → rtd → spp → imu_only
恢复策略: direct (GNSS 恢复立即回初始配置模式)
reboot: 持续无 GNSS > reboot_threshold → reboot(keep_random_walk=True)
"""

_DEGRADE_CHAIN = {"rtk": "rtd", "rtd": "spp", "spp": "imu_only", "imu_only": "imu_only"}


class TcDegradeManager:
    """紧组合降级管理器。

    Args:
        initial_mode: 初始 (最高精度) 模式 "rtk"/"rtd"/"spp"
        fail_threshold: 连续失败次数阈值, 达到后降级
        reboot_threshold: 持续无 GNSS 秒数阈值, 超过后触发 reboot
    """

    def __init__(self, initial_mode: str, fail_threshold: int = 3,
                 reboot_threshold: float = 30.0):
        self.initial_mode = initial_mode
        self.current_mode = initial_mode
        self.fail_threshold = fail_threshold
        self.reboot_threshold = reboot_threshold
        self._fail_count = 0
        self._rebooted = False

    def on_fail(self, estimator, reason: str = ""):
        """连续失败达阈值后降级一级。"""
        self._fail_count += 1
        if self._fail_count >= self.fail_threshold:
            new_mode = _DEGRADE_CHAIN.get(self.current_mode, "imu_only")
            if new_mode != self.current_mode:
                self.current_mode = new_mode
                estimator.switch_mode(new_mode, builder=None)
            self._fail_count = 0

    def on_success(self, estimator):
        """GNSS 量测成功后直接恢复到初始模式。"""
        if self.current_mode != self.initial_mode:
            self.current_mode = self.initial_mode
            estimator.switch_mode(self.initial_mode, builder=None)
        self._fail_count = 0
        self._rebooted = False

    def check_reboot(self, dt_no_gnss: float, estimator) -> bool:
        """持续无 GNSS 超阈值时触发 reboot (仅一次)。"""
        if dt_no_gnss > self.reboot_threshold and not self._rebooted:
            estimator.reboot(keep_random_walk=True)
            self._rebooted = True
            return True
        return False
