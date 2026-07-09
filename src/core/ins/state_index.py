"""状态参数块索引管理 (参考 gnss_ins_lc_nhc StateIndex)。

固定 15 维基础状态 + 可选参数块, 根据配置动态构建索引。
状态顺序:
  固定: pos(3), vel(3), att(3), gyro_bias(3), accel_bias(3)
  可选: lever_arm(3), imu_angle(2), imu_leverarm(3), time_sync(1)
"""
from dataclasses import dataclass


@dataclass
class StateIndex:
    """状态参数块索引管理。

    参考 gnss_ins_lc_nhc navstruct.hpp StateIndex 结构。
    未启用的可选块索引为 -1。
    """

    # 固定 15 维 (始终存在)
    pos: int = 0
    vel: int = 3
    att: int = 6
    gyro_bias: int = 9
    accel_bias: int = 12

    # 可选参数块 (-1 表示未启用)
    lever_arm: int = -1      # 3 维, GNSS 天线杆臂 (b 系)
    imu_angle: int = -1      # 2 维, IMU 安装角 [pitch, yaw]
    imu_leverarm: int = -1   # 3 维, IMU 杆臂 (b→v, NHC 用)
    time_sync: int = -1      # 1 维, 时间对齐误差

    # 总维数
    dim: int = 15

    @classmethod
    def from_config(cls, config: dict) -> "StateIndex":
        """根据配置构建状态索引。

        参考 gnss_ins_lc_nhc SetStateIndex (navinitialized.cc)。
        """
        si = cls()
        ins_cfg = config.get("ins", {}) if config else {}
        idx = 15

        if ins_cfg.get("estimate_leverarm", 0):
            si.lever_arm = idx
            idx += 3

        if ins_cfg.get("estimate_mounting_angle", 0):
            si.imu_angle = idx
            idx += 2
            if ins_cfg.get("estimate_imu_leverarm", 0):
                si.imu_leverarm = idx
                idx += 3

        if ins_cfg.get("estimate_time_sync", 0):
            si.time_sync = idx
            idx += 1

        si.dim = idx
        return si

    def has_lever_arm(self) -> bool:
        return self.lever_arm >= 0

    def has_imu_angle(self) -> bool:
        return self.imu_angle >= 0

    def has_imu_leverarm(self) -> bool:
        return self.imu_leverarm >= 0

    def has_time_sync(self) -> bool:
        return self.time_sync >= 0
