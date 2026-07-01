"""传感器工厂。"""
from queue import Queue
from typing import List

from src.core.thread_control import ThreadControl
from src.stream.base import BaseSensor
from src.stream.imu_sensor import ImuSensor
from src.stream.gnss_sol_sensor import GnssSolSensor


class SensorFactory:
    """根据配置创建传感器实例列表。"""

    @staticmethod
    def create_sensors(config: dict,
                       imu_queue: Queue,
                       gnss_queue: Queue,
                       control: ThreadControl) -> List[BaseSensor]:
        sensors: List[BaseSensor] = []
        gnss_source = config["gnss"]["gnss_source"]
        ins_enabled = config["ins"]["enabled"]

        if gnss_source == "external":
            # external + ins.enabled=on: IMU + 外部 GNSS 结果
            imu_path = config["ins"]["imu_data_path"]
            sensors.append(ImuSensor(imu_path, imu_queue, control))
            gnss_path = config["gnss"]["external_sol_path"]
            sensors.append(GnssSolSensor(gnss_path, gnss_queue, control))
        elif gnss_source == "internal" and ins_enabled == "off":
            # internal + off: 纯 GNSS，仅 InternalGnssSensor，无 IMU
            from src.stream.internal_gnss_sensor import InternalGnssSensor
            sensors.append(InternalGnssSensor(config, gnss_queue, control))
        # internal + on 已由 config_loader 抛 NotImplementedError

        return sensors
