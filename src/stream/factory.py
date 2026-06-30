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

        # IMU
        imu_path = config["ins"]["imu_data_path"]
        sensors.append(ImuSensor(imu_path, imu_queue, control))

        # GNSS（仅 external 模式，由 config_loader 保证）
        if config["gnss"]["gnss_source"] == "external":
            gnss_path = config["gnss"]["external_sol_path"]
            sensors.append(GnssSolSensor(gnss_path, gnss_queue, control))

        return sensors
