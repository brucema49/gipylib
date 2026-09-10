"""传感器工厂。"""
from queue import Queue
from typing import List

from src.core.thread_control import ThreadControl
from src.stream.base import BaseSensor
from src.stream.imu_sensor import ImuSensor
from src.stream.gnss_sol_sensor import GnssSolSensor
from src.stream.awesome_sensor import AwesomeGnssSensor, AwesomeImuSensor


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

        imu_coord = config.get("ins", {}).get("imu_coordinate_system", "FRD")
        imu_format = config.get("ins", {}).get("imu_format", "gpst")
        week = int(config.get("gnss", {}).get("week", 0))
        start_sow = config.get("ins", {}).get("start_sow")
        end_sow = config.get("ins", {}).get("end_sow")

        if gnss_source == "awesome_external":
            imu_path = config["ins"]["imu_data_path"]
            sensors.append(AwesomeImuSensor(
                imu_path, imu_queue, control, week, start_sow, end_sow))
            gnss_path = config["gnss"]["external_sol_path"]
            sensors.append(AwesomeGnssSensor(
                gnss_path, gnss_queue, control, week, start_sow, end_sow))
        elif gnss_source == "external":
            # external + ins.enabled=lc: IMU + 外部 GNSS 结果
            imu_path = config["ins"]["imu_data_path"]
            sensors.append(ImuSensor(imu_path, imu_queue, control, imu_coord, imu_format))
            gnss_path = config["gnss"]["external_sol_path"]
            sensors.append(GnssSolSensor(gnss_path, gnss_queue, control))
        elif gnss_source == "internal" and ins_enabled == "off":
            # internal + off: 纯 GNSS，仅 InternalGnssSensor，无 IMU
            from src.stream.internal_gnss_sensor import InternalGnssSensor
            sensors.append(InternalGnssSensor(config, gnss_queue, control))
        elif gnss_source == "internal" and ins_enabled == "lc":
            # internal + lc: IMU 流式 + 内部 GNSS 实时解算 → 对齐输出
            imu_path = config["ins"]["imu_data_path"]
            sensors.append(ImuSensor(imu_path, imu_queue, control, imu_coord, imu_format))
            from src.stream.internal_gnss_sensor import InternalGnssSensor
            sensors.append(InternalGnssSensor(config, gnss_queue, control))
        elif gnss_source == "internal" and ins_enabled == "tc":
            # internal + tc: IMU 流式 + TcGnssSensor (原始观测, 不做 GNSS 解算)
            # native increment 输入必须走 AwesomeImuSensor (它保留 dtheta/dvel 与
            # 来源 SOW); ImuSensor 仅理解 rate 文本格式, 用它会把增量误当 rate。
            imu_path = config["ins"]["imu_data_path"]
            if imu_format == "awesome_increment":
                sensors.append(AwesomeImuSensor(
                    imu_path, imu_queue, control, week, start_sow, end_sow))
            else:
                sensors.append(ImuSensor(
                    imu_path, imu_queue, control, imu_coord, imu_format))
            from src.stream.tc_gnss_sensor import TcGnssSensor
            sensors.append(TcGnssSensor(config, gnss_queue, control))

        return sensors
