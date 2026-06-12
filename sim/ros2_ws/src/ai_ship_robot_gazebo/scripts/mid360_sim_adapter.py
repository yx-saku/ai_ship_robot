#!/usr/bin/env python3
"""GazeboのMid-360 simulation出力をLivox driver相当topicへ補完する。"""

from __future__ import annotations

import csv
import math
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


ANGLE_SCALE = 1000
FULL_CIRCLE_KEY = 360 * ANGLE_SCALE


def default_scan_pattern_csv_file() -> str:
    return str(Path(get_package_share_directory("ros2_livox_simulation")) / "scan_mode" / "mid360.csv")


def normalize_degrees(degrees: float) -> float:
    normalized = math.fmod(degrees, 360.0)
    if normalized < 0.0:
        normalized += 360.0
    return normalized


def normalize_azimuth_key(azimuth_mdeg: int) -> int:
    normalized = azimuth_mdeg % FULL_CIRCLE_KEY
    if normalized < 0:
        normalized += FULL_CIRCLE_KEY
    return normalized


def make_scan_pattern_key(azimuth_deg: float, zenith_deg: float) -> tuple[int, int]:
    return (
        normalize_azimuth_key(round(normalize_degrees(azimuth_deg) * ANGLE_SCALE)),
        round(zenith_deg * ANGLE_SCALE),
    )


class Mid360SimAdapter(Node):
    def __init__(self) -> None:
        super().__init__("mid360_sim_adapter")

        # Gazebo pluginのraw topicと実機driver相当の出力topicをparameter化し、検証時の差し替えを容易にする。
        self.input_custom_topic = self.declare_parameter("input_custom_topic", "/left_lidar/custom").value
        self.input_imu_topic = self.declare_parameter("input_imu_topic", "/left_lidar/imu").value
        self.output_custom_topic = self.declare_parameter("output_custom_topic", "/livox/lidar").value
        self.output_imu_topic = self.declare_parameter("output_imu_topic", "/livox/imu").value
        self.output_lidar_frame = self.declare_parameter("output_lidar_frame", "left_lidar_link").value
        self.output_imu_frame = self.declare_parameter("output_imu_frame", "left_lidar_imu_link").value

        # UV-Lab版LIO-SAMはIMU加速度をG単位前提で再スケールするため、sim出力だけG単位に直す。
        self.gravity = float(self.declare_parameter("gravity", 9.80511).value)
        self.convert_imu_acceleration_to_g = bool(
            self.declare_parameter("convert_imu_acceleration_to_g", True).value
        )
        if self.gravity <= 0.0:
            raise ValueError("gravity must be positive.")

        # simulation点群はlineが欠落するため、Livox scan patternの照射順から疑似lineを復元する。
        self.synthesize_line_from_pattern = bool(self.declare_parameter("synthesize_line_from_pattern", True).value)
        self.force_zero_offset_time = bool(self.declare_parameter("force_zero_offset_time", False).value)
        self.scan_pattern_csv_file = self.declare_parameter(
            "scan_pattern_csv_file", default_scan_pattern_csv_file()
        ).value
        self.scan_pattern_physical_line_count = int(
            self.declare_parameter("scan_pattern_physical_line_count", 4).value
        )
        self.synthetic_line_count = int(self.declare_parameter("synthetic_line_count", 4).value)
        self.scan_pattern_line_by_direction = self.load_scan_pattern(self.scan_pattern_csv_file)

        self.custom_publisher = self.create_publisher(CustomMsg, self.output_custom_topic, qos_profile_sensor_data)
        self.imu_publisher = self.create_publisher(Imu, self.output_imu_topic, qos_profile_sensor_data)
        self.create_subscription(CustomMsg, self.input_custom_topic, self.custom_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, self.input_imu_topic, self.imu_callback, qos_profile_sensor_data)

        self.get_logger().info(
            "Mid-360 sim adapter: %s -> %s, %s -> %s"
            % (self.input_custom_topic, self.output_custom_topic, self.input_imu_topic, self.output_imu_topic)
        )

    def load_scan_pattern(self, csv_file: str) -> dict[tuple[int, int], int]:
        scan_pattern_path = Path(csv_file)
        if not self.synthesize_line_from_pattern:
            return {}
        if not scan_pattern_path.is_file():
            raise FileNotFoundError(f"scan pattern csv not found: {csv_file}")

        # CSVの行順がLivoxの照射順なので、物理line内の進行順からLIO-SAM用lineを均等に割り当てる。
        line_by_direction: dict[tuple[int, int], int] = {}
        collision_count = 0
        physical_line_count = max(1, self.scan_pattern_physical_line_count)
        synthetic_line_count = max(1, self.synthetic_line_count)
        with scan_pattern_path.open("r", encoding="utf-8", newline="") as stream:
            reader = csv.DictReader(stream)
            for pattern_index, row in enumerate(reader):
                try:
                    azimuth = row.get("Azimuth/deg", row.get("Azimuth"))
                    zenith = row.get("Zenith/deg", row.get("Zenith"))
                    key = make_scan_pattern_key(float(azimuth), float(zenith))
                except (KeyError, TypeError, ValueError):
                    continue
                line = (pattern_index // physical_line_count) % synthetic_line_count
                previous = line_by_direction.setdefault(key, line)
                if previous != line:
                    collision_count += 1

        if not line_by_direction:
            raise ValueError(f"scan pattern csv has no usable directions: {csv_file}")
        self.get_logger().info("Loaded %d Livox scan pattern directions." % len(line_by_direction))
        if collision_count > 0:
            self.get_logger().warning("Livox scan pattern had %d direction key collisions." % collision_count)
        return line_by_direction

    def custom_callback(self, message: CustomMsg) -> None:
        output = CustomMsg()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.output_lidar_frame or message.header.frame_id
        output.timebase = message.timebase
        output.lidar_id = message.lidar_id
        output.rsvd = self.normalized_reserved_bytes(message.rsvd)

        # point_numとpoints長のずれを吸収し、補正後の点数をdriver出力相当に再設定する。
        point_count = len(message.points)
        if message.point_num > 0:
            point_count = min(point_count, message.point_num)
        output.points = [self.convert_point(message.points[index], index) for index in range(point_count)]
        output.point_num = len(output.points)
        self.custom_publisher.publish(output)

    def convert_point(self, point: CustomPoint, point_index: int) -> CustomPoint:
        output = CustomPoint()
        output.offset_time = 0 if self.force_zero_offset_time else point.offset_time
        output.x = point.x
        output.y = point.y
        output.z = point.z
        output.reflectivity = point.reflectivity
        output.tag = point.tag
        output.line = self.output_line(point, point_index)
        return output

    def output_line(self, point: CustomPoint, point_index: int) -> int:
        if self.synthesize_line_from_pattern and point.line == 0 and self.synthetic_line_count > 1:
            return min(255, self.lookup_scan_pattern_line(point) or self.fallback_line(point_index))
        return min(255, int(point.line))

    def fallback_line(self, point_index: int) -> int:
        return point_index % max(1, self.synthetic_line_count)

    def lookup_scan_pattern_line(self, point: CustomPoint) -> int | None:
        horizontal_range = math.hypot(float(point.x), float(point.y))
        point_range = math.hypot(horizontal_range, float(point.z))
        if point_range <= 1.0e-12:
            return None

        # pluginのray方向をCSVのAzimuth/Zenithへ戻し、float丸めを考慮して隣接ミリ度まで探索する。
        azimuth_deg = math.degrees(math.atan2(float(point.y), float(point.x)))
        zenith_deg = 90.0 - math.degrees(math.atan2(float(point.z), horizontal_range))
        azimuth_key, zenith_key = make_scan_pattern_key(azimuth_deg, zenith_deg)
        for zenith_delta in (-1, 0, 1):
            for azimuth_delta in (-1, 0, 1):
                key = (normalize_azimuth_key(azimuth_key + azimuth_delta), zenith_key + zenith_delta)
                line = self.scan_pattern_line_by_direction.get(key)
                if line is not None:
                    return line
        return None

    def imu_callback(self, message: Imu) -> None:
        output = Imu()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self.output_imu_frame or message.header.frame_id
        output.orientation = message.orientation
        output.orientation_covariance = list(message.orientation_covariance)
        output.angular_velocity = message.angular_velocity
        output.angular_velocity_covariance = list(message.angular_velocity_covariance)

        # ROS標準のm/s^2をG単位へ変換し、UV-Lab版LIO-SAMのimuConverter前提に合わせる。
        acceleration_scale = 1.0 / self.gravity if self.convert_imu_acceleration_to_g else 1.0
        output.linear_acceleration.x = message.linear_acceleration.x * acceleration_scale
        output.linear_acceleration.y = message.linear_acceleration.y * acceleration_scale
        output.linear_acceleration.z = message.linear_acceleration.z * acceleration_scale
        output.linear_acceleration_covariance = self.scale_acceleration_covariance(
            message.linear_acceleration_covariance, acceleration_scale
        )
        self.imu_publisher.publish(output)

    @staticmethod
    def normalized_reserved_bytes(values: list[int]) -> list[int]:
        reserved = [int(value) & 0xFF for value in list(values)[:3]]
        while len(reserved) < 3:
            reserved.append(0)
        return reserved

    @staticmethod
    def scale_acceleration_covariance(covariance: list[float], scale: float) -> list[float]:
        values = list(covariance)
        if len(values) == 9 and values[0] != -1.0:
            covariance_scale = scale * scale
            return [value * covariance_scale for value in values]
        return values


def main() -> None:
    rclpy.init()
    node = Mid360SimAdapter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
