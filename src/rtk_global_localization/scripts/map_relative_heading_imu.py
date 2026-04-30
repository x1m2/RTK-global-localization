#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def parse_map_origin_file(path: str) -> dict:
    values = {}
    with open(path, "r", encoding="utf-8") as origin_file:
        for raw_line in origin_file:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip().strip("'\"")
    return values


class MapRelativeHeadingImu(Node):
    def __init__(self) -> None:
        super().__init__("map_relative_heading_imu")

        self.declare_parameter("input_topic", "/rtk/navsat_heading")
        self.declare_parameter("output_topic", "/rtk/navsat_heading/map_relative")
        self.declare_parameter("map_origin_path", "")
        self.declare_parameter("output_frame_id", "body")
        self.declare_parameter("yaw_offset", 0.0)
        self.declare_parameter("log_every_n", 100)
        self.declare_parameter("output_qos_reliability", "reliable")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.map_origin_path = str(self.get_parameter("map_origin_path").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.yaw_offset = float(self.get_parameter("yaw_offset").value)
        self.log_every_n = max(1, int(self.get_parameter("log_every_n").value))
        self.output_qos_reliability = str(
            self.get_parameter("output_qos_reliability").value
        ).strip().lower()

        if not self.map_origin_path:
            raise RuntimeError("map_origin_path must not be empty")

        origin = parse_map_origin_file(self.map_origin_path)
        self.origin_yaw_enu = float(origin["yaw_enu"])

        self.received = 0
        self.published = 0

        publisher_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if self.output_qos_reliability == "best_effort"
                else ReliabilityPolicy.RELIABLE
            ),
        )
        self.publisher = self.create_publisher(Imu, self.output_topic, publisher_qos)
        self.subscription = self.create_subscription(
            Imu, self.input_topic, self._callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            "Map-relative heading converter started: "
            f"{self.input_topic} -> {self.output_topic}, "
            f"origin_yaw_enu={self.origin_yaw_enu:.6f}, yaw_offset={self.yaw_offset:.6f}, "
            f"output_qos={self.output_qos_reliability}"
        )

    def _callback(self, msg: Imu) -> None:
        self.received += 1

        absolute_yaw = yaw_from_quaternion(msg.orientation) + self.yaw_offset
        relative_yaw = normalize_angle(absolute_yaw - self.origin_yaw_enu)

        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        out.orientation = quaternion_from_yaw(relative_yaw)
        out.orientation_covariance = list(msg.orientation_covariance)
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = list(msg.angular_velocity_covariance)
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = list(msg.linear_acceleration_covariance)

        self.publisher.publish(out)
        self.published += 1

        if self.published == 1 or self.published % self.log_every_n == 0:
            self.get_logger().info(
                "Map-relative heading stats: "
                f"received={self.received}, published={self.published}, "
                f"absolute_yaw={absolute_yaw:.4f}, relative_yaw={relative_yaw:.4f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapRelativeHeadingImu()
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
