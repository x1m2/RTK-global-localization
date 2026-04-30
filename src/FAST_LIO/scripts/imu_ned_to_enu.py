#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Imu


class ImuNedToEnu(Node):
    def __init__(self) -> None:
        super().__init__("imu_ned_to_enu")

        self.declare_parameter("input_topic", "/ap/imu/experimental/data")
        self.declare_parameter("output_topic", "/ap/imu/enu")
        self.declare_parameter("output_frame_id", "body")
        self.declare_parameter("default_covariance", 0.01)
        self.declare_parameter("yaw_correction", 0.0)

        input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.output_frame_id = self.get_parameter("output_frame_id").get_parameter_value().string_value
        self.default_covariance = (
            self.get_parameter("default_covariance").get_parameter_value().double_value
        )
        self.yaw_correction = (
            self.get_parameter("yaw_correction").get_parameter_value().double_value
        )

        qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub = self.create_subscription(Imu, input_topic, self.imu_callback, qos)
        self.pub = self.create_publisher(Imu, output_topic, qos)

        self.get_logger().info(
            "imu_ned_to_enu started: "
            f"input={input_topic}, output={output_topic}, frame={self.output_frame_id}, "
            f"yaw_correction={self.yaw_correction:.6f} rad"
        )

    def _fallback_covariance(self) -> List[float]:
        return [self.default_covariance, 0.0, 0.0, 0.0, self.default_covariance, 0.0, 0.0, 0.0, self.default_covariance]

    def _sanitize_covariance(self, covariance: List[float]) -> List[float]:
        if len(covariance) != 9:
            return self._fallback_covariance()
        if covariance[0] < 0.0:
            return self._fallback_covariance()
        if any(math.isnan(v) or math.isinf(v) for v in covariance):
            return self._fallback_covariance()
        return list(covariance)

    def _normalize_quaternion(self, x: float, y: float, z: float, w: float) -> List[float]:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-9:
            return [0.0, 0.0, 0.0, 1.0]
        return [x / norm, y / norm, z / norm, w / norm]

    @staticmethod
    def _quat_multiply(
        q1x: float, q1y: float, q1z: float, q1w: float,
        q2x: float, q2y: float, q2z: float, q2w: float
    ) -> List[float]:
        return [
            q1w * q2x + q1x * q2w + q1y * q2z - q1z * q2y,
            q1w * q2y - q1x * q2z + q1y * q2w + q1z * q2x,
            q1w * q2z + q1x * q2y - q1y * q2x + q1z * q2w,
            q1w * q2w - q1x * q2x - q1y * q2y - q1z * q2z,
        ]

    @staticmethod
    def _rotate_xy(x: float, y: float, yaw: float) -> List[float]:
        c = math.cos(yaw)
        s = math.sin(yaw)
        return [x * c - y * s, x * s + y * c]

    def imu_callback(self, msg: Imu) -> None:
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id

        # Step 1: NED -> ENU for vectors: x_e=y_n, y_e=x_n, z_e=-z_n
        ax_enu = msg.linear_acceleration.y
        ay_enu = msg.linear_acceleration.x
        az_enu = -msg.linear_acceleration.z

        wx_enu = msg.angular_velocity.y
        wy_enu = msg.angular_velocity.x
        wz_enu = -msg.angular_velocity.z

        # Step 2: Correct fixed yaw installation offset (sensor->body)
        ax_rot, ay_rot = self._rotate_xy(ax_enu, ay_enu, self.yaw_correction)
        wx_rot, wy_rot = self._rotate_xy(wx_enu, wy_enu, self.yaw_correction)

        out.linear_acceleration.x = ax_rot
        out.linear_acceleration.y = ay_rot
        out.linear_acceleration.z = az_enu

        out.angular_velocity.x = wx_rot
        out.angular_velocity.y = wy_rot
        out.angular_velocity.z = wz_enu

        # Base NED->ENU quaternion conversion consistent with axis mapping.
        qx, qy, qz, qw = self._normalize_quaternion(
            msg.orientation.y,
            msg.orientation.x,
            -msg.orientation.z,
            msg.orientation.w,
        )

        # Apply yaw correction quaternion: q_out = q_corr * q_enu
        half = self.yaw_correction * 0.5
        qc_x = 0.0
        qc_y = 0.0
        qc_z = math.sin(half)
        qc_w = math.cos(half)
        qox, qoy, qoz, qow = self._quat_multiply(qc_x, qc_y, qc_z, qc_w, qx, qy, qz, qw)
        qox, qoy, qoz, qow = self._normalize_quaternion(qox, qoy, qoz, qow)

        out.orientation.x = qox
        out.orientation.y = qoy
        out.orientation.z = qoz
        out.orientation.w = qow

        out.orientation_covariance = self._sanitize_covariance(list(msg.orientation_covariance))
        out.angular_velocity_covariance = self._sanitize_covariance(list(msg.angular_velocity_covariance))
        out.linear_acceleration_covariance = self._sanitize_covariance(list(msg.linear_acceleration_covariance))

        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuNedToEnu()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
