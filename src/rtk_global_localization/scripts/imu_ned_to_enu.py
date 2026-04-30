#!/usr/bin/env python3

import math
from typing import List

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuNedToEnu(Node):
    def __init__(self) -> None:
        super().__init__("imu_ned_to_enu")

        self.declare_parameter("input_topic", "/ap/imu/experimental/data")
        self.declare_parameter("output_topic", "/rtk/imu/enu")
        self.declare_parameter("output_frame_id", "body")
        self.declare_parameter("restamp_to_now", False)
        self.declare_parameter("default_orientation_variance", 0.05 * 0.05)
        self.declare_parameter("default_angular_velocity_variance", 0.02 * 0.02)
        self.declare_parameter("default_linear_acceleration_variance", 0.20 * 0.20)
        self.declare_parameter("yaw_correction", 0.0)
        self.declare_parameter("output_qos_reliability", "reliable")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.restamp_to_now = bool(self.get_parameter("restamp_to_now").value)
        self.default_orientation_variance = float(
            self.get_parameter("default_orientation_variance").value
        )
        self.default_angular_velocity_variance = float(
            self.get_parameter("default_angular_velocity_variance").value
        )
        self.default_linear_acceleration_variance = float(
            self.get_parameter("default_linear_acceleration_variance").value
        )
        self.yaw_correction = float(self.get_parameter("yaw_correction").value)
        self.output_qos_reliability = str(
            self.get_parameter("output_qos_reliability").value
        ).strip().lower()

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
            "IMU NED->ENU converter started: "
            f"{self.input_topic} -> {self.output_topic}, frame={self.output_frame_id}, "
            f"yaw_correction={self.yaw_correction:.6f}, output_qos={self.output_qos_reliability}"
        )

    @staticmethod
    def _normalize_quaternion(x: float, y: float, z: float, w: float) -> List[float]:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1.0e-9 or not math.isfinite(norm):
            return [0.0, 0.0, 0.0, 1.0]
        return [x / norm, y / norm, z / norm, w / norm]

    @staticmethod
    def _quat_multiply(q1, q2) -> List[float]:
        x1, y1, z1, w1 = q1
        x2, y2, z2, w2 = q2
        return [
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ]

    @staticmethod
    def _rotate_xy(x: float, y: float, yaw: float) -> List[float]:
        c = math.cos(yaw)
        s = math.sin(yaw)
        return [x * c - y * s, x * s + y * c]

    @staticmethod
    def _ned_to_enu_quaternion(q_ned) -> List[float]:
        # Rotate attitude from NED into ENU. This fixed transform is equivalent
        # to yaw +90 deg followed by roll 180 deg.
        q_rot = [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]
        return ImuNedToEnu._quat_multiply(q_rot, q_ned)

    @staticmethod
    def _covariance(default_variance: float) -> List[float]:
        return [default_variance, 0.0, 0.0, 0.0, default_variance, 0.0, 0.0, 0.0, default_variance]

    def _sanitize_covariance(self, covariance, default_variance: float) -> List[float]:
        if len(covariance) != 9 or covariance[0] < 0.0:
            return self._covariance(default_variance)
        if any(not math.isfinite(v) for v in covariance):
            return self._covariance(default_variance)
        if covariance[0] == 0.0 and covariance[4] == 0.0 and covariance[8] == 0.0:
            return self._covariance(default_variance)
        return list(covariance)

    def _callback(self, msg: Imu) -> None:
        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        if self.restamp_to_now:
            out.header.stamp = self.get_clock().now().to_msg()

        # NED vector to ENU vector: x_e=y_n, y_e=x_n, z_e=-z_n.
        ax_enu = msg.linear_acceleration.y
        ay_enu = msg.linear_acceleration.x
        az_enu = -msg.linear_acceleration.z
        wx_enu = msg.angular_velocity.y
        wy_enu = msg.angular_velocity.x
        wz_enu = -msg.angular_velocity.z

        ax_rot, ay_rot = self._rotate_xy(ax_enu, ay_enu, self.yaw_correction)
        wx_rot, wy_rot = self._rotate_xy(wx_enu, wy_enu, self.yaw_correction)

        out.linear_acceleration.x = ax_rot
        out.linear_acceleration.y = ay_rot
        out.linear_acceleration.z = az_enu
        out.angular_velocity.x = wx_rot
        out.angular_velocity.y = wy_rot
        out.angular_velocity.z = wz_enu

        q_ned = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        q_enu = self._normalize_quaternion(*self._ned_to_enu_quaternion(q_ned))
        half = self.yaw_correction * 0.5
        q_corr = [0.0, 0.0, math.sin(half), math.cos(half)]
        q_out = self._normalize_quaternion(*self._quat_multiply(q_corr, q_enu))
        out.orientation.x, out.orientation.y, out.orientation.z, out.orientation.w = q_out

        out.orientation_covariance = self._sanitize_covariance(
            msg.orientation_covariance, self.default_orientation_variance
        )
        out.angular_velocity_covariance = self._sanitize_covariance(
            msg.angular_velocity_covariance, self.default_angular_velocity_variance
        )
        out.linear_acceleration_covariance = self._sanitize_covariance(
            msg.linear_acceleration_covariance, self.default_linear_acceleration_variance
        )

        self.publisher.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuNedToEnu()
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
