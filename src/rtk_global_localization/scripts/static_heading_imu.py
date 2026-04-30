#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class StaticHeadingImu(Node):
    def __init__(self) -> None:
        super().__init__("static_heading_imu")

        self.declare_parameter("output_topic", "/rtk/navsat_heading")
        self.declare_parameter("output_frame_id", "body")
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("yaw_variance", 0.03 * 0.03)
        self.declare_parameter("roll_pitch_variance", 999.0)
        self.declare_parameter("frequency", 10.0)

        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.yaw = float(self.get_parameter("yaw").value)
        self.yaw_variance = float(self.get_parameter("yaw_variance").value)
        self.roll_pitch_variance = float(self.get_parameter("roll_pitch_variance").value)
        frequency = max(0.5, float(self.get_parameter("frequency").value))

        self.publisher = self.create_publisher(Imu, self.output_topic, 10)
        self.timer = self.create_timer(1.0 / frequency, self._publish)
        self.published = 0

        self.get_logger().info(
            "Static heading IMU started: "
            f"{self.output_topic}, frame={self.output_frame_id}, yaw={self.yaw:.6f} rad"
        )

    def _publish(self) -> None:
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame_id
        msg.orientation = quaternion_from_yaw(self.yaw)
        msg.orientation_covariance = [
            self.roll_pitch_variance, 0.0, 0.0,
            0.0, self.roll_pitch_variance, 0.0,
            0.0, 0.0, self.yaw_variance,
        ]
        msg.angular_velocity_covariance[0] = -1.0
        msg.linear_acceleration_covariance[0] = -1.0

        self.publisher.publish(msg)
        self.published += 1

        if self.published == 1:
            self.get_logger().info(
                f"Published initial/static heading yaw={self.yaw:.4f} rad"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = StaticHeadingImu()
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
