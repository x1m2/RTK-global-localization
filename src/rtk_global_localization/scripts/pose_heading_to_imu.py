#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Quaternion
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class PoseHeadingToImu(Node):
    def __init__(self) -> None:
        super().__init__("pose_heading_to_imu")

        self.declare_parameter("input_topic", "/ap/pose/filtered")
        self.declare_parameter("output_topic", "/rtk/heading")
        self.declare_parameter("output_frame_id", "body")
        self.declare_parameter("restamp_to_now", False)
        self.declare_parameter("yaw_offset", 0.0)
        self.declare_parameter("publish_yaw_only", True)
        self.declare_parameter("yaw_variance", 0.05 * 0.05)
        self.declare_parameter("roll_pitch_variance", 999.0)
        self.declare_parameter("log_every_n", 100)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.restamp_to_now = bool(self.get_parameter("restamp_to_now").value)
        self.yaw_offset = float(self.get_parameter("yaw_offset").value)
        self.publish_yaw_only = bool(self.get_parameter("publish_yaw_only").value)
        self.yaw_variance = float(self.get_parameter("yaw_variance").value)
        self.roll_pitch_variance = float(self.get_parameter("roll_pitch_variance").value)
        self.log_every_n = max(1, int(self.get_parameter("log_every_n").value))

        self.received = 0
        self.published = 0

        self.publisher = self.create_publisher(Imu, self.output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PoseStamped, self.input_topic, self._callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            "Pose heading converter started: "
            f"{self.input_topic} -> {self.output_topic}, frame={self.output_frame_id}, "
            f"yaw_offset={self.yaw_offset:.6f}, yaw_only={self.publish_yaw_only}"
        )

    @staticmethod
    def _valid_quaternion(q: Quaternion) -> bool:
        norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        return math.isfinite(norm) and norm > 1.0e-6

    def _callback(self, msg: PoseStamped) -> None:
        self.received += 1
        if not self._valid_quaternion(msg.pose.orientation):
            self.get_logger().warn("Dropped pose heading with invalid quaternion.")
            return

        out = Imu()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        if self.restamp_to_now:
            out.header.stamp = self.get_clock().now().to_msg()

        if self.publish_yaw_only:
            yaw = yaw_from_quaternion(msg.pose.orientation) + self.yaw_offset
            out.orientation = quaternion_from_yaw(yaw)
        else:
            out.orientation = msg.pose.orientation

        out.orientation_covariance = [
            self.roll_pitch_variance, 0.0, 0.0,
            0.0, self.roll_pitch_variance, 0.0,
            0.0, 0.0, self.yaw_variance,
        ]
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0

        self.publisher.publish(out)
        self.published += 1

        if self.published == 1 or self.published % self.log_every_n == 0:
            yaw = yaw_from_quaternion(out.orientation)
            self.get_logger().info(
                f"Heading stats: received={self.received}, published={self.published}, yaw={yaw:.4f} rad"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PoseHeadingToImu()
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
