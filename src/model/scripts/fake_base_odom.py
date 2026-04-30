#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def _normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def _yaw_to_quaternion(yaw: float):
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


def _quaternion_to_yaw(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class FakeBaseOdom(Node):
    def __init__(self):
        super().__init__("fake_base_odom")

        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "body")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("publish_rate", 30.0)
        self.declare_parameter("initial_x", 0.0)
        self.declare_parameter("initial_y", 0.0)
        self.declare_parameter("initial_yaw", 0.0)

        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_rate = float(self.get_parameter("publish_rate").value)

        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0

        cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        odom_topic = self.get_parameter("odom_topic").value

        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, cmd_vel_topic, self._cmd_vel_callback, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._initialpose_callback, 10
        )

        self.last_time = self.get_clock().now()
        period = 1.0 / max(self.publish_rate, 1.0)
        self.timer = self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"Fake odom running: {self.odom_frame} -> {self.base_frame}, "
            f"cmd_vel={cmd_vel_topic}, odom={odom_topic}"
        )

    def _cmd_vel_callback(self, msg: Twist):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z

    def _initialpose_callback(self, msg: PoseWithCovarianceStamped):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.yaw = _quaternion_to_yaw(q.x, q.y, q.z, q.w)
        self.get_logger().info(
            f"Initial pose updated: x={self.x:.3f}, y={self.y:.3f}, yaw={self.yaw:.3f}"
        )

    def _on_timer(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now
        if dt <= 0.0:
            return
        dt = min(dt, 0.1)

        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)
        self.x += (self.vx * cos_yaw - self.vy * sin_yaw) * dt
        self.y += (self.vx * sin_yaw + self.vy * cos_yaw) * dt
        self.yaw = _normalize_angle(self.yaw + self.wz * dt)

        qx, qy, qz, qw = _yaw_to_quaternion(self.yaw)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.wz
        self.odom_pub.publish(odom)

        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FakeBaseOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
