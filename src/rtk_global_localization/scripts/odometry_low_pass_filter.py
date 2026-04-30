#!/usr/bin/env python3

import copy
import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)


class OdometryLowPassFilter(Node):
    def __init__(self) -> None:
        super().__init__("odometry_low_pass_filter")

        self.declare_parameter("input_topic", "/odometry/gps")
        self.declare_parameter("output_topic", "/odometry/gps/smoothed")
        self.declare_parameter("alpha", 0.15)
        self.declare_parameter("position_variance_floor", 0.0)
        self.declare_parameter("log_every_n", 100)
        self.declare_parameter("output_qos_reliability", "reliable")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.alpha = min(max(float(self.get_parameter("alpha").value), 0.0), 1.0)
        self.position_variance_floor = max(
            0.0, float(self.get_parameter("position_variance_floor").value)
        )
        self.log_every_n = max(1, int(self.get_parameter("log_every_n").value))
        self.output_qos_reliability = str(
            self.get_parameter("output_qos_reliability").value
        ).strip().lower()

        self.received = 0
        self.published = 0
        self.filtered_x = None
        self.filtered_y = None
        self.filtered_z = None

        publisher_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=(
                ReliabilityPolicy.BEST_EFFORT
                if self.output_qos_reliability == "best_effort"
                else ReliabilityPolicy.RELIABLE
            ),
        )
        self.publisher = self.create_publisher(Odometry, self.output_topic, publisher_qos)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self._callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            "Odometry low-pass filter started: "
            f"{self.input_topic} -> {self.output_topic}, alpha={self.alpha:.3f}, "
            f"position_variance_floor={self.position_variance_floor:.3f}, "
            f"output_qos={self.output_qos_reliability}"
        )

    def _inflate_position_covariance(self, odom: Odometry) -> None:
        if self.position_variance_floor <= 0.0:
            return
        for idx in (0, 7, 14):
            current = odom.pose.covariance[idx]
            if not math.isfinite(current) or current < self.position_variance_floor:
                odom.pose.covariance[idx] = self.position_variance_floor

    def _callback(self, msg: Odometry) -> None:
        self.received += 1

        p = msg.pose.pose.position
        raw_x = float(p.x)
        raw_y = float(p.y)
        raw_z = float(p.z)
        if self.filtered_x is None:
            self.filtered_x = raw_x
            self.filtered_y = raw_y
            self.filtered_z = raw_z
        else:
            self.filtered_x += self.alpha * (raw_x - self.filtered_x)
            self.filtered_y += self.alpha * (raw_y - self.filtered_y)
            self.filtered_z += self.alpha * (raw_z - self.filtered_z)

        out = Odometry()
        out.header = msg.header
        out.child_frame_id = msg.child_frame_id
        out.pose = copy.deepcopy(msg.pose)
        out.twist = copy.deepcopy(msg.twist)
        out.pose.pose.position.x = self.filtered_x
        out.pose.pose.position.y = self.filtered_y
        out.pose.pose.position.z = self.filtered_z
        self._inflate_position_covariance(out)

        self.publisher.publish(out)
        self.published += 1

        if self.published == 1 or self.published % self.log_every_n == 0:
            step = math.hypot(raw_x - self.filtered_x, raw_y - self.filtered_y)
            self.get_logger().info(
                "GPS odom smoothing stats: "
                f"received={self.received}, published={self.published}, "
                f"raw=({raw_x:.3f}, {raw_y:.3f}), "
                f"filtered=({self.filtered_x:.3f}, {self.filtered_y:.3f}), "
                f"residual={step:.3f}m"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdometryLowPassFilter()
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
