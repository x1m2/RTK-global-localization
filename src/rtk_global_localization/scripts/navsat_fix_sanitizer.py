#!/usr/bin/env python3

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix


class NavSatFixSanitizer(Node):
    def __init__(self) -> None:
        super().__init__("navsat_fix_sanitizer")

        self.declare_parameter("input_topic", "/ap/navsat")
        self.declare_parameter("output_topic", "/rtk/fix")
        self.declare_parameter("output_frame_id", "body")
        self.declare_parameter("restamp_to_now", False)
        self.declare_parameter("reject_no_fix", True)
        self.declare_parameter("reject_zero_latlon", True)
        self.declare_parameter("minimum_status", 0)
        self.declare_parameter("default_horizontal_variance", 0.05 * 0.05)
        self.declare_parameter("default_vertical_variance", 0.20 * 0.20)
        self.declare_parameter("minimum_variance", 1.0e-6)
        self.declare_parameter("maximum_variance", 25.0)
        self.declare_parameter("log_every_n", 100)

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.restamp_to_now = bool(self.get_parameter("restamp_to_now").value)
        self.reject_no_fix = bool(self.get_parameter("reject_no_fix").value)
        self.reject_zero_latlon = bool(self.get_parameter("reject_zero_latlon").value)
        self.minimum_status = int(self.get_parameter("minimum_status").value)
        self.default_horizontal_variance = float(
            self.get_parameter("default_horizontal_variance").value
        )
        self.default_vertical_variance = float(
            self.get_parameter("default_vertical_variance").value
        )
        self.minimum_variance = float(self.get_parameter("minimum_variance").value)
        self.maximum_variance = float(self.get_parameter("maximum_variance").value)
        self.log_every_n = max(1, int(self.get_parameter("log_every_n").value))

        self.received = 0
        self.published = 0
        self.dropped = 0
        self.covariance_repaired = 0

        self.publisher = self.create_publisher(NavSatFix, self.output_topic, qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            NavSatFix, self.input_topic, self._callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            "NavSatFix sanitizer started: "
            f"{self.input_topic} -> {self.output_topic}, frame={self.output_frame_id}, "
            f"restamp_to_now={self.restamp_to_now}"
        )

    @staticmethod
    def _finite(value: float) -> bool:
        return math.isfinite(float(value))

    def _valid_fix(self, msg: NavSatFix) -> bool:
        if not (self._finite(msg.latitude) and self._finite(msg.longitude) and self._finite(msg.altitude)):
            return False
        if self.reject_zero_latlon and abs(msg.latitude) < 1.0e-12 and abs(msg.longitude) < 1.0e-12:
            return False
        if self.reject_no_fix and int(msg.status.status) < self.minimum_status:
            return False
        return True

    def _repair_covariance(self, msg: NavSatFix) -> bool:
        repaired = False
        cov = list(msg.position_covariance)
        diag_indices = (0, 4, 8)
        defaults = (
            self.default_horizontal_variance,
            self.default_horizontal_variance,
            self.default_vertical_variance,
        )

        if msg.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            repaired = True

        for idx, default in zip(diag_indices, defaults):
            value = cov[idx]
            if not math.isfinite(value) or value <= 0.0:
                cov[idx] = default
                repaired = True
            cov[idx] = min(max(cov[idx], self.minimum_variance), self.maximum_variance)

        # Keep only diagonal covariance if the source provided an unknown or zero matrix.
        if repaired:
            cov = [0.0] * 9
            cov[0] = min(max(defaults[0], self.minimum_variance), self.maximum_variance)
            cov[4] = min(max(defaults[1], self.minimum_variance), self.maximum_variance)
            cov[8] = min(max(defaults[2], self.minimum_variance), self.maximum_variance)
            msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        msg.position_covariance = cov
        return repaired

    def _callback(self, msg: NavSatFix) -> None:
        self.received += 1

        if not self._valid_fix(msg):
            self.dropped += 1
            if self.dropped == 1 or self.dropped % self.log_every_n == 0:
                self.get_logger().warn(
                    f"Dropped invalid RTK fix. received={self.received}, dropped={self.dropped}"
                )
            return

        out = NavSatFix()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        if self.restamp_to_now:
            out.header.stamp = self.get_clock().now().to_msg()
        out.status = msg.status
        out.latitude = msg.latitude
        out.longitude = msg.longitude
        out.altitude = msg.altitude
        out.position_covariance = list(msg.position_covariance)
        out.position_covariance_type = msg.position_covariance_type

        if self._repair_covariance(out):
            self.covariance_repaired += 1

        self.publisher.publish(out)
        self.published += 1

        if self.published == 1 or self.published % self.log_every_n == 0:
            self.get_logger().info(
                "RTK fix stats: "
                f"received={self.received}, published={self.published}, "
                f"dropped={self.dropped}, covariance_repaired={self.covariance_repaired}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavSatFixSanitizer()
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
