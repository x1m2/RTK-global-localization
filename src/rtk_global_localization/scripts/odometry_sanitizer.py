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


def yaw_from_quaternion(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class OdometrySanitizer(Node):
    def __init__(self) -> None:
        super().__init__("odometry_sanitizer")

        self.declare_parameter("input_topic", "/Odometry")
        self.declare_parameter("output_topic", "/rtk/fastlio/odom")
        self.declare_parameter("output_frame_id", "odom")
        self.declare_parameter("output_child_frame_id", "body")
        self.declare_parameter("restamp_to_now", False)
        self.declare_parameter("xy_variance", 0.05 * 0.05)
        self.declare_parameter("z_variance", 1.0)
        self.declare_parameter("roll_pitch_variance", 1.0)
        self.declare_parameter("yaw_variance", 0.05 * 0.05)
        self.declare_parameter("twist_linear_variance", 0.25)
        self.declare_parameter("twist_angular_variance", 0.10)
        self.declare_parameter("minimum_variance", 1.0e-6)
        self.declare_parameter("force_covariance", True)
        self.declare_parameter("estimate_twist_from_pose", True)
        self.declare_parameter("reject_kinematic_outliers", False)
        self.declare_parameter("max_linear_speed", 8.0)
        self.declare_parameter("max_position_step", 3.0)
        self.declare_parameter("max_yaw_rate", 2.5)
        self.declare_parameter("log_every_n", 100)
        self.declare_parameter("output_qos_reliability", "reliable")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.output_child_frame_id = str(self.get_parameter("output_child_frame_id").value)
        self.restamp_to_now = bool(self.get_parameter("restamp_to_now").value)
        self.xy_variance = float(self.get_parameter("xy_variance").value)
        self.z_variance = float(self.get_parameter("z_variance").value)
        self.roll_pitch_variance = float(self.get_parameter("roll_pitch_variance").value)
        self.yaw_variance = float(self.get_parameter("yaw_variance").value)
        self.twist_linear_variance = float(self.get_parameter("twist_linear_variance").value)
        self.twist_angular_variance = float(self.get_parameter("twist_angular_variance").value)
        self.minimum_variance = float(self.get_parameter("minimum_variance").value)
        self.force_covariance = bool(self.get_parameter("force_covariance").value)
        self.estimate_twist_from_pose = bool(
            self.get_parameter("estimate_twist_from_pose").value
        )
        self.reject_kinematic_outliers = bool(
            self.get_parameter("reject_kinematic_outliers").value
        )
        self.max_linear_speed = float(self.get_parameter("max_linear_speed").value)
        self.max_position_step = float(self.get_parameter("max_position_step").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.log_every_n = max(1, int(self.get_parameter("log_every_n").value))
        self.output_qos_reliability = str(
            self.get_parameter("output_qos_reliability").value
        ).strip().lower()

        self.received = 0
        self.published = 0
        self.dropped = 0
        self.last_published_pose = None
        self.last_published_time = None
        self.last_input_pose = None
        self.last_input_time = None

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
            "Odometry sanitizer started: "
            f"{self.input_topic} -> {self.output_topic}, frame={self.output_frame_id}, "
            f"child={self.output_child_frame_id}, restamp_to_now={self.restamp_to_now}, "
            f"estimate_twist_from_pose={self.estimate_twist_from_pose}, "
            f"reject_kinematic_outliers={self.reject_kinematic_outliers}, "
            f"output_qos={self.output_qos_reliability}"
        )

    @staticmethod
    def _covariance_is_usable(covariance, indices) -> bool:
        for idx in indices:
            value = covariance[idx]
            if not math.isfinite(value) or value <= 0.0:
                return False
        return True

    def _fill_pose_covariance(self, odom: Odometry) -> None:
        if (
            not self.force_covariance
            and self._covariance_is_usable(odom.pose.covariance, (0, 7, 14, 21, 28, 35))
        ):
            return

        cov = [0.0] * 36
        cov[0] = max(self.xy_variance, self.minimum_variance)
        cov[7] = max(self.xy_variance, self.minimum_variance)
        cov[14] = max(self.z_variance, self.minimum_variance)
        cov[21] = max(self.roll_pitch_variance, self.minimum_variance)
        cov[28] = max(self.roll_pitch_variance, self.minimum_variance)
        cov[35] = max(self.yaw_variance, self.minimum_variance)
        odom.pose.covariance = cov

    def _fill_twist_covariance(self, odom: Odometry) -> None:
        if (
            not self.force_covariance
            and self._covariance_is_usable(odom.twist.covariance, (0, 7, 14, 21, 28, 35))
        ):
            return

        cov = [0.0] * 36
        cov[0] = max(self.twist_linear_variance, self.minimum_variance)
        cov[7] = max(self.twist_linear_variance, self.minimum_variance)
        cov[14] = max(self.twist_linear_variance, self.minimum_variance)
        cov[21] = max(self.twist_angular_variance, self.minimum_variance)
        cov[28] = max(self.twist_angular_variance, self.minimum_variance)
        cov[35] = max(self.twist_angular_variance, self.minimum_variance)
        odom.twist.covariance = cov

    @staticmethod
    def _body_frame_velocity(dx: float, dy: float, yaw: float, dt: float) -> tuple[float, float]:
        vx_world = dx / dt
        vy_world = dy / dt
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        vx_body = cos_yaw * vx_world + sin_yaw * vy_world
        vy_body = -sin_yaw * vx_world + cos_yaw * vy_world
        return vx_body, vy_body

    def _estimate_twist_from_pose(self, odom: Odometry) -> None:
        if not self.estimate_twist_from_pose:
            return

        current_time = self._stamp_seconds(odom)
        current_pose = odom.pose.pose

        if self.last_input_pose is None or self.last_input_time is None:
            self.last_input_pose = current_pose
            self.last_input_time = current_time
            return

        dt = current_time - self.last_input_time
        if dt <= 1.0e-4:
            return

        current_yaw = yaw_from_quaternion(current_pose.orientation)
        last_yaw = yaw_from_quaternion(self.last_input_pose.orientation)
        dx = current_pose.position.x - self.last_input_pose.position.x
        dy = current_pose.position.y - self.last_input_pose.position.y
        dz = current_pose.position.z - self.last_input_pose.position.z

        vx_body, vy_body = self._body_frame_velocity(dx, dy, current_yaw, dt)
        odom.twist.twist.linear.x = vx_body
        odom.twist.twist.linear.y = vy_body
        odom.twist.twist.linear.z = dz / dt
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = normalize_angle(current_yaw - last_yaw) / dt

        self.last_input_pose = current_pose
        self.last_input_time = current_time

    def _stamp_seconds(self, odom: Odometry) -> float:
        stamp = odom.header.stamp
        stamp_seconds = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        if stamp_seconds > 0.0:
            return stamp_seconds
        now = self.get_clock().now().nanoseconds
        return float(now) * 1.0e-9

    def _is_kinematic_outlier(self, odom: Odometry) -> bool:
        if not self.reject_kinematic_outliers:
            return False
        if self.last_published_pose is None or self.last_published_time is None:
            return False

        current_time = self._stamp_seconds(odom)
        dt = current_time - self.last_published_time
        if dt <= 1.0e-4:
            return False

        p = odom.pose.pose.position
        last_p = self.last_published_pose.position
        dx = p.x - last_p.x
        dy = p.y - last_p.y
        dz = p.z - last_p.z
        distance = math.sqrt(dx * dx + dy * dy + dz * dz)
        speed = distance / dt

        yaw = yaw_from_quaternion(odom.pose.pose.orientation)
        last_yaw = yaw_from_quaternion(self.last_published_pose.orientation)
        yaw_rate = abs(normalize_angle(yaw - last_yaw)) / dt

        # A fixed step limit only makes sense for adjacent, high-rate samples.
        # After we drop one sample, the next candidate may be seconds later, so
        # judge longer gaps by speed or we can get stuck rejecting valid odom.
        distance_limit = max(self.max_position_step, self.max_linear_speed * dt)
        if distance > distance_limit or speed > self.max_linear_speed:
            self._warn_outlier(
                "position",
                dt,
                distance,
                speed,
                yaw_rate,
            )
            return True

        if yaw_rate > self.max_yaw_rate:
            self._warn_outlier("yaw", dt, distance, speed, yaw_rate)
            return True

        return False

    def _warn_outlier(
        self,
        reason: str,
        dt: float,
        distance: float,
        speed: float,
        yaw_rate: float,
    ) -> None:
        if self.dropped == 0 or self.dropped % self.log_every_n == 0:
            self.get_logger().warn(
                "Dropped FAST-LIO odometry kinematic outlier: "
                f"reason={reason}, dt={dt:.3f}s, step={distance:.3f}m, "
                f"speed={speed:.3f}m/s, yaw_rate={yaw_rate:.3f}rad/s, "
                f"limits=({self.max_position_step:.3f}m, "
                f"{self.max_linear_speed:.3f}m/s, {self.max_yaw_rate:.3f}rad/s)"
            )

    def _callback(self, msg: Odometry) -> None:
        self.received += 1
        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        if self.restamp_to_now:
            out.header.stamp = self.get_clock().now().to_msg()
        out.child_frame_id = self.output_child_frame_id
        out.pose = copy.deepcopy(msg.pose)
        out.twist = copy.deepcopy(msg.twist)

        self._estimate_twist_from_pose(out)
        self._fill_pose_covariance(out)
        self._fill_twist_covariance(out)

        if self._is_kinematic_outlier(out):
            self.dropped += 1
            return

        self.publisher.publish(out)
        self.published += 1
        self.last_published_pose = out.pose.pose
        self.last_published_time = self._stamp_seconds(out)

        if self.published == 1 or self.published % self.log_every_n == 0:
            p = out.pose.pose.position
            self.get_logger().info(
                f"Odom stats: received={self.received}, published={self.published}, "
                f"dropped={self.dropped}, x={p.x:.3f}, y={p.y:.3f}"
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdometrySanitizer()
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
