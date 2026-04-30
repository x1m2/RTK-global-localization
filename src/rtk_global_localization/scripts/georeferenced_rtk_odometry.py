#!/usr/bin/env python3

import math
import os
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix


WGS84_A = 6378137.0
WGS84_E2 = 6.6943799901413165e-3


@dataclass
class MapOrigin:
    latitude: float
    longitude: float
    altitude: float
    yaw_enu: float


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def wgs84_to_ecef(latitude_deg: float, longitude_deg: float, altitude: float) -> Tuple[float, float, float]:
    lat = math.radians(latitude_deg)
    lon = math.radians(longitude_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)
    normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)

    x = (normal + altitude) * cos_lat * cos_lon
    y = (normal + altitude) * cos_lat * sin_lon
    z = (normal * (1.0 - WGS84_E2) + altitude) * sin_lat
    return x, y, z


def ecef_delta_to_enu(
    dx: float,
    dy: float,
    dz: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
) -> Tuple[float, float, float]:
    lat = math.radians(origin_latitude_deg)
    lon = math.radians(origin_longitude_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def parse_flat_origin_file(path: str) -> Dict[str, str]:
    values: Dict[str, str] = {}
    with open(path, "r", encoding="utf-8") as origin_file:
        for raw_line in origin_file:
            line = raw_line.split("#", 1)[0].strip()
            if not line or ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip().strip("'\"")
    return values


class GeoreferencedRtkOdometry(Node):
    def __init__(self) -> None:
        super().__init__("georeferenced_rtk_odometry")

        self.declare_parameter("fix_topic", "/rtk/fix")
        self.declare_parameter("heading_topic", "/rtk/navsat_heading")
        self.declare_parameter("output_topic", "/rtk/map_origin/odom")
        self.declare_parameter("map_origin_path", "")
        self.declare_parameter("origin_latitude", float("nan"))
        self.declare_parameter("origin_longitude", float("nan"))
        self.declare_parameter("origin_altitude", 0.0)
        self.declare_parameter("origin_yaw_enu", float("nan"))
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("child_frame_id", "body")
        self.declare_parameter("restamp_to_now", False)
        self.declare_parameter("require_heading", True)
        self.declare_parameter("heading_yaw_offset", 0.0)
        self.declare_parameter("gps_x", 0.0)
        self.declare_parameter("gps_y", 0.0)
        self.declare_parameter("gps_z", 0.0)
        self.declare_parameter("position_variance_floor", 1.0e-4)
        self.declare_parameter("yaw_variance", 0.03 * 0.03)
        self.declare_parameter("roll_pitch_variance", 999.0)
        self.declare_parameter("log_every_n", 100)

        self.fix_topic = str(self.get_parameter("fix_topic").value)
        self.heading_topic = str(self.get_parameter("heading_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)
        self.child_frame_id = str(self.get_parameter("child_frame_id").value)
        self.restamp_to_now = bool(self.get_parameter("restamp_to_now").value)
        self.require_heading = bool(self.get_parameter("require_heading").value)
        self.heading_yaw_offset = float(self.get_parameter("heading_yaw_offset").value)
        self.gps_x = float(self.get_parameter("gps_x").value)
        self.gps_y = float(self.get_parameter("gps_y").value)
        self.gps_z = float(self.get_parameter("gps_z").value)
        self.position_variance_floor = float(
            self.get_parameter("position_variance_floor").value
        )
        self.yaw_variance = float(self.get_parameter("yaw_variance").value)
        self.roll_pitch_variance = float(self.get_parameter("roll_pitch_variance").value)
        self.log_every_n = max(1, int(self.get_parameter("log_every_n").value))

        self.origin = self._load_origin()
        self.origin_ecef = wgs84_to_ecef(
            self.origin.latitude,
            self.origin.longitude,
            self.origin.altitude,
        )

        self.latest_heading_yaw: Optional[float] = None
        self.received_fix = 0
        self.published = 0
        self.dropped_no_heading = 0

        self.publisher = self.create_publisher(Odometry, self.output_topic, qos_profile_sensor_data)
        self.fix_subscription = self.create_subscription(
            NavSatFix, self.fix_topic, self._fix_callback, qos_profile_sensor_data
        )
        self.heading_subscription = self.create_subscription(
            Imu, self.heading_topic, self._heading_callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            "Georeferenced RTK odometry started: "
            f"fix={self.fix_topic}, heading={self.heading_topic}, output={self.output_topic}, "
            f"origin=({self.origin.latitude:.9f}, {self.origin.longitude:.9f}, "
            f"{self.origin.altitude:.3f}), origin_yaw_enu={self.origin.yaw_enu:.6f}, "
            f"gps_offset=({self.gps_x:.3f}, {self.gps_y:.3f}, {self.gps_z:.3f})"
        )

    def _load_origin(self) -> MapOrigin:
        path = str(self.get_parameter("map_origin_path").value)
        values: Dict[str, str] = {}
        if path:
            expanded = os.path.expanduser(path)
            if os.path.exists(expanded):
                values = parse_flat_origin_file(expanded)
                self.get_logger().info(f"Loaded map origin file: {expanded}")
            else:
                self.get_logger().warn(
                    f"Map origin file does not exist: {expanded}. Falling back to parameters."
                )

        latitude = float(values.get("latitude", self.get_parameter("origin_latitude").value))
        longitude = float(values.get("longitude", self.get_parameter("origin_longitude").value))
        altitude = float(values.get("altitude", self.get_parameter("origin_altitude").value))
        yaw_enu = float(values.get("yaw_enu", self.get_parameter("origin_yaw_enu").value))

        if not (math.isfinite(latitude) and math.isfinite(longitude) and math.isfinite(yaw_enu)):
            raise RuntimeError(
                "Map origin is incomplete. Provide map_origin_path with latitude, longitude, "
                "altitude, yaw_enu, or set origin_latitude/origin_longitude/origin_yaw_enu."
            )

        return MapOrigin(
            latitude=latitude,
            longitude=longitude,
            altitude=altitude,
            yaw_enu=normalize_angle(yaw_enu),
        )

    def _heading_callback(self, msg: Imu) -> None:
        self.latest_heading_yaw = normalize_angle(
            yaw_from_quaternion(msg.orientation) + self.heading_yaw_offset
        )

    def _fix_callback(self, msg: NavSatFix) -> None:
        self.received_fix += 1
        if self.require_heading and self.latest_heading_yaw is None:
            self.dropped_no_heading += 1
            if self.dropped_no_heading == 1 or self.dropped_no_heading % self.log_every_n == 0:
                self.get_logger().warn(
                    "Waiting for absolute heading before publishing map-origin odometry."
                )
            return

        map_x, map_y, map_z = self._fix_to_map(msg)
        yaw_map = 0.0
        if self.latest_heading_yaw is not None:
            yaw_map = normalize_angle(self.latest_heading_yaw - self.origin.yaw_enu)

        body_x, body_y, body_z = self._remove_gps_antenna_offset(
            map_x,
            map_y,
            map_z,
            yaw_map,
        )

        out = Odometry()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        if self.restamp_to_now:
            out.header.stamp = self.get_clock().now().to_msg()
        out.child_frame_id = self.child_frame_id
        out.pose.pose.position.x = body_x
        out.pose.pose.position.y = body_y
        out.pose.pose.position.z = body_z
        out.pose.pose.orientation = quaternion_from_yaw(yaw_map)
        out.pose.covariance = self._pose_covariance_in_map(msg)

        self.publisher.publish(out)
        self.published += 1
        if self.published == 1 or self.published % self.log_every_n == 0:
            self.get_logger().info(
                "Map-origin RTK odom stats: "
                f"received={self.received_fix}, published={self.published}, "
                f"x={body_x:.3f}, y={body_y:.3f}, yaw={yaw_map:.4f}"
            )

    def _fix_to_map(self, msg: NavSatFix) -> Tuple[float, float, float]:
        ecef = wgs84_to_ecef(msg.latitude, msg.longitude, msg.altitude)
        dx = ecef[0] - self.origin_ecef[0]
        dy = ecef[1] - self.origin_ecef[1]
        dz = ecef[2] - self.origin_ecef[2]
        east, north, up = ecef_delta_to_enu(
            dx,
            dy,
            dz,
            self.origin.latitude,
            self.origin.longitude,
        )

        yaw = self.origin.yaw_enu
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        map_x = cos_yaw * east + sin_yaw * north
        map_y = -sin_yaw * east + cos_yaw * north
        return map_x, map_y, up

    def _remove_gps_antenna_offset(
        self,
        gps_map_x: float,
        gps_map_y: float,
        gps_map_z: float,
        yaw_map: float,
    ) -> Tuple[float, float, float]:
        cos_yaw = math.cos(yaw_map)
        sin_yaw = math.sin(yaw_map)
        offset_x = cos_yaw * self.gps_x - sin_yaw * self.gps_y
        offset_y = sin_yaw * self.gps_x + cos_yaw * self.gps_y
        return gps_map_x - offset_x, gps_map_y - offset_y, gps_map_z - self.gps_z

    def _pose_covariance_in_map(self, msg: NavSatFix):
        cov = list(msg.position_covariance)
        east_var = self._variance(cov[0])
        north_var = self._variance(cov[4])
        up_var = self._variance(cov[8])
        east_north = cov[1] if math.isfinite(cov[1]) else 0.0

        yaw = self.origin.yaw_enu
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        # P_map = R(-origin_yaw) * P_enu * R(-origin_yaw)^T
        p_xx = (
            cos_yaw * cos_yaw * east_var
            + 2.0 * cos_yaw * sin_yaw * east_north
            + sin_yaw * sin_yaw * north_var
        )
        p_yy = (
            sin_yaw * sin_yaw * east_var
            - 2.0 * sin_yaw * cos_yaw * east_north
            + cos_yaw * cos_yaw * north_var
        )
        p_xy = (
            -sin_yaw * cos_yaw * east_var
            + (cos_yaw * cos_yaw - sin_yaw * sin_yaw) * east_north
            + sin_yaw * cos_yaw * north_var
        )

        pose_cov = [0.0] * 36
        pose_cov[0] = self._variance(p_xx)
        pose_cov[1] = p_xy
        pose_cov[6] = p_xy
        pose_cov[7] = self._variance(p_yy)
        pose_cov[14] = up_var
        pose_cov[21] = self.roll_pitch_variance
        pose_cov[28] = self.roll_pitch_variance
        pose_cov[35] = max(self.yaw_variance, self.position_variance_floor)
        return pose_cov

    def _variance(self, value: float) -> float:
        if not math.isfinite(value) or value <= 0.0:
            return self.position_variance_floor
        return max(value, self.position_variance_floor)


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = GeoreferencedRtkOdometry()
    except Exception as exc:
        if rclpy.ok():
            tmp_node = rclpy.create_node("georeferenced_rtk_odometry_startup_error")
            tmp_node.get_logger().error(str(exc))
            tmp_node.destroy_node()
            rclpy.shutdown()
        return

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
