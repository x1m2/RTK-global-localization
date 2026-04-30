#!/usr/bin/env python3

import math
import os
from dataclasses import dataclass
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from tf2_ros import TransformBroadcaster


WGS84_A = 6378137.0
WGS84_E2 = 6.6943799901413165e-3


@dataclass
class MapOrigin:
    latitude: float
    longitude: float
    altitude: float
    yaw_enu: float


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


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def wgs84_to_ecef(
    latitude_deg: float,
    longitude_deg: float,
    altitude: float,
) -> Tuple[float, float, float]:
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


def ecef_to_wgs84(x: float, y: float, z: float) -> Tuple[float, float, float]:
    b = WGS84_A * math.sqrt(1.0 - WGS84_E2)
    ep2 = (WGS84_A * WGS84_A - b * b) / (b * b)
    p = math.hypot(x, y)
    theta = math.atan2(z * WGS84_A, p * b)
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)

    lon = math.atan2(y, x)
    lat = math.atan2(
        z + ep2 * b * sin_theta * sin_theta * sin_theta,
        p - WGS84_E2 * WGS84_A * cos_theta * cos_theta * cos_theta,
    )
    sin_lat = math.sin(lat)
    normal = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    altitude = p / math.cos(lat) - normal

    return math.degrees(lat), math.degrees(lon), altitude


def enu_delta_to_ecef_delta(
    east: float,
    north: float,
    up: float,
    origin_latitude_deg: float,
    origin_longitude_deg: float,
) -> Tuple[float, float, float]:
    lat = math.radians(origin_latitude_deg)
    lon = math.radians(origin_longitude_deg)
    sin_lat = math.sin(lat)
    cos_lat = math.cos(lat)
    sin_lon = math.sin(lon)
    cos_lon = math.cos(lon)

    dx = -sin_lon * east - sin_lat * cos_lon * north + cos_lat * cos_lon * up
    dy = cos_lon * east - sin_lat * sin_lon * north + cos_lat * sin_lon * up
    dz = cos_lat * north + sin_lat * up
    return dx, dy, dz


class SimulatedStartPose(Node):
    def __init__(self) -> None:
        super().__init__("simulated_start_pose")

        self.declare_parameter("map_origin_path", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "sim_body")
        self.declare_parameter("map_x", 0.0)
        self.declare_parameter("map_y", 0.0)
        self.declare_parameter("map_z", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("rate", 2.0)
        self.declare_parameter("odom_topic", "/simulated_start/odom")
        self.declare_parameter("point_topic", "/simulated_start/point")
        self.declare_parameter("navsat_topic", "/simulated_start/navsat")
        self.declare_parameter("log_every_n", 10)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.map_x = float(self.get_parameter("map_x").value)
        self.map_y = float(self.get_parameter("map_y").value)
        self.map_z = float(self.get_parameter("map_z").value)
        self.yaw = normalize_angle(float(self.get_parameter("yaw").value))
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.rate = max(0.1, float(self.get_parameter("rate").value))
        self.log_every_n = max(1, int(self.get_parameter("log_every_n").value))

        self.origin = self._load_origin()
        self.origin_ecef = wgs84_to_ecef(
            self.origin.latitude,
            self.origin.longitude,
            self.origin.altitude,
        )

        self.odom_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            10,
        )
        self.point_publisher = self.create_publisher(
            PointStamped,
            str(self.get_parameter("point_topic").value),
            10,
        )
        self.navsat_publisher = self.create_publisher(
            NavSatFix,
            str(self.get_parameter("navsat_topic").value),
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_count = 0

        self.add_on_set_parameters_callback(self._parameters_callback)
        self.timer = self.create_timer(1.0 / self.rate, self._publish)

        self.get_logger().info(
            "Simulated start pose started: "
            f"map=({self.map_x:.3f}, {self.map_y:.3f}, {self.map_z:.3f}), "
            f"yaw={self.yaw:.4f}, frame={self.map_frame}->{self.base_frame}, "
            f"origin=({self.origin.latitude:.12f}, {self.origin.longitude:.12f}, "
            f"{self.origin.altitude:.3f}), origin_yaw_enu={self.origin.yaw_enu:.6f}"
        )

    def _load_origin(self) -> MapOrigin:
        path = os.path.expanduser(str(self.get_parameter("map_origin_path").value))
        if not path:
            raise RuntimeError("map_origin_path must not be empty")
        if not os.path.exists(path):
            raise RuntimeError(f"map_origin_path does not exist: {path}")

        values = parse_flat_origin_file(path)
        try:
            return MapOrigin(
                latitude=float(values["latitude"]),
                longitude=float(values["longitude"]),
                altitude=float(values.get("altitude", 0.0)),
                yaw_enu=normalize_angle(float(values["yaw_enu"])),
            )
        except KeyError as exc:
            raise RuntimeError(f"map_origin_path is missing required key: {exc}") from exc

    def _parameters_callback(self, parameters):
        for parameter in parameters:
            if parameter.name == "map_x":
                self.map_x = float(parameter.value)
            elif parameter.name == "map_y":
                self.map_y = float(parameter.value)
            elif parameter.name == "map_z":
                self.map_z = float(parameter.value)
            elif parameter.name == "yaw":
                self.yaw = normalize_angle(float(parameter.value))
            elif parameter.name == "publish_tf":
                self.publish_tf = bool(parameter.value)

        return SetParametersResult(successful=True)

    def _publish(self) -> None:
        stamp = self.get_clock().now().to_msg()
        orientation = quaternion_from_yaw(self.yaw)
        latitude, longitude, altitude, east, north = self._map_to_wgs84()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.map_x
        odom.pose.pose.position.y = self.map_y
        odom.pose.pose.position.z = self.map_z
        odom.pose.pose.orientation = orientation
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 0.01
        odom.pose.covariance[35] = 0.0025
        self.odom_publisher.publish(odom)

        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = self.map_frame
        point.point.x = self.map_x
        point.point.y = self.map_y
        point.point.z = self.map_z
        self.point_publisher.publish(point)

        navsat = NavSatFix()
        navsat.header.stamp = stamp
        navsat.header.frame_id = self.map_frame
        navsat.status.status = NavSatStatus.STATUS_FIX
        navsat.status.service = NavSatStatus.SERVICE_GPS
        navsat.latitude = latitude
        navsat.longitude = longitude
        navsat.altitude = altitude
        navsat.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.navsat_publisher.publish(navsat)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.map_frame
            transform.child_frame_id = self.base_frame
            transform.transform.translation.x = self.map_x
            transform.transform.translation.y = self.map_y
            transform.transform.translation.z = self.map_z
            transform.transform.rotation = orientation
            self.tf_broadcaster.sendTransform(transform)

        self.publish_count += 1
        if self.publish_count == 1 or self.publish_count % self.log_every_n == 0:
            self.get_logger().info(
                "Simulated current pose: "
                f"map=({self.map_x:.3f}, {self.map_y:.3f}, {self.map_z:.3f}), "
                f"yaw={self.yaw:.4f}, enu=({east:.3f}E, {north:.3f}N), "
                f"lat={latitude:.12f}, lon={longitude:.12f}, alt={altitude:.3f}"
            )

    def _map_to_wgs84(self) -> Tuple[float, float, float, float, float]:
        yaw = self.origin.yaw_enu
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        east = cos_yaw * self.map_x - sin_yaw * self.map_y
        north = sin_yaw * self.map_x + cos_yaw * self.map_y
        up = self.map_z

        dx, dy, dz = enu_delta_to_ecef_delta(
            east,
            north,
            up,
            self.origin.latitude,
            self.origin.longitude,
        )
        latitude, longitude, altitude = ecef_to_wgs84(
            self.origin_ecef[0] + dx,
            self.origin_ecef[1] + dy,
            self.origin_ecef[2] + dz,
        )
        return latitude, longitude, altitude, east, north


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = SimulatedStartPose()
    except Exception as exc:
        if rclpy.ok():
            tmp_node = rclpy.create_node("simulated_start_pose_startup_error")
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
