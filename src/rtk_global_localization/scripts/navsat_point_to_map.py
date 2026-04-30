#!/usr/bin/env python3

import math
import os
from dataclasses import dataclass
from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PointStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
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


class NavSatPointToMap(Node):
    def __init__(self) -> None:
        super().__init__("navsat_point_to_map")

        self.declare_parameter("map_origin_path", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("child_frame", "manual_navsat")
        self.declare_parameter("latitude", float("nan"))
        self.declare_parameter("longitude", float("nan"))
        self.declare_parameter("altitude", float("nan"))
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("rate", 2.0)
        self.declare_parameter("fix_topic", "/manual_navsat/fix")
        self.declare_parameter("point_topic", "/manual_navsat/point")
        self.declare_parameter("odom_topic", "/manual_navsat/odom")
        self.declare_parameter("log_every_n", 10)

        self.map_frame = str(self.get_parameter("map_frame").value)
        self.child_frame = str(self.get_parameter("child_frame").value)
        self.latitude = float(self.get_parameter("latitude").value)
        self.longitude = float(self.get_parameter("longitude").value)
        self.altitude = float(self.get_parameter("altitude").value)
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

        self.point_publisher = self.create_publisher(
            PointStamped,
            str(self.get_parameter("point_topic").value),
            10,
        )
        self.odom_publisher = self.create_publisher(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            10,
        )
        self.fix_subscription = self.create_subscription(
            NavSatFix,
            str(self.get_parameter("fix_topic").value),
            self._fix_callback,
            10,
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.publish_count = 0

        self.add_on_set_parameters_callback(self._parameters_callback)
        self.timer = self.create_timer(1.0 / self.rate, self._publish)

        self.get_logger().info(
            "NavSat point to map converter started: "
            f"fix_topic={self.get_parameter('fix_topic').value}, frame={self.map_frame}, "
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
            if parameter.name == "latitude":
                self.latitude = float(parameter.value)
            elif parameter.name == "longitude":
                self.longitude = float(parameter.value)
            elif parameter.name == "altitude":
                self.altitude = float(parameter.value)
            elif parameter.name == "yaw":
                self.yaw = normalize_angle(float(parameter.value))
            elif parameter.name == "publish_tf":
                self.publish_tf = bool(parameter.value)

        return SetParametersResult(successful=True)

    def _fix_callback(self, msg: NavSatFix) -> None:
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
        self._publish()

    def _publish(self) -> None:
        if not (math.isfinite(self.latitude) and math.isfinite(self.longitude)):
            return
        altitude = self.altitude if math.isfinite(self.altitude) else self.origin.altitude
        map_x, map_y, map_z, east, north = self._navsat_to_map(
            self.latitude,
            self.longitude,
            altitude,
        )
        stamp = self.get_clock().now().to_msg()
        orientation = quaternion_from_yaw(self.yaw)

        point = PointStamped()
        point.header.stamp = stamp
        point.header.frame_id = self.map_frame
        point.point.x = map_x
        point.point.y = map_y
        point.point.z = map_z
        self.point_publisher.publish(point)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.map_frame
        odom.child_frame_id = self.child_frame
        odom.pose.pose.position.x = map_x
        odom.pose.pose.position.y = map_y
        odom.pose.pose.position.z = map_z
        odom.pose.pose.orientation = orientation
        odom.pose.covariance[0] = 0.01
        odom.pose.covariance[7] = 0.01
        odom.pose.covariance[14] = 0.01
        odom.pose.covariance[35] = 0.0025
        self.odom_publisher.publish(odom)

        if self.publish_tf:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self.map_frame
            transform.child_frame_id = self.child_frame
            transform.transform.translation.x = map_x
            transform.transform.translation.y = map_y
            transform.transform.translation.z = map_z
            transform.transform.rotation = orientation
            self.tf_broadcaster.sendTransform(transform)

        self.publish_count += 1
        if self.publish_count == 1 or self.publish_count % self.log_every_n == 0:
            self.get_logger().info(
                "NavSat point -> map: "
                f"lat={self.latitude:.12f}, lon={self.longitude:.12f}, alt={altitude:.3f}, "
                f"enu=({east:.3f}E, {north:.3f}N), "
                f"map=({map_x:.3f}, {map_y:.3f}, {map_z:.3f})"
            )

    def _navsat_to_map(
        self,
        latitude: float,
        longitude: float,
        altitude: float,
    ) -> Tuple[float, float, float, float, float]:
        ecef = wgs84_to_ecef(latitude, longitude, altitude)
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
        return map_x, map_y, up, east, north


def main(args=None) -> None:
    rclpy.init(args=args)
    try:
        node = NavSatPointToMap()
    except Exception as exc:
        if rclpy.ok():
            tmp_node = rclpy.create_node("navsat_point_to_map_startup_error")
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
