#!/usr/bin/env python3

import math
import os
from dataclasses import dataclass
from typing import Dict, Tuple

import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


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


class MapPointToNavSat(Node):
    def __init__(self) -> None:
        super().__init__("map_point_to_navsat")

        self.declare_parameter("input_topic", "/clicked_point")
        self.declare_parameter("output_topic", "/clicked_point/navsat")
        self.declare_parameter("map_origin_path", "")
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("output_frame_id", "map")

        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.map_frame = str(self.get_parameter("map_frame").value)
        self.output_frame_id = str(self.get_parameter("output_frame_id").value)

        self.origin = self._load_origin()
        self.origin_ecef = wgs84_to_ecef(
            self.origin.latitude,
            self.origin.longitude,
            self.origin.altitude,
        )

        self.publisher = self.create_publisher(NavSatFix, self.output_topic, 10)
        self.subscription = self.create_subscription(
            PointStamped,
            self.input_topic,
            self._point_callback,
            10,
        )

        self.get_logger().info(
            "Map point to NavSat converter started: "
            f"input={self.input_topic}, output={self.output_topic}, map_frame={self.map_frame}, "
            f"origin=({self.origin.latitude:.12f}, {self.origin.longitude:.12f}, "
            f"{self.origin.altitude:.3f}), yaw_enu={self.origin.yaw_enu:.6f}"
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

    def _point_callback(self, msg: PointStamped) -> None:
        frame_id = msg.header.frame_id.strip()
        if frame_id and frame_id != self.map_frame:
            self.get_logger().warn(
                f"Ignoring point in frame '{frame_id}'. Expected '{self.map_frame}'."
            )
            return

        latitude, longitude, altitude, east, north = self._map_to_wgs84(
            msg.point.x,
            msg.point.y,
            msg.point.z,
        )

        out = NavSatFix()
        out.header = msg.header
        out.header.frame_id = self.output_frame_id
        out.status.status = NavSatStatus.STATUS_FIX
        out.status.service = NavSatStatus.SERVICE_GPS
        out.latitude = latitude
        out.longitude = longitude
        out.altitude = altitude
        out.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

        self.publisher.publish(out)
        self.get_logger().info(
            "Clicked map point -> WGS84: "
            f"map=({msg.point.x:.3f}, {msg.point.y:.3f}, {msg.point.z:.3f}), "
            f"enu=({east:.3f}E, {north:.3f}N), "
            f"lat={latitude:.12f}, lon={longitude:.12f}, alt={altitude:.3f}"
        )

    def _map_to_wgs84(
        self,
        map_x: float,
        map_y: float,
        map_z: float,
    ) -> Tuple[float, float, float, float, float]:
        yaw = self.origin.yaw_enu
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        east = cos_yaw * map_x - sin_yaw * map_y
        north = sin_yaw * map_x + cos_yaw * map_y
        up = map_z

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
        node = MapPointToNavSat()
    except Exception as exc:
        if rclpy.ok():
            tmp_node = rclpy.create_node("map_point_to_navsat_startup_error")
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
