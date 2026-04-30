#!/usr/bin/env python3

import math

import rclpy
from rcl_interfaces.srv import GetParameters
from robot_localization.srv import SetDatum
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


class MapAnchorInjector(Node):
    def __init__(self) -> None:
        super().__init__("map_anchor_injector")

        self.declare_parameter("gps_topic", "/ap/navsat")
        self.declare_parameter("navsat_node", "/navsat_transform")
        self.declare_parameter("datum_service", "/datum")
        self.declare_parameter("imu_topic", "/ap/imu/enu")
        self.declare_parameter("use_imu_heading", False)
        self.declare_parameter("heading_override", float("nan"))
        self.declare_parameter("heading_offset", 0.0)
        self.declare_parameter("altitude_override", 0.0)
        self.declare_parameter("require_fix", True)

        self.gps_topic = self.get_parameter("gps_topic").value
        self.navsat_node = self.get_parameter("navsat_node").value
        self.datum_service_name = self.get_parameter("datum_service").value
        self.imu_topic = self.get_parameter("imu_topic").value
        self.use_imu_heading = self.get_parameter("use_imu_heading").value
        self.heading_override = self.get_parameter("heading_override").value
        self.heading_offset = float(self.get_parameter("heading_offset").value)
        self.altitude_override = self.get_parameter("altitude_override").value
        self.require_fix = self.get_parameter("require_fix").value

        self._datum_inflight = False
        self._shutdown_timer = None
        self._gps_sub = None
        self._imu_sub = None
        self._warned_waiting_imu = False

        self.heading = 0.0
        self.heading_ready = False
        self._resolve_heading_mode()

        self.datum_client = self.create_client(SetDatum, self.datum_service_name)
        while not self.datum_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                f"等待 {self.datum_service_name} 服务上线..."
            )

        self.get_logger().info(
            f"已连接 {self.datum_service_name}，监听 GPS={self.gps_topic}"
        )

        if not self.heading_ready and self.use_imu_heading:
            self._imu_sub = self.create_subscription(
                Imu, self.imu_topic, self._imu_callback, qos_profile_sensor_data
            )
            self.get_logger().info(
                f"等待 IMU 航向: {self.imu_topic}"
            )

        self._gps_sub = self.create_subscription(
            NavSatFix, self.gps_topic, self._gps_callback, qos_profile_sensor_data
        )

    def _resolve_heading_mode(self) -> None:
        if math.isfinite(self.heading_override):
            self.heading = float(self.heading_override)
            self.heading_ready = True
            self.get_logger().info(
                f"使用 heading_override: {self.heading:.6f} rad"
            )
            return

        if self.use_imu_heading:
            self.heading_ready = False
            return

        self.heading = self._resolve_heading_from_navsat_params()
        self.heading_ready = True

    def _resolve_heading_from_navsat_params(self) -> float:
        get_param_srv = f"{self.navsat_node}/get_parameters"
        get_param_client = self.create_client(GetParameters, get_param_srv)
        while not get_param_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"等待 {get_param_srv} 服务上线...")

        req = GetParameters.Request()
        req.names = ["use_odometry_yaw", "yaw_offset"]
        future = get_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)

        if not future.done() or future.result() is None or len(future.result().values) < 2:
            self.get_logger().warn("读取 navsat_transform 参数失败，回退到 heading=0.0 rad")
            return 0.0

        use_odom_yaw = bool(future.result().values[0].bool_value)
        yaw_offset = float(future.result().values[1].double_value)
        self.get_logger().info(
            f"自动读取参数: use_odometry_yaw={use_odom_yaw}, yaw_offset={yaw_offset:.6f} rad"
        )
        if use_odom_yaw:
            self.get_logger().info("use_odometry_yaw=true，注入 datum heading=0.0 rad")
            return 0.0
        return yaw_offset

    @staticmethod
    def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _imu_callback(self, msg: Imu) -> None:
        if self.heading_ready:
            return

        x = float(msg.orientation.x)
        y = float(msg.orientation.y)
        z = float(msg.orientation.z)
        w = float(msg.orientation.w)

        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm < 1e-6:
            self.get_logger().warn("收到无效 IMU 四元数，继续等待...")
            return

        x /= norm
        y /= norm
        z /= norm
        w /= norm

        imu_yaw = self._yaw_from_quaternion(x, y, z, w)
        self.heading = imu_yaw + self.heading_offset
        self.heading_ready = True

        self.get_logger().info(
            f"已锁定 IMU 航向 yaw={imu_yaw:.6f} rad, heading_offset={self.heading_offset:.6f}, datum heading={self.heading:.6f}"
        )

        if self._imu_sub is not None:
            self.destroy_subscription(self._imu_sub)
            self._imu_sub = None

    def _gps_callback(self, msg: NavSatFix) -> None:
        if self._datum_inflight:
            return

        if not self.heading_ready:
            if not self._warned_waiting_imu:
                self.get_logger().warn("IMU 航向尚未就绪，暂不注入 datum")
                self._warned_waiting_imu = True
            return

        lat_ok = math.isfinite(msg.latitude)
        lon_ok = math.isfinite(msg.longitude)
        fix_ok = (not self.require_fix) or (msg.status.status != NavSatStatus.STATUS_NO_FIX)

        if not (lat_ok and lon_ok and fix_ok):
            self.get_logger().warn("GPS 无效（No Fix 或 NaN），继续等待下一帧...")
            return

        self.get_logger().info(
            f"抓到首帧有效 GPS: lat={msg.latitude:.8f}, lon={msg.longitude:.8f}"
        )
        self._send_datum(msg.latitude, msg.longitude)

    def _send_datum(self, latitude: float, longitude: float) -> None:
        self._datum_inflight = True
        request = SetDatum.Request()
        request.geo_pose.position.latitude = latitude
        request.geo_pose.position.longitude = longitude
        request.geo_pose.position.altitude = float(self.altitude_override)

        cy = math.cos(self.heading * 0.5)
        sy = math.sin(self.heading * 0.5)
        request.geo_pose.orientation.x = 0.0
        request.geo_pose.orientation.y = 0.0
        request.geo_pose.orientation.z = sy
        request.geo_pose.orientation.w = cy

        self.get_logger().info("发送 Datum 到 navsat_transform...")
        future = self.datum_client.call_async(request)
        future.add_done_callback(self._on_datum_response)

    def _on_datum_response(self, future) -> None:
        try:
            _ = future.result()
            self.get_logger().info("Datum 注入成功，准备退出。")
            if self._gps_sub is not None:
                self.destroy_subscription(self._gps_sub)
                self._gps_sub = None
            self._shutdown_timer = self.create_timer(0.5, self._shutdown)
        except Exception as exc:
            self.get_logger().error(f"Datum 注入失败，将继续等待重试: {exc}")
            self._datum_inflight = False

    def _shutdown(self) -> None:
        if self._shutdown_timer is not None:
            self.destroy_timer(self._shutdown_timer)
            self._shutdown_timer = None
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MapAnchorInjector()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
