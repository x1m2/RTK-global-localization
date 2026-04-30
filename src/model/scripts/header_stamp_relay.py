#!/usr/bin/env python3

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2


class HeaderStampRelay(Node):
    def __init__(self):
        super().__init__("header_stamp_relay")

        self.declare_parameter("input_type", "laser_scan")
        self.declare_parameter("input_topic", "/scan_raw")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("queue_size", 10)

        input_type = str(self.get_parameter("input_type").value).strip().lower()
        self.input_topic = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        queue_size = int(self.get_parameter("queue_size").value)

        if input_type in ("laser", "laser_scan", "laserscan", "scan"):
            self.msg_type = LaserScan
        elif input_type in ("pointcloud2", "point_cloud2", "cloud"):
            self.msg_type = PointCloud2
        else:
            raise ValueError(
                f"Unsupported input_type '{input_type}'. Use laser_scan or point_cloud2."
            )

        self.publisher = self.create_publisher(self.msg_type, self.output_topic, queue_size)
        # Sensor topics in this pipeline may be published as best_effort, so the relay
        # subscribes with sensor-data QoS to avoid silently missing the raw cloud/scan.
        self.subscription = self.create_subscription(
            self.msg_type, self.input_topic, self._callback, qos_profile_sensor_data
        )

        self.get_logger().info(
            f"Stamp relay started: {self.input_topic} -> {self.output_topic} "
            f"({self.msg_type.__name__})"
        )

    def _callback(self, msg):
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = HeaderStampRelay()
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
