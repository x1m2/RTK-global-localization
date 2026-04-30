#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__("twist_to_twist_stamped")

        self.declare_parameter("input_topic", "/cmd_vel_ap_raw")
        self.declare_parameter("output_topic", "/ap/cmd_vel")
        self.declare_parameter("frame_id", "base_link")

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.frame_id = self.get_parameter("frame_id").value

        self.pub_ = self.create_publisher(TwistStamped, output_topic, 10)
        self.sub_ = self.create_subscription(
            Twist, input_topic, self._input_callback, 10
        )

        self.get_logger().info(
            f"Twist bridge started: {input_topic} (Twist) -> {output_topic} (TwistStamped)"
        )

    def _input_callback(self, msg: Twist):
        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.frame_id
        out.twist = msg
        self.pub_.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
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
