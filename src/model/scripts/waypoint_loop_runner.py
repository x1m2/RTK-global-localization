#!/usr/bin/env python3

import math
import time
import ast
from typing import List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import Marker, MarkerArray


class WaypointLoopRunner(Node):
    def __init__(self) -> None:
        super().__init__("waypoint_loop_runner")

        self.declare_parameter("action_name", "/follow_waypoints")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("waypoints_xyyaw", "")
        self.declare_parameter("waypoints_topic", "")
        self.declare_parameter("waypoints_topic_type", "marker_array")
        self.declare_parameter("path_stable_sec", 1.5)
        # Single-goal mode by default: one clicked goal is enough.
        self.declare_parameter("min_waypoints", 1)
        self.declare_parameter("loop_forever", True)
        self.declare_parameter("loop_count", 0)
        self.declare_parameter("pause_sec", 0.5)
        self.declare_parameter("retry_on_miss", True)
        self.declare_parameter("server_wait_sec", 20.0)

        self.action_name = str(self.get_parameter("action_name").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.waypoints_topic = str(self.get_parameter("waypoints_topic").value)
        self.waypoints_topic_type = str(self.get_parameter("waypoints_topic_type").value).lower()
        self.path_stable_sec = float(self.get_parameter("path_stable_sec").value)
        self.min_waypoints = int(self.get_parameter("min_waypoints").value)
        self.loop_forever = bool(self.get_parameter("loop_forever").value)
        self.loop_count = int(self.get_parameter("loop_count").value)
        self.pause_sec = float(self.get_parameter("pause_sec").value)
        self.retry_on_miss = bool(self.get_parameter("retry_on_miss").value)
        self.server_wait_sec = float(self.get_parameter("server_wait_sec").value)

        raw_text = str(self.get_parameter("waypoints_xyyaw").value).strip()
        raw = self._parse_waypoints_text(raw_text) if raw_text else []
        # In single-goal mode, only keep one goal point from static input.
        self.poses: List[PoseStamped] = self._build_poses(raw[-3:]) if raw else []
        self.last_waypoints_update = 0.0
        self.path_version = 0
        self.latest_start_pose: Optional[PoseStamped] = None
        self.loop_start_pose: Optional[PoseStamped] = None

        # Prefer /initialpose as loop start since user confirms it manually with 2D Estimate.
        self.initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self._initialpose_cb, 10
        )
        # Fallback source when /initialpose has not been clicked in current run.
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/amcl_pose", self._amcl_pose_cb, 10
        )

        self.path_sub = None
        if self.waypoints_topic:
            if self.waypoints_topic_type == "path":
                from nav_msgs.msg import Path  # lazy import to keep dependency flexible

                self.path_sub = self.create_subscription(
                    Path, self.waypoints_topic, self._waypoints_path_cb, 10
                )
            elif self.waypoints_topic_type in ("pose_stamped", "goal_pose"):
                self.path_sub = self.create_subscription(
                    PoseStamped, self.waypoints_topic, self._waypoints_pose_cb, 10
                )
            else:
                marker_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                )
                self.path_sub = self.create_subscription(
                    MarkerArray, self.waypoints_topic, self._waypoints_marker_cb, marker_qos
                )
            self.get_logger().info(
                f"启用 RViz 路径模式(type={self.waypoints_topic_type})，监听 {self.waypoints_topic}，"
                f"稳定 {self.path_stable_sec:.1f}s 后开始循环"
            )
        elif not self.poses:
            raise ValueError(
                "未提供 waypoints_xyyaw，且未设置 waypoints_topic，至少需要一种输入"
            )

        self.client = ActionClient(self, FollowWaypoints, self.action_name)

    @staticmethod
    def _parse_waypoints_text(raw_text: str) -> List[float]:
        try:
            value = ast.literal_eval(raw_text)
            if isinstance(value, (list, tuple)):
                return [float(v) for v in value]
        except Exception:
            pass

        # Fallback: comma-separated numbers like "x1,y1,yaw1,x2,y2,yaw2"
        parts = [p.strip() for p in raw_text.split(",") if p.strip()]
        return [float(p) for p in parts]

    def _build_poses(self, raw: List[float]) -> List[PoseStamped]:
        if len(raw) < 3 or len(raw) % 3 != 0:
            raise ValueError(
                "waypoints_xyyaw must be a flat list [x1,y1,yaw1,x2,y2,yaw2,...] with len%3==0"
            )

        poses: List[PoseStamped] = []
        for i in range(0, len(raw), 3):
            x = float(raw[i])
            y = float(raw[i + 1])
            yaw = float(raw[i + 2])

            pose = PoseStamped()
            pose.header.frame_id = self.frame_id
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = math.sin(yaw * 0.5)
            pose.pose.orientation.w = math.cos(yaw * 0.5)
            poses.append(pose)

        return poses

    @staticmethod
    def _copy_pose_stamped(src: PoseStamped) -> PoseStamped:
        dst = PoseStamped()
        dst.header = src.header
        dst.pose = src.pose
        return dst

    def _to_pose_stamped(self, msg: PoseWithCovarianceStamped) -> PoseStamped:
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = self.frame_id
        pose.pose = msg.pose.pose
        return pose

    def _initialpose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        pose = self._to_pose_stamped(msg)
        self.latest_start_pose = pose
        # Reset loop start lock so a new 2D Estimate can take effect immediately.
        self.loop_start_pose = None
        self.get_logger().info("收到 2D Estimate 作为循环起点")

    def _amcl_pose_cb(self, msg: PoseWithCovarianceStamped) -> None:
        if self.latest_start_pose is not None:
            return
        self.latest_start_pose = self._to_pose_stamped(msg)

    def _distance_xy(self, a: PoseStamped, b: PoseStamped) -> float:
        dx = a.pose.position.x - b.pose.position.x
        dy = a.pose.position.y - b.pose.position.y
        return math.hypot(dx, dy)

    def _get_locked_loop_start(self) -> PoseStamped:
        while rclpy.ok() and self.latest_start_pose is None:
            self.get_logger().warn("等待起点位姿（请先用 2D Estimate 确认位置）")
            rclpy.spin_once(self, timeout_sec=0.2)

        if self.loop_start_pose is None and self.latest_start_pose is not None:
            self.loop_start_pose = self._copy_pose_stamped(self.latest_start_pose)
            self.get_logger().info(
                f"锁定循环起点: x={self.loop_start_pose.pose.position.x:.2f}, "
                f"y={self.loop_start_pose.pose.position.y:.2f}"
            )
        return self._copy_pose_stamped(self.loop_start_pose)

    def _waypoints_path_cb(self, msg) -> None:
        if len(msg.poses) < 1:
            return

        frame_id = msg.header.frame_id if msg.header.frame_id else self.frame_id
        # Single-goal mode: always take the latest clicked/updated goal.
        p = msg.poses[-1]
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose = p.pose

        self.poses = [pose]
        self.path_version += 1
        self.last_waypoints_update = time.monotonic()
        self.get_logger().info(
            f"收到单目标点(path): x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f} "
            f"(version={self.path_version})"
        )

    def _waypoints_pose_cb(self, msg: PoseStamped) -> None:
        pose = PoseStamped()
        pose.header = msg.header
        if not pose.header.frame_id:
            pose.header.frame_id = self.frame_id
        pose.pose = msg.pose

        self.poses = [pose]
        self.path_version += 1
        self.last_waypoints_update = time.monotonic()
        self.get_logger().info(
            f"收到单目标点: x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f} "
            f"(version={self.path_version})"
        )

    def _waypoints_marker_cb(self, msg: MarkerArray) -> None:
        if not msg.markers:
            return

        if any(m.action == Marker.DELETEALL for m in msg.markers):
            self.poses = []
            return

        arrows = [m for m in msg.markers if m.type == Marker.ARROW and m.action == Marker.ADD]
        arrows.sort(key=lambda m: m.id)

        pose = PoseStamped()
        if arrows:
            m = arrows[-1]
            pose.header = m.header
            if not pose.header.frame_id:
                pose.header.frame_id = self.frame_id
            pose.pose = m.pose
        else:
            # Fallback for panels that only publish SPHERE waypoints.
            spheres = [m for m in msg.markers if m.type == Marker.SPHERE and m.action == Marker.ADD]
            spheres.sort(key=lambda m: m.id)
            if not spheres:
                return
            m = spheres[-1]
            pose.header = m.header
            if not pose.header.frame_id:
                pose.header.frame_id = self.frame_id
            pose.pose.position = m.pose.position
            pose.pose.position.z = 0.0
            if len(spheres) >= 2:
                p0 = spheres[-2].pose.position
                p1 = spheres[-1].pose.position
                yaw = math.atan2(p1.y - p0.y, p1.x - p0.x)
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = math.sin(yaw * 0.5)
                pose.pose.orientation.w = math.cos(yaw * 0.5)
            else:
                pose.pose.orientation.x = 0.0
                pose.pose.orientation.y = 0.0
                pose.pose.orientation.z = 0.0
                pose.pose.orientation.w = 1.0

        self.poses = [pose]
        self.path_version += 1
        self.last_waypoints_update = time.monotonic()
        self.get_logger().info(
            f"收到单目标点(marker): x={pose.pose.position.x:.2f}, y={pose.pose.position.y:.2f} "
            f"(version={self.path_version})"
        )

    def _wait_for_stable_waypoints(self) -> None:
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)
            if len(self.poses) < 1:
                continue
            elapsed = time.monotonic() - self.last_waypoints_update
            if elapsed >= self.path_stable_sec:
                return

    def _feedback_cb(self, feedback_msg) -> None:
        self.get_logger().info(
            f"巡航中: 当前航点索引 {feedback_msg.feedback.current_waypoint}/1"
        )

    def run(self) -> None:
        self.get_logger().info(
            f"等待 action server: {self.action_name} (timeout={self.server_wait_sec:.1f}s)"
        )
        if not self.client.wait_for_server(timeout_sec=self.server_wait_sec):
            raise RuntimeError(f"Action server {self.action_name} 未就绪")

        loop_idx = 0
        while rclpy.ok():
            if self.waypoints_topic:
                self._wait_for_stable_waypoints()

            if not self.loop_forever and self.loop_count > 0 and loop_idx >= self.loop_count:
                self.get_logger().info(f"已完成设定循环次数: {self.loop_count}")
                break

            if not self.poses:
                continue

            target_pose = self._copy_pose_stamped(self.poses[-1])
            start_pose = self._get_locked_loop_start()
            if not start_pose.header.frame_id:
                start_pose.header.frame_id = self.frame_id
            if not target_pose.header.frame_id:
                target_pose.header.frame_id = self.frame_id

            # Enforce loop pattern: start(2D Estimate confirmed) -> clicked goal.
            # If target and start are too close, skip sending an ineffective loop.
            if self._distance_xy(start_pose, target_pose) < 0.05:
                self.get_logger().warn("起点与目标点距离过近(<5cm)，请重新点击更远目标点")
                time.sleep(0.2)
                continue

            goal = FollowWaypoints.Goal()
            now = self.get_clock().now().to_msg()
            start_pose.header.stamp = now
            target_pose.header.stamp = now
            goal.poses = [start_pose, target_pose]

            self.get_logger().info(
                f"开始第 {loop_idx + 1} 轮单点往返: "
                f"start({start_pose.pose.position.x:.2f},{start_pose.pose.position.y:.2f}) -> "
                f"goal({target_pose.pose.position.x:.2f},{target_pose.pose.position.y:.2f})"
            )

            send_future = self.client.send_goal_async(goal, feedback_callback=self._feedback_cb)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()

            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError("FollowWaypoints 目标被拒绝")

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future)
            result = result_future.result()

            if result is None:
                raise RuntimeError("FollowWaypoints 无返回结果")

            missed = list(result.result.missed_waypoints)
            if missed:
                self.get_logger().warn(f"本轮有未到达航点: {missed}")
                if not self.retry_on_miss:
                    self.get_logger().warn("retry_on_miss=false，停止循环")
                    break
            else:
                self.get_logger().info(f"第 {loop_idx + 1} 轮完成")

            loop_idx += 1
            if self.pause_sec > 0:
                time.sleep(self.pause_sec)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointLoopRunner()
    try:
        node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
