#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""Fail-safe pure-pursuit controller for the Vbot locomotion interface.

The node is intentionally independent from ``dog_ego_planner``.  It consumes
an odometry estimate and a local geometric path, and can publish ``Twist``
commands only when both explicit software locks are released.

Default operation is safe: ``enabled=false`` and ``dry_run=true`` means that
the node never publishes to the velocity topic.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool


Point2D = Tuple[float, float]


@dataclass(frozen=True)
class PursuitCommand:
    """A validated planar command and the look-ahead point that produced it."""

    linear_x: float
    angular_z: float
    target_x: float
    target_y: float


def _all_finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def quaternion_to_yaw(
    x: float, y: float, z: float, w: float
) -> Optional[float]:
    """Convert a finite, non-zero quaternion to yaw.

    ``None`` is returned instead of silently accepting a malformed pose.
    """

    if not _all_finite((x, y, z, w)):
        return None
    norm_squared = x * x + y * y + z * z + w * w
    if norm_squared <= 1.0e-12:
        return None

    inverse_norm = 1.0 / math.sqrt(norm_squared)
    x *= inverse_norm
    y *= inverse_norm
    z *= inverse_norm
    w *= inverse_norm

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return yaw if math.isfinite(yaw) else None


def _cumulative_lengths(
    points: Sequence[Point2D],
) -> Tuple[Tuple[float, ...], float]:
    lengths = [0.0]
    for first, second in zip(points, points[1:]):
        lengths.append(
            lengths[-1]
            + math.hypot(
                second[0] - first[0], second[1] - first[1]
            )
        )
    return tuple(lengths), lengths[-1]


def _project_onto_path(
    robot_x: float,
    robot_y: float,
    points: Sequence[Point2D],
    cumulative_lengths: Sequence[float],
) -> Tuple[float, float]:
    """Return minimum path distance and projected arc length."""

    best_distance = math.inf
    best_arc_length = 0.0

    for index, (first, second) in enumerate(zip(points, points[1:])):
        dx = second[0] - first[0]
        dy = second[1] - first[1]
        segment_length_squared = dx * dx + dy * dy
        if segment_length_squared <= 1.0e-12:
            projection = 0.0
        else:
            projection = (
                (robot_x - first[0]) * dx + (robot_y - first[1]) * dy
            ) / segment_length_squared
            projection = min(1.0, max(0.0, projection))

        projected_x = first[0] + projection * dx
        projected_y = first[1] + projection * dy
        distance = math.hypot(projected_x - robot_x, projected_y - robot_y)
        if distance < best_distance:
            best_distance = distance
            segment_length = math.sqrt(segment_length_squared)
            best_arc_length = (
                cumulative_lengths[index] + projection * segment_length
            )

    return best_distance, best_arc_length


def _point_at_arc_length(
    points: Sequence[Point2D],
    cumulative_lengths: Sequence[float],
    target_arc_length: float,
) -> Point2D:
    for index, segment_end in enumerate(cumulative_lengths[1:]):
        if target_arc_length > segment_end:
            continue
        segment_start = cumulative_lengths[index]
        segment_length = segment_end - segment_start
        if segment_length <= 1.0e-12:
            return points[index + 1]
        ratio = (target_arc_length - segment_start) / segment_length
        first = points[index]
        second = points[index + 1]
        return (
            first[0] + ratio * (second[0] - first[0]),
            first[1] + ratio * (second[1] - first[1]),
        )
    return points[-1]


def calculate_pure_pursuit(
    robot_x: float,
    robot_y: float,
    robot_yaw: float,
    path_points: Sequence[Point2D],
    *,
    lookahead_distance: float,
    linear_speed: float,
    maximum_angular_speed: float,
    maximum_heading_error: float,
    maximum_lateral_acceleration: float,
    maximum_path_deviation: float,
    goal_tolerance: float,
) -> Tuple[Optional[PursuitCommand], str]:
    """Calculate one fail-safe pure-pursuit command.

    The returned reason is suitable for operator diagnostics.  Any rejected
    condition returns ``None`` and must be treated as a stop request.
    """

    scalar_values = (
        robot_x,
        robot_y,
        robot_yaw,
        lookahead_distance,
        linear_speed,
        maximum_angular_speed,
        maximum_heading_error,
        maximum_lateral_acceleration,
        maximum_path_deviation,
        goal_tolerance,
    )
    if not _all_finite(scalar_values):
        return None, "输入包含非有限数值"
    if len(path_points) < 2:
        return None, "路径点数量不足"
    if any(not _all_finite(point) for point in path_points):
        return None, "路径包含非有限数值"
    if lookahead_distance <= 0.0 or linear_speed <= 0.0:
        return None, "速度或预瞄距离参数无效"
    if (
        maximum_angular_speed <= 0.0
        or maximum_heading_error <= 0.0
        or maximum_lateral_acceleration <= 0.0
        or maximum_path_deviation < 0.0
        or goal_tolerance < 0.0
    ):
        return None, "安全限制参数无效"

    goal_x, goal_y = path_points[-1]
    if math.hypot(goal_x - robot_x, goal_y - robot_y) <= goal_tolerance:
        return None, "已进入路径终点容差"

    cosine = math.cos(robot_yaw)
    sine = math.sin(robot_yaw)
    local_points = tuple(
        (
            (point_x - robot_x) * cosine + (point_y - robot_y) * sine,
            -(point_x - robot_x) * sine + (point_y - robot_y) * cosine,
        )
        for point_x, point_y in path_points
    )
    if all(local_x < -1.0e-6 for local_x, _ in local_points):
        return None, "预瞄点不在机器人前方"

    cumulative_lengths, total_length = _cumulative_lengths(path_points)
    path_deviation, projected_arc_length = _project_onto_path(
        robot_x, robot_y, path_points, cumulative_lengths
    )
    if path_deviation > maximum_path_deviation:
        return None, "机器人偏离局部路径过远"

    target_arc_length = min(
        total_length, projected_arc_length + lookahead_distance
    )
    target_x, target_y = _point_at_arc_length(
        path_points, cumulative_lengths, target_arc_length
    )
    delta_x = target_x - robot_x
    delta_y = target_y - robot_y
    local_x = delta_x * cosine + delta_y * sine
    local_y = -delta_x * sine + delta_y * cosine
    target_distance = math.hypot(local_x, local_y)
    if target_distance <= 1.0e-6:
        return None, "预瞄点距离过小"

    heading_error = math.atan2(local_y, local_x)
    if abs(heading_error) > maximum_heading_error:
        return None, "目标方向偏差过大"
    if local_x <= 0.0:
        return None, "预瞄点不在机器人前方"

    curvature = 2.0 * local_y / (target_distance * target_distance)
    angular_z = linear_speed * curvature
    lateral_acceleration_limit = maximum_lateral_acceleration / linear_speed
    angular_limit = min(maximum_angular_speed, lateral_acceleration_limit)
    angular_z = min(angular_limit, max(-angular_limit, angular_z))

    if not _all_finite((linear_speed, angular_z, target_x, target_y)):
        return None, "控制结果包含非有限数值"
    return (
        PursuitCommand(
            linear_x=linear_speed,
            angular_z=angular_z,
            target_x=target_x,
            target_y=target_y,
        ),
        "ok",
    )


class VbotPathFollower(Node):
    """A parameterized, watchdog-protected path-following node."""

    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "vbot_path_follower_node", parameter_overrides=parameter_overrides
        )

        self.enabled = bool(self.declare_parameter("enabled", False).value)
        self.dry_run = bool(self.declare_parameter("dry_run", True).value)
        self.odom_topic = str(
            self.declare_parameter("odom_topic", "/location_now").value
        )
        self.path_topic = str(
            self.declare_parameter(
                "path_topic", "/dog_output_local_path_copy"
            ).value
        )
        self.goal_reached_topic = str(
            self.declare_parameter(
                "goal_reached_topic", "/goal_reached_copy"
            ).value
        )
        self.velocity_topic = str(
            self.declare_parameter("velocity_topic", "/vel_cmd_copy").value
        )
        self.expected_frame = str(
            self.declare_parameter(
                "expected_frame", "local_map_lidar_init_xyz"
            ).value
        )

        self.lookahead_distance = float(
            self.declare_parameter("lookahead_distance", 1.0).value
        )
        self.linear_speed = float(
            self.declare_parameter("linear_speed", 0.6).value
        )
        self.maximum_angular_speed = float(
            self.declare_parameter("maximum_angular_speed", 1.5).value
        )
        self.maximum_heading_error = float(
            self.declare_parameter("maximum_heading_error", 0.8).value
        )
        self.maximum_lateral_acceleration = float(
            self.declare_parameter("maximum_lateral_acceleration", 0.8).value
        )
        self.maximum_path_deviation = float(
            self.declare_parameter("maximum_path_deviation", 0.75).value
        )
        self.goal_tolerance = float(
            self.declare_parameter("goal_tolerance", 0.25).value
        )
        self.control_frequency = float(
            self.declare_parameter("control_frequency", 10.0).value
        )
        self.odometry_timeout = float(
            self.declare_parameter("odometry_timeout_s", 0.5).value
        )
        self.path_timeout = float(
            self.declare_parameter("path_timeout_s", 0.5).value
        )
        self.future_stamp_tolerance = float(
            self.declare_parameter("future_stamp_tolerance_s", 0.1).value
        )
        self.require_velocity_subscriber = bool(
            self.declare_parameter("require_velocity_subscriber", True).value
        )
        self.reject_other_velocity_publishers = bool(
            self.declare_parameter(
                "reject_other_velocity_publishers", True
            ).value
        )
        self.shutdown_stop_repetitions = int(
            self.declare_parameter("shutdown_stop_repetitions", 3).value
        )

        self._validate_parameters()

        odometry_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        path_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        velocity_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.current_odometry: Optional[Odometry] = None
        self.current_path: Optional[Path] = None
        self.goal_reached = False
        self._odometry_received_at: Optional[float] = None
        self._path_received_at: Optional[float] = None
        self._last_state_reason: Optional[str] = None
        self._motion_command_published = False

        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic, self._on_odometry, odometry_qos
        )
        self.path_sub = self.create_subscription(
            Path, self.path_topic, self._on_path, path_qos
        )
        self.goal_sub = self.create_subscription(
            Bool, self.goal_reached_topic, self._on_goal_reached, path_qos
        )
        self.vel_pub = self.create_publisher(
            Twist, self.velocity_topic, velocity_qos
        )
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_once
        )

        if not self._is_armed():
            self.get_logger().warning(
                "控制器处于安全锁定状态："
                f"enabled={self.enabled} dry_run={self.dry_run}；不会发布速度。"
            )
        else:
            self.get_logger().warning(
                "控制器已解除软件锁，等待有效且同坐标系的位姿和局部路径。"
            )
        self.get_logger().info(
            f"odom={self.odom_topic} path={self.path_topic} "
            f"velocity={self.velocity_topic} "
            f"expected_frame={self.expected_frame}"
        )

    def _validate_parameters(self) -> None:
        named_topics = {
            "odom_topic": self.odom_topic,
            "path_topic": self.path_topic,
            "goal_reached_topic": self.goal_reached_topic,
            "velocity_topic": self.velocity_topic,
        }
        for name, topic in named_topics.items():
            if not topic:
                raise ValueError(f"{name} must not be empty")
        if self.velocity_topic in {
            self.odom_topic,
            self.path_topic,
            self.goal_reached_topic,
        }:
            raise ValueError(
                "velocity_topic must differ from every input topic"
            )
        if not self.expected_frame or self.expected_frame.startswith("/"):
            raise ValueError(
                "expected_frame must be a non-empty TF frame "
                "without a leading slash"
            )

        positive_values = {
            "lookahead_distance": self.lookahead_distance,
            "linear_speed": self.linear_speed,
            "maximum_angular_speed": self.maximum_angular_speed,
            "maximum_heading_error": self.maximum_heading_error,
            "maximum_lateral_acceleration": self.maximum_lateral_acceleration,
            "control_frequency": self.control_frequency,
            "odometry_timeout_s": self.odometry_timeout,
            "path_timeout_s": self.path_timeout,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(
                    f"{name} must be finite and greater than zero"
                )
        nonnegative_values = {
            "maximum_path_deviation": self.maximum_path_deviation,
            "goal_tolerance": self.goal_tolerance,
            "future_stamp_tolerance_s": self.future_stamp_tolerance,
        }
        for name, value in nonnegative_values.items():
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{name} must be finite and non-negative")
        if self.shutdown_stop_repetitions < 1:
            raise ValueError("shutdown_stop_repetitions must be at least one")

    def _is_armed(self) -> bool:
        return self.enabled and not self.dry_run

    def _on_odometry(self, message: Odometry) -> None:
        self.current_odometry = message
        self._odometry_received_at = time.monotonic()

    def _on_path(self, message: Path) -> None:
        self.current_path = message
        self._path_received_at = time.monotonic()

    def _on_goal_reached(self, message: Bool) -> None:
        self.goal_reached = bool(message.data)

    def _set_state(self, reason: str, *, warning: bool = True) -> None:
        if reason == self._last_state_reason:
            return
        self._last_state_reason = reason
        if warning:
            self.get_logger().warning(reason)
        else:
            self.get_logger().info(reason)

    def _message_stamp_is_fresh(self, stamp, timeout: float) -> bool:
        stamp_nanoseconds = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_nanoseconds <= 0:
            return False
        age = (self.get_clock().now().nanoseconds - stamp_nanoseconds) / 1.0e9
        return -self.future_stamp_tolerance <= age <= timeout

    def _input_error(self) -> Optional[str]:
        now = time.monotonic()
        if self.current_odometry is None or self._odometry_received_at is None:
            return f"等待位姿：{self.odom_topic}"
        if now - self._odometry_received_at > self.odometry_timeout:
            return "位姿接收超时"
        if not self._message_stamp_is_fresh(
            self.current_odometry.header.stamp, self.odometry_timeout
        ):
            return "位姿时间戳过期、为零或来自未来"

        if self.current_path is None or self._path_received_at is None:
            return f"等待局部路径：{self.path_topic}"
        if now - self._path_received_at > self.path_timeout:
            return "局部路径接收超时"
        if not self._message_stamp_is_fresh(
            self.current_path.header.stamp, self.path_timeout
        ):
            return "局部路径时间戳过期、为零或来自未来"

        odometry_frame = self.current_odometry.header.frame_id
        path_frame = self.current_path.header.frame_id
        if odometry_frame != self.expected_frame:
            return (
                f"位姿frame不匹配：{odometry_frame!r} != "
                f"{self.expected_frame!r}"
            )
        if path_frame != self.expected_frame:
            return (
                f"路径frame不匹配：{path_frame!r} != "
                f"{self.expected_frame!r}"
            )
        for pose in self.current_path.poses:
            pose_frame = pose.header.frame_id
            if pose_frame and pose_frame != self.expected_frame:
                return (
                    f"路径点frame不匹配：{pose_frame!r} != "
                    f"{self.expected_frame!r}"
                )
        return None

    def _has_other_velocity_publishers(self) -> bool:
        if not self.reject_other_velocity_publishers:
            return False
        own_name = self.get_name()
        own_namespace = self.get_namespace()
        matching_own_identity = 0
        for endpoint in self.get_publishers_info_by_topic(self.velocity_topic):
            if (
                endpoint.node_name == own_name
                and endpoint.node_namespace == own_namespace
            ):
                matching_own_identity += 1
            else:
                return True
        # ROS允许重复使用相同node name/namespace。当前节点只创建一个速度
        # publisher，因此同一身份出现两个端点也代表另一个控制器在争抢输出。
        return matching_own_identity > 1

    def _publish_zero(self, reason: str, *, force: bool = False) -> None:
        self._set_state(reason)
        if not self._is_armed():
            return
        if not force and self.require_velocity_subscriber:
            if self.vel_pub.get_subscription_count() < 1:
                return
        self.vel_pub.publish(Twist())
        self._motion_command_published = False

    def control_once(self) -> None:
        """Run one watchdog and pure-pursuit control cycle."""

        if not self._is_armed():
            self._set_state(
                "控制器保持安全锁定；未发布速度。", warning=False
            )
            return

        if self.require_velocity_subscriber:
            if self.vel_pub.get_subscription_count() < 1:
                self._set_state("速度话题没有订阅者；拒绝发送控制命令。")
                return

        if self._has_other_velocity_publishers():
            if self._motion_command_published:
                self._publish_zero("检测到其他速度发布者；立即停车。", force=True)
            else:
                self._set_state("检测到其他速度发布者；拒绝争抢控制权。")
            return

        input_error = self._input_error()
        if input_error is not None:
            self._publish_zero(input_error)
            return
        if self.goal_reached:
            self._publish_zero("规划器报告已到达目标。")
            return

        assert self.current_odometry is not None
        assert self.current_path is not None
        pose = self.current_odometry.pose.pose
        yaw = quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        if yaw is None or not _all_finite((pose.position.x, pose.position.y)):
            self._publish_zero("位姿包含无效位置或四元数。")
            return

        path_points = tuple(
            (
                float(path_pose.pose.position.x),
                float(path_pose.pose.position.y),
            )
            for path_pose in self.current_path.poses
        )
        command, reason = calculate_pure_pursuit(
            float(pose.position.x),
            float(pose.position.y),
            yaw,
            path_points,
            lookahead_distance=self.lookahead_distance,
            linear_speed=self.linear_speed,
            maximum_angular_speed=self.maximum_angular_speed,
            maximum_heading_error=self.maximum_heading_error,
            maximum_lateral_acceleration=self.maximum_lateral_acceleration,
            maximum_path_deviation=self.maximum_path_deviation,
            goal_tolerance=self.goal_tolerance,
        )
        if command is None:
            self._publish_zero(reason)
            return

        message = Twist()
        message.linear.x = command.linear_x
        message.angular.z = command.angular_z
        self.vel_pub.publish(message)
        self._motion_command_published = True
        self._set_state(
            "路径跟踪有效："
            f"target=({command.target_x:.3f}, {command.target_y:.3f}) "
            f"linear_x={command.linear_x:.3f} "
            f"angular_z={command.angular_z:.3f}",
            warning=False,
        )

    def stop_before_shutdown(self) -> None:
        """Best-effort zero command for an already armed controller."""

        if not self._is_armed():
            return
        for _ in range(self.shutdown_stop_repetitions):
            self.vel_pub.publish(Twist())
        self._motion_command_published = False


def main(args=None) -> int:
    rclpy.init(args=args)
    node = VbotPathFollower()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop_before_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
