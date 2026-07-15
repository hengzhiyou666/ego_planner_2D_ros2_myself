#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-3.0-only
"""dog_move_forward.py 的纯算法和隔离 ROS 冒烟测试。"""

from __future__ import annotations

import math
import os
import pathlib
import sys
import time
import unittest

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


REPOSITORY_ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPOSITORY_ROOT))

from dog_move_forward import (  # noqa: E402
    VbotPathFollower,
    calculate_pure_pursuit,
    quaternion_to_yaw,
)


class PurePursuitTest(unittest.TestCase):
    def calculate(self, points, *, yaw=0.0):
        return calculate_pure_pursuit(
            0.0,
            0.0,
            yaw,
            points,
            lookahead_distance=1.0,
            linear_speed=0.6,
            maximum_angular_speed=1.5,
            maximum_heading_error=0.8,
            maximum_lateral_acceleration=0.8,
            maximum_path_deviation=0.75,
            goal_tolerance=0.25,
        )

    def test_invalid_quaternion_is_rejected(self):
        self.assertIsNone(quaternion_to_yaw(0.0, 0.0, 0.0, 0.0))
        self.assertIsNone(quaternion_to_yaw(math.nan, 0.0, 0.0, 1.0))

    def test_straight_path_commands_forward_motion(self):
        command, reason = self.calculate([(-1.0, 0.0), (0.0, 0.0), (2.0, 0.0)])
        self.assertEqual(reason, "ok")
        self.assertIsNotNone(command)
        self.assertAlmostEqual(command.linear_x, 0.6)
        self.assertAlmostEqual(command.angular_z, 0.0)
        self.assertGreater(command.target_x, 0.0)

    def test_path_behind_robot_is_rejected(self):
        command, reason = self.calculate([(-2.0, 0.0), (-1.0, 0.0)])
        self.assertIsNone(command)
        self.assertIn(reason, {"预瞄点不在机器人前方", "已进入路径终点容差"})

    def test_large_heading_error_is_rejected(self):
        command, reason = self.calculate([(0.0, 0.0), (0.0, 2.0)])
        self.assertIsNone(command)
        self.assertEqual(reason, "目标方向偏差过大")

    def test_non_finite_path_is_rejected(self):
        command, reason = self.calculate([(0.0, 0.0), (math.nan, 1.0)])
        self.assertIsNone(command)
        self.assertEqual(reason, "路径包含非有限数值")


@unittest.skipUnless(
    os.environ.get("MOVEFORWARD_RUN_ROS_TEST") == "1",
    "set MOVEFORWARD_RUN_ROS_TEST=1 for the isolated ROS smoke test",
)
class IsolatedRosTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        suffix = str(time.monotonic_ns())
        self.odom_topic = f"/moveforward_test_{suffix}/odometry"
        self.path_topic = f"/moveforward_test_{suffix}/path"
        self.goal_topic = f"/moveforward_test_{suffix}/goal"
        self.velocity_topic = f"/moveforward_test_{suffix}/velocity"
        self.driver = rclpy.create_node(f"moveforward_test_driver_{suffix}")
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.odom_publisher = self.driver.create_publisher(
            Odometry, self.odom_topic, 10
        )
        self.path_publisher = self.driver.create_publisher(
            Path, self.path_topic, path_qos
        )
        self.commands = []
        self.command_subscription = self.driver.create_subscription(
            Twist, self.velocity_topic, self.commands.append, 10
        )

    def tearDown(self):
        self.driver.destroy_node()

    def make_controller(self, enabled: bool, dry_run: bool):
        return VbotPathFollower(
            parameter_overrides=[
                Parameter("enabled", value=enabled),
                Parameter("dry_run", value=dry_run),
                Parameter("odom_topic", value=self.odom_topic),
                Parameter("path_topic", value=self.path_topic),
                Parameter("goal_reached_topic", value=self.goal_topic),
                Parameter("velocity_topic", value=self.velocity_topic),
                Parameter("expected_frame", value="head_init"),
                Parameter("require_velocity_subscriber", value=True),
            ]
        )

    def publish_inputs(self):
        stamp = self.driver.get_clock().now().to_msg()
        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = "head_init"
        odometry.pose.pose.orientation.w = 1.0
        path = Path()
        path.header.stamp = stamp
        path.header.frame_id = "head_init"
        for x in (0.0, 1.0, 2.0):
            pose = __import__("geometry_msgs.msg", fromlist=["PoseStamped"]).PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "head_init"
            pose.pose.position.x = x
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.odom_publisher.publish(odometry)
        self.path_publisher.publish(path)

    def spin_pair(self, controller, duration=0.5, publish_inputs=False):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if publish_inputs:
                self.publish_inputs()
            rclpy.spin_once(self.driver, timeout_sec=0.01)
            rclpy.spin_once(controller, timeout_sec=0.01)

    def test_default_locks_publish_nothing(self):
        controller = self.make_controller(enabled=False, dry_run=True)
        try:
            self.spin_pair(controller, publish_inputs=True)
            self.assertEqual(self.commands, [])
        finally:
            controller.destroy_node()

    def test_armed_controller_stops_on_empty_path(self):
        controller = self.make_controller(enabled=True, dry_run=False)
        try:
            self.spin_pair(controller, duration=0.8, publish_inputs=True)
            self.assertTrue(any(command.linear.x > 0.0 for command in self.commands))

            empty_path = Path()
            empty_path.header.stamp = self.driver.get_clock().now().to_msg()
            empty_path.header.frame_id = "head_init"
            self.path_publisher.publish(empty_path)
            self.spin_pair(controller, duration=0.5)
            self.assertTrue(any(command.linear.x == 0.0 for command in self.commands))
        finally:
            controller.stop_before_shutdown()
            controller.destroy_node()


if __name__ == "__main__":
    unittest.main()
