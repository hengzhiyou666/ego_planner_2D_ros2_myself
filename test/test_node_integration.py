#!/usr/bin/env python3
# Copyright 2026 hengzhiyou
# SPDX-License-Identifier: GPL-3.0-only

import math
import struct
import time
import unittest

import launch
import launch.actions
import launch.events
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Bool, Int32


TEST_NAMESPACE = "planner_integration"
PLANNING_FRAME = "local_map_lidar_init_xyz"


def generate_test_description():
    planner = launch_ros.actions.Node(
        package="dog_ego_planner",
        executable="dog_planner_node",
        namespace=TEST_NAMESPACE,
        name="dog_ego_planner",
        parameters=[
            {
                "planning_frame": PLANNING_FRAME,
                "replan_freq": 20.0,
                "goal_threshold": 0.2,
                "local_path_max_m": 3.0,
                "local_path_min_m": 1.0,
                "global_path.initial_search_length_m": 10.0,
                "global_path.reacquire_distance_m": 1.0,
                "safety.odometry_timeout_s": 0.35,
                "safety.point_cloud_timeout_s": 0.5,
                "safety.max_odometry_message_age_s": 2.0,
                "safety.max_point_cloud_message_age_s": 2.0,
                "safety.max_cloud_odometry_skew_s": 0.05,
                "safety.require_fresh_obstacle_data": True,
                "direct_follow_require_lidar": True,
                "occupancy_publish_freq": 20.0,
            }
        ],
        output="screen",
    )
    shutdown = launch.actions.TimerAction(
        period=20.0,
        actions=[
            launch.actions.EmitEvent(
                event=launch.events.Shutdown(reason="integration test timeout")
            )
        ],
    )
    return (
        launch.LaunchDescription(
            [planner, launch_testing.actions.ReadyToTest(), shutdown]
        ),
        {"planner": planner},
    )


class TestPlannerScenarios(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("dog_ego_planner_integration_client")
        sensor_qos = QoSProfile(depth=10)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        odometry_qos = QoSProfile(depth=10)
        odometry_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos = QoSProfile(depth=1)
        transient_qos.reliability = ReliabilityPolicy.RELIABLE
        transient_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        prefix = f"/{TEST_NAMESPACE}"
        cls.odom_publisher = cls.node.create_publisher(
            Odometry, f"{prefix}/lidar_location_now", odometry_qos
        )
        cls.path_publisher = cls.node.create_publisher(
            Path, f"{prefix}/pct_path_copy", transient_qos
        )
        cls.cloud_publisher = cls.node.create_publisher(
            PointCloud2, f"{prefix}/lidar_points_copy", sensor_qos
        )

        cls.local_paths = []
        cls.unfinished_paths = []
        cls.goal_states = []
        cls.legacy_goal_states = []
        cls.occupancy_grids = []
        cls.subscriptions = [
            cls.node.create_subscription(
                Path,
                f"{prefix}/dog_output_local_path_copy",
                cls.local_paths.append,
                transient_qos,
            ),
            cls.node.create_subscription(
                Path,
                f"{prefix}/dog_output_global_path_unfinished_copy",
                cls.unfinished_paths.append,
                transient_qos,
            ),
            cls.node.create_subscription(
                Bool,
                f"{prefix}/goal_reached_copy",
                cls.goal_states.append,
                transient_qos,
            ),
            cls.node.create_subscription(
                Int32,
                f"{prefix}/if_reach_the_goal_copy",
                cls.legacy_goal_states.append,
                transient_qos,
            ),
            cls.node.create_subscription(
                OccupancyGrid,
                f"{prefix}/dog_2Dmap_occupancy_copy",
                cls.occupancy_grids.append,
                transient_qos,
            ),
        ]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def spin_until(self, predicate, timeout=3.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if predicate():
                return True
        return predicate()

    def publish_repeatedly(self, publisher, message_factory, duration=0.25):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            publisher.publish(message_factory())
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.02)

    def publish_sensor_pair(
        self, x, points, y=0.0, yaw=0.0, duration=0.25
    ):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.odom_publisher.publish(self.make_odometry(x, y=y, yaw=yaw))
            self.cloud_publisher.publish(self.make_cloud(points))
            rclpy.spin_once(self.node, timeout_sec=0.02)
            time.sleep(0.02)

    def make_odometry(self, x, y=0.0, yaw=0.0, frame=PLANNING_FRAME):
        message = Odometry()
        message.header.frame_id = frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(yaw * 0.5)
        return message

    def make_path(self, points, frame=PLANNING_FRAME):
        message = Path()
        message.header.frame_id = frame
        message.header.stamp = self.node.get_clock().now().to_msg()
        for x, y in points:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            message.poses.append(pose)
        return message

    def make_cloud(self, points):
        message = PointCloud2()
        message.header.frame_id = PLANNING_FRAME
        message.header.stamp = self.node.get_clock().now().to_msg()
        message.height = 1
        message.width = len(points)
        message.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = b"".join(struct.pack("<fff", *point) for point in points)
        return message

    @staticmethod
    def is_stop_path(message):
        if len(message.poses) != 2:
            return False
        first = message.poses[0].pose.position
        second = message.poses[1].pose.position
        return math.hypot(first.x - second.x, first.y - second.y) < 1e-6

    @staticmethod
    def is_moving_path(message):
        if len(message.poses) < 2:
            return False
        first = message.poses[0].pose.position
        last = message.poses[-1].pose.position
        return math.hypot(first.x - last.x, first.y - last.y) > 0.5

    def test_direct_path_obstacle_stop_recovery_and_timeouts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=10)
        time.sleep(0.3)

        self.publish_repeatedly(
            self.odom_publisher, lambda: self.make_odometry(0.0)
        )
        moving_count = sum(self.is_moving_path(path) for path in self.local_paths)
        self.path_publisher.publish(self.make_path([(0.0, 0.0), (3.0, 0.0)]))
        self.assertTrue(
            self.spin_until(
                lambda: self.local_paths
                and not self.is_moving_path(self.local_paths[-1])
            )
        )
        self.assertEqual(
            sum(self.is_moving_path(path) for path in self.local_paths),
            moving_count,
        )
        self.assertEqual(self.local_paths[-1].header.frame_id, PLANNING_FRAME)
        self.assertTrue(
            all(
                pose.header.frame_id == PLANNING_FRAME
                for pose in self.local_paths[-1].poses
            )
        )

        clear_observation = [
            (4.0, 3.0 + 0.05 * index, 0.5) for index in range(10)
        ]
        stop_count = sum(self.is_stop_path(path) for path in self.local_paths)
        time.sleep(0.1)
        self.cloud_publisher.publish(self.make_cloud(clear_observation))
        self.assertTrue(
            self.spin_until(
                lambda: sum(self.is_stop_path(path) for path in self.local_paths)
                > stop_count
            )
        )
        self.publish_sensor_pair(0.0, clear_observation)
        self.assertTrue(
            self.spin_until(
                lambda: sum(self.is_moving_path(path) for path in self.local_paths)
                > moving_count
            )
        )
        self.assertTrue(
            self.spin_until(
                lambda: self.occupancy_grids
                and -1 in self.occupancy_grids[-1].data
                and 0 in self.occupancy_grids[-1].data
            )
        )
        self.assertEqual(self.occupancy_grids[-1].header.frame_id, PLANNING_FRAME)

        stop_count = sum(self.is_stop_path(path) for path in self.local_paths)
        wall = [(1.0, -5.0 + 0.25 * index, 0.5) for index in range(41)]
        self.publish_sensor_pair(0.0, wall, duration=0.1)
        self.assertTrue(
            self.spin_until(
                lambda: sum(self.is_stop_path(path) for path in self.local_paths)
                > stop_count
            )
        )

        stop_count = sum(self.is_stop_path(path) for path in self.local_paths)
        self.publish_sensor_pair(0.0, [], duration=0.1)
        self.assertTrue(
            self.spin_until(
                lambda: sum(self.is_stop_path(path) for path in self.local_paths)
                > stop_count
            )
        )

        moving_count = sum(self.is_moving_path(path) for path in self.local_paths)
        self.publish_sensor_pair(0.0, clear_observation, duration=0.2)
        self.assertTrue(
            self.spin_until(
                lambda: sum(self.is_moving_path(path) for path in self.local_paths)
                > moving_count
            )
        )

        self.path_publisher.publish(self.make_path([]))
        self.assertTrue(
            self.spin_until(
                lambda: self.unfinished_paths and not self.unfinished_paths[-1].poses
            )
        )
        self.assertEqual(self.unfinished_paths[-1].header.frame_id, PLANNING_FRAME)
        self.assertTrue(self.is_stop_path(self.local_paths[-1]))
        self.assertTrue(
            self.spin_until(
                lambda: self.legacy_goal_states
                and self.legacy_goal_states[-1].data == 0
            )
        )

        moving_count = sum(self.is_moving_path(path) for path in self.local_paths)
        self.path_publisher.publish(self.make_path([(0.0, 0.0), (3.0, 0.0)]))
        self.publish_sensor_pair(0.0, clear_observation, duration=0.2)
        self.assertTrue(
            self.spin_until(
                lambda: sum(self.is_moving_path(path) for path in self.local_paths)
                > moving_count
            )
        )

        goal_count = len(self.goal_states)
        loop_path = [
            (0.0, 0.0),
            (1.0, 1.0),
            (2.0, 0.0),
            (1.0, -1.0),
            (0.0, 0.0),
        ]
        self.path_publisher.publish(self.make_path(loop_path))
        self.publish_sensor_pair(0.0, clear_observation, duration=0.2)
        self.assertTrue(
            self.spin_until(
                lambda: len(self.goal_states) > goal_count
                and not self.goal_states[-1].data
            )
        )

        self.path_publisher.publish(self.make_path([(0.0, 0.0), (3.0, 0.0)]))
        for position in (0.5, 1.0, 1.5, 2.0, 2.5):
            self.publish_sensor_pair(
                position, clear_observation, yaw=1.0, duration=0.1
            )
        self.publish_sensor_pair(
            3.0, clear_observation, yaw=1.0, duration=0.2
        )
        self.assertTrue(
            self.spin_until(
                lambda: self.goal_states
                and self.goal_states[-1].data
                and self.legacy_goal_states
                and self.legacy_goal_states[-1].data == 1
            )
        )
        self.assertTrue(
            self.spin_until(
                lambda: self.local_paths and self.is_stop_path(self.local_paths[-1])
            )
        )
        stop_orientation = self.local_paths[-1].poses[0].pose.orientation
        self.assertAlmostEqual(stop_orientation.z, math.sin(0.5), places=3)
        self.assertAlmostEqual(stop_orientation.w, math.cos(0.5), places=3)

        self.path_publisher.publish(self.make_path([(3.0, 0.0), (5.0, 0.0)]))
        self.publish_sensor_pair(3.0, clear_observation, duration=0.2)
        self.assertTrue(
            self.spin_until(
                lambda: self.local_paths and self.is_moving_path(self.local_paths[-1])
            )
        )
        self.assertTrue(
            self.spin_until(
                lambda: self.local_paths and not self.local_paths[-1].poses,
                timeout=1.5,
            )
        )

        self.odom_publisher.publish(
            self.make_odometry(3.0, frame="missing_odometry_frame")
        )
        self.assertTrue(
            self.spin_until(
                lambda: self.local_paths and not self.local_paths[-1].poses,
                timeout=1.5,
            )
        )


@launch_testing.post_shutdown_test()
class TestPlannerIntegrationShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info, planner):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=planner, allowable_exit_codes=[0, -2, -15]
        )
