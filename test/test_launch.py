#!/usr/bin/env python3
# Copyright 2026 hengzhiyou
# SPDX-License-Identifier: GPL-3.0-only

import os
import unittest

import launch
import launch.actions
import launch.events
import launch_ros.actions
import launch_testing
import launch_testing.actions
import launch_testing.asserts
from ament_index_python.packages import get_package_share_directory


def generate_test_description():
    package_share = get_package_share_directory("dog_ego_planner")
    parameters = os.path.join(package_share, "config", "planner_params.yaml")
    planner = launch_ros.actions.Node(
        package="dog_ego_planner",
        executable="dog_planner_node",
        namespace="planner_smoke_test",
        name="dog_ego_planner",
        parameters=[parameters],
        output="screen",
    )
    shutdown = launch.actions.TimerAction(
        period=2.0,
        actions=[
            launch.actions.EmitEvent(
                event=launch.events.Shutdown(reason="launch smoke test complete")
            )
        ],
    )
    return (
        launch.LaunchDescription(
            [planner, launch_testing.actions.ReadyToTest(), shutdown]
        ),
        {"planner": planner},
    )


class TestPlannerRunning(unittest.TestCase):

    def test_process_starts(self, proc_info, planner):
        proc_info.assertWaitForStartup(process=planner, timeout=10)


@launch_testing.post_shutdown_test()
class TestPlannerShutdown(unittest.TestCase):

    def test_exit_code(self, proc_info, planner):
        launch_testing.asserts.assertExitCodes(
            proc_info, process=planner, allowable_exit_codes=[0, -2, -15]
        )
