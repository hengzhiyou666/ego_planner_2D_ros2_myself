# SPDX-License-Identifier: GPL-3.0-only
# Copyright 2026 hengzhiyou

"""Compatibility wrapper for the historical launch-file name."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("dog_ego_planner")
    default_params_file = PathJoinSubstitution(
        [package_share, "config", "planner_params.yaml"]
    )
    canonical_launch = PathJoinSubstitution(
        [package_share, "launch", "dog_ego_planner.launch.py"]
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    namespace = LaunchConfiguration("namespace")
    node_name = LaunchConfiguration("node_name")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="false"),
            DeclareLaunchArgument("namespace", default_value=""),
            DeclareLaunchArgument("node_name", default_value="dog_ego_planner"),
            LogInfo(
                msg=(
                    "robot_launch.py is deprecated; use "
                    "dog_ego_planner.launch.py instead."
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(canonical_launch),
                launch_arguments={
                    "params_file": params_file,
                    "use_sim_time": use_sim_time,
                    "launch_rviz": launch_rviz,
                    "namespace": namespace,
                    "node_name": node_name,
                }.items(),
            ),
        ]
    )
