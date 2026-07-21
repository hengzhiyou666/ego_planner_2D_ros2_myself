# SPDX-License-Identifier: GPL-3.0-only
# Copyright 2026 hengzhiyou

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("dog_ego_planner")
    default_params_file = PathJoinSubstitution(
        [package_share, "config", "planner_params.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [package_share, "rviz", "dog_debug.rviz"]
    )

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    namespace = LaunchConfiguration("namespace")
    node_name = LaunchConfiguration("node_name")
    odometry_topic = LaunchConfiguration("odometry_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params_file,
                description="Absolute path to the planner parameter YAML file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use the ROS simulation clock from /clock.",
            ),
            DeclareLaunchArgument(
                "launch_rviz",
                default_value="false",
                description="Start RViz with the package debug configuration.",
            ),
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Optional ROS namespace for this planner instance.",
            ),
            DeclareLaunchArgument(
                "node_name",
                default_value="dog_ego_planner",
                description="ROS node name; the executable name remains unchanged.",
            ),
            DeclareLaunchArgument(
                "odometry_topic",
                default_value="/relocalizing/map_frame/odometry",
                description="Current LiDAR pose topic produced by self localization.",
            ),
            Node(
                package="dog_ego_planner",
                executable="dog_planner_node",
                namespace=namespace,
                name=node_name,
                output="screen",
                parameters=[
                    params_file,
                    {
                        "use_sim_time": use_sim_time,
                        "topics.odom": ParameterValue(
                            odometry_topic, value_type=str
                        ),
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                namespace=namespace,
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(launch_rviz),
            ),
        ]
    )
