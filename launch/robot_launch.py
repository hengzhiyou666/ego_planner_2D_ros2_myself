import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _read_launch_rviz(params_path: str) -> bool:
    if not os.path.isfile(params_path):
        return False
    with open(params_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not data or not isinstance(data, dict):
        return False
    rp = data.get("dog_ego_planner", {}).get("ros__parameters", {})
    return bool(rp.get("launch_rviz", False))


def _launch_setup(context, *_args, **_kwargs):
    params_file = context.perform_substitution(LaunchConfiguration("params_file"))
    use_sim_time_str = context.perform_substitution(LaunchConfiguration("use_sim_time"))
    use_sim_time = use_sim_time_str.lower() in ("true", "1", "yes")

    dog_node = Node(
        package="dog_ego_planner",
        executable="dog_planner_node",
        name="dog_ego_planner",
        output="screen",
        parameters=[
            params_file,
            {"use_sim_time": use_sim_time},
        ],
    )

    nodes = [dog_node]
    if _read_launch_rviz(params_file):
        rviz_config = context.perform_substitution(
            PathJoinSubstitution(
                [FindPackageShare("dog_ego_planner"), "rviz", "dog_debug.rviz"]
            )
        )
        nodes.append(
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
            )
        )
    return nodes


def generate_launch_description():
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("dog_ego_planner"), "config", "planner_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
