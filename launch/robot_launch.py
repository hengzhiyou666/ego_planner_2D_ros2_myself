from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    debug = LaunchConfiguration("debug")
    params_file = LaunchConfiguration("params_file")

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("dog_ego_planner"), "rviz", "dog_debug.rviz"]
    )
    default_params_file = PathJoinSubstitution(
        [FindPackageShare("dog_ego_planner"), "config", "planner_params.yaml"]
    )

    dog_node = Node(
        package="dog_ego_planner",
        executable="dog_planner_node",
        name="dog_ego_planner",
        output="screen",
        parameters=[
            params_file,
            {"debug": debug},
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_dog_debug",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(debug),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("debug", default_value="true"),
            DeclareLaunchArgument("params_file", default_value=default_params_file),
            dog_node,
            rviz_node,
        ]
    )

