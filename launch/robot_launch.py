from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    debug = LaunchConfiguration("debug")

    goal_threshold = LaunchConfiguration("goal_threshold")
    pose_change_thresh = LaunchConfiguration("pose_change_thresh")
    obs_change_thresh = LaunchConfiguration("obs_change_thresh")

    planning_horizon = LaunchConfiguration("planning_horizon")
    control_point_interval = LaunchConfiguration("control_point_interval")
    replan_freq = LaunchConfiguration("replan_freq")
    local_traj_duration = LaunchConfiguration("local_traj_duration")

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("dog_ego_planner"), "planner", "plan_manage", "rviz", "dog_debug.rviz"]
    )

    dog_node = Node(
        package="dog_ego_planner",
        executable="dog_planner_node",
        name="dog_ego_planner",
        output="screen",
        parameters=[
            {"debug": debug},
            {"goal_threshold": goal_threshold},
            {"pose_change_thresh": pose_change_thresh},
            {"obs_change_thresh": obs_change_thresh},
            {"planning_horizon": planning_horizon},
            {"control_point_interval": control_point_interval},
            {"replan_freq": replan_freq},
            {"local_traj_duration": local_traj_duration},
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
            DeclareLaunchArgument("goal_threshold", default_value="0.1"),
            DeclareLaunchArgument("pose_change_thresh", default_value="0.05"),
            DeclareLaunchArgument("obs_change_thresh", default_value="0.1"),
            DeclareLaunchArgument("planning_horizon", default_value="7.0"),
            DeclareLaunchArgument("control_point_interval", default_value="0.3"),
            DeclareLaunchArgument("replan_freq", default_value="50.0"),
            DeclareLaunchArgument("local_traj_duration", default_value="1.0"),
            dog_node,
            rviz_node,
        ]
    )

