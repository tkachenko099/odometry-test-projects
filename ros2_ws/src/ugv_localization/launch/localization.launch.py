"""Launch the online ESKF estimator."""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    params = LaunchConfiguration("params_file")
    default_params = PathJoinSubstitution(
        [FindPackageShare("ugv_localization"), "config", "eskf.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_params),
            Node(
                package="ugv_localization",
                executable="eskf_node",
                name="eskf_node",
                output="screen",
                parameters=[params],
            ),
        ]
    )
