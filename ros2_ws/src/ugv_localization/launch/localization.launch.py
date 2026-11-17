"""Launch the online ESKF estimator."""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # NOTE: launch configurations are global, not scoped per included file, so
    # a generic name like "params_file" leaks between includes (the estimator
    # used to silently receive the fault injector's YAML). Keep it unique.
    params = LaunchConfiguration("eskf_params_file")
    default_params = PathJoinSubstitution(
        [FindPackageShare("ugv_localization"), "config", "eskf.yaml"]
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("eskf_params_file", default_value=default_params),
            Node(
                package="ugv_localization",
                executable="eskf_node",
                name="eskf_node",
                output="screen",
                parameters=[params],
            ),
        ]
    )
