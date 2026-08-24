"""Launch the fault injector, optionally with a scenario parameter file."""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _spawn(context, *_, **__):
    params_file = LaunchConfiguration("params_file").perform(context)
    return [
        Node(
            package="ugv_fault_injector",
            executable="fault_injector_node",
            name="fault_injector_node",
            output="screen",
            parameters=[params_file] if params_file else [],
        )
    ]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=""),
            OpaqueFunction(function=_spawn),
        ]
    )
