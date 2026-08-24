"""Publish the UGV robot_description (xacro -> URDF) via robot_state_publisher."""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")
    xacro_path = PathJoinSubstitution(
        [FindPackageShare("ugv_description"), "urdf", "ugv.urdf.xacro"]
    )
    robot_description = {
        "robot_description": Command(["xacro ", xacro_path]),
        "use_sim_time": use_sim_time,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
        ]
    )
