"""Bring up Gazebo Harmonic, spawn the UGV, and start the ros_gz bridge."""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_gazebo = FindPackageShare("ugv_gazebo")
    pkg_desc = FindPackageShare("ugv_description")
    pkg_ros_gz = FindPackageShare("ros_gz_sim")

    world = LaunchConfiguration("world")
    world_path = PathJoinSubstitution([pkg_gazebo, "worlds", world])
    bridge_config = PathJoinSubstitution([pkg_gazebo, "config", "bridge.yaml"])
    xacro_path = PathJoinSubstitution([pkg_desc, "urdf", "ugv.urdf.xacro"])

    # The xacro output is a URDF string; without an explicit type launch_ros
    # tries to parse it as YAML and aborts the whole bring-up.
    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", xacro_path]), value_type=str)
    }

    # headless:=true runs the server only (no GUI window) with off-screen
    # rendering, for containers / CI / batch experiments without a display.
    server_only = PythonExpression(
        ["' -s --headless-rendering' if '", LaunchConfiguration("headless"), "' == 'true' else ''"]
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_ros_gz, "launch", "gz_sim.launch.py"])
        ),
        launch_arguments={"gz_args": [world_path, " -r -v3", server_only]}.items(),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "ugv",
            "-topic", "/robot_description",
            "-x", "0", "-y", "0", "-z", "0.15",
        ],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output="screen",
        parameters=[{"config_file": bridge_config, "use_sim_time": True}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value="ugv_world.sdf"),
            DeclareLaunchArgument("headless", default_value="false"),
            gz_sim,
            robot_state_publisher,
            spawn,
            bridge,
        ]
    )
