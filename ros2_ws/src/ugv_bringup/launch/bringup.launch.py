"""Full system bring-up (Definition of Done, Section 24).

Gazebo + UGV + route + sensor fault injection + ESKF + baseline + metrics,
with optional RViz. Designed to be driven by experiments/run_experiment.py.
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    pkg_gazebo = FindPackageShare("ugv_gazebo")
    pkg_loc = FindPackageShare("ugv_localization")
    pkg_fault = FindPackageShare("ugv_fault_injector")
    pkg_bringup = FindPackageShare("ugv_bringup")
    pkg_desc = FindPackageShare("ugv_description")

    scenario = LaunchConfiguration("scenario_file")
    out_csv = LaunchConfiguration("output_csv")
    out_json = LaunchConfiguration("output_json")
    use_rviz = LaunchConfiguration("rviz")
    use_baseline = LaunchConfiguration("baseline")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo, "launch", "gazebo.launch.py"])
        )
    )
    fault = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_fault, "launch", "fault_injector.launch.py"])
        ),
        launch_arguments={"params_file": scenario}.items(),
    )
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_loc, "launch", "localization.launch.py"])
        )
    )

    commander = Node(
        package="ugv_bringup",
        executable="trajectory_commander",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    metrics = Node(
        package="ugv_metrics",
        executable="metrics_node",
        output="screen",
        parameters=[{"use_sim_time": True, "output_csv": out_csv, "output_json": out_json}],
    )

    baseline = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[PathJoinSubstitution([pkg_bringup, "config", "robot_localization.yaml"])],
        condition=IfCondition(use_baseline),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", PathJoinSubstitution([pkg_desc, "rviz", "ugv.rviz"])],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_file", default_value=""),
            DeclareLaunchArgument("output_csv", default_value="data.csv"),
            DeclareLaunchArgument("output_json", default_value="metrics.json"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("baseline", default_value="true"),
            gazebo,
            fault,
            localization,
            commander,
            metrics,
            baseline,
            rviz,
        ]
    )
