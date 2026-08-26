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
    use_return_home = LaunchConfiguration("return_home")
    rth_position_topic = LaunchConfiguration("rth_position_topic")
    rth_drop_after = LaunchConfiguration("rth_drop_after")

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

    # --- Return-home-on-link-loss failsafe (optional) ------------------------
    operator_link = Node(
        package="ugv_bringup",
        executable="operator_link",
        output="screen",
        parameters=[{"use_sim_time": True, "rate_hz": 2.0, "drop_after": rth_drop_after}],
        condition=IfCondition(use_return_home),
    )
    return_home = Node(
        package="ugv_return_home",
        executable="rth_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": True,
                "position_topic": rth_position_topic,
                "heartbeat_topic": "/operator/heartbeat",
                "min_record_distance": 0.5,
                "arrival_radius": 1.0,
                "link_timeout": 3.0,
                "resume_on_recovery": True,
                "tick_rate": 5.0,
                "use_mavros": False,
            }
        ],
        condition=IfCondition(use_return_home),
    )
    # Closes the loop: physically drives the robot along the backtrack path.
    return_follower = Node(
        package="ugv_bringup",
        executable="return_follower",
        output="screen",
        parameters=[{"use_sim_time": True, "odom_topic": "/ground_truth/odom"}],
        condition=IfCondition(use_return_home),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("scenario_file", default_value=""),
            DeclareLaunchArgument("output_csv", default_value="data.csv"),
            DeclareLaunchArgument("output_json", default_value="metrics.json"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("baseline", default_value="true"),
            DeclareLaunchArgument("return_home", default_value="false"),
            DeclareLaunchArgument("rth_position_topic", default_value="/gps/fix"),
            DeclareLaunchArgument("rth_drop_after", default_value="30.0"),
            gazebo,
            fault,
            localization,
            commander,
            metrics,
            baseline,
            rviz,
            operator_link,
            return_home,
            return_follower,
        ]
    )
