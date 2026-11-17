"""Regression tests for the /cmd_vel hand-over around a return-home event.

The bug they pin down: after the mission reached HOME nobody published
/cmd_vel any more, and Gazebo's DiffDrive kept executing the follower's last
non-zero command - the vehicle "arrived" and then drove away indefinitely.
"""
from __future__ import annotations

import sys
from pathlib import Path

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path as PathMsg
from std_msgs.msg import String

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ugv_bringup"))

from return_follower import ReturnFollower
from trajectory_commander import TrajectoryCommander


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


class Capture:
    """Stands in for a publisher and records what would have been sent."""

    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def _is_stop(twist) -> bool:
    return twist.linear.x == 0.0 and twist.angular.z == 0.0


def _path(points) -> PathMsg:
    msg = PathMsg()
    for x, y in points:
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = float(x), float(y)
        msg.poses.append(pose)
    return msg


def _odom(x, y) -> Odometry:
    msg = Odometry()
    msg.pose.pose.position.x, msg.pose.pose.position.y = float(x), float(y)
    msg.pose.pose.orientation.w = 1.0
    return msg


@pytest.fixture
def follower():
    node = ReturnFollower()
    node._cmd = Capture()
    yield node
    node.destroy_node()


@pytest.fixture
def commander():
    node = TrajectoryCommander()
    node._cmd_pub = Capture()
    yield node
    node.destroy_node()


def test_follower_drives_while_returning(follower):
    follower._on_path(_path([(10, 0), (5, 0), (0, 0)]))
    follower._on_odom(_odom(10, 5))
    follower._on_state(String(data="RETURNING"))
    follower._control()
    assert follower._cmd.sent and not _is_stop(follower._cmd.sent[-1])


def test_follower_stops_when_mission_declares_home_first(follower):
    # MissionController's arrival_radius trips while the follower is still
    # outside its own goal tolerance and mid-command.
    follower._on_path(_path([(10, 0), (5, 0), (0, 0)]))
    follower._on_odom(_odom(1.5, 0))
    follower._on_state(String(data="RETURNING"))
    follower._control()
    assert not _is_stop(follower._cmd.sent[-1])

    follower._on_state(String(data="HOME"))
    assert _is_stop(follower._cmd.sent[-1])  # stop sent on the transition itself

    # ...and the stop is held for as long as the mission stays HOME.
    follower._cmd.sent.clear()
    for _ in range(5):
        follower._control()
    assert len(follower._cmd.sent) == 5 and all(_is_stop(t) for t in follower._cmd.sent)


@pytest.mark.parametrize("next_state", ["RECORDING", "IDLE"])
def test_follower_stops_once_then_releases_on_abort_or_reset(follower, next_state):
    follower._on_path(_path([(10, 0), (0, 0)]))
    follower._on_odom(_odom(8, 1))
    follower._on_state(String(data="RETURNING"))
    follower._control()
    follower._on_state(String(data=next_state))
    assert _is_stop(follower._cmd.sent[-1])

    follower._cmd.sent.clear()
    follower._control()
    assert follower._cmd.sent == []  # /cmd_vel belongs to someone else again


def test_commander_yields_and_takes_cmd_vel_back(commander):
    commander._tick()
    assert len(commander._cmd_pub.sent) == 1

    commander._on_state(String(data="RETURNING"))
    assert _is_stop(commander._cmd_pub.sent[-1])
    commander._on_state(String(data="HOME"))
    sent = len(commander._cmd_pub.sent)
    commander._tick()
    assert len(commander._cmd_pub.sent) == sent  # silent while the failsafe owns it

    commander._on_state(String(data="RECORDING"))  # aborted or reset
    commander._tick()
    assert len(commander._cmd_pub.sent) == sent + 1  # route resumes
