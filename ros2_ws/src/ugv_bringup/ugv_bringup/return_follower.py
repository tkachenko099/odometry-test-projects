#!/usr/bin/env python3
"""Pure-pursuit backtrack follower.

While the ``ugv_return_home`` failsafe is ``RETURNING``, this node steers the
robot along the published backtrack path (``/rth/return_path``) by commanding
``/cmd_vel``, closing the loop so the simulated vehicle physically drives home.
It yields control (stops) once the mission reaches ``HOME``.

Pose is taken from ground-truth odometry (same metric frame as the ENU path,
whose origin is the launch point). A classic pure-pursuit controller selects a
look-ahead target and issues proportional angular + capped linear velocity.
"""
from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import String


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    """Extract the ZYX yaw [rad] from a quaternion."""
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class ReturnFollower(Node):
    """Drives /cmd_vel to track the backtrack path during a return-home event."""

    def __init__(self) -> None:
        super().__init__("return_follower")
        self.declare_parameter("lookahead", 1.2)      # [m]
        self.declare_parameter("max_linear", 0.9)     # [m/s]
        self.declare_parameter("max_angular", 1.2)    # [rad/s]
        self.declare_parameter("goal_tolerance", 0.6)  # [m]
        self.declare_parameter("k_angular", 1.8)
        self.declare_parameter("rate_hz", 20.0)
        self.declare_parameter("odom_topic", "/ground_truth/odom")

        self._lookahead = self.get_parameter("lookahead").value
        self._max_lin = self.get_parameter("max_linear").value
        self._max_ang = self.get_parameter("max_angular").value
        self._goal_tol = self.get_parameter("goal_tolerance").value
        self._k_ang = self.get_parameter("k_angular").value
        rate = self.get_parameter("rate_hz").value

        self._path: list[tuple[float, float]] = []
        self._pose: tuple[float, float, float] | None = None
        self._state = "IDLE"
        self._target = 0
        self._done = False

        self._cmd = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Path, "/rth/return_path", self._on_path, 10)
        self.create_subscription(String, "/rth/mission_state", self._on_state, 10)
        self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self._on_odom, 10
        )
        self.create_timer(1.0 / max(1.0, rate), self._control)
        self.get_logger().info("return_follower ready (activates on RETURNING).")

    # --- Callbacks ----------------------------------------------------------
    def _on_path(self, msg: Path) -> None:
        self._path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self._target = 0
        self._done = False

    def _on_state(self, msg: String) -> None:
        if msg.data != self._state:
            self._state = msg.data
            if self._state == "RETURNING":
                self._target = 0
                self._done = False
                self.get_logger().warn("taking over /cmd_vel: driving backtrack home.")

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self._pose = (p.x, p.y, _yaw_from_quat(q.x, q.y, q.z, q.w))

    # --- Control loop -------------------------------------------------------
    def _control(self) -> None:
        if self._state != "RETURNING" or self._done or not self._path or self._pose is None:
            return
        x, y, yaw = self._pose
        gx, gy = self._path[-1]
        if math.hypot(gx - x, gy - y) <= self._goal_tol:
            self._stop()
            self._done = True
            self.get_logger().info("reached home; releasing /cmd_vel.")
            return

        # Advance the look-ahead target past points already within reach.
        while (
            self._target < len(self._path) - 1
            and math.hypot(self._path[self._target][0] - x, self._path[self._target][1] - y)
            < self._lookahead
        ):
            self._target += 1
        tx, ty = self._path[self._target]

        err = _wrap(math.atan2(ty - y, tx - x) - yaw)
        cmd = Twist()
        cmd.angular.z = max(-self._max_ang, min(self._max_ang, self._k_ang * err))
        # Slow down for sharp heading errors; ease in near the final goal.
        turn_scale = max(0.15, 1.0 - abs(err) / (math.pi / 2.0))
        cmd.linear.x = self._max_lin * turn_scale
        self._cmd.publish(cmd)

    def _stop(self) -> None:
        self._cmd.publish(Twist())


def main() -> None:
    rclpy.init()
    node = ReturnFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
