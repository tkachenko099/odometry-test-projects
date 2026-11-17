#!/usr/bin/env python3
"""Deterministic route commander (Section 15).

Publishes a fixed ``/cmd_vel`` profile so every experiment traverses an
identical path: straight → 90° turn → straight → several turns → circular
arc → stop → re-accelerate. Signals completion on ``/route/done``.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String


@dataclass(frozen=True)
class Segment:
    """A constant-twist motion primitive."""

    linear: float  # [m/s]
    angular: float  # [rad/s]
    duration: float  # [s]
    label: str = ""


# 90° turn at 0.5 rad/s takes π/2 / 0.5 ≈ 3.14 s; full circle at r≈2 m.
ROUTE: tuple[Segment, ...] = (
    Segment(0.0, 0.0, 1.0, "settle"),
    Segment(1.0, 0.0, 8.0, "straight-1"),
    Segment(0.0, 0.5, math.pi / 2 / 0.5, "turn-90"),
    Segment(1.0, 0.0, 8.0, "straight-2"),
    Segment(0.8, 0.4, 4.0, "S-curve-a"),
    Segment(0.8, -0.4, 4.0, "S-curve-b"),
    Segment(1.0, 0.5, 2 * math.pi / 0.5, "circle"),
    Segment(0.0, 0.0, 3.0, "stop"),
    Segment(1.0, 0.0, 6.0, "re-accelerate"),
    Segment(0.0, 0.0, 1.0, "halt"),
)


class TrajectoryCommander(Node):
    """Streams the fixed route as Twist commands at a fixed rate."""

    def __init__(self) -> None:
        super().__init__("trajectory_commander")
        self.declare_parameter("rate_hz", 20.0)
        self._rate: float = self.get_parameter("rate_hz").value
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._done_pub = self.create_publisher(Bool, "/route/done", 1)
        # Yield /cmd_vel to the return-home follower once a failsafe is active.
        self._yielded = False
        self.create_subscription(String, "/rth/mission_state", self._on_state, 10)
        self._t = 0.0
        self._idx = 0
        self._seg_t = 0.0
        self._finished = False
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"trajectory_commander: {len(ROUTE)} segments, "
            f"{sum(s.duration for s in ROUTE):.1f}s total."
        )

    def _on_state(self, msg: String) -> None:
        failsafe = msg.data in ("RETURNING", "HOME")
        if failsafe and not self._yielded:
            self._yielded = True
            self._cmd_pub.publish(Twist())  # release with a stop command
            self.get_logger().warn("return-home active: yielding /cmd_vel.")
        elif not failsafe and self._yielded:
            # Return aborted (link recovered) or mission reset: take /cmd_vel
            # back and continue the route from where it was interrupted.
            self._yielded = False
            self.get_logger().info("return-home released: resuming the route.")

    def _tick(self) -> None:
        if self._yielded:
            return  # follower owns /cmd_vel during the return
        dt = 1.0 / self._rate
        msg = Twist()
        if self._idx < len(ROUTE):
            seg = ROUTE[self._idx]
            msg.linear.x = seg.linear
            msg.angular.z = seg.angular
            self._seg_t += dt
            if self._seg_t >= seg.duration:
                self._idx += 1
                self._seg_t = 0.0
        elif not self._finished:
            self._finished = True
            self.get_logger().info("route complete.")
        self._cmd_pub.publish(msg)
        done = Bool()
        done.data = self._finished
        self._done_pub.publish(done)


def main() -> None:
    rclpy.init()
    node = TrajectoryCommander()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl-C / launch shutdown: exit cleanly, not with a traceback
    except Exception:
        if rclpy.ok():
            raise  # a real failure; a timer publish racing shutdown is not
    finally:
        try:
            node.destroy_node()
            rclpy.try_shutdown()
        except KeyboardInterrupt:
            pass  # a second Ctrl-C arriving during teardown


if __name__ == "__main__":
    main()
