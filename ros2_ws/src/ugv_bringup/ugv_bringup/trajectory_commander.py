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
from rclpy.node import Node
from std_msgs.msg import Bool


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
        self._t = 0.0
        self._idx = 0
        self._seg_t = 0.0
        self._finished = False
        self._timer = self.create_timer(1.0 / self._rate, self._tick)
        self.get_logger().info(
            f"trajectory_commander: {len(ROUTE)} segments, "
            f"{sum(s.duration for s in ROUTE):.1f}s total."
        )

    def _tick(self) -> None:
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
