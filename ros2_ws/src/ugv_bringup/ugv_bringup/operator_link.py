#!/usr/bin/env python3
"""Operator datalink heartbeat publisher.

Publishes ``std_msgs/Empty`` on ``/operator/heartbeat`` at ``rate_hz`` to emulate
a healthy operator link. If ``drop_after`` > 0, the heartbeat stops after that
many seconds to simulate a loss of communication, which trips the
``ugv_return_home`` failsafe. Set ``drop_after`` <= 0 to keep the link alive.
"""
from __future__ import annotations

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty


class OperatorLink(Node):
    """Emulates the operator's MAVLink heartbeat with a configurable dropout."""

    def __init__(self) -> None:
        super().__init__("operator_link")
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter("drop_after", 30.0)  # [s]; <=0 keeps the link up
        self._rate: float = self.get_parameter("rate_hz").value
        self._drop_after: float = self.get_parameter("drop_after").value
        self._pub = self.create_publisher(Empty, "/operator/heartbeat", 10)
        self._t = 0.0
        self._dropped = False
        self._timer = self.create_timer(1.0 / max(0.1, self._rate), self._tick)
        self.get_logger().info(
            f"operator_link: {self._rate:.1f} Hz heartbeat, "
            + (f"link drops at {self._drop_after:.0f}s." if self._drop_after > 0 else "no dropout.")
        )

    def _tick(self) -> None:
        self._t += 1.0 / max(0.1, self._rate)
        if self._drop_after > 0.0 and self._t >= self._drop_after:
            if not self._dropped:
                self._dropped = True
                self.get_logger().warn(
                    f"operator link LOST at t={self._t:.1f}s (heartbeat stopped)."
                )
            return
        self._pub.publish(Empty())


def main() -> None:
    rclpy.init()
    node = OperatorLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
