#!/usr/bin/env python3
"""Scripted operator for the SITL return-home test (run_sitl_failsafe.sh).

Plays the human at the ground station against ArduRover SITL through mavros:

  1. wait for the autopilot link and a GNSS position,
  2. arm, switch to MANUAL and drive out with RC overrides while publishing the
     operator heartbeat,
  3. cut the heartbeat (the "datalink loss") and release the sticks,
  4. watch ``rth_node`` take over and check that the vehicle really comes home.

Exit status: 0 if the vehicle ends within ``--home-radius`` of the launch point
with the mission in HOME, 1 otherwise.
"""
from __future__ import annotations

import argparse
import math
import sys

import rclpy
from mavros_msgs.msg import OverrideRCIn, State
from mavros_msgs.srv import CommandBool, SetMode, StreamRate
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Empty, String

EARTH_RADIUS = 6_371_000.0
RC_RELEASE = 0        # hand the channel back to the autopilot's own RC input
RC_IGNORE = 65535     # leave the channel untouched
RC_NEUTRAL = 1500


class SitlOperator(Node):
    def __init__(self) -> None:
        super().__init__("sitl_operator")
        self.state: State | None = None
        self.fix: NavSatFix | None = None
        self.home: NavSatFix | None = None
        self.mission = "-"
        self.max_dist = 0.0

        self.create_subscription(State, "/mavros/state", self._on_state, 10)
        self.create_subscription(
            NavSatFix, "/mavros/global_position/global", self._on_fix, qos_profile_sensor_data
        )
        self.create_subscription(String, "/rth/mission_state", self._on_mission, 10)
        self._heartbeat = self.create_publisher(Empty, "/operator/heartbeat", 10)
        self._rc = self.create_publisher(OverrideRCIn, "/mavros/rc/override", 10)
        self._arming = self.create_client(CommandBool, "/mavros/cmd/arming")
        self._set_mode = self.create_client(SetMode, "/mavros/set_mode")
        self._stream = self.create_client(StreamRate, "/mavros/set_stream_rate")
        # The heartbeat runs on its own timer so that a slow service call can
        # never look like a datalink loss.
        self.link_up = False
        self.create_timer(0.2, self._beat)

    def _beat(self) -> None:
        if self.link_up:
            self._heartbeat.publish(Empty())

    # --- callbacks -----------------------------------------------------------
    def _on_state(self, msg: State) -> None:
        self.state = msg

    def _on_fix(self, msg: NavSatFix) -> None:
        if math.isnan(msg.latitude) or math.isnan(msg.longitude):
            return
        if msg.status.status < NavSatStatus.STATUS_FIX:
            return  # no fix yet: the autopilot reports lat = lon = 0
        self.fix = msg
        if self.home is not None:
            self.max_dist = max(self.max_dist, self.distance_home())

    def _on_mission(self, msg: String) -> None:
        self.mission = msg.data

    # --- helpers -------------------------------------------------------------
    def now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def spin_for(self, seconds: float) -> None:
        end = self.now() + seconds
        while rclpy.ok() and self.now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_until(self, predicate, timeout: float, what: str) -> None:
        end = self.now() + timeout
        while rclpy.ok() and not predicate():
            if self.now() > end:
                raise TimeoutError(f"timed out after {timeout:.0f}s waiting for {what}")
            rclpy.spin_once(self, timeout_sec=0.1)

    def call(self, client, request, what: str):
        if not client.wait_for_service(timeout_sec=20.0):
            raise TimeoutError(f"service for {what} is not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if future.result() is None:
            raise TimeoutError(f"no response to {what}")
        return future.result()

    def distance_home(self) -> float:
        if self.fix is None or self.home is None:
            return float("nan")
        lat0 = math.radians(self.home.latitude)
        north = math.radians(self.fix.latitude - self.home.latitude) * EARTH_RADIUS
        east = math.radians(self.fix.longitude - self.home.longitude) * EARTH_RADIUS * math.cos(lat0)
        return math.hypot(north, east)

    def rc(self, steering: int, throttle: int) -> None:
        msg = OverrideRCIn()
        msg.channels = [RC_IGNORE] * len(msg.channels)
        msg.channels[0] = steering   # ArduRover: channel 1 = steering
        msg.channels[2] = throttle   # ArduRover: channel 3 = throttle
        self._rc.publish(msg)

    def set_mode(self, mode: str) -> None:
        res = self.call(self._set_mode, SetMode.Request(custom_mode=mode), f"SET_MODE {mode}")
        if not res.mode_sent:
            raise RuntimeError(f"autopilot rejected mode {mode}")

    def log(self, text: str) -> None:
        print(f"[operator] {text}", flush=True)


def run(op: SitlOperator, args: argparse.Namespace) -> bool:
    op.wait_until(lambda: op.state is not None and op.state.connected, 120.0, "the FCU link")
    op.log("autopilot connected")
    # ArduPilot only streams what the ground station asks for.
    op.call(op._stream, StreamRate.Request(stream_id=0, message_rate=10, on_off=True),
            "stream rates")
    op.wait_until(lambda: op.fix is not None, 120.0, "a GNSS position")
    op.home = op.fix
    op.log(f"launch point lat={op.home.latitude:.6f} lon={op.home.longitude:.6f}")

    # Heartbeat first, so rth_node sees a healthy link before the vehicle moves.
    def beat_for(seconds: float, steering: int, throttle: int) -> None:
        end = op.now() + seconds
        while rclpy.ok() and op.now() < end:
            op.rc(steering, throttle)
            op.spin_for(0.1)

    op.link_up = True
    op.set_mode("MANUAL")
    op.log("arming (retrying until the pre-arm checks pass)...")
    deadline = op.now() + 180.0
    while not op.state.armed:
        if op.now() > deadline:
            raise TimeoutError("vehicle did not arm within 180s")
        op.call(op._arming, CommandBool.Request(value=True), "arming")
        beat_for(3.0, RC_NEUTRAL, RC_NEUTRAL)
    op.log("armed; driving out under operator control")

    beat_for(args.drive_time * 0.5, RC_NEUTRAL, args.throttle)        # straight
    beat_for(args.drive_time * 0.2, RC_NEUTRAL + 250, args.throttle)  # turn
    beat_for(args.drive_time * 0.3, RC_NEUTRAL, args.throttle)        # straight
    out = op.distance_home()
    op.log(f"drove out to {out:.1f} m from the launch point (mission={op.mission})")
    if out < args.min_distance:
        raise RuntimeError(f"vehicle only moved {out:.1f} m; nothing to backtrack")

    op.link_up = False
    op.log("DATALINK LOST: heartbeat stopped, sticks released")
    op.rc(RC_RELEASE, RC_RELEASE)

    op.wait_until(lambda: op.mission == "RETURNING", 30.0, "the failsafe to trigger")
    op.log(f"failsafe triggered, autopilot mode={op.state.mode}")

    end = op.now() + args.return_timeout
    next_report = 0.0
    while rclpy.ok() and op.now() < end and op.mission != "HOME":
        op.spin_for(0.2)
        if op.now() >= next_report:
            next_report = op.now() + 5.0
            op.log(f"  mode={op.state.mode:8s} mission={op.mission:9s} "
                   f"distance home={op.distance_home():6.1f} m")
    op.spin_for(5.0)  # let it settle, and catch a vehicle that keeps going

    final = op.distance_home()
    ok = op.mission == "HOME" and final <= args.home_radius
    op.log(f"final: mission={op.mission} mode={op.state.mode} distance home={final:.1f} m "
           f"(max {op.max_dist:.1f} m)")
    op.log("RESULT: PASS - vehicle backtracked home." if ok
           else "RESULT: FAIL - vehicle did not come home.")
    return ok


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--drive-time", type=float, default=25.0, help="outbound drive [s]")
    parser.add_argument("--throttle", type=int, default=1750, help="RC throttle PWM while driving")
    parser.add_argument("--min-distance", type=float, default=15.0,
                        help="required outbound distance [m]")
    parser.add_argument("--home-radius", type=float, default=5.0,
                        help="pass if the vehicle ends this close to the launch point [m]")
    parser.add_argument("--return-timeout", type=float, default=180.0, help="[s]")
    args = parser.parse_args()

    rclpy.init()
    op = SitlOperator()
    ok = False
    try:
        ok = run(op, args)
    except (TimeoutError, RuntimeError) as exc:
        op.log(f"RESULT: FAIL - {exc}")
    except KeyboardInterrupt:
        pass
    finally:
        op.destroy_node()
        rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
