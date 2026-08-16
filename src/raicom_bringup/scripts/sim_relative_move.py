#!/usr/bin/env python3
"""Bounded SIM-only odometry-relative base translation through the safety gate."""
from __future__ import annotations

import argparse
import json
import math
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

MAX_DISTANCE_M = 0.35
MAX_SPEED_M_S = 0.08
MIN_EFFECTIVE_SPEED_M_S = 0.035
WALL_TIMEOUT_S = 12.0


def yaw_from_odometry(message: Odometry) -> float:
    q = message.pose.pose.orientation
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class RelativeMover:
    def __init__(self, distance_m: float, lateral_m: float) -> None:
        self.distance_m = distance_m
        self.lateral_m = lateral_m
        self.publisher = rospy.Publisher(
            "/raicom/cmd_vel/relative", Twist, queue_size=1
        )

    def zero(self) -> None:
        rate = rospy.Rate(30)
        for _ in range(10):
            self.publisher.publish(Twist())
            rate.sleep()

    def run(self) -> int:
        if rospy.get_param("/raicom/backend", "") != "SIM":
            print(json.dumps({"status": "REJECTED", "reason": "REAL_BACKEND"}))
            return 2
        start = rospy.wait_for_message("/odom", Odometry, timeout=5.0)
        start_x = start.pose.pose.position.x
        start_y = start.pose.pose.position.y
        start_yaw = yaw_from_odometry(start)
        requested = self.distance_m if self.distance_m != 0.0 else self.lateral_m
        direction = math.copysign(1.0, requested)
        target = abs(requested)
        forward = lateral = 0.0
        deadline = time.monotonic() + WALL_TIMEOUT_S
        rate = rospy.Rate(30)
        status = "TIMEOUT"
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                current = rospy.wait_for_message("/odom", Odometry, timeout=1.0)
                dx = current.pose.pose.position.x - start_x
                dy = current.pose.pose.position.y - start_y
                signed_forward = math.cos(start_yaw) * dx + math.sin(start_yaw) * dy
                forward = direction * signed_forward
                lateral = -math.sin(start_yaw) * dx + math.cos(start_yaw) * dy
                progress = direction * (signed_forward if self.distance_m != 0.0 else lateral)
                cross_track = lateral if self.distance_m != 0.0 else signed_forward
                if progress >= target - 0.004:
                    status = "COMPLETE"
                    break
                remaining = target - progress
                speed = min(
                    MAX_SPEED_M_S,
                    max(MIN_EFFECTIVE_SPEED_M_S, 1.5 * remaining),
                )
                command = Twist()
                if self.distance_m != 0.0:
                    command.linear.x = direction * speed
                    command.linear.y = max(-0.025, min(0.025, -1.2 * cross_track))
                else:
                    command.linear.y = direction * speed
                    command.linear.x = max(-0.025, min(0.025, -1.2 * cross_track))
                self.publisher.publish(command)
                rate.sleep()
        finally:
            self.zero()
        report = {
            "status": status,
            "sim_only": True,
            "target_forward_m": self.distance_m,
            "target_lateral_m": self.lateral_m,
            "actual_forward_m": signed_forward,
            "actual_lateral_m": lateral,
            "output_zero": True,
        }
        print(json.dumps(report, sort_keys=True), flush=True)
        return 0 if status == "COMPLETE" else 2


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("distance_m", type=float, nargs="?", default=0.0)
    parser.add_argument("--lateral-m", type=float, default=0.0)
    args = parser.parse_args(rospy.myargv()[1:])
    values = (args.distance_m, args.lateral_m)
    if (not all(math.isfinite(value) for value in values)
            or sum(value != 0.0 for value in values) != 1
            or not 0.01 <= max(abs(value) for value in values) <= MAX_DISTANCE_M):
        parser.error("choose exactly one forward/lateral magnitude within [0.01, 0.35]")
    rospy.init_node("sim_relative_move", anonymous=True)
    raise SystemExit(RelativeMover(args.distance_m, args.lateral_m).run())


if __name__ == "__main__":
    main()
