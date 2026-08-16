#!/usr/bin/env python3
"""Bounded SIM-only bottom-camera marker X/Y/yaw fine alignment."""
from __future__ import annotations

import argparse
import json
import math
import threading
import time

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from tf.transformations import (
    euler_from_quaternion,
    quaternion_from_euler,
    quaternion_multiply,
)

MAX_OBSERVATION_AGE_S = 0.30
WALL_TIMEOUT_S = 25.0


class MarkerAligner:
    def __init__(self, marker_id: int, target_x: float, target_y: float) -> None:
        self.marker_id = marker_id
        self.target_x = target_x
        self.target_y = target_y
        self.lock = threading.Lock()
        self.observation = None
        self.publisher = rospy.Publisher(
            "/raicom/cmd_vel/relative", Twist, queue_size=1
        )
        rospy.Subscriber(
            "/raicom/workbench/markers", AlvarMarkers,
            self.on_markers, queue_size=1,
        )

    def on_markers(self, message: AlvarMarkers) -> None:
        selected = [marker for marker in message.markers
                    if marker.id == self.marker_id]
        if len(selected) != 1:
            return
        marker = selected[0]
        if marker.header.frame_id.lstrip("/") != "base_link":
            return
        position = marker.pose.pose.position
        q = marker.pose.pose.orientation
        target_q = quaternion_multiply(
            (q.x, q.y, q.z, q.w),
            quaternion_from_euler(-math.pi / 2.0, math.pi / 2.0, 0.0),
        )
        yaw = euler_from_quaternion(target_q)[2]
        values = (position.x, position.y, position.z, yaw)
        if not all(math.isfinite(value) for value in values):
            return
        with self.lock:
            self.observation = (time.monotonic(), values)

    def snapshot(self):
        with self.lock:
            return self.observation

    def zero(self) -> None:
        rate = rospy.Rate(30)
        for _ in range(10):
            self.publisher.publish(Twist())
            rate.sleep()

    def run(self) -> int:
        if rospy.get_param("/raicom/backend", "") != "SIM":
            print(json.dumps({"status": "REJECTED", "reason": "REAL_BACKEND"}))
            return 2
        deadline = time.monotonic() + WALL_TIMEOUT_S
        input_deadline = time.monotonic() + 5.0
        rate = rospy.Rate(30)
        status = "TIMEOUT"
        reason = "ALIGNMENT_NOT_CONVERGED"
        stable = 0
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                observed = self.snapshot()
                if observed is None:
                    if time.monotonic() < input_deadline:
                        rate.sleep()
                        continue
                    status, reason = "REJECTED", "MARKER_INPUT_TIMEOUT"
                    break
                if time.monotonic() - observed[0] > MAX_OBSERVATION_AGE_S:
                    status, reason = "REJECTED", "MARKER_STALE"
                    break
                x, y, _z, yaw = observed[1]
                error_x = x - self.target_x
                error_y = y - self.target_y
                if (abs(error_x) < 0.015 and abs(error_y) < 0.015
                        and abs(yaw) < math.radians(0.30)):
                    stable += 1
                    if stable >= 10:
                        status, reason = "ALIGNED", "POSE_WITHIN_TOLERANCE"
                        break
                else:
                    stable = 0
                command = Twist()
                command.linear.x = max(-0.055, min(0.055, error_x))
                command.linear.y = max(-0.055, min(0.055, error_y))
                angular = max(-0.10, min(0.10, 1.5 * yaw))
                if abs(yaw) >= math.radians(0.30) and abs(angular) < 0.035:
                    angular = math.copysign(0.035, angular)
                command.angular.z = angular
                self.publisher.publish(command)
                rate.sleep()
        finally:
            self.zero()
        observed = self.snapshot()
        values = None if observed is None else observed[1]
        report = {
            "status": status,
            "reason": reason,
            "sim_only": True,
            "marker_id": self.marker_id,
            "final_marker_base_m": None if values is None else values[:3],
            "forward_error_m": None if values is None else values[0] - self.target_x,
            "lateral_error_m": None if values is None else values[1] - self.target_y,
            "yaw_error_deg": None if values is None else math.degrees(values[3]),
            "output_zero": True,
        }
        print(json.dumps(report, sort_keys=True), flush=True)
        return 0 if status == "ALIGNED" else 2


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--marker-id", type=int, default=4)
    parser.add_argument("--target-x", type=float, default=0.415)
    parser.add_argument("--target-y", type=float, default=-0.013)
    args = parser.parse_args(rospy.myargv()[1:])
    if args.marker_id < 0 or not all(
        math.isfinite(value) for value in (args.target_x, args.target_y)
    ):
        parser.error("marker and target values are invalid")
    rospy.init_node("sim_workbench_marker_align", anonymous=True)
    raise SystemExit(
        MarkerAligner(args.marker_id, args.target_x, args.target_y).run()
    )


if __name__ == "__main__":
    main()
