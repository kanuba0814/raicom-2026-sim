#!/usr/bin/env python3
"""SIM-only one-shot chassis heading alignment from hand-camera markers 100/101."""
from __future__ import annotations

import json
import math
import threading
import time

import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

from raicom_workbench_vision import marker_baseline_pose

MARKER_IDS = (100, 101)
OBSERVATION_JOINTS_RAD = {
    "Joint1": math.radians(-42.0),
    "Joint2": 0.0,
    "Joint6": 0.0,
}
OBSERVATION_TOLERANCE_RAD = 0.03
SEPARATION_RANGE_M = (0.080, 0.110)
MAX_OBSERVATION_AGE_S = 0.30
HEADING_TOLERANCE_RAD = math.radians(0.20)
MIN_EFFECTIVE_ANGULAR_RAD_S = 0.035
MAX_ANGULAR_RAD_S = 0.060
GAIN = 1.4
STABLE_SAMPLES = 10
WALL_TIMEOUT_S = 12.0


class BaselineAligner:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._measurement = None
        self._joint_positions = None
        self._publisher = rospy.Publisher(
            "/raicom/cmd_vel/relative", Twist, queue_size=1
        )
        rospy.Subscriber(
            "/raicom/workbench/markers", AlvarMarkers,
            self._on_markers, queue_size=1,
        )
        rospy.Subscriber(
            "/swiftpro/joint_states", JointState,
            self._on_joint_state, queue_size=1,
        )

    def _on_markers(self, message: AlvarMarkers) -> None:
        by_id = {marker.id: marker for marker in message.markers
                 if marker.id in MARKER_IDS}
        if set(by_id) != set(MARKER_IDS):
            return
        positions = []
        for marker_id in MARKER_IDS:
            point = by_id[marker_id].pose.pose.position
            positions.append((point.x, point.y, point.z))
        measured = marker_baseline_pose(positions[0], positions[1])
        if measured is None or not (
            SEPARATION_RANGE_M[0]
            <= measured.separation_m
            <= SEPARATION_RANGE_M[1]
        ):
            return
        with self._lock:
            self._measurement = (time.monotonic(), measured)

    def _on_joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        if all(name in positions for name in OBSERVATION_JOINTS_RAD):
            with self._lock:
                self._joint_positions = positions

    def _snapshot(self):
        with self._lock:
            return self._measurement, self._joint_positions

    @staticmethod
    def _arm_is_fixed_observation(positions) -> bool:
        return positions is not None and all(
            abs(positions[name] - expected) <= OBSERVATION_TOLERANCE_RAD
            for name, expected in OBSERVATION_JOINTS_RAD.items()
        )

    def _publish_zero(self) -> None:
        rate = rospy.Rate(30)
        for _ in range(10):
            self._publisher.publish(Twist())
            rate.sleep()

    def run(self) -> int:
        if rospy.get_param("/raicom/backend", "") != "SIM":
            print(json.dumps({"status": "REJECTED", "reason": "REAL_BACKEND"}))
            return 2
        deadline = time.monotonic() + WALL_TIMEOUT_S
        input_deadline = time.monotonic() + 3.0
        stable = 0
        status = "TIMEOUT"
        reason = "HEADING_NOT_CONVERGED"
        rate = rospy.Rate(30)
        try:
            while not rospy.is_shutdown() and time.monotonic() < deadline:
                observation, joints = self._snapshot()
                if joints is None or observation is None:
                    if time.monotonic() < input_deadline:
                        rate.sleep()
                        continue
                    status, reason = "REJECTED", "INPUT_TIMEOUT"
                    break
                if not self._arm_is_fixed_observation(joints):
                    status, reason = "REJECTED", "ARM_NOT_FIXED_OBSERVATION"
                    break
                if time.monotonic() - observation[0] > MAX_OBSERVATION_AGE_S:
                    status, reason = "REJECTED", "BASELINE_MISSING_OR_STALE"
                    break
                measured = observation[1]
                if abs(measured.heading_rad) <= HEADING_TOLERANCE_RAD:
                    stable += 1
                    if stable >= STABLE_SAMPLES:
                        status, reason = "ALIGNED", "HEADING_WITHIN_TOLERANCE"
                        break
                else:
                    stable = 0
                command = max(
                    -MAX_ANGULAR_RAD_S,
                    min(MAX_ANGULAR_RAD_S, GAIN * measured.heading_rad),
                )
                if (abs(measured.heading_rad) > HEADING_TOLERANCE_RAD
                        and abs(command) < MIN_EFFECTIVE_ANGULAR_RAD_S):
                    command = math.copysign(MIN_EFFECTIVE_ANGULAR_RAD_S, command)
                message = Twist()
                message.angular.z = command
                self._publisher.publish(message)
                rate.sleep()
        finally:
            self._publish_zero()

        observation, joints = self._snapshot()
        measured = observation[1] if observation is not None else None
        report = {
            "status": status,
            "reason": reason,
            "sim_only": True,
            "arm_fixed_observation": self._arm_is_fixed_observation(joints),
            "heading_error_deg": None if measured is None else math.degrees(measured.heading_rad),
            "signed_lateral_origin_to_line_mm": None if measured is None else measured.signed_lateral_m * 1000.0,
            "along_from_100_to_origin_projection_mm": None if measured is None else measured.along_from_first_m * 1000.0,
            "baseline_length_mm": None if measured is None else measured.separation_m * 1000.0,
            "lateral_and_along_controlled": False,
            "output_zero": True,
        }
        print(json.dumps(report, sort_keys=True), flush=True)
        return 0 if status == "ALIGNED" else 2


def main() -> None:
    rospy.init_node("sim_marker_baseline_align", anonymous=True)
    raise SystemExit(BaselineAligner().run())


if __name__ == "__main__":
    main()
