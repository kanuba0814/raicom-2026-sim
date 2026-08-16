#!/usr/bin/env python3
"""One-shot SIM restore-to-home then base-only observation rotation."""

import json
import math
import threading

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from raicom_swiftpro_task_sim.kinematics import DEG, display_projection
from raicom_swiftpro_task_sim.motion_adapter import (
    CONTROLLER_JOINT_NAMES,
    FEEDBACK_JOINT_NAMES,
    JointObservation,
    MotionStatus,
    STEP_SECONDS,
    SwiftproSimMotionAdapter,
)


HOME_INDEPENDENT_RAD = (90.0 * DEG, 90.0 * DEG, 3.8 * DEG)
BASE_ONLY_OBSERVATION_RAD = (48.0 * DEG, 90.0 * DEG, 3.8 * DEG)
POST_TRAJECTORY_WAIT_S = 1.0


class BaseOnlyObservationRunner:
    def __init__(self):
        self._lock = threading.Lock()
        self._latest = None
        self._publishers = {
            name: rospy.Publisher(
                "/swiftpro/%s_position_controller/command" % name,
                Float64,
                queue_size=1,
            )
            for name in CONTROLLER_JOINT_NAMES
        }
        self._subscriber = rospy.Subscriber(
            "/swiftpro/joint_states", JointState, self._on_joint_state, queue_size=1
        )
        self._arm_extended = rospy.Publisher(
            "/raicom/safety/arm_extended", Bool, queue_size=1, latch=True
        )

    def _on_joint_state(self, message):
        observation = JointObservation(
            tuple(message.name), tuple(message.position),
            tuple(message.velocity), message.header.stamp.to_sec()
        )
        with self._lock:
            self._latest = observation

    def latest(self):
        with self._lock:
            return self._latest

    def publish_projection(self, projection):
        for controller, value in zip(CONTROLLER_JOINT_NAMES, projection):
            self._publishers[controller].publish(Float64(data=value))

    def wait_for_controller_connections(self):
        deadline = rospy.Time.now().to_sec() + 3.0
        while not rospy.is_shutdown():
            missing = [
                name for name, publisher in self._publishers.items()
                if publisher.get_num_connections() < 1
            ]
            if not missing:
                return True
            if rospy.Time.now().to_sec() >= deadline:
                return False
            rospy.sleep(0.02)
        return False

    def run_plan(self, target):
        rospy.sleep(0.2)
        planned = SwiftproSimMotionAdapter().plan(
            target, self.latest(), rospy.Time.now().to_sec()
        )
        if planned.status is not MotionStatus.READY or planned.plan is None:
            return planned.reason, None
        for projection in planned.plan.commands:
            if rospy.is_shutdown():
                return "SHUTDOWN", None
            self.publish_projection(projection)
            rospy.sleep(STEP_SECONDS)
        self.publish_projection(display_projection(target))
        rospy.sleep(POST_TRAJECTORY_WAIT_S)
        return "TRAJECTORY_COMPLETE", planned.plan

    def metrics(self, target):
        measured = self.latest()
        positions = dict(zip(measured.names, measured.positions_rad))
        velocities = dict(zip(measured.names, measured.velocities_rad_s))
        projected = display_projection(target)
        return {
            "max_position_error_rad": max(
                abs(positions[name] - value)
                for name, value in zip(FEEDBACK_JOINT_NAMES, projected)
            ),
            "max_abs_velocity_rad_s": max(
                abs(velocities[name]) for name in FEEDBACK_JOINT_NAMES
            ),
        }

    def run(self):
        deadline = rospy.Time.now().to_sec() + 5.0
        while not rospy.is_shutdown() and self.latest() is None:
            if rospy.Time.now().to_sec() >= deadline:
                print(json.dumps({"status": "FAILED", "reason": "NO_JOINT_STATES"}))
                return 1
            rospy.sleep(0.02)

        if not self.wait_for_controller_connections():
            print(json.dumps({
                "status": "FAILED",
                "reason": "CONTROLLER_CONNECTION_TIMEOUT",
            }))
            return 1

        home_reason, home_plan = self.run_plan(HOME_INDEPENDENT_RAD)
        if home_plan is None:
            print(json.dumps({"status": "FAILED", "phase": "RESTORE_HOME", "reason": home_reason}))
            return 1
        home_metrics = self.metrics(HOME_INDEPENDENT_RAD)

        rotate_reason, rotate_plan = self.run_plan(BASE_ONLY_OBSERVATION_RAD)
        if rotate_plan is None:
            print(json.dumps({"status": "FAILED", "phase": "BASE_ONLY_ROTATE", "reason": rotate_reason}))
            return 1
        rotate_metrics = self.metrics(BASE_ONLY_OBSERVATION_RAD)
        self._arm_extended.publish(Bool(data=False))

        print(json.dumps({
            "sim_only": True,
            "no_load": True,
            "status": "TRAJECTORY_COMPLETE",
            "reason": "HOME_RESTORED_THEN_BASE_ONLY_ROTATED",
            "home_independent_deg": [90.0, 90.0, 3.8],
            "observation_independent_deg": [48.0, 90.0, 3.8],
            "home_duration_s": home_plan.duration_s,
            "rotation_duration_s": rotate_plan.duration_s,
            "home_metrics": home_metrics,
            "rotation_metrics": rotate_metrics,
        }, sort_keys=True), flush=True)
        return 0


def main():
    rospy.init_node("swiftpro_sim_base_only_observation", anonymous=True)
    raise SystemExit(BaseOnlyObservationRunner().run())


if __name__ == "__main__":
    main()
