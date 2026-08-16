#!/usr/bin/env python3
"""Read-only SwiftPro idle-settling probe for the isolated Gazebo baseline."""

import json
import math
import sys
import threading
import time

import rospy
from control_msgs.msg import JointControllerState
from sensor_msgs.msg import JointState


JOINTS = [f"Joint{index}" for index in range(1, 10)] + ["virtual_end_joint"]
CONTROLLERS = [
    f"joint{index}_position_controller" for index in range(1, 10)
] + ["virtual_end_position_controller"]
class ArmSettledProbe:
    """Proves a continuous, fresh, passive settled window from arm telemetry."""

    def __init__(self) -> None:
        self.velocity_limit = float(rospy.get_param("~velocity_limit", 0.02))
        self.error_limit = float(rospy.get_param("~error_limit", 0.02))
        self.window_seconds = float(rospy.get_param("~window_seconds", 1.5))
        self.timeout_seconds = float(rospy.get_param("~timeout_seconds", 10.0))
        self.freshness_seconds = float(rospy.get_param("~freshness_seconds", 0.15))
        self.lock = threading.Lock()
        self.latest_velocity = {}
        self.latest_error = {}
        self.controller_receipts = {}
        self.window_start = None
        self.sample_count = 0
        self.max_velocity = 0.0
        self.max_error = 0.0
        self.max_velocity_by_joint = {joint: 0.0 for joint in JOINTS}
        self.max_error_by_controller = {controller: 0.0 for controller in CONTROLLERS}
        self.max_command_by_controller = {controller: 0.0 for controller in CONTROLLERS}
        self.invalid_messages = 0
        self.stale_samples = 0
        self.settled = False
        self.robot_description_check = self.check_robot_description()
        self.controller_pid_check = self.check_controller_pids()

        rospy.Subscriber("/swiftpro/joint_states", JointState, self.on_joint_state, queue_size=1)
        for controller in CONTROLLERS:
            rospy.Subscriber(
                f"/swiftpro/{controller}/state",
                JointControllerState,
                self.on_controller_state,
                callback_args=controller,
                queue_size=1,
            )

    @staticmethod
    def finite(values) -> bool:
        return all(math.isfinite(value) for value in values)

    def check_robot_description(self) -> dict:
        description = rospy.get_param("/robot_description", "")
        return {
            "available": isinstance(description, str) and bool(description),
            "expected_damping_count": 0,
            "observed_damping_count": description.count("<dynamics")
            if isinstance(description, str) else 0,
        }

    @staticmethod
    def check_controller_pids() -> dict:
        values = {}
        for controller in CONTROLLERS:
            parameter = f"/swiftpro/{controller}/pid/i"
            value = rospy.get_param(parameter, None)
            values[controller] = value if isinstance(value, (int, float)) else None
        return values

    def on_controller_state(self, message: JointControllerState, controller: str) -> None:
        values = [message.error, message.process_value, message.set_point, message.command]
        with self.lock:
            if not self.finite(values):
                self.invalid_messages += 1
                return
            error = abs(message.error)
            self.latest_error[controller] = error
            self.controller_receipts[controller] = time.monotonic()
            self.max_error_by_controller[controller] = max(self.max_error_by_controller[controller], error)
            self.max_command_by_controller[controller] = max(
                self.max_command_by_controller[controller], abs(message.command)
            )

    def on_joint_state(self, message: JointState) -> None:
        positions = dict(zip(message.name, message.position))
        velocities = dict(zip(message.name, message.velocity))
        if not all(joint in positions and joint in velocities for joint in JOINTS):
            return
        values = [positions[joint] for joint in JOINTS] + [velocities[joint] for joint in JOINTS]
        with self.lock:
            if not self.finite(values):
                self.invalid_messages += 1
                self.window_start = None
                return
            self.latest_velocity = {joint: abs(velocities[joint]) for joint in JOINTS}
            for joint, velocity in self.latest_velocity.items():
                self.max_velocity_by_joint[joint] = max(self.max_velocity_by_joint[joint], velocity)
            self.evaluate(time.monotonic())

    def evaluate(self, now: float) -> None:
        if not all(controller in self.latest_error for controller in CONTROLLERS):
            self.window_start = None
            return
        if any(now - self.controller_receipts[controller] > self.freshness_seconds for controller in CONTROLLERS):
            self.stale_samples += 1
            self.window_start = None
            return
        velocity = max(self.latest_velocity.values())
        error = max(self.latest_error.values())
        self.max_velocity = max(self.max_velocity, velocity)
        self.max_error = max(self.max_error, error)
        self.sample_count += 1
        if velocity > self.velocity_limit or error > self.error_limit:
            self.window_start = None
            return
        if self.window_start is None:
            self.window_start = now
        elif now - self.window_start >= self.window_seconds:
            self.settled = True

    def report(self) -> None:
        payload = {
            "read_only": True,
            "settled": self.settled,
            "velocity_limit_rad_s": self.velocity_limit,
            "error_limit_rad": self.error_limit,
            "required_window_seconds": self.window_seconds,
            "freshness_seconds": self.freshness_seconds,
            "observed_samples": self.sample_count,
            "stale_samples": self.stale_samples,
            "invalid_messages": self.invalid_messages,
            "max_abs_velocity_rad_s": self.max_velocity,
            "max_abs_error_rad": self.max_error,
            "max_abs_velocity_by_joint_rad_s": self.max_velocity_by_joint,
            "max_abs_error_by_controller_rad": self.max_error_by_controller,
            "max_abs_command_by_controller": self.max_command_by_controller,
            "joint_count": len(JOINTS),
            "controller_count": len(CONTROLLERS),
            "robot_description": self.robot_description_check,
            "controller_pid_i": self.controller_pid_check,
        }
        print(json.dumps(payload, sort_keys=True), flush=True)

    def run(self) -> int:
        deadline = time.monotonic() + self.timeout_seconds
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            with self.lock:
                if self.settled:
                    self.report()
                    return 0
            time.sleep(0.02)
        with self.lock:
            self.report()
        return 1


def main() -> int:
    rospy.init_node("swiftpro_arm_settled_probe", anonymous=True)
    return ArmSettledProbe().run()


if __name__ == "__main__":
    sys.exit(main())
