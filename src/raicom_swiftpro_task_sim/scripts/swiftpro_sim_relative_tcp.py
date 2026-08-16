#!/usr/bin/env python3
"""Execute one bounded SIM-only Cartesian displacement from joint feedback."""
from __future__ import annotations

import argparse
import json
import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64

from raicom_swiftpro_task_sim.kinematics import (
    admission_reason, display_projection, forward_position, inverse_kinematics,
)
from raicom_swiftpro_task_sim.motion_adapter import (
    CONTROLLER_JOINT_NAMES, JointObservation,
    MotionStatus, STEP_SECONDS, SwiftproSimMotionAdapter,
)

MAX_DELTA_M = 0.10
TERMINAL_ERROR_RAD = 0.02
TERMINAL_STABLE_SAMPLES = 3
TERMINAL_TIMEOUT_S = 6.0
DRIVEN_FEEDBACK_NAMES = ("Joint1", "Joint2", "Joint6")
DRIVEN_DISPLAY_INDICES = (0, 1, 5)


def observation(message):
    return JointObservation(
        tuple(message.name), tuple(message.position), tuple(message.velocity),
        message.header.stamp.to_sec(),
    )


def independent_from_feedback(joint):
    positions = dict(zip(joint.names, joint.positions_rad))
    if not all(name in positions for name in ("Joint1", "Joint2", "Joint6")):
        return None
    return (
        positions["Joint1"] + math.pi / 2.0,
        math.pi / 2.0 - positions["Joint2"],
        positions["Joint6"] + math.radians(3.8),
    )


class RelativeTcpMove:
    def __init__(self, delta):
        self.delta = delta
        self.publishers = {
            name: rospy.Publisher(
                "/swiftpro/%s_position_controller/command" % name,
                Float64, queue_size=1,
            )
            for name in CONTROLLER_JOINT_NAMES
        }
        self.arm_extended = rospy.Publisher(
            "/raicom/safety/arm_extended", Bool, queue_size=1, latch=True
        )

    def publish(self, projection):
        for name, value in zip(CONTROLLER_JOINT_NAMES, projection):
            self.publishers[name].publish(Float64(data=value))

    def wait_connections(self):
        deadline = rospy.Time.now().to_sec() + 3.0
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            if all(item.get_num_connections() >= 1 for item in self.publishers.values()):
                return True
            rospy.sleep(0.02)
        return False

    def run(self):
        if rospy.get_param("/raicom/backend", "") != "SIM":
            print(json.dumps({"status": "REJECTED", "reason": "REAL_BACKEND"}))
            return 2
        if not self.wait_connections():
            print(json.dumps({"status": "REJECTED", "reason": "CONTROLLER_CONNECTION_TIMEOUT"}))
            return 2
        # Sample only after publisher handshakes; motion planning has a strict
        # joint-observation freshness gate.
        joint_message = rospy.wait_for_message("/swiftpro/joint_states", JointState, timeout=5.0)
        joint = observation(joint_message)
        independent = independent_from_feedback(joint)
        current = None if independent is None else forward_position(independent)
        if current is None:
            print(json.dumps({"status": "REJECTED", "reason": "FEEDBACK_FK_FAILED"}))
            return 2
        target = tuple(current[index] + self.delta[index] for index in range(3))
        reason = admission_reason(target)
        if reason is not None:
            print(json.dumps({
                "status": "REJECTED", "reason": reason,
                "current_tcp_base_m": current,
                "delta_base_m": self.delta,
                "target_tcp_base_m": target,
            }, sort_keys=True))
            return 2
        target_independent = inverse_kinematics(target)
        if target_independent is None:
            print(json.dumps({"status": "REJECTED", "reason": "IK_FAILED"}))
            return 2
        planned = SwiftproSimMotionAdapter().plan(
            target_independent, joint, rospy.Time.now().to_sec()
        )
        if planned.status is not MotionStatus.READY or planned.plan is None:
            print(json.dumps({"status": "REJECTED", "reason": planned.reason}))
            return 2

        print(json.dumps({
            "event": "ADMITTED",
            "current_tcp_base_m": current,
            "delta_base_m": self.delta,
            "target_tcp_base_m": target,
            "oracle_used": False,
        }, sort_keys=True), flush=True)
        self.arm_extended.publish(Bool(data=True))
        for projection in planned.plan.commands:
            self.publish(projection)
            rospy.sleep(STEP_SECONDS)
        target_display = display_projection(target_independent)
        deadline = rospy.Time.now().to_sec() + TERMINAL_TIMEOUT_S
        stable = 0
        max_error = math.inf
        measured = joint
        while not rospy.is_shutdown() and rospy.Time.now().to_sec() < deadline:
            self.publish(target_display)
            rospy.sleep(0.02)
            measured = observation(rospy.wait_for_message(
                "/swiftpro/joint_states", JointState, timeout=1.0
            ))
            positions = dict(zip(measured.names, measured.positions_rad))
            max_error = max(
                abs(positions[name] - target_display[index])
                for name, index in zip(DRIVEN_FEEDBACK_NAMES, DRIVEN_DISPLAY_INDICES)
            )
            stable = stable + 1 if max_error <= TERMINAL_ERROR_RAD else 0
            if stable >= TERMINAL_STABLE_SAMPLES:
                break
        if stable < TERMINAL_STABLE_SAMPLES:
            print(json.dumps({
                "status": "FAILED", "reason": "TERMINAL_NOT_ACCEPTED",
                "max_error_rad": max_error,
            }, sort_keys=True))
            return 2
        feedback_independent = independent_from_feedback(measured)
        feedback = forward_position(feedback_independent)
        error_mm = [
            (feedback[index] - target[index]) * 1000.0 for index in range(3)
        ]
        print(json.dumps({
            "status": "HELD",
            "current_tcp_base_m": current,
            "target_tcp_base_m": target,
            "feedback_tcp_base_m": feedback,
            "feedback_error_mm": error_mm,
            "max_error_rad": max_error,
            "arm_extended_latched": True,
            "gripper_commanded": False,
            "oracle_used": False,
        }, sort_keys=True), flush=True)
        return 0


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--dx-m", type=float, default=0.0)
    parser.add_argument("--dy-m", type=float, default=0.0)
    parser.add_argument("--dz-m", type=float, default=0.0)
    args = parser.parse_args(rospy.myargv()[1:])
    delta = (args.dx_m, args.dy_m, args.dz_m)
    if (not all(math.isfinite(value) for value in delta)
            or not 1e-6 < math.sqrt(sum(value * value for value in delta)) <= MAX_DELTA_M):
        parser.error("finite nonzero displacement must not exceed 0.10 m")
    rospy.init_node("swiftpro_sim_relative_tcp", anonymous=True)
    raise SystemExit(RelativeTcpMove(delta).run())


if __name__ == "__main__":
    main()
