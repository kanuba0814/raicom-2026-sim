#!/usr/bin/env python3
"""Stationary invalid-Twist probes for the managed SIM_ONLY navigation graph.

This tool never publishes to ``/cmd_vel``.  It injects only an invalid command
through the gate's managed navigation input, then requires the gate to keep its
sole final output finite and all-zero before latching a soft E-stop.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

import rospy
import rosgraph
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

FIELDS = ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z")
CHOICES = ("invalid_twist_nan", "invalid_twist_envelope")
EXPECTED_REASONS = {
    "invalid_twist_nan": "non_finite_command",
    "invalid_twist_envelope": "velocity_envelope_exceeded",
}


class Evidence:
    def __init__(self) -> None:
        self.state = None
        self.twist = None
        rospy.Subscriber("/raicom/safety/state", String, self._state, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self._twist, queue_size=1)

    def _state(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            self.state = value if isinstance(value, dict) else None
        except (TypeError, ValueError):
            self.state = None

    def _twist(self, message: Twist) -> None:
        self.twist = {
            "linear_x": message.linear.x, "linear_y": message.linear.y, "linear_z": message.linear.z,
            "angular_x": message.angular.x, "angular_y": message.angular.y, "angular_z": message.angular.z,
        }

    def sole_gate_publisher(self) -> bool:
        try:
            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            names = next((nodes for topic, nodes in publishers if topic == "/cmd_vel"), [])
            return names == ["/raicom_cmd_vel_gate"]
        except Exception:
            return False

    def finite_six_field_zero(self) -> bool:
        return self.twist is not None and all(
            isinstance(self.twist[field], float)
            and math.isfinite(self.twist[field])
            and self.twist[field] == 0.0
            for field in FIELDS
        )

    def snapshot(self) -> dict:
        return {
            "gate_state": self.state,
            "final_twist": self.twist,
            "sole_final_publisher": self.sole_gate_publisher(),
            "finite_six_field_final_zero": self.finite_six_field_zero(),
        }


def wait_for(predicate, timeout_s: float = 4.0) -> bool:
    deadline = time.monotonic() + timeout_s
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return True
        rospy.sleep(0.05)
    return predicate()


def invalid_twist(choice: str) -> Twist:
    message = Twist()
    if choice == "invalid_twist_nan":
        message.linear.x = float("nan")
    else:
        message.linear.x = 1.0
    return message


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("fault", choices=CHOICES)
    args = parser.parse_args()
    rospy.init_node("raicom_navigation_fault_probe", anonymous=True, disable_signals=True)
    evidence = Evidence()
    if not wait_for(lambda: evidence.state is not None and evidence.twist is not None):
        print(json.dumps({"ok": False, "error": "managed navigation evidence unavailable", "evidence": evidence.snapshot()}))
        return 2

    publisher = rospy.Publisher("/raicom/cmd_vel/nav", Twist, queue_size=1)
    if not wait_for(lambda: publisher.get_num_connections() > 0, 2.0):
        print(json.dumps({"ok": False, "error": "managed navigation input unavailable", "evidence": evidence.snapshot()}))
        return 2
    publisher.publish(invalid_twist(args.fault))
    expected_reason = EXPECTED_REASONS[args.fault]
    rejected = wait_for(lambda: evidence.state is not None and evidence.state.get("reason") == expected_reason)
    zero = wait_for(evidence.finite_six_field_zero)
    sole = evidence.sole_gate_publisher()

    estop = rospy.Publisher("/raicom/safety/soft_estop", Bool, queue_size=1)
    wait_for(lambda: estop.get_num_connections() > 0, 2.0)
    estop.publish(Bool(data=True))
    latched = wait_for(lambda: evidence.state is not None and evidence.state.get("soft_estop_latched") is True)
    final_zero = wait_for(evidence.finite_six_field_zero)
    final_sole = evidence.sole_gate_publisher()
    ok = rejected and zero and sole and latched and final_zero and final_sole
    print(json.dumps({"ok": ok, "fault": args.fault, "motion_issued": False,
                      "expected_gate_reason": expected_reason, "rejection_observed": rejected,
                      "soft_estop_latched": latched, "evidence": evidence.snapshot()}))
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
