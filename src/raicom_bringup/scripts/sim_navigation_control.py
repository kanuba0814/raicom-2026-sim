#!/usr/bin/env python3
"""Bounded SIM_ONLY control evidence for the managed saved-map navigation graph."""
from __future__ import annotations

import argparse
import json
import sys
import time

import rospy
import rosgraph
from actionlib_msgs.msg import GoalID, GoalStatusArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, Empty, String

ACTIVE = {0, 1, 6, 7}
TERMINAL = {2, 3, 4, 5, 8, 9}
STATUS_NAMES = {
    0: "PENDING", 1: "ACTIVE", 2: "PREEMPTED", 3: "SUCCEEDED",
    4: "ABORTED", 5: "REJECTED", 6: "PREEMPTING", 7: "RECALLING",
    8: "RECALLED", 9: "LOST",
}
FIELDS = ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z")


class ControlEvidence:
    def __init__(self) -> None:
        self.statuses = []
        self.state = None
        self.twist = None
        self.twist_at = 0.0
        rospy.Subscriber("/move_base/status", GoalStatusArray, self._status, queue_size=1)
        rospy.Subscriber("/raicom/safety/state", String, self._state, queue_size=1)
        rospy.Subscriber("/cmd_vel", Twist, self._twist, queue_size=1)

    def _status(self, message: GoalStatusArray) -> None:
        self.statuses = list(message.status_list)

    def _state(self, message: String) -> None:
        try:
            parsed = json.loads(message.data)
            if isinstance(parsed, dict):
                self.state = parsed
        except (TypeError, ValueError):
            self.state = None

    def _twist(self, message: Twist) -> None:
        self.twist = {
            "linear_x": message.linear.x, "linear_y": message.linear.y, "linear_z": message.linear.z,
            "angular_x": message.angular.x, "angular_y": message.angular.y, "angular_z": message.angular.z,
        }
        self.twist_at = time.monotonic()

    def unique_goal(self):
        active = [item for item in self.statuses if item.status in ACTIVE]
        if len(active) != 1:
            return None, "no active move_base goal" if not active else "multiple active move_base goals"
        return active[0], ""

    def publisher_evidence(self):
        try:
            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            names = next((nodes for topic, nodes in publishers if topic == "/cmd_vel"), [])
        except Exception as exc:
            return {"publishers": [], "sole_raicom_gate": False, "error": str(exc)}
        return {"publishers": sorted(names), "sole_raicom_gate": names == ["/raicom_cmd_vel_gate"]}

    def snapshot(self):
        values = self.twist or {field: None for field in FIELDS}
        all_zero = all(values.get(field) == 0.0 for field in FIELDS)
        return {
            "goal": self.goal_snapshot(), "gate_state": self.state,
            "cmd_vel_publisher": self.publisher_evidence(),
            "current_twist": {"values": values, "all_six_zero": all_zero,
                              "received": self.twist is not None},
        }

    def goal_snapshot(self):
        goal, reason = self.unique_goal()
        if goal is None:
            return {"unique": False, "state": "UNKNOWN", "reason": reason}
        return {"unique": True, "id": goal.goal_id.id, "state": STATUS_NAMES.get(goal.status, str(goal.status))}


def wait_for(predicate, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while not rospy.is_shutdown() and time.monotonic() < deadline:
        if predicate():
            return True
        rospy.sleep(0.05)
    return predicate()


def require_graph(evidence):
    return wait_for(lambda: evidence.state is not None and evidence.twist is not None)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("operation", choices=("status", "cancel", "estop", "ack"))
    args = parser.parse_args()
    rospy.init_node("raicom_navigation_control", anonymous=True, disable_signals=True)
    evidence = ControlEvidence()
    if not require_graph(evidence):
        print(json.dumps({"ok": False, "error": "managed navigation graph evidence unavailable", "evidence": evidence.snapshot()}))
        return 2

    before = evidence.snapshot()
    if args.operation == "status":
        ok = before["cmd_vel_publisher"]["sole_raicom_gate"]
        print(json.dumps({"ok": ok, "evidence": before}))
        return 0 if ok else 2

    goal, error = evidence.unique_goal()
    if args.operation == "cancel" and goal is None:
        print(json.dumps({"ok": False, "error": error, "before": before, "evidence": evidence.snapshot()}))
        return 2
    if args.operation in ("cancel", "estop") and goal is not None:
        cancel = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        wait_for(lambda: cancel.get_num_connections() > 0, 2.0)
        cancel.publish(GoalID(id=goal.goal_id.id))
        terminal = wait_for(lambda: any(item.goal_id.id == goal.goal_id.id and item.status in TERMINAL for item in evidence.statuses))
        if not terminal and args.operation == "cancel":
            print(json.dumps({"ok": False, "error": "goal did not reach a terminal state after cancel", "before": before, "evidence": evidence.snapshot()}))
            return 2
        cancel_failed = not terminal
    else:
        cancel_failed = False
    if args.operation == "estop" and error == "multiple active move_base goals":
        print(json.dumps({"ok": False, "error": error, "before": before, "evidence": evidence.snapshot()}))
        return 2

    if args.operation == "estop":
        publisher = rospy.Publisher("/raicom/safety/soft_estop", Bool, queue_size=1)
        wait_for(lambda: publisher.get_num_connections() > 0, 2.0)
        publisher.publish(Bool(data=True))
        if not wait_for(lambda: evidence.state is not None and evidence.state.get("soft_estop_latched") is True):
            print(json.dumps({"ok": False, "error": "soft E-stop did not latch", "before": before, "evidence": evidence.snapshot()}))
            return 2
    elif args.operation == "ack":
        publisher = rospy.Publisher("/raicom/safety/ack", Empty, queue_size=1)
        wait_for(lambda: publisher.get_num_connections() > 0, 2.0)
        publisher.publish(Empty())
        valid = lambda: evidence.state is not None and not evidence.state.get("soft_estop_latched", True) and not evidence.state.get("hard_estop_latched", True) and evidence.state.get("localization_valid") is True and evidence.state.get("watchdog_healthy") is True
        if not wait_for(valid):
            print(json.dumps({"ok": False, "error": "ACK did not clear soft E-stop with healthy fail-closed safety state", "before": before, "evidence": evidence.snapshot()}))
            return 2

    zero = wait_for(lambda: evidence.twist is not None and all(evidence.twist[field] == 0.0 for field in FIELDS))
    after = evidence.snapshot()
    ok = not cancel_failed and zero and after["cmd_vel_publisher"]["sole_raicom_gate"]
    error = "goal did not reach a terminal state after cancel" if cancel_failed else ("six-field final /cmd_vel zero verification failed" if not zero else "")
    print(json.dumps({"ok": ok, "before": before, "evidence": after, "error": error}))
    return 0 if ok else 2


if __name__ == "__main__":
    sys.exit(main())
