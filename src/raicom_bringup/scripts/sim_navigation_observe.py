#!/usr/bin/env python3
"""Passive SIM_ONLY telemetry for one externally-issued move_base goal.

The observer never sends a goal or a Twist. It begins sampling when the next
action goal is received. Failure cleanup is deliberately limited to
cancelling that action ID and latching the existing soft E-stop.
"""
from __future__ import annotations

import argparse
import json
import math
import signal
import statistics
import sys
import time
from typing import Any, Optional

import rospy
import rosgraph
import tf2_ros
from actionlib_msgs.msg import GoalID, GoalStatusArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool, String

ACTIVE = 1
TERMINAL = {2: "PREEMPTED", 3: "SUCCEEDED", 4: "ABORTED", 5: "REJECTED", 8: "RECALLED", 9: "LOST"}


def yaw(quaternion: Any) -> float:
    return math.atan2(2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y), 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z))


def pose_data(pose: Any) -> dict[str, float]:
    return {"x": pose.position.x, "y": pose.position.y, "z": pose.position.z, "yaw": yaw(pose.orientation)}


def twist_data(message: Twist) -> dict[str, float]:
    return {"linear_x": message.linear.x, "linear_y": message.linear.y, "linear_z": message.linear.z, "angular_x": message.angular.x, "angular_y": message.angular.y, "angular_z": message.angular.z}


def all_zero(values: dict[str, float]) -> bool:
    return all(math.isfinite(value) and value == 0.0 for value in values.values())


def sleep_until_deadline(deadline: float, maximum_s: float = 0.05) -> None:
    """Sleep by wall clock only, bounded by a monotonic deadline."""
    remaining = deadline - time.monotonic()
    if remaining > 0.0:
        time.sleep(min(maximum_s, remaining))


class NavigationObserver:
    def __init__(self, timeout_s: float) -> None:
        self.timeout_s = timeout_s
        self.gazebo_model_name = str(rospy.get_param("~gazebo_model_name", "oryxbot"))
        self.goal: Optional[dict[str, Any]] = None
        self.goal_id = ""
        self.goal_started: Optional[float] = None
        self.goal_started_monotonic: Optional[float] = None
        self.shutdown_requested = False
        self.cleanup_started = False
        self.active_status_observed = False
        self.terminal: Optional[str] = None
        self.latest: dict[str, Any] = {}
        self.start_poses: dict[str, Any] = {}
        self.map: Optional[OccupancyGrid] = None
        self.nav_times: list[float] = []
        self.nav_values: list[dict[str, float]] = []
        self.final_values: list[dict[str, Any]] = []
        self.final_pulses: list[dict[str, Any]] = []
        self.y_values: list[float] = []
        self.z_values: list[float] = []
        self.plan_metrics = {"global": {"message_count": 0, "max_poses": 0}, "local": {"message_count": 0, "max_poses": 0}}
        self.unknown_samples = self.known_samples = 0
        self.last_tf: Optional[dict[str, float]] = None
        self.tf_increments: list[dict[str, float]] = []
        self.gate_reason, self.gate_soft_estop = "UNKNOWN", False
        self.last_nav_time: Optional[float] = None
        self.cleanup: dict[str, Any] = {"required": False, "cancel_published": False, "soft_estop_published": False, "soft_estop_latched": False, "final_zero_observed": False, "wait_s": 0.0}
        # Failure-only cleanup publishers; no goal or Twist publisher exists.
        self.cancel = rospy.Publisher("/move_base/cancel", GoalID, queue_size=1)
        self.soft_estop = rospy.Publisher("/raicom/safety/soft_estop", Bool, queue_size=1, latch=True)
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self.goal_callback, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback, queue_size=10)
        rospy.Subscriber("/raicom/cmd_vel/nav", Twist, self.nav_callback, queue_size=100)
        rospy.Subscriber("/cmd_vel", Twist, self.final_callback, queue_size=100)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, lambda m: self.pose_callback("amcl", m.pose.pose), queue_size=10)
        rospy.Subscriber("/odom", Odometry, lambda m: self.pose_callback("odom", m.pose.pose), queue_size=10)
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.gazebo_callback, queue_size=10)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
        rospy.Subscriber("/move_base/NavfnROS/plan", Path, lambda m: self.plan_callback("global", m), queue_size=10)
        rospy.Subscriber("/move_base/DWAPlannerROS/local_plan", Path, lambda m: self.plan_callback("local", m), queue_size=10)
        rospy.Subscriber("/raicom/safety/state", String, self.gate_callback, queue_size=10)

    def goal_callback(self, message: MoveBaseActionGoal) -> None:
        if self.goal is None:
            target = message.goal.target_pose
            self.goal = {"frame": target.header.frame_id, **pose_data(target.pose)}
            self.goal_id = message.goal_id.id
            # Begin collection at selected-goal receipt: very short goals can
            # publish terminal status before an observer sees ACTIVE.
            self.goal_started = rospy.get_time()
            self.goal_started_monotonic = time.monotonic()
            self.start_poses = dict(self.latest)
            self.last_tf = None

    def status_callback(self, message: GoalStatusArray) -> None:
        if not self.goal_id:
            return
        status = next((entry for entry in message.status_list if entry.goal_id.id == self.goal_id), None)
        if status is None:
            return
        if status.status == ACTIVE:
            self.active_status_observed = True
        if status.status in TERMINAL:
            self.terminal = TERMINAL[status.status]

    def nav_callback(self, message: Twist) -> None:
        if self.goal is None or self.terminal is not None:
            return
        now, value = rospy.get_time(), twist_data(message)
        self.last_nav_time = now
        self.nav_times.append(now)
        self.nav_values.append(value)
        self.y_values.append(value["linear_y"])
        self.z_values.append(value["angular_z"])

    def final_callback(self, message: Twist) -> None:
        if self.goal is None:
            return
        now, value = rospy.get_time(), twist_data(message)
        self.final_values.append({"time": now, "values": value})
        nav_fresh = self.last_nav_time is not None and now - self.last_nav_time <= 0.35
        nav_nonzero = bool(self.nav_values) and not all_zero(self.nav_values[-1])
        if self.goal is not None and self.terminal is None and all_zero(value) and nav_fresh and nav_nonzero:
            self.final_pulses.append({"time": now, "nav_age_s": now - self.last_nav_time, "gate_reason": self.gate_reason})

    def pose_callback(self, name: str, pose: Any) -> None:
        self.latest[name] = pose_data(pose)
        if self.goal is not None and self.terminal is None and name == "amcl":
            self.check_unknown(pose.position.x, pose.position.y)

    def gazebo_callback(self, message: ModelStates) -> None:
        if self.gazebo_model_name in message.name:
            self.latest["gazebo"] = pose_data(message.pose[message.name.index(self.gazebo_model_name)])

    def map_callback(self, message: OccupancyGrid) -> None: self.map = message

    def plan_callback(self, name: str, message: Path) -> None:
        if self.goal is not None and self.terminal is None:
            metric = self.plan_metrics[name]
            metric["message_count"] += 1
            metric["max_poses"] = max(metric["max_poses"], len(message.poses))

    def gate_callback(self, message: String) -> None:
        try:
            state = json.loads(message.data)
            self.gate_reason = str(state.get("reason", "UNKNOWN"))
            self.gate_soft_estop = bool(state.get("soft_estop_latched", False))
        except (TypeError, ValueError):
            self.gate_reason = "UNPARSEABLE"

    def check_unknown(self, x: float, y: float) -> None:
        if self.map is None: return
        info = self.map.info
        col, row = int((x - info.origin.position.x) / info.resolution), int((y - info.origin.position.y) / info.resolution)
        if 0 <= col < info.width and 0 <= row < info.height:
            if self.map.data[row * info.width + col] < 0: self.unknown_samples += 1
            else: self.known_samples += 1

    def sample_tf(self) -> None:
        if self.goal is None or self.terminal is not None: return
        try:
            transform = self.tf_buffer.lookup_transform("map", "odom", rospy.Time(0), rospy.Duration(0.0)).transform
            current = {"x": transform.translation.x, "y": transform.translation.y, "z": transform.translation.z, "yaw": yaw(transform.rotation)}
            if self.last_tf:
                delta = {key: current[key] - self.last_tf[key] for key in ("x", "y", "z")}
                delta["yaw"] = math.atan2(math.sin(current["yaw"] - self.last_tf["yaw"]), math.cos(current["yaw"] - self.last_tf["yaw"]))
                self.tf_increments.append(delta)
            self.last_tf = current
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException): pass

    def cleanup_failure(self) -> None:
        """Idempotently cancel only the selected active goal and prove a safe exit."""
        if self.cleanup_started:
            return
        self.cleanup_started = True
        self.cleanup.update({"required": True, "cancel_published": bool(self.goal_id), "soft_estop_published": True})
        if self.goal_id:
            self.cancel.publish(GoalID(id=self.goal_id))
        self.soft_estop.publish(Bool(data=True))
        started = time.monotonic()
        deadline = started + 3.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            final_zero = bool(self.final_values) and all_zero(self.final_values[-1]["values"])
            if self.gate_soft_estop and final_zero:
                break
            sleep_until_deadline(deadline)
        self.cleanup.update({"soft_estop_latched": self.gate_soft_estop, "final_zero_observed": bool(self.final_values) and all_zero(self.final_values[-1]["values"]), "wait_s": time.monotonic() - started})

    def request_shutdown(self, _signum: int, _frame: Any) -> None:
        self.shutdown_requested = True
        self.cleanup_failure()

    def cmd_vel_publishers(self) -> list[str]:
        try:
            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            return next((nodes for topic, nodes in publishers if topic == "/cmd_vel"), [])
        except Exception: return []

    def wait_for_post_terminal_zero(self) -> None:
        """Give the gate one wall-clock second to publish its final zero."""
        deadline = time.monotonic() + 1.0
        while not rospy.is_shutdown() and time.monotonic() < deadline:
            if self.final_values and all_zero(self.final_values[-1]["values"]):
                return
            sleep_until_deadline(deadline)

    def run(self) -> dict[str, Any]:
        deadline = time.monotonic() + self.timeout_s
        while not rospy.is_shutdown() and not self.shutdown_requested and time.monotonic() < deadline and self.terminal is None:
            self.sample_tf()
            sleep_until_deadline(deadline)
        failed = self.terminal != "SUCCEEDED"
        if failed:
            self.cleanup_failure()
        elif self.terminal == "SUCCEEDED":
            self.wait_for_post_terminal_zero()
        outcome = "shutdown" if self.shutdown_requested else ("timeout" if self.terminal is None else self.terminal)
        return self.report(outcome)

    def report(self, outcome: str) -> dict[str, Any]:
        def maximum(values: list[dict[str, float]], key: str) -> float: return max((abs(v[key]) for v in values), default=0.0)
        gaps = [b - a for a, b in zip(self.nav_times, self.nav_times[1:])]
        def percentile(p: float) -> Optional[float]:
            if not gaps: return None
            ordered = sorted(gaps); return ordered[min(len(ordered) - 1, math.ceil(p * len(ordered)) - 1)]
        def variation(values: list[float]) -> dict[str, Any]: return {"sign_reversals": sum(1 for a, b in zip(values, values[1:]) if a * b < 0), "total_variation": sum(abs(b - a) for a, b in zip(values, values[1:]))}
        def displacement(name: str) -> Optional[float]:
            if name not in self.start_poses or name not in self.latest: return None
            return math.hypot(self.latest[name]["x"] - self.start_poses[name]["x"], self.latest[name]["y"] - self.start_poses[name]["y"])
        final = self.final_values[-1] if self.final_values else None
        publishers = self.cmd_vel_publishers()
        return {"simulation_only": True, "observer_sent_goal": False, "observer_published_twist": False, "outcome": outcome, "terminal_action_status": self.terminal or "TIMEOUT", "goal": self.goal, "active_status_observed": self.active_status_observed, "start_poses": self.start_poses, "final_poses": self.latest, "gazebo_model": {"configured": self.gazebo_model_name, "observed": "gazebo" in self.latest}, "duration_s": None if self.goal_started_monotonic is None else time.monotonic() - self.goal_started_monotonic, "nav_max_abs": {"x": maximum(self.nav_values, "linear_x"), "y": maximum(self.nav_values, "linear_y"), "angular_z": maximum(self.nav_values, "angular_z")}, "final_max_abs": {"x": maximum([item["values"] for item in self.final_values], "linear_x"), "y": maximum([item["values"] for item in self.final_values], "linear_y"), "angular_z": maximum([item["values"] for item in self.final_values], "angular_z")}, "max_planar_speed": max((math.hypot(v["linear_x"], v["linear_y"]) for v in self.nav_values), default=0.0), "nav_interarrival_s": {"median": statistics.median(gaps) if gaps else None, "p95": percentile(.95), "p99": percentile(.99), "max": max(gaps) if gaps else None, "over_0_30": sum(g > .30 for g in gaps), "over_0_35": sum(g > .35 for g in gaps)}, "final_zero_pulses_while_nav_nonzero": self.final_pulses, "last_final_cmd_vel": None if final is None else {"timestamp": final["time"], "values": final["values"], "finite": all(math.isfinite(v) for v in final["values"].values()), "all_zero": all_zero(final["values"])}, "nav_y": variation(self.y_values), "nav_angular_z": variation(self.z_values), "displacement_m": {name: displacement(name) for name in ("amcl", "odom", "gazebo")}, "map_to_odom_increment_max": {key: max((abs(v[key]) for v in self.tf_increments), default=0.0) for key in ("x", "y", "z", "yaw")}, "plans": self.plan_metrics, "unknown_cell_traversal": {"feasible": self.map is not None, "unknown_samples": self.unknown_samples, "known_samples": self.known_samples}, "cmd_vel_publisher_check": {"publishers": publishers, "sole_raicom_gate": publishers == ["/raicom_cmd_vel_gate"]}, "cleanup": self.cleanup}


def main() -> None:
    parser = argparse.ArgumentParser(description="Passively observe one external move_base goal.")
    parser.add_argument("--timeout-s", type=float, required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    if not math.isfinite(args.timeout_s) or not 10 <= args.timeout_s <= 300: raise ValueError("--timeout-s must be finite and within 10..300")
    rospy.init_node("raicom_sim_navigation_observe", anonymous=True, disable_rosout=True)
    observer = NavigationObserver(args.timeout_s)
    signal.signal(signal.SIGTERM, observer.request_shutdown)
    signal.signal(signal.SIGINT, observer.request_shutdown)
    telemetry = observer.run()
    with open(args.output, "w", encoding="utf-8") as stream: json.dump(telemetry, stream, indent=2, sort_keys=True); stream.write("\n")
    print("sim_navigation_observe=" + json.dumps(telemetry, sort_keys=True))
    raise SystemExit(0 if telemetry["outcome"] == "SUCCEEDED" else 2)

if __name__ == "__main__": main()
