#!/usr/bin/env python3
"""Deterministic SIM_ONLY move_base action scenarios; never publishes Twist."""
from __future__ import annotations

import argparse
import json
import math
import signal
import sys
import time
from pathlib import Path
from typing import Any

import actionlib
import rospy
import rosgraph
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Path as NavPath
from std_msgs.msg import Bool, String
from tf.transformations import quaternion_from_euler

TERMINAL = {GoalStatus.PREEMPTED: "PREEMPTED", GoalStatus.SUCCEEDED: "SUCCEEDED", GoalStatus.ABORTED: "ABORTED", GoalStatus.REJECTED: "REJECTED", GoalStatus.RECALLED: "RECALLED", GoalStatus.LOST: "LOST"}


def six_fields(message: Twist) -> dict[str, float]:
    return {"linear_x": message.linear.x, "linear_y": message.linear.y, "linear_z": message.linear.z, "angular_x": message.angular.x, "angular_y": message.angular.y, "angular_z": message.angular.z}


def finite_zero(values: dict[str, float] | None) -> bool:
    return bool(values) and all(math.isfinite(value) and value == 0.0 for value in values.values())


class ScenarioRunner:
    def __init__(self, config: dict[str, Any], scenario: str, profile: str) -> None:
        self.config, self.scenario, self.profile = config, scenario, profile
        self.map: OccupancyGrid | None = None
        self.global_costmap: OccupancyGrid | None = None
        self.plan: NavPath | None = None
        self.final: dict[str, float] | None = None
        self.nav_samples = 0
        self.nav_peak = 0.0
        self.final_samples = 0
        self.final_peak = 0.0
        self.safety_state = ""
        self.active = False
        self.client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self.estop = rospy.Publisher("/raicom/safety/soft_estop", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/map", OccupancyGrid, self._map, queue_size=1)
        rospy.Subscriber("/move_base/global_costmap/costmap", OccupancyGrid, self._global_costmap, queue_size=1)
        rospy.Subscriber("/move_base/NavfnROS/plan", NavPath, self._plan, queue_size=10)
        rospy.Subscriber("/raicom/cmd_vel/nav", Twist, self._nav, queue_size=100)
        rospy.Subscriber("/cmd_vel", Twist, self._final, queue_size=100)
        rospy.Subscriber("/raicom/safety/state", String, self._safety, queue_size=10)

    def _map(self, message: OccupancyGrid) -> None: self.map = message
    def _global_costmap(self, message: OccupancyGrid) -> None: self.global_costmap = message
    def _plan(self, message: NavPath) -> None: self.plan = message
    def _nav(self, message: Twist) -> None:
        self.nav_samples += 1
        self.nav_peak = max(self.nav_peak, abs(message.linear.x), abs(message.linear.y), abs(message.angular.z))
    def _final(self, message: Twist) -> None:
        self.final = six_fields(message)
        self.final_samples += 1
        self.final_peak = max(self.final_peak, abs(message.linear.x), abs(message.linear.y), abs(message.angular.z))
    def _safety(self, message: Any) -> None: self.safety_state = message.data

    def publishers(self) -> list[str]:
        try:
            publishers, _, _ = rosgraph.Master(rospy.get_name()).getSystemState()
            return next((nodes for topic, nodes in publishers if topic == "/cmd_vel"), [])
        except Exception:
            return []

    def cell_value(self, x: float, y: float) -> int | None:
        if self.map is None:
            return None
        info = self.map.info
        col, row = int((x - info.origin.position.x) / info.resolution), int((y - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height):
            return None
        return self.map.data[row * info.width + col]

    def cell_free(self, x: float, y: float) -> bool:
        return self.cell_value(x, y) == 0

    @staticmethod
    def grid_cell_value(grid: OccupancyGrid | None, x: float, y: float) -> int | None:
        if grid is None:
            return None
        info = grid.info
        col, row = int((x - info.origin.position.x) / info.resolution), int((y - info.origin.position.y) / info.resolution)
        if not (0 <= col < info.width and 0 <= row < info.height):
            return None
        return grid.data[row * info.width + col]

    def first_invalid_plan_cell(self) -> dict[str, Any] | None:
        if self.map is None or self.plan is None or not self.plan.poses:
            return {"reason": "plan_or_map_missing"}
        first = self.plan.poses[0].pose.position
        for pose in self.plan.poses:
            x, y = pose.pose.position.x, pose.pose.position.y
            value = self.cell_value(x, y)
            if value == 0:
                continue
            prefix_distance = math.hypot(x - first.x, y - first.y)
            layered_cost = self.grid_cell_value(self.global_costmap, x, y)
            initial_footprint_unknown = (value == -1 and prefix_distance <= 0.10 and
                                         layered_cost is not None and layered_cost < 253)
            if not initial_footprint_unknown:
                return {"x": x, "y": y, "occupancy": value, "layered_cost": layered_cost,
                        "prefix_distance_m": prefix_distance}
        return None

    def plan_known(self) -> bool:
        return self.first_invalid_plan_cell() is None

    def goal(self, waypoint: dict[str, Any]) -> MoveBaseGoal:
        message = MoveBaseGoal()
        message.target_pose.header.frame_id = waypoint["frame"]
        message.target_pose.header.stamp = rospy.Time.now()
        message.target_pose.pose.position.x, message.target_pose.pose.position.y = waypoint["x"], waypoint["y"]
        quaternion = quaternion_from_euler(0, 0, waypoint["yaw"])
        message.target_pose.pose.orientation.x, message.target_pose.pose.orientation.y, message.target_pose.pose.orientation.z, message.target_pose.pose.orientation.w = quaternion
        return message

    def cleanup(self) -> None:
        self.client.cancel_all_goals()
        self.estop.publish(Bool(data=True))

    def wait_zero(self, deadline: float) -> bool:
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if finite_zero(self.final): return True
            time.sleep(.05)
        return False

    def run_goal(self, waypoint_name: str, overall_deadline: float) -> dict[str, Any]:
        waypoint = self.config["waypoints"][waypoint_name]
        if not self.cell_free(waypoint["x"], waypoint["y"]):
            raise RuntimeError("GOAL_CELL_NOT_KNOWN_FREE:" + waypoint_name)
        self.plan = None
        self.active = False
        self.client.send_goal(self.goal(waypoint), active_cb=lambda: setattr(self, "active", True))
        active_deadline = min(overall_deadline, time.monotonic() + 10)
        while not self.active and time.monotonic() < active_deadline:
            if self.client.get_state() in TERMINAL: break
            time.sleep(.05)
        if not self.active:
            raise RuntimeError("GOAL_NEVER_ACTIVE:" + waypoint_name)
        if self.scenario in ("cancel_active", "estop_active"):
            if self.scenario == "estop_active": self.estop.publish(Bool(data=True))
            self.client.cancel_goal()
        goal_deadline = min(overall_deadline, time.monotonic() + self.config["goal_timeout_s"])
        while self.client.get_state() not in TERMINAL and time.monotonic() < goal_deadline:
            if self.plan is not None and self.plan.poses:
                invalid = self.first_invalid_plan_cell()
                if invalid is not None:
                    raise RuntimeError("UNKNOWN_OR_NONFREE_PLAN_CELL:" + json.dumps(invalid, sort_keys=True))
            time.sleep(.05)
        state = self.client.get_state()
        terminal = TERMINAL.get(state, "TIMEOUT")
        expected = "SUCCEEDED" if self.scenario in ("nominal_out_and_return", "repeat", "thin_mission_navigation", "blue_workbench_1_approach", "blue_middle_workbench_approach", "blue_workbench_3_approach", "blue_center_turntable_north_approach", "manual_goal_regression", "blue_round_table_safe_staging", "blue_round_table_survey_1", "blue_round_table_survey_2", "blue_round_table_inward_clearance", "blue_round_table_boundary_lead_in", "sim_charge_pile_pre_dock") else "PREEMPTED"
        if terminal != expected:
            raise RuntimeError("UNEXPECTED_TERMINAL:" + terminal)
        if self.plan is None or not self.plan_known():
            raise RuntimeError("PLAN_MISSING_OR_UNKNOWN")
        return {"waypoint": waypoint_name, "terminal": terminal, "plan_cells_known_free": True}

    def run(self) -> dict[str, Any]:
        started, deadline = time.monotonic(), time.monotonic() + self.config["wall_timeout_s"]
        if rospy.get_param("/raicom/backend", "") != "SIM": raise RuntimeError("REAL_BACKEND_REJECTED")
        if rospy.get_param("/raicom_cmd_vel_gate/name", "") != self.profile: raise RuntimeError("EXECUTION_PROFILE_NOT_ACTIVE")
        if self.config.get("scope") != "SIM_ONLY": raise RuntimeError("SCENARIO_SCOPE_INVALID")
        if not self.client.wait_for_server(rospy.Duration(8)): raise RuntimeError("MOVE_BASE_UNAVAILABLE")
        map_deadline = time.monotonic() + 8
        while (self.map is None or self.global_costmap is None) and time.monotonic() < map_deadline: time.sleep(.05)
        if self.map is None or self.global_costmap is None: raise RuntimeError("MAP_OR_LAYERED_COSTMAP_UNAVAILABLE")
        if self.publishers() != ["/raicom_cmd_vel_gate"]: raise RuntimeError("CMD_VEL_OWNERSHIP_INVALID")
        goals = [self.run_goal(name, deadline) for name in self.config["scenarios"][self.scenario]]
        if not self.wait_zero(min(deadline, time.monotonic() + 3)): raise RuntimeError("FINAL_ZERO_INVALID")
        return {"simulation_only": True, "profile": self.profile, "scenario": self.scenario, "goals": goals, "wall_time_s": round(time.monotonic() - started, 3), "cmd_vel_publishers": self.publishers(), "last_final_cmd_vel": self.final, "nav_samples": self.nav_samples, "nav_peak": self.nav_peak, "final_samples": self.final_samples, "final_peak": self.final_peak, "safety_state": self.safety_state}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("scenario", choices=("nominal_out_and_return", "repeat", "cancel_active", "estop_active", "thin_mission_navigation", "blue_workbench_1_approach", "blue_middle_workbench_approach", "blue_workbench_3_approach", "blue_center_turntable_north_approach", "manual_goal_regression", "blue_round_table_safe_staging", "blue_round_table_survey_1", "blue_round_table_survey_2", "blue_round_table_inward_clearance", "blue_round_table_boundary_lead_in", "sim_charge_pile_pre_dock"))
    parser.add_argument("--profile", choices=("SIM_SAFE", "SIM_DEV_FAST"), required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(rospy.myargv(argv=sys.argv)[1:])
    config = json.loads(Path(args.config).read_text(encoding="utf-8"))
    rospy.init_node("raicom_sim_navigation_scenario", anonymous=True, disable_rosout=True)
    runner = ScenarioRunner(config, args.scenario, args.profile)
    def fail_closed(_signum: int, _frame: Any) -> None:
        runner.cleanup()
        runner.wait_zero(time.monotonic() + 3)
        raise SystemExit(2)
    signal.signal(signal.SIGTERM, fail_closed)
    signal.signal(signal.SIGINT, fail_closed)
    try:
        report = runner.run()
        report["ok"] = True
    except Exception as exc:
        runner.cleanup()
        runner.wait_zero(time.monotonic() + 3)
        report = {"ok": False, "simulation_only": True, "profile": args.profile, "scenario": args.scenario, "error": str(exc), "last_final_cmd_vel": runner.final, "nav_samples": runner.nav_samples, "nav_peak": runner.nav_peak, "final_samples": runner.final_samples, "final_peak": runner.final_peak, "safety_state": runner.safety_state}
    Path(args.output).write_text(json.dumps(report, sort_keys=True) + "\n", encoding="utf-8")
    print("sim_navigation_scenario=" + json.dumps(report, sort_keys=True))
    raise SystemExit(0 if report["ok"] else 2)

if __name__ == "__main__": main()
