#!/usr/bin/env python3
"""Bounded SIM_ONLY iterative occupancy-grid frontier survey.

This coordinator only sends vetted map-frame goals to move_base.  It never
publishes Twist; move_base remains routed through the existing safety gate.
"""
from __future__ import annotations

import importlib.util
import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import actionlib
import rospy
import tf
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetPlan, GetPlanRequest
from std_msgs.msg import Bool, Empty, String

_FRONTIER_SPEC = importlib.util.spec_from_file_location(
    "raicom_frontier_analysis", Path(__file__).with_name("frontier_analysis.py")
)
if _FRONTIER_SPEC is None or _FRONTIER_SPEC.loader is None:
    raise RuntimeError("frontier analysis module is unavailable")
_FRONTIER_MODULE = importlib.util.module_from_spec(_FRONTIER_SPEC)
sys.modules[_FRONTIER_SPEC.name] = _FRONTIER_MODULE
_FRONTIER_SPEC.loader.exec_module(_FRONTIER_MODULE)
GridSpec = _FRONTIER_MODULE.GridSpec
reachable_frontier_goals = _FRONTIER_MODULE.reachable_frontier_goals


class SurveyError(RuntimeError):
    """Raised when a bounded SIM_ONLY survey cannot continue safely."""


@dataclass(frozen=True)
class SurveyParameters:
    max_goals: int
    goal_timeout_s: float
    overall_timeout_s: float
    gate_timeout_s: float
    inflation_radius_m: float
    edge_margin_m: float
    standoff_m: float

    @classmethod
    def from_ros(cls) -> "SurveyParameters":
        result = cls(*(rospy.get_param("~" + name) for name in (
            "max_goals", "goal_timeout_s", "overall_timeout_s", "gate_timeout_s",
            "inflation_radius_m", "edge_margin_m", "standoff_m",
        )))
        if result.max_goals <= 0 or any(not math.isfinite(float(value)) or float(value) <= 0.0
                                   for value in (result.goal_timeout_s, result.overall_timeout_s, result.gate_timeout_s)):
            raise SurveyError("goal count and timeouts must be finite and positive")
        if any(not math.isfinite(float(value)) or float(value) < 0.0
               for value in (result.inflation_radius_m, result.edge_margin_m, result.standoff_m)):
            raise SurveyError("frontier distance parameters must be finite and non-negative")
        return result


class FrontierSurvey:
    def __init__(self, parameters: SurveyParameters) -> None:
        self._parameters = parameters
        self._map: Optional[OccupancyGrid] = None
        self._map_revision = None
        self._gate_state: Optional[dict] = None
        self._goal_active = False
        self._maximum_requested_linear = 0.0
        self._maximum_requested_angular = 0.0
        self._last_goal_diagnostic: Optional[dict] = None
        self._visited_goal_xy = []
        self._rejected_goal_xy = []
        self._initial_odom_xy: Optional[tuple] = None
        self._previous_odom_yaw: Optional[float] = None
        self._accumulated_odom_yaw = 0.0
        self._maximum_odom_displacement = 0.0
        self._listener = tf.TransformListener()
        self._client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
        self._make_plan = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
        self._ack = rospy.Publisher("/raicom/safety/ack", Empty, queue_size=1)
        self._soft_estop = rospy.Publisher("/raicom/safety/soft_estop", Bool, queue_size=1, latch=True)
        rospy.Subscriber("/map", OccupancyGrid, self._map_callback, queue_size=1)
        rospy.Subscriber("/raicom/safety/state", String, self._state_callback, queue_size=1)
        rospy.Subscriber("/raicom/cmd_vel/nav", Twist, self._nav_command_callback, queue_size=1)
        rospy.Subscriber("/odom", Odometry, self._odom_callback, queue_size=1)

    def run(self) -> None:
        started = rospy.get_time()
        completed = 0
        rejected = 0
        try:
            startup_timeout_s = min(30.0, self._parameters.overall_timeout_s)
            wall_deadline = time.monotonic() + startup_timeout_s
            while not rospy.is_shutdown() and time.monotonic() < wall_deadline:
                if self._client.wait_for_server(rospy.Duration(0.2)):
                    break
                time.sleep(0.05)
            else:
                raise SurveyError("move_base action server did not become available")
            try:
                rospy.wait_for_service("/move_base/make_plan", timeout=startup_timeout_s)
            except rospy.ROSException as exc:
                raise SurveyError("move_base make_plan service unavailable: {}".format(exc))
            while completed < self._parameters.max_goals and not rospy.is_shutdown():
                if rospy.get_time() - started >= self._parameters.overall_timeout_s:
                    raise SurveyError("overall survey timeout reached")
                # Ack must be freshly published before every single goal, even if
                # the prior goal had already cleared a soft E-stop.
                self._ack.publish(Empty())
                self._wait_for_safe_gate(started)
                goal = self._wait_for_frontier_goal(require_goal=completed == 0)
                if goal is None:
                    break
                dispatched_map_revision = self._map_revision
                self._client.send_goal(goal)
                self._goal_active = True
                remaining_s = started + self._parameters.overall_timeout_s - rospy.get_time()
                goal_deadline = rospy.get_time() + min(self._parameters.goal_timeout_s, remaining_s)
                rate = rospy.Rate(20.0)
                timed_out = True
                stalled = False
                progress_reference = self._maximum_odom_displacement
                angular_progress_reference = self._accumulated_odom_yaw
                progress_deadline = rospy.get_time() + 8.0
                while not rospy.is_shutdown() and rospy.get_time() < goal_deadline:
                    if self._client.wait_for_result(rospy.Duration(0.05)):
                        timed_out = False
                        break
                    if not self._gate_is_safe():
                        self._cancel_and_wait_terminal()
                        raise SurveyError("safety gate became invalid during active frontier goal")
                    if (self._maximum_odom_displacement - progress_reference >= 0.05
                            or self._accumulated_odom_yaw - angular_progress_reference >= 0.10):
                        progress_reference = self._maximum_odom_displacement
                        angular_progress_reference = self._accumulated_odom_yaw
                        progress_deadline = rospy.get_time() + 8.0
                    elif rospy.get_time() >= progress_deadline:
                        stalled = True
                        break
                    rate.sleep()
                if timed_out or stalled:
                    self._cancel_and_wait_terminal()
                    assert self._last_goal_diagnostic is not None
                    self._rejected_goal_xy.append((
                        self._last_goal_diagnostic["frontier_x"],
                        self._last_goal_diagnostic["frontier_y"],
                    ))
                    rejected += 1
                    print("sim_frontier_goal_rejected=" + json.dumps({
                        "state": "STALLED" if stalled else "TIMEOUT", "rejected_goals": rejected,
                        "gate_state": self._gate_state,
                        "maximum_requested_linear": self._maximum_requested_linear,
                        "maximum_requested_angular": self._maximum_requested_angular,
                        "maximum_odom_displacement": self._maximum_odom_displacement,
                        "goal": self._last_goal_diagnostic,
                    }, sort_keys=True))
                    self._soft_estop.publish(Bool(data=True))
                    if rejected >= 4:
                        raise SurveyError("frontier candidate failure budget exhausted")
                    continue
                state = self._client.get_state()
                if state != GoalStatus.SUCCEEDED:
                    self._cancel_and_wait_terminal()
                    assert self._last_goal_diagnostic is not None
                    self._rejected_goal_xy.append((
                        self._last_goal_diagnostic["frontier_x"],
                        self._last_goal_diagnostic["frontier_y"],
                    ))
                    rejected += 1
                    print("sim_frontier_goal_rejected=" + json.dumps({
                        "state": state, "rejected_goals": rejected,
                        "gate_state": self._gate_state,
                        "maximum_requested_linear": self._maximum_requested_linear,
                        "maximum_requested_angular": self._maximum_requested_angular,
                        "maximum_odom_displacement": self._maximum_odom_displacement,
                        "goal": self._last_goal_diagnostic,
                    }, sort_keys=True))
                    self._soft_estop.publish(Bool(data=True))
                    if rejected >= 4:
                        raise SurveyError("frontier candidate failure budget exhausted")
                    continue
                assert self._last_goal_diagnostic is not None
                self._visited_goal_xy.append((
                    self._last_goal_diagnostic["goal_x"],
                    self._last_goal_diagnostic["goal_y"],
                ))
                completed += 1
                self._wait_for_new_map_revision(dispatched_map_revision, started)
            print("sim_frontier_survey=" + json.dumps({
                "simulation_only": True, "completed_goals": completed,
                "max_goals": self._parameters.max_goals, "soft_estop_latched": True,
            }, sort_keys=True))
        finally:
            self._cancel_and_wait_terminal()
            self._soft_estop.publish(Bool(data=True))

    def _wait_for_new_map_revision(self, previous_revision: object, started: float) -> None:
        deadline = min(started + self._parameters.overall_timeout_s,
                       rospy.get_time() + self._parameters.gate_timeout_s)
        rate = rospy.Rate(10.0)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if self._map_revision is not None and self._map_revision != previous_revision:
                return
            if not self._gate_is_safe():
                raise SurveyError("safety gate invalid while awaiting post-motion map revision")
            rate.sleep()
        raise SurveyError("no newer occupancy-grid revision after completed frontier motion")

    def _gate_is_safe(self) -> bool:
        state = self._gate_state
        return bool(state and not state.get("soft_estop_latched", True)
                    and not state.get("hard_estop_latched", True)
                    and not state.get("arm_extended", True)
                    and state.get("localization_valid", False)
                    and state.get("watchdog_healthy", False))

    def _wait_for_safe_gate(self, started: float) -> None:
        deadline = min(started + self._parameters.overall_timeout_s,
                       rospy.get_time() + self._parameters.gate_timeout_s)
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if self._gate_is_safe():
                return
            rate.sleep()
        raise SurveyError("safety gate did not report soft_estop=false, localization_valid=true, watchdog_healthy=true")

    def _wait_for_frontier_goal(self, require_goal: bool) -> Optional[MoveBaseGoal]:
        deadline = rospy.get_time() + self._parameters.gate_timeout_s
        rate = rospy.Rate(5.0)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            goal = self._next_goal()
            if goal is not None:
                return goal
            rate.sleep()
        if require_goal:
            raise SurveyError("no reachable known-free frontier goal after live-map settle timeout")
        return None

    def _next_goal(self) -> Optional[MoveBaseGoal]:
        if self._map is None:
            raise SurveyError("no live occupancy grid received")
        try:
            (translation, rotation) = self._listener.lookupTransform("map", "base_footprint", rospy.Time(0))
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as exc:
            raise SurveyError("current robot TF seed unavailable: {}".format(exc))
        info = self._map.info
        grid = GridSpec(info.width, info.height, info.resolution, info.origin.position.x,
                        info.origin.position.y, self._map.data)
        goals = reachable_frontier_goals(grid, translation[0], translation[1],
                                         inflation_radius_m=self._parameters.inflation_radius_m,
                                         edge_margin_m=self._parameters.edge_margin_m,
                                         standoff_m=self._parameters.standoff_m,
                                         seed_search_radius_m=0.25)
        if not goals:
            return None
        start = PoseStamped()
        start.header.frame_id = "map"
        start.header.stamp = rospy.Time.now()
        start.pose.position.x = translation[0]
        start.pose.position.y = translation[1]
        start.pose.orientation.x, start.pose.orientation.y = rotation[0], rotation[1]
        start.pose.orientation.z, start.pose.orientation.w = rotation[2], rotation[3]
        preferred_goals = sorted(goals, key=lambda item: (
            not (0.60 <= math.hypot(item.world_x - translation[0], item.world_y - translation[1]) <= 1.50),
            abs(math.hypot(item.world_x - translation[0], item.world_y - translation[1]) - 1.0),
        ))
        for selected in preferred_goals:
            if math.hypot(selected.world_x - translation[0], selected.world_y - translation[1]) < 0.20:
                continue
            if any(math.hypot(selected.world_x - x, selected.world_y - y) < 0.20
                   for x, y in self._visited_goal_xy):
                continue
            if any(math.hypot(selected.world_x - x, selected.world_y - y) < 0.30
                   for x, y in self._rejected_goal_xy):
                continue
            target = PoseStamped()
            target.header.frame_id = "map"
            target.header.stamp = start.header.stamp
            target.pose.position.x = selected.world_x
            target.pose.position.y = selected.world_y
            target.pose.orientation.z, target.pose.orientation.w = rotation[2], rotation[3]
            request = GetPlanRequest(start=start, goal=target, tolerance=0.10)
            try:
                plan = self._make_plan(request)
            except rospy.ServiceException:
                continue
            if not plan.plan.poses:
                continue
            endpoint = plan.plan.poses[-1].pose.position
            if math.hypot(endpoint.x - selected.world_x, endpoint.y - selected.world_y) > 0.05:
                continue
            # Keep each survey leg bounded. A long reachable frontier is
            # approached through an existing global-plan pose; the next leg is
            # recomputed from the newer live map rather than committing to a
            # long stale route.
            bounded_pose = plan.plan.poses[-1]
            for pose in plan.plan.poses:
                if math.hypot(pose.pose.position.x - translation[0],
                              pose.pose.position.y - translation[1]) >= 0.80:
                    bounded_pose = pose
                    break
            target.pose = bounded_pose.pose
            self._last_goal_diagnostic = {
                "start_x": translation[0], "start_y": translation[1],
                "goal_x": target.pose.position.x, "goal_y": target.pose.position.y,
                "frontier_x": selected.world_x, "frontier_y": selected.world_y,
                "plan_poses": len(plan.plan.poses), "candidate_count": len(goals),
            }
            print("sim_frontier_goal=" + json.dumps(self._last_goal_diagnostic, sort_keys=True))
            goal = MoveBaseGoal()
            goal.target_pose = target
            return goal
        return None

    def _cancel_and_wait_terminal(self) -> None:
        self._client.cancel_all_goals()
        if self._goal_active:
            self._client.wait_for_result(rospy.Duration(min(2.0, self._parameters.goal_timeout_s)))
            self._goal_active = False

    def _map_callback(self, message: OccupancyGrid) -> None:
        self._map = message
        self._map_revision = (
            message.header.seq,
            message.header.stamp.secs,
            message.header.stamp.nsecs,
            message.info.map_load_time.secs,
            message.info.map_load_time.nsecs,
        )

    def _odom_callback(self, message: Odometry) -> None:
        current = (message.pose.pose.position.x, message.pose.pose.position.y)
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        if self._previous_odom_yaw is not None:
            delta = math.atan2(math.sin(yaw - self._previous_odom_yaw),
                               math.cos(yaw - self._previous_odom_yaw))
            self._accumulated_odom_yaw += abs(delta)
        self._previous_odom_yaw = yaw
        if self._initial_odom_xy is None:
            self._initial_odom_xy = current
        self._maximum_odom_displacement = max(
            self._maximum_odom_displacement,
            math.hypot(current[0] - self._initial_odom_xy[0], current[1] - self._initial_odom_xy[1]),
        )

    def _nav_command_callback(self, message: Twist) -> None:
        self._maximum_requested_linear = max(self._maximum_requested_linear, abs(message.linear.x))
        self._maximum_requested_angular = max(self._maximum_requested_angular, abs(message.angular.z))

    def _state_callback(self, message: String) -> None:
        try:
            parsed = json.loads(message.data)
            if isinstance(parsed, dict):
                self._gate_state = parsed
        except ValueError:
            self._gate_state = None


def main() -> None:
    rospy.init_node("raicom_sim_frontier_survey", anonymous=False, disable_rosout=True)
    try:
        FrontierSurvey(SurveyParameters.from_ros()).run()
    except SurveyError as exc:
        print("sim_frontier_survey_error=" + str(exc))
        raise SystemExit(2)


if __name__ == "__main__":
    main()
