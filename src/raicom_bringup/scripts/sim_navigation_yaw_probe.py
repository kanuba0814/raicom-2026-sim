#!/usr/bin/env python3
"""Bounded SIM_ONLY move_base yaw probe using the live AMCL pose.

The probe intentionally contains no waypoint or field coordinate. It asks for a
small orientation change at the current AMCL position, then cancels the goal
and latches soft E-stop on every exit. move_base remains responsible for the
velocity source; its output must already be remapped through the RAICOM gate.
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass
from typing import Optional

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseWithCovarianceStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool


class NavigationProbeError(RuntimeError):
    """Raised when the bounded yaw-only validation cannot run safely."""


@dataclass(frozen=True)
class ProbeParameters:
    yaw_delta_rad: float
    maximum_abs_yaw_delta_rad: float
    server_timeout_s: float
    result_timeout_s: float

    @classmethod
    def from_ros(cls) -> "ProbeParameters":
        values = cls(
            yaw_delta_rad=float(rospy.get_param("~yaw_delta_rad")),
            maximum_abs_yaw_delta_rad=float(rospy.get_param("~maximum_abs_yaw_delta_rad")),
            server_timeout_s=float(rospy.get_param("~server_timeout_s")),
            result_timeout_s=float(rospy.get_param("~result_timeout_s")),
        )
        values.validate()
        return values

    def validate(self) -> None:
        values = (
            self.yaw_delta_rad,
            self.maximum_abs_yaw_delta_rad,
            self.server_timeout_s,
            self.result_timeout_s,
        )
        if not all(math.isfinite(value) for value in values):
            raise NavigationProbeError("probe parameters must be finite")
        if self.maximum_abs_yaw_delta_rad <= 0.0:
            raise NavigationProbeError("maximum_abs_yaw_delta_rad must be positive")
        if abs(self.yaw_delta_rad) <= 0.0 or abs(self.yaw_delta_rad) > self.maximum_abs_yaw_delta_rad:
            raise NavigationProbeError("yaw_delta_rad exceeds the configured SIM_ONLY bound")
        if self.server_timeout_s <= 0.0 or self.result_timeout_s <= 0.0:
            raise NavigationProbeError("probe timeouts must be positive")


class NavigationYawProbe:
    def __init__(self, parameters: ProbeParameters) -> None:
        self._parameters = parameters
        self._latest_pose: Optional[PoseWithCovarianceStamped] = None
        self._soft_estop = rospy.Publisher(
            "/raicom/safety/soft_estop", Bool, queue_size=1, latch=True
        )
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._pose_callback, queue_size=1)
        self._client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

    def run(self) -> None:
        goal_sent = False
        goal_succeeded = False
        try:
            if not self._client.wait_for_server(rospy.Duration(self._parameters.server_timeout_s)):
                raise NavigationProbeError("move_base action server did not become available")
            pose = self._wait_for_pose()
            if pose.header.frame_id != "map":
                raise NavigationProbeError("AMCL pose is not expressed in the expected map frame")

            goal = self._goal_from_pose(pose)
            self._client.send_goal(goal)
            goal_sent = True
            completed = self._client.wait_for_result(rospy.Duration(self._parameters.result_timeout_s))
            state = self._client.get_state()
            if not completed or state != GoalStatus.SUCCEEDED:
                raise NavigationProbeError(f"yaw-only move_base goal did not succeed (state={state})")
            goal_succeeded = True
            print("sim_navigation_yaw_probe=" + json.dumps({
                "simulation_only": True,
                "goal_position_source": "live_amcl_pose",
                "yaw_delta_rad": self._parameters.yaw_delta_rad,
                "action_state": state,
                "soft_estop_latched": True,
            }, sort_keys=True))
        finally:
            if goal_sent and not goal_succeeded:
                self._client.cancel_goal()
            self._soft_estop.publish(Bool(data=True))

    def _wait_for_pose(self) -> PoseWithCovarianceStamped:
        deadline = rospy.get_time() + self._parameters.server_timeout_s
        rate = rospy.Rate(20.0)
        while not rospy.is_shutdown() and rospy.get_time() < deadline:
            if self._latest_pose is not None:
                return self._latest_pose
            rate.sleep()
        raise NavigationProbeError("fresh AMCL pose was not received")

    def _goal_from_pose(self, pose: PoseWithCovarianceStamped) -> MoveBaseGoal:
        orientation = pose.pose.pose.orientation
        current_yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        target_yaw = current_yaw + self._parameters.yaw_delta_rad
        goal = MoveBaseGoal()
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = pose.pose.pose.position.x
        goal.target_pose.pose.position.y = pose.pose.pose.position.y
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.z = math.sin(target_yaw / 2.0)
        goal.target_pose.pose.orientation.w = math.cos(target_yaw / 2.0)
        return goal

    def _pose_callback(self, pose: PoseWithCovarianceStamped) -> None:
        self._latest_pose = pose


def main() -> None:
    rospy.init_node("raicom_sim_navigation_yaw_probe", anonymous=False, disable_rosout=True)
    try:
        NavigationYawProbe(ProbeParameters.from_ros()).run()
    except NavigationProbeError as exc:
        print("sim_navigation_yaw_probe_error=" + str(exc))
        raise SystemExit(2)


if __name__ == "__main__":
    main()
