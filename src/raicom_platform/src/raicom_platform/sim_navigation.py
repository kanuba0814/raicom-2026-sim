"""SIM_ONLY named-waypoint adapter for the safety-gated ``move_base`` path.

Coordinates are intentionally confined to this platform module and its explicit
SIM development profile. Mission Core supplies only waypoint names.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
from pathlib import Path
from typing import Any, Mapping, Optional, Protocol

from .api import OperationResult, RobotPose


@dataclass(frozen=True)
class NavigationGoal:
    """Resolved platform-internal goal; this is never a Mission Core API."""

    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class NavigationSnapshot:
    result: Optional[OperationResult]
    active_waypoint: Optional[str]
    detail: str


class MoveBaseClient(Protocol):
    """Small action-client seam, deliberately independent of ROS in tests."""

    def wait_for_server(self, timeout_s: float) -> bool: ...
    def send_goal(self, goal: NavigationGoal) -> None: ...
    def wait_for_result(self, timeout_s: float) -> bool: ...
    def get_state(self) -> int: ...
    def cancel_goal(self) -> None: ...
    def pose(self) -> Optional[RobotPose]: ...


class SimWaypointProfile:
    """Validated named development waypoints for one explicit SIM-only profile."""

    def __init__(self, data: Mapping[str, Any], source: str = "in-memory SIM profile") -> None:
        if data.get("scope") != "SIM_ONLY":
            raise ValueError("SIM waypoint profile must declare scope SIM_ONLY")
        waypoints = data.get("waypoints")
        if not isinstance(waypoints, Mapping):
            raise ValueError("SIM waypoint profile must contain a waypoints mapping")
        timeout = data.get("goal_timeout_s", 35.0)
        if isinstance(timeout, bool) or not isinstance(timeout, (int, float)) or timeout <= 0:
            raise ValueError("SIM waypoint profile goal_timeout_s must be positive")
        self._waypoints = waypoints
        self.goal_timeout_s = float(timeout)
        self.source = source

    @classmethod
    def from_json_file(cls, path: Path) -> "SimWaypointProfile":
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise ValueError(f"cannot load SIM waypoint profile {path}: {exc}") from exc
        if not isinstance(data, Mapping):
            raise ValueError("SIM waypoint profile must be a mapping")
        return cls(data, str(path))

    def resolve(self, name: str) -> NavigationGoal:
        if not isinstance(name, str) or not name.strip():
            raise ValueError("waypoint name must be a non-empty string")
        waypoint = self._waypoints.get(name)
        if not isinstance(waypoint, Mapping):
            raise ValueError(f"waypoint {name!r} is not configured in {self.source}")
        frame = waypoint.get("frame")
        if not isinstance(frame, str) or not frame.strip():
            raise ValueError(f"waypoint {name!r} has no valid frame")
        values = []
        for key in ("x", "y", "yaw"):
            value = waypoint.get(key)
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"waypoint {name!r}.{key} must be numeric")
            normalized = float(value)
            if not math.isfinite(normalized):
                raise ValueError(f"waypoint {name!r}.{key} must be finite")
            values.append(normalized)
        return NavigationGoal(frame, *values)


class RosMoveBaseClient:
    """Thin lazy ROS adapter; it publishes no Twist or ``/cmd_vel`` message."""

    def __init__(self) -> None:
        import actionlib
        from move_base_msgs.msg import MoveBaseAction

        self._client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)

    def wait_for_server(self, timeout_s: float) -> bool:
        import rospy
        return self._client.wait_for_server(rospy.Duration(timeout_s))

    def send_goal(self, goal: NavigationGoal) -> None:
        from move_base_msgs.msg import MoveBaseGoal
        import rospy
        from tf.transformations import quaternion_from_euler

        message = MoveBaseGoal()
        message.target_pose.header.frame_id = goal.frame_id
        message.target_pose.header.stamp = rospy.Time.now()
        message.target_pose.pose.position.x = goal.x_m
        message.target_pose.pose.position.y = goal.y_m
        quaternion = quaternion_from_euler(0.0, 0.0, goal.yaw_rad)
        message.target_pose.pose.orientation.x = quaternion[0]
        message.target_pose.pose.orientation.y = quaternion[1]
        message.target_pose.pose.orientation.z = quaternion[2]
        message.target_pose.pose.orientation.w = quaternion[3]
        self._client.send_goal(message)

    def wait_for_result(self, timeout_s: float) -> bool:
        import rospy
        return self._client.wait_for_result(rospy.Duration(timeout_s))

    def get_state(self) -> int:
        return self._client.get_state()

    def cancel_goal(self) -> None:
        self._client.cancel_goal()

    def pose(self) -> Optional[RobotPose]:
        return None


def map_action_state(state: int) -> OperationResult:
    """Map actionlib terminal states without importing ROS action messages."""
    if state == 3:  # GoalStatus.SUCCEEDED
        return OperationResult.SUCCEEDED
    if state in (2, 6, 7, 8):  # PREEMPTED/PREEMPTING/RECALLING/RECALLED
        return OperationResult.CANCELED
    return OperationResult.FAILED
