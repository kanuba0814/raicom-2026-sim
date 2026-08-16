"""Configuration-only field geometry for semantic navigation.

This module deliberately has no ROS dependency and never derives coordinates
from Gazebo assets. A profile may describe a canonical waypoint and a side
transform only after both are status-labelled. Unresolved data is rejected at
resolution time instead of receiving guessed defaults.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum
from typing import Any, Mapping

from .config import LoadedProfile


class NavigationProfileError(ValueError):
    """Raised when a requested navigation value lacks an approved profile entry."""


class Side(str, Enum):
    RED = "RED"
    BLUE = "BLUE"


@dataclass(frozen=True)
class FieldPose:
    """A configuration-derived pose in a declared canonical field frame."""

    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class MirrorAcrossVerticalAxis:
    """Reflection across the vertical line ``x = axis_x_m`` in a 2D frame."""

    axis_x_m: float

    def apply(self, pose: FieldPose) -> FieldPose:
        return FieldPose(
            frame_id=pose.frame_id,
            x_m=2.0 * self.axis_x_m - pose.x_m,
            y_m=pose.y_m,
            yaw_rad=_normalize_yaw(math.pi - pose.yaw_rad),
        )


@dataclass(frozen=True)
class NavigationFieldProfile:
    """Lazy resolver for canonical waypoints and red/blue side transforms.

    ``WORKING_ASSUMPTION`` entries are usable only when the caller explicitly
    opts in. This permits future simulation-only profiles without allowing an
    accidental assumption to become a real-field default.
    """

    data: Mapping[str, Any]
    source: str
    allow_working_assumptions: bool = False

    @classmethod
    def from_loaded_profile(
        cls,
        profile: LoadedProfile,
        *,
        allow_working_assumptions: bool = False,
    ) -> "NavigationFieldProfile":
        return cls(
            data=profile.data,
            source=str(profile.path),
            allow_working_assumptions=allow_working_assumptions,
        )

    @classmethod
    def from_mapping(
        cls,
        data: Mapping[str, Any],
        *,
        source: str = "in-memory profile",
        allow_working_assumptions: bool = False,
    ) -> "NavigationFieldProfile":
        return cls(
            data=data,
            source=source,
            allow_working_assumptions=allow_working_assumptions,
        )

    def resolve_waypoint(self, name: str, side: Side) -> FieldPose:
        """Resolve a semantic waypoint, rejecting incomplete configuration."""
        if not isinstance(name, str) or not name.strip():
            raise NavigationProfileError("waypoint name must be a non-empty string")
        if not isinstance(side, Side):
            raise NavigationProfileError("side must be a Side enum value")

        frame_id = self._canonical_frame_id()
        waypoints = self._mapping("waypoints")
        entry = waypoints.get(name)
        if not isinstance(entry, Mapping):
            raise NavigationProfileError(
                f"waypoint {name!r} is not configured in {self.source}"
            )
        self._require_usable(entry, f"waypoint {name!r}")
        pose_data = entry.get("pose")
        if not isinstance(pose_data, Mapping):
            raise NavigationProfileError(f"waypoint {name!r} has no pose mapping")
        canonical_pose = FieldPose(
            frame_id=frame_id,
            x_m=self._number(pose_data, "x_m", f"waypoint {name!r}"),
            y_m=self._number(pose_data, "y_m", f"waypoint {name!r}"),
            yaw_rad=self._number(pose_data, "yaw_rad", f"waypoint {name!r}"),
        )
        return self._side_transform(side).apply(canonical_pose)

    def _canonical_frame_id(self) -> str:
        entry = self.data.get("canonical_frame")
        if not isinstance(entry, Mapping):
            raise NavigationProfileError(
                f"canonical_frame is not configured in {self.source}"
            )
        self._require_usable(entry, "canonical_frame")
        frame_id = entry.get("frame_id")
        if not isinstance(frame_id, str) or not frame_id.strip():
            raise NavigationProfileError("canonical_frame.frame_id must be a non-empty string")
        return frame_id

    def _side_transform(self, side: Side) -> "IdentityTransform | MirrorAcrossVerticalAxis":
        transforms = self._mapping("side_transforms")
        entry = transforms.get(side.value)
        if entry is None:
            entry = transforms.get(side.value.lower())
        if not isinstance(entry, Mapping):
            raise NavigationProfileError(
                f"side transform for {side.value} is not configured in {self.source}"
            )
        self._require_usable(entry, f"side transform for {side.value}")
        kind = entry.get("kind")
        if kind == "identity":
            return IdentityTransform()
        if kind == "mirror_across_vertical_axis":
            return MirrorAcrossVerticalAxis(
                axis_x_m=self._number(entry, "axis_x_m", f"side transform for {side.value}")
            )
        raise NavigationProfileError(
            f"side transform for {side.value} has unsupported kind {kind!r}"
        )

    def _mapping(self, key: str) -> Mapping[str, Any]:
        value = self.data.get(key, {})
        if not isinstance(value, Mapping):
            raise NavigationProfileError(f"{key} must be a mapping in {self.source}")
        return value

    def _require_usable(self, entry: Mapping[str, Any], label: str) -> None:
        status = entry.get("status")
        if status == "CONFIRMED":
            return
        if status == "WORKING_ASSUMPTION" and self.allow_working_assumptions:
            return
        if status == "WORKING_ASSUMPTION":
            raise NavigationProfileError(
                f"{label} is a WORKING_ASSUMPTION; explicit simulation opt-in is required"
            )
        raise NavigationProfileError(
            f"{label} is unresolved or lacks a usable status in {self.source}"
        )

    @staticmethod
    def _number(entry: Mapping[str, Any], key: str, label: str) -> float:
        value = entry.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise NavigationProfileError(f"{label}.{key} must be numeric")
        normalized = float(value)
        if not math.isfinite(normalized):
            raise NavigationProfileError(f"{label}.{key} must be finite")
        return normalized


@dataclass(frozen=True)
class IdentityTransform:
    """Identity side transform retained as an explicit configuration choice."""

    def apply(self, pose: FieldPose) -> FieldPose:
        return pose


def _normalize_yaw(yaw_rad: float) -> float:
    """Normalize an angle to the half-open interval [-pi, pi)."""
    return (yaw_rad + math.pi) % (2.0 * math.pi) - math.pi
