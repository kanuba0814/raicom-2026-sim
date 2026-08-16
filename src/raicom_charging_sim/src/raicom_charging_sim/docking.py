"""ROS-free, fail-closed geometry for SIM-only charge-dock alignment.

The dock frame follows REP-103: the pile's +X axis points out of its front face.
A robot approaching the face therefore has map yaw ``dock_yaw + pi``.  Marker
observations are already transformed into ``base_link`` by the detector.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Mapping


class DockPhase(str, Enum):
    ALIGN = "ALIGN"
    TOUCH_COMMIT = "TOUCH_COMMIT"
    CONTACT_DWELL = "CONTACT_DWELL"
    CURRENT_DWELL = "CURRENT_DWELL"
    COMPLETE = "COMPLETE"


@dataclass(frozen=True)
class DockProfile:
    station_id: str
    model_name: str
    pre_dock_frame: str
    pre_dock_x_m: float
    pre_dock_y_m: float
    pre_dock_yaw_rad: float
    coarse_xy_tolerance_m: float
    coarse_yaw_tolerance_rad: float
    marker_id: int
    marker_size_m: float
    marker_topic: str
    expected_frame: str
    marker_frame_correction_rpy_rad: tuple[float, float, float]
    marker_yaw_target_rad: float
    precontact_standoff_m: float
    contact_standoff_m: float
    yaw_tolerance_rad: float
    lateral_tolerance_m: float
    distance_tolerance_m: float
    max_observation_age_s: float
    stable_samples: int
    max_align_distance_m: float
    max_align_linear_speed_mps: float
    max_lateral_speed_mps: float
    max_angular_speed_radps: float
    max_touch_speed_mps: float
    max_touch_travel_m: float
    touch_timeout_s: float
    contact_dwell_s: float
    current_dwell_s: float
    total_timeout_s: float
    contact_topic: str
    required_robot_collision: str
    required_pad_collision: str

    @classmethod
    def from_mapping(cls, data: Mapping[str, Any]) -> "DockProfile":
        metadata = _mapping(data, "metadata")
        dock = _mapping(data, "dock")
        pre_dock = _mapping(dock, "pre_dock_pose")
        target = _mapping(data, "ar_target")
        motion = _mapping(data, "motion")
        evidence = _mapping(data, "evidence")
        orchestration = _mapping(data, "orchestration")
        if metadata.get("scope") != "SIM_ONLY" or metadata.get("assumption_status") != "SIM_WORKING_ASSUMPTION":
            raise ValueError("SIM_WORKING_ASSUMPTION_PROFILE_REQUIRED")
        if dock.get("front_axis") != "+X" or dock.get("approach_normal") != "+X":
            raise ValueError("DOCK_FRAME_CONVENTION_INVALID")
        if pre_dock.get("frame") != "map":
            raise ValueError("PRE_DOCK_FRAME_INVALID")
        values = dict(
            station_id=dock.get("station_id"), model_name=dock.get("model_name"),
            pre_dock_frame=pre_dock.get("frame"), pre_dock_x_m=pre_dock.get("x_m"),
            pre_dock_y_m=pre_dock.get("y_m"), pre_dock_yaw_rad=pre_dock.get("yaw_rad"),
            coarse_xy_tolerance_m=orchestration.get("coarse_xy_tolerance_m"),
            coarse_yaw_tolerance_rad=orchestration.get("coarse_yaw_tolerance_rad"),
            marker_id=target.get("marker_id"), marker_size_m=target.get("marker_size_m"),
            marker_topic=target.get("markers_topic"), expected_frame=target.get("expected_frame"),
            marker_frame_correction_rpy_rad=target.get("marker_frame_correction_rpy_rad"),
            marker_yaw_target_rad=target.get("marker_yaw_target_rad"),
            precontact_standoff_m=target.get("precontact_standoff_m"),
            contact_standoff_m=target.get("contact_standoff_m"),
            yaw_tolerance_rad=target.get("yaw_tolerance_rad"), lateral_tolerance_m=target.get("lateral_tolerance_m"),
            distance_tolerance_m=target.get("distance_tolerance_m"), max_observation_age_s=target.get("max_observation_age_s"),
            stable_samples=target.get("stable_samples"), max_align_distance_m=motion.get("max_align_distance_m"),
            max_align_linear_speed_mps=motion.get("max_align_linear_speed_mps"),
            max_lateral_speed_mps=motion.get("max_lateral_speed_mps"),
            max_angular_speed_radps=motion.get("max_angular_speed_radps"),
            max_touch_speed_mps=motion.get("max_touch_speed_mps"), max_touch_travel_m=motion.get("max_touch_travel_m"),
            touch_timeout_s=motion.get("touch_timeout_s"), contact_dwell_s=evidence.get("contact_dwell_s"),
            current_dwell_s=evidence.get("sim_current_dwell_s"), total_timeout_s=motion.get("total_timeout_s"),
            contact_topic=evidence.get("contact_topic"),
            required_robot_collision=evidence.get("required_robot_collision"),
            required_pad_collision=evidence.get("required_pad_collision"),
        )
        string_keys = {"station_id", "model_name", "pre_dock_frame", "marker_topic", "expected_frame",
                       "contact_topic", "required_robot_collision", "required_pad_collision"}
        for key in string_keys:
            if not isinstance(values[key], str) or not values[key].strip():
                raise ValueError(f"{key.upper()}_INVALID")
        correction = values["marker_frame_correction_rpy_rad"]
        if (not isinstance(correction, list) or len(correction) != 3 or
                not all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                        for value in correction)):
            raise ValueError("MARKER_FRAME_CORRECTION_INVALID")
        values["marker_frame_correction_rpy_rad"] = tuple(float(value) for value in correction)
        if values["marker_id"] != 0 or values["marker_size_m"] != 0.05:
            raise ValueError("AR_REFERENCE_INVALID")
        if isinstance(values["stable_samples"], bool) or not isinstance(values["stable_samples"], int) or values["stable_samples"] < 3:
            raise ValueError("STABLE_SAMPLES_INVALID")
        for key, value in values.items():
            if key in string_keys or key in {"marker_id", "stable_samples", "marker_frame_correction_rpy_rad"}:
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(f"{key.upper()}_INVALID")
            if key not in {"pre_dock_x_m", "pre_dock_y_m", "pre_dock_yaw_rad", "marker_yaw_target_rad"} and value <= 0.0:
                raise ValueError(f"{key.upper()}_INVALID")
        gap = values["precontact_standoff_m"] - values["contact_standoff_m"]
        if gap <= 0.0 or values["max_touch_travel_m"] < gap or values["max_touch_travel_m"] > 0.15:
            raise ValueError("TOUCH_TRAVEL_CONTRACT_INVALID")
        if values["max_touch_speed_mps"] > 0.02 or values["max_align_linear_speed_mps"] > 0.06:
            raise ValueError("SPEED_ENVELOPE_INVALID")
        return cls(**values)  # type: ignore[arg-type]


@dataclass(frozen=True)
class MarkerObservation:
    marker_id: int
    frame_id: str
    forward_m: float
    lateral_m: float
    yaw_rad: float
    count: int
    age_s: float


@dataclass(frozen=True)
class DockCommand:
    phase: DockPhase
    forward_mps: float = 0.0
    lateral_mps: float = 0.0
    angular_radps: float = 0.0


def wrap_angle(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))


def coarse_navigation_error(profile: DockProfile, x_m: float, y_m: float,
                            yaw_rad: float) -> tuple[float, float, bool]:
    """Check the coarse pre-dock pose before AR fine alignment begins."""
    values = (x_m, y_m, yaw_rad)
    if not all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
               for value in values):
        raise ValueError("COARSE_NAVIGATION_POSE_INVALID")
    xy_error = math.hypot(float(x_m) - profile.pre_dock_x_m,
                          float(y_m) - profile.pre_dock_y_m)
    yaw_error = abs(wrap_angle(float(yaw_rad) - profile.pre_dock_yaw_rad))
    accepted = (xy_error <= profile.coarse_xy_tolerance_m and
                yaw_error <= profile.coarse_yaw_tolerance_rad)
    return xy_error, yaw_error, accepted


def alignment_command(profile: DockProfile, marker: MarkerObservation) -> DockCommand:
    """Return a bounded three-axis pre-contact command or fail closed."""
    _validate_marker(profile, marker)
    distance_error = marker.forward_m - profile.precontact_standoff_m
    lateral_error = marker.lateral_m
    yaw_error = wrap_angle(marker.yaw_rad - profile.marker_yaw_target_rad)
    aligned = (abs(distance_error) <= profile.distance_tolerance_m and
               abs(lateral_error) <= profile.lateral_tolerance_m and
               abs(yaw_error) <= profile.yaw_tolerance_rad)
    if aligned:
        return DockCommand(DockPhase.TOUCH_COMMIT)
    return DockCommand(
        DockPhase.ALIGN,
        _clamp(0.8 * distance_error, profile.max_align_linear_speed_mps),
        _clamp(0.8 * lateral_error, profile.max_lateral_speed_mps),
        _clamp(1.5 * yaw_error, profile.max_angular_speed_radps),
    )


def touch_command(profile: DockProfile, marker: MarkerObservation, traveled_m: float, elapsed_s: float,
                  contact_confirmed: bool) -> DockCommand:
    """Commit only a positive straight motion; contact immediately returns zero."""
    _validate_marker(profile, marker)
    if contact_confirmed:
        return DockCommand(DockPhase.CONTACT_DWELL)
    if traveled_m < 0.0 or traveled_m > profile.max_touch_travel_m or elapsed_s < 0.0 or elapsed_s > profile.touch_timeout_s:
        raise ValueError("TOUCH_BOUND_EXCEEDED")
    yaw_error = wrap_angle(marker.yaw_rad - profile.marker_yaw_target_rad)
    if abs(marker.lateral_m) > profile.lateral_tolerance_m or abs(yaw_error) > profile.yaw_tolerance_rad:
        raise ValueError("TOUCH_ALIGNMENT_LOST")
    return DockCommand(DockPhase.TOUCH_COMMIT, profile.max_touch_speed_mps, 0.0, 0.0)


def valid_contact(profile: DockProfile, collision_a: str, collision_b: str) -> bool:
    """Accept only this robot charge port touching this configured dock pad."""
    expected = {profile.required_robot_collision, profile.required_pad_collision}
    return collision_a != collision_b and {collision_a, collision_b} == expected


def dwell_complete(*, now_s: float, started_s: float, duration_s: float,
                   deadline_s: float, contact_valid: bool) -> bool:
    """Fail closed on contact loss/deadline; otherwise report continuous dwell."""
    values = (now_s, started_s, duration_s, deadline_s)
    if not all(math.isfinite(value) for value in values) or duration_s <= 0.0:
        raise ValueError("DWELL_ARGUMENT_INVALID")
    if not contact_valid:
        raise ValueError("CONTACT_LOST_DURING_DWELL")
    if now_s >= deadline_s:
        raise TimeoutError("CHARGING_EVIDENCE_DEADLINE")
    if now_s < started_s:
        raise ValueError("DWELL_CLOCK_INVALID")
    return now_s - started_s >= duration_s


def _validate_marker(profile: DockProfile, marker: MarkerObservation) -> None:
    if marker.count != 1 or marker.marker_id != profile.marker_id:
        raise ValueError("MARKER_ID_OR_CARDINALITY_INVALID")
    if marker.frame_id.lstrip("/") != profile.expected_frame.lstrip("/"):
        raise ValueError("MARKER_FRAME_INVALID")
    values = (marker.forward_m, marker.lateral_m, marker.yaw_rad, marker.age_s)
    if not all(math.isfinite(value) for value in values) or not 0.0 <= marker.age_s <= profile.max_observation_age_s:
        raise ValueError("MARKER_STALE_OR_INVALID")
    if marker.forward_m <= 0.0 or marker.forward_m > profile.max_align_distance_m:
        raise ValueError("MARKER_DISTANCE_OUT_OF_BOUNDS")


def _mapping(data: Mapping[str, Any], key: str) -> Mapping[str, Any]:
    value = data.get(key)
    if not isinstance(value, Mapping):
        raise ValueError(f"{key.upper()}_MAPPING_REQUIRED")
    return value


def _clamp(value: float, limit: float) -> float:
    return max(-limit, min(limit, value))
