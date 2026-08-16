"""Pure-Python, SIM-only center-material-island mission safety core.

This module intentionally has no ROS publishers.  A ROS adapter may translate its
``MissionCommand`` only after the existing team-boundary, e-stop, and command-gate
contracts admit it.  It never supplies a fallback command path.
"""
from dataclasses import dataclass, field
from enum import Enum
import json
import math
from pathlib import Path
from typing import Optional, Sequence, Tuple


class TargetFeedState(str, Enum):
    DETECTED = "DETECTED"
    UNAVAILABLE = "UNAVAILABLE"
    UNKNOWN = "UNKNOWN"


class MissionState(str, Enum):
    NAVIGATE_STAGING = "NAVIGATE_STAGING"
    AR_ALIGN = "AR_ALIGN"
    BASE_HOLD = "BASE_HOLD"
    ARM_OBSERVE = "ARM_OBSERVE"
    SELECT_TARGET = "SELECT_TARGET"
    PICK = "PICK"
    STORE_REAR = "STORE_REAR"
    ARM_SAFE_RETRACT = "ARM_SAFE_RETRACT"
    REFEREE_HOLD = "REFEREE_HOLD"
    STOP = "STOP"


@dataclass(frozen=True)
class IslandConfig:
    ar_marker_id: int = 4
    ar_measurement_frame: str = "base_footprint"
    ar_distance_semantics: str = "marker_to_robot_envelope_clearance"
    ar_max_age_s: float = 0.25
    ar_stable_frames: int = 3
    robot_front_extent_m: float = 0.24
    robot_rear_extent_m: float = 0.24
    robot_side_extent_m: float = 0.20
    ar_hard_min_distance_m: float = 0.20
    ar_target_distance_m: float = 0.23
    ar_distance_tolerance_m: float = 0.01
    ar_bearing_tolerance_rad: float = math.radians(1.0)
    ar_max_approach_mps: float = 0.02
    ar_max_align_rps: float = 0.06
    target_frame: str = "Base"
    target_max_age_s: float = 0.20
    target_stable_frames: int = 3
    target_min_stable_s: float = 0.75
    target_max_speed_mps: float = 0.01
    target_max_drift_m: float = 0.008
    rotation_timing_status: str = "UNRESOLVED"
    nominal_rotation_period_s: float = 30.0
    nominal_rotation_degrees: float = 90.0
    known_rotation_guard_s: float = 3.0
    pick_required_window_s: float = 8.0
    arm_observation_rotation: dict = field(default_factory=lambda: {
        "semantic": "camera_observation_pose", "rotation_degrees": 90.0,
        "joint_mapping": "UNRESOLVED_CONFIGURE_BEFORE_SIM_RUN"})
    staging_scenario: str = "blue_center_turntable_north_approach"
    pick_pose: dict = field(default_factory=lambda: {"frame": "Base", "source": "selected_target_3d"})
    rear_storage_pose: dict = field(default_factory=lambda: {"frame": "Base", "xyz_m": [0.106, 0.191, 0.120]})
    safe_arm_pose: dict = field(default_factory=lambda: {"name": "safe_folded"})

    @classmethod
    def from_json(cls, path: str) -> "IslandConfig":
        values = json.loads(Path(path).read_text())
        return cls(**values)

    def __post_init__(self):
        numeric = (
            self.ar_max_age_s, self.robot_front_extent_m,
            self.robot_rear_extent_m, self.robot_side_extent_m,
            self.ar_hard_min_distance_m,
            self.ar_target_distance_m, self.ar_distance_tolerance_m,
            self.ar_bearing_tolerance_rad, self.ar_max_approach_mps,
            self.ar_max_align_rps, self.target_max_age_s,
            self.target_min_stable_s, self.target_max_speed_mps,
            self.target_max_drift_m, self.nominal_rotation_period_s,
            self.nominal_rotation_degrees, self.known_rotation_guard_s,
            self.pick_required_window_s,
        )
        if not all(math.isfinite(value) and value >= 0.0 for value in numeric):
            raise ValueError("mission numeric configuration must be finite and non-negative")
        if self.ar_marker_id != 4 or self.ar_hard_min_distance_m < 0.20:
            raise ValueError("AR ID4 and hard minimum distance >= 0.20m are mandatory")
        if (self.ar_measurement_frame != "base_footprint"
                or self.ar_distance_semantics != "marker_to_robot_envelope_clearance"):
            raise ValueError("AR distance must be robot-envelope clearance in base_footprint")
        if self.ar_target_distance_m - self.ar_distance_tolerance_m < self.ar_hard_min_distance_m:
            raise ValueError("complete tolerance must remain above the AR hard minimum")
        if self.ar_stable_frames < 1 or self.target_stable_frames < 1:
            raise ValueError("stable-frame requirements must be positive")
        if self.rotation_timing_status not in ("UNRESOLVED", "WORKING_ASSUMPTION"):
            raise ValueError("rotation timing must remain UNRESOLVED or WORKING_ASSUMPTION")
        if not isinstance(self.staging_scenario, str) or not self.staging_scenario.strip():
            raise ValueError("staging scenario must be a configured name")
        if (self.arm_observation_rotation.get("semantic") != "camera_observation_pose"
                or self.arm_observation_rotation.get("rotation_degrees") != 90.0):
            raise ValueError("arm observation must retain the configured 90-degree semantic")
        mapping = self.arm_observation_rotation.get("joint_mapping")
        if not isinstance(mapping, str) or not mapping.strip():
            raise ValueError("arm observation mapping must be explicit")
        if (mapping != "UNRESOLVED_CONFIGURE_BEFORE_SIM_RUN"
                and self.arm_observation_rotation.get("verification") not in (
                    "SIM_TESTED", "FIELD_VALIDATED")):
            raise ValueError("configured arm rotation requires verification provenance")
        for name, pose in (("pick_pose", self.pick_pose),
                           ("rear_storage_pose", self.rear_storage_pose)):
            if pose.get("frame") != self.target_frame:
                raise ValueError("%s must use the configured target frame" % name)


def point_to_robot_envelope_clearance(
        marker_x_m: float, marker_y_m: float, config: IslandConfig) -> float:
    """Shortest planar distance from an AR point to the robot envelope.

    The adapter must first transform the marker observation at its own timestamp
    into ``base_footprint``.  A camera-to-marker norm is not an acceptable
    substitute for this clearance.
    """
    if not all(math.isfinite(value) for value in (marker_x_m, marker_y_m)):
        return math.nan
    longitudinal_extent = (config.robot_front_extent_m
                           if marker_x_m >= 0.0 else config.robot_rear_extent_m)
    dx = max(abs(marker_x_m) - longitudinal_extent, 0.0)
    dy = max(abs(marker_y_m) - config.robot_side_extent_m, 0.0)
    return math.hypot(dx, dy)


@dataclass(frozen=True)
class ArObservation:
    marker_id: int
    frame_id: str
    marker_x_m: float
    marker_y_m: float
    age_s: float
    stable_frames: int

    def valid(self, config: IslandConfig) -> bool:
        values = (self.marker_x_m, self.marker_y_m, self.age_s)
        return (self.marker_id == config.ar_marker_id
                and self.frame_id == config.ar_measurement_frame
                and all(math.isfinite(value) for value in values)
                and 0.0 <= self.age_s <= config.ar_max_age_s
                and self.stable_frames >= config.ar_stable_frames)


@dataclass(frozen=True)
class TargetCandidate:
    identifier: str
    frame_id: str  # Must equal config.target_frame after a timestamped TF transform.
    xyz_arm_base_m: Tuple[float, float, float]
    age_s: float
    stable_frames: int
    stable_duration_s: float
    speed_mps: float
    ik_admitted: bool

    def grasp_distance_m(self) -> float:
        return math.sqrt(sum(value * value for value in self.xyz_arm_base_m))

    def eligible(self, config: IslandConfig) -> bool:
        return (self.frame_id == config.target_frame and all(math.isfinite(v) for v in self.xyz_arm_base_m)
                and 0.0 <= self.age_s <= config.target_max_age_s
                and self.stable_frames >= config.target_stable_frames
                and math.isfinite(self.stable_duration_s)
                and self.stable_duration_s >= config.target_min_stable_s
                and abs(self.speed_mps) <= config.target_max_speed_mps and self.ik_admitted)


@dataclass(frozen=True)
class SafetyInputs:
    sim_backend: bool
    estop_clear: bool
    team_boundary_permits_motion: bool
    base_gate_owns_publisher: bool
    base_hold_six_dof_zero: bool
    arm_adapter_available: bool
    arm_sequence_preflight_ok: bool
    arm_swept_volume_within_team_boundary: bool

    def failure_reason(self) -> Optional[str]:
        if not self.sim_backend:
            return "BACKEND_NOT_SIM"
        if not self.estop_clear:
            return "ESTOP_ACTIVE"
        if not self.team_boundary_permits_motion:
            return "TEAM_BOUNDARY_REJECTED"
        if not self.base_gate_owns_publisher:
            return "BASE_PUBLISHER_OWNERSHIP_REJECTED"
        return None

    def arm_failure_reason(self) -> Optional[str]:
        if not self.arm_adapter_available:
            return "ARM_ADAPTER_UNAVAILABLE"
        if not self.arm_sequence_preflight_ok:
            return "ARM_SEQUENCE_PREFLIGHT_REJECTED"
        if not self.arm_swept_volume_within_team_boundary:
            return "ARM_TEAM_BOUNDARY_REJECTED"
        return None


@dataclass(frozen=True)
class MissionInputs:
    safety: SafetyInputs
    navigation_complete: bool = False
    ar: Optional[ArObservation] = None
    target_feed_state: TargetFeedState = TargetFeedState.UNAVAILABLE
    targets: Sequence[TargetCandidate] = ()
    rotation_window_in_s: Optional[float] = None
    arm_observation_complete: bool = False
    pick_complete: bool = False
    vacuum_grasp_confirmed: bool = False
    store_complete: bool = False
    vacuum_release_confirmed: bool = False
    safe_retract_complete: bool = False


@dataclass(frozen=True)
class MissionCommand:
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    vz_mps: float = 0.0
    wx_rps: float = 0.0
    wy_rps: float = 0.0
    wz_rps: float = 0.0
    arm_action: str = "NONE"
    vacuum_on: bool = False
    reason: str = "HOLD"


class CenterIslandMission:
    def __init__(self, config: IslandConfig):
        self.config = config
        self.state = MissionState.NAVIGATE_STAGING
        self.failure_reason = ""
        self.arm_extended = False
        self.selected_target: Optional[TargetCandidate] = None

    def _stop(self, reason: str) -> MissionCommand:
        self.state = MissionState.STOP
        self.failure_reason = reason
        # Do not clear the interlock on intent alone.  The adapter must keep
        # arm_extended latched until joint feedback proves a safe retract.
        return MissionCommand(arm_action="SAFE_RETRACT", reason=reason)

    def _guard(self, inputs: MissionInputs) -> Optional[MissionCommand]:
        reason = inputs.safety.failure_reason()
        if reason:
            return self._stop(reason)
        if self.arm_extended and not inputs.safety.base_hold_six_dof_zero:
            return self._stop("BASE_MOVED_WHILE_ARM_EXTENDED")
        return None

    def _select(self, targets: Sequence[TargetCandidate]) -> Optional[TargetCandidate]:
        identifiers = [target.identifier for target in targets]
        if (any(not isinstance(identifier, str) or not identifier.strip()
                for identifier in identifiers)
                or len(identifiers) != len(set(identifiers))):
            return None
        eligible = [target for target in targets if target.eligible(self.config)]
        return min(eligible, key=lambda target: (target.grasp_distance_m(), target.identifier)) if eligible else None

    def _live_selected(self, inputs: MissionInputs) -> Optional[TargetCandidate]:
        if inputs.target_feed_state != TargetFeedState.DETECTED or self.selected_target is None:
            return None
        live = next((target for target in inputs.targets
                     if target.identifier == self.selected_target.identifier), None)
        if live is None or not live.eligible(self.config):
            return None
        drift = math.sqrt(sum((current - frozen) ** 2 for current, frozen in zip(
            live.xyz_arm_base_m, self.selected_target.xyz_arm_base_m)))
        return live if drift <= self.config.target_max_drift_m else None

    def _arm_observation_mapping_ready(self) -> bool:
        mapping = self.config.arm_observation_rotation.get("joint_mapping")
        verification = self.config.arm_observation_rotation.get("verification")
        return (mapping != "UNRESOLVED_CONFIGURE_BEFORE_SIM_RUN"
                and verification in ("SIM_TESTED", "FIELD_VALIDATED"))

    def _rotation_window_safe(self, inputs: MissionInputs) -> bool:
        remaining = inputs.rotation_window_in_s
        return (remaining is None
                or (math.isfinite(remaining)
                    and remaining > (self.config.known_rotation_guard_s
                                     + self.config.pick_required_window_s)))

    def tick(self, inputs: MissionInputs) -> MissionCommand:
        if self.state in (MissionState.STOP, MissionState.REFEREE_HOLD):
            return MissionCommand(arm_action="SAFE_RETRACT" if self.state == MissionState.STOP else "NONE",
                                  reason=self.failure_reason or self.state.value)
        guarded = self._guard(inputs)
        if guarded:
            return guarded
        if self.state == MissionState.NAVIGATE_STAGING:
            if inputs.navigation_complete:
                self.state = MissionState.AR_ALIGN
            return MissionCommand(reason="NAVIGATE_STAGING")
        if self.state == MissionState.AR_ALIGN:
            if not inputs.ar or not inputs.ar.valid(self.config):
                return self._stop("AR_STALE_OR_UNSTABLE")
            if inputs.ar.marker_x_m <= self.config.robot_front_extent_m:
                return self._stop("AR_NOT_IN_FRONT")
            clearance = point_to_robot_envelope_clearance(
                inputs.ar.marker_x_m, inputs.ar.marker_y_m, self.config)
            bearing = math.atan2(inputs.ar.marker_y_m, inputs.ar.marker_x_m)
            if not math.isfinite(clearance):
                return self._stop("AR_CLEARANCE_INVALID")
            if clearance < self.config.ar_hard_min_distance_m:
                return self._stop("AR_HARD_MIN_DISTANCE")
            if abs(bearing) > self.config.ar_bearing_tolerance_rad:
                angular = max(-self.config.ar_max_align_rps,
                              min(self.config.ar_max_align_rps, bearing))
                return MissionCommand(wz_rps=angular, reason="AR_HEADING_ALIGN")
            error = clearance - self.config.ar_target_distance_m
            if abs(error) <= self.config.ar_distance_tolerance_m:
                self.state = MissionState.BASE_HOLD
                return MissionCommand(reason="AR_TARGET_REACHED")
            if error < 0.0:
                return self._stop("AR_TARGET_OVERSHOOT")
            return MissionCommand(vx_mps=min(self.config.ar_max_approach_mps, error), reason="AR_MICRO_APPROACH")
        if self.state == MissionState.BASE_HOLD:
            if not inputs.safety.base_hold_six_dof_zero:
                return self._stop("BASE_NOT_SIX_DOF_HOLD")
            arm_reason = inputs.safety.arm_failure_reason()
            if arm_reason:
                return self._stop(arm_reason)
            if not self._arm_observation_mapping_ready():
                return self._stop("ARM_OBSERVATION_MAPPING_UNRESOLVED")
            self.state = MissionState.ARM_OBSERVE
            self.arm_extended = True
            return MissionCommand(arm_action="ARM_OBSERVE_CONFIGURED_ROTATION", reason="BASE_HELD")
        if not inputs.safety.base_hold_six_dof_zero:
            return self._stop("BASE_NOT_SIX_DOF_HOLD")
        arm_reason = inputs.safety.arm_failure_reason()
        if arm_reason:
            return self._stop(arm_reason)
        if self.state == MissionState.ARM_OBSERVE:
            if not inputs.arm_observation_complete:
                return MissionCommand(arm_action="ARM_OBSERVE_CONFIGURED_ROTATION",
                                      reason="WAIT_ARM_OBSERVATION_FEEDBACK")
            self.state = MissionState.SELECT_TARGET
            return MissionCommand(reason="ARM_OBSERVATION_SETTLED")
        if self.state == MissionState.SELECT_TARGET:
            if inputs.target_feed_state != TargetFeedState.DETECTED:
                return self._stop("PERCEPTION_UNAVAILABLE")
            self.selected_target = self._select(inputs.targets)
            if not self.selected_target:
                return self._stop("TARGET_UNAVAILABLE_OR_UNSAFE")
            if not self._rotation_window_safe(inputs):
                return self._stop("ROTATION_GUARD_WINDOW")
            self.state = MissionState.PICK
            self.arm_extended = True
            return MissionCommand(arm_action="PICK_CONFIGURED_POSE", vacuum_on=True, reason="TARGET_SELECTED")
        if self.state == MissionState.PICK:
            if not self._rotation_window_safe(inputs):
                return self._stop("ROTATION_WINDOW_BECAME_UNSAFE")
            if self._live_selected(inputs) is None:
                return self._stop("TARGET_MOVED_OR_LOST")
            if not inputs.pick_complete or not inputs.vacuum_grasp_confirmed:
                return MissionCommand(arm_action="PICK_CONFIGURED_POSE", vacuum_on=True,
                                      reason="WAIT_PICK_AND_VACUUM_FEEDBACK")
            self.state = MissionState.STORE_REAR
            return MissionCommand(arm_action="STORE_REAR_CONFIGURED_POSE", vacuum_on=True, reason="PICK_COMPLETE")
        if self.state == MissionState.STORE_REAR:
            if not self._rotation_window_safe(inputs):
                return self._stop("ROTATION_WINDOW_BECAME_UNSAFE")
            if not inputs.store_complete:
                if not inputs.vacuum_grasp_confirmed:
                    return self._stop("PAYLOAD_LOST_DURING_TRANSFER")
                return MissionCommand(arm_action="STORE_REAR_CONFIGURED_POSE", vacuum_on=True, reason="STORE_IN_PROGRESS")
            if not inputs.vacuum_release_confirmed:
                return MissionCommand(arm_action="STORE_REAR_CONFIGURED_POSE", vacuum_on=False,
                                      reason="WAIT_VACUUM_RELEASE_FEEDBACK")
            self.state = MissionState.ARM_SAFE_RETRACT
            return MissionCommand(arm_action="SAFE_RETRACT", reason="STORED_RETRACT_ARM")
        if self.state == MissionState.ARM_SAFE_RETRACT:
            if not self._rotation_window_safe(inputs):
                return self._stop("ROTATION_WINDOW_BECAME_UNSAFE")
            if not inputs.safe_retract_complete:
                return MissionCommand(arm_action="SAFE_RETRACT", reason="WAIT_SAFE_RETRACT_FEEDBACK")
            self.arm_extended = False
            self.state = MissionState.REFEREE_HOLD
            return MissionCommand(reason="STORED_WAIT_REFEREE")
        return self._stop("INVALID_STATE")
