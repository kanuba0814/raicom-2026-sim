"""Pure contract for one camera payload pick invocation; no Gazebo dependencies."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Protocol, Tuple


@dataclass(frozen=True)
class PickRequest:
    payload: str
    source_slot: int
    rear_slot: int


def request_from_config(config: dict, payload: Any) -> PickRequest | None:
    if not isinstance(payload, str) or not isinstance(config, dict):
        return None
    entries = config.get("payloads")
    if not isinstance(entries, dict) or payload not in entries:
        return None
    entry = entries[payload]
    if not isinstance(entry, dict) or set(entry) != {"source_slot", "rear_slot"}:
        return None
    source, rear = entry["source_slot"], entry["rear_slot"]
    if (not payload.strip() or isinstance(source, bool) or isinstance(rear, bool)
            or not isinstance(source, int) or not isinstance(rear, int)
            or source <= 0 or rear <= 0):
        return None
    return PickRequest(payload, source, rear)


def cartesian_line(start: Any, end: Any, max_spacing_m: Any) -> Tuple[Tuple[float, float, float], ...]:
    """Return endpoint-inclusive Cartesian waypoints with bounded straight-line spacing."""
    try:
        first = tuple(float(value) for value in start)
        last = tuple(float(value) for value in end)
        spacing = float(max_spacing_m)
    except (TypeError, ValueError):
        return ()
    if (len(first) != 3 or len(last) != 3 or spacing <= 0.0
            or not all(math.isfinite(value) for value in first + last + (spacing,))):
        return ()
    distance = math.dist(first, last)
    steps = max(1, math.ceil(distance / spacing))
    return tuple(
        tuple(first[axis] + (last[axis] - first[axis]) * step / steps
              for axis in range(3))
        for step in range(1, steps + 1)
    )


@dataclass(frozen=True)
class CartesianStage:
    name: str
    target_base_m: Tuple[float, float, float]


@dataclass(frozen=True)
class PickMotionPlan:
    pre_grasp: Tuple[CartesianStage, ...]
    post_grasp: Tuple[CartesianStage, ...]
    rear_place_stage: str = "REAR_PLACE"
    rear_retract_stage: str = "REAR_RETRACT"

    @property
    def all_cartesian(self) -> Tuple[CartesianStage, ...]:
        return self.pre_grasp + self.post_grasp


class PickTransactionBackend(Protocol):
    """Platform seam; implementations own transport, feedback, and admission."""

    def preflight(self, stages: Tuple[CartesianStage, ...]) -> bool: ...
    def move_and_settle(self, target_base_m: Tuple[float, float, float], stage: str) -> bool: ...
    def set_vacuum_verified(self, enabled: bool) -> bool: ...
    def home_and_settle(self) -> bool: ...
    def set_arm_extended(self, extended: bool) -> None: ...


@dataclass(frozen=True)
class PickTransactionResult:
    status: str
    reason: str
    grasped: bool
    arm_extended: bool


def execute_motion_plan(
    backend: PickTransactionBackend,
    plan: PickMotionPlan,
) -> PickTransactionResult:
    """Execute the shared transaction; backend feedback decides every transition."""
    backend.set_arm_extended(True)
    if not backend.set_vacuum_verified(False):
        return PickTransactionResult(
            "ABORTED", "INITIAL_VACUUM_OFF_UNVERIFIED", False, True,
        )
    if not backend.preflight(plan.all_cartesian):
        return PickTransactionResult("ABORTED", "PREFLIGHT_FAILED", False, True)
    for stage in plan.pre_grasp:
        if not backend.move_and_settle(stage.target_base_m, stage.name):
            released = backend.set_vacuum_verified(False)
            homed = released and backend.home_and_settle()
            if homed:
                backend.set_arm_extended(False)
            return PickTransactionResult(
                "ABORTED", "PRE_GRASP_MOTION_FAILED", False, not homed,
            )
    if not backend.set_vacuum_verified(True):
        released = backend.set_vacuum_verified(False)
        homed = released and backend.home_and_settle()
        if homed:
            backend.set_arm_extended(False)
        return PickTransactionResult(
            "ABORTED", "GRASP_FEEDBACK_UNVERIFIED", False, not homed,
        )
    released = False
    for stage in plan.post_grasp:
        if stage.name == plan.rear_retract_stage:
            if not backend.set_vacuum_verified(False):
                return PickTransactionResult(
                    "SAFE_HOLD", "RELEASE_FEEDBACK_UNVERIFIED", True, True,
                )
            released = True
        if not backend.move_and_settle(stage.target_base_m, stage.name):
            reason = "POST_RELEASE_MOTION_FAILED" if released else "POST_GRASP_MOTION_FAILED"
            return PickTransactionResult("SAFE_HOLD", reason, not released, True)
    if not backend.home_and_settle():
        return PickTransactionResult("SAFE_HOLD", "HOME_FAILED", False, True)
    backend.set_arm_extended(False)
    return PickTransactionResult("REFEREE_HOLD", "COMPLETE", False, False)


def build_motion_plan(
    pick: Any,
    normal: Any,
    rear_hover: Any,
    rear_place: Any,
    hover_m: Any,
    lift_m: Any,
    waypoint_spacing_m: Any,
) -> PickMotionPlan | None:
    """Build a transport-neutral Base-frame plan from calibrated geometry."""
    try:
        pick_xyz = tuple(float(value) for value in pick)
        outward = tuple(float(value) for value in normal)
        rear_hover_xyz = tuple(float(value) for value in rear_hover)
        rear_place_xyz = tuple(float(value) for value in rear_place)
        hover_distance = float(hover_m)
        lift_distance = float(lift_m)
        spacing = float(waypoint_spacing_m)
    except (TypeError, ValueError):
        return None
    values = pick_xyz + outward + rear_hover_xyz + rear_place_xyz + (
        hover_distance, lift_distance, spacing,
    )
    length = math.sqrt(sum(value * value for value in outward))
    if (any(len(item) != 3 for item in (pick_xyz, outward, rear_hover_xyz, rear_place_xyz))
            or not all(math.isfinite(value) for value in values)
            or abs(length - 1.0) > 0.02
            or hover_distance <= 0.0 or lift_distance <= 0.0 or spacing <= 0.0):
        return None
    outward = tuple(value / length for value in outward)
    hover = tuple(pick_xyz[i] + outward[i] * hover_distance for i in range(3))
    lift = tuple(pick_xyz[i] + outward[i] * lift_distance for i in range(3))
    descend = cartesian_line(hover, pick_xyz, spacing)
    lift_line = cartesian_line(pick_xyz, lift, spacing)
    if not descend or not lift_line:
        return None
    pre_grasp = (CartesianStage("SOURCE_HOVER", hover),) + tuple(
        CartesianStage("SOURCE_DESCEND_%02d" % index, point)
        for index, point in enumerate(descend, 1)
    )
    post_grasp = tuple(
        CartesianStage("SOURCE_LIFT_%02d" % index, point)
        for index, point in enumerate(lift_line, 1)
    ) + (
        CartesianStage("REAR_HOVER", rear_hover_xyz),
        CartesianStage("REAR_PLACE", rear_place_xyz),
        CartesianStage("REAR_RETRACT", rear_hover_xyz),
    )
    return PickMotionPlan(pre_grasp, post_grasp)


def sequence(request: PickRequest, destination_enabled: bool) -> Tuple[str, ...]:
    stages = ("OBSERVE", "PREFLIGHT", "SOURCE_HOVER", "SOURCE_PICK", "VACUUM_ON", "SOURCE_LIFT")
    if destination_enabled:
        stages += ("REAR_HOVER", "REAR_PLACE", "VACUUM_OFF", "REAR_RETRACT")
    return stages + ("HOME", "REFEREE_HOLD")
