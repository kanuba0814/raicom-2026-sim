"""Pure, SIM-only, bench-local SwiftPro slot pick/place planning.

The caller must transform the current camera-derived ``/raicom/workbench/slots``
poses into the configured arm base frame before calling this module.  The plan
contains no ROS transport and no executable end-effector command: attachment
transitions are explicit intents for a separately opt-in SIM executor.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Optional, Tuple

from .config import CONFIG, SimTaskArmConfig
from .kinematics import admission_reason, inverse_kinematics


class SlotSequenceStatus(str, Enum):
    READY = "READY"
    INVALID = "INVALID"
    STALE = "STALE"
    UNREACHABLE = "UNREACHABLE"


class SlotAction(str, Enum):
    MOVE = "MOVE"
    VACUUM_ON = "VACUUM_ON"
    VACUUM_OFF = "VACUUM_OFF"


@dataclass(frozen=True)
class SlotObservation:
    """A complete transformed camera observation; poses are arm-base-frame m."""
    frame_id: Any
    slots_xyz_m: Any
    observed_at_s: Any
    valid: Any


@dataclass(frozen=True)
class SlotSequenceStep:
    name: str
    action: SlotAction
    target_xyz_m: Optional[Tuple[float, float, float]] = None
    target_independent_rad: Optional[Tuple[float, float, float]] = None


@dataclass(frozen=True)
class SlotSequenceResult:
    status: SlotSequenceStatus
    reason: str
    placement_slot_index: Optional[int] = None
    steps: Tuple[SlotSequenceStep, ...] = ()


def _number_tuple(value: Any, size: int) -> Optional[Tuple[float, ...]]:
    if not isinstance(value, (tuple, list)) or len(value) != size:
        return None
    if any(isinstance(item, bool) for item in value):
        return None
    try:
        values = tuple(float(item) for item in value)
    except (TypeError, ValueError):
        return None
    return values if all(math.isfinite(item) for item in values) else None


def _validated_slots(observation: Any, now_s: Any, config: SimTaskArmConfig) -> tuple[Optional[Tuple[Tuple[float, float, float], ...]], str]:
    if not isinstance(observation, SlotObservation) or observation.valid is not True:
        return None, "SLOTS_INVALID"
    if observation.frame_id != config.frames["base"]:
        return None, "SLOT_FRAME_MISMATCH"
    try:
        observed, now = float(observation.observed_at_s), float(now_s)
    except (TypeError, ValueError):
        return None, "SLOTS_INVALID"
    if not math.isfinite(observed) or not math.isfinite(now):
        return None, "SLOTS_INVALID"
    if observed > now or now - observed > config.thresholds["observation_max_age_s"]:
        return None, "SLOTS_STALE"
    if not isinstance(observation.slots_xyz_m, (tuple, list)) or len(observation.slots_xyz_m) != 4:
        return None, "SLOT_COUNT_MISMATCH"
    slots = tuple(_number_tuple(slot, 3) for slot in observation.slots_xyz_m)
    if any(slot is None for slot in slots):
        return None, "SLOT_NONFINITE"
    return slots, ""


def _sequence_config(config: Any) -> Optional[Tuple[float, float, float, int, Tuple[int, int]]]:
    """Snapshot and validate the public configuration boundary."""
    if not isinstance(config, SimTaskArmConfig):
        return None
    try:
        sequence = config.slot_pick_place
        if set(sequence) != {
            "observation_height_m", "grasp_height_m", "place_height_m",
            "source_slot_index", "placement_slot_indices",
        }:
            return None
        heights = _number_tuple((
            sequence["observation_height_m"], sequence["grasp_height_m"],
            sequence["place_height_m"],
        ), 3)
        source = sequence["source_slot_index"]
        destinations = sequence["placement_slot_indices"]
        if (heights is None or heights[0] <= heights[1] or heights[0] <= heights[2]
                or isinstance(source, bool) or not isinstance(source, int)
                or not 0 <= source < 4
                or not isinstance(destinations, (tuple, list))
                or len(destinations) != 2
                or any(isinstance(slot, bool) or not isinstance(slot, int)
                       or not 0 <= slot < 4 for slot in destinations)
                or len(set(destinations)) != 2 or source in destinations):
            return None
        z_bounds = config.workspace_m[2]
        if (len(config.workspace_m) != 3 or len(z_bounds) != 2
                or not all(z_bounds[0] <= height <= z_bounds[1]
                           for height in heights)
                or not isinstance(config.frames["base"], str)
                or not config.frames["base"]
                or config.thresholds["observation_max_age_s"] <= 0.0):
            return None
        return heights + (source, tuple(destinations))  # type: ignore[return-value]
    except (KeyError, TypeError, ValueError, IndexError):
        return None


def plan_slot_pick_place(observation: Any, placement_choice: Any, now_s: Any,
                         config: SimTaskArmConfig = CONFIG) -> SlotSequenceResult:
    """Create a fail-closed plan for one configured destination slot.

    Slot Z values are intentionally not used for motion. Only transformed slot
    X/Y provide lateral alignment; every motion Z comes from explicit
    SIM_WORKING_ASSUMPTION configuration.
    """
    sequence = _sequence_config(config)
    if sequence is None:
        return SlotSequenceResult(SlotSequenceStatus.INVALID, "INVALID_CONFIG")
    observation_height, grasp_height, place_height, source_index, destinations = sequence
    if (isinstance(placement_choice, bool)
            or not isinstance(placement_choice, int)
            or not 0 <= placement_choice < len(destinations)):
        return SlotSequenceResult(SlotSequenceStatus.INVALID, "INVALID_PLACEMENT_CHOICE")
    try:
        slots, reason = _validated_slots(observation, now_s, config)
    except (KeyError, TypeError, ValueError, IndexError):
        return SlotSequenceResult(SlotSequenceStatus.INVALID, "INVALID_CONFIG")
    if slots is None:
        status = SlotSequenceStatus.STALE if reason == "SLOTS_STALE" else SlotSequenceStatus.INVALID
        return SlotSequenceResult(status, reason)

    destination_index = destinations[placement_choice]
    source_xy, destination_xy = slots[source_index][:2], slots[destination_index][:2]
    targets = (
        ("ALIGN_SOURCE", source_xy + (observation_height,)),
        ("DESCEND_SOURCE", source_xy + (grasp_height,)),
        ("LIFT_SOURCE", source_xy + (observation_height,)),
        ("ALIGN_DESTINATION", destination_xy + (observation_height,)),
        ("DESCEND_DESTINATION", destination_xy + (place_height,)),
        ("LIFT_DESTINATION", destination_xy + (observation_height,)),
    )
    moves = []
    for name, target in targets:
        try:
            rejected = admission_reason(target, config)
            joints = inverse_kinematics(target, config)
        except (KeyError, TypeError, ValueError, IndexError):
            return SlotSequenceResult(SlotSequenceStatus.INVALID, "INVALID_CONFIG")
        if rejected is not None or joints is None:
            return SlotSequenceResult(SlotSequenceStatus.UNREACHABLE, "%s_%s" % (name, rejected or "UNREACHABLE"), destination_index)
        moves.append(SlotSequenceStep(name, SlotAction.MOVE, target, joints))
    by_name = {step.name: step for step in moves}
    steps = (
        by_name["ALIGN_SOURCE"], by_name["DESCEND_SOURCE"],
        SlotSequenceStep("VACUUM_ON", SlotAction.VACUUM_ON),
        by_name["LIFT_SOURCE"], by_name["ALIGN_DESTINATION"],
        by_name["DESCEND_DESTINATION"],
        SlotSequenceStep("VACUUM_OFF", SlotAction.VACUUM_OFF),
        by_name["LIFT_DESTINATION"],
    )
    return SlotSequenceResult(SlotSequenceStatus.READY, "READY", destination_index, steps)
