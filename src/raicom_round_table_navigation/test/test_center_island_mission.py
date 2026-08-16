from dataclasses import replace
from pathlib import Path
import math

import pytest

from raicom_round_table_navigation.center_island_mission import (
    ArObservation, CenterIslandMission, IslandConfig, MissionInputs, MissionState,
    SafetyInputs, TargetCandidate, TargetFeedState,
    point_to_robot_envelope_clearance)


RAW_CONFIG = IslandConfig.from_json(str(Path(__file__).parents[1] / "config" / "center_island_mission.json"))
CONFIG = replace(RAW_CONFIG, arm_observation_rotation={
    "semantic": "camera_observation_pose",
    "rotation_degrees": 90.0,
    "joint_mapping": "test_observation_pose",
    "verification": "SIM_TESTED",
})
SAFE = SafetyInputs(
    sim_backend=True, estop_clear=True, team_boundary_permits_motion=True,
    base_gate_owns_publisher=True, base_hold_six_dof_zero=True,
    arm_adapter_available=True, arm_sequence_preflight_ok=True,
    arm_swept_volume_within_team_boundary=True)


def ar(distance=.23, bearing=0.0, age=0.01, frames=3):
    marker_x = CONFIG.robot_front_extent_m + distance
    marker_y = math.tan(bearing) * marker_x
    return ArObservation(4, "base_footprint", marker_x, marker_y, age, frames)


def target(identifier="near", xyz=(.1, .0, .1), **changes):
    values = dict(frame_id="Base", xyz_arm_base_m=xyz, age_s=.01, stable_frames=3,
                  stable_duration_s=1.0, speed_mps=.0, ik_admitted=True)
    values.update(changes)
    return TargetCandidate(identifier, **values)


def to_select(mission):
    mission.tick(MissionInputs(SAFE, navigation_complete=True))
    mission.tick(MissionInputs(SAFE, ar=ar()))
    mission.tick(MissionInputs(SAFE, ar=ar()))
    command = mission.tick(MissionInputs(SAFE))
    assert mission.state == MissionState.ARM_OBSERVE
    assert command.arm_action == "ARM_OBSERVE_CONFIGURED_ROTATION"
    mission.tick(MissionInputs(SAFE, arm_observation_complete=True))
    assert mission.state == MissionState.SELECT_TARGET


def test_config_preserves_id4_hard_min_margin_and_unresolved_timing():
    assert RAW_CONFIG.ar_marker_id == 4
    assert RAW_CONFIG.ar_hard_min_distance_m == .20
    assert RAW_CONFIG.ar_target_distance_m - RAW_CONFIG.ar_distance_tolerance_m >= RAW_CONFIG.ar_hard_min_distance_m
    assert RAW_CONFIG.ar_distance_semantics == "marker_to_robot_envelope_clearance"
    assert RAW_CONFIG.staging_scenario == "blue_center_turntable_north_approach"
    assert RAW_CONFIG.target_frame == "Base"
    assert RAW_CONFIG.rotation_timing_status == "UNRESOLVED"
    assert RAW_CONFIG.arm_observation_rotation["joint_mapping"].startswith("UNRESOLVED")


def test_ar_clearance_uses_robot_envelope_not_camera_range():
    assert point_to_robot_envelope_clearance(.44, 0.0, CONFIG) == .20
    assert point_to_robot_envelope_clearance(.24, .30, CONFIG) == pytest.approx(.10)
    mission = CenterIslandMission(CONFIG)
    mission.tick(MissionInputs(SAFE, navigation_complete=True))
    # A marker only 0.19 m beyond the front envelope must stop.  There is no
    # adapter-provided camera range field that can override this core result.
    observation = ArObservation(4, "base_footprint", .43, 0.0, .01, 3)
    command = mission.tick(MissionInputs(SAFE, ar=observation))
    assert mission.failure_reason == "AR_HARD_MIN_DISTANCE" and command.vx_mps == 0.0


def test_unresolved_arm_mapping_cannot_enter_observation_motion():
    mission = CenterIslandMission(RAW_CONFIG)
    mission.tick(MissionInputs(SAFE, navigation_complete=True))
    mission.tick(MissionInputs(SAFE, ar=ar()))
    command = mission.tick(MissionInputs(SAFE, ar=ar()))
    assert mission.failure_reason == "ARM_OBSERVATION_MAPPING_UNRESOLVED"
    assert command.arm_action == "SAFE_RETRACT"


def test_ar_marker_behind_robot_is_rejected_without_turning():
    mission = CenterIslandMission(CONFIG)
    mission.tick(MissionInputs(SAFE, navigation_complete=True))
    behind = ArObservation(4, "base_footprint", -.50, 0.0, .01, 3)
    command = mission.tick(MissionInputs(SAFE, ar=behind))
    assert mission.failure_reason == "AR_NOT_IN_FRONT"
    assert command.wz_rps == 0.0


def test_ar_stale_or_hard_min_stops_with_zero_output():
    stale = CenterIslandMission(CONFIG)
    stale.tick(MissionInputs(SAFE, navigation_complete=True))
    command = stale.tick(MissionInputs(SAFE, ar=ar(age=1.0)))
    assert stale.state == MissionState.STOP and command.vx_mps == 0.0 and not command.vacuum_on
    too_close = CenterIslandMission(CONFIG)
    too_close.tick(MissionInputs(SAFE, navigation_complete=True))
    command = too_close.tick(MissionInputs(SAFE, ar=ar(distance=.199)))
    assert too_close.failure_reason == "AR_HARD_MIN_DISTANCE" and command.vx_mps == 0.0


def test_ar_heading_is_aligned_before_forward_motion():
    mission = CenterIslandMission(CONFIG)
    mission.tick(MissionInputs(SAFE, navigation_complete=True))
    command = mission.tick(MissionInputs(SAFE, ar=ar(distance=.30, bearing=.1)))
    assert command.vx_mps == 0.0 and command.wz_rps > 0.0
    command = mission.tick(MissionInputs(SAFE, ar=ar(distance=.30)))
    assert command.vx_mps > 0.0 and command.wz_rps == 0.0


@pytest.mark.parametrize("safety_field,reason", [
    ("arm_sequence_preflight_ok", "ARM_SEQUENCE_PREFLIGHT_REJECTED"),
    ("arm_swept_volume_within_team_boundary", "ARM_TEAM_BOUNDARY_REJECTED"),
])
def test_arm_preflight_and_swept_boundary_fail_closed(safety_field, reason):
    mission = CenterIslandMission(CONFIG)
    mission.tick(MissionInputs(SAFE, navigation_complete=True))
    mission.tick(MissionInputs(SAFE, ar=ar()))
    command = mission.tick(MissionInputs(
        replace(SAFE, **{safety_field: False}), ar=ar()))
    assert mission.failure_reason == reason
    assert command.arm_action == "SAFE_RETRACT" and command.vx_mps == 0.0


def test_arm_requires_six_dof_hold_and_extended_arm_latches_base_motion():
    mission = CenterIslandMission(CONFIG)
    mission.tick(MissionInputs(SAFE, navigation_complete=True))
    mission.tick(MissionInputs(SAFE, ar=ar()))
    bad_hold = replace(SAFE, base_hold_six_dof_zero=False)
    command = mission.tick(MissionInputs(bad_hold, ar=ar()))
    assert mission.state == MissionState.STOP and command.arm_action == "SAFE_RETRACT"
    mission = CenterIslandMission(CONFIG)
    to_select(mission)
    assert mission.arm_extended
    mission.tick(MissionInputs(SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()]))
    assert mission.arm_extended
    command = mission.tick(MissionInputs(bad_hold, targets=[target()]))
    assert mission.failure_reason == "BASE_MOVED_WHILE_ARM_EXTENDED" and command.vx_mps == 0.0


def test_target_selection_requires_arm_frame_and_uses_3d_distance_then_ik():
    mission = CenterIslandMission(CONFIG)
    to_select(mission)
    far = target("far", (.3, .0, .1))
    nearest = target("nearest", (.1, .0, .1))
    pixel_only = target("pixel", (.01, .0, .1), frame_id="camera")
    rejected_ik = target("bad-ik", (.01, .0, .1), ik_admitted=False)
    mission.tick(MissionInputs(SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[far, pixel_only, rejected_ik, nearest]))
    assert mission.selected_target == nearest and mission.state == MissionState.PICK


def test_duplicate_target_identifiers_fail_closed():
    mission = CenterIslandMission(CONFIG)
    to_select(mission)
    command = mission.tick(MissionInputs(
        SAFE, target_feed_state=TargetFeedState.DETECTED,
        targets=[target("same", (.1, 0, .1)), target("same", (.2, 0, .1))]))
    assert mission.failure_reason == "TARGET_UNAVAILABLE_OR_UNSAFE"
    assert command.arm_action == "SAFE_RETRACT"


def test_rotation_guard_and_moving_or_lost_target_cancel_without_retry():
    mission = CenterIslandMission(CONFIG)
    to_select(mission)
    command = mission.tick(MissionInputs(SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()], rotation_window_in_s=10.0))
    assert mission.failure_reason == "ROTATION_GUARD_WINDOW" and command.vx_mps == 0.0
    mission = CenterIslandMission(CONFIG)
    to_select(mission)
    mission.tick(MissionInputs(SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()]))
    command = mission.tick(MissionInputs(SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target(speed_mps=.02)]))
    assert mission.failure_reason == "TARGET_MOVED_OR_LOST" and not command.vacuum_on
    mission = CenterIslandMission(CONFIG)
    to_select(mission)
    mission.tick(MissionInputs(SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()]))
    moved = target(xyz=(.12, .0, .1))
    command = mission.tick(MissionInputs(SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[moved]))
    assert mission.failure_reason == "TARGET_MOVED_OR_LOST" and command.arm_action == "SAFE_RETRACT"


def test_rotation_window_expiry_after_pick_stops_before_storage_motion():
    mission = CenterIslandMission(CONFIG)
    to_select(mission)
    mission.tick(MissionInputs(
        SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()],
        rotation_window_in_s=20.0))
    mission.tick(MissionInputs(
        SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()],
        rotation_window_in_s=20.0, pick_complete=True,
        vacuum_grasp_confirmed=True))
    command = mission.tick(MissionInputs(
        SAFE, rotation_window_in_s=10.0, vacuum_grasp_confirmed=True))
    assert mission.failure_reason == "ROTATION_WINDOW_BECAME_UNSAFE"
    assert command.arm_action == "SAFE_RETRACT" and not command.vacuum_on


def test_payload_loss_during_rear_transfer_stops_and_turns_vacuum_off():
    mission = CenterIslandMission(CONFIG)
    to_select(mission)
    mission.tick(MissionInputs(
        SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()]))
    mission.tick(MissionInputs(
        SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()],
        pick_complete=True, vacuum_grasp_confirmed=True))
    command = mission.tick(MissionInputs(SAFE, vacuum_grasp_confirmed=False))
    assert mission.failure_reason == "PAYLOAD_LOST_DURING_TRANSFER"
    assert not command.vacuum_on and command.arm_action == "SAFE_RETRACT"


def test_success_waits_for_referee_and_external_gates_are_not_bypassed():
    mission = CenterIslandMission(CONFIG)
    to_select(mission)
    mission.tick(MissionInputs(SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()]))
    mission.tick(MissionInputs(
        SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()],
        pick_complete=True, vacuum_grasp_confirmed=True))
    command = mission.tick(MissionInputs(
        SAFE, target_feed_state=TargetFeedState.DETECTED, targets=[target()],
        store_complete=True, vacuum_release_confirmed=True))
    assert mission.state == MissionState.ARM_SAFE_RETRACT
    assert command.arm_action == "SAFE_RETRACT" and mission.arm_extended
    command = mission.tick(MissionInputs(SAFE, safe_retract_complete=True))
    assert mission.state == MissionState.REFEREE_HOLD and command.vx_mps == 0.0
    assert mission.tick(MissionInputs(SAFE)).reason == "REFEREE_HOLD"
    unavailable = CenterIslandMission(CONFIG)
    to_select(unavailable)
    assert unavailable.tick(MissionInputs(SAFE)).reason == "PERCEPTION_UNAVAILABLE"
    blocked = CenterIslandMission(CONFIG)
    command = blocked.tick(MissionInputs(replace(SAFE, sim_backend=False)))
    assert blocked.failure_reason == "BACKEND_NOT_SIM" and command.vx_mps == 0.0
