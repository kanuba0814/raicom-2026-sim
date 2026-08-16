import math

from raicom_round_table_navigation.docking_geometry import (
    DockingConfig, DockingGeometry, DockingPhase, front_gap, rectangle_clearance,
)


def test_front_gap_is_measured_from_front_envelope_to_cylinder_edge():
    assert math.isclose(front_gap(.55, .25, .24), .06)


def test_side_front_target_can_rotate_in_place_when_envelope_is_safe():
    assert rectangle_clearance(.20, .60, .19, .24, .24, .20) > .01
    decision = DockingGeometry().decide(.20, .60, .19)
    assert decision.phase == DockingPhase.ALIGN_CENTER
    assert decision.suggestion.vx_mps == decision.suggestion.vy_mps == 0.0
    assert decision.suggestion.wz_rps > 0.0


def test_align_center_rotates_only_then_approaches_forward_only():
    dock = DockingGeometry()
    align = dock.decide(1.0, .06, .25)
    assert align.phase == DockingPhase.ALIGN_CENTER
    assert align.suggestion.vx_mps == 0.0 and align.suggestion.vy_mps == 0.0
    assert 0.0 < align.suggestion.wz_rps <= .15
    assert dock.decide(1.0, .01, .25).reason == "WAITING_FOR_ALIGNMENT_STABILITY"
    assert dock.decide(1.0, .01, .25).reason == "WAITING_FOR_ALIGNMENT_STABILITY"
    approach = dock.decide(1.0, .01, .25)
    assert approach.phase == DockingPhase.APPROACH_CENTER
    assert 0.0 < approach.suggestion.vx_mps <= .03
    assert approach.suggestion.vy_mps == approach.suggestion.wz_rps == 0.0


def test_approach_realigns_with_zero_when_angle_exceeds_limit():
    dock = DockingGeometry()
    for _ in range(3):
        dock.decide(1.0, 0.0, .25)
    decision = dock.decide(1.0, .06, .25)
    assert decision.phase == DockingPhase.ALIGN_CENTER
    assert decision.reason == "REALIGN_CENTER"
    assert decision.suggestion == type(decision.suggestion)()


def test_completion_requires_three_front_gap_frames_and_reports_uncertainty():
    dock = DockingGeometry(DockingConfig(required_complete_frames=3))
    for _ in range(2):
        assert dock.decide(.52, 0.0, .25, clearance_uncertainty_m=.01).reason == "WAITING_FOR_ALIGNMENT_STABILITY"
    for expected_frames in (1, 2):
        decision = dock.decide(.52, 0.0, .25, clearance_uncertainty_m=.01)
        assert decision.phase == DockingPhase.APPROACH_CENTER
        assert decision.stable_frames == expected_frames
    done = dock.decide(.52, 0.0, .25, clearance_uncertainty_m=.01)
    assert done.phase == DockingPhase.COMPLETE
    assert done.reason == "STABLE_FRONT_GAP_0.03M_PLUS_MINUS_0.01M"


def test_completion_frames_reset_when_gap_leaves_window():
    dock = DockingGeometry(DockingConfig(required_complete_frames=3))
    for _ in range(3):
        dock.decide(.52, 0.0, .25, clearance_uncertainty_m=.01)
    assert dock.decide(.52, 0.0, .25, clearance_uncertainty_m=.01).stable_frames == 2
    outside = dock.decide(.60, 0.0, .25, clearance_uncertainty_m=.01)
    assert outside.reason == "APPROACH_CENTER"
    resumed = dock.decide(.52, 0.0, .25, clearance_uncertainty_m=.01)
    assert resumed.phase == DockingPhase.APPROACH_CENTER
    assert resumed.stable_frames == 1


def test_hard_minimum_overshoot_behind_and_invalid_latch_stop():
    cases = [
        (.49, 0.0, .25, "HARD_MINIMUM_ENVELOPE_GAP_VIOLATION"),
        (-1.0, 0.0, .25, "TARGET_NOT_IN_FORWARD_HEMISPHERE"),
        (float("nan"), 0.0, .25, "OBSERVATION_INVALID_OR_UNSTABLE"),
    ]
    for cx, cy, radius, reason in cases:
        dock = DockingGeometry()
        decision = dock.decide(cx, cy, radius)
        assert decision.phase == DockingPhase.STOP and decision.reason == reason
        assert dock.decide(1.0, 0.0, .25).reason == "STOP_LATCHED"


def test_uncertainty_is_subtracted_from_hard_front_gap():
    dock = DockingGeometry()
    for _ in range(3):
        dock.decide(1.0, 0.0, .25)
    decision = dock.decide(.50, 0.0, .25, clearance_uncertainty_m=.01)
    assert decision.phase == DockingPhase.STOP
    assert decision.reason == "HARD_MINIMUM_ENVELOPE_GAP_VIOLATION"
