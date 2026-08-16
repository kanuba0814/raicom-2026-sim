import math

from raicom_round_table_navigation.boundary_guard import Footprint, guard_allowed_half_plane
from raicom_round_table_navigation.tangent_boundary import (
    TangentBoundary, feasible_center_offset_interval,
    qualify_tangent_lead_in, signed_normal_distance,
)


# Vertical travel (+y); left normal is -x. Allowed side is positive normal (x<0).
BOUNDARY = TangentBoundary(0.0, 0.0, 0.0, 1.0, 1)


def qualify(**changes):
    values = dict(boundary=BOUNDARY, robot_x_m=-0.24, robot_y_m=-0.50,
                  robot_yaw_rad=math.pi / 2.0, circle_x_m=0.20,
                  circle_y_m=0.20, circle_radius_m=0.20,
                  robot_center_offset_m=0.24,
                  robot_cross_track_tolerance_m=0.02,
                  heading_tolerance_rad=0.05,
                  circle_tangent_tolerance_m=0.02,
                  minimum_circle_ahead_m=0.20)
    values.update(changes)
    return qualify_tangent_lead_in(**values)


def test_signed_distance_and_offset_lead_in_qualification():
    assert signed_normal_distance(BOUNDARY, -0.24, 0.0) == 0.24
    result = qualify()
    assert result.qualified
    assert result.reason == "QUALIFIED"
    assert math.isclose(result.robot_cross_track_m, 0.24)
    assert math.isclose(result.circle_tangent_error_m, 0.0)


def test_qualification_rejects_wrong_half_plane_offset_heading_tangency_and_progress():
    assert qualify(robot_x_m=0.01).reason == "ROBOT_IN_FORBIDDEN_HALF_PLANE"
    assert qualify(robot_x_m=-0.20).reason == "ROBOT_TOO_CLOSE_TO_BOUNDARY_FOR_LEAD_IN"
    assert qualify(robot_x_m=-0.30).qualified  # farther inside the allowed side is safe
    assert qualify(robot_yaw_rad=0.0).reason == "ROBOT_NOT_ALIGNED_WITH_BOUNDARY"
    assert qualify(circle_x_m=0.25).reason == "CIRCLE_NOT_TANGENT_TO_BOUNDARY"
    assert qualify(circle_y_m=-0.40).reason == "CIRCLE_NOT_AHEAD_ON_TANGENT"


def test_half_plane_guard_accepts_offset_parallel_motion_and_rejects_crossing():
    footprint = Footprint(0.24, 0.24, 0.20, 0.20)
    # Center x=-0.25 leaves footprint on allowed x<=0 side.
    parallel = guard_allowed_half_plane(BOUNDARY, footprint, -0.25, -0.5,
                                        math.pi / 2.0, 0.0, 0.05, 0.2, 0.5)
    assert parallel.safe
    crossing = guard_allowed_half_plane(BOUNDARY, footprint, -0.25, -0.5,
                                        math.pi / 2.0, 0.20, 0.05, 0.2, 0.5)
    assert not crossing.safe
    assert crossing.reason == "FORBIDDEN_HALF_PLANE_CROSSED"


def test_boundary_and_circle_clearance_intervals_must_have_common_offset():
    feasible = feasible_center_offset_interval(
        circle_signed_distance_m=-0.20, circle_radius_m=0.20,
        robot_side_extent_m=0.20, clearance_min_m=0.03,
        clearance_max_m=0.05, total_circle_uncertainty_m=0.005,
        boundary_margin_m=0.005)
    assert feasible.feasible
    assert math.isclose(feasible.minimum_center_offset_m, 0.235)
    assert math.isclose(feasible.maximum_center_offset_m, 0.245)

    # The observed SIM raster can place the circle edge 4.5 cm behind the
    # boundary. With a 1 cm boundary margin, the robust 3--5 cm band has no
    # common robot-center offset and motion must remain blocked.
    infeasible = feasible_center_offset_interval(
        circle_signed_distance_m=-0.245, circle_radius_m=0.20,
        robot_side_extent_m=0.20, clearance_min_m=0.03,
        clearance_max_m=0.05, total_circle_uncertainty_m=0.005,
        boundary_margin_m=0.01)
    assert not infeasible.feasible
    assert infeasible.reason == "NO_FEASIBLE_BOUNDARY_CLEARANCE_INTERVAL"


def test_clearance_uncertainty_above_one_centimetre_is_not_robustly_feasible():
    result = feasible_center_offset_interval(-0.20, 0.20, 0.20,
                                             0.03, 0.05, 0.011, 0.0)
    assert not result.feasible
    assert result.reason == "CLEARANCE_UNCERTAINTY_EXCEEDS_HALF_BAND"


def test_half_plane_guard_fails_closed_for_invalid_boundary():
    invalid = TangentBoundary(0.0, 0.0, 0.0, 0.0, 1)
    result = guard_allowed_half_plane(invalid, Footprint(.24, .24, .20, .20),
                                      0.0, 0.0, 0.0)
    assert not result.safe
    assert result.reason == "BOUNDARY_OR_POSE_INVALID"
