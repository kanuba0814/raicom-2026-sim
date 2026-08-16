"""Pure geometry contracts for following a team-boundary tangent.

This module has no ROS or simulator dependency.  A boundary line is expressed
in a common 2-D frame.  Its unit tangent gives travel direction and its unit
left normal defines the signed half-plane.  Numeric defaults belong in caller
configuration and remain SIM_WORKING_ASSUMPTION values.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class TangentBoundary:
    point_x_m: float
    point_y_m: float
    tangent_x: float
    tangent_y: float
    allowed_normal_sign: int


@dataclass(frozen=True)
class BoundaryQualification:
    qualified: bool
    reason: str
    robot_cross_track_m: float = float("nan")
    robot_heading_error_rad: float = float("nan")
    circle_tangent_error_m: float = float("nan")
    circle_along_track_m: float = float("nan")


@dataclass(frozen=True)
class FeasibleOffsetInterval:
    feasible: bool
    reason: str
    minimum_center_offset_m: float = float("nan")
    maximum_center_offset_m: float = float("nan")


def feasible_center_offset_interval(circle_signed_distance_m, circle_radius_m,
                                    robot_side_extent_m, clearance_min_m,
                                    clearance_max_m, total_circle_uncertainty_m,
                                    boundary_margin_m):
    """Intersect boundary-safe and robust circle-clearance center offsets.

    Positive center offsets point into the allowed half-plane; a circle on the
    forbidden side has a negative signed distance.  The local line model is a
    qualification gate and never replaces exact mask/footprint sweep checks.
    """
    values = (circle_signed_distance_m, circle_radius_m, robot_side_extent_m,
              clearance_min_m, clearance_max_m, total_circle_uncertainty_m,
              boundary_margin_m)
    if (not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values) or
            circle_radius_m <= 0.0 or min(robot_side_extent_m, clearance_min_m,
            clearance_max_m, total_circle_uncertainty_m, boundary_margin_m) < 0.0 or
            clearance_min_m > clearance_max_m):
        return FeasibleOffsetInterval(False, "FEASIBILITY_INPUT_INVALID")
    if total_circle_uncertainty_m > (clearance_max_m - clearance_min_m) / 2.0:
        return FeasibleOffsetInterval(False, "CLEARANCE_UNCERTAINTY_EXCEEDS_HALF_BAND")
    robust_min_gap_m = clearance_min_m + total_circle_uncertainty_m
    robust_max_gap_m = clearance_max_m - total_circle_uncertainty_m
    # For a forbidden-side circle, center separation along the normal is
    # offset - circle_signed_distance.  Subtract both envelopes for edge gap.
    circle_min_offset_m = (circle_signed_distance_m + circle_radius_m +
                           robot_side_extent_m + robust_min_gap_m)
    circle_max_offset_m = (circle_signed_distance_m + circle_radius_m +
                           robot_side_extent_m + robust_max_gap_m)
    minimum_m = max(robot_side_extent_m + boundary_margin_m, circle_min_offset_m)
    maximum_m = circle_max_offset_m
    if minimum_m > maximum_m:
        return FeasibleOffsetInterval(False, "NO_FEASIBLE_BOUNDARY_CLEARANCE_INTERVAL",
                                      minimum_m, maximum_m)
    return FeasibleOffsetInterval(True, "FEASIBLE", minimum_m, maximum_m)


def _wrap(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def normalized_boundary(boundary):
    values = (boundary.point_x_m, boundary.point_y_m,
              boundary.tangent_x, boundary.tangent_y)
    if (not isinstance(boundary, TangentBoundary) or
            not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values) or
            boundary.allowed_normal_sign not in (-1, 1)):
        return None
    length = math.hypot(boundary.tangent_x, boundary.tangent_y)
    if length <= 1e-9:
        return None
    return TangentBoundary(boundary.point_x_m, boundary.point_y_m,
                           boundary.tangent_x / length, boundary.tangent_y / length,
                           boundary.allowed_normal_sign)


def signed_normal_distance(boundary, x_m, y_m):
    """Distance along the line's left normal; positive is tangent-left."""
    normalized = normalized_boundary(boundary)
    if normalized is None or not all(math.isfinite(value) for value in (x_m, y_m)):
        return float("nan")
    dx_m, dy_m = x_m - normalized.point_x_m, y_m - normalized.point_y_m
    return -normalized.tangent_y * dx_m + normalized.tangent_x * dy_m


def along_track_distance(boundary, x_m, y_m):
    normalized = normalized_boundary(boundary)
    if normalized is None or not all(math.isfinite(value) for value in (x_m, y_m)):
        return float("nan")
    return ((x_m - normalized.point_x_m) * normalized.tangent_x +
            (y_m - normalized.point_y_m) * normalized.tangent_y)


def qualify_tangent_lead_in(boundary, robot_x_m, robot_y_m, robot_yaw_rad,
                            circle_x_m, circle_y_m, circle_radius_m,
                            robot_center_offset_m,
                            robot_cross_track_tolerance_m,
                            heading_tolerance_rad,
                            circle_tangent_tolerance_m,
                            minimum_circle_ahead_m):
    """Validate the external premise required before tangent tracking.

    The circle tangent error is ``abs(line-to-center distance) - radius``.
    This deliberately does not infer a competition boundary from lidar.
    """
    normalized = normalized_boundary(boundary)
    values = (robot_x_m, robot_y_m, robot_yaw_rad, circle_x_m, circle_y_m,
              circle_radius_m, robot_center_offset_m, robot_cross_track_tolerance_m,
              heading_tolerance_rad, circle_tangent_tolerance_m,
              minimum_circle_ahead_m)
    if (normalized is None or not all(isinstance(value, (int, float)) and math.isfinite(value)
                                      for value in values) or
            circle_radius_m <= 0.0 or min(robot_center_offset_m, robot_cross_track_tolerance_m,
            heading_tolerance_rad, circle_tangent_tolerance_m, minimum_circle_ahead_m) < 0.0):
        return BoundaryQualification(False, "BOUNDARY_INPUT_INVALID")
    cross_track_m = signed_normal_distance(normalized, robot_x_m, robot_y_m)
    heading_error_rad = _wrap(robot_yaw_rad - math.atan2(normalized.tangent_y,
                                                         normalized.tangent_x))
    circle_normal_m = signed_normal_distance(normalized, circle_x_m, circle_y_m)
    tangent_error_m = abs(circle_normal_m) - circle_radius_m
    circle_ahead_m = along_track_distance(normalized, circle_x_m, circle_y_m) - \
        along_track_distance(normalized, robot_x_m, robot_y_m)
    allowed_cross_track_m = normalized.allowed_normal_sign * cross_track_m
    if allowed_cross_track_m < 0.0:
        return BoundaryQualification(False, "ROBOT_IN_FORBIDDEN_HALF_PLANE",
                                     cross_track_m, heading_error_rad, tangent_error_m, circle_ahead_m)
    if allowed_cross_track_m < robot_center_offset_m - robot_cross_track_tolerance_m:
        return BoundaryQualification(False, "ROBOT_TOO_CLOSE_TO_BOUNDARY_FOR_LEAD_IN",
                                     cross_track_m, heading_error_rad, tangent_error_m, circle_ahead_m)
    if abs(heading_error_rad) > heading_tolerance_rad:
        return BoundaryQualification(False, "ROBOT_NOT_ALIGNED_WITH_BOUNDARY",
                                     cross_track_m, heading_error_rad, tangent_error_m, circle_ahead_m)
    if abs(tangent_error_m) > circle_tangent_tolerance_m:
        return BoundaryQualification(False, "CIRCLE_NOT_TANGENT_TO_BOUNDARY",
                                     cross_track_m, heading_error_rad, tangent_error_m, circle_ahead_m)
    if circle_ahead_m < minimum_circle_ahead_m:
        return BoundaryQualification(False, "CIRCLE_NOT_AHEAD_ON_TANGENT",
                                     cross_track_m, heading_error_rad, tangent_error_m, circle_ahead_m)
    return BoundaryQualification(True, "QUALIFIED", cross_track_m,
                                 heading_error_rad, tangent_error_m, circle_ahead_m)
