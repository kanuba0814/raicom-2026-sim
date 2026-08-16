"""Pure, fail-closed occupancy-grid swept-footprint guards.

Unknown, occupied, malformed, and out-of-grid cells are never safe. Sampling
uses at most half a grid cell and expands the rectangle by a half-cell diagonal,
so occupied cells touched by a rotated footprint cannot be missed between samples.
"""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OccupancyGridData:
    origin_x_m: float
    origin_y_m: float
    resolution_m: float
    width: int
    height: int
    cells: object


@dataclass(frozen=True)
class Footprint:
    front_m: float
    rear_m: float
    left_m: float
    right_m: float


@dataclass(frozen=True)
class GuardResult:
    safe: bool
    reason: str


def stopping_distance(speed_mps, reaction_time_s, deceleration_mps2):
    if speed_mps < 0.0 or reaction_time_s < 0.0 or deceleration_mps2 <= 0.0:
        raise ValueError("invalid stopping-distance inputs")
    return speed_mps * reaction_time_s + speed_mps * speed_mps / (2.0 * deceleration_mps2)


def _valid_grid(grid):
    return (isinstance(grid, OccupancyGridData) and grid.resolution_m > 0.0 and grid.width > 0 and grid.height > 0 and
            len(grid.cells) >= grid.width * grid.height and all(math.isfinite(value) for value in
            (grid.origin_x_m, grid.origin_y_m, grid.resolution_m)))


def _footprint_samples(footprint, spacing_m, cell_coverage_margin_m):
    """Fill an outward-expanded rectangle, not only its perimeter.

    A half-cell diagonal margin encloses every grid cell that can intersect the
    physical rectangle even after its samples are rotated.
    """
    if (spacing_m <= 0.0 or cell_coverage_margin_m < 0.0 or not isinstance(footprint, Footprint) or
            min(footprint.front_m, footprint.rear_m, footprint.left_m, footprint.right_m) < 0.0):
        raise ValueError("invalid footprint, coverage margin, or sample spacing")
    minimum_x = -footprint.rear_m - cell_coverage_margin_m
    maximum_x = footprint.front_m + cell_coverage_margin_m
    minimum_y = -footprint.right_m - cell_coverage_margin_m
    maximum_y = footprint.left_m + cell_coverage_margin_m
    x_count = max(1, int(math.ceil((maximum_x - minimum_x) / spacing_m)))
    y_count = max(1, int(math.ceil((maximum_y - minimum_y) / spacing_m)))
    return [(minimum_x + x_index * (maximum_x - minimum_x) / x_count,
             minimum_y + y_index * (maximum_y - minimum_y) / y_count)
            for x_index in range(x_count + 1) for y_index in range(y_count + 1)]


def _cell_is_free(grid, x_m, y_m):
    column = int(math.floor((x_m - grid.origin_x_m) / grid.resolution_m))
    row = int(math.floor((y_m - grid.origin_y_m) / grid.resolution_m))
    if column < 0 or row < 0 or column >= grid.width or row >= grid.height:
        return False, "OUT_OF_BOUNDS"
    if grid.cells[row * grid.width + column] != 0:
        return False, "UNKNOWN_OR_OCCUPIED"
    return True, "FREE"


def _guard_pose(grid, samples, pose_x_m, pose_y_m, yaw_rad):
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    for local_x, local_y in samples:
        safe, reason = _cell_is_free(grid, pose_x_m + local_x * cos_yaw - local_y * sin_yaw,
                                     pose_y_m + local_x * sin_yaw + local_y * cos_yaw)
        if not safe:
            return GuardResult(False, reason)
    return GuardResult(True, "FREE")


def _prepare(grid, footprint, values, sample_spacing_m):
    if not _valid_grid(grid) or not all(math.isfinite(value) for value in values):
        return None, GuardResult(False, "GRID_OR_POSE_INVALID")
    spacing = sample_spacing_m if sample_spacing_m is not None else grid.resolution_m / 2.0
    # Callers cannot weaken the conservative default with a wider spacing.
    if not math.isfinite(spacing) or spacing <= 0.0 or spacing > grid.resolution_m / 2.0:
        return None, GuardResult(False, "SAMPLE_SPACING_INVALID")
    try:
        return _footprint_samples(footprint, spacing, grid.resolution_m / math.sqrt(2.0)), None
    except ValueError:
        return None, GuardResult(False, "FOOTPRINT_INVALID")


def guard_footprint(grid, footprint, pose_x_m, pose_y_m, yaw_rad, vx_mps=0.0, vy_mps=0.0,
                    reaction_time_s=0.0, deceleration_mps2=1.0, sample_spacing_m=None):
    """Guard every footprint along the full translation to the stopping position."""
    samples, failure = _prepare(grid, footprint, (pose_x_m, pose_y_m, yaw_rad, vx_mps, vy_mps), sample_spacing_m)
    if failure:
        return failure
    speed = math.hypot(vx_mps, vy_mps)
    try:
        distance = stopping_distance(speed, reaction_time_s, deceleration_mps2)
    except ValueError:
        return GuardResult(False, "STOPPING_MODEL_INVALID")
    end_x = pose_x_m + (vx_mps / speed * distance if speed else 0.0)
    end_y = pose_y_m + (vy_mps / speed * distance if speed else 0.0)
    pose_count = max(1, int(math.ceil(math.hypot(end_x - pose_x_m, end_y - pose_y_m) / (grid.resolution_m / 2.0))))
    for index in range(pose_count + 1):
        ratio = index / pose_count
        result = _guard_pose(grid, samples, pose_x_m + ratio * (end_x - pose_x_m), pose_y_m + ratio * (end_y - pose_y_m), yaw_rad)
        if not result.safe:
            return result
    return GuardResult(True, "FREE")


def guard_allowed_half_plane(boundary, footprint, pose_x_m, pose_y_m, yaw_rad,
                             vx_mps=0.0, vy_mps=0.0, reaction_time_s=0.0,
                             deceleration_mps2=1.0):
    """Guard the footprint and stopping translation against a configured line.

    This geometric gate complements the occupancy-grid guard; it never infers a
    competition boundary from sensor data.  Both the current and stopping poses
    are checked because signed distance is linear along the translation.
    """
    from .tangent_boundary import normalized_boundary, signed_normal_distance

    normalized = normalized_boundary(boundary)
    values = (pose_x_m, pose_y_m, yaw_rad, vx_mps, vy_mps,
              reaction_time_s, deceleration_mps2)
    if (normalized is None or not isinstance(footprint, Footprint) or
            not all(isinstance(value, (int, float)) and math.isfinite(value)
                    for value in values) or
            min(footprint.front_m, footprint.rear_m,
                footprint.left_m, footprint.right_m) < 0.0):
        return GuardResult(False, "BOUNDARY_OR_POSE_INVALID")
    try:
        distance_m = stopping_distance(math.hypot(vx_mps, vy_mps),
                                       reaction_time_s, deceleration_mps2)
    except ValueError:
        return GuardResult(False, "STOPPING_MODEL_INVALID")
    speed_mps = math.hypot(vx_mps, vy_mps)
    end_x_m = pose_x_m + (vx_mps / speed_mps * distance_m if speed_mps else 0.0)
    end_y_m = pose_y_m + (vy_mps / speed_mps * distance_m if speed_mps else 0.0)
    cos_yaw, sin_yaw = math.cos(yaw_rad), math.sin(yaw_rad)
    corners = ((footprint.front_m, footprint.left_m),
               (footprint.front_m, -footprint.right_m),
               (-footprint.rear_m, footprint.left_m),
               (-footprint.rear_m, -footprint.right_m))
    for center_x_m, center_y_m in ((pose_x_m, pose_y_m), (end_x_m, end_y_m)):
        for local_x_m, local_y_m in corners:
            x_m = center_x_m + local_x_m * cos_yaw - local_y_m * sin_yaw
            y_m = center_y_m + local_x_m * sin_yaw + local_y_m * cos_yaw
            distance = signed_normal_distance(normalized, x_m, y_m)
            if normalized.allowed_normal_sign * distance < -1e-9:
                return GuardResult(False, "FORBIDDEN_HALF_PLANE_CROSSED")
    return GuardResult(True, "ALLOWED_HALF_PLANE")


def guard_rotation_sweep(grid, footprint, pose_x_m, pose_y_m, yaw_start_rad, yaw_end_rad, sample_spacing_m=None):
    """Guard the full swept rectangle for an in-place rotation, e.g. ±90 degrees."""
    samples, failure = _prepare(grid, footprint, (pose_x_m, pose_y_m, yaw_start_rad, yaw_end_rad), sample_spacing_m)
    if failure:
        return failure
    corner_radius = math.hypot(max(footprint.front_m, footprint.rear_m), max(footprint.left_m, footprint.right_m))
    if corner_radius == 0.0:
        step_angle = math.pi
    else:
        step_angle = (grid.resolution_m / 2.0) / corner_radius
    sample_count = max(1, int(math.ceil(abs(yaw_end_rad - yaw_start_rad) / step_angle)))
    for index in range(sample_count + 1):
        result = _guard_pose(grid, samples, pose_x_m, pose_y_m, yaw_start_rad + index * (yaw_end_rad - yaw_start_rad) / sample_count)
        if not result.safe:
            return result
    return GuardResult(True, "FREE")
