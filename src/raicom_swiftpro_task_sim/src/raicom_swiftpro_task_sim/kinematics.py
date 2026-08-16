"""SIM-only kinematic projection and pure geometry helpers.

Projection provenance: vendor_upstream/oryxbot_ws/src/oryxbot_description/include/
forward_kinematics.hh and global_constants.hh. The upstream interface takes
three degree inputs (Joint1, Joint2, Joint6) and produces nine degree display
joints. This module converts the public interface to rad/m and reads all model
parameters from config.py. The Cartesian feedback projection is the algebraic
inverse of the same upstream inverse-kinematics triangle, not a separate vendor
FK claim.

The Cartesian functions retain only upstream inverse_kinematics.hh geometry,
under SIM_WORKING_ASSUMPTION configuration; they are not hardware validation.
"""
from __future__ import annotations
from dataclasses import dataclass
import math
from typing import Any, Optional, Sequence, Tuple
from .config import CONFIG, SimTaskArmConfig

DEG = math.pi / 180.0

@dataclass(frozen=True)
class RigidTransform:
    parent_frame: str
    child_frame: str
    rotation: Tuple[Tuple[float, float, float], ...]
    translation_m: Tuple[float, float, float]


@dataclass(frozen=True)
class ConditioningMetrics:
    elbow_rad: float
    fold_margin_m: float


def _numbers(values: Any, size: int) -> Optional[Tuple[float, ...]]:
    if not isinstance(values, (tuple, list)) or len(values) != size: return None
    if any(isinstance(value, bool) for value in values): return None
    try: result = tuple(float(value) for value in values)
    except (TypeError, ValueError): return None
    return result if all(math.isfinite(value) for value in result) else None


def independent_in_limits(joints_rad: Any, config: SimTaskArmConfig = CONFIG) -> bool:
    values = _numbers(joints_rad, 3)
    return (isinstance(config, SimTaskArmConfig) and values is not None
            and all(low <= value <= high for value, (low, high) in zip(values, config.joint_limits_rad)))


def display_projection(joints_rad: Any, config: SimTaskArmConfig = CONFIG) -> Optional[Tuple[float, ...]]:
    """Map independent Joint1/Joint2/Joint6 radians to nine display-joint radians."""
    if not independent_in_limits(joints_rad, config): return None
    rotation, left, right = (value / DEG for value in joints_rad)
    alpha3 = right - 3.8
    display_degrees = (rotation - 90, 90 - left, left + alpha3 - 86.11,
                       left - 90, 90 - left, alpha3, 90 - left - alpha3,
                       -3.89 - alpha3, alpha3 + 3.84)
    return tuple(value * DEG for value in display_degrees)


def _ik_geometry(xyz_m: Any, config: SimTaskArmConfig):
    position = _numbers(xyz_m, 3)
    if (not isinstance(config, SimTaskArmConfig) or position is None
            or not all(low <= value <= high for value, (low, high)
                       in zip(position, config.workspace_m))):
        return None
    x, y, z = position
    geometry = config.geometry_m
    if y == 0.0:
        rotation = math.pi / 2
    elif y < 0.0:
        rotation = -math.atan(x / y)
    else:
        rotation = math.pi - math.atan(x / y) if x != 0.0 else math.pi
    sine = math.sin(rotation)
    if abs(sine) < 1e-12:
        return None
    lower = geometry["lower_arm_m"]
    upper = geometry["upper_arm_m"]
    x_in = (x / sine - geometry["radial_offset_m"]) / lower
    z_in = (z + geometry["tool_z_offset_m"] - geometry["base_height_m"]) / lower
    radius = math.hypot(x_in, z_in)
    ratio = upper / lower
    if radius <= 0.0:
        return None
    cosine_right = (radius * radius + ratio * ratio - 1) / (2 * ratio * radius)
    cosine_left = (radius * radius + 1 - ratio * ratio) / (2 * radius)
    cosine_elbow = (1 + ratio * ratio - radius * radius) / (2 * ratio)
    if not all(-1.0 <= value <= 1.0
               for value in (cosine_right, cosine_left, cosine_elbow)):
        return None
    phi = math.atan2(z_in, x_in)
    candidate = (
        rotation,
        math.acos(cosine_left) + phi,
        math.acos(cosine_right) - phi,
    )
    if not independent_in_limits(candidate, config):
        return None
    metrics = ConditioningMetrics(
        math.acos(cosine_elbow),
        radius * lower - abs(upper - lower),
    )
    return candidate, metrics


def inverse_kinematics(xyz_m: Any, config: SimTaskArmConfig = CONFIG) -> Optional[Tuple[float, float, float]]:
    """Upstream inverse_kinematics.hh algebra in SI units; invalid inputs return None."""
    result = _ik_geometry(xyz_m, config)
    return None if result is None else result[0]


def forward_position(joints_rad: Any, config: SimTaskArmConfig = CONFIG) -> Optional[Tuple[float, float, float]]:
    """Invert the same official SwiftPro triangle geometry used by IK."""
    joints = _numbers(joints_rad, 3)
    if not isinstance(config, SimTaskArmConfig) or joints is None or not independent_in_limits(joints, config):
        return None
    rotation, left, right = joints
    geometry = config.geometry_m
    lower = geometry["lower_arm_m"]
    ratio = geometry["upper_arm_m"] / lower
    radius = math.sqrt(1.0 + ratio * ratio + 2.0 * ratio * math.cos(left + right))
    if radius <= 0.0:
        return None
    cosine_beta = (radius * radius + 1.0 - ratio * ratio) / (2.0 * radius)
    if not -1.0 <= cosine_beta <= 1.0:
        return None
    phi = left - math.acos(cosine_beta)
    radial = geometry["radial_offset_m"] + lower * radius * math.cos(phi)
    z = (lower * radius * math.sin(phi) - geometry["tool_z_offset_m"]
         + geometry["base_height_m"])
    position = (radial * math.sin(rotation), -radial * math.cos(rotation), z)
    return position if all(math.isfinite(value) for value in position) else None


def conditioning_metrics(xyz_m: Any, config: SimTaskArmConfig = CONFIG) -> Optional[ConditioningMetrics]:
    """Return generic SIM fold metrics for an otherwise valid IK target."""
    result = _ik_geometry(xyz_m, config)
    return None if result is None else result[1]


def admission_reason(xyz_m: Any, config: SimTaskArmConfig = CONFIG) -> Optional[str]:
    """Return a fail-closed SIM admission reason, or ``None`` when accepted."""
    if not isinstance(config, SimTaskArmConfig):
        return "INVALID_CONFIG"
    metrics = conditioning_metrics(xyz_m, config)
    if metrics is None:
        return "UNREACHABLE"
    if (metrics.elbow_rad < config.thresholds["fold_min_elbow_rad"]
            or metrics.fold_margin_m < config.thresholds["fold_min_margin_m"]):
        return "FOLDED_CONFIGURATION"
    return None


def transform_point(transform: Any, point_m: Any) -> Optional[Tuple[float, float, float]]:
    point = _numbers(point_m, 3)
    if not isinstance(transform, RigidTransform) or point is None or not isinstance(transform.parent_frame, str) or not isinstance(transform.child_frame, str): return None
    translation = _numbers(transform.translation_m, 3)
    if not isinstance(transform.rotation, (tuple, list)) or len(transform.rotation) != 3 or translation is None: return None
    rows = tuple(_numbers(row, 3) for row in transform.rotation)
    if any(row is None for row in rows): return None
    return tuple(sum(row[index] * point[index] for index in range(3)) + translation[row_index] for row_index, row in enumerate(rows))


def camera_project(camera_point_m: Any, fx: Any, fy: Any, cx: Any, cy: Any) -> Optional[Tuple[float, float]]:
    point = _numbers(camera_point_m, 3)
    raw_intrinsics = (fx, fy, cx, cy)
    if any(isinstance(value, bool) for value in raw_intrinsics): return None
    try: intrinsics = tuple(float(value) for value in raw_intrinsics)
    except (TypeError, ValueError): return None
    if point is None or not all(math.isfinite(value) for value in intrinsics) or intrinsics[0] <= 0.0 or intrinsics[1] <= 0.0 or point[2] <= 0.0: return None
    return (intrinsics[0] * point[0] / point[2] + intrinsics[2], intrinsics[1] * point[1] / point[2] + intrinsics[3])
