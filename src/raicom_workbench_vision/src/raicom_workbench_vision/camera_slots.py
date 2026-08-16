"""Pure camera-only workbench-circle and marker-plane geometry helpers."""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence, Tuple

import numpy as np

Vec3 = Tuple[float, float, float]


@dataclass(frozen=True)
class Circle:
    pixel: Tuple[float, float]
    radius: float


@dataclass(frozen=True)
class MarkerPlane:
    origin: Vec3
    left: Vec3
    down: Vec3
    normal: Vec3


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(value) for value in values)


def _dot(a: Vec3, b: Vec3) -> float:
    return sum(x * y for x, y in zip(a, b))


def _cross(a: Vec3, b: Vec3) -> Vec3:
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _normalize(vector: Vec3) -> Optional[Vec3]:
    magnitude = math.sqrt(_dot(vector, vector))
    if not math.isfinite(magnitude) or magnitude <= 1e-9:
        return None
    return tuple(value / magnitude for value in vector)


def quaternion_axes(quaternion: Sequence[float]) -> Optional[Tuple[Vec3, Vec3, Vec3]]:
    """Return marker-local left, down and outward-normal axes in camera coordinates."""
    if len(quaternion) != 4 or not _finite(quaternion):
        return None
    x, y, z, w = quaternion
    magnitude = math.sqrt(x * x + y * y + z * z + w * w)
    if magnitude <= 1e-9:
        return None
    x, y, z, w = (value / magnitude for value in (x, y, z, w))
    left = (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y + z * w), 2.0 * (x * z - y * w))
    down = (2.0 * (x * y - z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z + x * w))
    normal = _cross(left, down)
    left, down, normal = _normalize(left), _normalize(down), _normalize(normal)
    return None if left is None or down is None or normal is None else (left, down, normal)


def marker_plane(marker_poses: Sequence[Tuple[Vec3, Sequence[float]]]) -> Optional[MarkerPlane]:
    """Build a plane from IDs 100/101, requiring mutually consistent marker normals."""
    if len(marker_poses) != 2:
        return None
    positions = [item[0] for item in marker_poses]
    axes = [quaternion_axes(item[1]) for item in marker_poses]
    if any(not _finite(position) for position in positions) or any(item is None for item in axes):
        return None
    first, second = axes  # type: ignore[misc]
    if _dot(first[2], second[2]) < math.cos(math.radians(12.0)):
        return None
    normal = _normalize(tuple(first[2][i] + second[2][i] for i in range(3)))
    if normal is None:
        return None
    origin = tuple(sum(position[i] for position in positions) / 2.0 for i in range(3))
    # Workbench outward normal must point toward the observing camera at the
    # camera-frame origin; placement clearance then moves away from the table.
    if _dot(normal, tuple(-value for value in origin)) < 0.0:
        normal = tuple(-value for value in normal)
    # Marker IDs are semantic: 100 -> 101 defines table-down. Project the
    # measured baseline into the plane, then derive a right-handed left axis.
    baseline = tuple(positions[1][i] - positions[0][i] for i in range(3))
    baseline_length = math.sqrt(_dot(baseline, baseline))
    if baseline_length <= 1e-9 or abs(_dot(baseline, normal)) / baseline_length > 0.08:
        return None
    down = _normalize(tuple(
        baseline[i] - _dot(baseline, normal) * normal[i] for i in range(3)
    ))
    if down is None:
        return None
    left = _normalize(_cross(down, normal))
    if left is None:
        return None
    return MarkerPlane(origin, left, down, normal)


def detect_number_circles(image: np.ndarray) -> Tuple[Circle, ...]:
    """Find painted number circles, rejecting green annuli without number recognition."""
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        return ()
    import cv2
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    blurred = cv2.medianBlur(gray, 5)
    raw = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=45,
                           param1=100, param2=26, minRadius=12, maxRadius=28)
    if raw is None:
        return ()
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    circles = []  # green-annulus check: number rings must be surrounded by panel green.
    for x, y, radius in np.round(raw[0]).astype(int):
        if radius <= 0 or x - radius < 0 or y - radius < 0 or x + radius >= image.shape[1] or y + radius >= image.shape[0]:
            continue
        mask = np.zeros(gray.shape, dtype=np.uint8)
        cv2.circle(mask, (x, y), max(2, int(radius * 1.70)),
                   max(2, int(radius * 0.25)), -1)
        annulus = hsv[mask > 0]
        if not len(annulus):
            continue
        green = ((annulus[:, 0] >= 35) & (annulus[:, 0] <= 90)
                 & (annulus[:, 1] >= 70) & (annulus[:, 2] >= 35)).mean()
        if green >= 0.35:
            circles.append(Circle((float(x), float(y)), float(radius)))
    # Hough occasionally returns neighboring radii for one contour; retain only one center.
    unique = []
    for candidate in sorted(circles, key=lambda item: -item.radius):
        if all(math.dist(candidate.pixel, accepted.pixel) > max(candidate.radius, accepted.radius) * 0.65 for accepted in unique):
            unique.append(candidate)
    return tuple(unique)


def intersect_pixel_ray(pixel: Sequence[float], camera_matrix: Sequence[float], plane: MarkerPlane) -> Optional[Vec3]:
    """Intersect a calibrated optical ray (camera origin) with the observed marker plane."""
    if len(pixel) != 2 or len(camera_matrix) != 9 or not _finite(tuple(pixel) + tuple(camera_matrix)):
        return None
    fx, fy, cx, cy = camera_matrix[0], camera_matrix[4], camera_matrix[2], camera_matrix[5]
    if fx <= 0.0 or fy <= 0.0:
        return None
    direction = _normalize(((pixel[0] - cx) / fx, (pixel[1] - cy) / fy, 1.0))
    if direction is None:
        return None
    denominator = _dot(direction, plane.normal)
    if abs(denominator) <= 1e-9:
        return None
    scale = _dot(plane.origin, plane.normal) / denominator
    if not math.isfinite(scale) or scale <= 1e-6:
        return None
    return tuple(scale * value for value in direction)


def quaternion_from_plane(plane: MarkerPlane) -> Tuple[float, float, float, float]:
    """Quaternion whose local x/y/z axes are the marker-local left/down/normal axes."""
    matrix = np.array(((plane.left[0], plane.down[0], plane.normal[0]),
                       (plane.left[1], plane.down[1], plane.normal[1]),
                       (plane.left[2], plane.down[2], plane.normal[2])), dtype=float)
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = ((matrix[2, 1] - matrix[1, 2]) / scale,
                      (matrix[0, 2] - matrix[2, 0]) / scale,
                      (matrix[1, 0] - matrix[0, 1]) / scale, 0.25 * scale)
    else:
        index = int(np.argmax(np.diag(matrix)))
        next_index, last_index = (index + 1) % 3, (index + 2) % 3
        scale = math.sqrt(1.0 + matrix[index, index] - matrix[next_index, next_index] - matrix[last_index, last_index]) * 2.0
        values = [0.0, 0.0, 0.0]
        values[index] = 0.25 * scale
        values[next_index] = (matrix[next_index, index] + matrix[index, next_index]) / scale
        values[last_index] = (matrix[last_index, index] + matrix[index, last_index]) / scale
        quaternion = (values[0], values[1], values[2], (matrix[last_index, next_index] - matrix[next_index, last_index]) / scale)
    return tuple(float(value) for value in quaternion)


def complete_three_corner_grid(points: Sequence[Vec3], plane: MarkerPlane) -> Optional[Tuple[Vec3, Vec3, Vec3, Vec3]]:
    """Complete one camera-occluded corner from three observed rectangle corners."""
    if len(points) != 3 or any(not _finite(point) for point in points):
        return None
    local = [(_dot(tuple(point[i] - plane.origin[i] for i in range(3)), plane.left),
              _dot(tuple(point[i] - plane.origin[i] for i in range(3)), plane.down), point)
             for point in points]
    candidates = []
    for corner_index, corner in enumerate(local):
        others = [item for index, item in enumerate(local) if index != corner_index]
        first = (others[0][0] - corner[0], others[0][1] - corner[1])
        second = (others[1][0] - corner[0], others[1][1] - corner[1])
        first_length, second_length = math.hypot(*first), math.hypot(*second)
        if (not 0.03 <= first_length <= 0.12
                or not 0.03 <= second_length <= 0.12
                or abs(first_length - second_length) > 0.015
                or abs(first[0] * second[0] + first[1] * second[1])
                > 0.20 * first_length * second_length):
            continue
        inferred = tuple(others[0][2][axis] + others[1][2][axis] - corner[2][axis]
                         for axis in range(3))
        candidates.append(inferred)
    if len(candidates) != 1:
        return None
    return assign_slot_ids(tuple(points) + (candidates[0],), plane)


def assign_slot_ids(points: Sequence[Vec3], plane: MarkerPlane) -> Optional[Tuple[Vec3, Vec3, Vec3, Vec3]]:
    """Assign IDs 1/3 over 2/4 using marker-local left/down coordinates."""
    if len(points) != 4 or any(not _finite(point) for point in points):
        return None
    local = [(_dot(tuple(point[i] - plane.origin[i] for i in range(3)), plane.left),
              _dot(tuple(point[i] - plane.origin[i] for i in range(3)), plane.down), point) for point in points]
    # With Reinovo's marker convention, smaller local-left values are the
    # image/table left column (IDs 1/2), and larger local-down values are the
    # upper row (IDs 1/3). Derive positions from observed geometry only.
    ordered = sorted(local, key=lambda item: item[0])
    left, right = ordered[:2], ordered[2:]
    if max(item[0] for item in left) >= min(item[0] for item in right):
        return None
    left = sorted(left, key=lambda item: item[1], reverse=True)
    right = sorted(right, key=lambda item: item[1], reverse=True)
    column_gap = min(item[0] for item in right) - max(item[0] for item in left)
    left_row_gap = left[0][1] - left[1][1]
    right_row_gap = right[0][1] - right[1][1]
    if (not 0.03 <= column_gap <= 0.12
            or not 0.03 <= left_row_gap <= 0.12
            or not 0.03 <= right_row_gap <= 0.12
            or abs(left_row_gap - right_row_gap) > 0.015
            or abs(left[0][1] - right[0][1]) > 0.015
            or abs(left[1][1] - right[1][1]) > 0.015):
        return None
    return (left[0][2], left[1][2], right[0][2], right[1][2])
