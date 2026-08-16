"""ROS-independent LaserScan circle-cluster fitting for simulation perception."""
from dataclasses import dataclass
import math


@dataclass(frozen=True)
class DetectorConfig:
    min_range_m: float = 0.05
    max_range_m: float = 4.0
    max_cluster_gap_m: float = 0.06
    min_points: int = 8
    min_radius_m: float = 0.12
    max_radius_m: float = 0.40
    max_residual_m: float = 0.015
    min_angle_span_rad: float = 0.20
    required_stable_frames: int = 3
    max_center_delta_m: float = 0.03
    max_radius_delta_m: float = 0.02
    ambiguity_score_ratio: float = 1.20
    ambiguity_residual_epsilon_m: float = 0.001


@dataclass(frozen=True)
class CircleObservation:
    center_x_m: float
    center_y_m: float
    radius_m: float
    residual_m: float
    point_count: int
    angle_span_rad: float


@dataclass(frozen=True)
class DetectionResult:
    valid: bool
    reason: str
    observation: object = None
    stable_frames: int = 0


def _solve_3x3(matrix, vector):
    """Solve a small linear system using pivoted Gaussian elimination."""
    augmented = [list(row) + [value] for row, value in zip(matrix, vector)]
    for col in range(3):
        pivot = max(range(col, 3), key=lambda row: abs(augmented[row][col]))
        if abs(augmented[pivot][col]) < 1e-12:
            return None
        augmented[col], augmented[pivot] = augmented[pivot], augmented[col]
        divisor = augmented[col][col]
        augmented[col] = [value / divisor for value in augmented[col]]
        for row in range(3):
            if row == col:
                continue
            factor = augmented[row][col]
            augmented[row] = [value - factor * base for value, base in zip(augmented[row], augmented[col])]
    return [augmented[row][3] for row in range(3)]


def fit_circle(points):
    """Return (x, y, radius, rms_residual), or None for a degenerate cluster."""
    if len(points) < 3:
        return None
    sums = [0.0] * 9
    for x_value, y_value in points:
        squared = x_value * x_value + y_value * y_value
        sums[0] += x_value * x_value
        sums[1] += x_value * y_value
        sums[2] += x_value
        sums[3] += y_value * y_value
        sums[4] += y_value
        sums[5] += squared
        sums[6] += x_value * squared
        sums[7] += y_value * squared
        sums[8] += 1.0
    solution = _solve_3x3(
        ((sums[0], sums[1], sums[2]), (sums[1], sums[3], sums[4]), (sums[2], sums[4], sums[8])),
        (-sums[6], -sums[7], -sums[5]),
    )
    if solution is None:
        return None
    d_value, e_value, f_value = solution
    center_x, center_y = -d_value / 2.0, -e_value / 2.0
    radius_squared = center_x * center_x + center_y * center_y - f_value
    if radius_squared <= 0.0 or not math.isfinite(radius_squared):
        return None
    radius = math.sqrt(radius_squared)
    residual = math.sqrt(sum((math.hypot(x_value - center_x, y_value - center_y) - radius) ** 2 for x_value, y_value in points) / len(points))
    if not all(math.isfinite(value) for value in (center_x, center_y, radius, residual)):
        return None
    return center_x, center_y, radius, residual


class CircleDetector:
    """Detect one unambiguous, temporally stable circle; never controls a robot."""
    def __init__(self, config):
        self.config = config
        self._previous = None
        self._stable_frames = 0

    def reset(self):
        self._previous = None
        self._stable_frames = 0

    def detect(self, ranges, angle_min, angle_increment):
        if ranges is None or not math.isfinite(angle_min) or not math.isfinite(angle_increment) or angle_increment == 0.0:
            self.reset()
            return DetectionResult(False, "SCAN_MALFORMED")
        clusters = []
        active = []
        previous_point = None
        for index, distance in enumerate(ranges):
            if not isinstance(distance, (int, float)) or not math.isfinite(distance) or distance < self.config.min_range_m or distance > self.config.max_range_m:
                if active:
                    clusters.append(active)
                    active = []
                previous_point = None
                continue
            angle = angle_min + index * angle_increment
            if not math.isfinite(angle):
                self.reset()
                return DetectionResult(False, "SCAN_MALFORMED")
            point = (distance * math.cos(angle), distance * math.sin(angle), angle)
            if previous_point is not None and math.hypot(point[0] - previous_point[0], point[1] - previous_point[1]) > self.config.max_cluster_gap_m:
                clusters.append(active)
                active = []
            active.append(point)
            previous_point = point
        if active:
            clusters.append(active)
        candidates = []
        for cluster in clusters:
            if len(cluster) < self.config.min_points:
                continue
            points = [(point[0], point[1]) for point in cluster]
            fitted = fit_circle(points)
            if fitted is None:
                continue
            center_x, center_y, radius, residual = fitted
            angle_span = abs(cluster[-1][2] - cluster[0][2])
            if (self.config.min_radius_m <= radius <= self.config.max_radius_m and
                    residual <= self.config.max_residual_m and angle_span >= self.config.min_angle_span_rad):
                candidates.append(CircleObservation(center_x, center_y, radius, residual, len(points), angle_span))
        if not candidates:
            self.reset()
            return DetectionResult(False, "NO_VALID_CIRCLE")
        candidates.sort(key=lambda item: item.residual_m)
        if len(candidates) > 1:
            best_residual = candidates[0].residual_m
            second_residual = candidates[1].residual_m
            # The absolute residual-difference epsilon removes the
            # zero-residual multiplication ambiguity: a mathematically exact
            # first fit must not make a nearly exact second candidate unique.
            ambiguity_limit = max(
                best_residual * self.config.ambiguity_score_ratio,
                best_residual + self.config.ambiguity_residual_epsilon_m,
            )
            if second_residual <= ambiguity_limit:
                self.reset()
                return DetectionResult(False, "AMBIGUOUS_CIRCLES")
        selected = candidates[0]
        if self._previous is not None and math.hypot(selected.center_x_m - self._previous.center_x_m, selected.center_y_m - self._previous.center_y_m) <= self.config.max_center_delta_m and abs(selected.radius_m - self._previous.radius_m) <= self.config.max_radius_delta_m:
            self._stable_frames += 1
        else:
            self._stable_frames = 1
        self._previous = selected
        if self._stable_frames < self.config.required_stable_frames:
            return DetectionResult(False, "WAITING_FOR_STABILITY", selected, self._stable_frames)
        return DetectionResult(True, "VALID", selected, self._stable_frames)
