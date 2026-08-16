import math
import random

from raicom_round_table_navigation.circle_detector import CircleDetector, DetectorConfig


def scan_for_circles(circles, angle_min=-1.2, angle_increment=0.005, count=481, noise_stddev=0.0):
    random_generator = random.Random(11)
    ranges = []
    for index in range(count):
        angle = angle_min + index * angle_increment
        cos_angle, sin_angle = math.cos(angle), math.sin(angle)
        intersections = []
        for center_x, center_y, radius in circles:
            projection = center_x * cos_angle + center_y * sin_angle
            discriminant = projection * projection - (center_x * center_x + center_y * center_y - radius * radius)
            if discriminant >= 0.0 and projection > 0.0:
                intersections.append(projection - math.sqrt(discriminant))
        ranges.append(min(intersections) + random_generator.gauss(0.0, noise_stddev) if intersections else float("inf"))
    return ranges, angle_min, angle_increment


def detector(**overrides):
    config = DetectorConfig(required_stable_frames=1, min_angle_span_rad=0.15, **overrides)
    return CircleDetector(config)


def test_noisy_correct_circle_arc_is_fitted():
    ranges, angle_min, angle_increment = scan_for_circles([(2.0, 0.0, 0.25)], noise_stddev=0.01)
    result = detector(max_residual_m=0.02).detect(ranges, angle_min, angle_increment)
    assert result.valid
    assert abs(result.observation.center_x_m - 2.0) < 0.04
    assert abs(result.observation.center_y_m) < 0.04
    assert abs(result.observation.radius_m - 0.25) < 0.04


def test_straight_wall_is_rejected():
    angle_min, angle_increment = -0.5, 0.005
    ranges = [2.0 / math.cos(angle_min + index * angle_increment) for index in range(201)]
    result = detector().detect(ranges, angle_min, angle_increment)
    assert not result.valid
    assert result.reason == "NO_VALID_CIRCLE"


def test_invalid_and_nan_data_are_rejected():
    result = detector().detect([float("inf"), float("nan"), None, 0.01], -0.1, 0.01)
    assert not result.valid
    assert result.reason == "NO_VALID_CIRCLE"


def test_two_equally_good_circles_are_ambiguous():
    ranges, angle_min, angle_increment = scan_for_circles([(2.0, -0.65, 0.25), (2.0, 0.65, 0.25)])
    result = detector().detect(ranges, angle_min, angle_increment)
    assert not result.valid
    assert result.reason == "AMBIGUOUS_CIRCLES"


def test_exact_best_and_slightly_noisy_second_circle_are_ambiguous_with_absolute_epsilon():
    ranges, angle_min, angle_increment = scan_for_circles([(2.0, -0.65, 0.25), (2.0, 0.65, 0.25)])
    # Keep the lower circle mathematically exact, but perturb only the upper
    # circle by much less than the configured absolute ambiguity margin.
    for index, distance in enumerate(ranges):
        if angle_min + index * angle_increment > 0.15 and math.isfinite(distance):
            ranges[index] = distance + 0.0002 * math.sin(index)
    result = detector(ambiguity_residual_epsilon_m=0.001).detect(ranges, angle_min, angle_increment)
    assert not result.valid
    assert result.reason == "AMBIGUOUS_CIRCLES"


def test_observation_requires_consecutive_stable_frames():
    ranges, angle_min, angle_increment = scan_for_circles([(2.0, 0.0, 0.25)], noise_stddev=0.002)
    stable_detector = CircleDetector(DetectorConfig(required_stable_frames=3, min_angle_span_rad=0.15))
    first = stable_detector.detect(ranges, angle_min, angle_increment)
    second = stable_detector.detect(ranges, angle_min, angle_increment)
    third = stable_detector.detect(ranges, angle_min, angle_increment)
    assert first.reason == "WAITING_FOR_STABILITY"
    assert second.reason == "WAITING_FOR_STABILITY"
    assert third.valid
    assert third.stable_frames == 3
