"""Fail-closed, SIM-only SwiftPro motion adapter core.

There is no ROS transport or default execution API here.  An explicitly chosen
SIM lane supplies fresh observations, a clock, and the nine controller-topic
publisher.  It never controls end effectors, hardware transports, the base, or
Gazebo ground truth.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Callable, Optional, Tuple

from .config import CONFIG, SimTaskArmConfig
from .kinematics import DEG, display_projection, independent_in_limits

# JointState feedback uses the simulator's capitalized URDF joint names.  The
# controller endpoints are separately defined lower-case names.
FEEDBACK_JOINT_NAMES = tuple("Joint%d" % number for number in range(1, 10))
CONTROLLER_JOINT_NAMES = tuple("joint%d" % number for number in range(1, 10))
STEP_SECONDS = 0.01
MAX_VELOCITY_RAD_S = 55.0 * DEG
MIN_MOVE_SECONDS = 0.4


class MotionStatus(str, Enum):
    READY = "READY"; INVALID = "INVALID"; STALE = "STALE"; TIMEOUT = "TIMEOUT"
    CANCELLED = "CANCELLED"; FAILED = "FAILED"; SETTLED = "SETTLED"


@dataclass(frozen=True)
class JointObservation:
    names: Any
    positions_rad: Any
    velocities_rad_s: Any
    observed_at_s: Any


@dataclass(frozen=True)
class MotionPlan:
    start_independent_rad: Tuple[float, float, float]
    target_independent_rad: Tuple[float, float, float]
    duration_s: float
    commands: Tuple[Tuple[float, ...], ...]


@dataclass(frozen=True)
class MotionResult:
    status: MotionStatus
    reason: str
    plan: Optional[MotionPlan] = None


def _finite_tuple(value: Any, size: int) -> Optional[Tuple[float, ...]]:
    if not isinstance(value, (tuple, list)) or len(value) != size or any(isinstance(item, bool) for item in value):
        return None
    try: result = tuple(float(item) for item in value)
    except (TypeError, ValueError): return None
    return result if all(math.isfinite(item) for item in result) else None


def _validated_observation_vectors(observation: Any) -> tuple[Optional[dict[str, float]], Optional[dict[str, float]], str]:
    if not isinstance(observation, JointObservation):
        return None, None, "MISSING_JOINT_STATES"
    names = observation.names
    if not isinstance(names, (tuple, list)) or not names or any(not isinstance(name, str) for name in names):
        return None, None, "INVALID_JOINT_NAMES"
    positions = _finite_tuple(observation.positions_rad, len(names))
    velocities = _finite_tuple(observation.velocities_rad_s, len(names))
    if positions is None or velocities is None:
        return None, None, "INVALID_OBSERVATION"
    if len(set(names)) != len(names) or not set(FEEDBACK_JOINT_NAMES).issubset(names):
        return None, None, "AMBIGUOUS_JOINT_NAMES"
    return dict(zip(names, positions)), dict(zip(names, velocities)), ""


def _observation_independent(observation: Any, now_s: Any, config: SimTaskArmConfig) -> tuple[Optional[Tuple[float, float, float]], str]:
    by_name, _, vector_reason = _validated_observation_vectors(observation)
    if vector_reason:
        return None, vector_reason
    try: stamp, now = float(observation.observed_at_s), float(now_s)
    except (TypeError, ValueError): return None, "INVALID_OBSERVATION"
    if not math.isfinite(stamp) or not math.isfinite(now): return None, "INVALID_OBSERVATION"
    if stamp > now or now - stamp > config.thresholds["observation_max_age_s"]: return None, "STALE_JOINT_STATES"
    assert by_name is not None
    independent = (by_name["Joint1"] + 90.0 * DEG, 90.0 * DEG - by_name["Joint2"], by_name["Joint6"] + 3.8 * DEG)
    if not independent_in_limits(independent, config): return None, "JOINT_LIMIT"
    return independent, ""


def cosine_progress(fraction: Any) -> Optional[float]:
    try: value = float(fraction)
    except (TypeError, ValueError): return None
    if not math.isfinite(value) or not 0.0 <= value <= 1.0: return None
    return 0.5 * (1.0 - math.cos(math.pi * value))


def plan_motion(target_independent_rad: Any, observation: Any, now_s: Any, config: SimTaskArmConfig = CONFIG) -> MotionResult:
    if not isinstance(config, SimTaskArmConfig): return MotionResult(MotionStatus.INVALID, "INVALID_CONFIG")
    start, reason = _observation_independent(observation, now_s, config)
    if start is None: return MotionResult(MotionStatus.STALE if reason == "STALE_JOINT_STATES" else MotionStatus.INVALID, reason)
    target = _finite_tuple(target_independent_rad, 3)
    if target is None or not independent_in_limits(target, config): return MotionResult(MotionStatus.INVALID, "JOINT_LIMIT")
    positions, _, vector_reason = _validated_observation_vectors(observation)
    if vector_reason or positions is None:
        return MotionResult(MotionStatus.INVALID, vector_reason or "INVALID_OBSERVATION")
    start_display = tuple(positions[name] for name in FEEDBACK_JOINT_NAMES)
    target_display = display_projection(target, config)
    assert target_display is not None
    # A cosine trajectory's peak derivative is pi/(2*duration).  Use all nine
    # projected channels; do not cap duration, since that would violate 55 deg/s.
    duration = max(MIN_MOVE_SECONDS, math.pi * max(abs(a - b) for a, b in zip(start_display, target_display)) / (2.0 * MAX_VELOCITY_RAD_S))
    steps = max(1, math.ceil(duration / STEP_SECONDS))
    commands = []
    for step in range(1, steps + 1):
        eased = cosine_progress(step / steps)
        commands.append(tuple(
            start_value + eased * (target_value - start_value)
            for start_value, target_value in zip(start_display, target_display)
        ))
    return MotionResult(MotionStatus.READY, "READY", MotionPlan(start, target, duration, tuple(commands)))


class SwiftproSimMotionAdapter:
    """Explicitly selected adapter; injected callbacks keep it transport-neutral."""
    def __init__(self, config: SimTaskArmConfig = CONFIG) -> None: self.config = config
    def plan(self, target_independent_rad: Any, observation: Any, now_s: Any) -> MotionResult:
        return plan_motion(target_independent_rad, observation, now_s, self.config)

    def execute(self, target_independent_rad: Any, observation: Any, deadline_s: Any,
                publish: Callable[[str, float], None], sleep: Callable[[float], None],
                observe: Callable[[], Any], cancelled: Callable[[], bool], now: Callable[[], float]) -> MotionResult:
        try: start_now, deadline = float(now()), float(deadline_s)
        except (TypeError, ValueError): return MotionResult(MotionStatus.INVALID, "INVALID_DEADLINE")
        if not math.isfinite(start_now) or not math.isfinite(deadline): return MotionResult(MotionStatus.INVALID, "INVALID_DEADLINE")
        planned = self.plan(target_independent_rad, observation, start_now)
        if planned.status is not MotionStatus.READY: return planned
        plan = planned.plan; assert plan is not None
        # Reserve the actual discretized trajectory sleeps and the complete
        # measured settle window.  Never begin a motion that cannot finish its
        # acceptance gate within the authoritative sim/ROS deadline.
        required_time = (len(plan.commands) * STEP_SECONDS
                         + self.config.thresholds["settle_window_s"]
                         + STEP_SECONDS)
        if deadline - start_now < required_time:
            return MotionResult(MotionStatus.TIMEOUT, "DEADLINE_TOO_SHORT", plan)

        def hold(failure_status: MotionStatus, reason: str) -> MotionResult:
            try: hold_now, measured = float(now()), observe()
            except Exception: return MotionResult(MotionStatus.FAILED, "UNSAFE_HOLD", plan)
            _, invalid = _observation_independent(measured, hold_now, self.config)
            if invalid: return MotionResult(MotionStatus.FAILED, "UNSAFE_HOLD", plan)
            positions, _, vector_reason = _validated_observation_vectors(measured)
            if vector_reason or positions is None:
                return MotionResult(MotionStatus.FAILED, "UNSAFE_HOLD", plan)
            try:
                for feedback, controller in zip(FEEDBACK_JOINT_NAMES, CONTROLLER_JOINT_NAMES):
                    publish("/swiftpro/%s_position_controller/command" % controller, positions[feedback])
            except Exception: return MotionResult(MotionStatus.FAILED, "UNSAFE_HOLD", plan)
            return MotionResult(failure_status, reason, plan)

        for command in plan.commands:
            try: current = float(now())
            except Exception: return hold(MotionStatus.FAILED, "CLOCK_FAILED")
            if not math.isfinite(current): return hold(MotionStatus.FAILED, "CLOCK_FAILED")
            try:
                if cancelled(): return hold(MotionStatus.CANCELLED, "CANCELLED")
            except Exception: return hold(MotionStatus.FAILED, "CANCEL_CHECK_FAILED")
            if current >= deadline: return hold(MotionStatus.TIMEOUT, "TRAJECTORY_TIMEOUT")
            try:
                # Cancellation is checked before every individual publication;
                # a mid-batch cancellation cannot emit the remaining targets.
                for controller, position in zip(CONTROLLER_JOINT_NAMES, command):
                    if cancelled(): return hold(MotionStatus.CANCELLED, "CANCELLED")
                    publish_now = float(now())
                    if not math.isfinite(publish_now):
                        return hold(MotionStatus.FAILED, "CLOCK_FAILED")
                    if publish_now >= deadline:
                        return hold(MotionStatus.TIMEOUT, "TRAJECTORY_TIMEOUT")
                    publish("/swiftpro/%s_position_controller/command" % controller, position)
                before_sleep = current
                sleep(STEP_SECONDS)
                if float(now()) <= before_sleep: return hold(MotionStatus.FAILED, "CLOCK_NO_PROGRESS")
            except Exception: return hold(MotionStatus.FAILED, "COMMAND_TRANSPORT_FAILED")

        window_start = None
        previous_stamp = None
        while True:
            try: current = float(now())
            except Exception: return hold(MotionStatus.FAILED, "CLOCK_FAILED")
            if not math.isfinite(current): return hold(MotionStatus.FAILED, "CLOCK_FAILED")
            if current >= deadline: return hold(MotionStatus.TIMEOUT, "SETTLE_TIMEOUT")
            try:
                if cancelled(): return hold(MotionStatus.CANCELLED, "CANCELLED")
                measured = observe()
            except Exception: return hold(MotionStatus.FAILED, "OBSERVATION_FAILED")
            independent, reason = _observation_independent(measured, current, self.config)
            if independent is None: return hold(MotionStatus.STALE if reason == "STALE_JOINT_STATES" else MotionStatus.FAILED, reason)
            stamp = float(measured.observed_at_s)
            positions, velocities, vector_reason = _validated_observation_vectors(measured)
            target_display = display_projection(plan.target_independent_rad, self.config)
            assert not vector_reason and positions is not None and velocities is not None and target_display is not None
            settled = max(abs(positions[name] - target) for name, target in zip(FEEDBACK_JOINT_NAMES, target_display)) <= self.config.thresholds["settle_position_error_rad"] and max(abs(velocities[name]) for name in FEEDBACK_JOINT_NAMES) < self.config.thresholds["settle_velocity_rad_s"]
            if previous_stamp is not None and stamp <= previous_stamp: return hold(MotionStatus.FAILED, "NONINCREASING_SETTLE_STAMP")
            previous_stamp = stamp
            window_start = stamp if not settled else (stamp if window_start is None else window_start)
            if settled and stamp - window_start >= self.config.thresholds["settle_window_s"]: return MotionResult(MotionStatus.SETTLED, "SETTLED", plan)
            if not settled: window_start = None
            try:
                before_sleep = current; sleep(STEP_SECONDS)
                if float(now()) <= before_sleep: return hold(MotionStatus.FAILED, "CLOCK_NO_PROGRESS")
            except Exception: return hold(MotionStatus.FAILED, "WAIT_FAILED")
