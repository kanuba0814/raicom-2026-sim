"""ROS-independent SIM-only circle-centre alignment and forward approach."""
from dataclasses import dataclass
from enum import Enum
import math


class DockingPhase(str, Enum):
    ALIGN_CENTER = "ALIGN_CENTER"
    APPROACH_CENTER = "APPROACH_CENTER"
    COMPLETE = "COMPLETE"
    STOP = "STOP"


@dataclass(frozen=True)
class DockingConfig:
    """SIM_WORKING_ASSUMPTION tuning; this is not a physical-robot contract."""
    front_extent_m: float = 0.24
    rear_extent_m: float = 0.24
    side_extent_m: float = 0.20
    lateral_alignment_tolerance_m: float = 0.03
    face_alignment_tolerance_rad: float = 0.05
    target_front_gap_m: float = 0.03
    completion_gap_min_m: float = 0.02
    completion_gap_max_m: float = 0.04
    hard_minimum_gap_m: float = 0.01
    required_alignment_frames: int = 3
    required_complete_frames: int = 3
    maximum_linear_speed_mps: float = 0.03
    maximum_angular_speed_rps: float = 0.08
    linear_gain_per_s: float = 0.30
    angular_gain_per_s: float = 0.50


@dataclass(frozen=True)
class VelocitySuggestion:
    vx_mps: float = 0.0
    vy_mps: float = 0.0
    wz_rps: float = 0.0


@dataclass(frozen=True)
class DockingDecision:
    phase: DockingPhase
    reason: str
    suggestion: VelocitySuggestion
    face_error_rad: float = 0.0
    front_gap_m: float = float("nan")
    stable_frames: int = 0


def front_gap(cx_m, radius_m, front_extent_m):
    """Axial distance from the front envelope to the cylinder outer edge."""
    return cx_m - radius_m - front_extent_m


def rectangle_clearance(cx_m, cy_m, radius_m, front_extent_m, rear_extent_m, side_extent_m):
    """Shortest distance from the rectangular robot envelope to the circle edge."""
    longitudinal_extent = front_extent_m if cx_m >= 0.0 else rear_extent_m
    return math.hypot(max(abs(cx_m) - longitudinal_extent, 0.0),
                      max(abs(cy_m) - side_extent_m, 0.0)) - radius_m


def _finite_observation(cx_m, cy_m, radius_m):
    return (all(isinstance(value, (int, float)) and math.isfinite(value)
                for value in (cx_m, cy_m, radius_m)) and radius_m > 0.0)


class DockingGeometry:
    """Fail-closed state machine. STOP remains latched until acknowledge()."""
    def __init__(self, config=DockingConfig()):
        if not (0.0 <= config.hard_minimum_gap_m < config.completion_gap_min_m <=
                config.target_front_gap_m <= config.completion_gap_max_m):
            raise ValueError("invalid SIM_WORKING_ASSUMPTION gap config")
        self.config = config
        self.phase = DockingPhase.ALIGN_CENTER
        self._alignment_frames = 0
        self._complete_frames = 0

    def acknowledge(self):
        self.phase = DockingPhase.ALIGN_CENTER
        self._alignment_frames = 0
        self._complete_frames = 0

    reset = acknowledge

    def _stop(self, reason, face_error=0.0, gap=float("nan")):
        self.phase = DockingPhase.STOP
        self._alignment_frames = 0
        self._complete_frames = 0
        return DockingDecision(self.phase, reason, VelocitySuggestion(), face_error, gap)

    def decide(self, cx_m, cy_m, radius_m, observation_stable=True,
               clearance_uncertainty_m=0.01):
        if self.phase == DockingPhase.STOP:
            return DockingDecision(DockingPhase.STOP, "STOP_LATCHED", VelocitySuggestion())
        if not observation_stable or not _finite_observation(cx_m, cy_m, radius_m):
            return self._stop("OBSERVATION_INVALID_OR_UNSTABLE")
        gap = front_gap(cx_m, radius_m, self.config.front_extent_m)
        face_error = math.atan2(cy_m, cx_m)
        if (not isinstance(clearance_uncertainty_m, (int, float)) or
                not math.isfinite(clearance_uncertainty_m) or clearance_uncertainty_m < 0.01):
            return self._stop("GAP_UNCERTAINTY_INVALID")
        envelope_gap = rectangle_clearance(
            cx_m, cy_m, radius_m, self.config.front_extent_m,
            self.config.rear_extent_m, self.config.side_extent_m)
        if envelope_gap - clearance_uncertainty_m < self.config.hard_minimum_gap_m:
            return self._stop("HARD_MINIMUM_ENVELOPE_GAP_VIOLATION", face_error, gap)
        if self.phase == DockingPhase.ALIGN_CENTER:
            if cx_m <= 0.0:
                return self._stop("TARGET_NOT_IN_FORWARD_HEMISPHERE", face_error, gap)
            if abs(face_error) > self.config.face_alignment_tolerance_rad:
                self._alignment_frames = 0
                wz = max(-self.config.maximum_angular_speed_rps,
                         min(self.config.maximum_angular_speed_rps,
                             self.config.angular_gain_per_s * face_error))
                return DockingDecision(self.phase, "ALIGN_CENTER", VelocitySuggestion(wz_rps=wz), face_error, gap)
            self._alignment_frames += 1
            if self._alignment_frames < self.config.required_alignment_frames:
                return DockingDecision(self.phase, "WAITING_FOR_ALIGNMENT_STABILITY",
                                       VelocitySuggestion(), face_error, gap, self._alignment_frames)
            self.phase = DockingPhase.APPROACH_CENTER
        if self.phase == DockingPhase.APPROACH_CENTER:
            # Do not translate while turning: renewed angular error first
            # produces zero, then returns to ALIGN on the next tick.
            if abs(face_error) > self.config.face_alignment_tolerance_rad:
                self.phase = DockingPhase.ALIGN_CENTER
                self._alignment_frames = 0
                return DockingDecision(self.phase, "REALIGN_CENTER", VelocitySuggestion(), face_error, gap)
            if gap - clearance_uncertainty_m < self.config.hard_minimum_gap_m:
                return self._stop("HARD_MINIMUM_FRONT_GAP_VIOLATION", face_error, gap)
            if gap > self.config.completion_gap_max_m:
                self._complete_frames = 0
                vx = min(self.config.maximum_linear_speed_mps,
                         self.config.linear_gain_per_s * (gap - self.config.target_front_gap_m))
                return DockingDecision(self.phase, "APPROACH_CENTER", VelocitySuggestion(vx_mps=vx), face_error, gap)
            if self.config.completion_gap_min_m <= gap <= self.config.completion_gap_max_m:
                self._complete_frames += 1
                if self._complete_frames >= self.config.required_complete_frames:
                    self.phase = DockingPhase.COMPLETE
                    return DockingDecision(self.phase, "STABLE_FRONT_GAP_0.03M_PLUS_MINUS_0.01M",
                                           VelocitySuggestion(), face_error, gap, self._complete_frames)
                return DockingDecision(self.phase, "WAITING_FOR_COMPLETE_STABILITY",
                                       VelocitySuggestion(), face_error, gap, self._complete_frames)
            self._complete_frames = 0
            return self._stop("FRONT_GAP_OVERSHOOT", face_error, gap)
        return DockingDecision(DockingPhase.COMPLETE, "COMPLETE", VelocitySuggestion(), face_error, gap,
                               self._complete_frames)
