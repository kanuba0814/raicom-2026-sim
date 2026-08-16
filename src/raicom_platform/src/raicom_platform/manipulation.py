"""Pure, non-actuating manipulation planning contracts.

This module deliberately plans only in explicitly configured station-local frames.
It does not import ROS, transform pixels, or command an arm, gripper, or simulator.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum

from raicom_core.models import GraspEvidence, GraspState
import math
from typing import Protocol, Tuple


class PrimitiveState(str, Enum):
    READY = "READY"
    APPROACHING = "APPROACHING"
    ACTUATING = "ACTUATING"
    VERIFYING = "VERIFYING"
    RETRACTING = "RETRACTING"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


class ManipulationScope(str, Enum):
    """Deployment scope, kept explicit until the Hardware Contract Check."""

    SIM_WORKING_ASSUMPTION = "SIM_WORKING_ASSUMPTION"
    REAL_UNRESOLVED = "REAL_UNRESOLVED"


class ManipulationOutcome(str, Enum):
    """Bounded result of a dry-run plan, never a task-completion assertion."""

    SIM_TESTED = "SIM_TESTED"
    FAILED = "FAILED"
    UNAVAILABLE = "UNAVAILABLE"


@dataclass(frozen=True)
class VerticalMotionProfile:
    approach_height_m: float
    contact_height_m: float
    retract_height_m: float
    maximum_speed_m_s: float

    def __post_init__(self) -> None:
        values = (self.approach_height_m, self.contact_height_m,
                  self.retract_height_m, self.maximum_speed_m_s)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("motion profile must be finite")
        if self.maximum_speed_m_s <= 0.0:
            raise ValueError("maximum speed must be positive")
        if self.approach_height_m <= self.contact_height_m or self.retract_height_m <= self.contact_height_m:
            raise ValueError("approach/retract must remain above contact")


@dataclass(frozen=True)
class WorkspaceBounds:
    """Axis-aligned reachable workspace expressed only in a station-local frame."""

    minimum_x_m: float
    maximum_x_m: float
    minimum_y_m: float
    maximum_y_m: float
    minimum_z_m: float
    maximum_z_m: float

    def __post_init__(self) -> None:
        values = (
            self.minimum_x_m, self.maximum_x_m, self.minimum_y_m,
            self.maximum_y_m, self.minimum_z_m, self.maximum_z_m,
        )
        if not all(math.isfinite(value) for value in values):
            raise ValueError("workspace bounds must be finite")
        if (self.minimum_x_m > self.maximum_x_m or
                self.minimum_y_m > self.maximum_y_m or
                self.minimum_z_m > self.maximum_z_m):
            raise ValueError("workspace minimum must not exceed maximum")

    def contains(self, x_m: float, y_m: float, z_m: float) -> bool:
        return (self.minimum_x_m <= x_m <= self.maximum_x_m and
                self.minimum_y_m <= y_m <= self.maximum_y_m and
                self.minimum_z_m <= z_m <= self.maximum_z_m)


@dataclass(frozen=True)
class StationCalibration:
    """HCC-replaceable calibration status; values are not inferred from pixels."""

    calibrated: bool
    transform_resolved: bool
    heights_resolved: bool
    detail: str = ""

    @property
    def ready(self) -> bool:
        return self.calibrated and self.transform_resolved and self.heights_resolved


class CalibrationProvider(Protocol):
    """Seam for a future HCC-backed calibration source without Mission Core changes."""

    def calibration_for(self, station_id: str) -> StationCalibration: ...


@dataclass(frozen=True)
class StaticCalibrationProvider:
    """Explicit development configuration provider used by the digital twin."""

    calibration: StationCalibration

    def calibration_for(self, station_id: str) -> StationCalibration:
        return self.calibration


@dataclass(frozen=True)
class StationManipulationConfig:
    station_id: str
    local_frame_id: str
    scope: ManipulationScope
    workspace: WorkspaceBounds
    motion: VerticalMotionProfile
    target_max_age_s: float

    def __post_init__(self) -> None:
        if not self.station_id.strip() or not self.local_frame_id.strip():
            raise ValueError("station id and station-local frame must be non-empty")
        if not math.isfinite(self.target_max_age_s) or self.target_max_age_s <= 0.0:
            raise ValueError("target_max_age_s must be finite and positive")


@dataclass(frozen=True)
class ManipulationTarget:
    """Already station-local target observation; no pixel-to-arm conversion occurs here."""

    target_id: str
    station_id: str
    frame_id: str
    x_m: float
    y_m: float
    z_m: float
    observed_at_s: float

    def __post_init__(self) -> None:
        if not self.target_id.strip() or not self.station_id.strip() or not self.frame_id.strip():
            raise ValueError("target and frame identifiers must be non-empty")
        if not all(math.isfinite(value) for value in (self.x_m, self.y_m, self.z_m, self.observed_at_s)):
            raise ValueError("target coordinates and observation time must be finite")


@dataclass(frozen=True)
class ManipulationPlan:
    outcome: ManipulationOutcome
    terminal_state: PrimitiveState
    states: Tuple[PrimitiveState, ...]
    detail: str

    @property
    def task_succeeded(self) -> bool:
        """Always false: a dry-run/SIM plan cannot claim field task completion."""
        return False


class ManipulationPlanner:
    """Fail-closed digital-twin planner with no controller or attachment side effects."""

    _DRY_RUN_STATES = (
        PrimitiveState.APPROACHING,
        PrimitiveState.ACTUATING,
        PrimitiveState.VERIFYING,
        PrimitiveState.RETRACTING,
        PrimitiveState.SUCCEEDED,
    )

    def __init__(self, config: StationManipulationConfig, calibration_provider: CalibrationProvider) -> None:
        self._config = config
        self._calibration_provider = calibration_provider

    def plan(
        self,
        target: ManipulationTarget,
        grasp_evidence: GraspEvidence,
        now_s: float,
        *,
        emergency_stop: bool = False,
    ) -> ManipulationPlan:
        """Return a bounded dry-run outcome after all required preconditions pass."""
        if not math.isfinite(now_s):
            return self._failed("planner time is not finite")
        if emergency_stop:
            return self._failed("E-stop is active")
        if self._config.scope != ManipulationScope.SIM_WORKING_ASSUMPTION:
            return ManipulationPlan(
                ManipulationOutcome.UNAVAILABLE, PrimitiveState.FAILED, (),
                "REAL_UNRESOLVED: manipulation remains unavailable pending HCC",
            )
        calibration = self._calibration_provider.calibration_for(self._config.station_id)
        if not calibration.ready:
            return ManipulationPlan(
                ManipulationOutcome.UNAVAILABLE, PrimitiveState.FAILED, (),
                "station calibration, transform, or heights are unresolved",
            )
        if target.station_id != self._config.station_id or target.frame_id != self._config.local_frame_id:
            return self._failed("target is not expressed in the configured station-local frame")
        age_s = now_s - target.observed_at_s
        if age_s < 0.0 or age_s > self._config.target_max_age_s:
            return self._failed("target observation is stale or from the future")
        planned_heights = (
            target.z_m,
            self._config.motion.approach_height_m,
            self._config.motion.contact_height_m,
            self._config.motion.retract_height_m,
        )
        if not all(self._config.workspace.contains(target.x_m, target.y_m, height)
                   for height in planned_heights):
            return self._failed("target or planned motion is outside the configured reachable workspace")
        if not grasp_evidence.confirms(target.target_id, now_s, self._config.target_max_age_s):
            return self._failed(f"grasp evidence is not fresh confirmed evidence for {target.target_id}")
        return ManipulationPlan(
            ManipulationOutcome.SIM_TESTED,
            PrimitiveState.SUCCEEDED,
            self._DRY_RUN_STATES,
            "SIM dry-run validated; no controller command, gripper action, or attachment was issued",
        )

    @staticmethod
    def _failed(detail: str) -> ManipulationPlan:
        return ManipulationPlan(ManipulationOutcome.FAILED, PrimitiveState.FAILED, (), detail)


@dataclass(frozen=True)
class QrSlotModel:
    rows: int = 2
    columns: int = 2

    def __post_init__(self) -> None:
        if (self.rows, self.columns) != (2, 2):
            raise ValueError("industrial QR placement uses an explicit 2x2 slot model")

    def slot(self, row: int, column: int) -> Tuple[int, int]:
        if not 0 <= row < self.rows or not 0 <= column < self.columns:
            raise ValueError("slot is outside 2x2 model")
        return row, column


@dataclass
class ManipulationPrimitive:
    """Fail-closed primitive state model shared by pick/assembly/turntable tasks."""

    state: PrimitiveState = PrimitiveState.READY

    def begin_approach(self) -> None:
        if self.state != PrimitiveState.READY:
            raise RuntimeError("primitive is not ready")
        self.state = PrimitiveState.APPROACHING

    def contact_reached(self) -> None:
        if self.state != PrimitiveState.APPROACHING:
            raise RuntimeError("contact requires approach")
        self.state = PrimitiveState.ACTUATING

    def actuation_complete(self) -> None:
        if self.state != PrimitiveState.ACTUATING:
            raise RuntimeError("verification requires actuation")
        self.state = PrimitiveState.VERIFYING

    def verify(self, grasp: GraspState) -> None:
        if self.state != PrimitiveState.VERIFYING:
            raise RuntimeError("primitive is not verifying")
        if grasp == GraspState.CONFIRMED:
            self.state = PrimitiveState.RETRACTING
        else:
            self.state = PrimitiveState.FAILED

    def retract_complete(self) -> None:
        if self.state != PrimitiveState.RETRACTING:
            raise RuntimeError("retract completion requires retracting")
        self.state = PrimitiveState.SUCCEEDED
