"""Hybrid SIM/REPLAY/DRY_RUN mission-slice adapter.

Navigation alone delegates semantic waypoint names to the safety-gated SIM backend.
Perception consumes one explicit :class:`ReplayFrame`, and manipulation validates an
independently supplied station-local target.  It neither derives arm coordinates from
pixels nor imports ROS/controller APIs.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Mapping, Optional, Tuple

from raicom_core.models import GraspEvidence, SafeHoldEvidence, SafeHoldResult
from raicom_perception import ObservationState, ReplayFrame, SimReplayPerceptionAdapter

from .api import FakeBackend, OperationResult, SimBackend
from .manipulation import ManipulationOutcome, ManipulationPlanner, ManipulationTarget


@dataclass(frozen=True)
class ThinMissionWaypointConfig:
    """Map MissionRunner semantic stages to preconfigured SIM waypoint names.

    This configuration intentionally contains names only. Coordinates remain inside
    the injected :class:`SimBackend` waypoint profile.
    """

    names: Mapping[str, str]

    def resolve(self, semantic_waypoint: str) -> str:
        configured_name = self.names.get(semantic_waypoint)
        if not isinstance(configured_name, str) or not configured_name.strip():
            raise ValueError("semantic waypoint {!r} has no named SIM configuration".format(semantic_waypoint))
        return configured_name


@dataclass(frozen=True)
class ThinMissionSliceReport:
    """Explicit non-field-validation provenance emitted by the slice."""

    navigation: str
    perception: str
    manipulation: str
    task_result: str
    real_availability: str
    events: Tuple[str, ...]


class DeterministicThinMissionSliceBackend(FakeBackend):
    """MissionRunner hybrid backend with SIM navigation, REPLAY, and DRY_RUN.

    ``target`` is intentionally supplied independently of the replay detection. Only
    navigation may actuate, through the injected SIM-only backend and named waypoint
    configuration; replay and dry-run planning issue no robot commands.
    """

    def __init__(
        self,
        *,
        frame: ReplayFrame,
        perception: SimReplayPerceptionAdapter,
        navigation: SimBackend,
        waypoints: ThinMissionWaypointConfig,
        target: Optional[ManipulationTarget],
        planner: Optional[ManipulationPlanner],
        grasp_evidence: GraspEvidence,
        now: Callable[[], float],
    ) -> None:
        super().__init__(grasp_evidence=grasp_evidence)
        self._frame = frame
        self._perception = perception
        self._navigation = navigation
        self._waypoints = waypoints
        self._target = target
        self._planner = planner
        self._now = now
        self._last_plan_outcome: Optional[ManipulationOutcome] = None
        self._events: list[str] = []

    def report(self) -> ThinMissionSliceReport:
        manipulation = "DRY_RUN/{0}".format(
            self._last_plan_outcome.value if self._last_plan_outcome else "NOT_RUN"
        )
        return ThinMissionSliceReport(
            navigation="SIM_TESTED",
            perception="REPLAY",
            manipulation=manipulation,
            task_result="NOT_REAL_SUCCESS",
            real_availability="REAL_UNAVAILABLE",
            events=tuple(self._events),
        )

    def _deadline_result(self, deadline_s: Optional[float], operation: str) -> Optional[OperationResult]:
        if deadline_s is not None and self._now() > deadline_s:
            self._events.append("{0}:TIMEOUT".format(operation))
            return OperationResult.TIMEOUT
        return None

    def safe_hold(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        """Return explicit SIM-only hold evidence, never physical stop proof."""
        result = self._navigation.safe_hold(deadline_s)
        super().safe_hold(deadline_s)
        self._events.append("NAVIGATION:SIM_TESTED:HOLD:{0}".format(result.evidence.value))
        if result.evidence is SafeHoldEvidence.CONFIRMED_SIM:
            return SafeHoldResult(SafeHoldEvidence.CONFIRMED_SIM, "thin slice SIM navigation hold")
        if result.evidence is SafeHoldEvidence.TIMEOUT:
            return SafeHoldResult(SafeHoldEvidence.TIMEOUT, "thin slice SIM hold timeout")
        return SafeHoldResult(SafeHoldEvidence.UNCONFIRMED, "thin slice SIM hold " + result.evidence.value)

    def stop_all(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        return self.safe_hold(deadline_s)

    def cancel_active_operation(self) -> OperationResult:
        return self.cancel_navigation()

    def emergency_stop(self) -> None:
        """Latch this slice and cancel the active SIM navigation goal."""
        self._navigation.emergency_stop()
        super().emergency_stop()

    def cancel_navigation(self) -> OperationResult:
        result = self._navigation.cancel_navigation()
        self._events.append("NAVIGATION:SIM_TESTED:CANCEL:{0}".format(result.value))
        return result

    def goto(self, waypoint: str, deadline_s: Optional[float] = None) -> OperationResult:
        timed_out = self._deadline_result(deadline_s, "NAVIGATION")
        if timed_out is not None:
            return timed_out
        if self._safety_latched:
            return OperationResult.SAFETY_DENIED
        try:
            configured_waypoint = self._waypoints.resolve(waypoint)
        except ValueError as exc:
            result = OperationResult.NOT_CONFIGURED
            self._events.append("NAVIGATION:SIM_TESTED:{0}:{1}".format(result.value, exc))
            return result
        result = self._navigation.goto(configured_waypoint, deadline_s)
        self._events.append("NAVIGATION:SIM_TESTED:{0}".format(result.value))
        return result

    def perceive(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        timed_out = self._deadline_result(deadline_s, "PERCEPTION")
        if timed_out is not None:
            return timed_out
        if self._safety_latched:
            return OperationResult.SAFETY_DENIED
        result = self._perception.perceive(self._frame)
        self._events.append("PERCEPTION:REPLAY:{0}:{1}".format(result.state.value, result.reason))
        if result.state == ObservationState.DETECTED:
            self.last_perception_target = target
            return OperationResult.SUCCEEDED
        if result.state == ObservationState.UNAVAILABLE:
            return OperationResult.UNAVAILABLE
        return OperationResult.FAILED

    def grasp(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        timed_out = self._deadline_result(deadline_s, "MANIPULATION")
        if timed_out is not None:
            return timed_out
        if self._safety_latched:
            return OperationResult.SAFETY_DENIED
        if self._target is None:
            self._events.append("MANIPULATION:DRY_RUN:NOT_CONFIGURED:station target missing")
            return OperationResult.NOT_CONFIGURED
        if self._planner is None:
            self._events.append("MANIPULATION:DRY_RUN:UNAVAILABLE:planner missing")
            return OperationResult.UNAVAILABLE
        plan = self._planner.plan(self._target, self.grasp_evidence(target), self._now())
        self._last_plan_outcome = plan.outcome
        self._events.append("MANIPULATION:DRY_RUN:{0}:{1}".format(plan.outcome.value, plan.detail))
        if plan.outcome == ManipulationOutcome.SIM_TESTED:
            self.last_grasp_target = target
            return OperationResult.SUCCEEDED
        if plan.outcome == ManipulationOutcome.UNAVAILABLE:
            return OperationResult.UNAVAILABLE
        return OperationResult.FAILED

    def place(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        timed_out = self._deadline_result(deadline_s, "PLACEMENT")
        if timed_out is not None:
            return timed_out
        result = super().place(target, deadline_s)
        self._events.append("PLACEMENT:DRY_RUN:{0}".format(result.value))
        return result
