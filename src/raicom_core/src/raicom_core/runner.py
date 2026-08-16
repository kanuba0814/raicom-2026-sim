"""Deterministic semantic mission runner with stepwise primitive execution."""
from __future__ import annotations

import math
import threading
import time
from enum import Enum
from typing import Callable, List, Optional, Protocol

from .hmi import HmiAction, HmiRequest
from .mission import MissionCore, TransitionError
from .models import (
    AssignedWork,
    GraspEvidence,
    MissionPrimitive,
    MissionState,
    OperationResult,
    PrimitiveAttempt,
    PrimitiveOutcome,
    RunSubstate,
    SafeHoldResult,
    Task,
)
from .scheduler import MissionScheduler, SchedulerPolicy, TaskCandidate
from .turntable import TurntableWindowFSM


class MissionBackend(Protocol):
    def cancel_active_operation(self) -> OperationResult: ...
    def safe_hold(self, deadline_s: float) -> SafeHoldResult: ...
    def goto(self, waypoint: str, deadline_s: float) -> OperationResult: ...
    def perceive(self, target: str, deadline_s: float) -> OperationResult: ...
    def grasp(self, target: str, deadline_s: float) -> OperationResult: ...
    def place(self, target: str, deadline_s: float) -> OperationResult: ...
    def grasp_evidence(self, target: str) -> GraspEvidence: ...


class SafetyCoordinatorPort(Protocol):
    """Process-level E-stop latch seam; acknowledgement never gates Mission ESTOP."""

    def latch_estop(self, deadline_s: float) -> None: ...


class _NonActuatingBackend:
    def cancel_active_operation(self) -> OperationResult: return OperationResult.UNAVAILABLE
    def safe_hold(self, deadline_s: float) -> object: return None
    def goto(self, waypoint: str, deadline_s: float) -> OperationResult: return OperationResult.UNAVAILABLE
    def perceive(self, target: str, deadline_s: float) -> OperationResult: return OperationResult.UNAVAILABLE
    def grasp(self, target: str, deadline_s: float) -> OperationResult: return OperationResult.UNAVAILABLE
    def place(self, target: str, deadline_s: float) -> OperationResult: return OperationResult.UNAVAILABLE
    def grasp_evidence(self, target: str) -> GraspEvidence:
        from .models import GraspState
        return GraspEvidence(GraspState.UNKNOWN, target, 0.0, "unavailable")


class _NonActuatingSafetyCoordinator:
    def __init__(self) -> None: self.latched = False
    def latch_estop(self, deadline_s: float) -> None: self.latched = True


class _DispatchState(str, Enum):
    """Linearized backend dispatch admission state."""

    IDLE = "IDLE"
    RESERVED = "RESERVED"
    COMMITTED = "COMMITTED"


class MissionRunner:
    """Execute at most one backend operation for each issued primitive attempt."""

    _PRIMITIVES = {
        RunSubstate.NAVIGATE_COARSE: MissionPrimitive.NAVIGATE_PICKUP,
        RunSubstate.PERCEIVE: MissionPrimitive.PERCEIVE,
        RunSubstate.GRASP: MissionPrimitive.GRASP,
        RunSubstate.VERIFY_GRASP: MissionPrimitive.VERIFY_GRASP,
        RunSubstate.TRANSPORT: MissionPrimitive.NAVIGATE_DROPOFF,
        RunSubstate.PLACE_OR_ASSEMBLE: MissionPrimitive.PLACE,
        RunSubstate.VERIFY_LOCAL_RESULT: MissionPrimitive.HOLD_FOR_REFEREE,
    }

    def __init__(self, mission: MissionCore, backend: Optional[MissionBackend] = None,
                 scheduler: Optional[MissionScheduler] = None, *,
                 safety_coordinator: Optional[SafetyCoordinatorPort] = None,
                 now: Callable[[], float] = time.monotonic,
                 operation_timeout_s: float = 10.0, grasp_max_age_s: float = 1.0) -> None:
        if (not isinstance(operation_timeout_s, (int, float))
                or isinstance(operation_timeout_s, bool)
                or not math.isfinite(operation_timeout_s)
                or operation_timeout_s <= 0.0):
            raise ValueError("operation_timeout_s must be finite and positive")
        self.mission = mission
        self.backend = backend or _NonActuatingBackend()
        self.scheduler = scheduler or MissionScheduler()
        self.safety_coordinator = safety_coordinator or _NonActuatingSafetyCoordinator()
        self._now, self._operation_timeout_s, self._grasp_max_age_s = now, operation_timeout_s, grasp_max_age_s
        self.last_safety_errors: List[str] = []
        self.assigned_work: Optional[AssignedWork] = None
        self._assignment_sequence = 0
        self._attempt_sequence = 0
        self._active_attempt: Optional[PrimitiveAttempt] = None
        self._executed_outcome: Optional[PrimitiveOutcome] = None
        # The lock linearizes E-stop against backend invocation, but is never
        # held across a backend call. RESERVED can be canceled by E-stop;
        # COMMITTED means invocation is active/authorized.
        self._attempt_lock = threading.Lock()
        self._dispatch_state = _DispatchState.IDLE
        # Test-only deterministic seam between reservation and commit.
        self._pre_invocation_hook: Optional[Callable[[PrimitiveAttempt], None]] = None

    def start_from_hmi(self) -> bool:
        request = HmiRequest(HmiAction.START, self.mission.mode)
        if not request.authorized(safety_paused=True, referee_waiting=False): return False
        self.mission.precheck_passed(); self.mission.wait_for_start(); self.mission.start()
        return True

    def select_task(self, candidates: List[TaskCandidate], policy: SchedulerPolicy,
                    turntable_fsm: Optional[TurntableWindowFSM] = None,
                    now_s: Optional[float] = None) -> Optional[Task]:
        if self.mission.state != MissionState.RUNNING or self.mission.substate != RunSubstate.SELECT_TASK:
            return None
        task = self.scheduler.select(candidates, policy, turntable_fsm, now_s)
        if task is not None:
            self._assignment_sequence += 1
            self.assigned_work = AssignedWork("local-{}".format(self._assignment_sequence), task)
            self.mission.assign_task(task)
            self._clear_attempt()
        return task

    def next_primitive(self, *, now_s: Optional[float] = None) -> Optional[PrimitiveAttempt]:
        """Issue (or replay) the sole attempt authorized for the current substate.

        ``now_s`` lets an owning controller obtain an injected clock before it
        acquires its own mutation lock.
        """
        with self._attempt_lock:
            if self.mission.state == MissionState.ESTOP:
                return None
            work = self._validated_work()
            kind = self._PRIMITIVES.get(self.mission.substate)
            if work is None or kind is None:
                return None
            if self._active_attempt is not None:
                if self._attempt_matches_current(self._active_attempt):
                    return self._active_attempt
                self._clear_attempt()
                return None
            self._attempt_sequence += 1
            self._active_attempt = PrimitiveAttempt(
                attempt_id="primitive-{}".format(self._attempt_sequence),
                kind=kind,
                expected_substate=self.mission.substate,
                deadline_s=(self._now() if now_s is None else now_s) + self._operation_timeout_s,
                target_id=work.task.target or work.task.kind,
                assignment_id=work.task_id,
            )
            return self._active_attempt

    def execute_primitive(self, attempt: PrimitiveAttempt) -> PrimitiveOutcome:
        """Execute exactly one operation and never mutate mission progression."""
        if not isinstance(attempt, PrimitiveAttempt):
            raise TypeError("attempt must be a PrimitiveAttempt")
        with self._attempt_lock:
            if self.mission.state == MissionState.ESTOP or not self._attempt_matches_current(attempt):
                raise TransitionError("stale or mismatched primitive attempt")
            if self._executed_outcome is not None or self._dispatch_state is not _DispatchState.IDLE:
                raise TransitionError("primitive attempt has already executed")
            if self._now() >= attempt.deadline_s:
                outcome = PrimitiveOutcome(attempt.attempt_id, OperationResult.TIMEOUT, "attempt_expired")
                self._executed_outcome = outcome
                return outcome
            self._dispatch_state = _DispatchState.RESERVED

        outcome: Optional[PrimitiveOutcome] = None
        try:
            hook = self._pre_invocation_hook
            if hook is not None:
                hook(attempt)
            with self._attempt_lock:
                # E-stop cancels RESERVED under this same lock. Only a live
                # attempt may cross the commit linearization point.
                if (self._dispatch_state is not _DispatchState.RESERVED
                        or self.mission.state == MissionState.ESTOP
                        or not self._attempt_matches_current(attempt)):
                    outcome = PrimitiveOutcome(attempt.attempt_id, OperationResult.SAFETY_DENIED,
                                               "dispatch_denied")
                else:
                    self._dispatch_state = _DispatchState.COMMITTED
            if outcome is None:
                raw_result, detail = self._execute_backend(attempt)
                if not isinstance(raw_result, OperationResult):
                    outcome = PrimitiveOutcome(attempt.attempt_id, OperationResult.FAILED,
                                               "malformed_result:{}".format(type(raw_result).__name__))
                else:
                    outcome = PrimitiveOutcome(attempt.attempt_id, raw_result, detail)
        except BaseException as error:
            outcome = PrimitiveOutcome(attempt.attempt_id, OperationResult.FAILED,
                                       "operation_exception=" + self._error_detail(error))
        finally:
            # Every BaseException path resolves dispatch state before returning.
            with self._attempt_lock:
                self._dispatch_state = _DispatchState.IDLE
                if (outcome is not None and self.mission.state != MissionState.ESTOP
                        and self._attempt_matches_current(attempt)):
                    self._executed_outcome = outcome
        assert outcome is not None
        return outcome

    def apply_primitive_success(self, outcome: PrimitiveOutcome) -> bool:
        """Advance only the currently executed successful attempt, exactly once."""
        with self._attempt_lock:
            if not self._outcome_matches(outcome) or outcome.result is not OperationResult.SUCCEEDED:
                return False
            attempt = self._active_attempt
            assert attempt is not None
            if self.mission.state == MissionState.ESTOP or not self._attempt_matches_current(attempt):
                return False
            try:
                self._advance(attempt)
            except BaseException:
                return False
            self._clear_attempt()
            return True

    def reject_active_attempt(self, outcome: PrimitiveOutcome) -> bool:
        """Terminally fail an issued-but-uninvoked attempt after admission denial."""
        with self._attempt_lock:
            if (not isinstance(outcome, PrimitiveOutcome) or outcome.result is OperationResult.SUCCEEDED
                    or self._active_attempt is None or outcome.attempt_id != self._active_attempt.attempt_id
                    or self._dispatch_state is not _DispatchState.IDLE or self.mission.state == MissionState.ESTOP):
                return False
            attempt = self._active_attempt
            self._clear_attempt()
        self._failure("{}:{}:{}".format(attempt.expected_substate.value, outcome.result.value, outcome.detail))
        return self.mission.substate == RunSubstate.RECOVERY

    def fail_active_operation(self, outcome: PrimitiveOutcome, *, external_failure_complete: bool = False) -> bool:
        """Fail the matching executed attempt; stale outcomes cannot affect mission state.

        When an owner has already performed cancel/authoritative hold outside
        its lock, it may request the pure internal transition variant.
        """
        if (not self._outcome_matches(outcome)
                or outcome.result is OperationResult.SUCCEEDED
                or self.mission.state == MissionState.ESTOP):
            return False
        attempt = self._active_attempt
        assert attempt is not None
        self._clear_attempt()
        reason = "{}:{}{}".format(
            attempt.expected_substate.value, outcome.result.value,
            ":" + outcome.detail if outcome.detail else "")
        if external_failure_complete:
            if attempt.kind is MissionPrimitive.HOLD_FOR_REFEREE:
                return False
            self.mission.operation_failed(reason)
            return self.mission.substate == RunSubstate.RECOVERY
        if attempt.kind is MissionPrimitive.HOLD_FOR_REFEREE:
            self.mission.stop_unconfirmed(reason + ":hold_unconfirmed")
            self._latch_estop(reason + ":estop_latched")
            return False
        self._failure(reason)
        return self.mission.substate == RunSubstate.RECOVERY

    def _execute_backend(self, attempt: PrimitiveAttempt) -> tuple[object, str]:
        target = attempt.target_id
        if attempt.kind is MissionPrimitive.NAVIGATE_PICKUP:
            return self.backend.goto("pickup:" + target, attempt.deadline_s), ""
        if attempt.kind is MissionPrimitive.PERCEIVE:
            return self.backend.perceive(target, attempt.deadline_s), ""
        if attempt.kind is MissionPrimitive.GRASP:
            return self.backend.grasp(target, attempt.deadline_s), ""
        if attempt.kind is MissionPrimitive.VERIFY_GRASP:
            evidence = self.backend.grasp_evidence(target)
            if not isinstance(evidence, GraspEvidence):
                return OperationResult.FAILED, "malformed_grasp_evidence"
            valid = evidence.confirms(target, self._now(), self._grasp_max_age_s)
            return (OperationResult.SUCCEEDED if valid else OperationResult.FAILED), evidence.state.value
        if attempt.kind is MissionPrimitive.NAVIGATE_DROPOFF:
            return self.backend.goto("dropoff:" + target, attempt.deadline_s), ""
        if attempt.kind is MissionPrimitive.PLACE:
            return self.backend.place(target, attempt.deadline_s), ""
        if attempt.kind is MissionPrimitive.HOLD_FOR_REFEREE:
            result = self.backend.safe_hold(attempt.deadline_s)
            return (OperationResult.SUCCEEDED if isinstance(result, SafeHoldResult) and result.confirmed
                    else OperationResult.FAILED), ""
        raise TransitionError("unsupported primitive")

    def _advance(self, attempt: PrimitiveAttempt) -> None:
        transitions = {
            MissionPrimitive.NAVIGATE_PICKUP: (RunSubstate.PERCEIVE, "navigation_complete"),
            MissionPrimitive.PERCEIVE: (RunSubstate.GRASP, "perception_complete"),
            MissionPrimitive.GRASP: (RunSubstate.VERIFY_GRASP, "grasp_complete"),
            MissionPrimitive.VERIFY_GRASP: (RunSubstate.TRANSPORT, "grasp_confirmed"),
            MissionPrimitive.NAVIGATE_DROPOFF: (RunSubstate.PLACE_OR_ASSEMBLE, "transport_complete"),
            MissionPrimitive.PLACE: (RunSubstate.VERIFY_LOCAL_RESULT, "placement_complete"),
        }
        if attempt.kind is MissionPrimitive.HOLD_FOR_REFEREE:
            self.mission.physical_task_complete()
            self.mission.robot_stopped_for_referee()
            return
        next_substate, reason = transitions[attempt.kind]
        self.mission.advance_operation(attempt.expected_substate, next_substate, reason)

    def _validated_work(self) -> Optional[AssignedWork]:
        work = self.assigned_work
        if (self.mission.state != MissionState.RUNNING or self.mission.active_task is None
                or work is None or work.task != self.mission.active_task):
            return None
        return work

    def _attempt_matches_current(self, attempt: PrimitiveAttempt) -> bool:
        work = self._validated_work()
        return (self._active_attempt == attempt and work is not None
                and attempt.assignment_id == work.task_id
                and attempt.expected_substate == self.mission.substate
                and attempt.kind is self._PRIMITIVES.get(self.mission.substate)
                and attempt.target_id == (work.task.target or work.task.kind))

    def _outcome_matches(self, outcome: PrimitiveOutcome) -> bool:
        return (isinstance(outcome, PrimitiveOutcome)
                and isinstance(outcome.result, OperationResult)
                and self._active_attempt is not None
                and self._executed_outcome == outcome
                and outcome.attempt_id == self._active_attempt.attempt_id)

    def _clear_attempt(self) -> None:
        self._active_attempt = None
        self._executed_outcome = None
        # An E-stop concurrent with a committed backend call must continue to
        # expose that invocation as active until its dispatch finally runs.
        # A RESERVED dispatch, by contrast, is canceled before invocation.
        if self._dispatch_state is not _DispatchState.COMMITTED:
            self._dispatch_state = _DispatchState.IDLE

    @staticmethod
    def _error_detail(error: BaseException) -> str:
        """Return bounded diagnostics without allowing hostile exceptions to escape."""
        try:
            detail = str(error)
        except BaseException:
            detail = "<unprintable>"
        return "{}:{}".format(type(error).__name__, detail)[:256]

    def _latch_estop(self, reason: str) -> None:
        # Commit terminal state under the same lock as execution admission.  Do
        # not hold it while calling the coordinator, which may synchronously
        # interact with backend cancellation or another E-stop path.
        with self._attempt_lock:
            if self.mission.state == MissionState.ESTOP:
                return
            self.mission.estop(reason)
            self._clear_attempt()
        coordinator_detail = ""
        try:
            self.safety_coordinator.latch_estop(self._now() + self._operation_timeout_s)
        except BaseException as error:
            coordinator_detail = ":coordinator=" + self._error_detail(error)
            self.last_safety_errors.append(coordinator_detail[1:])
        if coordinator_detail:
            with self._attempt_lock:
                # A concurrent external E-stop owns its reason.
                if self.mission.last_transition_reason == reason:
                    self.mission.last_transition_reason = reason + coordinator_detail

    def _hold_or_estop(self, reason: str, confirmed_transition: Callable[[], None]) -> bool:
        if self.mission.state == MissionState.ESTOP:
            return False
        try:
            result = self.backend.safe_hold(self._now() + self._operation_timeout_s)
        except BaseException as error:
            if self.mission.state != MissionState.ESTOP:
                self._latch_estop(reason + ":hold_exception=" + self._error_detail(error))
            return False
        if self.mission.state == MissionState.ESTOP:
            return False
        if isinstance(result, SafeHoldResult) and result.confirmed:
            try:
                confirmed_transition()
                return True
            except BaseException as error:
                if self.mission.state != MissionState.ESTOP:
                    self._latch_estop(reason + ":transition_exception=" + self._error_detail(error))
                return False
        self.mission.stop_unconfirmed(reason + ":hold_unconfirmed")
        self._latch_estop(reason + ":estop_latched")
        return False

    def _failure(self, reason: str, *, already_cancelled: bool = False) -> None:
        if self.mission.state == MissionState.ESTOP:
            return
        if not already_cancelled:
            try:
                result = self.backend.cancel_active_operation()
                if not isinstance(result, OperationResult):
                    reason += ":malformed_cancel_result"
            except BaseException as error:
                reason += ":cancel_exception=" + self._error_detail(error)
        if self.mission.state != MissionState.ESTOP:
            self._hold_or_estop(reason, lambda: self.mission.operation_failed(reason))

    def execute_active_task(self) -> bool:
        """Compatibility loop over the explicit primitive seam."""
        if self.mission.state == MissionState.ESTOP:
            return False
        if self.mission.state != MissionState.RUNNING or self.mission.active_task is None:
            raise TransitionError("execution requires a selected running task")
        if self.assigned_work is None:
            # Compatibility for callers that assigned directly through MissionCore.
            self._assignment_sequence += 1
            self.assigned_work = AssignedWork(
                "compat-{}".format(self._assignment_sequence), self.mission.active_task)
        while True:
            attempt = self.next_primitive()
            if attempt is None:
                return self.mission.substate == RunSubstate.WAIT_REFEREE_VERDICT
            outcome = self.execute_primitive(attempt)
            if self.mission.state == MissionState.ESTOP:
                return False
            if outcome.result is not OperationResult.SUCCEEDED:
                self.fail_active_operation(outcome)
                return False
            if not self.apply_primitive_success(outcome):
                return False

    def cancel_active_operation(self) -> None:
        if self.mission.state != MissionState.RUNNING or self.mission.active_task is None:
            return
        try:
            result = self.backend.cancel_active_operation()
            reason = ("operation_cancel:" + result.value if isinstance(result, OperationResult)
                      else "operation_cancel:malformed_result")
        except BaseException as error:
            reason = "operation_cancel:cancel_exception=" + self._error_detail(error)
        self._clear_attempt()
        self._failure(reason, already_cancelled=True)

    def referee_verdict(self, accepted: bool) -> bool:
        action = HmiAction.REFEREE_ACCEPT if accepted else HmiAction.REFEREE_REJECT
        waiting = self.mission.state == MissionState.RUNNING and self.mission.substate == RunSubstate.WAIT_REFEREE_VERDICT
        if not HmiRequest(action, self.mission.mode).authorized(safety_paused=True, referee_waiting=waiting): return False
        self.mission.referee_verdict(accepted)
        self.assigned_work = None
        self._clear_attempt()
        return True

    def estop(self, reason: str = "estop_latched") -> None:
        self._latch_estop(reason)
