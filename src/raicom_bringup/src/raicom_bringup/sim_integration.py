"""Pure, non-actuating durable SIM integration controller (no ROS imports)."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import threading
import time
from typing import Callable, Optional, Protocol, Tuple

from raicom_core.dispatch import AbortCurrentState, DispatchQueue, DispatchTask
from raicom_core.events import AcceptedScoreMirror, EventJournal
from raicom_core.mission import MissionCore, TransitionError
from raicom_core.models import (AssignedWork, MissionPrimitive, MissionState, OperationResult,
    PrimitiveAttempt, PrimitiveOutcome, RunSubstate, SafeHoldResult, Task)
from raicom_core.recovery import RecoveryAction, RecoveryFailure, RecoveryPolicy, RequestedAction
from raicom_core.runner import MissionRunner
from raicom_core.turntable import TurntableState, TurntableWindowFSM
from raicom_platform.charging import ChargingBackend, ChargingOutcome, ChargingSession


class IntegrationMode(str, Enum):
    SIM = "SIM"
    FAKE = "FAKE"
    REAL = "REAL"


@dataclass(frozen=True)
class IntegrationAdmission:
    mode: IntegrationMode
    entirely_non_actuating: bool
    capability: str = "SIM_INTEGRATION"
    def __post_init__(self) -> None:
        if not isinstance(self.mode, IntegrationMode) or self.mode is IntegrationMode.REAL:
            raise ValueError("only SIM or FAKE integration modes are admissible")
        if self.entirely_non_actuating is not True or self.capability != "SIM_INTEGRATION":
            raise ValueError("integration must be explicitly and entirely non-actuating")


@dataclass(frozen=True)
class IntegrationSnapshot:
    mission_state: MissionState
    substate: Optional[RunSubstate]
    assignment_id: Optional[str]
    accepted_score: int
    mission_version: int
    estop_latched: bool
    resumable: bool
    admission_open: bool
    pending_dispatch_ids: Tuple[str, ...]
    current_dispatch_id: Optional[str]


class TrustedSafetyPort(Protocol):
    def safe_hold(self, deadline_s: float) -> SafeHoldResult: ...
    def latch_estop(self, deadline_s: float) -> None: ...


class SimRecoveryExecutor(Protocol):
    def execute(self, action: RequestedAction) -> OperationResult: ...


class _RunnerBackend:
    def cancel_active_operation(self) -> OperationResult: return OperationResult.UNAVAILABLE
    def safe_hold(self, deadline_s: float) -> SafeHoldResult: raise RuntimeError("use trusted safety port")
    def goto(self, waypoint: str, deadline_s: float) -> OperationResult: return OperationResult.UNAVAILABLE
    def perceive(self, target: str, deadline_s: float) -> OperationResult: return OperationResult.UNAVAILABLE
    def grasp(self, target: str, deadline_s: float) -> OperationResult: return OperationResult.UNAVAILABLE
    def place(self, target: str, deadline_s: float) -> OperationResult: return OperationResult.UNAVAILABLE
    def grasp_evidence(self, target: str): raise RuntimeError("no grasp evidence")


class SimIntegrationController:
    """Controller-owned mutation boundary. External ports are never called under its lock."""
    _DEPENDENCY_NAMES = frozenset({"_admission", "_journal", "_safety", "_mission", "_runner", "_dispatch", "_recovery", "_recovery_executor", "_turntable", "_charging"})

    def __setattr__(self, name, value) -> None:
        if name in self._DEPENDENCY_NAMES and getattr(self, "_dependencies_frozen", False):
            raise AttributeError("controller dependencies cannot be replaced")
        object.__setattr__(self, name, value)

    def __init__(self, admission: IntegrationAdmission, mission: MissionCore, journal: EventJournal,
                 safety: TrustedSafetyPort, *, backend=None, runner: Optional[MissionRunner] = None,
                 dispatch: Optional[DispatchQueue] = None, recovery: Optional[RecoveryPolicy] = None,
                 recovery_executor: Optional[SimRecoveryExecutor] = None,
                 turntable: Optional[TurntableWindowFSM] = None, charging: Optional[ChargingSession] = None,
                 now: Callable[[], float] = time.monotonic) -> None:
        """Copy initial semantics; caller-owned runner and queue injection is unsupported."""
        if not isinstance(admission, IntegrationAdmission) or not isinstance(mission, MissionCore):
            raise TypeError("admission and mission must be concrete types")
        if runner is not None or dispatch is not None:
            raise TypeError("caller-owned MissionRunner and DispatchQueue are unsupported")
        if not isinstance(journal, EventJournal) or type(safety) is None or not callable(getattr(safety, "safe_hold", None)) or not callable(getattr(safety, "latch_estop", None)):
            raise TypeError("concrete journal and safety port are required")
        if recovery is not None and not isinstance(recovery, RecoveryPolicy): raise TypeError("recovery must be RecoveryPolicy")
        self._admission, self._journal, self._safety = admission, journal, safety
        self._mission = MissionCore(mission.settlement_threshold, mission.mode, mission.state, mission.substate, mission.active_task, mission.accepted_score, mission.retry_count, mission.last_transition_reason)
        self._runner = MissionRunner(self._mission, backend or _RunnerBackend(), now=now)
        self._dispatch = DispatchQueue()
        self._recovery, self._recovery_executor, self._turntable, self._charging, self._now = recovery or RecoveryPolicy(), recovery_executor, turntable, charging, now
        events = EventJournal.read(journal.artifact_root, journal.relative_path)
        self._score_mirror = AcceptedScoreMirror.rebuild(events)
        if self._mission.accepted_score != self._score_mirror.score: raise TransitionError("core score differs from accepted score mirror")
        self._used_assignments = {e.payload.get("task_id") for e in events if e.event_type == "TASK_ASSIGNED" and isinstance(e.payload.get("task_id"), str)}
        self._assign_commands = {
            e.command_id: self._assignment_identity_from_payload(e.payload)
            for e in events if e.event_type == "TASK_ASSIGNED" and e.command_id
            and self._assignment_identity_from_payload(e.payload) is not None
        }
        self._lock = threading.RLock(); self._version = 0; self._estop = False; self._resumable = True; self._admission_open = True
        self._applied_command_ids: set[str] = set()
        # This controller deliberately has no crash-replay implementation.  A
        # durable terminal ESTOP is terminal regardless of supplied MissionCore;
        # every other lifecycle record is conservatively non-resumable too.
        if events:
            self._resumable = False
            self._admission_open = False
            if any(event.event_type == "ESTOP" for event in events):
                self._estop = True
                self._mission.estop("durable_estop_reconstructed")
            elif any(event.event_type in {"TASK_ASSIGNED", "PRIMITIVE_OUTCOME", "TURNTABLE_COMMIT", "TURNTABLE_PICK_DONE", "TURNTABLE_NEXT_WINDOW", "DISPATCH_PREPARED", "RECOVERY_REQUEST"} for event in events):
                # Do not claim idempotency from journal identity alone: none of
                # these controller-owned lifecycles is reconstructed here.
                self._resumable = False
        self._active: Optional[PrimitiveAttempt] = None
        self._last_completed_attempt: Optional[PrimitiveAttempt] = None
        self._last_completed_outcome: Optional[PrimitiveOutcome] = None
        self._last_completed_event = None
        self._abort_min_outcome_sequence: Optional[int] = None
        self._dependencies_frozen = True

    @classmethod
    def create(cls, admission: IntegrationAdmission, *, settlement_threshold: int, journal: EventJournal,
               safety: TrustedSafetyPort, backend=None, state: MissionState = MissionState.BOOT,
               substate: Optional[RunSubstate] = None, **kwargs):
        """Construct controller-owned mission, runner, queue and score mirror."""
        return cls(admission, MissionCore(settlement_threshold, state=state, substate=substate), journal, safety, backend=backend, **kwargs)

    @property
    def mission(self): return self._mission.snapshot()
    @property
    def runner(self): return self.snapshot()
    @property
    def dispatch(self): return self.dispatch_snapshot()
    def dispatch_snapshot(self) -> tuple[Tuple[str, ...], Optional[str], AbortCurrentState]:
        with self._lock: return tuple(x.task_id for x in self._dispatch.pending), (self._dispatch.current.task_id if self._dispatch.current else None), self._dispatch.abort_current_state

    def snapshot(self) -> IntegrationSnapshot:
        with self._lock:
            return IntegrationSnapshot(self._mission.state, self._mission.substate, self._runner.assigned_work.task_id if self._runner.assigned_work else None, self._score_mirror.score, self._version, self._estop, self._resumable, self._admission_open, tuple(x.task_id for x in self._dispatch.pending), self._dispatch.current.task_id if self._dispatch.current else None)

    def prepare_dispatch(self, task: DispatchTask, *, start: bool = False, command_id: Optional[str] = None) -> Optional[str]:
        if (not isinstance(task, DispatchTask) or not self._bounded_id(task.task_id)
                or not self._bounded_id(task.intent) or (task.target is not None and not self._bounded_id(task.target))):
            raise ValueError("invalid dispatch task")
        command_id = command_id or "dispatch:" + task.task_id
        self._require_command(command_id)
        with self._lock:
            self._require_admitted(command_id)
            selected: list[Optional[str]] = []
            def apply() -> None:
                self._dispatch.enqueue(task)
                current = self._dispatch.start_next() if start else self._dispatch.current
                selected.append(current.task_id if current else None)
                self._version += 1
            if not self._durable(command_id, "DISPATCH_PREPARED", {"task_id": task.task_id, "kind": task.intent, "target": task.target, "start": start}, apply):
                return None
            return selected[0] if selected else (self._dispatch.current.task_id if self._dispatch.current else None)

    def assign(self, assignment_id: str, task: Task, command_id: str, *, turntable_task: bool = False) -> bool:
        if not isinstance(task, Task) or not self._bounded_id(assignment_id) or not self._task_is_valid(task):
            raise ValueError("invalid assignment")
        identity = self._assignment_identity(assignment_id, task)
        # The turntable is an injected dependency: query it before acquiring the
        # controller lock, rather than assuming its methods are pure/nonblocking.
        if turntable_task and not self._turntable_admits_unlocked(task.target or task.kind, False):
            return False
        with self._lock:
            self._require_admitted(command_id)
            prior = self._assign_commands.get(command_id)
            if prior is not None:
                if prior == identity: return True
                self._fail_closed("assignment_command_conflict"); return False
            if assignment_id in self._used_assignments:
                self._fail_closed("assignment_id_reused"); return False
            if self._mission.state != MissionState.RUNNING or self._mission.substate != RunSubstate.SELECT_TASK: return False
            payload = {"task_id": assignment_id, "kind": task.kind, "target": task.target,
                       "score_on_referee_accept": task.score_on_referee_accept}
            ok = self._durable(command_id, "TASK_ASSIGNED", payload, lambda: self._apply_assignment(assignment_id, task))
            if ok: self._used_assignments.add(assignment_id); self._assign_commands[command_id] = identity
            return ok

    def _apply_assignment(self, assignment_id: str, task: Task) -> None:
        self._mission.assign_task(task); self._runner.assigned_work = AssignedWork(assignment_id, task); self._version += 1

    def begin_next_turntable_window(self, command_id: str, completed_target_id: str) -> bool:
        """Durably authorize the successor window after one completed target."""
        self._require_command(command_id)
        if not self._bounded_id(completed_target_id):
            raise ValueError("invalid completed target id")
        with self._lock:
            self._require_admitted(command_id)
            if command_id in self._applied_command_ids:
                return True
            if not self._durable(command_id, "TURNTABLE_NEXT_WINDOW", {"completed_target_id": completed_target_id}, lambda: setattr(self, "_version", self._version + 1)):
                return False
        # Event durability precedes the externally supplied FSM mutation.  A
        # malformed/stale lifecycle command therefore cannot silently reopen it.
        try:
            now_s = self._now()
            if self._turntable is None or not self._turntable.begin_next_window(completed_target_id, now_s):
                self._fail_closed("turntable_next_window_denied")
                return False
            with self._lock:
                # The externally supplied FSM may have blocked while ESTOP latched.
                return not self._estop
        except BaseException:
            self._fail_closed("turntable_next_window_exception")
            return False

    def step(self, command_id: str, *, turntable_pick: bool = False) -> Optional[PrimitiveOutcome]:
        # The injected clock may block/re-enter; obtain it before this lock.
        try: now_s = self._now()
        except BaseException: self._fail_closed("clock_query_exception"); return None
        with self._lock:
            self._require_admitted(command_id)
            attempt = self._runner.next_primitive(now_s=now_s)
            if attempt is None or self._active is not None: return None
        # The injected turntable and clock are queried outside the controller lock.
        if turntable_pick and attempt.kind is MissionPrimitive.GRASP:
            try:
                now_s = self._now()
                admitted = self._turntable_admits_unlocked(attempt.target_id, False)
                with self._lock:
                    if self._estop: return None
                if admitted:
                    with self._lock:
                        if self._estop or not self._durable(command_id + ":turntable_commit", "TURNTABLE_COMMIT", {"target_id": attempt.target_id, "attempt_id": attempt.attempt_id}, lambda: setattr(self, "_version", self._version + 1)):
                            return None
                committed = admitted and self._turntable is not None and self._turntable.commit(attempt.target_id, now_s)
                with self._lock:
                    if self._estop: return None
            except BaseException:
                self._fail_closed("turntable_precommit_exception")
                return None
            if not committed:
                outcome = PrimitiveOutcome(attempt.attempt_id, OperationResult.SAFETY_DENIED, "turntable_precommit_denied")
                with self._lock:
                    if self._estop: return outcome
                    return self._record_pre_execution_failure(command_id, outcome)
        with self._lock:
            if self._estop or self._active is not None: return None
            self._active = attempt
        # no command lock across runner/backend execution
        try: outcome = self._runner.execute_primitive(attempt)
        except BaseException as error: outcome = PrimitiveOutcome(attempt.attempt_id, OperationResult.FAILED, self._detail(error))
        with self._lock:
            self._active = None
            if self._estop: return outcome
            ok = self._record_outcome(command_id, outcome, external_failure_complete=False) if outcome.result is OperationResult.SUCCEEDED else None
            if outcome.result is not OperationResult.SUCCEEDED:
                # Reserve failure handling by releasing the lock before all ports.
                pass
            else:
                pass
            if outcome.result is OperationResult.SUCCEEDED:
                if not ok: return outcome
                # Abort evidence becomes visible only after the outcome append and apply commit.
                self._last_completed_attempt, self._last_completed_outcome = attempt, outcome
                self._last_completed_event = self._last_event
        if outcome.result is not OperationResult.SUCCEEDED:
            if not self._external_failure_phase(attempt):
                with self._lock:
                    # ESTOP won while an external failure action was blocked;
                    # expose no normal primitive completion to the caller.
                    return None if self._estop else outcome
            with self._lock:
                if self._estop: return outcome
                if not self._record_outcome(command_id, outcome, external_failure_complete=True): return outcome
        if turntable_pick and attempt.kind is MissionPrimitive.GRASP:
            try:
                if outcome.result is OperationResult.SUCCEEDED:
                    with self._lock:
                        if self._estop or not self._durable(command_id + ":pick_done", "TURNTABLE_PICK_DONE", {"target_id": attempt.target_id, "attempt_id": attempt.attempt_id}, lambda: setattr(self, "_version", self._version + 1)):
                            return None
                    if self._turntable is None or not self._turntable.mark_pick_done(attempt.target_id):
                        self._fail_closed("turntable_pick_lifecycle_failure")
                    with self._lock:
                        if self._estop: return None
                else:
                    continuation_admitted = self._turntable_admits_unlocked(attempt.target_id, True)
                    with self._lock:
                        if self._estop: return None
                    if not continuation_admitted:
                        with self._lock:
                            if not self._estop:
                                self._durable(command_id + ":window_lost", "TURNTABLE_WINDOW_LOST", {"target_id": attempt.target_id, "attempt_id": attempt.attempt_id}, lambda: setattr(self, "_version", self._version + 1))
                        self._fail_closed("turntable_window_lost_after_outcome")
            except BaseException:
                self._fail_closed("turntable_state_exception")
        return outcome

    def _record_pre_execution_failure(self, command_id: str, outcome: PrimitiveOutcome) -> PrimitiveOutcome:
        def apply() -> None:
            # Precommit denial did not call a backend but must make the active attempt
            # terminal so it cannot be retried as if it had never failed.
            if not self._runner.reject_active_attempt(outcome):
                raise TransitionError("pre-execution failure could not apply")
            self._version += 1
        self._durable(command_id, "PRIMITIVE_OUTCOME", {"attempt_id": outcome.attempt_id, "result": outcome.result.value, "detail": outcome.detail}, apply)
        return outcome

    def _external_failure_phase(self, attempt: PrimitiveAttempt) -> bool:
        """Cancel and hold outside controller lock; ESTOP wins before internal apply."""
        try:
            cancelled = self._runner.backend.cancel_active_operation()
            if not isinstance(cancelled, OperationResult):
                self._fail_closed("primitive_cancel_malformed"); return False
            hold = self._safety.safe_hold(attempt.deadline_s)
            if not isinstance(hold, SafeHoldResult) or not hold.confirmed:
                self._fail_closed("primitive_failure_hold_unconfirmed"); return False
        except BaseException:
            self._fail_closed("primitive_failure_external_exception"); return False
        with self._lock:
            return not self._estop

    def _record_outcome(self, command_id: str, outcome: PrimitiveOutcome, *, external_failure_complete: bool = False) -> bool:
        if not isinstance(outcome, PrimitiveOutcome) or not isinstance(outcome.result, OperationResult): self._fail_closed("malformed_primitive_outcome"); return False
        def apply() -> None:
            applied = self._runner.apply_primitive_success(outcome) if outcome.result is OperationResult.SUCCEEDED else self._runner.fail_active_operation(outcome, external_failure_complete=external_failure_complete)
            if not applied: raise TransitionError("primitive outcome could not apply")
            self._version += 1
        return self._durable(command_id, "PRIMITIVE_OUTCOME", {"attempt_id": outcome.attempt_id, "result": outcome.result.value, "detail": outcome.detail[:256]}, apply)

    def request_abort_current(self, command_id: str, primitive_in_progress: bool = False) -> AbortCurrentState:
        with self._lock:
            self._require_admitted(command_id)
            active = self._active is not None
            state_box: list[AbortCurrentState] = []
            if not self._durable(command_id, "ABORT_CURRENT_REQUESTED", {"primitive_in_progress": active}, lambda: state_box.append(self._dispatch.request_abort_current(active))): return AbortCurrentState.NONE
            # Only an outcome recorded after this abort intent may provide the
            # primitive half of its acknowledgement; prior completions are stale.
            self._abort_min_outcome_sequence = self._last_event.sequence
            self._admission_open = False
            return state_box[0]

    def acknowledge_abort_hold(self, command_id: str, deadline_s: float) -> bool:
        """Durably record the hold half of an abort acknowledgement matrix."""
        if not self._valid_deadline(deadline_s): raise ValueError("deadline must be finite and positive")
        with self._lock:
            self._require_command(command_id)
            replay = command_id in self._applied_command_ids
            if not replay and (self._estop or self._dispatch.abort_current_state is AbortCurrentState.NONE):
                return False
        # Safety is an external port and deliberately remains outside the command lock.
        if not replay:
            try: result = self._safety.safe_hold(deadline_s)
            except BaseException: self._fail_closed("abort_hold_exception"); return False
            if not isinstance(result, SafeHoldResult) or not result.confirmed: self._fail_closed("abort_hold_unconfirmed"); return False
        with self._lock:
            if self._estop: return False
            return self._durable(command_id, "ABORT_HOLD_CONFIRMED", {}, lambda: (self._dispatch.acknowledge_safety_stop(), setattr(self, "_version", self._version + 1)))

    def acknowledge_abort_primitive_complete(self, command_id: str, outcome: PrimitiveOutcome) -> bool:
        """Accept only the exact, durable, current-assignment primitive outcome."""
        with self._lock:
            self._require_command(command_id)
            attempt, completed, event = self._last_completed_attempt, self._last_completed_outcome, self._last_completed_event
            current = self._dispatch.current
            replay = command_id in self._applied_command_ids
            if replay:
                return (isinstance(outcome, PrimitiveOutcome) and outcome is completed
                        and attempt is not None and event is not None
                        and outcome.result is OperationResult.SUCCEEDED
                        and outcome.attempt_id == attempt.attempt_id)
            if (not isinstance(outcome, PrimitiveOutcome) or attempt is None or completed is None or event is None
                    # Object identity prevents a caller-created lookalike outcome from acting as proof.
                    or outcome is not completed or outcome.result is not OperationResult.SUCCEEDED
                    or outcome.attempt_id != attempt.attempt_id or self._abort_min_outcome_sequence is None
                    or event.sequence <= self._abort_min_outcome_sequence or self._runner.assigned_work is None
                    or attempt.assignment_id != self._runner.assigned_work.task_id
                    or current is None or current.task_id != attempt.assignment_id):
                return False
            payload = {"assignment_id": attempt.assignment_id, "attempt_id": outcome.attempt_id,
                       "result": outcome.result.value, "outcome_sequence": event.sequence,
                       "outcome_hash": event.event_hash}
        # The in-memory cache is not completion evidence. Re-read the canonical
        # hash-chained journal before allowing it to clear dispatch current work.
        if not self._outcome_event_is_durable(event, outcome):
            self._fail_closed("abort_outcome_not_durable")
            return False
        with self._lock:
            if self._estop or self._last_completed_attempt is not attempt or self._last_completed_outcome is not outcome or self._last_completed_event is not event:
                return False
            if self._dispatch.abort_current_state is AbortCurrentState.NONE:
                return False
            return self._durable(command_id, "ABORT_PRIMITIVE_COMPLETE", payload, lambda: (self._dispatch.acknowledge_primitive_complete(), setattr(self, "_version", self._version + 1)))

    def _outcome_event_is_durable(self, event, outcome: PrimitiveOutcome) -> bool:
        try:
            events = EventJournal.read(self._journal.artifact_root, self._journal.relative_path)
        except BaseException:
            return False
        return any(candidate == event and candidate.event_type == "PRIMITIVE_OUTCOME"
                   and candidate.payload.get("attempt_id") == outcome.attempt_id
                   and candidate.payload.get("result") == outcome.result.value
                   for candidate in events)

    def settle_charging(self, command_id: str, deadline_s: float) -> bool:
        if not self._valid_deadline(deadline_s): raise ValueError("deadline must be finite and positive")
        try:
            session = self._charging
            session_deadline = session.deadline_s if session is not None else None
            session_backend = session.backend if session is not None else None
            now_s = self._now()
        except BaseException: self._fail_closed("charging_query_exception"); return False
        with self._lock:
            self._require_admitted(command_id)
            if session is None or session_backend is not ChargingBackend.SIM or self._mission.state is not MissionState.SETTLEMENT: return False
            if deadline_s <= now_s:
                self._fail_closed("settlement_hold_deadline_invalid"); return False
            if (not isinstance(session_deadline, (int, float)) or isinstance(session_deadline, bool)
                    or not math.isfinite(session_deadline) or session_deadline <= now_s):
                self._fail_closed("charging_deadline_invalid"); return False
        try:
            hold = self._safety.safe_hold(deadline_s)
            after_hold_s = self._now()
        except BaseException: self._fail_closed("settlement_hold_exception"); return False
        if not isinstance(hold, SafeHoldResult) or not hold.confirmed: self._fail_closed("settlement_hold_unconfirmed"); return False
        if after_hold_s >= session_deadline: return self._record_charge_timeout(command_id)
        with self._lock:
            # Do not issue a charging-session query after a hold that raced ESTOP.
            if self._estop: return False
            if not self._durable(command_id + ":hold", "SETTLEMENT_HOLD_CONFIRMED", {}, lambda: setattr(self, "_version", self._version + 1)): return False
        try:
            session.check_timeout(self._now())
            with self._lock:
                if self._estop: return False
            session.settlement_safe_stop()
            with self._lock:
                if self._estop: return False
            outcome = session.outcome
            final_now_s = self._now()
        except BaseException: self._fail_closed("charging_exception"); return False
        with self._lock:
            # Charging accessors are injected and may have blocked while ESTOP latched.
            if self._estop: return False
        if final_now_s >= session_deadline: return self._record_charge_timeout(command_id)
        with self._lock:
            if outcome is not ChargingOutcome.SIM_EVIDENCE_CONFIRMED:
                failure_outcome = getattr(outcome, "value", "UNKNOWN")
                if not self._durable(command_id, "CHARGE_FAILED", {"outcome": failure_outcome}, lambda: setattr(self, "_version", self._version + 1)): return False
                return self.recover(command_id + ":recovery", RecoveryFailure.CHARGE_FAILED)
            return self._durable(command_id, "SETTLEMENT_COMPLETE", {"outcome": outcome.value}, self._complete_settlement)

    def _record_charge_timeout(self, command_id: str) -> bool:
        with self._lock:
            if not self._durable(command_id, "CHARGE_FAILED", {"outcome": "TIMEOUT"}, lambda: setattr(self, "_version", self._version + 1)): return False
        return self.recover(command_id + ":recovery", RecoveryFailure.CHARGE_FAILED)

    def _complete_settlement(self) -> None:
        self._mission.settlement_complete(); self._mission.recharge_complete(); self._version += 1

    def recover(self, command_id: str, failure: RecoveryFailure) -> bool:
        try:
            # Policy and clock are injected; do not invoke either under the lock.
            decision = self._recovery.handle_failure(failure, self._now())
        except BaseException: self._fail_closed("recovery_policy_exception"); return False
        with self._lock:
            # A blocking policy query can race ESTOP; terminate quietly instead
            # of treating this expected safety interleave as a command exception.
            if self._estop: return False
            self._require_admitted(command_id)
            requested_actions = [{"action": action.action.value, "deadline_s": action.deadline_s} for action in decision.requested_actions]
            if (any(not self._valid_deadline(action.deadline_s) for action in decision.requested_actions)
                    or not self._durable(command_id, "RECOVERY_REQUEST", {"failure": failure.value, "exhausted": decision.exhausted, "requested_actions": requested_actions}, lambda: None)): return False
        if self._recovery_executor is None: self._fail_closed("recovery_executor_missing"); return False
        for index, action in enumerate(decision.requested_actions):
            with self._lock:
                # A policy query may have blocked while ESTOP arrived; never start
                # an executor action after that terminal transition.
                if self._estop: return False
            try: outcome = self._recovery_executor.execute(action)
            except BaseException: self._fail_closed("recovery_executor_exception"); return False
            with self._lock:
                if not isinstance(outcome, OperationResult) or not self._durable(f"{command_id}:outcome:{index}", "RECOVERY_OUTCOME", {"action": action.action.value, "result": getattr(outcome, "value", "MALFORMED")}, lambda: None):
                    self._fail_closed("recovery_outcome_failure"); return False
            if outcome is not OperationResult.SUCCEEDED: self._fail_closed("recovery_action_failed"); return False
            if action.action is RecoveryAction.SAFE_HOLD:
                try: hold = self._safety.safe_hold(action.deadline_s)
                except BaseException: self._fail_closed("recovery_hold_exception"); return False
                if not isinstance(hold, SafeHoldResult) or not hold.confirmed: self._fail_closed("recovery_hold_unconfirmed"); return False
                with self._lock:
                    if not self._durable(f"{command_id}:safe_hold:{index}", "RECOVERY_SAFE_HOLD", {}, lambda: self._mission.safe_stop("recovery_safe_hold")): return False
                    self._admission_open = False
            if action.action is RecoveryAction.ABORT_CURRENT:
                # Abort queue mutation remains exclusively bound to acknowledgement flow.
                with self._lock: self._admission_open = False
        return True

    def referee_verdict(self, command_id: str, accepted: bool) -> bool:
        with self._lock:
            self._require_admitted(command_id)
            if type(accepted) is not bool or self._runner.assigned_work is None or self._mission.substate is not RunSubstate.WAIT_REFEREE_VERDICT: return False
            work = self._runner.assigned_work; event_type = "REFEREE_ACCEPTED" if accepted else "REFEREE_REJECTED"
            def apply() -> None:
                if accepted: self._score_mirror.apply(self._last_event)
                self._mission.referee_verdict_with_score(accepted, self._score_mirror.score)
                if accepted: self._runner.assigned_work = None
                self._version += 1
            return self._durable(command_id, event_type, {"task_id": work.task_id, "score": work.task.score_on_referee_accept if accepted else 0}, apply)

    def estop(self, reason: str = "integration_estop") -> None: self._fail_closed(reason)
    def _fail_closed(self, reason: str) -> None:
        with self._lock:
            if self._estop: return
            self._estop = True; self._resumable = False; self._admission_open = False
            try: self._mission.estop(reason)
            except BaseException: pass
            version = self._version
        # State is terminal before every best-effort external port.  A caller may
        # already hold the command lock, so release all recursive ownership first.
        lock_state = self._lock._release_save() if self._lock._is_owned() else None
        try:
            # ESTOP must not wait for an injected mission clock: that callable
            # is an external dependency and may itself be blocked or re-enter.
            estop_now_s = time.monotonic()
            try: self._safety.latch_estop(estop_now_s)
            except BaseException: pass
            try: self._journal.append_once("ESTOP:" + str(version), "ESTOP", {"reason": self._detail_text(reason)}, estop_now_s)
            except BaseException: pass
        finally:
            if lock_state is not None: self._lock._acquire_restore(lock_state)

    def _durable(self, command_id: str, event_type: str, payload: dict, apply: Callable[[], None]) -> bool:
        """Durably append outside the lock, then atomically perform a pure internal apply."""
        expected_version = self._version
        lock_state = self._lock._release_save()
        try:
            event = self._journal.append_once(command_id, event_type, payload, self._now())
        except BaseException:
            event = None
        finally:
            self._lock._acquire_restore(lock_state)
        if event is None:
            self._fail_closed("journal_failure_before_apply")
            return False
        # ESTOP and every state write share this lock. A changed version means some
        # command interleaved while append blocked, so this command must not apply.
        if self._estop or self._version != expected_version:
            return False
        if command_id in self._applied_command_ids:
            return True
        try:
            apply()  # Contract: internal state mutation only; no ports or journaling.
            self._applied_command_ids.add(command_id)
            self._last_event = event
            self._invariant()
            return True
        except BaseException:
            self._fail_closed("apply_failure_after_append")
            return False

    def _require_admitted(self, command_id: str) -> None:
        self._require_command(command_id)
        # Existing immutable commands are retries, not fresh admission attempts.
        if command_id in self._applied_command_ids:
            return
        if self._estop or not self._resumable or not self._admission_open: raise TransitionError("integration admission closed")
    def _require_command(self, command_id: str) -> None:
        if not self._bounded_id(command_id): raise ValueError("invalid command id")
    @staticmethod
    def _bounded_id(value: str) -> bool: return isinstance(value, str) and bool(value.strip()) and len(value) <= 128 and value == value.strip()
    @staticmethod
    def _valid_deadline(value: float) -> bool: return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value > 0
    @staticmethod
    def _detail(error: BaseException) -> str:
        try: text = str(error)
        except BaseException: text = "<unprintable>"
        return (type(error).__name__ + ":" + text)[:256]
    @staticmethod
    def _detail_text(value: str) -> str: return str(value)[:256]
    @staticmethod
    def _task_is_valid(task: Task) -> bool:
        return (isinstance(task.kind, str) and bool(task.kind.strip()) and len(task.kind) <= 128
                and (task.target is None or (isinstance(task.target, str) and bool(task.target.strip()) and len(task.target) <= 128))
                and isinstance(task.score_on_referee_accept, int) and not isinstance(task.score_on_referee_accept, bool))

    @staticmethod
    def _assignment_identity(assignment_id: str, task: Task) -> tuple[str, str, Optional[str], int]:
        return (assignment_id, task.kind, task.target, task.score_on_referee_accept)

    @classmethod
    def _assignment_identity_from_payload(cls, payload: dict):
        task_id, kind = payload.get("task_id"), payload.get("kind")
        target, score = payload.get("target"), payload.get("score_on_referee_accept")
        if (not cls._bounded_id(task_id) or not isinstance(kind, str) or not kind.strip()
                or (target is not None and (not isinstance(target, str) or not target.strip()))
                or not isinstance(score, int) or isinstance(score, bool)):
            return None
        return (task_id, kind, target, score)

    def _turntable_admits_unlocked(self, target: str, continuation: bool) -> bool:
        try:
            now_s = self._now()
            turntable = self._turntable
            if turntable is None:
                return False
            state = turntable.state
            with self._lock:
                if self._estop:
                    return False
            if state is TurntableState.WINDOW_LOST:
                return False
            admitted = turntable.admits_target(target, now_s, continuation=continuation)
            with self._lock:
                return admitted and not self._estop
        except BaseException:
            self._fail_closed("turntable_state_exception")
            return False

    def _turntable_admits(self, target: str, continuation: bool) -> bool:
        # Deprecated internal spelling; callers must invoke the unlocked variant.
        return self._turntable_admits_unlocked(target, continuation)
    def _invariant(self) -> None:
        if self._mission.accepted_score != self._score_mirror.score: raise TransitionError("core score differs from accepted score mirror")
        if self._estop and self._mission.state is not MissionState.ESTOP: raise TransitionError("estop must be terminal")
