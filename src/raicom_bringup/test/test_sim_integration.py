import sys
import tempfile
import threading
from pathlib import Path

import pytest

ROOT = Path(__file__).parents[2]
sys.path[:0] = [str(ROOT / name / "src") for name in ("raicom_bringup", "raicom_core", "raicom_platform", "raicom_perception")]

from raicom_bringup.sim_integration import IntegrationAdmission, IntegrationMode, SimIntegrationController
from raicom_core.dispatch import AbortCurrentState, DispatchTask
from raicom_core.events import EventJournal
from raicom_core.mission import MissionCore, TransitionError
from raicom_core.recovery import RecoveryFailure, RecoveryPolicy
from raicom_core.models import MissionState, OperationResult, PrimitiveOutcome, RunSubstate, SafeHoldEvidence, SafeHoldResult, Task
from raicom_core.runner import MissionRunner
from raicom_core.turntable import TurntableState, TurntableWindowFSM
from raicom_platform.charging import ChargingBackend, ChargingOutcome, ChargingSession


class Safety:
    def __init__(self, confirmed=True): self.confirmed, self.latched = confirmed, False
    def safe_hold(self, deadline): return SafeHoldResult(SafeHoldEvidence.CONFIRMED_FAKE if self.confirmed else SafeHoldEvidence.UNKNOWN)
    def latch_estop(self, deadline): self.latched = True


class SuccessBackend:
    def cancel_active_operation(self): return OperationResult.SUCCEEDED
    def safe_hold(self, deadline): return SafeHoldResult(SafeHoldEvidence.CONFIRMED_FAKE)
    def goto(self, waypoint, deadline): return OperationResult.SUCCEEDED
    def perceive(self, target, deadline): return OperationResult.SUCCEEDED
    def grasp(self, target, deadline): return OperationResult.SUCCEEDED
    def place(self, target, deadline): return OperationResult.SUCCEEDED
    def grasp_evidence(self, target): raise RuntimeError("unused")


class BlockingRecoveryPolicy(RecoveryPolicy):
    def __init__(self):
        super().__init__()
        self.entered, self.release = threading.Event(), threading.Event()
    def handle_failure(self, failure, now_s):
        self.entered.set(); self.release.wait(2)
        return super().handle_failure(failure, now_s)


class RecordingRecoveryExecutor:
    def __init__(self): self.calls = []
    def execute(self, action): self.calls.append(action); return OperationResult.SUCCEEDED


class BlockingBackend:
    def __init__(self): self.entered, self.release = threading.Event(), threading.Event()
    def cancel_active_operation(self): return OperationResult.SUCCEEDED
    def safe_hold(self, deadline): return SafeHoldResult(SafeHoldEvidence.CONFIRMED_FAKE)
    def goto(self, waypoint, deadline): self.entered.set(); self.release.wait(2); return OperationResult.SUCCEEDED
    def perceive(self, target, deadline): return OperationResult.SUCCEEDED
    def grasp(self, target, deadline): return OperationResult.SUCCEEDED
    def place(self, target, deadline): return OperationResult.SUCCEEDED
    def grasp_evidence(self, target): raise RuntimeError("unused")


class BlockingFailureBackend(SuccessBackend):
    """Fails its primitive and blocks either safety action in the failure path."""
    def __init__(self, blocked_action):
        self.blocked_action = blocked_action
        self.entered, self.release = threading.Event(), threading.Event()
        self.cancel_calls, self.goto_calls = 0, 0

    def goto(self, waypoint, deadline):
        self.goto_calls += 1
        return OperationResult.FAILED

    def cancel_active_operation(self):
        self.cancel_calls += 1
        if self.blocked_action == "cancel":
            self.entered.set()
            self.release.wait(2)
        return OperationResult.SUCCEEDED


class BlockingFailureSafety(Safety):
    def __init__(self):
        super().__init__()
        self.entered, self.release = threading.Event(), threading.Event()
        self.safe_hold_calls = 0

    def safe_hold(self, deadline):
        self.safe_hold_calls += 1
        self.entered.set()
        self.release.wait(2)
        return super().safe_hold(deadline)


class BlockingNow:
    """Blocks a specified clock query and can re-enter a read-only controller API."""
    def __init__(self, blocked_call):
        self.blocked_call = blocked_call
        self.entered, self.release = threading.Event(), threading.Event()
        self.calls = 0
        self.controller_hook = None

    def __call__(self):
        self.calls += 1
        if self.calls == self.blocked_call:
            self.entered.set()
            if self.controller_hook is not None:
                self.controller_hook()
            self.release.wait(2)
        return 1.0


class CountingBackend(SuccessBackend):
    def __init__(self): self.goto_calls = 0
    def goto(self, waypoint, deadline):
        self.goto_calls += 1
        return OperationResult.SUCCEEDED


class BlockingTurntable(TurntableWindowFSM):
    """Concrete test double: each controller-invoked accessor can block once."""
    def __init__(self, blocked_accessor, controller_hook=None, blocked_call=1):
        self.blocked_accessor = blocked_accessor
        self.blocked_call = blocked_call
        self.controller_hook = controller_hook
        self.entered, self.release = threading.Event(), threading.Event()
        self.calls = []
        self._state = TurntableState.TARGET_STABLE

    @property
    def state(self):
        self._access("state")
        return self._state

    def _access(self, name):
        self.calls.append(name)
        if name == self.blocked_accessor and self.calls.count(name) == self.blocked_call:
            self.entered.set()
            if self.controller_hook is not None:
                self.controller_hook()
            self.release.wait(2)

    def admits_target(self, target_id, now_s, *, continuation=False):
        self._access("admits_target")
        return True

    def commit(self, target_id, now_s):
        self._access("commit")
        return True

    def mark_pick_done(self, target_id):
        self._access("mark_pick_done")
        return True

    def begin_next_window(self, target_id, now_s):
        self._access("begin_next_window")
        return True


class BlockingCharging(ChargingSession):
    """Concrete test double for all controller-invoked charging accessors."""
    def __init__(self, blocked_accessor, controller_hook=None):
        self.blocked_accessor = blocked_accessor
        self.controller_hook = controller_hook
        self.entered, self.release = threading.Event(), threading.Event()
        self.calls = []
        self.backend = ChargingBackend.SIM
        self._deadline_s = 100.0
        self._outcome = ChargingOutcome.SIM_EVIDENCE_CONFIRMED

    def _access(self, name):
        self.calls.append(name)
        if name == self.blocked_accessor:
            self.entered.set()
            if self.controller_hook is not None:
                self.controller_hook()
            self.release.wait(2)

    @property
    def deadline_s(self):
        self._access("deadline_s")
        return self._deadline_s

    @property
    def outcome(self):
        self._access("outcome")
        return self._outcome

    def check_timeout(self, now_s):
        self._access("check_timeout")
        return False

    def settlement_safe_stop(self):
        self._access("settlement_safe_stop")


def controller(safety=None, backend=None, turntable=None):
    root = Path(tempfile.mkdtemp())
    mission = MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK)
    return SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True), mission, EventJournal(root, "events.jsonl"), safety or Safety(), backend=backend, turntable=turntable)


def test_assignment_is_durable_and_snapshots_are_read_only():
    subject = controller()
    assert subject.assign("work-1", Task("sort", score_on_referee_accept=3), "assign-1")
    snapshot = subject.snapshot()
    assert snapshot.assignment_id == "work-1"
    with pytest.raises(Exception): snapshot.assignment_id = "changed"
    assert subject.runner.assignment_id == "work-1"
    with pytest.raises(AttributeError): subject.mission.state = MissionState.ESTOP


class FaultingJournal(EventJournal):
    def append_once(self, *args):
        raise OSError("disk")


class BarrierJournal(EventJournal):
    def __init__(self, *args):
        super().__init__(*args)
        self.appended, self.release = threading.Event(), threading.Event()
    def append_once(self, *args):
        event = super().append_once(*args)
        if args[1] == "TASK_ASSIGNED":
            self.appended.set()
            self.release.wait(2)
        return event


def test_journal_failure_latches_before_apply():
    root = Path(tempfile.mkdtemp())
    subject = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True),
        MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK),
        FaultingJournal(root, "events.jsonl"), Safety())
    assert not subject.assign("work-1", Task("sort"), "assign-1")
    assert subject.snapshot().mission_state is MissionState.ESTOP


def test_caller_owned_mission_is_copied():
    mission = MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK)
    root = Path(tempfile.mkdtemp()); journal = EventJournal(root, "events.jsonl")
    subject = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True), mission, journal, Safety())
    mission.estop("caller mutation")
    assert subject.assign("work-1", Task("sort"), "assign-1")
    assert subject.snapshot().mission_state is MissionState.RUNNING
    assert len(EventJournal.read(journal.artifact_root, journal.relative_path)) == 1


def test_estop_after_append_before_apply_wins_without_assignment_mutation():
    root = Path(tempfile.mkdtemp()); journal = BarrierJournal(root, "events.jsonl")
    subject = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True),
        MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK), journal, Safety())
    result = []
    thread = threading.Thread(target=lambda: result.append(subject.assign("work", Task("sort"), "assign")))
    thread.start(); assert journal.appended.wait(1)
    subject.estop("between append and apply")
    journal.release.set(); thread.join(2)
    assert result == [False]
    snapshot = subject.snapshot()
    assert snapshot.estop_latched and snapshot.assignment_id is None
    assert snapshot.pending_dispatch_ids == () and snapshot.current_dispatch_id is None


def test_estop_latches_while_step_backend_is_blocked():
    mission = MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK)
    backend = BlockingBackend()
    root = Path(tempfile.mkdtemp())
    subject = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True), mission, EventJournal(root, "events.jsonl"), Safety(), backend=backend)
    assert subject.assign("work-1", Task("sort"), "assign")
    thread = threading.Thread(target=lambda: subject.step("step")); thread.start()
    assert backend.entered.wait(1)
    subject.estop("test")
    assert subject.snapshot().estop_latched
    backend.release.set(); thread.join(2)
    assert subject.snapshot().mission_state is MissionState.ESTOP


def _command_result(result, command):
    try:
        result.append(command())
    except TransitionError:
        result.append(False)


def _assert_estop_returns_while_blocked(subject, entered, release, worker):
    worker.start()
    assert entered.wait(1)
    estop_done = threading.Event()
    estop_thread = threading.Thread(target=lambda: (subject.estop("accessor-race"), estop_done.set()))
    estop_thread.start()
    assert estop_done.wait(1), "estop waited for an external accessor"
    assert worker.is_alive()
    events_at_estop = EventJournal.read(subject._journal.artifact_root, subject._journal.relative_path)
    normal_events_at_estop = [event for event in events_at_estop if event.event_type != "ESTOP"]
    release.set()
    worker.join(2); estop_thread.join(2)
    assert not worker.is_alive() and not estop_thread.is_alive()
    assert subject.snapshot().estop_latched
    events = EventJournal.read(subject._journal.artifact_root, subject._journal.relative_path)
    assert [event for event in events if event.event_type != "ESTOP"] == normal_events_at_estop


def test_turntable_accessors_release_controller_lock_and_estop_wins():
    # state is the external property gate; admits_target covers assignment, precommit, and continuation.
    for path in ("state", "assignment", "precommit", "continuation"):
        turntable = BlockingTurntable("state" if path == "state" else "admits_target", blocked_call=2 if path == "continuation" else 1)
        subject = controller(turntable=turntable)
        result = []
        if path in ("state", "assignment"):
            worker = threading.Thread(target=lambda: _command_result(result, lambda: subject.assign("work", Task("sort", target="target"), "assign", turntable_task=True)))
        else:
            assert subject.assign("work", Task("sort", target="target"), "assign")
            subject._mission.substate = RunSubstate.GRASP
            backend = SuccessBackend()
            subject._runner.backend = backend
            if path == "continuation":
                # Failed backend outcome reaches the continuation admission after ESTOP has won.
                backend.grasp = lambda target, deadline: OperationResult.FAILED
            worker = threading.Thread(target=lambda: _command_result(result, lambda: subject.step("step", turntable_pick=True)))
        _assert_estop_returns_while_blocked(subject, turntable.entered, turntable.release, worker)
        assert result == ([False] if path in ("state", "assignment") else [None])


def test_turntable_commit_mark_done_and_next_window_do_not_transition_after_estop():
    for accessor in ("commit", "mark_pick_done", "begin_next_window"):
        turntable = BlockingTurntable(accessor)
        subject = controller(turntable=turntable, backend=SuccessBackend())
        result = []
        if accessor == "begin_next_window":
            worker = threading.Thread(target=lambda: _command_result(result, lambda: subject.begin_next_turntable_window("next", "target")))
        else:
            assert subject.assign("work", Task("sort", target="target"), "assign")
            subject._mission.substate = RunSubstate.GRASP
            worker = threading.Thread(target=lambda: _command_result(result, lambda: subject.step("step", turntable_pick=True)))
        _assert_estop_returns_while_blocked(subject, turntable.entered, turntable.release, worker)
        assert result == ([False] if accessor == "begin_next_window" else [None])


def test_turntable_reentrant_accessor_can_snapshot_without_deadlock():
    turntable = BlockingTurntable("admits_target")
    subject = controller(turntable=turntable)
    turntable.controller_hook = lambda: subject.snapshot()
    worker = threading.Thread(target=lambda: subject.assign("work", Task("sort", target="target"), "assign", turntable_task=True))
    worker.start(); assert turntable.entered.wait(1)
    turntable.release.set(); worker.join(2)
    assert not worker.is_alive()


def _charging_subject(blocked_accessor):
    charging = BlockingCharging(blocked_accessor)
    root = Path(tempfile.mkdtemp())
    subject = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True),
        MissionCore(10, state=MissionState.SETTLEMENT), EventJournal(root, "events.jsonl"), Safety(), charging=charging,
        now=lambda: 1.0)
    return subject, charging


def test_charging_accessors_release_controller_lock_and_estop_wins():
    # deadline_s is queried before admission; check_timeout, settlement_safe_stop, and outcome share settlement flow.
    for accessor in ("deadline_s", "check_timeout", "settlement_safe_stop", "outcome"):
        subject, charging = _charging_subject(accessor)
        result = []
        worker = threading.Thread(target=lambda: _command_result(result, lambda: subject.settle_charging("charge", 10.0)))
        _assert_estop_returns_while_blocked(subject, charging.entered, charging.release, worker)
        assert result == [False]


def test_charging_settlement_requires_concrete_sim_source():
    fake_subject, fake_session = _charging_subject("never")
    fake_session.backend = ChargingBackend.FAKE
    assert not fake_subject.settle_charging("fake-charge", 10.0)
    sim_subject, _ = _charging_subject("never")
    assert sim_subject.settle_charging("sim-charge", 10.0)


def test_charging_reentrant_accessor_can_estop_without_deadlock():
    subject, charging = _charging_subject("check_timeout")
    charging.controller_hook = lambda: subject.estop("charging-reentrant")
    worker = threading.Thread(target=lambda: subject.settle_charging("charge", 10.0))
    worker.start(); assert charging.entered.wait(1)
    charging.release.set(); worker.join(2)
    assert not worker.is_alive() and subject.snapshot().estop_latched


def _failed_primitive_subject(backend, safety, executor):
    root = Path(tempfile.mkdtemp())
    subject = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True),
        MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK),
        EventJournal(root, "events.jsonl"), safety, backend=backend, recovery_executor=executor)
    assert subject.assign("work", Task("sort"), "assign")
    assert subject.prepare_dispatch(DispatchTask("current", "sort"), start=True) == "current"
    assert subject.prepare_dispatch(DispatchTask("pending", "sort")) == "current"
    return subject


def _assert_failed_primitive_estop_interleave(subject, entered, release, worker, executor):
    before = subject.snapshot()
    before_dispatch = subject.dispatch_snapshot()
    before_events = EventJournal.read(subject._journal.artifact_root, subject._journal.relative_path)
    worker.start()
    try:
        assert entered.wait(1), "failure path did not enter its required external phase"
        subject.estop("failure-path-race")
        terminal = subject.snapshot()
        assert terminal.estop_latched and terminal.mission_state is MissionState.ESTOP
        assert terminal.mission_version == before.mission_version
        assert subject.dispatch_snapshot() == before_dispatch
        assert worker.is_alive(), "step completed before the blocked external phase released"
    finally:
        release.set()
        worker.join(2)
    assert not worker.is_alive()
    assert executor.calls == []
    events = EventJournal.read(subject._journal.artifact_root, subject._journal.relative_path)
    assert [event for event in events if event.event_type != "ESTOP"] == before_events


def test_failed_primitive_cancel_block_does_not_delay_estop_or_apply_failure():
    backend, executor = BlockingFailureBackend("cancel"), RecordingRecoveryExecutor()
    subject = _failed_primitive_subject(backend, Safety(), executor)
    outcomes = []
    worker = threading.Thread(target=lambda: outcomes.append(subject.step("failed-step")))
    _assert_failed_primitive_estop_interleave(subject, backend.entered, backend.release, worker, executor)
    assert outcomes == [None]
    assert backend.cancel_calls == 1 and backend.goto_calls == 1


def test_failed_primitive_safe_hold_block_does_not_delay_estop_or_apply_failure():
    backend, safety, executor = BlockingFailureBackend("never"), BlockingFailureSafety(), RecordingRecoveryExecutor()
    subject = _failed_primitive_subject(backend, safety, executor)
    outcomes = []
    worker = threading.Thread(target=lambda: outcomes.append(subject.step("failed-step")))
    _assert_failed_primitive_estop_interleave(subject, safety.entered, safety.release, worker, executor)
    assert outcomes == [None]
    assert backend.cancel_calls == 1 and safety.safe_hold_calls == 1


def test_blocking_reentrant_clock_does_not_delay_estop_or_invoke_backend():
    now, backend = BlockingNow(blocked_call=2), CountingBackend()
    root = Path(tempfile.mkdtemp())
    subject = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True),
        MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK),
        EventJournal(root, "events.jsonl"), Safety(), backend=backend, now=now)
    assert subject.assign("work", Task("sort"), "assign")
    now.controller_hook = lambda: subject.snapshot()
    outcomes = []
    worker = threading.Thread(target=lambda: _command_result(outcomes, lambda: subject.step("clock-step")))
    before_events = EventJournal.read(subject._journal.artifact_root, subject._journal.relative_path)
    worker.start()
    try:
        assert now.entered.wait(1)
        subject.estop("clock-race")
        assert subject.snapshot().estop_latched
        assert worker.is_alive()
    finally:
        now.release.set()
        worker.join(2)
    assert not worker.is_alive() and outcomes == [False]
    assert backend.goto_calls == 0
    events = EventJournal.read(subject._journal.artifact_root, subject._journal.relative_path)
    assert [event for event in events if event.event_type != "ESTOP"] == before_events


def test_estop_wins_blocking_recovery_policy_query_before_executor_runs():
    policy, executor = BlockingRecoveryPolicy(), RecordingRecoveryExecutor()
    root = Path(tempfile.mkdtemp())
    subject = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True),
        MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK),
        EventJournal(root, "events.jsonl"), Safety(), recovery=policy, recovery_executor=executor)
    worker = threading.Thread(target=lambda: subject.recover("recovery", RecoveryFailure.NAV_FAILED))
    worker.start(); assert policy.entered.wait(1)
    subject.estop("policy-race")
    assert subject.snapshot().estop_latched
    policy.release.set(); worker.join(2)
    assert executor.calls == []
    assert not [event for event in EventJournal.read(subject._journal.artifact_root, subject._journal.relative_path)
                if event.event_type.startswith("RECOVERY_")]


def test_abort_is_durable_before_mutation_and_pending_is_retained():
    subject = controller()
    subject.prepare_dispatch(DispatchTask("current", "x"), start=False)
    subject.prepare_dispatch(DispatchTask("pending", "x"), start=False)
    subject.prepare_dispatch(DispatchTask("start-marker", "x"), start=True)
    # start-marker is current; remaining pending work must remain after abort failure.
    state = subject.request_abort_current("abort")
    assert state is AbortCurrentState.WAITING_FOR_SAFETY_STOP
    assert subject.dispatch_snapshot()[0] == ("pending", "start-marker")
    assert subject.acknowledge_abort_hold("hold", 1.0)
    assert subject.dispatch_snapshot()[0] == ("pending", "start-marker")


def _abort_subject_with_durable_outcome(dispatch_id="work"):
    backend = BlockingBackend()
    subject = controller(backend=backend)
    assert subject.assign("work", Task("sort"), "assign")
    assert subject.prepare_dispatch(DispatchTask(dispatch_id, "sort"), start=True) == dispatch_id
    assert subject.prepare_dispatch(DispatchTask("pending", "sort")) == dispatch_id
    completed = []
    worker = threading.Thread(target=lambda: completed.append(subject.step("primitive")))
    worker.start(); assert backend.entered.wait(1)
    assert subject.request_abort_current("abort") is AbortCurrentState.DEFERRED_PRIMITIVE_IN_PROGRESS
    backend.release.set(); worker.join(2)
    outcome = completed[0]
    assert outcome is not None and outcome.result is OperationResult.SUCCEEDED
    return subject, outcome


def test_abort_acknowledgement_matrix_hold_then_primitive_is_bound_and_durable():
    subject, outcome = _abort_subject_with_durable_outcome()
    assert subject.acknowledge_abort_hold("hold", 1.0)
    assert subject.dispatch_snapshot() == (("pending",), "work", AbortCurrentState.DEFERRED_PRIMITIVE_IN_PROGRESS)
    assert subject.acknowledge_abort_primitive_complete("primitive-ack", outcome)
    assert subject.dispatch_snapshot() == (("pending",), None, AbortCurrentState.NONE)
    # Exact command retries are idempotent and do not release or resume pending work.
    assert subject.acknowledge_abort_primitive_complete("primitive-ack", outcome)
    events = EventJournal.read(subject._journal.artifact_root, subject._journal.relative_path)
    ack = [event for event in events if event.event_type == "ABORT_PRIMITIVE_COMPLETE"]
    assert len(ack) == 1
    primitive_event = next(event for event in events if event.event_type == "PRIMITIVE_OUTCOME")
    assert ack[0].payload == {"assignment_id": "work", "attempt_id": outcome.attempt_id,
                              "result": "SUCCEEDED", "outcome_sequence": primitive_event.sequence,
                              "outcome_hash": primitive_event.event_hash}


def test_abort_acknowledgement_matrix_primitive_then_hold_and_rejects_forged_or_stale_outcomes():
    subject, outcome = _abort_subject_with_durable_outcome()
    # Equal values are not authoritative proof: only controller-produced outcome identity is accepted.
    forged = PrimitiveOutcome(outcome.attempt_id, OperationResult.SUCCEEDED)
    assert not subject.acknowledge_abort_primitive_complete("forged", forged)
    assert not subject.acknowledge_abort_primitive_complete("wrong-result", PrimitiveOutcome(outcome.attempt_id, OperationResult.FAILED))
    assert subject.dispatch_snapshot() == (("pending",), "work", AbortCurrentState.DEFERRED_PRIMITIVE_IN_PROGRESS)
    assert subject.acknowledge_abort_primitive_complete("primitive-ack", outcome)
    assert subject.dispatch_snapshot() == (("pending",), "work", AbortCurrentState.WAITING_FOR_SAFETY_STOP)
    assert subject.acknowledge_abort_hold("hold", 1.0)
    assert subject.dispatch_snapshot() == (("pending",), None, AbortCurrentState.NONE)


def test_abort_ack_rejects_stale_prior_outcome():
    subject = controller(backend=SuccessBackend())
    assert subject.assign("work", Task("sort"), "assign")
    outcome = subject.step("primitive")
    assert outcome is not None and outcome.result is OperationResult.SUCCEEDED
    assert subject.prepare_dispatch(DispatchTask("work", "sort"), start=True) == "work"
    assert subject.request_abort_current("abort") is AbortCurrentState.WAITING_FOR_SAFETY_STOP
    assert not subject.acknowledge_abort_primitive_complete("stale", outcome)
    assert subject.dispatch_snapshot()[1] == "work"


def test_abort_ack_rejects_wrong_assignment_and_non_durable_in_memory_completion():
    # Current dispatch identity is part of the public acknowledgement binding.
    subject, outcome = _abort_subject_with_durable_outcome("other")
    assert not subject.acknowledge_abort_primitive_complete("wrong-assignment", outcome)
    assert subject.dispatch_snapshot()[1] == "other"
    # Before a controller has appended an outcome, a caller-created in-memory
    # completion cannot satisfy the acknowledgement API or mutate current work.
    empty = controller()
    assert empty.prepare_dispatch(DispatchTask("work", "sort"), start=True) == "work"
    assert empty.request_abort_current("abort") is AbortCurrentState.WAITING_FOR_SAFETY_STOP
    assert not empty.acknowledge_abort_primitive_complete("in-memory", PrimitiveOutcome("attempt-1", OperationResult.SUCCEEDED))
    assert empty.dispatch_snapshot()[1] == "work"


def test_duplicate_assignment_is_idempotent_and_reuse_fail_closes():
    subject = controller()
    task = Task("sort")
    assert subject.assign("work", task, "assign")
    assert subject.assign("work", task, "assign")
    # Existing ID in journal is globally reserved even if caller tries a different command.
    subject.estop("end")
    assert subject.snapshot().estop_latched


def test_incomplete_assignment_restart_is_non_resumable_and_exact_replay_is_not_claimed():
    root = Path(tempfile.mkdtemp()); journal = EventJournal(root, "events.jsonl")
    first = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True),
        MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK), journal, Safety())
    full_task = Task("sort", target="bin-a", score_on_referee_accept=3)
    assert first.assign("historic-work", full_task, "old-assign")
    restarted = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True),
        MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK), EventJournal(root, "events.jsonl"), Safety())
    assert not restarted.snapshot().resumable and not restarted.snapshot().admission_open
    with pytest.raises(TransitionError):
        restarted.assign("historic-work", full_task, "old-assign")
    assert not restarted.snapshot().estop_latched


def test_restart_with_durable_estop_reconstructs_terminal_state():
    root = Path(tempfile.mkdtemp()); journal = EventJournal(root, "events.jsonl")
    first = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True),
        MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK), journal, Safety())
    first.estop("durable")
    restarted = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True),
        MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK), EventJournal(root, "events.jsonl"), Safety())
    snapshot = restarted.snapshot()
    assert snapshot.estop_latched and not snapshot.resumable and not snapshot.admission_open
    with pytest.raises(TransitionError):
        restarted.assign("work", Task("sort"), "new-assign")


def test_assignment_id_is_reserved_after_restart():
    root = Path(tempfile.mkdtemp()); journal = EventJournal(root, "events.jsonl")
    first_mission = MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK)
    first = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True), first_mission, journal, Safety())
    assert first.assign("historic-work", Task("sort"), "old-assign")
    restart_mission = MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK)
    restarted = SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True), restart_mission, EventJournal(root, "events.jsonl"), Safety())
    with pytest.raises(TransitionError):
        restarted.assign("historic-work", Task("new-sort"), "new-assign")
    assert not restarted.snapshot().admission_open


def test_initial_score_mirror_mismatch_is_rejected():
    root = Path(tempfile.mkdtemp()); journal = EventJournal(root, "events.jsonl")
    journal.append_once("accepted", "REFEREE_ACCEPTED", {"task_id": "old", "score": 2}, 1.0)
    mission = MissionCore(10, state=MissionState.RUNNING, substate=RunSubstate.SELECT_TASK)
    with pytest.raises(TransitionError): SimIntegrationController(IntegrationAdmission(IntegrationMode.FAKE, True), mission, journal, Safety())


def test_rejected_admission_and_bad_ids_are_rejected():
    with pytest.raises(ValueError): IntegrationAdmission(IntegrationMode.REAL, True)
    subject = controller()
    with pytest.raises(ValueError): subject.assign("work", Task("x"), " ")
    with pytest.raises(ValueError): subject.settle_charging("x", float("nan"))
