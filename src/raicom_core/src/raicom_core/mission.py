"""Small explicit mission FSM with mandatory referee gating.

The FSM is intentionally deterministic and ROS-independent so that its safety
and competition-rule transitions can be unit-tested without a simulator.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .models import MissionSnapshot, MissionState, OperationMode, RunSubstate, Task


class TransitionError(ValueError):
    """Raised when an operator or backend asks for an unsafe transition."""


@dataclass
class MissionCore:
    settlement_threshold: int
    mode: OperationMode = OperationMode.AUTO
    state: MissionState = MissionState.BOOT
    substate: Optional[RunSubstate] = None
    active_task: Optional[Task] = None
    accepted_score: int = 0
    retry_count: int = 0
    last_transition_reason: str = "initialized"

    def snapshot(self) -> MissionSnapshot:
        return MissionSnapshot(
            state=self.state,
            substate=self.substate,
            mode=self.mode,
            active_task=self.active_task.kind if self.active_task else None,
            last_transition_reason=self.last_transition_reason,
            retry_count=self.retry_count,
        )

    def _transition(
        self,
        state: MissionState,
        reason: str,
        substate: Optional[RunSubstate] = None,
    ) -> None:
        self.state = state
        self.substate = substate
        self.last_transition_reason = reason

    def precheck_passed(self) -> None:
        if self.state not in (MissionState.BOOT, MissionState.PRECHECK):
            raise TransitionError("precheck may only complete from BOOT or PRECHECK")
        self._transition(MissionState.READY, "precheck_passed")

    def wait_for_start(self) -> None:
        if self.state != MissionState.READY:
            raise TransitionError("start wait requires READY")
        self._transition(MissionState.WAIT_START, "operator_armed")

    def start(self) -> None:
        if self.state != MissionState.WAIT_START:
            raise TransitionError("start requires WAIT_START")
        self._transition(MissionState.RUNNING, "operator_start", RunSubstate.SELECT_TASK)

    def assign_task(self, task: Task) -> None:
        if self.state != MissionState.RUNNING or self.substate != RunSubstate.SELECT_TASK:
            raise TransitionError("a task may only be assigned while selecting work")
        if self.mode == OperationMode.MANUAL:
            raise TransitionError("manual mode cannot schedule autonomous tasks")
        self.active_task = task
        self.retry_count = 0
        self.substate = RunSubstate.NAVIGATE_COARSE
        self.last_transition_reason = "task_assigned"

    def advance_operation(self, expected: RunSubstate, next_substate: RunSubstate, reason: str) -> None:
        """Advance one completed bounded operation without bypassing the FSM."""
        if self.state != MissionState.RUNNING or self.active_task is None:
            raise TransitionError("operation completion requires an active task")
        if self.substate != expected:
            raise TransitionError("operation completion requires {0}".format(expected.value))
        self.substate = next_substate
        self.last_transition_reason = reason

    def operation_failed(self, reason: str) -> None:
        """Record a failed, unavailable, timed-out, or canceled operation.

        Recovery intentionally does not select or retry work automatically.
        """
        if self.state != MissionState.RUNNING or self.active_task is None:
            raise TransitionError("operation failure requires an active task")
        self.substate = RunSubstate.RECOVERY
        self.last_transition_reason = reason

    def physical_task_complete(self) -> None:
        if self.state != MissionState.RUNNING or self.active_task is None:
            raise TransitionError("physical completion requires an active task")
        self.substate = RunSubstate.STOP_FOR_REFEREE
        self.last_transition_reason = "physical_task_complete"

    def stop_unconfirmed(self, reason: str) -> None:
        """Latch progression until E-stop; an existing E-stop is terminal."""
        if self.state == MissionState.ESTOP:
            raise TransitionError("E-stop is terminal")
        self._transition(MissionState.STOP_UNCONFIRMED, reason)

    def robot_stopped_for_referee(self) -> None:
        if self.state != MissionState.RUNNING or self.substate != RunSubstate.STOP_FOR_REFEREE:
            raise TransitionError("referee gate requires STOP_FOR_REFEREE")
        self.substate = RunSubstate.WAIT_REFEREE_VERDICT
        self.last_transition_reason = "referee_requested"

    def referee_verdict(self, accepted: bool) -> None:
        """Apply a verdict using MissionCore's legacy local score projection."""
        score_after = self.accepted_score
        if accepted:
            if self.active_task is None:
                raise TransitionError("referee acceptance requires an active task")
            score_after += self.active_task.score_on_referee_accept
        self.referee_verdict_with_score(accepted, score_after)

    def referee_verdict_with_score(self, accepted: bool, accepted_score_after: int) -> None:
        """Apply a verdict using an already-durable authoritative score projection.

        Integration layers which journal referee acceptance use this narrow seam to
        avoid independently incrementing the core and their journal projection.
        """
        if self.state != MissionState.RUNNING or self.substate != RunSubstate.WAIT_REFEREE_VERDICT:
            raise TransitionError("referee verdict requires WAIT_REFEREE_VERDICT")
        if isinstance(accepted_score_after, bool) or not isinstance(accepted_score_after, int) or accepted_score_after < self.accepted_score:
            raise TransitionError("referee score projection must be a non-decreasing integer")
        if not accepted:
            self.substate = RunSubstate.RECOVERY
            self.last_transition_reason = "referee_rejected"
            return
        if self.active_task is None:
            raise TransitionError("referee acceptance requires an active task")
        self.accepted_score = accepted_score_after
        self.active_task = None
        if self.accepted_score >= self.settlement_threshold:
            self._transition(MissionState.SETTLEMENT, "settlement_threshold_reached")
        else:
            self.substate = RunSubstate.SELECT_TASK
            self.last_transition_reason = "referee_accepted"

    def settlement_complete(self) -> None:
        if self.state != MissionState.SETTLEMENT:
            raise TransitionError("settlement completion requires SETTLEMENT")
        self._transition(MissionState.RECHARGE, "settlement_complete")

    def recharge_complete(self) -> None:
        if self.state != MissionState.RECHARGE:
            raise TransitionError("recharge completion requires RECHARGE")
        self._transition(MissionState.FINISHED, "recharge_complete")

    def safe_stop(self, reason: str) -> None:
        if self.state == MissionState.ESTOP:
            raise TransitionError("E-stop is terminal")
        if self.state == MissionState.STOP_UNCONFIRMED:
            raise TransitionError("unconfirmed stop may only escalate to E-stop")
        self._transition(MissionState.SAFE_STOP, reason)

    def estop(self, reason: str = "estop_latched") -> None:
        self._transition(MissionState.ESTOP, reason)

    def select_mode(self, target_mode: OperationMode, safety_paused: bool) -> None:
        if target_mode == self.mode:
            return
        if target_mode == OperationMode.MANUAL and not safety_paused:
            raise TransitionError("manual mode requires a verified safety pause")
        if self.state in (MissionState.STOP_UNCONFIRMED, MissionState.ESTOP):
            raise TransitionError("mode cannot change while stop is unresolved or E-stop is latched")
        self.mode = target_mode
        self.last_transition_reason = "mode_changed"
