"""Pure semantic DISPATCH queue; it intentionally has no execution hooks."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class DispatchTask:
    """Caller-defined semantic work item, not a platform command."""

    task_id: str
    intent: str
    target: Optional[str] = None

    def __post_init__(self) -> None:
        if (not isinstance(self.task_id, str) or not isinstance(self.intent, str)
                or not self.task_id.strip() or not self.intent.strip()
                or len(self.task_id) > 128 or len(self.intent) > 128
                or (self.target is not None and (not isinstance(self.target, str) or not self.target.strip() or len(self.target) > 128))):
            raise ValueError("dispatch task identifiers must be non-empty and <=128")


class AbortCurrentState(str, Enum):
    """Semantic lifecycle for an abort request; no platform action is implied."""

    NONE = "NONE"
    DEFERRED_PRIMITIVE_IN_PROGRESS = "DEFERRED_PRIMITIVE_IN_PROGRESS"
    WAITING_FOR_SAFETY_STOP = "WAITING_FOR_SAFETY_STOP"
    WAITING_FOR_PRIMITIVE_COMPLETION = "WAITING_FOR_PRIMITIVE_COMPLETION"


class DispatchQueue:
    """Queue ownership and lifecycle policy for DISPATCH mode.

    ``start_next`` and ``complete_current`` only update semantic state. An
    external orchestrator must independently admit any platform work. An abort
    remains an intent until the current primitive is complete and safety has
    acknowledged a stop; it never switches tasks immediately.
    """

    def __init__(self) -> None:
        self._pending: List[DispatchTask] = []
        self._current: Optional[DispatchTask] = None
        self._stop_after_current = False
        self._abort_current_state = AbortCurrentState.NONE
        self._primitive_in_progress = False
        self._safety_stop_acknowledged = False

    @property
    def current(self) -> Optional[DispatchTask]:
        return self._current

    @property
    def pending(self) -> Tuple[DispatchTask, ...]:
        return tuple(self._pending)

    @property
    def stop_after_current_requested(self) -> bool:
        return self._stop_after_current

    @property
    def abort_current_state(self) -> AbortCurrentState:
        return self._abort_current_state

    def enqueue(self, task: DispatchTask, position: Optional[int] = None) -> None:
        if not isinstance(task, DispatchTask):
            raise ValueError("queue accepts DispatchTask instances")
        if task.task_id in self._task_ids():
            raise ValueError("dispatch task_id must be unique")
        if position is None:
            self._pending.append(task)
        elif 0 <= position <= len(self._pending):
            self._pending.insert(position, task)
        else:
            raise ValueError("queue position is out of bounds")

    def reorder(self, task_id: str, position: int) -> None:
        if not 0 <= position < len(self._pending):
            raise ValueError("queue position is out of bounds")
        index = self._pending_index(task_id)
        task = self._pending.pop(index)
        self._pending.insert(position, task)

    def abort_pending(self, task_id: Optional[str] = None) -> Tuple[DispatchTask, ...]:
        """Remove one pending task, or all pending tasks; never abort current."""
        if task_id is None:
            aborted = tuple(self._pending)
            self._pending.clear()
            return aborted
        index = self._pending_index(task_id)
        return (self._pending.pop(index),)

    def request_stop_after_current(self) -> None:
        self._stop_after_current = True

    def request_abort_current(self, primitive_in_progress: bool) -> AbortCurrentState:
        """Record abort intent; safety and primitive completion gate task clearing."""
        if self._current is None:
            raise ValueError("there is no current dispatch task")
        if self._abort_current_state is AbortCurrentState.NONE:
            self._primitive_in_progress = primitive_in_progress
            self._safety_stop_acknowledged = False
            self._abort_current_state = (
                AbortCurrentState.DEFERRED_PRIMITIVE_IN_PROGRESS
                if primitive_in_progress else AbortCurrentState.WAITING_FOR_SAFETY_STOP
            )
        return self._abort_current_state

    def acknowledge_safety_stop(self) -> Optional[DispatchTask]:
        """Record safety's stop completion; returns current only when it clears."""
        if self._abort_current_state is AbortCurrentState.NONE:
            raise ValueError("there is no current abort request")
        self._safety_stop_acknowledged = True
        return self._finish_abort_if_ready()

    def acknowledge_primitive_complete(self) -> Optional[DispatchTask]:
        """Record that an in-progress primitive reached its safe completion point."""
        if self._abort_current_state is AbortCurrentState.NONE:
            raise ValueError("there is no current abort request")
        self._primitive_in_progress = False
        if not self._safety_stop_acknowledged:
            self._abort_current_state = AbortCurrentState.WAITING_FOR_SAFETY_STOP
        return self._finish_abort_if_ready()

    def start_next(self) -> Optional[DispatchTask]:
        if self._current is not None:
            raise ValueError("a dispatch task is already current")
        if self._stop_after_current or not self._pending:
            return None
        self._current = self._pending.pop(0)
        return self._current

    def complete_current(self) -> DispatchTask:
        if self._current is None:
            raise ValueError("there is no current dispatch task")
        if self._abort_current_state is not AbortCurrentState.NONE:
            raise ValueError("current abort must complete through safety acknowledgement")
        completed = self._current
        self._current = None
        return completed

    def _finish_abort_if_ready(self) -> Optional[DispatchTask]:
        if self._primitive_in_progress:
            self._abort_current_state = AbortCurrentState.DEFERRED_PRIMITIVE_IN_PROGRESS
            return None
        if not self._safety_stop_acknowledged:
            self._abort_current_state = AbortCurrentState.WAITING_FOR_SAFETY_STOP
            return None
        aborted = self._current
        self._current = None
        self._abort_current_state = AbortCurrentState.NONE
        return aborted

    def _task_ids(self) -> set[str]:
        return {task.task_id for task in self._pending} | ({self._current.task_id} if self._current else set())

    def _pending_index(self, task_id: str) -> int:
        for index, task in enumerate(self._pending):
            if task.task_id == task_id:
                return index
        raise ValueError("task is not pending")
