"""Pure official-industrial task planning from a caller-supplied rule profile.

This module is deliberately ROS-independent and never invokes a backend.  Rule
parameters are supplied by the caller's profile; this module does not treat any
downloaded PDF as an official source.  The required profile shape is::

    {"task_order": ["QR", "QR", "QR", "QR",
                    "MATERIAL1_PACKAGE1", "MATERIAL2_PACKAGE2"],
     "tasks": {
        "qr": {"count": 4, "status": "CONFIRMED",
               "score": {"value": 10, "status": "CONFIRMED"}},
        "material1_package1": {"count": 2, "status": "CONFIRMED",
               "score": {"value": 20, "status": "CONFIRMED"}},
        "material2_package2": {"count": 2, "status": "CONFIRMED",
               "score": {"value": 20, "status": "CONFIRMED"}},
    }}

Only ``CONFIRMED`` values are admitted by default.  A SIM-only caller may
explicitly pass additional accepted status labels; the profile values retain
those labels and are never silently promoted.  Missing, unresolved, or
malformed parameters produce blockers rather than guessed tasks.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping, Optional, Tuple

from .models import MissionState, RunSubstate, Task
from .runner import MissionRunner
from .scheduler import SchedulerPolicy, TaskCandidate


OPTIONAL_MANUAL_STRATEGY = "OPTIONAL_MANUAL_STRATEGY"
"""Manual-only dynamic-obstacle strategy marker; never included in ``tasks``."""


@dataclass(frozen=True)
class OrchestrationPlan:
    """The next semantic task, or explicit fail-closed reasons not to issue one."""

    task: Optional[Task]
    blockers: Tuple[str, ...] = ()


class OfficialIndustrialOrchestrator:
    """Build and admit the fixed official-industrial sequence without a new FSM.

    ``MissionCore`` remains the owner of active work, referee gating, recovery,
    score settlement, recharge, and finish states.  This class only maps a
    confirmed profile to an ordered ``Task`` tuple and asks ``MissionRunner`` to
    select its next item when all caller-provided readiness facts are resolved.
    """

    _SPECS = {
        "QR": ("qr", 4, "OFFICIAL_QR_STACK", "OFFICIAL_QR"),
        "MATERIAL1_PACKAGE1": (
            "material1_package1", None,
            "OFFICIAL_CIRCLE_ASSEMBLY", "OFFICIAL_CIRCLE",
        ),
        "MATERIAL2_PACKAGE2": (
            "material2_package2", None,
            "OFFICIAL_TRIANGLE_ASSEMBLY", "OFFICIAL_TRIANGLE",
        ),
    }
    _BASE_READINESS = ("warehouse", "workbench")
    OPTIONAL_MANUAL_STRATEGY = OPTIONAL_MANUAL_STRATEGY

    def __init__(
        self,
        profile: Mapping[str, object],
        *,
        allowed_statuses: Tuple[str, ...] = ("CONFIRMED",),
    ) -> None:
        if (not allowed_statuses or any(
                not isinstance(status, str) or not status.strip()
                for status in allowed_statuses)):
            raise ValueError("allowed_statuses must contain non-empty strings")
        self._allowed_statuses = frozenset(allowed_statuses)
        self._tasks, self._profile_blockers = self._build_tasks(profile)

    @property
    def tasks(self) -> Tuple[Task, ...]:
        """The immutable configured sequence; empty when the profile is blocked."""
        return self._tasks

    @property
    def profile_blockers(self) -> Tuple[str, ...]:
        return self._profile_blockers

    def plan_next(self, mission: object, readiness: Mapping[str, object]) -> OrchestrationPlan:
        """Purely choose the next task from MissionCore state and readiness facts."""
        if self._profile_blockers:
            return OrchestrationPlan(None, self._profile_blockers)
        if not hasattr(mission, "state") or not hasattr(mission, "substate"):
            return OrchestrationPlan(None, ("MISSION_UNAVAILABLE",))
        if mission.state in (MissionState.SETTLEMENT, MissionState.RECHARGE, MissionState.FINISHED):
            return OrchestrationPlan(None, ("MISSION_TERMINAL",))
        if mission.state != MissionState.RUNNING or mission.substate != RunSubstate.SELECT_TASK:
            return OrchestrationPlan(None, ("MISSION_NOT_SELECTING",))
        if getattr(mission, "active_task", None) is not None:
            return OrchestrationPlan(None, ("ACTIVE_TASK",))

        completed = self._completed_count(getattr(mission, "accepted_score", None))
        if completed is None:
            return OrchestrationPlan(None, ("SCORE_NOT_ALIGNED_TO_PLAN",))
        if completed >= len(self._tasks):
            return OrchestrationPlan(None, ("PLAN_EXHAUSTED",))

        task = self._tasks[completed]
        blockers = self._readiness_blockers(task, readiness)
        return OrchestrationPlan(None, blockers) if blockers else OrchestrationPlan(task)

    def admit_next(self, runner: MissionRunner, readiness: Mapping[str, object]) -> OrchestrationPlan:
        """Select one planned task only while the existing runner is selecting work."""
        plan = self.plan_next(runner.mission, readiness)
        if plan.task is None:
            return plan
        selected = runner.select_task(
            [TaskCandidate(plan.task, True, True, 0.0)], SchedulerPolicy.NORMAL
        )
        if selected is None:
            return OrchestrationPlan(None, ("RUNNER_DID_NOT_ADMIT",))
        return OrchestrationPlan(selected)

    def _build_tasks(self, profile: Mapping[str, object]) -> Tuple[Tuple[Task, ...], Tuple[str, ...]]:
        if not isinstance(profile, Mapping):
            return (), ("PROFILE_MISSING",)
        groups = profile.get("tasks")
        if not isinstance(groups, Mapping):
            return (), ("PROFILE_TASKS_MISSING",)
        order = profile.get("task_order")
        if not isinstance(order, list) or not order:
            return (), ("PROFILE_TASK_ORDER_MISSING",)
        if any(item not in self._SPECS for item in order):
            return (), ("PROFILE_TASK_ORDER_INVALID",)

        group_values = {}
        blockers = []
        for order_key, (key, required_count, kind, target_prefix) in self._SPECS.items():
            entry = groups.get(key)
            if not isinstance(entry, Mapping):
                blockers.append("PROFILE_{}_MISSING".format(key.upper()))
                continue
            if entry.get("status") not in self._allowed_statuses:
                blockers.append("PROFILE_{}_STATUS_UNRESOLVED".format(key.upper()))
                continue
            count = entry.get("count")
            if isinstance(count, bool) or not isinstance(count, int) or count < 0:
                blockers.append("PROFILE_{}_COUNT_UNRESOLVED".format(key.upper()))
                continue
            if required_count is not None and count != required_count:
                blockers.append("PROFILE_QR_COUNT_MUST_BE_4")
                continue
            if order.count(order_key) != count:
                blockers.append("PROFILE_{}_ORDER_COUNT_MISMATCH".format(key.upper()))
                continue
            score = entry.get("score")
            if not isinstance(score, Mapping) or score.get("status") not in self._allowed_statuses:
                blockers.append("PROFILE_{}_SCORE_UNRESOLVED".format(key.upper()))
                continue
            value = score.get("value")
            if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
                blockers.append("PROFILE_{}_SCORE_UNRESOLVED".format(key.upper()))
                continue
            group_values[order_key] = (kind, target_prefix, value)
        if blockers:
            return (), tuple(blockers)

        counters = {key: 0 for key in self._SPECS}
        tasks = []
        for order_key in order:
            counters[order_key] += 1
            kind, target_prefix, value = group_values[order_key]
            tasks.append(Task(
                kind,
                "{}_{}".format(target_prefix, counters[order_key]),
                value,
            ))
        return tuple(tasks), ()

    def _completed_count(self, accepted_score: object) -> Optional[int]:
        if isinstance(accepted_score, bool) or not isinstance(accepted_score, int) or accepted_score < 0:
            return None
        total = 0
        if accepted_score == total:
            return 0
        for index, task in enumerate(self._tasks, start=1):
            total += task.score_on_referee_accept
            if accepted_score == total:
                return index
        return None

    def _readiness_blockers(self, task: Task, readiness: Mapping[str, object]) -> Tuple[str, ...]:
        if not isinstance(readiness, Mapping):
            return ("READINESS_MISSING",)
        required = self._BASE_READINESS
        if task.kind in ("OFFICIAL_CIRCLE_ASSEMBLY", "OFFICIAL_TRIANGLE_ASSEMBLY"):
            required += ("turntable",)
        blockers = ["{}_NOT_READY".format(key.upper()) for key in required
                    if readiness.get(key) is not True]
        capabilities = readiness.get("capabilities")
        if not isinstance(capabilities, Mapping) or capabilities.get(task.kind) is not True:
            blockers.append("CAPABILITY_NOT_READY:{}".format(task.kind))
        targets = readiness.get("targets")
        if not isinstance(targets, Mapping) or targets.get(task.target) is not True:
            blockers.append("TARGET_NOT_RESOLVED:{}".format(task.target))
        return tuple(blockers)
