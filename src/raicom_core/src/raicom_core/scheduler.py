"""Small deterministic safe/normal scheduler; no ROS or vendor calls."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Iterable, Optional

from .models import Task
from .turntable import TurntableWindowFSM


class SchedulerPolicy(str, Enum):
    SAFE = "SAFE"
    NORMAL = "NORMAL"


@dataclass(frozen=True)
class TaskCandidate:
    task: Task
    capability_ready: bool
    perception_ready: bool
    estimated_risk: float
    priority: int = 0
    requires_turntable_window: bool = False
    is_turntable_continuation: bool = False

    def is_admitted(
            self, turntable_fsm: Optional[TurntableWindowFSM], now_s: Optional[float]
    ) -> bool:
        """Apply optional turntable admission without changing unrelated tasks."""
        if not self.requires_turntable_window:
            return True
        return (turntable_fsm is not None and now_s is not None
                and turntable_fsm.admits_target(
                    self.task.target, now_s, continuation=self.is_turntable_continuation))


class MissionScheduler:
    def select(
            self,
            candidates: Iterable[TaskCandidate],
            policy: SchedulerPolicy,
            turntable_fsm: Optional[TurntableWindowFSM] = None,
            now_s: Optional[float] = None,
    ) -> Optional[Task]:
        eligible = [candidate for candidate in candidates
                    if (candidate.capability_ready and candidate.perception_ready
                        and candidate.is_admitted(turntable_fsm, now_s))]
        if not eligible:
            return None
        if policy == SchedulerPolicy.SAFE:
            eligible = [candidate for candidate in eligible if candidate.estimated_risk <= 0.25]
            if not eligible:
                return None
        selected = min(eligible, key=lambda item: (
            item.estimated_risk if policy == SchedulerPolicy.SAFE else -item.priority,
            -item.priority,
            item.task.kind,
        ))
        return selected.task
