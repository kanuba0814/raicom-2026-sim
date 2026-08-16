"""ROS-independent domain types for the 2026 RAICOM mission system.

These types intentionally contain no Gazebo model names, ROS topic names, or
vendor service names. ROS integration belongs to platform/bringup packages.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


class OperationResult(str, Enum):
    """Concrete result of one semantic backend operation."""

    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"
    UNAVAILABLE = "UNAVAILABLE"
    NOT_CONFIGURED = "NOT_CONFIGURED"
    TIMEOUT = "TIMEOUT"
    CANCELED = "CANCELED"
    SAFETY_DENIED = "SAFETY_DENIED"


class OperationMode(str, Enum):
    AUTO = "AUTO"
    DISPATCH = "DISPATCH"
    MANUAL = "MANUAL"


class MissionState(str, Enum):
    BOOT = "BOOT"
    PRECHECK = "PRECHECK"
    READY = "READY"
    WAIT_START = "WAIT_START"
    RUNNING = "RUNNING"
    STOP_UNCONFIRMED = "STOP_UNCONFIRMED"
    SETTLEMENT = "SETTLEMENT"
    RECHARGE = "RECHARGE"
    FINISHED = "FINISHED"
    PAUSED = "PAUSED"
    SAFE_STOP = "SAFE_STOP"
    ESTOP = "ESTOP"
    FAULT = "FAULT"


class SafeHoldEvidence(str, Enum):
    """Source-qualified safe-hold evidence accepted by MissionRunner.

    REAL is deliberately absent: this semantic test boundary cannot establish
    physical stop proof.  Unknown, timeout, and malformed values fail closed.
    """

    CONFIRMED_FAKE = "CONFIRMED_FAKE"
    CONFIRMED_SIM = "CONFIRMED_SIM"
    UNCONFIRMED = "UNCONFIRMED"
    TIMEOUT = "TIMEOUT"
    UNKNOWN = "UNKNOWN"
    MALFORMED = "MALFORMED"


@dataclass(frozen=True)
class SafeHoldResult:
    """Core semantic safe-hold evidence, qualified as SIM or fake only.

    Confirmation is intentionally derived only from this evidence enum.  Core
    does not consume coordinator acknowledgements, bare booleans, or REAL stop
    assertions; the platform composition root must validate those first.
    """

    evidence: SafeHoldEvidence
    detail: str = ""

    @property
    def confirmed(self) -> bool:
        return self.evidence in (SafeHoldEvidence.CONFIRMED_FAKE, SafeHoldEvidence.CONFIRMED_SIM)


class RunSubstate(str, Enum):
    SELECT_TASK = "SELECT_TASK"
    NAVIGATE_COARSE = "NAVIGATE_COARSE"
    ALIGN_STATION = "ALIGN_STATION"
    PERCEIVE = "PERCEIVE"
    PREPARE_MANIPULATOR = "PREPARE_MANIPULATOR"
    GRASP = "GRASP"
    VERIFY_GRASP = "VERIFY_GRASP"
    TRANSPORT = "TRANSPORT"
    ALIGN_TARGET = "ALIGN_TARGET"
    PLACE_OR_ASSEMBLE = "PLACE_OR_ASSEMBLE"
    VERIFY_LOCAL_RESULT = "VERIFY_LOCAL_RESULT"
    STOP_FOR_REFEREE = "STOP_FOR_REFEREE"
    WAIT_REFEREE_VERDICT = "WAIT_REFEREE_VERDICT"
    UPDATE_SCORE = "UPDATE_SCORE"
    RECOVERY = "RECOVERY"


class GraspState(str, Enum):
    """Verified grasp state; UNKNOWN always fails closed."""

    CONFIRMED = "CONFIRMED"
    FAILED = "FAILED"
    UNKNOWN = "UNKNOWN"


@dataclass(frozen=True)
class GraspEvidence:
    """Immutable, target-bound grasp observation from an identified source.

    Evidence is valid only when it is CONFIRMED, names the requested target, and
    is neither expired nor older than the caller's configured maximum age.
    """

    state: GraspState
    target_id: str
    observed_at_s: float
    source: str
    detail: str = ""
    deadline_s: Optional[float] = None

    def __post_init__(self) -> None:
        import math
        if not self.target_id.strip() or not self.source.strip():
            raise ValueError("grasp evidence target_id and source must be non-empty")
        if not math.isfinite(self.observed_at_s):
            raise ValueError("grasp evidence observed_at_s must be finite")
        if self.deadline_s is not None and (not math.isfinite(self.deadline_s) or self.deadline_s < self.observed_at_s):
            raise ValueError("grasp evidence deadline must be finite and not precede observation")

    def confirms(self, target_id: str, now_s: float, max_age_s: float) -> bool:
        import math
        if not math.isfinite(now_s) or not math.isfinite(max_age_s) or max_age_s < 0.0:
            return False
        return (self.state == GraspState.CONFIRMED and self.target_id == target_id
                and now_s >= self.observed_at_s and now_s - self.observed_at_s <= max_age_s
                and (self.deadline_s is None or now_s <= self.deadline_s))


class VerificationLevel(str, Enum):
    NOT_IMPLEMENTED = "NOT_IMPLEMENTED"
    SIM_TESTED = "SIM_TESTED"
    INTERFACE_COMPATIBLE = "INTERFACE_COMPATIBLE"
    HW_UNVERIFIED = "HW_UNVERIFIED"
    REAL_TESTED = "REAL_TESTED"
    FIELD_VALIDATED = "FIELD_VALIDATED"


@dataclass(frozen=True)
class MissionSnapshot:
    state: MissionState
    substate: Optional[RunSubstate]
    mode: OperationMode
    active_task: Optional[str]
    last_transition_reason: str
    retry_count: int


@dataclass(frozen=True)
class Task:
    """A semantic task request; execution mapping remains outside Mission Core."""

    kind: str
    target: Optional[str] = None
    score_on_referee_accept: int = 0


@dataclass(frozen=True)
class AssignedWork:
    """Application-owned identity bound to an otherwise unchanged semantic task."""

    task_id: str
    task: Task
    dispatch_task_id: Optional[str] = None

    def __post_init__(self) -> None:
        if not isinstance(self.task_id, str) or not self.task_id.strip() or len(self.task_id) > 128:
            raise ValueError("task_id must be non-empty and at most 128 characters")
        if not isinstance(self.task, Task):
            raise TypeError("task must be a Task")
        if self.dispatch_task_id is not None:
            if (not isinstance(self.dispatch_task_id, str) or not self.dispatch_task_id.strip()
                    or len(self.dispatch_task_id) > 128):
                raise ValueError("dispatch_task_id must be non-empty and at most 128 characters")


class MissionPrimitive(str, Enum):
    NAVIGATE_PICKUP = "NAVIGATE_PICKUP"
    PERCEIVE = "PERCEIVE"
    GRASP = "GRASP"
    VERIFY_GRASP = "VERIFY_GRASP"
    NAVIGATE_DROPOFF = "NAVIGATE_DROPOFF"
    PLACE = "PLACE"
    HOLD_FOR_REFEREE = "HOLD_FOR_REFEREE"


@dataclass(frozen=True)
class PrimitiveAttempt:
    """One immutable, assignment-bound primitive authorization."""

    attempt_id: str
    kind: MissionPrimitive
    expected_substate: RunSubstate
    deadline_s: float
    target_id: str
    assignment_id: str

    def __post_init__(self) -> None:
        for name, value in (("attempt_id", self.attempt_id), ("target_id", self.target_id),
                            ("assignment_id", self.assignment_id)):
            if not isinstance(value, str) or not value.strip() or len(value) > 128:
                raise ValueError("%s must be non-empty and at most 128 characters" % name)
        if not isinstance(self.kind, MissionPrimitive) or not isinstance(self.expected_substate, RunSubstate):
            raise TypeError("kind and expected_substate must be concrete enums")
        if not isinstance(self.deadline_s, (int, float)) or isinstance(self.deadline_s, bool) or not math.isfinite(self.deadline_s):
            raise ValueError("deadline_s must be finite")


@dataclass(frozen=True)
class PrimitiveOutcome:
    """Result returned by exactly one primitive execution; advancement is separate."""

    attempt_id: str
    result: OperationResult
    detail: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.attempt_id, str) or not self.attempt_id.strip() or len(self.attempt_id) > 128:
            raise ValueError("attempt_id must be non-empty and at most 128 characters")
        if not isinstance(self.result, OperationResult):
            raise TypeError("result must be an OperationResult")
