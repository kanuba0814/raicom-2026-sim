"""Pure, bounded recovery policy; it never invokes platform adapters."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Dict, Tuple


class RecoveryFailure(str, Enum):
    NAV_FAILED = "NAV_FAILED"
    TARGET_NOT_FOUND = "TARGET_NOT_FOUND"
    CHARGE_FAILED = "CHARGE_FAILED"


class RecoveryAction(str, Enum):
    CANCEL = "CANCEL"
    CLEAR_COSTMAP = "CLEAR_COSTMAP"
    RETRY_NAVIGATION = "RETRY_NAVIGATION"
    REOBSERVE = "REOBSERVE"
    RETRACT = "RETRACT"
    REALIGN = "REALIGN"
    RETRY_CHARGE = "RETRY_CHARGE"
    ABORT_CURRENT = "ABORT_CURRENT"
    SAFE_HOLD = "SAFE_HOLD"


@dataclass(frozen=True)
class RequestedAction:
    """A semantic action request with a caller-relative deadline."""

    action: RecoveryAction
    deadline_s: float

    def __post_init__(self) -> None:
        if not math.isfinite(self.deadline_s):
            raise ValueError("action deadline must be finite")


@dataclass(frozen=True)
class RecoveryLimits:
    """Per-failure retry budgets. Defaults implement one bounded retry each."""

    navigation_retries: int = 1
    reobserve_attempts: int = 1
    charge_retries: int = 1
    action_timeout_s: float = 10.0

    def __post_init__(self) -> None:
        if any(value < 0 for value in (
            self.navigation_retries, self.reobserve_attempts, self.charge_retries,
        )):
            raise ValueError("recovery retry limits must be non-negative")
        if not math.isfinite(self.action_timeout_s) or self.action_timeout_s < 0.0:
            raise ValueError("action timeout must be finite and non-negative")


@dataclass(frozen=True)
class RecoveryDecision:
    failure: RecoveryFailure
    requested_actions: Tuple[RequestedAction, ...]
    attempts_used: int
    exhausted: bool
    recovery_fault: bool


class RecoveryPolicy:
    """Returns declared recovery work, with no execution or device dependencies.

    A caller records the result and separately decides whether admitted platform
    components may fulfil the declared actions. Target acquisition exhaustion
    requests an abort through the safety path; other exhausted failures request
    a safe hold and latch a recovery fault.
    """

    def __init__(self, limits: RecoveryLimits = RecoveryLimits()) -> None:
        self._limits = limits
        self._attempts: Dict[RecoveryFailure, int] = {
            failure: 0 for failure in RecoveryFailure
        }
        self._faulted: Dict[RecoveryFailure, bool] = {
            failure: False for failure in RecoveryFailure
        }

    def attempts_used(self, failure: RecoveryFailure) -> int:
        return self._attempts[failure]

    def handle_failure(self, failure: RecoveryFailure, now_s: float) -> RecoveryDecision:
        if not isinstance(failure, RecoveryFailure):
            raise ValueError("failure must be a RecoveryFailure")
        if not math.isfinite(now_s):
            raise ValueError("recovery time must be finite")

        deadline = now_s + self._limits.action_timeout_s
        if self._faulted[failure] or self._attempts[failure] >= self._limit_for(failure):
            self._faulted[failure] = True
            exhausted_action = (
                RecoveryAction.ABORT_CURRENT
                if failure is RecoveryFailure.TARGET_NOT_FOUND
                else RecoveryAction.SAFE_HOLD
            )
            return RecoveryDecision(
                failure, (RequestedAction(exhausted_action, deadline),),
                self._attempts[failure], True, True,
            )

        self._attempts[failure] += 1
        actions = self._actions_for(failure, deadline)
        return RecoveryDecision(failure, actions, self._attempts[failure], False, False)

    def _limit_for(self, failure: RecoveryFailure) -> int:
        return {
            RecoveryFailure.NAV_FAILED: self._limits.navigation_retries,
            RecoveryFailure.TARGET_NOT_FOUND: self._limits.reobserve_attempts,
            RecoveryFailure.CHARGE_FAILED: self._limits.charge_retries,
        }[failure]

    @staticmethod
    def _actions_for(failure: RecoveryFailure, deadline: float) -> Tuple[RequestedAction, ...]:
        action_map = {
            RecoveryFailure.NAV_FAILED: (
                RecoveryAction.CANCEL, RecoveryAction.CLEAR_COSTMAP,
                RecoveryAction.RETRY_NAVIGATION,
            ),
            RecoveryFailure.TARGET_NOT_FOUND: (RecoveryAction.REOBSERVE,),
            RecoveryFailure.CHARGE_FAILED: (
                RecoveryAction.RETRACT, RecoveryAction.REALIGN, RecoveryAction.RETRY_CHARGE,
            ),
        }
        return tuple(RequestedAction(action, deadline) for action in action_map[failure])
