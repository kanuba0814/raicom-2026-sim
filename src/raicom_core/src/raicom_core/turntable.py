"""Pure, deterministic admission state machine for turntable pick windows.

The FSM consumes semantic observations only.  It deliberately does not infer a
turntable period or derive timing from images, poses, or coordinates.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


class TurntableState(str, Enum):
    TURN_WAIT = "TURN_WAIT"
    TARGET_APPROACHING = "TARGET_APPROACHING"
    TARGET_IN_WINDOW = "TARGET_IN_WINDOW"
    TARGET_STABLE = "TARGET_STABLE"
    PICK_COMMITTED = "PICK_COMMITTED"
    WINDOW_LOST = "WINDOW_LOST"
    PICK_DONE = "PICK_DONE"


class RoiState(str, Enum):
    UNKNOWN = "UNKNOWN"
    OUTSIDE = "OUTSIDE"
    APPROACHING = "APPROACHING"
    IN_WINDOW = "IN_WINDOW"


@dataclass(frozen=True)
class TurntableObservation:
    """A timestamped semantic observation supplied by perception/estimation."""

    observed_at_s: float
    target_id: Optional[str]
    roi_state: RoiState
    stable: bool
    time_to_exit_s: Optional[float] = None


@dataclass(frozen=True)
class TurntableConfig:
    """Explicit timing policy; these values are not official competition facts."""

    max_observation_age_s: float
    required_execution_budget_s: float
    max_time_to_exit_s: float

    def __post_init__(self) -> None:
        for name, value in (
            ("max_observation_age_s", self.max_observation_age_s),
            ("required_execution_budget_s", self.required_execution_budget_s),
            ("max_time_to_exit_s", self.max_time_to_exit_s),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("%s must be finite and positive" % name)


class TurntableWindowFSM:
    """Fail-closed window admission and pick commitment state machine."""

    def __init__(self, config: TurntableConfig) -> None:
        self.config = config
        self.state = TurntableState.TURN_WAIT
        self.target_id: Optional[str] = None
        self._last_observed_at_s: Optional[float] = None
        self._last_time_to_exit_s: Optional[float] = None
        self._committed_target_id: Optional[str] = None

    @property
    def committed_target_id(self) -> Optional[str]:
        return self._committed_target_id

    def observe(self, observation: TurntableObservation, now_s: float) -> TurntableState:
        """Consume one observation at a caller-provided time without assuming cadence."""
        if not self._is_finite(now_s):
            return self._lose_or_wait()
        if not self._observation_is_current(observation, now_s):
            return self._lose_or_wait()

        # A committed pick is target-bound. Any loss or target replacement waits
        # for an explicit recovery signal rather than silently reopening pickup.
        # Matching observations remain necessary to continue the in-progress pick.
        if self.state == TurntableState.PICK_COMMITTED:
            if (observation.target_id != self._committed_target_id
                    or observation.roi_state != RoiState.IN_WINDOW):
                self.state = TurntableState.WINDOW_LOST
            else:
                self._last_observed_at_s = observation.observed_at_s
                self._last_time_to_exit_s = observation.time_to_exit_s
            return self.state
        if self.state in (TurntableState.WINDOW_LOST, TurntableState.PICK_DONE):
            return self.state

        self._last_observed_at_s = observation.observed_at_s
        self._last_time_to_exit_s = observation.time_to_exit_s
        if observation.roi_state == RoiState.APPROACHING:
            self.target_id = observation.target_id
            self.state = TurntableState.TARGET_APPROACHING
        elif observation.roi_state == RoiState.IN_WINDOW:
            if self.target_id is not None and observation.target_id != self.target_id:
                self.target_id = observation.target_id
                self.state = TurntableState.TARGET_IN_WINDOW
            else:
                self.target_id = observation.target_id
                self.state = (TurntableState.TARGET_STABLE
                              if observation.stable and self._has_execution_budget(observation)
                              else TurntableState.TARGET_IN_WINDOW)
        else:
            self.target_id = None
            self.state = TurntableState.TURN_WAIT
        return self.state

    def commit(self, target_id: str, now_s: float) -> bool:
        """Commit once for the current target; repeated matching commits are no-ops."""
        if self.state == TurntableState.PICK_COMMITTED:
            return target_id == self._committed_target_id
        if (self.state != TurntableState.TARGET_STABLE or target_id != self.target_id
                or not isinstance(target_id, str) or not target_id.strip() or not self._is_finite(now_s)
                or self._last_observed_at_s is None or self._last_time_to_exit_s is None
                or now_s < self._last_observed_at_s
                or now_s - self._last_observed_at_s > self.config.max_observation_age_s
                or self._last_time_to_exit_s - (now_s - self._last_observed_at_s)
                    < self.config.required_execution_budget_s):
            return False
        self._committed_target_id = target_id
        self.state = TurntableState.PICK_COMMITTED
        return True

    def mark_pick_done(self, target_id: str) -> bool:
        if self.state == TurntableState.PICK_DONE:
            return target_id == self._committed_target_id
        if self.state != TurntableState.PICK_COMMITTED or target_id != self._committed_target_id:
            return False
        self.state = TurntableState.PICK_DONE
        return True

    def begin_next_window(self, target_id: str, now_s: float) -> bool:
        """Explicitly close a completed pick and admit observations for its successor.

        This lifecycle transition is deliberately target-bound and is legal only
        after a completed pick.  A lost committed window remains fail-closed and
        cannot be reopened by this API.
        """
        if (self.state != TurntableState.PICK_DONE
                or target_id != self._committed_target_id
                or not isinstance(target_id, str) or not target_id.strip()
                or not self._is_finite(now_s)):
            return False
        self.target_id = None
        self._committed_target_id = None
        self._last_observed_at_s = None
        self._last_time_to_exit_s = None
        self.state = TurntableState.TURN_WAIT
        return True

    # Kept as a compatibility spelling for callers which name the transition
    # after the completed pick rather than the following observation window.
    reset_after_pick = begin_next_window

    def recover(self) -> bool:
        """Acknowledge a lost window without reopening admission automatically."""
        return False

    def admits_target(
            self, target_id: Optional[str], now_s: float, *, continuation: bool = False
    ) -> bool:
        """Return whether a target may start, or continue, at ``now_s``.

        A commitment is not a new-task admission: it can only admit an explicit
        continuation of its matching in-progress action. Every admission also
        requires a current target-matched observation with remaining bounded
        execution budget.
        """
        if (not isinstance(target_id, str) or not target_id.strip()
                or not self._is_finite(now_s) or self.target_id != target_id
                or not self._has_current_execution_budget(now_s)):
            return False
        if self.state == TurntableState.TARGET_STABLE:
            return not continuation
        return (self.state == TurntableState.PICK_COMMITTED
                and self._committed_target_id == target_id
                and continuation)

    def _has_current_execution_budget(self, now_s: float) -> bool:
        if (self._last_observed_at_s is None or self._last_time_to_exit_s is None
                or not self._is_finite(self._last_observed_at_s)
                or not self._is_finite(self._last_time_to_exit_s)
                or now_s < self._last_observed_at_s):
            return False
        elapsed_s = now_s - self._last_observed_at_s
        remaining_s = self._last_time_to_exit_s - elapsed_s
        return (elapsed_s <= self.config.max_observation_age_s
                and self.config.required_execution_budget_s <= remaining_s
                <= self.config.max_time_to_exit_s)

    def _lose_or_wait(self) -> TurntableState:
        if self.state == TurntableState.PICK_COMMITTED:
            self.state = TurntableState.WINDOW_LOST
        elif self.state not in (TurntableState.WINDOW_LOST, TurntableState.PICK_DONE):
            self.target_id = None
            self._last_time_to_exit_s = None
            self.state = TurntableState.TURN_WAIT
        return self.state

    def _observation_is_current(self, observation: TurntableObservation, now_s: float) -> bool:
        if (not isinstance(observation.roi_state, RoiState)
                or not self._is_finite(observation.observed_at_s)
                or observation.observed_at_s > now_s
                or now_s - observation.observed_at_s > self.config.max_observation_age_s
                or (self._last_observed_at_s is not None
                    and observation.observed_at_s < self._last_observed_at_s)
                or not isinstance(observation.target_id, str) or not observation.target_id.strip()
                or observation.roi_state == RoiState.UNKNOWN):
            return False
        return True

    def _has_execution_budget(self, observation: TurntableObservation) -> bool:
        estimate = observation.time_to_exit_s
        return (estimate is not None and self._is_finite(estimate)
                and observation.stable is True
                and self.config.required_execution_budget_s <= estimate <= self.config.max_time_to_exit_s)

    @staticmethod
    def _is_finite(value: float) -> bool:
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
