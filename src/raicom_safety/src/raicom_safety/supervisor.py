"""Safety state and command-authorisation logic.

This ROS-independent core is the single policy authority. The future ROS gate
may publish final cmd_vel only after this class permits the command.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum


class CommandSource(str, Enum):
    NAVIGATION = "NAVIGATION"
    RELATIVE_MOVE = "RELATIVE_MOVE"
    MANUAL = "MANUAL"


@dataclass(frozen=True)
class SafetyState:
    soft_estop_latched: bool
    hard_estop_latched: bool
    localization_valid: bool
    watchdog_healthy: bool
    arm_extended: bool

    @property
    def motion_permitted(self) -> bool:
        return not self.soft_estop_latched and not self.hard_estop_latched and self.watchdog_healthy


class SafetySupervisor:
    """Conservative command gate policy; all invalid conditions deny motion."""

    def __init__(self) -> None:
        self._soft_estop_latched = False
        self._hard_estop_latched = False
        self._localization_valid = False
        self._watchdog_healthy = False
        self._arm_extended = False

    def snapshot(self) -> SafetyState:
        return SafetyState(
            soft_estop_latched=self._soft_estop_latched,
            hard_estop_latched=self._hard_estop_latched,
            localization_valid=self._localization_valid,
            watchdog_healthy=self._watchdog_healthy,
            arm_extended=self._arm_extended,
        )

    def update_inputs(self, *, localization_valid: bool, watchdog_healthy: bool, arm_extended: bool) -> None:
        self._localization_valid = localization_valid
        self._watchdog_healthy = watchdog_healthy
        self._arm_extended = arm_extended

    def soft_estop(self) -> None:
        self._soft_estop_latched = True

    def hard_estop(self) -> None:
        self._hard_estop_latched = True

    def acknowledge_soft_estop(self) -> bool:
        if self._hard_estop_latched:
            return False
        self._soft_estop_latched = False
        return True

    def clear_hard_estop_from_hardware(self) -> None:
        """Called only after a verified external hard-E-stop release."""
        self._hard_estop_latched = False

    def permits(self, source: CommandSource, linear_speed: float, angular_speed: float) -> bool:
        state = self.snapshot()
        if not state.motion_permitted:
            return False
        if source in (CommandSource.NAVIGATION, CommandSource.RELATIVE_MOVE) and not state.localization_valid:
            return False
        # Conservative V1 restriction: an extended arm allows only stationary
        # manipulation; no base motion is permitted until it is retracted.
        if state.arm_extended and (linear_speed != 0.0 or angular_speed != 0.0):
            return False
        return True
