"""Mode transition policy shared by AUTO, DISPATCH, and MANUAL controls."""
from __future__ import annotations

from .models import OperationMode
from .mission import TransitionError


class ModeManager:
    """Enforces that manual control cannot bypass a verified safe pause."""

    def __init__(self, initial: OperationMode = OperationMode.AUTO) -> None:
        self._mode = initial

    @property
    def mode(self) -> OperationMode:
        return self._mode

    def switch(
        self,
        target: OperationMode,
        *,
        primitive_safe_point: bool,
        safety_paused: bool,
        precheck_complete: bool = False,
    ) -> OperationMode:
        if target == self._mode:
            return self._mode
        if not primitive_safe_point:
            raise TransitionError("mode change requires primitive completion or a safe pause point")
        if target == OperationMode.MANUAL and not safety_paused:
            raise TransitionError("manual control requires safety pause")
        if self._mode == OperationMode.MANUAL and target == OperationMode.AUTO and not precheck_complete:
            raise TransitionError("return to AUTO requires a completed precheck")
        self._mode = target
        return self._mode
