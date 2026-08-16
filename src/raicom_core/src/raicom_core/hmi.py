"""Pure HMI command authorization; presentation remains replaceable."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
from .models import OperationMode


class HmiAction(str, Enum):
    START = "START"
    PAUSE = "PAUSE"
    ESTOP = "ESTOP"
    REFEREE_ACCEPT = "REFEREE_ACCEPT"
    REFEREE_REJECT = "REFEREE_REJECT"
    CALIBRATE = "CALIBRATE"


@dataclass(frozen=True)
class HmiRequest:
    action: HmiAction
    mode: OperationMode

    def authorized(self, safety_paused: bool, referee_waiting: bool) -> bool:
        if self.action == HmiAction.ESTOP:
            return True
        if self.action in (HmiAction.REFEREE_ACCEPT, HmiAction.REFEREE_REJECT):
            return referee_waiting
        if self.action == HmiAction.CALIBRATE:
            return self.mode == OperationMode.MANUAL and safety_paused
        if self.action == HmiAction.START:
            return self.mode in (OperationMode.AUTO, OperationMode.DISPATCH) and safety_paused
        return True
