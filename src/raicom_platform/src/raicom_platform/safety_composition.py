"""Platform composition root for the optional :mod:`raicom_safety` coordinator.

Core consumes only its own semantic evidence and latch port.  This module is the
one explicit dependency that validates coordinator authority and provenance.
"""
from __future__ import annotations

from typing import Optional

from raicom_core.models import SafeHoldEvidence, SafeHoldResult
from raicom_safety.safety_coordinator import CoordinatorMode, SafeHoldResult as CoordinatorSafeHoldResult, SafetyCoordinator


class CoordinatorMissionSafetyAdapter:
    """Adapt an authoritative SIM/FAKE coordinator without exposing it to core.

    ``latch_estop`` synchronously installs the authoritative latch, then returns
    immediately while the coordinator's pre-created lane collects evidence.
    """

    def __init__(self, coordinator: SafetyCoordinator) -> None:
        if not isinstance(coordinator, SafetyCoordinator):
            raise TypeError("coordinator must be an authoritative SafetyCoordinator")
        self._coordinator = coordinator
        self._last_latch_error: Optional[BaseException] = None

    @property
    def last_latch_error(self) -> Optional[BaseException]:
        return self._last_latch_error

    def safe_hold(self, deadline_s: float) -> SafeHoldResult:
        try:
            result = self._coordinator.safe_hold(deadline_s)
        except BaseException as error:
            return SafeHoldResult(SafeHoldEvidence.UNCONFIRMED, "coordinator exception: {}:{}".format(type(error).__name__, error))
        # Do not accept lookalikes or bare confirmed booleans.  The only source
        # that can cross this composition boundary is this coordinator's result.
        mode = getattr(self._coordinator, "_mode", None)
        if not isinstance(result, CoordinatorSafeHoldResult) or not result.confirmed:
            return SafeHoldResult(SafeHoldEvidence.UNCONFIRMED, "authoritative coordinator hold unconfirmed")
        if mode is CoordinatorMode.SIM:
            return SafeHoldResult(SafeHoldEvidence.CONFIRMED_SIM, "authoritative coordinator SIM hold")
        if mode is CoordinatorMode.FAKE:
            return SafeHoldResult(SafeHoldEvidence.CONFIRMED_FAKE, "authoritative coordinator FAKE hold")
        return SafeHoldResult(SafeHoldEvidence.UNCONFIRMED, "coordinator mode is not SIM or FAKE")

    def latch_estop(self, deadline_s: float) -> None:
        """Install the authoritative latch now; never wait for ACK evidence."""
        try:
            self._coordinator.latch_estop(deadline_s)
        except BaseException as error:
            self._last_latch_error = error
            raise
