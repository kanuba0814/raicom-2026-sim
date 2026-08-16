"""Pure SIM/fake charging semantics with a future HCC evidence-adapter seam.

This module has no ROS imports and never emits charger, motion, or stop commands.
It records only caller-supplied evidence; SIM and fake evidence is deliberately
incapable of asserting physical charging.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional, Protocol


class ChargingBackend(str, Enum):
    """Execution context; REAL is intentionally not available in this contract."""

    SIM = "SIM"
    FAKE = "FAKE"
    REAL = "REAL"


class ChargingState(str, Enum):
    UNAVAILABLE = "UNAVAILABLE"
    HW_UNVERIFIED = "HW_UNVERIFIED"
    APPROACH_READY = "APPROACH_READY"
    VERIFYING_CONTACT = "VERIFYING_CONTACT"
    CONTACT_UNKNOWN = "CONTACT_UNKNOWN"
    CONTACT_CONFIRMED = "CONTACT_CONFIRMED"
    CONTACT_FAILED = "CONTACT_FAILED"
    VERIFYING_CURRENT = "VERIFYING_CURRENT"
    CURRENT_UNKNOWN = "CURRENT_UNKNOWN"
    CURRENT_CONFIRMED = "CURRENT_CONFIRMED"
    CURRENT_FAILED = "CURRENT_FAILED"
    TIMED_OUT = "TIMED_OUT"
    CANCELED = "CANCELED"
    E_STOPPED = "E_STOPPED"
    SETTLEMENT_SAFE_STOP = "SETTLEMENT_SAFE_STOP"
    # Compatibility alias for callers of the original, much smaller contract.
    APPROACHING = "APPROACH_READY"
    VERIFYING = "VERIFYING_CURRENT"
    CHARGING = "CURRENT_CONFIRMED"
    FAILED = "CURRENT_FAILED"


class EvidenceStatus(str, Enum):
    UNKNOWN = "UNKNOWN"
    CONFIRMED = "CONFIRMED"
    FAILED = "FAILED"


class VerificationScope(str, Enum):
    """What an observation can prove, never an unbounded charger assertion."""

    SIMULATION = "SIMULATION"
    FAKE_TEST = "FAKE_TEST"
    HCC_PENDING = "HCC_PENDING"


class ChargingOutcome(str, Enum):
    PENDING = "PENDING"
    SIM_EVIDENCE_CONFIRMED = "SIM_EVIDENCE_CONFIRMED"
    CONTACT_UNKNOWN = "CONTACT_UNKNOWN"
    CONTACT_FAILED = "CONTACT_FAILED"
    CURRENT_UNKNOWN = "CURRENT_UNKNOWN"
    CURRENT_FAILED = "CURRENT_FAILED"
    TIMEOUT = "TIMEOUT"
    CANCELED = "CANCELED"
    E_STOP = "E_STOP"
    UNAVAILABLE = "UNAVAILABLE"
    HW_UNVERIFIED = "HW_UNVERIFIED"


@dataclass(frozen=True)
class ChargingEvidence:
    """Identity-bound evidence supplied by a fake/SIM or future HCC adapter.

    ``physical_charge_confirmed`` is rejected for SIM and fake sources. The
    HCC-pending scope also cannot make that assertion until a replacement HCC
    adapter defines and validates the hardware contract.
    """

    source: ChargingBackend
    station_id: str
    target_id: str
    observed_at_s: float
    deadline_s: float
    verification_scope: VerificationScope
    contact: EvidenceStatus = EvidenceStatus.UNKNOWN
    current: EvidenceStatus = EvidenceStatus.UNKNOWN
    physical_charge_confirmed: bool = False

    def __post_init__(self) -> None:
        if not self.station_id.strip() or not self.target_id.strip():
            raise ValueError("station and target identities must be non-empty")
        if not all(math.isfinite(value) for value in (self.observed_at_s, self.deadline_s)):
            raise ValueError("evidence timestamp and deadline must be finite")
        if self.deadline_s < self.observed_at_s:
            raise ValueError("evidence deadline must not precede its timestamp")
        if self.source == ChargingBackend.SIM and self.verification_scope != VerificationScope.SIMULATION:
            raise ValueError("SIM evidence must declare SIMULATION scope")
        if self.source == ChargingBackend.FAKE and self.verification_scope != VerificationScope.FAKE_TEST:
            raise ValueError("fake evidence must declare FAKE_TEST scope")
        if self.physical_charge_confirmed:
            raise ValueError("migration charging evidence cannot assert physical charging")

    @property
    def proves_physical_charging(self) -> bool:
        """Always false until an HCC replacement defines a validated REAL contract."""
        return False


class ChargingEvidenceAdapter(Protocol):
    """HCC replacement seam; Mission Core need not know any charger protocol."""

    def evidence_for(self, station_id: str, target_id: str, deadline_s: float) -> ChargingEvidence:
        """Return structured evidence only; implementations must not command a charger."""


@dataclass(frozen=True)
class StaticChargingEvidenceAdapter:
    """Deterministic SIM/fake adapter for tests and migration exercises."""

    evidence: ChargingEvidence

    def evidence_for(self, station_id: str, target_id: str, deadline_s: float) -> ChargingEvidence:
        if (self.evidence.station_id, self.evidence.target_id, self.evidence.deadline_s) != (
                station_id, target_id, deadline_s):
            raise ValueError("static evidence does not match the requested charging identity or deadline")
        return self.evidence


@dataclass
class ChargingSession:
    """Fail-closed semantic charging session; all stops are non-actuating records."""

    backend: ChargingBackend = ChargingBackend.FAKE
    state: ChargingState = ChargingState.HW_UNVERIFIED
    outcome: ChargingOutcome = ChargingOutcome.HW_UNVERIFIED
    station_id: Optional[str] = None
    target_id: Optional[str] = None
    deadline_s: Optional[float] = None
    attempts: int = 0
    maximum_attempts: int = 2

    def begin_candidate_approach(
        self,
        hardware_contract_verified: bool,
        station_id: str = "unspecified-station",
        target_id: str = "unspecified-target",
        deadline_s: Optional[float] = None,
    ) -> None:
        """Record approach readiness; this does not issue a navigation or charger command."""
        if self.backend == ChargingBackend.REAL or not hardware_contract_verified:
            self.state = ChargingState.HW_UNVERIFIED
            self.outcome = ChargingOutcome.HW_UNVERIFIED
            return
        if self.attempts >= self.maximum_attempts:
            self.state = ChargingState.UNAVAILABLE
            self.outcome = ChargingOutcome.UNAVAILABLE
            return
        if not station_id.strip() or not target_id.strip():
            raise ValueError("station and target identities must be non-empty")
        if deadline_s is not None and not math.isfinite(deadline_s):
            raise ValueError("deadline must be finite when supplied")
        self.attempts += 1
        self.station_id, self.target_id, self.deadline_s = station_id, target_id, deadline_s
        self.state = ChargingState.APPROACH_READY
        self.outcome = ChargingOutcome.PENDING

    def approach_complete(self) -> None:
        if self.state != ChargingState.APPROACH_READY:
            raise RuntimeError("charging approach is not ready")
        self.state = ChargingState.VERIFYING_CONTACT

    def verify_contact(self, evidence: ChargingEvidence) -> None:
        self._validate_evidence(evidence, ChargingState.VERIFYING_CONTACT)
        self.state = {
            EvidenceStatus.UNKNOWN: ChargingState.CONTACT_UNKNOWN,
            EvidenceStatus.CONFIRMED: ChargingState.CONTACT_CONFIRMED,
            EvidenceStatus.FAILED: ChargingState.CONTACT_FAILED,
        }[evidence.contact]
        self.outcome = (ChargingOutcome.PENDING if evidence.contact == EvidenceStatus.CONFIRMED else
                        ChargingOutcome.CONTACT_UNKNOWN if evidence.contact == EvidenceStatus.UNKNOWN else
                        ChargingOutcome.CONTACT_FAILED)

    def begin_current_verification(self) -> None:
        if self.state != ChargingState.CONTACT_CONFIRMED:
            raise RuntimeError("current verification requires confirmed contact")
        self.state = ChargingState.VERIFYING_CURRENT

    def verify_current(self, evidence: ChargingEvidence) -> None:
        self._validate_evidence(evidence, ChargingState.VERIFYING_CURRENT)
        self.state = {
            EvidenceStatus.UNKNOWN: ChargingState.CURRENT_UNKNOWN,
            EvidenceStatus.CONFIRMED: ChargingState.CURRENT_CONFIRMED,
            EvidenceStatus.FAILED: ChargingState.CURRENT_FAILED,
        }[evidence.current]
        self.outcome = (ChargingOutcome.SIM_EVIDENCE_CONFIRMED if evidence.current == EvidenceStatus.CONFIRMED else
                        ChargingOutcome.CURRENT_UNKNOWN if evidence.current == EvidenceStatus.UNKNOWN else
                        ChargingOutcome.CURRENT_FAILED)

    def check_timeout(self, now_s: float) -> bool:
        if not math.isfinite(now_s):
            raise ValueError("current time must be finite")
        if self.deadline_s is not None and now_s >= self.deadline_s and self.outcome == ChargingOutcome.PENDING:
            self.state, self.outcome = ChargingState.TIMED_OUT, ChargingOutcome.TIMEOUT
            return True
        return False

    def cancel(self) -> None:
        self.state, self.outcome = ChargingState.CANCELED, ChargingOutcome.CANCELED

    def emergency_stop(self) -> None:
        self.state, self.outcome = ChargingState.E_STOPPED, ChargingOutcome.E_STOP

    def settlement_safe_stop(self) -> None:
        """Record a required safe settlement; it sends no stop command to any robot."""
        self.state = ChargingState.SETTLEMENT_SAFE_STOP

    def _validate_evidence(self, evidence: ChargingEvidence, expected_state: ChargingState) -> None:
        if self.state != expected_state:
            raise RuntimeError("charging is not in the required verification state")
        if self.backend == ChargingBackend.REAL:
            raise RuntimeError("REAL charging remains HW_UNVERIFIED")
        if (evidence.source != self.backend or evidence.station_id != self.station_id or
                evidence.target_id != self.target_id or evidence.deadline_s != self.deadline_s):
            raise ValueError("evidence does not match this charging session")
