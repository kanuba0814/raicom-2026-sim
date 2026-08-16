"""Pure-Python shadow implementation of ADR-001 stop coordination.

This module deliberately has no ROS imports and rejects REAL use.  It provides
an auditable seam for SIM/FAKE stop adapters; it is not an actuator transport.
"""
from __future__ import annotations

import math
import queue
import threading
import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from types import MappingProxyType
from typing import Callable, Mapping, Optional, Tuple


class StopKind(str, Enum):
    SAFE_HOLD = "SAFE_HOLD"
    ESTOP = "ESTOP"


class AckStatus(str, Enum):
    ACKED = "ACKED"
    FAILED = "FAILED"
    TIMEOUT = "TIMEOUT"
    UNKNOWN = "UNKNOWN"


class CoordinatorState(str, Enum):
    OPEN = "OPEN"
    HOLDING = "HOLDING"
    HELD = "HELD"
    HOLD_FAULT = "HOLD_FAULT"
    ESTOP_LATCHED = "ESTOP_LATCHED"
    RESETTING = "RESETTING"


class CoordinatorMode(str, Enum):
    SHADOW = "SHADOW"
    SIM = "SIM"
    FAKE = "FAKE"
    REAL = "REAL"


def _finite_positive(value: float, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(name + " must be a finite positive number")
    value = float(value)
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(name + " must be a finite positive number")
    return value


def _target(value: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError("target must be a non-empty string")
    return value


@dataclass(frozen=True, order=True)
class TrustedStopTarget:
    """Immutable stop target and the actuator domains its ACK covers."""
    name: str
    actuator_domains: Tuple[str, ...]

    def __post_init__(self) -> None:
        _target(self.name)
        if not isinstance(self.actuator_domains, tuple) or not self.actuator_domains:
            raise ValueError("actuator_domains must be a non-empty tuple")
        if any(not isinstance(domain, str) or not domain.strip() for domain in self.actuator_domains):
            raise ValueError("actuator_domains must contain non-empty strings")
        if tuple(sorted(self.actuator_domains)) != self.actuator_domains or len(set(self.actuator_domains)) != len(self.actuator_domains):
            raise ValueError("actuator_domains must be sorted and unique")


@dataclass(frozen=True)
class StopToken:
    coordinator_instance_id: str
    kind: StopKind
    sequence: int
    deadline_s: float
    estop_epoch: int

    def __post_init__(self) -> None:
        if not isinstance(self.coordinator_instance_id, str) or not self.coordinator_instance_id:
            raise ValueError("coordinator_instance_id must be non-empty")
        if not isinstance(self.kind, StopKind):
            raise ValueError("kind must be StopKind")
        if isinstance(self.sequence, bool) or not isinstance(self.sequence, int) or self.sequence <= 0:
            raise ValueError("sequence must be a positive integer")
        _finite_positive(self.deadline_s, "deadline_s")
        if isinstance(self.estop_epoch, bool) or not isinstance(self.estop_epoch, int) or self.estop_epoch < 0:
            raise ValueError("estop_epoch must be a non-negative integer")


@dataclass(frozen=True)
class AdapterStopAck:
    target: TrustedStopTarget
    token: StopToken
    status: AckStatus
    observed_at_s: float
    detail: str = ""

    def __post_init__(self) -> None:
        if not isinstance(self.target, TrustedStopTarget):
            raise ValueError("target must be TrustedStopTarget")
        if not isinstance(self.token, StopToken):
            raise ValueError("token must be StopToken")
        if not isinstance(self.status, AckStatus):
            raise ValueError("status must be AckStatus")
        if isinstance(self.observed_at_s, bool) or not isinstance(self.observed_at_s, (int, float)) or not math.isfinite(self.observed_at_s) or self.observed_at_s < 0.0:
            raise ValueError("observed_at_s must be a finite non-negative number")
        if not isinstance(self.detail, str):
            raise ValueError("detail must be a string")


def _validate_result_vector(token: StopToken, acknowledgements: Tuple[AdapterStopAck, ...],
                            trusted_targets: Tuple[TrustedStopTarget, ...]) -> None:
    if not isinstance(acknowledgements, tuple) or not acknowledgements:
        raise ValueError("acknowledgements must be a non-empty tuple")
    if not isinstance(trusted_targets, tuple) or not trusted_targets:
        raise ValueError("trusted_targets must be a non-empty tuple")
    if tuple(sorted(trusted_targets)) != trusted_targets or len(set(trusted_targets)) != len(trusted_targets):
        raise ValueError("trusted_targets must be sorted and unique")
    targets = tuple(ack.target for ack in acknowledgements)
    if targets != trusted_targets:
        raise ValueError("acknowledgements must exactly cover trusted targets")
    if any(ack.token != token for ack in acknowledgements):
        raise ValueError("acknowledgement token mismatch")


def _confirmed_before_deadline(token: StopToken, acknowledgements: Tuple[AdapterStopAck, ...]) -> bool:
    return all(ack.status is AckStatus.ACKED and ack.observed_at_s <= token.deadline_s for ack in acknowledgements)


@dataclass(frozen=True)
class SafeHoldResult:
    token: StopToken
    acknowledgements: Tuple[AdapterStopAck, ...]
    trusted_targets: Tuple[TrustedStopTarget, ...]
    observed_estop_epoch: int
    confirmed: bool = field(init=False)

    def __post_init__(self) -> None:
        if self.token.kind is not StopKind.SAFE_HOLD:
            raise ValueError("SafeHoldResult requires a SAFE_HOLD token")
        _validate_result_vector(self.token, self.acknowledgements, self.trusted_targets)
        if isinstance(self.observed_estop_epoch, bool) or not isinstance(self.observed_estop_epoch, int) or self.observed_estop_epoch < 0:
            raise ValueError("observed_estop_epoch must be a non-negative integer")
        object.__setattr__(self, "confirmed", self.observed_estop_epoch == self.token.estop_epoch and _confirmed_before_deadline(self.token, self.acknowledgements))


@dataclass(frozen=True)
class EmergencyStopResult:
    token: StopToken
    acknowledgements: Tuple[AdapterStopAck, ...]
    trusted_targets: Tuple[TrustedStopTarget, ...]
    estop_epoch: int
    confirmed: bool = field(init=False)

    def __post_init__(self) -> None:
        if self.token.kind is not StopKind.ESTOP:
            raise ValueError("EmergencyStopResult requires an ESTOP token")
        _validate_result_vector(self.token, self.acknowledgements, self.trusted_targets)
        if self.estop_epoch != self.token.estop_epoch or self.estop_epoch <= 0:
            raise ValueError("estop_epoch must match a positive token epoch")
        object.__setattr__(self, "confirmed", _confirmed_before_deadline(self.token, self.acknowledgements))


StopAdapter = Callable[[StopToken], AdapterStopAck]


class _Lane:
    """One pre-created capacity-one lane; timeout/full lanes are permanently quarantined."""
    def __init__(self, target: TrustedStopTarget, adapter: StopAdapter, clock: Callable[[], float]) -> None:
        self.target = target
        self._adapter = adapter
        self._clock = clock
        self._requests: "queue.Queue[object]" = queue.Queue(maxsize=1)
        self._lock = threading.Lock()
        self._quarantined = False
        self._closed = False
        self._thread = threading.Thread(target=self._run, name="safety-stop-" + target.name, daemon=True)
        self._thread.start()

    def submit(self, token: StopToken) -> Optional["queue.Queue[object]"]:
        reply: "queue.Queue[object]" = queue.Queue(maxsize=1)
        with self._lock:
            if self._closed or self._quarantined:
                return None
            try:
                self._requests.put_nowait((token, reply))
            except queue.Full:
                self._quarantine_locked()
                return None
        return reply

    def quarantine(self) -> None:
        with self._lock:
            self._quarantine_locked()

    def _quarantine_locked(self) -> None:
        self._quarantined = True
        self._drain_locked()

    def _drain_locked(self) -> None:
        while True:
            try:
                self._requests.get_nowait()
            except queue.Empty:
                return

    def _run(self) -> None:
        while True:
            item = self._requests.get()
            if item is None:
                return
            token, reply = item
            try:
                expired = self._clock() >= token.deadline_s
            except BaseException as error:
                reply.put_nowait(error)
                self.quarantine()
                continue
            with self._lock:
                if self._closed or self._quarantined or expired:
                    self._quarantine_locked()
                    continue
            try:
                reply.put_nowait(self._adapter(token))
            except BaseException as error:
                reply.put_nowait(error)

    def shutdown(self, timeout_s: float) -> None:
        with self._lock:
            self._closed = True
            self._quarantined = True
            self._drain_locked()
            try:
                self._requests.put_nowait(None)
            except queue.Full:  # A worker can take an item immediately after drain.
                pass
        self._thread.join(timeout_s)


class _CoordinatorFanoutLane:
    """Single, pre-created ESTOP fanout worker; it never creates escalation threads."""
    def __init__(self, coordinator: "SafetyCoordinator") -> None:
        self._coordinator = coordinator
        self._requests: "queue.Queue[Optional[StopToken]]" = queue.Queue(maxsize=1)
        self._closed = False
        self._lock = threading.Lock()
        self._thread = threading.Thread(target=self._run, name="safety-estop-fanout", daemon=True)
        self._thread.start()

    def submit(self, token: StopToken) -> bool:
        with self._lock:
            if self._closed:
                return False
            try:
                self._requests.put_nowait(token)
            except queue.Full:
                return False
        return True

    def _run(self) -> None:
        while True:
            token = self._requests.get()
            if token is None:
                return
            self._coordinator._complete_estop(token)

    def shutdown(self, timeout_s: float) -> None:
        with self._lock:
            self._closed = True
            try:
                self._requests.put_nowait(None)
            except queue.Full:
                pass
        self._thread.join(timeout_s)


class SafetyCoordinator:
    """Fixed-registry, bounded-fanout coordinator restricted to shadow/SIM/fake."""
    def __init__(self, adapters: Mapping[TrustedStopTarget, StopAdapter], mode: CoordinatorMode = CoordinatorMode.SHADOW,
                 clock: Callable[[], float] = time.monotonic, instance_id: Optional[str] = None) -> None:
        if not isinstance(mode, CoordinatorMode) or mode is CoordinatorMode.REAL:
            raise ValueError("SafetyCoordinator permits SHADOW, SIM, or FAKE only")
        if not adapters:
            raise ValueError("at least one trusted stop target is required")
        if not callable(clock):
            raise ValueError("clock must be callable")
        validated = {}
        for target, adapter in adapters.items():
            if not isinstance(target, TrustedStopTarget):
                raise ValueError("trusted targets must be TrustedStopTarget")
            if not callable(adapter):
                raise ValueError("adapter for " + target.name + " must be callable")
            validated[target] = adapter
        if len(validated) != len(adapters):
            raise ValueError("trusted target descriptors must be unique")
        self._mode = mode
        self._clock = clock
        self._targets = tuple(sorted(validated))
        self._registry = MappingProxyType(dict(validated))
        self._lanes = {target: _Lane(target, validated[target], clock) for target in self._targets}
        self._lock = threading.RLock()
        self._state = CoordinatorState.OPEN
        self._admission_open = True
        self._sequence = 0
        self._estop_epoch = 0
        self._instance_id = instance_id or str(uuid.uuid4())
        if not isinstance(self._instance_id, str) or not self._instance_id:
            raise ValueError("instance_id must be non-empty")
        self._estop_complete = threading.Event()
        self._estop_result: Optional[EmergencyStopResult] = None
        self._active_estop_token: Optional[StopToken] = None
        self._shutdown = False
        # This is the only coordinator-level fanout worker and is deliberately
        # created during construction, not during an escalation.
        self._estop_fanout_lane = _CoordinatorFanoutLane(self)

    @property
    def coordinator_instance_id(self) -> str:
        return self._instance_id

    @property
    def trusted_targets(self) -> Tuple[TrustedStopTarget, ...]:
        return self._targets

    @property
    def state(self) -> CoordinatorState:
        with self._lock:
            return self._state

    @property
    def estop_epoch(self) -> int:
        with self._lock:
            return self._estop_epoch

    def admission_open(self) -> bool:
        with self._lock:
            return self._admission_open and not self._shutdown

    def admit_dispatch(self) -> bool:
        return self.admission_open()

    def _install_estop_locked(self, deadline_s: float) -> tuple[StopToken, bool]:
        if self._state is CoordinatorState.ESTOP_LATCHED:
            token = self._active_estop_token
            assert token is not None
            return token, False
        self._admission_open = False
        self._state = CoordinatorState.ESTOP_LATCHED
        self._estop_epoch += 1
        self._sequence += 1
        token = StopToken(self._instance_id, StopKind.ESTOP, self._sequence, deadline_s, self._estop_epoch)
        self._active_estop_token = token
        self._estop_result = None
        self._estop_complete.clear()
        return token, True

    def _unknown_acknowledgements(self, token: StopToken, detail: str) -> Tuple[AdapterStopAck, ...]:
        # Do not consult the configured clock on a coordinator failure: the
        # immutable token deadline is valid evidence and gives every target the
        # same conservative UNKNOWN observation.
        return tuple(AdapterStopAck(target, token, AckStatus.UNKNOWN, token.deadline_s, detail)
                     for target in self._targets)

    def _freeze_estop_unknown(self, token: StopToken, detail: str) -> EmergencyStopResult:
        result = EmergencyStopResult(token, self._unknown_acknowledgements(token, detail),
                                     self._targets, token.estop_epoch)
        return self._freeze_estop_result(result)

    def _freeze_estop_timeout(self, token: StopToken, detail: str) -> EmergencyStopResult:
        acknowledgements = tuple(
            AdapterStopAck(target, token, AckStatus.TIMEOUT, token.deadline_s, detail)
            for target in self._targets
        )
        return self._freeze_estop_result(
            EmergencyStopResult(token, acknowledgements, self._targets, token.estop_epoch)
        )

    def _freeze_estop_result(self, result: EmergencyStopResult) -> EmergencyStopResult:
        with self._lock:
            if self._estop_result is None:
                self._estop_result = result
                self._estop_complete.set()
            return self._estop_result

    def _complete_estop(self, token: StopToken) -> EmergencyStopResult:
        """Store exactly one result and signal waiters even on BaseException."""
        try:
            acknowledgements = self._aggregate(token)
            result = EmergencyStopResult(token, acknowledgements, self._targets, token.estop_epoch)
        except BaseException:
            return self._freeze_estop_unknown(token, "coordinator fanout failure")
        with self._lock:
            if self._estop_result is None:
                self._estop_result = result
                self._estop_complete.set()
            return self._estop_result

    def _queue_estop_locked(self, token: StopToken) -> None:
        try:
            queued = self._estop_fanout_lane.submit(token)
        except BaseException:
            queued = False
        if not queued:
            # Capacity one is an invariant; a failure must still provide a
            # bounded, authoritative result rather than strand callers.
            self._freeze_estop_unknown(token, "coordinator fanout lane unavailable")

    def latch_estop(self, deadline_s: float) -> StopToken:
        """Immediately close admission and enqueue the one ESTOP fanout request."""
        deadline_s = _finite_positive(deadline_s, "deadline_s")
        with self._lock:
            self._ensure_live()
            token, new_fanout = self._install_estop_locked(deadline_s)
            if new_fanout:
                self._queue_estop_locked(token)
            return token

    def estop_result(self, token: Optional[StopToken] = None) -> Optional[EmergencyStopResult]:
        """Return immutable authoritative ESTOP evidence when it is available."""
        with self._lock:
            if token is not None and token != self._active_estop_token:
                return None
            return self._estop_result

    def safe_hold(self, deadline_s: float) -> SafeHoldResult:
        deadline_s = _finite_positive(deadline_s, "deadline_s")
        with self._lock:
            self._ensure_live()
            self._admission_open = False
            self._sequence += 1
            token = StopToken(self._instance_id, StopKind.SAFE_HOLD, self._sequence, deadline_s, self._estop_epoch)
            if self._state is not CoordinatorState.ESTOP_LATCHED:
                self._state = CoordinatorState.HOLDING
        try:
            acknowledgements = self._aggregate(token)
        except BaseException:
            acknowledgements = self._unknown_acknowledgements(token, "safe-hold aggregation failure")
        # This is the hold finalization linearization point: an E-stop cannot
        # change the epoch between the observation used for confirmation and the
        # state transition below.
        with self._lock:
            result = SafeHoldResult(token, acknowledgements, self._targets, self._estop_epoch)
            if result.confirmed and self._state is not CoordinatorState.ESTOP_LATCHED:
                self._state = CoordinatorState.HELD
            elif not result.confirmed:
                if self._state is not CoordinatorState.ESTOP_LATCHED:
                    self._state = CoordinatorState.HOLD_FAULT
                self.latch_estop(deadline_s)
            return result

    def emergency_stop(self, deadline_s: float) -> EmergencyStopResult:
        """Compatibility API: latch immediately, then wait only to its token deadline."""
        token = self.latch_estop(deadline_s)
        try:
            remaining_s = max(0.0, token.deadline_s - self._clock())
        except BaseException:
            return self._freeze_estop_unknown(token, "coordinator clock failure")
        self._estop_complete.wait(remaining_s)
        with self._lock:
            result = self._estop_result
        if result is None:
            # A wedged or unscheduled fanout can never extend a caller's bound
            # or create alternate evidence for a duplicate caller.
            result = self._freeze_estop_timeout(token, "authoritative result missing by deadline")
        return result

    def _aggregate(self, token: StopToken) -> Tuple[AdapterStopAck, ...]:
        replies = {target: self._lanes[target].submit(token) for target in self._targets}
        answers = {}
        pending = {target for target, reply in replies.items() if reply is not None}
        while pending and self._clock() < token.deadline_s:
            for target in tuple(pending):
                try:
                    answers[target] = replies[target].get_nowait()
                    pending.remove(target)
                except queue.Empty:
                    pass
            if pending:
                time.sleep(min(0.001, max(0.0, token.deadline_s - self._clock())))
        for target in pending:
            self._lanes[target].quarantine()
        result = []
        for target in self._targets:
            if target not in answers:
                result.append(AdapterStopAck(target, token, AckStatus.TIMEOUT, self._clock(), "acknowledgement missing by deadline"))
            else:
                result.append(self._normalise_ack(target, token, answers[target]))
        return tuple(result)

    def _normalise_ack(self, target: TrustedStopTarget, token: StopToken, candidate: object) -> AdapterStopAck:
        now_s = self._clock()
        if not isinstance(candidate, AdapterStopAck):
            return AdapterStopAck(target, token, AckStatus.UNKNOWN, now_s, "malformed adapter acknowledgement")
        if candidate.target != target or candidate.token != token or candidate.observed_at_s > token.deadline_s:
            return AdapterStopAck(target, token, AckStatus.UNKNOWN, now_s, "stale or mismatched acknowledgement")
        return candidate

    def _ensure_live(self) -> None:
        if self._shutdown:
            raise RuntimeError("SafetyCoordinator is shut down")

    def shutdown(self, timeout_s: float = 0.1) -> None:
        timeout_s = _finite_positive(timeout_s, "timeout_s")
        with self._lock:
            if self._shutdown:
                return
            self._shutdown = True
            self._admission_open = False
        self._estop_fanout_lane.shutdown(timeout_s)
        for lane in self._lanes.values():
            lane.shutdown(timeout_s)
