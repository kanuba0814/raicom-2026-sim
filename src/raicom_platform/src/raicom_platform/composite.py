"""Routing-only composite facade with factory-bound, non-actuation evidence.

The facade does not discover, create, or control hardware.  Adapters are injected
through an opaque binding issued by :class:`TrustedAdapterFactory`; consequently a
caller cannot make an arbitrary adapter appear safe merely by supplying labels.
"""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Callable, FrozenSet, Optional, Protocol, Tuple

from .api import CapabilityState, OperationResult


class AdapterMode(str, Enum):
    FAKE = "FAKE"
    OBSERVE_ONLY = "OBSERVE_ONLY"
    REPLAY = "REPLAY"
    DRY_RUN = "DRY_RUN"
    DISABLED = "DISABLED"
    ACTUATE = "ACTUATE"


@dataclass(frozen=True)
class AdapterDescriptor:
    """Factory-attested adapter contract, including its object identity binding."""

    adapter: str
    source: str
    mode: AdapterMode
    config_digest: str
    verification_ref: str
    authorization_ref: str
    freshness_decision: str
    actuator_domains: FrozenSet[str] = frozenset()
    structurally_non_actuating: bool = True
    hardware_identity: Optional[str] = None
    hardware_ref: Optional[str] = None

    def __post_init__(self) -> None:
        if not all(isinstance(value, str) and value.strip() for value in (
            self.adapter, self.source, self.config_digest, self.verification_ref,
            self.authorization_ref, self.freshness_decision,
        )):
            raise ValueError("adapter descriptor fields must be non-empty")
        if any(not isinstance(domain, str) or not domain.strip() for domain in self.actuator_domains):
            raise ValueError("actuator domains must be non-empty names")
        if self.structurally_non_actuating and self.actuator_domains:
            raise ValueError("a structurally non-actuating adapter cannot declare actuator domains")
        if self.mode == AdapterMode.DISABLED and not self.structurally_non_actuating:
            raise ValueError("disabled adapters must be structurally non-actuating")
        if (self.hardware_identity is None) != (self.hardware_ref is None):
            raise ValueError("hardware identity and reference must be supplied together")


@dataclass(frozen=True)
class CapabilityProvenance:
    """Compatibility view derived only from a trusted descriptor."""
    adapter: str
    source: str
    mode: AdapterMode
    config_digest: str
    verification_scope: str


@dataclass(frozen=True)
class CapabilityHealth:
    state: CapabilityState
    provenance: CapabilityProvenance
    detail: str = ""


@dataclass(frozen=True)
class OperationEvidence:
    """Immutable evidence for one routed capability operation or safety ack."""
    capability: str
    operation: str
    timestamp_s: float
    outcome: OperationResult
    adapter: str
    source: str
    config_digest: str
    mode: AdapterMode
    verification_ref: str
    authorization_ref: str
    freshness_decision: str
    hardware_identity: Optional[str]
    hardware_ref: Optional[str]
    safety_ack_event: Optional[str] = None


@dataclass(frozen=True)
class PlatformHealthVector:
    """Independent capability records.  It deliberately cannot claim REAL validation."""
    navigation: CapabilityHealth
    perception: CapabilityHealth
    manipulation: CapabilityHealth
    charging: CapabilityHealth
    lifecycle: str = "MIXED_COMMISSIONING"


class NavigationAdapter(Protocol):
    def goto(self, waypoint: str, deadline_s: float) -> OperationResult: ...
class PerceptionAdapter(Protocol):
    def perceive(self, target: str, deadline_s: float) -> OperationResult: ...
class ManipulationAdapter(Protocol):
    def grasp(self, target: str, deadline_s: float) -> OperationResult: ...
    def place(self, target: str, deadline_s: float) -> OperationResult: ...
class ChargingAdapter(Protocol):
    def charge(self, station: str, target: str, deadline_s: float) -> OperationResult: ...
class EmergencyStopCapable(Protocol):
    def emergency_stop(self) -> None: ...
class SafeHoldCapable(Protocol):
    def safe_hold(self) -> None: ...


class AdapterBinding:
    """Opaque factory token; direct construction is rejected by the facade."""
    __slots__ = ("_adapter", "_descriptor", "_issuer")
    def __init__(self, adapter: object, descriptor: AdapterDescriptor, issuer: object) -> None:
        self._adapter, self._descriptor, self._issuer = adapter, descriptor, issuer


class TrustedAdapterFactory:
    """Single composition-root authority for identity-bound adapter descriptors."""
    def __init__(self) -> None:
        self._issuer = object()
        self._bindings: dict[int, Tuple[object, AdapterDescriptor]] = {}

    def bind(self, adapter: object, descriptor: AdapterDescriptor) -> AdapterBinding:
        if adapter is None:
            raise ValueError("adapter must be an object")
        _validate_descriptor(descriptor)
        binding = AdapterBinding(adapter, descriptor, self._issuer)
        self._bindings[id(binding)] = (adapter, descriptor)
        return binding

    def _validate(self, binding: AdapterBinding) -> AdapterDescriptor:
        registered = self._bindings.get(id(binding))
        if (not isinstance(binding, AdapterBinding) or binding._issuer is not self._issuer
                or registered is None or registered[0] is not binding._adapter
                or registered[1] is not binding._descriptor):
            raise ValueError("adapter binding was not issued by this trusted factory")
        _validate_descriptor(binding._descriptor)
        return binding._descriptor


class CompositePlatformFacade:
    """Forward semantic requests to factory-bound adapters and retain evidence."""
    def __init__(self, navigation: AdapterBinding, perception: AdapterBinding,
                 manipulation: AdapterBinding, *, factory: TrustedAdapterFactory,
                 charging: Optional[AdapterBinding] = None,
                 clock: Callable[[], float] = time.time) -> None:
        if not callable(clock):
            raise ValueError("clock must be callable")
        self._factory, self._clock = factory, clock
        self._navigation, self._navigation_descriptor = self._bound(navigation, "navigation")
        self._perception, self._perception_descriptor = self._bound(perception, "perception")
        self._manipulation, self._manipulation_descriptor = self._bound(manipulation, "manipulation")
        if charging is None:
            self._charging, self._charging_descriptor = None, None
        else:
            self._charging, self._charging_descriptor = self._bound(charging, "charging")
        self._evidence: dict[str, list[OperationEvidence]] = {
            "navigation": [], "perception": [], "manipulation": [], "charging": [],
        }
        self._health = PlatformHealthVector(
            _health(self._navigation_descriptor), _health(self._perception_descriptor),
            _health(self._manipulation_descriptor),
            _health(self._charging_descriptor) if self._charging_descriptor else _disabled_charging_health(),
        )

    def _bound(self, binding: AdapterBinding, capability: str) -> Tuple[object, AdapterDescriptor]:
        descriptor = self._factory._validate(binding)
        if descriptor.mode == AdapterMode.DISABLED:
            raise ValueError("a disabled capability cannot have an adapter")
        if not descriptor.structurally_non_actuating and descriptor.mode in {
            AdapterMode.OBSERVE_ONLY, AdapterMode.REPLAY, AdapterMode.DRY_RUN, AdapterMode.DISABLED,
        }:
            raise ValueError(f"{capability} actuating adapter is incompatible with {descriptor.mode.value}")
        return binding._adapter, descriptor

    def health(self) -> PlatformHealthVector:
        return self._health

    def operation_evidence(self, capability: Optional[str] = None) -> Tuple[OperationEvidence, ...]:
        if capability is None:
            return tuple(item for records in self._evidence.values() for item in records)
        if capability not in self._evidence:
            raise ValueError("unknown capability")
        return tuple(self._evidence[capability])

    def record_safety_ack(self, capability: str, event: str,
                          outcome: OperationResult = OperationResult.SUCCEEDED) -> None:
        if capability not in self._evidence or not isinstance(event, str) or event not in {"CANCEL", "HOLD", "ESTOP"}:
            raise ValueError("safety acknowledgement requires capability and CANCEL/HOLD/ESTOP event")
        descriptor = getattr(self, "_" + capability + "_descriptor")
        if descriptor is None:
            descriptor = _disabled_descriptor()
        self._record(capability, "safety_ack", outcome, descriptor, event)

    def goto(self, waypoint: str, deadline_s: float) -> OperationResult:
        return self._route("navigation", "goto", self._navigation_descriptor,
                           self._navigation.goto, _name("waypoint", waypoint), _deadline(deadline_s))
    def perceive(self, target: str, deadline_s: float) -> OperationResult:
        return self._route("perception", "perceive", self._perception_descriptor,
                           self._perception.perceive, _name("target", target), _deadline(deadline_s))
    def grasp(self, target: str, deadline_s: float) -> OperationResult:
        return self._route("manipulation", "grasp", self._manipulation_descriptor,
                           self._manipulation.grasp, _name("target", target), _deadline(deadline_s))
    def place(self, target: str, deadline_s: float) -> OperationResult:
        return self._route("manipulation", "place", self._manipulation_descriptor,
                           self._manipulation.place, _name("target", target), _deadline(deadline_s))
    def charge(self, station: str, target: str, deadline_s: float) -> OperationResult:
        if self._charging is None:
            return OperationResult.UNAVAILABLE
        return self._route("charging", "charge", self._charging_descriptor, self._charging.charge,
                           _name("station", station), _name("target", target), _deadline(deadline_s))

    def _route(self, capability: str, operation: str, descriptor: AdapterDescriptor,
               method: Callable[..., OperationResult], *args: object) -> OperationResult:
        result = method(*args)
        if not isinstance(result, OperationResult):
            raise ValueError("adapter must return an OperationResult")
        self._record(capability, operation, result, descriptor)
        return result

    def _record(self, capability: str, operation: str, outcome: OperationResult,
                descriptor: AdapterDescriptor, event: Optional[str] = None) -> None:
        timestamp = float(self._clock())
        if not math.isfinite(timestamp):
            raise ValueError("clock must return a finite timestamp")
        self._evidence[capability].append(OperationEvidence(
            capability, operation, timestamp, outcome, descriptor.adapter, descriptor.source,
            descriptor.config_digest, descriptor.mode, descriptor.verification_ref,
            descriptor.authorization_ref, descriptor.freshness_decision,
            descriptor.hardware_identity, descriptor.hardware_ref, event,
        ))


def _validate_descriptor(descriptor: AdapterDescriptor) -> None:
    if not isinstance(descriptor, AdapterDescriptor):
        raise ValueError("trusted binding requires an AdapterDescriptor")
    if descriptor.mode in {AdapterMode.OBSERVE_ONLY, AdapterMode.REPLAY, AdapterMode.DRY_RUN, AdapterMode.DISABLED} and not descriptor.structurally_non_actuating:
        raise ValueError("non-actuating execution modes require structural non-actuation")


def _health(descriptor: AdapterDescriptor) -> CapabilityHealth:
    return CapabilityHealth(CapabilityState.SIM_TESTED, CapabilityProvenance(
        descriptor.adapter, descriptor.source, descriptor.mode, descriptor.config_digest,
        descriptor.verification_ref,
    ))


def _disabled_descriptor() -> AdapterDescriptor:
    return AdapterDescriptor("disabled", "none", AdapterMode.DISABLED, "disabled", "not-configured", "not-configured", "not-applicable")
def _disabled_charging_health() -> CapabilityHealth:
    return _health(_disabled_descriptor())
def _name(label: str, value: str) -> str:
    if not isinstance(value, str) or not value.strip(): raise ValueError(f"{label} must be a non-empty semantic name")
    return value
def _deadline(value: float) -> float:
    if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value): raise ValueError("deadline_s must be finite")
    return float(value)
