"""Static, non-actuating trusted catalog contract for pre-HCC admission.

This module validates bytes and declarations only.  Its opaque handle resists
ordinary API caller forgery/mutation; it is not a security boundary against
arbitrary in-process code execution, monkeypatching, or a debugger.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import math
import os
import re
import stat
from datetime import datetime, timezone
from types import MappingProxyType
from typing import Any, Mapping

from .admission import ADMITTED, DENIED, evaluate_scoped_admission
from .capability_binding import profile_config_digest
from .evidence_bundle import (EvidenceBundleValidationError, evidence_bundle_digest,
                              validate_evidence_bundle)
from .provider_provenance import (ProviderProvenanceValidationError,
                                  validate_provider_provenance)

_PROVENANCE_EVALUATOR_ID = "trust-catalog-production-admission"
_PROVENANCE_PURPOSE = "production-admission"

CATALOG_SCHEMA_VERSION = "raicom.trust-catalog/v1"
_MAX_BYTES = 262144
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_LIVE_SAFETY_FUTURE_TOLERANCE_S = 1.0


class CatalogValidationError(ValueError):
    """A catalog is not a bounded canonical trusted catalog."""


@dataclass(frozen=True)
class HccEvidence:
    source_id: str
    record: Mapping[str, Any]
    bundle: Mapping[str, Any]
    provenance: Mapping[str, Any]


@dataclass(frozen=True)
class CoordinatorSession:
    source_id: str
    session_id: str
    epoch: int


@dataclass(frozen=True)
class Authorization:
    source_id: str
    record: Mapping[str, Any]
    provenance: Mapping[str, Any]


@dataclass(frozen=True)
class LiveSafety:
    """Unauthenticated provider-channel record contract pending HCC."""
    source_id: str
    record: Mapping[str, Any]
    provenance: Mapping[str, Any]


class ValidatedTrustCatalog:
    """Fieldless opaque handle; only this module's private factory registers it."""
    __slots__ = ()


def _catalog_handle_accessors():
    # This registry is intentionally closure-owned: no handle state, marker, or
    # registry is exposed as a module attribute. Process integrity is required.
    states: dict[ValidatedTrustCatalog, tuple[str, bytes, Mapping[str, Any]]] = {}

    def create(root: str, raw: bytes, catalog: Mapping[str, Any]) -> ValidatedTrustCatalog:
        handle = object.__new__(ValidatedTrustCatalog)
        states[handle] = (root, raw, catalog)
        return handle

    def get(handle: Any) -> tuple[str, bytes, Mapping[str, Any]] | None:
        if type(handle) is not ValidatedTrustCatalog:
            return None
        return states.get(handle)

    return create, get


_create_catalog_handle, _catalog_state_for = _catalog_handle_accessors()


def canonical_json_bytes(value: Any) -> bytes:
    """Return the only accepted JSON representation and reject nonfinite values."""
    _reject_nonfinite(value)
    try:
        return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True,
                          allow_nan=False).encode("utf-8")
    except (TypeError, ValueError) as error:
        raise CatalogValidationError("catalog_not_json") from error


def load_validated_catalog(*, expected_root_sha256: str, canonical_bytes: bytes | None = None,
                           absolute_path: str | None = None) -> ValidatedTrustCatalog:
    """Load canonical bytes or an absolute, component-wise nofollow regular file."""
    if (canonical_bytes is None) == (absolute_path is None):
        raise CatalogValidationError("exactly_one_catalog_source_required")
    if not _digest(expected_root_sha256):
        raise CatalogValidationError("expected_root_invalid")
    raw = canonical_bytes if canonical_bytes is not None else _read_absolute_nofollow(absolute_path or "")
    if not isinstance(raw, bytes) or not raw or len(raw) > _MAX_BYTES:
        raise CatalogValidationError("catalog_size_invalid")
    parsed = _parse_strict(raw)
    if canonical_json_bytes(parsed) != raw:
        raise CatalogValidationError("catalog_bytes_not_canonical")
    root = hashlib.sha256(raw).hexdigest()
    if root != expected_root_sha256:
        raise CatalogValidationError("catalog_root_mismatch")
    return _create_catalog_handle(root, raw, _validate_catalog(parsed))


def production_real_admission(catalog: ValidatedTrustCatalog, required_capabilities: list[str] | tuple[str, ...], *,
                              hcc_evidence: HccEvidence, session: CoordinatorSession,
                              authorization: Authorization, live_safety: LiveSafety,
                              now_s: float | int | None = None) -> dict[str, Any]:
    """Fail-closed production facade for a closure-registered catalog handle.

    ``now_s`` is injected Unix seconds. No ambient clock is consulted by this
    facade. Provider wrappers bind records to their named channel but do not
    authenticate that channel; HCC remains required for that assurance.
    """
    state = _catalog_state_for(catalog)
    if state is None:
        return _denied("validated_catalog_required")
    root, raw, stored_catalog = state
    # Reparse/revalidate before using closure state so an altered canonical root
    # or catalog cannot be substituted through an ordinary handle operation.
    try:
        parsed = _parse_strict(raw)
        if (hashlib.sha256(raw).hexdigest() != root or canonical_json_bytes(parsed) != raw
                or _validate_catalog(parsed) != stored_catalog):
            return _denied("validated_catalog_state_invalid")
    except CatalogValidationError:
        return _denied("validated_catalog_state_invalid")
    if not _finite_number(now_s):
        return _denied("live_safety_now_invalid")
    now_value = float(now_s)
    try:
        now_datetime = datetime.fromtimestamp(now_value, timezone.utc)
    except (ValueError, OverflowError, OSError):
        return _denied("live_safety_now_invalid")
    if not isinstance(hcc_evidence, HccEvidence) or not isinstance(session, CoordinatorSession) or not isinstance(authorization, Authorization) or not isinstance(live_safety, LiveSafety):
        return _denied("provider_bound_records_required")
    if not all(isinstance(x, str) and x.strip() for x in (hcc_evidence.source_id, session.source_id, authorization.source_id, live_safety.source_id)):
        return _denied("provider_source_identity_missing")
    if not isinstance(session.session_id, str) or not session.session_id.strip() or not isinstance(session.epoch, int) or isinstance(session.epoch, bool) or session.epoch < 0:
        return _denied("coordinator_session_invalid")
    entries = stored_catalog["entries"]
    wanted = sorted(set(required_capabilities)) if isinstance(required_capabilities, (list, tuple)) and all(isinstance(x, str) and x for x in required_capabilities) else []
    if not wanted or any(name not in entries for name in wanted):
        return _denied("catalog_capability_unknown")
    profile = _profile_for(entries, wanted)
    if profile is None:
        return _denied("catalog_profile_scope_mismatch")
    participating = _catalog_dependency_closure(entries, wanted)
    domains = sorted({domain for capability in participating for domain in entries[capability]["actuator_domains"]})
    manifests = {entry["adapter_id"]: _manifest_for(entry) for entry in entries.values()}
    try:
        # Validate untrusted structure before canonicalization/digesting it.
        bundle = validate_evidence_bundle(hcc_evidence.bundle, now_s=now_value)
        bundle_digest = evidence_bundle_digest(hcc_evidence.bundle)
        _bind_hcc_bundle(bundle, hcc_evidence, entries, profile, session, wanted, root)
        artifact_digest = bundle["artifact"]["sha256"]
        context_digest = admission_context_digest(bundle_digest=bundle_digest, artifact_digest=artifact_digest,
            catalog_root_sha256=root, profile_id=profile["profile_id"], session_id=session.session_id,
            session_epoch=session.epoch, operation_scope=wanted, capabilities=participating,
            actuator_domains=domains, purpose=_PROVENANCE_PURPOSE)
        safety_reason = _live_safety_reason(live_safety, root, profile, session, wanted, participating, domains,
            bundle_digest, artifact_digest, context_digest, now_value)
        if safety_reason:
            return _denied(safety_reason)
        for wrapper, initial_producer in ((hcc_evidence, bundle["producer"]["identity"]),
                                          (authorization, authorization.source_id),
                                          (live_safety, live_safety.source_id)):
            record = wrapper.record
            if (not isinstance(record, Mapping) or record.get("source_id") != wrapper.source_id
                    or record.get("bundle_sha256") != bundle_digest
                    or record.get("artifact_sha256") != artifact_digest
                    or record.get("admission_context_sha256") != context_digest):
                return _denied("provider_context_binding_mismatch")
            validate_provider_provenance(wrapper.provenance, expected_purpose=_PROVENANCE_PURPOSE,
                expected_initial_producer=initial_producer, expected_terminal_provider=wrapper.source_id,
                expected_evaluator=_PROVENANCE_EVALUATOR_ID, bundle_digest=bundle_digest,
                artifact_digest=artifact_digest, now_s=now_value)
    except (CatalogValidationError, EvidenceBundleValidationError, ProviderProvenanceValidationError,
            TypeError, ValueError, OverflowError) as error:
        return _denied(_bounded_reason(error))
    result = evaluate_scoped_admission(profile, manifests, wanted, safety_admission=live_safety.record,
        verification_records={"catalog-hcc": dict(hcc_evidence.record)}, authorization_records={"catalog-auth": dict(authorization.record)},
        trusted_profile={"profile_id": profile["profile_id"], "config_digest": profile["config_digest"], "safety_policy": profile["safety_policy"]},
        coordinator_session={"session_id": session.session_id, "epoch": session.epoch},
        now=now_datetime)
    return _denied("pre_hcc_real_admission_disabled", contract_evaluation=(
        "WOULD_ADMIT_IF_TRUSTED_AND_FIELD_VALIDATED" if result.get("status") == ADMITTED else "FAILED"))


def _bind_hcc_bundle(bundle: Mapping[str, Any], wrapper: HccEvidence, entries: Mapping[str, Mapping[str, Any]], profile: Mapping[str, Any], session: CoordinatorSession, scope: list[str], catalog_root_sha256: str) -> None:
    binding = bundle["binding"]
    if binding["catalog_root_sha256"] != catalog_root_sha256 or binding["provider_id"] != wrapper.source_id or binding["profile_id"] != profile["profile_id"] or binding["hardware_identity"] != profile["hardware_identity"] or binding["operation_scope"] != tuple(scope) or binding["session_id"] != session.session_id or binding["session_epoch"] != session.epoch:
        raise EvidenceBundleValidationError("bundle_catalog_binding_mismatch")
    for capability in scope:
        entry = entries[capability]
        if binding["adapter_id"] != entry["adapter_id"] or binding["adapter_digest"] != entry["adapter_digest"] or binding["measurement_algorithm"] != entry["measurement"]["algorithm"] or binding["measurement_digest"] != entry["measurement"]["expected_digest"]:
            raise EvidenceBundleValidationError("bundle_measurement_binding_mismatch")
    if profile["hardware_identity"] not in bundle["hardware_ids"]:
        raise EvidenceBundleValidationError("bundle_hardware_identity_mismatch")


def _live_safety_reason(wrapper: LiveSafety, root: str, profile: Mapping[str, Any], session: CoordinatorSession, scope: list[str], capabilities: list[str], domains: list[str], bundle_digest: str, artifact_digest: str, context_digest: str, now_s: float) -> str | None:
    record = wrapper.record
    if not isinstance(record, Mapping): return "live_safety_record_invalid"
    if record.get("source_id") != wrapper.source_id: return "live_safety_source_id_mismatch"
    if record.get("catalog_root_sha256") != root: return "live_safety_catalog_root_mismatch"
    if (record.get("bundle_sha256") != bundle_digest or record.get("artifact_sha256") != artifact_digest
            or record.get("admission_context_sha256") != context_digest): return "live_safety_context_binding_mismatch"
    if record.get("profile_id") != profile["profile_id"]: return "live_safety_profile_id_mismatch"
    if record.get("config_digest") != profile["config_digest"]: return "live_safety_profile_digest_mismatch"
    if record.get("session_id") != session.session_id: return "live_safety_session_id_mismatch"
    if record.get("session_epoch") != session.epoch: return "live_safety_epoch_mismatch"
    if record.get("operation_scope") != scope: return "live_safety_scope_mismatch"
    if record.get("capabilities") != capabilities: return "live_safety_capabilities_mismatch"
    if record.get("actuator_domains") != domains: return "live_safety_domains_mismatch"
    if record.get("status") != ADMITTED: return "live_safety_not_admitted"
    observed, max_age, expires = record.get("observed_at"), record.get("max_age_seconds"), record.get("expires_at")
    if not _finite_number(observed): return "live_safety_observed_at_invalid"
    if not _finite_number(max_age) or float(max_age) <= 0: return "live_safety_max_age_invalid"
    if not _finite_number(expires): return "live_safety_expires_at_invalid"
    if float(observed) > now_s + _LIVE_SAFETY_FUTURE_TOLERANCE_S: return "live_safety_observed_in_future"
    if now_s - float(observed) > float(max_age): return "live_safety_stale"
    if float(expires) <= now_s: return "live_safety_expired"
    return None


def _catalog_dependency_closure(entries: Mapping[str, Mapping[str, Any]], wanted: list[str]) -> list[str]:
    result: list[str] = []
    def visit(name: str) -> None:
        if name in result: return
        for dependency in entries[name]["dependencies"]: visit(dependency)
        result.append(name)
    for name in wanted: visit(name)
    return sorted(result)


def _validate_catalog(value: Any) -> Mapping[str, Any]:
    if not isinstance(value, dict) or set(value) != {"schema_version", "entries"} or value.get("schema_version") != CATALOG_SCHEMA_VERSION: raise CatalogValidationError("catalog_schema_invalid")
    entries = value.get("entries")
    if not isinstance(entries, list) or not entries or len(entries) > 128: raise CatalogValidationError("catalog_entries_invalid")
    result: dict[str, Any] = {}
    for raw in entries:
        entry = _validate_entry(raw); name = entry["capability_id"]
        if name in result: raise CatalogValidationError("catalog_duplicate_capability")
        result[name] = entry
    _validate_cycles(result)
    return MappingProxyType({"schema_version": CATALOG_SCHEMA_VERSION, "entries": MappingProxyType(result)})


def _validate_entry(raw: Any) -> Mapping[str, Any]:
    required = {"capability_id", "profile_id", "profile_preimage", "profile_digest", "safety_policy", "adapter_id", "adapter_contract_preimage", "adapter_digest", "measurement", "actuating", "actuator_domains", "dependencies"}
    if not isinstance(raw, dict) or set(raw) != required: raise CatalogValidationError("catalog_entry_keys_invalid")
    strings = ("capability_id", "profile_id", "profile_digest", "adapter_id", "adapter_digest")
    if any(not isinstance(raw[k], str) or not raw[k] for k in strings) or not _digest(raw["profile_digest"]) or not _digest(raw["adapter_digest"]): raise CatalogValidationError("catalog_entry_type_invalid")
    profile, contract = raw["profile_preimage"], raw["adapter_contract_preimage"]
    if not isinstance(profile, dict) or not isinstance(contract, dict): raise CatalogValidationError("catalog_preimage_invalid")
    if profile.get("profile_id") != raw["profile_id"] or profile_config_digest(profile) != raw["profile_digest"]: raise CatalogValidationError("profile_preimage_digest_mismatch")
    if _preimage_digest(contract) != raw["adapter_digest"] or contract.get("adapter_id") != raw["adapter_id"]: raise CatalogValidationError("adapter_preimage_digest_mismatch")
    policy = raw["safety_policy"]
    if not isinstance(policy, dict) or set(policy) != {"policy_id", "version", "digest"} or not all(isinstance(policy.get(k), str) and policy[k] for k in ("policy_id", "version")) or not _digest(policy.get("digest")): raise CatalogValidationError("safety_policy_invalid")
    measurement = raw["measurement"]
    if not isinstance(measurement, dict) or set(measurement) != {"algorithm", "expected_digest"} or not isinstance(measurement.get("algorithm"), str) or not _digest(measurement.get("expected_digest")): raise CatalogValidationError("measurement_invalid")
    domains, dependencies = raw["actuator_domains"], raw["dependencies"]
    if not isinstance(raw["actuating"], bool) or not isinstance(domains, list) or not isinstance(dependencies, list) or any(not isinstance(x, str) or not x for x in domains + dependencies) or len(set(domains)) != len(domains) or len(set(dependencies)) != len(dependencies) or raw["actuating"] != bool(domains): raise CatalogValidationError("domains_or_dependencies_invalid")
    return _freeze(raw)


def _profile_for(entries: Mapping[str, Mapping[str, Any]], wanted: list[str]) -> dict[str, Any] | None:
    entry = entries[wanted[0]]
    if any(entries[x]["profile_id"] != entry["profile_id"] or entries[x]["profile_digest"] != entry["profile_digest"] or entries[x]["safety_policy"] != entry["safety_policy"] for x in wanted): return None
    profile = _thaw(entry["profile_preimage"]); profile["config_digest"] = entry["profile_digest"]; profile["safety_policy"] = _thaw(entry["safety_policy"])
    capabilities = profile.get("capabilities")
    if not isinstance(capabilities, Mapping): return None
    for capability_id, item in entries.items():
        if item["profile_id"] == entry["profile_id"]:
            binding = capabilities.get(capability_id)
            if not isinstance(binding, Mapping) or binding.get("adapter_id") != item["adapter_id"] or binding.get("adapter_digest") != item["adapter_digest"]: return None
    return profile


def _manifest_for(entry: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(entry["adapter_contract_preimage"]); result.update(adapter_id=entry["adapter_id"], adapter_digest=entry["adapter_digest"], actuating=entry["actuating"], actuator_domains=list(entry["actuator_domains"]), dependencies=list(entry["dependencies"])); return result

def _preimage_digest(value: Mapping[str, Any]) -> str: return hashlib.sha256(canonical_json_bytes(value)).hexdigest()
def _digest(value: Any) -> bool: return isinstance(value, str) and bool(_SHA256.fullmatch(value))
def _finite_number(value: Any) -> bool: return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))
def _freeze(value: Any) -> Any:
    if isinstance(value, dict): return MappingProxyType({k: _freeze(v) for k, v in value.items()})
    if isinstance(value, list): return tuple(_freeze(v) for v in value)
    return value
def _thaw(value: Any) -> Any:
    if isinstance(value, Mapping): return {key: _thaw(item) for key, item in value.items()}
    if isinstance(value, tuple): return [_thaw(item) for item in value]
    return value
def _reject_nonfinite(value: Any) -> None:
    if isinstance(value, float) and not math.isfinite(value): raise CatalogValidationError("nonfinite_value")
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str): raise CatalogValidationError("nonstring_key")
            _reject_nonfinite(item)
    elif isinstance(value, (list, tuple)):
        for item in value: _reject_nonfinite(item)
def _parse_strict(raw: bytes) -> Any:
    try: return json.loads(raw.decode("utf-8"), object_pairs_hook=_no_duplicates, parse_constant=lambda x: (_ for _ in ()).throw(CatalogValidationError("nonfinite_value")))
    except (UnicodeDecodeError, json.JSONDecodeError) as error: raise CatalogValidationError("catalog_json_invalid") from error
def _no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result = {}
    for key, value in pairs:
        if key in result: raise CatalogValidationError("duplicate_key")
        result[key] = value
    return result
def _validate_cycles(entries: Mapping[str, Mapping[str, Any]]) -> None:
    active, done = set(), set()
    def visit(name: str) -> None:
        if name in active: raise CatalogValidationError("dependency_cycle")
        if name in done: return
        active.add(name)
        for dep in entries[name]["dependencies"]:
            if dep not in entries: raise CatalogValidationError("dependency_unknown")
            visit(dep)
        active.remove(name); done.add(name)
    for name in entries: visit(name)
def _read_absolute_nofollow(path: str) -> bytes:
    if not os.path.isabs(path): raise CatalogValidationError("catalog_path_not_absolute")
    parts = [part for part in path.split("/") if part]
    if not parts: raise CatalogValidationError("catalog_path_untrusted")
    fd = os.open("/", os.O_RDONLY | os.O_DIRECTORY)
    try:
        for index, part in enumerate(parts):
            final = index == len(parts) - 1
            flags = os.O_RDONLY | os.O_NOFOLLOW | (os.O_NONBLOCK if final else os.O_DIRECTORY)
            next_fd = os.open(part, flags, dir_fd=fd); os.close(fd); fd = next_fd
        if not stat.S_ISREG(os.fstat(fd).st_mode): raise CatalogValidationError("catalog_path_not_regular")
        data = os.read(fd, _MAX_BYTES + 1)
        if os.read(fd, 1): raise CatalogValidationError("catalog_size_invalid")
        return data
    except CatalogValidationError:
        raise
    except OSError as error:
        raise CatalogValidationError("catalog_path_untrusted") from error
    finally:
        try: os.close(fd)
        except OSError: pass
def admission_context_digest(*, bundle_digest: str, artifact_digest: str, catalog_root_sha256: str,
                             profile_id: str, session_id: str, session_epoch: int,
                             operation_scope: list[str], capabilities: list[str],
                             actuator_domains: list[str], purpose: str) -> str:
    """Digest the single canonical production-admission binding context."""
    return hashlib.sha256(canonical_json_bytes({"artifact_sha256": artifact_digest,
        "bundle_sha256": bundle_digest, "capabilities": capabilities,
        "catalog_root_sha256": catalog_root_sha256, "actuator_domains": actuator_domains,
        "operation_scope": operation_scope, "profile_id": profile_id, "purpose": purpose,
        "session_epoch": session_epoch, "session_id": session_id})).hexdigest()


def _bounded_reason(error: Exception) -> str:
    reason = str(error)
    return reason if reason and len(reason) <= 128 else "production_record_invalid"


def _denied(reason: str, *, contract_evaluation: str = "FAILED") -> dict[str, Any]:
    return {"status": DENIED, "authorization": "NONE", "required_capabilities": [],
            "capability_vector": [], "reasons": [reason], "assurance": "SIMULATED_CONTRACT_TEST",
            "field_validation": "NOT_FIELD_VALIDATED", "real_availability": "UNAVAILABLE",
            "readiness": "NOT_READY", "contract_evaluation": contract_evaluation}
