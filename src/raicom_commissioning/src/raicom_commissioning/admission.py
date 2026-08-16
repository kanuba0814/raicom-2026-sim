"""Pure, fail-closed ADR-001 scoped commissioning admission evaluator."""
from __future__ import annotations

from datetime import datetime, timezone
import re
from typing import Any, Mapping

from .capability_binding import EXECUTION_MODES, SCHEMA_VERSION, SOURCES, profile_config_digest

ADMITTED = "ADMITTED"
DENIED = "DENIED"
AUTHORIZATION_NONE = "NONE"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_NON_ACTUATING_MODES = frozenset({"OBSERVE_ONLY", "REPLAY", "DRY_RUN", "DISABLED"})


def evaluate_scoped_admission(
    profile: Mapping[str, Any] | None,
    trusted_manifests: Mapping[str, Mapping[str, Any]] | None,
    required_capabilities: list[str] | tuple[str, ...] | set[str] | None,
    *,
    safety_admission: Mapping[str, Any] | None,
    verification_records: Mapping[str, Mapping[str, Any]] | None = None,
    authorization_records: Mapping[str, Mapping[str, Any]] | None = None,
    trusted_profile: Mapping[str, Any] | None = None,
    coordinator_session: Mapping[str, Any] | None = None,
    now: datetime | None = None,
) -> dict[str, Any]:
    """Evaluate a requested scope without I/O or authorization side effects.

    ``trusted_profile`` supplies the immutable expected profile ID, canonical
    config digest, and safety policy identity/version/digest.  ``coordinator_session``
    supplies the current session ID and epoch.  Both are required for REAL
    ACTUATE but intentionally not for development-only fake/replay/dry-run.
    """
    current_time = _current_time(now)
    manifests = trusted_manifests if isinstance(trusted_manifests, Mapping) else {}
    verifications = verification_records if isinstance(verification_records, Mapping) else {}
    authorizations = authorization_records if isinstance(authorization_records, Mapping) else {}
    # Scope is set-like; normalize it before binding it into evidence so a
    # caller-provided set cannot create nondeterministic authorization checks.
    required = sorted(set(_string_list(required_capabilities)))
    global_reasons = _profile_reasons(profile)
    if not required:
        global_reasons.append("required_capabilities_missing")
    if not _safety_admitted(safety_admission):
        global_reasons.append("global_safety_not_admitted")

    bindings = profile.get("capabilities") if isinstance(profile, Mapping) else None
    bindings = bindings if isinstance(bindings, Mapping) else {}
    ordered, dependency_reasons = _dependency_closure(required, bindings, manifests)
    vector: list[dict[str, Any]] = []
    previous: dict[str, dict[str, Any]] = {}
    for capability_id in ordered:
        entry = _evaluate_capability(
            capability_id, bindings.get(capability_id), manifests, verifications,
            authorizations, profile, trusted_profile, coordinator_session, required,
            current_time, previous, dependency_reasons.get(capability_id, []),
        )
        vector.append(entry)
        previous[capability_id] = entry

    reasons = sorted(set(global_reasons + [reason for item in vector for reason in item["reasons"]]))
    if reasons:
        for item in vector:
            item["authorization"] = AUTHORIZATION_NONE
    return {"status": ADMITTED if not reasons else DENIED,
            "authorization": AUTHORIZATION_NONE if reasons or not any(item["actuating"] for item in vector) else "GRANTED",
            "required_capabilities": required, "capability_vector": vector, "reasons": reasons}


def _evaluate_capability(capability_id: str, binding: Any, manifests: Mapping[str, Mapping[str, Any]],
                         verifications: Mapping[str, Mapping[str, Any]], authorizations: Mapping[str, Mapping[str, Any]],
                         profile: Mapping[str, Any] | None, trusted_profile: Mapping[str, Any] | None,
                         coordinator_session: Mapping[str, Any] | None, requested_scope: list[str], now: datetime,
                         previous: Mapping[str, Mapping[str, Any]], initial_reasons: list[str]) -> dict[str, Any]:
    reasons = list(initial_reasons)
    if not isinstance(binding, Mapping):
        return _vector(capability_id, None, None, False, reasons + ["binding_missing"])
    adapter_id = binding.get("adapter_id")
    manifest = manifests.get(adapter_id) if isinstance(adapter_id, str) else None
    if not isinstance(manifest, Mapping):
        return _vector(capability_id, binding, adapter_id, False, reasons + ["trusted_manifest_missing"])
    if manifest.get("adapter_id") != adapter_id or not _valid_digest(manifest.get("adapter_digest")):
        reasons.append("trusted_manifest_invalid")
    if binding.get("adapter_digest") != manifest.get("adapter_digest"):
        reasons.append("adapter_digest_mismatch")
    source, mode = binding.get("source"), binding.get("execution_mode")
    if source not in SOURCES:
        reasons.append("source_invalid_or_unknown")
    if mode not in EXECUTION_MODES:
        reasons.append("execution_mode_invalid_or_unknown")
    deadlines = binding.get("deadlines")
    if not isinstance(deadlines, Mapping) or not all(isinstance(deadlines.get(key), int) and deadlines[key] > 0 for key in ("operation_seconds", "cancel_seconds")):
        reasons.append("deadlines_missing_or_invalid")
    policy = binding.get("freshness_policy")
    if not isinstance(policy, Mapping) or not isinstance(policy.get("max_age_seconds"), int) or policy["max_age_seconds"] < 0:
        reasons.append("freshness_policy_missing_or_invalid")
    actuating, domains = bool(manifest.get("actuating")), _string_list(manifest.get("actuator_domains"))
    if actuating and not domains:
        reasons.append("actuating_manifest_domains_missing")
    if mode in _NON_ACTUATING_MODES and actuating:
        reasons.append("non_actuating_mode_rejects_actuating_manifest")
    if mode == "ACTUATE" and not actuating:
        reasons.append("actuate_mode_requires_actuating_manifest")
    for dependency in _string_list(manifest.get("dependencies")):
        if previous.get(dependency, {}).get("status") != ADMITTED:
            reasons.append("dependency_not_admitted:" + dependency)
    if source == "REAL" and mode == "ACTUATE":
        _check_real_actuation(binding, manifest, profile, trusted_profile, coordinator_session, requested_scope,
                              verifications, authorizations, domains, now, reasons)
    return _vector(capability_id, binding, adapter_id, actuating and mode == "ACTUATE", reasons)


def _check_real_actuation(binding: Mapping[str, Any], manifest: Mapping[str, Any], profile: Mapping[str, Any] | None,
                          trusted_profile: Mapping[str, Any] | None, coordinator_session: Mapping[str, Any] | None,
                          requested_scope: list[str], verifications: Mapping[str, Mapping[str, Any]],
                          authorizations: Mapping[str, Mapping[str, Any]], domains: list[str], now: datetime, reasons: list[str]) -> None:
    if not isinstance(profile, Mapping) or not isinstance(profile.get("hardware_identity"), str) or not profile["hardware_identity"].strip():
        reasons.append("hardware_identity_missing")
    _check_profile_trust(profile, trusted_profile, reasons)
    session_id, epoch = _session_values(coordinator_session)
    if session_id is None:
        reasons.append("coordinator_session_missing")
    if epoch is None:
        reasons.append("coordinator_epoch_missing")
    verification = verifications.get(binding.get("verification_ref")) if isinstance(binding.get("verification_ref"), str) else None
    if not isinstance(verification, Mapping):
        reasons.append("verification_missing_or_unknown")
    else:
        _check_record_binding(verification, manifest, profile, requested_scope, session_id, epoch, "verification", reasons)
        if verification.get("status") != "VERIFIED" or verification.get("kind") != "HCC":
            reasons.append("verification_not_identity_bound_hcc")
        if not _fresh(verification, binding.get("freshness_policy"), now):
            reasons.append("verification_stale_or_invalid")
    authorization = authorizations.get(binding.get("authorization_ref")) if isinstance(binding.get("authorization_ref"), str) else None
    if not isinstance(authorization, Mapping):
        reasons.append("authorization_missing_or_unknown")
    else:
        _check_record_binding(authorization, manifest, profile, requested_scope, session_id, epoch, "authorization", reasons)
        if authorization.get("status") != "ACTIVE" or not _unexpired(authorization.get("expires_at"), now):
            reasons.append("authorization_expired_or_inactive")
        if set(_string_list(authorization.get("actuator_domains"))) != set(domains):
            reasons.append("authorization_domains_not_exact")


def _check_profile_trust(profile: Mapping[str, Any] | None, trusted: Mapping[str, Any] | None, reasons: list[str]) -> None:
    if not isinstance(trusted, Mapping):
        reasons.append("trusted_profile_missing")
        return
    if not isinstance(profile, Mapping) or profile.get("profile_id") != trusted.get("profile_id"):
        reasons.append("trusted_profile_id_mismatch")
    if not isinstance(profile, Mapping) or profile.get("config_digest") != profile_config_digest(profile):
        reasons.append("profile_config_digest_mismatch")
    if not isinstance(profile, Mapping) or profile.get("config_digest") != trusted.get("config_digest"):
        reasons.append("trusted_profile_digest_mismatch")
    if profile.get("safety_policy") != trusted.get("safety_policy"):
        reasons.append("safety_policy_identity_mismatch")


def _check_record_binding(record: Mapping[str, Any], manifest: Mapping[str, Any], profile: Mapping[str, Any] | None,
                          scope: list[str], session_id: str | None, epoch: int | None, prefix: str, reasons: list[str]) -> None:
    expected = {"adapter_id": manifest.get("adapter_id"), "adapter_digest": manifest.get("adapter_digest"),
                "hardware_identity": profile.get("hardware_identity") if isinstance(profile, Mapping) else None,
                "profile_id": profile.get("profile_id") if isinstance(profile, Mapping) else None,
                "config_digest": profile.get("config_digest") if isinstance(profile, Mapping) else None,
                "operation_scope": scope, "session_id": session_id, "session_epoch": epoch}
    for key, value in expected.items():
        if record.get(key) != value:
            reasons.append(prefix + "_" + key + "_mismatch")


def _vector(capability_id: str, binding: Mapping[str, Any] | None, adapter_id: Any, actuating: bool, reasons: list[str]) -> dict[str, Any]:
    return {"capability_id": capability_id, "status": ADMITTED if not reasons else DENIED,
            "adapter_id": adapter_id if isinstance(adapter_id, str) else None,
            "source": binding.get("source") if isinstance(binding, Mapping) else None,
            "execution_mode": binding.get("execution_mode") if isinstance(binding, Mapping) else None,
            "actuating": actuating,
            "authorization": "GRANTED" if not reasons and actuating and isinstance(binding, Mapping) and binding.get("source") == "REAL" else AUTHORIZATION_NONE,
            "reasons": sorted(set(reasons))}


def _dependency_closure(required: list[str], bindings: Mapping[str, Any], manifests: Mapping[str, Mapping[str, Any]]) -> tuple[list[str], dict[str, list[str]]]:
    result: list[str] = []
    active: list[str] = []
    issues: dict[str, list[str]] = {}
    def issue(capability_id: str, reason: str) -> None:
        issues.setdefault(capability_id, []).append(reason)
    def visit(capability_id: str) -> None:
        if capability_id in result:
            return
        if capability_id in active:
            cycle = active[active.index(capability_id):] + [capability_id]
            reason = "dependency_cycle:" + "->".join(cycle)
            for item in cycle[:-1]: issue(item, reason)
            return
        active.append(capability_id)
        binding = bindings.get(capability_id)
        adapter_id = binding.get("adapter_id") if isinstance(binding, Mapping) else None
        manifest = manifests.get(adapter_id) if isinstance(adapter_id, str) else None
        if isinstance(manifest, Mapping):
            raw = manifest.get("dependencies", [])
            if not isinstance(raw, (list, tuple, set)):
                issue(capability_id, "dependencies_invalid")
            else:
                for dependency in raw:
                    if not isinstance(dependency, str) or not dependency.strip():
                        issue(capability_id, "dependency_invalid")
                    elif dependency not in bindings:
                        issue(capability_id, "dependency_binding_missing:" + dependency)
                    else:
                        visit(dependency)
        active.pop()
        if capability_id not in result: result.append(capability_id)
    for item in required: visit(item)
    return result, issues


def _profile_reasons(profile: Mapping[str, Any] | None) -> list[str]:
    if not isinstance(profile, Mapping): return ["profile_missing_or_invalid"]
    reasons = []
    if profile.get("schema_version") != SCHEMA_VERSION: reasons.append("unsupported_schema_version")
    if not isinstance(profile.get("profile_id"), str) or not profile["profile_id"].strip(): reasons.append("profile_id_missing")
    if not _valid_digest(profile.get("config_digest")): reasons.append("config_digest_missing_or_invalid")
    if not isinstance(profile.get("safety_policy_ref"), str) or not profile["safety_policy_ref"].strip(): reasons.append("safety_policy_ref_missing")
    if not isinstance(profile.get("safety_policy"), Mapping) or not all(isinstance(profile["safety_policy"].get(key), str) and profile["safety_policy"][key].strip() for key in ("policy_id", "version", "digest")) or not _valid_digest(profile.get("safety_policy", {}).get("digest")): reasons.append("safety_policy_missing_or_invalid")
    if not isinstance(profile.get("capabilities"), Mapping): reasons.append("capabilities_missing_or_invalid")
    return reasons


def _safety_admitted(value: Mapping[str, Any] | None) -> bool: return isinstance(value, Mapping) and value.get("status") == ADMITTED

def _fresh(record: Mapping[str, Any], policy: Any, now: datetime) -> bool:
    observed = _parse_time(record.get("observed_at"))
    return isinstance(policy, Mapping) and isinstance(policy.get("max_age_seconds"), int) and policy["max_age_seconds"] >= 0 and observed is not None and 0 <= (now - observed).total_seconds() <= policy["max_age_seconds"] and _unexpired(record.get("expires_at"), now)
def _unexpired(value: Any, now: datetime) -> bool:
    parsed = _parse_time(value); return parsed is not None and parsed > now
def _parse_time(value: Any) -> datetime | None:
    if not isinstance(value, str): return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00")); return parsed if parsed.tzinfo else None
    except ValueError: return None
def _current_time(now: datetime | None) -> datetime:
    value = now or datetime.now(timezone.utc); return value if value.tzinfo else value.replace(tzinfo=timezone.utc)
def _valid_digest(value: Any) -> bool: return isinstance(value, str) and bool(_SHA256.fullmatch(value))
def _string_list(value: Any) -> list[str]: return [item for item in value if isinstance(item, str) and item.strip()] if isinstance(value, (list, tuple, set)) else []
def _session_values(session: Mapping[str, Any] | None) -> tuple[str | None, int | None]:
    if not isinstance(session, Mapping): return None, None
    session_id, epoch = session.get("session_id"), session.get("epoch")
    return (session_id.strip() if isinstance(session_id, str) and session_id.strip() else None, epoch if isinstance(epoch, int) and epoch >= 0 else None)
