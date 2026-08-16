"""Pure provider custody provenance structure; it is explicitly NOT authentication."""
from __future__ import annotations

import math
import re
from types import MappingProxyType
from typing import Any, Mapping

PROVIDER_PROVENANCE_SCHEMA_VERSION = "raicom.provider-provenance/v1"
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_MAX_STEPS = 8
_MAX_TEXT = 128


class ProviderProvenanceValidationError(ValueError):
    """Custody chain is structurally incomplete, stale, or mismatched."""


def validate_provider_provenance(value: Any, *, expected_purpose: str, expected_initial_producer: str,
                                 expected_terminal_provider: str, expected_evaluator: str,
                                 bundle_digest: str, artifact_digest: str, now_s: float | int) -> Mapping[str, Any]:
    """Validate custody ordering/continuity only; this authenticates nobody."""
    if not _finite(now_s): raise ProviderProvenanceValidationError("provenance_now_invalid")
    if (not all(_text(item) for item in (expected_purpose, expected_initial_producer,
            expected_terminal_provider, expected_evaluator)) or not _digest(bundle_digest)
            or not _digest(artifact_digest)):
        raise ProviderProvenanceValidationError("provenance_expected_invalid")
    if (not isinstance(value, dict) or set(value) != {"schema_version", "purpose", "provider_id", "steps"}
            or value.get("schema_version") != PROVIDER_PROVENANCE_SCHEMA_VERSION
            or value.get("purpose") != expected_purpose or value.get("provider_id") != expected_terminal_provider):
        raise ProviderProvenanceValidationError("provenance_schema_invalid")
    steps = value["steps"]
    if not isinstance(steps, list) or not steps or len(steps) > _MAX_STEPS: raise ProviderProvenanceValidationError("provenance_steps_invalid")
    previous_custodian = None
    observed_previous = None
    ids: set[str] = set()
    for index, step in enumerate(steps):
        keys = {"step_id", "producer_id", "custodian_id", "evaluator_id", "bundle_sha256", "artifact_sha256", "observed_at", "expires_at"}
        if not isinstance(step, dict) or set(step) != keys or any(not _text(step.get(k)) for k in ("step_id", "producer_id", "custodian_id", "evaluator_id")) or not _digest(step.get("bundle_sha256")) or not _digest(step.get("artifact_sha256")) or not _finite(step.get("observed_at")) or not _finite(step.get("expires_at")): raise ProviderProvenanceValidationError("provenance_step_invalid")
        if step["step_id"] in ids: raise ProviderProvenanceValidationError("provenance_duplicate_step")
        ids.add(step["step_id"])
        if step["bundle_sha256"] != bundle_digest or step["artifact_sha256"] != artifact_digest: raise ProviderProvenanceValidationError("provenance_digest_mismatch")
        if float(step["observed_at"]) > float(now_s) or float(step["expires_at"]) <= float(now_s) or float(step["expires_at"]) <= float(step["observed_at"]): raise ProviderProvenanceValidationError("provenance_freshness_invalid")
        if observed_previous is not None and float(step["observed_at"]) < observed_previous: raise ProviderProvenanceValidationError("provenance_time_reordered")
        if index == 0 and step["producer_id"] != expected_initial_producer: raise ProviderProvenanceValidationError("provenance_initial_producer_mismatch")
        if previous_custodian is not None and step["producer_id"] != previous_custodian: raise ProviderProvenanceValidationError("provenance_custody_gap")
        previous_custodian, observed_previous = step["custodian_id"], float(step["observed_at"])
        if index == len(steps) - 1:
            if step["custodian_id"] != expected_terminal_provider: raise ProviderProvenanceValidationError("provenance_terminal_provider_mismatch")
            if step["evaluator_id"] != expected_evaluator: raise ProviderProvenanceValidationError("provenance_terminal_evaluator_mismatch")
    return _freeze(value)


def _text(value: Any) -> bool: return isinstance(value, str) and bool(value.strip()) and len(value) <= _MAX_TEXT
def _digest(value: Any) -> bool: return isinstance(value, str) and bool(_SHA256.fullmatch(value))
def _finite(value: Any) -> bool: return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))
def _freeze(value: Any) -> Any:
    if isinstance(value, dict): return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list): return tuple(_freeze(item) for item in value)
    return value
