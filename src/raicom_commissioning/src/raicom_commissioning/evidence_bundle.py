"""Pure bounded HCC evidence-bundle structural contract; no artifact dereference or signature verification."""
from __future__ import annotations

from hashlib import sha256
import math
import re
from types import MappingProxyType
from typing import Any, Mapping

EVIDENCE_BUNDLE_SCHEMA_VERSION = "raicom.hcc-evidence-bundle/v1"
_MAX_ARTIFACT_ID = 128
_MAX_IDENTITY = 128
_MAX_HARDWARE_IDS = 32
_SHA256 = re.compile(r"^[0-9a-f]{64}$")


class EvidenceBundleValidationError(ValueError):
    """Evidence bundle is not an exact bounded canonical structural record."""


def canonical_evidence_bytes(value: Any) -> bytes:
    # Import lazily to retain one canonical JSON implementation without an import cycle.
    from .trust_catalog import canonical_json_bytes
    return canonical_json_bytes(value)


def evidence_bundle_digest(value: Mapping[str, Any]) -> str:
    return sha256(canonical_evidence_bytes(value)).hexdigest()


def validate_evidence_bundle(value: Any, *, now_s: float | int) -> Mapping[str, Any]:
    """Validate and deeply freeze a production evidence bundle.

    This deliberately establishes structure and bindings only. Attestation keys,
    artifact contents, URIs, and signatures are never read or verified here.
    """
    if not _finite(now_s): raise EvidenceBundleValidationError("bundle_now_invalid")
    required = {"schema_version", "artifact", "producer", "hardware_ids", "verification", "observed_at", "expires_at", "attestation", "binding"}
    if not isinstance(value, dict) or set(value) != required or value.get("schema_version") != EVIDENCE_BUNDLE_SCHEMA_VERSION:
        raise EvidenceBundleValidationError("bundle_schema_invalid")
    artifact = value["artifact"]
    if not isinstance(artifact, dict) or set(artifact) != {"artifact_id", "uri", "sha256"} or not _bounded(artifact.get("artifact_id"), _MAX_ARTIFACT_ID) or not _uri(artifact.get("uri")) or not _digest(artifact.get("sha256")):
        raise EvidenceBundleValidationError("bundle_artifact_invalid")
    producer = value["producer"]
    if not isinstance(producer, dict) or set(producer) != {"identity"} or not _bounded(producer.get("identity"), _MAX_IDENTITY): raise EvidenceBundleValidationError("bundle_producer_invalid")
    hardware_ids = value["hardware_ids"]
    if not isinstance(hardware_ids, list) or not hardware_ids or len(hardware_ids) > _MAX_HARDWARE_IDS or any(not _bounded(item, _MAX_IDENTITY) for item in hardware_ids) or len(set(hardware_ids)) != len(hardware_ids): raise EvidenceBundleValidationError("bundle_hardware_ids_invalid")
    if value["verification"] != "VERIFIED": raise EvidenceBundleValidationError("bundle_not_verified")
    observed, expires = value["observed_at"], value["expires_at"]
    if not _finite(observed) or not _finite(expires) or float(observed) > float(now_s) or float(expires) <= float(now_s) or float(expires) <= float(observed): raise EvidenceBundleValidationError("bundle_freshness_invalid")
    attestation = value["attestation"]
    if not isinstance(attestation, dict) or set(attestation) != {"id", "signer_identity", "sha256"} or not _bounded(attestation.get("id"), _MAX_ARTIFACT_ID) or not _bounded(attestation.get("signer_identity"), _MAX_IDENTITY) or not _digest(attestation.get("sha256")): raise EvidenceBundleValidationError("bundle_attestation_invalid")
    binding = value["binding"]
    binding_keys = {"adapter_id", "adapter_digest", "measurement_algorithm", "measurement_digest", "catalog_root_sha256", "profile_id", "hardware_identity", "operation_scope", "session_id", "session_epoch", "provider_id"}
    if not isinstance(binding, dict) or set(binding) != binding_keys: raise EvidenceBundleValidationError("bundle_binding_schema_invalid")
    strings = ("adapter_id", "measurement_algorithm", "profile_id", "hardware_identity", "session_id", "provider_id")
    if any(not _bounded(binding.get(k), _MAX_IDENTITY) for k in strings) or not _digest(binding.get("adapter_digest")) or not _digest(binding.get("measurement_digest")) or not _digest(binding.get("catalog_root_sha256")) or not isinstance(binding["session_epoch"], int) or isinstance(binding["session_epoch"], bool) or binding["session_epoch"] < 0: raise EvidenceBundleValidationError("bundle_binding_invalid")
    scope = binding["operation_scope"]
    if not isinstance(scope, list) or not scope or len(scope) > 128 or any(not _bounded(item, _MAX_IDENTITY) for item in scope) or scope != sorted(scope) or len(set(scope)) != len(scope): raise EvidenceBundleValidationError("bundle_scope_invalid")
    return _freeze(value)


def _bounded(value: Any, maximum: int) -> bool: return isinstance(value, str) and bool(value.strip()) and len(value) <= maximum
def _digest(value: Any) -> bool: return isinstance(value, str) and bool(_SHA256.fullmatch(value))
def _finite(value: Any) -> bool: return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(float(value))
def _uri(value: Any) -> bool:
    if not _bounded(value, 512): return False
    # Trusted relative references have no traversal/absolute form. urn: is an
    # explicit non-dereferenced identifier policy, not a fetch instruction.
    return value.startswith("urn:") or (not value.startswith("/") and ":" not in value and all(part not in ("", ".", "..") for part in value.split("/")))
def _freeze(value: Any) -> Any:
    if isinstance(value, dict): return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list): return tuple(_freeze(item) for item in value)
    return value
