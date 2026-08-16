"""Conservative, read-only REAL commissioning readiness evaluation.

A READY result only records that supplied evidence meets this schema.  It is not
field validation and never authorizes robot actuation.
"""
from __future__ import annotations

from datetime import datetime, timezone
import re
from typing import Any, Mapping

READY_FOR_REAL_COMMISSIONING = "READY_FOR_REAL_COMMISSIONING"
NOT_READY = "NOT_READY"
SCHEMA_VERSION = "raicom.real-commissioning-readiness/v1"

_SHA256 = re.compile(r"^[0-9a-f]{64}$")
_REJECTED_SOURCES = {"FAKE", "SIM", "REPLAY", "DRY_RUN", "HW_UNVERIFIED", "REAL_UNRESOLVED"}
_REQUIRED_CONTRACTS = ("tf", "odom", "lidar", "camera")
_REQUIRED_SAFETY = ("estop", "cancel", "zero_motion_deadline")

# This is deliberately dependency-free JSON Schema metadata. The evaluator
# below supplies semantic validation (timestamps, freshness, and provenance).
READINESS_SCHEMA: dict[str, Any] = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "$id": SCHEMA_VERSION,
    "type": "object",
    "required": ["schema_version", "source_environment", "profile", "hardware", "contracts", "calibration", "grasp", "safety"],
    "properties": {
        "schema_version": {"const": SCHEMA_VERSION},
        "source_environment": {"type": "string", "enum": ["REAL"]},
        "profile": {"type": "object", "required": ["scope", "evidence"]},
        "hardware": {"type": "object", "required": ["ids", "configuration_sha256", "attestation"]},
        "contracts": {"type": "object"},
        "calibration": {"type": "object", "required": ["status", "evidence"]},
        "grasp": {"type": "object", "required": ["format", "fresh", "evidence"]},
        "safety": {"type": "object"},
    },
    "$defs": {
        "evidence_ref": {
            "type": "object",
            "required": ["artifact_id", "uri", "sha256", "observed_at", "expires_at", "producer", "hardware_ids", "verification", "attestation"],
        }
    },
}


def evaluate_readiness(record: Mapping[str, Any] | None, *, now: datetime | None = None) -> dict[str, Any]:
    """Evaluate a record fail-closed and return machine-readable reasons.

    ``now`` is injectable solely for deterministic tests. Missing, malformed,
    unverifiable, or expired evidence always produces ``NOT_READY``.
    """
    current_time = now or datetime.now(timezone.utc)
    if current_time.tzinfo is None:
        current_time = current_time.replace(tzinfo=timezone.utc)
    reasons: list[str] = []
    if not isinstance(record, Mapping):
        return _result(reasons + ["record_missing_or_not_an_object"])

    if record.get("schema_version") != SCHEMA_VERSION:
        reasons.append("unsupported_schema_version")
    source = record.get("source_environment")
    if source in _REJECTED_SOURCES or source != "REAL":
        reasons.append("source_environment_not_real")

    profile = record.get("profile")
    if not isinstance(profile, Mapping):
        reasons.append("profile_missing")
    else:
        if profile.get("scope") == "SIM_ONLY":
            reasons.append("sim_only_profile")
        _check_evidence(profile.get("evidence"), "profile.evidence", current_time, reasons)

    hardware = record.get("hardware")
    if not isinstance(hardware, Mapping):
        reasons.append("hardware_missing")
    else:
        if not _nonempty_strings(hardware.get("ids")):
            reasons.append("hardware_ids_missing")
        if not _valid_digest(hardware.get("configuration_sha256")):
            reasons.append("hardware_configuration_digest_missing_or_invalid")
        _check_attestation(hardware.get("attestation"), "hardware.attestation", reasons)

    contracts = record.get("contracts")
    if not isinstance(contracts, Mapping):
        reasons.append("contracts_missing")
    else:
        for name in _REQUIRED_CONTRACTS:
            contract = contracts.get(name)
            if not isinstance(contract, Mapping) or contract.get("complete") is not True:
                reasons.append(f"{name}_contract_incomplete")
            else:
                _check_evidence(contract.get("evidence"), f"contracts.{name}.evidence", current_time, reasons)

    calibration = record.get("calibration")
    if not isinstance(calibration, Mapping) or calibration.get("status") != "RESOLVED":
        reasons.append("calibration_unresolved")
    else:
        _check_evidence(calibration.get("evidence"), "calibration.evidence", current_time, reasons)

    grasp = record.get("grasp")
    if not isinstance(grasp, Mapping) or grasp.get("format") != "STRUCTURED" or grasp.get("fresh") is not True:
        reasons.append("grasp_evidence_unstructured_or_stale")
    else:
        _check_evidence(grasp.get("evidence"), "grasp.evidence", current_time, reasons)

    safety = record.get("safety")
    if not isinstance(safety, Mapping):
        reasons.append("safety_evidence_missing")
    else:
        for name in _REQUIRED_SAFETY:
            item = safety.get(name)
            if not isinstance(item, Mapping) or item.get("passed") is not True:
                reasons.append(f"{name}_evidence_missing_or_failed")
            else:
                _check_evidence(item.get("evidence"), f"safety.{name}.evidence", current_time, reasons)
    return _result(reasons)


def _result(reasons: list[str]) -> dict[str, Any]:
    return {
        "status": READY_FOR_REAL_COMMISSIONING if not reasons else NOT_READY,
        "authorization": "NONE",
        "field_validation": "NOT_ASSERTED",
        "reasons": reasons,
    }


def _check_evidence(value: Any, path: str, now: datetime, reasons: list[str]) -> None:
    if not isinstance(value, Mapping):
        reasons.append(f"{path}_missing")
        return
    required = ("artifact_id", "uri", "producer")
    if any(not isinstance(value.get(field), str) or not value[field].strip() for field in required):
        reasons.append(f"{path}_immutable_reference_incomplete")
    if not _valid_digest(value.get("sha256")):
        reasons.append(f"{path}_sha256_missing_or_invalid")
    if not _nonempty_strings(value.get("hardware_ids")):
        reasons.append(f"{path}_hardware_ids_missing")
    if value.get("verification") != "VERIFIED":
        reasons.append(f"{path}_verification_not_verified")
    _check_attestation(value.get("attestation"), f"{path}.attestation", reasons)
    observed, expires = _parse_time(value.get("observed_at")), _parse_time(value.get("expires_at"))
    if observed is None or expires is None:
        reasons.append(f"{path}_timestamps_invalid_or_missing")
    elif expires <= observed or expires <= now:
        reasons.append(f"{path}_evidence_expired")


def _check_attestation(value: Any, path: str, reasons: list[str]) -> None:
    if not isinstance(value, Mapping) or not all(isinstance(value.get(field), str) and value[field].strip() for field in ("attestation_id", "signer", "sha256")) or not _valid_digest(value.get("sha256")):
        reasons.append(f"{path}_missing_or_invalid")


def _parse_time(value: Any) -> datetime | None:
    if not isinstance(value, str):
        return None
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
        return parsed if parsed.tzinfo else None
    except ValueError:
        return None


def _valid_digest(value: Any) -> bool:
    return isinstance(value, str) and bool(_SHA256.fullmatch(value))


def _nonempty_strings(value: Any) -> bool:
    return isinstance(value, list) and bool(value) and all(isinstance(item, str) and item.strip() for item in value)
