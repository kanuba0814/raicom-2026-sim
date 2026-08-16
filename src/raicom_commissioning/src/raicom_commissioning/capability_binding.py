"""ADR-001 v2 capability-binding profile schema and canonical digest helpers."""
from __future__ import annotations

import hashlib
import json
from typing import Any, Mapping

SCHEMA_VERSION = "raicom.capability-binding/v2"
SOURCES = frozenset({"REAL", "SIM", "REPLAY", "FAKE"})
EXECUTION_MODES = frozenset({"ACTUATE", "OBSERVE_ONLY", "REPLAY", "DRY_RUN", "DISABLED"})

# ``config_digest`` is deliberately not part of its own digest preimage.
# JSON's canonical separators and sorted keys make this stable across key order.
def canonical_profile_payload(profile: Mapping[str, Any]) -> bytes:
    """Return the canonical, self-digest-free serialized profile payload."""
    payload = {key: value for key, value in profile.items() if key != "config_digest"}
    return json.dumps(payload, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode("utf-8")


def profile_config_digest(profile: Mapping[str, Any]) -> str:
    """Return the SHA-256 digest over :func:`canonical_profile_payload`."""
    return hashlib.sha256(canonical_profile_payload(profile)).hexdigest()


CAPABILITY_BINDING_SCHEMA: dict[str, Any] = {
    "$schema": "https://json-schema.org/draft/2020-12/schema",
    "$id": SCHEMA_VERSION,
    "type": "object",
    "required": ["schema_version", "profile_id", "config_digest", "safety_policy_ref", "safety_policy", "capabilities"],
    "properties": {
        "schema_version": {"const": SCHEMA_VERSION},
        "profile_id": {"type": "string", "minLength": 1},
        "config_digest": {"type": "string", "pattern": "^[0-9a-f]{64}$"},
        "safety_policy_ref": {"type": "string", "minLength": 1},
        "safety_policy": {
            "type": "object",
            "required": ["policy_id", "version", "digest"],
            "properties": {
                "policy_id": {"type": "string", "minLength": 1},
                "version": {"type": "string", "minLength": 1},
                "digest": {"type": "string", "pattern": "^[0-9a-f]{64}$"},
            },
        },
        "hardware_identity": {"type": "string", "minLength": 1},
        "capabilities": {"type": "object", "additionalProperties": {"type": "object", "required": ["adapter_id", "adapter_digest", "source", "execution_mode", "verification_ref", "deadlines", "freshness_policy"]}},
    },
}
