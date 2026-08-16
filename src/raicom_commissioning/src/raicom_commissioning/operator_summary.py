"""Read-only, deterministic operator readiness presentation.

This module only formats supplied health and commissioning evidence.  It has no
ROS, shell, or actuator interface and its output always carries authorization
``NONE``.
"""
from __future__ import annotations

import json
from typing import Any, Mapping

from .readiness import NOT_READY, READY_FOR_REAL_COMMISSIONING

_VERIFICATION_SCOPES = frozenset({"SIM_TESTED", "HW_UNVERIFIED", "UNKNOWN", "UNAVAILABLE"})
_CAPABILITY_STATUSES = frozenset({"PASS", "DEGRADED", "UNKNOWN", "HW_UNVERIFIED", "UNAVAILABLE"})
_FAKE_BACKEND_KINDS = frozenset({"FAKE", "FakeBackend"})


def build_operator_readiness_summary(
    capability_health: Mapping[str, Any] | None,
    real_readiness: Mapping[str, Any] | None,
    *,
    backend_kind: str | None,
    verification_scope: str | None,
    safety: Mapping[str, Any] | None = None,
    referee_waiting: bool | None = None,
) -> dict[str, Any]:
    """Build a fail-closed field-checklist summary from supplied observations.

    ``real_readiness`` must be the result of the existing REAL readiness
    evaluator.  A FakeBackend is explicitly never commissioning-ready even if
    an incorrectly supplied result says READY.  This is presentation only; it
    cannot authorize movement or invoke commands.
    """
    normalized_backend = _string_or_unknown(backend_kind)
    normalized_scope = _verification_scope(verification_scope)
    capabilities = _capabilities(capability_health)
    supplied_status = real_readiness.get("status") if isinstance(real_readiness, Mapping) else None
    supplied_reasons = real_readiness.get("reasons") if isinstance(real_readiness, Mapping) else None
    readiness_reasons = _string_list(supplied_reasons)

    is_fake_backend = normalized_backend in _FAKE_BACKEND_KINDS
    if supplied_status != READY_FOR_REAL_COMMISSIONING:
        readiness_status = NOT_READY
        if not readiness_reasons:
            readiness_reasons.append("real_readiness_not_ready_or_unavailable")
    elif is_fake_backend:
        readiness_status = NOT_READY
        readiness_reasons.append("fake_backend_is_not_commissioning_ready")
    else:
        readiness_status = READY_FOR_REAL_COMMISSIONING

    estop_state = _estop_state(safety)
    referee_state = _referee_wait_state(referee_waiting)
    return {
        "schema_version": "raicom.operator-readiness-summary/v1",
        "backend": {"kind": normalized_backend, "verification_scope": normalized_scope},
        "authorization": "NONE",
        "commissioning": {
            "status": readiness_status,
            "real_not_ready_reasons": sorted(set(readiness_reasons)) if readiness_status == NOT_READY else [],
        },
        "safety": {"estop": estop_state, "referee_wait": referee_state},
        "capabilities": capabilities,
    }


def operator_readiness_json(*args: Any, **kwargs: Any) -> str:
    """Render :func:`build_operator_readiness_summary` as stable compact JSON."""
    return json.dumps(build_operator_readiness_summary(*args, **kwargs), sort_keys=True, separators=(",", ":"))


def _capabilities(capability_health: Mapping[str, Any] | None) -> list[dict[str, Any]]:
    if not isinstance(capability_health, Mapping):
        return [{"name": "capability_health", "status": "UNAVAILABLE", "reason": "health_input_unavailable"}]
    result = []
    for name in sorted(capability_health, key=str):
        item = capability_health[name]
        if isinstance(item, Mapping):
            status = item.get("status")
            reason = item.get("reason")
        else:
            status, reason = None, "health_entry_invalid"
        result.append({
            "name": str(name),
            "status": status if status in _CAPABILITY_STATUSES else "UNKNOWN",
            "reason": reason if isinstance(reason, str) and reason else "reason_unavailable",
        })
    return result


def _verification_scope(value: str | None) -> str:
    return value if value in _VERIFICATION_SCOPES else "UNKNOWN"


def _string_or_unknown(value: str | None) -> str:
    return value if isinstance(value, str) and value else "UNKNOWN"


def _string_list(value: Any) -> list[str]:
    return [item for item in value if isinstance(item, str) and item] if isinstance(value, list) else []


def _estop_state(safety: Mapping[str, Any] | None) -> str:
    if not isinstance(safety, Mapping):
        return "UNKNOWN"
    estop = safety.get("estop")
    if estop is True:
        return "ACTIVE"
    if estop is False:
        return "CLEAR"
    return "UNKNOWN"


def _referee_wait_state(referee_waiting: bool | None) -> str:
    if referee_waiting is True:
        return "WAITING"
    if referee_waiting is False:
        return "NOT_WAITING"
    return "UNKNOWN"
