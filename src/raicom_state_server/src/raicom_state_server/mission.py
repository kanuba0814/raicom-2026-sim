"""Schema boundary for read-only semantic mission snapshots.

This module accepts only bounded data-bearing mission state.  It has no command APIs.
"""
from __future__ import annotations

import copy
import json
import math

UNKNOWN = "UNKNOWN"
MAX_SNAPSHOT_UTF8_BYTES = 8192
MAX_NESTING_DEPTH = 8
MAX_MAP_ITEMS = 32
MAX_CAPABILITIES = 16
MAX_DISPATCH_ITEMS = 16
MAX_READINESS_REASON_ITEMS = 16
MAX_STRING_UTF8_BYTES = 128
_CAPABILITY_DEFAULT = {
    "source": "NOT_CONFIGURED",
    "mode": UNKNOWN,
    "verification": "HW_UNVERIFIED",
    "authorization": "NOT_AUTHORIZED",
}


def _safe_snapshot():
    return {
        "mode": UNKNOWN,
        "state": UNKNOWN,
        "task": UNKNOWN,
        "primitive": UNKNOWN,
        "dispatch_queue": [],
        "readiness_reasons": [],
        "referee": {"state": "WAITING", "reason": UNKNOWN},
        "safety": {"state": UNKNOWN, "estop": None, "motion_allowed": None},
        "capabilities": {},
    }


def _is_bounded_text(value):
    if not isinstance(value, str) or not value:
        return False
    try:
        return len(value.encode("utf-8")) <= MAX_STRING_UTF8_BYTES
    except UnicodeError:
        return False


def _json_depth_within_limit(text):
    """Reject deeply nested JSON before json.loads can recurse excessively."""
    depth = 0
    in_string = False
    escaped = False
    for character in text:
        if in_string:
            if escaped:
                escaped = False
            elif character == "\\":
                escaped = True
            elif character == '"':
                in_string = False
        elif character == '"':
            in_string = True
        elif character in "[{":
            depth += 1
            if depth > MAX_NESTING_DEPTH:
                return False
        elif character in "]}":
            depth -= 1
    return True


def _is_bounded_structure(payload):
    """Validate all containers iteratively, including ignored fields, before promotion."""
    pending = [(payload, 1)]
    while pending:
        value, depth = pending.pop()
        if depth > MAX_NESTING_DEPTH:
            return False
        if isinstance(value, str):
            if not _is_bounded_text(value):
                return False
        elif isinstance(value, dict):
            if len(value) > MAX_MAP_ITEMS:
                return False
            for key, item in value.items():
                if not _is_bounded_text(key):
                    return False
                pending.append((item, depth + 1))
        elif isinstance(value, list):
            if len(value) > MAX_MAP_ITEMS:
                return False
            pending.extend((item, depth + 1) for item in value)
        elif isinstance(value, float):
            if not math.isfinite(value):
                return False
        elif value is not None and not isinstance(value, (bool, int)):
            return False
    return True


def _text(value, default=UNKNOWN):
    return value if _is_bounded_text(value) else default


def _string_list(value, maximum):
    if not isinstance(value, list) or len(value) > maximum:
        return None
    if not all(_is_bounded_text(item) for item in value):
        return None
    return list(value)


def normalize_mission_snapshot(payload):
    """Return a bounded snapshot, atomically falling back on invalid input."""
    if isinstance(payload, str):
        try:
            if len(payload.encode("utf-8")) > MAX_SNAPSHOT_UTF8_BYTES or not _json_depth_within_limit(payload):
                return _safe_snapshot()
            payload = json.loads(payload)
        except (TypeError, ValueError, UnicodeError):
            return _safe_snapshot()
    if not isinstance(payload, dict) or not _is_bounded_structure(payload):
        return _safe_snapshot()

    raw_capabilities = payload.get("capabilities", {})
    raw_dispatch_queue = payload.get("dispatch_queue", [])
    raw_readiness_reasons = payload.get("readiness_reasons", [])
    raw_referee = payload.get("referee", {})
    raw_safety = payload.get("safety", {})
    if (not isinstance(raw_capabilities, dict) or len(raw_capabilities) > MAX_CAPABILITIES or
            not isinstance(raw_referee, dict) or not isinstance(raw_safety, dict)):
        return _safe_snapshot()
    dispatch_queue = _string_list(raw_dispatch_queue, MAX_DISPATCH_ITEMS)
    readiness_reasons = _string_list(raw_readiness_reasons, MAX_READINESS_REASON_ITEMS)
    if dispatch_queue is None or readiness_reasons is None:
        return _safe_snapshot()

    capabilities = {}
    for name in sorted(raw_capabilities):
        raw = raw_capabilities[name]
        if not _is_bounded_text(name) or not isinstance(raw, dict):
            return _safe_snapshot()
        capability = copy.deepcopy(_CAPABILITY_DEFAULT)
        for key in capability:
            value = raw.get(key, capability[key])
            if key in raw and not _is_bounded_text(value):
                return _safe_snapshot()
            capability[key] = _text(value, capability[key])
        capabilities[name] = capability

    for field in ("mode", "state", "task", "primitive"):
        if field in payload and not _is_bounded_text(payload[field]):
            return _safe_snapshot()
    for field in ("state", "reason"):
        if field in raw_referee and not _is_bounded_text(raw_referee[field]):
            return _safe_snapshot()
    for field in ("state",):
        if field in raw_safety and not _is_bounded_text(raw_safety[field]):
            return _safe_snapshot()
    for field in ("estop", "motion_allowed"):
        if field in raw_safety and not isinstance(raw_safety[field], bool):
            return _safe_snapshot()

    return {
        "mode": _text(payload.get("mode")),
        "state": _text(payload.get("state")),
        "task": _text(payload.get("task")),
        "primitive": _text(payload.get("primitive")),
        "dispatch_queue": dispatch_queue,
        "readiness_reasons": readiness_reasons,
        "referee": {
            "state": _text(raw_referee.get("state"), "WAITING"),
            "reason": _text(raw_referee.get("reason")),
        },
        "safety": {
            "state": _text(raw_safety.get("state")),
            "estop": raw_safety.get("estop"),
            "motion_allowed": raw_safety.get("motion_allowed"),
        },
        "capabilities": capabilities,
    }
