import json

from raicom_state_server.mission import (
    MAX_CAPABILITIES,
    MAX_DISPATCH_ITEMS,
    MAX_READINESS_REASON_ITEMS,
    MAX_SNAPSHOT_UTF8_BYTES,
    MAX_STRING_UTF8_BYTES,
    normalize_mission_snapshot,
)


def assert_safe_defaults(snapshot):
    assert snapshot["mode"] == "UNKNOWN"
    assert snapshot["state"] == "UNKNOWN"
    assert snapshot["dispatch_queue"] == []
    assert snapshot["readiness_reasons"] == []
    assert snapshot["capabilities"] == {}
    assert snapshot["referee"]["state"] == "WAITING"
    assert snapshot["safety"]["state"] == "UNKNOWN"


def test_json_snapshot_byte_limit_is_rejected_before_parse():
    payload = json.dumps({"mode": "AUTO", "padding": "x" * MAX_SNAPSHOT_UTF8_BYTES})
    assert_safe_defaults(normalize_mission_snapshot(payload))


def test_deep_json_snapshot_is_rejected_without_partial_promotion():
    payload = '{"mode":"AUTO","nested":' + ("[" * 16) + "0" + ("]" * 16) + "}"
    assert_safe_defaults(normalize_mission_snapshot(payload))


def test_oversized_collections_and_strings_become_atomic_safe_defaults():
    cases = [
        {"mode": "AUTO", "dispatch_queue": ["pick"] * (MAX_DISPATCH_ITEMS + 1)},
        {"mode": "AUTO", "readiness_reasons": ["reason"] * (MAX_READINESS_REASON_ITEMS + 1)},
        {"mode": "AUTO", "capabilities": {str(index): {} for index in range(MAX_CAPABILITIES + 1)}},
        {"mode": "AUTO", "task": "x" * (MAX_STRING_UTF8_BYTES + 1)},
        {"mode": "AUTO", "unknown_map": {str(index): "ok" for index in range(33)}},
    ]
    for payload in cases:
        assert_safe_defaults(normalize_mission_snapshot(payload))


def test_invalid_capability_data_does_not_promote_other_fields():
    snapshot = normalize_mission_snapshot({
        "mode": "AUTO",
        "capabilities": {"base": {"verification": "x" * (MAX_STRING_UTF8_BYTES + 1)}},
    })
    assert_safe_defaults(snapshot)
