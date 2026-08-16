from datetime import datetime, timezone
from copy import deepcopy

from raicom_commissioning.admission import ADMITTED, DENIED, evaluate_scoped_admission
from raicom_commissioning.capability_binding import profile_config_digest

NOW = datetime(2026, 1, 1, tzinfo=timezone.utc)
D = "a" * 64


def real_inputs():
    profile = {"schema_version": "raicom.capability-binding/v2", "profile_id": "real-test", "config_digest": "",
               "safety_policy_ref": "safety", "safety_policy": {"policy_id": "safety", "version": "v1", "digest": "b" * 64},
               "hardware_identity": "serial-1", "capabilities": {"drive": {"adapter_id": "drive-v1", "adapter_digest": D, "source": "REAL", "execution_mode": "ACTUATE", "verification_ref": "v", "authorization_ref": "a", "deadlines": {"operation_seconds": 10, "cancel_seconds": 1}, "freshness_policy": {"max_age_seconds": 60}}}}
    profile["config_digest"] = profile_config_digest(profile)
    manifest = {"adapter_id": "drive-v1", "adapter_digest": D, "actuating": True, "actuator_domains": ["base"]}
    common = {"adapter_id": "drive-v1", "adapter_digest": D, "hardware_identity": "serial-1", "profile_id": "real-test", "config_digest": profile["config_digest"], "operation_scope": ["drive"], "session_id": "session-1", "session_epoch": 2}
    verification = dict(common, status="VERIFIED", kind="HCC", observed_at="2026-01-01T00:00:00Z", expires_at="2026-01-01T01:00:00Z")
    authorization = dict(common, status="ACTIVE", expires_at="2026-01-01T01:00:00Z", actuator_domains=["base"])
    return profile, {"drive-v1": manifest}, {"v": verification}, {"a": authorization}, {"profile_id": "real-test", "config_digest": profile["config_digest"], "safety_policy": profile["safety_policy"]}, {"session_id": "session-1", "epoch": 2}


def evaluate(profile, manifests, verifications, authorizations, trusted, session):
    return evaluate_scoped_admission(profile, manifests, ["drive"], safety_admission={"status": ADMITTED}, verification_records=verifications, authorization_records=authorizations, trusted_profile=trusted, coordinator_session=session, now=NOW)


def test_real_record_full_binding_admits():
    assert evaluate(*real_inputs())["status"] == ADMITTED


def test_tampered_profile_and_replayed_record_are_denied():
    args = list(real_inputs()); args[0]["safety_policy"]["version"] = "tampered"
    result = evaluate(*args)
    assert result["status"] == DENIED and "profile_config_digest_mismatch" in result["reasons"]
    args = list(real_inputs()); args[2]["v"]["session_id"] = "old-session"
    assert "verification_session_id_mismatch" in evaluate(*args)["reasons"]


def test_real_requires_identity_session_epoch_and_exact_scope():
    for field, value, reason in [("hardware_identity", "", "hardware_identity_missing"), ("session", {"session_id": "session-1"}, "coordinator_epoch_missing")]:
        args = list(real_inputs())
        if field == "hardware_identity": args[0][field] = value
        else: args[5] = value
        assert reason in evaluate(*args)["reasons"]
    args = list(real_inputs()); args[2]["v"]["operation_scope"] = ["other"]
    assert "verification_operation_scope_mismatch" in evaluate(*args)["reasons"]
    args = list(real_inputs()); args[3]["a"]["actuator_domains"] = ["base", "arm"]
    assert "authorization_domains_not_exact" in evaluate(*args)["reasons"]


def test_dependency_cycle_and_invalid_dependency_have_stable_reasons():
    profile, manifests, verifications, authorizations, trusted, session = real_inputs()
    profile["capabilities"]["support"] = deepcopy(profile["capabilities"]["drive"])
    profile["capabilities"]["support"]["source"] = "FAKE"; profile["capabilities"]["support"]["execution_mode"] = "DRY_RUN"
    manifests["drive-v1"] = dict(manifests["drive-v1"], dependencies=["support"])
    manifests["support-v1"] = {"adapter_id": "support-v1", "adapter_digest": "c" * 64, "actuating": False, "dependencies": ["drive"]}
    profile["capabilities"]["support"].update(adapter_id="support-v1", adapter_digest="c" * 64)
    profile["config_digest"] = profile_config_digest(profile); trusted["config_digest"] = profile["config_digest"]
    result = evaluate(profile, manifests, verifications, authorizations, trusted, session)
    assert "dependency_cycle:drive->support->drive" in result["reasons"]
    manifests["drive-v1"]["dependencies"] = [None]
    result = evaluate(profile, manifests, verifications, authorizations, trusted, session)
    assert "dependency_invalid" in result["reasons"]


def test_development_profile_remains_valid_without_real_trust_inputs():
    import json
    from pathlib import Path
    profile = json.loads((Path(__file__).parent / "../config/capability_binding_development.json").read_text())
    manifests = {
        "navigation-replay-v1": {"adapter_id": "navigation-replay-v1", "adapter_digest": "1" * 64, "actuating": False},
        "manipulation-dry-run-v1": {"adapter_id": "manipulation-dry-run-v1", "adapter_digest": "2" * 64, "actuating": False},
    }
    result = evaluate_scoped_admission(profile, manifests, ["navigation_replay"], safety_admission={"status": ADMITTED}, now=NOW)
    assert result["status"] == ADMITTED and result["authorization"] == "NONE"
