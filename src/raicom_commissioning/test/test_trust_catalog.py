import copy
import dataclasses
import hashlib
import math
import os
import socket

import pytest

from raicom_commissioning.capability_binding import profile_config_digest
from raicom_commissioning.trust_catalog import (Authorization, CatalogValidationError,
    CoordinatorSession, HccEvidence, LiveSafety, canonical_json_bytes,
    load_validated_catalog, production_real_admission, ValidatedTrustCatalog)

D = "a" * 64
NOW = 1_893_456_000.0


def catalog():
    contract = {"adapter_id": "drive", "vendor": "test"}
    adapter_digest = hashlib.sha256(canonical_json_bytes(contract)).hexdigest()
    profile = {"schema_version": "raicom.capability-binding/v2", "profile_id": "p", "safety_policy_ref": "safe", "safety_policy": {"policy_id": "safe", "version": "1", "digest": "b" * 64}, "hardware_identity": "hw", "capabilities": {"drive": {"adapter_id": "drive", "adapter_digest": adapter_digest, "source": "REAL", "execution_mode": "ACTUATE", "verification_ref": "catalog-hcc", "authorization_ref": "catalog-auth", "deadlines": {"operation_seconds": 1, "cancel_seconds": 1}, "freshness_policy": {"max_age_seconds": 60}}}}
    return {"schema_version": "raicom.trust-catalog/v1", "entries": [{"capability_id": "drive", "profile_id": "p", "profile_preimage": profile, "profile_digest": profile_config_digest(profile), "safety_policy": profile["safety_policy"], "adapter_id": "drive", "adapter_contract_preimage": contract, "adapter_digest": adapter_digest, "measurement": {"algorithm": "sha256", "expected_digest": D}, "actuating": True, "actuator_domains": ["base"], "dependencies": []}]}


def handle(value=None):
    raw = canonical_json_bytes(value or catalog())
    return load_validated_catalog(canonical_bytes=raw, expected_root_sha256=hashlib.sha256(raw).hexdigest())


def records(safety_changes=None):
    profile = catalog()["entries"][0]["profile_preimage"]
    digest = profile_config_digest(profile)
    adapter_digest = profile["capabilities"]["drive"]["adapter_digest"]
    common = {"source_id": "provider", "adapter_id": "drive", "adapter_digest": adapter_digest,
              "hardware_identity": "hw", "profile_id": "p", "config_digest": digest,
              "operation_scope": ["drive"], "session_id": "s", "session_epoch": 1,
              "expires_at": "2030-01-01T00:01:00Z"}
    evidence = dict(common, status="VERIFIED", kind="HCC", observed_at="2030-01-01T00:00:00Z")
    bundle = {"schema_version": "raicom.hcc-evidence-bundle/v1", "artifact": {"artifact_id": "hcc-run", "uri": "evidence/hcc.json", "sha256": D}, "producer": {"identity": "hcc-producer"}, "hardware_ids": ["hw"], "verification": "VERIFIED", "observed_at": NOW - 1, "expires_at": NOW + 10, "attestation": {"id": "attestation-1", "signer_identity": "hcc-signer", "sha256": "c" * 64}, "binding": {"adapter_id": "drive", "adapter_digest": adapter_digest, "measurement_algorithm": "sha256", "measurement_digest": D, "catalog_root_sha256": hashlib.sha256(canonical_json_bytes(catalog())).hexdigest(), "profile_id": "p", "hardware_identity": "hw", "operation_scope": ["drive"], "session_id": "s", "session_epoch": 1, "provider_id": "provider"}}
    bundle_digest = hashlib.sha256(canonical_json_bytes(bundle)).hexdigest()
    authorization = dict(common, status="ACTIVE", actuator_domains=["base"], bundle_sha256=bundle_digest, artifact_sha256=D)
    root = hashlib.sha256(canonical_json_bytes(catalog())).hexdigest()
    safety = {"source_id": "safety-provider", "catalog_root_sha256": root, "profile_id": "p",
              "config_digest": digest, "session_id": "s", "session_epoch": 1,
              "operation_scope": ["drive"], "capabilities": ["drive"], "actuator_domains": ["base"],
              "status": "ADMITTED", "observed_at": NOW, "max_age_seconds": 10,
              "expires_at": NOW + 10}
    safety.update(safety_changes or {})
    safety.update(bundle_sha256=bundle_digest, artifact_sha256=D)
    context_digest = __import__("raicom_commissioning.trust_catalog", fromlist=["admission_context_digest"]).admission_context_digest(bundle_digest=bundle_digest, artifact_digest=D, catalog_root_sha256=root, profile_id="p", session_id="s", session_epoch=1, operation_scope=["drive"], capabilities=["drive"], actuator_domains=["base"], purpose="production-admission")
    evidence["bundle_sha256"] = bundle_digest; evidence["artifact_sha256"] = D; evidence["admission_context_sha256"] = context_digest
    authorization["admission_context_sha256"] = context_digest; safety["admission_context_sha256"] = context_digest
    def provenance(provider, producer):
        return {"schema_version": "raicom.provider-provenance/v1", "purpose": "production-admission", "provider_id": provider, "steps": [{"step_id": provider + "-1", "producer_id": producer, "custodian_id": provider, "evaluator_id": "trust-catalog-production-admission", "bundle_sha256": bundle_digest, "artifact_sha256": D, "observed_at": NOW - 1, "expires_at": NOW + 10}]}
    return evidence, authorization, safety, bundle, provenance("provider", "hcc-producer"), provenance("safety-provider", "safety-provider")


def admit(trusted, safety_changes=None, *, now_s=NOW):
    evidence, authorization, safety, bundle, provider_provenance, safety_provenance = records(safety_changes)
    authorization_provenance = copy.deepcopy(provider_provenance)
    authorization_provenance["steps"][0]["producer_id"] = "provider"
    return production_real_admission(trusted, ["drive"], hcc_evidence=HccEvidence("provider", evidence, bundle, provider_provenance),
        session=CoordinatorSession("session-provider", "s", 1), authorization=Authorization("provider", authorization, authorization_provenance),
        live_safety=LiveSafety("safety-provider", safety, safety_provenance), now_s=now_s)


def test_canonical_vector_and_fieldless_opaque_handle():
    assert canonical_json_bytes({"b": 1, "a": "x"}) == b'{"a":"x","b":1}'
    trusted = handle()
    assert not hasattr(trusted, "root_sha256") and not hasattr(trusted, "_catalog")
    assert not hasattr(trusted, "__dict__")
    with pytest.raises(AttributeError): object.__setattr__(trusted, "_catalog", {})
    with pytest.raises(TypeError): dataclasses.replace(trusted)


def test_forgery_copy_and_borrowed_attributes_fail_closed():
    trusted = handle()
    forged = object.__new__(ValidatedTrustCatalog)
    for candidate in (forged, copy.copy(forged), copy.deepcopy(forged), copy.copy(trusted), copy.deepcopy(trusted)):
        result = production_real_admission(candidate, ["drive"], hcc_evidence=HccEvidence("v", {}, {}, {}), session=CoordinatorSession("s", "x", 1), authorization=Authorization("a", {}, {}), live_safety=LiveSafety("l", {}, {}), now_s=NOW)
        assert "validated_catalog_required" in result["reasons"]
    # Fieldless slots prevent copied attributes or a fake marker from becoming state.
    with pytest.raises(AttributeError): object.__setattr__(forged, "_marker", object())
    assert admit(trusted)["status"] == "DENIED"
    assert admit(trusted)["authorization"] == "NONE"


def test_rejects_root_mismatch_duplicates_unknown_cycles_and_digest_substitution():
    raw = canonical_json_bytes(catalog())
    with pytest.raises(CatalogValidationError, match="root_mismatch"): load_validated_catalog(canonical_bytes=raw, expected_root_sha256="0" * 64)
    with pytest.raises(CatalogValidationError, match="duplicate_key"): load_validated_catalog(canonical_bytes=b'{"schema_version":"raicom.trust-catalog/v1","schema_version":"x","entries":[]}', expected_root_sha256=hashlib.sha256(b'{"schema_version":"raicom.trust-catalog/v1","schema_version":"x","entries":[]}').hexdigest())
    changed = catalog(); changed["entries"][0]["adapter_contract_preimage"]["vendor"] = "evil"
    with pytest.raises(CatalogValidationError, match="adapter_preimage_digest_mismatch"): handle(changed)
    unknown = catalog(); unknown["entries"][0]["dependencies"] = ["missing"]
    with pytest.raises(CatalogValidationError, match="dependency_unknown"): handle(unknown)
    cycle = catalog(); cycle["entries"][0]["dependencies"] = ["drive"]
    with pytest.raises(CatalogValidationError, match="dependency_cycle"): handle(cycle)


def test_loader_rejects_fifo_and_all_symlink_components_without_reading(tmp_path):
    fifo = tmp_path / "catalog.fifo"
    os.mkfifo(fifo)
    with pytest.raises(CatalogValidationError, match="not_regular"):
        load_validated_catalog(absolute_path=str(fifo), expected_root_sha256="a" * 64)
    socket_path = tmp_path / "catalog.socket"
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as server:
        server.bind(str(socket_path))
        with pytest.raises(CatalogValidationError):
            load_validated_catalog(absolute_path=str(socket_path), expected_root_sha256="a" * 64)
    with pytest.raises(CatalogValidationError, match="not_regular"):
        load_validated_catalog(absolute_path="/dev/null", expected_root_sha256="a" * 64)
    payload = canonical_json_bytes(catalog())
    target = tmp_path / "target.json"; target.write_bytes(payload)
    final_link = tmp_path / "final-link"; final_link.symlink_to(target)
    with pytest.raises(CatalogValidationError, match="path_untrusted"):
        load_validated_catalog(absolute_path=str(final_link), expected_root_sha256=hashlib.sha256(payload).hexdigest())
    directory = tmp_path / "directory"; directory.mkdir()
    intermediate = tmp_path / "intermediate"; intermediate.symlink_to(directory, target_is_directory=True)
    with pytest.raises(CatalogValidationError, match="path_untrusted"):
        load_validated_catalog(absolute_path=str(intermediate / "catalog.json"), expected_root_sha256=hashlib.sha256(payload).hexdigest())


@pytest.mark.parametrize("changes, reason", [
    ({"source_id": "other"}, "source_id"), ({"catalog_root_sha256": "0" * 64}, "catalog_root"),
    ({"profile_id": "other"}, "profile_id"), ({"config_digest": "0" * 64}, "profile_digest"),
    ({"session_id": "other"}, "session_id"), ({"session_epoch": 2}, "epoch"),
    ({"operation_scope": ["other"]}, "scope"), ({"capabilities": ["other"]}, "capabilities"),
    ({"actuator_domains": ["arm"]}, "domains"), ({"status": "DENIED"}, "not_admitted"),
    ({"observed_at": float("nan")}, "observed_at_invalid"), ({"max_age_seconds": 0}, "max_age_invalid"),
    ({"max_age_seconds": float("inf")}, "max_age_invalid"), ({"observed_at": NOW - 11}, "stale"),
    ({"observed_at": NOW + 2}, "observed_in_future"), ({"expires_at": float("nan")}, "expires_at_invalid"),
    ({"expires_at": NOW - 1}, "expired"), ({"expires_at": NOW}, "expired"),
])
def test_live_safety_requires_exact_typed_binding_and_freshness(changes, reason):
    result = admit(handle(), changes)
    assert result["status"] == "DENIED"
    assert any(reason in item for item in result["reasons"])


def test_live_safety_uses_injected_clock_not_ambient_clock():
    trusted = handle()
    assert admit(trusted, {"observed_at": NOW - 5, "expires_at": NOW + 5}, now_s=NOW)["status"] == "DENIED"
    assert "bundle_freshness_invalid" in admit(trusted, {"observed_at": NOW - 5, "expires_at": NOW + 5}, now_s=NOW + 20)["reasons"]
    assert "live_safety_now_invalid" in admit(trusted, now_s=float("nan"))["reasons"]


def test_production_facade_stays_synthetic_and_rejects_coordinated_substitution():
    trusted = handle()
    result = admit(trusted)
    assert result["assurance"] == "SIMULATED_CONTRACT_TEST" and result["field_validation"] == "NOT_FIELD_VALIDATED"
    altered = catalog(); altered["entries"][0]["actuator_domains"] = ["arm"]
    # A new root is required; record data bound to the old root cannot be reused.
    assert "bundle_catalog_binding_mismatch" in admit(handle(altered))["reasons"]

@pytest.mark.parametrize("mutate", [
    lambda evidence, authorization, safety, bundle, hcc_provenance, safety_provenance: bundle.update(observed_at=float("nan")),
    lambda evidence, authorization, safety, bundle, hcc_provenance, safety_provenance: bundle.update(binding=[]),
])
def test_facade_denies_malformed_non_json_hcc_before_digest(mutate):
    trusted = handle()
    evidence, authorization, safety, bundle, hcc_provenance, safety_provenance = records()
    mutate(evidence, authorization, safety, bundle, hcc_provenance, safety_provenance)
    result = production_real_admission(trusted, ["drive"], hcc_evidence=HccEvidence("provider", evidence, bundle, hcc_provenance),
        session=CoordinatorSession("session-provider", "s", 1), authorization=Authorization("provider", authorization, hcc_provenance),
        live_safety=LiveSafety("safety-provider", safety, safety_provenance), now_s=NOW)
    assert result["status"] == "DENIED" and result["authorization"] == "NONE"


def test_facade_denies_finite_timestamp_outside_datetime_range():
    result = admit(handle(), now_s=1e300)
    assert result["status"] == "DENIED" and result["authorization"] == "NONE"
    assert "live_safety_now_invalid" in result["reasons"]


def test_production_facade_never_exposes_admission_or_grant():
    result = admit(handle())
    assert result["status"] == "DENIED"
    assert result["authorization"] == "NONE"
    assert result["field_validation"] == "NOT_FIELD_VALIDATED"
    assert result["real_availability"] == "UNAVAILABLE" and result["readiness"] == "NOT_READY"
    assert result["contract_evaluation"] == "WOULD_ADMIT_IF_TRUSTED_AND_FIELD_VALIDATED"
