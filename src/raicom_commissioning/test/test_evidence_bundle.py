import copy
import hashlib

import pytest

from raicom_commissioning.evidence_bundle import EvidenceBundleValidationError, validate_evidence_bundle
from raicom_commissioning.provider_provenance import ProviderProvenanceValidationError, validate_provider_provenance
from raicom_commissioning.trust_catalog import canonical_json_bytes

NOW = 100.0
D = "a" * 64


def bundle():
    return {"schema_version": "raicom.hcc-evidence-bundle/v1", "artifact": {"artifact_id": "a", "uri": "hcc/a.json", "sha256": D}, "producer": {"identity": "producer"}, "hardware_ids": ["hw"], "verification": "VERIFIED", "observed_at": 99.0, "expires_at": 101.0, "attestation": {"id": "att", "signer_identity": "signer", "sha256": "b" * 64}, "binding": {"adapter_id": "drive", "adapter_digest": "c" * 64, "measurement_algorithm": "sha256", "measurement_digest": D, "catalog_root_sha256": "d" * 64, "profile_id": "p", "hardware_identity": "hw", "operation_scope": ["drive"], "session_id": "s", "session_epoch": 1, "provider_id": "provider"}}


def provenance(bundle_value, **changes):
    digest = hashlib.sha256(canonical_json_bytes(bundle_value)).hexdigest()
    value = {"schema_version": "raicom.provider-provenance/v1", "purpose": "production-admission", "provider_id": "provider", "steps": [{"step_id": "one", "producer_id": "producer", "custodian_id": "provider", "evaluator_id": "trust-catalog-production-admission", "bundle_sha256": digest, "artifact_sha256": D, "observed_at": 99.0, "expires_at": 101.0}]}
    value.update(changes)
    return value, digest


@pytest.mark.parametrize("mutate", [
    lambda x: x.update(schema_version="bad"), lambda x: x["artifact"].update(uri="../escape"),
    lambda x: x.update(hardware_ids=["hw", "hw"]), lambda x: x.update(verification="PENDING"),
    lambda x: x.update(observed_at=101.0), lambda x: x.update(expires_at=100.0),
    lambda x: x["attestation"].update(sha256="A" * 64), lambda x: x["binding"].update(operation_scope=["z", "a"]),
])
def test_bundle_rejects_malformed_binding_attestation_and_freshness(mutate):
    value = bundle(); mutate(value)
    with pytest.raises(EvidenceBundleValidationError): validate_evidence_bundle(value, now_s=NOW)


def test_bundle_is_deep_immutable():
    frozen = validate_evidence_bundle(bundle(), now_s=NOW)
    with pytest.raises(TypeError): frozen["artifact"]["uri"] = "x"


@pytest.mark.parametrize("mutate", [
    lambda x: x["steps"][0].update(bundle_sha256="0" * 64),
    lambda x: x["steps"].append(dict(x["steps"][0], step_id="one")),
    lambda x: x["steps"].append(dict(x["steps"][0], step_id="two", producer_id="wrong", observed_at=100.0)),
    lambda x: x["steps"].append(dict(x["steps"][0], step_id="two", observed_at=98.0)),
    lambda x: x["steps"][0].update(observed_at=101.0), lambda x: x["steps"][0].update(expires_at=100.0),
])
def test_provenance_rejects_digest_replay_gap_reorder_future_and_expiry(mutate):
    b = bundle(); value, digest = provenance(b); mutate(value)
    with pytest.raises(ProviderProvenanceValidationError): validate_provider_provenance(value, expected_purpose="production-admission", expected_initial_producer="producer", expected_terminal_provider="provider", expected_evaluator="trust-catalog-production-admission", bundle_digest=digest, artifact_digest=D, now_s=NOW)


@pytest.mark.parametrize("mutate", [
    lambda value: value.update(purpose="wrong-purpose"),
    lambda value: value["steps"][0].update(producer_id="wrong-producer"),
    lambda value: value.update(provider_id="wrong-provider"),
    lambda value: value["steps"][0].update(custodian_id="wrong-provider"),
    lambda value: value["steps"][0].update(artifact_sha256="0" * 64),
])
def test_provenance_requires_exact_purpose_endpoints_and_digests(mutate):
    b = bundle(); value, digest = provenance(b); mutate(value)
    with pytest.raises(ProviderProvenanceValidationError):
        validate_provider_provenance(value, expected_purpose="production-admission", expected_initial_producer="producer", expected_terminal_provider="provider", expected_evaluator="trust-catalog-production-admission", bundle_digest=digest, artifact_digest=D, now_s=NOW)


def test_provenance_positive_is_immutable_and_not_authentication():
    b = bundle(); value, digest = provenance(b)
    frozen = validate_provider_provenance(value, expected_purpose="production-admission", expected_initial_producer="producer", expected_terminal_provider="provider", expected_evaluator="trust-catalog-production-admission", bundle_digest=digest, artifact_digest=D, now_s=NOW)
    with pytest.raises(TypeError): frozen["steps"][0]["step_id"] = "changed"
