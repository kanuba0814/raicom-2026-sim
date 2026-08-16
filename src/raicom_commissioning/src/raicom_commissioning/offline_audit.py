"""Bounded descriptor-relative offline audit for readonly-runtime artifacts.

``PASS`` means artifact-contract verification against a caller-supplied trusted
configuration digest.  It is not a cryptographic signature or authenticity claim
and never represents real-world success.
"""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
import os
import re
import stat
from typing import Any, Mapping

from raicom_bringup.readonly_runtime import sealed_snapshot_to_mission_display, _validate_snapshot_schema
from raicom_hmi.presenter import present, render_for_log
from raicom_state_server.mission import normalize_mission_snapshot

from .readiness import NOT_READY, evaluate_readiness

AUDIT_SCHEMA = "raicom.readonly_offline_audit/v1"
_MAX_MANIFEST_BYTES = 16384
_MAX_JOURNAL_BYTES = 1
_MAX_REASON_BYTES = 256
_MAX_REASONS = 8
_MAX_REPORT_BYTES = 4096
_RUN_ID = re.compile(r"^[0-9]{8}T[0-9]{6}Z-[0-9a-f]{8}$")
_DIGEST = re.compile(r"^[0-9a-f]{64}$")
_MANIFEST_KEYS = frozenset({
    "schema", "version", "run_id", "config_digest", "raw_sealed_snapshot",
    "snapshot_sha256", "created_at", "REAL_ACTIVE", "authorization", "field_validation",
})


@dataclass(frozen=True)
class AuditReport:
    """Immutable and size-bounded result of one offline artifact inspection."""
    status: str
    reasons: tuple[str, ...]
    run_id: str | None
    artifact_integrity: str
    task_result: str = "NOT_REAL_SUCCESS"
    real_availability: str = "REAL_UNAVAILABLE"
    field_validation: str = "NOT_ASSERTED"
    authorization: str = "NONE"
    readiness: str = NOT_READY
    admission: str = "DENIED"
    readiness_evaluation: str = "EVALUATED_FAIL_CLOSED"
    admission_evaluation: str = "NOT_EVALUATED"
    admission_reason: str = "identity_and_session_facts_absent; not fabricated"

    def __post_init__(self) -> None:
        object.__setattr__(self, "status", self.status if self.status in {"PASS", "FAIL"} else "FAIL")
        object.__setattr__(self, "run_id", self.run_id if isinstance(self.run_id, str) and _RUN_ID.fullmatch(self.run_id) else None)
        bounded = tuple(_bounded_text(reason, _MAX_REASON_BYTES) for reason in self.reasons[:_MAX_REASONS]
                        if isinstance(reason, str))
        object.__setattr__(self, "reasons", bounded or (("audit failed",) if self.status == "FAIL" else ()))
        object.__setattr__(self, "artifact_integrity", self.artifact_integrity if self.artifact_integrity in {"ARTIFACT_CONTRACT_PASS", "FAILED"} else "FAILED")
        if len(self.canonical_json().encode("utf-8")) > _MAX_REPORT_BYTES:
            # Fixed fields leave ample headroom; this is a fail-closed invariant.
            object.__setattr__(self, "reasons", ("audit report exceeds bound",))

    def to_dict(self) -> dict[str, Any]:
        return {"schema": AUDIT_SCHEMA, "status": self.status, "reasons": list(self.reasons), "run_id": self.run_id,
                "artifact_integrity": self.artifact_integrity, "task_result": self.task_result,
                "real_availability": self.real_availability, "field_validation": self.field_validation,
                "authorization": self.authorization, "readiness": self.readiness, "admission": self.admission,
                "readiness_evaluation": self.readiness_evaluation, "admission_evaluation": self.admission_evaluation,
                "admission_reason": self.admission_reason}

    def canonical_json(self) -> str:
        return _canonical_json(self.to_dict())


def audit_readonly_manifest(artifact_root: str | os.PathLike[str], *, expected_config_digest: str | None,
                            run_id: str | None = None, manifest_path: str | os.PathLike[str] | None = None) -> AuditReport:
    """Verify one artifact against an out-of-band lowercase SHA-256 expectation.

    The root is opened from the filesystem anchor one component at a time; every
    later operation remains descriptor-relative.  A missing or malformed trusted
    expectation cannot produce ``PASS``.
    """
    selected_run_id: str | None = None
    try:
        if not isinstance(expected_config_digest, str) or not _DIGEST.fullmatch(expected_config_digest):
            raise ValueError("trusted expected config digest is missing or invalid")
        if (run_id is None) == (manifest_path is None):
            raise ValueError("exactly one of run_id or manifest_path is required")
        selected_run_id, manifest_name = _selector(run_id, manifest_path)
        root_fd = _open_trusted_root(artifact_root)
        try:
            run_fd = _open_directory_at(root_fd, selected_run_id)
            try:
                manifest_raw = _read_regular_at(run_fd, manifest_name, _MAX_MANIFEST_BYTES)
                journal_raw = _read_regular_at(run_fd, "events.jsonl", _MAX_JOURNAL_BYTES)
            finally:
                os.close(run_fd)
        finally:
            os.close(root_fd)
        if journal_raw:
            raise ValueError("readonly journal is nonempty")
        manifest = _canonical_object(manifest_raw, "manifest")
        _validate_manifest(manifest, selected_run_id)
        snapshot = manifest["raw_sealed_snapshot"]
        if hashlib.sha256(_canonical_json(snapshot).encode("utf-8")).hexdigest() != manifest["snapshot_sha256"]:
            raise ValueError("snapshot hash mismatch")
        _validate_snapshot_schema(snapshot)
        if snapshot["run_id"] != selected_run_id or snapshot["config_digest"] != manifest["config_digest"]:
            raise ValueError("snapshot identity mismatch")
        if manifest["config_digest"] != expected_config_digest:
            raise ValueError("trusted config digest mismatch")
        _validate_config_provenance(snapshot, expected_config_digest)
        _validate_nonactuating_provenance(snapshot)
        _validate_offline_hmi(snapshot)
        readiness = evaluate_readiness(None)
        if readiness["status"] != NOT_READY or readiness["authorization"] != "NONE":
            raise ValueError("readiness evaluator did not fail closed")
        return AuditReport("PASS", (), selected_run_id, "ARTIFACT_CONTRACT_PASS")
    except (OSError, TypeError, ValueError, json.JSONDecodeError, UnicodeError) as error:
        # Never echo a selector unless it was validated by _selector.
        return AuditReport("FAIL", (_bounded_reason(error),), selected_run_id, "FAILED")


def _selector(run_id: str | None, manifest_path: str | os.PathLike[str] | None) -> tuple[str, str]:
    if run_id is not None:
        if not isinstance(run_id, str) or not _RUN_ID.fullmatch(run_id):
            raise ValueError("run ID is not project format")
        return run_id, "manifest.json"
    if not isinstance(manifest_path, (str, os.PathLike)):
        raise ValueError("manifest path is invalid")
    raw = os.fspath(manifest_path)
    # Deliberately lexical: no filesystem path operation is performed on selector.
    parts = raw.split("/")
    if raw.startswith("/") or parts != [parts[0], "manifest.json"] or not _RUN_ID.fullmatch(parts[0]):
        raise ValueError("manifest path must be confined run-id/manifest.json")
    return parts[0], "manifest.json"


def _open_trusted_root(value: str | os.PathLike[str]) -> int:
    """Open an absolute root from ``/`` without lstat/full-path TOCTOU checks."""
    raw = os.fspath(value)
    if not isinstance(raw, str) or not raw.startswith("/"):
        raise ValueError("artifact root must be an absolute directory")
    parts = [part for part in raw.split("/") if part]
    if any(part in {".", ".."} for part in parts):
        raise ValueError("artifact root contains invalid component")
    descriptor = os.open("/", os.O_RDONLY | os.O_DIRECTORY)
    try:
        for part in parts:
            next_descriptor = _open_directory_at(descriptor, part)
            os.close(descriptor)
            descriptor = next_descriptor
        return descriptor
    except BaseException:
        os.close(descriptor)
        raise


def _open_directory_at(parent_fd: int, name: str) -> int:
    return os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=parent_fd)


def _read_regular_at(parent_fd: int, name: str, maximum: int) -> bytes:
    fd = os.open(name, os.O_RDONLY | os.O_NOFOLLOW, dir_fd=parent_fd)
    try:
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_size > maximum:
            raise ValueError("artifact file is not regular or exceeds bound")
        data = os.read(fd, maximum + 1)
        if len(data) > maximum:
            raise ValueError("artifact file exceeds bound")
        return data
    finally:
        os.close(fd)


def _canonical_object(raw: bytes, label: str) -> dict[str, Any]:
    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise ValueError(label + " has duplicate JSON keys")
            result[key] = value
        return result
    try:
        text = raw.decode("utf-8")
        result = json.loads(text, object_pairs_hook=reject_duplicates,
                            parse_constant=lambda _value: (_ for _ in ()).throw(ValueError("nonstandard JSON constant")))
    except (UnicodeDecodeError, ValueError, json.JSONDecodeError) as error:
        raise ValueError(label + " is malformed") from error
    if not isinstance(result, dict) or _canonical_json(result) != text:
        raise ValueError(label + " is not canonical JSON")
    return result


def _validate_manifest(manifest: Mapping[str, Any], run_id: str) -> None:
    if set(manifest) != _MANIFEST_KEYS or manifest.get("schema") != "raicom.readonly_runtime_manifest" or manifest.get("version") != 1:
        raise ValueError("manifest schema/version invalid")
    if manifest.get("run_id") != run_id or not _DIGEST.fullmatch(str(manifest.get("config_digest", ""))):
        raise ValueError("manifest identity invalid")
    if (not isinstance(manifest.get("raw_sealed_snapshot"), dict) or not _DIGEST.fullmatch(str(manifest.get("snapshot_sha256", "")))
            or not isinstance(manifest.get("created_at"), str) or len(manifest["created_at"].encode("utf-8")) > 64):
        raise ValueError("manifest snapshot fields invalid")
    if manifest.get("REAL_ACTIVE") is not False or manifest.get("authorization") != "NONE" or manifest.get("field_validation") != "NOT_ASSERTED":
        raise ValueError("manifest global readonly fields invalid")


def _validate_config_provenance(snapshot: Mapping[str, Any], expected_digest: str) -> None:
    """Recompute the runtime digest from sealed public config inputs."""
    provenance = snapshot.get("config_provenance")
    # The runtime schema verifier has already applied its exact known canonical
    # rules; reproducing its digest preimage here avoids trusting self-declaration.
    if not isinstance(provenance, Mapping):
        raise ValueError("config provenance unavailable")
    actual = hashlib.sha256(_canonical_json(dict(provenance)).encode("utf-8")).hexdigest()
    if actual != expected_digest or snapshot.get("config_digest") != actual:
        raise ValueError("sealed config provenance digest mismatch")


def _validate_nonactuating_provenance(snapshot: Mapping[str, Any]) -> None:
    capabilities = snapshot["capabilities"]
    expected = {"navigation": ("FAKE", "DISABLED", "sealed.fake_navigation.v1"),
                "perception": ("REPLAY", "REPLAY", "sealed.replay_detector.v1"),
                "manipulation": ("FAKE", "DRY_RUN", "sealed.dry_run_manipulation.v1")}
    for name, (source, mode, identity) in expected.items():
        item = capabilities[name]
        if (item["source"], item["mode"], item["adapter_identity"], item["verification"], item["authorization"],
                item["structurally_non_actuating"], item["promotion_allowed"]) != (source, mode, identity, "HW_UNVERIFIED", "NONE", True, False):
            raise ValueError("capability provenance is not sealed non-actuating")
        if name == "perception":
            replay = snapshot["config_provenance"]["replay"]
            if item["config_digest"] != hashlib.sha256(_canonical_json(replay).encode("utf-8")).hexdigest() or item["replay_mapping_digest"] != hashlib.sha256(_canonical_json(replay["class_mapping"]).encode("utf-8")).hexdigest():
                raise ValueError("replay capability digest mismatch")
        else:
            expected_digest = hashlib.sha256(_canonical_json({"adapter_identity": identity, "mode": mode}).encode("utf-8")).hexdigest()
            if item["config_digest"] != expected_digest:
                raise ValueError("sealed adapter digest mismatch")


def _validate_offline_hmi(snapshot: Mapping[str, Any]) -> None:
    display = sealed_snapshot_to_mission_display(snapshot)
    normalized = normalize_mission_snapshot(display)
    view = present(normalized)
    rendered = render_for_log(normalized)
    if normalized["state"] != "BOOT" or not normalized["capabilities"] or any(item["indicator"] == "green" for item in view["capabilities"]) or len(rendered.encode("utf-8")) > 8192:
        raise ValueError("offline HMI normalization fallback or unsafe presentation")


def _canonical_json(value: Any) -> str:
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def _bounded_text(value: object, maximum: int) -> str:
    return str(value).encode("utf-8", "replace")[:maximum].decode("utf-8", "ignore")


def _bounded_reason(error: BaseException) -> str:
    return _bounded_text(str(error) or type(error).__name__, _MAX_REASON_BYTES)
