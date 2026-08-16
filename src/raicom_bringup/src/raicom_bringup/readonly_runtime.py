"""Sealed, non-actuating readonly runtime for SIM-development diagnostics.

This module deliberately has no mission-controller, navigation adapter, ROS, or
hardware integration imports. Its backend is a sealed semantic boundary.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import hashlib
import json
import math
import os
from pathlib import Path
import secrets
import stat
from datetime import datetime, timezone
from typing import Any, Callable, Mapping, Optional

from raicom_core.models import GraspEvidence, GraspState, OperationResult, SafeHoldEvidence, SafeHoldResult
from raicom_perception.replay import ReplayConfig, ReplayDetector, ReplayFrame

_MAX_SNAPSHOT_BYTES = 4096
_MAX_MANIFEST_BYTES = 16384
_JOURNAL_NAME = "events.jsonl"
_MANIFEST_NAME = "manifest.json"
_SCHEMA = "raicom.readonly_mission_snapshot.v2"
_MANIFEST_SCHEMA = "raicom.readonly_runtime_manifest"
_MANIFEST_VERSION = 1
_PROFILE = "SIM_DEVELOPMENT_READONLY"
_DIGEST_LENGTH = 64
_CONSTRUCTION_MARKER = object()
# Test-only seam: invoked after the run directory is descriptor-opened.  Production
# code leaves it as a no-op; tests use it to prove pathname replacement cannot split
# the journal and manifest across directories.
_after_run_descriptor_open_for_test: Callable[[str, Path, int], None] = lambda _run_id, _directory, _descriptor: None
_CAPABILITY_FIELDS = frozenset({
    "source", "mode", "adapter_identity", "config_digest", "verification",
    "verification_reference", "authorization", "authorization_reference",
    "freshness_decision", "structurally_non_actuating", "status", "color",
    "promotion_allowed",
})


class ReadonlyRuntimeError(ValueError):
    """Configuration or fresh-run admission failed closed."""


@dataclass(frozen=True)
class TurntableConfig:
    max_observation_age_s: float
    required_execution_budget_s: float
    max_time_to_exit_s: float

    def __post_init__(self) -> None:
        for name in ("max_observation_age_s", "required_execution_budget_s", "max_time_to_exit_s"):
            _finite_number(getattr(self, name), "turntable." + name, float.fromhex("0x1.0p-1022"))


@dataclass(frozen=True)
class RecoveryConfig:
    navigation_retries: int
    reobserve_attempts: int
    charge_retries: int
    action_timeout_s: float

    def __post_init__(self) -> None:
        for name in ("navigation_retries", "reobserve_attempts", "charge_retries"):
            _nonnegative_integer(getattr(self, name), "recovery." + name)
        _finite_number(self.action_timeout_s, "recovery.action_timeout_s", 0.0)


@dataclass(frozen=True)
class RuntimeConfig:
    settlement_threshold: int
    snapshot_rate_hz: float
    replay: ReplayConfig
    turntable: TurntableConfig
    recovery: RecoveryConfig
    profile: str = _PROFILE
    config_digest: str = field(init=False)
    _construction_marker: object | None = field(init=False, repr=False, compare=False, default=None)

    def __post_init__(self) -> None:
        object.__setattr__(self, "config_digest", _canonical_digest(self._validated_payload()))
        object.__setattr__(self, "_construction_marker", _CONSTRUCTION_MARKER)

    def _validated_payload(self) -> dict[str, Any]:
        if self.profile != _PROFILE:
            raise ReadonlyRuntimeError("runtime profile must be SIM_DEVELOPMENT_READONLY")
        threshold = _nonnegative_integer(self.settlement_threshold, "settlement_threshold")
        rate = _finite_number(self.snapshot_rate_hz, "snapshot_rate_hz", 0.1, 10.0)
        if not isinstance(self.replay, ReplayConfig):
            raise TypeError("replay must be ReplayConfig")
        if not isinstance(self.turntable, TurntableConfig):
            raise TypeError("turntable must be TurntableConfig")
        if not isinstance(self.recovery, RecoveryConfig):
            raise TypeError("recovery must be RecoveryConfig")
        # Revalidate nested values at this trust boundary in case construction was
        # bypassed with object.__setattr__.
        self.replay.__post_init__()
        _validate_replay_mapping(_replay_payload(self.replay))
        self.turntable.__post_init__()
        self.recovery.__post_init__()
        return _validated_config_payload(threshold, rate, self.replay, self.turntable, self.recovery)

    @classmethod
    def from_mapping(cls, values: Mapping[str, Any]) -> "RuntimeConfig":
        expected = {"profile", "settlement_threshold", "snapshot_rate_hz", "replay", "turntable", "recovery"}
        if not isinstance(values, Mapping) or set(values) != expected or values["profile"] != _PROFILE:
            raise ReadonlyRuntimeError("runtime config must contain exactly the documented readonly development keys")
        replay_values = _exact_mapping(values["replay"], {
            "class_mapping", "minimum_confidence", "minimum_stability", "required_consecutive_frames",
            "maximum_frame_age_s", "model_path", "model_sha256", "model_verification",
        }, "replay")
        turntable_values = _exact_mapping(values["turntable"], {"max_observation_age_s", "required_execution_budget_s", "max_time_to_exit_s"}, "turntable")
        recovery_values = _exact_mapping(values["recovery"], {"navigation_retries", "reobserve_attempts", "charge_retries", "action_timeout_s"}, "recovery")
        try:
            return cls(values["settlement_threshold"], values["snapshot_rate_hz"],
                       ReplayConfig(**dict(replay_values)), TurntableConfig(**dict(turntable_values)),
                       RecoveryConfig(**dict(recovery_values)), values["profile"])
        except (TypeError, ValueError) as error:
            raise ReadonlyRuntimeError("invalid readonly runtime configuration") from error


@dataclass(frozen=True, slots=True)
class FakeNavigationSemanticAdapter:
    """Typed FAKE navigation semantic boundary; it contains no transport client."""
    source: str = "FAKE"
    mode: str = "DISABLED"


@dataclass(frozen=True, slots=True)
class DryRunManipulationSemanticAdapter:
    """Typed FAKE manipulation semantic boundary; it contains no actuator seam."""
    source: str = "FAKE"
    mode: str = "DRY_RUN"


class NonActuatingReadonlyBackend:
    """Final sealed backend: every operation is non-actuating and unavailable.

    Typed semantic adapters make provenance concrete while retaining no I/O-capable
    navigation or manipulation implementation. ReplayDetector is the only delegated
    capability and consumes recorded frame data only.
    """
    __slots__ = ("_navigation", "_detector", "_manipulation", "_frame", "_estop")

    def __init_subclass__(cls, **kwargs: object) -> None:
        raise TypeError("NonActuatingReadonlyBackend is sealed")

    def __init__(self, replay: ReplayConfig) -> None:
        self._navigation = FakeNavigationSemanticAdapter()
        self._detector = ReplayDetector(replay, now=lambda: 0.0)
        self._manipulation = DryRunManipulationSemanticAdapter()
        self._frame = ReplayFrame("readonly-empty", 0.0, camera_available=False)
        self._estop = False

    def _unavailable(self) -> OperationResult:
        return OperationResult.SAFETY_DENIED if self._estop else OperationResult.UNAVAILABLE

    def cancel_active_operation(self) -> OperationResult: return self._unavailable()
    def cancel_navigation(self) -> OperationResult: return self._unavailable()
    def goto(self, waypoint: str, deadline_s: Optional[float] = None) -> OperationResult: return self._unavailable()
    def clear_costmap(self) -> OperationResult: return self._unavailable()
    def track(self, marker_id: str, goal_distance_m: float) -> OperationResult: return self._unavailable()
    def relative_move(self, dx_m: float, dy_m: float, dyaw_rad: float) -> OperationResult: return self._unavailable()
    def grasp(self, target: str, deadline_s: Optional[float] = None) -> OperationResult: return self._unavailable()
    def place(self, target: str, deadline_s: Optional[float] = None) -> OperationResult: return self._unavailable()
    def pose(self) -> None: return None

    def safe_hold(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        return SafeHoldResult(SafeHoldEvidence.UNCONFIRMED, "readonly local hold; no physical stop evidence")

    def stop_all(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        return self.safe_hold(deadline_s)

    def emergency_stop(self) -> None:
        self._estop = True

    def health(self) -> Mapping[str, str]:
        return {"status": "UNAVAILABLE", "verification": "HW_UNVERIFIED", "authorization": "NONE"}

    def navigation_status(self) -> tuple[Optional[OperationResult], Optional[str], str]:
        return self._unavailable(), None, "readonly fake navigation is disabled"

    def perceive(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        # ReplayDetector owns only deterministic recorded-frame interpretation.
        result = self._detector.detect(self._frame)
        return OperationResult.SUCCEEDED if result.state.value == "DETECTED" else OperationResult.UNAVAILABLE

    def grasp_evidence(self, target: str) -> GraspEvidence:
        return GraspEvidence(GraspState.UNKNOWN, target, 0.0, "DRY_RUN", "no manipulator operation is permitted")


@dataclass(frozen=True, slots=True)
class ReadonlyRuntime:
    """Public readonly handle: metadata and bounded snapshot only."""
    run_id: str
    config: RuntimeConfig
    _journal_relative_path: str
    _backend: NonActuatingReadonlyBackend

    def snapshot_json(self) -> str:
        def capability(source: str, mode: str, adapter_identity: str, config_digest: str, **extra: str) -> dict[str, Any]:
            result = {
                "source": source, "mode": mode, "adapter_identity": adapter_identity,
                "config_digest": config_digest, "verification": "HW_UNVERIFIED",
                "verification_reference": "HW_UNVERIFIED", "authorization": "NONE",
                "authorization_reference": "NONE", "freshness_decision": "NOT_APPLICABLE",
                "structurally_non_actuating": True, "status": "UNAVAILABLE", "color": "GRAY",
                "promotion_allowed": False,
            }
            result.update(extra)
            return result

        replay_payload = _replay_payload(self.config.replay)
        payload = {
            "schema": _SCHEMA, "profile": _PROFILE, "run_id": self.run_id,
            "config_digest": self.config.config_digest,
            # Public, bounded, nonsecret input required for offline replay of the
            # runtime digest.  It is not an instruction to instantiate a runtime.
            "config_provenance": self.config._validated_payload(),
            "REAL_ACTIVE": False, "authorization": "NONE", "task_result": "NOT_REAL_SUCCESS",
            "real_availability": "REAL_UNAVAILABLE", "field_validation": "NOT_ASSERTED",
            "mission": {"state": "BOOT", "assignment_id": "UNKNOWN", "accepted_score": 0, "version": 0,
                        "estop_latched": False, "resumable": False, "admission_open": False},
            "capabilities": {
                "navigation": capability("FAKE", "DISABLED", "sealed.fake_navigation.v1",
                                         _canonical_digest({"adapter_identity": "sealed.fake_navigation.v1", "mode": "DISABLED"})),
                "perception": capability("REPLAY", "REPLAY", "sealed.replay_detector.v1",
                                         _canonical_digest(replay_payload), model_path=self.config.replay.model_path,
                                         model_sha256=self.config.replay.model_sha256,
                                         model_verification=self.config.replay.model_verification,
                                         replay_mapping_digest=_canonical_digest(dict(self.config.replay.class_mapping))),
                "manipulation": capability("FAKE", "DRY_RUN", "sealed.dry_run_manipulation.v1",
                                             _canonical_digest({"adapter_identity": "sealed.dry_run_manipulation.v1", "mode": "DRY_RUN"})),
            },

        }
        _validate_snapshot_schema(payload)
        encoded = _canonical_json(payload)
        if len(encoded.encode("utf-8")) > _MAX_SNAPSHOT_BYTES:
            raise ReadonlyRuntimeError("bounded snapshot exceeded")
        return encoded


def load_runtime_config(path: Path | str) -> RuntimeConfig:
    try:
        with Path(path).open("r", encoding="utf-8") as config_file:
            return RuntimeConfig.from_mapping(json.load(config_file))
    except BaseException as error:
        if isinstance(error, ReadonlyRuntimeError):
            raise
        raise ReadonlyRuntimeError("invalid runtime config") from error


def create_readonly_runtime(artifact_root: Path | str, config: RuntimeConfig) -> ReadonlyRuntime:
    if not isinstance(config, RuntimeConfig): raise TypeError("config must be RuntimeConfig")
    if getattr(config, "_construction_marker", None) is not _CONSTRUCTION_MARKER:
        raise ReadonlyRuntimeError("runtime config was not constructed through RuntimeConfig")
    try:
        expected_digest = _canonical_digest(config._validated_payload())
    except (TypeError, ValueError) as error:
        raise ReadonlyRuntimeError("runtime config invariants are invalid") from error
    if config.config_digest != expected_digest:
        raise ReadonlyRuntimeError("runtime config digest is forged or stale")
    root = _trusted_root(artifact_root)
    root_descriptor = _open_directory(root)
    try:
        run_id, directory = _new_run_directory_at(root_descriptor, root)
        run_descriptor = _open_directory_at(root_descriptor, run_id)
        try:
            _after_run_descriptor_open_for_test(run_id, directory, run_descriptor)
            _create_empty_file_at(run_descriptor, _JOURNAL_NAME)
            backend = NonActuatingReadonlyBackend(config.replay)
            if (type(backend) is not NonActuatingReadonlyBackend
                    or type(backend._navigation) is not FakeNavigationSemanticAdapter
                    or type(backend._detector) is not ReplayDetector
                    or type(backend._manipulation) is not DryRunManipulationSemanticAdapter):
                raise ReadonlyRuntimeError("readonly composition requires sealed concrete semantic adapters")
            runtime = ReadonlyRuntime(run_id, config, _JOURNAL_NAME, backend)
            raw_snapshot = runtime.snapshot_json()
            _write_manifest_at(run_descriptor, run_id, config.config_digest, raw_snapshot)
            return runtime
        finally:
            os.close(run_descriptor)
    finally:
        os.close(root_descriptor)


def sealed_snapshot_to_mission_display(snapshot: Mapping[str, Any] | str) -> dict[str, Any]:
    """Project sealed runtime provenance into the state-server display contract.

    This pure bridge intentionally exposes only the bounded fields consumed by the
    existing `/raicom/mission/snapshot` subscriber; the complete sealed record is
    retained verbatim in the run manifest.
    """
    if isinstance(snapshot, str):
        try:
            snapshot = json.loads(snapshot)
        except (TypeError, ValueError):
            snapshot = {}
    if not isinstance(snapshot, Mapping):
        snapshot = {}
    mission = snapshot.get("mission")
    if not isinstance(mission, Mapping):
        mission = {}
    capabilities = snapshot.get("capabilities")
    if not isinstance(capabilities, Mapping):
        capabilities = {}

    def token(value: Any, default: str) -> str:
        return value if isinstance(value, str) and value and len(value.encode("utf-8")) <= 128 else default

    displayed_capabilities = {}
    for name in ("navigation", "perception", "manipulation"):
        raw = capabilities.get(name)
        raw = raw if isinstance(raw, Mapping) else {}
        displayed_capabilities[name] = {
            "source": token(raw.get("source"), "NOT_CONFIGURED"),
            "mode": token(raw.get("mode"), "NOT_CONFIGURED"),
            "verification": token(raw.get("verification"), "HW_UNVERIFIED"),
            "authorization": token(raw.get("authorization"), "NOT_AUTHORIZED"),
        }
    return {
        "mode": token(snapshot.get("profile"), "NOT_CONFIGURED"),
        "state": token(mission.get("state"), "UNKNOWN"),
        "task": token(mission.get("assignment_id"), "UNKNOWN"),
        "referee": {"state": "WAITING", "reason": "REFEREE_NOT_CONFIGURED"},
        "safety": {"state": "STOP_UNCONFIRMED", "estop": bool(mission.get("estop_latched", False)), "motion_allowed": False},
        "dispatch_queue": [],
        "readiness_reasons": ["NON_ACTUATING_RUNTIME", "HW_UNVERIFIED"],
        "capabilities": displayed_capabilities,
    }


def _open_directory(path: Path) -> int:
    try:
        return os.open(str(path), os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0))
    except OSError as error:
        raise ReadonlyRuntimeError("trusted artifact root cannot be opened") from error


def _open_directory_at(parent_descriptor: int, name: str) -> int:
    try:
        return os.open(name, os.O_RDONLY | os.O_DIRECTORY | getattr(os, "O_NOFOLLOW", 0), dir_fd=parent_descriptor)
    except OSError as error:
        raise ReadonlyRuntimeError("fresh run directory cannot be opened") from error


def _create_empty_file_at(directory_descriptor: int, name: str) -> None:
    descriptor = None
    try:
        descriptor = os.open(name, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600, dir_fd=directory_descriptor)
        os.fsync(descriptor)
        os.close(descriptor)
        descriptor = None
        os.fsync(directory_descriptor)
    except OSError as error:
        raise ReadonlyRuntimeError("fresh run journal cannot be created") from error
    finally:
        if descriptor is not None:
            os.close(descriptor)


def _parse_canonical_snapshot(raw_snapshot: str, run_id: str, config_digest: str) -> dict[str, Any]:
    if not isinstance(raw_snapshot, str):
        raise ReadonlyRuntimeError("raw snapshot must be a JSON string")

    def reject_duplicate_keys(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result = {}
        for key, value in pairs:
            if key in result:
                raise ReadonlyRuntimeError("raw snapshot contains duplicate JSON keys")
            result[key] = value
        return result

    try:
        snapshot = json.loads(raw_snapshot, object_pairs_hook=reject_duplicate_keys,
                              parse_constant=lambda _value: (_ for _ in ()).throw(ValueError("nonstandard JSON constant")))
    except (TypeError, ValueError, json.JSONDecodeError) as error:
        if isinstance(error, ReadonlyRuntimeError):
            raise
        raise ReadonlyRuntimeError("raw snapshot is malformed") from error
    if not isinstance(snapshot, dict) or _canonical_json(snapshot) != raw_snapshot:
        raise ReadonlyRuntimeError("raw snapshot must be canonical JSON")
    _validate_snapshot_schema(snapshot)
    if snapshot["run_id"] != run_id or snapshot["config_digest"] != config_digest:
        raise ReadonlyRuntimeError("raw snapshot identity does not match manifest arguments")
    return snapshot


def _write_manifest_at(directory_descriptor: int, run_id: str, config_digest: str, raw_snapshot: str) -> None:
    snapshot = _parse_canonical_snapshot(raw_snapshot, run_id, config_digest)
    canonical_snapshot = _canonical_json(snapshot).encode("utf-8")
    snapshot_sha256 = hashlib.sha256(canonical_snapshot).hexdigest()
    manifest = {
        "schema": _MANIFEST_SCHEMA,
        "version": _MANIFEST_VERSION,
        "run_id": run_id,
        "config_digest": config_digest,
        "raw_sealed_snapshot": snapshot,
        "snapshot_sha256": snapshot_sha256,
        "created_at": datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z"),
        "REAL_ACTIVE": False,
        "authorization": "NONE",
        "field_validation": "NOT_ASSERTED",
    }
    encoded = _canonical_json(manifest).encode("utf-8")
    if len(encoded) > _MAX_MANIFEST_BYTES:
        raise ReadonlyRuntimeError("bounded manifest exceeded")
    try:
        descriptor = os.open(_MANIFEST_NAME, os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0), 0o600, dir_fd=directory_descriptor)
        try:
            offset = 0
            while offset < len(encoded):
                written = os.write(descriptor, encoded[offset:])
                if written <= 0:
                    raise OSError("manifest write made no progress")
                offset += written
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        os.fsync(directory_descriptor)
    except OSError as error:
        raise ReadonlyRuntimeError("manifest creation failed closed") from error


def _exact_mapping(value: Any, expected: set[str], name: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or set(value) != expected:
        raise ReadonlyRuntimeError("{} config keys are malformed".format(name))
    return value


def _validate_replay_mapping(values: Mapping[str, Any]) -> None:
    mapping = values["class_mapping"]
    if not isinstance(mapping, Mapping) or not 1 <= len(mapping) <= 64:
        raise ReadonlyRuntimeError("replay.class_mapping must contain 1..64 entries")
    for raw, material in mapping.items():
        if (not isinstance(raw, str) or not isinstance(material, str)
                or not 1 <= len(raw) <= 128 or not 1 <= len(material) <= 128):
            raise ReadonlyRuntimeError("replay.class_mapping keys and values must be bounded strings")
    _finite_number(values["minimum_confidence"], "replay.minimum_confidence", 0.0, 1.0)
    _finite_number(values["minimum_stability"], "replay.minimum_stability", 0.0, 1.0)
    if isinstance(values["required_consecutive_frames"], bool) or not isinstance(values["required_consecutive_frames"], int) or not 1 <= values["required_consecutive_frames"] <= 1000:
        raise ReadonlyRuntimeError("replay.required_consecutive_frames must be within 1..1000")
    _finite_number(values["maximum_frame_age_s"], "replay.maximum_frame_age_s", 0.0, 3600.0)
    _development_model_path(values["model_path"])
    if (not isinstance(values["model_sha256"], str)
            or (values["model_sha256"] and (len(values["model_sha256"]) != _DIGEST_LENGTH
                                             or any(character not in "0123456789abcdef" for character in values["model_sha256"])) )):
        raise ReadonlyRuntimeError("replay.model_sha256 must be lowercase hexadecimal when supplied")
    if values["model_verification"] not in {"NOT_FIELD_VALIDATED", "UNVERIFIED"}:
        raise ReadonlyRuntimeError("replay.model_verification is malformed or upgrades verification")


def _canonical_json(value: Any) -> str:
    """Canonical JSON for public nonsecret provenance and deterministic digests."""
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True, allow_nan=False)


def _canonical_digest(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value).encode("utf-8")).hexdigest()


def _replay_payload(replay: ReplayConfig) -> dict[str, Any]:
    return {
        "class_mapping": dict(replay.class_mapping),
        "minimum_confidence": replay.minimum_confidence,
        "minimum_stability": replay.minimum_stability,
        "required_consecutive_frames": replay.required_consecutive_frames,
        "maximum_frame_age_s": replay.maximum_frame_age_s,
        "model_path": replay.model_path,
        "model_sha256": replay.model_sha256,
        "model_verification": replay.model_verification,
    }


def _validated_config_payload(threshold: int, rate: float, replay: ReplayConfig,
                              turntable: TurntableConfig, recovery: RecoveryConfig) -> dict[str, Any]:
    return {
        "profile": _PROFILE,
        "settlement_threshold": threshold,
        "snapshot_rate_hz": rate,
        "replay": _replay_payload(replay),
        "turntable": {
            "max_observation_age_s": turntable.max_observation_age_s,
            "required_execution_budget_s": turntable.required_execution_budget_s,
            "max_time_to_exit_s": turntable.max_time_to_exit_s,
        },
        "recovery": {
            "navigation_retries": recovery.navigation_retries,
            "reobserve_attempts": recovery.reobserve_attempts,
            "charge_retries": recovery.charge_retries,
            "action_timeout_s": recovery.action_timeout_s,
        },
    }


def _development_model_path(value: Any) -> None:
    if not isinstance(value, str) or len(value) > 512 or "\x00" in value:
        raise ReadonlyRuntimeError("replay.model_path must be bounded development path string")
    # This is display-only provenance, not an openable path. It is intentionally
    # limited to relative development paths to avoid embedding host locations.
    if value and (Path(value).is_absolute() or any(part == ".." for part in Path(value).parts)):
        raise ReadonlyRuntimeError("replay.model_path must be a relative development path")


def _validate_snapshot_schema(payload: Mapping[str, Any]) -> None:
    required_top = {"schema", "profile", "run_id", "config_digest", "config_provenance", "REAL_ACTIVE", "authorization", "task_result", "real_availability", "field_validation", "mission", "capabilities"}
    if set(payload) != required_top or payload["schema"] != _SCHEMA or payload["profile"] != _PROFILE:
        raise ReadonlyRuntimeError("snapshot schema is malformed")
    if (not isinstance(payload["run_id"], str) or not payload["run_id"]
            or not isinstance(payload["config_digest"], str)
            or len(payload["config_digest"]) != _DIGEST_LENGTH
            or any(character not in "0123456789abcdef" for character in payload["config_digest"])):
        raise ReadonlyRuntimeError("snapshot identity is malformed")
    try:
        provenance_config = RuntimeConfig.from_mapping(payload["config_provenance"])
    except (TypeError, ValueError, ReadonlyRuntimeError) as error:
        raise ReadonlyRuntimeError("snapshot config provenance is malformed") from error
    if provenance_config.config_digest != payload["config_digest"]:
        raise ReadonlyRuntimeError("snapshot config provenance digest is malformed")
    if (payload["REAL_ACTIVE"] is not False or payload["authorization"] != "NONE"
            or payload["task_result"] != "NOT_REAL_SUCCESS"
            or payload["real_availability"] != "REAL_UNAVAILABLE"
            or payload["field_validation"] != "NOT_ASSERTED"):
        raise ReadonlyRuntimeError("snapshot global readonly fields are malformed")
    mission = payload["mission"]
    if not isinstance(mission, Mapping) or mission != {
            "state": "BOOT", "assignment_id": "UNKNOWN", "accepted_score": 0,
            "version": 0, "estop_latched": False, "resumable": False,
            "admission_open": False}:
        raise ReadonlyRuntimeError("snapshot mission fields are malformed")
    capabilities = payload["capabilities"]
    if not isinstance(capabilities, Mapping) or set(capabilities) != {"navigation", "perception", "manipulation"}:
        raise ReadonlyRuntimeError("snapshot capabilities are malformed")
    for name, capability in capabilities.items():
        expected_fields = _CAPABILITY_FIELDS | ({"model_path", "model_sha256", "model_verification", "replay_mapping_digest"} if name == "perception" else set())
        if not isinstance(capability, Mapping) or set(capability) != expected_fields:
            raise ReadonlyRuntimeError("snapshot capability provenance is malformed")
        for field in ("source", "mode", "adapter_identity", "config_digest", "verification", "verification_reference", "authorization", "authorization_reference", "freshness_decision", "status", "color"):
            if not isinstance(capability[field], str) or not capability[field] or len(capability[field]) > 512:
                raise ReadonlyRuntimeError("snapshot capability provenance is malformed")
        if (len(capability["config_digest"]) != _DIGEST_LENGTH
                or any(character not in "0123456789abcdef" for character in capability["config_digest"])
                or capability["verification"] != "HW_UNVERIFIED"
                or capability["authorization"] != "NONE"
                or capability["structurally_non_actuating"] is not True
                or capability["promotion_allowed"] is not False):
            raise ReadonlyRuntimeError("snapshot capability readonly fields are malformed")
        if name == "perception":
            if (len(capability["replay_mapping_digest"]) != _DIGEST_LENGTH
                    or not isinstance(capability["model_path"], str)
                    or not isinstance(capability["model_sha256"], str)
                    or capability["model_verification"] not in {"NOT_FIELD_VALIDATED", "UNVERIFIED"}):
                raise ReadonlyRuntimeError("snapshot replay provenance is malformed")


def _finite_number(value: Any, name: str, minimum: float, maximum: Optional[float] = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ReadonlyRuntimeError("{} must be finite number".format(name))
    if value < minimum or (maximum is not None and value > maximum):
        raise ReadonlyRuntimeError("{} is out of range".format(name))
    return float(value)


def _nonnegative_integer(value: Any, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ReadonlyRuntimeError("{} must be a non-negative integer".format(name))
    return value


def _trusted_root(value: Path | str) -> Path:
    root = Path(value)
    if not root.is_absolute() or not root.is_dir(): raise ReadonlyRuntimeError("artifact_root must be an existing absolute directory")
    current = Path(root.anchor)
    for part in root.parts[1:]:
        current /= part
        try: info = os.lstat(current)
        except OSError as error: raise ReadonlyRuntimeError("artifact_root path cannot be inspected") from error
        if stat.S_ISLNK(info.st_mode): raise ReadonlyRuntimeError("artifact_root path must not contain symlinks")
    return root


def _new_run_directory_at(root_descriptor: int, root: Path) -> tuple[str, Path]:
    run_id = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ-") + secrets.token_hex(4)
    try:
        os.mkdir(run_id, 0o700, dir_fd=root_descriptor)
        os.fsync(root_descriptor)
    except FileExistsError as error:
        raise ReadonlyRuntimeError("generated run ID already exists; refusing reuse") from error
    except OSError as error:
        raise ReadonlyRuntimeError("fresh run directory cannot be created") from error
    return run_id, root / run_id
