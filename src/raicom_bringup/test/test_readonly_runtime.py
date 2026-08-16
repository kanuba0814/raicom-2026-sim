import ast
import inspect
import importlib.util
import json
import os
import subprocess
import sys
import types
import stat
from pathlib import Path
import re
import tempfile
from types import MappingProxyType
import xml.etree.ElementTree as ET

import pytest

import raicom_bringup.readonly_runtime as readonly_runtime
from raicom_perception import ReplayConfig
from raicom_bringup.readonly_runtime import (
    DryRunManipulationSemanticAdapter,
    FakeNavigationSemanticAdapter,
    NonActuatingReadonlyBackend,
    ReadonlyRuntimeError,
    RecoveryConfig,
    RuntimeConfig,
    TurntableConfig,
    create_readonly_runtime,
    load_runtime_config,
    sealed_snapshot_to_mission_display,
)
from raicom_state_server.mission import normalize_mission_snapshot
from raicom_hmi.presenter import present, render_for_log

CONFIG = {
    "profile": "SIM_DEVELOPMENT_READONLY", "settlement_threshold": 0, "snapshot_rate_hz": 1.0,
    "replay": {"class_mapping": {"development_block": "development_block"}, "minimum_confidence": 0.0,
               "minimum_stability": 1.0, "required_consecutive_frames": 1, "maximum_frame_age_s": 1.0,
               "model_path": "", "model_sha256": "", "model_verification": "NOT_FIELD_VALIDATED"},
    "turntable": {"max_observation_age_s": 0.5, "required_execution_budget_s": 0.2, "max_time_to_exit_s": 1.0},
    "recovery": {"navigation_retries": 0, "reobserve_attempts": 0, "charge_retries": 0, "action_timeout_s": 1.0},
}


def test_snapshot_has_exact_non_green_schema_and_private_controller_boundary():
    with tempfile.TemporaryDirectory() as root:
        runtime = create_readonly_runtime(root, RuntimeConfig.from_mapping(CONFIG))
        payload = json.loads(runtime.snapshot_json())
        assert len(runtime.snapshot_json().encode()) <= 4096
        assert re.fullmatch(r"\d{8}T\d{6}Z-[0-9a-f]{8}", runtime.run_id)
        assert set(payload) == {"schema", "profile", "run_id", "config_digest", "config_provenance", "REAL_ACTIVE", "authorization", "task_result", "real_availability", "field_validation", "mission", "capabilities"}
        assert payload["config_provenance"] == CONFIG
        assert re.fullmatch(r"[0-9a-f]{64}", payload["config_digest"])
        assert payload["REAL_ACTIVE"] is False
        assert (payload["authorization"], payload["task_result"], payload["real_availability"], payload["field_validation"]) == ("NONE", "NOT_REAL_SUCCESS", "REAL_UNAVAILABLE", "NOT_ASSERTED")
        base_capability_fields = {"source", "mode", "adapter_identity", "config_digest", "verification", "verification_reference", "authorization", "authorization_reference", "freshness_decision", "structurally_non_actuating", "status", "color", "promotion_allowed"}
        for name, item in payload["capabilities"].items():
            expected_fields = base_capability_fields | ({"model_path", "model_sha256", "model_verification", "replay_mapping_digest"} if name == "perception" else set())
            assert set(item) == expected_fields
            assert re.fullmatch(r"sealed\.[a-z_]+\.v1", item["adapter_identity"])
            assert re.fullmatch(r"[0-9a-f]{64}", item["config_digest"])
            assert item["verification_reference"] == "HW_UNVERIFIED"
            assert item["authorization_reference"] == "NONE"
            assert item["freshness_decision"] == "NOT_APPLICABLE"
            assert item["verification"] == "HW_UNVERIFIED" and item["promotion_allowed"] is False and item["color"] != "GREEN"
            assert item["structurally_non_actuating"] is True
        assert {name: (item["source"], item["mode"]) for name, item in payload["capabilities"].items()} == {
            "navigation": ("FAKE", "DISABLED"), "perception": ("REPLAY", "REPLAY"), "manipulation": ("FAKE", "DRY_RUN"),
        }
        perception = payload["capabilities"]["perception"]
        assert {"model_path", "model_sha256", "model_verification", "replay_mapping_digest"}.issubset(perception)
        assert re.fullmatch(r"[0-9a-f]{64}", perception["replay_mapping_digest"])
        public = {name for name, value in inspect.getmembers(runtime) if not name.startswith("_") and callable(value)}
        assert public == {"snapshot_json"}
        assert not hasattr(runtime, "controller") and not hasattr(runtime, "journal")
        assert runtime._journal_relative_path == "events.jsonl"


@pytest.mark.parametrize("pathological", [
    {"replay": []},
    {"replay": {"class_mapping": {"x" * 129: "material"}}},
    {"turntable": {"max_observation_age_s": float("nan"), "required_execution_budget_s": 0.2, "max_time_to_exit_s": 1.0}},
    {"recovery": {"navigation_retries": True, "reobserve_attempts": 0, "charge_retries": 0, "action_timeout_s": 1.0}},
])
def test_nested_config_rejects_wrong_types_nonfinite_and_oversized_values(pathological):
    values = json.loads(json.dumps(CONFIG))
    values.update(pathological)
    with pytest.raises(ReadonlyRuntimeError):
        RuntimeConfig.from_mapping(values)


def test_runtime_config_copies_and_freezes_replay_mapping_at_boundary():
    values = json.loads(json.dumps(CONFIG))
    config = RuntimeConfig.from_mapping(values)
    with tempfile.TemporaryDirectory() as root:
        runtime = create_readonly_runtime(root, config)
        snapshot_before_source_mutation = runtime.snapshot_json()
        values["replay"]["class_mapping"]["development_block"] = "mutated"
        assert runtime.snapshot_json() == snapshot_before_source_mutation
    assert config.replay.class_mapping["development_block"] == "development_block"
    assert isinstance(config.replay.class_mapping, MappingProxyType)
    with pytest.raises(TypeError):
        config.replay.class_mapping["new"] = "value"
    with pytest.raises(Exception):
        config.replay.class_mapping = {}


def test_provenance_validation_rejects_oversized_or_unsafe_values():
    for key, value in (("model_path", "/host/model.pt"), ("model_path", "../model.pt"),
                       ("model_path", "x" * 513), ("model_sha256", "A" * 64),
                       ("model_verification", "FIELD_VALIDATED")):
        values = json.loads(json.dumps(CONFIG))
        values["replay"][key] = value
        with pytest.raises(ReadonlyRuntimeError):
            RuntimeConfig.from_mapping(values)


def test_direct_runtime_constructors_enforce_invariants_and_compute_digest():
    base = RuntimeConfig.from_mapping(CONFIG)
    assert base.config_digest
    with pytest.raises((TypeError, ValueError)):
        RuntimeConfig(0, True, base.replay, base.turntable, base.recovery)
    with pytest.raises((TypeError, ValueError)):
        RuntimeConfig(0, 1.0, base.replay, TurntableConfig(float("nan"), 0.2, 1.0), base.recovery)
    with pytest.raises((TypeError, ValueError)):
        RuntimeConfig(0, 1.0, base.replay, base.turntable, RecoveryConfig(True, 0, 0, 1.0))
    sim_tested_replay = ReplayConfig(
        class_mapping={"development_block": "development_block"}, minimum_confidence=0.0,
        minimum_stability=1.0, required_consecutive_frames=1, maximum_frame_age_s=1.0,
        model_path="", model_sha256="", model_verification="SIM_TESTED",
    )
    with pytest.raises(ReadonlyRuntimeError, match="model_verification"):
        RuntimeConfig(0, 1.0, sim_tested_replay, base.turntable, base.recovery)
    unsafe_replay = object.__new__(ReplayConfig)
    object.__setattr__(unsafe_replay, "class_mapping", {"raw": {"nested": "value"}})
    object.__setattr__(unsafe_replay, "minimum_confidence", 0.0)
    object.__setattr__(unsafe_replay, "minimum_stability", 1.0)
    object.__setattr__(unsafe_replay, "required_consecutive_frames", 1)
    object.__setattr__(unsafe_replay, "maximum_frame_age_s", 1.0)
    object.__setattr__(unsafe_replay, "model_path", "")
    object.__setattr__(unsafe_replay, "model_sha256", "")
    object.__setattr__(unsafe_replay, "model_verification", "NOT_FIELD_VALIDATED")
    with pytest.raises((TypeError, ValueError)):
        RuntimeConfig(0, 1.0, unsafe_replay, base.turntable, base.recovery)


def test_runtime_config_digest_cannot_be_forged_and_runtime_rechecks_it():
    config = RuntimeConfig.from_mapping(CONFIG)
    with pytest.raises(TypeError):
        RuntimeConfig(0, 1.0, config.replay, config.turntable, config.recovery, "SIM_DEVELOPMENT_READONLY", "f" * 64)
    object.__setattr__(config, "config_digest", "f" * 64)
    with tempfile.TemporaryDirectory() as root:
        with pytest.raises(ReadonlyRuntimeError, match="digest"):
            create_readonly_runtime(root, config)


def test_runtime_rejects_object_new_configs_without_genuine_construction_marker():
    legitimate = RuntimeConfig.from_mapping(CONFIG)
    bypass = object.__new__(RuntimeConfig)
    for name in ("settlement_threshold", "snapshot_rate_hz", "replay", "turntable", "recovery", "profile", "config_digest"):
        object.__setattr__(bypass, name, getattr(legitimate, name))
    assert "_construction_marker" not in bypass.__dict__
    with tempfile.TemporaryDirectory() as root:
        with pytest.raises(ReadonlyRuntimeError, match="constructed"):
            create_readonly_runtime(root, bypass)
        object.__setattr__(bypass, "_construction_marker", object())
        with pytest.raises(ReadonlyRuntimeError, match="constructed"):
            create_readonly_runtime(root, bypass)


def test_sealed_snapshot_bridge_reaches_state_server_and_hmi_without_fallback():
    with tempfile.TemporaryDirectory() as root:
        raw_snapshot = create_readonly_runtime(root, RuntimeConfig.from_mapping(CONFIG)).snapshot_json()
    display = sealed_snapshot_to_mission_display(raw_snapshot)
    assert set(display) == {"mode", "state", "task", "referee", "safety", "dispatch_queue", "readiness_reasons", "capabilities"}
    normalized = normalize_mission_snapshot(display)
    assert normalized["capabilities"] and normalized["state"] == "BOOT"
    assert normalized["safety"] == {"state": "STOP_UNCONFIRMED", "estop": False, "motion_allowed": False}
    assert normalized["referee"] == {"state": "WAITING", "reason": "REFEREE_NOT_CONFIGURED"}
    view = present(normalized)
    assert all(capability["indicator"] != "green" for capability in view["capabilities"])
    assert "NON_ACTUATING_RUNTIME" in normalized["readiness_reasons"]
    assert len(render_for_log(normalized).encode("utf-8")) <= 8192


def test_manifest_is_immutable_confined_and_has_a_matching_raw_snapshot_digest():
    with tempfile.TemporaryDirectory() as root:
        root_path = Path(root)
        runtime = create_readonly_runtime(root_path, RuntimeConfig.from_mapping(CONFIG))
        run_directory = root_path / runtime.run_id
        manifest_path = run_directory / "manifest.json"
        manifest = json.loads(manifest_path.read_text())
        assert manifest["schema"] == "raicom.readonly_runtime_manifest" and manifest["version"] == 1
        assert manifest["run_id"] == runtime.run_id and manifest["config_digest"] == runtime.config.config_digest
        raw_snapshot = json.dumps(manifest["raw_sealed_snapshot"], sort_keys=True, separators=(",", ":"))
        assert raw_snapshot == runtime.snapshot_json()
        assert manifest["snapshot_sha256"] == __import__("hashlib").sha256(raw_snapshot.encode()).hexdigest()
        assert (manifest["REAL_ACTIVE"], manifest["authorization"], manifest["field_validation"]) == (False, "NONE", "NOT_ASSERTED")
        assert stat.S_IMODE(manifest_path.stat().st_mode) == 0o600
        assert (run_directory / "events.jsonl").read_text() == ""
        with pytest.raises(ReadonlyRuntimeError, match="manifest"):
            # A partial fresh run is deliberately retained and cannot be resumed.
            _write_manifest_at = __import__("raicom_bringup.readonly_runtime", fromlist=["_write_manifest_at"])._write_manifest_at
            descriptor = os.open(run_directory, os.O_RDONLY | os.O_DIRECTORY)
            try:
                _write_manifest_at(descriptor, runtime.run_id, runtime.config.config_digest, runtime.snapshot_json())
            finally:
                os.close(descriptor)


def test_manifest_rejects_malformed_noncanonical_and_mismatched_snapshots_before_creation(tmp_path):
    config = RuntimeConfig.from_mapping(CONFIG)
    run_id = "20000101T000000Z-deadbeef"
    valid_snapshot = json.loads(create_readonly_runtime(tmp_path, config).snapshot_json())
    cases = [
        "{malformed",  # malformed JSON
        '{"schema":"one","schema":"two"}',  # duplicate key
        json.dumps(valid_snapshot, sort_keys=True),  # noncanonical whitespace
        json.dumps(dict(reversed(list(valid_snapshot.items()))), separators=(",", ":")),  # noncanonical key order
    ]
    for field, value in (("run_id", "other-run"), ("config_digest", "f" * 64),
                         ("REAL_ACTIVE", True), ("authorization", "LIVE"),
                         ("field_validation", "FIELD_VALIDATED")):
        mutated = dict(valid_snapshot)
        mutated[field] = value
        cases.append(json.dumps(mutated, sort_keys=True, separators=(",", ":")))
    for index, raw_snapshot in enumerate(cases):
        directory = tmp_path / "manifest-input-{}".format(index)
        directory.mkdir()
        descriptor = os.open(directory, os.O_RDONLY | os.O_DIRECTORY)
        try:
            with pytest.raises(ReadonlyRuntimeError):
                readonly_runtime._write_manifest_at(descriptor, run_id, config.config_digest, raw_snapshot)
        finally:
            os.close(descriptor)
        assert not (directory / "manifest.json").exists()


def test_run_path_replacement_cannot_split_descriptor_relative_journal_and_manifest(monkeypatch, tmp_path):
    config = RuntimeConfig.from_mapping(CONFIG)
    moved_directories = []

    def replace_run_path(_run_id, directory, _descriptor):
        moved = directory.with_name(directory.name + "-original")
        os.rename(directory, moved)
        directory.mkdir()
        moved_directories.append(moved)

    monkeypatch.setattr(readonly_runtime, "_after_run_descriptor_open_for_test", replace_run_path)
    runtime = create_readonly_runtime(tmp_path, config)
    assert len(moved_directories) == 1
    original, replacement = moved_directories[0], tmp_path / runtime.run_id
    assert (original / "events.jsonl").read_bytes() == b""
    assert (original / "manifest.json").is_file()
    assert list(replacement.iterdir()) == []


def test_snapshot_provenance_digest_is_canonical_and_bounded():
    first = RuntimeConfig.from_mapping(CONFIG)
    reordered = json.loads(json.dumps(CONFIG))
    reordered["replay"]["class_mapping"] = {"development_block": "development_block"}
    second = RuntimeConfig.from_mapping(reordered)
    assert first.config_digest == second.config_digest
    with tempfile.TemporaryDirectory() as root:
        assert len(create_readonly_runtime(root, first).snapshot_json().encode("utf-8")) <= 4096


def test_load_config_converts_all_boundary_failures_to_bounded_error(monkeypatch, tmp_path):
    config_path = tmp_path / "config.json"
    config_path.write_text(json.dumps(CONFIG))
    monkeypatch.setattr("raicom_bringup.readonly_runtime.json.load", lambda _: (_ for _ in ()).throw(KeyboardInterrupt()))
    with pytest.raises(ReadonlyRuntimeError, match="invalid runtime config"):
        load_runtime_config(config_path)


def test_root_symlink_and_collision_fail_closed(monkeypatch):
    with tempfile.TemporaryDirectory() as root:
        config = RuntimeConfig.from_mapping(CONFIG)
        link = Path(root) / "link"; link.symlink_to(root, target_is_directory=True)
        with pytest.raises(ReadonlyRuntimeError): create_readonly_runtime(link, config)
        monkeypatch.setattr("raicom_bringup.readonly_runtime.secrets.token_hex", lambda _: "deadbeef")
        directory = Path(root) / "20000101T000000Z-deadbeef"; directory.mkdir()
        monkeypatch.setattr("raicom_bringup.readonly_runtime.datetime", type("Clock", (), {"now": staticmethod(lambda _: __import__("datetime").datetime(2000, 1, 1))}))
        with pytest.raises(ReadonlyRuntimeError, match="refusing"): create_readonly_runtime(root, config)


def test_manifest_symlink_race_fails_closed_and_retains_nonresumable_partial_run(monkeypatch, tmp_path):
    fixed_id = "20000101T000000Z-deadbeef"
    monkeypatch.setattr("raicom_bringup.readonly_runtime.secrets.token_hex", lambda _: "deadbeef")
    monkeypatch.setattr("raicom_bringup.readonly_runtime.datetime", type("Clock", (), {"now": staticmethod(lambda _: __import__("datetime").datetime(2000, 1, 1))}))
    original_create = readonly_runtime._create_empty_file_at

    def create_journal_then_manifest_symlink(descriptor, name):
        original_create(descriptor, name)
        os.symlink("outside", "manifest.json", dir_fd=descriptor)

    monkeypatch.setattr(readonly_runtime, "_create_empty_file_at", create_journal_then_manifest_symlink)
    with pytest.raises(ReadonlyRuntimeError, match="manifest"):
        create_readonly_runtime(tmp_path, RuntimeConfig.from_mapping(CONFIG))
    partial = tmp_path / fixed_id
    assert partial.is_dir() and partial.joinpath("events.jsonl").read_text() == ""
    assert partial.joinpath("manifest.json").is_symlink()
    monkeypatch.setattr(readonly_runtime, "_create_empty_file_at", original_create)
    with pytest.raises(ReadonlyRuntimeError, match="refusing"):
        create_readonly_runtime(tmp_path, RuntimeConfig.from_mapping(CONFIG))


def test_runtime_import_graph_is_replay_only_without_ros_or_composition():
    workspace = Path(__file__).parents[3]
    environment = dict(os.environ, PYTHONPATH=os.pathsep.join(str(workspace / "src" / name / "src") for name in ("raicom_bringup", "raicom_core", "raicom_perception")))
    code = "import sys; import raicom_bringup.readonly_runtime; print('\\n'.join(sorted(sys.modules)))"
    completed = subprocess.run([sys.executable, "-c", code], env=environment, text=True, capture_output=True, check=True)
    forbidden = ("composition", "thin_mission_slice", "SimBackend", "sim_navigation", "actionlib", "rospy")
    assert not any(any(token in module for token in forbidden) for module in completed.stdout.splitlines())


def test_script_ast_permits_only_one_static_publisher_and_no_dynamic_ros_registration():
    script_source = (Path(__file__).parents[1] / "scripts/sim_readonly_runtime.py").read_text()
    tree = ast.parse(script_source)
    aliases = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            aliases.update({alias.asname or alias.name: alias.name for alias in node.names})
        elif isinstance(node, ast.ImportFrom) and node.module:
            aliases.update({alias.asname or alias.name: node.module + "." + alias.name for alias in node.names})
    calls = []
    for node in ast.walk(tree):
        if isinstance(node, ast.Call):
            if isinstance(node.func, ast.Attribute) and isinstance(node.func.value, ast.Name):
                calls.append(aliases.get(node.func.value.id, node.func.value.id) + "." + node.func.attr)
            elif isinstance(node.func, ast.Name):
                calls.append(aliases.get(node.func.id, node.func.id))
            elif isinstance(node.func, ast.Attribute) and node.func.attr in {"Publisher", "Subscriber", "Service", "ServiceProxy", "ActionClient", "SimpleActionClient"}:
                pytest.fail("dynamic ROS registration is forbidden")
    assert calls.count("rospy.Publisher") == 1
    assert not set(calls).intersection({"rospy.Subscriber", "rospy.Service", "rospy.ServiceProxy", "rospy.ActionClient", "rospy.SimpleActionClient", "rospy.get_param", "rospy.set_param", "getattr"})


def test_sealed_backend_is_exact_and_in_memory_only():
    with pytest.raises(TypeError):
        class AttemptedSubclass(NonActuatingReadonlyBackend):
            pass
    with tempfile.TemporaryDirectory() as root:
        runtime = create_readonly_runtime(root, RuntimeConfig.from_mapping(CONFIG))
        assert type(runtime._backend) is NonActuatingReadonlyBackend
        assert type(runtime._backend._navigation) is FakeNavigationSemanticAdapter
        assert type(runtime._backend._detector).__name__ == "ReplayDetector"
        assert type(runtime._backend._manipulation) is DryRunManipulationSemanticAdapter
        with pytest.raises((AttributeError, TypeError)):
            runtime._backend._navigation.mode = "LIVE"


def test_ros_graph_construction_registers_exactly_one_publisher(monkeypatch, tmp_path):
    registrations = []

    class FakePublisher:
        def __init__(self, *args, **kwargs): registrations.append((args, kwargs))
        def publish(self, value): pass

    fake_ros = types.SimpleNamespace(init_node=lambda *a, **k: None, Publisher=FakePublisher,
                                     Rate=lambda rate: types.SimpleNamespace(sleep=lambda: None),
                                     is_shutdown=lambda: True, logerr=lambda *a, **k: None)
    class FakeString:
        def __init__(self, data): self.data = data
    monkeypatch.setitem(sys.modules, "rospy", fake_ros)
    monkeypatch.setitem(sys.modules, "std_msgs", types.ModuleType("std_msgs"))
    message_module = types.ModuleType("std_msgs.msg"); message_module.String = FakeString
    monkeypatch.setitem(sys.modules, "std_msgs.msg", message_module)
    config_path = tmp_path / "config.json"; config_path.write_text(json.dumps(CONFIG))
    spec = importlib.util.spec_from_file_location("readonly_script", Path(__file__).parents[1] / "scripts/sim_readonly_runtime.py")
    module = importlib.util.module_from_spec(spec); assert spec.loader is not None; spec.loader.exec_module(module)
    monkeypatch.setattr(sys, "argv", ["readonly", "--config", str(config_path), "--artifact-root", str(tmp_path)])
    assert module.main() == 0
    assert registrations == [(("/raicom/mission/snapshot", FakeString), {"queue_size": 1, "latch": True})]


def test_script_error_path_registers_no_publisher(monkeypatch, tmp_path):
    registrations, errors = [], []
    fake_ros = types.SimpleNamespace(
        init_node=lambda *a, **k: pytest.fail("node initialization must not occur on config error"),
        Publisher=lambda *a, **k: registrations.append((a, k)),
        logerr=lambda *a, **k: errors.append((a, k)),
    )
    monkeypatch.setitem(sys.modules, "rospy", fake_ros)
    monkeypatch.setitem(sys.modules, "std_msgs", types.ModuleType("std_msgs"))
    message_module = types.ModuleType("std_msgs.msg"); message_module.String = object
    monkeypatch.setitem(sys.modules, "std_msgs.msg", message_module)
    spec = importlib.util.spec_from_file_location("readonly_script_error", Path(__file__).parents[1] / "scripts/sim_readonly_runtime.py")
    module = importlib.util.module_from_spec(spec); assert spec.loader is not None; spec.loader.exec_module(module)
    monkeypatch.setattr(sys, "argv", ["readonly", "--config", str(tmp_path / "missing.json"), "--artifact-root", str(tmp_path)])
    assert module.main() == 2
    assert registrations == [] and len(errors) == 1


def test_launch_uses_node_args_and_no_params():
    root = ET.fromstring((Path(__file__).parents[1] / "launch/sim_readonly_runtime.launch").read_text())
    node = root.find(".//node")
    assert node is not None and "--artifact-root" in node.attrib["args"] and "--config" in node.attrib["args"]
    assert not root.findall(".//param")
