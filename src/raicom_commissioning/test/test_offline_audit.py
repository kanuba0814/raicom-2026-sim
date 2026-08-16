import ast
import hashlib
import json
from pathlib import Path
import subprocess
import sys

from raicom_bringup.readonly_runtime import RuntimeConfig, create_readonly_runtime
from raicom_commissioning.offline_audit import AUDIT_SCHEMA, audit_readonly_manifest

CONFIG = {"profile": "SIM_DEVELOPMENT_READONLY", "settlement_threshold": 0, "snapshot_rate_hz": 1.0,
          "replay": {"class_mapping": {"development_block": "development_block"}, "minimum_confidence": 0.0,
                     "minimum_stability": 1.0, "required_consecutive_frames": 1, "maximum_frame_age_s": 1.0,
                     "model_path": "", "model_sha256": "", "model_verification": "NOT_FIELD_VALIDATED"},
          "turntable": {"max_observation_age_s": 0.5, "required_execution_budget_s": 0.2, "max_time_to_exit_s": 1.0},
          "recovery": {"navigation_retries": 0, "reobserve_attempts": 0, "charge_retries": 0, "action_timeout_s": 1.0}}


def _generated(root):
    runtime = create_readonly_runtime(root, RuntimeConfig.from_mapping(CONFIG))
    return runtime, root / runtime.run_id / "manifest.json"


def _audit(root, runtime, **selectors):
    return audit_readonly_manifest(root, expected_config_digest=runtime.config.config_digest, **selectors)


def test_audits_real_generated_readonly_manifest_offline(tmp_path):
    runtime, _ = _generated(tmp_path)
    report = _audit(tmp_path, runtime, run_id=runtime.run_id)
    assert report.status == "PASS" and report.artifact_integrity == "ARTIFACT_CONTRACT_PASS"
    assert report.to_dict()["schema"] == AUDIT_SCHEMA and report.authorization == "NONE"
    assert report.readiness == "NOT_READY" and report.admission == "DENIED"
    assert json.loads(report.canonical_json()) == report.to_dict()
    assert _audit(tmp_path, runtime, manifest_path=Path(runtime.run_id) / "manifest.json").status == "PASS"


def test_bad_artifacts_fail_closed(tmp_path):
    for name, mutate in {"hash": lambda v: v.update(snapshot_sha256="0" * 64),
                         "schema": lambda v: v.update(version=99),
                         "provenance": lambda v: v["raw_sealed_snapshot"]["capabilities"]["navigation"].update(source="REAL")}.items():
        root = tmp_path / name; root.mkdir(); runtime, manifest = _generated(root)
        value = json.loads(manifest.read_text()); mutate(value); manifest.write_text(json.dumps(value, sort_keys=True, separators=(",", ":")))
        assert _audit(root, runtime, run_id=runtime.run_id).status == "FAIL"
    root = tmp_path / "journal"; root.mkdir(); runtime, manifest = _generated(root)
    (manifest.parent / "events.jsonl").write_text("tamper\n")
    assert _audit(root, runtime, run_id=runtime.run_id).status == "FAIL"


def test_duplicate_noncanonical_oversized_and_malformed_manifest_fail_closed(tmp_path):
    cases = {"duplicate": '{"schema":1,"schema":1}', "noncanonical": '{ }', "malformed": '{', "oversized": "x" * 17000}
    for name, content in cases.items():
        root = tmp_path / name; root.mkdir(); runtime, manifest = _generated(root)
        manifest.write_text(content)
        assert _audit(root, runtime, run_id=runtime.run_id).status == "FAIL"


def test_missing_expectation_invalid_selector_and_report_are_bounded(tmp_path):
    runtime, _ = _generated(tmp_path)
    assert audit_readonly_manifest(tmp_path, expected_config_digest=None, run_id=runtime.run_id).status == "FAIL"
    report = audit_readonly_manifest(tmp_path, expected_config_digest="X" * 10000, run_id="X" * 10000)
    assert report.status == "FAIL" and report.run_id is None
    assert len(report.canonical_json().encode()) <= 4096
    assert all(len(reason.encode()) <= 256 for reason in report.reasons)


def test_coordinated_digest_tamper_is_caught_by_out_of_band_expectation(tmp_path):
    runtime, manifest = _generated(tmp_path)
    value = json.loads(manifest.read_text()); provenance = value["raw_sealed_snapshot"]["config_provenance"]
    provenance["settlement_threshold"] = 1
    forged = hashlib.sha256(json.dumps(provenance, sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()).hexdigest()
    value["config_digest"] = value["raw_sealed_snapshot"]["config_digest"] = forged
    value["snapshot_sha256"] = hashlib.sha256(json.dumps(value["raw_sealed_snapshot"], sort_keys=True, separators=(",", ":"), ensure_ascii=True).encode()).hexdigest()
    manifest.write_text(json.dumps(value, sort_keys=True, separators=(",", ":")))
    assert _audit(tmp_path, runtime, run_id=runtime.run_id).status == "FAIL"


def test_intermediate_root_component_swap_is_descriptor_safe(monkeypatch, tmp_path):
    import raicom_commissioning.offline_audit as audit
    safe = tmp_path / "safe"; safe.mkdir(); runtime, _ = _generated(safe)
    outside = tmp_path / "outside"; outside.mkdir(); original = audit._open_directory_at; swapped = False
    def swap(parent_fd, name):
        nonlocal swapped
        result = original(parent_fd, name)
        if not swapped and name == "safe":
            swapped = True; safe.rename(tmp_path / "safe-old"); safe.symlink_to(outside, target_is_directory=True)
        return result
    monkeypatch.setattr(audit, "_open_directory_at", swap)
    assert audit.audit_readonly_manifest(safe, expected_config_digest=runtime.config.config_digest, run_id=runtime.run_id).status == "PASS"


def test_module_ast_and_import_graph_have_no_actuation_or_networking():
    source = Path(__file__).parents[1] / "src" / "raicom_commissioning" / "offline_audit.py"; tree = ast.parse(source.read_text())
    imported = {a.name.split(".")[0] for n in ast.walk(tree) if isinstance(n, (ast.Import, ast.ImportFrom)) for a in n.names} | {n.module.split(".")[0] for n in ast.walk(tree) if isinstance(n, ast.ImportFrom) and n.module}
    denied = {"rospy", "rclpy", "actionlib", "subprocess", "socket", "requests", "urllib", "http", "serial", "roslib", "geometry_msgs", "nav_msgs"}
    assert not imported & denied
    calls = {n.func.attr for n in ast.walk(tree) if isinstance(n, ast.Call) and isinstance(n.func, ast.Attribute)}
    assert not calls & {"system", "popen", "run", "call", "connect", "send", "publish", "dispatch", "write", "mkdir", "unlink", "rename"}
    assert not any(isinstance(n, ast.Call) and isinstance(n.func, ast.Name) and n.func.id in {"__import__", "getattr"} for n in ast.walk(tree))
    done = subprocess.run([sys.executable, "-c", "import sys; import raicom_commissioning.offline_audit; print(','.join(sys.modules))"], check=True, text=True, capture_output=True)
    assert not any(n.split(".")[0] in {"rospy", "rclpy", "actionlib", "socket", "subprocess", "serial"} for n in done.stdout.split(","))
