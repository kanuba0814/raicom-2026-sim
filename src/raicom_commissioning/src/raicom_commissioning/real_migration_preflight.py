"""Dependency-free, read-only preflight for REAL migration preparation.

This module deliberately validates only repository artifacts.  It never imports
ROS libraries, opens devices, starts processes, or authorizes motion.
"""
from __future__ import annotations

import json
import re
import time
from pathlib import Path
from typing import Any

PREFLIGHT_SCHEMA = "raicom.real-migration-preflight/v1"
READY_FOR_READONLY_HCC = "READY_FOR_READONLY_HCC"
NOT_READY = "NOT_READY"
FIELD_CHECK_LIMIT_SECONDS = 1200

_ALLOWLIST = {
    "real_launch": "raicom_2026_sim_ws/src/raicom_bringup/launch/real.launch",
    "real_profile": "raicom_2026_sim_ws/src/raicom_bringup/config/execution_profiles_real_unresolved.json",
    "readiness_record": "raicom_2026_sim_ws/src/raicom_commissioning/config/real_commissioning_readiness.json",
    "field_template": "config/field/field_real_template.yaml",
    "site_calibration": "config/field/site_calibration.yaml",
    "robot_profile": "config/robot/oryx_real.yaml",
}


def _check(name: str, path: str, ok: bool, reason: str) -> dict[str, Any]:
    return {"name": name, "path": path, "status": "PASS" if ok else "FAIL", "reason": reason}


def _read_text(root: Path, name: str, checks: list[dict[str, Any]], reasons: list[str]) -> str | None:
    relative_path = _ALLOWLIST[name]
    root_resolved = root.resolve()
    candidate = root_resolved / relative_path
    try:
        current = root_resolved
        for part in Path(relative_path).parts:
            current = current / part
            if current.is_symlink():
                raise ValueError("symlink")
        resolved = candidate.resolve(strict=True)
        resolved.relative_to(root_resolved)
        if not resolved.is_file():
            raise ValueError("not_regular_file")
        text = resolved.read_text(encoding="utf-8")
    except (OSError, UnicodeDecodeError, ValueError):
        checks.append(_check(name, relative_path, False, "missing_unreadable_or_unsafe_path"))
        reasons.append(name + "_missing_unreadable_or_unsafe_path")
        return None
    checks.append(_check(name, relative_path, True, "readable_regular_file"))
    return text


def _load_json(name: str, text: str | None, checks: list[dict[str, Any]], reasons: list[str]) -> dict[str, Any] | None:
    if text is None:
        return None
    try:
        value = json.loads(text)
    except json.JSONDecodeError:
        checks.append(_check(name + "_json", _ALLOWLIST[name], False, "invalid_json"))
        reasons.append(name + "_invalid_json")
        return None
    if not isinstance(value, dict):
        checks.append(_check(name + "_json", _ALLOWLIST[name], False, "root_not_object"))
        reasons.append(name + "_root_not_object")
        return None
    checks.append(_check(name + "_json", _ALLOWLIST[name], True, "valid_json_object"))
    return value


def _basic_yaml_readable(name: str, text: str | None, checks: list[dict[str, Any]], reasons: list[str]) -> bool:
    """Check the project templates' dependency-free YAML subset is readable.

    Full YAML interpretation is intentionally left to the future human HCC;
    this preflight only rejects empty/binary text and obvious non-mapping files.
    """
    if text is None:
        return False
    meaningful = [line for line in text.splitlines() if line.strip() and not line.lstrip().startswith("#")]
    valid = "\x00" not in text and bool(meaningful) and re.match(r"^[^\s:#][^:#]*:\s*", meaningful[0]) is not None
    checks.append(_check(name + "_yaml", _ALLOWLIST[name], valid, "basic_yaml_readable" if valid else "invalid_basic_yaml"))
    if not valid:
        reasons.append(name + "_invalid_basic_yaml")
    return valid


def evaluate_real_migration_preflight(repo_root: Path | str) -> dict[str, Any]:
    """Return a stable report after reading only the fixed repository allowlist."""
    started = time.monotonic()
    checks: list[dict[str, Any]] = []
    reasons: list[str] = []
    root = Path(repo_root).resolve()
    texts = {name: _read_text(root, name, checks, reasons) for name in _ALLOWLIST}
    profile = _load_json("real_profile", texts["real_profile"], checks, reasons)
    readiness = _load_json("readiness_record", texts["readiness_record"], checks, reasons)

    launch_ok = texts["real_launch"] is not None and re.search(r'<param\s+name="/raicom/real_active"\s+value="false"\s*/>', texts["real_launch"] or "") is not None
    checks.append(_check("real_launch_fail_closed", _ALLOWLIST["real_launch"], launch_ok, "real_active_false" if launch_ok else "real_active_not_false"))
    if not launch_ok:
        reasons.append("real_launch_not_fail_closed")

    profile_ok = profile is not None and profile.get("name") == "REAL_UNRESOLVED" and profile.get("metadata", {}).get("verification") == "UNRESOLVED"
    checks.append(_check("real_profile_unresolved", _ALLOWLIST["real_profile"], profile_ok, "REAL_UNRESOLVED" if profile_ok else "profile_not_REAL_UNRESOLVED"))
    if not profile_ok:
        reasons.append("real_profile_not_unresolved")

    readiness_ok = readiness is not None and readiness.get("source_environment") == "REAL_UNRESOLVED"
    checks.append(_check("readiness_unresolved", _ALLOWLIST["readiness_record"], readiness_ok, "REAL_UNRESOLVED" if readiness_ok else "readiness_not_REAL_UNRESOLVED"))
    if not readiness_ok:
        reasons.append("readiness_not_unresolved")

    for name in ("field_template", "site_calibration", "robot_profile"):
        yaml_ok = _basic_yaml_readable(name, texts[name], checks, reasons)
        unresolved = yaml_ok and "status: UNRESOLVED" in (texts[name] or "")
        checks.append(_check(name + "_unresolved", _ALLOWLIST[name], unresolved, "explicit_unresolved" if unresolved else "unresolved_status_missing"))
        if not unresolved:
            reasons.append(name + "_unresolved_status_missing")

    elapsed = round(time.monotonic() - started, 6)
    return {
        "schema": PREFLIGHT_SCHEMA,
        "status": READY_FOR_READONLY_HCC if not reasons else NOT_READY,
        "authorization": "NONE",
        "motion_allowed": False,
        "field_check_limit_seconds": FIELD_CHECK_LIMIT_SECONDS,
        "elapsed_seconds": elapsed,
        "checks": checks,
        "reasons": reasons,
    }
