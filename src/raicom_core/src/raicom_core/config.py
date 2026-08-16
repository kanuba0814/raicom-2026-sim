"""Validated, hashable loading for externally supplied competition profiles."""
from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

_VALID_STATUSES = {"CONFIRMED", "WORKING_ASSUMPTION", "UNRESOLVED"}


class ConfigError(ValueError):
    """Raised when a profile is malformed or claims certainty without a status."""


@dataclass(frozen=True)
class LoadedProfile:
    path: Path
    sha256: str
    data: dict[str, Any]


def load_profile(path: Path) -> LoadedProfile:
    """Load one YAML profile and validate status-labelled configuration leaves.

    A profile can remain intentionally incomplete. Incompleteness must use a
    ``status: UNRESOLVED`` marker rather than being silently defaulted.
    """
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ConfigError(f"cannot read profile {path}: {exc}") from exc
    try:
        data = yaml.safe_load(raw) or {}
    except yaml.YAMLError as exc:
        raise ConfigError(f"invalid YAML in {path}: {exc}") from exc
    if not isinstance(data, dict):
        raise ConfigError(f"profile {path} must contain a mapping")
    _validate_statuses(data, path=str(path))
    return LoadedProfile(path=path, sha256=hashlib.sha256(raw).hexdigest(), data=data)


def _validate_statuses(node: Any, *, path: str) -> None:
    if isinstance(node, dict):
        status = node.get("status")
        if status is not None and status not in _VALID_STATUSES:
            raise ConfigError(f"{path}.status must be one of {sorted(_VALID_STATUSES)}")
        for key, value in node.items():
            _validate_statuses(value, path=f"{path}.{key}")
    elif isinstance(node, list):
        for index, value in enumerate(node):
            _validate_statuses(value, path=f"{path}[{index}]")
