"""Validation for explicit SIM navigation and safety execution profiles."""
from __future__ import annotations

import math
from typing import Any, Mapping

_LIMIT_NAMES = (
    "max_abs_linear_x",
    "max_abs_linear_y",
    "max_planar_speed",
    "max_abs_angular_z",
)
_NUMERIC_FIELDS = (
    "expected_producer_hz",
    "stale_timeout_s",
    "controller_hz",
    "mode_timeout_s",
    "localization_timeout_s",
    "publish_hz",
)


class ExecutionProfileError(ValueError):
    """Raised when a profile cannot safely configure the command gate."""


def validate_execution_profile(profile: Mapping[str, Any], backend: str) -> Mapping[str, Any]:
    """Return a validated profile or fail closed before the gate is started."""
    if backend != "SIM":
        raise ExecutionProfileError("execution profiles are activatable only for backend SIM")
    if profile.get("name") not in ("SIM_SAFE", "SIM_DEV_FAST"):
        raise ExecutionProfileError("profile is not an activatable SIM profile")

    metadata = _mapping(profile, "metadata")
    if metadata.get("scope") != "SIM_ONLY":
        raise ExecutionProfileError("activatable profile must have explicit SIM_ONLY scope")
    for field in ("verification", "provenance"):
        if not isinstance(metadata.get(field), str) or not metadata[field].strip():
            raise ExecutionProfileError("metadata.{} is required".format(field))

    for field in _NUMERIC_FIELDS:
        _positive(profile, field)
    if profile["stale_timeout_s"] < 1.0 / profile["expected_producer_hz"]:
        raise ExecutionProfileError("stale_timeout_s must cover one expected producer period")

    planner = _mapping(profile, "planner_limits")
    gate = _mapping(profile, "gate_limits")
    for field in _LIMIT_NAMES:
        _positive(planner, field)
        _positive(gate, field)
        if planner[field] > gate[field]:
            raise ExecutionProfileError("planner {} exceeds gate limit".format(field))
    return profile


def _mapping(value: Mapping[str, Any], field: str) -> Mapping[str, Any]:
    nested = value.get(field)
    if not isinstance(nested, Mapping):
        raise ExecutionProfileError("{} must be a mapping".format(field))
    return nested


def _positive(value: Mapping[str, Any], field: str) -> None:
    number = value.get(field)
    if not isinstance(number, (int, float)) or isinstance(number, bool) or not math.isfinite(number) or number <= 0.0:
        raise ExecutionProfileError("{} must be a finite positive number".format(field))
