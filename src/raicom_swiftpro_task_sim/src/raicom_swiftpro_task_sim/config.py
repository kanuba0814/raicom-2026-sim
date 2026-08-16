"""Validated immutable runtime configuration for the SIM-only model."""
from __future__ import annotations
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Optional, Tuple
import math
import yaml

ASSUMPTION = "SIM_WORKING_ASSUMPTION"

@dataclass(frozen=True)
class SimTaskArmConfig:
    scope: str
    units: Mapping[str, str]
    frames: Mapping[str, str]
    geometry_m: Mapping[str, float]
    workspace_m: Tuple[Tuple[float, float], ...]
    joint_limits_rad: Tuple[Tuple[float, float], ...]
    thresholds: Mapping[str, float]
    slot_pick_place: Mapping[str, Any]
    projection_fixtures: Tuple[Mapping[str, Tuple[float, ...]], ...]
    fixture_provenance: Mapping[str, str]


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, dict): raise ValueError(f"{label} must be a mapping")
    return value

def _keys(value: Mapping[str, Any], expected: set[str], label: str) -> Mapping[str, Any]:
    if set(value) != expected: raise ValueError(f"{label} keys are invalid")
    return value

def _assumed(value: Any, label: str, fields: set[str]) -> Mapping[str, Any]:
    result = _mapping(value, label)
    _keys(result, fields | {"assumption_status"}, label)
    if result["assumption_status"] != ASSUMPTION: raise ValueError(f"{label} must declare {ASSUMPTION}")
    return result

def _finite_number(value: Any, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)): raise ValueError(f"{label} must be finite")
    return float(value)

def _value(value: Any, label: str) -> Any:
    return _assumed(value, label, {"value"})["value"]

def _ranges(group: Mapping[str, Any], names: Tuple[str, ...], label: str) -> Tuple[Tuple[float, float], ...]:
    result = []
    if tuple(group) != names: raise ValueError(f"{label} keys are invalid")
    for name in names:
        item = _assumed(group[name], f"{label}.{name}", {"min", "max"})
        low, high = _finite_number(item["min"], f"{label}.{name}.min"), _finite_number(item["max"], f"{label}.{name}.max")
        if low > high: raise ValueError(f"{label}.{name} minimum exceeds maximum")
        result.append((low, high))
    return tuple(result)

def load_config(path: Optional[Path] = None) -> SimTaskArmConfig:
    config_path = path or Path(__file__).resolve().parents[2] / "config" / "sim_task_arm.yaml"
    raw = _mapping(yaml.safe_load(config_path.read_text()), "root")
    _keys(raw, {"assumption_status", "scope", "units", "frames", "geometry", "workspace", "independent_joint_limits_rad", "thresholds", "slot_pick_place", "upstream_projection_fixtures"}, "root")
    if raw["assumption_status"] != ASSUMPTION: raise ValueError("root must declare SIM_WORKING_ASSUMPTION")
    scope = _value(raw["scope"], "scope")
    if not isinstance(scope, str) or not scope: raise ValueError("scope must be non-empty")
    units = {name: _value(item, f"units.{name}") for name, item in _mapping(raw.get("units"), "units").items()}
    if units != {"length": "m", "angle": "rad", "time": "s"}: raise ValueError("units must be SI m/rad/s")
    frames = {name: _value(item, f"frames.{name}") for name, item in _mapping(raw.get("frames"), "frames").items()}
    if set(frames) != {"base", "end_effector", "camera"} or not all(isinstance(item, str) and item for item in frames.values()): raise ValueError("frames are invalid")
    geometry = {name: _finite_number(_value(item, f"geometry.{name}"), f"geometry.{name}.value") for name, item in _mapping(raw.get("geometry"), "geometry").items()}
    if set(geometry) != {"lower_arm_m", "upper_arm_m", "radial_offset_m", "base_height_m", "tool_z_offset_m"} or any(item <= 0 for item in geometry.values()): raise ValueError("geometry is invalid")
    thresholds = {name: _finite_number(_value(item, f"thresholds.{name}"), f"thresholds.{name}.value") for name, item in _mapping(raw.get("thresholds"), "thresholds").items()}
    if set(thresholds) != {"observation_max_age_s", "settle_position_error_m", "settle_position_error_rad", "settle_velocity_rad_s", "settle_window_s", "fold_min_elbow_rad", "fold_min_margin_m"} or any(item <= 0 for item in thresholds.values()): raise ValueError("thresholds are invalid")
    if thresholds["fold_min_elbow_rad"] >= math.pi or thresholds["fold_min_margin_m"] >= geometry["upper_arm_m"] + geometry["lower_arm_m"]: raise ValueError("conditioning thresholds are invalid")
    workspace_m = _ranges(_mapping(raw.get("workspace"), "workspace"), ("x_m", "y_m", "z_m"), "workspace")
    sequence = _mapping(raw.get("slot_pick_place"), "slot_pick_place")
    _keys(sequence, {"observation_height_m", "grasp_height_m", "place_height_m", "source_slot_index", "placement_slot_indices"}, "slot_pick_place")
    sequence_values = {name: _value(item, f"slot_pick_place.{name}") for name, item in sequence.items()}
    heights = tuple(_finite_number(sequence_values[name], f"slot_pick_place.{name}") for name in ("observation_height_m", "grasp_height_m", "place_height_m"))
    if not heights[0] > heights[1] or not heights[0] > heights[2] or not all(workspace_m[2][0] <= height <= workspace_m[2][1] for height in heights): raise ValueError("slot pick/place heights are invalid")
    source_slot = sequence_values["source_slot_index"]
    placement_slots = sequence_values["placement_slot_indices"]
    if (isinstance(source_slot, bool) or not isinstance(source_slot, int) or not 0 <= source_slot < 4
            or not isinstance(placement_slots, list) or len(placement_slots) != 2
            or any(isinstance(slot, bool) or not isinstance(slot, int) or not 0 <= slot < 4 for slot in placement_slots)
            or len(set(placement_slots)) != 2 or source_slot in placement_slots): raise ValueError("slot pick/place indices are invalid")
    sequence_values["placement_slot_indices"] = tuple(placement_slots)
    fixtures = _assumed(raw.get("upstream_projection_fixtures"), "upstream_projection_fixtures", {"provenance", "cases"})
    provenance_entry = _assumed(fixtures["provenance"], "fixture provenance", {"source_path", "source_sha256", "generated_from"})
    provenance = {key: provenance_entry[key] for key in ("source_path", "source_sha256", "generated_from")}
    if not all(isinstance(value, str) and value for value in provenance.values()): raise ValueError("fixture provenance is invalid")
    cases = fixtures["cases"]
    if not isinstance(cases, list) or len(cases) != 4: raise ValueError("projection fixtures are invalid")
    frozen_cases = []
    for index, case in enumerate(cases):
        entry = _assumed(case, f"fixture {index}", {"input_degrees", "output_degrees"})
        inputs, outputs = entry["input_degrees"], entry["output_degrees"]
        if not isinstance(inputs, list) or not isinstance(outputs, list) or len(inputs) != 3 or len(outputs) != 9: raise ValueError("fixture dimensions are invalid")
        frozen_cases.append(MappingProxyType({"input_degrees": tuple(_finite_number(v, "fixture input") for v in inputs), "output_degrees": tuple(_finite_number(v, "fixture output") for v in outputs)}))
    return SimTaskArmConfig(scope, MappingProxyType(units), MappingProxyType(frames), MappingProxyType(geometry), workspace_m, _ranges(_mapping(raw.get("independent_joint_limits_rad"), "joint limits"), ("joint1", "joint2", "joint6"), "joint limits"), MappingProxyType(thresholds), MappingProxyType(sequence_values), tuple(frozen_cases), MappingProxyType(dict(provenance)))

CONFIG = load_config()
