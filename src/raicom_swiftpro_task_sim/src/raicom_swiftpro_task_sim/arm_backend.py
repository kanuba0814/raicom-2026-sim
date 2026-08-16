"""Arm request preparation seam; no real transport is owned or discovered here."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math
from pathlib import Path
from types import MappingProxyType
from typing import Any, Mapping, Optional, Tuple

import yaml

from .config import CONFIG, SimTaskArmConfig
from .contract import PoseRequest, Reachability
from .kinematics import admission_reason
from .motion_adapter import SwiftproSimMotionAdapter


class BackendKind(str, Enum):
    SIM = "SIM"
    REAL = "REAL"


class BackendStatus(str, Enum):
    NOT_AUTHORIZED = "NOT_AUTHORIZED"
    NOT_WIRED = "NOT_WIRED"
    INVALID = "INVALID"
    UNREACHABLE = "UNREACHABLE"
    DEADLINE_TOO_SHORT = "DEADLINE_TOO_SHORT"
    REQUEST_PREPARED = "REQUEST_PREPARED"
    FAILED = "FAILED"


@dataclass(frozen=True)
class ArmMoveRequest:
    frame_id: Any
    xyz_m: Any
    observed_at_s: Any
    deadline_s: Any

    def as_pose_request(self) -> PoseRequest:
        return PoseRequest(self.frame_id, self.xyz_m, self.observed_at_s, self.deadline_s)


@dataclass(frozen=True)
class BackendResult:
    status: BackendStatus
    reason: str
    reachability: Reachability
    settlement: str = "NOT_MEASURED"
    vendor_request: Optional["VendorMoveRequest"] = None


@dataclass(frozen=True)
class ArmBackendCapabilities:
    backend: BackendKind
    task_motion: bool
    request_freshness: bool
    external_timeout: bool
    cancellation: bool
    measured_settle: bool
    native_units: Mapping[str, str]
    supported_frames: Tuple[str, ...]
    authorization_required: bool
    hw_verified: bool


@dataclass(frozen=True)
class VendorMoveRequest:
    x_mm: float
    y_mm: float
    z_mm: float


@dataclass(frozen=True)
class RealArmAdapterConfig:
    enable_real_arm: bool
    implicit_base_frame: str
    official_worst_case_s: float
    workspace_m: Tuple[Tuple[float, float], ...]


def _finite(value: Any) -> Optional[float]:
    if isinstance(value, bool):
        return None
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def load_real_arm_config(path: Optional[Path] = None) -> RealArmAdapterConfig:
    config_path = path or Path(__file__).resolve().parents[2] / "config" / "hardware_adapter.yaml"
    raw = yaml.safe_load(config_path.read_text())
    if not isinstance(raw, dict) or raw.get("assumption_status") != "HW_UNVERIFIED":
        raise ValueError("hardware adapter must be HW_UNVERIFIED")
    section = raw.get("real_arm")
    expected = {"assumption_status", "enable_real_arm", "implicit_base_frame", "official_worst_case_s", "native_units", "workspace_m"}
    if not isinstance(section, dict) or set(section) != expected or section["assumption_status"] != "HW_UNVERIFIED":
        raise ValueError("real_arm configuration is invalid")
    enabled = section["enable_real_arm"]
    frame = section["implicit_base_frame"]
    budget = _finite(section["official_worst_case_s"])
    units = section["native_units"]
    workspace = section["workspace_m"]
    if not isinstance(enabled, bool) or not isinstance(frame, str) or not frame or budget is None or budget <= 0:
        raise ValueError("real arm configuration values are invalid")
    if units != {"request_length": "mm", "seam_length": "m"} or not isinstance(workspace, dict) or tuple(workspace) != ("x_m", "y_m", "z_m"):
        raise ValueError("real arm units or workspace are invalid")
    ranges = []
    for name in ("x_m", "y_m", "z_m"):
        item = workspace[name]
        if not isinstance(item, dict) or set(item) != {"min", "max", "assumption_status"} or item["assumption_status"] != "SIM_WORKING_ASSUMPTION":
            raise ValueError("real arm workspace is invalid")
        low, high = _finite(item["min"]), _finite(item["max"])
        if low is None or high is None or low > high:
            raise ValueError("real arm workspace bounds are invalid")
        ranges.append((low, high))
    return RealArmAdapterConfig(enabled, frame, budget, tuple(ranges))


REAL_ARM_CONFIG = load_real_arm_config()


class _BaseBackend:
    def _validate(self, request: Any, now_s: Any, frame: str, workspace: Tuple[Tuple[float, float], ...], config: SimTaskArmConfig = CONFIG) -> BackendResult:
        if not isinstance(request, ArmMoveRequest):
            return BackendResult(BackendStatus.INVALID, "INVALID_REQUEST", Reachability.INVALID)
        error = request.as_pose_request().validation_error(now_s, config.thresholds["observation_max_age_s"])
        if error:
            return BackendResult(BackendStatus.INVALID, error, Reachability.INVALID)
        if request.frame_id != frame:
            return BackendResult(BackendStatus.INVALID, "FRAME_MISMATCH", Reachability.INVALID)
        xyz = tuple(_finite(item) for item in request.xyz_m)
        if any(item is None for item in xyz):
            return BackendResult(BackendStatus.INVALID, "INVALID_REQUEST", Reachability.INVALID)
        if any(value < bounds[0] or value > bounds[1] for value, bounds in zip(xyz, workspace)):
            return BackendResult(BackendStatus.UNREACHABLE, "WORKSPACE", Reachability.UNREACHABLE)
        return BackendResult(BackendStatus.NOT_WIRED, "VALID", Reachability.REACHABLE)


class SimArmBackend(_BaseBackend):
    """SIM seam with injected control only; no ROS execution entry point exists."""
    def __init__(self, control: Any = None, config: SimTaskArmConfig = CONFIG) -> None:
        self.control = control
        self.config = config
        self.adapter = SwiftproSimMotionAdapter(config)

    def capabilities(self) -> ArmBackendCapabilities:
        # Cartesian execution is intentionally not wired yet, even if a test or
        # future integration injects control callbacks.
        return ArmBackendCapabilities(
            BackendKind.SIM,
            False,
            True,
            True,
            True,
            True,
            MappingProxyType({"length": "m"}),
            (self.config.frames["base"],),
            False,
            False,
        )

    def move(self, request: Any, now_s: Any) -> BackendResult:
        valid = self._validate(request, now_s, self.config.frames["base"], self.config.workspace_m, self.config)
        if valid.reason != "VALID":
            return valid
        reason = admission_reason(request.xyz_m, self.config)
        if reason:
            return BackendResult(BackendStatus.UNREACHABLE, reason, Reachability.UNREACHABLE)
        if self.control is None:
            return BackendResult(BackendStatus.NOT_WIRED, "NOT_WIRED", Reachability.REACHABLE)
        return BackendResult(BackendStatus.NOT_WIRED, "CONTROL_ADAPTER_REQUIRES_EXPLICIT_EXECUTION", Reachability.REACHABLE)


class RealSwiftProBackend(_BaseBackend):
    """Pure request preparer for the official blocking ``goto_position`` seam.

    This module deliberately owns no transport and cannot dispatch. A future
    human-approved commissioning composition layer must own the real ROS client,
    authorization provenance, expiry, post-return deadline handling, and stop
    procedure. Until that separate layer exists, ``move`` is always NOT_WIRED.
    """
    def __init__(self, config: RealArmAdapterConfig = REAL_ARM_CONFIG,
                 sim_config: SimTaskArmConfig = CONFIG) -> None:
        self.config = config
        self.sim_config = sim_config

    def capabilities(self) -> ArmBackendCapabilities:
        return ArmBackendCapabilities(
            BackendKind.REAL,
            False,
            True,
            False,
            False,
            False,
            MappingProxyType({"request_length": "mm", "seam_length": "m"}),
            (self.config.implicit_base_frame,),
            False,
            False,
        )

    def prepare(self, request: Any, now_s: Any) -> BackendResult:
        valid = self._validate(
            request,
            now_s,
            self.config.implicit_base_frame,
            self.config.workspace_m,
            self.sim_config,
        )
        if valid.reason != "VALID":
            return valid
        if not self.config.enable_real_arm:
            return BackendResult(
                BackendStatus.NOT_WIRED,
                "REAL_ADAPTER_DISABLED",
                Reachability.REACHABLE,
            )
        now = _finite(now_s)
        deadline = _finite(request.deadline_s)
        if now is None or deadline is None or deadline - now < self.config.official_worst_case_s:
            return BackendResult(
                BackendStatus.DEADLINE_TOO_SHORT,
                "ADMISSION_DEADLINE_TOO_SHORT",
                Reachability.REACHABLE,
            )
        xyz_mm = tuple(float(value) * 1000.0 for value in request.xyz_m)
        return BackendResult(
            BackendStatus.REQUEST_PREPARED,
            "COMMISSIONING_DISPATCH_REQUIRED",
            Reachability.REACHABLE,
            "NOT_MEASURED",
            VendorMoveRequest(*xyz_mm),
        )

    def move(self, request: Any, now_s: Any) -> BackendResult:
        prepared = self.prepare(request, now_s)
        if prepared.status is not BackendStatus.REQUEST_PREPARED:
            return prepared
        return BackendResult(
            BackendStatus.NOT_WIRED,
            "COMMISSIONING_DISPATCH_NOT_IMPLEMENTED",
            Reachability.REACHABLE,
            "NOT_MEASURED",
            prepared.vendor_request,
        )
