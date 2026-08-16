"""Backend-neutral contract for a non-actuating SIM kinematic model."""
from __future__ import annotations
from dataclasses import dataclass
from enum import Enum
import math
from typing import Any, Optional, Protocol, Tuple
from .config import CONFIG, SimTaskArmConfig
from .kinematics import admission_reason

class MoveStatus(str, Enum): UNAVAILABLE="UNAVAILABLE"; NOT_WIRED="NOT_WIRED"; INVALID="INVALID"; UNREACHABLE="UNREACHABLE"
class Reachability(str, Enum): REACHABLE="REACHABLE"; UNREACHABLE="UNREACHABLE"; INVALID="INVALID"
class SettleState(str, Enum): SETTLED="SETTLED"; MOVING="MOVING"; STALE="STALE"; TIMEOUT="TIMEOUT"; INVALID="INVALID"

def _number(value: Any) -> Optional[float]:
    if isinstance(value, bool): return None
    try: result = float(value)
    except (TypeError, ValueError): return None
    return result if math.isfinite(result) else None

@dataclass(frozen=True)
class PoseRequest:
    frame_id: Any
    xyz_m: Any
    observed_at_s: Any
    deadline_s: Any
    def validation_error(self, now_s: Any, max_age_s: Any) -> Optional[str]:
        if not isinstance(self.frame_id, str) or not self.frame_id or not isinstance(self.xyz_m, (tuple, list)) or len(self.xyz_m) != 3: return "INVALID_REQUEST"
        values = tuple(_number(value) for value in (*self.xyz_m, self.observed_at_s, self.deadline_s, now_s, max_age_s))
        if any(value is None for value in values) or values[-1] <= 0: return "INVALID_REQUEST"
        observed, deadline, now, maximum_age = values[3:]
        if deadline <= now: return "TIMEOUT"
        if observed > now or now - observed > maximum_age: return "STALE"
        return None

@dataclass(frozen=True)
class MoveResult: status: MoveStatus; reason: str; reachability: Reachability
@dataclass(frozen=True)
class SettledEvidence:
    state: Any; observed_at_s: Any; position_error_m: Any = None; velocity_rad_s: Any = None
    def evaluate(self, now_s: Any, max_age_s: Any, position_limit_m: Any, velocity_limit_rad_s: Any, deadline_s: Any) -> SettleState:
        values = tuple(_number(value) for value in (self.observed_at_s, now_s, max_age_s, position_limit_m, velocity_limit_rad_s, deadline_s))
        if any(value is None for value in values) or values[2] <= 0 or values[3] < 0 or values[4] < 0: return SettleState.INVALID
        observed, now, maximum_age, position_limit, velocity_limit, deadline = values
        if now >= deadline: return SettleState.TIMEOUT
        if observed > now or now - observed > maximum_age: return SettleState.STALE
        position, velocity = _number(self.position_error_m), _number(self.velocity_rad_s)
        if position is None or velocity is None: return SettleState.INVALID
        return SettleState.SETTLED if position <= position_limit and abs(velocity) <= velocity_limit else SettleState.MOVING

@dataclass(frozen=True)
class ArmCapabilities:
    sim_only: bool=True; non_actuating: bool=True; kinematics: bool=True; task_motion: bool=False; dynamics: bool=False; payload: bool=False; collision: bool=False; suction: bool=False; p0_b: str="BLOCKED"
class ArmTaskContract(Protocol):
    def validate_move(self, request: Any, now_s: Any) -> MoveResult: ...
    def move(self, request: Any, now_s: Any) -> MoveResult: ...
    def capabilities(self) -> ArmCapabilities: ...
class NonActuatingArmPlanner:
    def __init__(self, config: SimTaskArmConfig = CONFIG) -> None: self.config = config
    def validate_move(self, request: Any, now_s: Any) -> MoveResult:
        if not isinstance(self.config, SimTaskArmConfig) or not isinstance(request, PoseRequest):
            return MoveResult(MoveStatus.INVALID, "INVALID_REQUEST", Reachability.INVALID)
        error = request.validation_error(now_s, self.config.thresholds["observation_max_age_s"])
        if error: return MoveResult(MoveStatus.INVALID, error, Reachability.INVALID)
        if request.frame_id != self.config.frames["base"]: return MoveResult(MoveStatus.INVALID, "FRAME_MISMATCH", Reachability.INVALID)
        reason = admission_reason(request.xyz_m, self.config)
        if reason: return MoveResult(MoveStatus.UNREACHABLE, reason, Reachability.UNREACHABLE)
        return MoveResult(MoveStatus.UNAVAILABLE, "NOT_WIRED", Reachability.REACHABLE)
    def move(self, request: Any, now_s: Any) -> MoveResult: return self.validate_move(request, now_s)
    def capabilities(self) -> ArmCapabilities: return ArmCapabilities()
