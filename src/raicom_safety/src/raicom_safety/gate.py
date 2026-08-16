"""Deterministic, ROS-independent velocity-source arbitration.

The ROS node is a thin transport wrapper around this class. It keeps safety
policy testable and guarantees that silence, an expired command or an invalid
mode resolves to a zero command rather than to the last command.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Dict, Optional, Tuple

from .supervisor import CommandSource, SafetySupervisor


@dataclass(frozen=True)
class VelocityCommand:
    """ROS-independent representation of all Twist components.

    The SIM_ONLY gate supports planar mecanum motion: ``linear_x``,
    ``linear_y`` and ``angular_z``. Vertical, roll and pitch components are
    retained so they can be rejected rather than silently stripped.
    """

    linear_x: float = 0.0
    angular_z: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0

    @property
    def is_zero(self) -> bool:
        return (
            self.linear_x == 0.0
            and self.linear_y == 0.0
            and self.linear_z == 0.0
            and self.angular_x == 0.0
            and self.angular_y == 0.0
            and self.angular_z == 0.0
        )

    @property
    def is_planar(self) -> bool:
        return (
            self.linear_z == 0.0
            and self.angular_x == 0.0
            and self.angular_y == 0.0
        )

    @property
    def is_finite(self) -> bool:
        return all(math.isfinite(value) for value in (
            self.linear_x,
            self.linear_y,
            self.linear_z,
            self.angular_x,
            self.angular_y,
            self.angular_z,
        ))


@dataclass(frozen=True)
class GateDecision:
    command: VelocityCommand
    source: Optional[CommandSource]
    reason: str


class VelocityGate:
    """Select one fresh source according to mode, then apply safety policy.

    Envelope defaults are unbounded to retain the existing caller behaviour;
    deployments can set finite limits without adding ROS dependencies here.
    """

    def __init__(
        self,
        supervisor: SafetySupervisor,
        command_timeout_s: float,
        *,
        max_abs_linear_x: float = math.inf,
        max_abs_linear_y: float = math.inf,
        max_planar_speed: float = math.inf,
        max_abs_angular_z: float = math.inf,
    ) -> None:
        if not math.isfinite(command_timeout_s) or command_timeout_s <= 0.0:
            raise ValueError("command_timeout_s must be finite and positive")
        self._validate_envelope("max_abs_linear_x", max_abs_linear_x)
        self._validate_envelope("max_abs_linear_y", max_abs_linear_y)
        self._validate_envelope("max_planar_speed", max_planar_speed)
        self._validate_envelope("max_abs_angular_z", max_abs_angular_z)
        self._supervisor = supervisor
        self._command_timeout_s = command_timeout_s
        self._max_abs_linear_x = max_abs_linear_x
        self._max_abs_linear_y = max_abs_linear_y
        self._max_planar_speed = max_planar_speed
        self._max_abs_angular_z = max_abs_angular_z
        self._commands: Dict[CommandSource, Tuple[VelocityCommand, float]] = {}

    def update(self, source: CommandSource, command: VelocityCommand, received_at_s: float) -> None:
        self._commands[source] = (command, received_at_s)

    def decide(self, mode: str, now_s: float) -> GateDecision:
        source = self._select_source(mode, now_s)
        if source is None:
            return GateDecision(VelocityCommand(), None, "no_fresh_authorized_command")

        command, _ = self._commands[source]
        if not command.is_finite:
            return GateDecision(VelocityCommand(), source, "non_finite_command")
        if not command.is_planar:
            return GateDecision(VelocityCommand(), source, "unsupported_twist_component")
        if (
            abs(command.linear_x) > self._max_abs_linear_x
            or abs(command.linear_y) > self._max_abs_linear_y
            or math.hypot(command.linear_x, command.linear_y) > self._max_planar_speed
            or abs(command.angular_z) > self._max_abs_angular_z
        ):
            return GateDecision(VelocityCommand(), source, "velocity_envelope_exceeded")
        if not self._supervisor.permits(
            source, math.hypot(command.linear_x, command.linear_y), command.angular_z
        ):
            return GateDecision(VelocityCommand(), source, "safety_denied")
        return GateDecision(command, source, "accepted")

    def _select_source(self, mode: str, now_s: float) -> Optional[CommandSource]:
        if mode == "MANUAL":
            candidates = (CommandSource.MANUAL,)
        elif mode in ("AUTO", "DISPATCH"):
            candidates = (CommandSource.RELATIVE_MOVE, CommandSource.NAVIGATION)
        else:
            return None
        for source in candidates:
            item = self._commands.get(source)
            if item is not None and now_s - item[1] <= self._command_timeout_s:
                return source
        return None

    @staticmethod
    def _validate_envelope(name: str, value: float) -> None:
        if math.isnan(value) or value < 0.0:
            raise ValueError("{} must be non-negative and not NaN".format(name))
