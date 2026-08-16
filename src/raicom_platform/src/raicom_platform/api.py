"""Semantic platform contracts shared by SIM, REAL, and fake test backends."""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from enum import Enum
from typing import Callable, Optional, Tuple
import threading
import time

from raicom_core.models import GraspEvidence, GraspState, OperationResult, SafeHoldEvidence, SafeHoldResult


class BackendKind(str, Enum):
    SIM = "SIM"
    REAL = "REAL"
    FAKE = "FAKE"


class CapabilityState(str, Enum):
    READY = "READY"
    SIM_TESTED = "SIM_TESTED"
    UNAVAILABLE = "UNAVAILABLE"
    FAULT = "FAULT"
    HW_UNVERIFIED = "HW_UNVERIFIED"


@dataclass(frozen=True)
class RobotPose:
    """A reported robot pose, never a competition-field calibration source."""

    frame_id: str
    x_m: float
    y_m: float
    yaw_rad: float


@dataclass(frozen=True)
class HealthReport:
    backend: BackendKind
    navigation: CapabilityState
    alignment: CapabilityState
    arm: CapabilityState
    gripper: CapabilityState
    charging: CapabilityState
    detail: str = ""


class PlatformBackend(ABC):
    """Semantic API boundary; Mission Core never calls vendor ROS interfaces."""

    @abstractmethod
    def cancel_active_operation(self) -> OperationResult:
        """Cancel one active operation without changing global safety admission."""

    @abstractmethod
    def safe_hold(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        """Return source-qualified bounded-hold evidence, never bare success."""

    @abstractmethod
    def stop_all(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        """Compatibility spelling of the source-qualified bounded safe hold."""

    @abstractmethod
    def emergency_stop(self) -> None:
        """Atomically latch safety and cancel active work; safe to repeat."""

    @abstractmethod
    def health(self) -> HealthReport:
        """Return health without asserting that hardware is field-validated."""

    @abstractmethod
    def goto(self, waypoint: str, deadline_s: Optional[float] = None) -> OperationResult:
        """Navigate to a configured semantic waypoint."""

    @abstractmethod
    def cancel_navigation(self) -> OperationResult:
        """Cancel an in-progress semantic navigation request."""

    @abstractmethod
    def navigation_status(self) -> tuple[Optional[OperationResult], Optional[str], str]:
        """Return semantic navigation outcome, active name, and adapter detail."""

    @abstractmethod
    def clear_costmap(self) -> OperationResult:
        """Request bounded navigation recovery without exposing vendor APIs."""

    @abstractmethod
    def track(self, marker_id: str, goal_distance_m: float) -> OperationResult:
        """Perform bounded station alignment against a configured marker."""

    @abstractmethod
    def relative_move(self, dx_m: float, dy_m: float, dyaw_rad: float) -> OperationResult:
        """Perform an adapter-supervised local alignment motion in the future."""

    @abstractmethod
    def pose(self) -> Optional[RobotPose]:
        """Return the latest reported pose, or ``None`` when unavailable."""

    @abstractmethod
    def perceive(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        """Request a bounded semantic observation; no image or ROS contract leaks here."""

    @abstractmethod
    def grasp(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        """Request a bounded semantic grasp operation."""

    @abstractmethod
    def place(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        """Request a bounded semantic placement operation."""

    @abstractmethod
    def grasp_evidence(self, target: str) -> GraspEvidence:
        """Return immutable target-bound evidence; UNKNOWN is never success."""


class UnavailableBackend(PlatformBackend):
    """Safe placeholder that performs no ROS I/O or motion commands."""

    def __init__(self, kind: BackendKind, detail: str) -> None:
        self._kind = kind
        self._detail = detail
        self.stop_calls = 0
        self.safe_hold_calls = 0

    def cancel_active_operation(self) -> OperationResult:
        return self.cancel_navigation()

    def safe_hold(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        self.safe_hold_calls += 1
        return SafeHoldResult(SafeHoldEvidence.UNCONFIRMED, self._detail)

    def stop_all(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        return self.safe_hold(deadline_s)

    def emergency_stop(self) -> None:
        # This placeholder cannot publish a ROS command, including zero velocity.
        self.stop_calls += 1

    def health(self) -> HealthReport:
        unavailable = (
            CapabilityState.HW_UNVERIFIED
            if self._kind == BackendKind.REAL
            else CapabilityState.UNAVAILABLE
        )
        return HealthReport(
            backend=self._kind,
            navigation=unavailable,
            alignment=unavailable,
            arm=unavailable,
            gripper=unavailable,
            charging=unavailable,
            detail=self._detail,
        )

    def goto(self, waypoint: str, deadline_s: Optional[float] = None) -> OperationResult:
        return OperationResult.UNAVAILABLE

    def cancel_navigation(self) -> OperationResult:
        return OperationResult.UNAVAILABLE

    def navigation_status(self) -> tuple[Optional[OperationResult], Optional[str], str]:
        return OperationResult.UNAVAILABLE, None, self._detail

    def clear_costmap(self) -> OperationResult:
        return OperationResult.UNAVAILABLE

    def track(self, marker_id: str, goal_distance_m: float) -> OperationResult:
        return OperationResult.UNAVAILABLE

    def relative_move(self, dx_m: float, dy_m: float, dyaw_rad: float) -> OperationResult:
        return OperationResult.UNAVAILABLE

    def pose(self) -> Optional[RobotPose]:
        return None

    def perceive(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        return OperationResult.UNAVAILABLE

    def grasp(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        return OperationResult.UNAVAILABLE

    def place(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        return OperationResult.UNAVAILABLE

    def grasp_evidence(self, target: str) -> GraspEvidence:
        return GraspEvidence(GraspState.UNKNOWN, target, 0.0, "unavailable", self._detail)


class SimBackend(UnavailableBackend):
    """Actuating SIM-only navigation through ``move_base`` and the Safety Gate.

    The backend accepts semantic names only. Its profile and ROS action client
    are injected so deterministic tests require neither ROS nor motion.
    """

    def __init__(
        self,
        profile: object | None = None,
        action_client: object | None = None,
        *,
        sim_active: bool = True,
        safety_authorized: object | None = None,
        server_timeout_s: float = 8.0,
        now: Callable[[], float] | None = None,
    ) -> None:
        super().__init__(BackendKind.SIM, "SIM navigation is not configured")
        self._profile = profile
        self._action_client = action_client
        self._sim_active = sim_active
        self._safety_authorized = safety_authorized
        self._server_timeout_s = server_timeout_s
        self._now = now or time.monotonic
        self._active_waypoint: Optional[str] = None
        self._active_client: object | None = None
        self._last_result: Optional[OperationResult] = None
        self._last_detail = "not configured"
        self._safety_latched = False
        self._safety_epoch = 0
        self._operation_generation = 0
        self._active_generation: Optional[int] = None
        # PREPARING owns admission; DISPATCHING owns the send_goal handoff;
        # SENT is a client-visible action goal.  External client calls never
        # execute while this (deliberately non-reentrant) mutex is held.
        self._dispatch_state = "IDLE"
        self._lock = threading.Lock()

    def _record(self, result: OperationResult, detail: str) -> OperationResult:
        with self._lock:
            # A late pre-latch validation or completion must not replace the
            # safety-visible status established by emergency_stop.
            if not self._safety_latched:
                self._last_result, self._last_detail = result, detail
        return result

    def _finish_operation(self, generation: int, result: OperationResult, detail: str) -> OperationResult:
        """Record only the operation that still owns the current generation."""
        with self._lock:
            if self._safety_latched or self._active_generation != generation:
                return result
            self._active_waypoint = None
            self._active_client = None
            self._active_generation = None
            self._dispatch_state = "IDLE"
            self._last_result, self._last_detail = result, detail
        return result

    def safe_hold(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        """Cancel current SIM navigation and return SIM-qualified hold evidence."""
        if deadline_s is not None and self._now() >= deadline_s:
            self._record(OperationResult.TIMEOUT, "safe hold deadline expired")
            return SafeHoldResult(SafeHoldEvidence.TIMEOUT, "safe hold deadline expired")
        with self._lock:
            if self._safety_latched:
                return SafeHoldResult(SafeHoldEvidence.UNCONFIRMED, "E-stop already latched")
            client, active = self._active_client, self._active_waypoint
            self._operation_generation += 1
            self._active_waypoint = None
            self._active_client = None
            self._active_generation = None
            self._dispatch_state = "IDLE"
            self._last_result, self._last_detail = OperationResult.SUCCEEDED, "safe hold requested"
        if client is not None and active is not None:
            client.cancel_goal()
        return SafeHoldResult(SafeHoldEvidence.CONFIRMED_SIM, "SIM navigation hold requested")

    def stop_all(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        return self.safe_hold(deadline_s)

    def cancel_active_operation(self) -> OperationResult:
        return self.cancel_navigation()

    def emergency_stop(self) -> None:
        """Latch once per epoch without calling an action client under the mutex.

        A DISPATCHING goal has not returned from ``send_goal`` yet, so its
        dispatcher performs the one required cancellation after that call.
        """
        with self._lock:
            if self._safety_latched:
                return
            self.stop_calls += 1
            self._safety_latched = True
            self._safety_epoch += 1
            self._operation_generation += 1
            client = self._active_client if self._dispatch_state == "SENT" else None
            self._active_waypoint = None
            self._active_client = None
            self._active_generation = None
            self._dispatch_state = "IDLE"
            self._last_result, self._last_detail = OperationResult.SAFETY_DENIED, "E-stop latched"
        if client is not None:
            client.cancel_goal()

    def _denied_if_latched(self) -> Optional[OperationResult]:
        with self._lock:
            if self._safety_latched:
                return OperationResult.SAFETY_DENIED
        return None

    def goto(self, waypoint: str, deadline_s: Optional[float] = None) -> OperationResult:
        denied = self._denied_if_latched()
        if denied is not None:
            return denied
        if deadline_s is not None and self._now() >= deadline_s:
            return self._record(OperationResult.TIMEOUT, "navigation deadline expired")
        if not self._sim_active:
            return self._record(OperationResult.UNAVAILABLE, "SIM profile rejected outside SIM")
        if self._profile is None:
            return self._record(OperationResult.UNAVAILABLE, "SIM waypoint profile is not configured")
        if self._safety_authorized is not None and not self._safety_authorized():
            return self._record(OperationResult.SAFETY_DENIED, "Safety Gate denied navigation")
        try:
            goal = self._profile.resolve(waypoint)
        except ValueError as exc:
            return self._record(OperationResult.NOT_CONFIGURED, str(exc))
        client = self._action_client
        if client is None:
            from .sim_navigation import RosMoveBaseClient
            client = self._action_client = RosMoveBaseClient()

        # Admission creates a visible, generation-bound handle before any
        # blocking client call.  A second goto is explicitly rejected rather
        # than replacing or orphaning that handle.
        with self._lock:
            if self._safety_latched:
                return OperationResult.SAFETY_DENIED
            if self._active_generation is not None:
                return OperationResult.FAILED
            generation = self._operation_generation + 1
            self._operation_generation = generation
            dispatch_epoch = self._safety_epoch
            self._active_generation = generation
            self._active_waypoint = waypoint
            self._active_client = client
            self._dispatch_state = "PREPARING"

        server_timeout_s = self._server_timeout_s if deadline_s is None else max(0.0, min(self._server_timeout_s, deadline_s - self._now()))
        if not client.wait_for_server(server_timeout_s):
            return self._finish_operation(generation, OperationResult.UNAVAILABLE, "move_base action server unavailable")

        # Commit dispatch under the mutex, then release it before entering
        # reentrant/external send_goal.  E-stop either wins this commit and
        # prevents send, or observes DISPATCHING and makes the sender cancel
        # immediately after send_goal returns.
        with self._lock:
            if self._safety_latched or self._safety_epoch != dispatch_epoch or self._active_generation != generation:
                return OperationResult.SAFETY_DENIED
            self._dispatch_state = "DISPATCHING"
        client.send_goal(goal)
        with self._lock:
            dispatch_survived = (not self._safety_latched and self._safety_epoch == dispatch_epoch
                                  and self._active_generation == generation)
            if dispatch_survived:
                self._dispatch_state = "SENT"
        if not dispatch_survived:
            # This is the deferred half of E-stop cancellation for a goal
            # whose send_goal call overlapped the latch.
            client.cancel_goal()
            return OperationResult.SAFETY_DENIED

        result_timeout_s = self._profile.goal_timeout_s if deadline_s is None else max(0.0, min(self._profile.goal_timeout_s, deadline_s - self._now()))
        if not client.wait_for_result(result_timeout_s):
            if self._denied_if_latched() is not None:
                return OperationResult.SAFETY_DENIED
            with self._lock:
                canceled_by_hold = self._active_generation != generation
            if canceled_by_hold:
                return OperationResult.CANCELED
            client.cancel_goal()
            return self._finish_operation(generation, OperationResult.TIMEOUT, "move_base result timeout; goal canceled")
        if self._denied_if_latched() is not None:
            return OperationResult.SAFETY_DENIED
        from .sim_navigation import map_action_state
        return self._finish_operation(generation, map_action_state(client.get_state()), "move_base terminal result")

    def cancel_navigation(self) -> OperationResult:
        denied = self._denied_if_latched()
        if denied is not None:
            return denied
        with self._lock:
            client, active = self._active_client, self._active_waypoint
            if client is None or active is None:
                self._last_result, self._last_detail = OperationResult.UNAVAILABLE, "no active move_base goal"
                return OperationResult.UNAVAILABLE
            self._operation_generation += 1
            self._active_waypoint = None
            self._active_client = None
            self._active_generation = None
            self._dispatch_state = "IDLE"
            self._last_result, self._last_detail = OperationResult.CANCELED, "move_base goal cancellation requested"
        client.cancel_goal()
        return OperationResult.CANCELED


    def perceive(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        return self._denied_if_latched() or super().perceive(target, deadline_s)

    def grasp(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        return self._denied_if_latched() or super().grasp(target, deadline_s)

    def place(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        return self._denied_if_latched() or super().place(target, deadline_s)

    def grasp_evidence(self, target: str) -> GraspEvidence:
        return GraspEvidence(GraspState.UNKNOWN, target, self._now(), "sim", "gripper contract unavailable")

    def pose(self) -> Optional[RobotPose]:
        if self._action_client is None:
            return None
        return self._action_client.pose()

    def navigation_status(self) -> tuple[Optional[OperationResult], Optional[str], str]:
        """Return semantic result, active waypoint name, and adapter detail."""
        with self._lock:
            return self._last_result, self._active_waypoint, self._last_detail

    def health(self) -> HealthReport:
        if not self._sim_active or self._profile is None:
            return super().health()
        return HealthReport(
            backend=BackendKind.SIM,
            navigation=CapabilityState.READY,
            alignment=CapabilityState.UNAVAILABLE,
            arm=CapabilityState.UNAVAILABLE,
            gripper=CapabilityState.UNAVAILABLE,
            charging=CapabilityState.UNAVAILABLE,
            detail="SIM_ONLY named-waypoint move_base adapter",
        )


class RealBackend(UnavailableBackend):
    """Non-actuating REAL backend locked pending Hardware Contract Check."""

    def __init__(self) -> None:
        super().__init__(
            BackendKind.REAL,
            "REAL backend is locked out pending Hardware Contract Check",
        )


class FakeBackend(PlatformBackend):
    """Deterministic test backend; it deliberately cannot command a robot."""

    def __init__(
        self,
        navigation_result: OperationResult = OperationResult.SUCCEEDED,
        alignment_result: OperationResult = OperationResult.SUCCEEDED,
        perception_result: OperationResult = OperationResult.SUCCEEDED,
        grasp_result: OperationResult = OperationResult.SUCCEEDED,
        placement_result: OperationResult = OperationResult.SUCCEEDED,
        grasp_evidence: Optional[GraspEvidence] = None,
        reported_pose: Optional[RobotPose] = None,
    ) -> None:
        self.navigation_result = navigation_result
        self.alignment_result = alignment_result
        self.perception_result = perception_result
        self.grasp_result = grasp_result
        self.placement_result = placement_result
        self._grasp_evidence = grasp_evidence
        self.reported_pose = reported_pose
        self.stop_calls = 0
        self.safe_hold_calls = 0
        self._safety_latched = False
        self.last_waypoint: Optional[str] = None
        self.last_track_request: Optional[Tuple[str, float]] = None
        self.last_relative_move: Optional[Tuple[float, float, float]] = None
        self.cancel_calls = 0
        self.clear_costmap_calls = 0
        self.last_perception_target: Optional[str] = None
        self.last_grasp_target: Optional[str] = None
        self.last_placement_target: Optional[str] = None

    def cancel_active_operation(self) -> OperationResult:
        return self.cancel_navigation()

    def safe_hold(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        """Return deterministic FAKE hold evidence, never REAL stop proof."""
        self.safe_hold_calls += 1
        # Fake time is intentionally deterministic and not tied to wall-clock
        # deadlines supplied by a semantic runner test.
        return SafeHoldResult(SafeHoldEvidence.CONFIRMED_FAKE, "deterministic fake hold")

    def stop_all(self, deadline_s: Optional[float] = None) -> SafeHoldResult:
        return self.safe_hold(deadline_s)

    def emergency_stop(self) -> None:
        self.stop_calls += 1
        self._safety_latched = True

    def health(self) -> HealthReport:
        return HealthReport(
            backend=BackendKind.FAKE,
            navigation=CapabilityState.SIM_TESTED,
            alignment=CapabilityState.SIM_TESTED,
            arm=CapabilityState.SIM_TESTED,
            gripper=CapabilityState.SIM_TESTED,
            charging=CapabilityState.HW_UNVERIFIED,
            detail="deterministic non-actuating test backend; never commissioning-ready",
        )

    def goto(self, waypoint: str, deadline_s: Optional[float] = None) -> OperationResult:
        if self._safety_latched: return OperationResult.SAFETY_DENIED
        self.last_waypoint = waypoint
        return self.navigation_result

    def cancel_navigation(self) -> OperationResult:
        self.cancel_calls += 1
        return OperationResult.CANCELED

    def navigation_status(self) -> tuple[Optional[OperationResult], Optional[str], str]:
        return self.navigation_result, self.last_waypoint, "deterministic test backend"

    def clear_costmap(self) -> OperationResult:
        self.clear_costmap_calls += 1
        return OperationResult.SUCCEEDED

    def track(self, marker_id: str, goal_distance_m: float) -> OperationResult:
        self.last_track_request = (marker_id, goal_distance_m)
        return self.alignment_result

    def relative_move(self, dx_m: float, dy_m: float, dyaw_rad: float) -> OperationResult:
        self.last_relative_move = (dx_m, dy_m, dyaw_rad)
        return self.alignment_result

    def pose(self) -> Optional[RobotPose]:
        return self.reported_pose

    def perceive(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        if self._safety_latched: return OperationResult.SAFETY_DENIED
        self.last_perception_target = target
        return self.perception_result

    def grasp(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        if self._safety_latched: return OperationResult.SAFETY_DENIED
        self.last_grasp_target = target
        return self.grasp_result

    def place(self, target: str, deadline_s: Optional[float] = None) -> OperationResult:
        if self._safety_latched: return OperationResult.SAFETY_DENIED
        self.last_placement_target = target
        return self.placement_result

    def grasp_evidence(self, target: str) -> GraspEvidence:
        return self._grasp_evidence or GraspEvidence(GraspState.UNKNOWN, target, time.monotonic(), "fake")
